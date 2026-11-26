import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    src_dir = '/opt/go1/src'
    slam_params = os.path.join(src_dir, 'slam_params.yaml')
    nav2_params = os.path.join(src_dir, 'nav2_params.yaml')

    odom_node = ExecuteProcess(
        cmd=['python3', '/opt/go1/odom_node.py'],
        output='screen'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,
        }.items()
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
        }.items()
    )

    aruco_node = ExecuteProcess(
        cmd=['python3', os.path.join(src_dir, 'aruco_node.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen'
    )

    return LaunchDescription([
        odom_node,
        slam_launch,
        nav2_launch,
        aruco_node,
    ])
