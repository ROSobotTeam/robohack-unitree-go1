FROM ros:humble-ros-base-jammy AS interfaces

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential git python3-colcon-common-extensions \
        ros-humble-rosidl-default-generators ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

ARG UNITREE_REF=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88
WORKDIR /opt/unitree_ws
RUN git init /tmp/unitree_ros2 \
    && git -C /tmp/unitree_ros2 fetch --depth 1 \
        https://github.com/unitreerobotics/unitree_ros2.git "$UNITREE_REF" \
    && git -C /tmp/unitree_ros2 checkout --detach FETCH_HEAD \
    && mkdir -p src \
    && cp -a /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go src/ \
    && source /opt/ros/humble/setup.bash \
    && colcon build --packages-select unitree_go --merge-install \
        --cmake-args -DBUILD_TESTING=OFF

FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        python3-pip libgl1 libglfw3 \
        ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-default-runtime \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir "numpy==1.26.4" "mujoco==3.3.7" \
    && useradd --create-home --uid 10001 participant

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    MUJOCO_GL=disable \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

COPY --from=interfaces /opt/unitree_ws/install /opt/unitree_ws/install
WORKDIR /opt/go1
COPY safety_checks.py collision_checks.py safety_block.py ping.json ./
COPY go1_policy.py policy.npz t3_controller.py ./
COPY model/ /opt/go1/model/
COPY --chmod=0755 entrypoint.sh /entrypoint.sh

USER participant
RUN source /opt/ros/humble/setup.bash \
    && source /opt/unitree_ws/install/local_setup.bash \
    && python3 -c 'import mujoco; from unitree_go.msg import LowCmd; from collision_checks import CollisionChecker; checker = CollisionChecker("/opt/go1/model"); assert mujoco.__version__ == "3.3.7" and len(LowCmd().motor_cmd) == 20 and checker.pose_is_clear(checker.home_target)'

ENTRYPOINT ["/entrypoint.sh"]
