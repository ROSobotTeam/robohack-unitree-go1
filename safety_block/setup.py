from setuptools import find_packages, setup

package_name = 'safety_block'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSobotTeam',
    maintainer_email='team@example.com',
    description='Safety block for Unitree Go1 T2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'safety_node = safety_block.safety_node:main',
        ],
    },
)