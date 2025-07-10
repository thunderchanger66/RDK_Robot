from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取功能包路径
    lidar_pkg = FindPackageShare('lslidar_driver').find('lslidar_driver')
    visualrobot_pkg = FindPackageShare('visual_robot').find('visual_robot')

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg, 'launch', 'lsn10_launch.py')
        )
    )

    visual_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(visualrobot_pkg, 'launch', 'visual_robot.launch.py')
        )
    )

    robot_driver = Node(
        package='robot_driver',
        executable='robot_driver_node',
        name='robot_driver'
    )

    arm_server = Node(
        package='arm_service',
        executable='arm_server.py',
        name='arm_service'
    )

    linear_act = Node(
        package='linear_actuator',
        executable='linear_actuator',
        name='linear_actuator'
    )

    return LaunchDescription([
        robot_driver,
        lidar_launch,
        visual_robot_launch,
        arm_server,
        linear_act
    ])
