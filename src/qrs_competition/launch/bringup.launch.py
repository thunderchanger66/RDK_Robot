from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 包含人脸识别启动文件
    start_detect_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('start_detect'),
                'launch',
                'start_detect.launch.py'
            ])
        ])
    )

    # 包含web服务启动文件
    web_service_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('web_service_pkg'),
                'launch',
                'start_web_service.launch.py'
            ])
        ])
    )

    # 返回启动描述
    return LaunchDescription([
        start_detect_launch,
        web_service_launch,
    ])