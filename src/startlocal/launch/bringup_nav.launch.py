from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取子 launch 文件路径
    pkg_path = FindPackageShare('startlocal').find('startlocal')

    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'cartolocal.launch.py')
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'cartonav.launch.py')
        )
    )

    return LaunchDescription([
        carto_launch,
        nav2_launch
    ])
