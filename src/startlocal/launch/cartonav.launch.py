import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('startlocal')

    # Launch parameters
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(pkg_share, 'map', 'my_map.yaml'))
    params_file = LaunchConfiguration('params_file', default=os.path.join(pkg_share, 'config', 'nav2.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Nav2 bringup launch file
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rviz2_config = os.path.join(pkg_share,'rviz','robot2.rviz')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'True',
            # Don't bring up AMCL because we use Cartographer
            'use_composition': 'True',
            'use_respawn': 'False'
        }.items()
    )

        # 添加 rviz2 节点
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz2_config],
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     output='screen'
    #)

    return LaunchDescription([
        nav2_launch
    ])
