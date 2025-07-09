import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='startlocal').find('startlocal')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(pkg_share, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='local.lua')
    load_state_filename = LaunchConfiguration('load_state_filename', default=os.path.join(pkg_share, 'map', 'my_map.pbstream'))

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', configuration_directory,
            '-configuration_basename', configuration_basename,
            '-load_state_filename', load_state_filename
        ]
    )

    cartographer_occupancy_grid_node = Node(
    package='cartographer_ros',
    executable='cartographer_occupancy_grid_node',
    name='cartographer_occupancy_grid_node',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
    arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
    )

    return LaunchDescription([
    cartographer_node,
    cartographer_occupancy_grid_node
])

