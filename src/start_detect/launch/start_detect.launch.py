from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('CAM_TYPE', 'usb'),
        Node(
            package='start_detect',
            executable='start_detect',
            name='start_detect_node',
            output='screen'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('face_landmarks_detection'),
                    'launch',
                    'body_det_face_landmarks_det.launch.py'
                )
            )
        ),
    ])
