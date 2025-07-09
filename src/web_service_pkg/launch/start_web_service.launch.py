from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # 启动 rosbridge_websocket
    rosbridge_process = ExecuteProcess(
        cmd=['ros2', 'run', 'rosbridge_server', 'rosbridge_websocket'],
        name='rosbridge_websocket',
        output='screen'
    )

    # 启动 WebRTC 服务器节点，使用 ExecuteProcess 而不是 Node
    webrtc_server_process = ExecuteProcess(
        cmd=['ros2', 'run', 'web_service_pkg', 'server',
             '--host', '0.0.0.0',
             '--port', '8001',
             '--mode', 'auto'],
        name='webrtc_server',
        output='screen'
    )

    # 返回启动描述
    return LaunchDescription([
        rosbridge_process,
        webrtc_server_process,
    ])