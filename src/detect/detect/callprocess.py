from toolmsg.srv import ToolDetection
import rclpy
from rclpy.node import Node

class Client(Node):
    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(ToolDetection, 'process_latest_image')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务还没启动...')
        self.req = ToolDetection.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            for tool in future.result().positions:
                print(f"{tool.tool_type}: x={tool.x:.3f}, y={tool.y:.3f}, z={tool.z:.3f}, confidence={tool.confidence:.2f}")
        else:
            print("服务调用失败")

def main():
    rclpy.init()
    client = Client()
    client.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
