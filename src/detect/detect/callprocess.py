#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ImageProcessClient(Node):
    def __init__(self):
        super().__init__('image_process_client')
        self.cli = self.create_client(Trigger, 'process_latest_image')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('服务 [process_latest_image] 不可用，等待中...')

        self.req = Trigger.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'调用结果: success={future.result().success}, message="{future.result().message}"')
        else:
            self.get_logger().error('服务调用失败')


def main(args=None):
    rclpy.init(args=args)
    client = ImageProcessClient()
    client.send_request()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
