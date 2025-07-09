import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import asyncio
import websockets
import base64
import json
import sys

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_websocket_publisher')
        self.subscription = self.create_subscription(
            CompressedImage,
            'camera/image/compressed',  # 替换为你的图像话题
            self.image_callback,
            10)
        self.clients = []  # 初始化为实例变量

    async def start_websocket_server(self):
        async with websockets.serve(self.handle_connection, "0.0.0.0", 8765):
            await asyncio.Future()  # 运行直到被取消

    async def handle_connection(self, websocket):
        self.clients.append(websocket)
        try:
            await websocket.wait_closed()
        finally:
            self.clients.remove(websocket)

    def image_callback(self, msg):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.send_image(msg))

    async def send_image(self, msg):
        image_data = base64.b64encode(msg.data).decode('utf-8')
        data = {
            "type": "image",
            "data": image_data
        }
        for client in self.clients:
            try:
                await client.send(json.dumps(data))
            except Exception as e:
                print(f"Error sending data to client: {e}")
                self.clients.remove(client)

def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    asyncio.get_event_loop().run_until_complete(image_publisher.start_websocket_server())
    rclpy.spin(image_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()