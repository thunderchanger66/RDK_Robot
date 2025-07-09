import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np
import threading


class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('compressed_image_viewer')
        self.sub_image = self.create_subscription(
            CompressedImage,
            '/image',
            self.image_callback,
            10)
        self.sub_target = self.create_subscription(
            PerceptionTargets,
            '/hobot_face_landmarks_detection',
            self.target_callback,
            10)

        self.current_image = None
        self.image_mutex = threading.Lock()

    def image_callback(self, msg):
        image_data = np.array(msg.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn("Failed to decode image.")
            return

        with self.image_mutex:
            self.current_image = image

    def target_callback(self, msg):
        with self.image_mutex:
            if self.current_image is None:
                return
            image = self.current_image.copy()

        for target in msg.targets:
            for kps in target.points:
                if kps.type != "face_kps":
                    continue
                for p in kps.point:
                    x = int(p.x)
                    y = int(p.y)
                    if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                        cv2.circle(image, (x, y), 3, (0, 0, 255), -1)

        cv2.imshow("with Keypoints", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
