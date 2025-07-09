import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ai_msgs.msg import PerceptionTargets
import cv2
import numpy as np
import threading
import pandas as pd
import os
import dlib
from std_msgs.msg import String

flag = 0

class CompressedImageViewer(Node):
    def __init__(self):
        super().__init__('start_detect_node')
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
        self.name_pub = self.create_publisher(String, '/face_name', 10)#发布识别到的人的名字

        self.current_image = None
        self.image_mutex = threading.Lock()

        # 人脸识别相关初始化
        resources_path = "/home/sunrise/ros_ws/src/start_detect/resource"
        faceDB_path = os.path.join(resources_path, "featureMean/")
        head = [f"feature_{i+1}" for i in range(128)]
        face_path = os.path.join(faceDB_path, "feature_all.csv")
        self.face_feature = pd.read_csv(face_path, names=head)
        self.face_feature_array = np.array(self.face_feature)
        faceS_path = os.path.join(resources_path, "faceS")
        self.face_list = sorted([name for name in os.listdir(faceS_path) if os.path.isdir(os.path.join(faceS_path, name))])
        # dlib模型加载
        model_path = os.path.join(resources_path, "dlib_face_recognition_resnet_model_v1.dat")
        self.facerec = dlib.face_recognition_model_v1(model_path)

    def compute_dst(self, feature_1, feature_2):
        feature_1 = np.array(feature_1)
        feature_2 = np.array(feature_2)
        dist = np.linalg.norm(feature_1 - feature_2)
        return dist

    def image_callback(self, msg):
        image_data = np.array(msg.data, dtype=np.uint8)
        image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().warn("Failed to decode image.")
            return

        with self.image_mutex:
            self.current_image = image

    def target_callback(self, msg):
        global flag
        with self.image_mutex:
            if self.current_image is None:
                return
            image = self.current_image.copy()

        if flag == 6:
            for target in msg.targets:
                for kps in target.points:
                    if kps.type != "face_kps":
                        continue
                    if len(kps.point) < 106:
                        continue  # 106点不足
                    # 用106点映射成dlib 68点
                    rect = dlib.rectangle(0, 0, image.shape[1], image.shape[0])
                    parts = [dlib.point(int(kps.point[idx].x), int(kps.point[idx].y)) for idx in mapping_106_to_68]
                    shape = dlib.full_object_detection(rect, parts)
                    # 用dlib提取特征
                    face_descriptor = self.facerec.compute_face_descriptor(image, shape)
                    v = np.array(face_descriptor)
                    flag = 0
                    for j, db_feature in enumerate(self.face_feature_array):
                        if self.compute_dst(v, db_feature) < 0.5:
                            flag = 1
                            name = self.face_list[j] if j < len(self.face_list) else "Unknown"
                            mesg = String()
                            mesg.data = name
                            self.name_pub.publish(mesg)
                            #cv2.putText(image, name, (int(kps.point[0].x), int(kps.point[0].y)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
                            break
                    if flag == 0:
                        #cv2.putText(image, "Unknown", (int(kps.point[0].x), int(kps.point[0].y)), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 255), 1, cv2.LINE_AA)
                        mesg = String()
                        mesg.data = "Unknown"
                        self.name_pub.publish(mesg)
                    # 画关键点
                    # for p in parts:
                    #     x = int(p.x)
                    #     y = int(p.y)
                    #     cv2.circle(image, (x, y), 2, (255, 0, 0), -1)
            flag = 0

        flag += 1
        # cv2.imshow("with Keypoints", image)
        # cv2.waitKey(1)

# 106点转dlib 68点映射表
mapping_106_to_68 = [
    # 脸轮廓（0-32）
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32,
    # 右眉（33-37）
    33, 34, 35, 36, 37,
    # 左眉（42-46）
    42, 43, 44, 45, 46,
    # 鼻梁（51-54）
    51, 52, 53, 54,
    # 鼻翼（58-62）
    58, 59, 60, 61, 62,
    # 右眼（66-73）
    66, 67, 69, 70, 71, 73,
    # 左眼（75-82）
    75, 76, 78, 79, 80, 82,
    # 外嘴（84-95）
    84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
    # 内嘴（96-104）
    96, 97, 98, 99, 100, 101, 102, 103, 104
][:68]

def main(args=None):
    rclpy.init(args=args)
    node = CompressedImageViewer()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
