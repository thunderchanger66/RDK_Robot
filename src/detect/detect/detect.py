#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from your_pkg.msg import ToolPosition  # 自定义消息类型
import cv2
from ultralytics import YOLO
import numpy as np

class VisionServoNode(Node):
    def __init__(self):
        super().__init__('vision_servo')
        
        # ROS2参数配置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', '/camera/image_raw'),
                ('output_topic', '/tool_position'),
                ('conf_threshold', 0.65),
                ('target_classes', [23, 45])  # YOLO类别ID
            ]
        )
        
        # 工具高度查表 (单位：米)
        self.tool_heights = {
            23: 0.12,   # 扳手
            45: 0.15    # 螺丝刀
        }
        
        # 相机标定参数（需根据实际校准）
        self.camera_params = {
            'cx': 320.5,  # 图像中心x
            'cy': 240.5,  # 图像中心y
            'scale_x': 0.002,  # mm/pixel
            'scale_y': 0.002
        }
        
        # YOLOv8模型初始化
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()
        
        # ROS2接口
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10
        )
        self.pos_pub = self.create_publisher(
            ToolPosition,
            self.get_parameter('output_topic').value,
            10
        )
        
        self.get_logger().info("视觉伺服节点已启动，等待图像输入...")

    def image_callback(self, msg):
        try:
            # 图像转换
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # YOLOv8检测
            results = self.model.predict(
                source=frame,
                conf=self.get_parameter('conf_threshold').value,
                classes=self.get_parameter('target_classes').value,
                verbose=False
            )
            
            if len(results[0].boxes) > 0:
                # 获取置信度最高的检测结果
                best_box = results[0].boxes[0]
                class_id = int(best_box.cls)
                conf = float(best_box.conf)
                
                # 计算中心点坐标
                x_center = (best_box.xyxy[0][0] + best_box.xyxy[0][2]) / 2
                y_center = (best_box.xyxy[0][1] + best_box.xyxy[0][3]) / 2
                
                # 转换为世界坐标系 (2D平面)
                world_x = (x_center - self.camera_params['cx']) * self.camera_params['scale_x']
                world_y = (y_center - self.camera_params['cy']) * self.camera_params['scale_y']
                
                # 创建并发布消息
                tool_msg = ToolPosition()
                tool_msg.x = float(world_x)
                tool_msg.y = float(world_y)
                tool_msg.z = float(self.tool_heights.get(class_id, 0.1))  # 默认高度10cm
                tool_msg.tool_type = str(results[0].names[class_id])
                tool_msg.confidence = conf
                
                self.pos_pub.publish(tool_msg)
                self.get_logger().debug(f"发布工具位姿: {tool_msg}")
                
        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()