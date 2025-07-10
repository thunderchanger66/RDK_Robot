#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from toolmsg.msg import ToolPosition
from toolmsg.srv import ToolDetection
from threading import Lock

# ====== 引入 BPU 模型相关依赖 ======
from BPU import YOLO11_Detect

def load_bpu_model():
    class Args:
        model_path = 'models/test1.bin'
        classes_num = 5
        nms_thres = 0.7
        score_thres = 0.25
        reg = 16
    return YOLO11_Detect(Args())

# 自定义类别名表
custom_class_names = [
    "tape",
    "scissor",
    "screwdriver",
    "plier",
    "knife"
]

class VisionServoNode(Node):
    def __init__(self):
        super().__init__('vision_servo')
        
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_topic', '/camera/image_raw'),
                ('service_name', 'process_latest_image'),
            ]
        )
        
        self.tool_heights = {
            0: 0.12,  # tape
            1: 0.15,  # scissor
            2: 0.1,   # screwdriver
            3: 0.1,   # plier
            4: 0.1    # knife
        }
        
        self.camera_params = {
            'cx': 320.5,
            'cy': 240.5,
            'scale_x': 0.002,
            'scale_y': 0.002
        }

        self.bpu_model = load_bpu_model()
        self.bridge = CvBridge()

        self.latest_frame = None
        self.frame_lock = Lock()  # 线程锁，防止图像访问冲突
        self.image_sub = self.create_subscription(
            Image,
            self.get_parameter('camera_topic').value,
            self.image_callback,
            10
        )
        # 新建一个服务，客户端调用这个服务时触发处理
        self.srv = self.create_service(
            ToolDetection, 
            self.get_parameter('service_name').value,
            self.handle_process_request
        )

    def image_callback(self, msg):
        # 只缓存最新图像，不做处理
        with self.frame_lock:
            self.latest_frame = msg

    def handle_process_request(self, request, response):
        with self.frame_lock:
            if self.latest_frame is None:
                response.success = False
                response.message = "No image received yet."
                response.positions = []
                return response

            try:
                # 图像解码
                frame = self.bridge.imgmsg_to_cv2(self.latest_frame, "bgr8")

                # 模型推理
                input_tensor = self.bpu_model.preprocess_yuv420sp(frame)
                outputs = self.bpu_model.c2numpy(self.bpu_model.forward(input_tensor))
                detections = self.bpu_model.postProcess(outputs)

                # 构建识别结果数组
                positions = []
                for class_id, conf, x1, y1, x2, y2 in detections:
                    x_center = (x1 + x2) / 2
                    y_center = (y1 + y2) / 2
                    world_x = (x_center - self.camera_params['cx']) * self.camera_params['scale_x']
                    world_y = (y_center - self.camera_params['cy']) * self.camera_params['scale_y']

                    tool = ToolPosition()
                    tool.x = float(world_x)
                    tool.y = float(world_y)
                    tool.z = float(self.tool_heights.get(class_id, 0.1))
                    tool.tool_type = custom_class_names[class_id] if class_id < len(custom_class_names) else str(class_id)
                    tool.confidence = conf
                    positions.append(tool)

                # 填充服务响应
                response.positions = positions
                response.success = True
                response.message = f"Detected {len(positions)} tools."
                self.get_logger().info(f"响应了 {len(positions)} 个工具检测结果")
            except Exception as e:
                self.get_logger().error(f"处理图像时出错: {str(e)}")
                response.positions = []
                response.success = False
                response.message = f"Error: {str(e)}"

        return response


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
