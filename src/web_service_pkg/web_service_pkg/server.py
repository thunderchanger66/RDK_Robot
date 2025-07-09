#!/usr/bin/env python3

"""
WebRTC 视频流服务器模块
提供 ROS2 图像话题到 WebRTC 的视频流转换与传输功能
"""

# 系统库导入
import argparse
import asyncio
import json
import logging
import os
import ssl
import uuid
import time
import threading
from fractions import Fraction
import numpy as np

# 第三方库导入
import cv2
import rclpy
import cv_bridge
from aiohttp import web
from av import VideoFrame
from aiortc import (
    MediaStreamTrack,
    RTCPeerConnection, 
    RTCSessionDescription,
    RTCRtpSender
)

# ROS相关导入
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

# 常量定义
ROOT = "/home/sunrise/ros_ws/src/web_service_pkg/resource/video/"  # 根目录
CSS_ROOT = os.path.join(ROOT, "css")      # CSS文件目录
JS_ROOT = os.path.join(ROOT, "js")        # JavaScript文件目录
IMAGE_ROOT = os.path.join(ROOT, "image") # 图片目录

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8001

# 视频质量配置
VIDEO_QUALITY_SETTINGS = {
    (0, 3): {'resolution': (1920, 1080), 'fps': 60},    # 低延迟
    (4, 7): {'resolution': (820, 720), 'fps': 30},      # 中等延迟
    (8, 11): {'resolution': (640, 480), 'fps': 15},     # 较高延迟
    (12, float('inf')): {'resolution': (320, 240), 'fps': 5}   # 高延迟
}

# 日志配置
logger = logging.getLogger("webrtc_server")
pcs = set()  # 存储活动的RTCPeerConnection对象

class ImageProcessor(Node):
    """图像处理节点类,负责接收和处理ROS图像消息"""
    
    def __init__(self, mode="auto"):
        """初始化图像处理节点"""
        super().__init__('image_processor')
        self._init_ros_components()
        self._init_variables(mode)
        logger.info(f"图像处理节点已初始化: 模式={mode}")

    def _init_ros_components(self):
        """初始化ROS相关组件"""
        self.bridge = cv_bridge.CvBridge()
        self.image_subscriber = self.create_subscription(
            CompressedImage,
            '/image',
            self._image_callback,
            10
        )

    def _init_variables(self, mode):
        """初始化类变量"""
        self.mode = mode
        self.last_time = time.time()
        self.fps = 30
        self.lock = threading.Lock()
        self.current_image = None
        self.rtt = None
        self.resolution = (1280, 720)
        # 修改占位图片路径
        placeholder_path = os.path.join(IMAGE_ROOT, "placeholder.jpg")
        if os.path.exists(placeholder_path):
            self.placeholder_image = cv2.imread(placeholder_path)
        else:
            logger.warning("占位图片不存在: %s", placeholder_path)
            self.placeholder_image = np.zeros((720, 1280, 3), dtype=np.uint8)


    def _image_callback(self, msg):
        """处理接收到的图像消息"""
        current_time = time.time()
        if current_time - self.last_time >= 1.0 / self.fps:
            try:
                self._process_image(msg)
                self.last_time = current_time
            except Exception as e:
                logger.error(f"图像处理错误: {str(e)}")

    def _process_image(self, msg):
        """处理和调整图像"""
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        with self.lock:
            if self.mode == "auto":
                self._adjust_quality()
            self.current_image = cv2.resize(cv_image, self.resolution)

    def _adjust_quality(self):
        """根据网络状况调整视频质量"""
        if self.rtt is None:
            return

        for (min_rtt, max_rtt), settings in VIDEO_QUALITY_SETTINGS.items():
            if min_rtt <= self.rtt < max_rtt:
                self.fps = settings['fps']
                self.resolution = settings['resolution']
                logger.debug(f"已调整视频质量: FPS={self.fps}, 分辨率={self.resolution}")
                break

    def get_current_frame(self):
        """获取当前图像帧"""
        with self.lock:
            return (self.current_image 
                   if self.current_image is not None 
                   else self.placeholder_image)

class VideoStreamTrack(MediaStreamTrack):
    """视频流轨道类,处理WebRTC视频传输"""

    kind = "video"

    def __init__(self, image_processor):
        """初始化视频流轨道"""
        super().__init__()
        self.image_processor = image_processor
        self.start_time = time.time()
        self.frame_count = 0

    async def next_timestamp(self):
        """计算下一帧时间戳"""
        self.frame_count += 1
        next_time = self.start_time + (self.frame_count / self.image_processor.fps)
        await asyncio.sleep(max(0, next_time - time.time()))
        return int((next_time - self.start_time) * 1000)

    async def recv(self):
        """接收并返回下一个视频帧"""
        frame = self.image_processor.get_current_frame()
        video_frame = VideoFrame.from_ndarray(frame, format="bgr24")
        video_frame.pts = await self.next_timestamp()
        video_frame.time_base = Fraction(1, 1000)
        return video_frame

class WebRTCServer:
    """WebRTC服务器类,处理信令和连接管理"""

    def __init__(self, image_processor):
        """初始化WebRTC服务器"""
        self.image_processor = image_processor
        self.app = web.Application()
        self._setup_routes()

    def _setup_routes(self):
        """设置HTTP路由"""
        self.app.on_shutdown.append(self._shutdown)
        self.app.router.add_get("/", self._handle_index)
        self.app.router.add_get("/js/{filename}", self._handle_js)
        self.app.router.add_get("/css/{filename}", self._handle_css)
        self.app.router.add_get("/images/{filename}", self._handle_image)
        self.app.router.add_post("/offer", self._handle_offer)


    async def _handle_index(self, request):
        """处理主页请求"""
        content = open(os.path.join(ROOT, "index.html"), "r").read()
        return web.Response(content_type="text/html", text=content)

    async def _handle_js(self, request):
        """处理 JavaScript 文件请求"""
        filename = request.match_info['filename']
        filepath = os.path.join(JS_ROOT, filename)
        if not os.path.exists(filepath):
            return web.Response(status=404)
        content = open(filepath, "r").read()
        return web.Response(content_type="application/javascript", text=content)

    async def _handle_css(self, request):
        """处理 CSS 文件请求"""
        filename = request.match_info['filename']
        filepath = os.path.join(CSS_ROOT, filename)
        if not os.path.exists(filepath):
            return web.Response(status=404)
        content = open(filepath, "r").read()
        return web.Response(content_type="text/css", text=content)

    async def _handle_image(self, request):
        """处理图片文件请求"""
        filename = request.match_info['filename']
        filepath = os.path.join(IMAGE_ROOT, filename)
        if not os.path.exists(filepath):
            return web.Response(status=404)
        with open(filepath, 'rb') as f:
            content = f.read()
        return web.Response(body=content, content_type='image/jpeg')


    async def _handle_offer(self, request):
        """处理WebRTC offer请求"""
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])
        pc = await self._create_peer_connection()
        
        # 设置远程描述
        await pc.setRemoteDescription(offer)
        
        # 创建应答
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({
                "sdp": pc.localDescription.sdp,
                "type": pc.localDescription.type
            })
        )

    async def _create_peer_connection(self):
        """创建新的对等连接"""
        pc = RTCPeerConnection()
        pc_id = str(uuid.uuid4())
        pcs.add(pc)

        @pc.on("datachannel")
        def on_datachannel(channel):
            self._setup_data_channel(channel, pc_id)

        @pc.on("iceconnectionstatechange")
        async def on_ice_connection_state_change():
            if pc.iceConnectionState == "failed":
                await pc.close()
                pcs.discard(pc)

        # 添加视频轨道
        pc.addTrack(VideoStreamTrack(self.image_processor))

        return pc

    def _setup_data_channel(self, channel, pc_id):
        """设置数据通道"""
        @channel.on("message")
        def on_message(message):
            if isinstance(message, str):
                if message.startswith("ping"):
                    channel.send("pong" + message[4:])
                elif message.startswith("latency"):
                    self.image_processor.rtt = int(message[7:])

    async def _shutdown(self, app):
        """关闭服务器"""
        coros = [pc.close() for pc in pcs]
        await asyncio.gather(*coros)
        pcs.clear()

def main(args=None):
    """主函数"""
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="WebRTC视频流服务器")
    parser.add_argument("--host", default=DEFAULT_HOST, help="服务器主机地址")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="服务器端口")
    parser.add_argument("--cert-file", help="SSL证书文件")
    parser.add_argument("--key-file", help="SSL密钥文件")
    parser.add_argument("--mode", choices=["auto", "manual"], default="auto",
                       help="视频质量控制模式")
    parser.add_argument("--verbose", "-v", action="store_true", help="启用详细日志")
    args = parser.parse_args(args)

    # 配置日志
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format='%(asctime)s %(levelname)s %(message)s'
    )

    # 配置SSL
    ssl_context = None
    if args.cert_file:
        ssl_context = ssl.SSLContext()
        ssl_context.load_cert_chain(args.cert_file, args.key_file)

    # 初始化ROS
    rclpy.init()
    image_processor = ImageProcessor(mode=args.mode)
    
    # 启动ROS线程
    ros_thread = threading.Thread(
        target=lambda: rclpy.spin(image_processor),
        daemon=True
    )
    ros_thread.start()

    # 创建并启动服务器
    server = WebRTCServer(image_processor)
    web.run_app(
        server.app,
        host=args.host,
        port=args.port,
        ssl_context=ssl_context,
        access_log=logger
    )

if __name__ == "__main__":
    main()