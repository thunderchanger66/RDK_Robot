#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import Hobot.GPIO as GPIO
import time
from threading import Timer

# 使用物理引脚号
IN1 = 11  # BOARD编号11
IN2 = 13  # BOARD编号13
RUN_DURATION = 8  # 运行持续时间(秒)

class linear_actuator(Node):
    def __init__(self):
        super().__init__('linear_actuator')
        
        # 初始化状态变量
        self.operation_complete = True  # 初始状态为已完成
        self.current_operation = None   # 当前操作类型
        
        # 初始化GPIO
        self._setup_gpio()
        self.current_timer = None
        
        # 创建服务
        self.forward_timed_srv = self.create_service(
            Trigger, '/linear_actuator/forward_timed', self.forward_timed_callback)
        self.reverse_timed_srv = self.create_service(
            Trigger, '/linear_actuator/reverse_timed', self.reverse_timed_callback)
        self.stop_srv = self.create_service(
            Trigger, '/linear_actuator/stop', self.stop_callback)
        self.check_complete_srv = self.create_service(
            Trigger, '/linear_actuator/check_complete', self.check_complete_callback)
    
    def _setup_gpio(self):
        """初始化GPIO设置"""
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(IN1, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(IN2, GPIO.OUT, initial=GPIO.LOW)
        self.get_logger().info("GPIO初始化完成")
    
    def _cleanup_gpio(self):
        """清理GPIO资源"""
        self._stop_motor()
        GPIO.cleanup()
        self.get_logger().info("GPIO资源已清理")
    
    def _run_forward(self):
        """正转电机"""
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
        self.operation_complete = False
        self.current_operation = "forward"
        self.get_logger().info("电机正转中...")
    
    def _run_reverse(self):
        """反转电机"""
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)
        self.operation_complete = False
        self.current_operation = "reverse"
        self.get_logger().info("电机反转中...")
    
    def _stop_motor(self):
        """停止电机并取消定时器"""
        if self.current_timer:
            self.current_timer.cancel()
            self.current_timer = None
        
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.LOW)
        
        # 标记操作完成
        if not self.operation_complete:
            self.operation_complete = True
            self.get_logger().info(f"电机{self.current_operation}操作已完成")
            self.current_operation = None
    
    def _start_timed_operation(self, direction_func, operation_name):
        """执行定时电机操作"""
        self._stop_motor()  # 先停止当前任何操作
        direction_func()
        
        # 设置定时停止
        self.current_timer = Timer(RUN_DURATION, self._stop_motor)
        self.current_timer.start()
        self.get_logger().info(f"电机{operation_name}将在{RUN_DURATION}秒后自动停止")
    
    def forward_timed_callback(self, request, response):
        """处理定时正转请求"""
        try:
            self._start_timed_operation(self._run_forward, "正转")
            response.success = True
            response.message = f"电机正转{RUN_DURATION}秒已启动"
        except Exception as e:
            response.success = False
            response.message = f"正转启动失败: {str(e)}"
            self.get_logger().error(response.message)
        return response
    
    def reverse_timed_callback(self, request, response):
        """处理定时反转请求"""
        try:
            self._start_timed_operation(self._run_reverse, "反转")
            response.success = True
            response.message = f"电机反转{RUN_DURATION}秒已启动"
        except Exception as e:
            response.success = False
            response.message = f"反转启动失败: {str(e)}"
            self.get_logger().error(response.message)
        return response
    
    def stop_callback(self, request, response):
        """处理立即停止请求"""
        self._stop_motor()
        response.success = True
        response.message = "电机已立即停止"
        self.get_logger().info(response.message)
        return response
    
    def check_complete_callback(self, request, response):
        """检查操作是否完成"""
        response.success = self.operation_complete
        if self.operation_complete:
            response.message = "操作已完成"
        else:
            response.message = f"操作进行中: {self.current_operation}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = linear_actuator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cleanup_gpio()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()