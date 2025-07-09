#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from .task_manager import TaskManager, TaskPhase
import time

class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')
        self.task_manager = TaskManager()
        self.running = True

        # 初始化服务客户端
        self.forward_client = self.create_client(Trigger, '/linear_actuator/forward_timed')
        self.reverse_client = self.create_client(Trigger, '/linear_actuator/reverse_timed')
        self.completion_client = self.create_client(Trigger, '/linear_actuator/check_complete')

        self.pub_arm = self.create_publisher(JointState, '/joint_states', 5)
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 5)

        # 等待服务可用
        self._wait_for_services()
            
        # 初始化通信接口
        self.status_pub = self.create_publisher(String, '/task_status', 10)
        self.create_subscription(
            String, '/task_command', self.handle_task_command, 10)
        
        # 使用定时器驱动主循环，避免阻塞spin
        self.create_timer(0.1, self._run_loop)

        self.auto_complete_timer = self.create_timer(10.0, self.auto_complete_task)


    def auto_complete_task(self):
        """每10秒自动标记任务完成(测试用)"""
        current_task = self.task_manager.get_current_task()
        if current_task and current_task.phase != TaskPhase.RELEASE_TOOL:
            self.task_manager.complete_current_task()
            self.publish_status(f"自动完成: {current_task.description}")
            self.get_logger().info(
                f"任务已完成，更新后的队列:\n"
                f"{self.task_manager.print_tasks()}"
            )

    def _wait_for_services(self):
        """等待所需服务可用"""
        while not self.forward_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待正转服务可用...', throttle_duration_sec=5)
        while not self.reverse_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待反转服务可用...', throttle_duration_sec=5)
        while not self.completion_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待完成检查服务可用...', throttle_duration_sec=5)



    def _call_service_async(self, client, done_cb=None):
        """异步调用服务，推荐在定时器/回调中使用"""
        req = Trigger.Request()
        future = client.call_async(req)
        if done_cb:
            future.add_done_callback(done_cb)
        return future
        

    def _on_forward_done(self, future):
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f"服务调用成功: {result.message}")
                # 这里可以推进任务状态
            else:
                self.get_logger().error(f"服务调用失败: {result.message}")
        except Exception as e:
            self.get_logger().error(f"服务回调异常: {str(e)}")



    def _on_check_complete(self, future):
        self.waiting_check = False
        try:
            result = future.result()
            if result.success:
                self.get_logger().info("操作已完成，推进任务")
                self.task_manager.complete_current_task()
                self.service_called = False
            else:
                self.get_logger().info(f"任务未完成: {result.message}")
                # 下次runloop会再次发起check_complete
        except Exception as e:
            self.get_logger().error(f"check_complete回调异常: {str(e)}")



    def publish_arm_position(self, positions):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = [f"joint{i+1}" for i in range(6)]  # ["joint1", ..., "joint6"]
        joint_state.position = [float(p) for p in positions]   # 保证为float类型
        self.pub_arm.publish(joint_state)
        self.get_logger().info(f"已发布关节角度: {joint_state.position}")


    def _run_loop(self):
        if not self.running or not rclpy.ok():
            return

        current_task = self.task_manager.get_current_task()
        if not current_task:
            self.service_called = False
            self.checking_complete = False
            return

        if not hasattr(self, 'last_task_id'):
            self.last_task_id = None
        if self.last_task_id != id(current_task):
            self.service_called = False
            self.checking_complete = False
            self.last_task_id = id(current_task)

        try:
            if current_task.phase == TaskPhase.RELEASE_TOOL:
                if not self.service_called:
                    self.service_called = True
                    self._call_service_async(self.forward_client, self._on_forward_done)
                    self.get_logger().info("已发起正转服务调用，等待操作完成...")

                # 只要没检测到完成且没在等待响应，就发起一次check_complete
                if self.service_called and not getattr(self, 'waiting_check', False):
                    self.waiting_check = True
                    self._call_service_async(self.completion_client, self._on_check_complete)
                    
            if current_task.phase == TaskPhase.GRAB_TOOL:#这里应该换成服务，先这么放一下 

                self.publish_arm_position([3, 2, 0, 0, 0, 0])
                self.task_manager.complete_current_task()

        except Exception as e:
            self.get_logger().error(f"任务执行失败: {str(e)}")
            time.sleep(1)

    def handle_task_command(self, msg):
        """处理任务命令"""
        try:
            parts = msg.data.split(',')
            if len(parts) == 3:
                task_type, obj, location = parts
                if task_type == "搬运":
                    self.task_manager.add_transport_task(obj, location)
                elif task_type == "整理":
                    self.task_manager.add_sort_task()
                
                self.get_logger().info(f"已添加任务: {msg.data}")
                self.get_logger().info(f"当前任务队列:\n{self.task_manager.print_tasks()}")
                
        except Exception as e:
            self.get_logger().error(f"无效任务格式: {msg.data}。错误: {str(e)}")

    def publish_status(self, status):
        """发布状态信息"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        executor = MultiThreadedExecutor()
        node = TaskScheduler()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            node.get_logger().info("收到关闭信号...")
        finally:
            node.destroy_node()
            executor.shutdown()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()