#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from custom_interfaces.srv import GetJointPose
import numpy as np

class ArmServer(Node):
    def __init__(self):
        super().__init__('arm_server')
        
        # 当前关节状态
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 预定义姿态库
        self.poses = {
            'home': [0, 0, 0, 0, 0, 0],
            'ready': [0, -np.pi/2, np.pi/2, 0, np.pi/2, 0],
            'straight': [0, 0, 0, 0, 0, 0]
        }
        
        # 创建服务
        self.srv = self.create_service(
            GetJointPose,
            '/arm/get_pose',
            self.pose_callback)
            
        # 关节状态订阅
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10)
            
        # 关节状态发布
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_command',
            10)
            
        self.get_logger().info("机械臂服务端已启动")

    def joint_callback(self, msg):
        """更新当前关节状态"""
        self.current_joints = list(msg.position)
        
    def pose_callback(self, request, response):
        """处理姿态请求"""
        pose_name = request.pose_name
        
        if pose_name in self.poses:
            target_angles = self.poses[pose_name]
            
            # 创建JointState消息
            joint_cmd = JointState()
            joint_cmd.header.stamp = self.get_clock().now().to_msg()
            joint_cmd.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            joint_cmd.position = target_angles
            
            # 发布目标姿态
            self.joint_pub.publish(joint_cmd)
            
            response.joint_angles = target_angles
            response.success = True
            response.message = f"成功切换到姿态: {pose_name}"
        else:
            response.joint_angles = self.current_joints
            response.success = False
            response.message = f"未知姿态: {pose_name}"
            
        return response

def main(args=None):
    rclpy.init(args=args)
    server = ArmServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()