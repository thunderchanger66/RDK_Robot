#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
from math import atan2, acos, sin, cos, pi, asin
from std_srvs.srv import Trigger
from ik_test.srv import SetPose  # 你需要用ros2 interface create创建这个srv
import time

'''
坐标系

向前是x，向左是y ，向上是z，以基底为原点，末端坐标是在夹爪舵机处

朝车头看
                z    y
                |   /
                |  /
                | /
                |/
   x -----------|

'''


class SimpleIKPublisher(Node):
    def __init__(self):
        super().__init__('simple_ik_publisher')
        
        # 初始化参数
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # 初始零位

        self.joint_signs = [-1, 1, -1, -1, -1]  # 关节方向

        # 创建关节状态发布器
        self.joint_pub = self.create_publisher(
            JointState, 
            '/joint_states', 
            10
        )
        
        # 创建目标位姿订阅器
        self.target_sub = self.create_subscription(
            Pose,
            '/target_pose',
            self.target_pose_callback,
            10
        )

        self.set_pose_srv = self.create_service(
            SetPose,
            'move_to_pose',
            self.handle_set_pose
        )


        self.multi_pose_srv = self.create_service(
            Trigger,
            'move_through_poses',
            self.handle_move_through_poses
        )

        # 创建定时器 (10Hz发布频率)
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        
        self.get_logger().info("简易逆运动学发布器已启动")


    def handle_set_pose(self, request, response):
        pose = Pose()
        pose.position.x = request.x
        pose.position.y = request.y
        pose.position.z = request.z
        pose.orientation.x = request.ox
        pose.orientation.y = request.oy
        pose.orientation.z = request.oz
        pose.orientation.w = request.ow
        self.get_logger().info(f"收到move_to_pose服务请求: {pose}")
        self.target_pose_callback(pose)
        response.success = True
        response.message = "已移动到指定位姿"
        return response



    def handle_move_through_poses(self, request, response):
        self.get_logger().info("收到move_through_poses服务请求，依次移动到多个点")
        for pose in self.pose_sequence:
            self.target_pose_callback(pose)
            time.sleep(2)  # 每个点之间等待2秒，可根据实际情况调整
        response.success = True
        response.message = "已依次到达所有预设点"
        return response


    def publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"  # 根据实际情况修改
        msg.name = self.joint_names
        # 这里加上取反
        msg.position = [p * s for p, s in zip(self.joint_positions, self.joint_signs)]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.joint_pub.publish(msg)


    def target_pose_callback(self, msg):
        """处理收到的目标位姿"""
        # 提取位置
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z
        
        # 四元数转欧拉角 (简化的转换)
        q = msg.orientation
        roll = atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2+q.y**2))
        pitch = asin(2*(q.w*q.y - q.z*q.x))
        yaw = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2+q.z**2))
        
        # 构建变换矩阵
        T = self.build_transform_matrix(x, y, z, roll, pitch, yaw)
        
        # 计算逆运动学 (使用之前实现的函数)
        solution = self.inverse_kinematics(T)
        
        if solution is not None:
            self.joint_positions = solution
            self.get_logger().info(f"逆解成功: {np.degrees(solution)}°")
        else:
            self.get_logger().warn("逆解失败，保持上一组关节角度")

    def build_transform_matrix(self, x, y, z, roll, pitch, yaw):
        """构建齐次变换矩阵 (X-Y-Z固定角顺序)"""
        # 计算旋转矩阵
        Rz = np.array([[cos(yaw), -sin(yaw), 0],
                      [sin(yaw), cos(yaw), 0],
                      [0, 0, 1]])
        
        Ry = np.array([[cos(pitch), 0, sin(pitch)],
                      [0, 1, 0],
                      [-sin(pitch), 0, cos(pitch)]])
        
        Rx = np.array([[1, 0, 0],
                      [0, cos(roll), -sin(roll)],
                      [0, sin(roll), cos(roll)]])
        
        R = Rz @ Ry @ Rx
        
        # 构建齐次变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def inverse_kinematics(self, T0t):
        """完整的逆运动学解算实现"""
        # D-H参数 (单位：米)
        l = 0.12273  # 工具长度
        a_i_1 = np.array([0, 0, 0.105, 0.0975, 0.02822])
        d_i = np.array([0.1085, 0, 0, 0, 0.0501])
        
        # 初始化变量
        theta_i = np.zeros((2, 5))
        t234 = np.zeros(2)  # θ2+θ3+θ4
        M = np.zeros(2)
        N = np.zeros(2)
        K = np.zeros(2)
        
        # 工具坐标系转换
        T5t = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, l],
            [0, 0, 0, 1]
        ])
        T05 = T0t @ np.linalg.inv(T5t)
        
        # ------------------- 求解θ1 -------------------
        theta_i[0, 0] = atan2(T05[1, 3], T05[0, 3])
        theta_i[1, 0] = atan2(-T05[1, 3], -T05[0, 3])
        
        # ------------------- 求解θ2+θ3+θ4 -------------------
        for i in range(2):
            if abs(sin(theta_i[i, 0])) < 1e-3:  # sinθ1≈0
                t234[i] = atan2(-T05[0, 2]/cos(theta_i[i, 0]), T05[2, 2])
                if abs(T05[1, 2] + sin(t234[i])*sin(theta_i[i, 0])) > 1e-3:
                    t234[i] = np.nan
            else:
                t234[i] = atan2(-T05[1, 2]/sin(theta_i[i, 0]), T05[2, 2])
                if abs(T05[0, 2] + sin(t234[i])*cos(theta_i[i, 0])) > 1e-3:
                    t234[i] = np.nan
        
        # ------------------- 求解θ5 -------------------
        for i in range(2):
            if abs(sin(t234[i])) < 1e-3:  # sin(θ2+θ3+θ4)≈0
                if abs(T05[2, 2] - 1) < 1e-3:  # cos(θ2+θ3+θ4)≈1
                    theta_i[i, 4] = atan2(-T05[1, 0], -T05[0, 0]) - theta_i[i, 0]
                elif abs(T05[2, 2] + 1) < 1e-3:  # cos(θ2+θ3+θ4)≈-1
                    theta_i[i, 4] = theta_i[i, 0] - atan2(T05[1, 0], T05[0, 0])
            else:
                theta_i[i, 4] = atan2(T05[2, 1]/sin(t234[i]), -T05[2, 0]/sin(t234[i]))
        
        # ------------------- 求解θ3 -------------------
        for i in range(2):
            if abs(sin(theta_i[i, 0])) < 1e-3:
                M[i] = (-T05[0, 3]/cos(theta_i[i, 0])) - a_i_1[4]*cos(t234[i]) - d_i[4]*sin(t234[i])
            else:
                M[i] = (-T05[1, 3]/sin(theta_i[i, 0])) - a_i_1[4]*cos(t234[i]) - d_i[4]*sin(t234[i])
            
            N[i] = T05[2, 3] - d_i[0] + a_i_1[4]*sin(t234[i]) - d_i[4]*cos(t234[i])
            K[i] = (M[i]**2 + N[i]**2 - a_i_1[2]**2 - a_i_1[3]**2) / (2*a_i_1[2]*a_i_1[3])
            
            # 处理数值误差
            if abs(K[i]-1) < 1e-3: K[i] = 1
            elif abs(K[i]+1) < 1e-3: K[i] = -1
            
            if abs(K[i]) > 1:
                theta_i[i, 2] = np.nan
            else:
                theta_i[i, 2] = acos(K[i])
        
        # 扩展解空间 (考虑θ3的正负解)
        theta_i = np.vstack([theta_i, theta_i])
        theta_i[2:, 2] = -theta_i[2:, 2]
        t234 = np.concatenate([t234, t234])
        M = np.concatenate([M, M])
        N = np.concatenate([N, N])
        
        # ------------------- 求解θ2和θ4 -------------------
        valid_solutions = []
        for i in range(4):
            if np.isnan(theta_i[i, 2]):
                continue
                
            P = a_i_1[2] + a_i_1[3]*cos(theta_i[i, 2])
            Q = a_i_1[3]*sin(theta_i[i, 2])
            s2 = (P*M[i] - Q*N[i]) / (P**2 + Q**2)
            c2 = (Q*M[i] + P*N[i]) / (P**2 + Q**2)
            theta_i[i, 1] = atan2(s2, c2)
            theta_i[i, 3] = t234[i] - theta_i[i, 1] - theta_i[i, 2]
            
            # 记录有效解
            if not any(np.isnan(theta_i[i, :])):
                valid_solutions.append(theta_i[i, :])
        
        if not valid_solutions:
            return None
        
        # 转换为numpy数组并处理角度范围
        theta_i = np.array(valid_solutions)
        theta_i = np.where(theta_i > pi, theta_i - 2*pi, theta_i)
        theta_i = np.where(theta_i < -pi, theta_i + 2*pi, theta_i)
        theta_i = np.round(theta_i * 1e3) / 1e3  # 保留3位小数
        
        # 去除重复解
        theta_i = np.unique(theta_i, axis=0)
        
        # ------------------- 选择最优解 -------------------
        # 方案1：选择1-范数最小的解
        norms = np.sum(np.abs(theta_i), axis=1)
        optimal_idx = np.argmin(norms)
        
        # 方案2：选择最接近当前关节位置的解 (如需连续运动)
        # current_angles = np.array(self.joint_positions)
        # diffs = np.sum((theta_i - current_angles)**2, axis=1)
        # optimal_idx = np.argmin(diffs)
        
        return theta_i[optimal_idx]



def main(args=None):
    rclpy.init(args=args)
    node = SimpleIKPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点已关闭")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()