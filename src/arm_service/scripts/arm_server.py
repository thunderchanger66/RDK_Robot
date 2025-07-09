#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import numpy as np
from math import atan2, acos, sin, cos, pi, asin
from std_srvs.srv import Trigger
from arm_service.srv import SetPose, SetJoints

class SimpleIKPublisher(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # 初始化参数
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_signs = [-1, 1, -1, -1, -1, 1]  # 关节方向补偿

        # 动作序列控制变量
        self.sequence = self._create_default_sequence()  # 默认动作序列
        self.current_step = 0
        self.is_running_sequence = False
        self.last_step_time = self.get_clock().now()

        # 创建发布器和订阅器
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.target_sub = self.create_subscription(Pose, '/target_pose', self.target_pose_callback, 10)

        # 创建服务
        self.set_pose_srv = self.create_service(SetPose, 'move_to_pose', self.handle_set_pose)
        self.set_joints_srv = self.create_service(SetJoints, 'move_to_joints', self.handle_set_joints)
        self.start_sequence_srv = self.create_service(Trigger, 'start_sequence', self.handle_start_sequence)
        self.stop_sequence_srv = self.create_service(Trigger, 'stop_sequence', self.handle_stop_sequence)

        # 主控制定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("机械臂控制器已启动，准备就绪")

    def _create_default_sequence(self):
        """创建默认动作序列"""
        return [
            {'type': 'joints', 'joints': [-1.47, 0.25, -1.05, -1.45, 0.0], 'gripper_open': True},
            {'type': 'joints', 'joints': [-1.47, 0.35, -1.57, -0.9, 0.0], 'gripper_open': True},
            {'type': 'joints', 'joints': [-1.47, -0.4, -1.57, -0.9, 0.0], 'gripper_open': True},
            {'type': 'joints', 'joints': [-1.47, -0.4, -1.57, -0.9, 0.0], 'gripper_open': False},
            {'type': 'joints', 'joints': [-1.47, -0.4, -0.71, -1.1, 0.0], 'gripper_open': False},
            {'type': 'joints', 'joints': [-1.47, -0.0, -0.8, -1.1, 0.0], 'gripper_open': False},
            {'type': 'joints', 'joints': [-1.47, 0.0, 0.0, -1.1, 0.0], 'gripper_open': False},
            {'type': 'joints', 'joints': [1.47, 0.0, 0.0, -1.1, 0.0], 'gripper_open': False},
            {'type': 'joints', 'joints': [1.47, -0.2, -0.8, -0.7, 0.0], 'gripper_open': False},
            {'type': 'joints', 'joints': [1.47, -0.2, -0.8, -0.7, 0.0], 'gripper_open': True},
            {'type': 'joints', 'joints': [1.47, 0.0, 0.0, 0.0, 0.0], 'gripper_open': True},
            {'type': 'joints', 'joints': [-1.47, 0.0, 0.0, 0.0, 0.0], 'gripper_open': True},
            {'type': 'joints', 'joints': [-1.47, 0.25, -1.05, -1.45, 0.0], 'gripper_open': True}
        ]

    def create_pose(self, x, y, z, ox, oy, oz, ow):
        """创建Pose消息并检查四元数归一化"""
        norm = (ox**2 + oy**2 + oz**2 + ow**2)**0.5
        if abs(norm - 1.0) > 1e-3:
            self.get_logger().warn(f"四元数未归一化 (norm={norm:.4f})，已自动修正")
            ox, oy, oz, ow = ox/norm, oy/norm, oz/norm, ow/norm
            
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = ox
        pose.orientation.y = oy
        pose.orientation.z = oz
        pose.orientation.w = ow
        return pose

    def handle_set_pose(self, request, response):
        """位姿控制服务回调"""
        try:
            pose = self.create_pose(
                request.x, request.y, request.z,
                request.ox, request.oy, request.oz, request.ow
            )
            self.target_pose_callback(pose)
            
            self.set_gripper(request.open)
            response.success = True
            response.message = "位姿控制成功"
        except Exception as e:
            response.success = False
            response.message = f"位姿控制失败: {str(e)}"
        return response

    def handle_set_joints(self, request, response):
        """关节角度控制服务回调"""
        try:
            if len(request.joints) != 5:
                raise ValueError("需要5个关节角度值")
                
            self.joint_positions[:5] = request.joints
            self.set_gripper(request.open)
            
            response.success = True
            response.message = "关节控制成功"
        except Exception as e:
            response.success = False
            response.message = f"关节控制失败: {str(e)}"
        return response

    def handle_start_sequence(self, request, response):
        """开始执行序列服务回调"""
        if self.is_running_sequence:
            response.success = False
            response.message = "已有动作序列正在执行"
            return response
            
        self.current_step = 0
        self.is_running_sequence = True
        self.last_step_time = self.get_clock().now()
        
        response.success = True
        response.message = "开始执行动作序列"
        return response

    def handle_stop_sequence(self, request, response):
        """停止动作序列服务回调"""
        self.is_running_sequence = False
        response.success = True
        response.message = "已停止动作序列"
        return response

    def set_gripper(self, open_gripper):
        """设置夹爪状态"""
        self.joint_positions[5] = 0.5 if open_gripper else -1.3
        self.get_logger().info(f"夹爪已{'张开' if open_gripper else '闭合'}")

    def control_loop(self):
        """主控制循环"""
        # 发布当前关节状态
        self.publish_joint_states()
        
        # 处理动作序列
        if self.is_running_sequence:
            current_time = self.get_clock().now()
            if (current_time - self.last_step_time).nanoseconds > 2e9:  # 2秒间隔
                self.execute_next_step()

    def publish_joint_states(self):
        """发布关节状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = self.joint_names
        msg.position = [p * s for p, s in zip(self.joint_positions, self.joint_signs)]
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)
        self.joint_pub.publish(msg)

    def execute_next_step(self):
        """执行序列中的下一步动作"""
        if self.current_step >= len(self.sequence):
            self.is_running_sequence = False
            self.get_logger().info("动作序列执行完成")
            return

        step = self.sequence[self.current_step]
        
        try:
            if step['type'] == 'pose':
                self.target_pose_callback(step['pose'])
            elif step['type'] == 'joints':
                self.joint_positions[:5] = step['joints']
            
            self.set_gripper(step['gripper_open'])
            self.current_step += 1
            self.last_step_time = self.get_clock().now()
            self.get_logger().info(f"执行步骤 {self.current_step}/{len(self.sequence)}")
            
        except Exception as e:
            self.get_logger().error(f"步骤执行失败: {str(e)}")
            self.is_running_sequence = False

    def target_pose_callback(self, msg):
        """位姿控制回调"""
        try:
            # 提取位置和方向
            x, y, z = msg.position.x, msg.position.y, msg.position.z
            q = msg.orientation
            
            # 四元数转欧拉角
            roll = atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x**2+q.y**2))
            pitch = asin(2*(q.w*q.y - q.z*q.x))
            yaw = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y**2+q.z**2))
            
            # 计算逆运动学
            T = self.build_transform_matrix(x, y, z, roll, pitch, yaw)
            solution = self.inverse_kinematics(T)
            
            if solution is not None:
                self.joint_positions = solution
                self.get_logger().info(f"逆解成功: {np.degrees(solution[:5])}°")
            else:
                raise ValueError("逆运动学无解")
                
        except Exception as e:
            self.get_logger().error(f"位姿控制失败: {str(e)}")
            raise

    def build_transform_matrix(self, x, y, z, roll, pitch, yaw):
        """构建齐次变换矩阵"""
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
        
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [x, y, z]
        return T

    def inverse_kinematics(self, T0t):
        """逆运动学解算"""
        # D-H参数
        l = 0.12273  # 工具长度
        a_i_1 = np.array([0, 0, 0.105, 0.0975, 0.02822])
        d_i = np.array([0.1085, 0, 0, 0, 0.0501])
        
        # 初始化变量
        theta_i = np.zeros((2, 5))
        t234 = np.zeros(2)
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
        
        # 求解θ1
        theta_i[0, 0] = atan2(T05[1, 3], T05[0, 3])
        theta_i[1, 0] = atan2(-T05[1, 3], -T05[0, 3])
        
        # 求解θ2+θ3+θ4
        for i in range(2):
            if abs(sin(theta_i[i, 0])) < 1e-3:
                t234[i] = atan2(-T05[0, 2]/cos(theta_i[i, 0]), T05[2, 2])
                if abs(T05[1, 2] + sin(t234[i])*sin(theta_i[i, 0])) > 1e-3:
                    t234[i] = np.nan
            else:
                t234[i] = atan2(-T05[1, 2]/sin(theta_i[i, 0]), T05[2, 2])
                if abs(T05[0, 2] + sin(t234[i])*cos(theta_i[i, 0])) > 1e-3:
                    t234[i] = np.nan
        
        # 求解θ5
        for i in range(2):
            if abs(sin(t234[i])) < 1e-3:
                if abs(T05[2, 2] - 1) < 1e-3:
                    theta_i[i, 4] = atan2(-T05[1, 0], -T05[0, 0]) - theta_i[i, 0]
                elif abs(T05[2, 2] + 1) < 1e-3:
                    theta_i[i, 4] = theta_i[i, 0] - atan2(T05[1, 0], T05[0, 0])
            else:
                theta_i[i, 4] = atan2(T05[2, 1]/sin(t234[i]), -T05[2, 0]/sin(t234[i]))
        
        # 求解θ3
        for i in range(2):
            if abs(sin(theta_i[i, 0])) < 1e-3:
                M[i] = (-T05[0, 3]/cos(theta_i[i, 0])) - a_i_1[4]*cos(t234[i]) - d_i[4]*sin(t234[i])
            else:
                M[i] = (-T05[1, 3]/sin(theta_i[i, 0])) - a_i_1[4]*cos(t234[i]) - d_i[4]*sin(t234[i])
            
            N[i] = T05[2, 3] - d_i[0] + a_i_1[4]*sin(t234[i]) - d_i[4]*cos(t234[i])
            K[i] = (M[i]**2 + N[i]**2 - a_i_1[2]**2 - a_i_1[3]**2) / (2*a_i_1[2]*a_i_1[3])
            
            if abs(K[i]-1) < 1e-3: K[i] = 1
            elif abs(K[i]+1) < 1e-3: K[i] = -1
            
            if abs(K[i]) > 1:
                theta_i[i, 2] = np.nan
            else:
                theta_i[i, 2] = acos(K[i])
        
        # 扩展解空间
        theta_i = np.vstack([theta_i, theta_i])
        theta_i[2:, 2] = -theta_i[2:, 2]
        t234 = np.concatenate([t234, t234])
        M = np.concatenate([M, M])
        N = np.concatenate([N, N])
        
        # 求解θ2和θ4
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
            
            if not any(np.isnan(theta_i[i, :])):
                valid_solutions.append(theta_i[i, :])
        
        if not valid_solutions:
            return None
        
        # 处理解
        theta_i = np.array(valid_solutions)
        theta_i = np.where(theta_i > pi, theta_i - 2*pi, theta_i)
        theta_i = np.where(theta_i < -pi, theta_i + 2*pi, theta_i)
        theta_i = np.round(theta_i * 1e3) / 1e3
        
        # 选择最优解（最接近当前角度）
        current_angles = np.array(self.joint_positions[:5])
        diffs = np.sum((theta_i - current_angles)**2, axis=1)
        optimal_idx = np.argmin(diffs)
        
        # 返回解（保持joint6不变）
        return np.append(theta_i[optimal_idx], self.joint_positions[5])

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