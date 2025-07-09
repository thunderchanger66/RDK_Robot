#!/usr/bin/env python3


"""
代码逻辑
创建张大头控制类，用以最基本的控制
创建轮子类，创建底盘类，用以解算麦轮与正运动学
创建云台电机类，创建云台来，用以直接发布角度 ！！！这里还没测试
创建PID类，用以位置控制，姿态控制，角度控制
创建ROS节点，主节点的命令  六个电机同步发布以防串口冲突

TO DO
给云台添加零点位置然后限幅
"""
import serial
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ZDTMotorController:
    def __init__(self, uart_dev, baudrate=115200, timeout=1):
        """
        初始化电机控制器
        :param uart_dev: 串口设备路径 (str)
        :param baudrate: 波特率 (int)
        :param timeout: 串口超时时间 (秒) (float)
        """
        self.uart_dev = uart_dev
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.connect()

    def connect(self):
        """
        连接到串口设备
        """
        try:
            self.ser = serial.Serial(self.uart_dev, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.uart_dev} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Failed to connect to {self.uart_dev}: {e}")
            self.ser = None

    def disconnect(self):
        """
        断开与串口设备的连接
        """
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"Disconnected from {self.uart_dev}.")

    def send_command(self, cmd, timeout=1.0):
        """
        发送命令前检查串口是否忙
        :param cmd: 要发送的命令 (bytearray)
        :param timeout: 等待串口空闲的超时时间 (秒) (float)
        :return: 是否成功发送 (bool)
        """
        if not self.ser or not self.ser.is_open:
            print("Serial port is not open.")
            return False

        start_time = time.time()
        while self.ser.out_waiting > 0:
            if time.time() - start_time > timeout:
                print("Timeout waiting for serial port to be ready.")
                return False
            time.sleep(0.01)  # 短暂休眠，避免CPU占用过高
        time.sleep(0.01)#没这个无法连发
        self.ser.write(cmd)
        return True

    def read_response(self):
        """
        读取串口响应
        :return: 读取到的数据 (bytes)
        """
        if self.ser and self.ser.is_open:
            return self.ser.read_all()
        else:
            print("Serial port is not open.")
            return b''

    def Emm_V5_Read_Sys_Params(self, addr, s):
        """
        读取系统参数
        :param addr: 电机地址 (int)
        :param s: 参数名称 (str)
        """
        func_codes = {
            'S_VER': 0x1F,  # 固件版本
            'S_RL': 0x20,   # 相电阻和相电感
            'S_PID': 0x21,  # PID参数
            'S_VBUS': 0x24, # 总线电压
            'S_CPHA': 0x27, # 相电流
            'S_ENCL': 0x31, # 编码器值
            'S_TPOS': 0x33, # 目标位置角度
            'S_VEL': 0x35,  # 实时转速
            'S_CPOS': 0x36, # 实时位置角度
            'S_PERR': 0x37, # 位置误差角度
            'S_FLAG': 0x3A, # 状态标志位
            'S_ORG': 0x3B,  # 回零状态
            'S_Conf': 0x42, # 驱动参数
            'S_State': 0x43 # 系统状态
        }
        if s in func_codes:
            cmd = bytearray([addr, func_codes[s], 0x6B])
            self.send_command(cmd)

    def Emm_V5_Reset_CurPos_To_Zero(self, addr):
        """
        将当前位置清零
        :param addr: 电机地址 (int)
        """
        cmd = bytearray([addr, 0x0A, 0x6D, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Reset_Clog_Pro(self, addr):
        """
        解除堵转保护
        :param addr: 电机地址 (int)
        """
        cmd = bytearray([addr, 0x0E, 0x52, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Modify_Ctrl_Mode(self, addr, svF, ctrl_mode):
        """
        修改控制模式
        :param addr: 电机地址 (int)
        :param svF: 是否存储标志 (bool)
        :param ctrl_mode: 控制模式 (int)
        """
        cmd = bytearray([addr, 0x46, 0x69, 0x01 if svF else 0x00, ctrl_mode, 0x6B])
        self.send_command(cmd)

    def Emm_V5_En_Control(self, addr, state, snF):
        """
        使能/禁用电机控制
        :param addr: 电机地址 (int)
        :param state: 使能状态 (bool)
        :param snF: 多机同步标志 (bool)
        """
        cmd = bytearray([addr, 0xF3, 0xAB, 0x01 if state else 0x00, 0x01 if snF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Vel_Control(self, addr, dir, vel, acc, snF):
        """
        速度控制
        :param addr: 电机地址 (int)
        :param dir: 方向 (0: CW, 1: CCW) (int)
        :param vel: 目标速度 (RPM) (int)
        :param acc: 加速度 (RPM/s) (int)
        :param snF: 多机同步标志 (bool)
        """
        cmd = bytearray([addr, 0xF6, dir, (vel >> 8) & 0xFF, vel & 0xFF, acc, 0x01 if snF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Pos_Control(self, addr, dir, vel, acc, clk, raF, snF):
        """
        位置控制
        :param addr: 电机地址 (int)
        :param dir: 方向 (0: CW, 1: CCW) (int)
        :param vel: 目标速度 (RPM) (int)
        :param acc: 加速度 (RPM/s) (int)
        :param clk: 脉冲数 (int)
        :param raF: 相对/绝对运动标志 (bool)
        :param snF: 多机同步标志 (bool)
        """
        cmd = bytearray([addr, 0xFD, dir, (vel >> 8) & 0xFF, vel & 0xFF, acc, (clk >> 24) & 0xFF, (clk >> 16) & 0xFF, (clk >> 8) & 0xFF, clk & 0xFF, 0x01 if raF else 0x00, 0x01 if snF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Stop_Now(self, addr, snF):
        """
        立即停止电机
        :param addr: 电机地址 (int)
        :param snF: 多机同步标志 (bool)
        """
        cmd = bytearray([addr, 0xFE, 0x98, 0x01 if snF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Synchronous_motion(self):
        """
        执行多机同步运动
        """
        cmd = bytearray([0x00, 0xFF, 0x66, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Origin_Set_O(self, addr, svF):
        """
        设置回零零点位置
        :param addr: 电机地址 (int)
        :param svF: 是否存储标志 (bool)
        """
        cmd = bytearray([addr, 0x93, 0x88, 0x01 if svF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Origin_Modify_Params(self, addr, svF, o_mode, o_dir, o_vel, o_tm, sl_vel, sl_ma, sl_ms, potF):
        """
        修改回零参数
        :param addr: 电机地址 (int)
        :param svF: 是否存储标志 (bool)
        :param o_mode: 回零模式 (int)
        :param o_dir: 回零方向 (int)
        :param o_vel: 回零速度 (RPM) (int)
        :param o_tm: 回零超时时间 (ms) (int)
        :param sl_vel: 无限位碰撞检测转速 (RPM) (int)
        :param sl_ma: 无限位碰撞检测电流 (int)
        :param sl_ms: 无限位碰撞检测时间 (ms) (int)
        :param potF: 上电自动触发回零 (bool)
        """
        cmd = bytearray([addr, 0x4C, 0xAE, 0x01 if svF else 0x00, o_mode, o_dir, (o_vel >> 8) & 0xFF, o_vel & 0xFF, (o_tm >> 24) & 0xFF, (o_tm >> 16) & 0xFF, (o_tm >> 8) & 0xFF, o_tm & 0xFF, (sl_vel >> 8) & 0xFF, sl_vel & 0xFF, (sl_ma >> 8) & 0xFF, sl_ma & 0xFF, (sl_ms >> 8) & 0xFF, sl_ms & 0xFF, 0x01 if potF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Origin_Trigger_Return(self, addr, o_mode, snF):
        """
        触发回零
        :param addr: 电机地址 (int)
        :param o_mode: 回零模式 (int)
        :param snF: 多机同步标志 (bool)
        """
        cmd = bytearray([addr, 0x9A, o_mode, 0x01 if snF else 0x00, 0x6B])
        self.send_command(cmd)

    def Emm_V5_Receive_Data(self):
        """
        接收数据
        :return: 接收到的数据 (str, int)
        """
        i = 0
        rxCmd = bytearray(128)
        lTime = cTime = time.time()
        while True:
            if self.ser.in_waiting > 0:
                if i < 128:
                    rxCmd[i] = self.ser.read(1)[0]
                    i += 1
                    lTime = time.time()
            else:
                cTime = time.time()
                if (cTime - lTime) * 1000 > 100:
                    hex_data = ' '.join(['{:02x}'.format(b) for b in rxCmd[:i]])
                    hex_data = hex_data.strip('00 ')
                    if hex_data and hex_data[0] != '0':
                        hex_data = '0' + hex_data
                    return hex_data, len(hex_data.replace(' ', '')) // 2


class Wheel:
    def __init__(self, motor_controller, addr, diameter, resolution=3200, cw_positive=True):
        """
        初始化轮子对象
        :param motor_controller: ZDTMotorController 实例
        :param addr: 电机地址 (int)
        :param diameter: 轮子直径 (米) (float)
        :param resolution: 每圈的脉冲数 (默认为16细分，即3200脉冲/圈) (int)
        :param cw_positive: 是否顺时针方向为正 (True: CW 为正, False: CCW 为正) (bool)
        """
        self.motor_controller = motor_controller
        self.addr = addr
        self.diameter = diameter  # 轮子直径 (米)
        self.resolution = resolution  # 每圈的脉冲数
        self.cw_positive = cw_positive  # 顺时针方向是否为正

    def calculate_rpm_from_velocity(self, velocity):
        """
        根据线速度计算转速 (RPM)
        :param velocity: 线速度 (米/秒) (float)
        :return: 转速 (RPM) (float)
        """
        circumference = self.diameter * 3.14159  # 轮子周长 (米)
        rotations_per_second = velocity / circumference  # 每秒转数
        rpm = rotations_per_second * 60  # 转换为每分钟转数
        return rpm

    def calculate_pulses_from_angle(self, angle):
        """
        根据角度计算脉冲数
        :param angle: 角度 (度) (float)
        :return: 脉冲数 (int)
        """
        pulses_per_revolution = self.resolution  # 每圈的脉冲数
        pulses = (angle / 360.0) * pulses_per_revolution  # 脉冲数
        return int(pulses)

    def calculate_pulses_from_distance(self, distance):
        """
        根据移动距离计算脉冲数
        :param distance: 移动距离 (米) (float)
        :return: 脉冲数 (int)
        """
        circumference = self.diameter * 3.14159  # 轮子周长 (米)
        revolutions = distance / circumference  # 需要的转数
        pulses = revolutions * self.resolution  # 需要的脉冲数
        return int(pulses)

    def set_velocity(self, velocity, acceleration=10, snF=False):
        """
        设置轮子的线速度
        :param velocity: 线速度 (米/秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param snF: 是否启用多机同步标志 (bool)
        """
        rpm = self.calculate_rpm_from_velocity(velocity)
        direction = 0 if (velocity >= 0 and self.cw_positive) or (velocity < 0 and not self.cw_positive) else 1
        self.motor_controller.Emm_V5_Vel_Control(self.addr, direction, int(abs(rpm)), acceleration, snF)

    def set_angle(self, angle, velocity, acceleration=10, raF=False, snF=False):
        """
        设置轮子的目标角度
        :param angle: 目标角度 (度) (float)
        :param velocity: 运动速度 (米/秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param raF: 相对/绝对运动标志 (True: 相对运动, False: 绝对运动) (bool)
        :param snF: 是否启用多机同步标志 (bool)
        """
        rpm = self.calculate_rpm_from_velocity(velocity)
        pulses = self.calculate_pulses_from_angle(angle)
        direction = 0 if (angle >= 0 and self.cw_positive) or (angle < 0 and not self.cw_positive) else 1
        self.motor_controller.Emm_V5_Pos_Control(self.addr, direction, int(abs(rpm)), acceleration, int(abs(pulses)), raF, snF)

    def set_distance(self, distance, velocity, acceleration=10, raF=False, snF=False):
        """
        设置轮子的移动距离
        :param distance: 移动距离 (米) (float)
        :param velocity: 运动速度 (米/秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param raF: 相对/绝对运动标志 (True: 相对运动, False: 绝对运动) (bool)
        :param snF: 是否启用多机同步标志 (bool)
        """
        rpm = self.calculate_rpm_from_velocity(velocity)
        pulses = self.calculate_pulses_from_distance(distance)
        direction = 0 if (distance >= 0 and self.cw_positive) or (distance < 0 and not self.cw_positive) else 1
        self.motor_controller.Emm_V5_Pos_Control(self.addr, direction, int(abs(rpm)), acceleration, int(abs(pulses)), raF, snF)

    def stop(self, snF=False):
        """
        停止轮子
        :param snF: 是否启用多机同步标志 (bool)
        """
        self.motor_controller.Emm_V5_Stop_Now(self.addr, snF)


'''
        X+
        |
        |
        |
Y+--------
z逆时针为正
'''
class Chassis:
    def __init__(self, ZDTMotorController, Wheel, wheelbase, track_width):
        """
        初始化底盘对象
        :param ZDTMotorController: 电机控制器类
        :param Wheel: 轮子类
        :param wheelbase: 轴距 (米) (float)
        :param track_width: 轮距 (米) (float)
        """
        self.wheelbase = wheelbase
        self.track_width = track_width

        # 创建电机控制器
        self.motor_controller = ZDTMotorController('/dev/ttyS1', 115200)

        # 初始化四个轮子
        self.wheels = [
            Wheel(self.motor_controller, addr=1, diameter=0.08, resolution=3200, cw_positive=True),  # 左前轮，顺时针为正
            Wheel(self.motor_controller, addr=2, diameter=0.08, resolution=3200, cw_positive=False), # 右前轮，逆时针为正
            Wheel(self.motor_controller, addr=3, diameter=0.08, resolution=3200, cw_positive=True),  # 左后轮，顺时针为正
            Wheel(self.motor_controller, addr=4, diameter=0.08, resolution=3200, cw_positive=False)  # 右后轮，逆时针为正
        ]

    def calculate_wheel_distances(self, x_distance, y_distance, z_angle_deg):
        """
        计算每个轮子的移动距离
        :param x_distance: X方向的移动距离 (米) (float)
        :param y_distance: Y方向的移动距离 (米) (float)
        :param z_angle_deg: Z方向的转动角度 (度) (float)
        :return: 每个轮子的移动距离 (list of float)
        """
        z_angle_rad = math.radians(z_angle_deg)  # 将角度从度转换为弧度

        distances = [
            x_distance - y_distance - (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad,  # 左前轮
            x_distance + y_distance + (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad,  # 右前轮
            x_distance + y_distance - (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad,  # 左后轮
            x_distance - y_distance + (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad   # 右后轮
        ]
        return distances
    
    def calculate_wheel_velocity(self, x_velocity, y_velocity, z_angle_velocity):
        """
        计算每个轮子的移动距离
        :param x_distance: X方向的移动距离 (米) (float)
        :param y_distance: Y方向的移动距离 (米) (float)
        :param z_angle_deg: Z方向的转动角度 (度) (float)
        :return: 每个轮子的移动距离 (list of float)
        """
        z_angle_rad = math.radians(z_angle_velocity)  # 将角度从度转换为弧度

        velocity = [
            x_velocity - y_velocity - (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad,  # 左前轮
            x_velocity + y_velocity + (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad,  # 右前轮
            x_velocity + y_velocity - (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad,  # 左后轮
            x_velocity - y_velocity + (self.wheelbase / 2 + self.track_width / 2)  * z_angle_rad   # 右后轮
        ]
        return velocity
  
    def move_to_position(self, x_distance, y_distance, z_angle, acceleration=10, duration=5.0):
        """
        移动到指定位置
        :param x_distance: X方向的移动距离 (米) (float)
        :param y_distance: Y方向的移动距离 (米) (float)
        :param z_angle: Z方向的转动角度 (度) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param duration: 运行时间 (秒) (float)
        """
        # 计算每个轮子的移动距离
        distances = self.calculate_wheel_distances(x_distance, y_distance, z_angle)
        # 计算每个轮子的速度
        velocities = [distance / duration for distance in distances]
        # 设置每个轮子的移动距离和速度
        for wheel, distance, velocity in zip(self.wheels, distances, velocities):
            wheel.set_distance(distance, velocity, acceleration, raF=False, snF=True)  
        # 启动多机同步运动
        # self.motor_controller.Emm_V5_Synchronous_motion()

    def move_in_velocity(self, x_velocity, y_velocity, z_velocity, acceleration=10):
        """
        以一定速度移动
        :param x_velocity: X方向的移动速度 (米/s) (float)
        :param y_velocity: Y方向的移动速度 (米/s) (float)
        :param z_velocity: Z方向的转动角速度 (度/s) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param duration: 运行时间 (秒) (float)
        """
        # 计算每个轮子的速度
        wheel_velocities = self.calculate_wheel_velocity(x_velocity, y_velocity, z_velocity)

        # 设置每个轮子的速度
        for wheel, velocity in zip(self.wheels, wheel_velocities):
            wheel.set_velocity(velocity, acceleration=acceleration, snF=True)
        
    def stop(self):
        """
        停止所有轮子
        """
        for wheel in self.wheels:
            wheel.stop(snF=True)
        # # 启动多机同步运动以确保所有轮子同时停止
        # self.motor_controller.Emm_V5_Synchronous_motion()
    
    def start_to_motion(self):
        self.motor_controller.Emm_V5_Synchronous_motion()


class PanTiltMotor:
    def __init__(self, motor_controller, addr, resolution=3200, cw_positive=True):
        """
        初始化云台电机对象
        :param motor_controller: ZDTMotorController 实例
        :param addr: 电机地址 (int)
        :param resolution: 每圈的脉冲数 (默认为16细分，即3200脉冲/圈) (int)
        :param cw_positive: 是否顺时针方向为正 (True: CW 为正, False: CCW 为正) (bool)
        """
        self.motor_controller = motor_controller
        self.addr = addr
        self.resolution = resolution  # 每圈的脉冲数
        self.cw_positive = cw_positive  # 顺时针方向是否为正

    def calculate_pulses_from_angle(self, angle):
        """
        根据角度计算脉冲数
        :param angle: 角度 (度) (float)
        :return: 脉冲数 (int)
        """
        pulses_per_revolution = self.resolution  # 每圈的脉冲数
        pulses = (angle / 360.0) * pulses_per_revolution  # 脉冲数
        return int(pulses)

    def set_angle(self, angle, velocity, acceleration=10, raF=False, snF=False):
        """
        设置电机的目标角度
        :param angle: 目标角度 (度) (float)
        :param velocity: 运动速度 (度/秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param raF: 相对/绝对运动标志 (True: 相对运动, False: 绝对运动) (bool)
        :param snF: 是否启用多机同步标志 (bool)
        """
        rpm = (velocity * 60) / 360  # 将速度从度/秒转换为RPM
        pulses = self.calculate_pulses_from_angle(angle)
        direction = 0 if (angle >= 0 and self.cw_positive) or (angle < 0 and not self.cw_positive) else 1
        self.motor_controller.Emm_V5_Pos_Control(self.addr, direction, int(abs(rpm)), acceleration, int(abs(pulses)), raF, snF)
    
    def set_velocity(self, velocity, acceleration=10, snF=False):
        """
        设置电机的速度
        :param velocity: 目标速度 (度/秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        :param snF: 是否启用多机同步标志 (bool)
        """
        rpm = (velocity * 60) / 360  # 将速度从度/秒转换为RPM
        direction = 0 if (velocity >= 0 and self.cw_positive) or (velocity < 0 and not self.cw_positive) else 1
        self.motor_controller.Emm_V5_Vel_Control(self.addr, direction, int(abs(rpm)), acceleration, snF)



    def stop(self, snF=False):
        """
        停止电机
        :param snF: 是否启用多机同步标志 (bool)
        """
        self.motor_controller.Emm_V5_Stop_Now(self.addr, snF)


class PanTilt:
    def __init__(self, ZDTMotorController, PanTiltMotor):
        """
        初始化二维云台对象
        :param ZDTMotorController: 电机控制器类
        :param PanTiltMotor: 云台电机类
        """
        self.motor_controller = ZDTMotorController('/dev/ttyS1', 115200)
        self.pan_motor = PanTiltMotor(self.motor_controller, addr=5, resolution=3200, cw_positive=True)  # 水平旋转电机
        self.tilt_motor = PanTiltMotor(self.motor_controller, addr=6, resolution=3200, cw_positive=True)  # 垂直旋转电机

    def move_to_position(self, pan_angle, tilt_angle, duration, acceleration=10):
        """
        移动到指定位置
        :param pan_angle: 水平旋转角度 (度) (float)
        :param tilt_angle: 垂直旋转角度 (度) (float)
        :param duration: 运动时间 (秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        """
        # 计算每个电机的速度
        pan_velocity = pan_angle / duration
        tilt_velocity = tilt_angle / duration

        # 设置水平旋转电机的角度和速度
        self.pan_motor.set_angle(pan_angle, pan_velocity, acceleration, raF=False, snF=True)

        # 设置垂直旋转电机的角度和速度
        self.tilt_motor.set_angle(tilt_angle, tilt_velocity, acceleration, raF=False, snF=True)

    def move_in_velocity(self, pan_velocity, tilt_velocity, acceleration=10):
        """
        以指定速度移动
        :param pan_velocity: 水平旋转速度 (度/秒) (float)
        :param tilt_velocity: 垂直旋转速度 (度/秒) (float)
        :param acceleration: 加速度 (RPM/s) (int)
        """
        # 设置水平旋转电机的速度
        self.pan_motor.set_velocity(pan_velocity, acceleration=acceleration, snF=True)
        # 设置垂直旋转电机的速度
        self.tilt_motor.set_velocity(tilt_velocity, acceleration=acceleration, snF=True)

    def stop(self):
        """
        停止所有电机
        """
        self.pan_motor.stop(snF=True)
        self.tilt_motor.stop(snF=True)
        # # 启动多机同步运动以确保所有电机同时停止
        # self.motor_controller.Emm_V5_Synchronous_motion()

    def start_to_motion(self):
        self.motor_controller.Emm_V5_Synchronous_motion()




class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # 初始化底盘控制类
        self.chassis = Chassis(ZDTMotorController, Wheel, wheelbase=0.135, track_width=0.19)
        # 初始化云台控制类
        self.pan_tilt = PanTilt(ZDTMotorController, PanTiltMotor)

        # 订阅底盘的 Twist 消息
        self.chassis_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.chassis_twist_callback, 10)  # 添加QoS参数

        # 订阅云台的 Twist 消息
        self.pan_tilt_subscription = self.create_subscription(
            Twist, 'pan_tilt_cmd', self.pan_tilt_twist_callback, 10)  # 添加QoS参数

    def chassis_twist_callback(self, msg):
        """
        处理底盘的 Twist 消息，更新底盘的速度指令
        :param msg: Twist 消息
        """
        # 更新底盘的速度指令
        self.control_chassis(msg.linear.x, msg.linear.y, msg.angular.z)

    def pan_tilt_twist_callback(self, msg):
        """
        处理云台的 Twist 消息，更新云台的速度指令
        :param msg: Twist 消息
        """
        # 更新云台的速度指令
        self.control_pan_tilt(msg.angular.z, msg.angular.x)

    def control_chassis(self, x_velocity, y_velocity, z_velocity):
        """
        根据当前的速度指令控制底盘运动
        """
        # 控制底盘运动
        self.chassis.move_in_velocity(
            x_velocity,  # 前进/后退速度
            y_velocity,  # 左移/右移速度
            z_velocity   # 旋转速度
        )
        # 启动多机同步运动
        self.chassis.start_to_motion()

    def control_pan_tilt(self, pan_velocity, tilt_velocity):
        """
        根据当前的速度指令控制云台运动
        """
        # 控制云台运动
        self.pan_tilt.move_in_velocity(
            pan_velocity,  # Pan 速度（水平旋转）
            tilt_velocity  # Tilt 速度（垂直旋转）
        )
        # 启动多机同步运动
        self.pan_tilt.start_to_motion()

    def stop_all(self):
        """
        停止所有运动
        """
        # 停止底盘
        self.chassis.stop()
        # 停止云台
        self.pan_tilt.stop()


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        control_node.get_logger().info("Stopping all movements...")
        control_node.stop_all()
    finally:
        control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()





# # 测试代码
# if __name__ == '__main__':
#     # 创建底盘对象
#     chassis = Chassis(ZDTMotorController, Wheel, wheelbase=0.135, track_width=0.19)

#     # 移动到指定位置    
#     chassis.move_to_position(x_distance=0.5, y_distance=0, z_angle=0, duration=2.0)
#     chassis.start_to_motion()   

    # chassis.move_in_velocity(x_velocity=0.3,y_velocity=0,z_velocity=0,acceleration=10)
    # chassis.start_to_motion()
    # time.sleep(5)
    # chassis.move_in_velocity(x_velocity=0,y_velocity=0,z_velocity=0,acceleration=10)
    # chassis.start_to_motion()


# if __name__ == '__main__':
#     # 创建二维云台对象
#     pan_tilt = PanTilt(ZDTMotorController, PanTiltMotor)

#     # 移动到指定位置
#     # pan_tilt.move_to_position(pan_angle=45.0, tilt_angle=20.0, duration=1.0, acceleration=10)
#     # pan_tilt.start_to_motion()

#     pan_tilt.move_in_velocity(pan_velocity=15,tilt_velocity=15,acceleration=10)
#     pan_tilt.start_to_motion()
#     time.sleep(3)
#     pan_tilt.move_in_velocity(pan_velocity=0,tilt_velocity=0,acceleration=10)
#     pan_tilt.start_to_motion()


