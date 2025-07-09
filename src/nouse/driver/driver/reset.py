#!/usr/bin/env python3

import serial
import time

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
        time.sleep(0.01)  # 避免连续发送冲突
        self.ser.write(cmd)
        return True

    def Emm_V5_Reset_Clog_Pro(self, addr):
        """
        解除堵转保护
        :param addr: 电机地址 (int)
        """
        cmd = bytearray([addr, 0x0E, 0x52, 0x6B])  # 构建命令
        return self.send_command(cmd)  # 发送命令并返回结果


def main():
    # 定义串口设备路径
    uart_dev = '/dev/ttyS1'
    baudrate = 115200

    # 创建电机控制器实例
    motor_controller = ZDTMotorController(uart_dev, baudrate)

    # 检查是否成功连接
    if motor_controller.ser is None:
        print("Failed to connect to the motor controller.")
        return

    # 解除1到6号电机的堵转保护
    for addr in range(1, 7):
        print(f"Resetting clog protection for motor address {addr}...")
        if motor_controller.Emm_V5_Reset_Clog_Pro(addr):
            print(f"Successfully reset clog protection for motor address {addr}.")
        else:
            print(f"Failed to reset clog protection for motor address {addr}.")

    # 断开连接
    motor_controller.disconnect()


if __name__ == '__main__':
    main()