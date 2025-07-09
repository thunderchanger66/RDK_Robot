from ast import Try
from time import sleep
import numpy as np
from numpy import array
import gatt
import time

from argparse import ArgumentParser
from array import array
import socket 
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import gatt
import time
import numpy as np
from array import array
import signal


class AnyDevice(gatt.Device):

    def __init__(self, manager, mac_address, service):
        super().__init__(manager=manager, mac_address=mac_address)
        self.sock_pc = None
        self.parse_imu_flage = True
        self.service = service  # 保存对 IMUService 的引用


    def connect_succeeded(self):
        super().connect_succeeded()
        print("[%s] Connected" % (self.mac_address))

    def connect_failed(self, error):
        super().connect_failed(error)
        print("[%s] Connection failed: %s" % (self.mac_address, str(error)))

    def disconnect_succeeded(self):
        super().disconnect_succeeded()
        print("[%s] Disconnected" % (self.mac_address))

    def services_resolved(self):
        super().services_resolved()

        print("[%s] Resolved services" % (self.mac_address))
        for service in self.services:
            print("[%s]\tService [%s]" % (self.mac_address, service.uuid))
            for characteristic in service.characteristics:
                print("[%s]\t\tCharacteristic [%s]" % (self.mac_address, characteristic.uuid))
               
        # 保持连接
        lzchar1 = next(
            c for c in service.characteristics
            if c.uuid == '0000ae01-0000-1000-8000-00805f9b34fb'.lower())
        lzchar1.write_value(')'.encode()) # 发送十六进制的0x29，让设备保持连接
        
        # 尝试采用蓝牙高速通信特性 0x46
        lzchar1.write_value(bytes([0x46]))

        # GPIO 上拉
        #lzchar1.write_value(bytes([0x27,0x10]))

        # 参数设置
        isCompassOn = 0        #1=使用磁场融合姿态，0=不使用
        barometerFilter = 2
        Cmd_ReportTag = 0x0FFF # 功能订阅标识
        params = bytearray([0x00 for i in range(0,11)])
        params[0] = 0x12
        params[1] = 5       #静止状态加速度阀值
        params[2] = 255     #静止归零速度(单位cm/s) 0:不归零 255:立即归零
        params[3] = 0       #动态归零速度(单位cm/s) 0:不归零
        params[4] = ((barometerFilter&3)<<1) | (isCompassOn&1);   
        params[5] = 60      #数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        params[6] = 1       #陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        params[7] = 3       #加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        params[8] = 5       #磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        params[9] = Cmd_ReportTag&0xff
        params[10] = (Cmd_ReportTag>>8)&0xff
        lzchar1.write_value(params)

        # 主动上报 0x19 
        lzchar1.write_value(bytes([0x19]))
        
        #lzchar1.write_value(bytes([0x51,0xAA,0xBB])) # 用总圈数代替欧拉角传输 并清零圈数 0x51
        #lzchar1.write_value(bytes([0x51,0x00,0x00])) # 输出欧拉角 0x51

        lzchar2 = next(
            c for c in service.characteristics
            if c.uuid == '0000ae02-0000-1000-8000-00805f9b34fb'.lower())
        lzchar2.enable_notifications()

    def descriptor_read_value_failed(self, descriptor, error):
        print('descriptor_value_failed')

    def characteristic_write_value_succeeded(self, characteristic):
        super().characteristic_write_value_succeeded(characteristic)
        print("[%s] wr ok" % (self.mac_address))

    def characteristic_write_value_failed(self, characteristic, error):
        super().characteristic_write_value_failed(characteristic, error)
        print("[%s] wr err %s" % (self.mac_address, error))
    
    def characteristic_enable_notifications_succeeded(self, characteristic):
        super().characteristic_enable_notifications_succeeded(characteristic)
        print("[%s] notify ok" % (self.mac_address))

    def characteristic_enable_notifications_failed(self, characteristic, error):
        super().characteristic_enable_notifications_failed(characteristic, error)
        print("[%s] notify err. %s" % (self.mac_address, error))

    def characteristic_value_updated(self, characteristic, value):
        # print("Lzchar:", value.hex()) 
        # print("value.size",len(value))
        #self.parse_imu(value)
        
        if characteristic.uuid == '0000ae02-0000-1000-8000-00805f9b34fb'.lower():
            if self.parse_imu_flage:
                self.parse_imu(value)

            if self.sock_pc is not None:
                print("send blue source data")
                self.sock_pc.sendall(value)
                
    #这个是在本地解析
    def parse_imu(self,buf):
        scaleAccel       = 0.00478515625      # 加速度 [-16g~+16g]    9.8*16/32768
        scaleQuat        = 0.000030517578125  # 四元数 [-1~+1]         1/32768
        scaleAngle       = 0.0054931640625    # 角度   [-180~+180]     180/32768
        scaleAngleSpeed  = 0.06103515625      # 角速度 [-2000~+2000]    2000/32768
        scaleMag         = 0.15106201171875   # 磁场 [-4950~+4950]   4950/32768
        scaleTemperature = 0.01               # 温度
        scaleAirPressure = 0.0002384185791    # 气压 [-2000~+2000]    2000/8388608
        scaleHeight      = 0.0010728836       # 高度 [-9000~+9000]    9000/8388608

        imu_dat = array('f',[0.0 for i in range(0,34)])

        if buf[0] == 0x11:
            ctl = (buf[2] << 8) | buf[1]
            # print(" subscribe tag: 0x%04x"%ctl)
            # print(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))

            L =7 # 从第7字节开始根据 订阅标识tag来解析剩下的数据
            if ((ctl & 0x0001) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
                # print("\taX: %.3f"%tmpX); # x加速度aX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
                # print("\taY: %.3f"%tmpY); # y加速度aY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\taZ: %.3f"%tmpZ); # z加速度aZ

                imu_dat[0] = float(tmpX)
                imu_dat[1] = float(tmpY)
                imu_dat[2] = float(tmpZ)
            
            # print(" ")
            if ((ctl & 0x0002) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tAX: %.3f"%tmpX) # x加速度AX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tAY: %.3f"%tmpY) # y加速度AY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tAZ: %.3f"%tmpZ) # z加速度AZ

                imu_dat[3] = float(tmpX)
                imu_dat[4] = float(tmpY)
                imu_dat[5] = float(tmpZ)

            # print(" ")
            if ((ctl & 0x0004) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                # print("\tGX: %.3f"%tmpX) # x角速度GX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                # print("\tGY: %.3f"%tmpY) # y角速度GY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
                # print("\tGZ: %.3f"%tmpZ) # z角速度GZ

                imu_dat[6] = float(tmpX)
                imu_dat[7] = float(tmpY)
                imu_dat[8] = float(tmpZ)
            
            # print(" ")
            if ((ctl & 0x0008) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
                # print("\tCX: %.3f"%tmpX); # x磁场CX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
                # print("\tCY: %.3f"%tmpY); # y磁场CY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
                # print("\tCZ: %.3f"%tmpZ); # z磁场CZ

                imu_dat[9] = float(tmpX)
                imu_dat[10] = float(tmpY)
                imu_dat[11] = float(tmpZ)
            
            # print(" ")
            if ((ctl & 0x0010) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
                # print("\ttemperature: %.2f"%tmpX) # 温度

                tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
                if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)      
                tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
                # print("\tairPressure: %.3f"%tmpY); # 气压

                tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
                if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)
                tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
                # print("\theight: %.3f"%tmpZ); # 高度

                imu_dat[12] = float(tmpX)
                imu_dat[13] = float(tmpY)
                imu_dat[14] = float(tmpZ)

            # print(" ")
            if ((ctl & 0x0020) != 0):
                tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                # print("\tw: %.3f"%tmpAbs); # w
                tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                # print("\tx: %.3f"%tmpX); # x
                tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                # print("\ty: %.3f"%tmpY); # y
                tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                # print("\tz: %.3f"%tmpZ); # z

                imu_dat[15] = float(tmpAbs)
                imu_dat[16] = float(tmpX)
                imu_dat[17] = float(tmpY)
                imu_dat[18] = float(tmpZ)

            # print(" ")
            if ((ctl & 0x0040) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleX: %.3f"%tmpX); # x角度
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleY: %.3f"%tmpY); # y角度
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                # print("\tangleZ: %.3f"%tmpZ); # z角度

                imu_dat[19] = float(tmpX)
                imu_dat[20] = float(tmpY)
                imu_dat[21] = float(tmpZ)

            # print(" ")
            if ((ctl & 0x0080) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
                # print("\toffsetX: %.3f"%tmpX); # x坐标
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
                # print("\toffsetY: %.3f"%tmpY); # y坐标
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
                # print("\toffsetZ: %.3f"%tmpZ); # z坐标

                imu_dat[22] = float(tmpX)
                imu_dat[23] = float(tmpY)
                imu_dat[24] = float(tmpZ)

            # print(" ")
            if ((ctl & 0x0100) != 0):
                tmpU32 = ((buf[L+3]<<24) | (buf[L+2]<<16) | (buf[L+1]<<8) | (buf[L]<<0)); L += 4
                # print("\tsteps: %u"%tmpU32); # 计步数
                tmpU8 = buf[L]; L += 1
                if (tmpU8 & 0x01):# 是否在走路
                    # print("\t walking yes")
                    imu_dat[25] = 100
                else:
                    # print("\t walking no")
                    imu_dat[25] = 0
                if (tmpU8 & 0x02):# 是否在跑步
                    # print("\t running yes")
                    imu_dat[26] = 100
                else:
                    # print("\t running no")
                    imu_dat[26] = 0
                if (tmpU8 & 0x04):# 是否在骑车
                    # print("\t biking yes")
                    imu_dat[27] = 100
                else:
                    # print("\t biking no")
                    imu_dat[27] = 0
                if (tmpU8 & 0x08):# 是否在开车
                    # print("\t driving yes")
                    imu_dat[28] = 100
                else:
                    # print("\t driving no")
                    imu_dat[28] = 0

            # print(" ")
            if ((ctl & 0x0200) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tasX: %.3f"%tmpX); # x加速度asX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tasY: %.3f"%tmpY); # y加速度asY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                # print("\tasZ: %.3f"%tmpZ); # z加速度asZ
            
                imu_dat[29] = float(tmpX)
                imu_dat[30] = float(tmpY)
                imu_dat[31] = float(tmpZ)
                
            # print(" ")
            if ((ctl & 0x0400) != 0):
                tmpU16 = ((buf[L+1]<<8) | (buf[L]<<0)); L += 2
                # print("\tadc: %u"%tmpU16); # adc测量到的电压值，单位为mv
                imu_dat[32] = float(tmpU16)

            # print(" ")
            if ((ctl & 0x0800) != 0):
                tmpU8 = buf[L]; L += 1
                # print("\t GPIO1  M:%X, N:%X"%((tmpU8>>4)&0x0f, (tmpU8)&0x0f))
                imu_dat[33] = float(tmpU8)

            self.service.pub(imu_dat)
            # self.service.get_logger().info(f'imu_dat:{imu_dat}')
            

            
        else:
            print("[error] data head not define")


class IMUService(Node):
    
    def __init__(self):
        super().__init__("IMUService")
        self.get_logger().info("Service im948 are Ready")
        self.publisher_ = self.create_publisher(Imu, 'imu/im948_raw', 10)
        time.sleep(1)  # 等待1秒，确保发布者已经初始化完成

        self.mac_address = "88:96:56:21:5F:A2"
        print("Connecting bluetooth ...")
        self.manager = gatt.DeviceManager(adapter_name='hci0')
        self.device = AnyDevice(manager=self.manager, mac_address=self.mac_address,service=self)
        self.device.connect()
        self.manager.run()

    def pub(self, imu_dat):
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'gyro_link'
        imu_msg.orientation = Quaternion(x=imu_dat[16], y=imu_dat[17], z=imu_dat[18], w=imu_dat[15])
        imu_msg.angular_velocity.x = imu_dat[6]
        imu_msg.angular_velocity.y = imu_dat[7]
        imu_msg.angular_velocity.z = imu_dat[8]
        imu_msg.linear_acceleration.x = imu_dat[0]
        imu_msg.linear_acceleration.y = imu_dat[1]
        imu_msg.linear_acceleration.z = imu_dat[2]
        self.publisher_.publish(imu_msg)
 


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


def main():
    rclpy.init()
    service_server = IMUService()
    rclpy.spin(service_server)

    service_server.get_logger().info("Caught Keyboard Interrupt. Shutting down.")
    service_server.device.disconnect()  # 断开蓝牙连接
    service_server.manager.stop()           # 停止蓝牙管理器
    time.sleep(1)
    service_server.destroy_node()          # 销毁节点
    rclpy.shutdown()                       # 关闭 ROS 2

            