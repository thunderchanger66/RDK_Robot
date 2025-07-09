#!/usr/bin/env python3
import serial
import time
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion


# 设置正确的串口参数------------------------
ser_port = '/dev/ttyS3'   # 此处需要替换为对应使用的串口号，windows系统写成COMx，若是linux则要根据所用系统进行调整如写成/dev/ttyUSBx或/dev/ttySx
ser_baudrate = 115200       # 串口波特率
ser_timeout = 5             # 串口操作超时时间
# Open the serial port
ser = serial.Serial(ser_port, ser_baudrate, timeout=ser_timeout)

CmdPacket_Begin = 0x49      # 起始码
CmdPacket_End = 0x4D        # 结束码
CmdPacketMaxDatSizeRx = 73  # 模块发来的数据包的数据体最大长度
CS = 0      # 校验和
i = 0
RxIndex = 0
buf = bytearray(5 + CmdPacketMaxDatSizeRx)  # 接收包缓存
cmdLen = 0  # 长度

# z轴清零指令
# 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 FF 00 FF 49 FF 01 05 05 4D


class JackIM948(Node):
    
    def __init__(self):
        super().__init__("JackIM948")
        self.get_logger().info("Service im948 are Ready")
        self.publisher_ = self.create_publisher(Imu, 'imu/im948_raw', 10)
        self.read_data()
        
    def pub(self, imu_data):
        # 创建一个IMU消息
        imu_msg = Imu()

        # 设置消息中的时间戳
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'gyro_link'
        
        imu_msg.orientation = Quaternion(x=imu_data['x'], y=imu_data['y'], z=imu_data['z'], w=imu_data['w'])  # 单位四元数
        imu_msg.angular_velocity.x = imu_data['angular_velocity_x']  
        imu_msg.angular_velocity.y = imu_data['angular_velocity_y']
        imu_msg.angular_velocity.z = imu_data['angular_velocity_z']
        imu_msg.linear_acceleration.x = imu_data['linear_acceleration_x']  
        imu_msg.linear_acceleration.y = imu_data['linear_acceleration_y']
        imu_msg.linear_acceleration.z = imu_data['linear_acceleration_z']
        self.publisher_.publish(imu_msg)
        # self.get_logger().info(f'Publishing: {imu_msg.header.stamp}')
        
    def log(self, *args):
        self.get_logger().info(" ".join(str(a) for a in args))
        
    def Cmd_RxUnpack(self, buf, DLen):        
        scaleAccel       = 0.00478515625
        scaleQuat        = 0.000030517578125
        scaleAngle       = 0.0054931640625
        scaleAngleSpeed  = 0.06103515625
        scaleMag         = 0.15106201171875
        scaleTemperature = 0.01
        scaleAirPressure = 0.0002384185791
        scaleHeight      = 0.0010728836

        # self.log("rev data:",buf)
        if buf[0] == 0x11:
            ctl = (buf[2] << 8) | buf[1]
            self.log(" subscribe tag: 0x%04x"%ctl)
            self.log(" ms: ", ((buf[6]<<24) | (buf[5]<<16) | (buf[4]<<8) | (buf[3]<<0)))
            
            imu_data = {}

            L =7 # 从第7字节开始根据 订阅标识tag来解析剩下的数据
            if ((ctl & 0x0001) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
                self.log("\taX: %.3f"%tmpX); # x加速度aX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2 
                self.log("\taY: %.3f"%tmpY); # y加速度aY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\taZ: %.3f"%tmpZ); # z加速度aZ        
            if ((ctl & 0x0002) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\tAX: %.3f"%tmpX) # x加速度AX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\tAY: %.3f"%tmpY) # y加速度AY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\tAZ: %.3f"%tmpZ) # z加速度AZ

            if ((ctl & 0x0004) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                self.log("\tGX: %.3f"%tmpX) # x角速度GX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2 
                self.log("\tGY: %.3f"%tmpY) # y角速度GY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngleSpeed; L += 2
                self.log("\tGZ: %.3f"%tmpZ) # z角速度GZ
                
                imu_data['angular_velocity_x'] = tmpX
                imu_data['angular_velocity_y'] = tmpY
                imu_data['angular_velocity_z'] = tmpZ
            
            if ((ctl & 0x0008) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
                self.log("\tCX: %.3f"%tmpX); # x磁场CX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
                self.log("\tCY: %.3f"%tmpY); # y磁场CY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleMag; L += 2
                self.log("\tCZ: %.3f"%tmpZ); # z磁场CZ
            
            if ((ctl & 0x0010) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleTemperature; L += 2
                self.log("\ttemperature: %.2f"%tmpX) # 温度

                tmpU32 = np.uint32(((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L])))
                if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)      
                tmpY = np.int32(tmpU32) * scaleAirPressure; L += 3
                self.log("\tairPressure: %.3f"%tmpY); # 气压

                tmpU32 = np.uint32((np.uint32(buf[L+2]) << 16) | (np.uint32(buf[L+1]) << 8) | np.uint32(buf[L]))
                if ((tmpU32 & 0x800000) == 0x800000): # 若24位数的最高位为1则该数值为负数，需转为32位负数，直接补上ff即可
                    tmpU32 = (tmpU32 | 0xff000000)
                tmpZ = np.int32(tmpU32) * scaleHeight; L += 3 
                self.log("\theight: %.3f"%tmpZ); # 高度

            if ((ctl & 0x0020) != 0):
                tmpAbs = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                self.log("\tw: %.3f"%tmpAbs); # w
                tmpX =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                self.log("\tx: %.3f"%tmpX); # x
                tmpY =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                self.log("\ty: %.3f"%tmpY); # y
                tmpZ =   np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleQuat; L += 2
                self.log("\tz: %.3f"%tmpZ); # z
                imu_data['w'] = tmpAbs
                imu_data['x'] = tmpX
                imu_data['y'] = tmpY
                imu_data['z'] = tmpZ

            if ((ctl & 0x0040) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                self.log("\tangleX: %.3f"%tmpX); # x角度
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                self.log("\tangleY: %.3f"%tmpY); # y角度
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAngle; L += 2
                self.log("\tangleZ: %.3f"%tmpZ); # z角度

            if ((ctl & 0x0080) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
                self.log("\toffsetX: %.3f"%tmpX); # x坐标
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
                self.log("\toffsetY: %.3f"%tmpY); # y坐标
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) / 1000.0; L += 2
                self.log("\toffsetZ: %.3f"%tmpZ); # z坐标

            if ((ctl & 0x0100) != 0):
                tmpU32 = ((buf[L+3]<<24) | (buf[L+2]<<16) | (buf[L+1]<<8) | (buf[L]<<0)); L += 4
                self.log("\tsteps: %u"%tmpU32); # 计步数
                tmpU8 = buf[L]; L += 1
                if (tmpU8 & 0x01):# 是否在走路
                    self.log("\t walking yes")
                else:
                    self.log("\t walking no")
                if (tmpU8 & 0x02):# 是否在跑步
                    self.log("\t running yes")
                else:
                    self.log("\t running no")
                if (tmpU8 & 0x04):# 是否在骑车
                    self.log("\t biking yes")
                else:
                    self.log("\t biking no")
                if (tmpU8 & 0x08):# 是否在开车
                    self.log("\t driving yes")
                else:
                    self.log("\t driving no")

            if ((ctl & 0x0200) != 0):
                tmpX = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\tasX: %.3f"%tmpX); # x加速度asX
                tmpY = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\tasY: %.3f"%tmpY); # y加速度asY
                tmpZ = np.short((np.short(buf[L+1])<<8) | buf[L]) * scaleAccel; L += 2
                self.log("\tasZ: %.3f"%tmpZ); # z加速度asZ
                
                imu_data['linear_acceleration_x'] = tmpX
                imu_data['linear_acceleration_y'] = tmpY
                imu_data['linear_acceleration_z'] = tmpZ
                        
            if ((ctl & 0x0400) != 0):
                tmpU16 = ((buf[L+1]<<8) | (buf[L]<<0)); L += 2
                self.log("\tadc: %u"%tmpU16); # adc测量到的电压值，单位为mv

            if ((ctl & 0x0800) != 0):
                tmpU8 = buf[L]; L += 1
                self.log("\tGPIO1  M:%X, N:%X"%((tmpU8>>4)&0x0f, (tmpU8)&0x0f))
            self.pub(imu_data)
            self.get_logger().info("IMU data published.")
        else:
            self.log("------data head not define")
        self.log(""); 

    def Cmd_GetPkt(self, byte):
        global CS, i, RxIndex, buf, cmdLen
        CS += byte # 边收数据边计算校验码，校验码为地址码开始(包含地址码)到校验码之前的数据的和
        if RxIndex == 0: # 起始码
            if byte == CmdPacket_Begin:
                self.get_logger().info("Packet begin detected.")
                i = 0
                buf[i] = CmdPacket_Begin
                i += 1
                CS = 0 # 下个字节开始计算校验码
                RxIndex = 1
        elif RxIndex == 1: # 数据体的地址码
            buf[i] = byte
            i += 1
            if byte == 255: # 255是广播地址，模块作为从机，它的地址不可会出现255
                RxIndex = 0
            else:
                RxIndex += 1
        elif RxIndex == 2: # 数据体的长度
            buf[i] = byte
            i += 1
            if byte > CmdPacketMaxDatSizeRx or byte == 0:  # 长度无效
                RxIndex = 0
            else:
                RxIndex += 1
                cmdLen = byte
        elif RxIndex == 3: # 获取数据体的数据
            buf[i] = byte
            i += 1
            if i >= cmdLen + 3: # 已收完数据体
                RxIndex += 1
        elif RxIndex == 4: # 对比 效验码
            CS -= byte
            if (CS&0xFF) == byte: # 校验正确
                buf[i] = byte
                i += 1
                RxIndex += 1
            else: # 校验失败
                RxIndex = 0
        elif RxIndex == 5: # 结束码
            RxIndex = 0
            if byte == CmdPacket_End: # 捕获到完整包
                self.get_logger().info("Packet end detected, full packet received.")
                buf[i] = byte
                i += 1
                hex_string = " ".join(f"{b:02X}" for b in buf[0:i])
                self.log(f"U-Rx[Len={i}]:{hex_string}")
                self.Cmd_RxUnpack(buf[3:i-2], i-5) # 处理数据包的数据体
                return 1
        else:
            RxIndex = 0
            self.get_logger().info("fail.")
        return 0

    def Cmd_PackAndTx(self, pDat, DLen):
        if DLen == 0 or DLen > 19:
            return -1  # 非法参数

        # 构建发送包缓存，包括50字节的前导码
        buf = bytearray([0x00]*46) + bytearray([0x00, 0xff, 0x00, 0xff,  0x49, 0xFF, DLen]) + bytearray(pDat[:DLen])

        # 计算校验和，从地址码开始到数据体结束
        CS = sum(buf[51:51+DLen+2]) & 0xFF  # 取低8位
        buf.append(CS)
        buf.append(0x4D)  # 添加结束码

        # 发送数据
        ser.write(buf)
        return 0

    def read_data(self):
        self.get_logger().info("Start configuring IMU sensor...")
        # 1.发送配置参数
        params = [0] * 11                   # 数组
        isCompassOn = 0                     # 1=开启磁场融合姿态 0=关闭磁场融合姿态
        barometerFilter = 2                 # 气压计的滤波等级[取值0-3]
        Cmd_ReportTag = 0x0FFF              # 功能订阅标识
        params[0] = 0x12            
        params[1] = 5                       # 静止状态加速度阀值
        params[2] = 255                     # 静止归零速度(单位cm/s) 0:不归零 255:立即归零
        params[3] = 0                       # 动态归零速度(单位cm/s) 0:不归零
        params[4] = ((barometerFilter & 3) << 1) | (isCompassOn & 1)
        params[5] = 30                      # 数据主动上报的传输帧率[取值0-250HZ], 0表示0.5HZ
        params[6] = 1                       # 陀螺仪滤波系数[取值0-2],数值越大越平稳但实时性越差
        params[7] = 3                       # 加速计滤波系数[取值0-4],数值越大越平稳但实时性越差
        params[8] = 5                       # 磁力计滤波系数[取值0-9],数值越大越平稳但实时性越差
        params[9] = Cmd_ReportTag & 0xff
        params[10] = (Cmd_ReportTag >> 8) & 0xff

        self.Cmd_PackAndTx(params, len(params))  # 发送指令给传感器
        time.sleep(0.2)
        self.get_logger().info("IMU sensor configured, start waking up...")
        # 2.唤醒传感器
        self.Cmd_PackAndTx([0x03], 1)
        time.sleep(0.2)
        self.get_logger().info("IMU sensor wakeup command sent.")
        # 3.开启主动上报
        self.Cmd_PackAndTx([0x19], 1)
        
        # time.sleep(0.2)
        # self.Cmd_PackAndTx([0x51,0xAA,0xBB], 3) # 用总圈数代替欧拉角传输 并清零圈数 0x51
        # self.Cmd_PackAndTx([0x51,0x00,0x00], 3) # 输出欧拉角 0x51

        # 循环接收数据并处理
        self.get_logger().info("IMU sensor reporting enabled. Start reading data...")
        while True:
            data = ser.read(1)  # read 1 bytes
            if len(data) > 0:   # if data is not empty
                self.Cmd_GetPkt(data[0])


def main():
    rclpy.init()
    service_server = JackIM948()
    rclpy.spin(service_server)    
    service_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
