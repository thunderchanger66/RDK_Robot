#!/usr/bin/env python3

################################################################################
# Copyright (c) 2024, D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from i2cdev import I2C
from std_srvs.srv import Trigger
import time

# 幻尔科技语音模块参数
ASR_I2C_ADDR = 0x34        # 语音模块的I2C设备地址
ASR_RESULT_REG = 0x64      # 语音识别结果寄存器地址
ASR_SPEAK_REG = 0x6E       # 语音播报控制寄存器地址

# 语音播报类型定义
ASR_CMDMAND = 0x00         # 命令词条播报模式
ASR_ANNOUNCER = 0xFF       # 普通播报模式

# 命令词ID映射表（包含指令名称和对应的响应）
COMMAND_IDS = {
    # 基础移动指令
    1: {"name": "前进", "response": (ASR_CMDMAND, 1, "正在前进")},
    2: {"name": "后退", "response": (ASR_CMDMAND, 2, "正在后退")},
    3: {"name": "左转", "response": (ASR_CMDMAND, 3, "正在左转")},
    4: {"name": "右转", "response": (ASR_CMDMAND, 4, "正在右转")},
    9: {"name": "停止", "response": (ASR_CMDMAND, 9, "收到")},
    
    # 工位指令（1-9号工位）
    128: {"name": "一号工位", "response": (ASR_CMDMAND, 128, "一号工位你好")},
    129: {"name": "二号工位", "response": (ASR_CMDMAND, 129, "二号工位你好")},
    130: {"name": "三号工位", "response": (ASR_CMDMAND, 130, "三号工位你好")},
    131: {"name": "四号工位", "response": (ASR_CMDMAND, 131, "四号工位你好")},
    132: {"name": "五号工位", "response": (ASR_CMDMAND, 132, "五号工位你好")},
    133: {"name": "六号工位", "response": (ASR_CMDMAND, 133, "六号工位你好")},
    134: {"name": "七号工位", "response": (ASR_CMDMAND, 134, "七号工位你好")},
    135: {"name": "八号工位", "response": (ASR_CMDMAND, 135, "八号工位你好")},
    136: {"name": "九号工位", "response": (ASR_CMDMAND, 136, "九号工位你好")},
    
    # 工具指令
    161: {"name": "剪刀", "response": (ASR_CMDMAND, 161, "剪刀马上送到")},
    162: {"name": "螺丝刀", "response": (ASR_CMDMAND, 162, "螺丝刀马上送到")},
    163: {"name": "胶带", "response": (ASR_CMDMAND, 163, "胶带马上送到")},
    164: {"name": "小刀", "response": (ASR_CMDMAND, 164, "小刀马上送到")},
    165: {"name": "扳手", "response": (ASR_CMDMAND, 165, "扳手马上送到")},
    
    # 系统指令
    171: {"name": "整理工具", "response": (ASR_CMDMAND, 171, "开始整理")},
}

# 预定义播报内容（非命令词的标准响应）
ANNOUNCEMENTS = [
    (ASR_ANNOUNCER, 32, "工具已送达"),  
    (ASR_ANNOUNCER, 33, "工具已整理"),  
    (ASR_ANNOUNCER, 34, "任务已收到开始搬运"),  
    (ASR_ANNOUNCER, 35, "请指定工位"),  
    (ASR_ANNOUNCER, 36, "请指定工具"),  
]

class VoiceTaskPublisher(Node):
    """语音任务发布节点，处理语音指令并发布对应的任务"""
    
    def __init__(self):
        super().__init__('voice_task_publisher')  # 初始化ROS2节点
        
        # 初始化语音识别模块
        self.asr = ASRModule(bus_num=0)  # 使用I2C总线0
        
                # 当前工位状态
        self.current_station = None
        self.current_tool = None
        self.time_flag = 0
        self.time = 0

        # 创建任务发布者，发布到/task_command话题
        self.task_pub = self.create_publisher(
            String, 
            '/task_command', 
            10  # 队列长度
        )
        
        self.srv = self.create_service(
            Trigger,
            'voice_1',
            self.handle_request_1
        )

        self.srv = self.create_service(
            Trigger,
            'voice_2',
            self.handle_request_2
        )

        self.srv = self.create_service(
            Trigger,
            'voice_3',
            self.handle_request_3
        )

        # 命令词到任务类型的映射关系
        self.command_mapping = {
            # 工位指令映射（128-136 -> 1-9号工位）
            **{id: ("搬运", f"{id-127}号工位") for id in range(128, 137)},
            
            # 工具指令映射
            161: ("搬运", "剪刀"),  # 剪刀指令
            162: ("搬运", "螺丝刀"),  
            163: ("搬运", "胶带"),  
            164: ("搬运", "小刀"),  
            165: ("搬运", "扳手"), 


            # 整理指令映射
            170: ("整理", "无")  # 整理工具指令
        }
        
        # 创建定时器，每0.1秒检查一次语音输入
        self.create_timer(0.1, self.process_voice_input)
        

    def handle_request_1(self, request, response):
        """处理语音播报服务请求
        
        参数:
            request: Trigger请求 (空)
            response: Trigger响应
            
        返回:
            带有success和message字段的Trigger响应
        """
        try:
            # 这里我们默认播放"工具已送达"的语音
            self.asr.speak(ASR_ANNOUNCER, 32)  # 32对应"工具已送达"
            
            response.success = True
            response.message = "语音播报已触发"
            self.get_logger().info("成功处理语音播报请求")
        except Exception as e:
            response.success = False
            response.message = f"语音播报失败: {str(e)}"
            self.get_logger().error(f"语音播报错误: {e}")
        
        return response


    def handle_request_2(self, request, response):
        """处理语音播报服务请求
        
        参数:
            request: Trigger请求 (空)
            response: Trigger响应
            
        返回:
            带有success和message字段的Trigger响应
        """
        try:
            # 这里我们默认播放"工具已整理"的语音
            self.asr.speak(ASR_ANNOUNCER, 33)  # 33对应"工具已整理"
            
            response.success = True
            response.message = "语音播报已触发"
            self.get_logger().info("成功处理语音播报请求")
        except Exception as e:
            response.success = False
            response.message = f"语音播报失败: {str(e)}"
            self.get_logger().error(f"语音播报错误: {e}")
        
        return response

    def handle_request_3(self, request, response):
        """处理语音播报服务请求
        
        参数:
            request: Trigger请求 (空)
            response: Trigger响应
            
        返回:
            带有success和message字段的Trigger响应
        """
        try:
            # 这里我们默认播放"工具已送达"的语音
            self.asr.speak(ASR_ANNOUNCER, 34)  # 32对应"工具已送达"
            
            response.success = True
            response.message = "语音播报已触发"
            self.get_logger().info("成功处理语音播报请求")
        except Exception as e:
            response.success = False
            response.message = f"语音播报失败: {str(e)}"
            self.get_logger().error(f"语音播报错误: {e}")
        
        return response


    def process_voice_input(self):
        """处理语音识别结果"""
        result_id = self.asr.read_result()
        
        if self.time_flag:
            self.time += 0.1
            if self.time >= 4:
                self.handle_timeout()

        # 结果为0表示没有新指令
        if result_id == 0:
            # 检查超时
            return

        # 检查是否在命令映射表中
        if result_id in self.command_mapping:
            task_type, target = self.command_mapping[result_id]
            
            if task_type == "整理":
                self.publish_task(task_type, "无", "无")
                return

            elif task_type == "搬运":
                # 工位指令处理
                if 128 <= result_id <= 136:
                    self.current_station = target
                    self.get_logger().info(f"设置工位: {target}")
                    self.time_flag = 1
                    self.time = 0
                
                # 工具指令处理
                elif 161 <= result_id <= 165:
                    self.current_tool = target
                    self.get_logger().info(f"设置工具: {target}")
                    self.time_flag = 1
                    self.time = 0
                
                # 检查是否可以执行任务
                if self.current_station and self.current_tool:
                    time.sleep(2)
                    self.asr.speak(ASR_ANNOUNCER, 34)
                    self.publish_task(task_type, self.current_tool, self.current_station)
                    self.current_station = None
                    self.current_tool = None
                    self.time_flag = 0

    def handle_timeout(self):
        """处理超时情况"""
        if not self.current_station:
            print("请指定工位")
            self.asr.speak(ASR_ANNOUNCER, 35)  # 播放"请指定工位"
        if not self.current_tool:
            print("请指定工具")
            self.asr.speak(ASR_ANNOUNCER, 36)  # 播放"请指定工具"
        self.time = 0
    

    def publish_task(self, task_type, tool, location):
        """发布任务到调度系统
        
        参数:
            task_type: 任务类型（"搬运"或"整理"）
            tool: 工具名称
            location: 目标位置
        """
        # 创建任务消息
        msg = String()
        msg.data = f"{task_type},{tool},{location}"
        
        # 发布任务
        self.task_pub.publish(msg)
        # 记录日志
        self.get_logger().info(f"发布任务: {msg.data}")


class ASRModule:
    """幻尔科技语音模块封装类"""
    
    def __init__(self, bus_num=0):
        """初始化I2C连接"""
        self.i2c = I2C(ASR_I2C_ADDR, bus_num)  # 创建I2C设备对象
        self.send = [0, 0]  # 发送缓冲区
    
    def read_result(self):
        """从语音模块读取识别结果
        
        返回:
            识别到的命令ID，如果没有新命令则返回0
        """
        try:
            # 写入要读取的寄存器地址
            self.i2c.write(bytes([ASR_RESULT_REG]))
            # 读取1字节结果
            result = self.i2c.read(1)
            return result[0] if result else 0
        except Exception as e:
            print(f"读取错误: {e}")
            return 0
    
    def speak(self, cmd_type, id):
        """控制语音模块播报内容
        
        参数:
            cmd_type: 播报类型（ASR_CMDMAND或ASR_ANNOUNCER）
            id: 要播报的内容ID
        """
        if cmd_type in (ASR_CMDMAND, ASR_ANNOUNCER):
            self.send = [cmd_type, id]  # 设置播报类型和ID
            # 写入播报寄存器
            self.i2c.write(bytes([ASR_SPEAK_REG]) + bytes(self.send))
    
    def close(self):
        """关闭I2C连接"""
        self.i2c.close()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)  # 初始化ROS2
    node = VoiceTaskPublisher()  # 创建节点
    
    try:
        rclpy.spin(node)  # 运行节点
    except KeyboardInterrupt:
        pass  # 捕获Ctrl+C中断
    finally:
        # 清理资源
        node.asr.close()  # 关闭语音模块
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()  # 关闭ROS2


if __name__ == '__main__':
    main()