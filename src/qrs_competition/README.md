# QRS 竞赛系统说明文档

## 系统架构


## 快速启动

```bash
# 一键启动所有功能
ros2 launch qrs_competition bringup.launch.py
```

### 访问界面
```
http://<你的IP>:8001
```

## 系统话题

### 订阅话题
- `/cmd_vel`: 底盘控制指令
- `/pan_tilt_cmd`: 云台控制指令

### 发布话题
- `/face_name`: 人脸识别结果
- `/image`: 压缩视频流



### 文件结构
```
qrs_competition/
├── launch/
│   └── bringup.launch.py      # 统一启动文件
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 依赖功能包
- **start_detect**: 人脸检测与识别
- **web_service_pkg**: Web服务与控制界面

## 功能说明

### 人脸识别系统
- 实时人脸检测与关键点提取
- 基于dlib的人脸特征识别
- 支持多人脸同时识别
- 发布识别结果到`/face_name`话题

### Web控制界面
- 左侧面板：
  - 实时视频流显示
  - 底盘移动控制
  - 云台控制
- 右侧面板：
  - 实时打卡记录
  - 按时间排序显示
  - 包含日期、时间、姓名

### 打卡功能
- 触发条件：3秒内连续识别到同一人脸
- 记录内容：日期、时间、人名
- 实时显示：自动更新到Web界面
- 查询功能：支持按名字搜索

