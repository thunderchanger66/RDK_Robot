## Web服务说明

### 文件结构
```
web_service_pkg/
├── launch/
│   └── start_web_service.launch.py  # 启动文件
├── resource/
│   └── video/
│       ├── index.html               # 主页面
│       ├── css/
│       │   └── style.css           # 样式文件
│       └── js/
│           ├── RosConnection.js     # ROS通信相关
│           ├── MovementControl.js   # 底盘控制相关
│           ├── AttendanceSystem.js  # 考勤系统相关
│           └── client.js           # WebRTC客户端相关
└── web_service_pkg/
    └── server.py                    # WebRTC服务器
```

### 功能说明
- 左侧面板：
  - 实时视频流显示
  - 底盘移动控制按钮
  - 云台控制按钮
- 右侧面板：
  - 打卡记录实时显示
  - 按时间顺序排列
  - 显示具体打卡时间

### 启动方式

1. 启动人脸识别系统
```bash
ros2 launch start_detect start_detect.launch.py
```

2. 启动Web服务
```bash
ros2 launch web_service_pkg start_web_service.launch.py
```

3. 访问Web界面
```
http://<你的IP>:8001
```

### 话题说明
- `/cmd_vel`: 底盘控制指令
- `/pan_tilt_cmd`: 云台控制指令
- `/face_name`: 人脸识别结果发布
- `/image`: 压缩后的视频流
