# 人脸检测与识别系统

## 功能介绍

本系统基于 ROS2 和 dlib 实现实时人脸检测、关键点提取和身份识别功能。

### 核心功能
- 实时人脸检测
- 106点人脸关键点提取
- dlib特征识别
- 身份匹配与发布

## 系统架构

### 文件结构
```
start_detect/
├── start_detect/
│   ├── __init__.py
│   └── start_detect.py      # 主程序
├── launch/
│   └── start_detect.launch.py   # 启动文件
├── resource/
│   ├── dlib_face_recognition_resnet_model_v1.dat    # 特征提取模型
│   └── face_db/            # 人脸数据库目录
├── package.xml
├── setup.py
└── setup.cfg
```



## 快速开始

### 安装依赖
```bash
ros2 launch start_detect start_detect.launch.py
```

## 系统参数

### 启动参数
- `face_threshold`: 人脸识别阈值 (默认: 0.56)
- `recognition_interval`: 识别间隔 (默认: 0.1s)

### ROS话题
- 订阅话题:
  - `/image`: 摄像头图像 (sensor_msgs/CompressedImage)
- 发布话题:
  - `/face_name`: 识别结果 (std_msgs/String)

## 使用说明

### 添加新人脸
1. 准备人脸图片，放入 `resource/face_db/` 目录
2. 图片命名格式：`姓名.jpg`
3. 重启节点，系统会自动加载新人脸

### 调整识别参数
```bash
# 修改识别阈值
ros2 launch start_detect start_detect.launch.py face_threshold:=0.5
```

### 性能优化
1. 建议在GPU环境下运行
2. 可调整图像分辨率提升性能
3. 优化光线条件提高准确率

## 常见问题

### 识别准确率低
- 检查光线条件
- 调整识别阈值
- 确保人脸图片质量
- 多角度采集人脸特征

### 系统延迟高
- 降低图像分辨率
- 检查CPU/GPU负载
- 调整识别间隔参数

## 开发说明

### 核心算法
1. 人脸检测：使用dlib内置检测器
2. 关键点提取：106点转68点映射
3. 特征提取：使用ResNet模型
4. 身份匹配：欧氏距离计算

### 代码结构
- `start_detect.py`: 主程序
  - `CompressedImageViewer`: 主类
  - `target_callback`: 识别回调
  - `compute_dst`: 特征匹配

## 维护说明
- 定期更新人脸库
- 备份重要配置
- 记录系统日志

## 许可证
Apache 2.0