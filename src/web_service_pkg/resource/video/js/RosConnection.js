let ros;
let cmdVel;
let panTiltCmd;
let faceNameListener;
let toolListener; // 新增工具监听器
let lastFaceName = "";
let lastFaceTime = null;
let currentTool = ""; // 当前工具

// 初始化 ROS 连接
function startConnection() {
  ros = new ROSLIB.Ros({
    url: 'ws://172.20.10.4:9090'
  });

  ros.on('connection', () => {
    console.log('已连接到 ROS Bridge');

    // 初始化底盘控制话题
    cmdVel = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    // 初始化云台控制话题
    panTiltCmd = new ROSLIB.Topic({
      ros: ros,
      name: '/pan_tilt_cmd',
      messageType: 'geometry_msgs/Twist'
    });

    // 初始化人脸识别话题
    faceNameListener = new ROSLIB.Topic({
      ros: ros,
      name: '/face_name',
      messageType: 'std_msgs/String'
    });

    // 新增工具识别话题
    toolListener = new ROSLIB.Topic({
      ros: ros,
      name: '/current_tool',
      messageType: 'std_msgs/String'
    });

    // 订阅人脸识别话题
    faceNameListener.subscribe((message) => {
      handleFaceRecognition(message.data);
    });

    // 订阅工具识别话题
    toolListener.subscribe((message) => {
      currentTool = message.data;
      console.log('当前工具:', currentTool);
    });

    initImageSubscription();
  });

    ros.on('error', (error) => {
    console.error('连接到 ROS 时出错：', error);
    alert('连接到 ROS 时出错：' + error);
    });

    ros.on('close', () => {
    console.log('与 ROS 断开连接');
    alert('与 ROS 断开连接');
    });

}
// 处理人脸识别结果
function handleFaceRecognition(faceName) {
    // 直接过滤掉"Unknown"的识别结果
    if (faceName === 'Unknown') return;

    const currentTime = new Date();
    
    const record = {
        date: currentTime.toLocaleDateString('zh-CN'),
        time: currentTime.toLocaleTimeString('zh-CN', {
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit',
            hour12: false
        }),
        name: faceName,
        tool: currentTool || "未指定"
    };
    
    addAttendanceRecord(record);
    console.log(`${faceName} 借用 ${record.tool} 成功! 时间: ${record.date} ${record.time}`);
    
    // 重置当前工具状态（保持人脸状态不重置，以便连续操作）
    currentTool = "";
}

startConnection();