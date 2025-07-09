// 默认停止状态的 Twist 消息
const defaultTwist = new ROSLIB.Message({
  linear: { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 }
});

// 底盘控制逻辑
function move(direction) {
  if (!ros || !cmdVel) {
    alert('ROS connection not established');
    return;
  }

  const twist = new ROSLIB.Message({
    linear: {
      x: direction.linear.x,
      y: direction.linear.y,
      z: direction.linear.z
    },
    angular: {
      x: direction.angular.x,
      y: direction.angular.y,
      z: direction.angular.z
    }
  });

  cmdVel.publish(twist);
}

// 按下按钮时发送运动命令
function moveForward() {
  move({ linear: { x: 0.4, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
}

function moveBackward() {
  move({ linear: { x: -0.4, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
}

function moveLeft() {
  move({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 30 } });
}

function moveRight() {
  move({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -30 } });
}

// 松开按钮时发送停止命令
function stopMovement() {
  cmdVel.publish(defaultTwist);
}

// 云台控制逻辑
function controlPanTilt(direction) {
  if (!ros || !panTiltCmd) {
    alert('Pan-Tilt control not initialized');
    return;
  }

  const twist = new ROSLIB.Message({
    linear: {
      x: direction.linear.x,
      y: direction.linear.y,
      z: direction.linear.z
    },
    angular: {
      x: direction.angular.x,
      y: direction.angular.y,
      z: direction.angular.z
    }
  });

  panTiltCmd.publish(twist);
}

// 按下按钮时发送云台运动命令
// 按下按钮时发送运动命令
function panLeft() {
  controlPanTilt({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 20 } });
}

function panRight() {
  controlPanTilt({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -20 } });
}

function tiltUp() {
  controlPanTilt({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 20, y: 0, z: 0 } });
}

function tiltDown() {
  controlPanTilt({ linear: { x: 0, y: 0, z: 0 }, angular: { x: -20, y: 0, z: 0 } });
}

// 松开按钮时发送停止命令
function stopPanTilt() {
  panTiltCmd.publish(defaultTwist);
}