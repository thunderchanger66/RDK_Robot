import os
import cv2
import dlib
import numpy as np
import pandas as pd

# ============================== 配置路径 ==============================
# 绝对路径
BASE_PATH = "/home/sunrise/ros_ws/src/dlib_face_detection/resource"

# 模型路径
PREDICTOR_PATH = os.path.join(BASE_PATH, "model", "shape_predictor_68_face_landmarks.dat")
MODEL_PATH = os.path.join(BASE_PATH, "model", "dlib_face_recognition_resnet_model_v1.dat")

# 人脸特征均值存储路径
PATH_FEATURE_MEAN = os.path.join(BASE_PATH, "featureMean")
# ============================== 初始化模型 ==============================
# 加载 Dlib 模型
detector = dlib.get_frontal_face_detector()  # 人脸检测器
predictor = dlib.shape_predictor(PREDICTOR_PATH)  # 人脸关键点标记器
face_rec = dlib.face_recognition_model_v1(MODEL_PATH)  # 人脸识别模型

# ============================== 函数定义 ==============================

def compute_distance(feature_1, feature_2):
    """
    计算两个 128D 特征向量的欧式距离
    """
    feature_1 = np.array(feature_1)
    feature_2 = np.array(feature_2)
    return np.linalg.norm(feature_1 - feature_2)


def load_face_database():
    """
    加载本地人脸特征数据库
    """
    head = [f"feature_{i+1}" for i in range(128)]
    face_path = os.path.join(PATH_FEATURE_MEAN, "feature_all.csv")
    face_feature = pd.read_csv(face_path, names=head)
    face_feature_array = np.array(face_feature)
    face_list = ["Chandler", "Joey", "Monica", "Phoebe", "Rachel", "Ross"]
    return face_feature_array, face_list


def process_camera(face_feature_array, face_list):
    """
    使用摄像头实时监测人脸
    """
    # 打开摄像头
    camera = cv2.VideoCapture(0)
    if not camera.isOpened():
        print("无法打开摄像头！")
        return

    # 创建窗口
    cv2.namedWindow("Face Recognition", cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow("Face Recognition", 680, 480)

    while True:
        ret, frame = camera.read()
        if not ret:
            print("无法读取摄像头帧！")
            break

        # 转为灰度图
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        dets = detector(gray, 1)  # 检测人脸

        if len(dets) > 0:
            for index, value in enumerate(dets):
                # 获取人脸关键点
                shape = predictor(gray, value)

                # 标记人脸区域
                cv2.rectangle(frame, (value.left(), value.top()), (value.right(), value.bottom()), (0, 255, 0), 2)

                # 提取 128D 特征
                face_descriptor = face_rec.compute_face_descriptor(frame, shape)
                face_vector = np.array(face_descriptor)

                # 人脸匹配
                recognized = False
                for j, known_face in enumerate(face_feature_array):
                    if compute_distance(face_vector, known_face) < 0.56:  # 阈值为 0.56
                        recognized = True
                        cv2.putText(frame, face_list[j], (value.left(), value.top()), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 255), 1, cv2.LINE_AA)
                        break

                if not recognized:
                    cv2.putText(frame, "Unknown", (value.left(), value.top()), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 255), 1, cv2.LINE_AA)

                # 标记关键点
                for pt in shape.parts():
                    pos = (pt.x, pt.y)
                    cv2.circle(frame, pos, 1, color=(0, 255, 0))

        # 显示结果
        cv2.imshow("Face Recognition", frame)
        if cv2.waitKey(1) == 27:  # 按下 ESC 键退出
            break

    # 释放资源
    camera.release()
    cv2.destroyAllWindows()

def main():
    face_feature_array, face_list = load_face_database()

    # 使用摄像头实时监测
    process_camera(face_feature_array, face_list)

# ============================== 主程序 ==============================
if __name__ == "__main__":
    # 加载人脸数据库
    main()