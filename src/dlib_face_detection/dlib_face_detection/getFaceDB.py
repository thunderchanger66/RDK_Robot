# 从人脸图像文件中提取人脸特征存入 CSV
# Features extraction from images and save into features_all.csv

import os
import cv2
import dlib
import csv
import numpy as np
import pandas as pd
from skimage import io

# ============================== 配置路径 ==============================
# 绝对路径基础路径
BASE_PATH = "/home/sunrise/ros_ws/src/dlib_face_detection/resource"

# 人脸图像文件路径
PATH_IMAGES_FROM_CAMERA = os.path.join(BASE_PATH, 'faces')

# 人脸特征存储路径
PATH_FEATURE_DB = os.path.join(BASE_PATH, 'featureDB')

# 人脸特征均值存储路径
PATH_FEATURE_MEAN = os.path.join(BASE_PATH, 'featureMean')

# 模型路径
PREDICTOR_PATH = os.path.join(BASE_PATH, "model", "shape_predictor_68_face_landmarks.dat")
MODEL_PATH = os.path.join(BASE_PATH, "model", "dlib_face_recognition_resnet_model_v1.dat")

# ============================== 初始化模型 ==============================
# Dlib 正向人脸检测器
detector = dlib.get_frontal_face_detector()
# Dlib 人脸预测器
predictor = dlib.shape_predictor(PREDICTOR_PATH)
# Dlib 人脸识别模型
face_rec = dlib.face_recognition_model_v1(MODEL_PATH)


# ============================== 函数定义 ==============================

def return_128d_features(path_img):
    """
    返回单张图像的 128D 特征
    """
    img_rd = io.imread(path_img)
    img_gray = cv2.cvtColor(img_rd, cv2.COLOR_BGR2RGB)
    faces = detector(img_gray, 1)

    print(f"检测到人脸的图像: {path_img}")

    if len(faces) != 0:
        shape = predictor(img_gray, faces[0])
        face_descriptor = face_rec.compute_face_descriptor(img_gray, shape)
    else:
        face_descriptor = 0
        print("未检测到人脸")

    return face_descriptor


def write_into_csv(path_faces_personX, path_csv):
    """
    将文件夹中照片特征提取出来, 写入 CSV
    """
    dir_pics = os.listdir(path_faces_personX)
    with open(path_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        for pic in dir_pics:
            img_path = os.path.join(path_faces_personX, pic)
            print(f"正在处理人脸图像: {img_path}")
            features_128d = return_128d_features(img_path)

            # 跳过未检测到人脸的图片
            if features_128d != 0:
                writer.writerow(features_128d)


def compute_mean(feature_path):
    """
    计算 128D 特征均值
    """
    head = [f"feature_{i+1}" for i in range(128)]
    rdata = pd.read_csv(feature_path, names=head)
    mean_value = rdata.mean()
    return mean_value

def main():
    faces = os.listdir(PATH_IMAGES_FROM_CAMERA)
    for person in faces:
        person_path = os.path.join(PATH_IMAGES_FROM_CAMERA, person)
        csv_path = os.path.join(PATH_FEATURE_DB, f"{person}.csv")
        print(f"正在处理: {csv_path}")
        write_into_csv(person_path, csv_path)

    # 计算各个特征文件的均值，并将结果存储到 feature_all.csv 文件中
    features = os.listdir(PATH_FEATURE_DB)
    with open(os.path.join(PATH_FEATURE_MEAN, "feature_all.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        for feature_file in features:
            feature_path = os.path.join(PATH_FEATURE_DB, feature_file)
            mean_value = compute_mean(feature_path)
            writer.writerow(mean_value)

    print("人脸特征提取和均值计算完成！")

# ============================== 主程序 ==============================

if __name__ == "__main__":
    # 读取所有的人脸图像数据，将不同人的数据存入不同的 CSV 文件
   main()