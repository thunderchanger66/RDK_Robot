# 从人脸图像文件中提取人脸特征存入 CSV
# Features extraction from images and save into features_all.csv

import cv2
import os
import dlib
from skimage import io
import csv
import numpy as np
import pandas as pd

# 要读取人脸图像文件的路径
path_images_from_camera= "./src/start_detect/resource/faceS/"
path_featureDB= "./src/start_detect/resource/featureDB/"
path_featureMean="./src/start_detect/resource/featureMean/"
resources_path = os.path.abspath(os.path.join("./src/start_detect", "resource"))
predictor_path     = os.path.join(resources_path, "shape_predictor_68_face_landmarks.dat")
model_path         = os.path.join(resources_path, "dlib_face_recognition_resnet_model_v1.dat")
print(model_path)
# Dlib 正向人脸检测器
detector = dlib.get_frontal_face_detector()

# Dlib 人脸预测器
predictor = dlib.shape_predictor(predictor_path)

# Dlib 人脸识别模型
# Face recognition model, the object maps human faces into 128D vectors
face_rec = dlib.face_recognition_model_v1(model_path)


# 返回单张图像的 128D 特征
def return_128d_features(path_img):
    img_rd = io.imread(path_img)
    img_gray = cv2.cvtColor(img_rd, cv2.COLOR_BGR2RGB)
    faces = detector(img_gray, 1)
    print("%-40s %-20s" % ("检测到人脸的图像 / image with faces detected:", path_img), '\n')
    # 因为有可能截下来的人脸再去检测，检测不出来人脸了
    # 所以要确保是 检测到人脸的人脸图像 拿去算特征
    if len(faces) != 0:
        shape = predictor(img_gray, faces[0])
        face_descriptor = face_rec.compute_face_descriptor(img_gray, shape)
    else:
        face_descriptor = 0
        print("there is no face")

    return face_descriptor


# 将文件夹中照片特征提取出来, 写入 CSV
def write_into_csv(path_faces_personX, path_csv):
    dir_pics = os.listdir(path_faces_personX)
    with open(path_csv, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        for i in range(len(dir_pics)):
            # 调用return_128d_features()得到128d特征
            print("正在读的人脸图像：", path_faces_personX + "/" + dir_pics[i])
            features_128d = return_128d_features(path_faces_personX + "/" + dir_pics[i])
            #  print(features_128d)
            # 遇到没有检测出人脸的图片跳过
            if features_128d == 0:
                i += 1
            else:
                writer.writerow(features_128d)

#对不同的人的特征数据进行取均值并将结果存储到all_feature。csv文件中
def computeMean(feature_path):
    head=[]
    for i in range(128):
        fe="feature_"+str(i+1)
        head.append(fe)
    #需设置表头，当表头缺省时，会将第一行数据当作表头
    rdata = pd.read_csv(feature_path,names=head)
    meanValue=rdata.mean()
    print(len(meanValue))
    print(type(meanValue))
    print(meanValue)
    return meanValue

#读取所有的人脸图像的数据，将不同人的数据存在不同的csv文件中，以便取均值进行误差降低
#faces = os.listdir(path_images_from_camera)
faces = sorted(os.listdir(path_images_from_camera))
i=0
for person in faces:
    i+=1
    print(path_featureDB+ person + ".csv")
    write_into_csv(path_images_from_camera+person, path_featureDB+ person+".csv")
print(i)

#计算各个特征文件中的均值，并将值存在feature_all文件中
#features=os.listdir(path_featureDB)
features = sorted(os.listdir(path_featureDB))
i=0
with open(path_featureMean + "feature_all.csv", "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    for fea in features:
        i+=1
        meanValue=computeMean(path_featureDB+fea)
        writer.writerow(meanValue)
print(i)