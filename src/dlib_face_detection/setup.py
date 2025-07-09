from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dlib_face_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 安装包索引标记文件
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml
        ('share/' + package_name, ['package.xml']),
        # 安装 featureDB 文件夹
        (os.path.join('share', package_name, 'resource', 'featureDB'), glob('resource/featureDB/*')),
        # 安装 featureMean 文件夹
        (os.path.join('share', package_name, 'resource', 'featureMean'), glob('resource/featureMean/*')),
        # 安装 model 文件夹
        (os.path.join('share', package_name, 'resource', 'model'), glob('resource/model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@todo.todo',
    description='A ROS 2 package for face detection and recognition using dlib.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_recognition = dlib_face_detection.face_recognition:main',  # 注册 face_recognition 脚本
            'get_face_db = dlib_face_detection.getFaceDB:main',  # 注册 getFaceDB 脚本
        ],
    },
)