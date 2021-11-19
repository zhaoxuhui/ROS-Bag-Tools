# coding=utf-8
import os
import sys
import time


def checkPythonVersion():
    return float(sys.version[:3])

if __name__ == '__main__':
    bag_file = str(sys.argv[1])
    # step1 导出影像
    print("Extracting images in bag file ...")
    os.system("python parseBag.py "+bag_file)

    # step2 导出IMU
    print("Extracting IMU dataset in bag file ...")
    os.system("python convertIMU2csv.py "+bag_file)

    # step3 生成时间戳文件
    print("Generating timestamp file ...")
    os.system("python genTimeStamps.py "+bag_file[:-4]+"/camera-infra1-image_rect_raw")