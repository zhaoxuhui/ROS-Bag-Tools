# coding=utf-8
import rosbag
import sys
import os
import numpy as np
import cv2
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import rospy
from geometry_msgs.msg import Vector3


def findFiles(root_dir, filter_type, reverse=False):
    """
    在指定目录查找指定类型文件 -> paths, names, files
    :param root_dir: 查找目录
    :param filter_type: 文件类型
    :param reverse: 是否返回倒序文件列表，默认为False
    :return: 路径、名称、文件全路径
    """

    separator = os.path.sep
    paths = []
    names = []
    files = []
    for parent, dirname, filenames in os.walk(root_dir):
        for filename in filenames:
            if filename.endswith(filter_type):
                paths.append(parent + separator)
                names.append(filename)
    for i in range(paths.__len__()):
        files.append(paths[i] + names[i])
    print(names.__len__().__str__() + " files have been found.")
    
    paths = np.array(paths)
    names = np.array(names)
    files = np.array(files)

    index = np.argsort(files)

    paths = paths[index]
    names = names[index]
    files = files[index]

    paths = list(paths)
    names = list(names)
    files = list(files)
    
    if reverse:
        paths.reverse()
        names.reverse()
        files.reverse()
    return paths, names, files

def readIMU(imu_path):
    timestamps = []
    wxs = []
    wys = []
    wzs = []
    axs = []
    ays = []
    azs = []
    fin = open(imu_path, 'r')
    fin.readline()
    line = fin.readline().strip()
    while line:
        parts = line.split(",")
        ts = float(parts[0])/1e9
        wx = float(parts[1])
        wy = float(parts[2])
        wz = float(parts[3])
        ax = float(parts[4])
        ay = float(parts[5])
        az = float(parts[6])
        timestamps.append(ts)

        wxs.append(wx)
        wys.append(wy)
        wzs.append(wz)
        axs.append(ax)
        ays.append(ay)
        azs.append(az)
        line = fin.readline().strip()
    return timestamps, wxs, wys, wzs, axs, ays, azs


if __name__ == '__main__':
    img_dir = sys.argv[1]   # 影像所在文件夹路径
    img_type = sys.argv[2]  # 影像文件类型
    img_topic_name = sys.argv[3]    # 影像Topic名称
    imu_path = sys.argv[4]  # IMU文件路径
    imu_topic_name = sys.argv[5]    # IMU Topic名称
    bag_path = sys.argv[6]  # Bag文件输出路径

    bag_out = rosbag.Bag(bag_path,'w')

    # 先处理IMU数据
    imu_ts, wxs, wys, wzs, axs, ays, azs = readIMU(imu_path)
    imu_msg = Imu()
    angular_v = Vector3()
    linear_a = Vector3()

    for i in range(len(imu_ts)):
        imu_ts_ros = rospy.rostime.Time.from_sec(imu_ts[i])
        imu_msg.header.stamp = imu_ts_ros
        
        angular_v.x = wxs[i]
        angular_v.y = wys[i]
        angular_v.z = wzs[i]

        linear_a.x = axs[i]
        linear_a.y = ays[i]
        linear_a.z = azs[i]

        imu_msg.angular_velocity = angular_v
        imu_msg.linear_acceleration = linear_a

        bag_out.write(imu_topic_name, imu_msg, imu_ts_ros)
        print('imu:',i,'/',len(imu_ts))

    # 再处理影像数据
    paths, names, files = findFiles(img_dir,img_type)
    cb = CvBridge()
    
    for i in range(len(files)):
        print('image:',i,'/',len(files))

        frame_img = cv2.imread(files[i])
        timestamp = int(names[i].split(".")[0])/1e9
        print(timestamp)

        ros_ts = rospy.rostime.Time.from_sec(timestamp)
        ros_img = cb.cv2_to_imgmsg(frame_img,encoding='bgr8')
        ros_img.header.stamp = ros_ts
        bag_out.write(img_topic_name,ros_img,ros_ts)
    
    bag_out.close()
