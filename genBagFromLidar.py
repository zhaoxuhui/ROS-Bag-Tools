# coding=utf-8
import os
import sys
import numpy as np
import cv2
import rosbag
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField


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

def readLidarCSV(file_path):
    point_list = []
    fin = open(file_path, 'r')
    fin.readline()
    line = fin.readline().strip()
    while line:
        parts = line.split(",")
        intensity = float(parts[0])
        azimuth = float(parts[2])
        distance = float(parts[3])
        vertical_angle = float(parts[5])
        pos_x = float(parts[6])
        pos_y = float(parts[7])
        pos_z = float(parts[8])

        point_list.append([pos_x, pos_y, pos_z, intensity, distance, azimuth, vertical_angle])

        line = fin.readline().strip()

    return point_list


if __name__ == '__main__':
    input_dir = sys.argv[1]
    out_dir = sys.argv[2]

    # lidar参数
    lidar_dir = input_dir
    lidar_type = ".csv"
    lidar_topic_name = "/VLP/pointcloud"

    # 统计信息
    fout = open(out_dir + "/summary.txt", 'w')
    # Bag输出路径
    bag_path = out_dir + "/combine.bag"

    # 新建ROS Bag输出
    bag_out = rosbag.Bag(bag_path, 'w')

    # LiDAR数据转换
    # ----------------------------------------------------------
    paths, names, files = findFiles(lidar_dir, lidar_type)

    for i in range(len(files)):
        ts = float(names[i].split(".")[0]) / 1e9
        tmp_path = files[i]

        point_list = readLidarCSV(tmp_path)

        points = np.array(point_list)

        lidar_msg = PointCloud2()
        lidar_msg.header.frame_id = "map"
        lidar_ts_ros = rospy.rostime.Time.from_sec(ts)
        lidar_msg.header.stamp = lidar_ts_ros

        if len(points.shape) == 3:
            lidar_msg.height = points.shape[1]
            lidar_msg.width = points.shape[0]
        else:
            lidar_msg.height = 1
            lidar_msg.width = len(points)

        lidar_msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('intensity', 12, PointField.FLOAT32, 1),
            PointField('distance', 16, PointField.FLOAT32, 1),
            PointField('azimuth', 20, PointField.FLOAT32, 1),
            PointField('vertical_angle', 24, PointField.FLOAT32, 1),
        ]
        lidar_msg.is_bigendian = False
        lidar_msg.point_step = 28
        lidar_msg.row_step = lidar_msg.point_step * points.shape[0]
        lidar_msg.is_dense = False
        lidar_msg.data = np.asarray(points, np.float32).tostring()

        bag_out.write(lidar_topic_name, lidar_msg, lidar_ts_ros)

        print("lidar", i + 1, "/", len(files),',',ts)
    fout.write("LiDAR start timestamp(unit:s)\t"+str(int(names[0].split(".")[0]) / 1e9)+"\n")
    fout.write("LiDAR end timestamp(unit:s)\t"+str(int(names[-1].split(".")[0]) / 1e9)+"\n")
    # ----------------------------------------------------------

    bag_out.close()
    fout.close()
