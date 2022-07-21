# coding=utf-8
import rosbag
import sys
import os
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy


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

if __name__ == '__main__':
    img_dir = sys.argv[1]   # 影像所在文件夹路径
    img_type = sys.argv[2]  # 影像类型
    topic_name = sys.argv[3]    # Topic名称
    bag_path = sys.argv[4]  # 输出Bag路径

    bag_out = rosbag.Bag(bag_path,'w')

    cb = CvBridge()

    paths, names, files = findFiles(img_dir,img_type)
    for i in range(len(files)):
        print(i,'/',len(files))

        frame_img = cv2.imread(files[i])
        timestamp = int(names[i].split(".")[0])/1e9
        print(timestamp)

        ros_ts = rospy.rostime.Time.from_sec(timestamp)
        ros_img = cb.cv2_to_imgmsg(frame_img,encoding='bgr8')
        ros_img.header.stamp = ros_ts
        bag_out.write(topic_name,ros_img,ros_ts)
    
    bag_out.close()
