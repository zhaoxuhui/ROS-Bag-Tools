# coding=utf-8
import rosbag
import rospy
import subprocess, yaml
import cv2
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import sys
import os


def getSummaryInfo(bag_path):
    info_strs = []
    info_dict = yaml.load(
        subprocess.Popen(['rosbag', 'info', '--yaml', bag_path], stdout=subprocess.PIPE).communicate()[0])
    end_timestamp = float(info_dict['end'])
    duration = float(info_dict['duration'])
    start_timestamp = end_timestamp - duration

    start_time_str = "Start timestamp:" + str(start_timestamp) + " s"
    end_time_str = "End timestamp:" + str(end_timestamp) + " s"
    duration_str = "Duration:" + str(duration) + " s"

    print("-" * 100)
    print("Summary Info:")
    print(start_time_str)
    print(end_time_str)
    print(duration_str)
    info_strs.append("-" * 100 + "\n")
    info_strs.append("Summary Info:\n")
    info_strs.append(start_time_str + "\n")
    info_strs.append(end_time_str + "\n")
    info_strs.append(duration_str + "\n")

    print("Topic list:")
    info_strs.append("Topic list:\n")

    topic_list = info_dict['topics']
    imu_topic_list = []
    for i in range(len(topic_list)):
        if topic_list[i]['type'].__contains__("Imu"):
            imu_topic_list.append(topic_list[i])
        topic_info_item = "No." + str(i + 1) + ": Name: " + topic_list[i]['topic'] + ", \t\tType: " + topic_list[i][
            'type'] + ", \t\tMessages:" + str(topic_list[i]['messages']) + ", \t\tFramerate:" + str(
            int(topic_list[i]['messages'] / duration))
        print(topic_info_item)
        info_strs.append(topic_info_item + "\n")
    print("-" * 100)
    info_strs.append("-" * 100 + "\n")
    return info_dict, topic_list, imu_topic_list, start_timestamp, end_timestamp, duration, info_strs

def getImuTopicInfo(bag_path, imu_topic_list, info_strs):
    name_list = []
    num_list = []
    framerate_list = []
    duration = float(info_dict['duration'])

    print("-" * 100)
    print("Imu topic list:")
    info_strs.append("-" * 100 + "\n")
    info_strs.append("Imu topic list:\n")

    for i in range(len(imu_topic_list)):
        topic_name = imu_topic_list[i]['topic']
        framerate = int(imu_topic_list[i]['messages'] / duration)
        msg_num = int(imu_topic_list[i]['messages'])

        item_str = "Topic " + str(i + 1) + ": Name: " + topic_name + ", \tType: " + imu_topic_list[i][
            'type'] + ", \tMessages:" + str(msg_num) + ", \tFramerate:" + str(framerate)

        print(item_str)
        info_strs.append(item_str + "\n")

        name_list.append(topic_name)
        num_list.append(msg_num)
        framerate_list.append(framerate)

    print("-" * 100)
    info_strs.append("-" * 100 + "\n")
    return name_list, num_list, framerate_list, info_strs

def saveTopicIMU(bag_path, topic_name, start_time, end_time, msg_num, save_path, file_type='.csv'):
    out_file_path = save_path + os.path.sep + "imu_data" + file_type
    out_file = open(out_file_path,"w")
    out_file.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]\n")

    counter = 0
    with rosbag.Bag(bag_path, 'r') as bag:
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages():
            cur_time = msg.header.stamp.to_sec()
            if cur_time < start_time:
                pass
            elif cur_time > end_time:
                break
            elif start_time <= cur_time <= end_time:
                if topic == topic_name:
                    try:
                        # 按格式保存IMU数据
                        timestr = "%.0f" % (msg.header.stamp.to_sec()*1000000000)  # ns(10^-9)
                        # timestr = "%.6f" % msg.header.stamp # s

                        w_x = msg.angular_velocity.x
                        w_y = msg.angular_velocity.y
                        w_z = msg.angular_velocity.z

                        a_x = msg.linear_acceleration.x
                        a_y = msg.linear_acceleration.y
                        a_z = msg.linear_acceleration.z

                        out_file.write(timestr+","+str(w_x)+","+str(w_y)+","+str(w_z)+","+str(a_x)+","+str(a_y)+","+str(a_z)+"\n")
                        counter += 1
                        print(str(counter) + os.path.sep + str(msg_num) + ", " + timestr + os.path.sep + str(end_time))
                    except CvBridgeError as e:
                        print(e)
    print("IMU data have been saved at:" + save_path)

def saveSummaryFile(bag_path, save_path, info_strs):
    save_path = save_path
    fout = open(save_path, 'w')
    for i in range(len(info_strs)):
        fout.write(info_strs[i])
    fout.close()
    print("Summary file has been saved at:" + save_path)

def isDirExist(test_path):
    if os.path.exists(test_path):
        return True
    else:
        os.makedirs(test_path)
        return False


if __name__ == "__main__":
    file_num = len(sys.argv) - 1
    if file_num <= 0:
        print("Not enough input files, exit. You should input one bag filepath at least.")
    else:
        for i in range(1, len(sys.argv)):

            print("Processing: " + sys.argv[i] + " " + str(i) + "/" + str(file_num))

            bag_path = sys.argv[i]

            info_dict, topic_list, imu_topic_list, bag_start_timestamp, bag_end_timestamp, bag_duration, info_strs = getSummaryInfo(
                bag_path)
            imu_name_list, imu_num_list, imu_framerate_list, info_strs = getImuTopicInfo(bag_path,imu_topic_list,info_strs)

            try:
                input_index = int(input("Select a imu topic to output(1 as default):\n")) - 1
            except:
                input_index = 0
            if input_index >= len(topic_list):
                input_index = 0
            print("==>Selected topic: " + str(input_index) + " " + str(imu_topic_list[input_index]['topic']))

            try:
                input_start_time = float(input("\nInput start second from the beginning(0 as default):\n"))
            except:
                input_start_time = 0
            if input_start_time < 0:
                input_start_time = 0
            input_start_time = bag_start_timestamp + input_start_time
            print("==>Start time:" + str(input_start_time))

            try:
                input_end_time = float(input("\nInput end second from the beginning(whole as default):\n"))
            except:
                input_end_time = bag_duration
            input_end_time = bag_start_timestamp + input_end_time
            if input_end_time > bag_end_timestamp:
                input_end_time = bag_end_timestamp
            print("==>End time:" + str(input_end_time))

            try:
                input_mode_flag = int(input(
                    "\nSelect operation(mode 1 as default):\n\t(1)Output IMU data\n\t(2)Output general summary file\n\t(3)Output general summary file and IMU data\n"))
            except:
                input_mode_flag = 1
            print("==>Select mode:" + str(input_mode_flag))

            print("\nStart processing...")
            if input_mode_flag == 1:
                # 输出IMU数据
                for k in range(len(imu_topic_list)):
                    save_topic_name = imu_topic_list[k]['topic'][1:].replace("/", "-")
                    frame_save_path = bag_path[:bag_path.rfind(os.path.sep) + 1]
                    if frame_save_path == "":
                        frame_save_path = "."
                    frame_save_path += os.path.sep + bag_path.split(os.path.sep)[-1].split(".")[
                        0] + os.path.sep + save_topic_name
                    isDirExist(frame_save_path)

                    frame_save_type = ".csv"

                    try:
                        frame_save_type = raw_input(
                            "\nInput file save format(.csv as dafault) for " + imu_name_list[k] + ":\n")
                    except:
                        frame_save_type = ".csv"
                    
                    if len(frame_save_type) == 0:
                        frame_save_type = ".csv"
                    
                    if not frame_save_type.startswith("."):
                        frame_save_type = "." + frame_save_type

                    print("==>Save file format:" + frame_save_type)

                    saveTopicIMU(bag_path, imu_topic_list[k]['topic'], input_start_time, input_end_time,
                                  imu_num_list[k], frame_save_path, frame_save_type)
            elif input_mode_flag == 2:
                # 输出汇总信息文件
                summary_save_path = bag_path.split(".")[0] + ".txt"
                saveSummaryFile(bag_path, summary_save_path, info_strs)
            elif input_mode_flag == 3:
                # 汇总信息文件+IMU数据
                summary_save_path = bag_path.split(".")[0] + ".txt"
                saveSummaryFile(bag_path, summary_save_path, info_strs)

                for k in range(len(imu_topic_list)):
                    save_topic_name = imu_topic_list[k]['topic'][1:].replace("/", "-")
                    frame_save_path = bag_path[:bag_path.rfind(os.path.sep) + 1]
                    if frame_save_path == "":
                        frame_save_path = "."
                    frame_save_path += os.path.sep + bag_path.split(os.path.sep)[-1].split(".")[
                        0] + os.path.sep + save_topic_name
                    isDirExist(frame_save_path)

                    frame_save_type = ".csv"

                    try:
                        frame_save_type = raw_input(
                            "\nInput file save format(.csv as dafault) for " + imu_name_list[k] + ":\n")
                    except:
                        frame_save_type = ".csv"
                    
                    if len(frame_save_type) == 0:
                        frame_save_type = ".csv"
                    
                    if not frame_save_type.startswith("."):
                        frame_save_type = "." + frame_save_type

                    print("==>Save file format:" + frame_save_type)

                    saveTopicIMU(bag_path, imu_topic_list[k]['topic'], input_start_time, input_end_time,
                                  imu_num_list[k], frame_save_path, frame_save_type)

            print("Processed: " + sys.argv[i] + " " + str(i) + "/" + str(file_num) + "\n")
