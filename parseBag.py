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
    img_topic_list = []
    for i in range(len(topic_list)):
        if topic_list[i]['type'].__contains__("Image"):
            img_topic_list.append(topic_list[i])
        topic_info_item = "No." + str(i + 1) + ": Name: " + topic_list[i]['topic'] + ", \t\tType: " + topic_list[i][
            'type'] + ", \t\tMessages:" + str(topic_list[i]['messages']) + ", \t\tFramerate:" + str(
            int(topic_list[i]['messages'] / duration))
        print(topic_info_item)
        info_strs.append(topic_info_item + "\n")
    print("-" * 100)
    info_strs.append("-" * 100 + "\n")
    return info_dict, topic_list, img_topic_list, start_timestamp, end_timestamp, duration, info_strs


def getTopicImgSize(bag_path, topic_name):
    img_width = 0
    img_height = 0
    img_encoding = "bgr8"
    with rosbag.Bag(bag_path, 'r') as bag:
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages():
            if topic == topic_name:
                img_width = msg.width
                img_height = msg.height
                img_encoding = msg.encoding
                break
    return img_width, img_height, img_encoding


def getImgTopicInfo(bag_path, img_topic_list, info_strs):
    name_list = []
    num_list = []
    size_list = []
    framerate_list = []
    encoding_list = []
    duration = float(info_dict['duration'])

    print("-" * 100)
    print("Image topic list:")
    info_strs.append("-" * 100 + "\n")
    info_strs.append("Image topic list:\n")

    for i in range(len(img_topic_list)):
        topic_name = img_topic_list[i]['topic']
        img_width, img_height, img_encoding = getTopicImgSize(bag_path, topic_name)
        framerate = int(img_topic_list[i]['messages'] / duration)
        msg_num = int(img_topic_list[i]['messages'])

        item_str = "Topic " + str(i + 1) + ": Name: " + topic_name + ", \tType: " + img_topic_list[i][
            'type'] + ", \tMessages:" + str(msg_num) + ", \tFramerate:" + str(framerate) + "\tSize:" + str(
            img_width) + "x" + str(img_height) + ", \tDatatype:" + img_encoding

        print(item_str)
        info_strs.append(item_str + "\n")

        name_list.append(topic_name)
        num_list.append(msg_num)
        size_list.append((img_width, img_height))
        framerate_list.append(framerate)
        encoding_list.append(img_encoding)

    print("-" * 100)
    info_strs.append("-" * 100 + "\n")
    return name_list, num_list, size_list, framerate_list, encoding_list, info_strs


def saveTopicImgs(bag_path, topic_name, start_time, end_time, msg_num, frame_encoding, save_path, file_type='.png'):
    if frame_encoding.__contains__("rgb8"):
        target_encoding = "bgr8"
    else:
        target_encoding = frame_encoding

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
                        cv_img = bridge.imgmsg_to_cv2(msg, target_encoding)
                        timestr = "%.0f" % (msg.header.stamp.to_sec()*1000000000)  # ns(10^-9)
                        # timestr = "%.6f" % msg.header.stamp.to_sec() # s
                        img_name = timestr + file_type
                        cv2.imwrite(save_path + os.path.sep + img_name, cv_img)
                        counter += 1
                        print(str(counter) + os.path.sep + str(msg_num) + ", " + timestr + os.path.sep + str(end_time))
                    except CvBridgeError as e:
                        print(e)
    print("Frame images have been saved at:" + save_path)


def showTopicImgs(bag_path, topic_name, start_time, end_time, msg_num, frame_rate, frame_encoding):
    if frame_encoding.__contains__("rgb8"):
        target_encoding = "bgr8"
    else:
        target_encoding = frame_encoding

    counter = 0
    with rosbag.Bag(bag_path, 'r') as bag:
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages():
            cur_time = msg.header.stamp.to_sec()
            if start_time <= cur_time <= end_time:
                if topic == topic_name:
                    try:
                        cv_img = bridge.imgmsg_to_cv2(msg, target_encoding)
                        # timestr = "%.0f" % (msg.header.stamp.to_sec()*1000000000)  # ns(10^-9)
                        timestr = "%.6f" % msg.header.stamp.to_sec() # s
                        cv2.imshow(topic_name, cv_img)
                        cv2.waitKey(int(1000.0 / frame_rate))
                        counter += 1
                        print(str(counter) + os.path.sep + str(msg_num) + ", " + timestr + os.path.sep + str(end_time))
                    except CvBridgeError as e:
                        print(e)


def saveTopicVideo(bag_path, topic_name, start_time, end_time, msg_num, frame_width, frame_height, frame_rate,
                   frame_encoding, save_path):
    if frame_encoding.__contains__("rgb8"):
        target_encoding = "bgr8"
    else:
        target_encoding = frame_encoding

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out_video = cv2.VideoWriter(save_path, fourcc, frame_rate, (frame_width, frame_height))

    counter = 0

    if frame_encoding == "mono8":
        with rosbag.Bag(bag_path, 'r') as bag:
            bridge = CvBridge()
            for topic, msg, t in bag.read_messages():
                cur_time = msg.header.stamp.to_sec()
                if start_time <= cur_time <= end_time:
                    if topic == topic_name:
                        try:
                            cv_img = bridge.imgmsg_to_cv2(msg, target_encoding)
                            out_video.write(cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR))

                            timestr = "%.6f" % msg.header.stamp.to_sec()
                            counter += 1

                            print(str(counter) + os.path.sep + str(msg_num) + ", " + timestr + os.path.sep + str(
                                end_time))
                        except CvBridgeError as e:
                            print(e)
    elif frame_encoding == "rgb8":
        with rosbag.Bag(bag_path, 'r') as bag:
            bridge = CvBridge()
            for topic, msg, t in bag.read_messages():
                cur_time = msg.header.stamp.to_sec()
                if start_time <= cur_time <= end_time:
                    if topic == topic_name:
                        try:
                            cv_img = bridge.imgmsg_to_cv2(msg, target_encoding)
                            out_video.write(cv_img)

                            timestr = "%.6f" % msg.header.stamp.to_sec()
                            counter += 1

                            print(str(counter) + os.path.sep + str(msg_num) + ", " + timestr + os.path.sep + str(
                                end_time))
                        except CvBridgeError as e:
                            print(e)
    elif frame_encoding == "bgr8":
        with rosbag.Bag(bag_path, 'r') as bag:
            bridge = CvBridge()
            for topic, msg, t in bag.read_messages():
                cur_time = msg.header.stamp.to_sec()
                if start_time <= cur_time <= end_time:
                    if topic == topic_name:
                        try:
                            cv_img = bridge.imgmsg_to_cv2(msg, target_encoding)
                            out_video.write(cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB))

                            timestr = "%.6f" % msg.header.stamp.to_sec()
                            counter += 1

                            print(str(counter) + os.path.sep + str(msg_num) + ", " + timestr + os.path.sep + str(
                                end_time))
                        except CvBridgeError as e:
                            print(e)
    else:
        print("Usupported data format:" + frame_encoding)

    out_video.release()
    print("Video file has been saved at:" + save_path)


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

            info_dict, topic_list, img_topic_list, bag_start_timestamp, bag_end_timestamp, bag_duration, info_strs = getSummaryInfo(
                bag_path)
            img_name_list, img_num_list, img_size_list, img_framerate_list, img_encoding_list, info_strs = getImgTopicInfo(
                bag_path, img_topic_list, info_strs)

            try:
                input_index = int(input("Select a image topic to output(1 as default):\n")) - 1
            except:
                input_index = 0
            if input_index >= len(topic_list):
                input_index = 0
            print("==>Selected topic: " + str(input_index) + " " + str(img_topic_list[input_index]['topic']))

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
                    "\nSelect operation(mode 1 as default):\n\t(1)Output preview video\n\t(2)Output general summary file\n\t(3)Output preview video and general summary file\n\t(4)Output frame images\n\t(5)Output frames of all image topics\n\t(6)Output preview video, summary file, frame images of all topics\n\t(7)Show frame images\n"))
            except:
                input_mode_flag = 1
            print("==>Select mode:" + str(input_mode_flag))

            print("\nStart processing...")
            if input_mode_flag == 1:
                # 输出预览视频
                try:
                    input_speed_factor = float(input("\nInput speed factor for output video(1.0 as default):\n"))
                except:
                    input_speed_factor = 1.0
                print("==>Speed factor:" + str(input_speed_factor))

                video_save_path = bag_path.split(".")[0] + ".mp4"
                saveTopicVideo(bag_path, img_topic_list[input_index]['topic'], input_start_time, input_end_time,
                               img_num_list[input_index], img_size_list[input_index][0], img_size_list[input_index][1],
                               int(input_speed_factor * img_framerate_list[input_index]),
                               img_encoding_list[input_index], video_save_path)
            elif input_mode_flag == 2:
                # 输出汇总信息文件
                summary_save_path = bag_path.split(".")[0] + ".txt"
                saveSummaryFile(bag_path, summary_save_path, info_strs)
            elif input_mode_flag == 3:
                # 输出预览视频+汇总信息文件
                try:
                    input_speed_factor = float(input("\nInput speed factor for output video(1.0 as default):\n"))
                except:
                    input_speed_factor = 1.0
                print("==>Speed factor:" + str(input_speed_factor))

                video_save_path = bag_path.split(".")[0] + ".mp4"
                saveTopicVideo(bag_path, img_topic_list[input_index]['topic'], input_start_time, input_end_time,
                               img_num_list[input_index], img_size_list[input_index][0], img_size_list[input_index][1],
                               int(input_speed_factor * img_framerate_list[input_index]),
                               img_encoding_list[input_index], video_save_path)
                summary_save_path = bag_path.split(".")[0] + ".txt"
                saveSummaryFile(bag_path, summary_save_path, info_strs)
            elif input_mode_flag == 4:
                # 输出指定Topic Message到影像文件
                save_topic_name = img_topic_list[input_index]['topic'][1:].replace("/", "-")
                frame_save_path = bag_path[:bag_path.rfind(os.path.sep) + 1]
                if frame_save_path == "":
                    frame_save_path = "."
                frame_save_path += os.path.sep + bag_path.split(os.path.sep)[-1].split(".")[
                    0] + os.path.sep + save_topic_name
                isDirExist(frame_save_path)

                frame_save_type = ".jpg"
                try:
                    frame_save_type = raw_input("\nInput file save format(.jpg as dafault):\n")
                except:
                    frame_save_type = ".jpg"
                if len(frame_save_type) == 0:
                    frame_save_type = ".jpg"
                if frame_save_type[0] != ".":
                    frame_save_type = "." + frame_save_type
                print("==>Save file format:" + frame_save_type)
                
                # msg_num有问题
                saveTopicImgs(bag_path, img_topic_list[input_index]['topic'], input_start_time, input_end_time,
                              img_num_list[input_index], img_encoding_list[input_index], frame_save_path,
                              frame_save_type)
            elif input_mode_flag == 5:
                # 输出Bag中所有Topic Message到影像文件
                for k in range(len(img_topic_list)):
                    save_topic_name = img_topic_list[k]['topic'][1:].replace("/", "-")
                    frame_save_path = bag_path[:bag_path.rfind(os.path.sep) + 1]
                    if frame_save_path == "":
                        frame_save_path = "."
                    frame_save_path += os.path.sep + bag_path.split(os.path.sep)[-1].split(".")[
                        0] + os.path.sep + save_topic_name
                    isDirExist(frame_save_path)

                    frame_save_type = ".jpg"
                    try:
                        frame_save_type = raw_input(
                            "\nInput file save format(.jpg as dafault) for " + img_name_list[k] + ":\n")
                    except:
                        frame_save_type = ".jpg"
                    
                    if len(frame_save_type) == 0:
                        frame_save_type = ".jpg"
                    
                    if not frame_save_type.startswith("."):
                        frame_save_type = "." + frame_save_type
                    
                    if not img_encoding_list[k].__contains__("8"):
                        print("Non 8bit quantification detected! Auto changed to png format.")
                        frame_save_type = ".png"
                    else:
                        frame_save_type = ".jpg"

                    print("==>Save file format:" + frame_save_type)

                    saveTopicImgs(bag_path, img_topic_list[k]['topic'], input_start_time, input_end_time,
                                  img_num_list[k], img_encoding_list[k], frame_save_path, frame_save_type)
            elif input_mode_flag == 6:
                # 输出预览视频+汇总信息文件+所有Topic Message影像
                try:
                    input_speed_factor = float(input("\nInput speed factor for output video(1.0 as default):\n"))
                except:
                    input_speed_factor = 1.0
                print("==>Speed factor:" + str(input_speed_factor))

                video_save_path = bag_path.split(".")[0] + ".mp4"
                saveTopicVideo(bag_path, img_topic_list[input_index]['topic'], input_start_time, input_end_time,
                               img_num_list[input_index], img_size_list[input_index][0], img_size_list[input_index][1],
                               int(input_speed_factor * img_framerate_list[input_index]),
                               img_encoding_list[input_index], video_save_path)
                summary_save_path = bag_path.split(".")[0] + ".txt"
                saveSummaryFile(bag_path, summary_save_path, info_strs)

                for k in range(len(img_topic_list)):
                    save_topic_name = img_topic_list[k]['topic'][1:].replace("/", "-")
                    frame_save_path = bag_path[:bag_path.rfind(os.path.sep) + 1]
                    if frame_save_path == "":
                        frame_save_path = "."
                    frame_save_path += os.path.sep + bag_path.split(os.path.sep)[-1].split(".")[
                        0] + os.path.sep + save_topic_name
                    isDirExist(frame_save_path)

                    frame_save_type = ".jpg"

                    try:
                        frame_save_type = raw_input(
                            "\nInput file save format(.jpg as dafault) for " + img_name_list[k] + ":\n")
                    except:
                        frame_save_type = ".jpg"
                    
                    if len(frame_save_type) == 0:
                        frame_save_type = ".jpg"
                    
                    if not frame_save_type.startswith("."):
                        frame_save_type = "." + frame_save_type
                    
                    if not img_encoding_list[k].__contains__("8"):
                        print("Non 8bit quantification detected! Auto changed to png format.")
                        frame_save_type = ".png"
                    else:
                        frame_save_type = ".jpg"

                    print("==>Save file format:" + frame_save_type)

                    saveTopicImgs(bag_path, img_topic_list[k]['topic'], input_start_time, input_end_time,
                                  img_num_list[k], img_encoding_list[k], frame_save_path, frame_save_type)
            elif input_mode_flag == 7:
                # 展示指定Topic Message
                try:
                    input_speed_factor = float(input("\nInput speed factor for showing(1.0 as default):\n"))
                except:
                    input_speed_factor = 1.0
                print("==>Speed factor:" + str(input_speed_factor))

                showTopicImgs(bag_path, img_topic_list[input_index]['topic'], input_start_time, input_end_time,
                              img_num_list[input_index], int(input_speed_factor * img_framerate_list[input_index]),
                              img_encoding_list[input_index])

            print("Processed: " + sys.argv[i] + " " + str(i) + "/" + str(file_num) + "\n")
