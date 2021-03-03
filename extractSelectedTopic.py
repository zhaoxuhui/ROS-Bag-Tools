# coding=utf-8
import rosbag
import subprocess, yaml
import sys

def getFramerateInfo(bag_path):
    info_dict = yaml.load(
        subprocess.Popen(['rosbag', 'info', '--yaml', bag_path], stdout=subprocess.PIPE).communicate()[0])

    duration = float(info_dict['duration'])
    end_timestamp = float(info_dict['end'])
    topic_list = info_dict['topics']

    name_list = []
    framerate_list = []
    num_list = []
    for i in range(len(topic_list)):
        name_list.append(topic_list[i]['topic'])
        framerate_list.append(int(topic_list[i]['messages'] / duration))
        num_list.append(topic_list[i]['messages'])

    return name_list, framerate_list, num_list, end_timestamp, duration

def findTopic(topic_list,target_topic):
    for i in range(len(topic_list)):
        if target_topic == topic_list[i]:
            return True, i
    return False, -1

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

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("You should input one bag filepath at least.")
        exit()
    bag_path = sys.argv[1]
    out_path = bag_path[:bag_path.rfind(".")]+"_extracted.bag"

    info_dict, topic_list, img_topic_list, start_timestamp, end_timestamp, duration, info_strs = getSummaryInfo(bag_path)
    input_indices = raw_input("Input selected topic indices to output(separated by ',' ):\n")

    processed_indices = input_indices.split(',')
    for i in range(len(processed_indices)):
        processed_indices[i]=int(processed_indices[i])
        if processed_indices[i]<1:
            processed_indices[i]=0
        elif processed_indices[i]>len(topic_list):
            processed_indices[i]=len(topic_list)-1
        else:
            processed_indices[i]=processed_indices[i]-1

    processed_indices = list(set(processed_indices))
    processed_indices.sort()
    
    print("Selected topics:")
    for i in range(len(processed_indices)):
        print("\t"+topic_list[processed_indices[i]]['topic'])

    bag_out = rosbag.Bag(out_path,'w')

    with rosbag.Bag(bag_path, 'r') as bag:
        for topic, msg, t in bag.read_messages():
            for i in range(len(processed_indices)):
                if topic == topic_list[processed_indices[i]]['topic']:
                    bag_out.write(topic,msg,t)
                    print(str(t.to_sec()) + "/" + str(end_timestamp))
    
    bag_out.close()
    print("Processed the whole bag, saved at: "+out_path)
