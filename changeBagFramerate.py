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

if __name__ == '__main__':
    bag_path = ""
    out_path = ""
    topic_names = []
    target_framerates = []

    diff_th = 5 # 迭代终止阈值，越小抽取的帧索引就越精确
    
    # 对于输入参数的判断
    if len(sys.argv) % 2 != 0:
        for i in range(2,len(sys.argv)-1,2):
            topic_names.append(sys.argv[i])
            target_framerates.append(float(sys.argv[i+1]))

        bag_path = sys.argv[1]
        out_path = sys.argv[-1]

        name_list, framerate_list,num_list,end_timestamp, duration = getFramerateInfo(bag_path)

        bag_out = rosbag.Bag(out_path,'w')

        for i in range(len(topic_names)):
            cur_name = topic_names[i]
            flag,index = findTopic(name_list,cur_name)
            if flag:
                cur_framerate = framerate_list[index]
                cur_msg_num = num_list[index]

                # 根据帧率计算步长
                if target_framerates[i] <=0 or target_framerates[i] > cur_framerate:
                    target_framerate = cur_framerate
                else:
                    target_framerate = target_framerates[i]
                
                target_msg_num = int(round((target_framerate/cur_framerate) * cur_msg_num,0))

                print("Processing topic: "+cur_name)
                print("Current rate: "+ str(cur_framerate)+", total number: "+str(cur_msg_num)+" ===> "+"Target rate: "+str(target_framerate)+", total number: "+str(target_msg_num))

                diff_num = target_msg_num
                prev_num = diff_num
                time_counter = 0

                # 不断迭代，直到满足要求
                while diff_num > diff_th:
                    time_counter += 1

                    if 1.0*cur_framerate/target_framerate <= 1:
                        step = 1
                    else:
                        step = int(1.0*cur_msg_num/diff_num)+1
                    actual_num = int(1.0*cur_msg_num/step)

                    # 以这种方式打开Bag，如果打印对象，就对得到它的汇总信息，可以不用自己写
                    bag_in = rosbag.Bag(bag_path)

                    # read_messages()得到的是一个生成器对象
                    bag_data = bag_in.read_messages()

                    msg_counter = 0
                    # 需要注意，这种方式进行遍历，并非是某一个指定的topic个数，而是全部所有topic里面msg的个数
                    for topic, msg, t in bag_data:
                        # 通过topic名称，这样来判断，得到的才是某一个topic的msg个数
                        if topic == cur_name:
                            if msg_counter % step == 0:
                                print("Iteration "+str(time_counter)+", converted ... "+ str(t.to_sec())+"/"+str(end_timestamp))
                                bag_out.write(topic,msg,t)
                            msg_counter += 1
                    bag_in.close()

                    prev_num = diff_num
                    diff_num = abs(prev_num - actual_num)
                
                print("Processed topic:"+cur_name+"\n")
        bag_out.close()
        print("Processed the whole bag, saved at: "+out_path)
    else:
        print("Error parameters. Input Format:\nInput_bag_path topic_name1 target_rate1 [topic_name2 target_rate2 ... ] Output_bag_path")