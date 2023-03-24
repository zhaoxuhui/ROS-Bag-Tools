# coding=utf-8
import sys
import rospy
from nav_msgs.msg import Path

def callback(Path):
    timestamp = Path.header.stamp
    pos = Path.poses.pose.position
    ori = Path.poses.pose.orientation

    pos_x = pos.x
    pos_y = pos.y
    pos_z = pos.z
    ori_x = ori.x
    ori_y = ori.y
    ori_z = ori.z
    ori_w = ori.w

    fout.write(str(timestamp)+" "+str(pos_x)+" "+str(pos_y)+" "+str(pos_z)+" "+str(ori_w)+" "+str(ori_x)+" "+str(ori_y)+" "+str(ori_z)+"\n")
    
    print("Save to file",timestamp)


if __name__ == '__main__':
    topic_name = sys.argv[1]
    out_path = sys.argv[2]
    subscriber_name = "subscriber_" + topic_name.split("/")[-1]

    fout = open(out_path,'a')

    print("waiting for message coming...")

    rospy.init_node(subscriber_name)
    rospy.Subscriber(topic_name, Path, callback)
    rospy.spin()