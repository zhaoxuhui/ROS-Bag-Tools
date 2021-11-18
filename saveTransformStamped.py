# coding=utf-8
import sys
import rospy
from geometry_msgs.msg import TransformStamped




def callback(TransformStamped):
    timestamp = TransformStamped.header.stamp
    pos = TransformStamped.transform.translation
    ori = TransformStamped.transform.rotation
    pos_x = pos.x
    pos_y = pos.y
    pos_z = pos.z
    ori_x = ori.x
    ori_y = ori.y
    ori_z = ori.z
    ori_w = ori.w
    fout.write(str(timestamp)+" "+str(pos_x)+" "+str(pos_y)+" "+str(pos_z)+" "+str(ori_x)+" "+str(ori_y)+" "+str(ori_z)+" "+str(ori_w)+"\n")
    print("Save to file",timestamp)


if __name__ == '__main__':
    subscriber_name = sys.argv[1]
    topic_name = sys.argv[2]
    out_path = sys.argv[3]

    fout = open(out_path,'a')

    # 初始化节点
    rospy.init_node(subscriber_name)
    # 开始订阅
    rospy.Subscriber(topic_name, TransformStamped, callback)
    # 循环
    rospy.spin()
