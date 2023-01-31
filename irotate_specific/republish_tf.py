#!/usr/bin/env python
import rospy
import ipdb
import random
from tf2_msgs.msg import TFMessage
import copy

def callback(data, pub):
    data_to_pub = TFMessage()
    data_to_pub.transforms = copy.copy(data.transforms)
    cnt = 0
    for i, d in enumerate(data.transforms):
        if  "x_link" in d.child_frame_id or "y_link" in d.child_frame_id or  "yaw_link" in d.child_frame_id or "base_link" in d.child_frame_id or  "cameraholder_link" in d.child_frame_id:
            data_to_pub.transforms.pop(i - cnt)
            cnt += 1
    pub.publish(data_to_pub)
    return

def listener():
    rospy.init_node('tf_republisher')
    pub = rospy.Publisher("tf", TFMessage, queue_size=1)
    rospy.Subscriber("/tf2", TFMessage, callback, callback_args=(pub))
    rospy.spin()

if __name__ == '__main__':
    listener()
