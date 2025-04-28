#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Bool, Int8, Empty  # http://wiki.ros.org/std_msgs


class FieldControl:
    def __init__(self, color):
        self.var = None

        rospy.Subscriber("/topicY", Int8, self.callback)

        self.pub = rospy.Publisher("/topicX", Int8, queue_size=1)  

    def callback(self, msg):
        self.var = msg.data


if __name__=="__main__":
    rospy.init_node("field")

    r_blue = FieldControl("blue")
    r_red = FieldControl("red")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()