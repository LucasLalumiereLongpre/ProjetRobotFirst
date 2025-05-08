#!/usr/bin/env python3

import rospy
import sys, select, os
import tty, termios

from enum import Enum
from std_msgs.msg import Bool, Int8, Empty


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class MatchControl:
    def __init__(self):
        self.var = None

        rospy.Subscriber("/topicY", Int8, self.callback)

        self.pub = rospy.Publisher("/topicX", Int8, queue_size=1)  

    def callback(self, msg):
        self.var = msg.data


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("match_ctrl")
    
    match = MatchControl()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        key = getKey()

        rate.sleep()

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)