#!/usr/bin/env python

import rospy
import sys, select, os
import tty, termios

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8, Empty
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class RobotControl:
    def __init__(self, color):
        self.var = None

        rospy.Subscriber("/topicY", Empty, self.callback)
        
        self.pub = rospy.Publisher("/topicX", Int8, queue_size=1)  

    def callback(self, msg):
        self.var = msg.data


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("robot_ctrl")

    r_blue = RobotControl("blue")
    r_red = RobotControl("red")

    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        key = getKey()
        
        # Blue bot cmd
    

        # Red bot cmd
        

        rate.sleep()


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)