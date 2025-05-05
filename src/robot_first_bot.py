#!/usr/bin/env python

import rospy
import sys, select, os
import tty, termios


from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int8, Empty, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

from multiprocessing import Queue

# Constantes de vitesse
MAX_LIN_VEL = 0.5
MIN_LIN_VEL = -0.5
MAX_ANG_VEL = 0.25
MIN_ANG_VEL = -0.25
NBR_ROBOTS  = 2

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
    def __init__(self, id):
        self.control_linear_vel = 0
        self.control_angular_vel=0
        self.name = id
        self.action = ""

        self.pub_vel = rospy.Publisher(f"/robot_{id}/cmd_vel", Twist, queue_size=1)
        self.pub_action = rospy.Publisher(f"/robot_{id}/cmd_action", String, queue_size=1)


    def callback(self, msg):
        self.var = msg.data

    def setSpeed(self,control_linear_vel):
        self.control_linear_vel = control_linear_vel
        twist = Twist()  # Crée un message Twist pour contrôler la vitesse du robot
        twist.linear.x = self.getSpeed()
        self.pub_vel.publish(twist)

    def setRotation(self,control_angular_vel):
        self.control_angular_vel=control_angular_vel
        twist = Twist()  # Crée un message Twist pour contrôler la vitesse du robot
        twist.angular.x = self.getRotatiomn()
        self.pub_vel.publish(twist)

    def getSpeed(self):
        return self.control_linear_vel
            
    def getRotation(self):
        return self.control_angular_vel
    
    def setAction(self,action):
        self.action = action
        self.pub_action.publish(self.action)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("robot_ctrl")
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    r_blue = RobotControl(0)
    ##r_red = RobotControl("robot_1","red")
    
    rate = rospy.Rate(10)
    try: 
        while not rospy.is_shutdown():

            key = getKey()
    ##################################### Robot Bleu #################################
            if key == 'w':
                # Avance ou arrête
                if r_blue.getSpeed() == 0:
                    r_blue.setSpeed(MAX_LIN_VEL)  # Avance
                elif r_blue.getSpeed() == MIN_LIN_VEL:
                    r_blue.setSpeed(0)            # Arrête
            elif key == 's':
                #Reculer ou arreter
                if r_blue.getSpeed()== 0:
                    r_blue.setSpeed(MIN_LIN_VEL)  # Limite la vitesse si elle dépasse le maximum
                elif r_blue.getSpeed() == MAX_LIN_VEL:
                    r_blue.setSpeed(0)
            elif key == 'a':
                #Tourne gauche ou arreter rotation
                if r_blue.getRotation() == 0:
                    r_blue.setgetRotation(MAX_ANG_VEL)  # Tourne gauche
                elif r_blue.getRotation() == MIN_ANG_VEL:
                    r_blue.setRotation(0)            # Arrête
            elif key == 'd':
                # Tourne droite ou arrête la rotation
                if r_blue.getRotation() == 0:
                    r_blue.setgetRotation(MIN_ANG_VEL)  # Tourne droite
                elif r_blue.getRotation() == MAX_ANG_VEL:
                    r_blue.setRotation(0)            # Arrête
            elif key == 'z':
                r_blue.setAction('z')
            elif key == 'x':
                r_blue.setAction('x')
            elif key == 'c':
                r_blue.setAction('c')

    #################################### Robot Rouge ####################################
            """elif key == 'i':
                # Avance ou arrête
                if r_red.getSpeed() == 0:
                    r_red.setSpeed(MAX_LIN_VEL, r_red.getRotation())  # Avance
                elif r_red.getSpeed() == MIN_LIN_VEL:
                    r_red.setSpeed(0, r_red.getRotation())            # Arrête

            elif key == 'k':
                # Reculer ou arrêter
                if r_red.getSpeed() == 0:
                    r_red.setSpeed(MIN_LIN_VEL, r_red.getRotation())  # Recul
                elif r_red.getSpeed() == MAX_LIN_VEL:
                    r_red.setSpeed(0, r_red.getRotation())            # Arrête

            elif key == 'j':
                # Tourne à gauche ou arrête rotation
                if r_red.getRotation() == 0:
                    r_red.setSpeed(r_red.getSpeed(), MAX_ANG_VEL)     # Tourne gauche
                elif r_red.getRotation() == MIN_ANG_VEL:
                    r_red.setSpeed(r_red.getSpeed(), 0)               # Arrête

            elif key == 'l':
                # Tourne à droite ou arrête rotation
                if r_red.getRotation() == 0:
                    r_red.setSpeed(r_red.getSpeed(), MIN_ANG_VEL)     # Tourne droite
                elif r_red.getRotation() == MAX_ANG_VEL:
                r_red.setSpeed(r_red.getSpeed(), 0)               # Arrête


            elif key == 'b':
                r_red.setAction('b')
            elif key == 'n':
                r_red.setAction('n')
            elif key == 'm':
                r_red.setAction('m')


            """
        
            # Blue bot cmd
        

            # Red bot cmd
            

            rate.sleep()

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


    finally:
        # Arrêt du robot lorsque l'on quitte
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)