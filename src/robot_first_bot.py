#!/usr/bin/env python3

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
        self.distance_at_90=0

        self.pub_vel = rospy.Publisher(f"/robot_{id}/cmd_vel", Twist, queue_size=1)
        self.pub_action = rospy.Publisher(f"/robot_{id}/cmd_action", String, queue_size=1)


    def callback(self, msg):
        self.var = msg.data

    def setSpeed(self,control_linear_vel):
        self.control_linear_vel = control_linear_vel
        twist = Twist()  # Crée un message Twist pour contrôler la vitesse du robot
        twist.angular.z = self.getRotation()
        twist.linear.x = self.getSpeed()

    def setRotation(self,control_angular_vel):
        self.control_angular_vel=control_angular_vel
        twist = Twist()  # Crée un message Twist pour contrôler la vitesse du robot
        twist.angular.z = self.getRotation()
        twist.linear.x = self.getSpeed()

    def getSpeed(self):
        return self.control_linear_vel
            
    def getRotation(self):
        return self.control_angular_vel
    
    def setAction(self,action):
        self.action = action
        rospy.loginfo(f"Publishing to {self.pub_action.name}: send action")
        self.pub_action.publish(self.action)

    def shotdown_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_vel.publish(twist)
    
    def move(self):
        if self.distance_at_90 <= 0.5: # protection si on est proche d'un mur
            if self.control_linear_vel == 0.5 :
                self.control_linear_vel=0
        twist = Twist()  # Crée un message Twist pour contrôler la vitesse du robot
        twist.angular.z = self.getRotation()
        twist.linear.x = self.getSpeed()
        #rospy.loginfo(f"Publishing to {self.pub_vel.name}   angular: {twist.angular.z}    lineaire:  {twist.linear.x}")
        self.pub_vel.publish(twist)

        # Callback du lidar pour récupérer la distance à 90 degrés
    def laser_callback(self,msg):
        angle_min = msg.angle_min  # Angle de départ du scan
        angle_increment = msg.angle_increment  # Pas entre chaque mesure
        index_90_deg = int((0.0 - angle_min) / angle_increment)  # Calcul de l'index correspondant à 90° (i.e., 0 rad dans ce contexte)

        self.distance_at_90 = msg.ranges[index_90_deg]  # Stocke la distance lue à 90°
        
        if self.distance_at_90 <= 0.5:
            rospy.logwarn("Index pour 90° hors limites.")
            self.control_linear_vel = 0.0
            twist = Twist()
            twist.linear.x = self.getSpeed()
            twist.angular.z = self.getRotation()
            self.pub_vel.publish(twist)


    rospy.on_shutdown(shotdown_robot)


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("robot_ctrl")

    r_blue = RobotControl(0)
    r_red = RobotControl(1)

    rospy.Subscriber('/robot_0/base_scan', LaserScan, r_blue.laser_callback)
    rospy.Subscriber('/robot_1/base_scan', LaserScan, r_red.laser_callback)
    
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
                    r_blue.setRotation(MAX_ANG_VEL)  # Tourne gauche
                elif r_blue.getRotation() == MIN_ANG_VEL:
                    r_blue.setRotation(0)            # Arrête
            elif key == 'd':
                # Tourne droite ou arrête la rotation
                if r_blue.getRotation() == 0:
                    r_blue.setRotation(MIN_ANG_VEL)  # Tourne droite
                elif r_blue.getRotation() == MAX_ANG_VEL:
                    r_blue.setRotation(0)            # Arrête

            elif key == 'c':
                r_blue.setAction('c')

    #################################### Robot Rouge ####################################
            if key == 'i':
                # Avance ou arrête
                if r_red.getSpeed() == 0:
                    r_red.setSpeed(MAX_LIN_VEL)  # Avance
                elif r_red.getSpeed() == MIN_LIN_VEL:
                    r_red.setSpeed(0)            # Arrête
            elif key == 'k':
                #Reculer ou arreter
                if r_red.getSpeed()== 0:
                    r_red.setSpeed(MIN_LIN_VEL)  # Limite la vitesse si elle dépasse le maximum
                elif r_red.getSpeed() == MAX_LIN_VEL:
                    r_red.setSpeed(0)
            elif key == 'j':
                #Tourne gauche ou arreter rotation
                if r_red.getRotation() == 0:
                    r_red.setRotation(MAX_ANG_VEL)  # Tourne gauche
                elif r_red.getRotation() == MIN_ANG_VEL:
                    r_red.setRotation(0)            # Arrête
            elif key == 'l':
                # Tourne droite ou arrête la rotation
                if r_red.getRotation() == 0:
                    r_red.setRotation(MIN_ANG_VEL)  # Tourne droite
                elif r_red.getRotation() == MAX_ANG_VEL:
                    r_red.setRotation(0)            # Arrête

            elif key == 'b':
                r_red.setAction('b')
    #####################################################################################

            r_blue.move()
            r_red.move()

            rate.sleep()

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    except KeyboardInterrupt:
        rospy.loginfo("Interruption clavier (Ctrl+C) détectée.")
        
    finally:
        
        r_blue.shotdown_robot()  # Sécurité si on n'est pas passé par rospy.on_shutdown
        r_red.shotdown_robot()  # Sécurité si on n'est pas passé par rospy.on_shutdown
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)