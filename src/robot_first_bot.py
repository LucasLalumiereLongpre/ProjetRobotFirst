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
import threading

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
        self.key = ""

        self.pub_vel = rospy.Publisher(f"/robot_{id}/cmd_vel", Twist, queue_size=1)
        self.pub_action = rospy.Publisher(f"/robot_{id}/cmd_action", String, queue_size=1)
        self.pub_ = rospy.Publisher(f"/robot_{id}/cmd_action", String, queue_size=1)

        self.sub = rospy.Subscriber(f"/robot_{id}/base_scan", LaserScan, self.laser_callback)
        ##self.sub = rospy.Subscriber(f"/robot_{id}/pts", String, self.callback_move)

    """
    def callback_move(self, msg):
        data = msg.data
        if data.startswith("s"):  # en train de grimper
            

        elif data.startswith("t" or "c"):  # en train de lancer
    """


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

    def input(self):

        while True:
            if self.name == 0:
                if self.key == 'w':
                    # Avance ou arrête
                    if self.getSpeed() == 0:
                        self.setSpeed(MAX_LIN_VEL)  # Avance
                    elif self.getSpeed() == MIN_LIN_VEL:
                        self.setSpeed(0)            # Arrête
                elif self.key == 's':
                    #Reculer ou arreter
                    if self.getSpeed()== 0:
                        self.setSpeed(MIN_LIN_VEL)  # Limite la vitesse si elle dépasse le maximum
                    elif self.getSpeed() == MAX_LIN_VEL:
                        self.setSpeed(0)
                elif self.key == 'a':
                    #Tourne gauche ou arreter rotation
                    if self.getRotation() == 0:
                        self.setRotation(MAX_ANG_VEL)  # Tourne gauche
                    elif self.getRotation() == MIN_ANG_VEL:
                        self.setRotation(0)            # Arrête
                elif self.key == 'd':
                    # Tourne droite ou arrête la rotation
                    if self.getRotation() == 0:
                        self.setRotation(MIN_ANG_VEL)  # Tourne droite
                    elif self.getRotation() == MAX_ANG_VEL:
                        self.setRotation(0)            # Arrête
                elif self.key == 'c':
                    self.setAction('c')


            if self.name == 1:
                if self.key == 'i':
                    # Avance ou arrête
                    if self.getSpeed() == 0:
                        self.setSpeed(MAX_LIN_VEL)  # Avance
                    elif self.getSpeed() == MIN_LIN_VEL:
                        self.setSpeed(0)            # Arrête
                elif self.key == 'k':
                    #Reculer ou arreter
                    if self.getSpeed()== 0:
                        self.setSpeed(MIN_LIN_VEL)  # Limite la vitesse si elle dépasse le maximum
                    elif self.getSpeed() == MAX_LIN_VEL:
                        self.setSpeed(0)
                elif self.key == 'j':
                    #Tourne gauche ou arreter rotation
                    if self.getRotation() == 0:
                        self.setRotation(MAX_ANG_VEL)  # Tourne gauche
                    elif self.getRotation() == MIN_ANG_VEL:
                        self.setRotation(0)            # Arrête
                elif key == 'l':
                    # Tourne droite ou arrête la rotation
                    if self.getRotation() == 0:
                        self.setRotation(MIN_ANG_VEL)  # Tourne droite
                    elif self.getRotation() == MAX_ANG_VEL:
                        self.setRotation(0)            # Arrête
                elif self.key == 'b':
                    self.setAction('b')
    def setkey(self,key):
        self.key=key


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("robot_ctrl")

    r_blue = RobotControl(0)
    r_red = RobotControl(1)

    rate = rospy.Rate(10)

    key=""
    thread_blue0 = threading.Thread(target=r_blue.input)
    thread_blue0.start()
    thread_red1 = threading.Thread(target=r_red.input)
    thread_red1.start()

    try: 
        while not rospy.is_shutdown():
            key=getKey()
            r_blue.setkey(key)
            r_red.setkey(key)


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