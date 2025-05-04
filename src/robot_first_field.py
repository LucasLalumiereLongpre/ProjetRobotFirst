#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Bool, Int8, Empty, String  # http://wiki.ros.org/std_msgs
from geometry_msgs.msg import Point


class FieldControl:
    def __init__(self, color):  #Constructeur
        self.color = color  #Couleur du robot, bleu ou rouge
        self.pos = Point    #Position XY du robot
        self.cmd = None     #Commande envoyee par le robot

        rospy.Subscriber(f"/pos_XY_{color}", Point, self.posCallback)   #Subscriber pour la position XY du robot
        rospy.Subscriber(f"/cmd_{color}", String, self.cmdCallback)     #Subscriber pour commandes robot

    def posCallback(self, msg): #Methode pour recevoir la position XY du robot
        self.pos = msg.data

    def cmdCallback(self, msg): #Methode qui recoit une commande du robot
        self.cmd = msg.data
    
    def getPos(self):   #Methode qui retourne la position XY du robot
        return self.pos
    
    def getCmd(self):   #Methode qui retourne la derniere commande recue
        return self.cmd
    
    def isInChargeZone(self):  #Methode qui verifie si le robot est dans la zone de charge
        if (self.color == "blue"):  #Si le robot est bleu
            pass
        elif (self.color == "red"): #Si le robot est rouge
            pass
    
    def isInThrowZone(self):  #Methode qui verifie si le robot est dans une des zones de tir
        if (self.color == "blue"):  #Si le robot est bleu
            pass
        elif (self.color == "red"): #Si le robot est rouge
            pass

    def isInClimbZone(self):  #Methode qui verifie si le robot est dans la zone d'escalade
        if (self.color == "blue"):  #Si le robot est bleu
            pass
        elif (self.color == "red"): #Si le robot est rouge
            pass

if __name__=="__main__":
    rospy.init_node("field")

    r_blue = FieldControl("blue")
    r_red = FieldControl("red")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        r_blue_cmd = r_blue.getCmd()    #On regarde si le robot a envoyer une commande
        r_red_cmd = r_red.getCmd()

        if (r_blue_cmd is not None):    #Si une commande du robot bleu est recue
            if (r_blue_cmd == "z"):     #Si le robot tente de charger un ballon
                if (r_blue.isInChargeZone() is True):   #Si le robot est dans la zone de charge
                    pass
            elif (r_blue_cmd == "x"):   #Si le robot tente de tirer un ballon
                if (r_blue.isInThrowZone() is True):   #Si le robot est dans une zone de tir
                    pass
            elif (r_blue_cmd == "c"):   #Si le robot essaie d'escalader
                if (r_blue.isInClimbZone() is True):   #Si le robot est dans la zone d'escalade
                    pass
        
        if (r_red_cmd is not None):    #Si une commande du robot rouge est recue
            if (r_red_cmd == "z"):
                if (r_red.isInChargeZone() is True):
                    pass
            elif (r_red_cmd == "x"):
                if (r_red.isInThrowZone() is True):
                    pass
            elif (r_red_cmd == "c"):
                if (r_red.isInClimbZone() is True):
                    pass

        rate.sleep()