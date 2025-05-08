#!/usr/bin/env python3

import rospy
import random
import math
import time
from std_msgs.msg import Bool, Int8, Empty, String  # http://wiki.ros.org/std_msgs
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import progressbar
import threading

class FieldControl:
    def __init__(self, id, color):  #Constructeur
        self.id = id        #id du robot de 0 a 5
        self.color = color  #Couleur du robot, bleu ou rouge
        self.pos = None     #Position XY du robot
        self.cmd = None     #Commande envoyee par le robot
        self.nbBalls = 0    #Nombre de balles detenu par le robot
        self.level = 0      #Niveau atteint

        rospy.Subscriber(f"/robot_{id}/cmd_action", String, self.cmdCallback)     #Subscriber pour commandes robot
        rospy.Subscriber(f"/robot_{id}/odom", Odometry, self.posCallback)     #Subscriber pour commandes robot
        self.pubPts = rospy.Publisher(f"/robot_{id}/pts", String, queue_size=1)

    def cmdCallback(self, msg): #Methode qui recoit une commande du robot
        self.cmd = msg.data

    def posCallback(self, msg): #Methode qui recoit la position XY du robot
        self.pos = msg.pose.pose
        #print(f"Robot {self.color} x={self.pos.position.x}, y={self.pos.position.y}")    #Affiche les coordonnes pour debogage
    
    def getPos(self):   #Methode qui retourne la position XY du robot
        return self.pos
    
    def getCmd(self):   #Methode qui retourne la derniere commande recue
        return self.cmd
    
    def clearCmd(self):
        self.cmd = None
    
    def isInChargeZone(self):  #Methode qui verifie si le robot est dans la zone de charge
        if (self.pos is not None):      #Attend que le robot commence a publier sa position
            if (self.color == "blue"):  #Si le robot est bleu
                #Points du rectangle (x1, x2, y1, y2)
                x_min = -8.0
                x_max = -6.0
                y_min = -4.0
                y_max = -2.0

                #Verifie si X et Y sont dans les limites
                inZone = (x_min <= self.pos.position.x <= x_max) and (y_min <= self.pos.position.y <= y_max)
                return inZone
            elif (self.color == "red"): #Si le robot est rouge
                x_min = 8.0
                x_max = 6.0
                y_min = 4.0
                y_max = 2.0

                inZone = (x_min >= self.pos.position.x >= x_max) and (y_min >= self.pos.position.y >= y_max)
                return inZone
    
    def isInThrowZone(self):  #Methode qui verifie si le robot est dans une des zones de tir
        slope = -2
        if (self.pos is not None):      #Attend que le robot commence a publier sa position
            if (self.color == "blue"):  #Si le robot est bleu
                r = math.sqrt(self.pos.position.x**2 + self.pos.position.y**2)
                side = slope * self.pos.position.x
                if (side > 0 and r <= 4):
                    return True
                else:
                    return False
            elif (self.color == "red"): #Si le robot est rouge
                r = math.sqrt(self.pos.position.x**2 + self.pos.position.y**2)
                side = slope * self.pos.position.x
                if (side < 0 and r >= -4):
                    return True
                else:
                    return False

    def isInClimbZone(self):  #Methode qui verifie si le robot est dans la zone d'escalade
        if (self.pos is not None):      #Attend que le robot commence a publier sa position
            if (self.color == "blue"):  #Si le robot est bleu
                #Points du rectangle (x1, x2, y1, y2)
                x_min = -8.0
                x_max = -5.0
                y_min = 4.0
                y_max = 1.0

                #Verifie si X et Y sont dans les limites
                inZone = (x_min <= self.pos.position.x <= x_max) and (y_min >= self.pos.position.y >= y_max)
                return inZone
            elif (self.color == "red"): #Si le robot est rouge
                x_min = 8.0
                x_max = 5.0
                y_min = -4.0
                y_max = -1.0

                inZone = (x_min >= self.pos.position.x >= x_max) and (y_min <= self.pos.position.y <= y_max)
                return inZone
    
    def getNbBalls(self):
        return self.nbBalls

    def takeBall(self):
        if (self.nbBalls < 2):
            self.nbBalls += 1
            print(f"{self.color} +1 ball")
        else:
            print(f"{self.color} cannot take ball")
    
    def throwBall(self):
        if (self.nbBalls > 0):
            self.nbBalls -= 1
            if(random.random() < 0.9):
                print(f"{self.color} +2 points")
                self.pubPts.publish("2g")   
            else:
                print(f"{self.color} missed!")
        else:
            print(f"{self.color} cannot throw ball")

    def climb(self):
        if (self.level < 3):
            duration = 3  #En secondes
            steps = 100    #De 0 a 100
            interval = duration / steps  #temps entre increment

            widgets = [' [',
                progressbar.Timer(format= 'Climbing: %(elapsed)s'), #Barre de progres
                '] ',
                progressbar.Bar('#' ),' (', 
                progressbar.ETA(), ') ',
            ]
        bar = progressbar.ProgressBar(max_value=steps, widgets=widgets).start() #Demarre la barre de progres
        for i in range(steps + 1):  # 0 to 100 inclusive
            self.progress = i
            time.sleep(interval)
            bar.update(i)
        print('\n')
        if(random.random() < 0.6):
            self.pubPts.publish("5c")
            self.level += 1   
            print(f"{self.color} +5 points, level {self.level}")
        else:
            print(f"{self.color} failed to climb!")

if __name__=="__main__":
    rospy.init_node("field")
    threads = []

    r_blue = FieldControl(0, "blue")
    r_red = FieldControl(1, "red")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        r_blue_cmd = r_blue.getCmd()    #On regarde si le robot a envoyer une commande
        r_red_cmd = r_red.getCmd()

        if (r_blue_cmd is not None):    #Si une commande du robot bleu est recue
            if (r_blue.isInChargeZone() is True):   #Si le robot est dans la zone de charge
                r_blue.takeBall()
            if (r_blue.isInThrowZone() is True):   #Si le robot est dans une zone de tir
                r_blue.throwBall()
            if (r_blue.isInClimbZone() is True):   #Si le robot est dans la zone d'escalade
                blueClimbThread = threading.Thread(target=r_blue.climb)
                threads.append(blueClimbThread) #Ajoute le thread a la liste de threads
                blueClimbThread.start()   #Part un thread pour atterir un avion
            r_blue.clearCmd()
        
        if (r_red_cmd is not None):    #Si une commande du robot rouge est recue
            if (r_red.isInChargeZone() is True):
                r_red.takeBall()
            if (r_red.isInThrowZone() is True):
                r_red.throwBall()
            if (r_red.isInClimbZone() is True):
                redClimbThread = threading.Thread(target=r_red.climb)
                threads.append(redClimbThread) #Ajoute le thread a la liste de threads
                redClimbThread.start()   #Part un thread pour atterir un avion
            r_red.clearCmd()

        rate.sleep()
    for thread in threads:
        thread.join()
