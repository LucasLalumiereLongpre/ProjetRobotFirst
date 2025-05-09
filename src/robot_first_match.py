#!/usr/bin/env python3

import rospy
import sys, select, os
import tty, termios

from enum import Enum
from std_msgs.msg import Bool, Int8, Empty, String

PTSTOTAL_BLUE = 0
PTSGOAL_BLUE = 0
PTSCLIMB_BLUE = 0 

PTSTOTAL_RED = 0
PTSGOAL_RED = 0
PTSCLIMB_RED = 0

# Codes ANSI pour la couleur
BLUE = "\033[94m"
RED = "\033[91m"
RESET = "\033[0m"  # Pour réinitialiser la couleur à la fin

msg_template = """
====================================================
               Equipe bleu  |   Equipe rouge
                            |
Escalade :      {:^5}           |     {:^5}
Panier   :      {:^5}           |     {:^5}
Total    :      {:^5}           |     {:^5}
====================================================
"""

def callback_pts(msg, robot_id):
    global PTSGOAL_BLUE, PTSGOAL_RED
    global PTSCLIMB_BLUE, PTSCLIMB_RED
    global PTSTOTAL_BLUE, PTSTOTAL_RED

    data = msg.data

    if data.startswith("2"):  # a marque 2 pts
        points = 2
        if robot_id % 2 == 0:
            PTSGOAL_BLUE += points
            PTSTOTAL_BLUE += points
        else:
            PTSGOAL_RED += points
            PTSTOTAL_RED += points

    elif data.startswith("5"):  # a marque 5 pts
        points = 5
        if robot_id % 2 == 0:
            PTSCLIMB_BLUE += points
            PTSTOTAL_BLUE += points
        else:
            PTSCLIMB_RED += points
            PTSTOTAL_RED += points


    os.system("clear")
    print(msg_template.format(
        f"{BLUE}{PTSCLIMB_BLUE}{RESET}", f"{RED}{PTSCLIMB_RED}{RESET}",
        f"{BLUE}{PTSGOAL_BLUE}{RESET}", f"{RED}{PTSGOAL_RED}{RESET}",
        f"{BLUE}{PTSTOTAL_BLUE}{RESET}", f"{RED}{PTSTOTAL_RED}{RESET}"
    ))

if __name__=="__main__":
    #settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("match_ctrl")

    # Liste des robots (0 à 5, pour un maximum de 6 robots)
    robots_Max = 6

    # Souscrire à chaque robot via une boucle
    for i in range(robots_Max):
        rospy.Subscriber(f"/robot_{i}/pts", String, lambda msg, robot_id=i: callback_pts(msg, robot_id))

    
    ##match = MatchControl()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():

        rate.sleep()

    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)