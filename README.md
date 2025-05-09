# ProjetRobotFirst
src contient 3 fichiers:
	-robot_first_bot.py
	-robot_first_field.py
	-robot_first_match.py
stageWorlds contient les cartes

## Prerequis
Installer la librairie progressbar pour l'affichage.
```sh
pip install progressbar2
```
Installer ROS noetic
https://wiki.ros.org/noetic/Installation/Ubuntu

Installer Terminator (facultatif)
sudo apt-get install terminator

## Installation 
1. Copiez le dossier ProjetRobotFirst vers catkin_ws/src/
2. Ouvrez un terminal
3. A partir du repertoire catkin_ws, entrez la commande 'catkin_make'
4. Si ce n'est pas déjà fait, entrez la commande 'source devel/setup.bash'

## Utilisation
1. Ouvrez un terminal (terminator fort recommandé)
2. Entrez la commande 'roscore'
3. Allez dans le répertoire du projet avec la commande 'cd catkin_ws/src/ProjetRobotFirst'
4. Pour ouvrir la carte, entrez 'rosrun stage_ros stageros stageWorlds/first_double.world'
5. Ouvrez 3 autres terminaux et entrez les commandes suivantes:
	a. 'rosrun ProjetRobotFirst robot_first_bot.py' pour contrôler les robots
	b. 'rosrun ProjetRobotFirst robot_first_field.py' pour gestion du terrain
	c. 'rosrun ProjetRobotFirst robot_first_match.py'	pour gestion des points et tableau des scores

Touches:
Bleu:
	W: Avance le robot
	A/D: Tourne le robot a gauche/droite.
	S: Recule le robot
	C: Commande pour action
Rouge: 
	I: Avance le robot
	J/L: Tourne le robot a gauche/droite.
	K: Recule le robot
	B: Commande pour action
	
CTRL+C pour fermer un programme

Règles de jeu:
Le jeu est basé sur la compétition "First Robotics", édition 2022.
Voici une vidéo démontrant le déroulement d'une partie: https://www.youtube.com/watch?v=b56UKoQq8fg