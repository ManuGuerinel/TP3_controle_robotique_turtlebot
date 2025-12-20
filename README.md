# TP3 ROS2 : Contrôle Robotique
Auteur : Manu GUERINEL

PROJET : Polytech Sorbonne ROB4 2025

Dans se projet, le but est de:
- Faire de la téléoperation avec son clavier du robot turtlebot
- Enregistrer et sauvegarder des trajectoires
- Annalyser ces Trajectoires
- Rejouer des trajectoires sauvegardé

## installation de turtlebot3 :

`sudo apt update
sudo apt install -y \
ros-jazzy-turtlebot3-gazebo \
ros-jazzy-turtlebot3-teleop`
#### Configuration
`echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc`

Dans un premier terminal :

`ros2 launch turtlebot3_gazebo empty_world.launch.py`

## module 1 : Téléopération Contrôler le robot manuellement via clavier

Dans un deuxième terminal :
(Se trouver dans votre ws)

`colcon build --packages-select turtlebot_controller` (une seule fois suffisante pour la suite)

`source install/setup.bash`

`ros2 run turtlebot_controller teleop_keyboard`

Pour controller le robot utiliser les touche : 
`devant: z | derriere: s | tourner gauche: q | tourner droite: d | arrêt d'urgence: a`

## module 2 : Enregistrement Capturer 3+ trajectoires
Pour lancer le noeud d'enregistrement de trajectoire :

`source install/setup.bash`

`ros2 run turtlebot_controller traj_enregistrement`

Pour lancer l'enregistrement de la trajectoire tapper sur la touche 'r'
```bash
tapper : 'r' (enregitrement) | 's' (stop) | 'save' (sauvegarde) | 'q' (quitter)
```
Une fois lancé pour stopper l'enregistrement tapper sur la touche 's' (attetion a respecter le temps d'enregistrement compris entre 5s et 10s). Puis tapper "save" dans votre terminal pour sauvegarder votre enregistrement de trajectoire. La trajectoire enregistré sera sauvegardé dans un dossier predefini "trajectoire_save" (à ne pas déplacer car utile pour la suite)

## Module 3 : Analyse Visualiser les données
script Python capable de charger les trajectoires enregistrées en choisissant la trajectoire désiré.
(cd ws/src/turtlebot_controller/turtlebot_controller/)

`python3 traj_annalyse.py`

affichage :
- des positions et vitesses angulaires des roues (left et right)
- des min/max des positions
- de la moyenne des vitesses de chacune des roues

```bash
--- Analyse de la trajectoire ---
Durée : 9.49 s
position  left : min = 2305.159, max = 2362.369
position right : min = 2236.548, max = 2279.288
vitesse lineaire : min = 0.110, max = 0.224
vitesse angulaire : min = -0.824, max = 0.093
vitesse moyenne : left = 0.175, right = -0.315
Graphique sauvegardé : ../../../plots/trajectory_2.png
```
Vous trouverez le graphique de votre trajectoire dans le dossier "plots".

Le choix de la trajectoire se fait en choisissant l'indice à gauche du nom des trajectoires disponibles:
```bash
Fichiers disponibles :
[0] trajectory_3.csv
[1] trajectory_2.csv
[2] trajectory_1.csv

Choisir un fichier (index) ou 'q' pour quitter : 
```

## Module 4 : Rejeu: Robot reproduit une trajectoire
node capable de charger une trajectoire sauvegardée et de publier les commandes en respectant le timing original.

Se trouver dans votre ws
`ros2 run turtlebot_controller traj_rejeu `

Le choix de la trajectoire se fait en choisissant l'indice à gauche du nom des trajectoires disponibles:

```bash
Fichiers disponibles :
[0] trajectory_3.csv
[1] trajectory_2.csv
[2] trajectory_1.csv

Choisir l'indice du fichier ou 'q' pour quitter : q
```

Exemple : pour executer la trajectoire "trajectory_3.csv" tapper 0

## A savoir

Tout les chemin et dossier son predefini par l'arboressence, permettent le bon fonctionnement du code si pour chaque commande on se trouve dans le bon dossier.

