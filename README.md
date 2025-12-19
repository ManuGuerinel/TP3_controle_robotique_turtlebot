# TP3 ROS2 .....


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

`colcon build --packages-select turtlebot_controller
source install/setup.bash
ros2 run turtlebot_controller teleop_keyboard`

Pour controller le robot utiliser les touche : 
`devant: z | derriere: s | tourner gauche: q | tourner droite: d | arrêt d'urgence: a`

## module 2 : Enregistrement Capturer 3+ trajectoires
Pour lancer le noeud d'enregistrement de trajectoire :
`colcon build --packages-select turtlebot_controller
source install/setup.bash
ros2 run turtlebot_controller traj_enregistrement`

## Module 3 : Analyse Visualiser les données
script Python capable de charger les trajectoires enregistrées
`python3 traj_annalyse.py`
