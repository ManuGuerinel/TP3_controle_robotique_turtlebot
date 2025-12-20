"""Noeud pour rejouer les trajectoires enregistrées du TurtleBot3."""

import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np
import time

TOPIC_CMD = '/cmd_vel'
REPERTOIRE_DATA = "trajectoire_save"

### =================[ Fonctions utilitaires ]==================
def charger_csv(chemin_fichier):
    data = np.genfromtxt(chemin_fichier, delimiter=',', skip_header=1)

    time_vel = data[1:, 3]
    vitesses = data[1:, 4:6]

    return time_vel, vitesses

def list_csv_fichiers():
    fichiers = [f for f in os.listdir(REPERTOIRE_DATA) if f.endswith(".csv")]
    return fichiers

### =================[ Noeud ROS2 ]==================
class RejeuTrajectoire(Node):
    def __init__(self, time_vel, vitesses):
        super().__init__('rejeu_trajectoire')

        self.cmd_vel_pub = self.create_publisher(Twist, TOPIC_CMD, 10)

        self.start_time = time.time()
        self.time_vel = time_vel
        self.linear_velocity = vitesses[:, 0]
        self.angular_velocity = vitesses[:, 1]

        self.index_traj = 0
        self.finished = False

        self.get_logger().info("Démarrage du rejeu de la trajectoire.")

        self.timer = self.create_timer(
            self.time_vel[1] - self.time_vel[0],
            self.control_loop
        )

    def control_loop(self):
        if self.index_traj >= len(self.linear_velocity):
            self.stop_robot()
            self.finished = True
            self.timer.cancel()
            self.get_logger().info("Rejeu terminé.")
            return

        msg = Twist()
        msg.linear.x = float(self.linear_velocity[self.index_traj])
        msg.angular.z = float(self.angular_velocity[self.index_traj])
        self.cmd_vel_pub.publish(msg)

        dt = self.time_vel[min(self.index_traj + 1, len(self.time_vel)-1)] - self.time_vel[self.index_traj]

        self.index_traj += 1

        self.timer.cancel()
        self.timer = self.create_timer(dt, self.control_loop)

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Fin de la trajectoire. durée totale: {:.2f} secondes".format(time.time() - self.start_time))

### =================[ Main ]==================

def main():
    while True:
        fichiers = list_csv_fichiers()
        print("\nFichiers disponibles :")
        for i, f in enumerate(fichiers):
            print(f"[{i}] {f}")

        choix = input("\nChoisir l'indice du fichier ou 'q' pour quitter : ")

        if choix.lower() == 'q':
            print("Fin du programme.")
            break

        try:
            fichier = fichiers[int(choix)]
        except:
            print("Choix invalide.")
            continue

        time_vel, vitesses = charger_csv(REPERTOIRE_DATA + "/" + fichier)

        rclpy.init()
        node = RejeuTrajectoire(time_vel, vitesses)

        # Spin jusqu'à la fin du rejeu
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node)

        node.destroy_node()
        rclpy.shutdown()
