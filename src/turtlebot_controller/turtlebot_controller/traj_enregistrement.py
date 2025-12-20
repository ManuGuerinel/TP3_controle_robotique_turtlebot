"""Noeud ROS2 pour enregistrer les trajectoires du TurtleBot3.
-positions roues droite/gauche
-vitesses lineaires
-vitesses angulaires"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np

import threading
import time
import csv
import os


class TrajectoireEnregistrement(Node):

    def __init__(self):
        super().__init__('trajectoire_enregistrement')

        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.sub_velocities = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Etat
        self.enregistrement = False
        self.trajectoires_pos = []
        self.trajectoires_vel = []
        self.start_time = time.time()
        self.trajectory_num = 0

        # Thread pour les entrée clavier
        self.command_thread = threading.Thread(
            target=self.command_listener,
            daemon=True
        )
        self.command_thread.start()

        self.get_logger().info("tapper : 'r' (enregitrement) | 's' (stop) | 'save' (sauvegarde) | 'q' (quitter)\n")

    def joint_state_callback(self, msg):
        # Récupération des positions des roues
        if not self.enregistrement:
            return
        
        if time.time() - self.start_time > 10:
                self.stop_enregistrement()
                self.get_logger().warn("Stop de l'enregistrement (>10s)\n")
                self.get_logger().info("tapper : 'r' (enregitrement) | 's' (stop) | 'save' (sauvegarde) | 'q' (quitter):\n")

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        positions = list(msg.position)      # repure les position des roues

        self.trajectoires_pos.append({
            'time': timestamp,
            'positions': positions,
        })

    def odom_callback(self, msg):
        # Récupération des vitesses
        if not self.enregistrement:
            return
        
        vel_lin = msg.twist.twist.linear.x      # vitesse lineaire en x
        vel_ang = msg.twist.twist.angular.z     # vitesse angulaire en z

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        self.trajectoires_vel.append({
            'time': timestamp,
            'vel_lin': vel_lin,
            'vel_ang': vel_ang
        })

    def command_listener(self):
        while rclpy.ok():
            cmd = input("tapper : 'r' (enregitrement) | 's' (stop) | 'save' (sauvegarde) | 'q' (quitter):\n")

            if cmd == 'r':
                self.start_enregistrement()

            elif cmd == 's':
                self.stop_enregistrement()

            elif cmd == 'save':
                self.save_trajectory()

            elif cmd == 'q':
                self.stop_enregistrement()
                self.get_logger().info("Quitter le programme")
                rclpy.shutdown()
                break

    def start_enregistrement(self):
        if self.enregistrement:
            self.get_logger().warn("Enregistrement déjà en cours")
            return

        self.trajectoires_pos = []
        self.trajectoires_vel = []
        self.start_time = time.time()
        self.enregistrement = True
        self.get_logger().info("Début enregistrement")

    def stop_enregistrement(self):
        if not self.enregistrement:
            return

        duree = time.time() - self.start_time
        self.enregistrement = False

        if 5 <= duree:
            self.get_logger().info(f"Fin enregistrement ({duree:.2f} s)")
        else:
            self.get_logger().warn(f"Durée trop courte : {duree:.2f} s")

    def save_trajectory(self):
        if len(self.trajectoires_pos) == 0:
            self.get_logger().warn("Aucune donnée à sauvegarder")
            return

        os.makedirs("trajectoire_save", exist_ok=True)

        self.trajectory_num += 1
        filename = f"trajectoire_save/trajectory_{self.trajectory_num}.csv"

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)

            nb_axes = len(self.trajectoires_pos[0]['positions'])
            header = ['timestamp_pos','Position wheel_left_joint', 'Position wheel_right_joint', 'timestamp_vel', 'Vitesse Linéaire', 'Vitesse angulaire']
            writer.writerow(header)

            first_sample_pos = self.trajectoires_pos[0]
            first_sample_vel = self.trajectoires_vel[0]
            init_time_pos = first_sample_pos['time']
            init_time_vel = first_sample_vel['time']
            nb_sample_pos = len(self.trajectoires_pos)
            nb_sample_vel = len(self.trajectoires_vel)
            max_nb_sample = np.max([nb_sample_pos,nb_sample_vel])

            # la premiere ligne de data est le nombre d'echantillons par callback
            writer.writerow([nb_sample_pos,nb_sample_pos,nb_sample_pos,nb_sample_vel,nb_sample_vel,nb_sample_vel] )
            
            for i in range(max_nb_sample):
                if(i > nb_sample_vel):
                    sample = self.trajectoires_pos[i]
                    pos_left = sample_pos['positions'][0]
                    pos_right = sample_pos['positions'][1]
                    writer.writerow(
                        [sample['time']-init_time_pos , pos_left, pos_right , "_","_","_"]
                    )
                elif(i > nb_sample_pos):
                    sample = self.trajectoires_vel[i]
                    writer.writerow(
                        ["_","_","_" , sample['time']-init_time_vel , sample['vel_lin'] , sample['vel_ang']]
                    )
                else:
                    sample_pos = self.trajectoires_pos[i]
                    sample_vel = self.trajectoires_vel[i]
                    pos_left = sample_pos['positions'][0]
                    pos_right = sample_pos['positions'][1]
                    writer.writerow(
                        [sample_pos['time']-init_time_pos , pos_left, pos_right , sample_vel['time']-init_time_vel , sample_vel['vel_lin'] , sample_vel['vel_ang']]
                    )

        self.get_logger().info(f"{filename} à bien été sauvegardé.")

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoireEnregistrement()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()