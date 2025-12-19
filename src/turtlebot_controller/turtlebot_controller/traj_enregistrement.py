import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import threading
import time
import csv
import os


class TrajectoireEnregistrement(Node):

    def __init__(self):
        super().__init__('trajectoire_enregistrement')

        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Etat
        self.enregistrement = False
        self.current_trajectory = []
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
        if not self.enregistrement:
            return
        
        if time.time() - self.start_time > 10:
                self.stop_enregistrement()
                self.get_logger().warn("Stop de l'enregistrement (>10s)\n")
                self.get_logger().info("tapper : 'r' (enregitrement) | 's' (stop) | 'save' (sauvegarde) | 'q' (quitter):\n")

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        positions = list(msg.position)      # repure les position des roues
        vitesse = list(msg.velocity)        # repure les vitesses des roues

        self.current_trajectory.append({
            'time': timestamp,
            'positions': positions,
            'vitesses': vitesse
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

        self.current_trajectory = []
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
        if len(self.current_trajectory) == 0:
            self.get_logger().warn("Aucune donnée à sauvegarder")
            return

        os.makedirs("trajectoire_save", exist_ok=True)

        self.trajectory_num += 1
        filename = f"trajectoire_save/trajectory_{self.trajectory_num}.csv"

        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)

            nb_axes = len(self.current_trajectory[0]['positions'])
            header = ['timestamp'] + ['"wheel_left_joint" : pos', '"wheel_right_joint" : pos', '"wheel_left_joint" : vel', '"wheel_right_joint" : vel']
            writer.writerow(header)

            first_sample = self.current_trajectory[0]
            init_time = first_sample['time']
            for sample in self.current_trajectory:
                writer.writerow(
                    [sample['time']-init_time] + sample['positions'] + sample['vitesses']
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