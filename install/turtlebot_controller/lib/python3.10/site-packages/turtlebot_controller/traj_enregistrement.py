import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

import threading
import time
import json
import csv


class TrajectoireEnregistrement(Node):

    def __init__(self):
        super().__init__('trajectoire_enregistrement')

        # Subscriber
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # Etat interne
        self.enregistrement = False
        self.current_trajectory = []
        self.trajectory_num = 0
        self.start_time = time.time()

        self.get_logger().info("Enregistrement de trajectoire prêt (r (lancer)/ s (stop)/ save)")

    def joint_state_callback(self, msg):
        if not self.enregistrement:
            return
        
        
        elif time.time() - self.start_time > 10 :
            self.get_logger().warn("Enregistrement trop long (>10s), arrêt automatique.")
            self.stop_enregistrement()
            return

        # - récupérer timestamp
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # - récupérer positions
        positions = list(msg.position)
        # - ajouter à la trajectoire courante
        self.current_trajectory.append({'time': timestamp, 'positions': positions})
        # - stocker dans self.current_trajectory
        pass

    def start_enregistrement(self):
        self.get_logger().info("Debut de l'enregistrement...")
        self.current_trajectory = []
        self.enregistrement = True
        self.start_time = time.time()

    def stop_enregistrement(self):
        duree = time.time() - self.start_time
        if 5 <= duree:  # le maximum 10s est gerer dans joint_state_callback
            self.get_logger().info(f"RFin de l'enregistrement, Duree: {duree:.2f} seconds")
            self.enregistrement = False
        pass

    def save_trajectory(self):
        if len(self.current_trajectory) == 0:
            self.get_logger().warn("Aucune trajectoire à sauvegarder.")
            return

        self.trajectory_num += 1
        filename = f"trajectoire_save/trajectory_{self.trajectory_num}.csv"

        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)

            # Header
            nb_axe = len(self.current_trajectory[0]['positions'])
            header = ['timestamp'] + [f'axe_{i+1}' for i in range(nb_axe)]
            writer.writerow(header)

            # Data
            for sample in self.current_trajectory:
                row = [sample['time']] + sample['positions']
                writer.writerow(row)

        self.get_logger().info(f"Trajectoire sauvegardée dans {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoireEnregistrement()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        cmd = input("Commande (r / s / save): ")

        if cmd == 'r':
            node.start_enregistrement()

        elif cmd == 's':
            node.stop_enregistrement()

        elif cmd == 'save':
            node.save_trajectory()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()