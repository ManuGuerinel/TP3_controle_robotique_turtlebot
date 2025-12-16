
"""
Contrôleur de téléopération pour TurtleBot3 utilisant les touches clavier
"""
import os, sys, termios
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist

LINEAR_SPEED = 0.2  # m/s
ANGULAR_SPEED = 1.0  # rad/s

TOPIC_CMD = '/cmd_vel'

KEY_MAPPING = {
    'z': (LINEAR_SPEED, 0.0),   # devant
    's': (-LINEAR_SPEED, 0.0),  # derriere
    'q': (0.0, ANGULAR_SPEED),   # gauche
    'd': (0.0, -ANGULAR_SPEED),  # droite
}

# ==================[ Gestion du clavier ]==================
class KeyReader:
    """Gère le terminal en mode raw pour lire une touche sans Enter.

    Met en mode non-canonique/no-echo à l'initialisation (si TTY) et restaure
    les paramètres à la fin via `restore()`.
    """
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.is_tty = sys.stdin.isatty()
        self.old_settings = None
        if self.is_tty:
            try:
                self.old_settings = termios.tcgetattr(self.fd)
                new = termios.tcgetattr(self.fd)
                new[3] = new[3] & ~(termios.ECHO | termios.ICANON)
                termios.tcsetattr(self.fd, termios.TCSANOW, new)
            except Exception:
                # Si on ne peut pas modifier (cas non-tty), on continue sans crash
                self.is_tty = False

    def read_key(self, timeout=0.05):
        """Retourne un caractère ou None si timeout."""
        if not self.is_tty:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                return sys.stdin.read(1)
            return None

        rlist, _, _ = select.select([self.fd], [], [], timeout)
        if rlist:
            b = os.read(self.fd, 1)
            try:
                return b.decode(errors='ignore')
            except Exception:
                return None
        return None

    def restore(self):
        if self.is_tty and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSANOW, self.old_settings)
            except Exception:
                pass
# =========================================================

class TeleopController(Node):
    def __init__(self):
        super().__init__('teleop_controller')
 

        # Topic pour publier les vitesses
        self.cmd_vel_pub = self.create_publisher(Twist, TOPIC_CMD, 10)

        # Lecteur de touches clavier
        self.key_reader = KeyReader()

        # Timer de contrôle à 20 Hz (0.05 secondes)
        self.timer = self.create_timer(0.01, self.control_loop)

        # initialisation des variables de vitesse
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.get_logger().info(
            f'Velocity Controller démarré\n'
            f'  Vitesse max: { LINEAR_SPEED} m/s\n'
            f'  Vitesse angulaire max: { ANGULAR_SPEED} rad/s'
        )
        self.get_logger().info(f"Contrôles :")
        self.get_logger().info(f"devant: z | derriere: s | tourner gauche: q | tourner droite: d | arrêt d'urgence: a")
    

    def control_loop(self):
        """
        Boucle de contrôle principale - appelée toutes les 0.1 secondes
        Implémente la logique d'évitement d'obstacles
        """
        # Créer le message de commande
        cmd = Twist()

        key = self.key_reader.read_key(0.08)
        
        if key is None:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            
        if key == '\x03' or key == '\x1b': # Ctrl+C (ou ESC)
            raise KeyboardInterrupt 

        if key == 'a':
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().warn("Arrêt d'urgence (a) : Vitesse réinitialisée à zéro.")
        
        if key in KEY_MAPPING:
            (self.linear_velocity, self.angular_velocity) = KEY_MAPPING[key]
            
            # Mise à jour de la vitesse du joint
            self.get_logger().info(f"Touche {key} pressée. Consigne: {self.linear_velocity}, {self.angular_velocity}")

        else:
            # Si une touche non mappée est pressée, réinitialiser la vitesse (comportement plus sûr)
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        # Remplir le message de commande avec les vitesses actuelles
        cmd.linear.x = self.linear_velocity
        cmd.angular.z = self.angular_velocity

        self.cmd_vel_pub.publish(cmd)
        


def main(args=None):
    rclpy.init(args=args)
    node = TeleopController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Arrêter le robot proprement
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()