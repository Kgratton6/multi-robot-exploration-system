import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import os

class IdentifyNode(Node):
    def __init__(self):
        super().__init__('identify_node')
        # Souscription sur le topic /identify (de type Empty, par exemple)
        self.subscription = self.create_subscription(Empty, '/identify', self.identify_callback, 10)
        self.get_logger().info("Identify node started, waiting for /identify messages.")

    def identify_callback(self, msg):
        self.get_logger().info("Identification demandée : lancement du son.")
        # Chemin absolu vers le fichier audio (adapté à votre installation)
        sound_file = '/home/equipe102/Desktop/INF3995-102/robot/common/son_identification.wav'
        if os.path.exists(sound_file):
            # Lancement de la commande pour jouer le son (ici, 'aplay' est utilisé pour Linux)
            subprocess.Popen(['aplay', sound_file])
        else:
            self.get_logger().error(f"Fichier son introuvable: {sound_file}")

def main(args=None):
    rclpy.init(args=args)
    node = IdentifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
