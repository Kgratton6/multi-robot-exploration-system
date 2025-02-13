import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# EXEMPLE OF MESSAGE
# {
#   "action": "move",
#   "speed": 0.5,
#   "duration": 2.0
# }

# EXEMPLE OF INFORMATION PATH TO MOVE
#   SERVER --> publish "/messages" --> CommunicationController
#   CommunicationController --> publish "/movement" --> MoveController
#   MoveController --> publish "/cmd_vel" --> ROBOT

# export ROS_DOMAIN_ID=102 on ROS
# ROS_DOMAIN_ID=102 in .env of server

# rotation problem = normaly should rotate 360 degre, but does more like a 40 degre
# ros2 topic pub /messages std_msgs/msg/String '{data: "{\"action\": \"move\", \"speed\": -0.2, \"duration\": 3.14}"}' -1
# identifier : ros2 topic pub /identify std_msgs/msg/Empty "{}" -1
# lancer la mission : ros2 topic pub /messages std_msgs/msg/String '{data: "{\"action\": \"start_mission\"}"}' -1
# arreter la mission : ros2 topic pub /messages std_msgs/msg/String '{data: "{\"action\": \"end_mission\"}"}' -1

class CommunicationController(Node):

    def __init__(self):
        super().__init__('communication_controller')
        self.subscription = self.create_subscription(String, '/messages', self.messages_callback, 10)
        
        # Création d'éditeurs pour chaque robot
        self.movement_publishers = {
            "robot1": self.create_publisher(String, '/robot1/movement', 10),
            "robot2": self.create_publisher(String, '/robot2/movement', 10)
        }
        
        self.feedback_subscription = self.create_subscription(String, '/feedback', self.feedback_callback, 10)
        self.server_feedback_publisher = self.create_publisher(String, '/server_feedback', 10)
        self.get_logger().info("CommunicationController ready")
        self.mission_active = False

    def messages_callback(self, msg):
        try:
            data = json.loads(msg.data)
            action = data.get('action')
            robot_id = data.get('robot_id')
            if robot_id is None:
                self.get_logger().error("robot_id non spécifié dans le message")
                return
            
            publisher = self.movement_publishers.get(robot_id)
            if publisher is None:
                self.get_logger().error(f"robot_id invalide: {robot_id}")
                return
            
            if action == 'start_mission':
                self.mission_active = True
                self.get_logger().info(f"Mission démarrée pour {robot_id}")
                publisher.publish(msg)
            elif action == 'end_mission':
                self.mission_active = False
                self.get_logger().info(f"Mission arrêtée pour {robot_id}")
                publisher.publish(msg)
            else:
                self.get_logger().info(f"Transmission de la commande vers le topic mouvement pour {robot_id}: {msg.data}")
                publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur dans le traitement du message: {str(e)}")

    def feedback_callback(self, msg):
        try:
            server_msg = String()
            server_msg.data = msg.data
            self.server_feedback_publisher.publish(server_msg)
        except Exception as e:
            self.get_logger().error(f"Erreur lors du transfert du feedback: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CommunicationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
