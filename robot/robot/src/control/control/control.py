import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class MoveController(Node):

    def __init__(self):
        super().__init__('move_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.movement_subscription = self.create_subscription(String, '/movement', self.movement_callback, 10)
        
        self.current_action = None
        self.action_start_time = None
        self.action_duration = 0
        self.current_speed = 0.0
        self.current_angular = 0.0

        # Variables ajoutées pour le feedback
        self.current_position = {"x": 0.0, "y": 0.0}  # Valeur fictive, à remplacer par la position réelle
        self.battery_level = 100                      # Valeur fictive, à remplacer par le niveau de batterie réel

        # Publisher pour le feedback (robot → serveur)
        self.feedback_publisher = self.create_publisher(String, '/feedback', 10)
        # Timer pour publier périodiquement le feedback (toutes les 1 seconde ici)
        self.feedback_timer = self.create_timer(1.0, self.publish_feedback)

    def movement_callback(self, msg):
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f"Received movement command: {command}")
            
            if command['action'] == 'move':
                self._cancel_current_action()
                self.current_speed = command.get('speed', 0.0)
                self.current_angular = 0.0
                self.action_duration = command.get('duration', 0.0)
                self.start_action_timer()

            elif command['action'] == 'turn':
                self._cancel_current_action()
                self.current_speed = 0.0
                self.current_angular = command.get('speed', 0.0)
                self.action_duration = command.get('duration', 0.0)
                self.start_action_timer()

            elif command['action'] == 'stop':
                self.stop()
            else:
                self.get_logger().warn(f"Unknown command: {command['action']}")

        except Exception as e:
            self.get_logger().error(f"Error processing movement command: {str(e)}")

    def start_action_timer(self):
        self.action_start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._execute_action)

    def _execute_action(self):
        elapsed = (self.get_clock().now() - self.action_start_time).nanoseconds / 1e9
        
        if elapsed < self.action_duration:
            twist = Twist()
            twist.linear.x = self.current_speed
            twist.angular.z = self.current_angular
            self.publisher.publish(twist)
        else:
            self.stop()
            self.timer.cancel()

    def _cancel_current_action(self):
        if hasattr(self, 'timer') and not self.timer.is_canceled():
            self.timer.cancel()
        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.current_speed = 0.0
        self.current_angular = 0.0

    def publish_feedback(self):
        # Période du timer (ici 1 seconde)
        dt = 1.0  
        # Mise à jour de la position en ajoutant la distance parcourue (vitesse * temps)
        self.current_position["x"] += self.current_speed * dt

        # Prépare un dictionnaire de feedback incluant la vitesse, l'angle, la position et le niveau de batterie
        feedback = {
            "speed": self.current_speed,
            "angular": self.current_angular,
            "position": self.current_position,
            "battery": self.battery_level
        }
        # Convertit le feedback en chaîne JSON
        new_feedback_str = json.dumps(feedback)

        # Initialisation de self.last_feedback si elle n'existe pas encore
        if not hasattr(self, 'last_feedback'):
            self.last_feedback = ""

        # Ne publie que si le nouveau feedback diffère du précédent
        if new_feedback_str != self.last_feedback:
            feedback_msg = String()
            feedback_msg.data = new_feedback_str
            self.feedback_publisher.publish(feedback_msg)
            self.get_logger().info(f"Feedback published: {feedback_msg.data}")
            # Met à jour la variable last_feedback
            self.last_feedback = new_feedback_str
        else:
            self.get_logger().debug("Feedback unchanged, not publishing.")


def main(args=None):
    rclpy.init(args=args)
    node = MoveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()