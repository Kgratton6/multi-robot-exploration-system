import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class MoveController(Node):

    def __init__(self):
        super().__init__('move_controller')
        self.declare_parameter('robot_id', 'limo1')
        self.robot_id = self.get_parameter('robot_id').value

        movement_topic = f'/{self.robot_id}/movement'
        self.subscription = self.create_subscription(
            String,
            movement_topic,
            self.movement_callback,
            10
        )

        cmd_vel_topic = f'/{self.robot_id}/cmd_vel'
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.action_timer = None
        self.mission_active = False
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

    def movement_callback(self, msg):
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f"Commande reçue sur {self.robot_id}: {command}")

            if command['action'] == 'move':
                self.stop_current_motion()
                speed = command.get('speed', 0.0)
                duration = command.get('duration', 0.0)
                self.start_timed_motion(speed, 0.0, duration)

            elif command['action'] == 'turn':
                self.stop_current_motion()
                speed = command.get('speed', 0.0)
                duration = command.get('duration', 0.0)
                self.start_timed_motion(0.0, speed, duration)

            elif command['action'] == 'start_mission':
                self.mission_active = True
                self.current_linear_speed = 0.1
                self.current_angular_speed = 0.2
                self.publish_velocity(self.current_linear_speed, self.current_angular_speed)

            elif command['action'] == 'end_mission':
                self.mission_active = False
                self.stop()

            elif command['action'] == 'stop':
                self.stop()

            else:
                self.get_logger().warn(f"Commande inconnue: {command['action']}")

        except Exception as e:
            self.get_logger().error(f"Erreur dans le traitement de la commande: {str(e)}")

    def start_timed_motion(self, linear_speed: float, angular_speed: float, duration: float):
        if duration > 0:
            self.current_linear_speed = linear_speed
            self.current_angular_speed = angular_speed
            self.publish_velocity(linear_speed, angular_speed)
            if self.action_timer:
                self.action_timer.cancel()
            self.action_timer = self.create_timer(duration, self.stop)

    def stop_current_motion(self):
        if self.action_timer:
            self.action_timer.cancel()
            self.action_timer = None
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

    def publish_velocity(self, linear: float, angular: float):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)
        self.get_logger().debug(f"Vitesse publiée: linear={linear}, angular={angular}")

    def stop(self):
        self.mission_active = False
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.publish_velocity(0.0, 0.0)

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
