import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import json

class MoveController(Node):

    def __init__(self):
        super().__init__('move_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.movement_subscription = self.create_subscription(String,'/movement', self.movement_callback, 10)
        
        self.current_action = None
        self.action_start_time = None
        self.action_duration = 0
        self.current_speed = 0.0
        self.current_angular = 0.0

    def movement_callback(self, msg):
        try:
            command = json.loads(msg.data)
            self.get_logger().info(f"Received movement command: {command}")
            
            if command['action'] == 'move':
                self._cancel_current_action()
                self.target_speed = command.get('speed', 0.0)
                self.target_angular = 0.0
                self.action_duration = command.get('duration', 0.0)
                self.start_action_timer()
            elif command['action'] == 'turn':
                self._cancel_current_action()
                self.target_speed = 0.0
                self.target_angular = command.get('speed', 0.0)
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
            twist.linear.x = self.target_speed
            twist.angular.z = self.target_angular
            self.publisher.publish(twist)
        else:
            self.stop()
            self.timer.cancel()

    def _cancel_current_action(self):
        if hasattr(self, 'timer') and self.timer.is_canceled() is False:
            self.timer.cancel()
        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

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