import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveController(Node):
    def __init__(self):
        super().__init__('move_controller')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Move Node started !')
    
    def move(self, speed, duration):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds < duration * 1e9:
            msg = Twist()
            msg.linear.x = speed
            self.publisher.publish(msg)
            self.get_logger().info(f"Publishing speed: {speed} m/s")
            time.sleep(0.1)  # Publie toutes les 100 ms
        self.stop()

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info("Stopping")

def main(args=None):
    rclpy.init(args=args)
    controller = MoveController()
    
    while True:
        try:
            controller.move(0.5, 5)  # avancer 5 secondes
            controller.move(-0.5, 5) # recules 5 secondes
            
        finally:
            controller.stop()
            #controller.destroy_node()
            #rclpy.shutdown()

if __name__ == '__main__':
    main()