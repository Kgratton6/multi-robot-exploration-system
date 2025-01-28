import rclpy
from rclpy.node import Node

class LedController(Node):
    def __init__(self):
        super().__init__('led_controller') # name of the node
        self.get_logger().info('LedController Node Started!')

    def control_led(self):
        self.get_logger().info('Controlling LED...')

def main(args=None):
    rclpy.init(args=args)
    node = LedController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()