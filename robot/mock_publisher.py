import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import time

class MissionClient(Node):
    def __init__(self):
        super().__init__('mission_client')
        self.client = self.create_client(Trigger, 'send_mission')

        while not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('⏳ Service not available, waiting...')

        self.get_logger().info('✅ Service "send_mission" is available!')

    def send_request(self):
        request = Trigger.Request()
        future = self.client.call_async(request)
        future.add_done_callback(self.callback_response)

    def callback_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"🛰️ Response received: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MissionClient()

    try:
        while rclpy.ok():
            node.get_logger().info("🚀 Sending mission request...")
            node.send_request()
            time.sleep(5)  # Wait 5 seconds before sending the next request
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Mission client shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
