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
# identifier : ros2 topic pub /identify std_msgs/msg/Empty "{}"

class CommunicationController(Node):

    def __init__(self):
        super().__init__('communication_controller')
        self.subscription = self.create_subscription(String,'/messages',self.messages_callback,10)
        self.movement_publisher = self.create_publisher(String, '/movement', 10)

        self.feedback_subscription = self.create_subscription(String, '/feedback', self.feedback_callback, 10)
        self.server_feedback_publisher = self.create_publisher(String, '/server_feedback', 10)
        self.get_logger().info("CommunicationController ready")

        self.mission_active = False

    def messages_callback(self, msg):
        try:

            data = json.loads(msg.data)
            action = data.get('action')

            if action == 'start_mission':
                if not self.mission_active:
                    self.mission_active = True
                    self.get_logger().info(f"Started mission")
                    movement_msg = String()
                    movement_msg.data = msg.data
                    self.movement_publisher.publish(movement_msg)

            elif action == 'end_mission':
                if self.mission_active:
                    self.mission_active = False
                    movement_msg = String()
                    movement_msg.data = msg.data
                    self.movement_publisher.publish(movement_msg)
            
            else :     
                self.get_logger().info(f"Forwarding message to movement topic: {msg.data}")
                movement_msg = String()
                movement_msg.data = msg.data
                self.movement_publisher.publish(movement_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing message: {str(e)}")

    def feedback_callback(self, msg):
        try:
            server_msg = String()
            server_msg.data = msg.data
            self.server_feedback_publisher.publish(server_msg)
        except Exception as e:
            self.get_logger().error(f"Error forwarding feedback: {str(e)}")


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