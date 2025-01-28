import rclpy
from rclpy.node import Node
import subprocess
from pathlib import Path

class SoundController(Node):
    def __init__(self):
        super().__init__('sound_controller') # name of the node
        self.get_logger().info('SoundController Node Started!')
        self.play_sound()

    def play_sound(self):
        try:

            current_dir = Path(__file__).parent.resolve()
            sound_path = current_dir / 'files' / 'start.mp3'
            
            self.get_logger().info(f"Sound path: {str(sound_path)}")
            
            # if sound_path.exists():
            #     subprocess.Popen(["aplay", str(sound_path)])  # Convert to string
            #     self.get_logger().info("Playing startup sound...")
            # else:
            #     self.get_logger().error(f"Sound file not found: {str(sound_path)}")
        except Exception as e:
            self.get_logger().error(f"Failed to play sound: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = SoundController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()