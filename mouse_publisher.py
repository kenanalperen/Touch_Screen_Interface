import math
import subprocess
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node

class MousePublisher(Node):
    def __init__(self):
        super().__init__('mouse_publisher')
        self.publisher_ = self.create_publisher(Point, 'mouse_position', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_mouse_position)
        self.get_logger().info('Mouse Publisher started.')

    def publish_mouse_position(self):
        try:
            # Get the mouse position in pixels using xdotool
            result = subprocess.run(['xdotool', 'getmouselocation'], capture_output=True, text=True)
            output = result.stdout.strip()
            parts = output.split()
            x = int(parts[0].split(':')[1])
            y = int(parts[1].split(':')[1])

            # Screen details: 15.6-inch 3:4 screen with 1920x1440 resolution
            screen_width = 1920
            screen_height = 1440
            screen_diagonal = 15.6  # inches

            # Calculate diagonal resolution
            diagonal_resolution = math.sqrt(screen_width**2 + screen_height**2)

            # Calculate PPI (pixels per inch)
            ppi = diagonal_resolution / screen_diagonal

            # Convert mouse position to millimeters (1 inch = 25.4 mm)
            x_mm = (x / ppi) * 25.4
            y_mm = (y / ppi) * 25.4

            # Log the position in mm
            self.get_logger().info(f"Mouse position (mm): x = {x_mm:.2f}, y = {y_mm:.2f}")

            # Create Point message to publish in mm
            point = Point()
            point.x = float(x_mm)
            point.y = float(y_mm)
            point.z = 0.0  # Unused
            self.publisher_.publish(point)
        except Exception as e:
            self.get_logger().error(f'Failed to get mouse position: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MousePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
