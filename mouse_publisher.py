import math
from pynput.mouse import Controller
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node

class MousePublisher(Node):
    def __init__(self):
        super().__init__('mouse_publisher')
        self.publisher_ = self.create_publisher(Point, 'mouse_position', 10)
        self.mouse = Controller()
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_mouse_position)
        print("Mouse Publisher started.")

    def publish_mouse_position(self):
        try:
            # Get global mouse position
            x, y = self.mouse.position

            # Screen resolution: 3840x2160, 13.3" diagonal
            screen_width = 3840
            screen_height = 2160
            screen_diagonal_inches = 13.3

            # Compute pixels per inch (PPI)
            diagonal_pixels = math.sqrt(screen_width**2 + screen_height**2)
            ppi = diagonal_pixels / screen_diagonal_inches

            # Convert to mm (1 inch = 25.4 mm)
            x_mm = (x / ppi) * 25.4
            y_mm = (y / ppi) * 25.4

            # Publish as ROS 2 Point message
            point = Point()
            point.x = x_mm
            point.y = y_mm
            point.z = 0.0
            self.publisher_.publish(point)

            # Clean output
            print(f"{x_mm:.2f}, {y_mm:.2f}")

        except Exception as e:
            print(f"Error getting mouse position: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MousePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
