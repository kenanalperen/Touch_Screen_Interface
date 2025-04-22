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
        self.last_mm_position = None
        self.inactive_counter = 999  # Force initial state to be "No touch"
        self.inactive_threshold = 10  # Number of cycles (0.1s each)
        self.movement_threshold_mm = 0.001  # Minimum movement to count as touch
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.publish_mouse_position)
        print("Mouse Publisher started.")

    def publish_mouse_position(self):
        try:
            x, y = self.mouse.position

            # Screen resolution: 3840x2160, 13.3" diagonal
            screen_width = 3840
            screen_height = 2160
            screen_diagonal_inches = 13.3
            diagonal_pixels = math.sqrt(screen_width**2 + screen_height**2)
            ppi = diagonal_pixels / screen_diagonal_inches

            # Convert to mm
            x_mm = (x / ppi) * 25.4
            y_mm = (y / ppi) * 25.4

            # Detect small movements in mm
            if self.last_mm_position is not None:
                dx = abs(x_mm - self.last_mm_position[0])
                dy = abs(y_mm - self.last_mm_position[1])
                if dx > self.movement_threshold_mm or dy > self.movement_threshold_mm:
                    self.inactive_counter = 0
                else:
                    self.inactive_counter += 1
            else:
                # First reading: don't immediately say "Touch"
                self.inactive_counter += 1

            self.last_mm_position = (x_mm, y_mm)

            # Publish
            point = Point()
            point.x = x_mm
            point.y = y_mm
            point.z = 0.0
            self.publisher_.publish(point)

            # Print with 3-digit precision
            touch_state = "Touch" if self.inactive_counter < self.inactive_threshold else "No touch"
            print(f"{x_mm:.3f}, {y_mm:.3f} - {touch_state}")

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
