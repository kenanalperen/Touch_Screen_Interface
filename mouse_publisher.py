#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from pynput.mouse import Controller

class MousePublisher:
    def __init__(self):
        rospy.init_node('mouse_publisher', anonymous=True)

        self.pub = rospy.Publisher('/mouse_position', PoseStamped, queue_size=10)
        self.mouse = Controller()
        self.last_mm_position = None
        self.inactive_counter = 999
        self.inactive_threshold = 10
        self.movement_threshold_mm = 0.001

        self.rate = rospy.Rate(10)  # 10 Hz

        # Screen parameters
        self.screen_width = 3840
        self.screen_height = 2160
        self.screen_diagonal_inches = 13
        diagonal_pixels = math.sqrt(self.screen_width**2 + self.screen_height**2)
        self.ppi = diagonal_pixels / self.screen_diagonal_inches

        # EE subscriber state
        self.ee_xy = None
        rospy.Subscriber('/ee_for_paris', PoseStamped, self.ee_callback)

        rospy.loginfo("Mouse Publisher (ROS 1) started.")

    def ee_callback(self, msg):
        x_adj = msg.pose.position.x
        y_adj = msg.pose.position.y

        # Convert to mm and apply offsets
        x_mm = x_adj * 1000.0 - 355.742
        y_mm = y_adj * 1000.0 - 404.941

        self.ee_xy = (x_mm, y_mm)

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def run(self):
        while not rospy.is_shutdown():
            try:
                y, x = self.mouse.position  # this swap is intentional

                # Convert mouse to mm
                x_mm = ((x / self.ppi) * 25.4 - 112.120 + 1.20) * 950.0 / 205.0
                y_mm = ((y / self.ppi) * 25.4 - 216.520 - 0.674) * 950.0 / 205.0

                # Clamp mouse position
                clamped_x_mm = self.clamp(x_mm, -90, 190)
                clamped_y_mm = self.clamp(y_mm, -720, 0)

                # Echo clamped mouse position
                print(f"Mouse: x = {clamped_x_mm:.3f}, y = {clamped_y_mm:.3f}")

                # Track activity
                if self.last_mm_position is not None:
                    dx = abs(x_mm - self.last_mm_position[0])
                    dy = abs(y_mm - self.last_mm_position[1])
                    if dx > self.movement_threshold_mm or dy > self.movement_threshold_mm:
                        self.inactive_counter = 0
                    else:
                        self.inactive_counter += 1
                else:
                    self.inactive_counter += 1

                self.last_mm_position = (x_mm, y_mm)

                # Determine publish position
                if self.ee_xy is not None:
                    print(f"EE:    x = {self.ee_xy[0]:.3f}, y = {self.ee_xy[1]:.3f}")

                    dx = clamped_x_mm - self.ee_xy[0]
                    dy = clamped_y_mm - self.ee_xy[1]
                    distance = math.sqrt(dx ** 2 + dy ** 2)

                    if distance < 100:
                        print(f"CLOSE TO EE: Mouse ({clamped_x_mm:.3f}, {clamped_y_mm:.3f})")
                        publish_x, publish_y = clamped_x_mm, clamped_y_mm
                    else:
                        print(f"FAR FROM EE: EE ({self.ee_xy[0]:.3f}, {self.ee_xy[1]:.3f})")
                        publish_x, publish_y = self.ee_xy[0], self.ee_xy[1]

                else:
                    print("EE:    No signal yet.")
                    publish_x, publish_y = 0.0, 0.0


                    

                z_value = 0.0  # No touch logic

                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.header.frame_id = "world"

                pose_stamped.pose.position.x = publish_x
                pose_stamped.pose.position.y = publish_y
                pose_stamped.pose.position.z = z_value

                pose_stamped.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

                self.pub.publish(pose_stamped)

                print("-" * 40)
                self.rate.sleep()

            except Exception as e:
                rospy.logerr(f"Error getting mouse position: {e}")

if __name__ == '__main__':
    try:
        node = MousePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
