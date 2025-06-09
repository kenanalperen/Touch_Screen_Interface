#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Quaternion
import time

class JoystickPublisher:
    def __init__(self):
        rospy.init_node('joystick_position_publisher', anonymous=True)

        self.pub = rospy.Publisher('/mouse_position', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/joy', Joy, self.joy_callback)

        self.velocity = {"vx": 0.0, "vy": 0.0}
        self.position = {"x": 0.0, "y": 0.0, "z": 0.0}

        self.scale_factor = 25.0  # Adjust as needed
        self.last_time = time.time()

        self.min_x, self.max_x = -720.0, 0.0
        self.min_y, self.max_y = -90.0, 190.0

        rospy.loginfo("Joystick Publisher running at 10 Hz.")

    def clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)

    def joy_callback(self, msg: Joy):
        # Store velocity from joystick input
        self.velocity["vx"] = -msg.axes[3] * self.scale_factor
        self.velocity["vy"] = -msg.axes[4] * self.scale_factor  # Invert Y

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz loop

        while not rospy.is_shutdown():
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            # Integrate velocity to update position
            self.position["x"] += self.velocity["vx"] * dt
            self.position["y"] += self.velocity["vy"] * dt

            # Clamp to limits
            self.position["x"] = self.clamp(self.position["x"], self.min_x, self.max_x)
            self.position["y"] = self.clamp(self.position["y"], self.min_y, self.max_y)

            # Always 0 for now
            self.position["z"] = 0.0

            # Publish
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position.x = self.position["y"]  # Y maps to X
            pose.pose.position.y = self.position["x"]  # X maps to Y
            pose.pose.position.z = self.position["z"]
            pose.pose.orientation = Quaternion(0.0, 1.0, 0.0, 0.0)

            self.pub.publish(pose)
            rospy.loginfo(f"Published: x={self.position['y']:.2f}, y={self.position['x']:.2f}")

            rate.sleep()


if __name__ == '__main__':
    try:
        node = JoystickPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
