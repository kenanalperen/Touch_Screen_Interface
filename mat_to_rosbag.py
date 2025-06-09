#!/usr/bin/env python3

import sys
import os
import rospy
import rosbag
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from scipy.io import loadmat

if len(sys.argv) != 2:
    print("Usage: python3 your_script.py <filename.mat>")
    sys.exit(1)

mat_file = sys.argv[1]

# Check the extension
if not mat_file.endswith('.mat'):
    print("Error: The input file must have a .mat extension")
    sys.exit(1)

# Derive the bag filename
bag_file = os.path.splitext(mat_file)[0] + '.bag'

# Load the .mat file
data = loadmat(mat_file)
t = data['t'].squeeze()
x = data['x'].squeeze()
y = data['y'].squeeze()

# Create a rosbag file
bag = rosbag.Bag(bag_file, 'w')

try:
    for i in range(len(t)):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = rospy.Time.from_sec(t[i])
        msg.header.frame_id = "map"

        msg.pose.position.x = float(x[i])
        msg.pose.position.y = float(y[i])
        msg.pose.position.z = 0.0

        # Orientation (no rotation)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        bag.write('/mouse_position', msg, t=rospy.Time.from_sec(t[i]))
finally:
    bag.close()
