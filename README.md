
<p align="center"><img src="https://github.com/user-attachments/assets/acbb23e4-441a-4693-88fc-804b2b04fbce" /></p> 

# Touch Screen Interface   
Codes used for the touch screen interface. The main sections are:

## Get touch screen position data

<pre> ros2 run mouse_tracker mouse_publisher  </pre>

The python code used can be found at the repository under mouse_publisher.py

P.S. Make sure to select "Ubuntu on Xorg" in the settings on the Ubuntu login screen. The position tracker does not function properly with Wayland, it only works over web pages and not across the entire screen due to security restrictions.

The output should show x,y coordinates in (mm) and the state of "touch" or "no touch"

<pre>170.374, 115.167 - No touch
172.061, 115.167 - Touch</pre>

## Simulate Franka Panda Arm and Digital Twin
Follow the instructions at the official website
https://github.com/frankaemika/franka_ros2

Run the rviz with 
<pre>ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true</pre>

## Video Feed Display

Start the pyhton code given by running

<pre>python3 webcam_gui.py</pre>

## Integration and Implementation with Impedance Controller
