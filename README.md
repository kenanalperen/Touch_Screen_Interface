<img src="https://github.com/user-attachments/assets/036008b6-ac72-4532-9da5-20636637d729" alt="Screenshot from 2025-06-09 13-45-44" width="70%">

# Touch Screen Interface   
This repository includes details regarding our communication channel and the ROS packages we utilised at the human-robot interface side. In Robot-Environment side we implemented a specialised impedance controller which is confidential to IIT.

## Communication Architecture
![A4 - 1](https://github.com/user-attachments/assets/435dd327-e2c3-4714-8e34-f4fc2613139f)

Above figure gives the overview of the communication architecture we utilised. There were 4 PCs involved during the remote teleoperation:

PC1: [Bristol, UK] Used as the human-robot interface. It took finger position data of the users and displayed the camera feed coming from the robot.
PC2: [Bristol, UK] Used for logging experiment data during the user studies.

PC3: [Genoa, Italy] Used for recieving position reference and converting into a PoseEstimate ros topic with timestamp adjusted to the robot.
PC4: [Genoa, Italy] Used as the ROS Master to control the Franka Panda Arm. Had virtual boundaries and end effector velocity limits for safety.

For the Virtual Private Network (VPN), we used two different third party software, and the important thing is that all four PCs has to be in the same virtual network. After this communication, PCs act as if they are located in the same local network. For better internet speed we connected the PCs through wired internet rather than wi-fi connection.


## ROS Nodes
https://github.com/user-attachments/assets/ef831d45-5722-4e35-8653-e6c80fcb0752

Here we explain the python packages we used for the experiments at the operator side.

### Finger Position Tracker (mouse_publisher.py)

This python node takes finger position input from the users from a touch screen, converts pixel values to real life values (with scaling involved if requested), and publishes these reference positions for the robot.

The python code used can be found at the repository under mouse_publisher.py

P.S. Make sure to select "Ubuntu on Xorg" in the settings on the Ubuntu login screen. The position tracker does not function properly with Wayland, it only works over web pages and not across the entire screen due to security restrictions.

### Joystick Position Reference (joystick_control.py)

As a comparison to the novel kinterface, this python node takes joystick reference input from the users through a commercial joystick, which is the industry standard, converts analog joystick angle to real life velocity values, includes a scaling parameter to adjust end effector speed, and publishes these reference positions for the robot.

The python code used can be found at the repository under joystick_control.py

### Calibrating angle of camera feed (calibrate_angle.py)

This node is used to wrap the video feed sent from Italy, which has an angle, to a top-view that looks like coordinate frame. In this node you click on four corners of your workspace and your initial end effector position to give reference values for the gui interface.

The python code used can be found at the repository under calibrate_angle.py

![image](https://github.com/user-attachments/assets/86dfdff6-8219-4c5f-b4ac-c12381f2dfc2)

### GUI (new_display.py)

This node is used for visualising the camera feed with robot end effector position highlighted and leaves a mark at the previous positions. It is a scaled donw version of the actual workspace. It requires a video feed input.

The python code used can be found at the repository under new_display.py


### Converting MATLAB generated trajectories to ROS topics (path_generator.mlx) and (mat_to_rosbag.py)

Automated trajectories are generated through a MATLAB code as a timeseries data, which you can adjust path parameters as well as initial wait time and end effector velocity using path_generator.mlx

These trajectory.mat files are converted into ROS Topics and recorded into rosbag.bag files using the mat_to_rosbag.py node.

As an example sinusoid_trajectory.mat and sinusoid_trajectory.bag files are atached to the repository, which when the .bag file is played, sends POseEstimate reference to the robot with defined parameters.


## Participant Invitation Poster
![image](https://github.com/user-attachments/assets/2429c645-1fe3-48eb-a894-443f50704196)
