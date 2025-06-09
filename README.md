<p align="center">
  <img src="https://github.com/user-attachments/assets/036008b6-ac72-4532-9da5-20636637d729" alt="Screenshot from 2025-06-09 13-45-44" width="70%">
</p>


# Touch Screen Interface

This repository includes details regarding our communication setup and the ROS packages we used on the human-robot interface side. On the robot-environment side, we implemented a specialised impedance controller, which is confidential to IIT.

## Communication Architecture

![Communication Architecture](https://github.com/user-attachments/assets/435dd327-e2c3-4714-8e34-f4fc2613139f)

The figure above provides an overview of the communication architecture we used. Four PCs were involved in the remote teleoperation setup:

- **PC1 [Bristol, UK]:** Served as the human-robot interface. It collected finger position data from users and displayed the camera feed coming from the robot.
- **PC2 [Bristol, UK]:** Used for logging experimental data during the user studies.

- **PC3 [Genoa, Italy]:** Received position reference data and converted it into a `PoseEstimate` ROS topic with a timestamp adjusted for the robot.
- **PC4 [Genoa, Italy]:** Acted as the ROS Master to control the Franka Panda arm. It also enforced virtual boundaries and end-effector velocity limits for safety.

The robot in Italy required the use of ROS1. Initially, we implemented ROS bridging in Bristol, UK to convert our code from ROS2 to ROS1 before sharing it, which was functional. However, it turned out to be easier to modify our code so that everything runs directly on ROS1.

For the Virtual Private Network (VPN), we used two different third-party software tools. It is essential that all four PCs are connected to the same virtual network. Once connected, they behave as if they are on the same local network. To ensure better internet performance, we used wired connections instead of Wi-Fi.

## ROS Nodes

https://github.com/user-attachments/assets/2bc98b0d-5b56-4d39-8313-eb77d4530146

Below are the Python packages we used on the operator side during the experiments:

### Finger Position Tracker (`mouse_publisher.py`)

This Python node takes finger position input from a touchscreen, converts pixel coordinates to real-world values (with optional scaling), and publishes these reference positions to the robot.

> **Note:** Ensure "Ubuntu on Xorg" is selected at the login screen. The position tracker does not work properly with Wayland, as it only captures inputs from web pages due to security restrictions.

### Joystick Position Reference (`joystick_control.py`)

This node provides a baseline comparison to our novel interface. It takes input from a commercial joystick—an industry-standard device—converts the joystick angle into velocity values, and applies a scaling factor to adjust the end-effector speed before publishing the reference positions to the robot.

### Camera Angle Calibration (`calibrate_angle.py`)

This node adjusts the angled video feed received from Italy into a top-down view resembling a coordinate frame. Users are asked to click on the four corners of the workspace and the initial end-effector position to calibrate the GUI.

The corresponding Python script is available as `calibrate_angle.py`.

<img src="https://github.com/user-attachments/assets/86dfdff6-8219-4c5f-b4ac-c12381f2dfc2" alt="Calibration Screenshot" width="50%">


### GUI Interface (`new_display.py`)

This node visualises the video feed along with the robot's end-effector position, which is highlighted and leaves a trail of previous positions. The display is a scaled-down version of the actual workspace and requires a video stream as input.

The script is located at `new_display.py`.

### Converting MATLAB Trajectories to ROS Topics (`path_generator.mlx` & `mat_to_rosbag.py`)

Automated trajectories are generated using a MATLAB script (`path_generator.mlx`) as time series data. Users can configure path parameters, initial delay, and end-effector velocity.

These `.mat` files are then converted into ROS topics and recorded as `.bag` files using `mat_to_rosbag.py`.

Example files such as `sinusoid_trajectory.mat` and `sinusoid_trajectory.bag` are included in the repository. Playing the `.bag` file sends the defined `PoseEstimate` reference commands to the robot.

## Participant Invitation Poster
![repository_Pic](https://github.com/user-attachments/assets/6a3f10e3-d321-4b91-9de9-61e257e41566)

