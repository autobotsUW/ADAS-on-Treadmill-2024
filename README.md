# ADAS-on-Treadmill-2024

## Introduction
The University of Waterloo’s Real-time Embedded Systems Lab hosts an autonomous driving system operating radio-controlled (RC) cars on a treadmill. The aim of this project is to simulate ADAS on RC cars. The cars move on a treadmill. A camera is placed above the system to detect both the position of the vehicles and obstacles. Obstacles can be studs, which move only on one axis, or balls. The cars maintain a position on the conveyor belt until an obstacle is detected. They must then move to avoid it. 
Information from the camera is received on a PC and processed by ROS 2, before being transmitted to the cars via Bluetooth.
We use ROS2 Humble with ubuntu 22.04

Oh to launch car controller:
```
cd ros2_jf_waterloo
source install/setup.bash
colcon build
cd launch
ros2 launch <launch_file>
```

To launch the treadmill: use treadmill_control.py with a good configuration.

# Package

## Nodes

### controller_node
- **Description**: The main node responsible for defining commands for the car. It allows changing algorithms to test different car behaviors.

### camera_node
- **Description**: This node captures images from the camera and provides the position of the cars and obstacles.

### bluetooth_node
- **Description**: This node sends commands to the car via Bluetooth.

### input_node
- **Description**: This node receives user inputs to change the car's position.

### display_node
- **Description**: This node displays the position of the cars and obstacles.

### avoiding_obstacles_node
- **Description**: This node defines input positions to avoid obstacles.

### treadmill_control_node
- **Description**: This node controls the treadmill.

### security_node
- **Description**: This node detects anomalies.

## Topics

To allow communication between these nodes, several topics are used:

### /car_position
- **Type**: Float32 list
- **Description**: List of car positions and their dimensions.
- **Format**: `[x1, y1, θ1, height1, width1, x2, y2, θ2, height2, width2, ...]`

### /input_position
- **Type**: Float32 list
- **Description**: User input position.
- **Format**: `[Xinput, Yinput]`

### /command
- **Type**: Int32 list
- **Description**: Speed and angle commands for the car.
- **Format**: `[speed, angle]`

### /obstacles_position
- **Type**: Float32 list
- **Description**: Position of obstacles.
- **Format**: `[x1, y1, radius1, x2, y2, radius2, ...]`

### /error
- **Type**: String
- **Description**: Error messages.

### /treadmill
- **Type**: String
- **Description**: Commands for the treadmill.

## Launch files:
- **ADAS_on_treadmill_launch**: the car go to the position that you define.
- **ADAS_on_treadmill_launch_with_obstacles**: the car avoid obstacles.

