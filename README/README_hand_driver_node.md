
# Hand Driver Node

This ROS2 node is responsible for controlling the positions of hand motors through serial communication, subscribing to desired motor positions and publishing the current motor states.

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
  - [Dependencies](#dependencies)
  - [Build Instructions](#build-instructions)
- [Usage](#usage)
  - [Running the Node](#running-the-node)
  - [Subscribing Topics](#subscribing-topics)
  - [Publishing Topics](#publishing-topics)
  - [Parameters](#parameters)
- [Examples](#examples)

---

## Overview

The `HandDriver` node manages communication with multiple motors on a robotic hand, handling the movement and control of fingers and wrist joints. It interacts with the hardware through a low-latency serial connection, and it provides real-time control of motor positions.

---

## Features

- Publishes the current state of all motors in the hand.
- Subscribes to desired motor positions and moves motors accordingly.
- Supports error handling for out-of-bound motor positions.
- Dynamixel-based communication with multiple motors.

---

## Installation

### Dependencies

- ROS2 Foxy (or later).
- `uclv_seed_robotics_ros_interfaces` for motor positions messages.
- `uclv_dynamixel_utils` for Dynamixel motor utilities.
- `serial` for serial communication with hardware.

### Build Instructions

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd /path/to/your/ros2/workspace
   git clone https://github.com/Vanvitelli-Robotics/uclv-seed-robotics-ros.git
   ```

2. Build the workspace:
   ```bash
   cd /path/to/your/ros2/workspace
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## Usage

### Running the Node

To run the node:
```bash
ros2 run uclv_seed_robotics_ros hand_driver_node
```

### Subscribing Topics

- `/desired_position_topic` (`uclv_seed_robotics_ros_interfaces/msg/MotorPositions`): Subscribes to the desired motor positions for hand control.

### Publishing Topics

- `/motor_state_topic` (`uclv_seed_robotics_ros_interfaces/msg/MotorPositions`): Publishes the current motor positions.

### Parameters

- `serial_port` (`string`): Serial port for communication with the motors.
- `baudrate` (`int64_t`): Baudrate for the serial connection.
- `millisecondsTimer` (`int64_t`): Frequency (in milliseconds) for publishing motor states.
- `protocol_version` (`double`): Protocol version for Dynamixel communication.
- `motor_ids` (`std::vector<int64_t>`): List of motor IDs controlled by the node.
- `motor_thresholds` (`std::vector<int64_t>`): Thresholds for valid motor position ranges.

---

## Examples

Example service call for moving the motors to desired positions:
```bash
ros2 topic pub /cmd/positions uclv_seed_robotics_ros_interfaces/msg/MotorPositions "{ids: [31, 32, 33, 34, 35, 36, 37, 38], positions: [100, 2000, 2000, 100, 100, 100, 100, 100]}"
```
