
# UCLV Seed Robotics ROS


This repository contains the ROS2 and C++ driver for the RH8D robotic hand by Seed Robotics, developed at the Università degli Studi della Campania Luigi Vanvitelli.

## Description

This project provides a driver for controlling the RH8D robotic hand using ROS 2. It enables a flexible and robust interface to manage the robotic hand’s movements, easily integrating with other ROS components. It’s ideal for advanced robotic manipulation applications like human-robot interactions and research.

## Table of Contents

- [Features](#features)
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Additional Dependencies](#additional-dependencies)
- [Usage](#usage)
- [Nodes](#nodes)
- [Launch Files](#launch-files)
- [Configuration Files](#configuration-files)
- [Contributing](#contributing)
- [Authors](#authors)

## Features

- ✅ Compatible with ROS2
- ✅ Supports multiple nodes
- ✅ Includes launch files for easy startup
- ✅ Configuration file support for custom settings
- ✅ RH8D hand control interface
- ✅ Integrates custom messages and libraries for robot control

## Installation

1. Clone this repository into your ROS2 workspace:
   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone https://github.com/Vanvitelli-Robotics/uclv-seed-robotics-ros.git
   ```
2. Build the package using `colcon`:
   ```bash
   cd /path/to/your/ros2/workspace
   colcon build --packages-select uclv_seed_robotics_ros
   ```

## Dependencies

- [ROS 2](https://index.ros.org/doc/ros2/) - Robot Operating System 2
- `rclcpp` - ROS 2 C++ Client Library

## Additional Dependencies

This project requires additional dependencies from other repositories. Clone the following repositories into your ROS2 workspace:
- **Custom Messages**: Contains ROS2 custom message definitions.
- **Robotic Hand Library**: External library for  hand control (already included in this package).

1. Custom Interfaces:
   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone https://github.com/Vanvitelli-Robotics/uclv-seed-robotics-interfaces.git
   ```
2. Dynamixel Utils:
   ```bash
   cd /path/to/your/ros2/workspace/src
   git clone https://github.com/Vanvitelli-Robotics/uclv-dynamixel-utils.git
   ```

## Usage

Practical usage examples for the robotic hand controller will be added here.

## Nodes

This project contains multiple nodes for robotic hand control.

### Node 1: `hand_driver_node`
- **Description**: Controls the movements of the RH8D robotic hand throught Dynamixel actuators.
- **Topics/Subscribers**: Describe the topics this node subscribes to and publishes.
- **Services**: List any services this node provides.

## Launch Files

This project includes launch files for easier management of nodes.

### Launch File: `bringup_hand.launch.py`
- **Description**: Launches the main hand controller node with some default parameters

#### Running the Launch File

```bash
ros2 launch uclv_seed_robotics_ros bringup_hand.launch.py
```

## Configuration Files

Configuration files are provided to set parameters for the nodes.

### Config File: `hand_config.yaml`
- **Parameters**: Lists and describes the parameters used by the hand controller.

## Contributing

If you would like to contribute to this project, please follow these guidelines:

1. Fork the repository
2. Create a branch for your feature (`git checkout -b feature/your-feature-name`)
3. Commit your changes (`git commit -m 'Add a new feature'`)
4. Push the branch (`git push origin feature/your-feature-name`)
5. Open a Pull Request


## Authors

- Roberto Chello - [GitHub Profile](https://github.com/robertochello)
