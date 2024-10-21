
# Fingertip Sensors Node

This ROS2 node facilitates communication with a set of fingertip force sensors, collecting data and providing services for calibration.

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
  - [Services](#services)
  - [Parameters](#parameters)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The `FingertipSensors` node enables ROS2-based control and monitoring of fingertip force sensors. The node handles communication over serial, publishing the force sensor data to a ROS topic, and offers services for calibration.

**Example Use**: 
This node is useful in robotics applications requiring precise fingertip force measurements, such as grasping tasks in manipulation.

---

## Features

- Periodically reads force data from a set of fingertip sensors.
- Publishes force data to a ROS2 topic.
- Provides a calibration service for resetting the sensor readings.
- Low-latency serial communication setup for real-time applications.

---

## Installation

### Dependencies

- ROS2 Foxy (or later)
- `geometry_msgs` for vector messages.
- `uclv_seed_robotics_ros_interfaces` package for sensor messages and services.
- `serial` library for serial communication.

### Build Instructions

1. Clone the repository into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/yourusername/fingertip_sensors_node.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
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
ros2 run fingertip_sensors_node fingertip_sensors
```

### Subscribing Topics

- `/sensor_state_topic` (`uclv_seed_robotics_ros_interfaces/msg/FTS3Sensors`): Publishes the current forces from sensors.

### Publishing Topics

- `/desired_norm_topic` (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Publishes adjusted force values with sensor IDs.

### Services

- `/calibrate_sensors` (`std_srvs/srv/Trigger`): Calibrates the fingertip sensors.

### Parameters

- `serial_port` (`string`): Serial port for sensor communication.
- `baudrate` (`int`): Baudrate for serial communication.
- `millisecondsTimer` (`int`): Frequency for publishing sensor data (in milliseconds).

---

## Examples

Example of calling the calibration service:
```bash
ros2 service call /calibrate_sensors std_srvs/srv/Trigger
```

---

## Contributing

Feel free to open a pull request or issue on the repository.

---

## License

This project is licensed under the MIT License.
