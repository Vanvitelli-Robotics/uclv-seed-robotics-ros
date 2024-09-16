from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Common parameters for both nodes
    common_params = {
        "baudrate": 1000000,  # Baud rate for serial communication
        "millisecondsTimer": 2  # Timer interval for publishing
    }

    return LaunchDescription([
        Node(
            output='screen',
            package='uclv_seed_robotics_ros',
            executable='hand_driver',
            name='hand_driver',
            parameters=[
                common_params,  # Common parameters
                {"serial_port": "/dev/ttyUSB0"},  # Serial port for hand driver
                {"motor_ids": [31, 32, 33, 34, 35, 36, 37, 38]},  # Motor IDs for the left hand
                {"motor_thresholds": [100, 3995]},  # Threshold values for the motors
                {"protocol_version": 2.0},  # Communication protocol version
                {"motor_state_topic": "motor_state"},  # Topic name for motor state
                {"desired_position_topic": "desired_position"}  # Topic name for desired motor positions
            ]
        ),
        Node(
            output='screen',
            package="uclv_seed_robotics_ros",
            executable='fingertip_sensors',
            name='fingertip_sensors',
            parameters=[
                common_params,  # Common parameters
                {"serial_port": "/dev/ttyUSB1"},  # Serial port for the fingertip sensors
                {"sensor_state_topic": "sensor_state"}  # Topic name for sensor state
            ]
        ),
    ])
