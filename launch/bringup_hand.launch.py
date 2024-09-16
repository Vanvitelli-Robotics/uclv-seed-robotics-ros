from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Parametri comuni tra i nodi
    common_params = {
        "baudrate": 1000000,  # Baudrate
        "millisecondsTimer": 2  # Timer per la pubblicazione
    }

    return LaunchDescription([
        Node(
            output='screen',
            package='uclv_seed_robotics_ros',
            executable='hand_driver',
            name='hand_driver',
            parameters=[
                common_params,  # Parametri comuni
                {"serial_port": "/dev/ttyUSB0"},
                {"motor_ids": [31, 32, 33, 34, 35, 36, 37, 38]},  # ID dei motori
                {"motor_thresholds": [100, 3995]},  # Soglie dei motori
                {"protocol_version": 2.0},  # Versione del protocollo
                {"motor_state_topic": "motor_state"},  # Nome del topic di stato motori
                {"desired_position_topic": "desired_position"}  # Nome del topic di posizioni desiderate
            ]
        ),
        Node(
            output='screen',
            package="uclv_seed_robotics_ros",
            executable='fingertip_sensors',
            name='fingertip_sensors',
            parameters=[
                common_params,
                {"serial_port": "/dev/ttyUSB1"},  # 
                {"sensor_state_topic": "sensor_state"}  # Nome del topic per lo stato dei sensori
            ]
        ),
    ])
