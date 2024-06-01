from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            output="screen",
            package='hand_driver',
            # namespace='',
            executable='hand_driver',
            name='hand_driver',
            parameters=[
                {"motor_ids": [31, 32, 33, 34, 35, 36, 37, 38]} # left hand
            ]
        ),
    ])
    