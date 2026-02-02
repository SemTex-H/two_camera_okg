from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 4. Your Custom Controller Node
        Node(
            package='joy_motor_control',
            executable='motor_control',
            name='motor_control'
        )
    ])
