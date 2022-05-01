from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0']
        ),
        Node(
            package='teensy_reset_node',
            executable='subscriber_teensy_reset',
            name='teensy_reset_node'
        )

    ])