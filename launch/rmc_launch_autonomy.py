from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rmc_camera',
            executable='publisher_feed',
            name='publisher_feed'
        ),
        Node(
            package='rmc_camera',
            executable='publisher_obstruction',
            name='publisher_obstruction'
        ),
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0']
        )

    ])