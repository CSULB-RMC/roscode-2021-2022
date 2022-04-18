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

    ])