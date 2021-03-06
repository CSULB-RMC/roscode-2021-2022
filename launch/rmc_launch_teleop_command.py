from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rmc_teleop',
            executable='subscriber_web_server',
            name='subscriber_web_server'
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            parameters=[{'autorepeat_rate': 0.0, 'coalesce_interval': 0.01}]
        ),
        Node(
            package='rmc_teleop',
            executable='gamepad_control',
            name='gamepad_control'
        ),

    ])