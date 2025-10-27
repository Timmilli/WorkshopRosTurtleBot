from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher',
            executable='talker4',
            name='sim',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='publisher',
            executable='listener4',
            name='sim',
            output='screen',
            emulate_tty=True,
        )
    ])
