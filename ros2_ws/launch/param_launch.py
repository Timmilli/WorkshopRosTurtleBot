from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher',
            executable='talker6',
            name='sim',
            output='screen',
            emulate_tty=True,
            parameters=[
            {
                'correct_computer_name': 'nano',
                'correct_domain_id': 33
             }
            ]
        ),
        Node(
            package='publisher',
            executable='listener6',
            name='sim',
            output='screen',
            emulate_tty=True,
        )
    ])
