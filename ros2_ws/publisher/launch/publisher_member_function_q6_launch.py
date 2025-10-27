from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='publisher',
            executable='talker6',
            name='custom_minimal_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[
            {
                'correct_computer_name': 'calipso',
                'correct_domain_id': 67
             }
            ]
        )
    ])
