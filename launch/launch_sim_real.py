from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leap_sim',
            namespace='leap_sim',
            executable='leap_sim',
            emulate_tty=True,
            output='screen',
            name='leap_sim'
        ),
        Node(
            package='leap_hand',
            namespace='leap_hand',
            executable='ros2_example.py',
            name='ros2_leap_monitor',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),
        Node(
            package='leap_hand',
            executable='leaphand_node.py',
            name='leaphand_node',
            emulate_tty=True,
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0}
            ]
        )
    ])

