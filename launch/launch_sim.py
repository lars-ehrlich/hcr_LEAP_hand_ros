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
            name='ros2_example',
            emulate_tty=True,
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        )
    ])