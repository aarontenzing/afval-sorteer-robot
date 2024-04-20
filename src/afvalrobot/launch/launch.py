from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='afvalrobot',
            executable='gripper',
            name='gripper'
        ),
        Node(
            package='afvalrobot',
            executable='sonar',
            name='sonar'
        ),
        Node(
            package='afvalrobot',
            executable='wheels',
            name='wheels'
        )
    ])