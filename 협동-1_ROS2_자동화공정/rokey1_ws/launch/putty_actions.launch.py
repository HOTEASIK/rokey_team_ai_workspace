"""퍼티 Action 서버만 시작한다. Goal 수신 전에는 로봇을 움직이지 않는다."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rokey1', executable='putty_task_server', output='screen'),
    ])
