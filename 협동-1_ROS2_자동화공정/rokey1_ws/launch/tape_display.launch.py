"""테이프 롤 URDF 를 TF / RViz 로 띄우는 launch.

  ros2 launch rokey1 tape_display.launch.py outer_d:=0.105 z:=0.25
  RViz: Add → RobotModel, Description Topic = /tape/robot_description
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGS = {
    'x': '0.5', 'y': '0.0', 'z': '0.2', 'yaw': '0.0',
    'outer_d': '0.040', 'core_d': '0.035', 'width': '0.050',
    'shaft_d': '0.010', 'shaft_len': '0.080',
    'flap_angle': '0.0', 'flap_len': '0.015', 'flap_lift': '0.3',
}


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('rokey1'), 'urdf', 'tape_roll.urdf.xacro')
    cmd = ['xacro ', xacro_file]
    for k in ARGS:
        cmd += [f' {k}:=', LaunchConfiguration(k)]

    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in ARGS.items()] + [
            # 로봇의 robot_state_publisher 와 겹치지 않게 'tape' 네임스페이스 사용
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace='tape',
                parameters=[{'robot_description':
                             ParameterValue(Command(cmd), value_type=str)}],
            ),
        ])
