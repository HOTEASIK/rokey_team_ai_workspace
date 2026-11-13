"""wall_f 를 파라미터로 생성해서 RViz2 + 슬라이더(joint_state_publisher_gui)로 띄운다.

예)
  ros2 launch ./wall_f_rviz.launch.py
  ros2 launch ./wall_f_rviz.launch.py chamfer:=0.03
  ros2 launch ./wall_f_rviz.launch.py set:="ul.front_top=0.08 lr.back=0.0 ll.dz=0.1"
  ros2 launch ./wall_f_rviz.launch.py params:=/path/to/my_wall.json
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
import wall_f_gen  # noqa: E402


def _setup(context):
    params_file = LaunchConfiguration('params').perform(context) or None
    chamfer = LaunchConfiguration('chamfer').perform(context)
    sets = LaunchConfiguration('set').perform(context).split()

    p = wall_f_gen.make_params(params_file, float(chamfer) if chamfer else None, sets)
    out_urdf = os.path.join(HERE, 'meshes', 'launch', 'wall_f_launch.urdf')
    xml = wall_f_gen.build(p, out_urdf, os.path.join(HERE, 'meshes', 'launch'))

    rviz_cfg = os.path.join(HERE, 'wall_f.rviz')
    if not os.path.exists(rviz_cfg):
        from ament_index_python.packages import get_package_share_directory
        rviz_cfg = os.path.join(get_package_share_directory('urdf_tutorial'), 'rviz', 'urdf.rviz')

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': xml}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui',
             output='screen'),
        Node(package='rviz2', executable='rviz2', output='screen', arguments=['-d', rviz_cfg]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params', default_value='', description='JSON 파라미터 파일 경로'),
        DeclareLaunchArgument('chamfer', default_value='', description='모든 패널 8모서리 모따기 [m]'),
        DeclareLaunchArgument('set', default_value='',
                              description='공백 구분 KEY=VAL 목록 (예: "ul.front_top=0.05 ll.dz=0.1")'),
        OpaqueFunction(function=_setup),
    ])
