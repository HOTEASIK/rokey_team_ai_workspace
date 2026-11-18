"""매니저 + rosbridge(웹 비상정지/상태 표시용, ws://<IP>:9090)

  ros2 launch ~/ws_cobot_pjt/ws_dsr/src/rokey1_ws/launch/manager.launch.py
  ros2 launch ~/ws_cobot_pjt/ws_dsr/src/rokey1_ws/launch/manager.launch.py fake:=true   # 가짜 공정 서버 포함

robot_state_publisher 는 bringup 이 이미 띄우므로 넣지 않는다.
공정 노드는 담당자 노드가 준비되면 여기에 추가.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false'),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml']))),
        Node(package='rokey1', executable='manager', output='screen'),
        Node(package='rokey1', executable='fake_process', output='screen',
             condition=IfCondition(LaunchConfiguration('fake'))),
    ])
