"""웹 HMI(공정 실행 + DB 기록) 실행. fake:=true 면 가짜 /process/* 액션 서버도 같이 띄운다.

  ros2 launch ~/ws_cobot_pjt/ws_dsr/src/rokey1/launch/process.launch.py fake:=true      # 로봇 없이 DB 흐름 확인
  ros2 launch ~/ws_cobot_pjt/ws_dsr/src/rokey1/launch/process.launch.py host:=0.0.0.0   # 다른 PC 에서 접속 허용
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fake', default_value='false'),
        DeclareLaunchArgument('host', default_value='127.0.0.1'),
        Node(package='rokey1', executable='hmi_server', output='screen',
             parameters=[{'host': LaunchConfiguration('host')}]),
        Node(package='rokey1', executable='fake_process', output='screen',
             condition=IfCondition(LaunchConfiguration('fake'))),
    ])
