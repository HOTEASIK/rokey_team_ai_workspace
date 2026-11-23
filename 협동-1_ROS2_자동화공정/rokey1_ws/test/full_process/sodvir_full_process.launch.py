"""sodvir(에뮬레이터)로 HMI [전체 공정] 을 실제 서버들과 돌려 보는 테스트용 launch. DB 는 가짜(JSON).

  ros2 launch ~/ws_cobot_pjt/ws_dsr/src/rokey1/test/full_process/sodvir_full_process.launch.py
  → 브라우저 http://127.0.0.1:8090 → [sodvir] → [공정] 탭 [전체 공정] ▶ 시작

하는 일
  - 이 launch 안의 모든 노드(HMI 가 켜는 bringup·액션 서버 포함)를 이 PC 안에서만 통신하게 함
    (ROS_DISCOVERY_SERVER / ROS_SUPER_CLIENT 해제, ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
    → 다른 PC 의 실제 로봇·서버와 섞이지 않음
  - hmi_server 를 port 8090 + fake_db 로 실행 (MySQL 안 씀, 결과: 이 폴더 db_sodvir.json)
  - ROKEY_CONFIG_OVERRIDE=sodvir_override.yaml (가상 벽 틈 좌표만 덮어씀)
  - 액션 서버(explore / putty / sanding)는 [전체 공정] 시작 때 HMI 가 virtual 모드로 자동으로 켬
주의: 실제 로봇 bringup(sodreal)이 이 PC 에서 떠 있으면 먼저 끌 것. HMI 화면 모드는 '가상' 으로.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8090'),
        DeclareLaunchArgument('fake_db', default_value=os.path.join(HERE, 'db_sodvir.json')),
        UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER'),
        UnsetEnvironmentVariable('ROS_SUPER_CLIENT'),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('ROKEY_CONFIG_OVERRIDE', os.path.join(HERE, 'sodvir_override.yaml')),
        Node(package='rokey1', executable='hmi_server', output='screen',
             parameters=[{'port': LaunchConfiguration('port'), 'fake_db': LaunchConfiguration('fake_db')}]),
    ])
