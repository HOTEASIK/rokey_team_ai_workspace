"""Return/release the spatula, then initialize. Motion is virtual-only."""

# ===== 파일 역할 / 통신 안내 =====
# launch는 반납 성공 후 초기화(홈 복귀)를 실행한다. 실패하면 다음 프로세스를 시작하지 않는다.
# 각 노드는 ROS 서비스로 컨트롤러와 통신하고, launch는 프로세스 종료 코드만 확인한다.
# 현재 이 반납 launch는 가상 집게와 가상 컨트롤러 경로를 사용한다.

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def after_return(event, initialize):
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason='Return failed; initialization skipped'))]
    return [LogInfo(msg='Return and release completed; starting initialization'), initialize]


def launch_sequence(context):
    value = LaunchConfiguration('execute_motion').perform(context).lower()
    if value not in ('true', 'false'):
        raise ValueError('execute_motion must be true or false')
    execute = value == 'true'
    if not execute:
        raise ValueError('이동 실행에는 execute_motion:=true를 지정하세요.')
    for name in ('transfer_speed', 'place_speed', 'transfer_z', 'settle'):
        number = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(number) or number < 0 or (name != 'settle' and number == 0):
            raise ValueError(f'Invalid {name}: {number}')
    command = [FindExecutable(name='ros2'), 'run', 'rokey1', 'return_spatula',
               '--execute-motion']
    for option in ('transfer-speed', 'place-speed', 'transfer-z', 'settle'):
        command.extend([f'--{option}', LaunchConfiguration(option.replace('-', '_'))])
    returning = ExecuteProcess(cmd=command, output='screen')
    initialize = Node(
        package='rokey1', executable='initialize_device', output='screen',
        parameters=[{'execute_motion': True, 'virtual_gripper': True}],
    )
    return [
        RegisterEventHandler(OnProcessExit(
            target_action=returning,
            on_exit=lambda event, ctx: after_return(event, initialize),
        )),
        returning,
    ]


def generate_launch_description():
    defaults = {
        'execute_motion': ('false', 'Required execution opt-in on virtual controller'),
        'transfer_speed': ('30.0', 'Return transfer speed (mm/s)'),
        'place_speed': ('10.0', 'Return descent/retreat speed (mm/s)'),
        'transfer_z': ('245.14', 'Minimum transfer height (BASE mm)'),
        'settle': ('1.0', 'Wait after gripper opening (s)'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=launch_sequence),
    ])
