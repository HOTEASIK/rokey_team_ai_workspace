"""Initialize successfully, then pick up the spatula. Requires explicit execute_motion:=true."""

# ===== 파일 역할 / 통신 안내 =====
# launch는 초기화 Action 결과를 받은 클라이언트가 성공 종료한 뒤 픽업 Goal을 보낸다.
# OnProcessExit는 운영체제 프로세스 종료 이벤트이며 ROS 토픽/서비스 메시지가 아니다.
# 픽업에 좌표를 전달하는 통신은 없고, 각 실행 노드가 자신의 설정과 컨트롤러 조회를 사용한다.

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


def after_initialization(event, pickup):
    if event.returncode != 0:
        return [EmitEvent(event=Shutdown(reason='Initialization failed; pickup skipped'))]
    return [LogInfo(msg='Initialization completed; starting pickup'), pickup]


def launch_sequence(context):
    def boolean(name):
        value = LaunchConfiguration(name).perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false')
        return value == 'true'

    execute = boolean('execute_motion')
    if not execute:
        raise ValueError('이동 실행에는 execute_motion:=true를 지정하세요.')
    virtual = boolean('virtual_gripper')
    for name in ('transfer_speed', 'pick_speed', 'clearance', 'settle'):
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value) or value < 0 or (name != 'settle' and value == 0):
            raise ValueError(f'Invalid {name}: {value}')
    # 각 클라이언트는 PuttyTask 결과를 받은 뒤 종료한다.
    base = [FindExecutable(name='ros2'), 'run', 'rokey1', 'putty_task_client']
    mode = ['--virtual-mode'] if virtual else []
    init = ExecuteProcess(cmd=[*base, 'initialize', *mode], output='screen')
    command = [*base, 'pickup', *mode, '--']
    for option in ('transfer-speed', 'pick-speed', 'clearance', 'settle'):
        command.extend([f'--{option}', LaunchConfiguration(option.replace('-', '_'))])
    pickup = ExecuteProcess(cmd=command, output='screen')
    server = ([Node(package='rokey1', executable='putty_task_server', output='screen',
                    sigterm_timeout='30')] if boolean('start_action_server') else [])
    return [
        *server,
        RegisterEventHandler(OnProcessExit(
            target_action=init,
            on_exit=lambda event, ctx: after_initialization(event, pickup),
        )),
        init,
    ]


def generate_launch_description():
    defaults = {
        'execute_motion': ('false', 'Required execution opt-in; false rejects launch'),
        'start_action_server': ('true', 'Start one putty action server; false if already running'),
        'virtual_gripper': ('false', 'Use /onrobot/sendCommand instead of physical DO'),
        'transfer_speed': ('15.0', 'Pickup transfer speed (mm/s)'),
        'pick_speed': ('5.0', 'Pickup descent/lift speed (mm/s)'),
        'clearance': ('50.0', 'Pickup clearance (mm)'),
        'settle': ('1.0', 'Gripper settling time (s)'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=launch_sequence),
    ])
