"""가상 초기화 → 집기 → 찍기 → J6 도포 → 반납 → 홈 복귀."""

# ===== 파일 역할 / 통신 안내 =====
# 전체 순서: 초기화 → 집기 → 스쿱 → 평면 도포 → 반납 → 초기화.
# 각 단계는 PuttyTask Action을 요청한다. 결과 확인 후 클라이언트가 종료하면 다음 Goal을 보낸다.
# 옵션은 명령행 인자/ROS 파라미터로 전달하고 실제 로봇 상태는 각 노드가 서비스로 조회한다.
# 현재 전체 launch는 가상 집게 및 기존 geometry 스쿱 경로를 사용하며, 실물 전체 자동화 경로가 아니다.

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


DEFAULTS = {
    'execute_motion': 'false',
    'start_action_server': 'true',
    'plane': 'xy',
    'normal_sign': 'positive',
    'start': '695.42 -223.46 35',
    'end': '574.26 -227.09 35',
    'blade_angle': '45',
    'blade_end_angle': '30',
    'approach_distance': '50',
    'transfer_speed': '30',
    'pick_speed': '5',
    'scoop_speed': '15',
    'scoop_angle': '45',
    'scoop_depth': '3',
    'scoop_distance': '10',
    'apply_speed': '15',
    'place_speed': '10',
    'transfer_z': '245.14',
    'surface_z': '35',
    'settle': '1',
}


def after_step(event, next_action):
    if event.returncode != 0:
        return [LogInfo(msg=f'공정 실패 (exit {event.returncode}); 후속 단계 중단'),
                EmitEvent(event=Shutdown(reason='Putty cycle stage failed'))]
    if next_action is None:
        return [LogInfo(msg='퍼티 전체 공정 완료'),
                EmitEvent(event=Shutdown(reason='Putty cycle completed'))]
    return [next_action]


def launch_sequence(context):
    values = {name: LaunchConfiguration(name).perform(context) for name in DEFAULTS}
    if values['execute_motion'].lower() not in ('true', 'false'):
        raise ValueError('execute_motion must be true or false')
    execute = values['execute_motion'].lower() == 'true'
    if not execute:
        raise ValueError('이동 실행에는 execute_motion:=true를 지정하세요.')
    for name, value in values.items():
        if name in ('execute_motion', 'start_action_server', 'plane', 'normal_sign', 'start', 'end'):
            continue
        number = float(value)
        if not math.isfinite(number):
            raise ValueError(f'Invalid {name}')
        if name not in ('surface_z', 'settle') and number <= 0:
            raise ValueError(f'{name} must be positive')
    if not 0 < float(values['scoop_angle']) < 90 or float(values['settle']) < 0:
        raise ValueError('Invalid scoop_angle or settle')

    def process(executable, options=(), extra=()):
        tasks = {'pickup_spatula': 'pickup', 'scoop_putty': 'scoop',
                 'apply_putty_plane': 'apply', 'return_spatula': 'return'}
        # 가상 전체 공정의 기존 수치/경로는 그대로 전달한다.
        extra = tuple(value for value in extra if value != '--virtual-gripper')
        cmd = [FindExecutable(name='ros2'), 'run', 'rokey1', 'putty_task_client',
               tasks[executable], '--virtual-mode', '--', *extra]
        for name in options:
            cmd.extend(['--'+name.replace('_', '-'), values[name]])
        return ExecuteProcess(cmd=cmd, output='screen')

    def initialize():
        return ExecuteProcess(cmd=[FindExecutable(name='ros2'), 'run', 'rokey1',
                                   'putty_task_client', 'initialize', '--virtual-mode'], output='screen')

    # 시작 전에 평면 입력을 검증해 초기화 후 잘못된 입력으로 중단하지 않게 한다.
    from types import SimpleNamespace
    from putty_demo.apply_putty_plane import validate
    options = dict(values)
    for name in ('transfer_speed', 'apply_speed',
                 'transfer_z', 'surface_z', 'blade_angle', 'blade_end_angle', 'approach_distance'):
        options[name] = float(options[name])
    options.update(acceleration=20.0, edge_offset=(0, 0, 126),
                   start=[float(v) for v in values['start'].split()],
                   end=[float(v) for v in values['end'].split()])
    if len(options['start']) != 3 or len(options['end']) != 3:
        raise ValueError('start/end require three BASE coordinates')
    validate(SimpleNamespace(**options))
    applying = process('apply_putty_plane',
                       ('transfer_speed', 'apply_speed',
                        'transfer_z', 'plane', 'normal_sign', 'blade_angle', 'blade_end_angle',
                        'approach_distance'),
                       ('--start', *values['start'].split(), '--end', *values['end'].split()))

    stages = [
        initialize(),
        process('pickup_spatula', ('transfer_speed', 'pick_speed', 'settle'),
                ('--virtual-gripper',)),
        process('scoop_putty', ('transfer_speed', 'scoop_speed', 'scoop_angle',
                               'scoop_depth', 'scoop_distance', 'transfer_z'),
                ('--path', 'geometry')),
        applying,
        process('return_spatula', ('transfer_speed', 'place_speed', 'transfer_z',
                                  'surface_z', 'settle')),
        initialize(),
    ]
    handlers = []
    for index, stage in enumerate(stages):
        following = stages[index+1] if index+1 < len(stages) else None
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=stage,
            on_exit=lambda event, ctx, following=following: after_step(event, following),
        )))
    start_server = values['start_action_server'].lower()
    if start_server not in ('true', 'false'):
        raise ValueError('start_action_server must be true or false')
    server = ([Node(package='rokey1', executable='putty_task_server', output='screen',
                    sigterm_timeout='30')] if start_server == 'true' else [])
    return [*server, *handlers, LogInfo(msg='가상 퍼티 Action 공정 시작'), stages[0]]


def generate_launch_description():
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in DEFAULTS.items()),
        OpaqueFunction(function=launch_sequence),
    ])
