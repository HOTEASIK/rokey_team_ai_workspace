"""퍼티 전체 공정: 초기화 → 헤라 집기 → (파지 확인) → 퍼티 뜨기 → 벽 도포 → 헤라 반납 → 초기화."""

# ===== 파일 역할 / 통신 안내 =====
# 액션 서버(putty_task_server) 하나를 띄우고, 단계마다 putty_task_client로 PuttyTask Goal을 보낸다.
# 앞 단계 클라이언트가 성공(종료 코드 0)으로 끝나야 다음 단계를 시작한다. 실패·취소면 그 자리에서 launch를 끝낸다.
# OnProcessExit는 운영체제 프로세스 종료 이벤트이며 ROS 토픽/서비스 메시지가 아니다.
# 실물 뜨기 Goal에는 '헤라 파지 확인(tool_held_confirmed)'이 필요하다(센서 판정 없음).
# 기본(confirm_grasp:=false)은 집기가 성공하면 잡혀 있다고 가정하고 확인 표시를 붙여 바로 뜨기로 넘어간다.
#   헤라가 실제로 없으면 뜨기의 벽 접촉 탐색이 힘을 찾지 못해 그 자리에서 멈춘다.
# confirm_grasp:=true면 집기 뒤에 터미널(/dev/tty)에서 y 입력을 받는다. 터미널이 없는 환경(HMI 등)에서는 멈춘다.
# 정지: Ctrl+C로 launch를 끝내면 실행 중인 클라이언트가 Goal을 취소하고, 서버가 정지·힘 해제 정리를 한다.
#       그래도 비상정지는 항상 손 닿는 곳에 둔다.

import math

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node

# 터미널에서 직접 y를 받는 확인 단계. launch 자식 프로세스는 표준입력이 없어서 /dev/tty를 직접 읽는다.
GRASP_PROMPT = ('read -r -p "[퍼티 공정] 헤라가 실제로 잡혀 있는지 확인하세요. 뜨기를 시작하려면 y 입력: " answer '
                '</dev/tty && [ "$answer" = y ]')


# 단계 목록을 순서대로 이어 붙인다. 앞 단계가 성공해야 다음 단계를 실행하고, 마지막 단계 뒤에는 launch를 끝낸다.
def chain(steps):
    handlers = []
    for index, (name, action) in enumerate(steps):
        following = steps[index+1] if index+1 < len(steps) else None

        def on_exit(event, context, name=name, following=following):
            if event.returncode != 0:
                return [EmitEvent(event=Shutdown(reason=f'{name} 실패(종료 코드 {event.returncode}); 이후 단계 생략'))]
            if following is None:
                return [LogInfo(msg=f'{name} 완료. 퍼티 전체 공정 완료'),
                        EmitEvent(event=Shutdown(reason='퍼티 전체 공정 완료'))]
            return [LogInfo(msg=f'{name} 완료 → {following[0]} 시작'), following[1]]

        handlers.append(RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit)))
    return handlers


# launch 인자를 읽어 (단계 이름, 실행 액션) 목록을 만든다. 실행은 하지 않는다.
def build_steps(context):
    def text(name):
        return LaunchConfiguration(name).perform(context)

    def boolean(name):
        value = text(name).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name}은 true 또는 false여야 합니다.')
        return value == 'true'

    def positive(name):
        value = float(text(name))
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다: {value}')
        return text(name)

    if not boolean('execute_motion'):
        raise ValueError('이동 실행에는 execute_motion:=true를 지정하세요.')
    virtual = boolean('virtual_mode')
    joint = ['--joint-speed', positive('joint_speed'), '--joint-acceleration', positive('joint_acceleration')]

    def task(name, *options, confirmed=False):
        command = [FindExecutable(name='ros2'), 'run', 'rokey1', 'putty_task_client', name]
        command += ['--virtual-mode'] if virtual else []
        command += ['--tool-held-confirmed'] if confirmed else []
        if options:
            command += ['--', *options]
        return ExecuteProcess(cmd=command, output='screen')

    # 실물 도포는 서버가 --contact-search를 붙이므로 바닥·벽 감지 힘을 반드시 준다.
    apply_options = [*joint, '--press-force', positive('press_force')]
    if not virtual:
        apply_options += ['--floor-force', positive('floor_force'), '--wall-force', positive('wall_force')]

    steps = [
        ('초기화', task('initialize')),
        ('헤라 집기', task('pickup', *joint, '--transfer-speed', positive('transfer_speed'))),
    ]
    if not virtual and boolean('confirm_grasp'):
        steps.append(('헤라 파지 확인', ExecuteProcess(cmd=['bash', '-c', GRASP_PROMPT], output='screen')))
    steps += [
        ('퍼티 뜨기', task('scoop', '--scoop-angle', positive('scoop_angle'), '--scoop-pass', text('scoop_pass'),
                        '--wall-scrape-height', text('wall_scrape_height'),
                        '--transfer-speed', positive('transfer_speed'), confirmed=not virtual)),
        ('벽 도포', task('apply', *apply_options)),
        ('헤라 반납', task('return', *joint)),
        ('마무리 초기화', task('initialize')),
    ]
    return steps, boolean('start_action_server')


# launch가 띄운 액션 서버가 먼저 끝나면(예: 다른 서버가 이미 떠 있어 중복 실행 잠금에 걸림) 공정 전체를 끝낸다.
# 그대로 두면 클라이언트가 예전에 떠 있던 다른 서버(옛 코드일 수 있음)에 Goal을 보내게 된다.
def stop_when_server_exits(event, context):
    return [EmitEvent(event=Shutdown(
        reason=f'퍼티 액션 서버 종료(코드 {event.returncode}). 이미 떠 있는 서버가 있으면 끄거나 '
               'start_action_server:=false로 실행하세요'))]


def launch_sequence(context):
    steps, start_server = build_steps(context)
    actions = []
    if start_server:
        server = Node(package='rokey1', executable='putty_task_server', output='screen', sigterm_timeout='30')
        actions += [server, RegisterEventHandler(OnProcessExit(target_action=server, on_exit=stop_when_server_exits))]
    return [*actions, *chain(steps), LogInfo(msg='[퍼티 공정] 초기화 시작'), steps[0][1]]


def generate_launch_description():
    defaults = {
        'execute_motion': ('false', '이동 실행 동의. false면 launch를 거부한다'),
        'virtual_mode': ('false', 'true면 가상 컨트롤러로 실행(파지 확인 단계 생략)'),
        'confirm_grasp': ('false', 'true면 집기 뒤 터미널에서 y로 헤라 파지를 확인. false는 잡혔다고 가정'),
        'start_action_server': ('true', '액션 서버를 함께 띄운다. 이미 떠 있으면 false'),
        'joint_speed': ('10.0', '집기·도포·반납 관절 이동 속도 deg/s'),
        'joint_acceleration': ('10.0', '관절 이동 가속도 deg/s²'),
        'transfer_speed': ('15.0', '집기·뜨기 직선 이송 속도 mm/s'),
        'scoop_angle': ('55.0', '뜨기 헤라 각도(바닥 기준). 45°는 어깨 특이점, 55~60 권장'),
        'scoop_pass': ('1', '몇 번째 뜨기인지 1~4 (끝점 +X (N-1)×20 mm)'),
        'wall_scrape_height': ('25.0', '뜨기 후 -X로 벽을 누르며 올라가는 높이 mm (0이면 바로 상승)'),
        'press_force': ('3.0', '도포 중 벽 누르는 힘 N'),
        'floor_force': ('5.0', '실물 도포 바닥 접촉 감지 힘 N'),
        'wall_force': ('5.0', '실물 도포 벽 접촉 감지 힘 N'),
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in defaults.items()),
        OpaqueFunction(function=launch_sequence),
    ])
