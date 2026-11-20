"""평면 도포의 내부 ROS 서비스 실행부. 단독 실행 노드가 아니다."""

# ===== 파일 역할 / 통신 안내 =====
# 내부 공통 실행 모듈: import만으로 로봇을 움직이지 않는다.
# run_robot() 호출 시 평면 도포 ROS 노드를 만들고 dsr_msgs2 서비스를 직접 호출한다.
# 서비스 공통 경로는 /dsr01/dsr_controller2.
#
# DSR_ROBOT2 함수 대신 서비스를 직접 부르는 이유: DSR_ROBOT2는 응답을 시간 제한 없이 기다리므로
# 이동 중 취소·시간 초과·특이점 알람 확인을 할 수 없다. 대응하는 두산 API는 각 내부 함수 주석에 적었다.
#
# 실행 순서: 시스템 확인 → Tool Weight/TCP 선택 → 특이점 정책·감시 → 출발 전 관절(ikin) 검사 → 경로 실행.
# 접근·후퇴는 MoveLine(movel). 힘 없이 도포하면 MoveSplineTask(movesx) 한 번으로 보낸다.
# 컴플라이언스 중에는 컨트롤러가 MoveSplineTask를 거부하므로 비동기 MoveLine(amovel) 구간으로 나눠 보낸다.
# 힘 조절 루프 자체는 로봇 컨트롤러가 수행하고, 이 모듈은 외력을 조회해 이상 여부만 판정한다.

import argparse
import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace
import time

from .contact_search import (
    WallForceHold, PressForceMonitor, wait_for_press_force, checked_force, press_force_request,
    force_limit, position_press, press_stiffness, press_tolerance, position_press_limit,
    direct_press,
)
from .singularity_guard import SingularityDetected, start_singularity_watch

from .putty_geometry import (
    EXPECTED_TCP, ROBOT_ID, TRANSFER_Z, format_values, orientation_error,
)


# ===== 하이퍼파라미터 =====
# 실행 전 관절 검사(샌딩 reach_check와 같은 ikin 방식). 실측: J3=39.0°에서 3205 알람, 44.1°는 통과.
PREFLIGHT_J3_MIN_DEG = 41.0   # 팔꿈치(팔 펴짐) 특이점 여유
PREFLIGHT_J5_MIN_DEG = 10.0   # 손목 특이점 여유
# 어깨 특이점: 손목 중심(J5 축)이 베이스 회전축(J1) 가까이 오면 J1이 정해지지 않는다.
# 2026-11-19 실측: 뜨기 45°의 사선 시작점에서 손목 중심이 축에서 약 132 mm → 3205(특이점 진입) 후
# 다음 movel이 1206(NOT REACHABLE)로 거부됐다. 55°(약 219 mm)부터 여유가 있다.
PREFLIGHT_WRIST_RADIUS_MIN_MM = 180.0
UPPER_ARM_MM, FOREARM_MM = 411.0, 368.0   # m0609 xacro: J2→J3, J3→J5(손목 중심)


# 관절값으로 손목 중심(J5 축)과 베이스 회전축(J1) 사이 수평 거리를 구한다.
# J2는 위팔이 수직에서 앞으로 기운 각, J2+J3는 아래팔이 수직에서 기운 각이다(홈 J2=0, J3=90 → 368 mm).
def wrist_radius(joints):
    j2, j3 = math.radians(joints[1]), math.radians(joints[2])
    return abs(UPPER_ARM_MM*math.sin(j2)+FOREARM_MM*math.sin(j2+j3))
from utils.robot import TOOL_NAME  # 펜던트에 등록된 공구 무게 설정 (config/robot.yaml, 샌딩과 공용)
# direct/hybrid 도포 중 계획 대비 X 편차 한계. X는 힘제어가 벽을 따라가므로 벽이 기울면 계획과
# 수십 mm 벌어질 수 있다(실측: 위로 갈수록 벽이 멀어져 Z≈250에서 30 mm 초과). 폭주 방지용 한계다.
X_FOLLOW_LIMIT_MM = 60.0
# 이 설명의 이동은 직선(movel)이 아니라 관절 이동(movej)으로 실행한다.
# 퍼티를 뜬 뒤 로봇 뒤쪽(J1≈150~170°)에서 벽 앞(J1≈0°)으로 오는 긴 이동이라, 직선이면 베이스 옆을 지난다.
# 위에서 볼 때 시계 방향(J1 감소)으로 돌고, 같은 이동 안에서 J6이 약 180° 돌아 퍼티 면이 +X 벽을 향한다.
WALL_TRANSFER_STEP = '벽 앞으로 관절 이동하며 자세 정렬'
# 'nearest': 뒤쪽(J1≈150°)에서 바로 오면 시계(cw)와 같고, 홈(J1≈0°)을 거쳐 오면 벽 위치(+Y면 J1>0)에 따라
# 몇 도만 돈다. 'cw' 고정이면 +Y 틈(J1≈+10°)에서 348.8° 를 돌아야 해 멈췄다 (2026-11-20 run102 2번째 틈 Y=120).
WALL_TRANSFER_DIRECTION = 'nearest'
HOME_TRANSFER_DIRECTION = 'cw'
# 로봇 뒤쪽(용기·보관대)에서 출발할 때: 기둥(보관대 옆, 방위 약 128°)을 피하려고 경유지까지 직선으로 온 뒤
# 이 설명의 이동으로 홈 위치 쪽(J1≈0°)까지 시계 방향 관절 이동한다. 헤라 자세(뜬 각도)는 바꾸지 않는다.
HOME_TRANSFER_STEP = '홈 위치로 관절 이동(뜬 자세 유지)'
JOINT_TRANSFER_STEPS = (HOME_TRANSFER_STEP, WALL_TRANSFER_STEP)
FLOOR_SEARCH_STIFFNESS_Z = 1000.0  # 바닥 탐색 중 Z 강성. 닿아도 1 mm당 1 N씩 올라 충돌 감지(7060) 회피
HYBRID_STIFFNESS_X = 200.0  # hybrid: 벽 감지 후 힘제어 축(BASE X) 강성. 샌딩의 누르는 축과 같은 값


# 내부 데이터 묶음이다. ROS 메시지/Action 타입이 아니며 TCP 자세 목록과 속도를 보관한다.
@dataclass(frozen=True)
class SplineStroke:
    label: str
    poses: tuple
    speed: float


# 여러 도포 점을 하나의 SplineStroke로 묶어 점마다 정지하지 않게 한다.
def continuous_commands(moves):
    """연속 도포 점만 한 요청으로 묶고 접근·후퇴와 다른 구간은 분리한다."""
    index = 0
    previous = None
    while index < len(moves):
        label, target, speed = moves[index]
        if not any(name in label for name in ('평면 각도 변화 도포', '평면 고정 각도 도포')):
            yield moves[index]
            previous = target
            index += 1
            continue
        if previous is None:
            raise ValueError('스플라인 도포에는 접촉 시작 자세가 필요합니다.')
        points = [tuple(previous)]
        while index < len(moves):
            step, point, velocity = moves[index]
            if not any(name in step for name in ('평면 각도 변화 도포', '평면 고정 각도 도포')) or velocity != speed:
                break
            points.append(tuple(point))
            index += 1
        if not 2 <= len(points) <= 100:
            raise ValueError('한 번의 연속 도포는 시작점을 포함해 2~100개 자세여야 합니다.')
        if any(len(p) != 6 or not all(map(math.isfinite, p)) for p in points):
            raise ValueError('유효하지 않은 스플라인 자세입니다.')
        yield SplineStroke(label, tuple(points), speed)
        previous = points[-1]


# 각 TCP 자세를 Float64MultiArray로 넣어 MoveSplineTask 요청을 만든다.
def spline_request(points, speed, acceleration):
    from dsr_msgs2.srv import MoveSplineTask
    from std_msgs.msg import Float64MultiArray

    request = MoveSplineTask.Request()
    request.pos = [Float64MultiArray(data=list(point)) for point in points]
    request.pos_cnt = len(points)
    request.vel = [speed, 10.0]
    request.acc = [acceleration, 20.0]
    request.ref = request.mode = request.sync_type = request.opt = 0
    return request


# 힘 제어 중에는 X 위치를 자유롭게 두므로 Y/Z와 자세로 도착을 판정한다.
def stroke_arrived(actual, target, force_active=False):
    """X 힘 제어 시 X 위치 오차를 완료 조건으로 쓰지 않는다."""
    position_error = (math.dist(actual[1:3], target[1:3]) if force_active
                      else math.dist(actual[:3], target[:3]))
    # 컴플라이언스 중에는 회전 강성이 낮아 접촉 반력으로 자세가 조금 비틀릴 수 있다.
    angle_tolerance = 5.0 if force_active else 2.0
    # 힘제어 중에는 접촉 마찰로 Y/Z도 몇 mm 뒤처질 수 있다(실측 Z 2.5 mm 부족으로 실패한 사례).
    position_tolerance = 5.0 if force_active else 2.0
    return position_error <= position_tolerance and orientation_error(actual[3:], target[3:]) <= angle_tolerance


# 평면 도포의 기본 CLI 옵션(속도·이송 높이·날 오프셋)을 정의한다. 나머지는 apply_putty_plane이 더한다.
def make_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True,
                        help='real/virtual 컨트롤러에 이동 요청')
    parser.add_argument('--transfer-speed', type=float, default=30.0, help='mm/s')
    parser.add_argument('--apply-speed', type=float, default=10.0, help='mm/s')
    parser.add_argument('--acceleration', type=float, default=20.0, help='mm/s²')
    parser.add_argument('--transfer-z', type=float, default=TRANSFER_Z)
    # 기본값은 apply_putty_plane이 실측 헤라 길이(WALL_EDGE_OFFSET)로 정한다.
    parser.add_argument('--edge-offset', type=float, nargs=3, help='TCP → 날 끝 Tool 오프셋 mm')
    return parser


# 숫자 범위·좌표·길이 등이 유효한지 확인한다. 역기구학이나 충돌 검사는 아니다.
def validate(args):
    names = ('transfer_speed', 'apply_speed', 'acceleration')
    for name in names:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다.')
    values = (args.transfer_z, *args.edge_offset)
    if not all(map(math.isfinite, values)):
        raise ValueError('높이·오프셋은 유한한 값이어야 합니다.')


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args, motion_builder, allow_real=False,
              continuous=False, contact_probe=None, press_force=None):
    import rclpy
    from utils.shutdown import safe_shutdown
    from dsr_msgs2.srv import (
        GetRobotSystem, GetCurrentTcp, GetCurrentPosx, MoveLine,
    )
    from dsr_msgs2.srv import SetCurrentTcp, SetRobotMode, GetCurrentTool, SetCurrentTool
    from utils.service_waiter import ServiceWaiter

    rclpy.init(args=[])
    node = None
    hold = None
    force_monitor = None
    watch = None
    cleaning_up = False
    motion_started = False
    guard_enabled = getattr(args, 'singularity_guard', False)
    try:
        node = rclpy.create_node('apply_putty_plane',
                                 namespace=ROBOT_ID)
        # 이 노드는 클라이언트다. 아래 create_client는 서버를 만들지 않고 DSR 서버에 연결할 창구를 만든다.
        prefix = f'/{ROBOT_ID}/dsr_controller2'
        system = node.create_client(GetRobotSystem, f'{prefix}/system/get_robot_system')
        tcp = node.create_client(GetCurrentTcp, f'{prefix}/tcp/get_current_tcp')
        set_tcp = node.create_client(SetCurrentTcp, f'{prefix}/tcp/set_current_tcp')
        pose = node.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx')
        line = node.create_client(MoveLine, f'{prefix}/motion/move_line')
        robot_mode = node.create_client(SetRobotMode, f'{prefix}/system/set_robot_mode')
        get_tool = node.create_client(GetCurrentTool, f'{prefix}/tool/get_current_tool')
        set_tool = node.create_client(SetCurrentTool, f'{prefix}/tool/set_current_tool')
        from .pickup_spatula import create_joint_clients, move_joint_to_pose
        joint_clients = create_joint_clients(node, prefix)  # 벽 앞 관절 이동(ikin + movej)용
        clients = (system, tcp, set_tcp, robot_mode, get_tool, set_tool, pose, line,
                   joint_clients.joints, joint_clients.solution, joint_clients.ikin, joint_clients.joint)
        if continuous:
            from dsr_msgs2.srv import MoveSplineTask
            spline = node.create_client(MoveSplineTask, f'{prefix}/motion/move_spline_task')
            clients += (spline,)
        if contact_probe is not None or guard_enabled:
            from dsr_msgs2.srv import GetToolForce, MoveStop, GetRobotState
            force = node.create_client(GetToolForce, f'{prefix}/aux_control/get_tool_force')
            stop = node.create_client(MoveStop, f'{prefix}/motion/move_stop')
            state = node.create_client(GetRobotState, f'{prefix}/system/get_robot_state')
            clients += (force, stop, state)
        if guard_enabled:
            from dsr_msgs2.srv import SetSingularityHandling, SetSingularHandlingForce
            avoid = node.create_client(SetSingularityHandling, f'{prefix}/motion/set_singularity_handling')
            force_policy = node.create_client(SetSingularHandlingForce,
                                             f'{prefix}/motion/set_singular_handling_force')
            for client in (avoid, force_policy):
                if not client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError('특이점 정책 서비스 없음. 이동하지 않습니다.')
        if press_force is not None:
            if contact_probe is None:
                raise ValueError('도포 힘 유지는 접촉 탐색이 먼저 필요합니다.')
            from dsr_msgs2.srv import (
                TaskComplianceCtrl, SetDesiredForce, ReleaseForce, ReleaseComplianceCtrl,
            )
            compliance = node.create_client(TaskComplianceCtrl, f'{prefix}/force/task_compliance_ctrl')
            desired = node.create_client(SetDesiredForce, f'{prefix}/force/set_desired_force')
            release = node.create_client(ReleaseForce, f'{prefix}/force/release_force')
            release_comp = node.create_client(ReleaseComplianceCtrl, f'{prefix}/force/release_compliance_ctrl')
            from dsr_msgs2.srv import CheckMotion, SetStiffnessx
            motion_check = node.create_client(CheckMotion, f'{prefix}/motion/check_motion')
            stiffness_client = node.create_client(SetStiffnessx, f'{prefix}/force/set_stiffnessx')
            clients += (compliance, desired, release, release_comp, motion_check, stiffness_client)
        ServiceWaiter(node).wait(*clients)

        def call(client, request, step, timeout=5.0):
            if watch is not None and not cleaning_up:
                watch.check()
            # 요청 전송은 비동기지만, 바로 아래 spin으로 응답을 기다려 작업 순서를 보장한다.
            future = client.call_async(request)
            deadline = time.monotonic()+timeout
            # 긴 동기 이동 대기도 짧게 나눠, 별도 스레드가 받은 특이점 알람에 반응한다.
            while not future.done():
                if watch is not None and not cleaning_up:
                    watch.check()
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f'{step} 응답 시간 초과. 정지 절차를 확인하세요.')
                rclpy.spin_until_future_complete(node, future, timeout_sec=min(0.05, remaining))
            if watch is not None and not cleaning_up:
                watch.check()
            result = future.result()
            # success는 서비스 처리 결과다. 실제 힘 도달/실제 파지는 별도로 확인해야 한다.
            if result is None or not result.success:
                raise RuntimeError(f'{step} 서비스 실패. 컨트롤러 알람을 확인하세요.')
            return result

        def values(data):
            result = tuple(data)
            if len(result) != 6 or not all(map(math.isfinite, result)):
                raise RuntimeError('유효한 6축 좌표를 조회하지 못했습니다.')
            return result

        # 두산 API: get_current_posx(ref=DR_BASE). 현재 TCP의 BASE XYZABC.
        def read_pose(timeout=5.0):
            request = GetCurrentPosx.Request()
            request.ref = 0
            rows = call(pose, request, 'BASE TCP 조회', timeout=timeout).task_pos_info
            return values(rows[0].data[:6] if rows else ())

        def preflight_joints(commands):
            """샌딩 reach_check처럼 계획 점마다 ikin으로 관절각을 계산해 특이점 근처면 출발하지 않는다.

            계획 경로 기준이다. 힘제어로 X가 벽을 따라가며 생기는 편차는 실행 중 알람 감시가 맡는다.
            """
            from dsr_msgs2.srv import Ikin, GetCurrentSolutionSpace
            ik = node.create_client(Ikin, f'{prefix}/motion/ikin')
            space = node.create_client(GetCurrentSolutionSpace, f'{prefix}/aux_control/get_current_solution_space')
            ServiceWaiter(node).wait(ik, space)
            sol = call(space, GetCurrentSolutionSpace.Request(), '현재 팔 형태 조회').sol_space
            points = []
            for command in commands:
                if isinstance(command, SplineStroke):
                    points += [(f'{command.label} {i}/{len(command.poses)-1}', p)
                               for i, p in enumerate(command.poses[1:], 1)]
                else:
                    points.append((command[0], command[1]))
            worst = None
            for name, pose in points:
                request = Ikin.Request()
                request.pos, request.sol_space, request.ref = [float(v) for v in pose], sol, 0
                joints = list(call(ik, request, f'{name} 관절 계산').conv_posj)
                j3, j5 = abs(joints[2]), abs(joints[4])
                if worst is None or j3 < worst[1]:
                    worst = (name, j3, joints)
                text = ' '.join(f'J{i+1}={v:.1f}' for i, v in enumerate(joints))
                radius = wrist_radius(joints)
                if radius < PREFLIGHT_WRIST_RADIUS_MIN_MM:
                    raise RuntimeError(f'[관절 검사] {name} BASE [{format_values(pose)}]에서 어깨 특이점 근처 '
                                       f'[{text}] (손목 중심이 베이스 축에서 {radius:.0f} mm, '
                                       f'기준 {PREFLIGHT_WRIST_RADIUS_MIN_MM:g} mm 이상). 로봇은 움직이지 않았습니다.')
                if j3 < PREFLIGHT_J3_MIN_DEG or j5 < PREFLIGHT_J5_MIN_DEG:
                    kind = '팔꿈치(J3)' if j3 < PREFLIGHT_J3_MIN_DEG else '손목(J5)'
                    raise RuntimeError(f'[관절 검사] {name} BASE [{format_values(pose)}]에서 {kind} 특이점 근처 '
                                       f'[{text}] (기준 J3≥{PREFLIGHT_J3_MIN_DEG:g}°, J5≥{PREFLIGHT_J5_MIN_DEG:g}°). '
                                       '로봇은 움직이지 않았습니다.')
            print(f'[관절 검사] {len(points)}점 통과 (sol {sol}) | J3 최소 {worst[1]:.1f}° @ {worst[0]}', flush=True)

        def joints_text():
            return watch.joint_text() if watch is not None else '관절각 감시 없음'

        # 두산 API: movel(sync). 이동이 끝날 때까지 기다린 뒤 실제 도착(2 mm / 2°)을 다시 확인한다.
        def move_line(step, target, speed, tolerance=2.0):
            nonlocal motion_started
            print(f'[{step}] BASE [{format_values(target)}] | {speed:g} mm/s | 출발 관절각 [{joints_text()}]',
                  flush=True)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = request.sync_type = 0
            motion_started = True  # 응답 전 오류여도 명령 수신 여부가 불확실하므로 정지한다.
            call(line, request, step, timeout=120.0)
            actual = read_pose()
            if (math.dist(actual[:3], target[:3]) > tolerance or
                    orientation_error(actual[3:], target[3:]) > 2.0):
                raise RuntimeError(f'{step} 목표 미도착 (허용 {tolerance:g} mm): {actual}')

        # 두산 API: get_tool_force(DR_BASE). BASE 기준 [Fx,Fy,Fz,Mx,My,Mz]. 로봇을 움직이지 않는다.
        def read_force():
            request = GetToolForce.Request()
            request.ref = 0
            return call(force, request, 'BASE 외력 조회', timeout=0.5).tool_force

        log_started = time.monotonic()

        def log_force(stage, actual=None, measured=None, planned_angle=None):
            """진단용 순간값. 추가 조회 실패가 기존 이동/힘 제어 흐름을 바꾸지 않게 한다.

            위치와 힘은 순차 조회이며 동시 측정이 아니다. 목표 힘 도달이나
            컨트롤러 힘 제어 활성 여부를 판정하는 함수는 아니다.
            """
            try:
                actual = read_pose(timeout=0.5) if actual is None else actual
                measured = checked_force(read_force() if measured is None else measured)
                active = bool(hold and hold.active)
                target = f'{press_force:g}' if active else '미설정'
                error = f'{press_force-abs(measured[0]):+.3f}' if active else '해당없음'
                angle = '' if planned_angle is None else f' | 계획날각={planned_angle:.2f} deg'
                node.get_logger().info(
                    f'[힘진단] t={time.monotonic()-log_started:.3f}s | 단계={stage}'
                    f' | 힘설정응답확인={active} | 목표BASE_Fx={target} N'
                    f' | 실측BASE_Fxyz=[{format_values(measured[:3])}] N'
                    f' | BASE_Mxyz=[{format_values(measured[3:])}] Nm'
                    f' | 실측|Fx|={abs(measured[0]):.3f} N'
                    f' | 크기차이(목표-|실측|)={error} N'
                    f' | TCP_XYZ=[{format_values(actual[:3])}] mm'
                    f' | TCP_ABC=[{format_values(actual[3:])}] deg{angle}'
                    f' | 관절=[{joints_text()}]')
            except SingularityDetected:
                raise
            except Exception as exc:
                node.get_logger().warning(f'[힘진단] 단계={stage} 로그 조회 실패: {exc}')

        # 두산 API: amovel. sync_type=1은 이동 완료를 기다리지 않는다는 뜻이다. 이동 중 힘을 감시하려고 쓴다.
        def move_async(target, speed, step='비동기 직선 이동', timeout=3.0):
            # 컴플라이언스 중에는 컨트롤러 응답이 0.5 s를 넘을 수 있어 3 s까지 기다린다.
            nonlocal motion_started
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = 0
            request.sync_type = 1
            motion_started = True
            call(line, request, step, timeout=timeout)

        # 두산 API: stop(DR_HOLD). ROS2 DSR_ROBOT2에는 stop 래퍼가 없어 MoveStop 서비스를 직접 부른다.
        # 정지 요청 후 STANDBY가 될 때까지 확인한다.
        def stop_and_wait():
            request = MoveStop.Request()
            request.stop_mode = 3  # DR_HOLD
            call(stop, request, '접촉 탐색 정지', timeout=2.0)
            deadline = time.monotonic()+5
            while time.monotonic() < deadline:
                status = call(state, GetRobotState.Request(), '정지 상태 확인', timeout=0.5).robot_state
                if status == 1:  # STANDBY
                    return
                if status != 2:  # MOVING 이외 보호정지 등은 자동 복구하지 않는다.
                    raise RuntimeError(f'탐색 정지 후 로봇 상태 {status}. 다음 이동을 생략합니다.')
                time.sleep(0.05)
            raise TimeoutError('탐색 정지 완료를 확인하지 못했습니다.')

        # 두산 API: check_motion.
        def motion_idle():
            # 컴플라이언스 중에는 GetRobotState가 이동 중에도 STANDBY를 줄 수 있어
            # 이동 완료는 check_motion(0=IDLE, 1=계산 중, 2=이동 중)으로 판정한다(샌딩과 동일).
            from dsr_msgs2.srv import CheckMotion
            return call(motion_check, CheckMotion.Request(), '이동 상태 확인', timeout=0.5).status == 0

        # 두산 API: get_robot_state. 1=STANDBY, 2=MOVING. 컴플라이언스 중 이동 완료 판정에는 쓰지 않는다.
        def read_state():
            return call(state, GetRobotState.Request(), '로봇 상태 조회', timeout=0.5).robot_state

        # 두산 API: task_compliance_ctrl(stx, time=0.3). BASE 기준 6축 강성 N/m, Nm/rad.
        def start_compliance(stiffness):
            request = TaskComplianceCtrl.Request()
            request.stx = list(stiffness)
            request.ref, request.time = 0, 0.3
            call(compliance, request, 'BASE 컴플라이언스 시작')

        # 두산 API: set_desired_force(fd=[value,0,0,0,0,0], dir=[1,0,0,0,0,0], mod). BASE +X만 힘제어한다.
        def set_press_force(value):
            mode = 0 if getattr(args, 'force_mode', 'relative') == 'absolute' else 1
            request = press_force_request(value, mode=mode)
            call(desired, request, f'BASE +X {value:g} N 힘 유지')

        # 두산 API: release_force(time=0.3) / release_compliance_ctrl(). 힘 → 컴플라이언스 순서로 해제한다.
        def release_press_force():
            request = ReleaseForce.Request()
            request.time = 0.3
            call(release, request, '목표 힘 해제')

        def release_compliance():
            call(release_comp, ReleaseComplianceCtrl.Request(), '컴플라이언스 해제')

        robot_system = call(system, GetRobotSystem.Request(), '시스템 확인').robot_system
        if robot_system not in (0, 1):
            raise RuntimeError(f'지원하지 않는 로봇 시스템: {robot_system}')
        if robot_system == 0 and not allow_real:
            raise RuntimeError('이 도포 실행은 가상 컨트롤러에서만 허용합니다.')
        # 두산 API: get_tool / get_tcp.
        def read_tool():
            return str(call(get_tool, GetCurrentTool.Request(), 'Tool 확인').info).strip()

        def read_tcp():
            return str(call(tcp, GetCurrentTcp.Request(), 'TCP 확인').info).strip()

        def set_mode(mode, label):
            request = SetRobotMode.Request()
            request.robot_mode = mode
            call(robot_mode, request, f'{label} 모드 전환')
            time.sleep(0.5)

        # 두산 API: set_tool / set_tcp. Tool Weight는 외력 추정에 쓰는 공구 무게라서, 틀리면 힘 감지와
        # 힘제어가 모두 어긋난다(예전에 힘이 형성되지 않던 원인 중 하나). 실물 샌딩과 같은 순서로 바꾼다:
        # 수동 모드 → set_tool → set_tcp → 자동 모드. 자동 모드에서는 변경이 거부되고,
        # Tool이 비어 있으면 TCP 변경도 실패했다. 로봇은 움직이지 않는다.
        def ensure_tool_and_tcp():
            tool_now, tcp_now = read_tool(), read_tcp()
            if tool_now == TOOL_NAME and tcp_now == EXPECTED_TCP:
                return
            print(f'[Tool/TCP] 현재 Tool={tool_now!r}, TCP={tcp_now!r} → '
                  f'Tool={TOOL_NAME!r}, TCP={EXPECTED_TCP!r} 적용 (로봇은 움직이지 않음)', flush=True)
            set_mode(0, '수동(Tool/TCP 변경용)')
            try:
                if tool_now != TOOL_NAME:
                    try:
                        request = SetCurrentTool.Request()
                        request.name = TOOL_NAME
                        call(set_tool, request, f'Tool {TOOL_NAME} 적용')
                    except RuntimeError as exc:
                        print(f'[Tool/TCP] 경고: Tool 선택 실패({exc}). Tool 없이 진행하면 힘 감지가 부정확할 수 있습니다.',
                              flush=True)
                if tcp_now != EXPECTED_TCP:
                    request = SetCurrentTcp.Request()
                    request.name = EXPECTED_TCP
                    call(set_tcp, request, f'TCP {EXPECTED_TCP} 적용')
            finally:
                set_mode(1, '자동')  # 변경에 실패해도 자동 모드로는 반드시 되돌린다
            tool_now, tcp_now = read_tool(), read_tcp()
            print(f'[Tool/TCP] 적용 후 Tool={tool_now!r}, TCP={tcp_now!r}', flush=True)
            if tcp_now != EXPECTED_TCP:
                raise RuntimeError(f'TCP를 {EXPECTED_TCP}로 바꾸지 못했습니다(현재 {tcp_now!r}). '
                                   '로봇이 STANDBY인지, 펜던트에 등록돼 있는지 확인하세요. 로봇은 움직이지 않았습니다.')

        # ===== 실행 전 준비: Tool/TCP → 특이점 정책·감시 → 경로 계산 → 관절 검사 =====
        ensure_tool_and_tcp()
        if guard_enabled:
            watch = start_singularity_watch(ROBOT_ID)
            watch.wait_ready()
            from utils.singularity import apply as apply_singularity
            def set_singularity_mode(mode):
                request = SetSingularityHandling.Request()
                request.mode = mode
                call(avoid, request, f'팀 YAML 특이점 정책 mode={mode} 적용')
                return 0
            configured_mode = apply_singularity(SimpleNamespace(set_singularity_handling=set_singularity_mode))
            request = SetSingularHandlingForce.Request()
            request.mode = 0  # DR_SINGULARITY_ERROR: 특이점 내 힘 제어 에러 무시 금지.
            call(force_policy, request, '힘 제어 특이점 에러 차단 적용')
            print(f'[특이점] 팀 YAML 정책 mode={configured_mode} 적용 / 힘 제어 ERROR 정책 적용 / '
                  f'/{ROBOT_ID}/error 감시 시작. 진입 시 정지·힘 해제; 자동 MoveJ 회복 없음.',
                  flush=True)
        # 실행 직전 현재 TCP를 기준으로 전체 경로를 구성한다.
        moves = list(motion_builder(read_pose(), args))
        # generator 경로를 전부 계산한 뒤 실행한다. 경로 계산 중 오류라면 첫 이동도 보내지 않는다.
        commands = list(continuous_commands(moves)) if continuous else moves
        if guard_enabled:
            preflight_joints(commands)
        # ===== 경로 실행 =====
        # shift: 계획 경로와 실제 접촉 위치의 차이(XYZ). 바닥·벽을 실제로 찾은 뒤 남은 경로를 그만큼 옮긴다.
        shift = (0.0, 0.0, 0.0)
        free_fx = None  # 벽에서 떨어진 허공의 Fx. '이미 벽에 닿았는지' 판단하는 기준값
        for command in commands:
            # --- 상향 도포 구간 (헤라 각도를 바꾸며 위로) ---
            if isinstance(command, SplineStroke):
                print(f'[{command.label}] 연속 도포 {len(command.poses)}점 | '
                      f'{command.speed:g} mm/s', flush=True)
                points = tuple((*(p[i]+shift[i] for i in range(3)), *p[3:]) for p in command.poses)
                request = spline_request(points, command.speed, args.acceleration)
                motion_started = True
                if hold is not None and hold.active:
                    if force_monitor is None or not force_monitor.ready:
                        raise RuntimeError('실측 힘 안정 확인 없이 상향 도포를 시작할 수 없습니다.')
                    # 컴플라이언스 제어 중에는 컨트롤러가 MoveSplineTask를 거부한다
                    # ([1202] TASK_MOTION_w_COMPLIANCE_CONTROL rejected eMoveSX).
                    # 그래서 점마다 비동기 MoveLine으로 이어 가며 힘/위치를 감시한다(샌딩의 amovel과 같은 방식).
                    deadline = time.monotonic()+180
                    next_force_log = 0.0
                    for index, point in enumerate(points[1:], 1):
                        move_async(point, command.speed, f'도포 {index}/{len(points)-1}점 이동')
                        sent = time.monotonic()
                        moved = False
                        before = read_pose(timeout=0.5)
                        while True:
                            actual = read_pose(timeout=0.5)
                            measured = checked_force(read_force())
                            try:
                                verdict = force_monitor.update(measured, time.monotonic())
                            except RuntimeError:
                                log_force('도포 힘 이상·정지 요청', actual, measured)
                                raise
                            if time.monotonic() >= next_force_log:
                                progress = index/(len(points)-1)
                                angle = args.blade_angle+(args.blade_end_angle-args.blade_angle)*progress
                                log_force(f'상향 도포 중 {index}/{len(points)-1}·{verdict}',
                                          actual, measured, angle)
                                next_force_log = time.monotonic()+0.5
                            # X는 힘제어가 벽을 따라가므로 계획과 다를 수 있다. 한계는 X_FOLLOW_LIMIT_MM 주석 참고.
                            x_limit =(X_FOLLOW_LIMIT_MM if getattr(args, 'press_method', 'force') in ('hybrid', 'direct')
                                       else args.search_distance)
                            if abs(actual[0]-point[0]) > x_limit:
                                log_force('X 편차 한계 초과·정지', actual, measured)
                                raise RuntimeError(f'도포 중 X 편차 {actual[0]-point[0]:+.1f} mm가 '
                                                   f'한계 {x_limit:g} mm를 벗어났습니다.')
                            status = read_state()
                            if status not in (1, 2):
                                raise RuntimeError(f'도포 중 로봇 상태 {status}')
                            # 구간 완료: 한 번이라도 움직인 뒤 정지했거나, 이미 목표 Y/Z 근처에서 정지한 경우.
                            # 명령이 거부되면 check_motion이 바로 IDLE이라 '움직인 적 없음'으로 구분한다.
                            idle = motion_idle()
                            if not idle or math.dist(actual[:3], before[:3]) > 0.3:
                                moved = True
                            if idle and (moved or math.dist(actual[1:3], point[1:3]) <= 2.0):
                                break
                            if not moved and time.monotonic()-sent > 3.0:
                                raise RuntimeError(f'도포 {index}점 이동이 시작되지 않았습니다. '
                                                   '컨트롤러 알람(명령 거부)을 확인하세요.')
                            if time.monotonic() >= deadline:
                                raise TimeoutError('도포 이동 시간 초과')
                            time.sleep(0.05)
                else:
                    call(spline, request, '연속 스플라인 도포', timeout=120.0)
                target = points[-1]
                actual = read_pose()
                # 힘 제어 중 X는 목표 좌표에 고정하지 않는다.
                if not stroke_arrived(actual, target, force_active=bool(hold and hold.active)):
                    raise RuntimeError(f'연속 도포 끝점 미도착: {actual}')
                if hold is not None and hold.active:
                    log_force('도포 끝·힘 해제 전', actual=actual)
                    hold.release()
                    actual = read_pose()
                    # 힘 제어로 조정된 실제 X에서 -X 후퇴한다.
                    shift = (shift[0]+actual[0]-target[0], shift[1], shift[2])
            else:
                step, target, speed = command
                target = (*(target[i]+shift[i] for i in range(3)), *target[3:])
                if step in JOINT_TRANSFER_STEPS:
                    # 두산 API: ikin + movej(동기, 시계 방향). 관절 검사·도착 확인은 move_joint_to_pose가 한다.
                    motion_started = True
                    move_joint_to_pose(call, joint_clients, step, target, args.joint_speed,
                                       args.joint_acceleration, read_pose,
                                       info=lambda text: print(text, flush=True),
                                       direction=(HOME_TRANSFER_DIRECTION if step == HOME_TRANSFER_STEP
                                                  else WALL_TRANSFER_DIRECTION))
                else:
                    move_line(step, target, speed)
                if contact_probe is not None and step == '면 바깥 접근점':
                    # 벽에서 approach_distance만큼 떨어진 허공의 Fx. 나중에 '이미 벽에 닿았는지' 비교 기준.
                    free_fx = checked_force(read_force())[0]
                if contact_probe is not None and step == '면에 수직 접근':
                    io = SimpleNamespace(read_pose=read_pose, read_force=read_force,
                                         move_async=move_async, move_line=move_line,
                                         stop_and_wait=stop_and_wait, motion_idle=motion_idle,
                                         log_force=log_force,
                                         info=lambda text: print(text, flush=True))
                    # io 함수 묶음을 접촉 탐색에 전달한다. 새 노드/토픽 없이 이 노드의 서비스를 재사용한다.
                    method = getattr(args, 'press_method', 'force')
                    compliant = press_force is not None and method in ('position', 'hybrid', 'direct')
                    if compliant:
                        # 위치 누르기: 컴플라이언스만 켜고 위치로 밀어 스프링 눌림으로 힘을 만든다.
                        k = press_stiffness(args)
                        stiffness = (k, *args.force_stiffness[1:])
                        no_force = lambda *a: None

                        def unload_press():
                            """컴플라이언스 해제 전에 기준 X를 실제 X보다 1 mm 뒤로 옮겨 눌린 스프링을 푼다.

                            눌린 채 해제하면 스프링이 한 번에 풀리며 덜컹거린다. 실패해도 해제는 계속한다.
                            """
                            try:
                                here = read_pose(timeout=1.0)
                                request = MoveLine.Request()
                                request.pos = [here[0]-1.0, *here[1:]]
                                request.vel = [5.0, 10.0]
                                request.acc = [args.acceleration, 20.0]
                                request.ref = request.mode = request.sync_type = 0
                                call(line, request, '컴플라이언스 해제 전 누름 풀기', timeout=10.0)
                                time.sleep(0.3)
                            except Exception as exc:
                                node.get_logger().warning(f'누름 풀기 실패, 그대로 해제: {exc}')

                        def release_hybrid():
                            try:
                                release_press_force()
                            finally:
                                unload_press()

                        # 바닥 탐색부터 순응제어로 내려간다. 20°로 세운 헤라는 길이 방향으로 단단해
                        # 위치제어로 바닥에 닿으면 힘이 순간적으로 치솟아 안전 충돌 감지(7060)가 먼저 걸렸다.
                        search_stiffness = (k, stiffness[1], FLOOR_SEARCH_STIFFNESS_Z, *stiffness[3:])
                        hold = WallForceHold(SimpleNamespace(compliance=start_compliance, set_force=no_force,
                                                             release_force=(release_hybrid if method in ('hybrid', 'direct')
                                                                            else unload_press),
                                                             release_compliance=release_compliance),
                                             press_force, search_stiffness)
                        log_force('컴플라이언스(탐색용) 시작 직전')
                        hold.start()
                        time.sleep(0.3)  # 컴플라이언스 전환 안정화
                    actual = contact_probe(io)
                    # '면에 수직 접근'은 벽에서 wall_backoff만큼 띄운 점이다. 계획 접촉점 기준으로 보정한다.
                    approach_offset = getattr(args, 'approach_offset', (0.0, 0.0, 0.0))
                    target = (*(target[i]-approach_offset[i] for i in range(3)), *target[3:])
                    shift = tuple(actual[i]-target[i] for i in range(3))
                    print(f'접촉점 기준 도포 경로 보정 XYZ: {shift}', flush=True)
                    if compliant:
                        # 탐색 후: Z를 다시 단단하게(도포 경로 추종), X는 누르기 강성 유지.
                        request = SetStiffnessx.Request()
                        request.stx, request.ref, request.time = list(stiffness), 0, 0.3
                        call(stiffness_client, request, 'Z 강성 복원(탐색 후)')
                        time.sleep(0.3)
                        press_io = SimpleNamespace(read_pose=read_pose, read_force=read_force,
                                                   move_async=move_async, stop_and_wait=stop_and_wait,
                                                   motion_idle=motion_idle,
                                                   info=lambda text: print(text, flush=True))
                        if method == 'direct':
                            # 벽 탐색 없이 측정된 벽 X(계획 시작점, target[0])로 바로 간다.
                            actual, reference_x, offset = direct_press(press_io, args, target[0], free_fx=free_fx)
                        else:
                            actual, reference_x, offset = position_press(press_io, args,
                                                                         push=(method == 'position'))
                        if method in ('hybrid', 'direct'):
                            # 벽에 닿은 뒤: X 강성을 샌딩처럼 낮추고 +X 목표 힘을 켠 채 도포한다.
                            soft = (HYBRID_STIFFNESS_X, *stiffness[1:])
                            request = SetStiffnessx.Request()
                            request.stx, request.ref, request.time = list(soft), 0, 0.3
                            call(stiffness_client, request, f'X 강성 {HYBRID_STIFFNESS_X:g} N/m로 변경')
                            time.sleep(0.3)
                            set_press_force(press_force)
                            # 맨 아래부터 벽에 붙어서 시작하도록, 힘이 목표의 70%에 닿을 때까지 최대 3 s 기다린다.
                            built_until = time.monotonic()+3.0
                            pushed = 0.0
                            while time.monotonic() < built_until:
                                pushed = abs(checked_force(read_force())[0]-offset)
                                if pushed >= 0.7*press_force:
                                    break
                                time.sleep(0.05)
                            time.sleep(0.3)
                            print(f'[힘검증] 도포 시작 전 누름 {pushed:.2f} N (목표 {press_force:g} N)'
                                  + ('' if pushed >= 0.7*press_force else ' · 70% 미달, 그대로 시작'),
                                  flush=True)
                            actual = read_pose()
                            # 힘을 거는 동안 헤라가 벽 쪽으로 더 들어간다. 감지 X를 기준으로 출발하면
                            # 이동 시작과 동시에 기준이 뒤로 당겨져 누름이 풀리므로 현재 실제 X에서 출발한다.
                            reference_x = actual[0]
                            log_force('힘제어 시작 후', actual=actual)
                        else:
                            log_force('밀어 넣기 후', actual=actual)
                        # 힘은 벽 찾기에만 썼다. 도포 중에는 상한 초과만 중단하고 부족은 경고 로그만 남긴다.
                        force_monitor = PressForceMonitor(press_force, press_tolerance(args),
                                                          args.force_stable_time, args.force_loss_timeout,
                                                          position_press_limit(args), offset=offset,
                                                          warn_only=True)
                        force_monitor.ready = True
                        # 도포 경로 X는 벽 X + 밀기 깊이(기준 X)를 따른다.
                        shift = (reference_x-target[0], actual[1]-target[1], actual[2]-target[2])
                        if method in ('hybrid', 'direct'):
                            print(f'[힘검증] 벽 위치 → +X {press_force:g} N 힘제어(상대) 유지하며 상향 도포 '
                                  f'(상한 {position_press_limit(args):g} N 초과 시만 정지)', flush=True)
                        else:
                            print(f'[힘검증] 벽 감지 + 위치 밀기 완료 → 기준 X {reference_x:.2f} 유지하며 상향 도포 '
                                  f'(상한 {position_press_limit(args):g} N 초과 시만 정지)', flush=True)
                    elif press_force is not None:
                        selected_force = press_force
                        print(f'이번 도포 +X 유지 힘: {selected_force:g} N', flush=True)
                        force_io = SimpleNamespace(compliance=start_compliance, set_force=set_press_force,
                                                   release_force=release_press_force,
                                                   release_compliance=release_compliance)
                        hold = WallForceHold(force_io, selected_force, args.force_stiffness)
                        log_force('힘 설정 직전')
                        relative = getattr(args, 'force_mode', 'relative') != 'absolute'
                        offset = checked_force(read_force())[0] if relative else 0.0
                        hold.start()
                        log_force('힘 설정 응답 후·상승 전')
                        force_monitor = PressForceMonitor(selected_force, args.force_tolerance,
                                                          args.force_stable_time, args.force_loss_timeout,
                                                          force_limit(args), offset=offset)
                        if relative:
                            print(f'[힘검증] 상대 모드: 설정 직전 Fx {offset:+.2f} N을 기준으로 감시', flush=True)
                        wait_io = SimpleNamespace(read_pose=read_pose, read_force=read_force,
                                                  read_state=read_state, log_force=log_force)
                        actual = wait_for_press_force(wait_io, force_monitor,
                                                      args.force_wait_timeout, args.search_distance,
                                                      origin=actual)
                        # 힘 형성으로 X가 달라질 수 있으므로 출발점과 후속 경로를 재정렬한다.
                        shift = tuple(actual[i]-target[i] for i in range(3))
                        print(f'[힘검증] 실측 힘 안정 확인 완료 → +X {selected_force:g} N '
                              '명령을 유지하며 상향 도포',
                              flush=True)
        if watch is not None:
            watch.check()
        print('평면 도포 완료', flush=True)
    finally:
        failed = sys.exc_info()[0] is not None
        # 감지 알람이 정지/힘 해제 요청 자체를 막지 않도록 정리 중에는 감시 판정을 생략한다.
        cleaning_up = True
        try:
            if (hold is not None and hold.active) or (motion_started and failed):
                if node is not None:
                    node.get_logger().warning('[작업중단] 정지 확인 후 활성 힘/컴플라이언스 해제')
                try:
                    stop_and_wait()
                finally:
                    if hold is not None and hold.active:
                        hold.release()
        finally:
            try:
                if watch is not None:
                    watch.close()
            finally:
                safe_shutdown(node)
