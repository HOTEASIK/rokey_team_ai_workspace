"""평면 도포·스쿱의 내부 ROS 서비스 실행부. 단독 실행 노드가 아니다."""

# ===== 파일 역할 / 통신 안내 =====
# 내부 공통 실행 모듈: import만으로 로봇을 움직이지 않는다.
# run_robot() 호출 시 평면 도포 ROS 노드를 만들고 서비스 클라이언트를 생성한다.
# 서비스 공통 경로는 /dsr01/dsr_controller2. 요청/응답 타입은 dsr_msgs2/srv이다.
# 접근/후퇴: MoveLine. 도포: MoveSplineTask. 탐색: 비동기 MoveLine + 외력 조회 + MoveStop.
# 힘 유지: 컴플라이언스 시작 → BASE X 목표 힘 설정 → 도포 → 힘/컴플라이언스 해제.
# 출발 전 실측 힘을 확인하고 도포 중에도 감시한다. 힘 조절 루프 자체는 로봇 컨트롤러가 수행한다.

import argparse
import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace
import time

from .contact_search import (
    WallForceHold, PressForceMonitor, wait_for_press_force, checked_force, press_force_request,
)
from .singularity_guard import SingularityDetected, start_singularity_watch

from .putty_geometry import (
    EDGE_OFFSET, EXPECTED_TCP, ROBOT_ID, START_XY, END_XY, SURFACE_Z,
    TRANSFER_Z, format_values, orientation_error, zyz_axes, build_stroke,
    application_moves, PICKUP_ABC,
)


# ===== 하이퍼파라미터 =====


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
    return position_error <= 2.0 and orientation_error(actual[3:], target[3:]) <= 2.0

# ===== 로봇 초기화 설정 =====
# namespace와 TCP는 공통 설정을 사용한다. 실물 시스템에는 이동을 요청하지 않는다.


def make_parser(include_j6=False):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True,
                        help='real/virtual 컨트롤러에 이동 요청')
    if include_j6:
        raise ValueError('J6 독립 실행 케이스는 제거됐습니다.')
    parser.add_argument('--transfer-speed', type=float, default=30.0, help='mm/s')
    parser.add_argument('--apply-speed', type=float, default=10.0, help='mm/s')
    parser.add_argument('--acceleration', type=float, default=20.0, help='mm/s²')
    parser.add_argument('--transfer-z', type=float, default=TRANSFER_Z)
    parser.add_argument('--surface-z', type=float, default=SURFACE_Z,
                        help='도포할 날 기준점의 BASE Z, mm')
    parser.add_argument('--edge-offset', type=float, nargs=3, default=EDGE_OFFSET)
    return parser




# 숫자 범위·좌표·길이 등이 유효한지 확인한다. 역기구학이나 충돌 검사는 아니다.
def validate(args, include_j6=False):
    names = ('transfer_speed', 'apply_speed', 'acceleration')
    if include_j6:
        raise ValueError('J6 독립 실행 케이스는 제거됐습니다.')
    for name in names:
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다.')
    values = (args.transfer_z, args.surface_z, *args.edge_offset)
    if not all(map(math.isfinite, values)):
        raise ValueError('각도·높이·오프셋은 유한한 값이어야 합니다.')




# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args, motion_builder, use_j6=False, allow_real=False,
              continuous=False, contact_probe=None, press_force=None):
    import rclpy
    from utils.shutdown import safe_shutdown
    from dsr_msgs2.srv import (
        GetRobotSystem, GetCurrentTcp, GetCurrentPosx, MoveLine,
    )
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
        pose = node.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx')
        line = node.create_client(MoveLine, f'{prefix}/motion/move_line')
        clients = (system, tcp, pose, line)
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
            clients += (compliance, desired, release, release_comp)
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

        def read_pose(timeout=5.0):
            request = GetCurrentPosx.Request()
            request.ref = 0
            rows = call(pose, request, 'BASE TCP 조회', timeout=timeout).task_pos_info
            return values(rows[0].data[:6] if rows else ())

        def move_line(step, target, speed):
            nonlocal motion_started
            print(f'[{step}] BASE [{format_values(target)}] | {speed:g} mm/s', flush=True)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = request.sync_type = 0
            motion_started = True  # 응답 전 오류여도 명령 수신 여부가 불확실하므로 정지한다.
            call(line, request, step, timeout=120.0)
            actual = read_pose()
            if (math.dist(actual[:3], target[:3]) > 2.0 or
                    orientation_error(actual[3:], target[3:]) > 2.0):
                raise RuntimeError(f'{step} 목표 미도착: {actual}')

        # ref=0: BASE 기준 [Fx,Fy,Fz,...]를 조회한다. 이 함수 자체는 로봇을 움직이지 않는다.
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
                    f' | TCP_ABC=[{format_values(actual[3:])}] deg{angle}')
            except SingularityDetected:
                raise
            except Exception as exc:
                node.get_logger().warning(f'[힘진단] 단계={stage} 로그 조회 실패: {exc}')

        # sync_type=1은 로봇 이동 완료를 기다리지 않는다는 뜻이다. 이동 중 힘을 감시하려고 사용한다.
        def move_async(target, speed):
            nonlocal motion_started
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = 0
            request.sync_type = 1
            motion_started = True
            call(line, request, '유한 거리 접촉 탐색', timeout=0.5)

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

        def read_state():
            return call(state, GetRobotState.Request(), '로봇 상태 조회', timeout=0.5).robot_state

        def start_compliance(stiffness):
            request = TaskComplianceCtrl.Request()
            request.stx = list(stiffness)
            request.ref, request.time = 0, 0.3
            call(compliance, request, 'BASE 컴플라이언스 시작')

        def set_press_force(value):
            mode = 0 if getattr(args, 'force_mode', 'relative') == 'absolute' else 1
            request = press_force_request(value, mode=mode)
            call(desired, request, f'BASE +X {value:g} N 힘 유지')

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
        if call(tcp, GetCurrentTcp.Request(), 'TCP 확인').info != EXPECTED_TCP:
            raise RuntimeError(f'{EXPECTED_TCP} TCP를 적용하세요.')
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
        current = read_pose()
        # 실행 직전 현재 TCP를 기준으로 전체 경로를 구성한다.
        moves = list(motion_builder(read_pose(), args))
        # generator 경로를 전부 계산한 뒤 실행한다. 경로 계산 중 오류라면 첫 이동도 보내지 않는다.
        commands = list(continuous_commands(moves)) if continuous else moves
        shift = (0.0, 0.0, 0.0)
        for command in commands:
            if isinstance(command, SplineStroke):
                print(f'[{command.label}] 연속 도포 {len(command.poses)}점 | '
                      f'{command.speed:g} mm/s', flush=True)
                points = tuple((*(p[i]+shift[i] for i in range(3)), *p[3:]) for p in command.poses)
                request = spline_request(points, command.speed, args.acceleration)
                motion_started = True
                if hold is not None and hold.active:
                    if force_monitor is None or not force_monitor.ready:
                        raise RuntimeError('실측 힘 안정 확인 없이 상향 도포를 시작할 수 없습니다.')
                    # 비동기로 실행해 힘 제어 중 X 이동 범위와 센서 오류를 감시한다.
                    request.sync_type = 1
                    call(spline, request, '힘 유지 연속 도포 시작', timeout=0.5)
                    deadline = time.monotonic()+120
                    next_force_log = 0.0
                    while True:
                        actual = read_pose(timeout=0.5)
                        measured = checked_force(read_force())
                        closest = min(points, key=lambda p: math.dist(actual[1:3], p[1:3]))
                        try:
                            verdict = force_monitor.update(measured, time.monotonic())
                        except RuntimeError:
                            log_force('도포 힘 이상·정지 요청', actual, measured)
                            raise
                        # 기존 감시에서 읽은 값을 재사용한다. 출력은 최대 약 2 Hz.
                        if time.monotonic() >= next_force_log:
                            progress = points.index(closest)/(len(points)-1)
                            angle = args.blade_angle+(args.blade_end_angle-args.blade_angle)*progress
                            log_force(f'상향 도포 중·{verdict}', actual, measured, angle)
                            next_force_log = time.monotonic()+0.5
                        if abs(actual[0]-closest[0]) > args.search_distance:
                            raise RuntimeError('힘 유지 중 X 편차가 최대 탐색 범위를 벗어났습니다.')
                        status = read_state()
                        arrived = stroke_arrived(actual, points[-1], force_active=True)
                        if status == 1 and arrived:
                            break
                        if status not in (1, 2):
                            raise RuntimeError(f'힘 유지 도포 중 로봇 상태 {status}')
                        if time.monotonic() >= deadline:
                            raise TimeoutError('힘 유지 연속 도포 시간 초과')
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
                move_line(step, target, speed)
                if contact_probe is not None and step == '면에 수직 접근':
                    io = SimpleNamespace(read_pose=read_pose, read_force=read_force,
                                         move_async=move_async,
                                         stop_and_wait=stop_and_wait,
                                         log_force=log_force,
                                         info=lambda text: print(text, flush=True))
                    # io 함수 묶음을 접촉 탐색에 전달한다. 새 노드/토픽 없이 이 노드의 서비스를 재사용한다.
                    actual = contact_probe(io)
                    shift = tuple(actual[i]-target[i] for i in range(3))
                    print(f'접촉점 기준 도포 경로 보정 XYZ: {shift}', flush=True)
                    if press_force is not None:
                        selected_force = press_force
                        print(f'이번 도포 +X 유지 힘: {selected_force:g} N', flush=True)
                        force_io = SimpleNamespace(compliance=start_compliance, set_force=set_press_force,
                                                   release_force=release_press_force,
                                                   release_compliance=release_compliance)
                        hold = WallForceHold(force_io, selected_force, args.force_stiffness)
                        log_force('힘 설정 직전')
                        hold.start()
                        log_force('힘 설정 응답 후·상승 전')
                        force_monitor = PressForceMonitor(selected_force, args.force_tolerance,
                                                          args.force_stable_time, args.force_loss_timeout)
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


# 기존 가상 geometry 스쿱을 위한 서비스 실행부다.
def run_geometry_robot(poses, args, motion_builder=None):
    import rclpy
    from utils.shutdown import safe_shutdown
    from dsr_msgs2.srv import GetRobotSystem, GetCurrentTcp, GetCurrentPosx, MoveLine
    from utils.service_waiter import ServiceWaiter

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('scoop_putty' if motion_builder else 'apply_putty', namespace=ROBOT_ID)
        prefix = f'/{ROBOT_ID}/dsr_controller2'
        system = node.create_client(GetRobotSystem, f'{prefix}/system/get_robot_system')
        tcp = node.create_client(GetCurrentTcp, f'{prefix}/tcp/get_current_tcp')
        pose = node.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx')
        motion = node.create_client(MoveLine, f'{prefix}/motion/move_line')
        ServiceWaiter(node).wait(system, tcp, pose, motion)

        def call(client, request, step, timeout=5.0):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                raise TimeoutError(f'{step} 응답 시간 초과. 이동 취소는 수행하지 않습니다.')
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError(f'{step} 서비스 실패. 컨트롤러 알람을 확인하세요.')
            return result

        if call(system, GetRobotSystem.Request(), '가상 시스템 확인').robot_system != 1:
            raise RuntimeError('이 도포 실행은 가상 컨트롤러에서만 허용합니다.')
        if call(tcp, GetCurrentTcp.Request(), 'TCP 확인').info != EXPECTED_TCP:
            raise RuntimeError(f'{EXPECTED_TCP} TCP를 적용하세요.')

        def read_pose():
            request = GetCurrentPosx.Request()
            request.ref = 0
            rows = call(pose, request, 'BASE TCP 조회').task_pos_info
            values = tuple(rows[0].data[:6]) if rows else ()
            if len(values) != 6 or not all(map(math.isfinite, values)):
                raise RuntimeError('BASE TCP 자세가 유효하지 않습니다.')
            return values

        current = read_pose()
        if orientation_error(current[3:], PICKUP_ABC) > 2.0:
            raise RuntimeError('픽업 완료 자세에서 시작하세요. 초기화 및 집기 후 실행하세요.')
        poses = build_stroke(args.surface_z, args.edge_offset, args.samples, current[3:],
                             normal_x=getattr(args, 'pickup_normal_x', -1.0))
        moves = (motion_builder(current, poses, args) if motion_builder else
                 application_moves(current, poses, args.transfer_z, args.transfer_speed, args.apply_speed))
        # 전체 계획을 먼저 검증한 뒤 첫 이동을 요청한다.
        moves = list(moves)
        for step, target, speed in moves:
            print(f'[{step}] BASE [{format_values(target)}] | {speed:g} mm/s', flush=True)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = request.sync_type = 0
            call(motion, request, step, timeout=120.0)
            actual = read_pose()
            distance = math.dist(actual[:3], target[:3])
            angle = orientation_error(actual[3:], target[3:])
            if distance > 2.0 or angle > 2.0:
                raise RuntimeError(f'{step} 미도착: 오차 {distance:.2f} mm, {angle:.2f}도')
        print('가상 퍼티 뜨기/도포 완료' if motion_builder else '가상 도포 및 수직 후퇴 완료', flush=True)
    finally:
        safe_shutdown(node)
