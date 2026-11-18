"""측정한 TCP 자세를 유지해 퍼티를 찍고 수직 상승한다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/scoop_putty. 기본 taught 경로는 측정 기준 자세를 보정해 퍼티를 찍는다.
# 동작 순서: 시스템/TCP 확인 → 실물 파지 수동 확인 → 자세 정렬 → 사선 진입 → 수직 상승.
# --scoop-angle은 헤라 기울기와 진입각의 공통 값이며, 기준 기울기는 추정 보정값이다.
# 통신: GetRobotSystem/GetCurrentTcp/GetCurrentPosx/MoveLine 서비스를 호출한다.
# geometry 경로는 putty_motion.run_geometry_robot을 일반 Python 함수로 호출한다.

import argparse
import math
import sys

from .putty_motion import run_geometry_robot
from .putty_geometry import EXPECTED_TCP, ROBOT_ID, format_values, orientation_error

from .putty_geometry import (
    BLADE_WIDTH, BLADE_LENGTH, BLADE_THICKNESS, EDGE_OFFSET, SURFACE_Z,
    SAMPLES, PICKUP_ABC, TRANSFER_Z, START_TILT, build_stroke,
    zyz_axes, axes_to_zyz, rotate_base_y,
)


# ===== 설정 / 하이퍼파라미터 =====
CONTAINER_CENTER = (723.83, -75.62, 35.36)  # 12번 좌표 = 용기 바닥 중심 가정
CONTAINER_SIZE = (120.0, 120.0, 50.0)      # 내부 치수 가정
MARGIN = 5.0
TAUGHT_SCOOP_POSE = (327.73, 211.45, 138.58, 8.45, 154.91, 6.58)
TAUGHT_LIFT_MM = 120.0


# ===== 메인 =====
# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True, help='연결된 컨트롤러에 이동 요청')
    parser.add_argument('--path', choices=('taught', 'geometry'), default='taught',
                        help='측정한 고정 자세 경로 / 기존 가상 기하 경로')
    parser.add_argument('--taught-pose', nargs=6, type=float, default=TAUGHT_SCOOP_POSE,
                        help='측정한 BASE TCP XYZABC')
    parser.add_argument('--lift', type=float, default=TAUGHT_LIFT_MM, help='찍은 뒤 수직 상승 거리 mm')
    parser.add_argument('--entry-height', type=float, default=120.0,
                        help='측정 목표보다 높은 사선 진입 시작 높이 mm')
    parser.add_argument('--reference-blade-angle', type=float, default=60.0,
                        help='측정 ABC에서 추정한 헤라 판의 수평면 기준 각도 degree')
    parser.add_argument('--apply-after', action='store_true', help='뜬 후 16→15 도포까지 실행')
    parser.add_argument('--container-center', nargs=3, type=float, default=CONTAINER_CENTER,
                        help='용기 바닥 중심 BASE XYZ mm')
    parser.add_argument('--container-size', nargs=3, type=float, default=CONTAINER_SIZE,
                        help='용기 내부 X/Y/높이 mm')
    parser.add_argument('--blade-width', type=float, default=BLADE_WIDTH)
    parser.add_argument('--blade-length', type=float, default=BLADE_LENGTH)
    parser.add_argument('--scoop-angle', '--entry-angle', dest='scoop_angle', type=float, default=45.0,
                        help='측정 경로의 헤라 기울기와 사선 진입 공통 각도 degree; 기하 경로는 기존 회전각')
    parser.add_argument('--scoop-depth', type=float, default=3.0, help='표면 아래 최대 깊이 mm')
    parser.add_argument('--scoop-distance', type=float, default=10.0, help='사선 진입 시 BASE -X 이동 거리 mm')
    parser.add_argument('--scoop-speed', type=float, default=3.0)
    parser.add_argument('--transfer-speed', type=float, default=30.0)
    parser.add_argument('--apply-speed', type=float, default=10.0)
    parser.add_argument('--acceleration', type=float, default=20.0)
    parser.add_argument('--transfer-z', type=float, default=TRANSFER_Z)
    parser.add_argument('--surface-z', type=float, default=SURFACE_Z)
    parser.add_argument('--edge-offset', nargs=3, type=float, default=EDGE_OFFSET)
    parser.add_argument('--samples', type=int, default=SAMPLES)
    args = parser.parse_args()
    args.pickup_normal_x = 1.0  # 퍼티가 묻는 면은 픽업 상태에서 BASE +X
    try:
        if args.path == 'taught':
            validate_taught(args)
            run_robot((), args)
        else:
            validate(args)
            poses = build_stroke(args.surface_z, args.edge_offset, args.samples, normal_x=1.0)
            run_robot(poses, args, motion_builder=scoop_moves)
        return 0
    except KeyboardInterrupt:
        return 130
    except (ValueError, RuntimeError, TimeoutError) as exc:
        print(f'퍼티 뜨기 중단: {exc}', file=sys.stderr)
        return 1


# ===== 보조 함수 =====
# 실물 측정 경로의 값과 지원 옵션을 검사한다.
def validate_taught(args):
    if (not all(map(math.isfinite, (*args.taught_pose, args.lift))) or args.lift <= 0):
        raise ValueError('측정 자세는 유한하고 상승 거리는 양수여야 합니다.')
    if not math.isfinite(args.entry_height) or args.entry_height <= 0:
        raise ValueError('진입 시작 높이는 양의 유한한 값이어야 합니다.')
    if not math.isfinite(args.scoop_angle) or not 0 < args.scoop_angle < 90:
        raise ValueError('진입각은 0도 초과 90도 미만이어야 합니다.')
    if (not math.isfinite(args.reference_blade_angle) or
            not 0 < args.reference_blade_angle < 90 or
            not all(map(math.isfinite, args.edge_offset))):
        raise ValueError('기준 헤라 각도는 0~90도 사이, 날 오프셋은 유한해야 합니다.')
    for value in (args.scoop_speed, args.transfer_speed, args.acceleration, args.transfer_z):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('속도·가속도·이송 높이는 양의 유한한 값이어야 합니다.')
    if args.apply_after:
        raise ValueError('측정 경로는 스쿱만 실행합니다. 도포 실물 전환은 별도입니다.')


# 측정 자세의 추정 기준 기울기를 보정하고 날 기준점을 유지하도록 TCP도 보정한다.
def taught_target(args):
    """추정 기준각에서 BASE Y로 회전하고 기존 날 기준점을 유지하도록 TCP를 보정한다.

    날이 -X 쪽으로 기울고 손잡이가 위쪽인 장착을 전제로 한다.
    기준각 60도와 TCP→날 오프셋은 실측 보정값이 아니므로 결과 각도는 추정이다.
    """
    reference = tuple(args.taught_pose)
    initial = zyz_axes(reference[3:])
    rotation = args.scoop_angle - args.reference_blade_angle
    axes = tuple(rotate_base_y(axis, rotation) for axis in initial)
    edge = tuple(reference[i]+sum(initial[j][i]*args.edge_offset[j] for j in range(3))
                 for i in range(3))
    target = tuple(edge[i]-sum(axes[j][i]*args.edge_offset[j] for j in range(3))
                   for i in range(3))
    return (*target, *axes_to_zyz(axes))


# 보정한 자세를 유지해 사선 진입하고 Z축으로 상승하는 경유점을 생성한다.
def taught_moves(current, args):
    validate_taught(args)
    target = taught_target(args)
    above = (*target[:2], target[2]+args.lift, *target[3:])
    dx = args.entry_height / math.tan(math.radians(args.scoop_angle))
    entry = (target[0]-dx, target[1], target[2]+args.entry_height, *target[3:])
    if not all(map(math.isfinite, entry)):
        raise ValueError('사선 진입점이 유한하지 않습니다.')
    height = max(current[2], entry[2], above[2], args.transfer_z)
    yield '현재 자세로 이송 높이 확보', (*current[:2], height, *current[3:]), args.transfer_speed
    yield '용기 접근 전 측정 자세 정렬', (*current[:2], height, *target[3:]), args.transfer_speed
    yield 'Y축 평행 자세로 사선 시작점 위 이송', (*entry[:2], height, *target[3:]), args.transfer_speed
    yield 'Y축 평행 자세로 사선 시작점 이동', entry, args.transfer_speed
    yield f'Y축 평행 유지하며 +X 방향 {args.scoop_angle:g}도 사선 찍기', target, args.scoop_speed
    yield 'Y축 평행 유지하며 수직 상승', above, args.scoop_speed


# 실제 파지 여부를 사람이 터미널 입력으로 확인한다. ROS 센서의 자동 파지 감지가 아니다.
def confirm_grasp():
    # 현재 드라이버는 grip-detected 값을 제공하는 서비스를 노출하지 않는다.
    try:
        answer = input('헤라가 실제로 잡혀 있는지 확인하세요. 계속하려면 y 입력: ')
    except EOFError as exc:
        raise RuntimeError('파지 확인 입력이 없어 이동을 시작하지 않습니다.') from exc
    if answer.strip().lower() != 'y':
        raise RuntimeError('헤라 파지가 확인되지 않아 이동을 시작하지 않습니다.')


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(poses, args, motion_builder=None):
    if args.path == 'geometry':
        return run_geometry_robot(poses, args, motion_builder=motion_builder)
    import rclpy
    from dsr_msgs2.srv import GetRobotSystem, GetCurrentTcp, GetCurrentPosx, MoveLine
    from utils.service_waiter import ServiceWaiter
    from utils.shutdown import safe_shutdown

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('scoop_putty', namespace=ROBOT_ID)
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

        kind = call(system, GetRobotSystem.Request(), '시스템 확인').robot_system
        if kind not in (0, 1):
            raise RuntimeError(f'지원하지 않는 시스템: {kind}')
        if call(tcp, GetCurrentTcp.Request(), 'TCP 확인').info != EXPECTED_TCP:
            raise RuntimeError(f'{EXPECTED_TCP} TCP를 적용하세요.')

        def read_pose():
            request = GetCurrentPosx.Request()
            request.ref = 0
            rows = call(pose, request, 'BASE TCP 조회').task_pos_info
            values = tuple(rows[0].data[:6]) if rows else ()
            if len(values) != 6 or not all(map(math.isfinite, values)):
                raise RuntimeError('유효한 BASE TCP 자세를 읽지 못했습니다.')
            return values

        print('헤라 기준각'+
              f' {args.reference_blade_angle:g}°는 추정값입니다. '
              f'헤라·진입 공통 목표각 {args.scoop_angle:g}°, 오프셋 {tuple(args.edge_offset)} mm', flush=True)
        print('보정된 찍기 TCP ['+format_values(taught_target(args))+']', flush=True)
        if kind == 0:
            confirm_grasp()
        current = read_pose()
        moves = list(taught_moves(current, args))
        for step, target, speed in moves:
            print(f'[{step}] BASE [{format_values(target)}] | {speed:g} mm/s', flush=True)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = request.sync_type = 0
            call(motion, request, step, timeout=120.0)
            actual = read_pose()
            if (math.dist(actual[:3], target[:3]) > 2.0 or
                    orientation_error(actual[3:], target[3:]) > 2.0):
                raise RuntimeError(f'{step} 목표 XYZABC 미도착: {actual}')
        print('측정 자세 퍼티 찍기 및 수직 상승 완료', flush=True)
    finally:
        safe_shutdown(node)



# 숫자 범위·좌표·길이 등이 유효한지 확인한다. 역기구학이나 충돌 검사는 아니다.
def validate(args):
    if not math.isfinite(args.scoop_angle) or not 0 < args.scoop_angle < 90:
        raise ValueError('scoop-angle은 0도 초과 90도 미만이어야 합니다.')
    for name in ('blade_width', 'blade_length', 'scoop_depth', 'scoop_distance',
                 'scoop_speed', 'transfer_speed', 'apply_speed', 'acceleration'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다.')
    values = (*args.container_center, *args.container_size, *args.edge_offset,
              args.transfer_z, args.surface_z)
    if not all(map(math.isfinite, values)) or min(args.container_size) <= 0:
        raise ValueError('용기·오프셋·높이 값이 유효하지 않습니다.')
    if not 2 <= args.samples <= 500:
        raise ValueError('samples는 2~500이어야 합니다.')
    sx, sy, sz = args.container_size
    if args.blade_width + 2*MARGIN >= sy:
        raise ValueError(f'날 너비 {args.blade_width:g} mm가 용기 폭 {sy:g} mm에 맞지 않습니다. '
                         '실제 날 너비와 용기 내부 치수를 확인하세요.')
    if args.blade_length + args.scoop_distance + 2*MARGIN >= sx:
        raise ValueError('날 길이 + 뜨기 거리 + 여유가 용기 X 크기보다 작아야 합니다.')
    if args.scoop_depth + BLADE_THICKNESS >= sz - MARGIN:
        raise ValueError('퍼티 뜨기 깊이가 용기 바닥에 너무 가깝습니다.')


# 날의 BASE 좌표에서 회전된 Tool 오프셋을 빼 TCP XYZABC를 구한다.
def pose_from_edge(edge, rotation, initial_axes, edge_offset):
    axes = tuple(rotate_base_y(axis, rotation) for axis in initial_axes)
    tcp = tuple(edge[i] - sum(axes[j][i]*edge_offset[j] for j in range(3))
                for i in range(3))
    return (*tcp, *axes_to_zyz(axes))


# 기존 용기 모델에서 날 모서리의 여유만 검사한다. 로봇 전체 충돌 검사는 아니다.
def check_blade(edge, rotation, args):
    """지정한 직사각형 날의 모서리/두께가 용기 벽·바닥을 넘지 않는지 검사."""
    cx, cy, bottom = args.container_center
    sx, sy, sz = args.container_size
    top = bottom + sz
    handle = rotate_base_y((0, 0, 1), rotation)
    normal = rotate_base_y((1, 0, 0), rotation)
    for length in (0, args.blade_length):
        for side in (-1, 1):
            for thickness in (-BLADE_THICKNESS/2, BLADE_THICKNESS/2):
                point = tuple(edge[i]+length*handle[i]+thickness*normal[i]
                              +(side*args.blade_width/2 if i == 1 else 0)
                              for i in range(3))
                if point[2] < top:
                    if (abs(point[0]-cx) > sx/2-MARGIN or
                            abs(point[1]-cy) > sy/2-MARGIN or point[2] < bottom+MARGIN):
                        raise ValueError('계획된 날이 용기 벽 또는 바닥 여유를 침범합니다.')


# 기존 geometry 스쿱 경로. measured/taught 스쿱과 다른 가상 시험 계산이다.
def scoop_moves(current, poses, args):
    validate(args)
    axes = zyz_axes(current[3:])
    cx, cy, bottom = args.container_center
    sx, _, sz = args.container_size
    top = bottom+sz
    # 날 기준점: 중심에서 전체 X 폭의 1/4만큼 +X에 찍는다.
    target_x = cx + sx/4
    n = max(21, args.samples)
    rotation = -args.scoop_angle
    scoop = []
    for i in range(n):
        u = i/(n-1)
        edge = (target_x + args.scoop_distance*(1-u), cy,
                top + 10.0*(1-u) - args.scoop_depth*u)
        check_blade(edge, rotation, args)
        scoop.append(pose_from_edge(edge, rotation, axes, args.edge_offset))
    height = max(args.transfer_z, current[2], top+args.blade_length+50,
                 *(p[2] for p in scoop), *(p[1][2] for p in poses))
    start = scoop[0]
    yield '집기 후 이송 높이 상승', (*current[:2], height, *current[3:]), args.transfer_speed
    # 높은 위치에서 이송과 자세 변경을 동시에 수행한다.
    # 표면 위 진입점까지 같은 자세로 수직 하강한다.
    raised = (*start[:2], height, *start[3:])
    yield f'12번 위로 이송하며 찍기 {args.scoop_angle:g}도 자세', raised, args.transfer_speed
    yield '퍼티 표면 접근', start, args.scoop_speed
    for i, target in enumerate(scoop[1:], 2):
        yield f'고정 자세 사선 찍기 {i}/{n}', target, args.scoop_speed
    # 찍은 위치의 XY와 자세를 유지한 채 수직으로 빠져나온다.
    end = scoop[-1]
    height = max(height, end[2])
    yield '찍기 자세 유지하며 상승', (*end[:2], height, *end[3:]), args.scoop_speed
    if not args.apply_after:
        return
    _, first_tcp, _, *_ = poses[0]
    yield '찍기 자세 유지하며 도포 위치 이송', (
        *first_tcp[:2], height, *end[3:]), args.transfer_speed
    # 도포 직전에 찍기 각도에서 도포 각도로 전환한다.
    anchor = (*first_tcp[:2], height)
    rotations = max(1, math.ceil((args.scoop_angle+90-START_TILT)/5))
    for i in range(1, rotations+1):
        rotation = -args.scoop_angle+((90-START_TILT)+args.scoop_angle)*i/rotations
        angles = axes_to_zyz(tuple(rotate_base_y(axis, rotation) for axis in axes))
        yield f'도포 전 면 뒤집기 {i}/{rotations}', (*anchor, *angles), args.transfer_speed
    _, tcp, abc, *_ = poses[0]
    yield '16번 도포 시작점 접근', (*tcp, *abc), args.apply_speed
    for i, (_, tcp, abc, *_rest) in enumerate(poses[1:], 2):
        yield f'16→15 도포 {i}/{len(poses)}', (*tcp, *abc), args.apply_speed
    yield '도포 후 수직 상승', (*tcp[:2], height, *abc), args.apply_speed


# ===== 실행 진입점 =====
if __name__ == '__main__':
    raise SystemExit(main())
