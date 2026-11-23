"""YZ 벽에 접근해 헤라 각도를 바꾸며 위로 도포한다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 진입점: ros2 run rokey1 apply_putty_plane. 생성되는 ROS 노드명은 /dsr01/apply_putty_plane.
# 이 파일은 설정값과 기하 경로(날 끝 위치·헤라 각도 → TCP 자세)만 만든다.
# 실제 ROS 서비스 호출은 putty_motion.run_robot, 접촉 탐색·누르기 판정은 contact_search가 맡는다.
# 세 파일은 별도 노드가 아니라 같은 프로세스 안의 Python 모듈이다.
#
# --contact-search 사용 시 기본 동작(press_method='direct')의 순서:
#   벽 앞 정렬 → 컴플라이언스를 켠 채 바닥 -Z 탐색 → 0.5 mm 들기
#   → 측정된 벽 X(+1 mm)까지 위치 이동해 벽에 붙기 → X 강성 200 + BASE +X 3 N 힘제어(상대 모드)
#   → 헤라 각도를 25°→40°로 바꾸며 MoveLine 20구간으로 상향 도포 → 힘·컴플라이언스 해제 → 후퇴
# 뜨기 직후처럼 로봇 뒤쪽(X<0)에서 출발하면 벽 앞 정렬 전에 기둥을 피하는 경로를 먼저 지난다:
#   이송 높이 확보 → 경유지로 직선 이동(movel) → 홈 위치로 관절 이동(movej, 시계) — 헤라는 뜬 자세 그대로
#   → 벽 앞 정렬(movej, 시계)에서 도포 자세로 바뀐다.

import math
import sys
from pathlib import Path

from .putty_geometry import (
    BLADE_REFERENCE_ABC, SAMPLES, axes_to_zyz, cross, zyz_axes,
)
from .putty_motion import (
    HOME_TRANSFER_STEP, WALL_TRANSFER_STEP, make_parser, run_robot as run_motion, validate as validate_common,
)
from .pickup_spatula import (
    HOLDER_DETOUR_POSE, JOINT_SPEED_DEG_S, JOINT_ACCELERATION_DEG_S2, RECORDED_HOME_POSE, check_pillar_clearance,
)
from .contact_search import validate_search, seek_floor_then_wall


# ===== 하이퍼파라미터 =====
PLANE_AXIS = {'xy': 2, 'yz': 0, 'xz': 1}  # 평면 이름 → 그 평면의 법선 축 번호 (X=0, Y=1, Z=2)

# 날 끝이 벽에 닿는 BASE XYZ (mm). 20° 시작 자세 기준 실측값이다.
# 실측 경위: 20° 자세에서 바닥 접촉 후 +X로 27.1~27.5 mm 가서 벽에 막혔다(TCP X ≈ 775.3).
# 모델(TCP+120 mm)로 계산한 X보다 22.5 mm 멀어서 그만큼 보정했다. Z는 실행 중 바닥 탐색이 다시 맞춘다.
# 시작 각도를 30°로 되돌린다면 X는 796.94(30° 실측 보정값)를 쓴다.
START_CONTACT = (819.41, -15.52, 27.20)
STROKE_END_Z = 206.0          # 도포가 끝나는 날 끝 BASE Z (mm)
STROKE_HEIGHT = round(STROKE_END_Z-START_CONTACT[2], 3)  # 27.2 → 206 이므로 178.8 mm
WALL_EDGE_OFFSET = (0.0, 0.0, 120.0)  # TCP → 날 끝, Tool 좌표계 (헤라 길이 약 120 mm)

PRESS_FORCE_N = 3.0           # 상향 도포 동안 BASE +X로 벽을 누르는 목표 힘 (사용자 지정)
# 헤라 각도 = 벽(수직면)과 헤라 사이 각도. 0°는 벽에 붙어 선 상태, 90°는 바닥과 평행. 바닥 기준으로는 90°-각도.
# 도포하면서 시작 → 끝 각도로 선형 변화한다.
# 시작 25°(바닥 기준 65°): 2026-11-19 ikin 실측에서 20°는 도포 시작부 J3 37.8°로 특이점 검사(41°)에 걸렸다.
#   25°는 J3 최소 52.0°. (인수인계 경위: 날 위쪽 면의 퍼티까지 벽에 닿게 30°→20°로 낮췄었다.)
# 끝 40°: 각도가 클수록 TCP가 낮고 로봇 쪽에 가까워져 팔이 덜 펴진다(팔꿈치 특이점 회피).
#         실측상 30° 고정은 TCP Z≈306, 30→15°는 Z≈225에서 특이점 알람(3205)에 들어갔다.
BLADE_START_ANGLE_DEG = 25.0
BLADE_END_ANGLE_DEG = 40.0
# 벽 접근 시 자세 정렬 높이. ikin 실측: X 725에서 Z 245는 J3=39.0°(3205 알람 발생 각),
# Z 200은 J3=48.6°로 팔꿈치 특이점 여유 확보. 공용 transfer_z(245)는 다른 작업용이라 그대로 둔다.
WALL_APPROACH_Z = 200.0
# 벽으로 다가가는 동안 날 끝을 계획 높이보다 이만큼 띄운다. 헤라가 깊게 잡히면 날이 바닥과 거의 붙어
# 수평 접근 중 바닥/모서리 퍼티에 걸려 안전 충돌 감지(7060)가 났다. 바닥 높이는 순응제어 바닥 탐색이 찾는다.
APPROACH_CLEARANCE_Z = 10.0
WALL_BACKOFF_MM = 10.0       # 날 끝을 벽에서 이만큼 띄워 바닥에 내린다. 헤라 파지 깊이 편차로 5 mm에서 벽 접촉(3501) → 10 mm
SEARCH_DISTANCE_MM = 30.0    # −Z / +X 탐색 최대 거리
SEARCH_SPEED_MM_S = 2.0      # 접촉 탐색 속도
FLOOR_LIFT_MM = 0.5          # 바닥 감지 후 벽 탐색 전 +Z 들기 (날이 바닥을 끌며 생기는 마찰 오판 방지)
PRESS_METHOD = 'direct'       # 측정된 벽 위치로 바로 가서 +X 힘제어 도포(벽 탐색 생략). hybrid=힘으로 벽 탐색
PRESS_STIFFNESS = 1000.0     # position 방식 X 강성 N/m (11 N → 약 11 mm 눌림)
PRESS_TOLERANCE_N = 2.0      # 도포 중 힘 허용오차(경고 기준). 목표 힘보다 작아야 한다
CONTACT_CONFIRM = 2          # 접촉 판정에 필요한 연속 감지 횟수 (약 0.1~0.2 s)

# 누르기 방식별 '벽에 닿은 뒤' 동작 설명. 실행 전 설정 로그에 그대로 출력한다.
PRESS_METHOD_SUMMARY = {
    'direct': '측정된 벽 X(+1 mm)까지 위치 이동 → +X 힘제어 유지 도포 (힘 부족은 경고만)',
    'hybrid': '힘으로 벽 탐색 → +X 힘제어 유지 도포 (힘 부족은 경고만)',
    'position': '힘으로 벽 탐색 → 목표 힘/강성만큼 위치로 밀어 넣고 도포 (힘 부족은 경고만)',
    'force': 'set_desired_force 후 실측 힘 안정 확인 → 유지 도포 (힘 부족 지속 시 중단)',
}


# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    args = make_plane_parser().parse_args()
    try:
        resolve_plane_args(args)
        validate(args)
        validate_search(args)
        print_settings(args)
        run_robot(args, motion_builder=application_moves)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'평면 도포 중단: {exc}', file=sys.stderr)
        return 1


# 이동 전에 실제로 쓰일 경로·힘 설정을 로그로 남긴다. 현장에서 값 확인용이다.
def print_settings(args):
    print(f'[실행 소스] {Path(__file__).resolve()}', flush=True)
    print(f'[도포 설정] 날 {tuple(args.start)} → {tuple(args.end)} | '
          f'{args.blade_angle:g}→{args.blade_end_angle:g}° | '
          f'접촉 탐색={args.contact_search}', flush=True)
    if not args.contact_search:
        return
    print(f'[접촉 후 동작] {args.press_method}: {PRESS_METHOD_SUMMARY[args.press_method]} | '
          f'+X {args.press_force:g} N, 허용오차 ±{args.force_tolerance:g} N | '
          f'대기 {args.force_wait_timeout:g}s / 안정 {args.force_stable_time:g}s / '
          f'이탈 {args.force_loss_timeout:g}s', flush=True)
    print(f'[탐색 설정] 바닥 {args.floor_force:g} N / 벽 {args.wall_force:g} N | '
          f'{args.search_speed:g} mm/s, 최대 {args.search_distance:g} mm | '
          f'연속 {args.contact_confirm}회 | 들기 {args.floor_lift:g} mm | '
          f'강성 {list(args.force_stiffness)} | '
          f'위치누르기 강성 {args.press_stiffness:g} N/m ±{args.press_tolerance:g} N', flush=True)


# 터미널 옵션을 정의한다. 여기의 default는 위쪽 설정부 상수와 연결된다.
def make_plane_parser():
    parser = make_parser()
    parser.description = __doc__
    # 도포 실행은 특이점 자동회피/알람 감시/힘 제어 진입 차단을 항상 적용한다.
    parser.set_defaults(edge_offset=WALL_EDGE_OFFSET, singularity_guard=True)
    parser.add_argument('--plane', choices=PLANE_AXIS, default='yz')
    parser.add_argument('--normal-sign', choices=('positive', 'negative'), default=None,
                        help='면 바깥 방향: 기본 YZ 벽은 negative (+X 접근)')
    parser.add_argument('--start', nargs=3, type=float, help='날 시작 BASE XYZ mm')
    parser.add_argument('--end', nargs=3, type=float, help='날 끝 BASE XYZ mm')
    parser.add_argument('--stroke-height', type=float, default=STROKE_HEIGHT,
                        help='plane에서 end 생략 시 시작점에서 +Z로 바를 높이 mm')
    parser.add_argument('--blade-angle', type=float, default=BLADE_START_ANGLE_DEG,
                        help='도포 시작 시 날과 평면 사이 각도, deg')
    parser.add_argument('--blade-end-angle', type=float, default=BLADE_END_ANGLE_DEG,
                        help='도포 종료 시 날과 평면 사이 각도, deg')
    parser.add_argument('--samples', type=int, default=SAMPLES, help='각도 변화 도포 경로의 자세 개수')
    parser.add_argument('--approach-distance', type=float, default=50.0,
                        help='날 기준점의 면 법선 방향 접근/후퇴 거리 mm')
    parser.add_argument('--joint-speed', type=float, default=JOINT_SPEED_DEG_S,
                        help='벽 앞으로 관절 이동(movej)하는 속도 deg/s')
    parser.add_argument('--joint-acceleration', type=float, default=JOINT_ACCELERATION_DEG_S2,
                        help='벽 앞으로 관절 이동(movej)하는 가속도 deg/s²')
    parser.add_argument('--contact-search', action='store_true', help='도포 전 −Z/+X 접촉 탐색')
    parser.add_argument('--floor-force', type=float, help='−Z 탐색의 외력 변화 감지 기준 N, 필수')
    parser.add_argument('--wall-force', type=float, help='+X 탐색의 외력 변화 감지 기준 N, 필수')
    parser.add_argument('--search-distance', type=float, default=SEARCH_DISTANCE_MM, help='각 축 탐색 최대 거리 mm')
    parser.add_argument('--search-speed', type=float, default=SEARCH_SPEED_MM_S, help='접촉 탐색 속도 mm/s')
    parser.add_argument('--wall-approach-z', type=float, default=WALL_APPROACH_Z,
                        help='벽 앞에서 자세를 정렬하는 이송 높이(TCP Z). 높으면 팔이 펴져 특이점에 가까워진다')
    parser.add_argument('--approach-clearance-z', type=float, default=APPROACH_CLEARANCE_Z,
                        help='접촉 탐색 시 벽으로 다가갈 때 날 끝을 바닥(계획 높이)보다 띄우는 높이 mm')
    parser.add_argument('--wall-backoff', type=float, default=WALL_BACKOFF_MM,
                        help='접촉 탐색 시 날 끝을 벽에서 띄워 바닥에 내리는 거리 mm (날 끝 기준)')
    parser.add_argument('--floor-below-gap', type=float, default=None,
                        help='바닥 탐색을 틈 좌표 Z 아래 이만큼까지만. 못 닿으면 그 자리를 바닥으로 봄 mm '
                             '(없으면 search-distance 까지 내려가고 못 찾으면 실패)')
    parser.add_argument('--floor-lift', type=float, default=FLOOR_LIFT_MM,
                        help='바닥 감지 후 벽 탐색 전에 +Z로 띄우는 거리 mm. 바닥 마찰 오판 방지')
    parser.add_argument('--contact-confirm', type=int, default=CONTACT_CONFIRM,
                        help='접촉 판정에 필요한 연속 감지 횟수')
    parser.add_argument('--press-method', choices=('direct', 'hybrid', 'position', 'force'), default=PRESS_METHOD,
                        help='direct: 측정된 벽 위치로 바로 가서 힘제어 도포(기본) / hybrid: 힘으로 벽 탐색 후 힘제어 / '
                             'position: 위치로 밀어 넣기 / '
                             'force: set_desired_force 후 힘 도달 대기')
    parser.add_argument('--press-stiffness', type=float, default=PRESS_STIFFNESS,
                        help='position 방식의 BASE X 강성 N/m. 눌림 깊이 = 목표 힘/강성')
    parser.add_argument('--press-tolerance', type=float, default=PRESS_TOLERANCE_N,
                        help='position 방식 도포 중 허용 힘 오차 N')
    parser.add_argument('--press-force', type=float, default=PRESS_FORCE_N,
                        help='벽 접촉 탐색 후 도포 중 BASE +X 유지 힘 N')
    parser.add_argument('--force-mode', choices=('absolute', 'relative'), default='relative',
                        help='목표 힘 기준. 실물 샌딩과 같은 상대 모드가 기본. 감시는 설정 직전 Fx를 기준으로 뺀다')
    # 누르는 방향(BASE X) 강성이 높으면 힘 명령을 줘도 로봇이 벽까지 가지 못한다
    # (3000 N/m에서 11 N ≈ 3.7 mm). 실물 동작하는 샌딩과 같이 누르는 축만 부드럽게 둔다.
    parser.add_argument('--force-stiffness', type=float, nargs=6,
                        default=(200.0, 3000.0, 3000.0, 200.0, 200.0, 200.0))
    # 새 실측 감시 기준. 기존 이동·각도·목표 힘 설정과 구분한다.
    parser.add_argument('--force-tolerance', type=float, default=1.0, help='|BASE Fx| 허용오차 N')
    parser.add_argument('--force-limit', type=float, default=None,
                        help='즉시 중단할 |BASE Fx| 상한 N. 기본 목표+허용오차×3')
    parser.add_argument('--force-wait-timeout', type=float, default=30.0, help='상승 전 힘 형성 대기 한도 s')
    parser.add_argument('--force-stable-time', type=float, default=0.5, help='출발 전 힘 안정 유지 시간 s')
    parser.add_argument('--force-loss-timeout', type=float, default=0.5, help='도포 중 힘 부족 허용 시간 s')
    return parser


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args, motion_builder):
    probe = (lambda io: seek_floor_then_wall(io, args)) if getattr(args, 'contact_search', False) else None
    # motion_builder는 TCP 경로를 만드는 Python 함수다. ROS 메시지로 전송하는 콜백이 아니다.
    return run_motion(args, motion_builder=motion_builder, allow_real=True,
                      continuous=True, contact_probe=probe,
                      press_force=getattr(args, 'press_force', 6.0) if probe is not None else None)


# 생략된 면 방향/접촉 시작점/끝점을 기본 설정과 도포 길이로 채운다.
def resolve_plane_args(args):
    """단일 평면 케이스의 접촉 시작점과 도포 높이를 실제 입력 좌표로 바꾼다."""
    if args.normal_sign is None:
        args.normal_sign = 'negative' if args.plane == 'yz' else 'positive'
    if args.start is None:
        args.start = START_CONTACT
    if args.end is None:
        if not math.isfinite(args.stroke_height) or args.stroke_height <= 0:
            raise ValueError('도포 높이는 양의 유한한 값이어야 합니다.')
        x, y, z = args.start
        args.end = (x, y, round(z+args.stroke_height, 6))


# 숫자 범위·좌표·길이 등이 유효한지 확인한다. 역기구학이나 충돌 검사는 아니다.
def validate(args):
    validate_common(args)
    end_angle = getattr(args, 'blade_end_angle', 30.0)
    samples = getattr(args, 'samples', SAMPLES)
    if not math.isfinite(end_angle) or not 0 < end_angle < 90:
        raise ValueError('종료 각도는 0도 초과 90도 미만이어야 합니다.')
    if not isinstance(samples, int) or not 2 <= samples <= 100:
        raise ValueError('연속 도포 samples는 2~100 정수여야 합니다.')
    for name in ('joint_speed', 'joint_acceleration'):
        value = getattr(args, name, 1.0)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다.')
    stiffness = getattr(args, 'force_stiffness', (3000, 3000, 3000, 200, 200, 200))
    if len(stiffness) != 6 or not all(math.isfinite(v) and v >= 0 for v in stiffness):
        raise ValueError('force-stiffness는 0 이상의 유한한 값 6개여야 합니다.')
    if args.start is None or args.end is None:
        raise ValueError('plane 케이스는 --start 및 --end가 필요합니다.')
    if args.plane not in PLANE_AXIS or args.normal_sign not in ('positive', 'negative'):
        raise ValueError('지원하지 않는 평면 또는 면 방향입니다.')
    if not all(map(math.isfinite, (*args.start, *args.end, args.blade_angle,
                                   args.approach_distance))):
        raise ValueError('도포 좌표·각도·접근 거리는 유한해야 합니다.')
    if not 0 < args.blade_angle < 90 or args.approach_distance <= 0:
        raise ValueError('날 각도는 0~90도 사이, 접근 거리는 양수여야 합니다.')
    axis = PLANE_AXIS[args.plane]
    if abs(args.start[axis]-args.end[axis]) > 1e-6:
        raise ValueError('시작점과 끝점은 지정 평면의 같은 높이에 있어야 합니다.')
    if math.dist(args.start, args.end) <= 1e-6:
        raise ValueError('시작점과 끝점이 같을 수 없습니다.')


# 벽의 법선과 도포 진행 방향으로 헤라 면·길이·폭 방향을 만든다.
def surface_frame(args, blade_angle=None):
    """픽업 상태 면 +X/날 길이 +Z 가정을 원하는 면과 이동 방향에 정렬한다."""
    normal = tuple((1.0 if args.normal_sign == 'positive' else -1.0)
                   if i == PLANE_AXIS[args.plane] else 0.0 for i in range(3))
    delta = tuple(b-a for a, b in zip(args.start, args.end))
    length = math.sqrt(sum(v*v for v in delta))
    travel = tuple(v/length for v in delta)
    angle = math.radians(args.blade_angle if blade_angle is None else blade_angle)
    # 퍼티 면은 벽을 향하고, 손잡이 쪽은 벽 밖 자유 공간에 둔다.
    face = tuple(-math.cos(angle)*n+math.sin(angle)*t for n, t in zip(normal, travel))
    handle = tuple(math.sin(angle)*n+math.cos(angle)*t for n, t in zip(normal, travel))
    width = cross(handle, face)
    axes = tuple(tuple(face[i]*v[0]+width[i]*v[1]+handle[i]*v[2] for i in range(3))
                 for v in zyz_axes(BLADE_REFERENCE_ABC))
    return normal, axes


# 접근→도포→후퇴 순서의 (설명, TCP XYZABC, 속도)를 yield한다. 계산만 수행한다.
def application_moves(current, args):
    validate(args)
    yield from single_stroke_moves(current, args)


# 날 접촉점 경로와 각도를 선형 보간하고 회전된 오프셋을 빼서 TCP를 계산한다.
def stroke_poses(args):
    """날 기준점을 선 위에 두고 각도별 TCP 오프셋을 다시 계산한다."""
    end_angle = getattr(args, 'blade_end_angle', 30.0)
    # 각도가 고정이면 시작·끝 두 점이면 충분하다. 각도가 변하면 samples개로 나눠 자세를 바꾼다.
    count = 2 if end_angle == args.blade_angle else getattr(args, 'samples', SAMPLES)
    for index in range(count):
        # 진행률 u(0→1) 하나로 날 끝 위치와 헤라 각도를 함께 보간한다.
        u = index/(count-1)
        tilt = args.blade_angle+(end_angle-args.blade_angle)*u
        normal, axes = surface_frame(args, tilt)
        edge = tuple(a+(b-a)*u for a, b in zip(args.start, args.end))
        # 자세가 바뀔 때 Tool +Z 길이의 BASE X/Z 성분도 바뀌므로 매 지점에서 다시 계산한다.
        offset = tuple(sum(axes[j][i]*args.edge_offset[j] for j in range(3)) for i in range(3))
        # 날 기준점을 유지하는 Tool 오프셋 보정만 적용한다.
        tcp = tuple(edge[i]-offset[i] for i in range(3))
        yield edge, (*tcp, *axes_to_zyz(axes)), tilt


# 로봇 뒤쪽(용기·보관대)에서 벽 쪽으로 나오는 경로. 보관대 옆 기둥 방향(약 128°)을 movej로 지나지 않게,
# 경유지까지는 직선(movel)으로 오고 그 다음 홈 위치까지 시계 방향 관절 이동한다.
# 헤라에 퍼티가 올라가 있으므로 자세는 현재(뜬 각도) 그대로 두고, 도포 자세로는 벽 앞 정렬에서 바꾼다.
def behind_robot_exit(current, height, args):
    lifted = (*current[:2], height)
    detour = (*HOLDER_DETOUR_POSE[:2], height, *current[3:])
    check_pillar_clearance((lifted, detour))
    yield '기둥 피해 경유지로 직선 이동', detour, args.transfer_speed
    yield HOME_TRANSFER_STEP, (*RECORDED_HOME_POSE[:2], height, *current[3:]), args.transfer_speed


# 한 구간을 위한 이송 높이 확보·접근·도포·법선 후퇴 경로를 구성한다.
def single_stroke_moves(current, args):
    validate(args)
    normal, _ = surface_frame(args)
    poses = list(stroke_poses(args))
    start, end = poses[0][1], poses[-1][1]
    searching = getattr(args, 'contact_search', False)
    clearance = (0.0, 0.0, getattr(args, 'approach_clearance_z', 0.0) if searching else 0.0)
    approach = (*(start[i]+normal[i]*args.approach_distance+clearance[i] for i in range(3)), *start[3:])
    retreat = (*(end[i]+normal[i]*args.approach_distance for i in range(3)), *end[3:])
    height = max(current[2], args.transfer_z, approach[2])
    # 벽 앞 정렬 높이는 팔이 덜 펴지도록 낮춘다(팔꿈치 특이점 회피). 접근점보다 낮게는 내려가지 않는다.
    align_z = max(approach[2], min(height, getattr(args, 'wall_approach_z', height)))
    yield '면 접근 전 이송 높이 확보', (*current[:2], height, *current[3:]), args.transfer_speed
    if current[0] < 0:
        yield from behind_robot_exit(current, height, args)
    # 벽 앞으로 오는 긴 이동은 관절 이동(movej, 시계 방향)이다. 실행부(putty_motion)가 이 설명으로 구분한다.
    yield WALL_TRANSFER_STEP, (*approach[:2], align_z, *start[3:]), args.transfer_speed
    yield '면 바깥 접근점', approach, args.transfer_speed
    # 접촉 탐색 시에는 날 끝이 벽에서 wall_backoff만큼 떨어진 곳에 내린다. 자세가 같으므로
    # TCP를 법선 방향으로 옮기면 날 끝도 같은 거리만큼 옮겨진다. 실행부는 이 오프셋을 빼서 경로를 보정한다.
    backoff = getattr(args, 'wall_backoff', 0.0) if searching else 0.0
    args.approach_offset = tuple(normal[i]*backoff+clearance[i] for i in range(3))
    touch = (*(start[i]+args.approach_offset[i] for i in range(3)), *start[3:])
    yield '면에 수직 접근', touch, args.apply_speed
    for index, (_, target, tilt) in enumerate(poses[1:], 2):
        label = ('평면 고정 각도 도포' if args.blade_angle == getattr(args, 'blade_end_angle', 30.0)
                 else f'평면 각도 변화 도포 {index}/{len(poses)} ({tilt:.2f}도)')
        yield label, target, args.apply_speed
    yield '면 법선 방향 후퇴', retreat, args.apply_speed


if __name__ == '__main__':
    raise SystemExit(main())
