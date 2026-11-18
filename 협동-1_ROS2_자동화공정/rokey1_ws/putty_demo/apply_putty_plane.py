"""YZ 벽에 접근해 헤라 각도를 바꾸며 위로 도포한다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 진입점: ros2 run rokey1 apply_putty_plane. 생성되는 ROS 노드명은 /dsr01/apply_putty_plane.
# 이 파일은 설정·기하 경로를 만들고, 실제 ROS 요청은 putty_motion.run_robot에 위임한다.
# 설정값 → 날 접촉 경로/기울기 계산 → TCP 경로 계산 → ROS 서비스 실행 순서다.
# --contact-search 사용 시: 지정점 → 바닥 -Z 감지 → 벽 +X 감지 → 힘 도달 확인 → 힘 유지 도포.
# contact_search/putty_geometry/putty_motion은 별도 노드가 아니라 같은 프로세스의 Python 모듈이다.

import math
import sys
from pathlib import Path

from .putty_geometry import (
    PICKUP_ABC, SAMPLES, axes_to_zyz, cross, zyz_axes,
)
from .putty_motion import make_parser, run_robot as run_motion, validate as validate_common
from .contact_search import validate_search, seek_floor_then_wall


# ===== 하이퍼파라미터 =====
PLANE_AXIS = {'xy': 2, 'yz': 0, 'xz': 1}
# 측정점 [784.04,-15.52,32.20]에 사용자 지정 BASE 보정 X +5 / Z -5 mm 적용.
START_CONTACT = (789.04, -15.52, 27.20)
STROKE_HEIGHT = 180.0
WALL_EDGE_OFFSET = (0.0, 0.0, 120.0)
# 현재 실물 YZ 벽 도포 설정. 값은 기존 설정을 그대로 모았다.
PRESS_FORCE_N = 6.0          # 위로 바르는 동안 BASE +X 유지 힘
BLADE_START_ANGLE_DEG = 20.0  # 벽과 헤라 사이 시작 각도
BLADE_END_ANGLE_DEG = 10.0    # 벽과 헤라 사이 종료 각도
SEARCH_DISTANCE_MM = 20.0    # −Z / +X 탐색 최대 거리
SEARCH_SPEED_MM_S = 2.0      # 접촉 탐색 속도

# ===== 로봇 초기화 설정 =====
# 공통 runner가 로봇 시스템과 TCP를 확인한다.


# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    args = make_plane_parser().parse_args()
    try:
        resolve_plane_args(args)
        validate(args)
        validate_search(args)
        print(f'[실행 소스] {Path(__file__).resolve()}', flush=True)
        print(f'[도포 설정] 날 {tuple(args.start)} → {tuple(args.end)} | '
              f'{args.blade_angle:g}→{args.blade_end_angle:g}° | '
              f'접촉 탐색={args.contact_search}', flush=True)
        if args.contact_search:
            print(f'[접촉 후 동작] 추가 위치 이동 없음 → +X {args.press_force:g} N '
                  f'실측 안정 확인 → 유지 도포 | 허용오차 ±{args.force_tolerance:g} N | '
                  f'대기 {args.force_wait_timeout:g}s / 안정 {args.force_stable_time:g}s / '
                  f'이탈 {args.force_loss_timeout:g}s', flush=True)
        run_robot(args, motion_builder=application_moves)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'평면 도포 중단: {exc}', file=sys.stderr)
        return 1


# 터미널 옵션을 정의한다. 여기의 default는 위쪽 설정부 상수와 연결된다.
def make_plane_parser():
    parser = make_parser(include_j6=False)
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
    parser.add_argument('--contact-search', action='store_true', help='도포 전 −Z/+X 접촉 탐색')
    parser.add_argument('--floor-force', type=float, help='−Z 탐색의 외력 변화 감지 기준 N, 필수')
    parser.add_argument('--wall-force', type=float, help='+X 탐색의 외력 변화 감지 기준 N, 필수')
    parser.add_argument('--search-distance', type=float, default=SEARCH_DISTANCE_MM, help='각 축 탐색 최대 거리 mm')
    parser.add_argument('--search-speed', type=float, default=SEARCH_SPEED_MM_S, help='접촉 탐색 속도 mm/s')
    parser.add_argument('--press-force', type=float, default=PRESS_FORCE_N,
                        help='벽 접촉 탐색 후 도포 중 BASE +X 유지 힘 N')
    parser.add_argument('--force-mode', choices=('absolute', 'relative'), default='relative',
                        help='목표 힘 기준. 현재 현장 설정인 상대 모드를 기본으로 유지')
    parser.add_argument('--force-stiffness', type=float, nargs=6,
                        default=(3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0))
    # 새 실측 감시 기준. 기존 이동·각도·목표 힘 설정과 구분한다.
    parser.add_argument('--force-tolerance', type=float, default=1.0, help='|BASE Fx| 허용오차 N')
    parser.add_argument('--force-wait-timeout', type=float, default=30.0, help='상승 전 힘 형성 대기 한도 s')
    parser.add_argument('--force-stable-time', type=float, default=0.5, help='출발 전 힘 안정 유지 시간 s')
    parser.add_argument('--force-loss-timeout', type=float, default=0.5, help='도포 중 힘 부족 허용 시간 s')
    return parser


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args, motion_builder):
    probe = (lambda io: seek_floor_then_wall(io, args)) if getattr(args, 'contact_search', False) else None
    # motion_builder는 TCP 경로를 만드는 Python 함수다. ROS 메시지로 전송하는 콜백이 아니다.
    return run_motion(args, motion_builder=motion_builder, use_j6=False, allow_real=True,
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
    validate_common(args, include_j6=False)
    end_angle = getattr(args, 'blade_end_angle', 30.0)
    samples = getattr(args, 'samples', SAMPLES)
    if not math.isfinite(end_angle) or not 0 < end_angle < 90:
        raise ValueError('종료 각도는 0도 초과 90도 미만이어야 합니다.')
    if not isinstance(samples, int) or not 2 <= samples <= 100:
        raise ValueError('연속 도포 samples는 2~100 정수여야 합니다.')
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
                 for v in zyz_axes(PICKUP_ABC))
    return normal, axes


# 접근→도포→후퇴 순서의 (설명, TCP XYZABC, 속도)를 yield한다. 계산만 수행한다.
def application_moves(current, args):
    validate(args)
    yield from single_stroke_moves(current, args)


# 날 접촉점 경로와 각도를 선형 보간하고 회전된 오프셋을 빼서 TCP를 계ㅁ산한다.
def stroke_poses(args):
    """날 기준점을 선 위에 두고 각도별 TCP 오프셋을 다시 계산한다."""
    end_angle = getattr(args, 'blade_end_angle', 30.0)
    count = 2 if end_angle == args.blade_angle else getattr(args, 'samples', SAMPLES)
    for index in range(count):
        u = index/(count-1)
        # 이동 거리와 기울기를 동일한 진행률 u로 선형 보간한다.
        progress = u
        tilt = args.blade_angle+(end_angle-args.blade_angle)*u
        normal, axes = surface_frame(args, tilt)
        edge = tuple(a+(b-a)*progress for a, b in zip(args.start, args.end))
        # 자세가 바뀔 때 Tool +Z 길이의 BASE X/Z 성분도 바뀌므로 매 지점에서 다시 계산한다.
        offset = tuple(sum(axes[j][i]*args.edge_offset[j] for j in range(3)) for i in range(3))
        # 날 기준점을 유지하는 Tool 오프셋 보정만 적용한다.
        tcp = tuple(edge[i]-offset[i] for i in range(3))
        yield edge, (*tcp, *axes_to_zyz(axes)), tilt


# 한 구간을 위한 이송 높이 확보·접근·도포·법선 후퇴 경로를 구성한다.
def single_stroke_moves(current, args):
    validate(args)
    normal, _ = surface_frame(args)
    poses = list(stroke_poses(args))
    start, end = poses[0][1], poses[-1][1]
    approach = (*(start[i]+normal[i]*args.approach_distance for i in range(3)), *start[3:])
    retreat = (*(end[i]+normal[i]*args.approach_distance for i in range(3)), *end[3:])
    height = max(current[2], args.transfer_z, approach[2])
    yield '면 접근 전 이송 높이 확보', (*current[:2], height, *current[3:]), args.transfer_speed
    yield '면 접근점 위로 이송하며 자세 정렬', (*approach[:2], height, *start[3:]), args.transfer_speed
    yield '면 바깥 접근점', approach, args.transfer_speed
    yield '면에 수직 접근', start, args.apply_speed
    for index, (_, target, tilt) in enumerate(poses[1:], 2):
        label = ('평면 고정 각도 도포' if args.blade_angle == getattr(args, 'blade_end_angle', 30.0)
                 else f'평면 각도 변화 도포 {index}/{len(poses)} ({tilt:.2f}도)')
        yield label, target, args.apply_speed
    yield '면 법선 방향 후퇴', retreat, args.apply_speed


if __name__ == '__main__':
    raise SystemExit(main())
