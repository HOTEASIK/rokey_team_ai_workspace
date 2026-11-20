"""헤라를 기울인 채 날 방향으로 사선 진입해 퍼티를 뜨고 수직 상승한다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/scoop_putty. 측정한 '날 끝 접촉 자세'에서 기울인 뜨기 경로를 만든다.
# 동작 순서와 두산 API (모두 movel, 동기):
#   시스템/TCP 확인 → 실물 파지 수동 확인 → 출발 전 관절 검사(ikin) → 이송 높이 확보
#   → 그 자리에서 기울인 자세로 정렬 → 사선 시작점 위 → 사선 시작점
#   → 각도를 유지한 채 -X·-Z 사선 진입(날이 퍼티를 썰듯이). --scoop-pass N이면 끝점을 +X로 (N-1)×20 mm 옮긴다.
#   [실물] → 컴플라이언스(task_compliance_ctrl) → -X로 밀며 벽 접촉 감지(amovel + get_tool_force → stop)
#          → -X 힘 유지(set_desired_force, 상대) 상태로 +Z 긁어 올리기(movel) → 힘·컴플라이언스 해제
#   [가상] → 기록된 벽 위치까지 -X 이동 → 긁어 올리기 높이까지 +Z (위치 이동만, 시뮬레이터 외력은 믿지 않는다)
#   → 남은 높이 수직 상승(movel)
# 보관대(헤라 집기 위치) 바로 옆이라 직선 이동(movel)만으로 가고, 위에서 본 헤라 방향(J6)도 바꾸지 않는다.
# --scoop-angle은 헤라 판 기울기(수평면 기준)와 사선 진입각의 공통 값이다.
# 통신: GetRobotSystem/GetCurrentTcp/GetCurrentPosx/MoveLine (+ ikin 검사용, 실물 힘제어용 서비스)

import argparse
import math
import sys

from .contact_search import press_force_request, seek_axis
from .pickup_spatula import HOLDER_TRANSFER_Z_MM

from .putty_geometry import (
    BLADE_WIDTH, BLADE_THICKNESS, EXPECTED_TCP, ROBOT_ID,
    axes_to_zyz, format_values, orientation_error, rotate_base_y, zyz_axes,
)


# ===== 설정 / 하이퍼파라미터 =====
# 2026-11-19 측정. 헤라를 집은 자세 그대로(수직, 축 이동만) 날 폭 가운데 끝이 용기의 -X 쪽 맨 끝
# 바닥 구석에 닿았을 때의 TCP BASE XYZABC (mm, deg). 헤라 면은 -X를 보고 있다(보관대 방향 때문).
# 가정: 이때 헤라 -X 면은 -X 벽에, 날 끝은 바닥에 닿아 있다.
TAUGHT_SCOOP_POSE = (-553.78, 82.25, 122.70, 160.95, -179.05, 160.87)
# TCP → 날 끝(폭 가운데), Tool 좌표 mm. 2026-11-19 확정: 147.7 (실물 뜨기에서 130보다 끝점이 더 잘 맞았다).
#   유래: 수직 자세 TAUGHT_SCOOP_POSE와 약 29° 기울인 자세 (-479.01, 78.58, 103.63, 170.27, 150.85, 170.58)가
#   같은 바닥 구석에 닿았다는 조건으로 역산한 값 (0.5, 0, 147.7).
#   참고: 플랜지→날 끝 실측 약 335 mm와 '플랜지→TCP 약 205 mm(정기구학 추정)'로 계산하면 130이 나오지만
#   실물 결과와 맞지 않았다. TCP(GripperDA_v1) 오프셋 실제 값 확인 전까지는 실물 결과를 따른다.
#   예전 가정값 120으로는 45° 끝점에서 날 끝이 바닥에 닿지 않았다.
#   도포(apply)는 120 모델 + START_CONTACT 실측 보정 상태로 둔다(접촉 탐색으로 동작 확인됨).
SCOOP_EDGE_OFFSET = (0.0, 0.0, 147.7)
SCOOP_CONTAINER_SIZE = (110.0, 120.0, 40.0)  # 용기 안쪽 X·Y·깊이 mm (X 110, Y 120)
SCOOP_ANGLE_RANGE_DEG = (45.0, 60.0)  # 헤라 판 기울기(수평면 기준) 허용 범위
SCOOP_DEFAULT_ANGLE_DEG = 55.0         # 기본 각도. 45°는 어깨 특이점(아래 --scoop-angle 주석 참고)
TAUGHT_LIFT_MM = 120.0
# 여러 번 뜰 때: 앞에서 퍼낸 자리를 피하려고 N번째 뜨기의 썰기 끝점을 +X로 (N-1)×20 mm 옮긴다.
# 용기 안쪽 X가 110 mm라 4회(+60 mm)까지만 허용한다.
SCOOP_PASS_STEP_MM = 20.0
SCOOP_MAX_PASS = 4
# 썰기 끝점에서 -X 벽까지 밀어 퍼티를 모은다. 탐색 값은 반납·도포 접촉 탐색과 같다.
WALL_CONTACT_FORCE_N = 5.0     # 탐색 시작 대비 Fx 변화가 이 이상이면 벽(또는 벽 앞에 모인 퍼티)에 닿은 것으로 본다
WALL_CONTACT_CONFIRM = 2       # 연속 감지 횟수(순간 잡음 오판 방지)
WALL_SEARCH_SPEED_MM_S = 2.0
WALL_SEARCH_MARGIN_MM = 10.0   # 기록된 벽보다 이만큼 더 밀어도 닿지 않으면 멈춘다
# 컴플라이언스(X 강성 1 N/mm)라 막혀도 힘이 5 N 까지 안 오를 수 있음 → 이 시간 동안 이 거리도 못 가면 접촉
WALL_STALL_MM = 0.3
# 시작 TCP X 가 이보다 크면 벽(+X) 쪽 = 앞 틈 도포 직후 → 보관대 위로 먼저 옮긴 뒤 뜬다 (용기·보관대는 -X 쪽)
WALL_SIDE_X_MM = 0.0
WALL_STALL_SEC = 1.0
# 벽에 닿은 뒤 -X로 누르며 올라와 구석·벽 아래쪽 퍼티를 날 위로 긁어 올린다.
# 용기 벽(깊이 40 mm)이 끝나기 전에 힘을 풀어야 누르던 방향으로 튀어 나가지 않는다.
WALL_PUSH_FORCE_N = 3.0        # 도포의 벽 누르는 힘과 같다
WALL_SCRAPE_HEIGHT_MM = 25.0   # 힘을 유지하며 올라가는 높이. 0이면 벽에 닿은 뒤 바로 힘을 풀고 상승한다
WALL_SCRAPE_TOP_MARGIN_MM = 10.0  # 긁어 올리기는 용기 깊이보다 이만큼 낮은 곳에서 끝나야 한다
# X·Z를 부드럽게(1 mm당 1 N) 해서 벽·바닥에 닿아도 힘이 천천히 오르게 한다.
# 헤라가 길이 방향으로 단단해 위치제어로 닿으면 충돌 감지(7060)가 걸리기 때문이다(반납 바닥 탐색과 같은 이유).
SCRAPE_STIFFNESS = (1000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0)


# ===== 메인 =====
# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True, help='연결된 컨트롤러에 이동 요청')
    parser.add_argument('--taught-pose', nargs=6, type=float, default=TAUGHT_SCOOP_POSE,
                        help='헤라를 수직으로 쥔 채 날 끝이 용기 -X 끝 바닥에 닿았을 때의 BASE TCP XYZABC')
    parser.add_argument('--lift', type=float, default=TAUGHT_LIFT_MM, help='찍은 뒤 수직 상승 거리 mm')
    parser.add_argument('--entry-height', type=float, default=120.0,
                        help='썰기 끝점보다 높은 사선 진입 시작 높이 mm')
    # 기본 55°: 45°는 사선 시작점에서 손목 중심이 베이스 축 위(약 132 mm)로 와 어깨 특이점(3205)·도달 불가(1206)가
    # 났다(2026-11-19). 50°는 약 174 mm로 출발 전 관절 검사(180 mm)에 걸린다.
    parser.add_argument('--scoop-angle', '--entry-angle', dest='scoop_angle', type=float, default=SCOOP_DEFAULT_ANGLE_DEG,
                        help='헤라 판 기울기(수평면 기준)와 사선 진입 공통 각도 45~60 deg (권장 55~60)')
    parser.add_argument('--scoop-speed', type=float, default=3.0, help='사선 진입·상승 속도 mm/s')
    parser.add_argument('--transfer-speed', type=float, default=30.0, help='이송 속도 mm/s')
    parser.add_argument('--acceleration', type=float, default=20.0, help='mm/s²')
    # 보관대 근처에서 높이 올라가면 팔이 펴져 팔꿈치 특이점에 가까워진다. 집기와 같은 보관대 위 높이를 쓴다.
    parser.add_argument('--transfer-z', type=float, default=HOLDER_TRANSFER_Z_MM, help='이송 높이(TCP Z) mm')
    parser.add_argument('--edge-offset', nargs=3, type=float, default=SCOOP_EDGE_OFFSET,
                        help='TCP → 날 끝 Tool 오프셋 mm')
    parser.add_argument('--scoop-pass', type=int, default=1,
                        help=f'몇 번째 뜨기인지 1~{SCOOP_MAX_PASS}. 끝점을 +X로 (N-1)×{SCOOP_PASS_STEP_MM:g} mm 옮긴다')
    parser.add_argument('--wall-scrape-height', type=float, default=WALL_SCRAPE_HEIGHT_MM,
                        help='벽에 닿은 뒤 -X로 누르며 올라가는 높이 mm (0이면 바로 상승)')
    parser.add_argument('--scoop-pass-step', type=float, default=SCOOP_PASS_STEP_MM,
                        help='회차마다 썰기 끝점을 +X(몸체 쪽)로 옮기는 간격 mm')
    parser.add_argument('--wall-search-margin', type=float, default=WALL_SEARCH_MARGIN_MM,
                        help='벽 탐색: 기록된 벽보다 이만큼 더 밀어도 안 닿으면 멈춤 mm')
    parser.add_argument('--wall-search-speed', type=float, default=WALL_SEARCH_SPEED_MM_S,
                        help='벽 탐색 속도 mm/s')
    parser.add_argument('--wall-push-force', type=float, default=WALL_PUSH_FORCE_N,
                        help='긁어 올리는 동안 -X 로 벽을 누르는 힘 N')
    parser.add_argument('--wall-contact-confirm', type=int, default=WALL_CONTACT_CONFIRM,
                        help='벽 접촉 판정 연속 감지 횟수')
    parser.add_argument('--scrape-stiffness', nargs=6, type=float, default=SCRAPE_STIFFNESS,
                        help='벽 탐색·긁기 컴플라이언스 강성 [x,y,z N/m, rx,ry,rz Nm/rad]')
    parser.add_argument('--wall-contact-force', type=float, default=WALL_CONTACT_FORCE_N,
                        help='벽 탐색: 시작 대비 Fx 변화가 이 이상이면 접촉 N')
    parser.add_argument('--wall-stall-mm', type=float, default=WALL_STALL_MM,
                        help='벽 탐색: --wall-stall-sec 동안 이만큼도 못 나아가면 막힘 = 접촉 mm')
    parser.add_argument('--wall-stall-sec', type=float, default=WALL_STALL_SEC,
                        help='벽 탐색 막힘 판정 시간 s (0 = 힘으로만 판정)')
    args = parser.parse_args()
    try:
        validate_taught(args)
        run_robot(args)
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
    low, high = SCOOP_ANGLE_RANGE_DEG
    if not math.isfinite(args.scoop_angle) or not low <= args.scoop_angle <= high:
        raise ValueError(f'뜨기 각도는 {low:g}~{high:g}도여야 합니다.')
    if not all(map(math.isfinite, args.edge_offset)):
        raise ValueError('날 오프셋은 유한해야 합니다.')
    for value in (args.scoop_speed, args.transfer_speed, args.acceleration, args.transfer_z):
        if not math.isfinite(value) or value <= 0:
            raise ValueError('속도·가속도·이송 높이는 양의 유한한 값이어야 합니다.')
    scoop_pass = getattr(args, 'scoop_pass', 1)
    if not isinstance(scoop_pass, int) or not 1 <= scoop_pass <= SCOOP_MAX_PASS:
        raise ValueError(f'--scoop-pass는 1~{SCOOP_MAX_PASS} 정수여야 합니다.')
    step = getattr(args, 'scoop_pass_step', SCOOP_PASS_STEP_MM)
    if not math.isfinite(step) or not 0 < step <= SCOOP_PASS_STEP_MM:
        raise ValueError(f'--scoop-pass-step은 0 초과 {SCOOP_PASS_STEP_MM:g} mm 이하여야 합니다 '
                         f'(용기 X {SCOOP_CONTAINER_SIZE[0]:g} mm 안에서 {SCOOP_MAX_PASS}회).')
    force = getattr(args, 'wall_contact_force', WALL_CONTACT_FORCE_N)
    stall_mm = getattr(args, 'wall_stall_mm', WALL_STALL_MM)
    stall_sec = getattr(args, 'wall_stall_sec', WALL_STALL_SEC)
    if not all(map(math.isfinite, (force, stall_mm, stall_sec))) or force <= 0 or stall_mm < 0 or stall_sec < 0:
        raise ValueError('벽 탐색 기준 힘은 양수, 막힘 거리·시간은 0 이상이어야 합니다.')
    margin = getattr(args, 'wall_search_margin', WALL_SEARCH_MARGIN_MM)
    speed = getattr(args, 'wall_search_speed', WALL_SEARCH_SPEED_MM_S)
    push = getattr(args, 'wall_push_force', WALL_PUSH_FORCE_N)
    if not all(map(math.isfinite, (margin, speed, push))) or margin <= 0 or speed <= 0 or not 0 < push <= 20:
        raise ValueError('벽 탐색 거리·속도는 양수, 누르는 힘은 0 초과 20 N 이하여야 합니다.')
    if int(getattr(args, 'wall_contact_confirm', WALL_CONTACT_CONFIRM)) < 1:
        raise ValueError('벽 접촉 연속 감지 횟수는 1 이상이어야 합니다.')
    stiff = list(getattr(args, 'scrape_stiffness', SCRAPE_STIFFNESS))
    if len(stiff) != 6 or not all(map(math.isfinite, stiff)) or min(stiff) <= 0:
        raise ValueError('긁기 강성은 양수 6개여야 합니다.')
    height = scrape_height(args)
    top = SCOOP_CONTAINER_SIZE[2]-WALL_SCRAPE_TOP_MARGIN_MM
    if not math.isfinite(height) or not 0 <= height <= top:
        raise ValueError(f'--wall-scrape-height는 0~{top:g} mm여야 합니다(용기 깊이 {SCOOP_CONTAINER_SIZE[2]:g} mm '
                         f'- 여유 {WALL_SCRAPE_TOP_MARGIN_MM:g} mm).')
    if height > args.lift:
        raise ValueError('긁어 올리는 높이는 전체 상승 거리(--lift)보다 클 수 없습니다.')


# 몇 번째 뜨기인지에 따라 썰기 끝점을 +X로 옮기는 거리 mm.
def pass_shift(args):
    return (getattr(args, 'scoop_pass', 1)-1)*getattr(args, 'scoop_pass_step', SCOOP_PASS_STEP_MM)


def scrape_height(args):
    return getattr(args, 'wall_scrape_height', WALL_SCRAPE_HEIGHT_MM)


# 기울이는 회전량: 측정 자세의 헤라는 수직(수평면 기준 90°)이므로 (90° - 목표각)만큼 BASE Y축으로 돈다.
# 양(+)의 BASE Y 회전은 손잡이(위쪽)를 +X로 눕힌다. 그러면 날 끝은 -X를 향하고 -X 면이 위를 본다.
def tilt_rotation(args):
    return 90.0-args.scoop_angle


# 측정 자세에서 용기 -X 벽 X와 바닥 Z를 구한다.
def container_walls(args):
    """측정 시 헤라 -X 면이 -X 벽에, 날 끝(폭 가운데)이 바닥에 닿아 있었다고 가정한다."""
    reference = tuple(args.taught_pose)
    initial = zyz_axes(reference[3:])
    touch = tuple(reference[i]+sum(initial[j][i]*args.edge_offset[j] for j in range(3)) for i in range(3))
    return touch, touch[0]-BLADE_THICKNESS/2, touch[2]


# 썰기 끝점의 날 끝(폭 가운데) BASE XYZ를 구한다.
def scoop_end_edge(args):
    """측정 자세의 날 끝 접촉점(여유 없음, 사용자 결정 2026-11-19)을 회차만큼 +X로 옮긴 점을 쓴다.

    주의: 기울이면 두께 때문에 날 아래 모서리가 (두께/2)·sin(90°-각도)만큼 바닥 아래로 계산된다
    (45°에서 약 2.1 mm). 위치제어라 실제로 바닥을 밀면 충돌 감지(7060)가 날 수 있다.
    """
    touch, _, _ = container_walls(args)
    return (touch[0]+pass_shift(args), touch[1], touch[2])


# 기울인 자세에서 썰기 끝점의 TCP XYZABC를 구한다. 날 끝을 고정점으로 놓고 TCP를 역산한다.
def taught_target(args):
    initial = zyz_axes(tuple(args.taught_pose)[3:])
    axes = tuple(rotate_base_y(axis, tilt_rotation(args)) for axis in initial)
    edge = scoop_end_edge(args)
    tcp = tuple(edge[i]-sum(axes[j][i]*args.edge_offset[j] for j in range(3)) for i in range(3))
    return (*tcp, *axes_to_zyz(axes))


# 날 폭이 용기 Y 폭에 들어가는지, 사선 진입 중 날 끝이 +X 벽 위로 지나가는지 검사한다.
# Y 방향 위치는 검사하지 않는다(측정점이 용기 Y 가운데라고 가정).
def check_scoop_clearance(args, container=SCOOP_CONTAINER_SIZE):
    size_x, size_y, depth = container
    if BLADE_WIDTH > size_y:
        raise ValueError(f'헤라 폭 {BLADE_WIDTH:g} mm가 용기 Y 폭 {size_y:g} mm보다 넓습니다.')
    _, wall_x, floor_z = container_walls(args)
    edge = scoop_end_edge(args)
    # 날 끝은 끝점에서 +X·+Z로 각도만큼 거슬러 올라간 직선 위를 지난다.
    far_wall_x = wall_x+size_x
    height_at_far_wall = edge[2]-floor_z+(far_wall_x-edge[0])*math.tan(math.radians(args.scoop_angle))
    if height_at_far_wall < depth:
        raise ValueError(f'사선 진입 중 날 끝이 +X 벽 위치에서 바닥 위 {height_at_far_wall:.1f} mm로 지나 '
                         f'벽 높이 {depth:g} mm에 걸립니다.')
    return height_at_far_wall


# 기울인 자세를 유지해 -X·-Z 사선으로 썰고 들어가 썰기 끝점까지 가는 경유점을 생성한다.
# 끝점 이후(벽 밀기·긁어 올리기·상승)는 wall_scrape_poses가 맡는다.
def taught_moves(current, args):
    validate_taught(args)
    check_scoop_clearance(args)
    target = taught_target(args)
    _, _, above = wall_scrape_poses(args)
    # 사선 시작점: 끝점에서 날 방향의 반대(+X·+Z)로 entry_height만큼 올라간 점
    dx = args.entry_height/math.tan(math.radians(args.scoop_angle))
    entry = (target[0]+dx, target[1], target[2]+args.entry_height, *target[3:])
    if not all(map(math.isfinite, entry)):
        raise ValueError('사선 진입점이 유한하지 않습니다.')
    height = max(current[2], entry[2], above[2], args.transfer_z)
    yield '현재 자세로 이송 높이 확보', (*current[:2], height, *current[3:]), args.transfer_speed
    yield f'그 자리에서 헤라를 {args.scoop_angle:g}도로 기울이기', (*current[:2], height, *target[3:]), \
        args.transfer_speed
    yield '기울인 자세로 사선 시작점 위 이송', (*entry[:2], height, *target[3:]), args.transfer_speed
    yield '기울인 자세로 사선 시작점 하강', entry, args.transfer_speed
    yield f'각도 유지하며 -X·-Z 방향 {args.scoop_angle:g}도 사선 진입', target, args.scoop_speed


# 썰기 끝점 이후의 계획 TCP: 기록된 벽 위치(끝점에서 -X로 회차 이동량만큼), 긁어 올린 점, 최종 상승점.
# 실물은 벽 위치를 힘으로 찾으므로 이 값은 관절 사전 검사와 가상 실행에만 그대로 쓴다.
def wall_scrape_poses(args):
    target = taught_target(args)
    wall = (target[0]-pass_shift(args), *target[1:])
    scraped = (*wall[:2], wall[2]+scrape_height(args), *wall[3:])
    above = (*wall[:2], target[2]+args.lift, *wall[3:])
    return wall, scraped, above


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
def run_robot(args):
    # 벽 탐색 기준 (예전 인자 객체에 없으면 기본값)
    for name, default in (('wall_contact_force', WALL_CONTACT_FORCE_N), ('wall_stall_mm', WALL_STALL_MM),
                          ('wall_stall_sec', WALL_STALL_SEC), ('wall_search_margin', WALL_SEARCH_MARGIN_MM),
                          ('wall_search_speed', WALL_SEARCH_SPEED_MM_S), ('wall_push_force', WALL_PUSH_FORCE_N),
                          ('wall_contact_confirm', WALL_CONTACT_CONFIRM), ('scrape_stiffness', SCRAPE_STIFFNESS)):
        if not hasattr(args, name):
            setattr(args, name, default)
    import time
    from types import SimpleNamespace

    import rclpy
    from dsr_msgs2.srv import (
        CheckMotion, GetCurrentPosx, GetCurrentTcp, GetRobotSystem, GetToolForce, MoveLine, MoveStop,
        ReleaseComplianceCtrl, ReleaseForce, SetDesiredForce, TaskComplianceCtrl,
    )
    from utils.service_waiter import ServiceWaiter
    from utils.shutdown import safe_shutdown

    rclpy.init(args=[])
    node = None
    force_active = compliance_active = False
    try:
        node = rclpy.create_node('scoop_putty', namespace=ROBOT_ID)
        prefix = f'/{ROBOT_ID}/dsr_controller2'
        system = node.create_client(GetRobotSystem, f'{prefix}/system/get_robot_system')
        tcp = node.create_client(GetCurrentTcp, f'{prefix}/tcp/get_current_tcp')
        pose = node.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx')
        motion = node.create_client(MoveLine, f'{prefix}/motion/move_line')
        from .pickup_spatula import create_joint_clients, preflight_poses
        joint_clients = create_joint_clients(node, prefix)  # 출발 전 ikin 관절 검사용
        ServiceWaiter(node).wait(system, tcp, pose, motion, joint_clients.joints,
                                 joint_clients.solution, joint_clients.ikin, joint_clients.joint,
                                 joint_clients.fkin)

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
        real = kind == 0
        if call(tcp, GetCurrentTcp.Request(), 'TCP 확인').info != EXPECTED_TCP:
            raise RuntimeError(f'{EXPECTED_TCP} TCP를 적용하세요.')
        # 실물 벽 긁기에만 쓰는 외력·정지·컴플라이언스·힘제어 서비스. 가상은 위치 이동만 한다.
        if real:
            force = node.create_client(GetToolForce, f'{prefix}/aux_control/get_tool_force')
            stop = node.create_client(MoveStop, f'{prefix}/motion/move_stop')
            motion_check = node.create_client(CheckMotion, f'{prefix}/motion/check_motion')
            compliance = node.create_client(TaskComplianceCtrl, f'{prefix}/force/task_compliance_ctrl')
            desired = node.create_client(SetDesiredForce, f'{prefix}/force/set_desired_force')
            release = node.create_client(ReleaseForce, f'{prefix}/force/release_force')
            release_comp = node.create_client(ReleaseComplianceCtrl,
                                              f'{prefix}/force/release_compliance_ctrl')
            ServiceWaiter(node).wait(force, stop, motion_check, compliance, desired, release, release_comp)

        # 두산 API: get_current_posx(ref=DR_BASE).
        def read_pose():
            request = GetCurrentPosx.Request()
            request.ref = 0
            rows = call(pose, request, 'BASE TCP 조회').task_pos_info
            values = tuple(rows[0].data[:6]) if rows else ()
            if len(values) != 6 or not all(map(math.isfinite, values)):
                raise RuntimeError('유효한 BASE TCP 자세를 읽지 못했습니다.')
            return values

        def line_request(target, speed, sync_type=0):
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [args.acceleration, 20.0]
            request.ref = request.mode = 0
            request.sync_type = sync_type
            return request

        # 두산 API: movel(동기). 이동 후 실제 TCP가 목표 2 mm / 2° 안인지 다시 확인한다.
        # x_free: -X 힘제어 중에는 X가 벽을 따라 움직이므로 X는 도착 판정에서 빼고,
        #         벽 마찰로 Z가 컴플라이언스(1 mm당 1 N)만큼 덜 올라갈 수 있어 3 mm까지 허용한다.
        def move(step, target, speed, x_free=False):
            print(f'[{step}] BASE [{format_values(target)}] | {speed:g} mm/s', flush=True)
            call(motion, line_request(target, speed), step, timeout=120.0)
            actual = read_pose()
            axes, tolerance = ((1, 2), 3.0) if x_free else ((0, 1, 2), 2.0)
            if (max(abs(actual[i]-target[i]) for i in axes) > tolerance or
                    orientation_error(actual[3:], target[3:]) > 2.0):
                raise RuntimeError(f'{step} 목표 XYZABC 미도착: {actual}')

        # 두산 API: amovel(sync_type=1). 이동 완료를 기다리지 않아 밀면서 외력을 계속 읽을 수 있다.
        def move_async(target, speed):
            call(motion, line_request(target, speed, sync_type=1), '벽 접촉 탐색 이동', timeout=3.0)

        # 두산 API: get_tool_force(DR_BASE). [Fx,Fy,Fz,Mx,My,Mz]
        def read_force():
            request = GetToolForce.Request()
            request.ref = 0
            return call(force, request, 'BASE 외력 조회', timeout=0.5).tool_force

        # 두산 API: stop(DR_HOLD) + check_motion. 컴플라이언스 중에는 GetRobotState가 이동 중에도
        # STANDBY를 줄 수 있어(인수인계 기록) 정지 확인은 check_motion(0=IDLE)으로 한다.
        def stop_and_wait():
            request = MoveStop.Request()
            request.stop_mode = 3  # DR_HOLD
            call(stop, request, '벽 접촉 정지', timeout=2.0)
            deadline = time.monotonic()+5.0
            while call(motion_check, CheckMotion.Request(), '이동 상태 확인', timeout=0.5).status != 0:
                if time.monotonic() >= deadline:
                    raise TimeoutError('벽 접촉 정지 완료를 확인하지 못했습니다.')
                time.sleep(0.05)

        # 두산 API: task_compliance_ctrl(stx, time=0.3).
        def start_compliance():
            nonlocal compliance_active
            request = TaskComplianceCtrl.Request()
            request.stx = [float(v) for v in args.scrape_stiffness]
            request.ref, request.time = 0, 0.3
            compliance_active = True  # 응답 실패여도 적용됐을 수 있으므로 정리 대상으로 표시한다
            call(compliance, request, '벽 긁기 컴플라이언스 시작')
            time.sleep(0.3)  # 컴플라이언스 전환 시간(time=0.3) 동안 기다린다

        # 두산 API: set_desired_force(fd=[-F,0,0,0,0,0], dir=[1,0,0,0,0,0], mod=DR_FC_MOD_REL).
        # 상대 모드라 힘제어를 시작한 순간의 Fx를 0으로 보고 그 위에 -X로 F만큼 더 민다.
        def start_push_force():
            nonlocal force_active
            force_active = True
            call(desired, press_force_request(args.wall_push_force, mode=1, sign=-1), f'-X {args.wall_push_force:g} N 유지')

        # 두산 API: release_force(time=0.3) → release_compliance_ctrl. 힘 → 컴플라이언스 순서로 푼다.
        def release_push_force():
            nonlocal force_active
            request = ReleaseForce.Request()
            request.time = 0.3
            call(release, request, '목표 힘 해제')
            force_active = False

        def release_compliance():
            nonlocal compliance_active
            # 벽·바닥에 눌린 상태로 풀면 덜컹거리므로, 현재 실제 위치로 movel해 눌림을 없앤 뒤 해제한다.
            call(motion, line_request(read_pose(), 2.0), '눌림 풀기', timeout=10.0)
            time.sleep(0.3)
            call(release_comp, ReleaseComplianceCtrl.Request(), '컴플라이언스 해제')
            compliance_active = False

        edge = scoop_end_edge(args)
        print(f'[뜨기 설정] 각도 {args.scoop_angle:g}° (BASE Y +{tilt_rotation(args):g}° 회전) | '
              f'날 끝 오프셋 {tuple(args.edge_offset)} mm | {args.scoop_pass}회차 (끝점 +X {pass_shift(args):g} mm)',
              flush=True)
        print('[뜨기 설정] 썰기 끝 날 끝 ['+format_values(edge)+'] / TCP ['+format_values(taught_target(args))+']',
              flush=True)
        print(f'[벽 긁기 설정] -X 탐색 최대 {pass_shift(args)+args.wall_search_margin:g} mm, '
              f'{args.wall_search_speed:g} mm/s, Fx 변화 {args.wall_contact_force:g} N 또는 '
              f'{args.wall_stall_sec:g} s 동안 {args.wall_stall_mm:g} mm 미만 진행(막힘)에서 정지 | '
              f'-X {args.wall_push_force:g} N 유지하며 {scrape_height(args):g} mm 상승 | 강성 {list(args.scrape_stiffness)}',
              flush=True)
        if real:
            confirm_grasp()
        current = read_pose()
        list(taught_moves(current, args))       # 인자·용기 여유 검사 (이동 전)
        if current[0] > WALL_SIDE_X_MM:
            # 두 번째 틈부터는 앞 틈 도포를 끝낸 벽 앞(+X)에서 시작한다. 그 자리에서 뜨기 자세로 기울이면
            # 도달 불가(run101: X 724 에서 NOT REACHABLE). 반납과 같은 경로(상승 → 홈 → 경유 관절 →
            # 보관대 위 직선)로 집기 직후와 같은 자리로 먼저 옮긴다.
            from .pickup_spatula import (HOLDER_DETOUR_J, HOME_J, JOINT_ACCELERATION_DEG_S2, JOINT_SPEED_DEG_S,
                                         PICK_POSITION, PICKUP_ABC, check_pillar_clearance, joint_pose,
                                         move_to_joints)
            detour_pose = joint_pose(call, joint_clients, HOLDER_DETOUR_J, '경유 관절 자세')
            above_holder = (PICK_POSITION[0], PICK_POSITION[1], HOLDER_TRANSFER_Z_MM, *PICKUP_ABC)
            check_pillar_clearance((detour_pose, above_holder))
            print(f'[뜨기 준비] 벽 쪽(X {current[0]:.0f} mm)에서 시작 → 홈·경유 관절로 보관대 위까지 이동', flush=True)
            move('벽 앞에서 수직 상승', (*current[:2], max(args.transfer_z, current[2]), *current[3:]),
                 args.transfer_speed)
            for step, joints in (('홈 관절 자세로 관절 이동', HOME_J), ('경유 관절 자세로 관절 이동', HOLDER_DETOUR_J)):
                move_to_joints(call, joint_clients, step, joints, JOINT_SPEED_DEG_S, JOINT_ACCELERATION_DEG_S2,
                               info=lambda text: print(text, flush=True), direction=None)
            move('보관대 위로 직선 이동', above_holder, args.transfer_speed)
            current = read_pose()
        # 경로 전체를 먼저 계산·검사한 뒤 첫 이동을 보낸다. 검사 실패 시 로봇은 움직이지 않는다.
        moves = list(taught_moves(current, args))
        wall, scraped, above = wall_scrape_poses(args)
        # 기울이기는 처음 하는 자세라 특이점·손목 뒤집힘을 경유점마다 미리 확인한다(두산 API: ikin).
        checks = [(step, target) for step, target, _ in moves]
        checks += [('벽 위치(계획)', wall), ('긁어 올린 점(계획)', scraped), ('최종 상승점(계획)', above)]
        preflight_poses(call, joint_clients, checks, info=lambda text: print(text, flush=True))
        for step, target, speed in moves:
            move(step, target, speed)

        # ===== 끝점 이후: 벽으로 밀어 모으기 → 긁어 올리기 → 상승 =====
        if real:
            start_compliance()
            io = SimpleNamespace(read_pose=read_pose, read_force=read_force,
                                 move_async=move_async, stop_and_wait=stop_and_wait)
            contact = seek_axis(io, 0, -1, args.wall_contact_force, pass_shift(args)+args.wall_search_margin,
                                args.wall_search_speed, confirm=args.wall_contact_confirm,
                                stall_mm=args.wall_stall_mm, stall_sec=args.wall_stall_sec)
            print(f'[벽 긁기] 접촉 TCP X {contact[0]:.2f} mm (계획 벽 TCP X {wall[0]:.2f} mm, '
                  f'차이 {contact[0]-wall[0]:+.1f} mm)', flush=True)
            if scrape_height(args) > 0:
                start_push_force()
                move(f'-X {args.wall_push_force:g} N 유지하며 긁어 올리기',
                     (*contact[:2], contact[2]+scrape_height(args), *contact[3:]), args.scoop_speed, x_free=True)
                release_push_force()
            release_compliance()
            here = read_pose()
            move('기울인 자세로 수직 상승', (*here[:2], above[2], *above[3:]), args.scoop_speed)
        else:
            move('기록된 벽 위치까지 -X 이동(가상)', wall, args.wall_search_speed)
            if scrape_height(args) > 0:
                move('긁어 올리기 높이까지 상승(가상)', scraped, args.scoop_speed)
            move('기울인 자세로 수직 상승', above, args.scoop_speed)
        print('측정 자세 퍼티 뜨기, 벽 긁기, 수직 상승 완료', flush=True)
    finally:
        # 실패로 끝났는데 힘·컴플라이언스가 켜져 있으면 정지 후 해제를 시도한다. 자동 후퇴는 하지 않는다.
        if (force_active or compliance_active) and node is not None:
            for name, action in (('정지', stop_and_wait), ('목표 힘 해제', release_push_force),
                                 ('컴플라이언스 해제', lambda: call(release_comp, ReleaseComplianceCtrl.Request(),
                                                               '컴플라이언스 해제'))):
                if name == '목표 힘 해제' and not force_active:
                    continue
                try:
                    action()
                except Exception as exc:
                    node.get_logger().error(f'정리 중 {name} 실패: {exc}')
        safe_shutdown(node)


# ===== 실행 진입점 =====
if __name__ == '__main__':
    raise SystemExit(main())
