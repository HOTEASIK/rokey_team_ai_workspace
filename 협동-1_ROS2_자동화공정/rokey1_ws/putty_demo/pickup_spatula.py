"""지정 보관 위치에 놓인 헤라를 RG2로 집어 접근 높이까지 들어 올린다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/pickup_spatula. 홈 위치에서 출발해 보관 위치의 헤라를 집는다.
# 동작 순서와 두산 API:
#   TCP 적용/확인(set_tcp) → 홈 관절 확인(get_current_posj)
#   → 홈 위로 수직 상승(movel, 동기)
#   → 기둥 피하는 경유 관절 자세(사용자 직접 측정)로 관절 이동(movej, 동기): 위에서 볼 때 반시계(J1 증가)
#   → 그 자리에서 이송 높이로 수직 상승(movel, 동기) → 헤라 바로 위로 직선 이동(movel, 동기)
#   → 열기(set_digital_output) → 수직 하강(movel, 동기) → 닫기 → 수직 상승(movel, 동기)
# 통신: dsr_msgs2 서비스를 직접 호출한다. --virtual-gripper면 여닫기만 onrobot SetCommand로 바뀐다.
# 집게 닫기 요청 성공이 실제 헤라 파지 성공을 뜻하지는 않는다.

import argparse
import math
import sys
import time


# ===== 설정 / 하이퍼파라미터 =====
from utils.robot import HOME_J, ROBOT_ID, TCP_NAME  # config/robot.yaml robot 섹션
HOME_TOLERANCE_DEG = 0.5
EXPECTED_TCP = TCP_NAME
# 헤라를 집는 TCP의 BASE XYZABC (mm, deg). 2026-11-19 보관 위치를 로봇 뒤쪽(-X, +Y)으로 옮겼다.
# 홈 자세에서 회전 없이 축 방향 이동만 해서 찍은 값이라 ABC는 홈과 같은 '툴 수직 아래' 자세다.
# 베이스 기준 방위각 약 +150.7°라서 홈(J1=0°)에서 J1을 약 150° 돌려야 닿는다.
# 반납(return_spatula)도 같은 위치에 내려놓는다.
PICK_POSE = (-508.39, 284.79, 121.85, 136.31, -179.38, 136.56)
# robot.yaml putty.pick_pose 가 있으면 그 값 (보관대를 옮겼을 때). 집기·반납·뜨기(경유)가 같은 값을 쓴다.
# 기둥 위치(HOLDER_PILLAR_XY)는 보관대 기준이라 같이 움직인다.
from utils.config import CFG as _CFG
if (_CFG.get('putty') or {}).get('pick_pose') is not None:
    _pose = tuple(float(v) for v in _CFG['putty']['pick_pose'])
    if len(_pose) != 6 or not all(map(math.isfinite, _pose)):
        raise ValueError(f'robot.yaml putty.pick_pose 는 유한한 x,y,z,a,b,c 6개여야 합니다: {_pose}')
    PICK_POSE = _pose
PICK_POSITION = PICK_POSE[:3]
PICKUP_ABC = PICK_POSE[3:]  # 실제 집기 자세. 도포 경로의 헤라 방향 기준값은 putty_geometry에 따로 있다.
# PICK_POSE는 헤라가 보관대 바닥까지 완전히 내려앉은 상태에서 잡는 위치다. 그래서 매번 같은 깊이를 잡는다.
# 보관대 위 이송 높이(TCP Z). 이 높이에서 관절 이동(movej)한 뒤 수직으로 내려간다. 집기·반납·뜨기가 같이 쓴다.
# 2026-11-19 실물 ikin 측정(sol_space 2): 보관대 위에서 높을수록 팔이 펴져 팔꿈치 J3이 작아진다.
#   Z 245.14 → J3 40.4° (관절 검사 41°에 걸림), 240 → 41.6°, 230 → 44.0°, 225 → 45.1°, 222 → 45.8°
# 230은 인수인계 기록의 '44.1°는 특이점 알람 없이 통과'와 비슷하고, 보관대(깊이 80) 위로 28 mm 여유가 있다.
from utils.config import CFG
HOLDER_TRANSFER_Z_MM = float(CFG['putty']['holder_transfer_z'])  # robot.yaml putty.holder_transfer_z
# 지그(보관대) 기준 +X 150, +Y 170 mm에 기둥이 있다(2026-11-19 현장 확인). 홈에서 보관대 위로 바로 movej하면
# TCP가 곡선으로 움직여 기둥에 부딪쳤다. 그래서 movej는 사용자가 직접 찍은 경유 관절 자세까지만 하고, 이후는 직선(movel)이다.
HOLDER_PILLAR_XY = (PICK_POSE[0]+150.0, PICK_POSE[1]+170.0)
# 경유 관절 자세(deg). 2026-11-19 사용자가 직접 찍은 경유지. 홈에서 여기까지 movej로 기둥에 닿지 않는다.
# 이때 TCP BASE (-185.48, 292.47, 204.82, 101.65, -179.37, 102.36): 베이스에서 약 347 mm, 방위 121.7°로
# 기둥(579 mm, 128°)보다 안쪽이고, 툴 수직 아래(집기와 같은 방향) 자세다.
HOLDER_DETOUR_J = (121.69, -2.45, 94.67, 0.17, 88.37, 121.89)
HOLDER_DETOUR_POSE = (-185.48, 292.47, 204.82, 101.65, -179.37, 102.36)  # 위 관절의 BASE TCP (측정값)
# 직선 구간에서 TCP와 기둥 중심 사이 최소 거리. 헤라 폭 절반(50)과 기둥 두께·여유를 합한 가정값이다.
PILLAR_CLEARANCE_MM = 100.0
# 보관대 깊이. 관절 이동 전에 헤라가 보관대에서 완전히 빠져 있어야 한다.
HOLDER_DEPTH_MM = 80.0
HOLDER_CLEARANCE_MM = 20.0  # 보관대 위로 더 띄울 여유
MIN_LIFT_MM = HOLDER_DEPTH_MM+HOLDER_CLEARANCE_MM  # 바닥에 앉은 높이에서 최소 이만큼 올라가야 한다
TRANSFER_SPEED_MM_S = 15.0
PICK_SPEED_MM_S = 5.0
# 홈 ↔ 보관 위치 관절 이동(movej) 속도. J1이 약 150° 돌기 때문에 직선 이동 대신 관절 이동을 쓴다.
JOINT_SPEED_DEG_S = 20.0
JOINT_ACCELERATION_DEG_S2 = 20.0
JOINT_TOLERANCE_DEG = 1.0   # 관절 이동 후 목표 관절각 도착 허용 오차
# 관절 이동 한 번의 J1 회전량 상한. 보관 위치 ↔ 벽 앞이 약 150°이므로 이보다 크면 계산이 잘못된 것으로 본다.
MAX_TURN_DEG = 180.0
# 반납 및 전체 사이클 드라이런에서 사용하는 기존 8번 기준값.
HOLE_8 = (724.14, 73.22, 35.14)
PICK_Z_OFFSET_MM = 160.0
RECORDED_HOME_POSE = (367.41, 3.06, 219.28, 140.70, 179.97, 141.21)
OPEN = ((1, 0), (2, 0), (2, 1))
CLOSE = ((1, 0), (2, 0), (1, 1), (2, 0))



# ===== 메인 =====
# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True, help='실물 로봇에 요청 전송')
    parser.add_argument('--virtual-gripper', action='store_true',
                        help='가상 RG2 서비스로 열기·닫기 (기본은 실물 DO)')
    parser.add_argument('--clearance', type=float, default=50.0, help='집기점 위 접근 거리(mm)')
    parser.add_argument('--settle', type=float, default=1.0, help='집게 출력 후 대기 시간(초)')
    parser.add_argument('--transfer-speed', type=float, default=TRANSFER_SPEED_MM_S,
                        help='접근 이송 속도(mm/s)')
    parser.add_argument('--pick-speed', type=float, default=PICK_SPEED_MM_S,
                        help='집기 하강 및 들어 올리기 속도(mm/s)')
    parser.add_argument('--joint-speed', type=float, default=JOINT_SPEED_DEG_S,
                        help='보관 위치 위로 관절 이동하는 속도(deg/s)')
    parser.add_argument('--joint-acceleration', type=float, default=JOINT_ACCELERATION_DEG_S2,
                        help='보관 위치 위로 관절 이동하는 가속도(deg/s²)')
    args = parser.parse_args()
    for name in ('transfer_speed', 'pick_speed', 'joint_speed', 'joint_acceleration'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")}는 0보다 큰 유한한 값이어야 합니다.')
    if not math.isfinite(args.clearance) or args.clearance <= 0:
        parser.error('--clearance는 0보다 큰 유한한 값이어야 합니다.')
    if not math.isfinite(args.settle) or args.settle < 0:
        parser.error('--settle은 0 이상인 유한한 값이어야 합니다.')

    try:
        run_robot(args)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'집기 중단: {exc}', file=sys.stderr)
        return 1


# ===== 함수 정의 =====
# 경유점과 집게 동작 순서만 정의한다. 실제 요청은 전달받은 move/move_joints/출력 함수가 맡는다.
#   move(step, pose, speed)   : 직선 이동(movel)
#   move_joints(step, joints) : 관절값으로 바로 관절 이동(movej)하고 도착한 BASE TCP 자세를 돌려준다
def pickup(home_pose, clearance, settle, move, move_joints, set_output, wait,
           transfer_speed=TRANSFER_SPEED_MM_S, pick_speed=PICK_SPEED_MM_S, gripper_command=None):
    """집기 경로 및 그리퍼 출력 순서를 실행한다."""
    x, y, pick_z = PICK_POSITION
    clear_z = round(max(pick_z + clearance, home_pose[2], HOLDER_TRANSFER_Z_MM), 2)
    check_holder_clearance(clear_z)
    abc = PICKUP_ABC
    approach = (x, y, clear_z, *abc)
    grasp = (x, y, pick_z, *abc)

    # 홈에서 먼저 수직으로 올라가 이송 높이를 확보한다. 관절 이동 중 주변과 부딪히지 않게 하려는 것이다.
    if clear_z > home_pose[2]:
        move('홈 위로 상승', (*home_pose[:2], clear_z, *home_pose[3:]), transfer_speed)
    # 홈에서 보관대 위로 바로 movej하면 기둥에 부딪친다. 실제로 안전이 확인된 경유 관절 자세로만 관절 이동한다.
    reached = move_joints('기둥 피하는 경유 자세로 관절 이동', HOLDER_DETOUR_J)
    # 경유지(Z 약 205)는 보관대 위 이송 높이(230)보다 낮다. 보관대 쪽으로 가기 전에 그 자리에서 먼저 올린다.
    lift = (*reached[:2], clear_z, *abc)
    # 남은 직선 구간이 기둥과 충분히 떨어져 있는지 다음 이동 전에 계산으로 확인한다.
    check_pillar_clearance((lift, approach))
    move('경유 자세에서 이송 높이로 상승', lift, transfer_speed)
    move('헤라 바로 위로 직선 이동', approach, transfer_speed)

    if gripper_command is not None:
        gripper_command('o')
    else:
        for channel, value in OPEN:
            set_output('RG2 열기', channel, value)
    wait(settle)
    move('집기 높이까지 하강', grasp, pick_speed)

    if gripper_command is not None:
        gripper_command('c')
    else:
        for channel, value in CLOSE:
            set_output('RG2 닫기', channel, value)
    wait(settle)
    move('헤라 들어 올리기', approach, pick_speed)


# 이송 높이에서 헤라 끝이 보관대 밖으로 완전히 나와 있는지 확인한다. 아니면 관절 이동 중 보관대에 걸린다.
def check_holder_clearance(transfer_z):
    lift = transfer_z-PICK_POSITION[2]
    if lift < MIN_LIFT_MM:
        raise ValueError(f'이송 높이 {transfer_z:g} mm는 바닥 대비 {lift:.1f} mm로, 보관대 깊이 '
                         f'{HOLDER_DEPTH_MM:g} mm + 여유 {HOLDER_CLEARANCE_MM:g} mm보다 낮습니다.')


# 직선 구간(점 → 점)마다 위에서 본 TCP 경로와 기둥 중심의 최소 거리를 확인한다. 부족하면 움직이지 않는다.
def check_pillar_clearance(points, pillar=HOLDER_PILLAR_XY, clearance=PILLAR_CLEARANCE_MM):
    for start, end in zip(points, points[1:]):
        distance = segment_distance_xy(pillar, start[:2], end[:2])
        if distance < clearance:
            raise ValueError(f'직선 구간 {format_xy(start)} → {format_xy(end)}이 기둥 {format_xy(pillar)}에 '
                             f'{distance:.0f} mm까지 접근합니다(최소 {clearance:g} mm). 로봇은 움직이지 않았습니다.')


# 점 p에서 선분 a-b까지의 XY 최소 거리.
def segment_distance_xy(p, a, b):
    ab = (b[0]-a[0], b[1]-a[1])
    length2 = ab[0]**2+ab[1]**2
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((p[0]-a[0])*ab[0]+(p[1]-a[1])*ab[1])/length2))
    return math.dist(p, (a[0]+t*ab[0], a[1]+t*ab[1]))


def format_xy(point):
    return f'({point[0]:.0f}, {point[1]:.0f})'


# ikin이 준 목표 관절의 J1을 원하는 회전 방향으로 도는 값으로 맞춘다.
# J1에 360°를 더하거나 빼도 같은 자세이므로, 위에서 볼 때
#   'ccw'(반시계) → 현재 J1보다 큰 값, 'cw'(시계) → 현재 J1보다 작은 값을 고른다.
# 회전량이 max_turn을 넘으면 거의 한 바퀴를 도는 것이라 계산 오류로 보고 멈춘다.
# 'nearest' → 더 적게 도는 쪽(같은 J1 ±360 중 현재와 가장 가까운 값). 홈(J1≈0)에서 벽 앞처럼 조금만 돌 때 쓴다.
def turn_joint_target(current_joints, target_joints, direction, max_turn=MAX_TURN_DEG):
    target = list(target_joints)
    if direction == 'nearest':
        direction = 'cw' if (current_joints[0]-target[0]) % 360.0 <= 180.0 else 'ccw'
    sign = {'ccw': 1.0, 'cw': -1.0}[direction]
    name = '반시계' if direction == 'ccw' else '시계'
    turn = (sign*(target[0]-current_joints[0])) % 360.0
    if turn > max_turn:
        raise RuntimeError(f'{name} J1 회전량 {turn:.1f}°가 허용 범위(0~{max_turn:g}°)를 벗어났습니다. '
                           '목표 좌표나 현재 자세를 확인하세요. 로봇은 움직이지 않았습니다.')
    target[0] = current_joints[0]+sign*turn
    if abs(target[0]) > 360.0:
        raise RuntimeError(f'목표 J1 {target[0]:.1f}°가 관절 범위(±360°)를 벗어납니다.')
    return tuple(target)


# 보관 위치로 갈 때(집기·반납)는 항상 반시계로 돈다.
def ccw_joint_target(current_joints, target_joints, max_turn=MAX_TURN_DEG):
    return turn_joint_target(current_joints, target_joints, 'ccw', max_turn)


# ikin 결과가 특이점 근처이거나 손목이 뒤집힌 해인지 검사한다. 해당하면 movej를 보내지 않는다.
# - J3(팔꿈치)·J5(손목) 여유: 도포의 출발 전 관절 검사와 같은 기준(J3≥41°, |J5|≥10°)
# - 손목 뒤집힘: 인수인계본 실측에서 벽 앞 자세 기준으로 집기 자세를 풀면 J5≈220°(=-140°) 해가 나왔다.
#   현재 J5와 부호가 바뀌면 손목이 반대로 꺾이는 해로 보고 멈춘다.
# - 어깨 특이점: 손목 중심이 베이스 회전축에서 180 mm 미만이면 멈춘다(뜨기 45° 사선 시작점 실측 132 mm에서 3205·1206).
def check_joint_target(current, target):
    from .putty_motion import (
        PREFLIGHT_J3_MIN_DEG, PREFLIGHT_J5_MIN_DEG, PREFLIGHT_WRIST_RADIUS_MIN_MM, wrist_radius,
    )

    def wrap(angle):  # -180 ~ 180 범위로 바꾼다. 220°와 -140°는 같은 손목 자세다.
        return (angle+180.0) % 360.0-180.0

    j3, j5, now_j5 = abs(wrap(target[2])), wrap(target[4]), wrap(current[4])
    text = ' '.join(f'J{i+1}={v:.1f}' for i, v in enumerate(target))
    if j3 < PREFLIGHT_J3_MIN_DEG or abs(j5) < PREFLIGHT_J5_MIN_DEG:
        raise RuntimeError(f'목표 관절 [{text}]이 특이점 근처입니다(J3≥{PREFLIGHT_J3_MIN_DEG:g}°, '
                           f'|J5|≥{PREFLIGHT_J5_MIN_DEG:g}° 필요). 로봇은 움직이지 않았습니다.')
    if j5*now_j5 < 0:
        raise RuntimeError(f'목표 관절 [{text}]은 손목이 반대로 꺾이는 해입니다(현재 J5 {now_j5:.1f}°). '
                           '로봇은 움직이지 않았습니다.')
    radius = wrist_radius(target)
    if radius < PREFLIGHT_WRIST_RADIUS_MIN_MM:
        raise RuntimeError(f'목표 관절 [{text}]은 어깨 특이점 근처입니다(손목 중심이 베이스 축에서 {radius:.0f} mm, '
                           f'{PREFLIGHT_WRIST_RADIUS_MIN_MM:g} mm 이상 필요). 로봇은 움직이지 않았습니다.')


def preflight_poses(call, clients, waypoints, info=print):
    """경유점(설명, TCP XYZABC)마다 ikin으로 관절을 계산해 특이점·손목 뒤집힘이면 출발하지 않는다.

    두산 API 대응: get_current_solution_space → 점마다 ikin(pos, sol_space, DR_BASE).
    직선 이동(movel) 경로의 중간은 검사하지 않고 경유점만 본다.
    """
    from dsr_msgs2.srv import GetCurrentPosj, GetCurrentSolutionSpace, Ikin

    current = checked_joints(call(clients.joints, GetCurrentPosj.Request(), '현재 관절각 조회').pos)
    solution = call(clients.solution, GetCurrentSolutionSpace.Request(), '현재 팔 형태 조회').sol_space
    for step, pose in waypoints:
        request = Ikin.Request()
        request.pos, request.sol_space, request.ref = [float(v) for v in pose], solution, 0
        joints = checked_joints(call(clients.ikin, request, f'{step} 관절 계산').conv_posj)
        try:
            check_joint_target(current, joints)
        except RuntimeError as exc:
            raise RuntimeError(f'[관절 검사] {step}: {exc}') from exc
    info(f'[관절 검사] {len(waypoints)}개 경유점 통과 (sol {solution})')


def move_joint_to_pose(call, clients, step, pose, speed, acceleration, read_pose, info=print,
                       direction='ccw'):
    """TCP 목표를 관절값으로 바꿔 지정한 방향(direction: 'ccw'/'cw')으로 관절 이동하고 도착을 확인한다.

    두산 API 대응: get_current_solution_space → ikin(pos, sol_space, DR_BASE) → movej(동기).
    clients: joints(GetCurrentPosj), solution(GetCurrentSolutionSpace), ikin(Ikin), joint(MoveJoint).
    반납 노드와 도포(벽 앞으로 돌아가는 이동, 'cw')도 같은 함수를 쓴다.
    """
    from dsr_msgs2.srv import GetCurrentSolutionSpace, Ikin

    # 현재 팔 형태(solution space)를 유지해야 팔꿈치·손목이 뒤집히지 않는 해가 나온다.
    solution = call(clients.solution, GetCurrentSolutionSpace.Request(), '현재 팔 형태 조회').sol_space
    request = Ikin.Request()
    request.pos, request.sol_space, request.ref = [float(v) for v in pose], solution, 0
    solved = checked_joints(call(clients.ikin, request, f'{step} 관절 계산').conv_posj)
    move_to_joints(call, clients, step, solved, speed, acceleration, info, direction)
    from .putty_geometry import orientation_error
    actual = read_pose()
    if math.dist(actual[:3], pose[:3]) > 2.0 or orientation_error(actual[3:], pose[3:]) > 2.0:
        raise RuntimeError(f'{step} 목표 TCP 미도착: {actual}')


def move_to_joints(call, clients, step, joints, speed, acceleration, info=print, direction='ccw'):
    """관절 목표를 지정한 방향으로 도는 값으로 맞추고 검사한 뒤 관절 이동하고, 관절 도착을 확인한다.

    두산 API 대응: get_current_posj → movej(절대 관절각, 동기) → get_current_posj.
    직접 찍은 관절 자세는 ikin 없이 이 함수로 바로 간다(찍은 팔 모양 그대로 이동).
    direction=None이면 J1을 바꾸지 않고 준 관절값 그대로 보낸다(홈 복귀처럼 목표 관절이 정해진 경우).
    """
    from dsr_msgs2.srv import GetCurrentPosj, MoveJoint

    current = checked_joints(call(clients.joints, GetCurrentPosj.Request(), '현재 관절각 조회').pos)
    target = tuple(joints) if direction is None else turn_joint_target(current, joints, direction)
    check_joint_target(current, target)
    turn_name = {'ccw': '반시계', 'cw': '시계', 'nearest': '가까운 쪽', None: '그대로'}[direction]
    info(f'[{step}] 관절 [{", ".join(f"{v:.1f}" for v in target)}] | J1 {target[0]-current[0]:+.1f}° {turn_name} | '
         f'{speed:g} deg/s')

    request = MoveJoint.Request()
    request.pos = list(target)
    request.vel, request.acc = float(speed), float(acceleration)
    request.mode = 0       # 절대 관절각
    request.sync_type = 0  # 동기: 이동이 끝난 뒤 응답
    call(clients.joint, request, step, timeout=120.0)

    arrived = checked_joints(call(clients.joints, GetCurrentPosj.Request(), '도착 관절각 조회').pos)
    if max(abs(a-b) for a, b in zip(arrived, target)) > JOINT_TOLERANCE_DEG:
        raise RuntimeError(f'{step} 목표 관절각 미도착: {arrived}')


def checked_joints(values):
    joints = tuple(values)
    if len(joints) != 6 or not all(map(math.isfinite, joints)):
        raise RuntimeError('유효한 6축 관절각을 얻지 못했습니다.')
    return joints


def show_move(step, pose, speed):
    values = ', '.join(f'{value:.2f}' for value in pose)
    print(f'[{step}] BASE [{values}] | {speed:g} mm/s', flush=True)


def show_output(step, channel, value):
    state = 'ON' if value else 'OFF'
    print(f'[{step}] 출력 {channel}번 {state}', flush=True)


# 필요한 TCP 이름을 선택하고 재조회한다. TCP 오프셋 자체를 새로 정의하는 기능은 아니다.
def ensure_tcp(read_tcp, set_tcp, info):
    """팀 변경: 필요한 TCP를 적용하고 재조회해 확인한 뒤 집기를 진행한다."""
    active = read_tcp()
    if active == EXPECTED_TCP:
        return
    info(f'현재 TCP가 {active!r}입니다. {EXPECTED_TCP}을 적용합니다.')
    try:
        set_tcp(EXPECTED_TCP)
    except RuntimeError as exc:
        raise RuntimeError(f'{exc}. 펜던트에 {EXPECTED_TCP} TCP가 등록돼 있는지 확인하세요.') from exc
    active = read_tcp()
    if active != EXPECTED_TCP:
        raise RuntimeError(f'TCP 적용 후에도 현재 TCP가 {active!r}입니다. 펜던트 설정을 확인하세요.')
    info(f'TCP {EXPECTED_TCP} 적용 완료')


# 관절 이동(movej)에 필요한 서비스 클라이언트 묶음을 만든다. 반납 노드도 같은 함수를 쓴다.
def create_joint_clients(node, controller):
    from types import SimpleNamespace
    from dsr_msgs2.srv import Fkin, GetCurrentPosj, GetCurrentSolutionSpace, Ikin, MoveJoint
    return SimpleNamespace(
        joints=node.create_client(GetCurrentPosj, f'{controller}/aux_control/get_current_posj'),
        solution=node.create_client(GetCurrentSolutionSpace,
                                    f'{controller}/aux_control/get_current_solution_space'),
        ikin=node.create_client(Ikin, f'{controller}/motion/ikin'),
        joint=node.create_client(MoveJoint, f'{controller}/motion/move_joint'),
        fkin=node.create_client(Fkin, f'{controller}/motion/fkin'),
    )


# 관절값의 BASE TCP 자세를 컨트롤러 정기구학으로 계산한다(이동 없음). 두산 API: fkin(pos, DR_BASE).
def joint_pose(call, clients, joints, step):
    from dsr_msgs2.srv import Fkin
    request = Fkin.Request()
    request.pos, request.ref = [float(v) for v in joints], 0
    pose = tuple(call(clients.fkin, request, f'{step} 위치 계산').conv_posx)
    if len(pose) != 6 or not all(map(math.isfinite, pose)):
        raise RuntimeError(f'{step} 정기구학 결과가 유효하지 않습니다.')
    return pose


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args):
    import rclpy
    from utils.shutdown import safe_shutdown
    from utils.service_waiter import ServiceWaiter
    from .putty_geometry import orientation_error
    from dsr_msgs2.srv import (
        GetCurrentPosj, GetCurrentPosx, GetCurrentTcp, SetCurrentTcp, MoveLine, SetCtrlBoxDigitalOutput,
    )

    # 스크립트 옵션은 argparse에서 이미 처리했다.
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('pickup_spatula', namespace=ROBOT_ID)
        controller = f'/{ROBOT_ID}/dsr_controller2'
        joint_clients = create_joint_clients(node, controller)
        joints = joint_clients.joints
        pose = node.create_client(GetCurrentPosx, f'{controller}/aux_control/get_current_posx')
        tcp = node.create_client(GetCurrentTcp, f'{controller}/tcp/get_current_tcp')
        set_tcp = node.create_client(SetCurrentTcp, f'{controller}/tcp/set_current_tcp')
        line = node.create_client(MoveLine, f'{controller}/motion/move_line')
        if args.virtual_gripper:
            from onrobot_rg_msgs.srv import SetCommand
            output = node.create_client(SetCommand, '/onrobot/sendCommand')
        else:
            output = node.create_client(SetCtrlBoxDigitalOutput,
                                        f'{controller}/io/set_ctrl_box_digital_output')

        # ===== 서비스 준비 =====
        ServiceWaiter(node).wait(joints, joint_clients.solution, joint_clients.ikin, joint_clients.joint,
                                 pose, tcp, set_tcp, line, output)

        # 모든 요청은 이 함수로 전송하고 응답을 확인한다. 실패하면 다음 이동/출력을 보내지 않는다.
        def call(client, request, step, timeout=5.0):
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
            if not future.done():
                raise TimeoutError(f'{step} 응답 시간 초과. 다음 명령을 보내지 않습니다.')
            response = future.result()
            if response is None or not getattr(response, 'success', True):
                raise RuntimeError(f'{step} 서비스 실패')
            return response

        def read_pose():
            request = GetCurrentPosx.Request()
            request.ref = 0  # DR_BASE
            rows = call(pose, request, 'BASE TCP 자세 조회').task_pos_info
            values = tuple(rows[0].data[:6]) if rows else ()
            if len(values) != 6 or not all(map(math.isfinite, values)):
                raise RuntimeError('유효한 BASE TCP 자세를 읽지 못했습니다.')
            return values

        # ===== TCP 적용 및 시작 자세 확인 =====
        def apply_tcp(name):
            request = SetCurrentTcp.Request()
            request.name = name
            call(set_tcp, request, f'TCP {name} 적용')

        ensure_tcp(lambda: call(tcp, GetCurrentTcp.Request(), '현재 TCP 조회').info,
                   apply_tcp, lambda message: print(message, flush=True))

        positions = call(joints, GetCurrentPosj.Request(), '현재 관절각 조회').pos
        if (len(positions) != 6 or not all(map(math.isfinite, positions))
                or any(abs(value - home) > HOME_TOLERANCE_DEG
                       for value, home in zip(positions, HOME_J))):
            raise RuntimeError('홈 자세에서 시작해야 합니다. 초기화 후 다시 실행해 주세요.')
        home_pose = read_pose()

        # 두산 API: movel(동기). 이동이 끝나면 실제 TCP가 목표 2 mm / 2° 안에 왔는지 다시 확인한다.
        def move(step, target, speed):
            show_move(step, target, speed)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [20.0, 20.0]
            request.ref = 0       # DR_BASE
            request.mode = 0      # 절대 좌표
            request.sync_type = 0  # 동기 이동
            call(line, request, step, timeout=120.0)
            actual = read_pose()
            if (math.dist(actual[:3], target[:3]) > 2.0 or
                    orientation_error(actual[3:], target[3:]) > 2.0):
                raise RuntimeError(f'{step} 목표 XYZABC에 도착하지 않았습니다: {actual}')

        def set_output(step, channel, value):
            show_output(step, channel, value)
            request = SetCtrlBoxDigitalOutput.Request()
            request.index = channel
            request.value = value
            call(output, request, f'{step} 출력 {channel}번')

        def gripper_command(command):
            print(f'[가상 RG2] 명령 {command}', flush=True)
            request = SetCommand.Request()
            request.command = command
            call(output, request, f'가상 RG2 {command}', timeout=10.0)

        # 두산 API: movej(절대 관절각, 동기). 찍은 관절 자세로 바로 가므로 ikin을 쓰지 않는다.
        # 도착 후 실제 BASE TCP를 읽어 돌려준다. 다음 직선 이동의 출발 XY로 쓴다.
        def move_joints(step, joints):
            move_to_joints(call, joint_clients, step, joints, args.joint_speed, args.joint_acceleration,
                           info=lambda text: print(text, flush=True))
            reached = read_pose()
            print(f'[{step}] 도착 BASE TCP [{", ".join(f"{v:.2f}" for v in reached)}]', flush=True)
            return reached

        # ===== 집기 동작 =====
        pickup(home_pose, args.clearance, args.settle, move, move_joints, set_output, time.sleep,
               transfer_speed=args.transfer_speed, pick_speed=args.pick_speed,
               gripper_command=gripper_command if args.virtual_gripper else None)
        print('집기 및 상승 명령 완료. 헤라가 실제로 잡혔는지는 확인이 필요합니다.')
    finally:
        safe_shutdown(node)


if __name__ == '__main__':
    raise SystemExit(main())
