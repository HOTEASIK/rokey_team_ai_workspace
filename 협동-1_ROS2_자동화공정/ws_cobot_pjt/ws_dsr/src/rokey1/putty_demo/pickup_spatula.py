"""7번 위치에 놓인 헤라를 RG2로 집어 접근 높이까지 들어 올린다."""

import argparse
import math
import sys
import time


# ===== 설정 / 하이퍼파라미터 =====
HOME_J = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
HOME_TOLERANCE_DEG = 0.5
EXPECTED_TCP = 'GripperDA_v1'
# 집기 TCP의 직접 측정한 BASE XYZ. 단위는 mm.
PICK_POSITION = (589.0, 76.0, 152.0)
# 기존에 확인한 이송 높이를 유지한다.
MIN_APPROACH_Z_MM = 245.14
TRANSFER_SPEED_MM_S = 15.0
PICK_SPEED_MM_S = 5.0
# 반납 및 전체 사이클 드라이런에서 사용하는 기존 8번 기준값.
HOLE_8 = (724.14, 73.22, 35.14)
PICK_Z_OFFSET_MM = 160.0
RECORDED_HOME_POSE = (367.41, 3.06, 219.28, 140.70, 179.97, 141.21)
OPEN = ((1, 0), (2, 0), (2, 1))
CLOSE = ((1, 0), (2, 0), (1, 1), (2, 0))

# ===== 로봇 초기화 설정 =====
ROBOT_ID = 'dsr01'


# ===== 메인 =====
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--dry-run', action='store_true', help='가상 실행 (기본)')
    modes.add_argument('--execute-motion', action='store_true', help='실물 로봇에 요청 전송')
    parser.add_argument('--clearance', type=float, default=50.0, help='집기점 위 접근 거리(mm)')
    parser.add_argument('--settle', type=float, default=1.0, help='집게 출력 후 대기 시간(초)')
    parser.add_argument('--transfer-speed', type=float, default=TRANSFER_SPEED_MM_S,
                        help='접근 이송 속도(mm/s)')
    parser.add_argument('--pick-speed', type=float, default=PICK_SPEED_MM_S,
                        help='집기 하강 및 들어 올리기 속도(mm/s)')
    args = parser.parse_args()
    for name in ('transfer_speed', 'pick_speed'):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f'--{name.replace("_", "-")}는 0보다 큰 유한한 값이어야 합니다.')
    if not math.isfinite(args.clearance) or args.clearance <= 0:
        parser.error('--clearance는 0보다 큰 유한한 값이어야 합니다.')
    if not math.isfinite(args.settle) or args.settle < 0:
        parser.error('--settle은 0 이상인 유한한 값이어야 합니다.')

    try:
        if args.execute_motion:
            run_robot(args)
        else:
            run_dry(args)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'집기 중단: {exc}', file=sys.stderr)
        return 1


# ===== 함수 정의 =====
def pickup(home_pose, clearance, settle, move, set_output, wait,
           transfer_speed=TRANSFER_SPEED_MM_S, pick_speed=PICK_SPEED_MM_S):
    """실물과 dry-run에서 같은 경로 및 출력 순서를 사용한다."""
    x, y, pick_z = PICK_POSITION
    clear_z = round(max(pick_z + clearance, home_pose[2], MIN_APPROACH_Z_MM), 2)
    abc = tuple(home_pose[3:])
    approach = (x, y, clear_z, *abc)
    grasp = (x, y, pick_z, *abc)

    if clear_z > home_pose[2]:
        move('홈 위로 상승', (*home_pose[:2], clear_z, *abc), transfer_speed)
    move('헤라 바로 위로 이동', approach, transfer_speed)

    for channel, value in OPEN:
        set_output('RG2 열기', channel, value)
    wait(settle)
    move('집기 높이까지 하강', grasp, pick_speed)

    for channel, value in CLOSE:
        set_output('RG2 닫기', channel, value)
    wait(settle)
    move('헤라 들어 올리기', approach, pick_speed)


def show_move(step, pose, speed):
    values = ', '.join(f'{value:.2f}' for value in pose)
    print(f'[{step}] BASE [{values}] | {speed:g} mm/s', flush=True)


def show_output(step, channel, value):
    state = 'ON' if value else 'OFF'
    print(f'[{step}] 출력 {channel}번 {state}', flush=True)


def run_dry(args):
    print('[DRY RUN] 기록한 홈 자세로 경로를 계산합니다. 모든 요청은 로그로만 표시합니다.')
    pickup(RECORDED_HOME_POSE, args.clearance, args.settle, show_move, show_output,
           lambda seconds: print(f'[DRY RUN] 집게 동작 대기 {seconds:g}초'),
           transfer_speed=args.transfer_speed, pick_speed=args.pick_speed)
    print('[DRY RUN] 집기 명령 순서 확인 완료')


def run_robot(args):
    import rclpy
    from dsr_msgs2.srv import (
        GetCurrentPosj, GetCurrentPosx, GetCurrentTcp, MoveLine, SetCtrlBoxDigitalOutput,
        SetCurrentTcp,
    )

    from utils.shutdown import safe_shutdown

    # 스크립트 옵션은 argparse에서 이미 처리했다.
    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('pickup_spatula', namespace=ROBOT_ID)
        controller = f'/{ROBOT_ID}/dsr_controller2'
        joints = node.create_client(GetCurrentPosj, f'{controller}/aux_control/get_current_posj')
        pose = node.create_client(GetCurrentPosx, f'{controller}/aux_control/get_current_posx')
        tcp = node.create_client(GetCurrentTcp, f'{controller}/tcp/get_current_tcp')
        set_tcp = node.create_client(SetCurrentTcp, f'{controller}/tcp/set_current_tcp')
        line = node.create_client(MoveLine, f'{controller}/motion/move_line')
        output = node.create_client(SetCtrlBoxDigitalOutput,
                                    f'{controller}/io/set_ctrl_box_digital_output')

        # ===== 서비스 준비 =====
        for client in (joints, pose, tcp, set_tcp, line, output):
            while not client.wait_for_service(timeout_sec=1.0):
                print(f'서비스 대기 중: {client.srv_name}', flush=True)
                if not rclpy.ok():
                    raise RuntimeError('ROS 2 연결이 종료됐습니다.')

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
        # 컨트롤러를 다시 켜면 현재 TCP 가 비어('') 있으므로, 다르면 여기서 직접 적용한 뒤 다시 읽어 확인한다.
        # (PICK_POSITION 등 모든 좌표는 GripperDA_v1 TCP 기준)
        active_tcp = call(tcp, GetCurrentTcp.Request(), '현재 TCP 조회').info
        if active_tcp != EXPECTED_TCP:
            print(f'현재 TCP가 {active_tcp!r}입니다. {EXPECTED_TCP}을 적용합니다.', flush=True)
            request = SetCurrentTcp.Request()
            request.name = EXPECTED_TCP
            try:
                call(set_tcp, request, f'TCP {EXPECTED_TCP} 적용')
            except RuntimeError as e:
                raise RuntimeError(f'{e}. 티칭 펜던트에 {EXPECTED_TCP} TCP가 등록돼 있는지 확인해 주세요.') from e
            active_tcp = call(tcp, GetCurrentTcp.Request(), '현재 TCP 재조회').info
            if active_tcp != EXPECTED_TCP:
                raise RuntimeError(f'TCP 적용 후에도 현재 TCP가 {active_tcp!r}입니다. '
                                   f'티칭 펜던트에서 {EXPECTED_TCP}을 확인해 주세요.')
            print(f'TCP {EXPECTED_TCP} 적용 완료', flush=True)

        positions = call(joints, GetCurrentPosj.Request(), '현재 관절각 조회').pos
        if (len(positions) != 6 or not all(map(math.isfinite, positions))
                or any(abs(value - home) > HOME_TOLERANCE_DEG
                       for value, home in zip(positions, HOME_J))):
            raise RuntimeError('홈 자세에서 시작해야 합니다. 초기화 후 다시 실행해 주세요.')
        home_pose = read_pose()

        def move(step, target, speed):
            show_move(step, target, speed)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 0.0]
            request.acc = [20.0, 0.0]
            request.ref = 0       # DR_BASE
            request.mode = 0      # 절대 좌표
            request.sync_type = 0  # 동기 이동
            call(line, request, step, timeout=120.0)
            actual = read_pose()
            if math.dist(actual[:3], target[:3]) > 2.0:
                raise RuntimeError(f'{step} 목표 XYZ에 도착하지 않았습니다: {actual[:3]}')

        def set_output(step, channel, value):
            show_output(step, channel, value)
            request = SetCtrlBoxDigitalOutput.Request()
            request.index = channel
            request.value = value
            call(output, request, f'{step} 출력 {channel}번')

        # ===== 집기 동작 =====
        pickup(home_pose, args.clearance, args.settle, move, set_output, time.sleep,
               transfer_speed=args.transfer_speed, pick_speed=args.pick_speed)
        print('집기 및 상승 명령 완료. 헤라가 실제로 잡혔는지는 확인이 필요합니다.')
    finally:
        safe_shutdown(node)


if __name__ == '__main__':
    raise SystemExit(main())
