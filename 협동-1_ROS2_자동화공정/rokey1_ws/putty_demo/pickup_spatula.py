"""지정 보관 위치에 놓인 헤라를 RG2로 집어 접근 높이까지 들어 올린다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/pickup_spatula. 지정한 BASE TCP XYZABC에서 헤라를 집는다.
# 동작 순서: TCP 적용/확인 → 홈 관절 확인 → 상승/이송 → 열기 → 하강 → 닫기 → 상승.
# 통신: dsr_msgs2 서비스로 자세 조회·TCP 선택·직선 이동·디지털 출력을 요청한다.
# --virtual-gripper 사용 시 여닫기만 onrobot_rg_msgs/SetCommand 서비스로 바뀐다.
# 집게 닫기 요청 성공이 실제 헤라 파지 성공을 뜻하지는 않는다.

import argparse
import math
import sys
import time


# ===== 설정 / 하이퍼파라미터 =====
HOME_J = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
HOME_TOLERANCE_DEG = 0.5
EXPECTED_TCP = 'GripperDA_v1'
# 집기 TCP의 측정 BASE XYZABC. 단위는 mm, degree.
PICK_POSE = (308.42, 120.70, 152.43, 18.32, 178.35, 17.14)
PICK_POSITION = PICK_POSE[:3]
PICKUP_ABC = PICK_POSE[3:]
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
    args = parser.parse_args()
    for name in ('transfer_speed', 'pick_speed'):
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
# 경유점과 집게 동작 순서만 정의한다. 실제 요청은 전달받은 move/출력 함수가 맡는다.
def pickup(home_pose, clearance, settle, move, set_output, wait,
           transfer_speed=TRANSFER_SPEED_MM_S, pick_speed=PICK_SPEED_MM_S, gripper_command=None):
    """집기 경로 및 그리퍼 출력 순서를 실행한다."""
    x, y, pick_z = PICK_POSITION
    clear_z = round(max(pick_z + clearance, home_pose[2], MIN_APPROACH_Z_MM), 2)
    abc = PICKUP_ABC
    approach = (x, y, clear_z, *abc)
    grasp = (x, y, pick_z, *abc)

    if clear_z > home_pose[2]:
        move('홈 위로 상승', (*home_pose[:2], clear_z, *home_pose[3:]), transfer_speed)
    move('헤라 바로 위로 이동', approach, transfer_speed)

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
        joints = node.create_client(GetCurrentPosj, f'{controller}/aux_control/get_current_posj')
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
        ServiceWaiter(node).wait(joints, pose, tcp, set_tcp, line, output)

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

        # ===== 집기 동작 =====
        pickup(home_pose, args.clearance, args.settle, move, set_output, time.sleep,
               transfer_speed=args.transfer_speed, pick_speed=args.pick_speed,
               gripper_command=gripper_command if args.virtual_gripper else None)
        print('집기 및 상승 명령 완료. 헤라가 실제로 잡혔는지는 확인이 필요합니다.')
    finally:
        safe_shutdown(node)


if __name__ == '__main__':
    raise SystemExit(main())
