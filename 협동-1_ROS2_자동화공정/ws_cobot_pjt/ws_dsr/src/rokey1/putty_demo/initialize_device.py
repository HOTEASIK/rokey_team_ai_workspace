"""Return the arm to the taught home posture, then close the empty RG2."""

import argparse
import math
import sys
from types import SimpleNamespace


# ===== 설정 / 하이퍼파라미터 =====
HOME_J = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
HOME_TOLERANCE_DEG = 0.5
ON = 1
OFF = 0
DRY_RUN_SCENARIOS = (
    'away', 'already_home', 'joint_failure', 'move_failure', 'home_not_reached', 'io_failure',
)

# ===== 로봇 초기화 설정 =====
ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'


# ===== 메인 =====
def main(args=None):
    argv = sys.argv[1:] if args is None else args
    if '--dry-run' in argv:
        parser = argparse.ArgumentParser(description='가상 응답으로 초기화 순서를 검증합니다.')
        parser.add_argument('--dry-run', action='store_true')
        parser.add_argument('--scenario', choices=DRY_RUN_SCENARIOS, default='away')
        options = parser.parse_args(argv)
        try:
            run_dry_run(options.scenario, lambda message: print(f'[DRY RUN] {message}', flush=True))
            return 0
        except RuntimeError as exc:
            print(f'[DRY RUN] 초기화 실패: {exc}', file=sys.stderr)
            return 1

    import rclpy
    from rclpy.node import Node

    from utils.shutdown import safe_shutdown

    rclpy.init(args=args)
    node = Node('initialize_device', namespace=ROBOT_ID)
    try:
        initialize(node)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        node.get_logger().error(f'초기화 실패: {exc}')
        return 1
    finally:
        safe_shutdown(node)


# ===== 함수 정의 =====
def wait_for_result(node, future, timeout, operation):
    import rclpy

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f'{operation} 응답 대기 시간 초과. 장치 상태를 확인하세요.')
    response = future.result()
    if response is None:
        raise RuntimeError(f'{operation} 응답 없음')
    return response


def read_joint_positions(node, client):
    from dsr_msgs2.srv import GetCurrentPosj

    return wait_for_result(
        node, client.call_async(GetCurrentPosj.Request()), 5.0, '관절 조회'
    )


def checked_joint_positions(read_joints):
    result = read_joints()
    if not result.success or len(result.pos) != 6 or not all(map(math.isfinite, result.pos)):
        raise RuntimeError('현재 관절 자세를 읽지 못했습니다.')
    return result.pos


def at_home(positions):
    return all(abs(actual - target) <= HOME_TOLERANCE_DEG
               for actual, target in zip(positions, HOME_J))


def move_joint(node, client, target):
    from dsr_msgs2.srv import MoveJoint

    request = MoveJoint.Request()
    request.pos = list(target)
    request.vel = 10.0
    request.acc = 10.0
    request.mode = 0       # Absolute joint position.
    request.sync_type = 0  # The controller responds after the move.
    return wait_for_result(node, client.call_async(request), 120.0, '홈 이동')


def return_home(read_joints, move_home, info):
    if at_home(checked_joint_positions(read_joints)):
        info('이미 홈에 있습니다.')
        return

    result = move_home(HOME_J)
    if not result.success or not at_home(checked_joint_positions(read_joints)):
        raise RuntimeError('홈 도착을 확인하지 못했습니다.')
    info('홈에 도착했습니다.')


def close_gripper(set_digital_output):
    # 기존 grip_test.py의 닫기 순서. 두 출력을 끈 뒤 1번만 켠다.
    for channel, value in ((1, OFF), (2, OFF), (1, ON), (2, OFF)):
        if set_digital_output(channel, value) != 0:
            state = 'ON' if value == ON else 'OFF'
            raise RuntimeError(f'RG2 디지털 출력 {channel}번 {state} 설정 실패')


def run_initialization(read_joints, move_home, set_output, info):
    return_home(read_joints, move_home, info)
    close_gripper(set_output)
    info('초기화 완료: 홈 도착 및 RG2 닫기 출력 설정 성공')


def run_dry_run(scenario, info):
    if scenario not in DRY_RUN_SCENARIOS:
        raise ValueError(f'지원하지 않는 dry_run 시나리오: {scenario}')

    positions = HOME_J if scenario == 'already_home' else (5.0, *HOME_J[1:])

    def read_joints():
        success = scenario != 'joint_failure'
        info(f'가상 관절 조회 → success={success}, pos={list(positions)}')
        return SimpleNamespace(success=success, pos=positions)

    def move_home(target):
        nonlocal positions
        success = scenario != 'move_failure'
        if success and scenario != 'home_not_reached':
            positions = target
        info(f'가상 홈 이동 → 목표 {list(target)}, success={success}')
        return SimpleNamespace(success=success)

    def set_output(channel, value):
        code = -1 if scenario == 'io_failure' and (channel, value) == (1, ON) else 0
        state = 'ON' if value == ON else 'OFF'
        info(f'가상 RG2 출력 → {channel}번 {state}, 반환 코드={code}')
        return code

    info(f'시나리오: {scenario}. 모든 로봇 응답은 가상 데이터입니다.')
    run_initialization(read_joints, move_home, set_output, info)


def initialize(node):
    # 드라이런에서는 두산 드라이버를 불러오지 않는다.
    node.declare_parameter('execute_motion', False)
    if not node.get_parameter('execute_motion').value:
        node.get_logger().info(f'미리보기: 홈 {list(HOME_J)} → RG2 닫기 (1번 ON, 2번 OFF)')
        return

    from dsr_msgs2.srv import GetCurrentPosj, MoveJoint

    from utils.service_waiter import ServiceWaiter

    import DR_init

    # ===== DR_init 설정 =====
    # 두산 Python API는 import 시 이 노드를 사용해 서비스 클라이언트를 만든다.
    DR_init.__dsr__id = node.get_namespace().strip('/')
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = node
    import DSR_ROBOT2

    # ===== 서비스 준비 및 초기화 동작 =====
    controller = f"{node.get_namespace().rstrip('/')}/dsr_controller2"
    joints = node.create_client(GetCurrentPosj, f'{controller}/aux_control/get_current_posj')
    motion = node.create_client(MoveJoint, f'{controller}/motion/move_joint')
    node.get_logger().info(f'컨트롤러 서비스 경로: {controller}')
    ServiceWaiter(node).wait(joints, motion, DSR_ROBOT2._ros2_set_digital_output)
    run_initialization(
        lambda: read_joint_positions(node, joints),
        lambda target: move_joint(node, motion, target),
        DSR_ROBOT2.set_digital_output,
        node.get_logger().info,
    )


if __name__ == '__main__':
    raise SystemExit(main())
