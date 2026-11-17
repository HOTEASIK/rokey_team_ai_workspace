"""도포 종료 자세에서 헤라를 집기 위치에 반납하는 경로를 계획한다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/return_spatula. 도포 후 헤라를 집기 위치에 내려놓는다.
# 동작 순서: 현재 TCP 조회 → 상승 → 집기 자세 복원 → 보관 위치 접근/하강 → 열기 → 상승.
# 통신: dsr_msgs2 서비스로 이동/조회, /onrobot/sendCommand로 집게 열기.
# 현재 이 노드의 실행 경로는 가상 컨트롤러 전용이다. 홈 복귀는 초기화 노드가 담당한다.

import argparse
import math
import sys

from .putty_geometry import EDGE_OFFSET, SURFACE_Z, format_values, orientation_error
from .pickup_spatula import PICK_POSITION, PICKUP_ABC, RECORDED_HOME_POSE, EXPECTED_TCP, ROBOT_ID


# ===== 설정 / 하이퍼파라미터 =====
TRANSFER_Z = 245.14


# ===== 메인 =====
# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True, help='가상 컨트롤러에서 반납 실행')
    parser.add_argument('--transfer-speed', type=float, default=30.0, help='이송 속도(mm/s)')
    parser.add_argument('--place-speed', type=float, default=10.0, help='하강·후퇴 속도(mm/s)')
    parser.add_argument('--settle', type=float, default=1.0, help='열기 후 대기(초)')
    parser.add_argument('--surface-z', type=float, default=SURFACE_Z,
                        help='도포 노드와 같은 시험판 높이 (BASE mm)')
    parser.add_argument('--edge-offset', type=float, nargs=3, default=EDGE_OFFSET,
                        help='도포 노드와 같은 TCP → 날 오프셋 (Tool XYZ mm)')
    parser.add_argument('--transfer-z', type=float, default=TRANSFER_Z,
                        help='회전과 수평 이송 높이 (BASE mm)')
    args = parser.parse_args()
    if not all(map(math.isfinite, (args.surface_z, args.transfer_z, *args.edge_offset))):
        parser.error('높이와 오프셋은 유한한 값이어야 합니다.')

    for name in ('transfer_speed', 'place_speed', 'settle'):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != 'settle' and value == 0):
            parser.error(f'{name} 값이 유효하지 않습니다.')
    try:
        run_robot(args)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'반납 중단: {exc}', file=sys.stderr)
        return 1


# ===== 함수 정의 =====
# 집게를 열기 전/후의 반납 경유점을 나눠 반환한다.
def return_waypoints(end_pose, home_pose, transfer_z):
    """BASE 기준 TCP 6축 자세와 집게를 여는 시점을 반환한다."""
    home_abc = PICKUP_ABC
    x, y, place_z = PICK_POSITION
    if transfer_z <= max(place_z, home_pose[2]):
        raise ValueError('이송 높이는 반납점과 홈의 TCP 높이보다 높아야 합니다.')
    transfer_z = max(transfer_z, end_pose[2])

    lifted = (*end_pose[:2], transfer_z)
    above_holder = (x, y, transfer_z, *home_abc)
    return (
        ('시험판에서 수직으로 떼고 상승', (*lifted, *end_pose[3:])),
        ('이송 높이에서 집기 방향으로 회전', (*lifted, *home_abc)),
        ('집기 보관 위치 위로 이동', above_holder),
        ('헤라를 내려놓기', (x, y, place_z, *home_abc)),
    ), (
        ('그리퍼 수직 후퇴', above_holder),
    )


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args):
    import time
    import rclpy
    from utils.shutdown import safe_shutdown
    from dsr_msgs2.srv import GetRobotSystem, GetCurrentTcp, GetCurrentPosx, MoveLine
    from onrobot_rg_msgs.srv import SetCommand
    from utils.service_waiter import ServiceWaiter

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('return_spatula', namespace=ROBOT_ID)
        prefix = f'/{ROBOT_ID}/dsr_controller2'
        system = node.create_client(GetRobotSystem, f'{prefix}/system/get_robot_system')
        tcp = node.create_client(GetCurrentTcp, f'{prefix}/tcp/get_current_tcp')
        pose = node.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx')
        motion = node.create_client(MoveLine, f'{prefix}/motion/move_line')
        gripper = node.create_client(SetCommand, '/onrobot/sendCommand')
        ServiceWaiter(node).wait(system, tcp, pose, motion, gripper)

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
            raise RuntimeError('반납 실행은 가상 컨트롤러에서만 허용합니다.')
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

        def move(step, target, speed):
            print(f'[{step}] BASE [{format_values(target)}] | {speed:g} mm/s', flush=True)
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [20.0, 20.0]
            request.ref = request.mode = request.sync_type = 0
            call(motion, request, step, timeout=120.0)
            actual = read_pose()
            distance = math.dist(actual[:3], target[:3])
            angle = orientation_error(actual[3:], target[3:])
            if distance > 2.0 or angle > 2.0:
                raise RuntimeError(f'{step} 미도착: 오차 {distance:.2f} mm, {angle:.2f}도')

        before, after = return_waypoints(read_pose(), RECORDED_HOME_POSE, args.transfer_z)
        for index, (step, target) in enumerate(before):
            move(step, target, args.place_speed if index in (0, len(before)-1)
                 else args.transfer_speed)
        request = SetCommand.Request()
        request.command = 'o'
        print('[가상 RG2] 열기', flush=True)
        call(gripper, request, '가상 RG2 열기', timeout=10.0)
        time.sleep(args.settle)
        for step, target in after:
            move(step, target, args.place_speed)
        print('가상 반납 및 수직 후퇴 완료', flush=True)
    finally:
        safe_shutdown(node)


if __name__ == '__main__':
    raise SystemExit(main())
