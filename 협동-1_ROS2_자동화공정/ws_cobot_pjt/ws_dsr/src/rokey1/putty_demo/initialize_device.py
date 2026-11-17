"""Return the arm to the taught home posture, then close the empty RG2."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/initialize_device. 홈 관절 자세로 복귀한 뒤 RG2를 닫는다.
# 동작 순서: 실행 허용 확인 → 현재 관절 조회 → 필요 시 홈 이동 → 도착 확인 → 집게 닫기.
# 실물: DSR_ROBOT2가 ROS 서비스로 디지털 출력을 전송한다.
# 가상 집게: /onrobot/sendCommand 서비스에 c를 보내 닫는다.
# 다른 퍼티 노드와 직접 메시지를 주고받지 않는다. 순서는 launch가 프로세스 종료로 연결한다.

import math


# ===== 설정 / 하이퍼파라미터 =====
HOME_J = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)
HOME_TOLERANCE_DEG = 0.5
ON = 1
OFF = 0
# ===== 로봇 초기화 설정 =====
ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'


# ===== 메인 =====
# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main(args=None):
    import rclpy
    from utils.shutdown import safe_shutdown
    from rclpy.node import Node

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
# call_async가 반환한 Future를 spin으로 처리하고 제한 시간 안에 응답이 왔는지 확인한다.
def wait_for_result(node, future, timeout, operation):
    import rclpy

    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise TimeoutError(f'{operation} 응답 대기 시간 초과. 장치 상태를 확인하세요.')
    response = future.result()
    if response is None:
        raise RuntimeError(f'{operation} 응답 없음')
    return response


# GetCurrentPosj 서비스로 현재 관절각을 읽는다.
def read_joint_positions(node, client):
    from dsr_msgs2.srv import GetCurrentPosj

    return wait_for_result(
        node, client.call_async(GetCurrentPosj.Request()), 5.0, '관절 조회'
    )


# 서비스 응답 성공 여부와 관절 데이터 형식을 검사한다.
def checked_joint_positions(read_joints):
    result = read_joints()
    if not result.success or len(result.pos) != 6 or not all(map(math.isfinite, result.pos)):
        raise RuntimeError('현재 관절 자세를 읽지 못했습니다.')
    return result.pos


# 현재 관절각과 홈 관절각의 차이를 허용 오차와 비교한다.
def at_home(positions):
    return all(abs(actual - target) <= HOME_TOLERANCE_DEG
               for actual, target in zip(positions, HOME_J))


# MoveJoint 요청을 구성한다. pos는 관절각이고 mode=0은 절대값 이동이다.
def move_joint(node, client, target):
    from dsr_msgs2.srv import MoveJoint

    request = MoveJoint.Request()
    request.pos = list(target)
    request.vel = 10.0
    request.acc = 10.0
    request.mode = 0       # Absolute joint position.
    request.sync_type = 0  # The controller responds after the move.
    return wait_for_result(node, client.call_async(request), 120.0, '홈 이동')


# 이미 홈이면 생략하고, 아니면 이동 후 관절각을 다시 읽어 도착을 확인한다.
def return_home(read_joints, move_home, info):
    if at_home(checked_joint_positions(read_joints)):
        info('이미 홈에 있습니다.')
        return

    result = move_home(HOME_J)
    if not result.success or not at_home(checked_joint_positions(read_joints)):
        raise RuntimeError('홈 도착을 확인하지 못했습니다.')
    info('홈에 도착했습니다.')


# 실물 RG2 닫기용 디지털 출력 순서를 실행한다.
def close_gripper(set_digital_output):
    # 기존 grip_test.py의 닫기 순서. 두 출력을 끈 뒤 1번만 켠다.
    for channel, value in ((1, OFF), (2, OFF), (1, ON), (2, OFF)):
        if set_digital_output(channel, value) != 0:
            state = 'ON' if value == ON else 'OFF'
            raise RuntimeError(f'RG2 디지털 출력 {channel}번 {state} 설정 실패')


# 홈 복귀와 집게 닫기를 순서대로 묶는다.
def run_initialization(read_joints, move_home, set_output, info, virtual_close=None):
    return_home(read_joints, move_home, info)
    if virtual_close is not None:
        virtual_close()
        info('초기화 완료: 홈 도착 및 가상 RG2 닫기 성공')
    else:
        close_gripper(set_output)
        info('초기화 완료: 홈 도착 및 RG2 닫기 출력 설정 성공')




# 실물/가상 집게에 맞는 서비스와 드라이버를 준비한다.
def initialize(node):
    node.declare_parameter('execute_motion', False)
    node.declare_parameter('virtual_gripper', False)
    if not node.get_parameter('execute_motion').value:
        raise RuntimeError('이동 실행에는 execute_motion:=true를 지정하세요.')

    from dsr_msgs2.srv import GetCurrentPosj, MoveJoint

    from utils.service_waiter import ServiceWaiter

    if node.get_parameter('virtual_gripper').value:
        from onrobot_rg_msgs.srv import SetCommand
        from dsr_msgs2.srv import GetRobotSystem

        controller = f"{node.get_namespace().rstrip('/')}/dsr_controller2"
        joints = node.create_client(GetCurrentPosj, f'{controller}/aux_control/get_current_posj')
        motion = node.create_client(MoveJoint, f'{controller}/motion/move_joint')
        gripper = node.create_client(SetCommand, '/onrobot/sendCommand')
        system = node.create_client(GetRobotSystem, f'{controller}/system/get_robot_system')
        ServiceWaiter(node).wait(joints, motion, gripper, system)
        result = wait_for_result(node, system.call_async(GetRobotSystem.Request()),
                                 5.0, '가상 시스템 확인')
        if not result.success or result.robot_system != 1:
            raise RuntimeError('가상 집게 초기화는 가상 컨트롤러에서만 허용합니다.')

        def virtual_close():
            request = SetCommand.Request()
            request.command = 'c'
            result = wait_for_result(node, gripper.call_async(request), 10.0, '가상 RG2 닫기')
            if not result.success:
                raise RuntimeError(f'가상 RG2 닫기 실패: {result.message}')

        run_initialization(
            lambda: read_joint_positions(node, joints),
            lambda target: move_joint(node, motion, target),
            None,
            node.get_logger().info,
            virtual_close=virtual_close,
        )
        return

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
