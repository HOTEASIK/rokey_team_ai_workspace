"""도포 종료 자세에서 헤라를 보관대에 반납한다. 실물은 바닥 접촉을 힘으로 감지해 내려놓는다."""

# ===== 파일 역할 / 통신 안내 =====
# 실행 노드: /dsr01/return_spatula. 도포 후 헤라를 집기 위치(pickup_spatula.PICK_POSE)에 내려놓는다.
# PICK_POSE는 헤라가 보관대 바닥에 완전히 앉은 높이다. 도포 중 파지가 밀리면 실제 바닥 높이가 달라지므로
# 실물에서는 고정 높이로 내리지 않고 바닥 접촉을 힘으로 찾는다.
#
# 보관대 근처에 기둥이 있어 movej로 보관대 위까지 바로 가면 부딪친다(2026-11-19 현장). 그래서 집기와 같은
# 경유 관절 자세(pickup_spatula.HOLDER_DETOUR_J)를 쓰고, 보관대 쪽 구간은 직선(movel)으로만 움직인다.
#
# 동작 순서와 두산 API:
#   현재 TCP 조회, 경유 관절 자세의 TCP 계산(fkin) → 이송 높이까지 수직 상승(movel, 동기)
#   → 홈 관절 자세로 관절 이동(movej, 동기)
#   → 경유 관절 자세로 관절 이동(movej, 동기, 관절값 그대로): 반시계, 자세도 이때 집기 방향으로 바뀐다
#   → 보관대 위로 직선 이동(movel, 동기) → 바닥보다 PLACE_SEARCH_START_MM 위까지 하강(movel, 동기)
#   [실물] → 컴플라이언스(task_compliance_ctrl, Z만 부드럽게)
#          → 비동기 하강(amovel) 중 외력(get_tool_force) 감시 → 바닥 접촉 시 정지(MoveStop) 후 check_motion으로 확인
#          → 눌린 스프링 풀기(movel, 현재 위치로) → 컴플라이언스 해제(release_compliance_ctrl)
#          → 그리퍼 열기(set_digital_output)
#   [가상] → 바닥 높이까지 하강(movel) → 그리퍼 열기(onrobot). 시뮬레이터 외력값은 믿을 수 없어 힘 탐색을 쓰지 않는다.
#   → 수직 후퇴(movel) → 경유지로 직선 이동(movel)
#   → 홈으로 관절 이동(movej): 집기 때 기둥에 닿지 않는 관절 경로를 거꾸로 간다.
#   → 홈에서 그리퍼 닫기(set_digital_output / 가상은 onrobot).

import argparse
import math
import sys
import time
from types import SimpleNamespace

from .contact_search import seek_axis
from .putty_geometry import format_values, orientation_error
from .pickup_spatula import (
    PICK_POSITION, PICKUP_ABC, RECORDED_HOME_POSE, EXPECTED_TCP, ROBOT_ID, OPEN, CLOSE, HOME_J, HOLDER_DETOUR_J,
    HOLDER_TRANSFER_Z_MM, JOINT_SPEED_DEG_S, JOINT_ACCELERATION_DEG_S2, check_holder_clearance,
    check_pillar_clearance, create_joint_clients, joint_pose, move_to_joints,
)


# ===== 설정 / 하이퍼파라미터 =====
TRANSFER_Z = 245.14   # 도포 끝에서 먼저 올라갈 최소 높이. 보관대 위 높이는 HOLDER_TRANSFER_Z_MM(230)을 따로 쓴다.
# 바닥 접촉 탐색. 값은 도포의 바닥 탐색(인수인계본 실측: 5 N, Z 강성 1000 N/m)과 맞췄다.
PLACE_SEARCH_START_MM = 15.0      # 바닥에 앉는 높이보다 이만큼 위에서 탐색을 시작한다
PLACE_SEARCH_DISTANCE_MM = 30.0   # 탐색 최대 하강 거리. 바닥 높이 아래 15 mm까지
PLACE_SEARCH_SPEED_MM_S = 2.0
PLACE_CONTACT_FORCE_N = 5.0       # 탐색 시작 대비 Fz 변화가 이 이상이면 바닥에 닿은 것으로 본다
PLACE_CONTACT_CONFIRM = 2         # 연속 감지 횟수(순간 잡음 오판 방지)
# 헤라는 길이 방향으로 단단해서 위치제어로 바닥에 닿으면 힘이 순간적으로 치솟아 충돌 감지(7060)가 걸린다.
# Z만 1000 N/m(1 mm당 1 N)로 부드럽게 해서 닿은 뒤에도 힘이 천천히 오르게 한다.
PLACE_STIFFNESS = (3000.0, 3000.0, 1000.0, 200.0, 200.0, 200.0)
# 찾은 바닥이 기록된 바닥 높이와 이보다 크게 다르면 보관대 모서리 등 다른 곳에 닿은 것으로 보고 놓지 않는다.
# 헤라 파지 깊이 편차(최대 약 10 mm, 인수인계 기록)를 허용한다.
PLACE_HEIGHT_TOLERANCE_MM = 10.0


# ===== 메인 =====
# 터미널에서 실행할 때의 시작점. 인자를 읽고 검증한 뒤 실행 함수를 호출한다.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-motion', action='store_true', required=True,
                        help='real/virtual 컨트롤러에 반납 이동 요청')
    # 이송 구간이 관절 이동(movej)으로 바뀌어 지금은 쓰지 않는다. 기존 launch/HMI 명령 호환을 위해 남긴다.
    parser.add_argument('--transfer-speed', type=float, default=30.0,
                        help='(미사용, 호환용) 이송은 --joint-speed로 조절한다')
    parser.add_argument('--place-speed', type=float, default=10.0, help='하강·후퇴 속도(mm/s)')
    parser.add_argument('--settle', type=float, default=1.0, help='열기 후 대기(초)')
    parser.add_argument('--transfer-z', type=float, default=TRANSFER_Z,
                        help='도포 끝에서 먼저 올라갈 최소 높이 (BASE TCP Z mm). 보관대 위 높이는 230 고정')
    parser.add_argument('--joint-speed', type=float, default=JOINT_SPEED_DEG_S,
                        help='보관 위치 위로 관절 이동하는 속도(deg/s)')
    parser.add_argument('--joint-acceleration', type=float, default=JOINT_ACCELERATION_DEG_S2,
                        help='보관 위치 위로 관절 이동하는 가속도(deg/s²)')
    parser.add_argument('--contact-force', type=float, default=PLACE_CONTACT_FORCE_N,
                        help='실물 바닥 접촉 감지 기준: 탐색 시작 대비 Fz 변화 N')
    parser.add_argument('--search-speed', type=float, default=PLACE_SEARCH_SPEED_MM_S,
                        help='실물 바닥 접촉 탐색 하강 속도(mm/s)')
    parser.add_argument('--contact-confirm', type=int, default=PLACE_CONTACT_CONFIRM,
                        help='바닥 접촉 판정 연속 감지 횟수')
    parser.add_argument('--seat-tolerance', type=float, default=PLACE_HEIGHT_TOLERANCE_MM,
                        help='찾은 바닥 높이가 기록과 이 이상 다르면 놓지 않음 mm')
    args = parser.parse_args()
    if not math.isfinite(args.transfer_z):
        parser.error('이송 높이는 유한한 값이어야 합니다.')

    for name in ('transfer_speed', 'place_speed', 'settle', 'joint_speed', 'joint_acceleration',
                 'contact_force', 'search_speed', 'seat_tolerance'):
        value = getattr(args, name)
        if not math.isfinite(value) or value < 0 or (name != 'settle' and value == 0):
            parser.error(f'{name} 값이 유효하지 않습니다.')
    if args.contact_confirm < 1:
        parser.error('contact_confirm 은 1 이상이어야 합니다.')
    try:
        run_robot(args)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'반납 중단: {exc}', file=sys.stderr)
        return 1


# ===== 함수 정의 =====
# 집게를 열기 전/후의 반납 경유점을 나눠 반환한다. 각 항목은 (설명, 목표, 이동 종류)이다.
#   'movel' : 목표는 TCP XYZABC, 직선 이동
#   'joints': 목표는 관절값 6개, 그 값 그대로 관절 이동(movej, ikin 없음)
# before의 마지막 점은 바닥 접촉 탐색 시작점(바닥 + PLACE_SEARCH_START_MM)이다.
# detour_pose는 경유 관절 자세(HOLDER_DETOUR_J)의 BASE TCP(fkin 결과)다.
def return_waypoints(end_pose, home_pose, transfer_z, detour_pose):
    """반납 경로(들어갈 때 / 놓은 뒤 나올 때)를 반환한다."""
    x, y, place_z = PICK_POSITION
    if transfer_z <= max(place_z, home_pose[2]):
        raise ValueError('이송 높이는 반납점과 홈의 TCP 높이보다 높아야 합니다.')
    check_holder_clearance(HOLDER_TRANSFER_Z_MM)

    lifted = (*end_pose[:2], max(transfer_z, end_pose[2]), *end_pose[3:])
    above_holder = (x, y, HOLDER_TRANSFER_Z_MM, *PICKUP_ABC)
    search_start = (x, y, place_z+PLACE_SEARCH_START_MM, *PICKUP_ABC)
    # 경유지 ↔ 보관대 위 직선 구간이 기둥과 충분히 떨어져 있는지 출발 전에 확인한다.
    check_pillar_clearance((detour_pose, above_holder))
    return (
        ('시험판에서 수직으로 떼고 상승', lifted, 'movel'),
        # 벽 앞에서 바로 보관대 쪽으로 돌면 팔이 펴진 채 기둥 쪽을 지난다. 먼저 홈 자세로 모은다.
        ('홈 관절 자세로 관절 이동', HOME_J, 'joints'),
        # 홈 → 경유지는 집기와 같은 관절 경로(반시계)다. 집기 방향 자세 정렬도 이때 된다.
        ('경유 관절 자세로 관절 이동', HOLDER_DETOUR_J, 'joints'),
        ('보관대 위로 직선 이동', above_holder, 'movel'),
        ('바닥 탐색 시작점까지 하강', search_start, 'movel'),
    ), (
        ('그리퍼 수직 후퇴', above_holder, 'movel'),
        ('경유지로 직선 이동', detour_pose, 'movel'),
        # 집기 때 홈 → 경유지로 기둥에 닿지 않은 관절 경로를 그대로 거꾸로 간다(시계 방향).
        ('홈으로 관절 이동', HOME_J, 'joints'),
    )


# 찾은 바닥 높이가 기록된 바닥 높이와 크게 다르면 헤라를 놓지 않는다.
def check_seated_height(contact_z, place_z=PICK_POSITION[2], tolerance=PLACE_HEIGHT_TOLERANCE_MM):
    if abs(contact_z-place_z) > tolerance:
        raise RuntimeError(f'바닥 접촉 높이 TCP Z {contact_z:.2f} mm가 기록된 바닥 {place_z:.2f} mm와 '
                           f'{tolerance:g} mm 넘게 다릅니다. 보관대 위치·헤라 파지를 확인하세요. 그리퍼는 열지 않았습니다.')


# ROS 노드와 클라이언트를 준비하고 요청·응답·도착 확인을 순서대로 실행한다.
def run_robot(args):
    import rclpy
    from utils.shutdown import safe_shutdown
    from utils.service_waiter import ServiceWaiter
    from dsr_msgs2.srv import (
        GetRobotSystem, GetCurrentTcp, GetCurrentPosx, MoveLine, GetToolForce, MoveStop, CheckMotion,
        TaskComplianceCtrl, ReleaseComplianceCtrl, SetCtrlBoxDigitalOutput,
    )
    from onrobot_rg_msgs.srv import SetCommand

    rclpy.init(args=[])
    node = None
    compliance_active = False
    try:
        node = rclpy.create_node('return_spatula', namespace=ROBOT_ID)
        prefix = f'/{ROBOT_ID}/dsr_controller2'
        system = node.create_client(GetRobotSystem, f'{prefix}/system/get_robot_system')
        tcp = node.create_client(GetCurrentTcp, f'{prefix}/tcp/get_current_tcp')
        pose = node.create_client(GetCurrentPosx, f'{prefix}/aux_control/get_current_posx')
        motion = node.create_client(MoveLine, f'{prefix}/motion/move_line')
        joint_clients = create_joint_clients(node, prefix)
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

        robot_system = call(system, GetRobotSystem.Request(), '시스템 확인').robot_system
        if robot_system not in (0, 1):
            raise RuntimeError(f'지원하지 않는 로봇 시스템: {robot_system}')
        real = robot_system == 0
        if call(tcp, GetCurrentTcp.Request(), 'TCP 확인').info != EXPECTED_TCP:
            raise RuntimeError(f'{EXPECTED_TCP} TCP를 적용하세요.')

        # 실물/가상에 따라 필요한 서비스가 다르다. 실물: 외력·정지·컴플라이언스·DO / 가상: onrobot 집게.
        if real:
            force = node.create_client(GetToolForce, f'{prefix}/aux_control/get_tool_force')
            stop = node.create_client(MoveStop, f'{prefix}/motion/move_stop')
            motion_check = node.create_client(CheckMotion, f'{prefix}/motion/check_motion')
            compliance = node.create_client(TaskComplianceCtrl, f'{prefix}/force/task_compliance_ctrl')
            release_comp = node.create_client(ReleaseComplianceCtrl,
                                              f'{prefix}/force/release_compliance_ctrl')
            output = node.create_client(SetCtrlBoxDigitalOutput, f'{prefix}/io/set_ctrl_box_digital_output')
            ServiceWaiter(node).wait(force, stop, motion_check, compliance, release_comp, output)
        else:
            gripper = node.create_client(SetCommand, '/onrobot/sendCommand')
            ServiceWaiter(node).wait(gripper)

        # 두산 API: get_current_posx(ref=DR_BASE).
        def read_pose():
            request = GetCurrentPosx.Request()
            request.ref = 0
            rows = call(pose, request, 'BASE TCP 조회').task_pos_info
            values = tuple(rows[0].data[:6]) if rows else ()
            if len(values) != 6 or not all(map(math.isfinite, values)):
                raise RuntimeError('BASE TCP 자세가 유효하지 않습니다.')
            return values

        # 두산 API: movel(동기). 이동이 끝나면 실제 TCP가 목표 2 mm / 2° 안에 왔는지 다시 확인한다.
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

        # 두산 API: movej(절대 관절각, 동기). 홈·경유지처럼 관절값이 정해진 목표는 ikin 없이 그대로 보낸다.
        def move_joints(step, joints):
            move_to_joints(call, joint_clients, step, joints, args.joint_speed, args.joint_acceleration,
                           info=lambda text: print(text, flush=True), direction=None)

        def run(waypoints):
            for step, target, kind in waypoints:
                if kind == 'joints':
                    move_joints(step, target)
                else:
                    move(step, target, args.place_speed)

        # 두산 API: get_tool_force(DR_BASE). [Fx,Fy,Fz,Mx,My,Mz]
        def read_force():
            request = GetToolForce.Request()
            request.ref = 0
            return call(force, request, 'BASE 외력 조회', timeout=0.5).tool_force

        # 두산 API: amovel. 이동 완료를 기다리지 않아서 하강 중에 외력을 계속 읽을 수 있다.
        def move_async(target, speed):
            request = MoveLine.Request()
            request.pos = list(target)
            request.vel = [speed, 10.0]
            request.acc = [20.0, 20.0]
            request.ref = request.mode = 0
            request.sync_type = 1
            call(motion, request, '바닥 접촉 탐색 하강', timeout=3.0)

        # 두산 API: stop(DR_HOLD) + check_motion. 컴플라이언스 중에는 GetRobotState가 이동 중에도
        # STANDBY를 줄 수 있어(인수인계 기록) 정지 확인은 check_motion(0=IDLE)으로 한다.
        def stop_and_wait():
            request = MoveStop.Request()
            request.stop_mode = 3  # DR_HOLD
            call(stop, request, '바닥 접촉 정지', timeout=2.0)
            deadline = time.monotonic()+5.0
            while call(motion_check, CheckMotion.Request(), '이동 상태 확인', timeout=0.5).status != 0:
                if time.monotonic() >= deadline:
                    raise TimeoutError('바닥 접촉 정지 완료를 확인하지 못했습니다.')
                time.sleep(0.05)

        # 두산 API: task_compliance_ctrl(stx, time=0.3).
        def start_compliance():
            nonlocal compliance_active
            request = TaskComplianceCtrl.Request()
            request.stx = list(PLACE_STIFFNESS)
            request.ref, request.time = 0, 0.3
            compliance_active = True  # 응답 실패여도 적용됐을 수 있으므로 정리 대상으로 표시한다
            call(compliance, request, '바닥 탐색 컴플라이언스 시작')
            time.sleep(0.3)  # 컴플라이언스 전환 시간(time=0.3) 동안 기다린다

        # 두산 API: release_compliance_ctrl.
        def release_compliance():
            nonlocal compliance_active
            call(release_comp, ReleaseComplianceCtrl.Request(), '컴플라이언스 해제')
            compliance_active = False

        # 바닥에 닿아 멈춘 뒤에는 목표 위치가 실제보다 아래에 남아 스프링이 눌려 있다.
        # 그대로 해제하면 덜컹거리므로, 현재 실제 위치로 movel해 눌림을 0으로 만든 뒤 해제한다.
        def unload_and_release():
            here = read_pose()
            request = MoveLine.Request()
            request.pos = list(here)
            request.vel = [2.0, 10.0]
            request.acc = [20.0, 20.0]
            request.ref = request.mode = request.sync_type = 0
            call(motion, request, '바닥 접촉 눌림 풀기', timeout=10.0)
            time.sleep(0.3)
            release_compliance()

        # 두산 API: set_digital_output. 실물 RG2 열기·닫기 순서는 pickup_spatula.OPEN/CLOSE와 같다.
        # 가상은 onrobot SetCommand('o'/'c')다.
        def set_gripper(name, pattern, command):
            if real:
                for channel, value in pattern:
                    request = SetCtrlBoxDigitalOutput.Request()
                    request.index, request.value = channel, value
                    call(output, request, f'RG2 {name} 출력 {channel}번')
            else:
                request = SetCommand.Request()
                request.command = command
                call(gripper, request, f'가상 RG2 {name}', timeout=10.0)
            print(f'[RG2] {name}', flush=True)

        # ===== 반납 동작 =====
        # 두산 API: fkin. 경유 관절 자세의 TCP를 계산만 한다(이동 없음).
        detour_pose = joint_pose(call, joint_clients, HOLDER_DETOUR_J, '경유 관절 자세')
        before, after = return_waypoints(read_pose(), RECORDED_HOME_POSE, args.transfer_z, detour_pose)
        run(before)
        if real:
            start_compliance()
            io = SimpleNamespace(read_pose=read_pose, read_force=read_force,
                                 move_async=move_async, stop_and_wait=stop_and_wait)
            print(f'[바닥 탐색] -Z 최대 {PLACE_SEARCH_DISTANCE_MM:g} mm, {args.search_speed:g} mm/s, '
                  f'Fz 변화 {args.contact_force:g} N에서 정지', flush=True)
            contact = seek_axis(io, 2, -1, args.contact_force, PLACE_SEARCH_DISTANCE_MM,
                                args.search_speed,
                                confirm=getattr(args, 'contact_confirm', PLACE_CONTACT_CONFIRM))
            print(f'[바닥 탐색] 접촉 TCP Z {contact[2]:.2f} mm (기록 바닥 {PICK_POSITION[2]:.2f} mm)',
                  flush=True)
            check_seated_height(contact[2], tolerance=getattr(args, 'seat_tolerance', PLACE_HEIGHT_TOLERANCE_MM))
            unload_and_release()
        else:
            move('바닥 높이까지 하강(가상)', (*PICK_POSITION, *PICKUP_ABC), args.place_speed)
        set_gripper('열기', OPEN, 'o')
        time.sleep(args.settle)
        run(after)
        # 홈에 도착한 뒤 그리퍼를 닫아 다음 작업(초기화와 같은 닫힌 상태)에 맞춘다.
        set_gripper('닫기', CLOSE, 'c')
        print('반납, 홈 복귀, 그리퍼 닫기 완료', flush=True)
    finally:
        # 실패로 끝났는데 컴플라이언스가 켜져 있으면 정지 후 해제를 시도한다. 그리퍼는 열지 않는다.
        if compliance_active and node is not None:
            try:
                stop_and_wait()
            except Exception as exc:
                node.get_logger().error(f'정리 중 정지 확인 실패: {exc}')
            try:
                release_compliance()
            except Exception as exc:
                node.get_logger().error(f'정리 중 컴플라이언스 해제 실패: {exc}')
        safe_shutdown(node)


if __name__ == '__main__':
    raise SystemExit(main())
