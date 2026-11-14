#!/usr/bin/env python3
"""DART 프로그램 'm0609_pick_3'를 ROS 2 / DSR_ROBOT2 용으로 옮긴 노드.

[작업 개요]
  - 두산 M0609 로봇으로 기어 4개를 순서대로 집어서(pick) 놓는다(place).
  - 1~3번은 관절 좌표(movej), 4번은 직교 좌표(movel)로 이동한다.
  - 집을 때(-50N)와 놓을 때(-80N) 힘 제어로 아래 방향으로 눌러준다.
  - 4번 작업은 놓은 뒤 손목을 좌우로 흔들어(move_periodic) 끼워 맞춘다.

[주의]
  실제 로봇을 돌리기 전에 반드시 아래 항목을 확인할 것.
    - 티칭 펜던트에 등록된 Tool / TCP 이름
    - 그리퍼 디지털 출력(1번, 2번) 배선
    - 각 좌표의 궤적 (충돌 여부)
    - 힘 제어 방향과 크기

[실행]
  source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
  ros2 run rokey1 sun_gear
"""
import rclpy
import DR_init

# =============================================================================
# 로봇 / 공구 설정
# =============================================================================
ROBOT_ID = 'dsr01'             # 로봇 네임스페이스 (bringup 할 때 쓴 name과 같아야 함)
ROBOT_MODEL = 'm0609'          # 로봇 모델명
TOOL_NAME = 'Tool Weight'      # 티칭 펜던트에 등록된 공구 무게 이름 (힘 센서 보정에 사용)
TCP_NAME = 'GripperDA_v1'      # 티칭 펜던트에 등록된 TCP 이름 (movel 좌표 기준점)

# =============================================================================
# 안전 스위치
# =============================================================================
# DRY_RUN=True 이면 로봇에 아무 명령도 보내지 않고, 실행될 순서만 로그로 출력한다.
# 실제 로봇을 움직이려면 충분히 확인한 뒤 DRY_RUN=False 로 바꾼다.
#
# [주의] ENABLE_FORCE=False 이면 로봇이 부품까지 눌러 내려가지 않는다.
#        (실제로 부품에 닿는 동작은 힘 제어 단계에서 일어남)
#        → 모션만 확인할 때는 ENABLE_GRIPPER 도 같이 False 로 둘 것.
#
# [권장 테스트 순서]
#   1) DRY_RUN=True                         : 로그로 순서 확인
#   2) DRY_RUN=False, 나머지 전부 False      : 모션만 확인 (속도 낮게)
#   3) ENABLE_FORCE=True                    : 힘 로그 보면서 PICK_FORCE_RATIO 조정
#   4) ENABLE_GRIPPER / ENABLE_PERIODIC=True : 전체 공정 확인
DRY_RUN = False
ENABLE_FORCE = True           # 힘 제어(눌러주기) 사용 여부
ENABLE_PERIODIC = True        # 4번 작업의 손목 흔들기(move_periodic) 사용 여부
ENABLE_GRIPPER = True         # 그리퍼 열기/닫기(디지털 출력) 사용 여부
CHECK_PICK_FORCE = True        # 집을 때 힘이 정상 범위인지 검사할지 여부
LOG_TOOL_FORCE = True          # 누르는 단계마다 실제 측정된 힘을 로그로 출력 (범위 튜닝용)
PICK_FORCE_RATIO = (0.7, 1.2)  # 집을 때 정상으로 보는 Z축 힘 범위 = |목표 힘| x 비율 [N]
                               # 예) -50N → 35~60N, -70N → 49~84N (원본 DART 는 고정 10~15)
                               # -50N 실측 45.6N 기준. LOG_TOOL_FORCE 로그를 보고 조정할 것
REPEAT_COUNT = 1               # 전체 사이클 반복 횟수

# =============================================================================
# 좌표 / 파라미터
# =============================================================================
# 홈 자세 (관절 각도, 단위: 도)
HOME = [0.00, 0.01, 90.01, 0.00, 90.00, 0.00]

# 힘 제어할 때 쓰는 강성값 [X, Y, Z (N/m), Rx, Ry, Rz (Nm/rad)]
# 값이 작을수록 로봇이 외력에 부드럽게 밀린다.
STIFFNESS = [3000.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]

# 1~3번 작업: (집는 위치, 집은 뒤 빠지는 위치, 놓는 위치)
# 원본 DART 의 관절 각도 그대로 (단위: 도)
JOINT_TASKS = [
    # 1번 기어
    ([-6.45, 13.78, 92.47, -0.09, 74.52, 0.00],        # pick
     [-6.45, 13.78, 92.47, -0.09, 74.52, 0.00],        # retreat (pick 과 같음)
     [-21.35, -14.73, 112.64, -1.04, 82.18, 0.15]),    # place
    # 2번 기어
    ([-4.03, 28.63, 71.93, -0.07, 80.02, 2.23],        # pick
     [-3.97, 27.60, 58.07, -0.07, 94.76, 2.24],        # retreat
     [-23.88, 1.25, 100.42, -0.18, 78.31, -23.37]),    # place
    # 3번 기어
    ([-15.27, 23.73, 79.40, -0.12, 77.53, -8.73],      # pick
     [-15.27, 23.73, 79.40, -0.12, 77.53, -8.73],      # retreat (pick 과 같음)
     [-39.13, -3.74, 112.32, -0.51, 72.26, -32.48]),   # place
]

# 4번 작업: (집는 위치, 빠지는 위치, 놓는 위치, 놓은 뒤 위치)
# 원본 DART 의 직교 좌표 [x, y, z (mm), rx, ry, rz (도)], BASE 기준.
# 이 좌표는 TCP_NAME 기준이므로 TCP 가 다르면 실제 위치가 달라진다.
CART_TASK = (
    [500.65, -76.26, 130.00, 176.68, 179.11, -176.23],   # pick
    [499.95, -76.48, 130.00, 176.51, 178.95, -176.39],   # retreat
    [277.89, -152.46, 150.00, 169.31, 178.82, 176.40],   # place
    [277.89, -152.46, 98.57, 169.31, 178.82, 176.40],    # post-place (Z 가 더 낮음, 아래 경고 참고)
)


def main(args=None):
    rclpy.init(args=args)
    node = None
    robot = None
    # 힘 제어 / 컴플라이언스가 켜져 있는지 추적하는 변수.
    # 에러로 중간에 멈춰도 finally 에서 확실히 해제하기 위해 사용한다.
    force_active = False
    compliance_active = False
    try:
        # DSR_ROBOT2 는 import 되는 순간 DR_init 의 값을 읽어서
        # 서비스 클라이언트를 만든다. 따라서 아래 값을 먼저 설정해야 한다.
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        node = rclpy.create_node('m0609_pick_3', namespace=ROBOT_ID)
        DR_init.__dsr__node = node

        # 반드시 __dsr__node 를 설정한 '뒤에' import 할 것
        import DSR_ROBOT2 as robot
        from DSR_ROBOT2 import posj, posx

        # ---------------------------------------------------------------------
        # 공통 헬퍼
        # ---------------------------------------------------------------------
        def log(msg):
            """ROS 로거로 정보 메시지 출력."""
            node.get_logger().info(msg)

        def command(name, fn, *args, **kwargs):
            """로봇 명령을 실행하는 공통 함수.

            - DRY_RUN 이면 실제로 보내지 않고 로그만 남긴다.
            - DSR_ROBOT2 함수는 실패하면 음수(-1)를 돌려주므로,
              음수가 나오면 예외를 발생시켜 프로그램을 멈춘다.
            """
            if DRY_RUN:
                log(f'[DRY RUN] {name}: args={args}, kwargs={kwargs}')
                return None
            result = fn(*args, **kwargs)
            if isinstance(result, (int, float)) and not isinstance(result, bool) and result < 0:
                raise RuntimeError(f'{name} failed: {result}')
            return result

        def wait_for_services():
            """사용할 ROS 2 서비스가 모두 준비될 때까지 기다린다.

            서비스 디스커버리가 끝나기 전에 요청을 보내면
            응답을 영원히 기다리며 멈출 수 있으므로, 시작 전에 미리 확인한다.
            켜져 있는 기능(힘 제어, 손목 흔들기)의 서비스만 추가로 기다린다.
            """
            clients = [
                robot._ros2_set_robot_mode,
                robot._ros2_set_current_tool,
                robot._ros2_set_current_tcp,
                robot._ros2_movej,
                robot._ros2_movel,
                robot._ros2_set_digital_output,
            ]
            if ENABLE_FORCE:
                clients += [
                    robot._ros2_set_ref_coord,
                    robot._ros2_task_compliance_ctrl,
                    robot._ros2_set_desired_force,
                    robot._ros2_check_force_condition,
                    robot._ros2_release_force,
                    robot._ros2_release_compliance_ctrl,
                    robot._ros2_get_tool_force,
                ]
            if ENABLE_PERIODIC:
                clients.append(robot._ros2_move_periodic)
            for cli in clients:
                while not cli.wait_for_service(timeout_sec=1.0):
                    log(f'Waiting for service {cli.srv_name} ...')

        # ---------------------------------------------------------------------
        # 이동
        # ---------------------------------------------------------------------
        def move_joint(p):
            """관절 이동. 속도 60 deg/s, 가속도 100 deg/s^2."""
            command('movej', robot.movej, posj(*p), vel=60.0, acc=100.0)

        def move_linear(p):
            """직선 이동 (BASE 기준 절대 좌표).

            vel = [병진 속도 mm/s, 회전 속도 deg/s]
            acc = [병진 가속도 mm/s^2, 회전 가속도 deg/s^2]
            """
            command('movel', robot.movel, posx(*p), vel=[250.0, 80.625],
                    acc=[1000.0, 322.5], ref=robot.DR_BASE,
                    mod=robot.DR_MV_MOD_ABS)

        # ---------------------------------------------------------------------
        # 그리퍼 (컨트롤 박스 디지털 출력 1, 2번)
        #   닫기: DO1=ON,  DO2=OFF
        #   열기: DO1=OFF, DO2=ON
        # 출력 후 그리퍼가 완전히 동작할 때까지 1초 기다린다.
        # ---------------------------------------------------------------------
        def gripper_close():
            if not ENABLE_GRIPPER:
                log('Gripper CLOSE skipped (ENABLE_GRIPPER=False)')
                return
            command('digital output 1 ON', robot.set_digital_output, 1, 1)
            command('digital output 2 OFF', robot.set_digital_output, 2, 0)
            command('wait', robot.wait, 1.0)

        def gripper_open():
            if not ENABLE_GRIPPER:
                log('Gripper OPEN skipped (ENABLE_GRIPPER=False)')
                return
            command('digital output 1 OFF', robot.set_digital_output, 1, 0)
            command('digital output 2 ON', robot.set_digital_output, 2, 1)
            command('wait', robot.wait, 1.0)

        # ---------------------------------------------------------------------
        # 힘 제어
        # ---------------------------------------------------------------------
        def apply_force(target_n, hold_sec, check=False, grip_while_pressing=False):
            """Z축 방향으로 target_n [N] 의 힘으로 hold_sec 초 동안 누른다.

            순서:
              1) 컴플라이언스 제어 시작 (로봇을 스프링처럼 부드럽게 만듦)
              2) 강성값 설정
              3) Z축으로 목표 힘 인가 → 로봇이 부품에 닿을 때까지 내려감
              3-1) (grip_while_pressing=True 이면) 내려가는 동안 그리퍼 열기(1초) → 닫기(1초)
                   원본 DART 는 힘 인가 직후 check_force_condition 이 항상 실패해서
                   else 분기(oo → cc)로 '내려가면서 열고 → 닫아 집는' 동작을 했다.
              4) hold_sec 동안 유지
              5) (check=True 이면) 측정 힘이 |target_n| x PICK_FORCE_RATIO 범위 안인지 검사
              6) 힘 제어 해제 → 컴플라이언스 해제 (예외가 나도 반드시 실행)

            target_n 이 음수이면 아래(-Z) 방향으로 누른다.
            """
            nonlocal force_active, compliance_active
            if DRY_RUN or not ENABLE_FORCE:
                log(f'Force stage {target_n}N SKIPPED (ENABLE_FORCE=False)')
                return
            # 원본 DART 코드는 기준 좌표계를 따로 정하지 않았으므로 기본값인 BASE 를 사용.
            # 누르는 방향이 BASE -Z 인지 TOOL -Z 인지 실제 로봇에서 확인할 것.
            robot.set_ref_coord(robot.DR_BASE)
            result = robot.task_compliance_ctrl()
            if result != 0:
                raise RuntimeError(f'task_compliance_ctrl failed: {result}')
            compliance_active = True
            try:
                robot.set_stiffnessx(STIFFNESS, time=0.0)
                robot.set_desired_force(
                    [0.0, 0.0, target_n, 0.0, 0.0, 0.0],  # 목표 힘 [Fx, Fy, Fz, Mx, My, Mz]
                    [0, 0, 1, 0, 0, 0],                     # 힘 제어할 축 선택 (Z축만)
                    time=0.0,
                    mod=robot.DR_FC_MOD_REL,
                )
                force_active = True
                if grip_while_pressing:
                    # 내려가는 중에 그리퍼를 벌려 부품을 감싼 뒤 닫아서 집는다 (원본 DART 순서)
                    gripper_open()
                    gripper_close()
                # 힘을 건 직후에는 아직 부품에 닿지 않았을 수 있으므로
                # 유지 시간이 끝난 뒤에 검사한다.
                robot.wait(hold_sec)
                if LOG_TOOL_FORCE:
                    # PICK_FORCE_RATIO 를 정하기 위한 실측값 출력
                    log(f'Tool force @ {target_n}N stage: {robot.get_tool_force(robot.DR_TOOL)}')
                if check and CHECK_PICK_FORCE:
                    # 목표 힘 크기에 비례해서 범위를 정한다 (누르는 힘을 바꿔도 따라감)
                    fmin = round(abs(target_n) * PICK_FORCE_RATIO[0], 1)
                    fmax = round(abs(target_n) * PICK_FORCE_RATIO[1], 1)
                    in_range = robot.check_force_condition(
                        axis=robot.DR_AXIS_Z, min=fmin, max=fmax, ref=robot.DR_TOOL
                    )
                    # [중요] DSR_ROBOT2(ROS 2) 의 check_force_condition 은
                    #        조건 만족 시 0, 불만족 시 -1 을 돌려준다.
                    #        (DRL 의 True/False 와 반대이므로 'if 결과:' 로 쓰면 안 됨)
                    if in_range != 0:
                        # 부품을 제대로 못 잡았다고 보고 사이클을 중단한다.
                        raise RuntimeError(
                            f'Pick force check failed ({fmin}~{fmax}N): {in_range}; stopping cycle'
                        )
            finally:
                # 정상이든 예외든 힘 제어 → 컴플라이언스 순서로 반드시 해제
                if force_active:
                    robot.release_force(time=0.0)
                    force_active = False
                if compliance_active:
                    robot.release_compliance_ctrl()
                    compliance_active = False

        # ---------------------------------------------------------------------
        # 작업 시퀀스
        # ---------------------------------------------------------------------
        def pick_place_joint(index, pick, retreat, place):
            """1~3번 작업: 관절 좌표로 pick & place."""
            log(f'Pick & Place {index} start')
            gripper_close()                     # 원본 DART 순서 그대로: 접근 전에 닫기
            move_joint(pick)                    # 집는 위치 위로 이동
            # 50N 으로 누르면서 내려가는 중에 열기→닫기로 집고, 2초 유지 후 힘 검사
            apply_force(-50.0, 2.0, check=True, grip_while_pressing=True)
            move_joint(retreat)                 # 위로 빠지기
            move_joint(place)                   # 놓는 위치로 이동
            apply_force(-80.0, 5.0)             # 80N 으로 5초 눌러 끼우기
            gripper_open()                      # 그리퍼 열어서 놓기
            move_joint(place)                   # 놓는 위치(접근 자세)로 복귀
            log(f'Pick & Place {index} complete')

        def pick_place4():
            """4번 작업: 직교 좌표로 pick & place + 손목 흔들어 끼우기."""
            log('Pick & Place 4 start')
            pick, retreat, place, post_place = CART_TASK
            gripper_close()
            move_linear(pick)
            apply_force(-70.0, 2.0, check=True, grip_while_pressing=True)
            move_linear(retreat)
            move_linear(place)
            apply_force(-80.0, 5.0)
            if ENABLE_PERIODIC and not DRY_RUN:
                # TOOL 기준 Rz 축으로 ±50도, 주기 5초로 3번 흔들기
                # (기어 이빨이 맞물리도록 돌려서 끼워 넣는 동작)
                robot.move_periodic(
                    amp=[0.0, 0.0, 0.0, 0.0, 0.0, 50.0],
                    period=[0.0, 0.0, 0.0, 0.0, 0.0, 5.0],
                    atime=0.0, repeat=3, ref=robot.DR_TOOL,
                )
            else:
                log('Periodic wrist motion SKIPPED (ENABLE_PERIODIC=False or DRY_RUN)')
            gripper_open()
            # [경고] 원본 코드는 그리퍼를 연 '뒤에' Z 150mm → 98.57mm 로 '내려간다'.
            #        의도한 동작인지 확인할 것 (보통은 놓은 뒤 위로 빠짐).
            move_linear(post_place)
            log('Pick & Place 4 complete')

        # ---------------------------------------------------------------------
        # 초기 설정
        # ---------------------------------------------------------------------
        if DRY_RUN:
            log('DRY_RUN=True: no motion, I/O or mode changes will be sent')
            log(f'[DRY RUN] set_tool({TOOL_NAME!r}), set_tcp({TCP_NAME!r})')
        else:
            wait_for_services()
            # Tool / TCP 는 MANUAL 모드에서만 바꿀 수 있다.
            # 먼저 MANUAL 에서 설정한 뒤, 동작을 위해 AUTONOMOUS 로 전환한다.
            #   - Tool : 공구 무게 → 힘/토크 센서 보정에 사용
            #   - TCP  : 공구 끝점 → movel(posx) 좌표의 기준점
            command('set_robot_mode MANUAL', robot.set_robot_mode, robot.ROBOT_MODE_MANUAL)
            command('set_tool', robot.set_tool, TOOL_NAME)
            command('set_tcp', robot.set_tcp, TCP_NAME)
            command('set_robot_mode AUTONOMOUS', robot.set_robot_mode, robot.ROBOT_MODE_AUTONOMOUS)
            # 특이점 근처에서는 자동으로 회피하도록 설정
            robot.set_singularity_handling(robot.DR_AVOID)
            # 전역 기본 속도/가속도 (원본 DART 값)
            robot.set_velj(60.0)               # 관절 속도 [deg/s]
            robot.set_accj(100.0)              # 관절 가속도 [deg/s^2]
            robot.set_velx(250.0, 80.625)      # 직선 속도 [mm/s, deg/s]
            robot.set_accx(1000.0, 322.5)      # 직선 가속도 [mm/s^2, deg/s^2]

        # ---------------------------------------------------------------------
        # 메인 루프: 홈 → 1~3번(관절) → 4번(직교) → 홈
        # ---------------------------------------------------------------------
        for cycle in range(REPEAT_COUNT):
            log(f'Cycle {cycle + 1}/{REPEAT_COUNT}')
            move_joint(HOME)
            for index, poses in enumerate(JOINT_TASKS, start=1):
                pick_place_joint(index, *poses)
            pick_place4()
            move_joint(HOME)
        log('All cycles complete')

    except Exception as exc:
        # 어떤 에러든 로그를 남기고 다시 던져서 비정상 종료 코드로 끝낸다.
        if node is not None:
            node.get_logger().error(f'Program aborted: {exc}')
        raise
    finally:
        # 힘 제어가 켜진 채로 프로그램이 끝나지 않도록 마지막으로 한 번 더 해제한다.
        # (해제 중 에러는 무시: 이미 종료 중이므로 원래 에러를 덮지 않기 위함)
        if robot is not None:
            if force_active:
                try:
                    robot.release_force(time=0.0)
                except Exception:
                    pass
            if compliance_active:
                try:
                    robot.release_compliance_ctrl()
                except Exception:
                    pass
        # ROS 2 노드 정리
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
