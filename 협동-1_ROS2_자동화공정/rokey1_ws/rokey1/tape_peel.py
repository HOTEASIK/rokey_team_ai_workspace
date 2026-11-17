#!/usr/bin/env python3
"""비전 없이 접촉(힘)만으로 테이프 롤의 끝단을 찾아 떼어내는 노드.

[방식]
  그리퍼를 롤 접선 방향으로 눕히고 손가락을 벌린 채 롤 윗부분 원호를 따라 진행한다.
  들뜬 끝단 밑으로 아래 손가락이 들어가고, 끝단 뿌리에 닿으면 Tool Z 반력이 커진다.
  → 정지 → 그리퍼 닫기 → 반대 방향으로 당겨 떼어낸다.

[티칭 (TCP = TCP_NAME 기준, BASE 좌표)]
  P_START / P_MID / P_END : 롤 윗부분 원호 위 3점.
    - 그리퍼 Tool Z 가 진행 방향(접선)을 향하게
    - 손가락 벌림 방향이 롤 반지름 방향이 되게 (아래 손가락 끝이 표면에 거의 닿게)
    - START → END 방향은 '끝단이 들려 있는 쪽으로' 진행 (끝단 밑으로 파고드는 방향)
  끝단은 사람이 롤을 거치할 때 START~END 구간 안에 오게 둔다.

[튜닝]
  처음엔 FORCE_THRESHOLD 를 크게(예: 999) 두고 돌려서 로그의 dF 값을 보고 정한다.

[실행]
  ros2 run rokey1 tape_peel
"""
import rclpy
import DR_init

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
TOOL_NAME = 'Tool Weight'
TCP_NAME = 'GripperDA_v1'

HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# TODO: 티칭값으로 교체 [x, y, z, rx, ry, rz]
P_START = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P_MID = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
P_END = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

APPROACH_OFFSET = 50.0    # P_START 위쪽(BASE +Z) 접근 높이 [mm]
SCAN_VEL = [10.0, 10.0]   # 탐색 속도 [mm/s, deg/s]
FORCE_THRESHOLD = 5.0     # Tool Z 반력 변화량 임계값 [N] (실측 후 조정)
PULL_DIST = 80.0          # 떼어낼 때 Tool -Z 로 당기는 거리 [mm]
PULL_VEL = [20.0, 10.0]


def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('tape_peel', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    import DSR_ROBOT2 as robot
    from DSR_ROBOT2 import posj, posx
    from dsr_msgs2.srv import MoveStop

    log = node.get_logger().info
    stop_cli = node.create_client(MoveStop, 'dsr_controller2/motion/move_stop')

    def gripper(close):
        robot.set_digital_output(1, 1 if close else 0)
        robot.set_digital_output(2, 0 if close else 1)
        robot.wait(1.0)

    def quick_stop():
        req = MoveStop.Request()
        req.stop_mode = robot.DR_QSTOP
        rclpy.spin_until_future_complete(node, stop_cli.call_async(req))

    def tool_fz():
        return robot.get_tool_force(robot.DR_TOOL)[2]

    try:
        robot.set_robot_mode(robot.ROBOT_MODE_MANUAL)
        robot.set_tool(TOOL_NAME)
        robot.set_tcp(TCP_NAME)
        robot.set_robot_mode(robot.ROBOT_MODE_AUTONOMOUS)

        # 1) 접근
        robot.movej(posj(*HOME), vel=30, acc=60)
        gripper(close=False)
        approach = list(P_START)
        approach[2] += APPROACH_OFFSET
        robot.movel(posx(*approach), vel=[50.0, 30.0], acc=[100.0, 60.0])
        robot.movel(posx(*P_START), vel=SCAN_VEL, acc=[50.0, 30.0])
        robot.wait(0.5)

        # 2) 원호 탐색 + 반력 감시
        base = tool_fz()
        log(f'baseline Fz = {base:.2f} N')
        robot.amovec(posx(*P_MID), posx(*P_END), vel=SCAN_VEL, acc=[50.0, 30.0])
        found = False
        while robot.check_motion() != robot.DR_STATE_IDLE:
            dF = tool_fz() - base
            log(f'dF = {dF:+.2f} N')
            if abs(dF) > FORCE_THRESHOLD:
                quick_stop()
                found = True
                break

        if not found:
            log('끝단을 찾지 못함 → 후퇴')
            robot.movel(posx(*approach), vel=[50.0, 30.0], acc=[100.0, 60.0])
            return
        log(f'끝단 감지 @ {robot.get_current_posx()[0]}')

        # 3) 잡고 당기기 (진행 반대 방향 = Tool -Z)
        robot.wait(0.5)
        gripper(close=True)
        robot.movel(posx(0, 0, -PULL_DIST, 0, 0, 0), vel=PULL_VEL, acc=[50.0, 30.0],
                    ref=robot.DR_TOOL, mod=robot.DR_MV_MOD_REL)
        log('떼어내기 완료')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
