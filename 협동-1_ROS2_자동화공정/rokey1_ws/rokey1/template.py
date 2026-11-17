import rclpy
import DR_init
from rokey1.service_waiter import ServiceWaiter

# ===== 로봇 설정 =====
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30   # 속도, 가속도
CYCLE = 1                # 반복 횟수

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("node_name", namespace=ROBOT_ID)   # TODO: 노드 이름

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            # TODO: 사용할 함수 추가
        )

        from DR_common2 import posx, posj

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # ===== 서비스 준비될 때까지 대기 =====
    # 사용하는 함수의 서비스 클라이언트를 적는다
    #   set_tool -> _ros2_set_current_tool,  set_tcp -> _ros2_set_current_tcp
    #   movej    -> _ros2_movej,             movel   -> _ros2_movel
    import DSR_ROBOT2
    ServiceWaiter(node).wait(
        DSR_ROBOT2._ros2_set_current_tool,
        DSR_ROBOT2._ros2_set_current_tcp,
        DSR_ROBOT2._ros2_movej,
        DSR_ROBOT2._ros2_movel,
        # TODO: 추가로 사용하는 함수의 서비스 클라이언트
    )

    # ===== 도구 / TCP 설정 =====
    set_tool("")   # TODO: 도구 무게 이름
    set_tcp("")    # TODO: TCP 이름

    # ===== 포즈 정의 =====
    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    # TODO: posj1 = posj([J1, J2, J3, J4, J5, J6])
    # TODO: posx1 = posx([X, Y, Z, A, B, C])

    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        for i in range(CYCLE):
            node.get_logger().info(f"Cycle {i+1}")

            # ===== 동작 작성 =====
            # TODO: movej(posj1, vel=VELOCITY, acc=ACC)
            # TODO: movel(posx1, vel=VELOCITY, acc=ACC)
            pass

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")

    finally:
        movej(homej, vel=VELOCITY, acc=ACC)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
