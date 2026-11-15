import rclpy
import DR_init

from utils.service_waiter import ServiceWaiter
from utils.shutdown import safe_shutdown

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30
CYCLE = 3

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_move", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
        )

        from DR_common2 import posx, posj # posx, posj  포인트 좌표계

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 서비스 디스커버리가 끝나기 전에 요청을 보내면 응답이 오지 않아 무한 대기하므로
    # 사용할 서비스가 준비될 때까지 기다린다
    import DSR_ROBOT2
    ServiceWaiter(node).wait(
        DSR_ROBOT2._ros2_set_current_tool,
        DSR_ROBOT2._ros2_set_current_tcp,
        DSR_ROBOT2._ros2_movej,
        DSR_ROBOT2._ros2_movel,
    )

    # 설정 권장 : 도구,무게
    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    # 포즈 j,x 구별하기 쉽게 
    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0]) 
    posj1 = posj([0.0, 0.0, 90.0, 0.0, 30.0, 0.0]) 

    posx1 = posx([350.0, 34.5, 350.0, 45.0, 180.0, 45.0]) 
    posx2 = posx([350.0, 34.5, 300.0, 45.0, 180.0, 45.0])

    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        for i in range(CYCLE):
            node.get_logger().info(f"Cycle {i+1}")

            node.get_logger().info(f"Moving to joint position: {posj1}")
            movej(posj1, vel=VELOCITY, acc=ACC)

            node.get_logger().info(f"Moving to task position: {posx1}")
            movel(posx1, vel=VELOCITY, acc=ACC)

            node.get_logger().info(f"Moving to task position: {posx2}")
            movel(posx2, vel=VELOCITY, acc=ACC)

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")

    finally:
        safe_shutdown(node, ("Failed to move home",
                             lambda: movej(homej, vel=VELOCITY, acc=ACC)))


if __name__ == "__main__":
    main()
