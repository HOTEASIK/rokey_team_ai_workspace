import time

from utils.aspects import dsr_main, timed
from utils.service_waiter import ServiceWaiter

VELOCITY, ACC = 30, 30

ON, OFF = 1, 0
CYCLE = 3


@dsr_main("rokey_grip_simple")
def main(app):
    node = app.node

    import DSR_ROBOT2
    from DSR_ROBOT2 import (
        set_digital_output,
        set_tool,
        set_tcp,
        movej,
        wait,
    )
    from DR_common2 import posj

    # 서비스 디스커버리가 끝나기 전에 요청을 보내면 무한 대기하므로
    # 사용할 서비스가 준비될 때까지 기다린다
    ServiceWaiter(node).wait(
        DSR_ROBOT2._ros2_set_current_tool,
        DSR_ROBOT2._ros2_set_current_tcp,
        DSR_ROBOT2._ros2_movej,
        DSR_ROBOT2._ros2_set_digital_output,
    )

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    @timed(app.logger, "Grip")
    def grip():
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        time.sleep(1)

    @timed(app.logger, "Release")
    def release():
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)

    homej = posj([0, 0, 90, 0, 90, 0])
    app.on_shutdown("Failed to move home", lambda: movej(homej, vel=VELOCITY, acc=ACC))

    node.get_logger().info(f"Moving to joint position: {homej}")
    movej(homej, vel=VELOCITY, acc=ACC)

    for i in range(CYCLE):
        node.get_logger().info(f"=== Cycle {i+1}/{CYCLE} ===")
        grip()
        release()
        wait(1)

    node.get_logger().info("Gripper Test Complete")


if __name__ == "__main__":
    main()
