import rclpy
import DR_init
import time

ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0
CYCLE = 3

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_grip_simple", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_digital_output,
            set_tool,
            set_tcp,
            movej,
            wait,
        )
        from DR_common2 import posj
    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    # 서비스 디스커버리가 끝나기 전에 요청을 보내면 무한 대기하므로
    # 사용할 서비스가 준비될 때까지 기다린다
    import DSR_ROBOT2
    for cli in (DSR_ROBOT2._ros2_set_current_tool,
                DSR_ROBOT2._ros2_set_current_tcp,
                DSR_ROBOT2._ros2_movej,
                DSR_ROBOT2._ros2_set_digital_output):
        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f"Waiting for service {cli.srv_name} ...")

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    def grip():
        start_time = time.perf_counter()
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        time.sleep(1)
        elapsed = time.perf_counter() - start_time
        node.get_logger().info(f"[Timing] Grip elapsed time: {elapsed:.4f} sec")

    def release():
        start_time = time.perf_counter()
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1)
        elapsed = time.perf_counter() - start_time
        node.get_logger().info(f"[Timing] Release elapsed time: {elapsed:.4f} sec")
     
    homej = posj([0, 0, 90, 0, 90, 0])

    node.get_logger().info(f"Moving to joint position: {homej}")
    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        for i in range(CYCLE):
            node.get_logger().info(f"=== Cycle {i+1}/{CYCLE} ===")
            grip()
            release()
            wait(1)

        node.get_logger().info("Gripper Test Complete")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except TimeoutError as e:
        node.get_logger().error(str(e))

    finally:
        try:
            movej(homej, vel=VELOCITY, acc=ACC)
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()