#!/usr/bin/env python3
# rg2_width_sender.py

import rclpy
from onrobot_rg_msgs.srv import SetCommand
from rclpy.node import Node


class GripperWidthController(Node):
    def __init__(self):
        super().__init__("gripper_width_controller")

        # /onrobot/sendCommand 서비스 클라이언트
        self.client = self.create_client(SetCommand, "/onrobot/sendCommand")
        self.get_logger().info("Waiting for /onrobot/sendCommand service...")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /onrobot/sendCommand...")

        self.get_logger().info("Connected to /onrobot/sendCommand")

    def send_width(self, width):
        """그리퍼 width를 전송"""

        request = SetCommand.Request()

        # 서버의 genCommand()가 숫자로 변환하므로
        # 문자열로 전달
        request.command = str(width)

        self.get_logger().info(f"Sending width = {width}")

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error("Service call failed")
            return False

        response = future.result()

        if response.success:
            self.get_logger().info(f"Gripper width command sent: {width}")
            return True

        self.get_logger().error(f"Gripper command failed: {response.message}")

        return False


def main(args=None):

    rclpy.init(args=args)

    node = GripperWidthController()

    try:
        while rclpy.ok():
            user_input = input("\nEnter gripper width (0~1600, q to quit): ")

            if user_input.lower() == "q":
                break

            try:
                width = int(user_input)
            except ValueError:
                print("Please enter an integer.")
                continue

            if width < 0 or width > 1100:
                print("Width must be between 0 and 1600.")
                continue

            node.send_width(width)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
