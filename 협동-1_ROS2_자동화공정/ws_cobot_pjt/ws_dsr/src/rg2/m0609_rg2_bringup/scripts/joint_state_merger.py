#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):

    def __init__(self):

        super().__init__("joint_state_merger")

        self.robot_state = None
        self.gripper_state = None

        self.declare_parameter("robot_joint_topic", "/dsr01/robot_joint_states")
        robot_topic = self.get_parameter("robot_joint_topic").value

        self.create_subscription(JointState, robot_topic, self.robot_callback, 10)

        self.create_subscription(
            JointState, "gripper_joint_states", self.gripper_callback, 10
        )

        self.publisher = self.create_publisher(JointState, "joint_states", 10)

    def robot_callback(self, msg):
        self.robot_state = msg
        self.publish_joint_state()

    def gripper_callback(self, msg):
        self.gripper_state = msg

    def publish_joint_state(self):

        if self.robot_state is None:
            return

        out = JointState()

        out.header = self.robot_state.header

        out.name = list(self.robot_state.name)
        out.position = list(self.robot_state.position)

        # velocity
        if len(self.robot_state.velocity):
            out.velocity = list(self.robot_state.velocity)
        else:
            out.velocity = []

        # effort
        if len(self.robot_state.effort):
            out.effort = list(self.robot_state.effort)
        else:
            out.effort = []

        if self.gripper_state is not None:

            out.name.extend(self.gripper_state.name)
            out.position.extend(self.gripper_state.position)

            # velocity 길이 맞추기
            if out.velocity:
                if len(self.gripper_state.velocity):
                    out.velocity.extend(self.gripper_state.velocity)
                else:
                    out.velocity.extend([0.0] * len(self.gripper_state.name))

            # effort 길이 맞추기
            if out.effort:
                if len(self.gripper_state.effort):
                    out.effort.extend(self.gripper_state.effort)
                else:
                    out.effort.extend([0.0] * len(self.gripper_state.name))
        self.publisher.publish(out)


def main():

    rclpy.init()

    node = JointStateMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
