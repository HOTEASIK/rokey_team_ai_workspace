#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped

from moveit.planning import MoveItPy
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

from moveit.core.planning_scene import PlanningScene


def main():

    rclpy.init()

    # MoveIt 초기화
    moveit = MoveItPy(
        node_name="moveit_py_demo",
    )

    arm = moveit.get_planning_component("manipulator")

    # Planning Scene 가져오기
    planning_scene_monitor = moveit.get_planning_scene_monitor()

    # Collision Object 생성
    box = CollisionObject()

    box.header.frame_id = "base_link"
    box.id = "box1"

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.BOX
    primitive.dimensions = [0.3, 0.3, 0.3]

    box.primitives.append(primitive)
    box.primitive_poses.append(pose)
    box.operation = CollisionObject.ADD

    pose = Pose()
    pose.position.x = 0.45
    pose.position.y = 0.0
    pose.position.z = 0.15

    # Scene에 추가
    with planning_scene_monitor.read_write() as scene:
        scene.apply_collision_object(box)

    planning_scene_monitor.trigger_scene_update()

    # 목표 Pose
    goal = PoseStamped()
    goal.header.frame_id = "base_link"
    goal.pose.position.x = 0.60
    goal.pose.position.y = 0.00
    goal.pose.position.z = 0.40
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.707
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 0.707

    # Goal 설정
    arm.set_goal_state(pose_stamped_msg=goal, pose_link="tool0")

    # Planning
    plan_result = arm.plan()

    # Execute
    if plan_result:
        print("Planning Success")
        moveit.execute(plan_result.trajectory)

    else:
        print("Collision!! Planning Failed")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
