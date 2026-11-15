"""ToolTask 액션 테스트 클라이언트

나중에는 DB에서 읽은 시작/종료 위치를 goal 로 보내면 됩니다.
지금은 아래 기본값 또는 ROS 파라미터로 테스트합니다.

사용법:
  ros2 run sanding tool_task_client
  ros2 run sanding tool_task_client --ros-args -p tool_id:=1 \
      -p start_posx:="[400.0, 0.0, 200.0, 90.0, 180.0, 0.0]" \
      -p end_posx:="[500.0, 0.0, 200.0, 90.0, 180.0, 0.0]" \
      -p return_tool:=true

  Ctrl+C 를 누르면 실행 중인 goal 에 취소 요청을 보냅니다.
"""
import rclpy
from rclpy.action import ActionClient

from rokey_interfaces.action import ToolTask

ROBOT_ID = "dsr01"

# 테스트용 기본값 (도구 1 위치 기준)
#   도구 1  : [364.96, -234.31, 92.91, 61.79, -177.53, 61.44]
#   시작    : 도구 위치 + y 50
#   종료    : 시작 위치 + x 100
DEFAULT_START = [364.96, -184.31, 92.91, 61.79, -177.53, 61.44]
DEFAULT_END = [464.96, -184.31, 92.91, 61.79, -177.53, 61.44]


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("sanding_tool_task_client", namespace=ROBOT_ID)
    logger = node.get_logger()

    tool_id = node.declare_parameter("tool_id", 1).value
    start = node.declare_parameter("start_posx", DEFAULT_START).value
    end = node.declare_parameter("end_posx", DEFAULT_END).value
    return_tool = node.declare_parameter("return_tool", False).value

    client = ActionClient(node, ToolTask, "tool_task")
    if not client.wait_for_server(timeout_sec=5.0):
        logger.error("tool_task 액션 서버가 없습니다. tool_task_server 를 먼저 실행하세요.")
        node.destroy_node()
        rclpy.shutdown()
        return

    goal = ToolTask.Goal()
    goal.tool_id = int(tool_id)
    goal.start_posx = [float(v) for v in start]
    goal.end_posx = [float(v) for v in end]
    goal.return_tool = bool(return_tool)

    def on_feedback(msg):
        fb = msg.feedback
        logger.info(f"[{fb.step}/{fb.total_steps}] {fb.stage}")

    send_future = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send_future)
    goal_handle = send_future.result()
    if not goal_handle.accepted:
        logger.error("goal 이 거절되었습니다. (도구 번호 / 위치 값 / 작업 중 여부 확인)")
        node.destroy_node()
        rclpy.shutdown()
        return

    logger.info("goal 수락됨")
    result_future = goal_handle.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        logger.warn("취소 요청 전송")
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel_future)
        rclpy.spin_until_future_complete(node, result_future)

    result = result_future.result().result
    logger.info(f"결과: success={result.success}, message={result.message}")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
