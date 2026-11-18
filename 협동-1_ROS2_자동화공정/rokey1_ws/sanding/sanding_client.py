"""샌딩 액션 테스트 클라이언트 (/dsr01/tool_task) - HMI 의 [샌딩 작업 실행] 과 같은 goal 을 보냄

HMI 없이 터미널에서 시험할 때 사용합니다. (HMI 를 쓰면 이 파일은 필요 없음)

기본 (파라미터 없이 실행) = 서버의 기본 작업:
  도구 1 [347.35, -16.43, 90.10, 14.00, 178.37, 12.66] 집기 →
  시작 관절각 [-8.21, 31.18, 114.93, -10.05, -50.25, 100.44] 의 TCP 에서 베이스 +Y 10cm 작업
  (start_posx 를 비워서(전부 0) 보내면 서버가 기본 작업을 함)

사용:
  ros2 run rokey1 sanding_client                 # 기본 작업
  # 다른 위치에서 작업할 때
  ros2 run rokey1 sanding_client --ros-args \
      -p tool_id:=1 \
      -p start_posx:="[738.56, -56.3, 145.1, 0.0, 90.0, 94.84]" \
      -p end_posx:="[738.56, 43.7, 145.1, 0.0, 90.0, 94.84]" \
      -p return_tool:=true

  Ctrl+C: 실행 중인 goal 취소 (이동 중이면 즉시 정지, 그 자리에서 멈춤)
"""
import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions

# 팀 레포(rokey1)로 옮길 때는 rokey_interfaces 로 바꾸면 됩니다
from rokey_interfaces.action import ToolTask

from sanding.sanding_job import ROBOT_ID
from sanding.sanding_server import ACTION_NAME

DEFAULT_TOOL_ID = 1
DEFAULT_START_POSX = [0.0] * 6   # 전부 0 = 서버 기본 작업 (sanding_server.py 의 DEFAULT_START_POSJ / WORK_VECTOR)
DEFAULT_END_POSX = [0.0] * 6


def main(args=None):
    # Ctrl+C 때 rclpy 가 먼저 종료되면 취소 요청을 못 보내므로 기본 핸들러 끔
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("rokey1_sanding_client", namespace=ROBOT_ID)
    log = node.get_logger()
    p = node.declare_parameter
    goal = ToolTask.Goal()
    goal.tool_id = int(p("tool_id", DEFAULT_TOOL_ID).value)
    goal.start_posx = [float(v) for v in p("start_posx", DEFAULT_START_POSX).value]
    goal.end_posx = [float(v) for v in p("end_posx", DEFAULT_END_POSX).value]
    goal.return_tool = bool(p("return_tool", True).value)
    # surface_normal 은 비워 둠 → 서버가 시작 자세의 툴 Z 로 벽 방향 자동 판단 (HMI 와 같음)

    client = ActionClient(node, ToolTask, ACTION_NAME)
    if not client.wait_for_server(timeout_sec=5.0):
        log.error(f"{ACTION_NAME} 액션 서버가 없습니다. sanding_server 를 먼저 실행하세요.")
        node.destroy_node()
        rclpy.shutdown()
        return

    def on_feedback(msg):
        fb = msg.feedback
        log.info(f"[{fb.step}/{fb.total_steps}] {fb.stage}")

    if not any(goal.start_posx):
        log.info(f"goal 전송: tool={goal.tool_id}, 기본 작업 (서버의 시작 관절각 + 10cm)")
    else:
        log.info(f"goal 전송: tool={goal.tool_id}, start={list(goal.start_posx)}, end={list(goal.end_posx)}")
    send = client.send_goal_async(goal, feedback_callback=on_feedback)
    rclpy.spin_until_future_complete(node, send)
    gh = send.result()
    if not gh.accepted:
        log.error("goal 거절됨 (도구 번호 / 값 / 작업 중 여부 확인)")
        node.destroy_node()
        rclpy.shutdown()
        return

    result_future = gh.get_result_async()
    try:
        rclpy.spin_until_future_complete(node, result_future)
    except KeyboardInterrupt:
        log.warn("취소 요청 전송")
        cancel = gh.cancel_goal_async()
        rclpy.spin_until_future_complete(node, cancel)
        rclpy.spin_until_future_complete(node, result_future)

    res = result_future.result().result
    (log.info if res.success else log.error)(f"결과: success={res.success}, {res.message}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
