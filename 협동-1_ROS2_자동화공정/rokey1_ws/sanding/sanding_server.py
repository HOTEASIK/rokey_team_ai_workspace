"""샌딩 액션 서버 - 팀 HMI(hmi_server) 형식에 맞춤 (MoveIt 미사용, 두산 순응/힘제어)

액션: /dsr01/tool_task  (ToolTask) ← HMI 의 [샌딩 작업 실행] 이 보내는 이름/형식 그대로
  Goal    : tool_id, start_posx[6], end_posx[6], return_tool  (+ surface_normal[3], 선택)
  Result  : success, message
  Feedback: step, total_steps, stage   → HMI 진행률 / 로그에 표시

  start_posx : 작업 시작 TCP 위치/자세 (TCP 가 벽을 향하게 교시한 값)
  end_posx   : 작업 종료 TCP 위치 (자세는 무시 - 시작 자세로 작업)
  surface_normal : 비우면([0,0,0] 또는 필드 없음) 시작 자세의 툴 Z 로 벽 방향 자동 판단 → 90도 보정
  start_posx 를 비우면(전부 0) 기본 작업: 아래 TOOL_POSX[1] 도구 → DEFAULT_START_POSJ 에서 DEFAULT_WORK_VECTOR 만큼

동작은 sanding_job.py 의 SandingJob (계획 → 도구 집기 → 20cm 들기 → 관절로 자세 꺾기 → 수평 이동 → 벽 앞 위쪽 →
수직 하강 → 접촉 탐색 → 순응/힘제어 샌딩 → 후퇴 → 도구 반납 → 10cm 위)
  - goal 은 한 번에 하나만 (실행 중이면 거절)
  - 취소 (HMI STOP / E-STOP / 클라이언트 Ctrl+C): 이동 중이면 즉시 정지, 힘제어 해제, 그 자리에서 멈춤
    -p retreat_on_cancel:=true 면 샌딩 중 취소 시 벽에서 후퇴
  - 홈 자세: 현재 팔 형태로 도구에 못 가면 자동으로 홈 [0,0,90,0,90,0] 먼저. -p go_home:=true 면 항상 홈부터
  - 바닥 여유: 원형 도구 가장자리(TCP 아래 반지름)가 바닥 + 여유보다 낮게 주면 그 높이로 맞춤
    (-p floor_z:=35.0  -p floor_clear:=1.0  -p tool_radius:=45.0)

실행:
  (터미널 1) sod && sodreal
  (터미널 2) sod && ros2 run rokey1 sanding_server
  (터미널 3) HMI 의 [샌딩 작업 실행]  또는  sod && ros2 run rokey1 sanding_client
  ※ 예전 tool_task_server 와 동시에 띄우지 마세요
    (같은 /dsr01/tool_task 이름)
"""
import math
import threading

import rclpy
import DR_init
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rokey_interfaces.action import ToolTask

from sanding.sanding_job import (FLOOR_CLEAR_MM, PRESS_FORCE_MAX_N, PRESS_FORCE_MIN_N, PRESS_FORCE_N, ROBOT_ID, TOOL_RADIUS_MM, Abort, Canceled, PlanError,
                                  SandingJob)

ACTION_NAME = "tool_task"   # HMI 가 /dsr01/tool_task 로 보냄

# 도구 잡는 TCP 자세 [x, y, z (mm), rx, ry, rz (deg)] - 베이스 기준. 측정 전은 None
TOOL_POSX = {
    1: [347.35, -16.43, 90.10, 14.00, 178.37, 12.66],
    2: None,
    3: None,
}

# 기본 작업 - goal 의 start_posx / end_posx 를 비우면(전부 0) 이걸로 작업 (실제 테스트에서 잘 된 값)
DEFAULT_START_POSJ = [-8.21, 31.18, 114.93, -10.05, -50.25, 100.44]   # 시작 관절각 (deg) → fkin 으로 TCP
DEFAULT_WORK_VECTOR = [0.0, 100.0, 0.0]                                 # 베이스 +Y 10cm

# 벽 아래 바닥 높이 (베이스 z, mm). 원형 도구(반지름 45mm) 가장자리가 바닥 + 5mm 보다 낮아지면 작업을 위로 올림
#   측정값: 벽 아래 바닥이 베이스 기준 z≈35mm
#   (바닥 가까이 주면(이 값 + 50 아래) 전부 '바닥에 붙여서 작업' 으로 보고 도구 아래끝이 바닥 + 5mm 가 되게 맞춤)
#   -p floor_z:=<값> 으로 바꿀 수 있음. 아주 작은 값(예 -9999)이면 사실상 검사 안 함
FLOOR_Z_MM = 35.0


def goal_normal(goal):
    """goal 의 surface_normal (팀 ToolTask 에는 필드가 없음). 0 벡터/없음 → None (자동 판단)"""
    n = [float(v) for v in getattr(goal, "surface_normal", [])]   # numpy 배열로 올 수 있음
    return n if len(n) == 3 and any(abs(v) > 1e-9 for v in n) else None


def is_default(goal):
    """start_posx 가 비어 있으면(전부 0) 기본 작업"""
    return all(abs(float(v)) < 1e-9 for v in goal.start_posx)


class SandingServer:
    def __init__(self):
        # 로봇 제어 노드 (executor 에 넣지 않음 - DSR_ROBOT2 가 직접 spin)
        self.dsr_node = rclpy.create_node("sanding_dsr", namespace=ROBOT_ID)
        setattr(DR_init, "__dsr__node", self.dsr_node)
        # 액션 노드 (executor 에서 spin)
        self.node = rclpy.create_node("sanding_server", namespace=ROBOT_ID)
        self.log = self.node.get_logger()
        retreat = bool(self.node.declare_parameter("retreat_on_cancel", False).value)
        go_home = bool(self.node.declare_parameter("go_home", False).value)
        floor_z = float(self.node.declare_parameter("floor_z", FLOOR_Z_MM).value)
        floor_clear = float(self.node.declare_parameter("floor_clear", FLOOR_CLEAR_MM).value)
        tool_radius = float(self.node.declare_parameter("tool_radius", TOOL_RADIUS_MM).value)
        # 샌딩 누르는 힘 (N): HMI 가 goal 보내기 직전에 이 파라미터를 바꿈 → 작업 시작 때 읽음
        self.node.declare_parameter("press_force", PRESS_FORCE_N)

        self._gh = None
        self.job = SandingJob(self.dsr_node, self.log,
                              is_canceled=lambda: self._gh is not None and self._gh.is_cancel_requested,
                              on_step=self._feedback, retreat_on_cancel=retreat,
                              go_home=go_home, floor_z=floor_z, floor_clear=floor_clear,
                              tool_radius=tool_radius)
        self.log.info(f"바닥 z={floor_z:.1f}mm, 도구 반지름 {tool_radius:.1f}mm, 바닥 여유 {floor_clear:.1f}mm "
                      f"→ 바닥 쪽 작업 TCP 최저 z={floor_z + tool_radius + floor_clear:.1f}mm")
        self._busy = False
        self._lock = threading.Lock()
        self.action_server = ActionServer(
            self.node, ToolTask, ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),   # 실행 중에도 취소 요청 받기
        )

    def _feedback(self, step, total, text):
        if self._gh is not None:
            fb = ToolTask.Feedback()
            fb.step, fb.total_steps, fb.stage = step, total, text
            self._gh.publish_feedback(fb)

    def goal_callback(self, goal):
        if TOOL_POSX.get(goal.tool_id) is None:
            self.log.error(f"tool_id={goal.tool_id}: 등록되지 않은 도구 (TOOL_POSX 확인)")
            return GoalResponse.REJECT
        values = list(goal.start_posx) + list(goal.end_posx) + list(getattr(goal, "surface_normal", []))
        if not all(math.isfinite(v) for v in values):
            self.log.error("goal 에 NaN/inf 값이 있습니다.")
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.log.warn("이미 작업 중 - 새 goal 거절")
                return GoalResponse.REJECT
            self._busy = True
        where = (f"기본 작업 (시작 관절 {DEFAULT_START_POSJ}, +{DEFAULT_WORK_VECTOR} mm)" if is_default(goal)
                 else f"start={list(goal.start_posx)}, end={list(goal.end_posx)}")
        self.log.info(f"goal 수신: tool={goal.tool_id}, {where}, normal={goal_normal(goal) or '자동'}, "
                      f"return_tool={goal.return_tool}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.log.warn("취소 요청 수신 - 이동 중이면 즉시 정지합니다.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self._gh = goal_handle
        g = goal_handle.request
        result = ToolTask.Result()
        try:
            force = float(self.node.get_parameter("press_force").value)
            if not PRESS_FORCE_MIN_N <= force <= PRESS_FORCE_MAX_N:
                raise PlanError(f"누르는 힘 {force} N 은 {PRESS_FORCE_MIN_N}~{PRESS_FORCE_MAX_N} N 이어야 합니다")
            self.job.press_force = force
            self.log.info(f"샌딩 누르는 힘: {force:.1f} N")
            if is_default(g):
                self.job.run(TOOL_POSX[g.tool_id], None, None, goal_normal(g), g.return_tool,
                             start_posj=DEFAULT_START_POSJ, work_vector=DEFAULT_WORK_VECTOR)
            else:
                self.job.run(TOOL_POSX[g.tool_id], list(g.start_posx), list(g.end_posx),
                             goal_normal(g), g.return_tool)
            goal_handle.succeed()
            result.success, result.message = True, "완료"
        except PlanError as e:
            goal_handle.abort()
            result.success, result.message = False, f"계획 불가 (로봇 안 움직임): {e}"
        except Canceled as e:
            goal_handle.canceled()
            result.success, result.message = False, f"취소됨 ('{e}'), 도구를 들고 있을 수 있음"
        except Abort as e:
            goal_handle.abort()
            result.success, result.message = False, f"샌딩 중단 (후퇴 완료, 도구는 들고 있음): {e}"
        except Exception as e:
            self.job.force_off()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.message = f"취소/정지됨 (그 자리에서 멈춤, 도구를 들고 있을 수 있음): {e}"
            else:
                goal_handle.abort()
                result.message = f"오류 (그 자리에서 멈춤, 도구를 들고 있을 수 있음): {e}"
            result.success = False
        finally:
            self._gh = None
            with self._lock:
                self._busy = False
        (self.log.info if result.success else self.log.error)(f"결과: {result.message}")
        return result

    def destroy(self):
        self.action_server.destroy()
        self.node.destroy_node()
        self.dsr_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = SandingServer()
    try:
        server.job.wait_services()
    except RuntimeError as e:
        server.log.error(str(e))
        server.destroy()
        rclpy.shutdown()
        return
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server.node)
    server.log.info(f"샌딩 액션 서버 시작: /{ROBOT_ID}/{ACTION_NAME} (HMI [샌딩 작업 실행] 대기)")
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
