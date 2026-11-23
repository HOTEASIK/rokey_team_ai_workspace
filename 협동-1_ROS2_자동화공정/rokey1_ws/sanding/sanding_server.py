"""샌딩 액션 서버 - 팀 HMI(hmi_server) 형식에 맞춤 (MoveIt 미사용, 두산 순응/힘제어)

액션: /process/sanding  (Process) ← hmi_server [공정(DB)] 가 틈마다 보냄
  Goal    : start_point[3], end_point[3] (퍼티가 작업한 DB gap 좌표), wall[12] (그 벽 4점)
            → 벽 법선 = utils.wall.wall_normal(wall), 자세는 TOOL_POSX[1] 자세를 그 법선으로 보정
            → wall 이 비어 있으면 E_NO_WALL (로봇 안 움직임)
  Feedback: 단계(phase)가 끝날 때마다 1회 (→ task_log 1줄), Result: success, error_code, message

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
  ※ HMI 의 [샌딩 액션 서버] 버튼(= 이 sanding_server)을 두 번 띄우지 마세요
    (같은 /dsr01/tool_task 이름)
"""
import json
import math
import threading
import time

import rclpy
import DR_init
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rokey_interfaces.action import Process, ToolTask

from utils.config import CFG
from utils.wall import wall_normal
from sanding.sanding_job import (FLOOR_CLEAR_MM, PRESS_FORCE_MAX_N, PRESS_FORCE_MIN_N, PRESS_FORCE_N, ROBOT_ID, TOOL_RADIUS_MM, Abort, Canceled, PlanError,
                                  SandingJob)

ACTION_NAME = "tool_task"   # HMI 가 /dsr01/tool_task 로 보냄

SCFG = CFG["sanding"]   # config/robot.yaml sanding 섹션 (도구 위치·바닥 높이 등)

# 도구 잡는 TCP 자세 [x, y, z (mm), rx, ry, rz (deg)] - 베이스 기준. 측정 전은 null (robot.yaml sanding.tool_posx)
TOOL_POSX = {int(k): (list(map(float, v)) if v else None) for k, v in SCFG["tool_posx"].items()}

# 기본 작업 - goal 의 start_posx / end_posx 를 비우면(전부 0) 이걸로 작업 (실제 테스트에서 잘 된 값)
DEFAULT_START_POSJ = [-8.21, 31.18, 114.93, -10.05, -50.25, 100.44]   # 시작 관절각 (deg) → fkin 으로 TCP
DEFAULT_WORK_VECTOR = [0.0, 100.0, 0.0]                                 # 베이스 +Y 10cm

# 벽 아래 바닥 높이 (베이스 z, mm). 원형 도구(반지름 45mm) 가장자리가 바닥 + 5mm 보다 낮아지면 작업을 위로 올림
#   측정값: 벽 아래 바닥이 베이스 기준 z≈35mm
#   (바닥 가까이 주면(이 값 + 50 아래) 전부 '바닥에 붙여서 작업' 으로 보고 도구 아래끝이 바닥 + 5mm 가 되게 맞춤)
#   -p floor_z:=<값> 으로 바꿀 수 있음. 아주 작은 값(예 -9999)이면 사실상 검사 안 함
FLOOR_Z_MM = float(SCFG["floor_z"])


PROCESS_TOOL_ID = 1


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
        self.dsr_node = rclpy.create_node("rokey1_sanding_dsr", namespace=ROBOT_ID)
        setattr(DR_init, "__dsr__node", self.dsr_node)
        # 액션 노드 (executor 에서 spin)
        self.node = rclpy.create_node("rokey1_sanding_server", namespace=ROBOT_ID)
        self.log = self.node.get_logger()
        retreat = bool(self.node.declare_parameter("retreat_on_cancel", False).value)
        go_home = bool(self.node.declare_parameter("go_home", False).value)
        virtual = bool(self.node.declare_parameter("virtual", False).value)   # sodvir: 힘 대신 위치로 접촉 판단
        floor_z = float(self.node.declare_parameter("floor_z", FLOOR_Z_MM).value)
        floor_clear = float(self.node.declare_parameter("floor_clear", float(SCFG.get("floor_clear", FLOOR_CLEAR_MM))).value)
        tool_radius = float(self.node.declare_parameter("tool_radius", float(SCFG.get("tool_radius", TOOL_RADIUS_MM))).value)
        # 샌딩 누르는 힘 (N): HMI 가 goal 보내기 직전에 이 파라미터를 바꿈 → 작업 시작 때 읽음
        self.node.declare_parameter(
            "press_force", float((CFG["force"].get("sanding") or {}).get("press_force") or PRESS_FORCE_N))  # robot.yaml

        self._gh = None
        self._process, self._phase, self._t0 = False, 0, 0.0
        self._params = {}          # /process/sanding: 이번 작업에 쓴 힘/법선/좌표 → 피드백 params_json
        self.job = SandingJob(self.dsr_node, self.log,
                              is_canceled=lambda: self._gh is not None and self._gh.is_cancel_requested,
                              on_step=self._feedback, retreat_on_cancel=retreat,
                              go_home=go_home, floor_z=floor_z, floor_clear=floor_clear,
                              tool_radius=tool_radius, virtual=virtual)
        self.log.info(f"바닥 z={floor_z:.1f}mm → 작업 TCP 최저 z={floor_z + self.job.floor_min_tcp:.1f}mm "
                      f"(바닥 + {self.job.floor_min_tcp:.0f}) | 몸체 쪽 하강 {self.job.body_side_mm:.0f}mm")
        self._busy = False
        self._lock = threading.Lock()
        # /process/sanding 도구 상태 (전체 공정: 첫 틈에서 집고, SANDING_KEEP 이면 들고 있고, SANDING 이면 반납)
        #   empty: 거치대에 있음 / held: 이 run 에서 들고 있음 / unknown: 도중에 멈춤 → 사람이 확인 후 서버 재시작
        self._tool_state, self._tool_run = "empty", None
        self.action_server = ActionServer(
            self.node, ToolTask, ACTION_NAME,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),   # 실행 중에도 취소 요청 받기
        )
        self.process_server = ActionServer(
            self.node, Process, "/process/sanding",
            execute_callback=self.process_execute,
            goal_callback=self.process_goal,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _feedback(self, step, total, text):
        if self._gh is None:
            return
        if self._process:
            if step != self._phase:
                self._phase_done(True)
                self._phase, self._t0 = step, time.monotonic()
            return
        fb = ToolTask.Feedback()
        fb.step, fb.total_steps, fb.stage = step, total, text
        self._gh.publish_feedback(fb)

    def _phase_done(self, ok, code="", reason=""):
        """Process 피드백: 직전 단계(phase) 1개가 끝났음을 알림"""
        if self._phase:
            self._gh.publish_feedback(Process.Feedback(
                state="RUNNING" if ok else "ERROR", step=f"sanding_{self._phase:02d}",
                step_success=ok, error_code=code, fail_reason=reason,
                duration_ms=int((time.monotonic() - self._t0) * 1000),
                params_json=json.dumps(self._params)))

    def _accept(self, values, text):
        if not all(math.isfinite(v) for v in values):
            self.log.error("goal 에 NaN/inf 값이 있습니다.")
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.log.warn("이미 작업 중 - 새 goal 거절")
                return GoalResponse.REJECT
            self._busy = True
        self.log.info(f"goal 수신: {text}")
        return GoalResponse.ACCEPT

    def goal_callback(self, goal):
        if TOOL_POSX.get(goal.tool_id) is None:
            self.log.error(f"tool_id={goal.tool_id}: 등록되지 않은 도구 (TOOL_POSX 확인)")
            return GoalResponse.REJECT
        where = (f"기본 작업 (시작 관절 {DEFAULT_START_POSJ}, +{DEFAULT_WORK_VECTOR} mm)" if is_default(goal)
                 else f"start={list(goal.start_posx)}, end={list(goal.end_posx)}")
        return self._accept(list(goal.start_posx) + list(goal.end_posx) + list(getattr(goal, "surface_normal", [])),
                            f"tool={goal.tool_id}, {where}, normal={goal_normal(goal) or '자동'}, "
                            f"return_tool={goal.return_tool}")

    def process_goal(self, goal):
        return self._accept(list(goal.start_point) + list(goal.end_point),
                            f"/process/sanding run={goal.run_id} gap={goal.gap_id}, "
                            f"start={list(goal.start_point)}, end={list(goal.end_point)}, normal={wall_normal(goal.wall)}")

    def cancel_callback(self, goal_handle):
        self.log.warn("취소 요청 수신 - 이동 중이면 즉시 정지합니다.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        g = goal_handle.request
        self._process = False
        if is_default(g):
            ok, msg, _ = self._job(goal_handle, TOOL_POSX[g.tool_id], None, None, goal_normal(g), g.return_tool,
                                   start_posj=DEFAULT_START_POSJ, work_vector=DEFAULT_WORK_VECTOR)
        else:
            ok, msg, _ = self._job(goal_handle, TOOL_POSX[g.tool_id], list(g.start_posx), list(g.end_posx),
                                   goal_normal(g), g.return_tool)
        return ToolTask.Result(success=ok, message=msg)

    def process_execute(self, goal_handle):
        g = goal_handle.request
        tool = TOOL_POSX[PROCESS_TOOL_ID]
        keep = g.task_code == "SANDING_KEEP"     # 다음 틈이 남음 → 반납하지 않고 들고 몸체 쪽 위에서 끝
        state, held_run = self._tool_state, self._tool_run
        if state == "unknown" or (state == "held" and held_run != g.run_id):
            msg = ("앞 샌딩이 도중에 멈춰 도구를 들고 있을 수 있음 → 로봇이 도구를 쥐고 있으면 손으로 거치대에 "
                   "돌려놓고 샌딩 액션 서버를 다시 시작하세요" if state == "unknown"
                   else f"도구를 들고 있음 (run {held_run}) - 다른 run({g.run_id}) 에서는 쓰지 않음. 확인 후 서버 재시작")
            self.log.error(msg)
            goal_handle.abort()
            with self._lock:
                self._busy = False
            return Process.Result(success=False, error_code="E_GRIP", message=msg)
        holding = state == "held"
        normal = wall_normal(g.wall)
        if normal is None:
            msg = "벽 4점(goal.wall)이 없어 벽 법선 계산 불가 → 로봇 안 움직임"
            self.log.error(msg)
            goal_handle.abort()
            with self._lock:
                self._busy = False
            return Process.Result(success=False, error_code="E_NO_WALL", message=msg)
        self._process, self._phase, self._t0 = True, 0, time.monotonic()
        self._params = {"normal": [round(v, 4) for v in normal],
                        "start": [float(v) for v in g.start_point], "end": [float(v) for v in g.end_point]}
        self.log.info(f"도구: {'들고 있음 → 집기 생략' if holding else '거치대 → 집기'}, "
                      f"끝나면 {'들고 몸체 쪽 위에서 대기 (다음 틈)' if keep else '반납'}")
        ok, msg, code = self._job(goal_handle, tool, list(g.start_point) + tool[3:], list(g.end_point) + tool[3:],
                                  normal, not keep, holding=holding)
        if ok:
            self._tool_state, self._tool_run = ("held", g.run_id) if keep else ("empty", None)
        elif code != "E_SINGULAR":      # 계획 불가(로봇 안 움직임)만 상태 유지. 나머지는 도구 위치를 모름
            self._tool_state = "unknown"
        return Process.Result(success=ok, error_code=code, message=msg)

    def _job(self, goal_handle, tool, start, end, normal, return_tool, **kw):
        """SandingJob 실행. 반환: (성공, 메시지, Process 오류코드)"""
        self._gh = goal_handle
        ok, msg, code = False, "중단됨 (서버 종료 중)", "E_MOTION"   # 예외 처리 중 또 예외가 나도 finally 가 쓸 값
        try:
            force = float(self.node.get_parameter("press_force").value)
            if not PRESS_FORCE_MIN_N <= force <= PRESS_FORCE_MAX_N:
                raise PlanError(f"누르는 힘 {force} N 은 {PRESS_FORCE_MIN_N}~{PRESS_FORCE_MAX_N} N 이어야 합니다")
            self.job.press_force = force
            self._params["press_force"] = force
            self.log.info(f"샌딩 누르는 힘: {force:.1f} N")
            self.job.run(tool, start, end, normal, return_tool, **kw)
            goal_handle.succeed()
            ok, msg, code = True, "완료", ""
        except PlanError as e:
            goal_handle.abort()
            ok, msg, code = False, f"계획 불가 (로봇 안 움직임): {e}", "E_SINGULAR"
        except Canceled as e:
            goal_handle.canceled()
            ok, msg, code = False, f"취소됨 ('{e}'), 도구를 들고 있을 수 있음", "CANCELED"
        except Abort as e:
            goal_handle.abort()
            ok, msg, code = False, f"샌딩 중단 (후퇴 완료, 도구는 들고 있음): {e}", "E_FORCE"
        except Exception as e:
            self.job.force_off()
            ok = False
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                msg, code = f"취소/정지됨 (그 자리에서 멈춤, 도구를 들고 있을 수 있음): {e}", "CANCELED"
            else:
                goal_handle.abort()
                msg = f"오류 (그 자리에서 멈춤, 도구를 들고 있을 수 있음): {e}"
                code = "E_GRIP" if "그리퍼" in str(e) else "E_MOTION"
        finally:
            if self._process:
                self._phase_done(ok, code, "" if ok else msg[:100])
            self._gh = None
            with self._lock:
                self._busy = False
        # rclpy 로거는 같은 줄에서 심각도를 바꾸면 ValueError → 줄을 나눔
        if ok:
            self.log.info(f"결과: {msg}")
        else:
            self.log.error(f"결과: {msg}")
        return ok, msg, code

    def destroy(self):
        self.process_server.destroy()
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
        return 1        # 실패를 종료코드로 알림 (예전 0 → HMI 에 '완료'로 보였음)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server.node)
    server.log.info(f"샌딩 액션 서버 시작: /{ROBOT_ID}/{ACTION_NAME}, /process/sanding")
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
