"""두산 M0609 + OnRobot RG2 : 도구 작업 액션 서버 (베이스 좌표계 기준)

액션: /dsr01/tool_task  (rokey_interfaces/action/ToolTask)
  Goal    : tool_id (1~3), start_posx[6], end_posx[6], return_tool
  Result  : success, message
  Feedback: step, total_steps, stage

동작 순서:
  1. 현재 위치에서 도구 위치(TOOL_POSX[tool_id]) 위 APPROACH_Z 지점으로 이동
     (홈 이동 없음, 이동 중 그리퍼는 현재 상태 유지)
  2. 접근 위치에서 그리퍼 열기 → 하강 → 그리퍼 닫기 → 들어올리기
  3. 작업 시작 위치 위로 접근 → 시작 위치로 하강
  4. 시작 위치 → 종료 위치까지 직선 이동 (작업)
  5. 종료 위치에서 위로 빠지기
  6. (return_tool=true) 도구를 원래 자리에 반납 → 홈 복귀

도구 위치 3개는 아래 TOOL_POSX 에 고정값으로 넣으세요 (get_pose 로 측정).
작업 시작/종료 위치는 goal 로 받습니다 (나중에 DB에서 읽어서 전달).

구조:
  DSR_ROBOT2 는 매 호출마다 rclpy.spin_until_future_complete(dsr_node) 로
  자기 노드를 직접 spin 하므로, dsr_node 는 executor 에 넣으면 안 됩니다.
  그래서 액션 서버 노드(action_node)와 로봇 제어 노드(dsr_node)를 분리합니다.
  goal 은 한 번에 하나만 받습니다 (실행 중이면 새 goal 거절).
  취소는 각 단계 사이에서만 반영됩니다 (이동 중에는 해당 이동이 끝난 뒤 멈춤).

사용법:
  (터미널 1) sod && sodreal
  (터미널 2) sod && ros2 run rokey1 tool_task_server
  (터미널 3) sod && ros2 run rokey1 tool_task_client   # 테스트용 클라이언트
"""
import math
import threading
import time

import rclpy
import DR_init
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from onrobot_rg_msgs.srv import SetCommand
from rokey_interfaces.action import ToolTask

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ── 위치 설정 ─────────────────────────────────────────────────────
# 고정 도구 위치 [x, y, z (mm), rx, ry, rz (deg)] - 베이스 기준
# 측정 전인 도구는 None 으로 두면 해당 tool_id goal 은 거절됩니다.
TOOL_POSX = {
    1: [364.96, -234.31, 92.91, 61.79, -177.53, 61.44],
    2: None,
    3: None,
}
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

APPROACH_Z = 100.0  # 도구/작업 위치 위에서 접근하는 높이 (mm)
LIFT_Z = 200.0      # 도구를 집은 뒤 들어올리는 높이 (mm)

# ── 속도 설정 (실제 로봇이므로 느리게) ─────────────────────────────
VELJ, ACCJ = 30, 30            # movej (deg/s, deg/s^2)
VELX, ACCX = 80, 80            # movel 일반 이동 (mm/s, mm/s^2)
SLOW_VELX, SLOW_ACCX = 30, 30  # 하강 / 들어올리기
WORK_VELX, WORK_ACCX = 30, 30  # 시작 → 종료 작업 이동

GRIPPER_WAIT = 2.0  # 그리퍼 동작 완료 대기 시간 (초)


def offset_z(pos, dz):
    """pos 의 z 에 dz 를 더한 새 리스트"""
    return pos[:2] + [pos[2] + dz] + pos[3:]


class Canceled(Exception):
    pass


class ToolTaskServer:
    def __init__(self):
        # 로봇 제어용 노드 (executor 에 넣지 않음 - DSR_ROBOT2 가 직접 spin)
        self.dsr_node = rclpy.create_node("sanding_tool_task_dsr", namespace=ROBOT_ID)
        # 클래스 안에서 DR_init.__dsr__node = ... 로 쓰면 이름 맹글링으로
        # DR_init._ToolTaskServer__dsr__node 에 들어가 버리므로 setattr 사용
        setattr(DR_init, "__dsr__node", self.dsr_node)

        # 액션 서버용 노드 (별도 스레드의 executor 에서 spin)
        self.node = rclpy.create_node("sanding_tool_task_server", namespace=ROBOT_ID)
        self.logger = self.node.get_logger()

        from DSR_ROBOT2 import movej, movel
        from DR_common2 import posj, posx
        self.movej, self.movel = movej, movel
        self.posj, self.posx = posj, posx

        self.gripper_cli = self.dsr_node.create_client(SetCommand, "/onrobot/sendCommand")

        self._busy = False
        self._lock = threading.Lock()

        self.action_server = ActionServer(
            self.node,
            ToolTask,
            "tool_task",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup(),  # 실행 중에도 cancel 요청 처리
        )

    # ── 그리퍼 ──────────────────────────────────────────────────
    def gripper(self, cmd):
        """cmd: 'o' = 열기, 'c' = 닫기"""
        req = SetCommand.Request()
        req.command = cmd
        future = self.gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self.dsr_node, future, timeout_sec=5.0)
        if not future.done() or not future.result().success:
            raise RuntimeError(f"Gripper command '{cmd}' failed")
        time.sleep(GRIPPER_WAIT)

    # ── goal / cancel ──────────────────────────────────────────
    def goal_callback(self, goal):
        tool_pos = TOOL_POSX.get(goal.tool_id)
        if tool_pos is None:
            self.logger.error(f"tool_id={goal.tool_id}: 등록되지 않은 도구입니다. (TOOL_POSX 확인)")
            return GoalResponse.REJECT

        values = list(goal.start_posx) + list(goal.end_posx)
        if not all(math.isfinite(v) for v in values):
            self.logger.error("start_posx / end_posx 에 잘못된 값(NaN/inf)이 있습니다.")
            return GoalResponse.REJECT

        with self._lock:
            if self._busy:
                self.logger.warn("이미 작업 중입니다. 새 goal 을 거절합니다.")
                return GoalResponse.REJECT
            self._busy = True

        self.logger.info(
            f"goal 수신: tool={goal.tool_id}, start={list(goal.start_posx)}, "
            f"end={list(goal.end_posx)}, return_tool={goal.return_tool}"
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.logger.info("취소 요청 수신 - 현재 단계가 끝나면 멈춥니다.")
        return CancelResponse.ACCEPT

    # ── 실행 ────────────────────────────────────────────────────
    def execute_callback(self, goal_handle):
        try:
            return self._run(goal_handle)
        finally:
            with self._lock:
                self._busy = False

    def _run(self, goal_handle):
        g = goal_handle.request
        tool = TOOL_POSX[g.tool_id]
        start = list(g.start_posx)
        end = list(g.end_posx)

        home = self.posj(HOME_POSJ)
        P = self.posx

        # (설명, 동작) 목록
        steps = [
            # 홈으로 가지 않고 현재 위치에서 바로 이동, 그리퍼는 현재 상태 유지
            (f"도구 {g.tool_id} 접근 위치로 이동 (현재 위치에서)",
             lambda: self.movel(P(offset_z(tool, APPROACH_Z)), vel=VELX, acc=ACCX)),
            ("접근 위치에서 그리퍼 열기", lambda: self.gripper("o")),
            ("도구 위치로 하강", lambda: self.movel(P(tool), vel=SLOW_VELX, acc=SLOW_ACCX)),
            ("그리퍼 닫기 (도구 잡기)", lambda: self.gripper("c")),
            ("도구 들어올리기",
             lambda: self.movel(P(offset_z(tool, LIFT_Z)), vel=SLOW_VELX, acc=SLOW_ACCX)),
            ("작업 시작 접근 위치로 이동",
             lambda: self.movel(P(offset_z(start, APPROACH_Z)), vel=VELX, acc=ACCX)),
            ("작업 시작 위치로 하강", lambda: self.movel(P(start), vel=SLOW_VELX, acc=SLOW_ACCX)),
            ("작업: 시작 → 종료 위치 이동", lambda: self.movel(P(end), vel=WORK_VELX, acc=WORK_ACCX)),
            ("종료 위치에서 위로 빠지기",
             lambda: self.movel(P(offset_z(end, APPROACH_Z)), vel=SLOW_VELX, acc=SLOW_ACCX)),
        ]
        if g.return_tool:
            steps += [
                ("도구 반납: 도구 위치 위로 이동",
                 lambda: self.movel(P(offset_z(tool, LIFT_Z)), vel=VELX, acc=ACCX)),
                ("도구 반납: 하강", lambda: self.movel(P(tool), vel=SLOW_VELX, acc=SLOW_ACCX)),
                ("도구 반납: 그리퍼 열기", lambda: self.gripper("o")),
                ("도구 반납: 위로 빠지기",
                 lambda: self.movel(P(offset_z(tool, APPROACH_Z)), vel=SLOW_VELX, acc=SLOW_ACCX)),
                ("홈 위치로 복귀", lambda: self.movej(home, vel=VELJ, acc=ACCJ)),
            ]

        result = ToolTask.Result()
        total = len(steps)
        try:
            for i, (stage, action) in enumerate(steps, start=1):
                if goal_handle.is_cancel_requested:
                    raise Canceled(stage)

                fb = ToolTask.Feedback()
                fb.step, fb.total_steps, fb.stage = i, total, stage
                goal_handle.publish_feedback(fb)
                self.logger.info(f"[{i}/{total}] {stage}")

                action()

            goal_handle.succeed()
            result.success = True
            result.message = "완료"
            self.logger.info("작업 완료")

        except Canceled as e:
            goal_handle.canceled()
            result.success = False
            result.message = f"취소됨 ('{e}' 단계 전)"
            self.logger.warn(result.message)

        except Exception as e:
            goal_handle.abort()
            result.success = False
            result.message = f"오류: {e}"
            self.logger.error(result.message)

        return result

    def destroy(self):
        self.action_server.destroy()
        self.node.destroy_node()
        self.dsr_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = ToolTaskServer()

    if not server.gripper_cli.wait_for_service(timeout_sec=5.0):
        server.logger.error("/onrobot/sendCommand 서비스가 없습니다. sodreal이 실행 중인지 확인하세요.")
        server.destroy()
        rclpy.shutdown()
        return

    # 액션 노드만 executor 로 spin (dsr_node 는 DSR_ROBOT2 가 직접 spin)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server.node)
    server.logger.info("ToolTask 액션 서버 시작: /dsr01/tool_task")

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
