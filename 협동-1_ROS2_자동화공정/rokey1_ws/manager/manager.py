"""통합 매니저

흐름:
  웹 → command 테이블 INSERT('START') → 매니저가 poll_sec 마다 확인
  → EXPLORE 1회 → wall/gap DB 저장 → 틈마다 manager.order 공정 순서대로 실행
  → 공정 피드백 1회 = task_log 1줄 (DB 쓰기는 매니저만)

비상정지 (DB 거치지 않음):
  웹 → rosbridge → /estop (std_msgs/Empty) → move_stop(DR_QSTOP) + 현재 goal 취소 → ESTOP 기록
  해제는 다음 START 명령.

상태 방송: /manager/state (std_msgs/String) - 웹/HMI 표시용

액션 이름: /process/explore, /process/putty, /process/sanding (rokey_interfaces/action/Process)

실행:
  ros2 run rokey1 manager
"""
import os
import subprocess
import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String

from dsr_msgs2.srv import MoveJoint, MoveStop
from rokey_interfaces.action import Process
from utils import config
from utils.config import CFG
from utils.db import DB

ROBOT_ID = 'dsr01'
DR_QSTOP = 1
TASKS = ['EXPLORE', 'PUTTY', 'SANDING']


def code_version():
    """run 테이블 기록용 git 짧은 해시 (+ 커밋 안 된 변경이 있으면 -dirty)"""
    try:
        return subprocess.check_output(
            ['git', 'describe', '--always', '--dirty'], text=True,
            cwd=os.path.dirname(config.PATH), stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return ''

# 오류 코드: (재시도 횟수, 이후 대응)
#   HOME = 대기 자세 복귀 후 종료 / HOLD = 그 자리 정지, 사용자 START 대기 / SKIP = 이 틈 건너뜀
POLICY = {
    'E_GRIP': (1, 'HOLD'),
    'E_FORCE': (1, 'HOME'),
    'E_SINGULAR': (0, 'HOME'),   # 회피 재시도는 singularity util 안에서
    'E_TIMEOUT': (0, 'HOLD'),
    'E_NO_GAP': (0, 'SKIP'),
}


class Manager(Node):
    def __init__(self):
        super().__init__('manager', namespace=ROBOT_ID)
        cb = ReentrantCallbackGroup()
        self.db = DB()
        self.clients_ = {t: ActionClient(self, Process, f'/process/{t.lower()}', callback_group=cb)
                         for t in TASKS}
        self.stop_cli = self.create_client(MoveStop, 'dsr_controller2/motion/move_stop', callback_group=cb)
        self.movej_cli = self.create_client(MoveJoint, 'dsr_controller2/motion/move_joint', callback_group=cb)
        self.state_pub = self.create_publisher(String, '/manager/state', 10)
        self.create_subscription(Empty, '/estop', self.on_estop, 10, callback_group=cb)
        self.create_timer(CFG['manager']['poll_sec'], self.poll, callback_group=cb)

        self.worker = None
        self.goal_handle = None
        self.abort = None       # None / 'STOP' / 'ESTOP'
        self.run_id = 0
        self.version = code_version()
        self.set_state('IDLE')

    def set_state(self, s):
        self.state = s
        self.state_pub.publish(String(data=s))
        self.get_logger().info(f'state: {s}')

    # ── 명령 / 비상정지 ───────────────────────────────
    def poll(self):
        row = self.db.fetch_command()
        if row is None:
            return
        cmd_id, cmd, purpose, material, note = row
        busy = self.worker is not None and self.worker.is_alive()
        if cmd == 'START' and not busy:
            self.db.set_command(cmd_id, 'ACCEPTED')
            self.db.start_run(cmd_id, purpose, material, note, self.version)
            self.abort = None
            self.worker = threading.Thread(target=self.run, args=(cmd_id,), daemon=True)
            self.worker.start()
        elif cmd == 'STOP':
            self.cancel('STOP')
            self.db.set_command(cmd_id, 'DONE')
        else:
            self.db.set_command(cmd_id, 'FAILED')   # 실행 중 START, 알 수 없는 명령

    def on_estop(self, _):
        self.stop_cli.call_async(MoveStop.Request(stop_mode=DR_QSTOP))   # 정지가 먼저
        self.cancel('ESTOP')
        self.set_state('ESTOP')
        self.log(self.run_id, 'ESTOP', -1, 0, 'estop', 'ESTOP', 'emergency stop')

    def cancel(self, why):
        self.abort = why
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

    def log(self, run_id, task, gap_id, attempt, step, code, reason):
        """매니저가 직접 남기는 실패 기록 (공정 result 실패, 비상정지)"""
        z6 = [0.0] * 6
        self.db.log_step(run_id, task, gap_id, attempt, SimpleNamespace(
            step=step, step_success=False, error_code=code, fail_reason=reason,
            joint=z6, task=z6, force=z6, duration_ms=0,
            params_json='', contact_ratio=-1.0, force_stats_json=''))

    # ── 전체 순서 ─────────────────────────────────────
    def run(self, run_id):
        self.run_id = run_id
        self.set_state('RUNNING')
        verdict, res = self.do('EXPLORE', run_id, -1, [0.0] * 3, [0.0] * 3)
        if verdict == 'OK':
            wall_id = self.db.insert_wall(run_id, list(res.wall))
            g = list(res.gaps)
            gaps = [(self.db.insert_gap(wall_id, g[i:i + 6]), g[i:i + 6]) for i in range(0, len(g), 6)]
            for task in CFG['manager']['order']:
                for gap_id, p in gaps:
                    verdict, _ = self.do(task, run_id, gap_id or -1, p[:3], p[3:])
                    if verdict not in ('OK', 'SKIP'):
                        break
                if verdict not in ('OK', 'SKIP'):
                    break
        self.finish(run_id, verdict)

    def finish(self, run_id, verdict):
        if verdict == 'HOME':
            self.home()
        ok = verdict in ('OK', 'SKIP')
        self.db.set_command(run_id, 'DONE' if ok else 'FAILED')
        self.db.end_run(run_id, verdict)
        if self.state != 'ESTOP':
            self.set_state('IDLE' if ok or verdict == 'STOP' else 'ERROR')

    def do(self, task, run_id, gap_id, start, end):
        """공정 1개 실행 (재시도 포함). 반환: (OK|SKIP|HOME|HOLD|STOP|ESTOP, result)"""
        tries = 0
        while True:
            res = self.send(task, run_id, gap_id, start, end, tries)
            if self.abort:
                return self.abort, res
            if res is not None and res.success:
                return 'OK', res
            code = res.error_code if res else 'E_NO_SERVER'
            self.log(run_id, task, gap_id, tries, f'{task.lower()}_result', code,
                     res.message if res else 'action server not found')
            if task == 'PUTTY':
                # 현장 퍼티는 특이점/힘 형성 문제를 검증 중이다.
                # 팀의 기본 재시도·HOME 정책으로 자동 관절 이동을 추가하지 않는다.
                return 'HOLD', res
            retry, action = POLICY.get(code, (0, 'HOLD'))
            if tries >= retry:
                return action, res
            tries += 1

    def send(self, task, run_id, gap_id, start, end, attempt):
        cli = self.clients_[task]
        if not cli.wait_for_server(timeout_sec=3.0):
            return None
        goal = Process.Goal(task_code=task, run_id=run_id, gap_id=gap_id,
                            start_point=[float(v) for v in start], end_point=[float(v) for v in end])
        fb_cb = lambda m: self.db.log_step(run_id, task, gap_id, attempt, m.feedback)  # noqa: E731
        self.goal_handle = wait(cli.send_goal_async(goal, feedback_callback=fb_cb))
        if not self.goal_handle.accepted:
            self.goal_handle = None
            return None
        if self.abort:
            self.goal_handle.cancel_goal_async()  # Goal 수락 대기 중 들어온 STOP도 반영
        res = wait(self.goal_handle.get_result_async()).result
        self.goal_handle = None
        return res

    def home(self):
        p = CFG['poses']['standby']['value']
        wait(self.movej_cli.call_async(MoveJoint.Request(
            pos=[float(v) for v in p], vel=float(CFG['common']['vel']), acc=float(CFG['common']['acc']))))


def wait(future):
    """executor 가 다른 스레드에서 spin 중이므로 완료까지 대기만 한다."""
    while not future.done():
        time.sleep(0.05)
    return future.result()


def main():
    rclpy.init()
    node = Manager()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
