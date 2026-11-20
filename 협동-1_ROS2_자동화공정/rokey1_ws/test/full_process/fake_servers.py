"""가짜 /process/* 액션 (ROS 통신 없음, 로봇 안 움직임).

rclpy ActionClient 자리에 FakeActionClient 를 넣으면 goal 을 바로 아래 함수로 처리한다.
  EXPLORE    : 가짜 벽 4점 + 세로 틈 1개를 돌려줌 (FAKE_WALL / FAKE_GAPS)
  PUTTY_PICKUP / PUTTY_SCOOP / PUTTY / PUTTY_RETURN
             : 실제 putty_demo.action_protocol.process_putty_goal 로 goal → 워커 인자 변환·검사
               (실물 모드 규칙으로 검사: 뜨기는 같은 서버에서 집기가 성공했어야 함, 도포는 벽 법선·세로 홈·힘 설정)
  SANDING    : 벽 법선 계산(utils.wall) + robot.yaml sanding 값 검사
검사에 걸리면 실제 서버처럼 success=False + error_code 로 돌려준다.
"""
import json
from concurrent.futures import Future
from types import SimpleNamespace

from rokey_interfaces.action import Process

from putty_demo.action_protocol import process_error_code, process_putty_goal
from utils.force_stats import summarize
from utils.wall import wall_normal

# +X 벽 (x=600): top, floor, right, left — 법선이 정확히 −X 가 되도록
FAKE_WALL = [600.0, 0.0, 400.0, 600.0, 0.0, 35.0, 600.0, -300.0, 200.0, 600.0, 300.0, 200.0]
FAKE_GAPS = [600.0, 0.0, 100.0, 600.0, 0.0, 300.0]          # 세로 틈 1개 [sx,sy,sz, ex,ey,ez]


class FakeServers:
    def __init__(self, settings):
        self.settings = settings      # robot.yaml 내용 (dict)
        self.goals = []               # 받은 goal 순서 기록 (task_code, gap_id, start, end)
        self.held = False             # putty_task_server.spatula_held 와 같은 규칙
        self.commands = []            # 변환된 워커 명령 (task, arguments)
        self.scoop_pass = None        # putty_task_server.scoop_pass 와 같은 규칙

    def handle(self, goal):
        self.goals.append((goal.task_code, goal.gap_id, list(goal.start_point), list(goal.end_point)))
        try:
            if goal.task_code == 'EXPLORE':
                return Process.Result(success=True, wall=FAKE_WALL, gaps=FAKE_GAPS), 'explore_done'
            if goal.task_code.startswith('PUTTY'):
                req = process_putty_goal(goal, self.settings, virtual_mode=False, held=self.held,
                                         scoop_pass=self.scoop_pass)
                self.commands.append((req.task, list(req.arguments)))
                first = int(self.settings['force']['putty_scoop']['scoop_pass'])
                if goal.task_code in ('PUTTY_PICKUP', 'PUTTY_RETURN'):
                    self.held = goal.task_code == 'PUTTY_PICKUP'
                    self.scoop_pass = first if self.held else None
                if goal.task_code == 'PUTTY_SCOOP':
                    from putty_demo.scoop_putty import SCOOP_MAX_PASS
                    self.scoop_pass = (self.scoop_pass or first) % SCOOP_MAX_PASS + 1
                return Process.Result(success=True), f'{req.task}: {" ".join(req.arguments)}'
            if goal.task_code in ('SANDING', 'SANDING_KEEP'):
                normal = wall_normal(goal.wall)
                if normal is None:
                    return Process.Result(success=False, error_code='E_NO_WALL', message='벽 4점 없음'), ''
                force = float(self.settings['force']['sanding']['press_force'])
                if not 3.0 <= force <= 30.0:
                    return Process.Result(success=False, error_code='E_FORCE', message=f'press_force {force}'), ''
                if not self.settings['sanding']['tool_posx'].get(1):
                    return Process.Result(success=False, error_code='E_GRIP', message='도구 1 위치 없음'), ''
                return Process.Result(success=True), f'normal={[round(v, 3) for v in normal]} force={force}'
        except ValueError as e:
            return Process.Result(success=False, error_code=process_error_code(str(e)), message=str(e)), ''
        return Process.Result(success=False, error_code='E_FAIL', message=goal.task_code), ''


def _done(value):
    f = Future()
    f.set_result(value)
    return f


class FakeActionClient:
    """rclpy.action.ActionClient 대신. ProcessRunner 가 쓰는 메서드만."""
    servers = None      # 테스트에서 FakeServers 를 넣는다

    def __init__(self, node, action_type, name, callback_group=None):
        self.name = name

    def wait_for_server(self, timeout_sec=None):
        return True

    def server_is_ready(self):
        return True

    def send_goal_async(self, goal, feedback_callback=None):
        result, detail = self.servers.handle(goal)
        if feedback_callback:      # 실제 서버처럼 피드백 1번 → task_log 1줄 (params/force_stats JSON 포함)
            ratio, stats = (summarize([2.5, 3.1, 3.4, 2.9], target=3.0, tol=1.0, axis='fx')
                            if goal.task_code == 'PUTTY' else (-1.0, ''))
            fb = Process.Feedback(state='RUNNING' if result.success else 'ERROR',
                                  step=f'{goal.task_code.lower()}_fake', step_success=result.success,
                                  error_code=result.error_code, fail_reason=result.message[:100],
                                  joint=[0.0, 0.0, 90.0, 0.0, 90.0, 0.0], task=[367.4, 3.1, 219.3, 140.7, 180.0, 141.2],
                                  force=[0.1, -0.2, 0.3, 0.0, 0.0, 0.0], duration_ms=1234,
                                  params_json=json.dumps({'worker': detail, 'gap_id': int(goal.gap_id)},
                                                         ensure_ascii=False),
                                  contact_ratio=float(ratio), force_stats_json=stats)
            feedback_callback(SimpleNamespace(feedback=fb))
        handle = SimpleNamespace(accepted=True, detail=detail,
                                 get_result_async=lambda: _done(SimpleNamespace(result=result)),
                                 cancel_goal_async=lambda: None)
        return _done(handle)
