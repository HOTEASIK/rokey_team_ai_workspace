"""웹 시작 버튼 → /process/* 액션 호출 → DB 기록.

  ALL     : EXPLORE → 찾은 틈마다 PUTTY → 틈마다 SANDING
  PUTTY 1회 = /process/putty_load (판에서 퍼티 묻히기, robot.yaml putty_load_line) → /process/putty (틈 도포)
  EXPLORE : 탐색만 (wall / gap 저장)
  PUTTY / SANDING : 웹에서 지정한 gap_id 한 틈만 (좌표는 DB gap 에서 읽음)

액션 피드백 1번 = task_log 1줄. 실패하면 그 자리에서 중단 (재시도 없음).
끝나면 run.result(OK / STOP / ESTOP / E_xxx) + run.data_check 기록.
hmi_server 가 실행 도중 꺼지면 다음 시작 때 그 run 을 ABORTED 로 마감한다.
"""
import os
import subprocess
import time
from types import SimpleNamespace

from rclpy.action import ActionClient

from rokey_interfaces.action import Process
from utils import config
from utils.config import CFG
from utils.db import DB

TASKS = ('EXPLORE', 'PUTTY_LOAD', 'PUTTY', 'SANDING')


def db_task(task):
    """task_log.task_code: 퍼티 묻히기도 PUTTY 로 묶는다 (step 이 putty_load 로 구분됨)"""
    return 'PUTTY' if task == 'PUTTY_LOAD' else task


def code_version():
    try:
        return subprocess.check_output(
            ['git', 'describe', '--always', '--dirty'], text=True,
            cwd=os.path.dirname(config.PATH), stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def wait(future):
    """executor 가 다른 스레드에서 spin 중이므로 완료까지 대기만 한다."""
    while not future.done():
        time.sleep(0.05)
    return future.result()


class ProcessRunner:
    def __init__(self, node, hub):
        self.hub = hub
        self.db = DB()
        self.version = code_version()
        self.clients = {t: ActionClient(node, Process, f'/process/{t.lower()}', callback_group=node.cb)
                        for t in TASKS}
        try:
            n = self.db.abort_open_runs()
        except Exception as e:
            hub.log('ERROR', 'DB', f'DB 연결 실패: {e}')
            return
        if n:
            hub.log('WARN', 'DB', f'이전 실행 중 끊긴 run {n}개를 ABORTED 로 마감')

    def run(self, run, is_estop):
        """run: hmi_server 의 Run. 반환: run.result 문자열."""
        p = run.params
        task = run.task['process']
        gap_id = int(p['gap_id']) if task in ('PUTTY', 'SANDING') else None
        run_id = self.db.start_run(task, gap_id, run.mode, p['purpose'], p['material'],
                                   p['note'] or None, self.version)
        if run_id is None:
            self.hub.log('ERROR', 'DB', 'run 저장 실패 → DB 연결 확인 (~/ws_cobot_pjt/logs/pending.jsonl)',
                         run_id=run.id)
            return 'E_DB'
        self.hub.log('INFO', 'DB', f'DB run_id={run_id} · {task}', run_id=run.id)

        result = self._steps(run, run_id, task, gap_id)
        if run.stop_requested:
            result = 'ESTOP' if is_estop() else 'STOP'
        check = self.db.check_run(run_id, task)
        self.db.end_run(run_id, result, check)
        self.hub.log('INFO' if result == 'OK' else 'WARN', 'DB',
                     f'DB run_id={run_id} 결과={result} · 데이터={check}', run_id=run.id)
        return result

    def _steps(self, run, run_id, task, gap_id):
        if task in ('PUTTY', 'SANDING'):
            row = self.db.gap_points(gap_id)
            if row is None:
                self.hub.log('ERROR', 'DB', f'gap_id={gap_id} 가 DB에 없습니다', run_id=run.id)
                return 'E_NO_GAP'
            return self._each(run, run_id, [task], [(gap_id, row[0])], row[1])

        code, res = self._do(run, run_id, 'EXPLORE', -1, [0.0] * 6)
        if code != 'OK':
            return code
        wall_id = self.db.insert_wall(run_id, list(res.wall))
        g = list(res.gaps)
        gaps = [(self.db.insert_gap(wall_id, g[i:i + 6]), g[i:i + 6]) for i in range(0, len(g), 6)] if wall_id else []
        if wall_id is None or any(gid is None for gid, _ in gaps):
            self.hub.log('ERROR', 'DB', 'wall/gap 저장 실패 → 퍼티·샌딩 중단 (DB 연결 확인)', run_id=run.id)
            return 'E_DB'
        self.hub.log('INFO', 'DB', f'wall_id={wall_id} · gap_id={[gid for gid, _ in gaps]}', run_id=run.id)
        if task == 'EXPLORE':
            return 'OK'
        return self._each(run, run_id, ['PUTTY', 'SANDING'], gaps, list(res.wall))

    def _each(self, run, run_id, tasks, gaps, wall):
        """wall: 틈이 있는 벽 4점 (퍼티/샌딩 goal 에 같이 보냄 → 샌딩이 벽 법선 계산)"""
        line = CFG.get('putty_load_line') or {}
        load = list(line.get('start', [])) + list(line.get('end', []))
        for task in tasks:
            for gid, pts in gaps:
                steps = [('PUTTY_LOAD', load), ('PUTTY', pts)] if task == 'PUTTY' else [(task, pts)]
                for t, p in steps:
                    if len(p) != 6:
                        self.hub.log('ERROR', t, 'robot.yaml putty_load_line {start, end} 설정 필요', run_id=run.id)
                        return 'E_NO_GAP'
                    code, _ = self._do(run, run_id, t, gid or -1, p, wall)
                    if code != 'OK':
                        return code
        return 'OK'

    def _do(self, run, run_id, task, gap_id, pts, wall=None):
        """공정 1개. 반환: ('OK' | 'STOP' | 오류코드, result)"""
        if run.stop_requested:
            return 'STOP', None
        res = self._send(run, run_id, task, gap_id, pts, wall)
        if run.stop_requested:
            return 'STOP', res
        if res is not None and res.success:
            return 'OK', res
        code = (res.error_code if res else 'E_NO_SERVER') or 'E_FAIL'
        reason = res.message if res else f'/process/{task.lower()} 액션 서버 없음'
        z6 = [0.0] * 6
        self.db.log_step(run_id, db_task(task), gap_id, 0, SimpleNamespace(
            step=f'{task.lower()}_result', step_success=False, error_code=code, fail_reason=reason,
            joint=z6, task=z6, force=z6, duration_ms=0,
            params_json='', contact_ratio=-1.0, force_stats_json=''))
        self.hub.log('ERROR', task, f'{code}: {reason}', run_id=run.id)
        return code, res

    def _send(self, run, run_id, task, gap_id, pts, wall=None):
        cli = self.clients[task]
        if not cli.wait_for_server(timeout_sec=3.0):
            return None
        goal = Process.Goal(task_code=task, run_id=run_id, gap_id=gap_id,
                            start_point=[float(v) for v in pts[:3]], end_point=[float(v) for v in pts[3:6]],
                            wall=[float(v) for v in (wall or [0.0] * 12)])

        def on_feedback(msg):
            fb = msg.feedback
            self.db.log_step(run_id, db_task(task), gap_id, 0, fb)
            self.hub.log('INFO' if fb.step_success else 'WARN', task, f'{fb.step} · 성공={fb.step_success}',
                         run_id=run.id)

        run.stage = task
        handle = wait(cli.send_goal_async(goal, feedback_callback=on_feedback))
        if not handle.accepted:
            return None
        run.goal_handle = handle
        if run.stop_requested:
            handle.cancel_goal_async()
        res = wait(handle.get_result_async()).result
        run.goal_handle = None
        return res
