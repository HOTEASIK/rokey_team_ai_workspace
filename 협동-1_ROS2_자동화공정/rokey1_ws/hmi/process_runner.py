"""웹 시작 버튼 → /process/* 액션 호출 → DB 기록.

  ALL     : EXPLORE(외부 탐색) → INNER_EXPLORE(내부 탐색) → 찾은 틈마다 PUTTY → 틈마다 SANDING
            start_stage (웹 [시작 단계]) 0 처음부터 / 1 내부 탐색부터 / 2 퍼티부터 / 3 샌딩부터
            1 이상이면 외부 탐색을 건너뛰고 DB 의 마지막 벽(wall)과 그 틈(gap)을 쓴다.
  EXPLORE (외부 탐색) : /process/explore (explore_server → pick_tap/wall_tap) → 벽 4점 + robot.yaml explore.gaps
  INNER_EXPLORE (내부 탐색) : robot.yaml inner_explore.enabled=false 면 건너뜀,
            true 면 /process/inner_explore (wall_scan_server) 에 goal(외부 탐색 벽 4점) → result.gaps 를 DB 저장.
            처음부터(0) 실행이면 외부 탐색의 손 입력 틈(explore.gaps)은 DB 에 넣지 않고 내부 탐색 결과만 넣는다.
            [예외] 내부 탐색이 틈 없음(gaps=[])으로 판단 → 서버가 홈으로 복귀한 뒤, 퍼티·샌딩 없이
            INFO '틈 없음' 으로 종료 (run.result = NO_GAP, 정상 종료).
  PUTTY = /process/putty_pickup (헤라 집기 1회) → 틈마다 [/process/putty_scoop (퍼티 뜨기) → /process/putty (도포)]
          → /process/putty_return (헤라 반납 1회, 샌딩이 그리퍼로 도구를 집기 전에 비워야 함)
  EXPLORE : 외부 탐색만 (wall / gap 저장)
  PUTTY / SANDING : 웹에서 지정한 gap_id 한 틈만 (좌표는 DB gap 에서 읽음)

액션 피드백 1번 = task_log 1줄. 실패하면 그 자리에서 중단 (재시도 없음).
끝나면 run.result(OK / NO_GAP / STOP / ESTOP / E_xxx) + run.data_check 기록.
  NO_GAP = 내부 탐색이 틈을 못 찾아 홈 복귀 후 종료 (오류 아님).
hmi_server 가 실행 도중 꺼지면 다음 시작 때 그 run 을 ABORTED 로 마감한다.
"""
import os
import subprocess
import time
from types import SimpleNamespace

import yaml
from rclpy.action import ActionClient

from hmi.tasks import inner_explore_on, start_stage
from rokey_interfaces.action import Process
from utils import config
from utils.db import DB

NORMAL_END = ('OK', 'NO_GAP')     # 정상 종료 결과 (NO_GAP: 내부 탐색 틈 없음 → 이후 작업 없이 종료)
TASKS = ('EXPLORE', 'INNER_EXPLORE', 'PUTTY_PICKUP', 'PUTTY_SCOOP', 'PUTTY', 'PUTTY_RETURN', 'SANDING')
STAGE_NAMES = ('처음부터 (외부 탐색)', '내부 탐색부터', '퍼티 작업부터', '샌딩 작업부터')


def db_task(task):
    """task_log.task_code: 퍼티 집기/뜨기/반납도 PUTTY 로 묶는다 (step 으로 구분됨)"""
    return 'PUTTY' if task.startswith('PUTTY') else 'SANDING' if task.startswith('SANDING') else task


def action_of(task):
    """보낼 /process/* 액션. SANDING_KEEP(도구 들고 다음 틈으로)은 /process/sanding 에 task_code 만 다르게"""
    return 'SANDING' if task == 'SANDING_KEEP' else task


def code_version():
    try:
        return subprocess.check_output(
            ['git', 'describe', '--always', '--dirty'], text=True,
            cwd=os.path.dirname(config.PATH), stderr=subprocess.DEVNULL).strip()
    except (OSError, subprocess.CalledProcessError):
        return ''


def wait(future, gone=None):
    """executor 가 다른 스레드에서 spin 중이므로 완료까지 대기만 한다.
    gone(): 액션 서버가 사라졌는지. 3 s 계속 사라져 있으면 None (서버가 죽으면 결과가 영영 안 옴)."""
    lost = None
    while not future.done():
        if gone and gone():
            lost = lost or time.time()
            if time.time() - lost > 3.0:
                return None
        else:
            lost = None
        time.sleep(0.05)
    return future.result()


class ProcessRunner:
    def __init__(self, node, hub, db=None):
        self.hub = hub
        self.db = db or DB()     # db: 가짜 DB (hmi_server -p fake_db:=<json>)
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
        stage = start_stage(p) if task == 'ALL' else 0
        note = p['note'] or None
        if stage:      # DB run 에 어느 단계부터 했는지 남김 (run.task 는 ALL 그대로)
            note = f'[start={stage}] {note or ""}'.strip()[:200]
        run_id = self.db.start_run(task, gap_id, run.mode, p['purpose'], p['material'], note, self.version)
        if run_id is None:
            self.hub.log('ERROR', 'DB', 'run 저장 실패 → DB 연결 확인 (~/ws_cobot_pjt/logs/pending.jsonl)',
                         run_id=run.id)
            return 'E_DB'
        self.hub.log('INFO', 'DB', f'DB run_id={run_id} · {task}', run_id=run.id)

        result = self._steps(run, run_id, task, gap_id, stage)
        if run.stop_requested:
            result = 'ESTOP' if is_estop() else 'STOP'
        check = self.db.check_run(run_id, task, explored=stage == 0, need_gap=result != 'NO_GAP')
        self.db.end_run(run_id, result, check)
        self.hub.log('INFO' if result in NORMAL_END else 'WARN', 'DB',
                     f'DB run_id={run_id} 결과={result} · 데이터={check}', run_id=run.id)
        return result

    def _steps(self, run, run_id, task, gap_id, stage=0):
        if task in ('PUTTY', 'SANDING'):
            row = self.db.gap_points(gap_id)
            if row is None:
                self.hub.log('ERROR', 'DB', f'gap_id={gap_id} 가 DB에 없습니다', run_id=run.id)
                return 'E_NO_GAP'
            return self._each(run, run_id, [task], [(gap_id, row[0])], row[1])

        if task == 'ALL':
            self.hub.log('INFO', 'HMI', f'전체 공정 시작 단계 {stage}: {STAGE_NAMES[stage]}', run_id=run.id)
        if stage == 0:
            code, res = self._do(run, run_id, 'EXPLORE', -1, [0.0] * 6)
            if code != 'OK':
                return code
            wall = list(res.wall)
            wall_id = self.db.insert_wall(run_id, wall)
            explore_gaps = list(res.gaps)
            if task == 'ALL' and inner_explore_on():
                if wall_id is None:         # 벽 저장 실패면 내부 탐색(로봇 이동) 전에 멈춤
                    self.hub.log('ERROR', 'DB', 'wall 저장 실패 → 내부 탐색·퍼티·샌딩 중단 (DB 연결 확인)', run_id=run.id)
                    return 'E_DB'
                gaps = explore_gaps         # 아직 저장 안 함 (내부 탐색 결과가 우선, _inner_explore 에서 결정)
            else:
                gaps = self._save_gaps(run, wall_id, explore_gaps)
                if gaps is None:
                    return 'E_DB'
                if task == 'EXPLORE':
                    return 'OK'
        else:
            last = self.db.latest_wall()
            if last is None:
                self.hub.log('ERROR', 'DB', '저장된 벽(wall)이 없습니다 → 시작 단계 0 (외부 탐색부터) 으로 실행하세요',
                             run_id=run.id)
                return 'E_NO_WALL'
            wall_id, wall_run, wall, db_gaps = last
            # 틈 좌표: robot.yaml explore.gaps 가 먼저. 비어 있으면 DB (그 벽에 저장된 틈)
            try:
                manual = self._manual_gaps()
            except ValueError as e:
                self.hub.log('ERROR', 'HMI', str(e), run_id=run.id)
                return 'E_NO_GAP'
            if manual:
                gaps = self._match_gaps(run, wall_id, db_gaps, manual)
                if gaps is None:
                    return 'E_DB'
                src = f'robot.yaml explore.gaps {len(gaps)}개'
            else:
                gaps = db_gaps
                src = f'robot.yaml explore.gaps 비어 있음 → DB 틈 {len(gaps)}개'
            self.hub.log('INFO', 'DB', f'외부 탐색 건너뜀 → 마지막 벽 wall_id={wall_id} (run_id={wall_run}) · {src} · '
                                       f'gap_id={[gid for gid, _ in gaps]} · '
                                       f'좌표={[[round(v, 1) for v in pts] for _, pts in gaps]}', run_id=run.id)

        if stage <= 1:
            gaps = self._inner_explore(run, run_id, wall_id, wall, gaps)
            if isinstance(gaps, str):
                return gaps                 # 오류 코드
        if not gaps:
            self.hub.log('ERROR', 'HMI', '틈(gap)이 없습니다 → 퍼티·샌딩 안 함 (헤라를 집지 않음). '
                                         'robot.yaml explore.gaps 도 비어 있고 DB 에도 틈 없음 → explore.gaps 를 넣으세요', run_id=run.id)
            return 'E_NO_GAP'
        return self._each(run, run_id, ['PUTTY', 'SANDING'] if stage <= 2 else ['SANDING'], gaps, wall)

    @staticmethod
    def _manual_gaps():
        """robot.yaml explore.gaps → [sx,sy,sz, ex,ey,ez, ...] 한 줄로 (explore_server.manual_gaps 와 같은 규칙)"""
        try:            # hmi_server 를 다시 켜지 않아도 고친 값이 바로 쓰이게 파일을 새로 읽음
            with open(config.PATH) as f:
                cfg = yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError) as e:
            raise ValueError(f'robot.yaml 읽기 실패: {e}') from None
        out = []
        for g in (cfg.get('explore') or {}).get('gaps') or []:
            if len(g) != 6:
                raise ValueError(f'robot.yaml explore.gaps 는 [sx,sy,sz, ex,ey,ez] 6개씩: {g}')
            out += [float(v) for v in g]
        return out

    def _match_gaps(self, run, wall_id, db_gaps, flat):
        """yaml 틈을 그 벽의 DB 틈과 맞춤: 좌표가 같은(0.05 mm) DB 틈이 있으면 그 gap_id, 없으면 새로 저장.
        (매 실행마다 같은 틈이 DB 에 쌓이지 않게)"""
        out, new = [], []
        for i in range(0, len(flat), 6):
            pts = flat[i:i + 6]
            gid = next((g for g, p in db_gaps if all(abs(float(a) - b) < 0.05 for a, b in zip(p, pts))), None)
            if gid is None:
                new.append(len(out))
            out.append((gid, pts))
        if new:
            saved = self._save_gaps(run, wall_id, [v for k in new for v in out[k][1]])
            if saved is None:
                return None
            for k, (gid, _) in zip(new, saved):
                out[k] = (gid, out[k][1])
        return out

    def _save_gaps(self, run, wall_id, g):
        """틈 좌표 [sx,sy,sz, ex,ey,ez, ...] → DB gap 저장. 반환: [(gap_id, 6점), ...] 또는 None (DB 실패)"""
        gaps = [(self.db.insert_gap(wall_id, g[i:i + 6]), g[i:i + 6]) for i in range(0, len(g), 6)] if wall_id else []
        if wall_id is None or any(gid is None for gid, _ in gaps):
            self.hub.log('ERROR', 'DB', 'wall/gap 저장 실패 → 퍼티·샌딩 중단 (DB 연결 확인)', run_id=run.id)
            return None
        self.hub.log('INFO', 'DB', f'wall_id={wall_id} · gap_id={[gid for gid, _ in gaps]}', run_id=run.id)
        return gaps

    def _inner_explore(self, run, run_id, wall_id, wall, gaps):
        """내부 탐색 /process/inner_explore (goal.wall = 외부 탐색 벽 4점).
        gaps: [(gap_id, 6점), ...] (이미 DB 에 있는 틈) 또는 [sx,sy,sz, ex,ey,ez, ...] (외부 탐색 손 입력 틈, 아직 저장 전)
        반환: 이후 단계에 쓸 [(gap_id, 6점), ...] 또는 오류 코드 문자열."""
        unsaved = bool(gaps) and not isinstance(gaps[0], tuple)

        def keep():                         # 기존 틈 사용 (손 입력 틈이면 이제 저장)
            if not unsaved:
                return gaps
            saved = self._save_gaps(run, wall_id, gaps)
            return 'E_DB' if saved is None else saved
        if not inner_explore_on():
            self.hub.log('WARN', 'INNER_EXPLORE', '내부 탐색 꺼짐 (robot.yaml inner_explore.enabled=false) '
                                                  '→ 건너뜀 (기존 틈 그대로 사용)', run_id=run.id)
            return keep()
        code, res = self._do(run, run_id, 'INNER_EXPLORE', -1, [0.0] * 6, wall)
        if code != 'OK':
            return code
        found = list(res.gaps)
        if len(found) % 6:
            self.hub.log('ERROR', 'INNER_EXPLORE', f'result.gaps 가 6개씩이 아님 ({len(found)}개) → 중단', run_id=run.id)
            return 'E_INNER_EXPLORE'
        if not found:               # [예외] 틈 없음: 서버가 홈 복귀까지 마침 → 이후 작업 없이 종료
            self.hub.log('INFO', 'INNER_EXPLORE', '틈 없음 → 홈 복귀 완료, 이후 작업(퍼티·샌딩) 없이 종료',
                         run_id=run.id)
            return 'NO_GAP'
        self.hub.log('INFO', 'INNER_EXPLORE', f'내부 탐색: 틈 {len(found) // 6}개 '
                                              f'{[[round(v, 1) for v in found[i:i + 6]] for i in range(0, len(found), 6)]}',
                     run_id=run.id)
        new = self._save_gaps(run, wall_id, found)
        return 'E_DB' if new is None else new

    def _each(self, run, run_id, tasks, gaps, wall):
        """wall: 틈이 있는 벽 4점 (퍼티/샌딩 goal 에 같이 보냄 → 샌딩이 벽 법선 계산)"""
        z6 = [0.0] * 6
        for task in tasks:
            steps = [(task, gid, pts) for gid, pts in gaps]
            if task == 'PUTTY':     # 헤라 집기 1회 → 틈마다 뜨기·도포 → 반납 1회
                steps = ([('PUTTY_PICKUP', -1, z6)]
                         + [s for gid, pts in gaps for s in (('PUTTY_SCOOP', gid, z6), ('PUTTY', gid, pts))]
                         + [('PUTTY_RETURN', -1, z6)])
            elif task == 'SANDING':  # 도구 집기 1회(첫 틈) → 틈마다 샌딩, 사이는 들고 이동 → 마지막 틈 뒤 반납
                steps = [('SANDING_KEEP' if i < len(gaps) - 1 else 'SANDING', gid, pts)
                         for i, (gid, pts) in enumerate(gaps)]
            for t, gid, pts in steps:
                code, _ = self._do(run, run_id, t, gid or -1, pts, wall)
                if code != 'OK':
                    return code     # 실패하면 그 자리에서 중단 (헤라를 쥐고 있을 수 있음 → 사람이 확인)
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
        reason = res.message if res else f'/process/{action_of(task).lower()} 액션 서버 없음'
        z6 = [0.0] * 6
        self.db.log_step(run_id, db_task(task), gap_id, 0, SimpleNamespace(
            step=f'{task.lower()}_result', step_success=False, error_code=code, fail_reason=reason,
            joint=z6, task=z6, force=z6, duration_ms=0,
            params_json='', contact_ratio=-1.0, force_stats_json=''))
        self.hub.log('ERROR', task, f'{code}: {reason}', run_id=run.id)
        return code, res

    def _send(self, run, run_id, task, gap_id, pts, wall=None):
        cli = self.clients[action_of(task)]
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
        handle = wait(cli.send_goal_async(goal, feedback_callback=on_feedback), gone=lambda: not cli.server_is_ready())
        if handle is None or not handle.accepted:
            return None
        run.goal_handle = handle
        if run.stop_requested:
            handle.cancel_goal_async()
        res = wait(handle.get_result_async(), gone=lambda: not cli.server_is_ready())
        run.goal_handle = None
        return res.result if res else None
