"""MySQL 접근 모듈. hmi_server 만 이 모듈로 DB에 쓴다.
HTTP 스레드 / 공정 실행 스레드 / 액션 피드백 콜백이 같은 연결을 쓰므로 쿼리마다 락을 건다
(락 없이 동시에 쓰면 pymysql "Packet sequence number wrong" 오류).

무조건 DB에 남기기:
  - DB 연결 실패  → ~/ws_cobot_pjt/logs/pending.jsonl 에 보관, 다음 쓰기 성공 때 재전송
  - 데이터 오류(잘못된 JSON 등) → ~/ws_cobot_pjt/logs/rejected.jsonl 에 보관 (재전송 안 함, 사람이 확인)
"""
import json
import os
import threading
from datetime import datetime, timedelta

import pymysql

from utils.config import CFG

LOG_DIR = os.path.expanduser('~/ws_cobot_pjt/logs')
PENDING = os.path.join(LOG_DIR, 'pending.jsonl')
REJECTED = os.path.join(LOG_DIR, 'rejected.jsonl')


def _append(path, rows):
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(path, 'a') as f:
        f.writelines(json.dumps(r, default=str) + '\n' for r in rows)


def _json(s):
    """빈 문자열은 NULL"""
    return s if s else None


class DB:
    def __init__(self):
        self.conn = None
        self.lock = threading.Lock()

    def _exec(self, sql, args=(), fetch=False):
        with self.lock:
            if self.conn is None or not self.conn.open:
                self.conn = pymysql.connect(**CFG['db'], autocommit=True)
            with self.conn.cursor() as cur:
                cur.execute(sql, args)
                return cur.fetchone() if fetch else cur.lastrowid

    def _write(self, sql, args):
        """성공하면 row id, 실패하면 None (파일에 보관)."""
        try:
            row_id = self._exec(sql, args)
        except pymysql.OperationalError:          # 연결 문제 → 나중에 재전송
            self.conn = None
            _append(PENDING, [[sql, list(args)]])
            return None
        except pymysql.MySQLError as e:           # 데이터 문제 → 따로 보관
            _append(REJECTED, [[sql, list(args), str(e)]])
            return None
        self._flush()
        return row_id

    def _flush(self):
        if not os.path.exists(PENDING):
            return
        with open(PENDING) as f:
            rows = [json.loads(line) for line in f]
        os.remove(PENDING)
        for i, (sql, args) in enumerate(rows):
            try:
                self._exec(sql, args)
            except pymysql.OperationalError:
                _append(PENDING, rows[i:])
                return
            except pymysql.MySQLError as e:
                _append(REJECTED, [[sql, args, str(e)]])

    # ── 실행 ──────────────────────────────────────────
    def start_run(self, task, gap_id, mode, purpose, material, note, code_version):
        """run 1줄 INSERT. 반환값이 run_id (DB 연결 실패면 None)."""
        return self._write(
            "INSERT INTO run (task, gap_id, mode, purpose, material, note, code_version, started_at)"
            " VALUES (%s,%s,%s,%s,%s,%s,%s,%s)",
            (task, gap_id, mode, purpose, material, note, code_version, datetime.now()))

    def abort_open_runs(self):
        """hmi_server 가 실행 도중 꺼져서 끝나지 않은 run 을 ABORTED 로 마감. 마감한 개수 반환."""
        with self.lock:
            if self.conn is None or not self.conn.open:
                self.conn = pymysql.connect(**CFG['db'], autocommit=True)
            with self.conn.cursor() as cur:
                return cur.execute("UPDATE run SET ended_at=%s, result='ABORTED' WHERE result IS NULL",
                                   (datetime.now(),))

    def end_run(self, run_id, result, data_check):
        self._write("UPDATE run SET ended_at=%s, result=%s, data_check=%s WHERE run_id=%s",
                    (datetime.now(), result, data_check, run_id))

    def gap_points(self, gap_id):
        """([sx,sy,sz, ex,ey,ez], 벽 4점 12개) 또는 None (없는 gap_id)."""
        row = self._exec("SELECT g.start_x, g.start_y, g.start_z, g.end_x, g.end_y, g.end_z,"
                         " w.top_x, w.top_y, w.top_z, w.floor_x, w.floor_y, w.floor_z,"
                         " w.right_x, w.right_y, w.right_z, w.left_x, w.left_y, w.left_z"
                         " FROM gap g JOIN wall w ON w.wall_id=g.wall_id WHERE g.gap_id=%s", (gap_id,), fetch=True)
        if not row:
            return None
        row = [float(v) for v in row]
        return row[:6], row[6:]

    def latest_wall(self):
        """가장 최근 탐색한 벽과 그 틈: (wall_id, run_id, 벽 4점 12개, [(gap_id, [sx,sy,sz, ex,ey,ez]), ...])
        또는 None (벽 없음). [전체 공정] 시작 단계 1~3 이 탐색 대신 쓴다."""
        row = self._exec("SELECT wall_id, run_id, top_x, top_y, top_z, floor_x, floor_y, floor_z,"
                         " right_x, right_y, right_z, left_x, left_y, left_z"
                         " FROM wall ORDER BY wall_id DESC LIMIT 1", fetch=True)
        if not row:
            return None
        with self.lock:
            with self.conn.cursor() as cur:
                cur.execute("SELECT gap_id, start_x, start_y, start_z, end_x, end_y, end_z"
                            " FROM gap WHERE wall_id=%s ORDER BY gap_id", (row[0],))
                gaps = [(int(g[0]), [float(v) for v in g[1:]]) for g in cur.fetchall()]
        return int(row[0]), row[1], [float(v) for v in row[2:]], gaps

    def check_run(self, run_id, task, explored=True):
        """필수 데이터가 들어갔는지만 확인. 'OK' 또는 빠진 항목.
        explored=False: [전체 공정] 을 탐색 뒤 단계부터 시작 → 이 run 의 wall/gap 은 없는 게 정상."""
        missing = []
        if not self._exec("SELECT 1 FROM task_log WHERE run_id=%s LIMIT 1", (run_id,), fetch=True):
            missing.append('task_log 없음')
        if task in ('ALL', 'EXPLORE') and explored:
            if not self._exec("SELECT 1 FROM wall w JOIN gap g ON g.wall_id=w.wall_id"
                              " WHERE w.run_id=%s LIMIT 1", (run_id,), fetch=True):
                missing.append('wall/gap 없음')
        return ', '.join(missing) or 'OK'

    # ── 결과 ──────────────────────────────────────────
    def insert_wall(self, run_id, w):
        """w: [top xyz, floor xyz, right xyz, left xyz] 12개"""
        return self._write(
            "INSERT INTO wall (run_id, top_x, top_y, top_z, floor_x, floor_y, floor_z,"
            " right_x, right_y, right_z, left_x, left_y, left_z)"
            " VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)", (run_id, *w))

    def insert_gap(self, wall_id, g):
        """g: [sx,sy,sz, ex,ey,ez]"""
        return self._write(
            "INSERT INTO gap (wall_id, start_x, start_y, start_z, end_x, end_y, end_z)"
            " VALUES (%s,%s,%s,%s,%s,%s,%s)", (wall_id, *g))

    def log_step(self, run_id, task_code, gap_id, attempt, fb):
        """fb: Process.action Feedback (또는 같은 필드를 가진 객체)"""
        now = datetime.now()
        self._write(
            "INSERT INTO task_log (run_id, task_code, step, attempt, success, error_code,"
            " fail_reason, gap_id, joint, task, force_val, params, contact_ratio, force_stats,"
            " started_at, duration_ms) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)",
            (run_id, task_code, fb.step, attempt, fb.step_success, fb.error_code or None,
             (fb.fail_reason or "")[:100], gap_id if gap_id > 0 else None,
             json.dumps(list(fb.joint)), json.dumps(list(fb.task)), json.dumps(list(fb.force)),
             _json(fb.params_json), fb.contact_ratio if fb.contact_ratio >= 0 else None,
             _json(fb.force_stats_json), now - timedelta(milliseconds=fb.duration_ms),
             fb.duration_ms))
