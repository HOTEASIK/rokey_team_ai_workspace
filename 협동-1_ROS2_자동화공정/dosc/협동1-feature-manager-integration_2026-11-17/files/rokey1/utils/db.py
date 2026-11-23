"""MySQL 접근 모듈. 매니저만 이 모듈로 DB에 쓴다 (command 상태 갱신 포함).
웹이 쓰는 곳: command INSERT, quality INSERT/UPDATE.

무조건 DB에 남기기:
  - DB 연결 실패  → ~/ws_cobot_pjt/logs/pending.jsonl 에 보관, 다음 쓰기 성공 때 재전송
  - 데이터 오류(잘못된 JSON 등) → ~/ws_cobot_pjt/logs/rejected.jsonl 에 보관 (재전송 안 함, 사람이 확인)
"""
import json
import os
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

    def _exec(self, sql, args=(), fetch=False):
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

    # ── 명령 / 실행 ───────────────────────────────────
    def fetch_command(self):
        """가장 오래된 PENDING 명령 (cmd_id, cmd, purpose, material, note) 또는 None."""
        try:
            return self._exec(
                "SELECT cmd_id, cmd, purpose, material, note FROM command "
                "WHERE status='PENDING' ORDER BY cmd_id LIMIT 1", fetch=True)
        except pymysql.MySQLError:
            self.conn = None
            return None

    def set_command(self, cmd_id, status):
        self._write("UPDATE command SET status=%s WHERE cmd_id=%s", (status, cmd_id))

    def start_run(self, run_id, purpose, material, note, code_version):
        self._write(
            "INSERT INTO run (run_id, purpose, material, note, code_version, started_at)"
            " VALUES (%s,%s,%s,%s,%s,%s)",
            (run_id, purpose, material, note, code_version, datetime.now()))

    def end_run(self, run_id, result):
        self._write("UPDATE run SET ended_at=%s, result=%s WHERE run_id=%s",
                    (datetime.now(), result, run_id))

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
             fb.fail_reason, gap_id if gap_id > 0 else None,
             json.dumps(list(fb.joint)), json.dumps(list(fb.task)), json.dumps(list(fb.force)),
             _json(fb.params_json), fb.contact_ratio if fb.contact_ratio >= 0 else None,
             _json(fb.force_stats_json), now - timedelta(milliseconds=fb.duration_ms),
             fb.duration_ms))
