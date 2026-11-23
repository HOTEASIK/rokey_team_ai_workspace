#!/usr/bin/env python3
"""DB 저장 확인 (읽기 전용, SELECT 만). 탐색(EXPLORE/ALL) 실행 1번이 DB 에 잘 들어갔는지 본다.

  1. run      : 결과(result) / data_check
  2. wall     : 1줄 있는지, 12개 좌표가 비었는지
                + ~/wall_scan_data/explore_run<id>_*.json (wall_tap 원본) 에서 계산한 4점과 같은지
  3. gap      : 개수 (robot.yaml explore.gaps 와 같은지)
  4. task_log : 단계별 성공/실패
  5. 못 쓴 데이터: ~/ws_cobot_pjt/logs/pending.jsonl (연결 실패) / rejected.jsonl (데이터 오류)

실행:
  python3 ~/ws_cobot_pjt/ws_dsr/src/rokey1/analysis/check_db.py          # 가장 최근 탐색 run
  python3 ~/ws_cobot_pjt/ws_dsr/src/rokey1/analysis/check_db.py 12       # run_id 12
"""
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # rokey1/ (utils, navigation)

import pymysql

from utils.config import CFG
from utils.db import PENDING, REJECTED

COLS = ['top_x', 'top_y', 'top_z', 'floor_x', 'floor_y', 'floor_z',
        'right_x', 'right_y', 'right_z', 'left_x', 'left_y', 'left_z']


def main():
    conn = pymysql.connect(**CFG['db'], cursorclass=pymysql.cursors.DictCursor)
    cur = conn.cursor()
    q = lambda sql, *a: (cur.execute(sql, a), cur.fetchall())[1]
    ok = True

    if len(sys.argv) > 1:
        run_id = int(sys.argv[1])
    else:
        rows = q("SELECT run_id FROM run WHERE task IN ('EXPLORE','ALL') ORDER BY run_id DESC LIMIT 1")
        if not rows:
            print('[X] 탐색 run 이 없음')
            return 1
        run_id = rows[0]['run_id']

    run = q('SELECT * FROM run WHERE run_id=%s', run_id)
    if not run:
        print(f'[X] run {run_id} 없음')
        return 1
    r = run[0]
    print(f'[run {run_id}] task={r["task"]} mode={r["mode"]} result={r["result"]} '
          f'data_check={r["data_check"]} {r["started_at"]} ~ {r["ended_at"]}')
    ok &= r['result'] == 'OK'

    walls = q('SELECT * FROM wall WHERE run_id=%s', run_id)
    if len(walls) != 1:
        print(f'[X] wall {len(walls)}줄 (1줄이어야 함)')
        ok = False
    for w in walls:
        vals = [w[c] for c in COLS]
        empty = [c for c, v in zip(COLS, vals) if v is None]
        print(f'[wall {w["wall_id"]}] ' + ', '.join(f'{c}={v:.1f}' for c, v in zip(COLS, vals) if v is not None))
        if empty:
            print(f'[X] 빈 좌표: {empty}')
            ok = False
        files = sorted(glob.glob(os.path.expanduser(f'~/wall_scan_data/explore_run{run_id}_*.json')))
        if files:
            from navigation.explore_server import wall_corners
            with open(files[-1]) as f:
                ref = wall_corners(json.load(f))
            diff = max(abs(a - b) for a, b in zip(vals, ref)) if not empty else float('inf')
            print(f'[{"O" if diff < 0.01 else "X"}] 원본 JSON 과 최대 차이 {diff:.4f}mm ({os.path.basename(files[-1])})')
            ok &= diff < 0.01
        else:
            print('[-] 원본 JSON 없음 (다른 PC 에서 실행했으면 정상) → 비교 생략')

    gaps = q('SELECT g.* FROM gap g JOIN wall w ON g.wall_id=w.wall_id WHERE w.run_id=%s', run_id)
    want = len(CFG['explore']['gaps'] or [])
    print(f'[{"O" if len(gaps) == want else "X"}] gap {len(gaps)}개 (robot.yaml {want}개)')
    ok &= len(gaps) == want

    logs = q('SELECT task_code, step, success, error_code, duration_ms FROM task_log WHERE run_id=%s ORDER BY log_id',
             run_id)
    print(f'[task_log] {len(logs)}줄')
    for lg in logs:
        print(f'   {"O" if lg["success"] else "X"} {lg["task_code"]:8s} {lg["step"]:24s} '
              f'{lg["duration_ms"] or 0:>7}ms {lg["error_code"] or ""}')
    ok &= bool(logs) and all(lg['success'] for lg in logs)

    for path in (PENDING, REJECTED):
        n = sum(1 for _ in open(path)) if os.path.exists(path) else 0
        print(f'[{"O" if n == 0 else "X"}] {os.path.basename(path)} {n}줄')
        ok &= n == 0

    print('==> 정상' if ok else '==> 확인 필요 ([X] 항목)')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
