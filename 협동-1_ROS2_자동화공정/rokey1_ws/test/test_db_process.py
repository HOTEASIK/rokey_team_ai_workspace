"""실제 MySQL(테스트 DB)로 HMI 공정 ↔ DB 왕복 검사 (로봇·ROS 통신 없음, 액션 서버는 가짜).

확인하는 것
  1. [전체 공정] → run / wall / gap / task_log 가 MySQL 에 맞는 값·타입으로 들어가는지
  2. DB 에서 읽은 값으로 공정이 도는지: [퍼티] / [샌딩] (gap_id 지정) → utils.db.gap_points 로 MySQL 에서 읽은
     틈·벽 좌표가 그대로 goal 로 가고, 실제 퍼티 변환 코드(action_protocol: 벽 법선·세로 틈 검사)를 통과하는지
  3. 없는 gap_id → E_NO_GAP, run 결과 기록

안전
  - 실제 공정 DB(robot.yaml db.database = cobot)에는 절대 안 씀: 테스트 DB 이름이 같으면 실행 거부
  - utils.db 의 pending/rejected 보관 파일을 임시 폴더로 바꿈 (실패한 테스트 SQL 이 나중에 실제 DB 로 재전송되지 않게)

테스트 DB 준비 (한 번만, 관리자 권한):
  sudo mysql -e "CREATE DATABASE IF NOT EXISTS cobot_test DEFAULT CHARACTER SET utf8mb4;
                 GRANT ALL PRIVILEGES ON cobot_test.* TO 'cobot'@'%'; FLUSH PRIVILEGES;"
실행:
  pytest test/test_db_process.py -v            # 테스트 DB 가 없으면 skip
  ROKEY_TEST_DB=<이름> pytest ...              # 다른 테스트 DB 이름
테이블은 매 실행 때 db/schema.sql 로 다시 만든다 (테스트 DB 안의 데이터는 지워짐).
"""
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
os.environ.setdefault('ROKEY_CONFIG', os.path.join(PKG, 'config', 'robot.yaml'))
sys.path[:0] = [PKG, os.path.join(HERE, 'full_process')]

import pymysql  # noqa: E402
import pytest  # noqa: E402

from fake_servers import FAKE_GAPS, FAKE_WALL  # noqa: E402
from test_full_process import ORDER, press_full_process  # noqa: E402
from utils import db as dbmod  # noqa: E402
from utils.config import CFG  # noqa: E402

TEST_DB = os.environ.get('ROKEY_TEST_DB', 'cobot_test')
REAL_DB = CFG['db']['database']
TABLES = ('quality', 'task_log', 'gap', 'wall', 'run')     # 외래키 역순


def _schema_statements():
    """db/schema.sql 에서 CREATE TABLE 문만 (CREATE DATABASE / USE 제외)."""
    sql = open(os.path.join(PKG, 'db', 'schema.sql')).read()
    sql = '\n'.join(line.split('--')[0] for line in sql.splitlines())
    return [s.strip() for s in sql.split(';') if re.match(r'\s*CREATE TABLE', s, re.I)]


@pytest.fixture(scope='module')
def mysql_db():
    if TEST_DB == REAL_DB:
        pytest.fail(f'테스트 DB 이름이 실제 DB({REAL_DB})와 같습니다. 실제 DB 에는 테스트하지 않습니다.')
    conf = dict(CFG['db'], database=TEST_DB)
    try:
        conn = pymysql.connect(**conf, autocommit=True)
    except pymysql.MySQLError as e:
        pytest.skip(f'테스트 DB {TEST_DB} 접속 불가 ({e}). 파일 위 docstring 의 준비 명령 실행')
    with conn.cursor() as cur:
        cur.execute('SELECT DATABASE()')
        assert cur.fetchone()[0] == TEST_DB
        for t in TABLES:
            cur.execute(f'DROP TABLE IF EXISTS {t}')
        for stmt in _schema_statements():
            cur.execute(stmt)
    yield conn, conf
    conn.close()


@pytest.fixture
def real_db(mysql_db, monkeypatch, tmp_path):
    """ProcessRunner 가 실제 utils.db.DB(MySQL) 를 테스트 DB 로 쓰게 한다."""
    conn, conf = mysql_db
    monkeypatch.setitem(CFG, 'db', conf)                   # utils.db 는 연결할 때 CFG['db'] 를 읽음
    monkeypatch.setattr(dbmod, 'LOG_DIR', str(tmp_path))
    monkeypatch.setattr(dbmod, 'PENDING', str(tmp_path / 'pending.jsonl'))
    monkeypatch.setattr(dbmod, 'REJECTED', str(tmp_path / 'rejected.jsonl'))
    import hmi.process_runner as pr
    monkeypatch.setattr(pr, 'DB', dbmod.DB)                 # test_full_process 가 바꿔 둔 가짜 DB 를 실제로

    def query(sql, args=()):
        with conn.cursor(pymysql.cursors.DictCursor) as cur:
            cur.execute(sql, args)
            return cur.fetchall()
    yield query
    assert not (tmp_path / 'pending.jsonl').exists(), 'DB 연결 실패로 보관된 쓰기가 있음'
    assert not (tmp_path / 'rejected.jsonl').exists(), \
        'DB 가 거부한 쓰기가 있음: ' + (tmp_path / 'rejected.jsonl').read_text()


def press(task_id='process_all', gap_id=None):
    params = {'purpose': 'TEST', 'material': 'putty', 'note': 'DB 왕복 테스트'}
    if gap_id is not None:
        params['gap_id'] = gap_id
    run, hub, _, servers = press_full_process(task_id=task_id, params=params, use_db=True)
    return run, hub, servers


def test_full_process_writes_mysql(real_db):
    run, hub, servers = press()
    assert run.state == 'done', hub.logs
    r = real_db('SELECT * FROM run ORDER BY run_id DESC LIMIT 1')[0]
    assert (r['task'], r['mode'], r['purpose'], r['result'], r['data_check']) == ('ALL', 'virtual', 'TEST', 'OK', 'OK')
    assert r['started_at'] and r['ended_at'] and r['ended_at'] >= r['started_at']

    w = real_db('SELECT * FROM wall WHERE run_id=%s', (r['run_id'],))
    assert len(w) == 1
    cols = [f'{p}_{a}' for p in ('top', 'floor', 'right', 'left') for a in 'xyz']
    assert [w[0][c] for c in cols] == FAKE_WALL                       # DOUBLE → 값 그대로

    g = real_db('SELECT * FROM gap WHERE wall_id=%s', (w[0]['wall_id'],))
    assert len(g) == 1
    assert [g[0][c] for c in ('start_x', 'start_y', 'start_z', 'end_x', 'end_y', 'end_z')] == FAKE_GAPS

    logs = real_db('SELECT * FROM task_log WHERE run_id=%s ORDER BY log_id', (r['run_id'],))
    assert [x['step'] for x in logs] == [f'{t.lower()}_fake' for t in ORDER]
    assert [x['task_code'] for x in logs] == ['EXPLORE', 'PUTTY', 'PUTTY', 'PUTTY', 'PUTTY', 'SANDING']
    gid = g[0]['gap_id']
    # 탐색·헤라 집기·반납은 틈과 무관 → NULL, 뜨기·도포·샌딩은 그 틈
    assert [x['gap_id'] for x in logs] == [None, None, gid, gid, None, gid]
    for x in logs:
        assert x['success'] == 1 and x['error_code'] is None
        assert json.loads(x['joint']) == [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        assert len(json.loads(x['task'])) == 6 and len(json.loads(x['force_val'])) == 6
        assert 'worker' in json.loads(x['params']) and x['duration_ms'] == 1234
    apply_log = logs[ORDER.index('PUTTY')]
    assert json.loads(apply_log['force_stats'])['n'] == 4 and 0.0 <= apply_log['contact_ratio'] <= 1.0
    assert logs[0]['contact_ratio'] is None and logs[0]['force_stats'] is None   # 힘제어 없는 단계 → NULL


def test_putty_and_sanding_use_values_read_from_mysql(real_db):
    """DB 에 직접 넣은 틈(소수점 값)을 [퍼티]/[샌딩] gap_id 로 지정 → MySQL 에서 읽은 값이 그대로 goal 로 간다."""
    real_db("INSERT INTO run (task, result) VALUES ('EXPLORE', 'OK')")
    run_id = real_db('SELECT LAST_INSERT_ID() AS id')[0]['id']
    wall = [718.37, 1.25, 290.5, 718.37, 1.25, 95.2, 718.37, -199.8, 190.1, 718.37, 200.3, 190.1]
    gap = [718.37, 12.5, 131.75, 718.37, 12.5, 248.25]
    real_db('INSERT INTO wall (run_id, top_x, top_y, top_z, floor_x, floor_y, floor_z, right_x, right_y, right_z,'
            ' left_x, left_y, left_z) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)', (run_id, *wall))
    wall_id = real_db('SELECT LAST_INSERT_ID() AS id')[0]['id']
    real_db('INSERT INTO gap (wall_id, start_x, start_y, start_z, end_x, end_y, end_z) VALUES (%s,%s,%s,%s,%s,%s,%s)',
            (wall_id, *gap))
    gap_id = real_db('SELECT LAST_INSERT_ID() AS id')[0]['id']

    run, hub, servers = press('process_putty', gap_id)
    assert run.state == 'done', hub.logs
    assert [s[0] for s in servers.goals] == ['PUTTY_PICKUP', 'PUTTY_SCOOP', 'PUTTY', 'PUTTY_RETURN']
    apply_goal = servers.goals[2]
    assert apply_goal[1] == gap_id and list(apply_goal[2]) == gap[:3] and list(apply_goal[3]) == gap[3:]
    args = dict(servers.commands)['apply']
    assert args[args.index('--start') + 1:args.index('--start') + 4] == [str(v) for v in gap[:3]]
    assert args[args.index('--end') + 1:args.index('--end') + 4] == [str(v) for v in gap[3:]]

    run, hub, servers = press('process_sanding', gap_id)
    assert run.state == 'done', hub.logs
    assert [s[0] for s in servers.goals] == ['SANDING'] and list(servers.goals[0][2]) == gap[:3]

    rows = real_db("SELECT task, gap_id, result FROM run WHERE task IN ('PUTTY', 'SANDING') ORDER BY run_id")
    assert [(x['task'], x['gap_id'], x['result']) for x in rows][-2:] == [('PUTTY', gap_id, 'OK'),
                                                                          ('SANDING', gap_id, 'OK')]


def test_unknown_gap_id_is_recorded(real_db):
    run, hub, servers = press('process_putty', 999999)
    assert run.state == 'failed' and servers.goals == []          # 로봇 동작 요청 없음
    r = real_db('SELECT * FROM run ORDER BY run_id DESC LIMIT 1')[0]
    assert (r['task'], r['gap_id'], r['result']) == ('PUTTY', 999999, 'E_NO_GAP')
