"""HMI [공정] 탭의 [전체 공정] ▶ 시작 을 그대로 흉내 내는 테스트 (로봇·ROS 통신·MySQL 없음).

흐름: TaskManager.start('process_all')  ← 웹 POST /api/start 와 같은 호출
      → ProcessRunner.run → /process/explore
      → /process/putty_pickup (헤라 집기 1회) → 틈마다 [putty_scoop (뜨기) → putty (도포)] → putty_return (반납 1회)
      → 틈마다 /process/sanding → DB(run / wall / gap / task_log) 기록
바꾼 것: DB → manager/fake_db.py (sodvir 테스트도 같은 것 사용),  ActionClient → test/full_process/fake_servers.py
         (fake 서버는 실제 putty_demo.action_protocol 로 goal 을 검사 → robot.yaml 값이 맞는지도 확인됨)

실행:
  pytest test/test_full_process.py -v            # 테스트
  python3 test/test_full_process.py              # 단계별 결과 + DB 로 갔을 데이터 출력
                                                 #   (test/full_process/db_full_process.json 에 저장)
※ 실제 /process/* 서버로는 아무것도 보내지 않는다 (ActionClient 를 가짜로 바꿈).
"""
import copy
import os
import sys
import time
from types import SimpleNamespace

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
os.environ.setdefault('ROKEY_CONFIG', os.path.join(PKG, 'config', 'robot.yaml'))   # 이 폴더의 robot.yaml
sys.path[:0] = [PKG, os.path.join(HERE, 'full_process')]

import pytest  # noqa: E402

import fake_servers  # noqa: E402
from manager.fake_db import FakeDB  # noqa: E402
from fake_servers import FAKE_GAPS, FakeActionClient, FakeServers  # noqa: E402
from hmi import hmi_server, process_runner  # noqa: E402
from utils.config import CFG  # noqa: E402

PARAMS = {'purpose': 'TEST', 'material': 'putty', 'note': '전체공정 가짜 테스트'}
ORDER = ['EXPLORE', 'PUTTY_PICKUP', 'PUTTY_SCOOP', 'PUTTY', 'PUTTY_RETURN', 'SANDING']


class Hub:
    def __init__(self):
        self.logs = []

    def log(self, level, src, msg, **kw):
        self.logs.append((level, src, msg))

    def publish(self, *a, **kw):
        pass


class Ros:
    def set_operation_speed(self, pct):
        pass

    def ensure_virtual_tcp(self):   # 가상 모드 공정 전 에뮬레이터 TCP 등록 (로봇 없는 테스트에서는 할 일 없음)
        pass


def press_full_process(settings=CFG, task_id='process_all', params=PARAMS, use_db=False, db=None):
    """[전체 공정] 버튼 1번. 반환: (run, hub, db, servers). use_db=True 면 DB 는 바꾸지 않음 (test_db_process: MySQL)
    db: 미리 벽/틈을 넣어 둔 FakeDB (시작 단계 1~3 테스트)"""
    db, servers = db or FakeDB(), FakeServers(settings)
    if not use_db:
        process_runner.DB = lambda: db
    process_runner.ActionClient = FakeActionClient
    FakeActionClient.servers = servers
    hub = Hub()
    manager = hmi_server.TaskManager(hub, Ros())
    manager.runner = process_runner.ProcessRunner(SimpleNamespace(cb=None), hub)
    run = manager.start(task_id, 'virtual', params)
    t0 = time.time()
    while run.active and time.time() - t0 < 10:
        time.sleep(0.02)
    return run, hub, db, servers


def test_full_process_runs_all_steps_in_order():
    """지금 robot.yaml 로 탐색 → 헤라 집기 → 뜨기 → 도포 → 반납 → 샌딩 이 순서대로 끝나고 DB 가 채워진다."""
    run, hub, db, servers = press_full_process()
    assert run.state == 'done', hub.logs
    assert [g[0] for g in servers.goals] == ORDER
    # 탐색이 준 틈 좌표가 도포·샌딩 goal 로 그대로 간다
    for i in (ORDER.index('PUTTY'), ORDER.index('SANDING')):
        assert list(servers.goals[i][2:]) == [FAKE_GAPS[:3], FAKE_GAPS[3:]]
    # robot.yaml 값이 워커 인자로 간다
    cmd = dict(servers.commands)
    scoop = CFG['force']['putty_scoop']
    assert cmd['scoop'][cmd['scoop'].index('--scoop-angle') + 1] == str(scoop['scoop_angle'])
    assert cmd['scoop'][cmd['scoop'].index('--transfer-z') + 1] == str(CFG['putty']['holder_transfer_z'])
    assert cmd['pickup'][cmd['pickup'].index('--joint-speed') + 1] == str(CFG['putty']['pickup']['joint_speed'])
    # DB
    assert db.runs[0]['process'] == 'ALL' and db.runs[0]['result'] == 'OK' and db.runs[0]['data_check'] == 'OK'
    assert len(db.walls) == 1 and len(db.gaps) == 1
    assert [r['task_code'] for r in db.task_log] == ['EXPLORE', 'PUTTY', 'PUTTY', 'PUTTY', 'PUTTY', 'SANDING']
    assert all(r['success'] for r in db.task_log)


def test_scoop_without_pickup_is_rejected():
    """헤라 집기 없이 뜨기만 오면 E_GRIP (실물 모드 파지 확인 = 같은 서버에서 집기 성공)."""
    servers = FakeServers(CFG)
    goal = SimpleNamespace(task_code='PUTTY_SCOOP', gap_id=1, start_point=[0.0] * 3, end_point=[0.0] * 3,
                           wall=[0.0] * 12)
    result, _ = servers.handle(goal)
    assert not result.success and result.error_code == 'E_GRIP'


def test_missing_putty_setting_stops_before_motion():
    """robot.yaml 퍼티 값이 비면 첫 퍼티 단계에서 거부 (로봇 동작 전)."""
    settings = copy.deepcopy(CFG)
    settings['putty']['pickup']['joint_speed'] = None
    run, hub, db, servers = press_full_process(settings)
    assert run.state == 'failed'
    assert [g[0] for g in servers.goals] == ['EXPLORE', 'PUTTY_PICKUP']
    assert db.runs[0]['result'] == 'E_FORCE'


def test_full_process_rejects_wall_not_facing_x():
    """탐색이 +X 가 아닌 벽을 주면 도포가 E_NO_WALL 로 거부 (merge 때 다시 넣은 벽 법선 검사)."""
    old = fake_servers.FAKE_WALL
    fake_servers.FAKE_WALL = [0.0, 600.0, 400.0, 0.0, 600.0, 35.0, 300.0, 600.0, 200.0, -300.0, 600.0, 200.0]   # +Y 벽
    try:
        run, hub, db, servers = press_full_process()
    finally:
        fake_servers.FAKE_WALL = old
    assert run.state == 'failed'
    assert db.runs[0]['result'] == 'E_NO_WALL'
    assert [g[0] for g in servers.goals][-1] == 'PUTTY'       # 도포에서 멈춤 → 반납 안 함 (헤라 쥔 채, 사람이 확인)



def _db_with_wall():
    """지난 탐색 결과(벽 1개 + 틈 1개)가 DB 에 있는 상태"""
    db = FakeDB()
    wall_id = db.insert_wall(99, fake_servers.FAKE_WALL)
    db.insert_gap(wall_id, FAKE_GAPS)
    return db


@pytest.mark.parametrize('stage, order', [
    ('1: 내부 탐색부터', ORDER[1:]),           # 내부 탐색은 기능 없음 → 건너뜀
    ('2: 퍼티 작업부터', ORDER[1:]),
    ('3: 샌딩 작업부터', ['SANDING']),
])
def test_start_stage_skips_explore_and_uses_last_wall(stage, order, monkeypatch):
    """시작 단계 1~3: 탐색 없이 DB 마지막 벽/틈으로 그 단계부터 (robot.yaml explore.gaps 비어 있을 때)."""
    monkeypatch.setattr(process_runner.ProcessRunner, '_manual_gaps', staticmethod(lambda: []))
    db = _db_with_wall()
    run, hub, db, servers = press_full_process(params=dict(PARAMS, start_stage=stage), db=db)
    assert run.state == 'done', hub.logs
    assert [g[0] for g in servers.goals] == order
    assert list(servers.goals[-1][2:]) == [FAKE_GAPS[:3], FAKE_GAPS[3:]]
    r = db.runs[0]
    assert r['result'] == 'OK' and r['data_check'] == 'OK' and r['note'].startswith(f'[start={stage[0]}]')
    assert len(db.walls) == 1                                    # 새 벽을 만들지 않음
    assert any(src == 'INNER_EXPLORE' for _, src, _ in hub.logs) == (stage[0] == '1')


def test_start_stage_uses_yaml_gaps_when_last_wall_has_none(monkeypatch):
    """robot.yaml explore.gaps 가 있으면 그것을 그 벽의 틈으로 저장하고 진행 (run92: E_NO_GAP)."""
    monkeypatch.setattr(process_runner.ProcessRunner, '_manual_gaps', staticmethod(lambda: list(FAKE_GAPS)))
    db = FakeDB()
    db.insert_wall(99, fake_servers.FAKE_WALL)
    run, hub, db, servers = press_full_process(params=dict(PARAMS, start_stage='2: 퍼티 작업부터'), db=db)
    assert run.state == 'done', hub.logs
    assert [g[0] for g in servers.goals] == ORDER[1:]
    assert len(db.gaps) == 1 and db.gaps[0]['wall_id'] == 1


def test_scoop_pass_moves_toward_body_each_gap_and_wraps(monkeypatch):
    """헤라 집은 뒤 뜨기마다 회차 +1 (끝점 +X = 몸체 쪽), 4 다음은 1. 간격은 robot.yaml scoop_pass_step."""
    monkeypatch.setattr(fake_servers, 'FAKE_GAPS', [v for y in (-200, -100, 0, 100, 200) for v in
                                                   [600.0, float(y), 100.0, 600.0, float(y), 300.0]])
    run, hub, db, servers = press_full_process()
    assert run.state == 'done', hub.logs
    scoops = [args for task, args in servers.commands if task == 'scoop']
    assert [a[a.index('--scoop-pass') + 1] for a in scoops] == ['1', '2', '3', '4', '1']
    step = CFG['force']['putty_scoop']['scoop_pass_step']
    assert all(a[a.index('--scoop-pass-step') + 1] == str(step) for a in scoops)


def test_start_stage_without_wall_in_db():
    """DB 에 벽이 없으면 E_NO_WALL, 로봇 동작 없음."""
    run, hub, db, servers = press_full_process(params=dict(PARAMS, start_stage='2: 퍼티 작업부터'))
    assert run.state == 'failed' and servers.goals == []
    assert db.runs[0]['result'] == 'E_NO_WALL'


def test_no_gap_does_not_pick_up_spatula():
    """탐색이 틈을 하나도 안 주면 헤라를 집지 않고 E_NO_GAP (run91: 집기 → 바로 반납하다 충돌)."""
    old = fake_servers.FAKE_GAPS
    fake_servers.FAKE_GAPS = []
    try:
        run, hub, db, servers = press_full_process()
    finally:
        fake_servers.FAKE_GAPS = old
    assert run.state == 'failed'
    assert [g[0] for g in servers.goals] == ['EXPLORE']
    assert db.runs[0]['result'] == 'E_NO_GAP'


if __name__ == '__main__':
    run, hub, db, servers = press_full_process()
    print(f'\n===== [전체 공정] → {run.state} · DB result={db.runs[0]["result"]}')
    for level, src, msg in hub.logs:
        print(f'  {level:5} {src:12} {msg}')
    print('  보낸 goal:', [g[0] for g in servers.goals])
    for task, args in servers.commands:
        print(f'  워커 {task:7}: {" ".join(args)}')
    out = os.path.join(HERE, 'full_process', 'db_full_process.json')
    db.dump(out)
    print(f'  DB 로 갔을 데이터: {out}\n')
    sys.exit(pytest.main([__file__, '-q', '-p', 'no:cacheprovider']))


def test_sanding_keeps_tool_between_gaps_and_returns_after_last(monkeypatch):
    """샌딩: 첫 틈에서 도구 집기, 사이 틈은 SANDING_KEEP(들고 이동), 마지막만 SANDING(반납). DB 는 모두 SANDING."""
    monkeypatch.setattr(fake_servers, 'FAKE_GAPS', [v for y in (-100, 0, 100) for v in
                                                   [600.0, float(y), 100.0, 600.0, float(y), 300.0]])
    run, hub, db, servers = press_full_process()
    assert run.state == 'done', hub.logs
    assert [g[0] for g in servers.goals if g[0].startswith('SANDING')] == ['SANDING_KEEP', 'SANDING_KEEP', 'SANDING']
    assert {r['task_code'] for r in db.task_log if 'SAND' in r['task_code']} == {'SANDING'}


def test_yaml_gaps_win_over_db_and_reuse_matching_gap_ids(monkeypatch):
    """틈 좌표는 robot.yaml 이 먼저. DB 에 같은 좌표가 있으면 그 gap_id 를 쓰고, 새 좌표만 저장 (퍼티·샌딩 같은 틈)."""
    other = [700.0, 50.0, 100.0, 700.0, 50.0, 250.0]
    monkeypatch.setattr(process_runner.ProcessRunner, '_manual_gaps',
                        staticmethod(lambda: list(FAKE_GAPS) + other))
    db = _db_with_wall()                     # DB: FAKE_GAPS 1개 (gap_id 1)
    for _ in range(2):                       # 두 번 실행해도 DB 틈은 2개 그대로
        run, hub, db, servers = press_full_process(params=dict(PARAMS, start_stage='2: 퍼티 작업부터'), db=db)
        assert run.state == 'done', hub.logs
    assert len(db.gaps) == 2
    puts = [g for g in servers.goals if g[0] == 'PUTTY']
    sands = [g for g in servers.goals if g[0].startswith('SANDING')]
    assert [g[1] for g in puts] == [g[1] for g in sands] == [1, 2]
    assert [list(g[2]) + list(g[3]) for g in sands] == [FAKE_GAPS, other]


def test_empty_yaml_gaps_falls_back_to_db(monkeypatch):
    monkeypatch.setattr(process_runner.ProcessRunner, '_manual_gaps', staticmethod(lambda: []))
    run, hub, db, servers = press_full_process(params=dict(PARAMS, start_stage='3: 샌딩 작업부터'),
                                               db=_db_with_wall())
    assert run.state == 'done', hub.logs
    assert [g[0] for g in servers.goals] == ['SANDING']
    assert any('비어 있음 → DB 틈 1개' in m for _, _, m in hub.logs)
