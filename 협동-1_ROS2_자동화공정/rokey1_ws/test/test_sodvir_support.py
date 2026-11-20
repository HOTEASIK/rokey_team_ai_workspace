"""sodvir 전체 공정 테스트용 기능 검사 (로봇·ROS 통신 없음).
  - ROKEY_CONFIG_OVERRIDE 덮어쓰기, FakeDB JSON 저장, 샌딩 가상 모드(접촉 = 위치 이동, 힘제어 생략)
"""
import json
import os
import subprocess
import sys
from types import SimpleNamespace

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)
os.environ.setdefault('ROKEY_CONFIG', os.path.join(PKG, 'config', 'robot.yaml'))


def test_config_override_only_changes_given_keys():
    over = os.path.join(PKG, 'test', 'full_process', 'sodvir_override.yaml')
    code = ('from utils.config import CFG; import json; '
            'print(json.dumps([CFG["explore"]["gaps"], CFG["explore"]["timeout_sec"], CFG["robot"]["id"]]))')
    env = dict(os.environ, ROKEY_CONFIG_OVERRIDE=over, PYTHONPATH=PKG)
    gaps, timeout, rid = json.loads(subprocess.check_output([sys.executable, '-c', code], env=env, text=True))
    assert gaps == [[720.0, 0.0, 130.0, 720.0, 0.0, 250.0]]
    assert timeout == 900 and rid == 'dsr01'          # 나머지는 robot.yaml 그대로


def test_fake_db_writes_json(tmp_path):
    from manager.fake_db import FakeDB
    path = tmp_path / 'db.json'
    db = FakeDB(str(path))
    run_id = db.start_run('ALL', None, 'virtual', 'TEST', 'putty', None, '')
    db.log_step(run_id, 'PUTTY', -1, 0, SimpleNamespace(step='putty_pickup', step_success=True, error_code='',
                                                        fail_reason='', params_json=''))
    db.end_run(run_id, 'OK', db.check_run(run_id, 'PUTTY'))
    data = json.loads(path.read_text())
    assert data['run'][0]['result'] == 'OK' and data['task_log'][0]['step'] == 'putty_pickup'


def test_sanding_virtual_contact_is_position_and_no_force_control():
    from sanding import sanding_job
    job = object.__new__(sanding_job.SandingJob)          # __init__ 은 DSR_ROBOT2 를 불러오므로 건너뜀
    moves = []
    job.virtual, job.log = True, SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    job.movel = lambda pose, vel, acc, what: moves.append(pose)
    job.dr = None                                         # 가상 모드에서 로봇 API 를 부르면 여기서 에러
    assert job.seek_contact([0, 180, 0], [1, 0, 0], [0, 0, 0], '접촉', target=[720, 0, 130]) is True
    assert moves == [[720, 0, 130, 0, 180, 0]]
    assert job.align_to_wall([0, 180, 0], [0, 0, 0], [1, 0, 0]) == [0, 180, 0]
    job.force_on()                                        # 로봇 API 안 부름


def test_sanding_plan_is_simple_gap_path():
    """단순화 경로: 틈 시작 −10mm 접근, 몸체 쪽 body_side 에서 하강, TCP 는 바닥+50 이상, 끝까지 좌우 왕복."""
    from sanding import sanding_job, surface_math as sm
    job = object.__new__(sanding_job.SandingJob)
    job.log = SimpleNamespace(info=lambda *a: None, warn=lambda *a: None)
    job.floor_z, job.floor_min_tcp, job.body_side_mm = 35.0, 50.0, 80.0
    tool = [-499.65, -365.2, 70.02, 135.28, 179.65, 133.01]
    g = job.plan(tool, [800, 0, 30] + tool[3:], [800, 0, 150] + tool[3:], [-1.0, 0.0, 0.0])
    assert g['s'][2] == 85.0                                   # 30 → 바닥 35 + 50
    assert [round(v, 1) for v in g['approach']] == [790.0, 0.0, 85.0]
    assert [round(v, 1) for v in g['approach_body']] == [710.0, 0.0, 85.0]
    assert g['approach_high'][2] == g['transit_z'] == tool[2] + 200
    pts = job.zigzag(g['s'], g['e'], g['u'], g['side'], g['half'], g['length'])
    assert pts[-1] == g['e'] and len(pts) == 14                # 65mm / 5mm = 13 구간 + 끝
    lat = [sm.dot(sm.sub(p, g['s']), g['side']) for p in pts[:-1]]
    assert all(abs(abs(d) - 12.5) < 1e-6 for d in lat) and lat[0] == -lat[1]
