"""robot.yaml 로더. 소스 폴더의 파일을 바로 읽으므로 값 수정 후 빌드 없이 재시작만 하면 된다.
다른 경로를 쓰려면 환경변수 ROKEY_CONFIG 로 지정.
ROKEY_CONFIG_OVERRIDE=<yaml> 이면 그 파일의 값만 덮어쓴다 (sodvir 테스트: 가상 벽 틈 좌표 등).
ROKEY_MODE=virtual 이면 robot.yaml 의 virtual_override 섹션을 마지막에 덮어쓴다 (가상 시험용 빠른 속도 등).
  hmi_server 가 virtual 모드로 켜는 서버/노드에 자동으로 넣는다. real 이거나 없으면 적용 안 함.

사용 예:
    from utils.config import CFG
    CFG['force']['putty_scrape']['target']
"""
import os

import yaml

PATH = os.environ.get(
    'ROKEY_CONFIG',
    os.path.expanduser('~/ws_cobot_pjt/ws_dsr/src/rokey1/config/robot.yaml'))
OVERRIDE = os.environ.get('ROKEY_CONFIG_OVERRIDE', '')
VIRTUAL = os.environ.get('ROKEY_MODE', '') == 'virtual'


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v


def load():
    """robot.yaml (+ OVERRIDE) 를 지금 다시 읽는다 (노드를 다시 켜지 않아도 고친 값이 바로 쓰이게)."""
    with open(PATH) as f:
        cfg = yaml.safe_load(f) or {}
    if OVERRIDE:
        with open(OVERRIDE) as f:
            _merge(cfg, yaml.safe_load(f) or {})
    over = cfg.pop('virtual_override', None) or {}
    if VIRTUAL:
        _merge(cfg, over)
    return cfg


CFG = load()
