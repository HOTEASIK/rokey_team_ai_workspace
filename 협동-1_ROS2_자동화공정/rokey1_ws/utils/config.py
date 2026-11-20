"""robot.yaml 로더. 소스 폴더의 파일을 바로 읽으므로 값 수정 후 빌드 없이 재시작만 하면 된다.
다른 경로를 쓰려면 환경변수 ROKEY_CONFIG 로 지정.
ROKEY_CONFIG_OVERRIDE=<yaml> 이면 그 파일의 값만 덮어쓴다 (sodvir 테스트: 가상 벽 틈 좌표 등).

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


def _merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v


with open(PATH) as f:
    CFG = yaml.safe_load(f)
if OVERRIDE:
    with open(OVERRIDE) as f:
        _merge(CFG, yaml.safe_load(f) or {})
