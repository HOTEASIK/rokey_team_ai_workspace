"""robot.yaml 로더. 소스 폴더의 파일을 바로 읽으므로 값 수정 후 빌드 없이 재시작만 하면 된다.
다른 경로를 쓰려면 환경변수 ROKEY_CONFIG 로 지정.

사용 예:
    from utils.config import CFG
    CFG['force']['putty_scrape']['target']
"""
import os

import yaml

PATH = os.environ.get(
    'ROKEY_CONFIG',
    os.path.expanduser('~/ws_cobot_pjt/ws_dsr/src/rokey1/config/robot.yaml'))

with open(PATH) as f:
    CFG = yaml.safe_load(f)
