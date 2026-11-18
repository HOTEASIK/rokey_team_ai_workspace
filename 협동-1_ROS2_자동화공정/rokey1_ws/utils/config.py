"""robot.yaml 로더. 소스 폴더의 파일을 바로 읽으므로 값 수정 후 빌드 없이 재시작만 하면 된다.
다른 경로를 쓰려면 환경변수 ROKEY_CONFIG 로 지정.

사용 예:
    from utils.config import CFG
    CFG['force']['putty_scrape']['target']
"""
import os
from pathlib import Path

import yaml

def default_path():
    source = Path(__file__).resolve().parents[1] / 'config' / 'robot.yaml'
    if source.is_file():
        return str(source)
    from ament_index_python.packages import get_package_share_directory
    return str(Path(get_package_share_directory('rokey1')) / 'config' / 'robot.yaml')


PATH = os.environ.get('ROKEY_CONFIG') or default_path()

with open(PATH) as f:
    CFG = yaml.safe_load(f)
if not isinstance(CFG, dict):
    raise ValueError(f'설정 파일은 YAML mapping이어야 합니다: {PATH}')
