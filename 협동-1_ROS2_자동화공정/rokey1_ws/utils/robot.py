"""로봇 공통 상수 (config/robot.yaml 의 robot 섹션). 노드마다 따로 적지 말고 여기서 가져다 쓴다.

사용 예:
    from utils.robot import ROBOT_ID, ROBOT_MODEL, TOOL_NAME, TCP_NAME, HOME_J
"""
from utils.config import CFG

_R = CFG['robot']
ROBOT_ID = _R['id']
ROBOT_MODEL = _R['model']
TOOL_NAME = _R['tool']
TCP_NAME = _R['tcp']
HOME_J = [float(v) for v in _R['home_j']]
