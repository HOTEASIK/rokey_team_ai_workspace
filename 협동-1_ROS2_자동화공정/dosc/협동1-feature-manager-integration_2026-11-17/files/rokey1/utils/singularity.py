"""특이점 관리 util (뼈대)

정해진 것: 두산 set_singularity_handling 사용, robot.yaml util.singularity 에서 모드/재시도 읽음
아직 정할 것: 회피 실패 시 movej 로 빠져나갈 자세, 실패를 E_SINGULAR 로 보고하는 조건

사용 예 (DSR_ROBOT2 import 이후):
    from utils.singularity import apply
    apply(DSR_ROBOT2)
"""
from utils.config import CFG

MODES = {'avoid': 0, 'task_stop': 1, 'var_vel': 2}   # DR_AVOID / DR_TASK_STOP / DR_VAR_VEL


def apply(dsr):
    """공정 노드 시작 시 1회 호출."""
    dsr.set_singularity_handling(MODES[CFG['util']['singularity']['mode']])
