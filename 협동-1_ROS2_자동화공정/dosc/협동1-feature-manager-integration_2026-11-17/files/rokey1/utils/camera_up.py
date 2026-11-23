"""z 가 임계값 아래로 내려가면 J6 카메라를 항상 위로 유지하는 util (뼈대)

정해진 것: 임계값 robot.yaml util.camera_up.z_threshold (base 기준 mm)
아직 정할 것:
  - 카메라가 '위'일 때의 J6 각도 (실측)
  - J6 를 돌리면 도구도 같이 돈다 → 도구 쥐고 작업 중에도 적용할지
  - J6 ±360 중 회전 방향 (케이블 꼬임)
"""
from utils.config import CFG

Z_TH = CFG['util']['camera_up']['z_threshold']


def need(z):
    """이 z 에서 카메라 위 유지가 필요한가."""
    return z < Z_TH
