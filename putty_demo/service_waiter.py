"""기존 import 경로 호환용. 팀 공통 utils.service_waiter를 사용한다."""

# ===== 파일 역할 / 통신 안내 =====
# 이전 import 경로를 위한 호환 모듈. 실제 구현은 utils.service_waiter.ServiceWaiter이다.
# 별도 ROS 노드가 아니며, 서비스가 발견될 때까지 기다리는 기능만 제공한다.

from utils.service_waiter import ServiceWaiter

__all__ = ['ServiceWaiter']
