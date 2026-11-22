"""팀 공통 유틸 호환 및 종료 처리 검사. 실제 ROS 호출은 하지 않는다."""

from types import SimpleNamespace

from utils import shutdown
from utils.service_waiter import ServiceWaiter
from putty_demo.service_waiter import ServiceWaiter as LegacyWaiter


def test_legacy_waiter_uses_team_class():
    assert LegacyWaiter is ServiceWaiter


def test_cleanup_failure_still_closes_node_and_context(monkeypatch):
    events = []
    logger = SimpleNamespace(error=lambda text: events.append('error'))
    node = SimpleNamespace(get_logger=lambda: logger,
                           destroy_node=lambda: events.append('destroy'))
    monkeypatch.setattr(shutdown, 'rclpy', SimpleNamespace(
        ok=lambda: True, shutdown=lambda: events.append('shutdown')))

    def failing_cleanup():
        raise RuntimeError('expected')

    shutdown.safe_shutdown(node, ('cleanup failed', failing_cleanup),
                           lambda: events.append('second cleanup'))
    assert events == ['error', 'second cleanup', 'destroy', 'shutdown']


def test_shutdown_without_node_after_context_closed(monkeypatch):
    monkeypatch.setattr(shutdown, 'rclpy', SimpleNamespace(
        ok=lambda: False, logging=SimpleNamespace(get_logger=lambda name: None)))
    shutdown.safe_shutdown(None)
