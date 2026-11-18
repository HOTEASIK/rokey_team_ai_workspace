"""실물 통신 없이 특이점 알람 분류와 이동 중 정지·힘 해제 경로를 확인한다."""

from types import SimpleNamespace
import sys
import time

import pytest

from putty_demo.singularity_guard import SingularityLatch, SingularityDetected, SingularityWatch
from putty_demo.apply_putty_plane import make_plane_parser


def alarm(text, code=3205, level=1, group=2):
    return SimpleNamespace(msg1=text, msg2='', msg3='', code=code, level=level, group=group)


def test_region_entry_latches_even_when_informational():
    latch = SingularityLatch()
    assert latch.observe(alarm('Change singularity region status(Outside -> Inside), mode(1)'))
    with pytest.raises(SingularityDetected, match='3205'):
        latch.check()


def test_exit_before_entry_is_not_danger_but_cannot_clear_latched_entry():
    latch = SingularityLatch()
    assert not latch.observe(alarm('Inside -> Outside'))
    latch.check()
    latch.observe(alarm('Outside -> Inside'))
    latch.observe(alarm('Inside -> Outside'))
    with pytest.raises(SingularityDetected):
        latch.check()


def test_unrelated_alarm_is_not_mistaken_for_singularity():
    latch = SingularityLatch()
    assert not latch.observe(alarm('Other motion information', code=1234))
    assert not latch.observe(alarm('Outside -> Inside', group=1))
    latch.check()


def test_unrecognized_region_event_and_singularity_error_stop_conservatively():
    for message in (alarm('unrecognized region message'),
                    alarm('Singularity force control error', code=9999, level=3)):
        latch = SingularityLatch()
        latch.observe(message)
        with pytest.raises(SingularityDetected):
            latch.check()


@pytest.mark.parametrize('fault', ['publisher_lost', 'executor_dead', 'executor_stale'])
def test_loss_of_monitoring_is_not_treated_as_safe(fault):
    watch = SimpleNamespace(latch=SingularityLatch(), failure=None, topic='/dsr01/error',
                            last_spin=time.monotonic(),
                            thread=SimpleNamespace(is_alive=lambda: fault != 'executor_dead'),
                            node=SimpleNamespace(count_publishers=lambda topic:
                                                 0 if fault == 'publisher_lost' else 1))
    if fault == 'executor_stale':
        watch.last_spin -= 2
    with pytest.raises(RuntimeError):
        SingularityWatch.check(watch)


@pytest.mark.parametrize('phase', ['configuration', 'line', 'force', 'spline'])
def test_mid_operation_alarm_stops_and_cleanup_bypasses_alarm_latch(monkeypatch, phase):
    from putty_demo import putty_motion as motion
    import utils.shutdown

    events = []
    latch = SingularityLatch()
    actual = [100., 0., 100., 0., 90., 0.]
    injected = [False]
    stage = {'configuration': 'set_singularity_handling', 'line': 'move_line',
             'force': 'set_desired_force', 'spline': 'move_spline_task'}[phase]

    class Future:
        def __init__(self, response, done=True):
            self.response, self.finished = response, done
        def done(self):
            return self.finished
        def result(self):
            return self.response

    class Client:
        def __init__(self, path):
            self.name = path.rsplit('/', 1)[-1]
        def wait_for_service(self, **kwargs):
            return True
        def call_async(self, request):
            name = self.name
            events.append((name, getattr(request, 'mode', None)))
            if name == stage and not injected[0]:
                injected[0] = True
                # 동기 이동을 미응답 상태로 두고 알람에 의한 중단을 확인
                latch.observe(alarm('Outside -> Inside'))
                return Future(None, done=False)
            if name == 'move_line':
                actual[:] = request.pos
            response = SimpleNamespace(success=True, robot_system=0, info=motion.EXPECTED_TCP,
                                       task_pos_info=[SimpleNamespace(data=actual.copy())],
                                       tool_force=[6., 0., 0., 0., 0., 0.], robot_state=1)
            return Future(response)

    logger = SimpleNamespace(info=lambda *a: None, warning=lambda *a: None)
    node = SimpleNamespace(create_client=lambda typ, path: Client(path), get_logger=lambda: logger)
    ros = SimpleNamespace(init=lambda **kw: None, create_node=lambda *a, **kw: node,
                          spin_until_future_complete=lambda *a, **kw: None)
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setattr(utils.shutdown, 'safe_shutdown', lambda n: events.append(('shutdown', None)))
    watch = SimpleNamespace(wait_ready=lambda: None, check=latch.check,
                            close=lambda: events.append(('watch_closed', None)))
    monkeypatch.setattr(motion, 'start_singularity_watch', lambda rid: watch)
    def ready(io, monitor, *a, **kw):
        monitor.ready = True
        return tuple(actual)
    monkeypatch.setattr(motion, 'wait_for_press_force', ready)
    args = make_plane_parser().parse_args(['--execute-motion', '--contact-search',
                                          '--floor-force', '2.3', '--wall-force', '2.3', '--press-force', '10'])
    def builder(current, args):
        yield '면에 수직 접근', tuple(actual), 10
        yield '평면 각도 변화 도포', (100, 0, 280, 0, 90, 0), 10
    with pytest.raises(SingularityDetected):
        motion.run_robot(args, builder, allow_real=True, continuous=True,
                         contact_probe=lambda io: tuple(actual), press_force=10)
    names = [name for name, mode in events]
    assert injected[0]
    assert ('set_singularity_handling', 0) in events
    assert 'watch_closed' in names and names[-1] == 'shutdown'
    if phase == 'configuration':
        assert 'move_line' not in names and 'move_spline_task' not in names
    else:
        assert ('set_singular_handling_force', 0) in events
        assert 'move_stop' in names
    if phase in ('force', 'spline'):
        assert names.index('move_stop') < names.index('release_force')
        assert names.index('release_force') < names.index('release_compliance_ctrl')
    if phase != 'spline':
        assert 'move_spline_task' not in names
