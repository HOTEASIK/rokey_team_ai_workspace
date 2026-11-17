"""실제 로봇 없이 출발 허가·힘 유지 감시·제한 및 이전 옵션 제거를 검증한다."""

from types import SimpleNamespace

import pytest

from putty_demo.contact_search import PressForceMonitor, wait_for_press_force
from putty_demo.apply_putty_plane import make_plane_parser, resolve_plane_args
from putty_demo.contact_search import validate_search


def force(fx):
    return (fx, 0, 0, 0, 0, 0)


def monitor():
    return PressForceMonitor(6, 1, 0.5, 0.5)


@pytest.mark.parametrize('sign', [-1, 1])
def test_stable_signed_measurements_required_before_start(sign):
    m = monitor()
    m.update(force(6*sign), 0)
    m.update(force(6*sign), 0.25)
    assert not m.ready
    m.update(force(6*sign), 0.5)
    assert m.ready
    assert m.sign == sign


def test_brief_contact_does_not_allow_start():
    m = monitor()
    for t, fx in [(0, 6), (0.3, 6), (0.4, 2), (0.5, 6), (0.8, 6)]:
        m.update(force(fx), t)
    assert not m.ready


def ready_monitor():
    m = monitor()
    for t in (0, 0.25, 0.5):
        m.update(force(-6), t)
    return m


def test_sustained_loss_aborts_and_transient_loss_recovers():
    m = ready_monitor()
    m.update(force(-2), 1)
    m.update(force(-6), 1.4)
    m.update(force(-2), 2)
    with pytest.raises(RuntimeError, match='힘 부족'):
        m.update(force(-2), 2.5)


@pytest.mark.parametrize('fx', [-7.01, 7.01])
def test_overload_is_immediate(fx):
    with pytest.raises(RuntimeError, match='상한'):
        monitor().update(force(fx), 0)


def test_opposite_sign_force_is_not_mistaken_for_maintained_contact():
    with pytest.raises(RuntimeError, match='부호 반전'):
        ready_monitor().update(force(6), 1)


def wait_io(fx):
    return SimpleNamespace(read_pose=lambda: (100, 0, 100, 0, 90, 0),
                           read_force=lambda: force(fx), read_state=lambda: 1,
                           log_force=lambda *a, **kw: None)


def test_wait_times_out_without_sending_any_motion():
    now = [0.0]
    def sleep(seconds):
        now[0] += seconds
    with pytest.raises(TimeoutError):
        wait_for_press_force(wait_io(1), monitor(), 0.6, 20,
                             clock=lambda: now[0], sleep=sleep)


def test_wait_returns_only_after_stability():
    now = [0.0]
    def sleep(seconds):
        now[0] += seconds
    m = monitor()
    actual = wait_for_press_force(wait_io(-6), m, 5, 20,
                                  clock=lambda: now[0], sleep=sleep)
    assert m.ready and now[0] >= 0.5
    assert actual[:3] == (100, 0, 100)


def test_wait_has_finite_travel_limit_even_with_valid_force():
    io = wait_io(-6)
    poses = iter([(100, 0, 100, 0, 90, 0), (120, 0, 100, 0, 90, 0)])
    io.read_pose = lambda: next(poses)
    with pytest.raises(RuntimeError, match='X 이동'):
        wait_for_press_force(io, monitor(), 5, 20)


def test_no_extra_approach_option_and_default_guard_configuration():
    parser = make_plane_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(['--execute-motion', '--extra-approach', '7'])
    args = parser.parse_args(['--execute-motion', '--contact-search',
                              '--floor-force', '2.3', '--wall-force', '2.3'])
    resolve_plane_args(args)
    validate_search(args)
    assert args.press_force == 6
    assert args.force_tolerance == 1
    assert args.force_wait_timeout == 5
    assert not hasattr(args, 'extra_approach')


@pytest.mark.parametrize('name,value', [('force_tolerance', 6), ('force_wait_timeout', 0),
                                      ('force_stable_time', 5), ('force_loss_timeout', -1)])
def test_bad_guard_settings_reject_before_robot_execution(name, value):
    args = make_plane_parser().parse_args(['--execute-motion', '--contact-search',
                                           '--floor-force', '2.3', '--wall-force', '2.3'])
    resolve_plane_args(args)
    setattr(args, name, value)
    with pytest.raises(ValueError):
        validate_search(args)


def test_force_gate_failure_never_sends_spline_and_stops_before_release(monkeypatch):
    """실제 ROS 없이 실행부의 출발 금지 및 정지·해제 순서를 확인한다."""
    import sys
    from putty_demo import putty_motion as motion
    import utils.shutdown

    events = []
    actual = [100., 0., 100., 0., 90., 0.]
    log = SimpleNamespace(info=lambda *a: None, warning=lambda *a: None)

    class Future:
        def __init__(self, value):
            self.value = value
        def done(self):
            return True
        def result(self):
            return self.value

    class Client:
        def __init__(self, path):
            self.path = path
        def wait_for_service(self, **kwargs):
            return True
        def call_async(self, request):
            key = self.path.rsplit('/', 1)[-1]
            events.append(key)
            if key == 'move_line':
                actual[:] = request.pos
            response = SimpleNamespace(success=True, robot_system=0, info=motion.EXPECTED_TCP,
                                       task_pos_info=[SimpleNamespace(data=actual.copy())],
                                       tool_force=force(1), robot_state=1)
            return Future(response)

    node = SimpleNamespace(create_client=lambda typ, path: Client(path), get_logger=lambda: log)
    ros = SimpleNamespace(init=lambda **kw: None, create_node=lambda *a, **kw: node,
                          spin_until_future_complete=lambda *a, **kw: None)
    monkeypatch.setitem(sys.modules, 'rclpy', ros)
    monkeypatch.setattr(utils.shutdown, 'safe_shutdown', lambda n: None)
    def fail_gate(*a, **kw):
        raise TimeoutError('force not reached')
    monkeypatch.setattr(motion, 'wait_for_press_force', fail_gate)
    args = make_plane_parser().parse_args(['--execute-motion', '--contact-search',
                                           '--floor-force', '2.3', '--wall-force', '2.3'])
    def builder(current, args):
        yield '면에 수직 접근', tuple(actual), 10
        yield '평면 각도 변화 도포', (100, 0, 280, 0, 90, 0), 10
    with pytest.raises(TimeoutError, match='force not reached'):
        motion.run_robot(args, builder, allow_real=True, continuous=True,
                         contact_probe=lambda io: tuple(actual), press_force=6)
    assert 'set_desired_force' in events
    assert 'move_spline_task' not in events
    assert events.index('move_stop') < events.index('release_force')
    assert events.index('release_force') < events.index('release_compliance_ctrl')
