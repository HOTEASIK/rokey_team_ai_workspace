"""실물 이동 없이 접촉 탐색의 제한·정지·순서·실패 처리를 검사한다."""

from types import SimpleNamespace

import pytest

from putty_demo.contact_search import seek_axis, seek_floor_then_wall, validate_search
from putty_demo.contact_search import WallForceHold


class FakeIO:
    def __init__(self, forces):
        self.forces = iter(forces)
        self.pose = [100., 0., 100., 0., 90., 0.]
        self.target = None
        self.events = []

    def read_pose(self):
        return tuple(self.pose)

    def read_force(self):
        return next(self.forces)

    def move_async(self, target, speed):
        self.target = target
        self.events.append(('move', target, speed))

    def stop_and_wait(self):
        self.events.append(('stop',))
        if self.target is not None:
            self.pose[:3] = [(a+b)/2 for a, b in zip(self.pose[:3], self.target[:3])]

    def info(self, text):
        pass


def args():
    return SimpleNamespace(contact_search=True, case='plane', plane='yz', normal_sign='negative',
                           start=(100, 0, 100), end=(100, 0, 280), floor_force=5,
                           wall_force=5, search_distance=20, search_speed=2,
                           force_tolerance=1.0, force_wait_timeout=5.0,
                           force_stable_time=0.5, force_loss_timeout=0.5)


def test_contacts_stop_in_order_and_use_actual_floor_position():
    io = FakeIO([(0, 0, 0, 0, 0, 0), (0, 0, 5, 0, 0, 0),
                 (0, 0, 5, 0, 0, 0), (-5, 0, 5, 0, 0, 0)])
    result = seek_floor_then_wall(io, args())
    assert [e[0] for e in io.events] == ['move', 'stop', 'move', 'stop']
    assert io.events[0][1][:3] == (100, 0, 80)
    assert io.events[2][1][:3] == (120, 0, 90)
    assert result[:3] == (110, 0, 90)


def test_sensor_failure_stops_and_does_not_start_wall_search():
    io = FakeIO([(0, 0, 0, 0, 0, 0), (0, 0, float('nan'), 0, 0, 0)])
    with pytest.raises(RuntimeError, match='외력'):
        seek_floor_then_wall(io, args())
    assert [e[0] for e in io.events] == ['move', 'stop']


def test_no_contact_within_distance_stops_and_fails():
    io = FakeIO([(0,)*6, (0,)*6])
    original = io.read_pose
    io.read_pose = lambda: io.target if io.target is not None else original()
    with pytest.raises(RuntimeError, match='접촉을 찾지'):
        seek_axis(io, 2, -1, 5, 20, 2)
    assert [e[0] for e in io.events] == ['move', 'stop']


@pytest.mark.parametrize('name', ['floor_force', 'wall_force'])
def test_force_thresholds_must_be_explicit(name):
    values = args()
    setattr(values, name, None)
    with pytest.raises(ValueError, match=name):
        validate_search(values)


def test_missing_thresholds_reject_before_robot_runner(monkeypatch):
    from putty_demo import apply_putty_plane as plane
    calls = []
    monkeypatch.setattr(plane, 'run_robot', lambda *args, **kwargs: calls.append(True))
    monkeypatch.setattr('sys.argv', ['apply_putty_plane', '--execute-motion', '--contact-search'])
    assert plane.main() == 1
    assert calls == []


def test_force_hold_lifecycle_and_target():
    events = []
    io = SimpleNamespace(compliance=lambda k: events.append('compliance'),
                         set_force=lambda f: events.append(('force', f)),
                         release_force=lambda: events.append('release_force'),
                         release_compliance=lambda: events.append('release_compliance'))
    hold = WallForceHold(io, 2.3, (3000, 3000, 3000, 200, 200, 200))
    hold.start()
    assert hold.active
    hold.release()
    hold.release()
    assert events == ['compliance', ('force', 2.3), 'release_force', 'release_compliance']
    assert not hold.active


def test_release_compliance_even_if_force_release_fails():
    events = []
    def fail():
        raise RuntimeError('release failed')
    io = SimpleNamespace(compliance=lambda k: None, set_force=lambda f: None,
                         release_force=fail, release_compliance=lambda: events.append('released'))
    hold = WallForceHold(io, 2.3, (3000,)*6)
    hold.start()
    with pytest.raises(RuntimeError):
        hold.release()
    assert events == ['released']


def test_force_mode_only_excludes_x_from_goal_check():
    from putty_demo.putty_motion import stroke_arrived
    target = (700, 0, 200, 0, 90, 0)
    actual = (710, 0, 200, 0, 90, 0)
    assert stroke_arrived(actual, target, force_active=True)
    assert not stroke_arrived(actual, target)
    assert not stroke_arrived((710, 0, 210, 0, 90, 0), target, force_active=True)


@pytest.mark.parametrize('force', [10.0, 12.0])
@pytest.mark.parametrize('mode', [0, 1])
def test_base_positive_x_force_request(force, mode):
    from putty_demo.contact_search import press_force_request
    request = press_force_request(force, mode=mode)
    assert list(request.fd) == [force, 0, 0, 0, 0, 0]
    assert list(request.dir) == [1, 0, 0, 0, 0, 0]
    assert request.ref == 0
    assert request.mod == mode


