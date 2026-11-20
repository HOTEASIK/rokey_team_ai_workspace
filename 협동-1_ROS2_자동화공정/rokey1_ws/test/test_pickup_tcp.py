"""TCP 적용·재조회 검사. 실제 서비스나 이동은 실행하지 않는다."""

import pytest

from putty_demo.pickup_spatula import EXPECTED_TCP, ensure_tcp


def test_matching_tcp_does_not_set_again():
    changes = []
    ensure_tcp(lambda: EXPECTED_TCP, changes.append, lambda message: None)
    assert changes == []


@pytest.mark.parametrize('initial', ['', 'other_tcp'])
def test_mismatched_tcp_is_set_and_read_back(initial):
    state = {'name': initial}
    events = []

    def read():
        events.append('read')
        return state['name']

    def write(name):
        events.append(('set', name))
        state['name'] = name

    ensure_tcp(read, write, lambda message: None)
    assert events == ['read', ('set', EXPECTED_TCP), 'read']


def test_failed_set_aborts_with_pendant_hint():
    def fail(name):
        raise RuntimeError('service failed')
    with pytest.raises(RuntimeError, match='펜던트'):
        ensure_tcp(lambda: '', fail, lambda message: None)


def test_successful_set_but_wrong_readback_aborts():
    with pytest.raises(RuntimeError, match='적용 후에도'):
        ensure_tcp(lambda: 'other_tcp', lambda name: None, lambda message: None)


def test_nearest_turn_handles_small_ccw_from_home_and_cw_from_behind():
    from putty_demo.pickup_spatula import turn_joint_target
    rest = (0.0,)*5
    # run102: 홈(J1 0.2)에서 Y=120 틈(J1 +11.4) — 'cw' 고정이면 348.8° 로 멈췄다
    assert turn_joint_target((0.2, *rest), (11.4, *rest), 'nearest')[0] == 11.4
    assert turn_joint_target((163.6, *rest), (-5.0, *rest), 'nearest')[0] == -5.0
    assert turn_joint_target((163.6, *rest), (9.7, *rest), 'nearest')[0] == pytest.approx(9.7)
