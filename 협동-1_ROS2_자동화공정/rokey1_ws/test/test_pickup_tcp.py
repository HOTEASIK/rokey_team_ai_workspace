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
