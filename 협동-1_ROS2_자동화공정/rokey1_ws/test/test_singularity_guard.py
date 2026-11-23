"""실물 통신 없이 특이점 알람 분류와 이동 중 정지·힘 해제 경로를 확인한다."""

from types import SimpleNamespace
import time

import pytest

from putty_demo.singularity_guard import SingularityLatch, SingularityDetected, SingularityWatch


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


