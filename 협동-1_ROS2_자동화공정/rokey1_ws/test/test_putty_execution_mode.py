"""이동 요청을 모킹하여 로그 전용 분기 제거와 명시적 실행 조건을 확인한다."""

import importlib
from types import SimpleNamespace

import pytest


MODULES = ('pickup_spatula', 'scoop_putty', 'apply_putty_plane', 'return_spatula')


@pytest.mark.parametrize('name', MODULES)
def test_node_requires_explicit_execution(name, monkeypatch):
    module = importlib.import_module(f'putty_demo.{name}')
    monkeypatch.setattr('sys.argv', [name])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2


@pytest.mark.parametrize('name', MODULES)
def test_execution_reaches_controller_runner(name, monkeypatch):
    module = importlib.import_module(f'putty_demo.{name}')
    calls = []
    monkeypatch.setattr(module, 'run_robot', lambda *args, **kwargs: calls.append((args, kwargs)))
    argv = [name, '--execute-motion']
    monkeypatch.setattr('sys.argv', argv)
    assert module.main() == 0
    assert len(calls) == 1


@pytest.mark.parametrize('name', MODULES)
def test_old_log_only_option_is_rejected(name, monkeypatch):
    module = importlib.import_module(f'putty_demo.{name}')
    monkeypatch.setattr('sys.argv', [name, '--execute-motion', '--dry-run'])
    with pytest.raises(SystemExit) as error:
        module.main()
    assert error.value.code == 2


def test_initialization_false_rejects_instead_of_preview():
    from putty_demo.initialize_device import initialize
    node = SimpleNamespace(declare_parameter=lambda *args: None,
                           get_parameter=lambda name: SimpleNamespace(value=False))
    with pytest.raises(RuntimeError, match='execute_motion'):
        initialize(node)


def test_hmi_virtual_uses_virtual_services():
    from hmi.tasks import _initialize_args, _pickup_args, TASK_MAP
    assert 'virtual_gripper:=true' in _initialize_args('virtual', {})
    params = dict(clearance=50, settle=1, transfer_speed=15, pick_speed=5)
    assert '--virtual-gripper' in _pickup_args('virtual', params)
    assert '--virtual-gripper' not in _pickup_args('real', params)
    assert TASK_MAP['pickup_spatula']['motion']('virtual')
