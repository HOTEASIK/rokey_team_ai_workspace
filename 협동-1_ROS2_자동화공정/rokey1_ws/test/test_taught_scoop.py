"""측정 XYZABC로 접근·찍기·상승하며 회전을 유지하는 경로 검사."""

from types import SimpleNamespace

import pytest

from putty_demo.scoop_putty import (
    TAUGHT_SCOOP_POSE, TAUGHT_LIFT_MM, validate_taught, confirm_grasp,
)


def settings():
    return SimpleNamespace(taught_pose=TAUGHT_SCOOP_POSE, lift=TAUGHT_LIFT_MM,
                           entry_height=120, scoop_angle=45, reference_blade_angle=60, edge_offset=(0, 0, 126),
                           scoop_speed=3, transfer_speed=30, acceleration=20,
                           transfer_z=245.14, apply_after=False)




def test_real_grasp_confirmation_is_not_assumed(monkeypatch):
    monkeypatch.setattr('builtins.input', lambda prompt: 'n')
    with pytest.raises(RuntimeError, match='파지'):
        confirm_grasp()
    monkeypatch.setattr('builtins.input', lambda prompt: 'y')
    confirm_grasp()






@pytest.mark.parametrize('name,value', [('entry_height', 0), ('entry_height', float('nan')),
                                      ('scoop_angle', 0), ('scoop_angle', 90)])
def test_invalid_diagonal_entry(name, value):
    args = settings()
    setattr(args, name, value)
    with pytest.raises(ValueError):
        validate_taught(args)
