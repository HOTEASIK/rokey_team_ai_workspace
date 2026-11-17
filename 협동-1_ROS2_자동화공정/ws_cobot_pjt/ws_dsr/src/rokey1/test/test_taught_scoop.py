"""측정 XYZABC로 접근·찍기·상승하며 회전을 유지하는 경로 검사."""

from types import SimpleNamespace

import pytest

from putty_demo.scoop_putty import (
    TAUGHT_SCOOP_POSE, TAUGHT_LIFT_MM, taught_moves, taught_target, validate_taught, confirm_grasp,
)


def settings():
    return SimpleNamespace(taught_pose=TAUGHT_SCOOP_POSE, lift=TAUGHT_LIFT_MM,
                           entry_height=120, scoop_angle=45, reference_blade_angle=60, edge_offset=(0, 0, 126),
                           scoop_speed=3, transfer_speed=30, acceleration=20,
                           transfer_z=245.14, apply_after=False)


@pytest.mark.parametrize('height', [245.14, 400])
def test_measured_approach_and_lift_keep_y_parallel_pose(height):
    args = settings()
    current = (308.42, 120.70, height, 18.32, 178.35, 17.14)
    moves = list(taught_moves(current, args))
    assert moves[0][1][3:] == current[3:]
    assert moves[1][1][:2] == current[:2]
    corrected = taught_target(args)
    assert all(target[3:] == corrected[3:] for _, target, _ in moves[1:])
    assert moves[-2][1] == corrected
    assert moves[-1][1] == pytest.approx((*corrected[:2], corrected[2]+120, *corrected[3:]))
    entry = moves[-3][1]
    assert entry == pytest.approx((corrected[0]-120, corrected[1], corrected[2]+120, *corrected[3:]))
    target = moves[-2][1]
    assert target[0]-entry[0] == pytest.approx(entry[2]-target[2])
    assert entry[1] == target[1]
    assert moves[-1][1][:2] == target[:2]
    assert moves[2][1][2] >= height


def test_real_grasp_confirmation_is_not_assumed(monkeypatch):
    monkeypatch.setattr('builtins.input', lambda prompt: 'n')
    with pytest.raises(RuntimeError, match='파지'):
        confirm_grasp()
    monkeypatch.setattr('builtins.input', lambda prompt: 'y')
    confirm_grasp()


def test_reject_bad_lift_and_unvalidated_apply_after():
    args = settings()
    args.lift = -1
    with pytest.raises(ValueError):
        validate_taught(args)

    args.lift = 120
    args.apply_after = True
    with pytest.raises(ValueError):
        validate_taught(args)


@pytest.mark.parametrize('angle', [30, 45, 60])
def test_common_angle_controls_path_and_blade_pose(angle):
    import math
    args = settings()
    args.scoop_angle, args.entry_height, args.lift = angle, 60, 120
    moves = list(taught_moves((308.42, 120.70, 400, 18.32, 178.35, 17.14), args))
    entry, target, lifted = [pose for _, pose, _ in moves[-3:]]
    assert math.degrees(math.atan2(entry[2]-target[2], target[0]-entry[0])) == pytest.approx(angle)
    assert entry[3:] == target[3:] == lifted[3:]
    assert lifted[2]-target[2] == pytest.approx(120)
    from putty_demo.putty_geometry import zyz_axes, rotate_base_y
    reference_axes = zyz_axes(TAUGHT_SCOOP_POSE[3:])
    target_axes = zyz_axes(target[3:])
    for initial, actual in zip(reference_axes, target_axes):
        assert actual == pytest.approx(rotate_base_y(initial, angle-60))
    # BASE +Y축은 회전해도 변하지 않으므로 확인된 모서리 평행을 유지한다.
    assert rotate_base_y((0, 1, 0), angle-60) == pytest.approx((0, 1, 0))
    for i in range(3):
        reference_edge = TAUGHT_SCOOP_POSE[i]+sum(reference_axes[j][i]*args.edge_offset[j]
                                                  for j in range(3))
        corrected_edge = target[i]+sum(target_axes[j][i]*args.edge_offset[j] for j in range(3))
        assert corrected_edge == pytest.approx(reference_edge)


@pytest.mark.parametrize('name,value', [('entry_height', 0), ('entry_height', float('nan')),
                                      ('scoop_angle', 0), ('scoop_angle', 90)])
def test_invalid_diagonal_entry(name, value):
    args = settings()
    setattr(args, name, value)
    with pytest.raises(ValueError):
        validate_taught(args)
