"""Offline checks for scoop geometry and material-face transition."""

import math
from types import SimpleNamespace

import pytest

from putty_demo.putty_geometry import (
    EDGE_OFFSET, PICKUP_ABC, SURFACE_Z, build_stroke, orientation_error, zyz_axes,
    rotate_base_y, axes_to_zyz,
)
from putty_demo.scoop_putty import scoop_moves, validate, CONTAINER_CENTER, CONTAINER_SIZE


def settings(**overrides):
    values = dict(container_center=CONTAINER_CENTER, container_size=CONTAINER_SIZE,
                  blade_width=100, blade_length=70, scoop_depth=3, scoop_distance=10, scoop_angle=45,
                  scoop_speed=5, transfer_speed=30, apply_speed=10, acceleration=20,
                  transfer_z=245.14, surface_z=SURFACE_Z, edge_offset=EDGE_OFFSET,
                  samples=21, apply_after=True)
    values.update(overrides)
    return SimpleNamespace(**values)


def test_container_must_fit_tool_and_depth():
    for changes in (dict(blade_width=150), dict(scoop_depth=48),
                    dict(scoop_distance=50), dict(transfer_z=float('nan'))):
        with pytest.raises(ValueError):
            validate(settings(**changes))


def test_material_face_and_sequence():
    poses = build_stroke(SURFACE_Z, EDGE_OFFSET, 21, normal_x=1.0)
    moves = list(scoop_moves((589,76,245.14,*PICKUP_ABC), poses, settings()))
    assert all(all(map(math.isfinite, target)) for _, target, _ in moves)
    up = next(i for i, (label, _, _) in enumerate(moves) if label.startswith('찍기 자세 유지하며 상승'))
    flip = [i for i, (label, _, _) in enumerate(moves) if label.startswith('도포 전 면 뒤집기')]
    first_apply = next(i for i, (label, _, _) in enumerate(moves) if label.startswith('16번 도포 시작점'))
    assert up < flip[0] < flip[-1] < first_apply
    # Recover the material face from its pickup-local coordinates.
    initial = zyz_axes(PICKUP_ABC)
    local_normal = tuple(axis[0] for axis in initial)
    def normal(abc):
        axes = zyz_axes(abc)
        return tuple(sum(axes[j][i]*local_normal[j] for j in range(3)) for i in range(3))
    assert normal(moves[up][1][3:])[2] > .7
    assert normal(moves[first_apply][1][3:])[2] < -.7
    for i in flip:
        assert orientation_error(moves[i-1][1][3:], moves[i][1][3:]) <= 5.00001
    assert math.dist(moves[first_apply][1][:3], poses[0][1]) < 1e-8
    assert moves[-1][1][2] >= moves[0][1][2]


def test_scoop_only_does_not_flip_or_apply():
    poses = build_stroke(SURFACE_Z, EDGE_OFFSET, 21, normal_x=1.0)
    moves = list(scoop_moves((589,76,245.14,*PICKUP_ABC), poses, settings(apply_after=False)))
    assert moves[-1][0] == '찍기 자세 유지하며 상승'


def test_euler_boundary_is_same_rotation():
    for abc in ((0,0,0), (160.32,180,160.32), (35,180,20)):
        assert orientation_error(abc, axes_to_zyz(zyz_axes(abc))) < 1e-5


def test_transfer_arrives_at_45_degrees_before_vertical_approach():
    poses = build_stroke(SURFACE_Z, EDGE_OFFSET, 21, normal_x=1.0)
    moves = list(scoop_moves((589, 76, 245.14, *PICKUP_ABC), poses, settings()))
    _, transfer, _ = moves[1]
    label, approach, _ = moves[2]
    assert label == '퍼티 표면 접근'
    assert transfer[:2] == approach[:2]
    assert transfer[3:] == approach[3:]
    assert transfer[2] > approach[2]
    expected = axes_to_zyz(tuple(rotate_base_y(axis, -45) for axis in zyz_axes(PICKUP_ABC)))
    assert orientation_error(transfer[3:], expected) < 1e-5


@pytest.mark.parametrize('angle', [30, 45, 60])
def test_fixed_angle_target_and_vertical_lift(angle):
    args = settings(scoop_angle=angle, apply_after=False)
    poses = build_stroke(SURFACE_Z, EDGE_OFFSET, 21, normal_x=1.0)
    moves = list(scoop_moves((589, 76, 245.14, *PICKUP_ABC), poses, args))
    forward = [p for label, p, _ in moves if label.startswith('고정 자세 사선 찍기')]
    assert not any(label.startswith('같은 경로 후퇴') for label, _, _ in moves)
    approach = moves[2][1]
    assert all(orientation_error(p[3:], approach[3:]) < 1e-5 for p in forward)
    end = forward[-1]
    lift = moves[-1][1]
    assert lift[:2] == end[:2]
    assert lift[3:] == end[3:]
    assert lift[2] > end[2]
    axes = zyz_axes(end[3:])
    edge = tuple(end[i]+sum(axes[j][i]*EDGE_OFFSET[j] for j in range(3)) for i in range(3))
    cx, cy, bottom = args.container_center
    assert edge == pytest.approx((cx+args.container_size[0]/4, cy,
                                  bottom+args.container_size[2]-args.scoop_depth))


@pytest.mark.parametrize('angle', [0, 90, -45, float('nan')])
def test_invalid_angle(angle):
    with pytest.raises(ValueError):
        validate(settings(scoop_angle=angle))
