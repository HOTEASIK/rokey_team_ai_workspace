"""평면 법선, 날 자세, TCP 오프셋 및 후퇴 방향 검사."""

import math
from types import SimpleNamespace

import pytest

from putty_demo.putty_geometry import EDGE_OFFSET, PICKUP_ABC, cross, zyz_axes
from putty_demo.apply_putty_plane import (
    PLANE_AXIS, application_moves, validate,
)


def settings(plane='xy', sign='positive', angle=45):
    start = [400., 0., 200.]
    end = list(start)
    end[(PLANE_AXIS[plane]+1) % 3] += 100
    return SimpleNamespace(plane=plane, normal_sign=sign, blade_angle=angle, blade_end_angle=angle, samples=21,
                           approach_distance=50, start=start, end=end,
                           edge_offset=EDGE_OFFSET, transfer_z=300, surface_z=35,
                           j6_delta=0, j6_speed=20, j6_acceleration=20,
                           transfer_speed=30, apply_speed=10, acceleration=20)


@pytest.mark.parametrize('plane', PLANE_AXIS)
@pytest.mark.parametrize('sign', ['positive', 'negative'])
@pytest.mark.parametrize('angle', [30, 45, 60])
def test_plane_geometry(plane, sign, angle):
    args = settings(plane, sign, angle)
    moves = list(application_moves((500, 0, 300, *PICKUP_ABC), args))
    approach, start, end, retreat = [move[1] for move in moves[2:]]
    assert approach[3:] == start[3:] == end[3:] == retreat[3:]
    normal = tuple((1 if sign == 'positive' else -1) if i == PLANE_AXIS[plane]
                   else 0 for i in range(3))
    for i in range(3):
        assert approach[i]-start[i] == pytest.approx(50*normal[i])
        assert retreat[i]-end[i] == pytest.approx(50*normal[i])
    axes = zyz_axes(start[3:])
    assert cross(axes[0], axes[1]) == pytest.approx(axes[2])
    for target, edge in ((start, args.start), (end, args.end)):
        actual_edge = tuple(target[i]+sum(axes[j][i]*EDGE_OFFSET[j] for j in range(3))
                            for i in range(3))
        assert actual_edge == pytest.approx(edge)
    initial = zyz_axes(PICKUP_ABC)
    face = tuple(sum(axes[j][i]*initial[j][0] for j in range(3)) for i in range(3))
    handle = tuple(sum(axes[j][i]*initial[j][2] for j in range(3)) for i in range(3))
    assert sum(f*n for f, n in zip(face, normal)) == pytest.approx(-math.cos(math.radians(angle)))
    assert sum(h*n for h, n in zip(handle, normal)) == pytest.approx(math.sin(math.radians(angle)))


def test_reject_wrong_plane_and_empty_stroke():
    args = settings()
    args.end[2] += 1
    with pytest.raises(ValueError):
        validate(args)

    args.end = list(args.start)
    with pytest.raises(ValueError):
        validate(args)


@pytest.mark.parametrize('plane', PLANE_AXIS)
@pytest.mark.parametrize('sign', ['positive', 'negative'])
def test_progressive_tilt_preserves_surface_contact(plane, sign):
    from putty_demo.apply_putty_plane import stroke_poses, single_stroke_moves
    args = settings(plane, sign)
    args.blade_end_angle = 30
    if plane == 'yz':
        args.end = [args.start[0], args.start[1], args.start[2]+150]
    validate(args)
    poses = list(stroke_poses(args))
    assert len(poses) == 21
    assert poses[0][2] == 45
    assert poses[-1][2] == 30
    assert all(a[2] >= b[2] for a, b in zip(poses, poses[1:]))
    axis = PLANE_AXIS[plane]
    for edge, target, tilt in poses:
        axes = zyz_axes(target[3:])
        recovered = tuple(target[i]+sum(axes[j][i]*EDGE_OFFSET[j] for j in range(3))
                          for i in range(3))
        assert recovered == pytest.approx(edge)
        assert edge[axis] == pytest.approx(args.start[axis])
        initial = zyz_axes(PICKUP_ABC)
        handle = tuple(sum(axes[j][i]*initial[j][2] for j in range(3)) for i in range(3))
        outward = 1 if sign == 'positive' else -1
        assert handle[axis]*outward == pytest.approx(math.sin(math.radians(tilt)))
    assert poses[0][0] == pytest.approx(args.start)
    assert poses[-1][0] == pytest.approx(args.end)
    moves = list(single_stroke_moves((500, 0, 300, *PICKUP_ABC), args))
    assert moves[-1][1][3:] == poses[-1][1][3:]
    assert moves[3][1][3:] == poses[0][1][3:]


@pytest.mark.parametrize('angle', [0, 90, float('nan')])
def test_invalid_end_tilt(angle):
    args = settings()
    args.blade_end_angle = angle
    with pytest.raises(ValueError):
        validate(args)


def test_measured_yz_contact_stroke_height_and_tilts():
    from putty_demo.apply_putty_plane import resolve_plane_args, stroke_poses
    args = settings('yz', 'negative')
    args.case = 'plane'
    args.start = args.end = None
    args.stroke_height = 180
    args.blade_end_angle = 30
    resolve_plane_args(args)
    validate(args)
    assert args.start == (789.04, -15.52, 27.20)
    assert args.end == (789.04, -15.52, 207.20)
    poses = list(stroke_poses(args))
    for edge, target, angle in poses:
        assert edge[:2] == pytest.approx((789.04, -15.52))
        axes = zyz_axes(target[3:])
        # 모델의 날 폭 방향이 BASE Y와 평행한지 확인한다.
        initial = zyz_axes(PICKUP_ABC)
        width = tuple(sum(axes[j][i]*initial[j][1] for j in range(3)) for i in range(3))
        assert abs(width[1]) == pytest.approx(1)
        assert width[0] == pytest.approx(0, abs=1e-10)
        assert width[2] == pytest.approx(0, abs=1e-10)
    assert poses[0][2] == 45
    assert poses[-1][2] == 30


def test_height_resolution_preserves_explicit_end():
    from putty_demo.apply_putty_plane import resolve_plane_args
    args = settings('yz')
    args.case = 'plane'
    original = args.end
    resolve_plane_args(args)
    assert args.end is original


def test_plane_runner_disables_joint_stage(monkeypatch):
    from putty_demo import apply_putty_plane as module
    calls = []
    monkeypatch.setattr(module, 'run_motion', lambda *args, **kwargs: calls.append(kwargs))
    module.run_robot(settings(), motion_builder=module.application_moves)
    assert calls[0]['use_j6'] is False
    assert calls[0]['allow_real'] is True
    assert calls[0]['continuous'] is True
    assert calls[0]['press_force'] is None


def test_contact_mode_restores_positive_x_six_newton_hold(monkeypatch):
    from putty_demo import apply_putty_plane as module
    calls = []
    monkeypatch.setattr(module, 'run_motion', lambda *args, **kwargs: calls.append(kwargs))
    values = settings('yz', 'negative')
    values.contact_search = True
    values.press_force = 6.0
    module.run_robot(values, motion_builder=module.application_moves)
    assert calls[0]['press_force'] == 6.0
    assert calls[0]['contact_probe'] is not None
    assert calls[0]['use_j6'] is False


def test_yz_default_approaches_in_positive_x():
    from putty_demo.apply_putty_plane import resolve_plane_args, single_stroke_moves
    args = settings('yz')
    args.case, args.start, args.end, args.normal_sign = 'plane', None, None, None
    args.stroke_height = 180
    args.blade_end_angle = 30
    resolve_plane_args(args)
    assert args.normal_sign == 'negative'
    moves = list(single_stroke_moves((300, 0, 300, *PICKUP_ABC), args))
    approach, contact = moves[2][1], moves[3][1]
    assert contact[0]-approach[0] == pytest.approx(50)
    assert moves[-1][1][0]-moves[-2][1][0] == pytest.approx(-50)


def test_plane_parser_has_no_j6_options():
    from putty_demo.putty_motion import make_parser
    parser = make_parser(include_j6=False)
    args = parser.parse_args(['--execute-motion'])
    assert not hasattr(args, 'j6_delta')
    assert not hasattr(args, 'j6_speed')
    with pytest.raises(SystemExit):
        parser.parse_args(['--execute-motion', '--j6-delta', '180'])

