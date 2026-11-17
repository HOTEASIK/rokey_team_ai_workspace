"""연속 요청 구성, 비선형 기울기, 새 벽 접촉 경로 검사."""

import pytest

from putty_demo.putty_motion import continuous_commands, SplineStroke, spline_request
from putty_demo.apply_putty_plane import (
    stroke_poses, application_moves, resolve_plane_args, WALL_EDGE_OFFSET,
)
from test_apply_putty_plane import settings


def wall_args():
    args = settings('yz', 'negative')
    args.case, args.start, args.end = 'plane', None, None
    args.stroke_height = 180
    args.blade_end_angle = 30
    args.edge_offset = WALL_EDGE_OFFSET
    resolve_plane_args(args)
    return args


def test_angle_changes_linearly_with_stroke_distance():
    args = wall_args()
    poses = list(stroke_poses(args))
    assert poses[0][0] == pytest.approx((789.04, -15.52, 27.20))
    assert poses[-1][0] == pytest.approx((789.04, -15.52, 207.20))
    assert poses[0][2] == 45
    assert poses[10][2] == pytest.approx(37.5)
    assert poses[-1][2] == 30
    changes = [a[2]-b[2] for a, b in zip(poses, poses[1:])]
    assert changes == pytest.approx([15/20]*20)


def test_one_stroke_is_one_spline_with_approach_and_retreat_separate():
    args = wall_args()
    moves = list(application_moves((300, 0, 300, 0, 135, 0), args))
    commands = list(continuous_commands(moves))
    splines = [c for c in commands if isinstance(c, SplineStroke)]
    assert len(splines) == 1
    assert len(splines[0].poses) == 21
    assert splines[0].poses[0] == moves[3][1]
    assert splines[0].poses[-1] == moves[-2][1]
    assert commands[0] == moves[0]
    assert commands[-1] == moves[-1]


def test_spline_request_uses_all_poses_and_existing_speeds():
    args = wall_args()
    points = tuple(p for _, p, _ in stroke_poses(args))
    request = spline_request(points, args.apply_speed, args.acceleration)
    assert request.pos_cnt == 21
    assert list(request.pos[0].data) == list(points[0])
    assert list(request.pos[-1].data) == list(points[-1])
    assert list(request.vel) == [args.apply_speed, 10]
    assert list(request.acc) == [args.acceleration, 20]
    assert request.ref == request.mode == request.sync_type == 0


def test_constant_angle_still_groups_for_force_mode():
    args = wall_args()
    args.blade_end_angle = args.blade_angle
    moves = list(application_moves((300, 0, 300, 0, 135, 0), args))
    splines = [c for c in continuous_commands(moves) if isinstance(c, SplineStroke)]
    assert len(splines) == 1
    assert len(splines[0].poses) == 2
