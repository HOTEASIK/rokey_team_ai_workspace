"""로봇 없이 벽 내부 탐색의 경로(테두리·바닥 여유, 가로/세로)와 틈/돌출 분석을 검사한다."""

import math

import numpy as np
import pytest

from navigation.wall_scanner import (
    CORNERS, VirtualWallScan, analyze, make_cols, make_lines, make_rows, wall_point,
)


def test_rows_stay_inside_border_with_margins_and_zigzag():
    rows = make_rows(CORNERS, row_step=20.0, margin=15.0, bottom_margin=15.0)
    assert len(rows) >= 7
    for r in rows:
        for p in (r['start'], r['end']):
            # 같은 Y 위치의 아래/위 테두리 높이와 비교 (좌우 테두리 높이가 조금씩 다름)
            u = (CORNERS['bottom_left'][1]-p[1])/(CORNERS['bottom_left'][1]-CORNERS['bottom_right'][1])
            assert wall_point(CORNERS, u, 0.0)[2]+14.0 <= p[2] <= wall_point(CORNERS, u, 1.0)[2]-14.0
        # 좌우 여유: 그 줄 높이(v)에서의 왼쪽/오른쪽 테두리 기준 (테두리가 조금 기울어 있음)
        left, right = wall_point(CORNERS, 0.0, r['v']), wall_point(CORNERS, 1.0, r['v'])
        ys = sorted((r['start'][1], r['end'][1]))
        assert ys[0] >= right[1]+14.9 and ys[1] <= left[1]-14.9
    # 지그재그: 이웃 줄은 반대 방향
    assert np.sign(rows[0]['end'][1]-rows[0]['start'][1]) == -np.sign(rows[1]['end'][1]-rows[1]['start'][1])
    # 같은 쪽(왼쪽 끝) 높이로 줄 간격 비교 → 요청한 20 mm 를 넘지 않음
    left_z = [wall_point(CORNERS, 0.0, r['v'])[2] for r in rows]
    assert max(b-a for a, b in zip(left_z, left_z[1:])) <= 20.0+1e-6


def test_cols_cover_width_with_step_and_alternate_direction():
    cols = make_cols(CORNERS, col_step=20.0, margin=15.0)
    ys = [wall_point(CORNERS, c['u'], 0.5)[1] for c in cols]     # 같은 높이에서 열 간격 비교
    assert max(abs(b-a) for a, b in zip(ys, ys[1:])) <= 20.0+0.5
    assert np.sign(cols[0]['end'][2]-cols[0]['start'][2]) == -np.sign(cols[1]['end'][2]-cols[1]['start'][2])
    lines = make_lines(CORNERS, dirs='both')
    assert [line['line'] for line in lines] == list(range(len(lines)))
    assert {line['dir'] for line in lines} == {'h', 'v'}


def test_floor_raises_bottom_row_on_both_sides():
    floor_z = 45.0
    rows = make_rows(CORNERS, row_step=20.0, margin=15.0, bottom_margin=5.0, floor_z=floor_z, floor_clearance=10.0)
    assert min(min(r['start'][2], r['end'][2]) for r in rows) >= floor_z+10.0-0.01
    cols = make_cols(CORNERS, margin=15.0, bottom_margin=5.0, floor_z=floor_z, floor_clearance=10.0)
    assert min(min(c['start'][2], c['end'][2]) for c in cols) >= floor_z+10.0-0.5


def test_impossible_margins_rejected():
    with pytest.raises(ValueError):
        make_rows(CORNERS, margin=200.0, bottom_margin=200.0)


def simulate(wall, lines, step=2.0, press=0.25):
    samples = []
    for line in lines:
        a, b = np.array(line['start']), np.array(line['end'])
        n = int(np.linalg.norm(b-a)/step)
        for t in np.linspace(0, 1, n+1):
            p = a+(b-a)*t
            p[0] = wall.surface_x(p[1], p[2])+press
            samples.append({'line': line['line'], 'dir': line['dir'], 'index': line['index'],
                            'xyz': p.tolist(), 'force': 5.0, 'contact': True})
    return samples


def test_rows_and_cols_find_vertical_groove_thin_horizontal_crack_and_bump():
    wall = VirtualWallScan(CORNERS)
    lines = make_lines(CORNERS, row_step=20.0, col_step=20.0, dirs='both')
    result = analyze(simulate(wall, lines), lines)
    gaps = [r for r in result['regions'] if r['kind'] == 'gap']
    bumps = [r for r in result['regions'] if r['kind'] == 'bump']
    vertical = [r for r in gaps if r['orientation'] == 'vertical']
    horizontal = [r for r in gaps if r['orientation'] == 'horizontal']
    assert vertical and -10 <= vertical[0]['y_range'][0] and vertical[0]['y_range'][1] <= 10
    assert vertical[0]['max_depth'] == pytest.approx(4.0, abs=0.5)
    # 6 mm 두께 가로 틈: 가로 줄 사이로 빠지지만 세로 열이 가로질러 찾는다
    assert horizontal and horizontal[0]['cols']
    assert horizontal[0]['y_range'][0] <= -130 and horizontal[0]['y_range'][1] >= -80
    assert bumps and bumps[0]['max_depth'] == pytest.approx(-3.0, abs=0.5)
    assert result['plane']['rms_mm'] < 1.0


def test_flat_wall_has_no_regions_and_contact_loss_is_gap():
    wall = VirtualWallScan(CORNERS, defects=[])
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    samples = simulate(wall, lines)
    assert analyze(samples, lines)['regions'] == []
    first = lines[0]['line']
    for s in samples:
        if s['line'] == first and abs(s['xyz'][1]) <= 5.0:
            s['contact'] = False          # 한 줄 가운데 10 mm 동안 접촉 끊김 → 틈
    gaps = [r for r in analyze(samples, lines)['regions'] if r['kind'] == 'gap']
    assert len(gaps) == 1 and gaps[0]['rows'] == [lines[0]['index']]


def test_wall_point_corners():
    assert wall_point(CORNERS, 0, 0) == CORNERS['bottom_left']
    assert wall_point(CORNERS, 1, 1) == CORNERS['top_right']
    mid = wall_point(CORNERS, 0.5, 0.5)
    assert math.isclose(mid[1], np.mean([c[1] for c in CORNERS.values()]), abs_tol=1.0)


def test_probe_offset_points_along_tool_axis_toward_wall_and_down():
    from navigation.wall_scanner import TOOL_ORI, tool_z_axis
    z = tool_z_axis(TOOL_ORI)
    assert np.linalg.norm(z) == pytest.approx(1.0)
    assert z[0] > 0.5 and z[2] < -0.5     # 탐침은 벽(+X) 쪽 아래로 뻗음


def test_process_result_fields_wall_edges_and_vertical_gap_lines():
    from navigation.wall_scanner import vertical_gap_lines, wall_edges
    edges = np.ravel(wall_edges(CORNERS))
    assert len(edges) == 12                               # Process.Result.wall float64[12]
    top, floor = edges[:3], edges[3:6]
    assert top[2] > 200 and floor[2] < 40
    lines = make_lines(CORNERS, dirs='rows')
    result = analyze(simulate(VirtualWallScan(CORNERS), lines), lines)
    segs = vertical_gap_lines(result)
    # 가상 세로 홈(y≈0) 하나만 세로선으로. 한 줄에만 걸린 가로 홈·돌출은 내보내지 않음
    assert len(segs) == 1 and len(segs[0]) == 6
    sx, sy, sz, ex, ey, ez = segs[0]
    assert abs(sy) < 5 and sy == ey and sz < ez          # 같은 Y, 아래 → 위
    assert sx == ex                                       # X 는 양 끝 같은 값
    wall = VirtualWallScan(CORNERS, defects=[])
    assert sx == pytest.approx(min(wall.surface_x(0.0, sz), wall.surface_x(0.0, ez)), abs=0.3)   # 홈 속이 아니라 벽면


def gap_piece(row, y, z=None):
    """vertical_gap_lines 입력용 가로 줄 틈 선분 (폭 8 mm)."""
    z = 50.0+20.0*row if z is None else z
    return {'kind': 'gap', 'dir': 'h', 'index': row, 'y_range': [y-4.0, y+4.0], 'z_range': [z, z]}


def test_vertical_gap_lines_link_rows_drop_single_and_merge_same_y():
    from navigation.wall_scanner import vertical_gap_lines
    plane = {'normal': [-1.0, 0.0, 0.0], 'd': 750.0}          # 벽 X = 750
    segments = ([gap_piece(r, 40.0+0.5*r) for r in (0, 1, 2)]   # 3줄 연속 → 세로 틈
                + [gap_piece(3, -80.0)]                        # 1줄만 → 버림
                + [gap_piece(r, -55.0) for r in (0, 1)]        # 2줄 …
                + [gap_piece(r, -56.0) for r in (5, 6)]        # … 3줄 건너 2줄 → 같은 Y 라 하나로 합침
                + [gap_piece(2, 100.0), gap_piece(4, 100.0)]   # 1줄 건너 1줄씩 (가짜 끊김 모양) → 버림
                + [gap_piece(1, 150.0), gap_piece(4, 150.0)]   # 2줄 놓침 → 각각 1줄 → 버림
                + [{'kind': 'bump', 'dir': 'h', 'index': 0, 'y_range': [0, 8], 'z_range': [50, 50]}])
    lines = vertical_gap_lines({'plane': plane, 'segments': segments})
    ys = [line[1] for line in lines]
    assert ys == pytest.approx([40.5, -55.5])                  # 왼쪽(+Y)부터 평균 Y, 100/150/−80 은 버림
    merged = lines[1]
    assert merged[2] == pytest.approx(50.0) and merged[5] == pytest.approx(170.0)   # 줄 0 → 줄 6
    assert merged[0] == pytest.approx(750.0) and merged[3] == pytest.approx(750.0)  # X = 벽 평면
    # 틈 옆 벽면 X 를 잰 줄이 있으면 그 값들 중 가장 작은 값(로봇 쪽)으로 양 끝 통일
    measured = [dict(gap_piece(r, 40.0), wall_x=753.0+0.5*r) for r in (0, 1, 2)]
    line = vertical_gap_lines({'plane': plane, 'segments': measured})[0]
    assert line[0] == pytest.approx(753.0) and line[3] == pytest.approx(753.0)
    relaxed = vertical_gap_lines({'plane': plane, 'segments': segments}, max_missed=1)
    assert [round(line[1]) for line in relaxed][0] == 100                     # 1줄 놓침 허용하면 연결


# ----------------------------------------------------------------------------- 테두리 학습 / 재탐색
from types import SimpleNamespace

from navigation.wall_scanner import (
    WallScanner, adjust_line, base_requirements, border_distances,
)


def test_adjust_line_keeps_line_when_no_extra_and_moves_inward_with_extra():
    lines = make_lines(CORNERS, dirs='both')
    base = base_requirements(lines, CORNERS)
    row0 = next(line for line in lines if line['dir'] == 'h' and line['index'] == 0)
    same = adjust_line(row0, CORNERS, base)
    assert np.allclose(same['start'], row0['start'], atol=0.5) and np.allclose(same['end'], row0['end'], atol=0.5)
    up = adjust_line(row0, CORNERS, {**base, 'bottom': base['bottom']+10})
    assert min(up['start'][2], up['end'][2]) >= min(row0['start'][2], row0['end'][2])+9.0     # 바닥 쪽 → 위로 이동
    trimmed = adjust_line(row0, CORNERS, {**base, 'left': base['left']+20})
    left_end = max(trimmed['start'][1], trimmed['end'][1])
    assert left_end <= max(row0['start'][1], row0['end'][1])-19.0                                # 왼쪽 끝 → 줄임
    col0 = next(line for line in lines if line['dir'] == 'v' and line['index'] == 0)
    shifted = adjust_line(col0, CORNERS, {**base, 'left': base['left']+10})
    assert shifted['start'][1] <= col0['start'][1]-9.0                                          # 왼쪽 열 → 오른쪽(−Y)으로
    assert adjust_line(row0, CORNERS, {**base, 'left': 470.0}) is None                          # 벽 너비보다 크면 다 잘려 건너뜀


class LipIO:
    """가짜 실물 로봇: 평면 벽 + 바닥 쪽 '턱'(z < lip_z 에서 옆으로 20 N). 순응제어처럼 벽에서 막힌다."""

    mode = 'real'

    def __init__(self, lip_z):
        self.lip_z = lip_z
        self.pos = np.array([700.0, 0.0, 150.0])
        self.goal = None
        self.wall = VirtualWallScan(CORNERS, defects=[])
        self.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=print, warn=print))
        self.moves = []

    def surface(self, p):
        return self.wall.surface_x(p[1], p[2])

    def _advance(self):
        if self.goal is None:
            return
        step = self.goal-self.pos
        dist = np.linalg.norm(step)
        self.pos = self.goal.copy() if dist <= 4.0 else self.pos+step/dist*4.0
        self.pos[0] = min(self.pos[0], self.surface(self.pos))   # 벽에 막힘
        self.cmd_x = (self.goal if dist <= 4.0 else self.pos)[0]

    def get_pos(self):
        self._advance()
        return self.pos.copy()

    def get_force(self, pos=None):
        p = self.pos
        f = np.zeros(6)
        target_x = self.goal[0] if self.goal is not None else p[0]
        if target_x > self.surface(p)-0.01:
            f[0] = -min(10.0, (target_x-self.surface(p))*0.5)
        if p[2] < self.lip_z and abs(p[0]-self.surface(p)) < 1.0:
            f[1] = 20.0
        return f

    def move_l(self, xyz, vel=0, acc=0):
        self.goal = None
        self.pos = np.array(xyz, float)[:3]
        self.pos[0] = min(self.pos[0], self.surface(self.pos))
        self.moves.append(self.pos.copy())

    def amove_l(self, xyz, vel, acc):
        self.goal = np.array(xyz, float)[:3]

    def is_moving(self):
        return self.goal is not None and np.linalg.norm(self.goal[1:]-self.pos[1:]) > 0.5

    def wait_motion_done(self, timeout=10.0):
        while self.is_moving():
            self._advance()

    def stop(self):
        self.goal = None

    def zero_force(self):
        pass

    def start_compliance(self, k):
        pass

    def release_press(self):
        pass


def test_scanner_moves_bottom_row_up_after_hitting_lip_and_continues():
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    bottom_z = min(min(line['start'][2], line['end'][2]) for line in lines)
    io = LipIO(lip_z=bottom_z+8.0)                       # 맨 아래 줄 높이 근처에 턱
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_method='spring')
    scanner.left_home = True
    scanner.base = base_requirements(lines, CORNERS)
    row0 = lines[0]
    n = scanner.scan_line_with_retry(row0)
    assert n > 50                                        # 옮긴 뒤 끝까지 기록
    # 줄 시작점이 아래/왼쪽 테두리에서 같은 거리라 먼저 'left' 를 의심할 수 있다 → 결국 'bottom' 을 학습하면 됨
    assert 'bottom' in {hit['border'] for hit in scanner.border_hits}
    assert scanner.extra['bottom'] >= 10.0
    zs = [s['xyz'][2] for s in scanner.samples if s['line'] == row0['line']]
    assert max(zs) >= bottom_z+9.0                        # 재탐색은 위로 옮겨진 높이에서
    # 학습한 여유는 다음 줄에도 적용된다 (아래 테두리 거리 ≥ 기본 + 학습 여유)
    adj = adjust_line(lines[1], CORNERS, scanner.required())
    d = min(border_distances(CORNERS, p)['bottom'] for p in (adj['start'], adj['end']))
    assert d >= scanner.base['bottom']+scanner.extra['bottom']-0.5


def test_tool_roll_keeps_probe_axis_and_falls_back_to_opposite_direction():
    from navigation.wall_scanner import TOOL_ORI, rolled, tool_z_axis
    r = rolled(TOOL_ORI, 90.0)
    assert r[:2] == TOOL_ORI[:2] and r[2] == pytest.approx(-178.82)
    assert np.allclose(tool_z_axis(r), tool_z_axis(TOOL_ORI))     # 탐침 방향(끝 위치) 그대로

    class RollIO:
        mode = 'real'
        node = SimpleNamespace(get_logger=lambda: SimpleNamespace(info=print, warn=print))

        def __init__(self):
            self.tool_roll, self.tool_ori = 90.0, rolled(TOOL_ORI, 90.0)
            self.dsr = SimpleNamespace(get_current_solution_space=lambda: 2)

        def joints_at(self, xyz, sol):
            # +90° 회전 자세는 손목 특이점(J5 5°), -90° 는 정상이라고 가정
            return [0, 30, 80, 0, 5.0 if self.tool_roll > 0 else 40.0, 0]

    io = RollIO()
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_method='spring')
    scanner.preflight_with_roll(make_lines(CORNERS, dirs='rows'))
    assert io.tool_roll == -90.0 and io.tool_ori == rolled(TOOL_ORI, -90.0)


class BoardIO(LipIO):
    """벽 가운데(|y| < 10 mm)에 붙인 폼보드: 그 위에서 옆 방향 20 N."""

    def get_force(self, pos=None):
        f = super().get_force(pos)
        if abs(self.pos[1]) < 10.0 and abs(self.pos[0]-self.surface(self.pos)) < 1.0:
            f[1] = 20.0
        return f


def test_after_calibration_interior_hit_is_saved_as_obstacle_and_line_continues():
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    io = BoardIO(lip_z=-1000.0)                          # 바닥 턱 없음
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_method='spring')
    scanner.left_home = True
    scanner.base = base_requirements(lines, CORNERS)
    row = lines[3]
    # 위치 확인 전: 테두리로 보고 여유를 늘린다 (기존 동작)
    before = WallScanner(BoardIO(lip_z=-1000.0), CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_method='spring')
    before.left_home, before.base = True, scanner.base
    before.scan_line_with_retry(row)
    assert before.border_hits and not before.obstacles
    # 위치 확인 뒤: 장애물 추정으로 저장하고 건너뛰어 끝까지
    scanner.calibrated = True
    scanner.scan_line_with_retry(row)
    assert scanner.obstacles and abs(scanner.obstacles[0]['pos'][1]) < 15
    assert all(v == 0.0 for v in scanner.extra.values())   # 테두리 여유는 그대로
    ys = [s['xyz'][1] for s in scanner.samples if s['line'] == row['line']]
    assert max(ys) > 150 and min(ys) < -150                 # 장애물 양쪽 모두 기록


def test_suspect_segment_spans_skipped_part_for_process_result():
    from navigation.wall_scanner import suspect_lines
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    scanner = WallScanner(BoardIO(lip_z=-1000.0), CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_method='spring')
    scanner.left_home, scanner.base, scanner.calibrated = True, base_requirements(lines, CORNERS), True
    scanner.scan_line_with_retry(lines[3])
    segs = suspect_lines(scanner.obstacles)
    assert segs and len(segs[0]) == 6
    assert math.dist(segs[0][:3], segs[0][3:]) == pytest.approx(30.0, abs=1.0)   # 걸린 곳 ~ 30 mm 건너뛴 끝


class ForceIO(LipIO):
    """가짜 힘제어: 힘제어가 켜지면 탐침이 벽면(구멍 포함)을 따라간다. 가운데 |y|<10 은 20 mm 깊은 구멍."""

    def __init__(self):
        super().__init__(lip_z=-1000.0)
        self.force_on = False
        self.calls = []
        self.dsr = SimpleNamespace(
            DR_FC_MOD_REL=1,
            set_stiffnessx=lambda stx, time=0: self.calls.append(('stiffness', stx[0])),
            set_desired_force=lambda fd, dir, time=0, mod=0: self.calls.append(('force', fd[0], mod)))

    def surface(self, p):
        return super().surface(p)+(20.0 if abs(p[1]) < 10.0 else 0.0)

    def _advance(self):
        super()._advance()
        if self.force_on:
            self.pos[0] = self.surface(self.pos)          # 힘제어가 X 를 벽면에 붙여 따라감

    def get_force(self, pos=None):
        if self.force_on:
            return np.array([-3.0, 0, 0, 0, 0, 0])
        return super().get_force(pos)

    def release_press(self):
        self.force_on = False


def test_force_mode_presses_gently_and_treats_deep_dive_as_gap_suspect():
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    io = ForceIO()
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False)   # 기본 = force, 2 N
    scanner.left_home, scanner.base = True, base_requirements(lines, CORNERS)
    scanner.scan_line_with_retry(lines[3])
    assert ('stiffness', 200.0) in io.calls and ('force', 2.0, 1) in io.calls   # 강성 200, 상대 2 N
    assert scanner.obstacles and abs(scanner.obstacles[0]['pos'][1]) < 15        # 구멍 → 틈 추정
    assert all(v == 0.0 for v in scanner.extra.values())                          # 테두리로 오판 안 함
    ys = [s['xyz'][1] for s in scanner.samples if s['line'] == lines[3]['line']]
    assert max(ys) > 150 and min(ys) < -150                                      # 구멍 건너 양쪽 기록
    with pytest.raises(ValueError):
        WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_force=8.0)  # 너무 센 힘 거부


# ----------------------------------------------------------------------------- 접촉 판정 (힘 + 위치)
from navigation.wall_scanner import ContactDetector, contact_state, short_contact_losses


def feed(detector, samples, period=0.02):
    """(X, 힘) 목록을 20 ms 간격으로 넣고, 처음 접촉으로 판정된 샘플 번호 (없으면 None)."""
    for i, (x, force) in enumerate(samples):
        if detector.update(i*period, x, force):
            return i
    return None


def test_contact_detector_ignores_force_while_still_advancing():
    # 허공: 힘은 2 N 으로 읽히지만 5 mm/s 로 계속 전진 → 접촉 아님 (2026-11-20 실측 상황)
    moving = [(742.0+0.1*i, 2.0) for i in range(100)]
    assert feed(ContactDetector(1.0), moving) is None


def test_contact_detector_needs_stall_and_held_force_after_settle():
    # 0.3 s 전진 후 벽(747)에 막힘 + 힘 3 N → 출발 0.5 s 지나고 멈춤 0.2 s 확인 뒤 접촉
    samples = [(742.0+0.1*i, 0.0) for i in range(15)] + [(743.5, 3.0)]*60
    hit = feed(ContactDetector(1.0), samples)
    assert hit is not None and hit*0.02 >= 0.5
    # 멈췄지만 힘이 없음 (목표 도착 등) → 접촉 아님
    assert feed(ContactDetector(1.0), [(743.5, 0.2)]*60) is None
    # 출발 직후 '덜컥'으로 잠깐 멈춘 것처럼 보이고 힘이 튀어도, settle 동안은 접촉 아님
    jolt = [(742.0, 3.0)]*20 + [(742.0+0.1*i, 0.3) for i in range(60)]
    assert feed(ContactDetector(1.0), jolt) is None


def test_contact_state_hysteresis():
    assert contact_state(0.8, True, 1.0, 0.5) is True       # 끊김 기준 위 → 접촉 유지
    assert contact_state(0.8, False, 1.0, 0.5) is False     # 접촉 기준 아래 → 끊김 유지
    assert contact_state(0.4, True, 1.0, 0.5) is False
    assert contact_state(1.2, False, 1.0, 0.5) is True


def test_short_contact_loss_is_not_a_gap():
    pts = [{'xyz': [750.0, float(y), 100.0], 'contact': not (10 <= y <= 11 or 30 <= y <= 40)} for y in range(50)]
    short = short_contact_losses(pts, axis=1, min_mm=3.0)
    assert {p['xyz'][1] for p in pts if id(p) in short} == {10.0, 11.0}   # 1 mm 끊김만 잡음


class FalseForceIO(LipIO):
    """허공에서도 벽 쪽 힘이 2 N 으로 읽히는 가짜 로봇 (순응제어 전환 뒤 힘 오프셋 재현)."""

    def get_force(self, pos=None):
        f = super().get_force(pos)
        if abs(f[0]) < 2.0:
            f[0] = -2.0
        return f


def test_attach_does_not_stop_in_air_on_false_force():
    io = FalseForceIO(lip_z=-1000.0)
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False)
    line = make_lines(CORNERS, row_step=20.0, dirs='rows')[3]
    start = np.array(line['start'])
    io.move_l(start-np.array([5.0, 0, 0]))                  # 벽 앞 5 mm (PRE_GAP)
    assert scanner._attach(line, start+np.array([30.0, 0, 0]), 1.0, stop_on_contact=True)
    assert io.pos[0] == pytest.approx(io.surface(io.pos), abs=0.5)   # 허공이 아니라 실제 벽에서 멈춤


class AirIO(ForceIO):
    """가짜 힘제어: 가운데 |y| < 40 은 벽이 없음 → 힘 0, 힘제어가 허공으로 조금씩(1 mm) 밀고 들어감."""

    def surface(self, p):
        return LipIO.surface(self, p)+(50.0 if abs(p[1]) < 40.0 else 0.0)

    def _advance(self):
        LipIO._advance(self)
        if self.force_on:
            wall = LipIO.surface(self, self.pos)
            in_air = abs(self.pos[1]) < 40.0
            self.pos[0] = self.pos[0]+1.0 if in_air else wall

    def get_force(self, pos=None):
        if self.force_on:
            return np.zeros(6) if abs(self.pos[1]) < 40.0 else np.array([-3.0, 0, 0, 0, 0, 0])
        return LipIO.get_force(self, pos)


def test_force_mode_stops_early_when_pushing_into_air():
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    io = AirIO()
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False)
    scanner.left_home, scanner.base, scanner.calibrated = True, base_requirements(lines, CORNERS), True
    scanner.scan_line_with_retry(lines[3])
    assert scanner.obstacles                                         # 허공 → 틈 추정으로 정지
    first = scanner.obstacles[0]['pos']
    assert first[0]-CORNERS['bottom_left'][0] < 6.0     # 8 mm(MAX_DIVE_MM) 까지 기다리지 않고 멈춤


# ----------------------------------------------------------------------------- 높이별 누르는 힘
from navigation.wall_scanner import scaled_scan_force


def test_scaled_scan_force_bottom_top_and_between():
    assert scaled_scan_force(3.0, 1.3, 48.0, 48.0, 189.0) == pytest.approx(3.0)        # 맨 아래 줄
    assert scaled_scan_force(3.0, 1.3, 189.0, 48.0, 189.0) == pytest.approx(3.9)       # 맨 위 줄
    assert scaled_scan_force(3.0, 1.3, 118.5, 48.0, 189.0) == pytest.approx(3.45)      # 가운데
    assert scaled_scan_force(3.0, 1.3, 500.0, 48.0, 189.0) == pytest.approx(3.9)       # 범위 밖 → 끝 값
    assert scaled_scan_force(3.0, 1.3, 100.0, 100.0, 100.0) == pytest.approx(3.0)      # 줄 하나뿐


def test_force_mode_uses_height_scaled_force_per_row():
    lines = make_lines(CORNERS, row_step=20.0, dirs='rows')
    io = ForceIO()
    scanner = WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_force=3.0, top_force_ratio=1.3)
    heights = [(line['start'][2]+line['end'][2])/2 for line in lines]
    scanner.height_range = (min(heights), max(heights))
    assert scanner.line_force(lines[0]) == pytest.approx(3.0)
    assert scanner.line_force(lines[-1]) == pytest.approx(3.9)
    scanner.left_home, scanner.base = True, base_requirements(lines, CORNERS)
    scanner.scan_line_with_retry(lines[-1])
    assert ('force', pytest.approx(3.9), 1) in io.calls                                # 맨 위 줄은 3.9 N 으로 누름
    with pytest.raises(ValueError):
        WallScanner(io, CORNERS, 20.0, 20.0, 'rows', 15.0, False, scan_force=4.0, top_force_ratio=1.5)   # 6 N > 5 N


def test_empty_or_zero_top_force_ratio_means_no_correction():
    from navigation.wall_scanner import top_force_ratio_or_default
    assert top_force_ratio_or_default(0.0) == 1.0          # HMI 빈 칸 → Number('') = 0
    assert top_force_ratio_or_default('') == 1.0
    assert top_force_ratio_or_default(None) == 1.0
    assert top_force_ratio_or_default(-1.0) == 1.0
    assert top_force_ratio_or_default(1.3) == 1.3
