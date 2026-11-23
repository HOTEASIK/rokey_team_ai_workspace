#!/usr/bin/env python3
"""벽 내부 탐색: 테두리(네 모서리) 안쪽을 탐침으로 누른 채 훑으며 표면을 전부 기록하고 틈/홈/돌출을 찾는다.

[순서]
  0. 벽면 크기 탐색(wall_size 등)으로 구한 네 모서리 → 테두리. 여기서 여유(MARGIN)를 뺀 안쪽만 탐색
  1. 출발 전 점검: 모든 줄 시작/끝/중간 점을 ikin 으로 계산 → 도달 불가·특이점(J3/J5) 이면 움직이지 않음
  2. 바닥 측정(선택): 왼쪽 아래 앞에서 순응제어로 천천히 내려가 바닥 Z → 맨 아래 줄을 바닥 + 여유 위로
  3. 선 단위 지그재그 스캔: 가로 줄(높이 ROW_STEP 간격) + 세로 열(좌우 COL 간격)
       가로 줄은 세로 틈을, 세로 열은 줄 사이로 빠지는 얇은 가로 틈을 가로질러 찾는다
       - 줄 시작점 앞(PRE_GAP)까지 접근 → 순응제어(벽 쪽 X 만 부드럽게) 켜기
       - 목표를 벽 '안쪽' PRESS_DEPTH 에 두고 이동 → 스프링 힘(≈ 강성 × 깊이)으로 벽을 누르며 미끄러짐
         (힘제어처럼 허공으로 계속 밀고 나가지 않아, 큰 틈에서도 최대 PRESS_DEPTH 만큼만 들어감)
       - 이동 중 50 Hz 로 탐침 끝 TCP 위치 + 힘을 기록 (1 mm 이상 움직였을 때마다)
       - 줄 끝에서 순응제어 해제 → 벽에서 후퇴
  4. 분석: 접촉 점 전체로 벽 평면을 (이상치 제외하며) 맞추고,
       평면보다 GAP_DEPTH_MM 이상 '깊게' 들어간 곳(또는 접촉이 끊긴 곳) = 틈/홈,
       평면보다 BUMP_HEIGHT_MM 이상 튀어나온 곳 = 돌출
       → 줄마다 연속 구간을 선분으로, 이웃 줄과 겹치는 선분은 하나의 영역으로 묶음
  5. 저장: ~/wall_scan_data/interior_*.json  (wall_modeling 형식 'points' 포함 → 모델링/RViz 재사용 가능)
           + RESULT_FILE (예전 gap_results.json 형식: gap_points / gap_segments)

[실행]
  ros2 run rokey1 wall_scanner --ros-args -p mode:=virtual      # 에뮬레이터 (가상 벽 + 가상 홈)
  ros2 run rokey1 wall_scanner --ros-args -p mode:=real         # 실제 로봇 (y 입력 확인)
  옵션: -p dirs:=both|rows|cols -p row_step:=20.0 -p col_step:=20.0 -p margin:=15.0
        -p floor_detect:=true -p tcp_name:=GripperDA_v1 -p probe_length:=0.0 -p tool_roll:=90 -p confirm:=true
        -p scan_force:=3.0 -p top_force_ratio:=1.3   (맨 위 줄만 3.0 × 1.3 = 3.9 N, 사이 줄은 높이에 비례)
  시작 자세: 빈 그리퍼(닫힘) 끝이 탐침, '홈'에서 시작 (real 은 홈이 아니면 움직이지 않음)
  HMI 표시: /dsr01/wall_scan/points (JSON) → HMI 로봇 화면에 탐지 점/테두리/틈 표시
  결과 다시 보기: ros2 run rokey1 wall_modeling   (가장 최근 scan/interior 파일이 아니면 경로 지정)

[힘/안전]
  - 벽을 누르는 힘 ≈ APPROACH_STIFFNESS × PRESS_DEPTH (기본 500 N/m × 10 mm = 5 N)
  - 어떤 방향이든 HARD_FORCE 초과, 옆 방향 SIDE_FORCE 초과 → 즉시 정지 후 전체 중단
  - 이전 wall_scanner 는 wall_scanner_prev.py 로 보관
"""
import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime
from itertools import groupby

import numpy as np
import rclpy
import DR_init

from utils.config import CFG
from utils.shutdown import run_cleanups, safe_shutdown

try:
    from navigation.wall_scan_io import RobotIO, robust_fit_plane
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from wall_scan_io import RobotIO, robust_fit_plane

# =============================================================================
# 로봇 / 공구
# =============================================================================
ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
TOOL_NAME = 'Tool Weight'          # 펜던트 등록 Tool
TCP_NAME = 'GripperDA_v1'          # 탐침 끝 TCP. 탐침봉 TCP 를 따로 등록했으면 그 이름으로 (-p tcp_name:=...)
# 탐침 자세 ZYZ. 기존 측정값 [13.68, 132.34, 91.18] 은 탐침 32 mm 에서 왼쪽 아래 후퇴점 J5=9.5°(손목 특이점 근처).
# ikin 으로 비교해 B 만 3.7° 더 기울임 → 전 경로 J3 ≥ 50°, J5 ≥ 16.5° (경로는 탐침 끝 기준이라 위치는 그대로)
TOOL_ORI = [13.68, 136.0, 91.18]
TOOL_ROLL_DEG = 90.0     # 탐색 때 공구를 자기 Z축 기준으로 더 돌리는 각도 (ZYZ 의 C 에 더함, 탐침 끝 위치는 그대로).
                         # 출발 전 점검에서 이 각도가 안 되면 반대(−) 각도를 자동으로 시도
READY_J = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
SAFE_WAYPOINT = [733.62, 203.20, 274.61]   # 벽 앞 상공 경유점 (기존 wall_scanner 의 TARGET_TCP)
PROBE_LENGTH = 0.0       # 탐침봉이 TCP 끝에서 공구 Z 방향으로 더 나온 길이 [mm]. 탐침 TCP 를 따로 등록했으면 0
HOME_TOL_DEG = 5.0       # 시작 시 홈 자세 허용 오차 (빈 그리퍼로 홈에서 시작)
SCAN_TOPIC = 'wall_scan/points'   # HMI 표시용 (std_msgs/String, JSON)
PUBLISH_PERIOD = 0.5

# =============================================================================
# 벽 테두리 (벽면 크기 탐색 결과, BASE mm) — 탐침 끝이 벽 표면에 닿은 위치
# =============================================================================
CORNERS = {
    'bottom_left': [747.14, 221.34, 32.93],
    'top_left': [747.14, 221.34, 204.32],
    'top_right': [747.14, -254.90, 204.32],
    'bottom_right': [747.14, -254.90, 32.93],
}

# =============================================================================
# 탐색 영역 / 경로
# =============================================================================
ROW_STEP = 20.0          # 줄 간격 (높이 방향) [mm]
MARGIN = 15.0            # 테두리 안쪽 여유 (좌/우/위) [mm] — 탐침이 모서리에 걸리지 않게
BOTTOM_MARGIN = 15.0     # 아래 테두리에서 띄울 여유 [mm]
FLOOR_CLEARANCE = 10.0   # 바닥에서 최소 이만큼 위에서만 스캔 [mm]
APPROACH_GAP = 30.0      # 줄 사이 이동 / 후퇴 때 벽에서 띄우는 거리 [mm]
PRE_GAP = 5.0            # 누르기 시작 전 벽 앞 대기 거리 [mm]
START_BACKOFF = float((CFG.get('inner_explore') or {}).get('start_backoff', 50.0))   # robot.yaml inner_explore.start_backoff
                         # 줄 시작을 받은 벽 좌표보다 이만큼 더 몸 쪽에서 (벽 좌표가 실제보다 깊을 때 벽 안에서 시작하지 않게) [mm]
                         #   (wall_id 15: 받은 왼쪽 x 799 / 실제 ≈751 → 48mm 깊음. 60 은 낮은 줄 물림점이 J5 특이점, run164 → 50)
                         #   붙기(_attach)는 그만큼 더 멀리서 힘/순응으로 다가감 (제한 시간은 거리에 맞춰 자동으로 늘어남). 0 = 이전과 같음
PRESS_DEPTH = 10.0       # 목표점을 벽 안쪽으로 이만큼 → 스프링으로 누름 [mm]

# =============================================================================
# 속도 / 강성 / 힘
# =============================================================================
MOVE_VEL, MOVE_ACC = 60.0, 100.0
PROBE_VEL, PROBE_ACC = 5.0, 20.0
SCAN_VEL, SCAN_ACC = 15.0, 30.0
APPROACH_STIFFNESS = 500.0    # 벽 쪽(X) 강성 [N/m] → 10 mm 누르면 약 5 N
FLOOR_STIFFNESS = 1000.0      # 바닥 측정 때 Z 강성 (단단한 탐침이 바닥에 부딪혀 충돌감지 나지 않게)
OTHER_STIFFNESS = 3000.0
ROT_STIFFNESS = 200.0
CONTACT_FORCE = 2.0           # (spring) 이 이상이면 벽에 닿아 있음 [N]
# ---- 힘제어 스캔 (scan_method=force, 기본) ----
SCAN_METHOD = 'force'         # force: 벽에 닿은 뒤 set_desired_force 로 누르며 이동 / spring: 순응제어 + 벽 안쪽 목표
SCAN_FORCE = 2.0              # 접촉 뒤 추가로 미는 힘 (상대 모드) [N]. 접촉 힘과 합쳐 약 3 N
SCAN_FORCE_MAX = 5.0          # 너무 세게 누르지 않도록 상한
# 위로 갈수록 실제로 누르는 힘이 약해져 그리퍼가 틈에 덜 들어감(2026-11-20 실측: 같은 세로 틈이 줄 0 에서 X 1.4 mm,
# 줄 8 에서 0.1 mm). 그래서 맨 위 줄의 목표 힘 = scan_force × 이 비율, 그 사이 줄은 높이에 비례. 1.0 = 보정 없음
TOP_FORCE_RATIO = 1.0
TOP_FORCE_RATIO_DEFAULT = 1.0  # 입력이 비었거나 0 이하일 때 쓰는 값 (보정 없음)
FORCE_CONTACT = 1.0           # (force) 벽에 닿았다고 보는 힘 [N] — 닿으면 바로 멈추고 힘제어 시작
FORCE_STIFFNESS_X = 200.0     # 힘제어 축(X) 강성 [N/m] (샌딩·퍼티와 같은 값)
WALL_SEARCH_MM = 30.0         # 줄 시작에서 (주어진) 벽 위치보다 최대 이만큼 더 들어가며 실제 벽을 찾음
MAX_DIVE_MM = 8.0             # 힘제어 중 시작 접촉면보다 이만큼 더 들어가면 구멍/틈 → 즉시 정지 후 틈 추정
FLOOR_FORCE = 5.0             # 바닥 감지 [N]
HARD_FORCE = 30.0             # 어느 방향이든 이 이상이면 사고로 보고 중단 [N]
SIDE_FORCE = 15.0             # 벽 쪽이 아닌 방향(Y/Z) 힘 상한 [N] → 스캔 중이면 '테두리에 걸림'으로 보고 안쪽으로 옮겨 재탐색
INWARD_STEP_MM = 10.0         # 테두리에 걸릴 때마다 그 테두리 쪽 여유를 이만큼 늘림 (이후 모든 선에 적용)
MAX_LINE_RETRY = 4            # 한 선을 안쪽으로 옮겨 다시 시도하는 최대 횟수 → 넘으면 그 선만 건너뜀
SAME_HIT_MM = 15.0            # 옮긴 뒤에도 거의 같은 곳에서 걸리면 '다른 테두리' 탓으로 봄
OBSTACLE_HOP_MM = 30.0        # 첫 줄 이후 걸린 곳(틈/단차 추정)을 이만큼 건너뛰고 그 줄 나머지를 계속 탐색
MAX_OBSTACLE_HOPS = 6         # 한 줄에서 건너뛰기 최대 횟수 → 넘으면 그 줄 나머지는 건너뜀
CONTACT_WAIT = 5.0            # 줄 시작에서 벽에 붙을 때까지 기다리는 최대 시간 [s]
# ---- 접촉 판정 (힘 + 위치) ----
# 2026-11-20 실측: −Y 끝·위쪽 자세에서는 순응제어를 켠 뒤 허공에서도 1~2 N 이 읽혀, 힘만 보면 벽 10 mm 앞에서
# '접촉'으로 멈추고 허공에서 힘제어를 시작했다. 그래서 힘이 '유지'되고 X 가 '멈춰야'(벽에 막힘) 접촉으로 본다.
CONTACT_HOLD_SEC = 0.2        # 힘이 기준 이상으로 이만큼 계속돼야 함 (순간 튐 무시)
STALL_WINDOW_SEC = 0.2        # 이 시간 동안의 X 전진량으로 멈춤 판단
STALL_MM = 0.2                # 그동안 X 전진이 이보다 작으면 멈춤 (허공이면 5 mm/s 로 약 1 mm 전진)
ATTACH_SETTLE_SEC = 0.5       # 출발 직후(가속, 순응제어 전환 '덜컥') 동안은 접촉으로 보지 않음
CONTACT_RELEASE_RATIO = 0.5   # 기록 중 '끊김' 기준 = 접촉 기준 × 이 값 (히스테리시스: force 1 N 접촉 ↔ 0.5 N 끊김)
FREE_ADVANCE_MM = 3.0         # (force) 접촉이 끊긴 채 X 가 이만큼 더 들어가면 허공을 밀고 들어감 → 정지
LOST_MIN_MM = 3.0             # 접촉 끊김이 이보다 짧게 이어지면 잡음으로 보고 틈에서 제외
SAMPLE_PERIOD = 0.02
SAMPLE_MIN_MM = 1.0           # 이만큼 움직일 때마다 한 점 기록

# 특이점 / 도달 점검 (퍼티 실측: J3≈39° 에서 3205 알람, 44° 통과)
J3_MIN_DEG = 41.0
J5_MIN_DEG = 10.0
PREFLIGHT_POINTS_PER_ROW = 5

# =============================================================================
# 분석
# =============================================================================
GAP_DEPTH_MM = 1.5            # 평면보다 이만큼 깊으면 틈/홈 후보
BUMP_HEIGHT_MM = 1.5          # 평면보다 이만큼 튀어나오면 돌출 후보
SEGMENT_MAX_GAP = 5.0         # 같은 줄에서 이보다 가까운 후보 점은 한 선분 [mm]
SEGMENT_MIN_LEN = 2.0         # 이보다 짧은 선분은 잡음으로 버림 [mm]
REGION_OVERLAP_TOL = 5.0      # 이웃 줄 선분이 이만큼 겹치거나 가까우면 같은 영역 [mm]
# ---- 세로 틈 선 (결과 gaps / DB gap 테이블) ----
GAP_Y_TOL_MM = 8.0            # 줄마다 찾은 틈 조각의 가운데 Y 가 이만큼 안이면 같은 세로 틈 (실측 흔들림 ±1.5 mm)
# 사이에 놓친 줄이 이 수 이하일 때만 바로 연결. 0 = 바로 위아래 줄만.
# 1 로 하면 줄 시작 구간의 가짜 끊김(같은 쪽 출발 = 한 줄 건너 한 줄, 같은 Y)이 이어져 가짜 세로선이 됐다(2 N 실측).
# 가운데를 놓쳐 쪼개진 틈은 '같은 Y 묶음끼리 합치기'로 다시 하나가 된다.
MAX_MISSED_ROWS = 0
MIN_GAP_ROWS = 2              # 서로 다른 줄 이 수 이상에서 나와야 세로 틈 (한 줄짜리 끊김은 잡음으로 버림)
WALL_REF_NEAR_MM = 10.0       # 틈 옆 벽면 X 를 잴 구간: 틈 끝에서 이만큼 떨어진 곳부터 (모서리 영향 제외)
WALL_REF_FAR_MM = 30.0        # … 이만큼 떨어진 곳까지

SAVE_DIR = os.path.expanduser('~/wall_scan_data')
RESULT_FILE = os.path.expanduser('~/ws_cobot_pjt/ws_dsr/gap_results.json')   # 예전 형식 호환

APP_DIR = np.array([1.0, 0.0, 0.0])   # 벽을 향하는 방향 (BASE +X)


class Canceled(Exception):
    """액션 취소/정지 요청."""


class DiveHit(Exception):
    """(force) 힘제어 중 탐침이 접촉면보다 MAX_DIVE_MM 넘게 들어감 = 구멍/틈. 틈 추정으로 저장하고 건너뛴다."""

    def __init__(self, message, pos):
        super().__init__(message)
        self.pos = [float(v) for v in pos[:3]]
        self.force = [0.0, 0.0, 0.0]


class BorderHit(RuntimeError):
    """스캔 중 비정상 힘 (테두리/턱에 걸림). 안쪽으로 옮겨 재탐색한다."""

    def __init__(self, message, pos, force):
        super().__init__(message)
        self.pos = [float(v) for v in pos[:3]]
        self.force = [float(v) for v in force[:3]]


def rolled(ori, roll_deg):
    """공구 Z축 기준 회전: ZYZ [a, b, c] → [a, b, c + roll] (c 는 -180~180 으로 정리)."""
    c = (ori[2]+roll_deg+180.0) % 360.0-180.0
    return [ori[0], ori[1], c]


def tool_z_axis(ori):
    """ZYZ [a, b, c] (deg) → 공구 Z축 (BASE). 탐침 끝 = TCP + 길이 × 이 방향."""
    a, b = math.radians(ori[0]), math.radians(ori[1])
    return np.array([math.cos(a)*math.sin(b), math.sin(a)*math.sin(b), math.cos(b)])

# (virtual) 가상 홈: 세로 홈 하나 + 가로 홈 하나 (탐색 로직 검증용)
VIRTUAL_DEFECTS = [
    {'kind': 'groove', 'y': (-5.0, 5.0), 'z': (60.0, 180.0), 'depth': 4.0},
    {'kind': 'groove', 'y': (-150.0, -60.0), 'z': (118.0, 124.0), 'depth': 3.0},
    {'kind': 'bump', 'y': (100.0, 140.0), 'z': (90.0, 110.0), 'depth': -3.0},
]
VIRTUAL_STIFFNESS = 20.0      # [N/mm]
VIRTUAL_FLOOR_Z = 30.0


# =============================================================================
# 기하 (순수 함수: 테스트 가능)
# =============================================================================
def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(3)]


def wall_point(corners, u, v):
    """u: 0=왼쪽 → 1=오른쪽, v: 0=아래 → 1=위. 네 모서리 쌍선형 보간 (벽이 기울어도 됨)."""
    bottom = lerp(corners['bottom_left'], corners['bottom_right'], u)
    top = lerp(corners['top_left'], corners['top_right'], u)
    return lerp(bottom, top, v)


def _v_bounds(corners, margin, bottom_margin, floor_z, floor_clearance):
    """탐색할 v(높이 비율) 범위. 아래는 테두리 여유와 (좌/우 모두) 바닥 위 여유, 위는 테두리 여유."""
    def height_at(u):
        return math.dist(wall_point(corners, u, 0.0), wall_point(corners, u, 1.0))

    h = min(height_at(0.0), height_at(1.0))
    v_lo = bottom_margin/h
    if floor_z is not None:
        for side in ('left', 'right'):
            bottom, top = corners[f'bottom_{side}'], corners[f'top_{side}']
            v_lo = max(v_lo, (floor_z+floor_clearance-bottom[2])/(top[2]-bottom[2]))
    v_hi = 1.0-margin/h
    if v_hi <= v_lo:
        raise ValueError('여유를 빼면 탐색할 높이가 없습니다. margin/bottom_margin/바닥 높이를 확인하세요.')
    return v_lo, v_hi, h


def _steps(lo, hi, span_mm, step):
    """lo~hi 를 간격이 step 이하가 되도록 나눈 값들 (올림: 내림이면 간격이 step 보다 넓어짐)."""
    count = max(2, int(math.ceil(span_mm/step-1e-9))+1)
    return [lo+(hi-lo)*i/(count-1) for i in range(count)]


def make_rows(corners, row_step=ROW_STEP, margin=MARGIN, bottom_margin=BOTTOM_MARGIN,
              floor_z=None, floor_clearance=FLOOR_CLEARANCE):
    """가로 줄(h): 높이 row_step 간격, 각 줄은 좌우 테두리에서 margin 안쪽. 지그재그(왼→오, 오→왼).

    가로 줄은 '세로로 난' 틈/홈을 가로질러 잘 찾는다.
    Returns: [{'dir': 'h', 'index', 'v', 'start': [x,y,z], 'end': [x,y,z]}, ...]
    """
    v_lo, v_hi, h = _v_bounds(corners, margin, bottom_margin, floor_z, floor_clearance)
    rows = []
    for index, v in enumerate(_steps(v_lo, v_hi, (v_hi-v_lo)*h, row_step)):
        width = math.dist(wall_point(corners, 0.0, v), wall_point(corners, 1.0, v))
        u_lo, u_hi = margin/width, 1.0-margin/width
        if u_hi <= u_lo:
            continue
        a, b = wall_point(corners, u_lo, v), wall_point(corners, u_hi, v)
        if index % 2:
            a, b = b, a
        rows.append({'dir': 'h', 'index': index, 'v': v, 'start': a, 'end': b})
    return rows


def make_cols(corners, col_step=ROW_STEP, margin=MARGIN, bottom_margin=BOTTOM_MARGIN,
              floor_z=None, floor_clearance=FLOOR_CLEARANCE):
    """세로 열(v): 좌우 col_step 간격, 각 열은 아래(바닥 여유)~위 테두리 여유. 지그재그(아래→위, 위→아래).

    세로 열은 '가로로 난' 얇은 틈을 가로질러 잘 찾는다 (가로 줄 사이로 빠지는 틈).
    """
    v_lo, v_hi, _ = _v_bounds(corners, margin, bottom_margin, floor_z, floor_clearance)
    width = min(math.dist(corners['bottom_left'], corners['bottom_right']),
                math.dist(corners['top_left'], corners['top_right']))
    u_lo, u_hi = margin/width, 1.0-margin/width
    cols = []
    for index, u in enumerate(_steps(u_lo, u_hi, (u_hi-u_lo)*width, col_step)):
        a, b = wall_point(corners, u, v_lo), wall_point(corners, u, v_hi)
        if index % 2:
            a, b = b, a
        cols.append({'dir': 'v', 'index': index, 'u': u, 'start': a, 'end': b})
    return cols


def make_lines(corners, row_step=ROW_STEP, col_step=ROW_STEP, dirs='both', margin=MARGIN,
               bottom_margin=BOTTOM_MARGIN, floor_z=None, floor_clearance=FLOOR_CLEARANCE):
    """스캔할 선 전체. dirs: 'rows' | 'cols' | 'both'. 각 선에 고유 번호 'line' 부여."""
    lines = []
    if dirs in ('rows', 'both'):
        lines += make_rows(corners, row_step, margin, bottom_margin, floor_z, floor_clearance)
    if dirs in ('cols', 'both'):
        lines += make_cols(corners, col_step, margin, bottom_margin, floor_z, floor_clearance)
    if not lines:
        raise ValueError("dirs 는 'rows', 'cols', 'both' 중 하나여야 합니다.")
    for number, line in enumerate(lines):
        line['line'] = number
    return lines


# ----------------------------------------------------------------------------- 테두리 학습
BORDERS = ('left', 'right', 'bottom', 'top')
INWARD = {'left': np.array([0.0, -1.0, 0.0]), 'right': np.array([0.0, 1.0, 0.0]),
          'bottom': np.array([0.0, 0.0, 1.0]), 'top': np.array([0.0, 0.0, -1.0])}   # 그 테두리에서 안쪽 방향
ALONG = {'h': ('left', 'right'), 'v': ('bottom', 'top')}   # 선의 진행 방향 끝에 있는 테두리


def nominal_uv(corners, p):
    """점 p(BASE) 의 테두리 기준 (u: 0 왼쪽 → 1 오른쪽, v: 0 아래 → 1 위) 근사."""
    u, v = 0.5, 0.5
    for _ in range(6):
        left, right = wall_point(corners, 0.0, v), wall_point(corners, 1.0, v)
        u = (left[1]-p[1])/(left[1]-right[1])
        bottom, top = wall_point(corners, u, 0.0), wall_point(corners, u, 1.0)
        v = (p[2]-bottom[2])/(top[2]-bottom[2])
    return u, v


def border_distances(corners, p, floor_z=None):
    """점 p 에서 (주어진) 왼/오른/아래/위 테두리까지 거리 [mm]. 아래는 바닥이 더 높으면 바닥 기준."""
    u, v = nominal_uv(corners, p)
    left, right = wall_point(corners, 0.0, v), wall_point(corners, 1.0, v)
    bottom, top = wall_point(corners, u, 0.0), wall_point(corners, u, 1.0)
    bottom_z = bottom[2] if floor_z is None else max(bottom[2], floor_z)
    return {'left': left[1]-p[1], 'right': p[1]-right[1], 'bottom': p[2]-bottom_z, 'top': top[2]-p[2]}


def base_requirements(lines, corners, floor_z=None):
    """처음 만든 경로가 지키는 테두리별 최소 거리 (학습 여유는 여기에 더해진다)."""
    req = {k: math.inf for k in BORDERS}
    for line in lines:
        for p in (line['start'], line['end']):
            d = border_distances(corners, p, floor_z)
            for k in BORDERS:
                req[k] = min(req[k], d[k])
    return req


def adjust_line(line, corners, required, floor_z=None, min_len=10.0):
    """테두리별 최소 거리(required)를 지키도록 선을 안쪽으로 줄이거나(진행 방향 끝) 옮긴다(옆 테두리).
    지킬 수 없으면(선이 다 잘림) None."""
    a, b = np.array(line['start'], float), np.array(line['end'], float)
    along = ALONG[line['dir']]
    for _ in range(3):                  # 옮기면 다른 테두리 거리도 바뀌므로 몇 번 반복
        for k in BORDERS:
            da = border_distances(corners, a, floor_z)[k]
            db = border_distances(corners, b, floor_z)[k]
            deficit_a, deficit_b = required[k]-da, required[k]-db
            if k in along:              # 선 끝을 안쪽으로 당김 (그 테두리에 가까운 끝)
                unit = (b-a)/max(np.linalg.norm(b-a), 1e-9)
                if deficit_a > 1e-6:
                    a = a+unit*deficit_a/max(abs(float(np.dot(unit, INWARD[k]))), 0.2)
                if deficit_b > 1e-6:
                    b = b-unit*deficit_b/max(abs(float(np.dot(unit, INWARD[k]))), 0.2)
            else:                       # 선 전체를 그 테두리 안쪽으로 평행 이동
                shift = max(deficit_a, deficit_b, 0.0)
                if shift > 1e-6:
                    a, b = a+INWARD[k]*shift, b+INWARD[k]*shift
    if np.linalg.norm(b-a) < min_len or np.dot(b-a, np.subtract(line['end'], line['start'])) <= 0:
        return None
    if any(border_distances(corners, p, floor_z)[k] < required[k]-0.5 for p in (a, b) for k in BORDERS):
        return None                     # 양쪽 테두리 사이가 너무 좁아짐
    out = dict(line)
    out['start'], out['end'] = a.tolist(), b.tolist()
    return out


def line_name(line):
    return f'{"가로 줄" if line["dir"] == "h" else "세로 열"} {line["index"]}'


def plane_x(plane, y, z):
    """평면 위 (y, z) 의 X."""
    a, b, c = plane['normal']
    return -(b*y+c*z+plane['d'])/a


def extract_segments(samples, flag, axis, max_gap=SEGMENT_MAX_GAP, min_len=SEGMENT_MIN_LEN):
    """한 선 안에서 flag 가 True 인 연속 점들을 선분으로. axis: 선의 진행 좌표 (1=Y 가로, 2=Z 세로)."""
    segments, current = [], []

    def close():
        if len(current) >= 2:
            values = [p['xyz'][axis] for p in current]
            if max(values)-min(values) >= min_len:
                segments.append(current[:])

    for sample in samples:
        if current and abs(sample['xyz'][axis]-current[-1]['xyz'][axis]) > max_gap:
            close()
            current = []
        if flag(sample):
            current.append(sample)
        elif current:
            close()
            current = []
    close()
    return segments


def short_contact_losses(line_samples, axis, min_mm=LOST_MIN_MM):
    """한 선에서 접촉 끊김이 min_mm 보다 짧게 이어진 점들의 id 집합.

    힘이 접촉 기준 근처를 오르내리면 1~2 mm 짜리 끊김이 자주 생기는데(실측), 이건 틈이 아니라 잡음이라 뺀다.
    """
    short = set()
    for lost, group in groupby(line_samples, key=lambda s: not s['contact']):
        run = list(group)
        if lost and abs(run[-1]['xyz'][axis]-run[0]['xyz'][axis]) < min_mm:
            short.update(id(s) for s in run)
    return short


def mark_samples(line_samples, axis, gap_depth, bump_height):
    """한 선의 점마다 'mark' = 'gap' | 'bump' | None 을 붙인다 (depth 계산 뒤에 호출).
    틈 = LOST_MIN_MM 이상 이어진 접촉 끊김 또는 평면보다 gap_depth 이상 깊음, 돌출 = bump_height 이상 튀어나옴."""
    noise = short_contact_losses(line_samples, axis)
    for s in line_samples:
        if not s['contact']:
            s['mark'] = None if id(s) in noise else 'gap'
        elif s['depth'] >= gap_depth:
            s['mark'] = 'gap'
        elif s['depth'] <= -bump_height:
            s['mark'] = 'bump'
        else:
            s['mark'] = None


def local_wall_x(line_samples, axis, lo, hi, near=WALL_REF_NEAR_MM, far=WALL_REF_FAR_MM):
    """틈 조각(진행 좌표 lo~hi) 바로 옆 벽면의 X = 양옆 near~far mm 구간에서 벽에 붙어 있던 점 X 의 중앙값.

    틈 안의 점은 그리퍼가 0.5~1.4 mm 들어간 X 라 쓰면 선이 벽 속으로 그어진다 → 틈/돌출이 아닌 접촉 점만 쓴다.
    전체 벽 평면보다 그 자리의 실제 벽면에 가깝다. 쓸 점이 없으면 None (호출하는 쪽이 평면 값으로 대신).
    """
    xs = [s['xyz'][0] for s in line_samples
          if s['contact'] and s.get('mark') is None
          and (lo-far <= s['xyz'][axis] <= lo-near or hi+near <= s['xyz'][axis] <= hi+far)]
    return round(float(np.median(xs)), 3) if xs else None


def summarize_segment(points, line, kind, sid):
    axis = 1 if line['dir'] == 'h' else 2
    along = [p['xyz'][axis] for p in points]
    across = float(np.mean([p['xyz'][3-axis] for p in points]))   # 가로 줄이면 z, 세로 열이면 y
    depths = [p['depth'] for p in points if p['depth'] is not None]
    rng = [round(min(along), 2), round(max(along), 2)]
    first = min(points, key=lambda p: p['xyz'][axis])
    last = max(points, key=lambda p: p['xyz'][axis])
    return {
        'id': sid, 'kind': kind, 'line': line['line'], 'dir': line['dir'], 'index': line['index'],
        'start': [round(v, 2) for v in first['xyz']], 'end': [round(v, 2) for v in last['xyz']],
        'y_range': rng if axis == 1 else [round(across, 2)]*2,
        'z_range': rng if axis == 2 else [round(across, 2)]*2,
        'length': round(max(along)-min(along), 2),
        'max_depth': round(max(depths, key=abs), 3) if depths else None,
        'contact_lost': sum(1 for p in points if not p['contact']),
        'points': len(points),
    }


def _overlap(a, b, tol):
    return a[0] <= b[1]+tol and b[0] <= a[1]+tol


def connected_groups(items, linked):
    """linked(a, b) 가 True 인 것끼리 (사슬로 이어진 것까지) 한 묶음. 입력 순서를 지킨 묶음 목록."""
    parent = list(range(len(items)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    for i, a in enumerate(items):
        for j in range(i+1, len(items)):
            if linked(a, items[j]):
                parent[find(i)] = find(j)
    groups = {}
    for i, item in enumerate(items):
        groups.setdefault(find(i), []).append(item)
    return list(groups.values())


def group_regions(segments, tol=REGION_OVERLAP_TOL):
    """같은 종류 선분을 영역으로 묶는다 (기록·HMI 표시용. 결과 gaps 는 vertical_gap_lines).
      - 같은 방향 이웃 선(번호 차 1)에서 진행 방향 구간이 겹치면 연결
      - 가로 선분과 세로 선분이 서로 교차하면 연결
    """
    def linked(a, b):
        if a['kind'] != b['kind']:
            return False
        if a['dir'] == b['dir']:
            key = 'y_range' if a['dir'] == 'h' else 'z_range'
            return abs(a['index']-b['index']) == 1 and _overlap(a[key], b[key], tol)
        return _overlap(a['y_range'], b['y_range'], tol) and _overlap(a['z_range'], b['z_range'], tol)

    regions = []
    for index, members in enumerate(connected_groups(segments, linked), 1):
        ys = [y for m in members for y in m['y_range']]
        zs = [z for m in members for z in m['z_range']]
        depths = [m['max_depth'] for m in members if m['max_depth'] is not None]
        width, height = max(ys)-min(ys), max(zs)-min(zs)
        regions.append({
            'id': index, 'kind': members[0]['kind'], 'segments': [m['id'] for m in members],
            'rows': sorted({m['index'] for m in members if m['dir'] == 'h'}),
            'cols': sorted({m['index'] for m in members if m['dir'] == 'v'}),
            'y_range': [round(min(ys), 2), round(max(ys), 2)],
            'z_range': [round(min(zs), 2), round(max(zs), 2)],
            'width': round(width, 2), 'height': round(height, 2),
            'max_depth': round(max(depths, key=abs), 3) if depths else None,
            'orientation': 'horizontal' if width >= height else 'vertical',
        })
    return regions


def analyze(samples, lines, gap_depth=GAP_DEPTH_MM, bump_height=BUMP_HEIGHT_MM):
    """기록한 점 전체 → 벽 평면, 각 점 깊이, 틈/돌출 선분과 영역.

    samples: [{'line', 'xyz': [x,y,z], 'force': N, 'contact': bool}, ...] (선 안에서 이동 순서)
    깊이 depth = 측정 X - 평면 X  (+ = 평면보다 벽 안쪽으로 들어감 = 홈/틈, - = 튀어나옴)
    """
    contact = [s for s in samples if s['contact']]
    if len(contact) < 4:
        raise ValueError(f'벽에 닿은 점이 {len(contact)}개뿐이라 평면을 만들 수 없습니다.')
    plane, _ = robust_fit_plane([s['xyz'] for s in contact])
    for s in samples:
        s['depth'] = float(s['xyz'][0]-plane_x(plane, s['xyz'][1], s['xyz'][2])) if s['contact'] else None

    by_line = {line['line']: line for line in lines}
    segments = []
    for number in sorted({s['line'] for s in samples}):
        line = by_line[number]
        line_samples = [s for s in samples if s['line'] == number]
        axis = 1 if line['dir'] == 'h' else 2
        mark_samples(line_samples, axis, gap_depth, bump_height)
        for kind in ('gap', 'bump'):
            for points in extract_segments(line_samples, lambda s, k=kind: s['mark'] == k, axis):
                segment = summarize_segment(points, line, kind, len(segments)+1)
                along = [p['xyz'][axis] for p in points]
                segment['wall_x'] = local_wall_x(line_samples, axis, min(along), max(along))
                segments.append(segment)
    depths = np.array([s['depth'] for s in contact])
    return {
        'plane': {'normal': [float(v) for v in plane['normal']], 'd': float(plane['d']),
                  'rms_mm': float(np.sqrt(np.mean(depths**2)))},
        'segments': segments,
        'regions': group_regions(segments),
        'stats': {'samples': len(samples), 'contact': len(contact),
                  'contact_lost': len(samples)-len(contact),
                  'depth_min': float(depths.min()), 'depth_max': float(depths.max())},
    }


# =============================================================================
# 높이별 누르는 힘 (순수 로직: 테스트 가능)
# =============================================================================
def line_height(line):
    """선의 높이 = 시작/끝 Z 평균 (가로 줄은 Z 가 일정, 세로 열은 가운데 높이)."""
    return (line['start'][2]+line['end'][2])/2.0


def top_force_ratio_or_default(value):
    """HMI 에서 '맨 위 줄 힘 비율' 칸을 비우면 0 이 넘어온다(Number('') = 0) → 비었거나 0 이하면 1.0(보정 없음).
    비율 0 은 '힘 0' 이라 의미가 없으므로 실수로 비운 것으로 본다."""
    try:
        ratio = float(value)
    except (TypeError, ValueError):
        return TOP_FORCE_RATIO_DEFAULT
    return ratio if ratio > 0.0 else TOP_FORCE_RATIO_DEFAULT


def scaled_scan_force(base_force, top_ratio, z, z_bottom, z_top):
    """높이 비율로 목표 힘 보정: 맨 아래 줄 = base_force, 맨 위 줄 = base_force × top_ratio, 사이는 직선.
    줄이 하나뿐이거나(z_top == z_bottom) 범위 밖이면 가까운 끝 값을 쓴다."""
    if z_top-z_bottom < 1e-6:
        return base_force
    t = min(max((z-z_bottom)/(z_top-z_bottom), 0.0), 1.0)
    return base_force*(1.0+(top_ratio-1.0)*t)


# =============================================================================
# 접촉 판정 (순수 로직: 테스트 가능)
# =============================================================================
def contact_state(force, was_contact, on_force, off_force):
    """히스테리시스 접촉 상태: on_force 이상이면 접촉, off_force 미만이면 끊김, 그 사이면 이전 상태 유지.
    기준 하나(1 N)만 쓰면 1 N 근처에서 접촉/끊김이 계속 뒤바뀌어 가짜 틈이 생기기 때문."""
    if force >= on_force:
        return True
    if force < off_force:
        return False
    return was_contact


class ContactDetector:
    """벽 찾기(_attach) 접촉 판정 = 힘 유지 + X 멈춤.

    ① 힘이 기준 이상으로 hold_sec 동안 계속되고
    ② 벽 쪽으로 전진 명령 중인데 X 가 window_sec 동안 stall_mm 미만으로만 움직였을 때(= 벽에 막힘)만 접촉.
    출발 직후 settle_sec 동안은 가속·순응제어 전환 때문에 판단하지 않는다.
    멈춘 '움직임'만 보고 벽 좌표와는 비교하지 않으므로, 모서리 좌표 오차가 있어도 결과가 같다.
    """

    def __init__(self, force_threshold, hold_sec=CONTACT_HOLD_SEC, window_sec=STALL_WINDOW_SEC,
                 stall_mm=STALL_MM, settle_sec=ATTACH_SETTLE_SEC):
        self.force_threshold = force_threshold
        self.hold_sec, self.window_sec = hold_sec, window_sec
        self.stall_mm, self.settle_sec = stall_mm, settle_sec
        self.start_time = None
        self.force_since = None
        self.recent_x = deque()          # (시각, X): 최근 window_sec 구간

    def update(self, now, x, force):
        """샘플 하나(시각 s, 벽 방향 X mm, 벽 방향 힘 크기 N)를 넣고, 지금 접촉으로 볼 수 있으면 True."""
        if self.start_time is None:
            self.start_time = now
        self._track_force(now, force)
        self._track_x(now, x)
        settled = now-self.start_time >= self.settle_sec
        return settled and self._force_held(now) and self._stalled(now, x)

    def _track_force(self, now, force):
        if force < self.force_threshold:
            self.force_since = None
        elif self.force_since is None:
            self.force_since = now

    def _force_held(self, now):
        return self.force_since is not None and now-self.force_since >= self.hold_sec

    def _track_x(self, now, x):
        self.recent_x.append((now, x))
        while len(self.recent_x) > 1 and now-self.recent_x[1][0] >= self.window_sec:
            self.recent_x.popleft()

    def _stalled(self, now, x):
        oldest_time, oldest_x = self.recent_x[0]
        return now-oldest_time >= self.window_sec and abs(x-oldest_x) < self.stall_mm


# =============================================================================
# 로봇 입출력
# =============================================================================
class VirtualWallScan:
    """(virtual) 네 모서리로 만든 평면 벽 + 가상 홈/돌출 + 바닥. 위치 → 반력."""

    def __init__(self, corners, defects=VIRTUAL_DEFECTS):
        pts = np.array(list(corners.values()), float)
        a = np.c_[np.ones(len(pts)), pts[:, 1], pts[:, 2]]
        self.coef = np.linalg.lstsq(a, pts[:, 0], rcond=None)[0]   # x = c0 + c1 y + c2 z
        self.defects = defects
        self.rng = np.random.default_rng()

    def surface_x(self, y, z):
        x = self.coef[0]+self.coef[1]*y+self.coef[2]*z
        for d in self.defects:
            if d['y'][0] <= y <= d['y'][1] and d['z'][0] <= z <= d['z'][1]:
                x += d['depth']
        return x

    def force_at(self, pos):
        x, y, z = pos[:3]
        f = np.zeros(6)
        f[0] = -VIRTUAL_STIFFNESS*max(0.0, x-self.surface_x(y, z))
        if z < VIRTUAL_FLOOR_Z:
            f[2] = VIRTUAL_STIFFNESS*(VIRTUAL_FLOOR_Z-z)
        f[:3] += self.rng.normal(0.0, 0.2, 3)
        return f


class ScanIO(RobotIO):
    """wall_scan_io.RobotIO 재사용 + 탐침 자세/Tool·TCP/순응제어/가상 벽만 바꿈."""

    def __init__(self, node, dsr, mode, tool_name, tcp_name, corners, probe_length=0.0, tool_roll=0.0):
        super().__init__(node, dsr, mode)
        self.tool_roll = float(tool_roll)
        self.tool_ori = rolled(TOOL_ORI, self.tool_roll)
        self.tool_name, self.tcp_name = tool_name, tcp_name
        self.virtual = VirtualWallScan(corners) if mode == 'virtual' else None
        # 모든 좌표(테두리·경로·기록)는 '탐침 끝' 기준. TCP = 탐침 끝 - 오프셋
        self.probe_offset = tool_z_axis(TOOL_ORI)*float(probe_length)

    def _posx(self, xyz):
        tcp = np.asarray(xyz, float)[:3]-self.probe_offset
        return self.dsr.posx(*[float(v) for v in tcp], *self.tool_ori)

    def get_pos(self):
        return super().get_pos()+self.probe_offset

    def get_joints(self):
        q = self.dsr.get_current_posj()
        return [float(v) for v in list(q)[:6]]

    def get_force(self, pos=None):
        if self.mode == 'virtual':
            return self.virtual.force_at(pos if pos is not None else self.get_pos())
        return super().get_force(pos)

    def setup(self):
        """자동 모드 + (real) Tool/TCP. 다르면 수동 모드에서 바꾸고 자동 모드로 (자동 모드에서는 거부됨)."""
        d = self.dsr
        if self.mode == 'real':
            tool, tcp = str(d.get_tool()).strip(), str(d.get_tcp()).strip()
            if tool != self.tool_name or tcp != self.tcp_name:
                self.node.get_logger().info(f'[Tool/TCP] {tool!r}/{tcp!r} → {self.tool_name!r}/{self.tcp_name!r}')
                d.set_robot_mode(d.ROBOT_MODE_MANUAL)
                time.sleep(0.5)
                try:
                    if tool != self.tool_name:
                        d.set_tool(self.tool_name)
                    if tcp != self.tcp_name:
                        d.set_tcp(self.tcp_name)
                finally:
                    d.set_robot_mode(d.ROBOT_MODE_AUTONOMOUS)
                    time.sleep(0.5)
                if str(d.get_tcp()).strip() != self.tcp_name:
                    raise RuntimeError(f'TCP 를 {self.tcp_name} 로 바꾸지 못했습니다. 펜던트에서 확인하세요.')
        d.set_robot_mode(d.ROBOT_MODE_AUTONOMOUS)
        if self.mode == 'real':
            d.set_ref_coord(d.DR_BASE)

    def start_compliance(self, stiffness):
        if self.mode != 'real':
            return
        self.dsr.task_compliance_ctrl(list(stiffness))
        self.compliance_on = True
        time.sleep(0.3)

    def joints_at(self, xyz, sol):
        """역기구학. 실패(-1 등)면 None."""
        q = self.dsr.ikin(self._posx(xyz), sol, ref=self.dsr.DR_BASE)
        try:
            q = [float(v) for v in list(q)[:6]]
        except TypeError:
            return None
        return q if len(q) == 6 else None


# =============================================================================
# 스캐너
# =============================================================================
class WallScanner:
    def __init__(self, io, corners, row_step, col_step, dirs, margin, floor_detect, publisher=None,
                 is_canceled=None, on_line=None, scan_method=SCAN_METHOD, scan_force=SCAN_FORCE,
                 top_force_ratio=TOP_FORCE_RATIO):
        self.io, self.corners = io, corners
        if scan_method not in ('force', 'spring'):
            raise ValueError("scan_method 는 force 또는 spring")
        if not 0.5 <= float(scan_force) <= SCAN_FORCE_MAX:
            raise ValueError(f'scan_force 는 0.5~{SCAN_FORCE_MAX:g} N (너무 세게 누르지 않게)')
        # 맨 위 줄 힘(scan_force × 비율)도 같은 범위 안이어야 한다
        if not 0.5 <= float(scan_force)*float(top_force_ratio) <= SCAN_FORCE_MAX:
            raise ValueError(f'scan_force × top_force_ratio 는 0.5~{SCAN_FORCE_MAX:g} N '
                             f'(입력: {scan_force} × {top_force_ratio})')
        self.scan_method, self.scan_force = scan_method, float(scan_force)
        self.top_force_ratio = float(top_force_ratio)
        self.height_range = None            # (맨 아래 줄 Z, 맨 위 줄 Z): run() 에서 경로를 만든 뒤 정함
        self.publisher = publisher
        self.is_canceled = is_canceled or (lambda: False)
        self.on_line = on_line              # (선, 순번, 전체, 기록 점 수) → 액션 피드백
        self.lines_done = []
        self.left_home = False              # 홈을 떠났는지 (실패 정리 때 후퇴/홈 복귀 여부 판단)
        self.extra = {k: 0.0 for k in BORDERS}   # 테두리에 걸려 학습한 추가 여유 [mm]
        self.base = None                    # 처음 경로의 테두리별 최소 거리
        self.border_hits = []
        self.obstacles = []                 # 첫 줄 이후 걸린 곳 = 틈/단차 '추정' (+ 건너뛴 구간)
        self.calibrated = False             # 첫 줄이 끝나면 True → 그 뒤로는 테두리 판정 안 함
        self.pending = []
        self.last_publish = 0.0
        self.row_step, self.col_step, self.dirs = row_step, col_step, dirs
        self.margin, self.floor_detect = margin, floor_detect
        self.floor_z = None
        self.samples = []
        self.line_log = []
        self.contact_now = True             # 기록 중 현재 접촉 상태 (히스테리시스)
        self.contact_x = None               # 마지막으로 접촉해 있던 X (허공 전진 감시 기준)
        self.log = io.node.get_logger()

    # ------------------------------------------------------------ HMI 표시
    def publish(self, payload):
        if self.publisher is None:
            return
        from std_msgs.msg import String
        self.publisher.publish(String(data=json.dumps(payload)))

    def flush_points(self, force=False):
        """기록한 점을 PUBLISH_PERIOD 마다 묶어서 HMI 로 보낸다. code 0=접촉, 1=접촉 끊김."""
        if not self.pending or (not force and time.monotonic()-self.last_publish < PUBLISH_PERIOD):
            return
        self.publish({'points': self.pending})
        self.pending, self.last_publish = [], time.monotonic()

    def publish_result(self, result):
        """분석이 끝나면 틈/돌출 점과 영역을 보낸다."""
        marks = [[*[round(v, 1) for v in s['xyz']], s['mark']] for s in self.samples if s.get('mark')]
        self.publish({'final': True, 'marks': marks, 'regions': result['regions'] if result else []})

    def lines(self, floor_z=None):
        return make_lines(self.corners, self.row_step, self.col_step, self.dirs, self.margin, floor_z=floor_z)

    # ------------------------------------------------------------ 안전 점검
    def check_cancel(self):
        if self.is_canceled():
            self.io.stop()
            raise Canceled('취소 요청')

    def check_force(self, f, context, pos=None):
        """pos 를 주면(스캔 중) 테두리 걸림(BorderHit) → 안쪽으로 옮겨 재탐색. 그 외(바닥 측정 등)는 중단."""
        self.check_cancel()
        mag = float(np.linalg.norm(f[:3]))
        side = float(np.linalg.norm(f[1:3]))
        if mag > HARD_FORCE or side > SIDE_FORCE:
            self.io.stop()
            text = f'[{context}] 비정상 힘 |F|={mag:.1f} N, 옆 {side:.1f} N'
            if pos is not None:
                raise BorderHit(text+' → 테두리에 걸림', pos, f)
            raise RuntimeError(text+' → 정지 후 중단')

    def preflight_with_roll(self, lines, extra=()):
        """설정한 공구 회전(tool_roll)으로 점검 → 실패하면 반대 방향 회전으로 다시 점검. 통과한 자세를 쓴다."""
        roll = self.io.tool_roll
        candidates = [roll] if abs(roll) < 1e-9 else [roll, -roll]
        last = None
        for candidate in candidates:
            self.io.tool_roll, self.io.tool_ori = candidate, rolled(TOOL_ORI, candidate)
            try:
                self.preflight(lines, extra)
                self.log.info(f'[공구 자세] Z축 {candidate:+g}° 회전 → {np.round(self.io.tool_ori, 2).tolist()}')
                return
            except RuntimeError as exc:
                last = exc
                if len(candidates) > 1 and candidate == candidates[0]:
                    self.log.warn(f'[공구 자세] Z축 {candidate:+g}° 회전 점검 실패 → {-candidate:+g}° 로 다시 점검: {exc}')
        raise last

    def preflight(self, lines, extra=()):
        """모든 선의 시작~끝 몇 점 + 후퇴점(+ extra 점)을 ikin 으로 확인. 하나라도 위험하면 움직이지 않는다."""
        if self.io.mode != 'real':
            return
        sol = self.io.dsr.get_current_solution_space()
        checks = [('경유점', np.asarray(q, float)) for q in extra]
        for line in lines:
            for k in range(PREFLIGHT_POINTS_PER_ROW):
                t = k/(PREFLIGHT_POINTS_PER_ROW-1)
                for offset in (0.0, -APPROACH_GAP, -(APPROACH_GAP+START_BACKOFF)):
                    checks.append((line_name(line), np.array(lerp(line['start'], line['end'], t))+APP_DIR*offset))
        worst = None
        for name, point in checks:
            q = self.io.joints_at(point, sol)
            if q is None:
                raise RuntimeError(f'[출발 전 점검] 도달 불가: {name} {np.round(point, 1).tolist()}. '
                                   '로봇은 움직이지 않았습니다.')
            if worst is None or abs(q[2]) < worst[0]:
                worst = (abs(q[2]), name)
            if abs(q[2]) < J3_MIN_DEG or abs(q[4]) < J5_MIN_DEG:
                kind = '팔꿈치(J3)' if abs(q[2]) < J3_MIN_DEG else '손목(J5)'
                raise RuntimeError(
                    f'[출발 전 점검] {name} {np.round(point, 1).tolist()} 에서 {kind} 특이점 근처 '
                    f'(J={np.round(q, 1).tolist()}). margin 을 키우거나 영역을 줄이세요. 로봇은 움직이지 않았습니다.')
        self.log.info(f'[출발 전 점검] {len(checks)}점 통과 (sol {sol}) | J3 최소 {worst[0]:.1f}° @ {worst[1]}')

    # ------------------------------------------------------------ 바닥
    def floor_start(self):
        """바닥 측정 시작점: 왼쪽 아래 모서리 안쪽(여유) · 벽 앞 APPROACH_GAP · 40 mm 위."""
        width = math.dist(self.corners['bottom_left'], self.corners['bottom_right'])
        bl = np.array(wall_point(self.corners, self.margin/width, 0.0))
        return bl-APP_DIR*APPROACH_GAP+np.array([0.0, 0.0, 40.0])

    def measure_floor(self):
        start = self.floor_start()
        self.log.info('[바닥] 왼쪽 아래 앞에서 순응제어로 내려가며 바닥 찾기')
        self.io.move_l(start, vel=MOVE_VEL, acc=MOVE_ACC)
        self.io.zero_force()
        contact = None
        try:
            self.io.start_compliance([OTHER_STIFFNESS, OTHER_STIFFNESS, FLOOR_STIFFNESS] + [ROT_STIFFNESS]*3)
            self.io.amove_l(start-np.array([0.0, 0.0, 100.0]), PROBE_VEL, PROBE_ACC)
            t0 = time.time()
            while time.time()-t0 < 40.0:
                pos = self.io.get_pos()
                f = self.io.get_force(pos)
                self.check_force(f, '바닥 찾기')
                if f[2] >= FLOOR_FORCE:
                    self.io.stop()
                    contact = pos
                    break
                if time.time()-t0 > 0.5 and not self.io.is_moving():
                    break
                time.sleep(SAMPLE_PERIOD)
            self.io.wait_motion_done()
        finally:
            self.io.release_press()
        self.io.move_l(start, vel=PROBE_VEL*3, acc=PROBE_ACC)
        if contact is None:
            self.log.info('  -> 바닥 못 찾음 → 아래 테두리 높이를 바닥으로 사용')
            return None
        self.log.info(f'  -> 바닥 Z = {contact[2]:.2f}')
        return float(contact[2])

    # ------------------------------------------------------------ 한 선
    def scan_line(self, line):
        start, end = np.array(line['start']), np.array(line['end'])
        self.log.info(f'[{line_name(line)}] ({start[1]:.0f}, {start[2]:.0f}) → ({end[1]:.0f}, {end[2]:.0f})')
        self.io.move_l(start-APP_DIR*(APPROACH_GAP+START_BACKOFF), vel=MOVE_VEL, acc=MOVE_ACC)
        self.io.move_l(start-APP_DIR*(PRE_GAP+START_BACKOFF), vel=PROBE_VEL*2, acc=PROBE_ACC)
        if self.io.mode == 'virtual':
            n = self._scan_virtual(line, start, end)
        else:
            n = self._scan_real(line, start, end)
        self.flush_points(force=True)
        pos = self.io.get_pos()
        self.io.move_l(pos-APP_DIR*APPROACH_GAP, vel=PROBE_VEL*3, acc=PROBE_ACC)
        self.log.info(f'  -> {n}점 기록')
        return n

    def _contact_force(self):
        return FORCE_CONTACT if self.scan_method == 'force' else CONTACT_FORCE

    def _begin_contact_tracking(self, pos):
        """미끄러지기 시작 = 벽에 붙은 상태. 접촉 상태와 허공 전진 기준 X 를 초기화한다."""
        self.contact_now, self.contact_x = True, float(pos[0])

    def _record(self, line, pos, f):
        """접촉 상태를 매 샘플 갱신(히스테리시스)하고, 1 mm 이상 움직였을 때만 점으로 기록한다."""
        on_force = self._contact_force()
        self.contact_now = contact_state(abs(f[0]), self.contact_now, on_force, on_force*CONTACT_RELEASE_RATIO)
        if self.contact_now:
            self.contact_x = float(pos[0])
        contact = self.contact_now
        last = next((s for s in reversed(self.samples) if s['line'] == line['line']), None)
        if last is None or np.linalg.norm(np.subtract(pos, last['xyz'])) >= SAMPLE_MIN_MM:
            self.samples.append({'line': line['line'], 'dir': line['dir'], 'index': line['index'],
                                 'xyz': [float(v) for v in pos[:3]],
                                 'force': float(abs(f[0])), 'contact': bool(contact)})
            self.pending.append([round(float(v), 1) for v in pos[:3]]+[0 if contact else 1])
            self.flush_points()
            return 1
        return 0

    def _scan_real(self, line, start, end):
        if self.scan_method == 'force':
            return self._scan_real_force(line, start, end)
        return self._scan_real_spring(line, start, end)

    def _attach(self, line, target, contact_force, stop_on_contact):
        """벽 쪽으로 다가가 붙기. 닿으면 True (stop_on_contact 면 그 자리에서 정지). 못 닿으면 False.

        amovel(비동기)로 벽 안쪽 목표까지 천천히 보내 놓고 20 ms 마다 위치·힘을 읽는다.
        접촉은 ContactDetector(힘 유지 + X 멈춤)로 판정한다 — 힘만 보면 허공에서 멈추는 일이 있었다.
        """
        name = line_name(line)
        self.io.amove_l(target, PROBE_VEL, PROBE_ACC)
        detector = ContactDetector(contact_force)
        t0 = time.monotonic()
        timeout = max(CONTACT_WAIT, np.linalg.norm(np.subtract(target, self.io.get_pos()))/PROBE_VEL*2+3)
        # 접촉 판정이 처음 가능해지는 시각 이후에만 '움직임 끝 = 못 닿음' 으로 본다
        motion_check_after = ATTACH_SETTLE_SEC+STALL_WINDOW_SEC+CONTACT_HOLD_SEC
        while True:
            pos = self.io.get_pos()
            f = self.io.get_force(pos)
            self.check_force(f, f'{name} 붙이기', pos=pos)
            now = time.monotonic()
            if detector.update(now, float(pos[0]), abs(f[0])):
                if stop_on_contact:
                    self.io.stop()
                break
            elapsed = now-t0
            if (elapsed > motion_check_after and not self.io.is_moving()) or elapsed > timeout:
                self.io.stop()
                self.line_log.append({'line': line['line'], 'name': name, 'status': 'no_contact'})
                self.log.info('  -> 벽에 닿지 못함 → 이 선 건너뜀 (벽이 더 멀거나 테두리 밖)')
                return False
            time.sleep(SAMPLE_PERIOD)
        self.io.wait_motion_done()
        return True

    def _slide(self, line, start, end, target_end, dive_ref=None):
        """끝점까지 미끄러지며 기록. dive_ref=(접촉점, 진행 단위벡터) 면 접촉면보다 깊이 들어가는지 감시."""
        name = line_name(line)
        count = 0
        self.io.amove_l(target_end, SCAN_VEL, SCAN_ACC)
        self._begin_contact_tracking(self.io.get_pos())
        t0 = time.time()
        timeout = np.linalg.norm(end-start)/SCAN_VEL*3+10
        slope = (end[0]-start[0])/max(np.linalg.norm(end-start), 1e-9)   # 주어진 벽의 진행 방향 X 기울기
        while True:
            pos = self.io.get_pos()
            f = self.io.get_force(pos)
            self.check_force(f, name, pos=pos)
            count += self._record(line, pos, f)
            if dive_ref is not None:
                self._check_dive(name, pos, dive_ref, slope)
            if time.time()-t0 > 0.5 and not self.io.is_moving():
                break
            if time.time()-t0 > timeout:
                self.io.stop()
                raise RuntimeError(f'{name} 스캔 시간 초과')
            time.sleep(SAMPLE_PERIOD)
        self.line_log.append({'line': line['line'], 'name': name, 'status': 'ok', 'points': count})
        return count

    def _check_dive(self, name, pos, dive_ref, slope):
        """(force) 힘제어가 벽 안쪽으로 너무 들어가면 정지 → DiveHit (구멍/틈 추정으로 처리되고 건너뜀).
          ① 접촉면보다 MAX_DIVE_MM 넘게 들어감
          ② 접촉이 끊긴 채 FREE_ADVANCE_MM 넘게 전진 — 허공을 밀고 들어가는 중 (8 mm 를 기다리면 너무 늦음)"""
        contact, unit = dive_ref
        expected_x = contact[0]+slope*float(np.dot(pos-contact, unit))
        if pos[0]-expected_x > MAX_DIVE_MM:
            self.io.stop()
            raise DiveHit(f'[{name}] 접촉면보다 {pos[0]-expected_x:.1f} mm 더 들어감 → 구멍/틈', pos)
        free_advance = pos[0]-self.contact_x
        if not self.contact_now and free_advance > FREE_ADVANCE_MM:
            self.io.stop()
            raise DiveHit(f'[{name}] 접촉이 끊긴 채 {free_advance:.1f} mm 전진 → 구멍/틈 또는 허공', pos)

    def line_force(self, line):
        """이 선에서 쓸 힘제어 목표 힘 [N]. 맨 아래 줄 = scan_force, 맨 위 줄 = scan_force × top_force_ratio.
        경로를 만들기 전(height_range 없음)이면 scan_force 그대로."""
        if self.height_range is None:
            return self.scan_force
        return scaled_scan_force(self.scan_force, self.top_force_ratio, line_height(line), *self.height_range)

    def _start_soft_x_compliance(self):
        """벽 쪽(X)만 부드러운 순응제어를 켜고, 안정된 뒤 힘 영점을 잡는다.
        순응제어로 바뀔 때 자세에 따라 힘 값이 1~2 N 틀어지므로(실측) 영점은 반드시 켠 '뒤에' 잡는다.
        (start_compliance 안에서 0.3 s 기다린 뒤 영점)"""
        self.io.start_compliance([APPROACH_STIFFNESS, OTHER_STIFFNESS, OTHER_STIFFNESS] + [ROT_STIFFNESS]*3)
        self.io.zero_force()

    def _scan_real_spring(self, line, start, end):
        """순응제어 + 목표를 벽 안쪽 PRESS_DEPTH 에 두는 스프링 방식."""
        try:
            self._start_soft_x_compliance()
            if not self._attach(line, start+APP_DIR*PRESS_DEPTH, CONTACT_FORCE, stop_on_contact=False):
                return 0
            return self._slide(line, start, end, end+APP_DIR*PRESS_DEPTH)
        finally:
            self.io.release_press()

    def _scan_real_force(self, line, start, end):
        """힘제어 방식: 순응제어로 다가가 실제 벽에 닿으면(FORCE_CONTACT) 멈춤 →
        X 강성 낮추고 set_desired_force(+X, scan_force, 상대) → 누른 채 미끄러짐.
        구멍에서 계속 밀고 들어가지 않도록 접촉면보다 MAX_DIVE_MM 넘게 들어가면 정지(DiveHit)."""
        d = self.io.dsr
        try:
            self._start_soft_x_compliance()
            # 주어진 벽 위치가 정확하지 않으므로, 벽보다 WALL_SEARCH_MM 더 안쪽까지 가며 실제 벽을 찾는다
            if not self._attach(line, start+APP_DIR*WALL_SEARCH_MM, FORCE_CONTACT, stop_on_contact=True):
                return 0
            contact = self.io.get_pos()
            force = self.line_force(line)
            self.log.info(f'  -> 벽 접촉 X {contact[0]:.1f} (계획 X 보다 {contact[0]-start[0]:+.1f} mm), '
                          f'누르는 힘 {force:.2f} N')
            d.set_stiffnessx([FORCE_STIFFNESS_X, OTHER_STIFFNESS, OTHER_STIFFNESS] + [ROT_STIFFNESS]*3, time=0.3)
            time.sleep(0.3)
            d.set_desired_force([force, 0.0, 0.0, 0.0, 0.0, 0.0], [1, 0, 0, 0, 0, 0],
                                time=0.3, mod=d.DR_FC_MOD_REL)
            self.io.force_on = True
            time.sleep(0.5)
            unit = (end-start)/max(np.linalg.norm(end-start), 1e-9)
            # 끝점 X 는 실제 접촉 X 기준으로 옮김 (X 는 힘제어가 벽을 따라감)
            target_end = end+(contact-start)*np.array([1.0, 0.0, 0.0])
            return self._slide(line, start, end, target_end, dive_ref=(contact, unit))
        finally:
            self.io.release_press()

    def _scan_virtual(self, line, start, end):
        """(virtual) 에뮬레이터는 순응제어가 없으므로, 누르는 힘만큼 표면을 파고든 경로를 직접 따라간다."""
        wall = self.io.virtual
        press = 5.0/VIRTUAL_STIFFNESS
        steps = max(int(np.linalg.norm(end-start)/3.0), 1)
        count = 0
        self._begin_contact_tracking(start)
        for t in np.linspace(0.0, 1.0, steps+1):
            self.check_cancel()
            p = start+(end-start)*t
            p[0] = wall.surface_x(p[1], p[2])+press
            self.io.move_l(p, vel=SCAN_VEL*4, acc=SCAN_ACC*4)
            pos = self.io.get_pos()
            count += self._record(line, pos, self.io.get_force(pos))
        self.line_log.append({'line': line['line'], 'name': line_name(line), 'status': 'ok', 'points': count})
        return count

    # ------------------------------------------------------------ 전체
    def check_home(self):
        """빈 그리퍼(닫힘)로 홈 자세에서 시작한다. 홈이 아니면 움직이지 않는다."""
        q = self.io.get_joints()
        off = max(abs(a-b) for a, b in zip(q, HOME_J))
        if off > HOME_TOL_DEG:
            raise RuntimeError(f'홈 자세에서 시작하세요 (현재 관절 {np.round(q, 1).tolist()}, 홈과 최대 {off:.1f}° 차이). '
                               'HMI [홈] 으로 보낸 뒤 다시 실행. 로봇은 움직이지 않았습니다.')

    def go_to_start(self):
        self.left_home = True
        """홈 → 벽 앞 경유점. 관절 이동(movej)으로 가서 직선 이동 중 특이점을 피한다.
        (기존 READY_J 는 J5=0 이라 손목 특이점에서 직선 이동을 시작하게 됨)"""
        q = None
        if self.io.mode == 'real' or hasattr(self.io.dsr, 'ikin'):
            try:
                q = self.io.joints_at(np.array(SAFE_WAYPOINT), self.io.dsr.get_current_solution_space())
            except Exception:
                q = None
        if q is not None:
            self.log.info(f'[시작] 홈 → 경유점 관절 이동 {np.round(q, 1).tolist()}')
            self.io.move_j(q)
        else:
            self.log.info('[시작] 경유점 역기구학 실패 → READY_J 경유')
            self.io.move_j(READY_J)
            self.io.move_l(SAFE_WAYPOINT, vel=MOVE_VEL, acc=MOVE_ACC)

    def run(self):
        self.publish({'reset': True, 'corners': self.corners})
        if self.io.mode == 'real':
            self.check_home()
        # 움직이기 전에, 바닥 측정 전 기준(가장 넓은 영역)으로 전 경로 + 경유점을 점검
        extra = [SAFE_WAYPOINT, self.floor_start()]
        deferred = False
        try:
            self.preflight_with_roll(self.lines(), extra=extra)
        except RuntimeError as exc:
            if not (self.floor_detect and self.io.mode == 'real'):
                raise
            # 받은 바닥이 실제보다 낮으면 바닥 잰 뒤엔 빠질 낮은 줄까지 점검됨 (run164/167: 받은 바닥 5.7, 실측 34 → z 20.7 줄)
            # → 경유점·바닥 측정 위치만 점검하고 출발, 줄들은 바닥을 잰 뒤 다시 점검
            self.log.warn(f'[출발 전 점검] 전체 줄 점검 실패 → 바닥을 잰 뒤 다시 점검 (경유점·바닥 측정 위치만 먼저): {exc}')
            self.preflight_with_roll([], extra=extra)
            deferred = True
        self.go_to_start()
        if self.floor_detect:
            self.floor_z = self.measure_floor() if self.io.mode == 'real' else VIRTUAL_FLOOR_Z
        lines = self.lines(self.floor_z)
        if deferred:
            # 공구 자세(롤)는 이미 정해져 있음 → 그 자세로만 점검. 실패하면 여기서 멈춤 (오류 처리에서 물러나 홈)
            self.preflight(lines)
            self.log.info(f'[출발 전 점검] 바닥 {self.floor_z if self.floor_z is not None else "(못 찾음)"} 기준 '
                          f'{len(lines)}줄 다시 점검 통과')
        heights = [line_height(line) for line in lines]
        self.height_range = (min(heights), max(heights))
        rows = sum(1 for line in lines if line['dir'] == 'h')
        self.log.info(f'[경로] 가로 {rows}줄 + 세로 {len(lines)-rows}열 | 간격 {self.row_step:g}/{self.col_step:g} mm, '
                      f'여유 {self.margin:g} mm, 바닥 {self.floor_z if self.floor_z is not None else "-"}')
        self.base = base_requirements(lines, self.corners, self.floor_z)
        for number, line in enumerate(lines, 1):
            n = self.scan_line_with_retry(line)
            if not self.calibrated:
                # 테두리 판정은 첫 줄(받은 테두리 위치로 처음 가 보는 줄)에서만
                self.calibrated = True
                self.log.info(f'[테두리 확인 끝] 학습 여유 {self.extra} → 이후 걸림은 틈/단차 추정으로 처리')
            self.lines_done.append(line['line'])
            if self.on_line is not None:
                self.on_line(line, number, len(lines), n)
        return lines

    def required(self):
        return {k: self.base[k]+self.extra[k] for k in BORDERS}

    def learn_border(self, hit, line, previous):
        """걸린 위치에서 가장 가까운 테두리의 여유를 INWARD_STEP_MM 늘린다.
        옮긴 뒤에도 거의 같은 곳(SAME_HIT_MM)에서 또 걸리면 그다음으로 가까운 테두리 탓으로 본다."""
        d = border_distances(self.corners, hit.pos, self.floor_z)
        order = sorted(BORDERS, key=lambda k: d[k])
        border = order[0]
        if previous is not None and previous['border'] == border and \
                math.dist(previous['pos'][1:], hit.pos[1:]) <= SAME_HIT_MM+INWARD_STEP_MM:
            border = order[1]
        self.extra[border] += INWARD_STEP_MM
        record = {'line': line['line'], 'name': line_name(line), 'pos': hit.pos, 'force': hit.force,
                  'border': border, 'distances': {k: round(v, 1) for k, v in d.items()},
                  'extra': dict(self.extra)}
        self.border_hits.append(record)
        names = {'left': '왼쪽', 'right': '오른쪽', 'bottom': '아래(바닥)', 'top': '위'}
        self.log.warn(f'[테두리 감지] {line_name(line)} {np.round(hit.pos, 1).tolist()} 에서 걸림 '
                      f'({hit}) → {names[border]} 테두리 여유 +{INWARD_STEP_MM:g} mm '
                      f'(누적 {self.extra[border]:g} mm), 안쪽으로 옮겨 다시 탐색')
        self.publish({'border_hit': [*[round(v, 1) for v in hit.pos], border]})
        return record

    def record_obstacle(self, hit, line, skip_end):
        """첫 줄 이후 걸림 → 테두리가 아니라 벽 위 틈/단차 '추정'으로 저장 (테두리 여유는 바꾸지 않음).
        걸린 곳 ~ 건너뛴 끝(skip_end) 구간을 추정 선분으로 함께 남긴다."""
        record = {'line': line['line'], 'name': line_name(line), 'pos': hit.pos, 'force': hit.force,
                  'skip_end': [round(float(v), 2) for v in skip_end], 'kind': 'gap_or_step_suspect'}
        self.obstacles.append(record)
        self.log.warn(f'[틈/단차 추정] {line_name(line)} {np.round(hit.pos, 1).tolist()} 에서 걸림 ({hit}) '
                      f'→ 추정으로 저장, {OBSTACLE_HOP_MM:g} mm 건너뛰어 계속')
        self.publish({'obstacle': [round(v, 1) for v in hit.pos]})

    def scan_line_with_retry(self, line):
        """한 선 탐색. 비정상 힘(걸림)이 나면
          - 첫 줄(테두리 확인 전): 테두리로 보고 안쪽으로 옮겨 다시 (최대 MAX_LINE_RETRY)
          - 그 뒤: 테두리 판정 안 함. 틈/단차 '추정'으로 저장하고 OBSTACLE_HOP_MM 건너뛰어 그 줄 나머지 계속
        걸리기 전까지 기록한 점은 그대로 둔다. Returns: 이 선에서 기록한 점 수."""
        def recorded():
            return sum(1 for s in self.samples if s['line'] == line['line'])

        previous, retries, hops, resume = None, 0, 0, None
        while True:
            self.check_cancel()
            adjusted = adjust_line(line, self.corners, self.required(), self.floor_z)
            if adjusted is None:
                self.log.info(f'[{line_name(line)}] 학습한 테두리 여유를 빼면 남는 구간이 없음 → 건너뜀')
                self.line_log.append({'line': line['line'], 'name': line_name(line), 'status': 'skipped_border'})
                return recorded()
            if resume is not None:          # 장애물 뒤부터 이어서
                a, b = np.array(adjusted['start']), np.array(adjusted['end'])
                unit = (b-a)/np.linalg.norm(b-a)
                along = float(np.dot(np.asarray(resume)-a, unit))
                if along >= np.linalg.norm(b-a)-SEGMENT_MIN_LEN*5:
                    self.log.info(f'[{line_name(line)}] 장애물 뒤로 남은 구간이 없음 → 이 줄 끝')
                    self.line_log.append({'line': line['line'], 'name': line_name(line), 'status': 'ok_partial'})
                    return recorded()
                adjusted['start'] = (a+unit*max(along, 0.0)).tolist()
            try:
                self.scan_line(adjusted)
                return recorded()
            except (BorderHit, DiveHit) as hit:
                self.flush_points(force=True)
                self.io.release_press()
                self.retreat()
                if self.calibrated or isinstance(hit, DiveHit):   # 구멍(깊이 들어감)은 항상 틈 추정
                    a, b = np.array(adjusted['start']), np.array(adjusted['end'])
                    unit = (b-a)/np.linalg.norm(b-a)
                    skip_end = np.asarray(hit.pos)+unit*OBSTACLE_HOP_MM
                    self.record_obstacle(hit, line, skip_end)
                    hops += 1
                    if hops > MAX_OBSTACLE_HOPS:
                        self.log.warn(f'[{line_name(line)}] 장애물 추정이 {MAX_OBSTACLE_HOPS}번 넘음 → 이 줄 나머지 건너뜀')
                        self.line_log.append({'line': line['line'], 'name': line_name(line),
                                              'status': 'skipped_obstacles'})
                        return recorded()
                    resume = skip_end.tolist()
                    continue
                previous = self.learn_border(hit, line, previous)
                retries += 1
                if retries > MAX_LINE_RETRY:
                    break
        self.log.warn(f'[{line_name(line)}] {MAX_LINE_RETRY}번 옮겨도 계속 걸림 → 이 선 건너뜀')
        self.line_log.append({'line': line['line'], 'name': line_name(line), 'status': 'skipped_retry'})
        return recorded()

    def finish(self, lines):
        """분석 → 결과 로그 → HMI 표시 → 저장. 점이 없으면 (None, None)."""
        if not self.samples:
            return None, None
        self.flush_points(force=True)
        used = lines or self.lines(self.floor_z)
        result = None
        try:
            result = analyze(self.samples, used)
            report(self.log, result)
            self.publish_result(result)
        except ValueError as exc:
            self.log.warn(f'분석 생략: {exc}')
        return result, self.save(used, result)

    def retreat(self):
        """벽에서 뒤로 물러남. 홈을 떠나지 않았으면(출발 전 점검 실패 등) 아무것도 하지 않는다.
        (홈 자세에서 탐침 자세로 직선 이동하면 자세가 크게 돌며 특이점에 들어간다)"""
        if not self.left_home:
            return False
        pos = self.io.get_pos()
        self.io.move_l(pos-APP_DIR*APPROACH_GAP, vel=PROBE_VEL*3, acc=PROBE_ACC)
        return True

    def save(self, lines, result):
        os.makedirs(SAVE_DIR, exist_ok=True)
        path = os.path.join(SAVE_DIR, datetime.now().strftime('interior_%Y%m%d_%H%M%S.json'))
        data = {
            'mode': self.io.mode, 'robot_model': ROBOT_MODEL,
            'created': datetime.now().isoformat(timespec='seconds'),
            'corners': self.corners, 'floor_z': self.floor_z,
            'config': {'row_step': self.row_step, 'col_step': self.col_step, 'dirs': self.dirs,
                       'margin': self.margin, 'press_depth': PRESS_DEPTH, 'stiffness_x': APPROACH_STIFFNESS,
                       'gap_depth_mm': GAP_DEPTH_MM, 'bump_height_mm': BUMP_HEIGHT_MM,
                       'scan_method': self.scan_method, 'scan_force': self.scan_force,
                       'top_force_ratio': self.top_force_ratio,
                       'tool_ori': self.io.tool_ori, 'tool_roll': self.io.tool_roll},
            'lines': [{k: line[k] for k in ('line', 'dir', 'index', 'start', 'end')} for line in lines],
            'line_status': self.line_log,
            'border_hits': self.border_hits,
            'obstacle_suspects': self.obstacles,
            'learned_extra_margin_mm': self.extra,
            'analysis': result,
            # 결과 gaps / DB gap 테이블과 같은 목록: 틈 1개당 [start_x,start_y,start_z, end_x,end_y,end_z] (아래 → 위)
            'gap_lines': vertical_gap_lines(result),
            'samples': self.samples,
            # wall_modeling / wall_rviz 형식: 가로 줄 = scan_h, 세로 열 = scan_v, 접촉 끊긴 점 = probe(빨강)
            'points': [{'kind': ('probe' if not s['contact'] else 'scan_h' if s['dir'] == 'h' else 'scan_v'),
                        'xyz': s['xyz'], 'force': s['force']} for s in self.samples],
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        segments = (result or {}).get('segments', [])
        legacy = {
            'robot': ROBOT_MODEL, 'timestamp': data['created'], 'wall_corners': self.corners,
            'gap_points': [{'line': s['line'], 'dir': s['dir'], 'position': dict(zip('xyz', s['xyz'])),
                            'depth_mm': s.get('depth'), 'contact': s['contact']}
                           for s in self.samples
                           if not s['contact'] or (s.get('depth') is not None and s['depth'] >= GAP_DEPTH_MM)],
            'gap_segments': [s for s in segments if s['kind'] == 'gap'],
            'gap_regions': [r for r in (result or {}).get('regions', []) if r['kind'] == 'gap'],
        }
        os.makedirs(os.path.dirname(RESULT_FILE), exist_ok=True)
        with open(RESULT_FILE, 'w') as f:
            json.dump(legacy, f, indent=2, ensure_ascii=False)
        self.log.info(f'저장: {path} (+ {RESULT_FILE})')
        return path


def report(log, result):
    st, pl = result['stats'], result['plane']
    log.info('================ 내부 탐색 결과 ================')
    log.info(f'  기록 {st["samples"]}점 (접촉 {st["contact"]}, 접촉 끊김 {st["contact_lost"]})')
    log.info(f'  평면 법선 {np.round(pl["normal"], 4).tolist()}, 평탄도 RMS {pl["rms_mm"]:.2f} mm, '
             f'깊이 {st["depth_min"]:+.2f} ~ {st["depth_max"]:+.2f} mm')
    for r in result['regions']:
        name = '틈/홈' if r['kind'] == 'gap' else '돌출'
        log.info(f'  [{name} {r["id"]}] {r["orientation"]} | Y {r["y_range"][0]:.0f}~{r["y_range"][1]:.0f}, '
                 f'Z {r["z_range"][0]:.0f}~{r["z_range"][1]:.0f} | 최대 깊이 {r["max_depth"]} mm | '
                 f'가로줄 {r["rows"]} 세로열 {r["cols"]}')
    if not result['regions']:
        log.info(f'  틈/홈/돌출 없음 (기준: 깊이 +{GAP_DEPTH_MM:g} / -{BUMP_HEIGHT_MM:g} mm)')
    lines = vertical_gap_lines(result)
    log.info(f'  → 결과로 내보낼 세로 틈 {len(lines)}개 ({MIN_GAP_ROWS}줄 이상, Y ±{GAP_Y_TOL_MM:g} mm)')
    for n, (sx, sy, sz, ex, _, ez) in enumerate(lines, 1):
        log.info(f'    [세로 틈 {n}] Y {sy:.1f} | Z {sz:.1f} → {ez:.1f} | X {sx:.1f} → {ex:.1f}')
    log.info('================================================')


def wall_edges(corners):
    """Process.Result.wall 순서 [top, floor, right, left] 각 변 중점 (BASE xyz)."""
    c = {k: np.asarray(v, float) for k, v in corners.items()}
    return [(c['top_left']+c['top_right'])/2, (c['bottom_left']+c['bottom_right'])/2,
            (c['top_right']+c['bottom_right'])/2, (c['top_left']+c['bottom_left'])/2]


def suspect_lines(obstacles):
    """틈/단차 추정(걸린 곳 ~ 건너뛴 끝) → Process.Result.gaps 와 같은 선분 형식."""
    return [[round(float(v), 2) for v in (*o['pos'], *o['skip_end'])] for o in obstacles]


def vertical_gap_lines(result, y_tol=GAP_Y_TOL_MM, max_missed=MAX_MISSED_ROWS, min_rows=MIN_GAP_ROWS):
    """가로 줄에서 찾은 틈 선분 → 세로 틈 선 [sx,sy,sz, ex,ey,ez] 목록 (Process.Result.gaps / DB gap 테이블 형식).

    세로 틈만 찾으므로 가로 줄 하나를 가로지른 '틈 조각'을 위아래로 쌓아 세로선을 만든다.
      1) 조각마다 가운데 Y. Y 차이 ≤ y_tol 이고 사이에 놓친 줄이 max_missed 줄 이하면 같은 묶음
      2) 서로 다른 줄 min_rows 개 미만인 묶음은 버림 (한 줄에서만 잠깐 끊긴 것 = 잡음)
      3) 살아남은 묶음끼리 Y 가 y_tol 안이면 하나로 합침 (가운데 여러 줄을 놓쳐도 같은 1자 틈)
      4) 선: Y = 평균, Z = 가장 아래 줄 → 가장 위 줄 (start = 아래, end = 위)
         X = 줄마다 잰 '틈 옆 벽면 X'(local_wall_x) 중 가장 작은 값(로봇에 가장 가까운 쪽)으로 양 끝 통일
             → 벽이 기울어 있어도 선이 벽 속으로 들어가지 않는다 (대신 반대쪽 끝은 벽에서 최대 1~2 mm 뜰 수 있음)
             틈 옆 벽면을 못 잰 줄은 벽 평면 값으로 대신한다
    가로 틈·걸림 추정은 내보내지 않는다 (DB 에 들어가면 그 자리에 퍼티 작업이 실행되므로 확인된 것만).
    """
    if not result:
        return []

    def piece(s):
        y, z = (s['y_range'][0]+s['y_range'][1])/2.0, s['z_range'][0]
        wall_x = s.get('wall_x')
        return {'row': s['index'], 'y': y, 'z': z,
                'x': wall_x if wall_x is not None else plane_x(result['plane'], y, z)}

    pieces = [piece(s) for s in result['segments'] if s['kind'] == 'gap' and s['dir'] == 'h']

    def near_rows(a, b):
        return abs(a['y']-b['y']) <= y_tol and abs(a['row']-b['row']) <= max_missed+1

    confirmed = [g for g in connected_groups(pieces, near_rows) if len({p['row'] for p in g}) >= min_rows]

    def same_y(a, b):
        return abs(np.mean([p['y'] for p in a])-np.mean([p['y'] for p in b])) <= y_tol

    lines = []
    for group in connected_groups(confirmed, same_y):
        members = [p for g in group for p in g]
        y = float(np.mean([p['y'] for p in members]))
        x = min(p['x'] for p in members)
        z_bottom, z_top = min(p['z'] for p in members), max(p['z'] for p in members)
        start, end = [x, y, z_bottom], [x, y, z_top]
        lines.append([round(float(v), 2) for v in start+end])
    return sorted(lines, key=lambda line: -line[1])          # 왼쪽(+Y)부터


# =============================================================================
# main
# =============================================================================
def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('wall_scanner', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    mode = node.declare_parameter('mode', 'virtual').value
    row_step = float(node.declare_parameter('row_step', ROW_STEP).value)
    col_step = float(node.declare_parameter('col_step', ROW_STEP).value)
    dirs = node.declare_parameter('dirs', 'both').value
    margin = float(node.declare_parameter('margin', MARGIN).value)
    floor_detect = bool(node.declare_parameter('floor_detect', True).value)
    tcp_name = node.declare_parameter('tcp_name', TCP_NAME).value
    probe_length = float(node.declare_parameter('probe_length', PROBE_LENGTH).value)
    tool_roll = float(node.declare_parameter('tool_roll', TOOL_ROLL_DEG).value)
    scan_method = node.declare_parameter('scan_method', SCAN_METHOD).value
    scan_force = float(node.declare_parameter('scan_force', SCAN_FORCE).value)
    top_force_ratio = top_force_ratio_or_default(node.declare_parameter('top_force_ratio', TOP_FORCE_RATIO).value)
    confirm = bool(node.declare_parameter('confirm', True).value)   # HMI 버튼 실행은 false (버튼이 확인)
    if mode not in ('virtual', 'real') or dirs not in ('rows', 'cols', 'both'):
        node.get_logger().error(f"mode 는 virtual|real, dirs 는 rows|cols|both (입력: {mode}, {dirs})")
        safe_shutdown(node)
        return
    try:
        import DSR_ROBOT2 as dsr
    except ImportError as e:
        node.get_logger().error(f'DSR_ROBOT2 import 실패: {e}')
        safe_shutdown(node)
        return

    from std_msgs.msg import String
    publisher = node.create_publisher(String, SCAN_TOPIC, 10)
    io = ScanIO(node, dsr, mode, TOOL_NAME, tcp_name, CORNERS, probe_length, tool_roll)
    scanner = WallScanner(io, CORNERS, row_step, col_step, dirs, margin, floor_detect, publisher,
                          scan_method=scan_method, scan_force=scan_force, top_force_ratio=top_force_ratio)
    node.get_logger().info(f'[설정] mode={mode}, dirs={dirs}, 탐침 길이 {probe_length:g} mm, TCP {tcp_name}')
    lines, result = [], None
    io.wait_for_services()
    if mode == 'real' and confirm:
        ans = input('[REAL] 탐침으로 벽 내부를 훑습니다. 주변 확인 후 y 입력: ')
        if ans.strip().lower() != 'y':
            node.get_logger().info('취소했습니다.')
            safe_shutdown(node)
            return
    try:
        io.setup()
        lines = scanner.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 (Ctrl+C)')
        io.stop()
    except Exception as e:
        node.get_logger().error(f'에러: {e}')
        io.stop()
    finally:
        def analyze_and_save():
            nonlocal result
            result, _ = scanner.finish(lines)

        def retreat_home():
            if scanner.retreat():
                io.move_j(HOME_J)

        run_cleanups(node, ('순응/힘 제어 해제 실패', io.release_press),
                     ('분석/저장 실패', analyze_and_save),
                     ('홈 복귀 실패', retreat_home))
        safe_shutdown(node)


if __name__ == '__main__':
    main()
