#!/usr/bin/env python3
# [work18 11/17] DR_init 설정 + 노드 생성을 utils.aspects.create_dsr_node 로 통합
"""두산 M0609 로 '작업공간 상판(테이블 윗면)'을 지정하고, 천천히 더듬어 탐색하는 노드.

[전체 흐름]
  task:=teach   : 상판 모서리 3~4 점을 티칭 → ~/table_scan_data/table_area.json 저장
                  (상판 모서리가 라운드 → 라운드가 시작되는 곳보다 '안쪽 평평한 면'에
                   TCP 를 대고 Enter. 찍은 사각형 = 영역 경계, 둥근 모서리는 영역 밖이 됨.
                   4점이면 한 평면에 있는지 검사해서 경사면에 찍힌 점을 경고)
  task:=explore : 지정한 영역 안에서만 탐색
      STEP 0. 진입          : 현재 위치에서 '수직으로' 올라간 뒤, 안전 높이로 영역 위까지 이동
      STEP 1. 격자 프로빙   : 격자 점마다 아래(-Z)로 천천히 내려가다 힘 감지 → 상판 높이
      STEP 2. 평면 산출     : 접촉점으로 상판 평면(높이/기울기) 계산
      STEP 3. 슬라이딩 스캔 : 상판을 일정한 힘으로 누른 채 가로/세로 줄로 미끄러지며 연속 측정
      STEP 4. 저장 + 모델링 : ~/table_scan_data/scan_*.json, *_model.json, *_model.png
                              + 최신 모델은 ~/table_scan_data/table_model.json (다른 노드에서 사용)
  task:=model   : 로봇 없이, 최근(또는 file:=...) 탐색 데이터로 모델링만 다시

[주변 물체 대비 안전 장치]
  - 탐색 영역은 BASE X > 0 (로봇 앞쪽) 만 허용. 모서리/모든 이동 목표점의 X 를 검사.
  - 진입/탐색 중 모든 이동 목표점은 '지정 영역(사각형) 안'인지 검사 → 밖이면 움직이기 전에 중단.
  - 영역 경계에서 EDGE_MARGIN 만큼 안쪽만 누름 (모서리 밖 물체/상판 끝 보호).
  - 수평 이동은 항상 안전 높이(상판 + SAFE_HEIGHT)에서만, 오르내림은 수직으로만.
  - 이동 중에도 힘을 감시(가드 이동) → GUARD_FORCE 이상이면 즉시 정지 후 중단.
  - 기본 속도가 느리고, speed:=0.5 처럼 전체 속도를 더 줄일 수 있음.
  - dry_run:=true : 상판에 닿지 않고 안전 높이에서 경로만 순회 (경로/도달성 확인용).
  - 끝나면 그 자리에서 수직으로 올라가 정지 (홈 복귀는 go_home:=true 일 때만).

[실행]
  # 1) bringup
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609

  # 2-a) 영역 티칭 (모서리를 '둘레를 따라 순서대로'. 3점이면 가운데 점이 꺾이는 모서리)
  ros2 run rokey1 table_navigation --ros-args -p mode:=real -p task:=teach
  # 2-b) 또는 좌표로 직접 지정 (사각형 x_min,x_max,y_min,y_max + 상판 대략 높이 z)
  ros2 run rokey1 table_navigation --ros-args -p mode:=virtual -p area:="300,600,-200,200" -p table_z:=0.0

  # 3) 탐색 (area 를 안 주면 table_area.json 사용, table_z 만 주면 높이만 덮어씀)
  ros2 run rokey1 table_navigation --ros-args -p mode:=real -p dry_run:=true   # 먼저 경로 확인!
  ros2 run rokey1 table_navigation --ros-args -p mode:=real -p speed:=0.5

  # 4) 모델링만 다시
  ros2 run rokey1 table_navigation --ros-args -p task:=model

[RViz]  Fixed Frame = base_link,  Add → By topic → /dsr01/table_markers

[주의]
  - teach 와 explore 의 TCP 가 같아야 한다 (real 은 TCP_NAME 을 둘 다 set_tcp).
  - 티칭할 때의 공구 자세가 탐색 자세로 저장된다 → 공구가 아래(-Z)를 보게 하고 티칭할 것.
    (좌표로 지정한 경우 TOOL_ORI 사용)
  - 측정값은 TCP 위치 = 공구 끝이 닿은 높이.
"""
import json
import math
import os
import sys
import time
from datetime import datetime
import glob

import numpy as np
import rclpy

from utils.aspects import create_dsr_node
from utils.move_stop import MoveStopClient
from utils.service_waiter import ServiceWaiter
from utils.shutdown import run_cleanups, safe_shutdown

try:
    from navigation.wall_modeling import robust_fit_plane, plane_basis, fit_quadratic_surface
    from navigation.wall_rviz import WallMarkerPublisher
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from wall_modeling import robust_fit_plane, plane_basis, fit_quadratic_surface
    from wall_rviz import WallMarkerPublisher

# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'
TOOL_NAME = 'Tool Weight'     # (real 전용)
TCP_NAME = 'GripperDA_v1'     # (real 전용)
TOOL_ORI = [0.0, 180.0, 0.0]  # 공구가 아래(-Z)를 보는 자세 (좌표로 영역 지정 시 사용) TODO: 실제 값 확인
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# =============================================================================
# 탐색 설정 (mm, BASE 기준)
# =============================================================================
#   상판을 위에서 본 그림 (모서리 c0→c1→c2→c3 순서, s/t = 영역 안의 비율 좌표)
#
#        c3 ─────────────── c2
#        │  •      •      •  │   • = 프로빙 지점 (PROBE_GRID)
#      t │  •      •      •  │   가로 스캔: t 고정, s 방향 (SCAN_ROWS)
#        │  •      •      •  │   세로 스캔: s 고정, t 방향 (SCAN_COLS)
#        c0 ─────────────── c1
#                 s
X_MIN_LIMIT = 0.0             # 탐색 허용 범위: BASE X > 이 값 (로봇 앞쪽만)
EDGE_MARGIN = 30.0            # 영역 경계에서 이만큼 안쪽만 누름
SAFE_HEIGHT = 80.0            # 수평 이동 높이 = 상판 최고 높이 + 이 값
APPROACH_GAP = 30.0           # 예상 상판보다 이만큼 위에서 프로빙 시작
SEARCH_DEPTH = 20.0           # 예상 상판보다 이만큼 더 내려가도 안 닿으면 '상판 없음'
DEEP_SEARCH_DEPTH = 500.0     # 1차(SEARCH_DEPTH)에서 못 닿으면 2차로 예상 높이보다 이만큼 아래까지
                              # (실제 상판이 table_z/티칭 높이보다 낮을 때 대비, deep_depth 파라미터)
OBJECT_WARN = 10.0            # 예상 높이보다 이만큼 위에서 닿으면 '물체일 수 있음' 경고
TEACH_Z_TOL = 0.75            # 티칭 4점 평면 잔차 경고 기준 [mm]
                              # (한 점이 e mm 어긋나면 잔차는 e/4 → 0.75 = 한 점 약 3mm 어긋남)
PROBE_GRID = (3, 3)           # 프로빙 격자 (s 방향 개수, t 방향 개수)
SCAN_ROWS = [0.25, 0.5, 0.75] # 가로 스캔 줄 위치 (t 비율)
SCAN_COLS = [0.25, 0.5, 0.75] # 세로 스캔 줄 위치 (s 비율)
SCAN_STEP = 10.0              # (virtual) 슬라이딩 경로 간격
RVIZ_EVERY = 10               # (real) 슬라이딩 중 N 개 점마다 RViz 갱신

# =============================================================================
# 속도 / 힘 (speed 파라미터로 전체 배율 조절)
# =============================================================================
MOVE_VELOCITY, MOVE_ACC = 50.0, 100.0      # 일반 이동 (느리게)
DESCEND_VELOCITY = 20.0                    # 안전 높이 → 프로빙 시작점 하강
PROBE_VELOCITY, PROBE_ACC = 5.0, 30.0      # 더듬는 속도 (정밀 측정)
DEEP_PROBE_VELOCITY = 20.0                 # 2차 깊은 탐색 속도 (deep_vel 파라미터)
                                      # 빠를수록 접촉 순간 더 파고듦 → 실제 로봇은 30 이하 권장
DEEP_BACKOFF = 15.0                   # 2차에서 닿으면 이만큼 올라가 PROBE_VELOCITY 로 정밀 재측정
SCAN_VELOCITY, SCAN_ACC = 10.0, 30.0       # 슬라이딩 속도
ROT_VELOCITY, ROT_ACC = 20.0, 40.0         # 자세 회전 속도 [deg/s]
JOINT_VELOCITY, JOINT_ACC = 20.0, 20.0

FORCE_THRESHOLD = 5.0                 # 접촉 판정 [N]
GUARD_FORCE = 25.0                    # 일반 이동 중 이 힘 이상이면 충돌로 보고 정지 [N]
SCAN_FORCE = 5.0                      # 슬라이딩 누르는 힘 [N]
SCAN_STIFFNESS = [3000.0, 3000.0, 500.0, 200.0, 200.0, 200.0]  # (real) Z 만 부드럽게
SAMPLE_PERIOD = 0.02
PROBE_TIMEOUT_MARGIN = 10.0   # 프로빙 시간 제한 = 내려갈 거리/속도 + 이 값 [s]
MOVE_TIMEOUT = 60.0

SAVE_DIR = os.path.expanduser('~/table_scan_data')
AREA_FILE = os.path.join(SAVE_DIR, 'table_area.json')
LATEST_MODEL = os.path.join(SAVE_DIR, 'table_model.json')

# (virtual 전용) 가상 상판: 예상 높이에서 z_offset 만큼 어긋나고, 살짝 기울고, 가운데 혹
VIRTUAL_TABLE = {
    'z_offset': -4.0,       # 진짜 상판 = 지정 높이 - 4mm (예상과 다를 때도 찾는지 확인)
    'tilt_x': 0.01,         # X 1mm 당 Z 변화 (≈0.57°)
    'tilt_y': -0.015,       # Y 1mm 당 Z 변화 (≈0.86°)
    'bump_height': 2.0,     # 가운데 볼록 [mm]
    'bump_sigma': 50.0,
    'stiffness': 20.0,      # [N/mm]
    'noise': 0.2,           # [N]
}


# 로봇 상태 / 자주 나오는 알람 (DRFC.py 기준)
STATE_NAMES = {0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
               5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING', 8: 'RECOVERY',
               9: 'SAFE_STOP2', 10: 'SAFE_OFF2'}
SAFE_STOP_STATES = (5, 9)
CONTROL_RESET_SAFE_STOP = 2
ALARM_HINTS = {
    7060: '충돌 감지 안전정지 (COLLISION_VIOLATION). 외력/모델 토크 차이를 충돌로 판단함. '
          'real: 실제로 부딪혔는지 확인. virtual: 실제 접촉이 없으므로 오감지 → '
          'collision_sensitivity 파라미터로 감도를 낮추거나 speed 를 낮춰 보세요',
}


class SafetyError(RuntimeError):
    """영역 밖 이동 / 예상치 못한 접촉 등 안전 관련 중단."""


def _cross2(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def zyz_to_matrix(ori):
    """두산 posx 자세 (rx, ry, rz) [deg] = ZYZ 오일러 → 회전행렬."""
    a, b, c = np.radians(ori[:3])

    def rz(t):
        return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])

    def ry(t):
        return np.array([[np.cos(t), 0, np.sin(t)], [0, 1, 0], [-np.sin(t), 0, np.cos(t)]])
    return rz(a) @ ry(b) @ rz(c)


def rotation_angle_deg(ori1, ori2):
    """두 자세 사이의 실제 회전 각도 [deg] (ZYZ 표현이 달라도 같은 자세면 0)."""
    R = zyz_to_matrix(ori1).T @ zyz_to_matrix(ori2)
    return float(np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1.0, 1.0))))


def tool_down_angle_deg(ori):
    """공구 Z 축이 BASE -Z(아래)에서 벗어난 각도 [deg]."""
    z_axis = zyz_to_matrix(ori)[:, 2]
    return float(np.degrees(np.arccos(np.clip(-z_axis[2], -1.0, 1.0))))


# =============================================================================
# 작업 영역 (상판 사각형)
# =============================================================================
class WorkArea:
    def __init__(self, corners, tool_ori=None, source=''):
        c = np.asarray(corners, dtype=float)
        if len(c) == 3:                      # 평행사변형의 나머지 모서리
            c = np.vstack([c, c[0] + c[2] - c[1]])
        if c.shape != (4, 3):
            raise ValueError(f'모서리는 3개 또는 4개여야 합니다 (현재 {len(c)}개)')
        self.corners = c
        self.tool_ori = list(tool_ori) if tool_ori is not None else None
        self.source = source

        if (c[:, 0] <= X_MIN_LIMIT).any():
            raise SafetyError(f'영역 모서리 X 가 {X_MIN_LIMIT}mm 이하입니다 (BASE +X 쪽만 탐색 가능): '
                              f'{np.round(c[:, 0], 1).tolist()}')

        xy = c[:, :2]
        crosses = [_cross2(xy[(i + 1) % 4] - xy[i], xy[(i + 2) % 4] - xy[(i + 1) % 4]) for i in range(4)]
        if not (all(v > 0 for v in crosses) or all(v < 0 for v in crosses)):
            raise ValueError('모서리 순서가 꼬였거나 볼록 사각형이 아닙니다 (둘레를 따라 순서대로 지정)')
        self.orient = 1.0 if crosses[0] > 0 else -1.0

        # 모서리 높이로 대략적인 상판 평면 z = p*x + q*y + r
        A = np.c_[xy, np.ones(4)]
        self.guess_coef = np.linalg.lstsq(A, c[:, 2], rcond=None)[0]

        p0, p1, p2, p3 = xy
        self.len_s = min(np.linalg.norm(p1 - p0), np.linalg.norm(p2 - p3))
        self.len_t = min(np.linalg.norm(p3 - p0), np.linalg.norm(p2 - p1))
        if self.len_s <= 2 * EDGE_MARGIN or self.len_t <= 2 * EDGE_MARGIN:
            raise ValueError(f'영역({self.len_s:.0f}x{self.len_t:.0f}mm)이 EDGE_MARGIN 에 비해 너무 작습니다')

    # ---------------------------------------------------------------- 생성
    @classmethod
    def from_rect(cls, x_min, x_max, y_min, y_max, z):
        return cls([[x_min, y_min, z], [x_max, y_min, z], [x_max, y_max, z], [x_min, y_max, z]],
                   source='params')

    @classmethod
    def from_file(cls, path=AREA_FILE):
        with open(path) as f:
            d = json.load(f)
        return cls(d['corners'], d.get('tool_ori'), source=path)

    def with_height(self, z):
        c = self.corners.copy()
        c[:, 2] = z
        return WorkArea(c, self.tool_ori, self.source + f' (z={z})')

    def to_dict(self):
        return {'corners': self.corners.round(2).tolist(), 'tool_ori': self.tool_ori,
                'source': self.source, 'size_mm': [round(self.len_s, 1), round(self.len_t, 1)]}

    # ---------------------------------------------------------------- 기하
    @property
    def center(self):
        return self.corners[:, :2].mean(axis=0)

    @property
    def top_z(self):
        return float(self.corners[:, 2].max())

    def guess_z(self, x, y):
        p, q, r = self.guess_coef
        return float(p * x + q * y + r)

    def point(self, s, t, margin=EDGE_MARGIN):
        """비율 좌표 (s, t ∈ [0,1]) → 경계에서 margin 만큼 안쪽의 (x, y)."""
        ms, mt = margin / self.len_s, margin / self.len_t
        s = ms + s * (1 - 2 * ms)
        t = mt + t * (1 - 2 * mt)
        p0, p1, p2, p3 = self.corners[:, :2]
        return (1 - s) * (1 - t) * p0 + s * (1 - t) * p1 + s * t * p2 + (1 - s) * t * p3

    def contains(self, xy, tol=1.0):
        """(x, y) 가 영역 사각형 안(경계에서 tol mm 바깥까지 허용)인지."""
        xy = np.asarray(xy[:2], dtype=float)
        for i in range(4):
            a, b = self.corners[i, :2], self.corners[(i + 1) % 4, :2]
            dist = self.orient * _cross2(b - a, xy - a) / np.linalg.norm(b - a)
            if dist < -tol:
                return False
        return True


# =============================================================================
# (virtual) 가상 상판
# =============================================================================
class VirtualTable:
    def __init__(self, cfg, area):
        self.cfg = cfg
        self.cx, self.cy = area.center
        self.z0 = area.guess_z(self.cx, self.cy) + cfg['z_offset']
        self.rng = np.random.default_rng()

    def plane_z(self, x, y):
        c = self.cfg
        return self.z0 + c['tilt_x'] * (x - self.cx) + c['tilt_y'] * (y - self.cy)

    def surface_z(self, x, y):
        c = self.cfg
        bump = c['bump_height'] * np.exp(-((x - self.cx) ** 2 + (y - self.cy) ** 2) / (2 * c['bump_sigma'] ** 2))
        return self.plane_z(x, y) + bump

    def force_at(self, pos):
        x, y, z = pos[:3]
        pen = self.surface_z(x, y) - z               # + 이면 상판 속으로 들어감
        fz = self.cfg['stiffness'] * max(0.0, pen)   # 상판은 +Z 로 밀어냄
        n = self.rng.normal(0.0, self.cfg['noise'], 3)
        return [n[0], n[1], fz + n[2], 0.0, 0.0, 0.0]

    def truth(self):
        return {**self.cfg, 'z0': self.z0, 'center_xy': [self.cx, self.cy]}


# =============================================================================
# 로봇 입출력 (virtual/real 차이 숨김 + 안전 검사)
# =============================================================================
class RobotIO:
    def __init__(self, node, dsr, mode, speed=1.0):
        self.node = node
        self.dsr = dsr
        self.mode = mode
        self.speed = float(np.clip(speed, 0.05, 1.0))
        self.tool_ori = list(TOOL_ORI)
        self.area = None                 # 설정되면 모든 목표점을 영역 안인지 검사
        self.virtual_table = None
        self.force_bias = np.zeros(6)
        self.force_on = False
        self.compliance_on = False
        self.stopper = MoveStopClient(node)
        from dsr_msgs2.srv import SetRobotControl
        self._SetRobotControl = SetRobotControl
        self.ctrl_cli = node.create_client(SetRobotControl, 'dsr_controller2/system/set_robot_control')

    def wait_for_services(self):
        d = self.dsr
        clients = [d._ros2_set_robot_mode, d._ros2_movej, d._ros2_movel,
                   d._ros2_get_current_posx, d._ros2_check_motion, self.stopper.cli,
                   d._ros2_ikin, d._ros2_get_robot_state, d._ros2_get_last_alarm,
                   d._ros2_change_collision_sensitivity, self.ctrl_cli]
        if self.mode == 'real':
            clients += [d._ros2_set_current_tool, d._ros2_set_current_tcp,
                        d._ros2_get_tool_force, d._ros2_set_ref_coord,
                        d._ros2_task_compliance_ctrl, d._ros2_set_desired_force,
                        d._ros2_release_force, d._ros2_release_compliance_ctrl]
        ServiceWaiter(self.node).wait(*clients)

    # ---------------------------------------------------------------- 상태 / 알람
    def robot_state(self):
        st = self.dsr.get_robot_state()
        return int(st) if isinstance(st, (int, float)) else -1

    def diagnose(self):
        """로봇 상태 + 마지막 알람을 사람이 읽을 수 있게 출력 (에러 났을 때 원인 파악용)."""
        try:
            st = self.robot_state()
            alarm = self.dsr.get_last_alarm()
            msg = f'[진단] 로봇 상태: {STATE_NAMES.get(st, st)}'
            if not isinstance(alarm, (int, float)):
                msg += f', 마지막 알람: group {alarm.group} / index {alarm.index} (level {alarm.level})'
                if alarm.index in ALARM_HINTS:
                    msg += f'\n       → {ALARM_HINTS[alarm.index]}'
            self.node.get_logger().warn(msg)
            return st
        except Exception as e:
            self.node.get_logger().warn(f'[진단] 상태 읽기 실패: {e}')
            return -1

    def is_safe_stopped(self):
        try:
            return self.robot_state() in SAFE_STOP_STATES
        except Exception:
            return False

    def ensure_ready(self):
        """시작 전에 안전정지 상태면: virtual 은 자동 해제, real 은 사람이 확인하고 해제하도록 중단."""
        st = self.robot_state()
        if st not in SAFE_STOP_STATES:
            return
        self.diagnose()
        if self.mode == 'real':
            raise SafetyError('로봇이 안전정지 상태입니다. 원인 확인 후 티칭 펜던트에서 해제하고 다시 실행하세요')
        self.node.get_logger().info('[virtual] 에뮬레이터 안전정지 해제 (CONTROL_RESET_SAFE_STOP)')
        req = self._SetRobotControl.Request()
        req.robot_control = CONTROL_RESET_SAFE_STOP
        future = self.ctrl_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        time.sleep(1.0)
        if self.robot_state() in SAFE_STOP_STATES:
            raise SafetyError('안전정지 해제 실패 - 에뮬레이터를 다시 실행해 보세요')

    def setup(self, collision_sensitivity=None):
        d = self.dsr
        self.ensure_ready()
        d.set_robot_mode(d.ROBOT_MODE_AUTONOMOUS)
        if collision_sensitivity is not None:
            ret = d.change_collision_sensitivity(int(collision_sensitivity))
            self.node.get_logger().info(f'충돌 감도 = {int(collision_sensitivity)} (ret={ret})')
        if self.mode == 'real':
            d.set_tool(TOOL_NAME)
            d.set_tcp(TCP_NAME)
            d.set_ref_coord(d.DR_BASE)

    # ---------------------------------------------------------------- 안전 검사
    def check_target(self, xyz, in_area=True):
        if xyz[0] <= X_MIN_LIMIT:
            raise SafetyError(f'목표 X={xyz[0]:.1f} 가 허용 범위(X > {X_MIN_LIMIT}) 밖입니다')
        if in_area and self.area is not None and not self.area.contains(xyz):
            raise SafetyError(f'목표 ({xyz[0]:.1f}, {xyz[1]:.1f}) 가 지정 영역 밖입니다')

    # ---------------------------------------------------------------- 이동
    def move_j(self, q):
        self._check(self.dsr.movej(self.dsr.posj(*q), vel=JOINT_VELOCITY * self.speed,
                                   acc=JOINT_ACC * self.speed), 'movej', q)

    def move_l(self, xyz, vel=MOVE_VELOCITY, acc=MOVE_ACC, guard=True, in_area=True, ori=None):
        """직선 이동 (끝까지 대기). guard=True 면 이동 중 힘 감시 → 충돌 시 정지.
        ori 를 주면 그 자세로, 안 주면 탐색 자세(tool_ori)로."""
        self.check_target(xyz, in_area)
        target = self._posx(xyz, ori)
        if not guard:
            self._check(self.dsr.movel(target, vel=self._v(vel), acc=self._a(acc),
                                       ref=self.dsr.DR_BASE), 'movel', target)
            return
        self._check(self.dsr.amovel(target, vel=self._v(vel), acc=self._a(acc),
                                    ref=self.dsr.DR_BASE), 'amovel', target)
        t0 = time.time()
        while True:
            pos = self.get_pos()
            f = float(np.linalg.norm(self.get_force(pos)[:3]))
            if f >= GUARD_FORCE:
                self.stop()
                raise SafetyError(f'이동 중 예상치 못한 접촉 {f:.1f}N @ {np.round(pos, 1).tolist()}')
            if time.time() - t0 > 0.5 and not self.is_moving():
                return
            if time.time() - t0 > MOVE_TIMEOUT:
                self.stop()
                raise RuntimeError('이동 시간 초과')
            time.sleep(SAMPLE_PERIOD)

    def rotate_to_tool_ori(self):
        """현재 위치에서 자세만 tool_ori 로 바꾼다.
        movel 로 크게 회전하면 손목 특이점/관절 한계로 실패할 수 있어서,
        ikin 으로 관절값을 구해 movej 로 돌린다 (현재 팔 형태 solution space 유지)."""
        posx, sol = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        angle = rotation_angle_deg(posx[3:], self.tool_ori)
        if angle < 2.0:
            return
        target = self.dsr.posx(*[float(v) for v in posx[:3]], *self.tool_ori)
        q = self.dsr.ikin(target, int(sol), ref=self.dsr.DR_BASE)
        if isinstance(q, (int, float)):        # 실패하면 -1
            raise RuntimeError(f'ikin 실패: 자세 {self.tool_ori} 를 현재 위치에서 만들 수 없음')
        self.node.get_logger().info(
            f'[자세 변경] {np.round(posx[3:], 1).tolist()} → {self.tool_ori} ({angle:.0f}° 회전, movej)')
        self.move_j([float(v) for v in q])

    def amove_l(self, xyz, vel, acc, in_area=True):
        self.check_target(xyz, in_area)
        target = self._posx(xyz)
        self._check(self.dsr.amovel(target, vel=self._v(vel), acc=self._a(acc),
                                    ref=self.dsr.DR_BASE), 'amovel', target)

    def stop(self):
        self.stopper.stop(self.dsr.DR_QSTOP, timeout_sec=2.0)

    def is_moving(self):
        return self.dsr.check_motion() != self.dsr.DR_STATE_IDLE

    def wait_motion_done(self, timeout=10.0):
        t0 = time.time()
        while self.is_moving() and time.time() - t0 < timeout:
            time.sleep(SAMPLE_PERIOD)

    # ---------------------------------------------------------------- 센서
    def get_posx(self):
        posx, _ = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        return [float(v) for v in posx]

    def get_pos(self):
        return np.array(self.get_posx()[:3])

    def get_force(self, pos=None):
        if self.mode == 'virtual':
            if self.virtual_table is None:
                return np.zeros(6)
            return np.array(self.virtual_table.force_at(pos if pos is not None else self.get_pos()))
        f = self.dsr.get_tool_force(ref=self.dsr.DR_BASE)
        if not isinstance(f, list):
            raise RuntimeError('get_tool_force 실패')
        return np.array(f, dtype=float) - self.force_bias

    def zero_force(self, samples=10):
        if self.mode != 'real':
            return
        self.force_bias = np.zeros(6)
        self.force_bias = np.mean([self.get_force() for _ in range(samples)], axis=0)

    # ---------------------------------------------------------------- 힘 제어 (real)
    def start_press(self, force):
        """BASE -Z 방향으로 force[N] 누르기."""
        d = self.dsr
        d.task_compliance_ctrl(SCAN_STIFFNESS)
        self.compliance_on = True
        time.sleep(0.2)
        d.set_desired_force([0, 0, -force, 0, 0, 0], [0, 0, 1, 0, 0, 0], mod=d.DR_FC_MOD_REL)
        self.force_on = True

    def release_press(self):
        if self.force_on:
            self.dsr.release_force()
            self.force_on = False
        if self.compliance_on:
            self.dsr.release_compliance_ctrl()
            self.compliance_on = False

    # ---------------------------------------------------------------- 내부
    def _v(self, vel):
        return [vel * self.speed, ROT_VELOCITY * self.speed]

    def _a(self, acc):
        return [acc * self.speed, ROT_ACC * self.speed]

    def _posx(self, xyz, ori=None):
        ori = self.tool_ori if ori is None else ori
        return self.dsr.posx(*[float(v) for v in xyz[:3]], *[float(v) for v in ori])

    @staticmethod
    def _check(ret, name, target=None):
        if isinstance(ret, (int, float)) and ret < 0:
            where = f' 목표 {np.round(list(target), 1).tolist()}' if target is not None else ''
            raise RuntimeError(f'{name} 실패 (ret={ret}){where} - 도달 불가/특이점/관절 한계 가능성')


# =============================================================================
# RViz (벽 퍼블리셔 재사용 + 영역 윤곽 + 상판용 텍스트)
# =============================================================================
class TableMarkerPublisher(WallMarkerPublisher):
    def publish_area(self, area):
        from visualization_msgs.msg import MarkerArray
        ms = []
        for ns, pts, color in (('area', area.corners, (1.0, 1.0, 0.0, 1.0)),
                               ('area_margin', None, (1.0, 0.6, 0.0, 0.7))):
            m = self._base(ns, 0, self._Marker.LINE_STRIP)
            m.scale.x = 0.003
            m.color = self._Color(r=color[0], g=color[1], b=color[2], a=color[3])
            if pts is None:   # 실제로 누르는 범위 (margin 안쪽)
                pts = [[*area.point(s, t), area.guess_z(*area.point(s, t))]
                       for s, t in ((0, 0), (1, 0), (1, 1), (0, 1))]
            pts = list(pts) + [pts[0]]
            m.points = [self._pt(p) for p in pts]
            ms.append(m)
        self.pub.publish(MarkerArray(markers=ms))

    def _info_text(self, model, centroid, normal):
        m = self._base('info', 0, self._Marker.TEXT_VIEW_FACING)
        p, f = model['plane'], model['flatness']
        m.pose.position = self._pt(np.array(centroid) + [0.0, 0.0, 80.0])
        m.scale.z = 0.025
        m.color = self._Color(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = (f'z@center {p["height_at_centroid_mm"]:.2f}mm  tilt {p["tilt_deg"]:.2f}deg\n'
                  f'slope x {p["slope_x_deg"]:.2f}deg  y {p["slope_y_deg"]:.2f}deg\n'
                  f'RMS {f["rms_mm"]:.2f}mm  P-V {f["peak_to_valley_mm"]:.2f}mm  '
                  f'({model["num_inliers"]}/{model["num_points"]} pts)')
        return m


# =============================================================================
# 상판 탐색 로직
# =============================================================================
class TableExplorer:
    def __init__(self, io, area, viz=None, dry_run=False, deep_depth=DEEP_SEARCH_DEPTH,
                 deep_vel=DEEP_PROBE_VELOCITY, min_z=None):
        self.io = io
        self.area = area
        self.viz = viz
        self.dry_run = dry_run
        self.deep_depth = float(deep_depth)
        self.deep_vel = float(deep_vel)
        # 절대 최저 높이 (이 아래로는 절대 안 내려감). 기본: 지정 높이 중 최저 - deep_depth
        self.min_z = float(min_z) if min_z is not None else \
            float(area.corners[:, 2].min()) - self.deep_depth
        self.z_offset = None        # 첫 접촉에서 구한 (실제 높이 - 지정 높이) 보정값
        self.plane = None           # STEP 2 결과 (normal 위쪽)
        self.points = []
        self.safe_z = area.top_z + SAFE_HEIGHT

    def update_rviz(self, model=None):
        if self.viz is None or not self.points:
            return
        try:
            self.viz.publish([p['xyz'] for p in self.points], [p['kind'] for p in self.points],
                             plane=self.plane, model=model)
        except Exception as e:
            self.io.node.get_logger().warn(f'RViz 퍼블리시 실패: {e}')

    def expected_z(self, x, y):
        """평면을 구했으면 평면 높이, 아니면 영역 지정 높이 (+ 첫 접촉 보정)."""
        if self.plane is not None:
            a, b, c = self.plane['normal']
            return float(-(a * x + b * y + self.plane['d']) / c)
        return self.area.guess_z(x, y) + (self.z_offset or 0.0)

    # ------------------------------------------------------------ 이동 패턴
    def enter(self):
        """현재 위치 → (자세 유지) 수직 상승 → 필요하면 movej 로 자세 변경 → 영역 위(첫 지점)까지."""
        posx = self.io.get_posx()
        cur, cur_ori = np.array(posx[:3]), posx[3:]
        if cur[0] <= X_MIN_LIMIT:
            raise SafetyError(f'현재 TCP X={cur[0]:.1f} 입니다. 로봇을 BASE +X 쪽으로 옮긴 뒤 실행하세요')

        # 영역을 좌표로 지정했고(티칭 자세 없음) 지금 공구가 이미 아래를 보고 있으면 그 자세를 그대로 사용
        if self.area.tool_ori is None and tool_down_angle_deg(cur_ori) < 15.0:
            self.io.tool_ori = [float(v) for v in cur_ori]
            self.io.node.get_logger().info(
                f'[진입] 현재 공구가 아래를 보고 있어 현재 자세 {np.round(cur_ori, 1).tolist()} 를 탐색 자세로 사용')
        elif tool_down_angle_deg(self.io.tool_ori) > 15.0:
            self.io.node.get_logger().warn(f'탐색 자세 {self.io.tool_ori} 의 공구가 아래를 보지 않습니다 '
                                           f'({tool_down_angle_deg(self.io.tool_ori):.0f}° 벗어남). TOOL_ORI/티칭 자세 확인')

        z = max(cur[2], self.safe_z)
        self.io.node.get_logger().info(f'[진입] 수직 상승 z={z:.0f} (자세 유지) → 자세 맞춤 → 영역 위로 이동')
        # 진입은 자세가 바뀔 수 있어 힘 영점이 맞지 않으므로 가드 없이 천천히
        self.io.move_l([cur[0], cur[1], z], guard=False, in_area=False, ori=cur_ori)
        self.io.rotate_to_tool_ori()
        first = self.area.point(0, 0)
        self.io.move_l([first[0], first[1], z], guard=False, in_area=False)
        self.io.move_l([first[0], first[1], self.safe_z], guard=False)
        self.io.zero_force()

    def goto_safe(self, xy):
        """안전 높이로 올라간 뒤 xy 위로 수평 이동."""
        pos = self.io.get_pos()
        if pos[2] < self.safe_z - 1.0:
            self.io.move_l([pos[0], pos[1], self.safe_z], guard=False)
        self.io.move_l([xy[0], xy[1], self.safe_z])

    def lift(self):
        pos = self.io.get_pos()
        if pos[2] < self.safe_z - 1.0:
            self.io.move_l([pos[0], pos[1], self.safe_z], vel=PROBE_VELOCITY * 3,
                           guard=False, in_area=False)

    # ------------------------------------------------------------ STEP 1
    def _descend(self, x, y, z_from, z_to, vel):
        """(x, y, z_from) 에서 z_to 까지 vel 로 내려가며 힘 감시.
        Returns: (접촉 위치 or None, 힘, 실제로 내려간 최저 z)"""
        z_to = max(z_to, self.min_z)                    # 절대 최저 높이 제한
        self.io.amove_l([x, y, z_to], vel, PROBE_ACC)
        contact, f_norm, lowest = None, 0.0, z_from
        t0 = time.time()
        timeout = abs(z_from - z_to) / (vel * self.io.speed) + PROBE_TIMEOUT_MARGIN
        while time.time() - t0 < timeout:
            pos = self.io.get_pos()
            lowest = min(lowest, float(pos[2]))
            f_norm = float(np.linalg.norm(self.io.get_force(pos)[:3]))
            if f_norm >= FORCE_THRESHOLD:
                self.io.stop()
                contact = pos
                break
            if time.time() - t0 > 0.5 and not self.io.is_moving():
                break
            time.sleep(SAMPLE_PERIOD)
        else:
            self.io.stop()
            self.io.node.get_logger().warn('  -> 시간 초과로 정지')
        self.io.wait_motion_done()
        return contact, f_norm, lowest

    def _deep_search(self, x, y, z_from, z_to):
        """2차: 빠르게(deep_vel) 깊이 내려가 대략 위치를 찾고,
        DEEP_BACKOFF 만큼 올라가 PROBE_VELOCITY 로 다시 눌러 정확한 접촉점을 구한다."""
        self.io.node.get_logger().info(
            f'  -> 2차 탐색: z {z_from:.1f} → {max(z_to, self.min_z):.1f} ({self.deep_vel:.0f}mm/s)')
        rough, f, lowest = self._descend(x, y, z_from, z_to, self.deep_vel)
        if rough is None:
            return None, f, lowest
        self.io.node.get_logger().info(f'  -> 대략 접촉 z={rough[2]:.1f} ({f:.1f}N), 정밀 측정')
        top = rough[2] + DEEP_BACKOFF
        self.io.move_l([x, y, top], vel=PROBE_VELOCITY * 2, guard=False)
        self.io.zero_force()
        return self._descend(x, y, top, rough[2] - DEEP_BACKOFF, PROBE_VELOCITY)

    def probe_point(self, xy):
        x, y = xy
        zg = self.expected_z(x, y)
        first = self.z_offset is None          # 아직 한 번도 안 닿음
        z_start = zg + APPROACH_GAP
        self.io.node.get_logger().info(
            f'[프로빙] ({x:.0f}, {y:.0f}) 예상 z={zg:.1f}' + (' (첫 점)' if first else ''))
        self.goto_safe(xy)
        if self.dry_run:
            return None
        self.io.move_l([x, y, z_start], vel=DESCEND_VELOCITY)
        self.io.zero_force()

        # 1차: 예상 높이 근처만 천천히
        contact, f_norm, lowest = self._descend(x, y, z_start, zg - SEARCH_DEPTH, PROBE_VELOCITY)
        if contact is None:                    # 예상보다 낮음 → 2차: 이어서 빠르고 깊게
            self.io.node.get_logger().warn(f'  -> 1차 {lowest:.1f}까지 접촉 없음 → 2차 탐색')
            contact, f_norm, lowest = self._deep_search(x, y, lowest, zg - self.deep_depth)
        self.lift()

        if contact is None:
            hit_floor = lowest <= self.min_z + 1.0
            self.io.node.get_logger().warn(
                f'  -> 접촉 없음: z={lowest:.1f}까지 내려감 (지정 높이보다 {zg - lowest:.0f}mm 아래). '
                + ('min_z 제한에 걸림 → min_z 를 낮추세요' if hit_floor
                   else 'table_z/티칭 높이가 실제와 맞는지, deep_depth 확인'))
            return None
        self.io.node.get_logger().info(
            f'  -> 접촉 z={contact[2]:.2f} (예상 대비 {contact[2] - zg:+.2f}mm), 힘 {f_norm:.2f}N')
        if first:
            self.z_offset = float(contact[2] - self.area.guess_z(x, y))
            self.io.node.get_logger().info(
                f'  -> 첫 접촉: 지정 높이 대비 {self.z_offset:+.2f}mm → 이후 예상 높이에 반영')
        if contact[2] - zg > OBJECT_WARN:
            self.io.node.get_logger().warn('  -> 예상보다 많이 높음: 상판 위 물체일 수 있음 (모델링에서 이상치로 처리)')
        self.points.append({'kind': 'probe', 'xyz': contact.tolist(), 'force': f_norm})
        self.update_rviz()
        return contact

    # ------------------------------------------------------------ STEP 2
    def calculate_plane(self):
        pts = [p['xyz'] for p in self.points if p['kind'] == 'probe']
        if len(pts) < 3:
            raise RuntimeError(f'접촉점이 {len(pts)}개뿐이라 평면을 만들 수 없습니다')
        self.plane = table_plane(pts)
        p = self.plane
        self.io.node.get_logger().info('=== 상판 평면 계산 완료 ===')
        self.io.node.get_logger().info(
            f'  중심 높이 z={p["centroid"][2]:.2f}mm, 기울기 {p["tilt_deg"]:.3f}°, RMS {p["rms"]:.3f}mm')
        self.update_rviz()

    # ------------------------------------------------------------ STEP 3
    def sliding_scan(self, xy0, xy1, kind):
        name = '가로' if kind == 'scan_h' else '세로'
        self.io.node.get_logger().info(
            f'[슬라이딩-{name}] ({xy0[0]:.0f}, {xy0[1]:.0f}) → ({xy1[0]:.0f}, {xy1[1]:.0f})')
        self.goto_safe(xy0)
        if self.dry_run:
            self.goto_safe(xy1)
            return
        n = self._sliding_virtual(xy0, xy1, kind) if self.io.mode == 'virtual' \
            else self._sliding_real(xy0, xy1, kind)
        self.update_rviz()
        self.io.node.get_logger().info(f'  -> {n}개 점 기록')

    def _sliding_virtual(self, xy0, xy1, kind):
        table = self.io.virtual_table
        press = SCAN_FORCE / VIRTUAL_TABLE['stiffness']
        p0, p1 = np.asarray(xy0, float), np.asarray(xy1, float)
        steps = max(int(round(np.linalg.norm(p1 - p0) / SCAN_STEP)), 1)
        self.io.move_l([*p0, table.surface_z(*p0) + APPROACH_GAP], vel=DESCEND_VELOCITY)
        count = 0
        for t in np.linspace(0.0, 1.0, steps + 1):
            x, y = p0 + t * (p1 - p0)
            self.io.move_l([x, y, table.surface_z(x, y) - press], vel=SCAN_VELOCITY, acc=SCAN_ACC,
                           guard=False)
            pos = self.io.get_pos()
            f = float(np.linalg.norm(self.io.get_force(pos)[:3]))
            self.points.append({'kind': kind, 'xyz': pos.tolist(), 'force': f})
            count += 1
            self.update_rviz()
        self.lift()
        return count

    def _sliding_real(self, xy0, xy1, kind):
        z0, z1 = self.expected_z(*xy0), self.expected_z(*xy1)
        self.io.move_l([xy0[0], xy0[1], z0 + APPROACH_GAP], vel=DESCEND_VELOCITY)
        self.io.zero_force()
        self.io.move_l([xy0[0], xy0[1], z0 + 10.0], vel=PROBE_VELOCITY * 2)
        count = 0
        try:
            self.io.start_press(SCAN_FORCE)
            t0 = time.time()
            while abs(self.io.get_force()[2]) < 0.8 * SCAN_FORCE:
                if time.time() - t0 > 5.0:
                    raise RuntimeError('힘 제어로 상판에 닿지 못함 (평면 높이 확인)')
                time.sleep(SAMPLE_PERIOD)
            self.io.amove_l([xy1[0], xy1[1], z1], SCAN_VELOCITY, SCAN_ACC)
            t0 = time.time()
            while time.time() - t0 < 0.5 or self.io.is_moving():
                pos = self.io.get_pos()
                fv = self.io.get_force()
                f = float(np.linalg.norm(fv[:3]))
                if abs(fv[2]) >= 0.5 * SCAN_FORCE:
                    self.points.append({'kind': kind, 'xyz': pos.tolist(), 'force': f})
                    count += 1
                    if count % RVIZ_EVERY == 0:
                        self.update_rviz()
                if f > 3 * SCAN_FORCE:      # 상판 위 물체에 걸림 등
                    self.io.stop()
                    raise SafetyError(f'슬라이딩 중 과도한 힘 {f:.1f}N @ {np.round(pos, 1).tolist()}')
                time.sleep(SAMPLE_PERIOD)
        finally:
            self.io.release_press()
            self.lift()
        return count

    def check_crossings(self, tol=3.0):
        h = np.array([p['xyz'] for p in self.points if p['kind'] == 'scan_h'])
        v = np.array([p['xyz'] for p in self.points if p['kind'] == 'scan_v'])
        if len(h) == 0 or len(v) == 0:
            return
        diffs = []
        for t in SCAN_ROWS:
            for s in SCAN_COLS:
                c = self.area.point(s, t)
                hh = h[np.hypot(h[:, 0] - c[0], h[:, 1] - c[1]) < tol + SCAN_STEP / 2]
                vv = v[np.hypot(v[:, 0] - c[0], v[:, 1] - c[1]) < tol + SCAN_STEP / 2]
                if len(hh) and len(vv):
                    diffs.append(hh[:, 2].mean() - vv[:, 2].mean())
        if diffs:
            d = np.abs(diffs)
            self.io.node.get_logger().info(
                f'[교차점 점검] {len(d)}곳, 가로-세로 Z 차이 평균 {d.mean():.3f}mm, 최대 {d.max():.3f}mm')

    # ------------------------------------------------------------ 전체
    def run_pipeline(self):
        a = self.area
        self.io.node.get_logger().info(
            f'=== 상판 탐색 시작 (mode={self.io.mode}, dry_run={self.dry_run}, speed={self.io.speed}) ===')
        self.io.node.get_logger().info(
            f'  영역 {a.len_s:.0f} x {a.len_t:.0f} mm, 안전 높이 z={self.safe_z:.0f}')
        if self.viz is not None:
            self.viz.clear()
            self.viz.publish_area(a)
        self.enter()

        # STEP 1: 격자 프로빙 (지그재그 순서로 이동 거리 최소화)
        ns, nt = PROBE_GRID
        for j, t in enumerate(np.linspace(0, 1, nt)):
            ss = np.linspace(0, 1, ns)
            for s in (ss if j % 2 == 0 else ss[::-1]):
                self.probe_point(a.point(s, t))
        if self.dry_run:
            self.plane = None
        else:
            self.calculate_plane()

        # STEP 3: 가로(s 방향) / 세로(t 방향) 슬라이딩, 지그재그
        self.io.node.get_logger().info('=== 가로 슬라이딩 스캔 ===')
        for i, t in enumerate(SCAN_ROWS):
            s0, s1 = (0, 1) if i % 2 == 0 else (1, 0)
            self.sliding_scan(a.point(s0, t), a.point(s1, t), 'scan_h')
        self.io.node.get_logger().info('=== 세로 슬라이딩 스캔 ===')
        for i, s in enumerate(SCAN_COLS):
            t0, t1 = (0, 1) if i % 2 == 0 else (1, 0)
            self.sliding_scan(a.point(s, t0), a.point(s, t1), 'scan_v')

        self.check_crossings()
        self.io.node.get_logger().info('=== 탐색 완료 ===')

    def save(self):
        os.makedirs(SAVE_DIR, exist_ok=True)
        path = os.path.join(SAVE_DIR, datetime.now().strftime('scan_%Y%m%d_%H%M%S.json'))
        data = {
            'mode': self.io.mode,
            'robot_model': ROBOT_MODEL,
            'created': datetime.now().isoformat(timespec='seconds'),
            'area': self.area.to_dict(),
            'config': {'probe_grid': list(PROBE_GRID), 'scan_rows': SCAN_ROWS, 'scan_cols': SCAN_COLS,
                       'edge_margin': EDGE_MARGIN, 'safe_height': SAFE_HEIGHT,
                       'force_threshold': FORCE_THRESHOLD, 'scan_force': SCAN_FORCE,
                       'deep_depth': self.deep_depth, 'deep_vel': self.deep_vel, 'z_offset': self.z_offset,
                       'min_z': self.min_z,
                       'tool_ori': self.io.tool_ori, 'speed': self.io.speed},
            'virtual_table': self.io.virtual_table.truth() if self.io.virtual_table else None,
            'points': self.points,
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        self.io.node.get_logger().info(f'탐색 데이터 저장: {path} ({len(self.points)}개 점)')
        return path


# =============================================================================
# 상판 모델링
# =============================================================================
def table_plane(points):
    """이상치 제거 평면 피팅 + 법선을 항상 위(+Z)로. 기울기 정보 포함."""
    pts = np.asarray(points, dtype=float)
    plane, mask = robust_fit_plane(pts)
    n = np.asarray(plane['normal'], dtype=float)
    if n[2] < 0:
        n = -n
    c = np.asarray(plane['centroid'], dtype=float)
    res = (pts - c) @ n
    return {
        'normal': n, 'centroid': c, 'd': -float(n @ c), 'inliers': mask,
        'residuals_all': res, 'rms': float(np.sqrt(np.mean(res[mask] ** 2))),
        'tilt_deg': float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0)))),
    }


def build_table_model(points):
    pts = np.asarray(points, dtype=float)
    pl = table_plane(pts)
    n, c, inl, w = pl['normal'], pl['centroid'], pl['inliers'], pl['residuals_all']
    a, b, cz = n
    zc = [float(-a / cz), float(-b / cz), float(-pl['d'] / cz)]   # z = p*x + q*y + r
    u_ax, v_ax = plane_basis(n)
    rel = pts[inl] - c
    u, v = rel @ u_ax, rel @ v_ax
    w_in = w[inl]
    out = [{'xyz': pts[i].tolist(), 'height_above_plane_mm': float(w[i])} for i in np.where(~inl)[0]]
    return {
        'num_points': int(len(pts)),
        'num_inliers': int(inl.sum()),
        'inlier_mask': inl.tolist(),
        'plane': {
            'normal': n.tolist(),
            'centroid': c.tolist(),
            'equation': [float(a), float(b), float(cz), pl['d']],
            'equation_str': f'{a:.5f}x + {b:.5f}y + {cz:.5f}z + {pl["d"]:.3f} = 0',
            'z_coeffs': zc,                                  # z = p*x + q*y + r
            'z_formula': f'z = {zc[0]:.6f}*x + {zc[1]:.6f}*y + {zc[2]:.3f}',
            'height_at_centroid_mm': float(c[2]),
            'slope_x_deg': float(np.degrees(np.arctan(zc[0]))),
            'slope_y_deg': float(np.degrees(np.arctan(zc[1]))),
            'tilt_deg': pl['tilt_deg'],
        },
        'flatness': {
            'rms_mm': pl['rms'],
            'max_abs_mm': float(np.abs(w_in).max()),
            'peak_to_valley_mm': float(w_in.max() - w_in.min()),
        },
        'quadratic_surface': fit_quadratic_surface(u, v, w_in),
        'extent_uv_mm': {'u': [float(u.min()), float(u.max())], 'v': [float(v.min()), float(v.max())]},
        'outliers': out,     # 평면에서 튀는 점 (상판 위 물체 / 오측정 가능성)
    }


def compare_with_truth(model, truth):
    if not truth:
        return None
    cx, cy, cz = model['plane']['centroid']
    z_true = truth['z0'] + truth['tilt_x'] * (cx - truth['center_xy'][0]) + truth['tilt_y'] * (cy - truth['center_xy'][1])
    p = model['plane']
    return {
        'height_error_mm': float(cz - z_true),     # (혹 때문에 약간 + 로 나오는 게 정상)
        'slope_x_error_deg': float(p['slope_x_deg'] - np.degrees(np.arctan(truth['tilt_x']))),
        'slope_y_error_deg': float(p['slope_y_deg'] - np.degrees(np.arctan(truth['tilt_y']))),
    }


def plot_table(data, model, png_path, show=True):
    import matplotlib
    if not show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    pts = np.array([p['xyz'] for p in data['points']])
    kinds = np.array([p['kind'] for p in data['points']])
    inl = np.asarray(model['inlier_mask'], dtype=bool)
    n, c = np.array(model['plane']['normal']), np.array(model['plane']['centroid'])
    w = (pts - c) @ n

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.5))
    fig.suptitle(f'Table model ({data.get("mode", "?")}) - {os.path.basename(png_path)}')
    corners = np.array(data['area']['corners'])
    loop = np.vstack([corners, corners[:1]])
    ax1.plot(loop[:, 0], loop[:, 1], 'k-', lw=1.5, label='area')
    lim = max(np.abs(w[inl]).max(), 0.5)
    sc = ax1.scatter(pts[inl, 0], pts[inl, 1], c=w[inl], cmap='coolwarm', vmin=-lim, vmax=lim,
                     s=np.where(kinds[inl] == 'probe', 60, 12))
    if (~inl).any():
        ax1.scatter(pts[~inl, 0], pts[~inl, 1], c='k', marker='x', s=50, label='outlier')
    fig.colorbar(sc, ax=ax1, label='height above plane [mm]')
    ax1.set_xlabel('X [mm]')
    ax1.set_ylabel('Y [mm]')
    ax1.set_aspect('equal', adjustable='datalim')
    ax1.legend(loc='upper right')
    ax1.grid(alpha=0.3)
    p, f = model['plane'], model['flatness']
    ax1.set_title(f'z@center {p["height_at_centroid_mm"]:.2f}mm, tilt {p["tilt_deg"]:.3f}°')

    ax2.hist(w[inl], bins=20, color='tab:blue', alpha=0.8)
    ax2.set_title(f'RMS {f["rms_mm"]:.2f} mm / P-V {f["peak_to_valley_mm"]:.2f} mm')
    ax2.set_xlabel('height above plane [mm]')
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    print(f'[모델링] 그림 저장: {png_path}')
    if show:
        plt.show()
    plt.close(fig)


def model_from_file(json_path, show=True, on_model=None):
    with open(json_path) as f:
        data = json.load(f)
    model = build_table_model([p['xyz'] for p in data['points']])
    model['source'] = os.path.abspath(json_path)
    model['mode'] = data.get('mode')
    model['area'] = data.get('area')
    model['truth_check'] = compare_with_truth(model, data.get('virtual_table'))

    p, fl = model['plane'], model['flatness']
    print('================ 상판 모델 ================')
    print(f' 점 개수        : {model["num_inliers"]}/{model["num_points"]} (이상치 {len(model["outliers"])}개)')
    print(f' 평면식         : {p["z_formula"]}')
    print(f' 중심 높이      : {p["height_at_centroid_mm"]:.3f} mm')
    print(f' 기울기         : 전체 {p["tilt_deg"]:.3f}°, X방향 {p["slope_x_deg"]:.3f}°, Y방향 {p["slope_y_deg"]:.3f}°')
    print(f' 평탄도         : RMS {fl["rms_mm"]:.3f} mm, 최대 {fl["max_abs_mm"]:.3f} mm, P-V {fl["peak_to_valley_mm"]:.3f} mm')
    for o in model['outliers']:
        print(f' 이상치         : {np.round(o["xyz"], 1).tolist()} 평면 위 {o["height_above_plane_mm"]:+.2f} mm')
    if model['truth_check']:
        t = model['truth_check']
        print(f' [가상 상판 비교] 높이 오차 {t["height_error_mm"]:+.3f} mm, '
              f'기울기 오차 X {t["slope_x_error_deg"]:+.3f}°, Y {t["slope_y_error_deg"]:+.3f}°')
    print('==========================================')

    base = os.path.splitext(json_path)[0]
    for path in (base + '_model.json', LATEST_MODEL):
        with open(path, 'w') as f:
            json.dump(model, f, indent=2, ensure_ascii=False)
    print(f'[모델링] 모델 저장: {base}_model.json, {LATEST_MODEL}')
    if on_model is not None:
        on_model(model)
    plot_table(data, model, base + '_model.png', show=show)
    return model


def latest_scan_file():
    files = [f for f in glob.glob(os.path.join(SAVE_DIR, 'scan_*.json')) if not f.endswith('_model.json')]
    if not files:
        raise FileNotFoundError(f'{SAVE_DIR} 에 scan_*.json 이 없습니다')
    return max(files, key=os.path.getmtime)


# =============================================================================
# 작업
# =============================================================================
def teach_plane_residuals(corners):
    """4점을 평면 z = p*x + q*y + r 에 맞췄을 때 각 점의 z 잔차 [mm].
    한 점만 라운드 경사면/가장자리에 찍히면 그 점(과 대각 점)의 잔차가 커진다."""
    c = np.asarray(corners, dtype=float)
    A = np.c_[c[:, :2], np.ones(len(c))]
    coef = np.linalg.lstsq(A, c[:, 2], rcond=None)[0]
    return c[:, 2] - A @ coef


def do_teach(io):
    """상판 모서리 근처 '평평한 면'에 TCP 를 대고 Enter → 저장. 3~4 점.

    상판 모서리가 둥글어(라운드) 진짜 꼭짓점은 공중이므로, 라운드가 시작되는 곳보다
    안쪽의 평평한 면을 찍는다. 찍은 사각형이 그대로 작업영역 경계가 되고
    (둥근 모서리는 영역 밖), 실제로 누르는 곳은 그보다 EDGE_MARGIN 안쪽이다.
    """
    print('\n[티칭] 상판 모서리가 둥글므로, 각 모서리의 라운드가 시작되는 곳보다 "안쪽 평평한 면"에')
    print('       TCP 를 살짝 닿게 대고 Enter. (라운드 경사면/가장자리에 걸치지 않게)')
    print('       둘레를 따라 순서대로, BASE X > 0. 3점이면 가운데(2번째) 점이 꺾이는 모서리.')
    print('       q = 종료, u = 마지막 점 취소\n')
    print('         ╭──────────╮')
    print('         │ ●      ● │   ● = 티칭점')
    print('         │          │')
    print('         │ ●      ● │')
    print('         ╰──────────╯\n')
    corners, oris = [], []
    while len(corners) < 4:
        ans = input(f'모서리 {len(corners) + 1}/4 > ').strip().lower()
        if ans == 'q':
            break
        if ans == 'u':
            if corners:
                corners.pop()
                oris.pop()
                print('  마지막 점 취소')
            continue
        px = io.get_posx()
        if px[0] <= X_MIN_LIMIT:
            print(f'  X={px[0]:.1f} 는 허용 범위(X > {X_MIN_LIMIT}) 밖입니다. 다시 지정하세요.')
            continue
        msg = ''
        if corners:
            dz = px[2] - corners[0][2]
            msg = f'  (1번 점 대비 z {dz:+.1f}mm)'
            if abs(dz) > 5.0:       # 상판 기울기로도 생길 수 있어 참고용 경고
                msg += '  ← 높이 차이가 큼: 라운드/가장자리에 걸치지 않았는지 확인 (u 로 취소 가능)'
        corners.append(px[:3])
        oris.append(px[3:])
        print(f'  저장: [{px[0]:.1f}, {px[1]:.1f}, {px[2]:.1f}] 자세 [{px[3]:.1f}, {px[4]:.1f}, {px[5]:.1f}]{msg}')

    if len(corners) < 3:
        print('모서리가 3개 미만이라 저장하지 않았습니다.')
        return
    if len(corners) == 4:
        res = teach_plane_residuals(corners)
        print(f'  평면 일치 검사 (z 잔차): {np.round(res, 2).tolist()} mm')
        if np.abs(res).max() > TEACH_Z_TOL:
            print(f'  ⚠ 4점이 한 평면에서 {np.abs(res).max():.1f}mm 벗어남 → 한 점이 라운드 경사면/가장자리에')
            print('    찍혔거나 TCP 가 덜/더 닿았을 수 있습니다.')
            if input('    그래도 저장할까요? (y/N): ').strip().lower() != 'y':
                print('저장하지 않았습니다. 다시 티칭하세요.')
                return
    else:
        print('  (3점은 평면 일치 검사를 할 수 없음 → 정확도를 위해 4점 티칭 권장)')
    area = WorkArea(corners, tool_ori=oris[0], source='teach')   # 검증 (순서/크기/X 범위)
    os.makedirs(SAVE_DIR, exist_ok=True)
    data = {**area.to_dict(), 'taught_corners': corners, 'taught_ori': oris,
            'created': datetime.now().isoformat(timespec='seconds'), 'mode': io.mode}
    with open(AREA_FILE, 'w') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    zs = area.corners[:, 2]
    print(f'\n영역 저장: {AREA_FILE}')
    print(f'  크기 {area.len_s:.0f} x {area.len_t:.0f} mm, 모서리 높이 {zs.min():.1f} ~ {zs.max():.1f} mm')
    print(f'  탐색 자세(1번 모서리 자세): {[round(v, 1) for v in oris[0]]}')


def load_area(node):
    from rcl_interfaces.msg import ParameterDescriptor
    any_type = ParameterDescriptor(dynamic_typing=True)   # 50 / 50.0 / "50" / [..] 모두 허용
    area_v = node.declare_parameter('area', '', any_type).value      # "x_min,x_max,y_min,y_max" 또는 [..]
    z_v = node.declare_parameter('table_z', '', any_type).value      # 상판 대략 높이
    path = node.declare_parameter('area_file', AREA_FILE).value
    z = float(z_v) if str(z_v).strip() else None
    if isinstance(area_v, (list, tuple)):
        area_v = ','.join(str(v) for v in area_v)
    if str(area_v).strip():
        vals = [float(v) for v in str(area_v).strip('[] ').split(',')]
        if len(vals) != 4 or z is None:
            raise ValueError('area:="x_min,x_max,y_min,y_max" 와 table_z:=<높이> 를 함께 주세요')
        x0, x1, y0, y1 = vals
        return WorkArea.from_rect(min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), z)
    if not os.path.exists(path):
        raise FileNotFoundError(f'{path} 없음. task:=teach 로 티칭하거나 area/table_z 파라미터를 주세요')
    area = WorkArea.from_file(path)
    return area.with_height(z) if z is not None else area


def main(args=None):
    rclpy.init(args=args)
    node = create_dsr_node('table_navigation', ROBOT_ID, ROBOT_MODEL)

    task = node.declare_parameter('task', 'explore').value     # teach | explore | model
    mode = node.declare_parameter('mode', 'virtual').value
    from rcl_interfaces.msg import ParameterDescriptor
    num = ParameterDescriptor(dynamic_typing=True)     # 1 / 1.0 둘 다 허용
    speed = float(node.declare_parameter('speed', 1.0, num).value)
    dry_run = node.declare_parameter('dry_run', False).value
    deep_depth = float(node.declare_parameter('deep_depth', DEEP_SEARCH_DEPTH, num).value)
    deep_vel = float(node.declare_parameter('deep_vel', DEEP_PROBE_VELOCITY, num).value)
    min_z_v = node.declare_parameter('min_z', '', num).value     # 절대 최저 높이 [mm] (BASE)
    min_z = float(min_z_v) if str(min_z_v).strip() else None
    run_modeling = node.declare_parameter('modeling', True).value
    show_plot = node.declare_parameter('show_plot', True).value
    use_rviz = node.declare_parameter('rviz', True).value
    from rcl_interfaces.msg import ParameterDescriptor as _PD
    cs_v = node.declare_parameter('collision_sensitivity', '', _PD(dynamic_typing=True)).value
    collision_sensitivity = int(float(cs_v)) if str(cs_v).strip() else None   # 0~100, 낮을수록 둔감
    go_home = node.declare_parameter('go_home', False).value
    frame_id = node.declare_parameter('frame_id', 'base_link').value
    file_s = node.declare_parameter('file', '').value

    if task == 'model':
        try:
            model_from_file(file_s or latest_scan_file(), show=show_plot)
        except Exception as e:
            node.get_logger().error(f'모델링 실패: {e}')
        safe_shutdown(node)
        return
    if task not in ('teach', 'explore') or mode not in ('virtual', 'real'):
        node.get_logger().error(f'task(teach/explore/model)={task}, mode(virtual/real)={mode} 확인')
        safe_shutdown(node)
        return

    try:
        import DSR_ROBOT2 as dsr
    except ImportError as e:
        node.get_logger().info(f'Error importing DSR_ROBOT2 : {e}')
        return
    io = RobotIO(node, dsr, mode, speed)
    io.wait_for_services()

    if task == 'teach':
        try:
            # 티칭 중엔 펜던트/직접교시로 움직여야 하므로 AUTONOMOUS 로 바꾸지 않고 TCP 만 맞춘다
            if mode == 'real':
                dsr.set_tool(TOOL_NAME)
                dsr.set_tcp(TCP_NAME)
            do_teach(io)
        except Exception as e:
            node.get_logger().error(f'티칭 실패: {e}')
        safe_shutdown(node)
        return

    # ---------------------------------------------------------------- explore
    try:
        area = load_area(node)
    except Exception as e:
        node.get_logger().error(f'영역 설정 오류: {e}')
        safe_shutdown(node)
        return
    io.area = area
    if area.tool_ori is not None:
        io.tool_ori = area.tool_ori
    if mode == 'virtual':
        io.virtual_table = VirtualTable(VIRTUAL_TABLE, area)

    viz = TableMarkerPublisher(node, topic='table_markers', frame_id=frame_id) if use_rviz else None
    explorer = TableExplorer(io, area, viz, dry_run=dry_run, deep_depth=deep_depth,
                             deep_vel=deep_vel, min_z=min_z)
    node.get_logger().info(f'2차 탐색: {deep_vel:.0f}mm/s, 예상 높이 -{deep_depth:.0f}mm 까지, '
                           f'최저 높이 제한 min_z={explorer.min_z:.1f}mm')
    node.get_logger().info(f'영역: {area.source}, 모서리 {np.round(area.corners, 1).tolist()}')

    if mode == 'real':
        cur = io.get_posx()
        print(f'\n[REAL] 현재 TCP {np.round(cur, 1).tolist()}')
        print(f'       탐색 자세 {np.round(io.tool_ori, 1).tolist()}, 안전 높이 z={explorer.safe_z:.0f}, '
              f'speed={io.speed}, dry_run={dry_run}')
        print('       진입: 수직 상승 → 안전 높이로 영역 위까지 수평 이동 (진입 중엔 자세도 바뀜)')
        if input('       주변 물체 확인 후 y 입력: ').strip().lower() != 'y':
            node.get_logger().info('취소했습니다.')
            safe_shutdown(node)
            return

    saved = None
    try:
        io.setup(collision_sensitivity)
        explorer.run_pipeline()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 (Ctrl+C)')
        io.stop()
    except Exception as e:
        node.get_logger().error(f'에러: {e}')
        io.stop()
        io.diagnose()
    finally:
        def lift():
            nonlocal go_home
            if io.is_safe_stopped():
                node.get_logger().warn('안전정지 상태라 상승/홈 복귀를 하지 않습니다 (해제 후 수동으로 이동)')
                go_home = False
            else:
                explorer.lift()             # 그 자리에서 수직으로만 올라감

        def save_points():
            nonlocal saved
            if explorer.points:
                saved = explorer.save()

        def move_home():
            if go_home:
                io.move_j(HOME_J)

        run_cleanups(node, ('힘 제어 해제 실패', io.release_press),
                     ('상승 실패', lift),
                     ('데이터 저장 실패', save_points),
                     ('홈 복귀 실패', move_home))

    try:
        if saved and run_modeling and len(explorer.points) >= 4:
            model_from_file(saved, show=show_plot, on_model=lambda m: explorer.update_rviz(model=m))
    except Exception as e:
        node.get_logger().error(f'모델링 실패: {e}')
    finally:
        safe_shutdown(node)


if __name__ == '__main__':
    main()
