#!/usr/bin/env python3
# [work18 11/17] DR_init 설정 + 노드 생성을 utils.aspects.create_dsr_node 로 통합
"""두산 M0609 로 벽을 '더듬어서' 벽의 높이 / 너비를 재는 노드 (virtual / real).

[알고리즘]
  1. 시작(벽 찾기)  : 바닥 근처에서 벽 쪽(APPROACH 방향)으로 천천히 전진
                       → 힘이 임계값을 넘으면 '막혔다' = 벽의 시작점
  2. 벽 따라가기    : 벽을 살짝 누른 채로 벽면을 따라 이동 (위로 / 옆으로)
                       → '비어있음(접촉 힘 사라짐)' 또는 '더 못 감(막힘/정체)' 상태가
                         END_HOLD_SEC(2초) 동안 계속되면 벽이 끝났다고 판단
                       → 마지막으로 벽에 닿아 있던 위치 = 벽의 끝점
  3. 선 측정        : 시작점 ~ 끝점 거리 = 그 선 위의 벽 길이
                       - line_v : 바닥 → 위로   → 그 위치(x)에서의 벽 높이
                       - line_h : 접촉점 → +x, 접촉점 → -x → 그 높이(z)에서의 벽 너비
  4. 스윕           : 시작점을 STEP(10mm)씩 옮기며 3 을 반복
                       - width  : z 를 STEP 씩 올리면서 x 방향(옆) 탐색 → 높이별 너비
                       - height : x 를 STEP 씩 옮기면서 z 방향(위) 탐색 → 위치별 높이
  5. 결과           : 각 스윕에서 '총 높이 / 총 너비' 출력 + ~/wall_scan_data/size_*.json 저장

[좌표]  (모두 로봇 BASE 기준, mm)
        Z(위, UP_AXIS)
        │      ┌────── 벽 (X-Z 평면) ──────┐
        │      │                           │
        │      │      로봇은 +Y 방향으로    │
        │      │      벽을 바라본다        │
  floor ┴──────┴───────────────────────────┴──── X (옆, LAT_AXIS)
  - 옆 방향 = X, 위 방향 = Z, 벽을 향하는 방향 = +Y  (축 설정은 아래 '축 설정'에서 변경)

[공구 = 닫은 그리퍼 끝 (탐침)]
  - 시작할 때 그리퍼를 닫고 (DO1=ON, DO2=OFF), 닫힌 손가락 끝(TCP)으로 더듬는다.
  - 그리퍼를 바닥과 벽 양쪽에서 45° 눕힌다 (옆에서 본 모습):

            Z ↑        │ 벽
              │  그리퍼 │
              │   ╲    │        공구 축 = (벽 쪽 +Y) 와 (아래 -Z) 사이 45° 대각선
              │    ╲   │        → 손가락 끝만 바닥/벽에 닿고, 그리퍼 몸체는
              │     ●  │          뒤-위쪽으로 빠져 있어서 벽/바닥과 부딪히지 않는다
       ───────┴────────┴──── 바닥       ● = 닫힌 손가락 끝 (TCP)

  - 측정값 = TCP(손가락 끝) 위치. TCP_NAME 이 '닫힌 손가락 끝' 으로 등록돼 있어야 함.

[벽 따라가기 방법]
  real   : task_compliance_ctrl 로 '벽 쪽 축만' 부드럽게(스프링) 만든 뒤, 목표점을
           벽 표면보다 PRESS_DEPTH 만큼 안쪽으로 두고 amovel → 스프링 힘으로 벽을 누르며
           미끄러짐. 벽이 끝나면 최대 PRESS_DEPTH 만큼만 앞으로 튀어나감 (힘 제어처럼
           허공으로 계속 밀고 나가지 않아서 안전).
  virtual: 에뮬레이터는 외력이 0 이므로 코드 안의 '가상 박스 벽'으로 힘을 계산.
           탐색 로직(전진→감지→따라가기→2초 판정)은 real 과 완전히 같다.

[실행]
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609

  ros2 run rokey1 wall_size --ros-args -p mode:=virtual                      # width + height 스윕
  ros2 run rokey1 wall_size --ros-args -p mode:=virtual -p scan:=width       # z 올리며 너비 스윕
  ros2 run rokey1 wall_size --ros-args -p mode:=virtual -p scan:=height      # x 옮기며 높이 스윕
  ros2 run rokey1 wall_size --ros-args -p mode:=virtual -p scan:=line_v -p x:=0.0    # 높이 1줄
  ros2 run rokey1 wall_size --ros-args -p mode:=virtual -p scan:=line_h -p z:=200.0  # 너비 1줄
  ros2 run rokey1 wall_size --ros-args -p mode:=real -p step:=20.0          # 실제 로봇, 20mm 간격
  # 빠른 측정: 너비 3줄(바닥 위 50/150/250) + 높이 3줄(x 자동) → 평균/최소/최대
  ros2 run rokey1 wall_size --ros-args -p mode:=virtual -p scan:=lines
  ros2 run rokey1 wall_size --ros-args -p mode:=real -p scan:=lines -p face:=500.0 \
      -p center:=0.0 -p lat_min:=-320.0 -p lat_max:=320.0 -p z_limit:=450.0 -p zs:="50,150,250" -p xs:=""

[예외 처리]  암/툴에 예상 밖 힘이 걸리면 → force_guard.py (감지/분류/정지/후퇴)
  - 툴 충돌        : 정지 → 벽 반대로 후퇴 → 힘 빠지면 그 선만 건너뛰고 계속 / 안 빠지면 중단
  - 팔 충돌/안전정지/센서 고장 : 즉시 정지, 전체 중단, 작업자 확인 후에만 홈 복귀
  - 가상 모드 시험 : -p inject:=tool_side|tool_overload|tool_torque|arm|estop|sensor -p inject_after:=20.0

[RViz]  Fixed Frame = base_link,  Add → By topic → /dsr01/wall_size_markers
        탐색 경로(회색 이동 / 노랑 전진 / 초록 따라가기), 접촉점, 측정선(H/W), 추정 벽,
        현재 그리퍼 자세를 실시간 표시 → 자세한 내용은 wall_size_rviz.py
        저장된 결과 다시 보기: ros2 run rokey1 wall_size_rviz

[실제 로봇 전 체크리스트]
  - WALL_FACE_GUESS : 벽 표면 Y 위치보다 '조금 앞' 으로 (줄자로 측정)
  - FLOOR_Z_DEFAULT / FLOOR_SEARCH_START_Z : 바닥 높이 근처인지
  - LAT_RANGE / Z_LIMIT : 벽보다 넓게, 하지만 로봇 작업 영역 안
  - READY_J 자세에서 공구가 +Y(벽 방향)를 바라보는지 (티칭 펜던트로 확인)
  - 처음엔 step 을 크게(예: 30), 속도는 낮게
"""
import json
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy

from utils.aspects import create_dsr_node
from utils.service_waiter import ServiceWaiter
from utils.shutdown import safe_shutdown

try:
    from navigation.wall_navigation import RobotIO
    from navigation.wall_size_rviz import WallSizeViz
    from navigation.force_guard import (ForceGuard, Injector, ForceGuardError, ToolCollision,
                                        ArmCollision, ControllerStop, SensorFault,
                                        MAX_TOOL_COLLISIONS, BAD_STATES)
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from wall_navigation import RobotIO
    from wall_size_rviz import WallSizeViz
    from force_guard import (ForceGuard, Injector, ForceGuardError, ToolCollision,
                             ArmCollision, ControllerStop, SensorFault,
                             MAX_TOOL_COLLISIONS, BAD_STATES)

# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'

# =============================================================================
# 축 설정  (0=X, 1=Y, 2=Z)
# =============================================================================
LAT_AXIS = 0            # 옆 방향 (너비를 재는 축)
APPROACH_AXIS = 1       # 벽을 향하는 축
APPROACH_SIGN = +1.0    # +1: +축 방향으로 가면 벽, -1: -축 방향으로 가면 벽
UP_AXIS = 2             # 위 방향 (높이를 재는 축)

# 그리퍼 기울기: 0 = 벽을 수평으로 봄, 90 = 바닥을 수직으로 봄 → 45 = 바닥/벽 모두에서 45°
TILT_DEG = 75.0  # 45° 는 벽 아래쪽에서 손목 특이점(J5≈0) 통과 → J4/J6 꼬임. 75° 는 J5≥22°
# ZYZ [a, b, c]: 공구 Z축 = (cos a·sin b, sin a·sin b, cos b)
#   a=90, b=90+45=135 → (0, 0.707, -0.707) = 벽 쪽(+Y) 45° + 아래(-Z) 45°
#   c = 공구 축 둘레 회전 (손가락이 벌어지는 방향). 필요하면 바꿀 것
TOOL_ORI = [90.0, 90.0 + TILT_DEG, 0.0]
READY_J = [90.0, 0.0, 90.0, 0.0, TILT_DEG, 0.0]  # 시작 자세 (J1=90 → +Y, J5=45 → 45° 숙임)
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

# 그리퍼 (tape_peel.py / sun_gear.py 와 같은 배선): 닫기 = DO1 ON, DO2 OFF
GRIP_DO_CLOSE = (1, 1)      # (포트, 값)
GRIP_DO_OPEN = (2, 0)
GRIP_WAIT_SEC = 1.0         # 그리퍼가 다 닫힐 때까지 대기

# =============================================================================
# 탐색 영역  (mm)
# =============================================================================
WALL_FACE_GUESS = 500.0     # 벽 표면이 있을 것이라 '예상'하는 APPROACH 축 좌표
APPROACH_GAP = 40.0         # 예상 표면보다 이만큼 앞에서 전진 시작 / 후퇴 거리
SEARCH_DEPTH = 60.0         # 예상 표면보다 이만큼 더 들어가도 안 닿으면 '벽 없음'

LAT_CENTER_GUESS = 0.0      # 벽 가운데쯤의 옆(X) 위치 (너비 스윕 첫 줄 시작점)
LAT_RANGE = (-320.0, 320.0) # 옆 방향 탐색 한계 (벽보다 넓게! 높이 스윕 x 범위 + 따라가기 한계)
Z_LIMIT = 450.0             # 위 방향 탐색 한계 (벽 윗면보다 높게. 너비 스윕 z 범위 + 따라가기 한계)

FLOOR_DETECT = True         # 시작할 때 아래로 내려가서 바닥 높이를 직접 잰다
FLOOR_Z_DEFAULT = 50.0      # 바닥 측정 안 할 때 / 실패했을 때 쓸 바닥 높이
FLOOR_SEARCH_START_Z = 100.0  # 바닥 찾기 시작 높이
FLOOR_SEARCH_DEPTH = 100.0    # 여기까지 내려가도 안 닿으면 바닥 측정 실패
FLOOR_CLEARANCE = 5.0       # 바닥에서 이만큼 띄운 높이에서 벽 쪽으로 전진

STEP = 10.0                 # 스윕 간격 [mm]  (알고리즘 4번의 '10 단위')
MISS_LIMIT = 2              # 벽을 찾은 뒤 연속 N 번 벽이 없으면 스윕 종료
TIP_RADIUS = 0.0            # 공구 끝 반지름 [mm] (끝점 보정: 구형 팁이면 반지름 입력)

# =============================================================================
# 속도 / 힘 / 판정
# =============================================================================
MOVE_VELOCITY, MOVE_ACC = 100.0, 200.0
PROBE_VELOCITY, PROBE_ACC = 10.0, 50.0       # 벽/바닥 찾을 때 전진 속도
FOLLOW_VELOCITY, FOLLOW_ACC = 20.0, 50.0     # 벽 따라갈 때 속도

CONTACT_FORCE = 5.0         # 벽/바닥 '막힘' 판정 힘 [N]
FREE_FORCE = 2.0            # 따라가는 중 이 힘보다 작으면 '비어있음'
BLOCK_FORCE = 15.0          # 진행 방향 힘이 이 이상이면 '더 못 감'
# 사고(충돌) 판정 임계값 / 처리는 force_guard.py 에 있음 (툴 과부하, 옆 충돌, 암 충돌, 안전정지 ...)
END_HOLD_SEC = 2.0          # '비어있음/못 감' 이 이만큼 계속되면 벽 끝 (알고리즘 2번)
STALL_MM = 0.5              # 이 거리 이상 전진해야 '진행 중' 으로 봄

PRE_GAP = 5.0               # 따라가기 시작 전 벽 앞에 대기하는 거리
PRESS_DEPTH = 10.0          # (real) 목표점을 벽 안쪽으로 이만큼 → 스프링으로 누름
APPROACH_STIFFNESS = 500.0  # (real) 벽 쪽 축 강성 [N/m] → 0.5N/mm × 10mm ≈ 5N
OTHER_STIFFNESS = 3000.0    # (real) 나머지 축 강성 [N/m]
ROT_STIFFNESS = 200.0       # (real) 회전 강성 [Nm/rad]

# 특이점 (singularity)
#   그리퍼를 45° 눕히면 탐색 영역 한가운데서 손목(J5)이 0° 를 지나간다 (손목 특이점).
#   두산 기본 설정(DR_AVOID)은 특이점 근처에서 '경로를 휘어서' 피하므로 TCP 가 직선에서
#   수 mm 벗어나 벽을 과하게 누르거나 떨어진다 → 측정이 틀어짐.
#   DR_VAR_VEL 은 경로는 그대로 두고 속도만 줄인다 → 측정용으로는 이게 맞다.
SINGULARITY_VAR_VEL = True   # True: 시작 시 DR_VAR_VEL 로 바꾸고, 끝나면 DR_AVOID 로 복구
SING_WARN_DEG = 5.0          # 사전 점검: |J5| 가 이보다 작은 위치가 있으면 경고
VALID_REASONS = ('free', 'blocked')   # 이 이유로 끝난 선만 결과에 사용

SAMPLE_PERIOD = 0.02
PROBE_TIMEOUT = 30.0
FOLLOW_TIMEOUT = 120.0
PATH_MIN_MM = 1.0           # 경로 기록 간격 (이만큼 움직여야 새 점 추가)

SAVE_DIR = os.path.expanduser('~/wall_scan_data')

# =============================================================================
# (virtual 전용) 가상 박스 벽 + 바닥
# =============================================================================
VIRTUAL_BOX = {
    'face': 500.0,          # 벽 표면 APPROACH 좌표
    'thickness': 20.0,      # 벽 두께 20mm
    'lat': (-250.0, 250.0), # 벽 옆 범위 → 정답 너비 500mm
    'floor': 50.0,          # 바닥 높이
    'top': 350.0,           # 벽 윗면 높이 → 정답 높이 300mm
    'stiffness': 20.0,      # [N/mm]
    'noise': 0.2,           # [N]
}

UNIT = np.eye(3)
APP_DIR = UNIT[APPROACH_AXIS] * APPROACH_SIGN


def _tool_dir(ori):
    """ZYZ [a, b, c] (deg) → 공구 Z축 방향 (손가락이 향하는 방향, RViz 그리퍼 표시용)."""
    a, b = np.radians(ori[0]), np.radians(ori[1])
    return np.array([np.cos(a) * np.sin(b), np.sin(a) * np.sin(b), np.cos(b)])


TOOL_DIR = _tool_dir(TOOL_ORI)
UP_DIR = UNIT[UP_AXIS]
LAT_DIR = UNIT[LAT_AXIS]


def make_point(lat, app, up):
    """(옆, 벽쪽, 위) 좌표 → BASE [x, y, z]. app 은 APPROACH_SIGN 이 곱해진 '벽 방향 거리'."""
    p = np.zeros(3)
    p[LAT_AXIS], p[APPROACH_AXIS], p[UP_AXIS] = lat, app * APPROACH_SIGN, up
    return p


def app_of(p):
    """BASE 좌표 → 벽 방향 거리 (클수록 벽 쪽)."""
    return float(p[APPROACH_AXIS]) * APPROACH_SIGN


class VirtualBox:
    """에뮬레이터용 가상 벽(박스) + 바닥. TCP 위치 → 반력 [Fx,Fy,Fz,0,0,0]."""

    def __init__(self, cfg):
        self.c = cfg
        self.rng = np.random.default_rng()

    def force_at(self, pos):
        c = self.c
        f = np.zeros(6)
        lat, a, up = pos[LAT_AXIS], app_of(pos), pos[UP_AXIS]
        pen = a - c['face']
        if (c['lat'][0] <= lat <= c['lat'][1] and c['floor'] <= up <= c['top']
                and 0.0 < pen < c['thickness']):
            f[:3] += -APP_DIR * c['stiffness'] * pen          # 벽이 로봇 쪽으로 밀어냄
        if up < c['floor']:
            f[:3] += UP_DIR * c['stiffness'] * (c['floor'] - up)   # 바닥이 위로 밀어냄
        f[:3] += self.rng.normal(0.0, c['noise'], 3)
        return f


class SizeIO(RobotIO):
    """wall_navigation.RobotIO 재사용 + 공구 자세/가상 벽/컴플라이언스만 바꿈."""

    def __init__(self, node, dsr, mode):
        super().__init__(node, dsr, mode)
        self.box = VirtualBox(VIRTUAL_BOX) if mode == 'virtual' else None

    on_move = None              # move_l 끝날 때 호출 (경로 기록용 콜백)
    guard = None                # force_guard.ForceGuard (있으면 모든 move_l 을 감시)

    def _posx(self, xyz):
        return self.dsr.posx(*[float(v) for v in xyz[:3]], *TOOL_ORI)

    def move_l(self, xyz, vel=MOVE_VELOCITY, acc=MOVE_ACC, expect=()):
        """감시하면서 직선 이동 (끝날 때까지 기다림).
        expect: 이동 중 정상적으로 생길 수 있는 접촉 힘 방향 (예: 벽에서 떨어지는 중 → (APP_DIR,)).
                비워두면 '어떤 접촉도 없어야 하는 이동' → 작은 힘에도 충돌로 처리."""
        if self.guard is None:
            return self.move_l_unguarded(xyz, vel, acc)
        target = np.asarray(xyz, float)[:3]
        start = self.get_pos()
        self.amove_l(target, vel, acc)
        timeout = np.linalg.norm(target - start) / max(vel, 1e-3) * 3.0 + 5.0
        t0 = time.time()
        while True:
            pos = self.get_pos()
            self.guard.check(pos, None, expected=expect, context='move')
            if time.time() - t0 > 0.3 and not self.is_moving():
                break
            if time.time() - t0 > timeout:
                self.stop()
                raise RuntimeError(f'이동 시간 초과 (목표 {np.round(target, 1).tolist()})')
            time.sleep(SAMPLE_PERIOD)
        if self.on_move is not None:
            self.on_move(xyz)

    def move_l_unguarded(self, xyz, vel=MOVE_VELOCITY, acc=MOVE_ACC):
        """감시 없는 직선 이동 (가드가 회복 동작에 사용)."""
        super().move_l(xyz, vel, acc)
        if self.on_move is not None:
            self.on_move(xyz)

    def wait_for_services(self):
        super().wait_for_services()
        d = self.dsr
        ServiceWaiter(self.node).wait(d._ros2_set_digital_output, d._ros2_set_singularity_handling,
                                      d._ros2_ikin, d._ros2_get_current_solution_space)

    def set_singularity(self, var_vel):
        """특이점 처리: True → DR_VAR_VEL (경로 유지, 감속), False → DR_AVOID (기본)."""
        d = self.dsr
        mode = d.DR_VAR_VEL if var_vel else d.DR_AVOID
        ret = d.set_singularity_handling(mode)
        self.node.get_logger().info(
            f'[특이점 처리] {"DR_VAR_VEL (경로 유지)" if var_vel else "DR_AVOID (기본)"}'
            f'{"" if ret == 0 else " - 설정 실패!"}')
        return ret == 0

    def joint_at(self, xyz, sol_space):
        """역기구학: TCP 위치 → 관절각 [deg] (실패/도달 불가면 None)."""
        q = self.dsr.ikin(self._posx(xyz), sol_space, ref=self.dsr.DR_BASE)
        if not isinstance(q, (list, tuple)) or len(q) < 6:
            return None
        return [float(v) for v in q]

    def gripper_close(self):
        """그리퍼를 닫아 손가락 끝을 탐침으로 쓴다."""
        self.node.get_logger().info('[그리퍼] 닫기 → 손가락 끝으로 탐색')
        self.dsr.set_digital_output(*GRIP_DO_OPEN)    # 반대쪽 신호 먼저 끄기
        self.dsr.set_digital_output(*GRIP_DO_CLOSE)
        time.sleep(GRIP_WAIT_SEC)

    def get_force(self, pos=None):
        if self.mode == 'virtual':
            return self.box.force_at(pos if pos is not None else self.get_pos())
        return super().get_force(pos)

    def start_compliance(self):
        """(real) 벽 쪽 축만 부드럽게 → 목표를 벽 안에 두면 스프링처럼 누른다."""
        if self.mode != 'real':
            return
        k = [OTHER_STIFFNESS] * 3 + [ROT_STIFFNESS] * 3
        k[APPROACH_AXIS] = APPROACH_STIFFNESS
        self.dsr.task_compliance_ctrl(k)
        self.compliance_on = True
        time.sleep(0.2)


# =============================================================================
# 측정 로직
# =============================================================================
class WallSizeMeasurer:
    def __init__(self, io, viz=None):
        self.io = io
        self.viz = viz
        self.points = []            # 접촉/따라가기 점 [{'kind','xyz','force'}]
        self.skipped = []           # 툴 충돌로 건너뛴 선
        self.path = []              # TCP 경로 [[x, y, z, phase], ...]
        self.lines = []             # 선 측정 결과 [{'kind','p0','p1','value'}]
        self.face_a = WALL_FACE_GUESS
        self.floor_z = None
        self.results = {}
        io.on_move = lambda xyz: self.trace(xyz, 'move', publish=True)

    def sense(self, pos, expected, context):
        """힘 읽기 + 사고 검사 (가드가 있으면 가드가 읽고 검사, 문제면 예외)."""
        if self.io.guard is None:
            return self.io.get_force(pos)
        return self.io.guard.check(pos, None, expected=expected, context=context)

    def add_point(self, kind, pos, f):
        self.points.append({'kind': kind, 'xyz': [float(v) for v in pos], 'force': float(f)})

    def trace(self, pos, phase, publish=False):
        """TCP 경로에 점 추가 (PATH_MIN_MM 이상 움직였을 때만) + RViz 갱신."""
        pos = [float(v) for v in pos[:3]]
        if not self.path or np.linalg.norm(np.subtract(pos, self.path[-1][:3])) >= PATH_MIN_MM:
            self.path.append(pos + [phase])
        self.update_rviz(force=publish)

    def add_line(self, kind, p0, p1, value):
        self.lines.append({'kind': kind, 'p0': [float(v) for v in p0],
                           'p1': [float(v) for v in p1], 'value': float(value)})
        self.update_rviz(force=True)

    def scene(self, with_tool=True):
        """RViz/저장용 장면 데이터 (형식은 wall_size_rviz.py 참고)."""
        tool = None
        if with_tool and self.path:
            tool = {'pos': self.path[-1][:3], 'dir': TOOL_DIR.tolist()}
        return {
            'axes': {'lat': LAT_AXIS, 'approach': APPROACH_AXIS,
                     'approach_sign': APPROACH_SIGN, 'up': UP_AXIS},
            'path': self.path,
            'points': [{'kind': p['kind'], 'xyz': p['xyz']} for p in self.points],
            'lines': self.lines,
            'floor_z': self.floor_z,
            'face': self.face_a,
            'lat_range': list(LAT_RANGE),
            'tool': tool,
            'truth': VIRTUAL_BOX if self.io.mode == 'virtual' else None,
            'summary': {'total_height': self.results.get('total_height'),
                        'total_width': self.results.get('total_width')},
        }

    def update_rviz(self, force=False):
        """RViz 갱신. force=False (힘 감시 루프 안) → 백그라운드 스레드에 맡김 (루프 지연 X),
        force=True (이동/측정이 끝난 시점) → 바로 퍼블리시."""
        if self.viz is None:
            return
        if not force:
            self.viz.mark_dirty()
            return
        try:
            self.viz.publish(self.scene(), force=True)
        except Exception as e:
            self.io.node.get_logger().warn(f'RViz 퍼블리시 실패: {e}')

    # ------------------------------------------------------------ 가드 이동
    def guarded_move(self, start, direction, dist, axis, vel=PROBE_VELOCITY):
        """start 에서 direction 으로 dist 만큼 천천히 가다가 axis 축 힘이
        CONTACT_FORCE 이상이면 정지. Returns: 접촉 위치 또는 None."""
        start = np.asarray(start, float)
        self.io.move_l(start)
        self.io.zero_force()
        self.io.amove_l(start + direction * dist, vel, PROBE_ACC)
        contact = None
        t0 = time.time()
        while time.time() - t0 < PROBE_TIMEOUT:
            pos = self.io.get_pos()
            self.trace(pos, 'seek')
            f = self.sense(pos, (direction,), 'seek')
            if abs(f[axis]) >= CONTACT_FORCE:
                self.io.stop()
                contact = pos
                break
            if time.time() - t0 > 0.5 and not self.io.is_moving():
                break
            time.sleep(SAMPLE_PERIOD)
        else:
            self.io.stop()
            self.io.node.get_logger().info('  -> 가드 이동 시간 초과')
        self.io.wait_motion_done()
        return contact

    # ------------------------------------------------------------ 바닥
    def detect_floor(self):
        """벽 앞에서 아래로 내려가 바닥 높이를 잰다 ('바닥부터 시작')."""
        if not FLOOR_DETECT:
            self.floor_z = FLOOR_Z_DEFAULT
            self.io.node.get_logger().info(f'[바닥] 측정 생략 → 설정값 {self.floor_z:.1f} 사용')
            return self.floor_z
        start = make_point(LAT_CENTER_GUESS, self.face_a - APPROACH_GAP, FLOOR_SEARCH_START_Z)
        self.io.node.get_logger().info('[바닥] 아래로 내려가며 바닥 찾기')
        c = self.guarded_move(start, -UP_DIR, FLOOR_SEARCH_DEPTH, UP_AXIS)
        self.io.move_l(start, vel=PROBE_VELOCITY * 3, expect=(UP_DIR,))      # 바닥에서 떨어지기
        if c is None:
            self.io.node.get_logger().info(f'  -> 바닥 못 찾음 → 설정값 {FLOOR_Z_DEFAULT:.1f} 사용')
            self.floor_z = FLOOR_Z_DEFAULT
        else:
            self.floor_z = float(c[UP_AXIS])
            self.add_point('floor', c, CONTACT_FORCE)
            self.io.node.get_logger().info(f'  -> 바닥 Z = {self.floor_z:.2f}')
        return self.floor_z

    # ------------------------------------------------------------ 알고리즘 1
    def seek_wall(self, lat, up):
        """(lat, up) 에서 벽 쪽으로 전진 → 막히는 곳 = 벽의 시작점."""
        start = make_point(lat, self.face_a - APPROACH_GAP, up)
        c = self.guarded_move(start, APP_DIR, APPROACH_GAP + SEARCH_DEPTH, APPROACH_AXIS)
        self.io.move_l(start, vel=PROBE_VELOCITY * 3, expect=(APP_DIR,))     # 벽에서 후퇴
        if c is None:
            return None
        self.face_a = app_of(c)                           # 다음 탐색은 측정한 표면 기준
        self.add_point('probe', c, CONTACT_FORCE)
        self.update_rviz(force=True)
        return c

    # ------------------------------------------------------------ 알고리즘 2
    def follow_wall(self, contact, direction, kind):
        """벽을 누른 채 direction 으로 따라가다 '비어있음/못 감' 이 END_HOLD_SEC
        동안 이어지면 정지. Returns: (마지막 접촉 위치, 종료 이유)."""
        contact = np.asarray(contact, float)
        real = self.io.mode == 'real'
        press = PRESS_DEPTH if real else (FREE_FORCE + CONTACT_FORCE) / VIRTUAL_BOX['stiffness']
        press_pt = contact + APP_DIR * press

        # 따라가기 한계 (작업 영역) 까지의 거리
        lim = {LAT_AXIS: LAT_RANGE, UP_AXIS: (self.floor_z, Z_LIMIT)}
        ax = int(np.argmax(np.abs(direction)))
        lo, hi = lim[ax]
        dist = (hi - contact[ax]) if direction[ax] > 0 else (contact[ax] - lo)
        if dist <= STALL_MM:
            return contact, 'limit'

        self.io.move_l(contact - APP_DIR * APPROACH_GAP)
        self.io.move_l(contact - APP_DIR * PRE_GAP, vel=PROBE_VELOCITY * 2)
        self.io.zero_force()

        last_contact, reason = contact.copy(), 'no_contact'
        try:
            self.io.start_compliance()
            # 벽에 붙이기
            self.io.amove_l(press_pt, PROBE_VELOCITY, PROBE_ACC)
            self.io.wait_motion_done()
            t0 = time.time()
            pressed = True
            while abs(self.sense(None, (APP_DIR,), 'press')[APPROACH_AXIS]) < FREE_FORCE:
                if time.time() - t0 > 5.0:
                    pressed = False
                    break
                time.sleep(SAMPLE_PERIOD)
            if pressed:
                last_contact, reason = self._follow_loop(contact, press_pt, direction, dist, ax, kind)
            else:
                # 탐색 전체를 멈추지 않고 이 방향만 '무효' 처리하고 계속
                self.io.node.get_logger().info('  -> 벽에 다시 붙지 못함 → 이 방향은 무효 처리')
        finally:
            self.io.release_press()
            # 가드가 발동했으면 여기서 움직이지 않는다 (회복은 run_line 이 가드로 처리)
            if self.io.guard is None or self.io.guard.tripped is None:
                pos = self.io.get_pos()
                self.io.move_l(pos - APP_DIR * (APPROACH_GAP + press), vel=PROBE_VELOCITY * 3,
                               expect=(APP_DIR, direction))
        self.update_rviz(force=True)
        return last_contact, reason

    def _follow_loop(self, contact, press_pt, direction, dist, ax, kind):
        """벽에 붙은 상태에서 따라가며 감시. Returns: (마지막 접촉 위치, 종료 이유)."""
        last_contact, reason = contact.copy(), 'timeout'
        self.io.amove_l(press_pt + direction * dist, FOLLOW_VELOCITY, FOLLOW_ACC)
        t0 = time.time()
        t_free = t_block = None
        best_s, t_prog = 0.0, t0
        while True:
            now = time.time()
            pos = self.io.get_pos()
            self.trace(pos, 'follow')
            # 벽 쪽(누르는 힘) + 진행 방향(마찰/막힘) 힘은 정상, 그 외 방향/과부하는 가드가 예외
            f = self.sense(pos, (APP_DIR, direction), 'follow')
            fn = abs(f[APPROACH_AXIS])                 # 벽이 밀어내는 힘
            ff = abs(f[ax])                            # 진행 방향을 막는 힘
            s = float(np.dot(pos - press_pt, direction))

            # 비어있음 판정
            if fn >= FREE_FORCE:
                last_contact, t_free = pos, None
                self.add_point(kind, pos, fn)
            elif t_free is None:
                t_free = now

            # 못 감 판정 (막는 힘 또는 진행 정체)
            if s > best_s + STALL_MM:
                best_s, t_prog = s, now
            blocked = ff >= BLOCK_FORCE or (now - t_prog) >= END_HOLD_SEC
            if blocked and t_block is None:
                t_block = now
            elif not blocked:
                t_block = None

            if t_free is not None and now - t_free >= END_HOLD_SEC:
                self.io.stop()
                reason = 'free'
                break
            if t_block is not None and now - t_block >= END_HOLD_SEC:
                self.io.stop()
                reason = 'blocked'
                break
            if now - t0 > 0.5 and not self.io.is_moving():
                reason = 'limit'                       # 한계까지 갔는데 벽이 계속됨
                break
            if now - t0 > FOLLOW_TIMEOUT:
                self.io.stop()
                break
            time.sleep(SAMPLE_PERIOD)
        self.io.wait_motion_done()
        return last_contact, reason

    # ------------------------------------------------------------ 알고리즘 3
    def measure_height_line(self, lat):
        """lat 위치에서 바닥 → 위로 따라가 벽 높이 측정."""
        self.io.node.get_logger().info(f'[높이 측정] x={lat:.1f}')
        c = self.seek_wall(lat, self.floor_z + FLOOR_CLEARANCE)
        if c is None:
            self.io.node.get_logger().info('  -> 벽 없음')
            return None
        top, reason = self.follow_wall(c, UP_DIR, 'scan_v')
        h = float(top[UP_AXIS]) + TIP_RADIUS - self.floor_z
        valid = reason in VALID_REASONS
        if valid:
            bottom = top.copy()
            bottom[UP_AXIS] = self.floor_z
            self.add_line('height', bottom, top, h)
            self.io.node.get_logger().info(
                f'  -> 높이 {h:.1f} mm (끝 z={top[UP_AXIS]:.1f}, 종료={reason})')
        else:
            self.io.node.get_logger().info(f'  -> [무효] 종료={reason} → 결과에서 제외 (측정값 {h:.1f} mm)')
        return {'lat': float(lat), 'start': c.tolist(), 'end': top.tolist(),
                'height': h, 'reason': reason, 'valid': valid}

    def measure_width_line(self, up, center):
        """up 높이에서 벽을 찾은 뒤 +옆 / -옆으로 따라가 너비 측정."""
        self.io.node.get_logger().info(f'[너비 측정] z={up:.1f} (시작 x={center:.1f})')
        c = self.seek_wall(center, up)
        if c is None:
            self.io.node.get_logger().info('  -> 벽 없음')
            return None
        right, r1 = self.follow_wall(c, LAT_DIR, 'scan_h')
        left, r2 = self.follow_wall(c, -LAT_DIR, 'scan_h')
        w = float(right[LAT_AXIS] - left[LAT_AXIS]) + 2 * TIP_RADIUS
        valid = r1 in VALID_REASONS and r2 in VALID_REASONS
        if valid:
            p0, p1 = c.copy(), c.copy()
            p0[LAT_AXIS], p1[LAT_AXIS] = left[LAT_AXIS], right[LAT_AXIS]
            self.add_line('width', p0, p1, w)
            self.io.node.get_logger().info(
                f'  -> 너비 {w:.1f} mm (x {left[LAT_AXIS]:.1f} ~ {right[LAT_AXIS]:.1f}, '
                f'종료={r2}/{r1})')
        else:
            self.io.node.get_logger().info(f'  -> [무효] 종료={r2}/{r1} → 결과에서 제외 (측정값 {w:.1f} mm)')
        return {'up': float(up), 'start': c.tolist(), 'left': float(left[LAT_AXIS]),
                'right': float(right[LAT_AXIS]), 'width': w, 'reason': [r2, r1], 'valid': valid}

    # ------------------------------------------------------------ 툴 충돌 회복
    def run_line(self, fn, *args):
        """선 측정 하나를 실행. 툴 충돌(ToolCollision)이면:
          정지(가드가 이미 함) → 벽 반대쪽으로 후퇴 → 그 선은 건너뜀(None 대신 'skip') → 다음 선 계속.
          연속 MAX_TOOL_COLLISIONS 번이면 전체 중단 (원인이 계속 남아있다는 뜻).
        후퇴했는데도 힘이 안 빠지면(무언가 계속 누름) 바로 전체 중단.
        암 충돌 / 안전정지 / 센서 고장은 여기서 잡지 않고 main 까지 올려서 전체 중단."""
        guard = self.io.guard
        try:
            r = fn(*args)
            if guard is not None and r is not None:
                guard.ok_streak()
            return r
        except ToolCollision as e:
            ev = e.event
            self.io.node.get_logger().info(f'  -> [툴 충돌] {ev.message} → 후퇴 후 이 선은 건너뜀 '
                                           f'(연속 {guard.tool_streak}/{MAX_TOOL_COLLISIONS})')
            if ev.pos:
                self.add_point('collision', ev.pos, ev.value)
            self.skipped.append({'line': fn.__name__, 'args': [float(a) for a in args],
                                 'event': ev.to_dict()})
            self.update_rviz(force=True)
            ok = guard.relieve(-APP_DIR)
            if not ok:
                raise
            if not guard.wait_clear():             # 물러났는데도 계속 눌림 → 원인이 남아있음
                self.io.node.get_logger().info('[툴 충돌] 후퇴 후에도 힘이 남아있음 → 전체 중단')
                raise
            guard.reset()
            if guard.tool_streak >= MAX_TOOL_COLLISIONS:
                self.io.node.get_logger().info(
                    f'[툴 충돌] 연속 {guard.tool_streak}번 → 원인이 남아있는 것 같아 전체 중단')
                raise
            return 'skip'

    # ------------------------------------------------------------ 알고리즘 4, 5
    def sweep_widths(self):
        """z 를 STEP 씩 올리면서 옆(x) 방향 너비 측정 → 총 높이/너비."""
        self.io.node.get_logger().info(f'=== 너비 스윕: z 를 {STEP:.0f}mm 씩 올리며 x 방향 탐색 ===')
        rows, center, misses = [], LAT_CENTER_GUESS, 0
        for up in np.arange(self.floor_z + FLOOR_CLEARANCE, Z_LIMIT + 1e-6, STEP):
            r = self.run_line(self.measure_width_line, float(up), center)
            if r == 'skip':
                continue
            if r is None:
                misses += 1
                if rows and misses >= MISS_LIMIT:
                    break
                continue
            misses = 0
            if r['valid']:
                center = (r['left'] + r['right']) / 2.0     # 다음 줄은 벽 가운데서 시작
            rows.append(r)
        good = [r for r in rows if r['valid']]
        out = {'rows': rows, 'invalid': len(rows) - len(good)}
        if good:
            best = max(good, key=lambda r: r['width'])
            out['total_width'] = best['width']
            out['total_height'] = rows[-1]['up'] - self.floor_z   # 해상도 = STEP (벽에 닿기만 하면 됨)
            out['lat_range'] = [min(r['left'] for r in good), max(r['right'] for r in good)]
            self.io.node.get_logger().info(
                f'[너비 스윕 결과] 총 너비 {out["total_width"]:.1f} mm (z={best["up"]:.0f}), '
                f'총 높이 ≈ {out["total_height"]:.1f} mm (±{STEP:.0f}, 벽이 닿은 마지막 줄 기준), '
                f'무효 {out["invalid"]}/{len(rows)}줄')
        elif rows:
            self.io.node.get_logger().info(f'[너비 스윕 결과] 벽은 찾았지만 유효한 줄이 없음 ({len(rows)}줄 모두 무효)')
        else:
            self.io.node.get_logger().info('[너비 스윕 결과] 벽을 찾지 못함')
        self.results['width_sweep'] = out
        return out

    def sweep_heights(self):
        """x 를 STEP 씩 옮기면서 위(z) 방향 높이 측정 → 총 높이/너비."""
        self.io.node.get_logger().info(f'=== 높이 스윕: x 를 {STEP:.0f}mm 씩 옮기며 z 방향 탐색 ===')
        cols, misses = [], 0
        for lat in np.arange(LAT_RANGE[0], LAT_RANGE[1] + 1e-6, STEP):
            r = self.run_line(self.measure_height_line, float(lat))
            if r == 'skip':
                continue
            if r is None:
                misses += 1
                if cols and misses >= MISS_LIMIT:
                    break
                continue
            misses = 0
            cols.append(r)
        good = [r for r in cols if r['valid']]
        out = {'cols': cols, 'invalid': len(cols) - len(good)}
        if good:
            best = max(good, key=lambda r: r['height'])
            out['total_height'] = best['height']
            out['total_width'] = cols[-1]['lat'] - cols[0]['lat']      # 해상도 = STEP (벽에 닿기만 하면 됨)
            self.io.node.get_logger().info(
                f'[높이 스윕 결과] 총 높이 {out["total_height"]:.1f} mm (x={best["lat"]:.0f}), '
                f'총 너비 ≈ {out["total_width"]:.1f} mm (+최대 {2 * STEP:.0f}, 벽이 닿은 첫/끝 열 기준), '
                f'무효 {out["invalid"]}/{len(cols)}열')
        elif cols:
            self.io.node.get_logger().info(f'[높이 스윕 결과] 벽은 찾았지만 유효한 열이 없음 ({len(cols)}열 모두 무효)')
        else:
            self.io.node.get_logger().info('[높이 스윕 결과] 벽을 찾지 못함')
        self.results['height_sweep'] = out
        return out

    def measure_lines(self, zs, xs=None):
        """몇 줄만 측정 (빠른 측정).
        zs: 너비를 잴 높이들 (바닥 기준 높이 [mm])
        xs: 높이를 잴 옆 위치들 (BASE [mm]). None 이면 너비 결과로 자동 결정
            (벽 가운데, 가운데 ± 너비의 30%)."""
        self.io.node.get_logger().info(
            f'=== 빠른 측정: 너비 {len(zs)}줄 (바닥 위 {zs}), 높이 {"자동 3" if xs is None else len(xs)}줄 ===')
        widths, heights, center = [], [], LAT_CENTER_GUESS
        for dz in zs:
            r = self.run_line(self.measure_width_line, self.floor_z + float(dz), center)
            if isinstance(r, dict):
                widths.append(r)
                if r['valid']:
                    center = (r['left'] + r['right']) / 2.0
        good_w = [r['width'] for r in widths if r['valid']]
        if xs is None:
            if good_w:
                half = 0.3 * float(np.median(good_w))
                xs = [center - half, center, center + half]
            else:
                xs = [center]
            self.io.node.get_logger().info(f'[빠른 측정] 높이 줄 위치 자동: x = {[round(x, 1) for x in xs]}')
        for x in xs:
            r = self.run_line(self.measure_height_line, float(x))
            if isinstance(r, dict):
                heights.append(r)
        good_h = [r['height'] for r in heights if r['valid']]

        def stats(v):
            return None if not v else {'mean': float(np.mean(v)), 'min': float(np.min(v)),
                                       'max': float(np.max(v)), 'std': float(np.std(v)), 'n': len(v)}
        out = {'width_lines': widths, 'height_lines': heights,
               'width_stats': stats(good_w), 'height_stats': stats(good_h)}
        for name, st, tot in (('너비', out['width_stats'], len(widths)), ('높이', out['height_stats'], len(heights))):
            if st:
                self.io.node.get_logger().info(
                    f'[빠른 측정] {name}: 평균 {st["mean"]:.1f} mm '
                    f'(최소 {st["min"]:.1f}, 최대 {st["max"]:.1f}, '
                    f'σ {st["std"]:.2f}, 유효 {st["n"]}/{tot}줄)')
            else:
                self.io.node.get_logger().info(f'[빠른 측정] {name}: 유효한 줄 없음 ({tot}줄)')
        self.results['lines'] = out
        self.results['width_sweep'] = {'total_width': out['width_stats']['mean']} if good_w else {}
        self.results['height_sweep'] = {'total_height': out['height_stats']['mean']} if good_h else {}
        return out

    def check_singularity(self):
        """탐색 영역을 역기구학으로 미리 훑어서 손목 특이점(|J5| 작음) 근처인 위치를 경고."""
        try:
            sol = self.io.dsr.get_current_solution_space()
            if not isinstance(sol, int) or sol < 0:
                self.io.node.get_logger().info('[특이점 점검] 현재 solution space 를 못 읽어서 건너뜀')
                return
            lats = np.linspace(LAT_RANGE[0], LAT_RANGE[1], 5)
            ups = np.arange(self.floor_z + FLOOR_CLEARANCE, Z_LIMIT + 1e-6, 20.0)
            near, unreachable = [], []
            for up in ups:
                for lat in lats:
                    q = self.io.joint_at(make_point(lat, self.face_a, up), sol)
                    if q is None:
                        unreachable.append((lat, up))
                    elif abs(q[4]) < SING_WARN_DEG:
                        near.append((lat, up, q[4]))
            if near:
                zs = sorted({round(u) for _, u, _ in near})
                self.io.node.get_logger().info(f'[특이점 점검] 손목 특이점 근처(|J5|<{SING_WARN_DEG:.0f}°): '
                                               f'z={zs[0]}~{zs[-1]} mm ({len(near)}곳)'
                                               + (' → DR_VAR_VEL 이라 경로는 유지, 그 구간에서 느려짐' if SINGULARITY_VAR_VEL
                                                  else ' → DR_AVOID 라 경로가 휘어 측정이 틀어질 수 있음!'))
            else:
                self.io.node.get_logger().info('[특이점 점검] 탐색 영역에 손목 특이점 없음')
            if unreachable:
                self.io.node.get_logger().info(f'[특이점 점검] 도달 불가/역기구학 실패 {len(unreachable)}곳 '
                                               f'(예: x={unreachable[0][0]:.0f}, z={unreachable[0][1]:.0f}) → 영역 확인')
        except Exception as e:
            self.io.node.get_logger().info(f'[특이점 점검] 실패해서 건너뜀: {e}')

    def summary(self):
        w = self.results.get('width_sweep', {})
        h = self.results.get('height_sweep', {})
        height = h.get('total_height', w.get('total_height'))
        width = w.get('total_width', h.get('total_width'))
        self.io.node.get_logger().info('=================== 최종 결과 ===================')
        self.io.node.get_logger().info(f'  바닥 Z    : {self.floor_z:.1f} mm')
        self.io.node.get_logger().info(
            f'  총 높이   : {height:.1f} mm' if height is not None else '  총 높이   : -')
        self.io.node.get_logger().info(
            f'  총 너비   : {width:.1f} mm' if width is not None else '  총 너비   : -')
        if self.io.mode == 'virtual':
            b = VIRTUAL_BOX
            self.io.node.get_logger().info(
                f'  (정답: 높이 {b["top"] - b["floor"]:.1f}, 너비 {b["lat"][1] - b["lat"][0]:.1f})')
        self.io.node.get_logger().info('=================================================')
        self.results['total_height'] = height
        self.results['total_width'] = width
        self.update_rviz(force=True)

    def save(self):
        os.makedirs(SAVE_DIR, exist_ok=True)
        path = os.path.join(SAVE_DIR, datetime.now().strftime('size_%Y%m%d_%H%M%S.json'))
        data = {
            'mode': self.io.mode, 'robot_model': ROBOT_MODEL,
            'created': datetime.now().isoformat(timespec='seconds'),
            'config': {'step': STEP, 'end_hold_sec': END_HOLD_SEC, 'tilt_deg': TILT_DEG,
                       'probe': 'closed_gripper_tip', 'axes': {
                'lat': LAT_AXIS, 'approach': APPROACH_AXIS, 'approach_sign': APPROACH_SIGN,
                'up': UP_AXIS}, 'tool_ori': TOOL_ORI},
            'virtual_box': VIRTUAL_BOX if self.io.mode == 'virtual' else None,
            'floor_z': self.floor_z,
            'results': self.results,
            'points': self.points,
            'skipped_lines': self.skipped,
            'force_events': [e.to_dict() for e in self.io.guard.events] if self.io.guard else [],
            'scene': self.scene(with_tool=False),     # wall_size_rviz 로 다시 보기
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        self.io.node.get_logger().info(f'결과 저장: {path}')
        return path


# =============================================================================
# 끝난 뒤 홈 복귀 (사고 종류에 따라 다르게)
# =============================================================================
def go_home_after_stop(io, guard, policy):
    """policy='auto'   : 벽에서 떨어진 뒤 홈으로.
       policy='confirm': 팔 충돌/안전정지/센서 고장 뒤 → 자동으로 움직이지 않는다.
                         real 은 작업자가 주변/상태를 확인하고 y 를 입력해야 이동.
                         virtual 은 로그만 남기고 이동 (테스트 편의)."""
    guard.reset()
    if policy == 'confirm':
        if io.mode == 'real':
            ans = input('[확인] 사고로 멈췄습니다. 주변이 안전하고 정지가 해제됐으면 y (홈 복귀), '
                        '그 외 입력은 그대로 둡니다: ')
            if ans.strip().lower() != 'y':
                io.node.get_logger().info('홈 복귀 안 함 (현재 위치 유지)')
                return
        else:
            io.node.get_logger().info('[virtual] 사고 후지만 테스트이므로 홈 복귀 진행')
        try:
            state = io.dsr.get_robot_state()
            if state in BAD_STATES:
                io.node.get_logger().info(
                    f'로봇이 아직 정지 상태(state={state}) → 홈 복귀 불가. 펜던트에서 해제 후 직접 이동하세요')
                return
        except Exception:
            pass
    try:
        pos = io.get_pos()
        io.move_l(pos - APP_DIR * APPROACH_GAP, expect=(APP_DIR,))   # 벽에서 떨어진 뒤
        io.move_j(HOME_J)
    except ForceGuardError as e:
        io.node.get_logger().info(f'홈 복귀 중 다시 사고 감지 → 정지: {e}')
    except Exception as e:
        io.node.get_logger().info(f'홈 복귀 실패: {e}')


# =============================================================================
# main
# =============================================================================
def main(args=None):
    global STEP, WALL_FACE_GUESS, LAT_RANGE, Z_LIMIT, LAT_CENTER_GUESS
    rclpy.init(args=args)
    node = create_dsr_node('wall_size', ROBOT_ID, ROBOT_MODEL)

    mode = node.declare_parameter('mode', 'virtual').value
    scan = node.declare_parameter('scan', 'both').value     # both|width|height|line_v|line_h|lines
    # 벽 위치 / 탐색 범위 (real 에서 실제 벽에 맞게 지정)
    WALL_FACE_GUESS = float(node.declare_parameter('face', WALL_FACE_GUESS).value)
    LAT_RANGE = (float(node.declare_parameter('lat_min', LAT_RANGE[0]).value),
                 float(node.declare_parameter('lat_max', LAT_RANGE[1]).value))
    Z_LIMIT = float(node.declare_parameter('z_limit', Z_LIMIT).value)
    LAT_CENTER_GUESS = float(node.declare_parameter('center', LAT_CENTER_GUESS).value)
    # scan:=lines 용: zs = 너비 잴 높이들(바닥 기준), xs = 높이 잴 x 들 (비우면 자동)
    zs_s = str(node.declare_parameter('zs', '50,150,250').value)
    xs_s = str(node.declare_parameter('xs', '').value)
    STEP = float(node.declare_parameter('step', STEP).value)
    lat = float(node.declare_parameter('x', LAT_CENTER_GUESS).value)
    try:
        zs = [float(v) for v in zs_s.strip('[] ').split(',') if v.strip()]
        xs = [float(v) for v in xs_s.strip('[] ').split(',') if v.strip()] or None
    except ValueError:
        node.get_logger().error(f'zs/xs 는 "50,150,250" 형식: zs={zs_s}, xs={xs_s}')
        rclpy.shutdown()
        return
    up = float(node.declare_parameter('z', -1.0).value)      # line_h 높이 (-1 = 바닥+여유)
    use_rviz = node.declare_parameter('rviz', True).value
    frame_id = node.declare_parameter('frame_id', 'base_link').value
    inject = node.declare_parameter('inject', 'none').value               # (virtual) 사고 주입 테스트
    inject_after = float(node.declare_parameter('inject_after', 20.0).value)
    inject_duration = float(node.declare_parameter('inject_duration', 1.5).value)
    if mode == 'real' and inject != 'none':
        node.get_logger().error('inject 는 virtual 모드에서만 쓸 수 있습니다')
        rclpy.shutdown()
        return
    if mode not in ('virtual', 'real') or scan not in ('both', 'width', 'height', 'line_v', 'line_h', 'lines'):
        node.get_logger().error(f'잘못된 파라미터: mode={mode}, scan={scan}')
        rclpy.shutdown()
        return

    try:
        import DSR_ROBOT2 as dsr
    except ImportError as e:
        node.get_logger().info(f'Error importing DSR_ROBOT2 : {e}')
        return

    io = SizeIO(node, dsr, mode)
    try:
        injector = Injector(inject, inject_after, inject_duration, side_axis=LAT_AXIS)
    except ValueError as e:
        node.get_logger().error(str(e))
        rclpy.shutdown()
        return
    guard = ForceGuard(io, injector)
    viz = WallSizeViz(node, topic='wall_size_markers', frame_id=frame_id) if use_rviz else None
    if viz is not None:
        node.get_logger().info(
            f'RViz 마커: {node.resolve_topic_name("wall_size_markers")} (Fixed Frame: {frame_id})')
    m = WallSizeMeasurer(io, viz)
    if viz is not None:
        viz.start_background(m.scene)

    io.wait_for_services()
    guard.wait_for_services()
    if inject != 'none':
        node.get_logger().info(
            f'[가드] 테스트: {inject_after:.0f}초 뒤 \'{inject}\' 사고를 {inject_duration:.1f}초 동안 주입')
    if mode == 'real':
        ans = input('[REAL] 실제 로봇이 벽/바닥 쪽으로 움직입니다. 주변 확인 후 y 입력: ')
        if ans.strip().lower() != 'y':
            node.get_logger().info('취소했습니다.')
            safe_shutdown(node)
            return

    home_policy = 'auto'      # auto: 바로 홈 복귀 / confirm: 작업자 확인 후 (팔 충돌, 안전정지 등)
    try:
        io.setup()
        if viz is not None:
            viz.clear()
        if SINGULARITY_VAR_VEL:
            io.set_singularity(True)
        io.gripper_close()
        io.move_j(READY_J)
        node.get_logger().info(f'[자세] 그리퍼 {TILT_DEG:.0f}° 기울임, TOOL_ORI={TOOL_ORI}')
        guard.zero()                     # 접촉 없는 정지 상태에서 관절 토크 영점
        io.guard = guard                 # 이제부터 모든 직선 이동/탐색을 감시
        m.detect_floor()
        m.check_singularity()
        if scan == 'line_v':
            m.results['line_v'] = m.run_line(m.measure_height_line, lat)
        elif scan == 'line_h':
            z = up if up >= 0 else m.floor_z + FLOOR_CLEARANCE
            m.results['line_h'] = m.run_line(m.measure_width_line, z, lat)
        elif scan == 'lines':
            m.measure_lines(zs, xs)
            m.summary()
        else:
            if scan in ('both', 'width'):
                m.sweep_widths()
            if scan in ('both', 'height'):
                m.sweep_heights()
            m.summary()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 (Ctrl+C)')
        io.stop()
    except ToolCollision as e:
        # 툴 충돌이 연속 → 전체 중단. 로봇 자체는 움직일 수 있는 상태
        node.get_logger().error(f'[중단] 툴 충돌 반복: {e}')
    except ArmCollision as e:
        node.get_logger().error(f'[중단] 팔 몸통 충돌: {e}')
        home_policy = 'confirm'
    except ControllerStop as e:
        node.get_logger().error(f'[중단] 컨트롤러 정지: {e}')
        node.get_logger().error('  → 티칭 펜던트에서 원인 확인 후 정지 해제(Reset) 하세요')
        home_policy = 'confirm'
    except SensorFault as e:
        node.get_logger().error(f'[중단] 센서 고장: {e}')
        home_policy = 'confirm'
    except Exception as e:
        node.get_logger().error(f'에러: {e}')
        io.stop()
    finally:
        def save_result():
            if m.points or guard.events:
                m.save()
            if guard.events:
                node.get_logger().info(
                    f'[가드] 이번 실행 사고 기록 {len(guard.events)}건: '
                    + ', '.join(f'{ev.kind}({ev.context})' for ev in guard.events))

        def restore_singularity():
            if SINGULARITY_VAR_VEL:
                io.set_singularity(False)          # 다른 프로그램을 위해 기본값으로 복구

        def stop_viz():
            if viz is not None:
                viz.stop_background()
                m.update_rviz(force=True)        # 마지막 상태 (TRANSIENT_LOCAL 로 남음)

        safe_shutdown(node, ('컴플라이언스 해제 실패', io.release_press),
                      ('결과 저장 실패', save_result),
                      ('홈 복귀 실패', lambda: go_home_after_stop(io, guard, home_policy)),
                      ('특이점 처리 복구 실패', restore_singularity),
                      ('RViz 정리 실패', stop_viz))


if __name__ == '__main__':
    main()
