#!/usr/bin/env python3
"""두산 M0609 로 '벽면을 손으로 더듬듯이' 탐색하는 노드 (가상 / 실제 모드 지원).

[전체 흐름]  (하이브리드 벽 탐색)
  STEP 1. 프로빙(Probing)      : 격자 위치마다 벽 쪽(+X)으로 천천히 전진하다가
                                 힘이 임계값을 넘으면 정지 → 그 위치 = 벽 접촉점
  STEP 2. 평면 산출            : 접촉점들로 벽 평면 / 법선 벡터 계산 (wall_modeling.py)
  STEP 3. 슬라이딩 스캔        : 벽을 일정한 힘으로 누른 채 미끄러지며
                                 벽 표면 위치를 연속으로 기록
                                 - 가로 스캔(scan_h): 높이 z 고정, Y 방향 이동
                                 - 세로 스캔(scan_v): 위치 y 고정, Z 방향 이동
                                 → 가로+세로 격자로 덮어서 좌우/앞뒤 기울기와
                                   휘어짐을 모두 정확하게 잡는다
                                 (탐색 중 RViz 의 /dsr01/wall_markers 로 실시간 표시)
  STEP 4. 저장 + 모델링        : 모든 점을 ~/wall_scan_data/scan_*.json 으로 저장하고
                                 wall_modeling 으로 평면/평탄도/곡면 모델 + 그림 생성

[모드 차이]
  ┌──────────┬──────────────────────────────┬───────────────────────────────┐
  │          │ virtual (에뮬레이터)          │ real (실제 로봇)               │
  ├──────────┼──────────────────────────────┼───────────────────────────────┤
  │ 힘 값     │ 코드 안의 '가상 벽'으로 계산   │ get_tool_force() 실측값        │
  │ 접촉 정지 │ 가상 힘 ≥ 임계값 → 정지        │ 실측 힘 ≥ 임계값 → 정지         │
  │ 슬라이딩  │ 가상 벽 표면을 따라 movel     │ 힘 제어(set_desired_force) +   │
  │          │                              │ amovel 로 실제로 눌러가며 이동  │
  │ Tool/TCP │ 설정 생략 (에뮬레이터에 없음)  │ set_tool / set_tcp 적용         │
  └──────────┴──────────────────────────────┴───────────────────────────────┘
  ※ 에뮬레이터는 외력이 항상 0 이라 진짜 접촉을 못 느끼므로, 가상 벽을 만들어서
    '그 위치를 지나가면 힘이 생기는 것처럼' 흉내낸다. 탐색 로직(전진→감지→정지→후퇴)은
    두 모드가 완전히 같다.

[실행 방법]
  # 1) 로봇 bringup (터미널 1)
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.1.100 port:=12345 model:=m0609

  # 2) 탐색 실행 (터미널 2) - bringup 과 같은 mode 로!
  ros2 run rokey1 wall_navigation --ros-args -p mode:=virtual
  ros2 run rokey1 wall_navigation --ros-args -p mode:=real

  # 3) 저장된 데이터로 모델링만 다시 하고 싶을 때 (--rviz: RViz 에도 표시)
  ros2 run rokey1 wall_modeling
  ros2 run rokey1 wall_modeling --rviz

[RViz 에서 보기]
  Fixed Frame = base_link,  Add → By topic → /dsr01/wall_markers (MarkerArray)

[실제 로봇 전 체크리스트]
  - WALL_GUESS_X : 실제 벽 위치보다 '조금 앞'으로 대략 맞출 것 (줄자로 측정)
  - GRID_Y/GRID_Z/SCAN_* : 벽 범위 안쪽인지, 로봇 작업 영역 안인지
  - READY_J 자세에서 공구가 +X(벽 방향)를 바라보는지 (티칭 펜던트로 확인)
  - 처음엔 PROBE_VEL 을 낮게, FORCE_THRESHOLD 를 작게 (공구/벽 보호)
"""
import json
import os
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
import DR_init

# 같은 폴더의 모델링 모듈 (ros2 run 으로 실행할 때 / python3 로 직접 실행할 때 모두 지원)
try:
    from navigation.wall_modeling import fit_plane, model_from_file
    from navigation.wall_rviz import WallMarkerPublisher
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from wall_modeling import fit_plane, model_from_file
    from wall_rviz import WallMarkerPublisher

# =============================================================================
# 로봇 설정
# =============================================================================
ROBOT_ID = 'dsr01'            # bringup 의 name 과 같아야 함
ROBOT_MODEL = 'm0609'
TOOL_NAME = 'Tool Weight'     # (real 전용) 티칭 펜던트에 등록된 Tool 이름
TCP_NAME = 'GripperDA_v1'     # (real 전용) 티칭 펜던트에 등록된 TCP 이름

# =============================================================================
# 탐색 영역 설정  (단위: mm, 도 / 좌표는 모두 로봇 BASE 기준)
# =============================================================================
#   로봇 정면(+X)에 벽이 Y-Z 평면처럼 서 있다고 가정한다.
#
#        Z(위)
#        │   ┌───────── 벽 ─────────┐
#        │   │  •      •      •    │  ← GRID_Z[2]
#        │   │  •      •      •    │  ← GRID_Z[1]       • = 프로빙 지점
#        │   │  •      •      •    │  ← GRID_Z[0]
#        │   └──────────────────────┘
#        └──────────────────────────── Y
#
WALL_GUESS_X = 550.0          # 벽이 있을 것이라 '예상'하는 X 위치
APPROACH_GAP = 60.0           # 예상 위치보다 이만큼 앞에서 전진 시작 (안전 거리)
SEARCH_DEPTH = 60.0           # 예상 위치보다 이만큼 더 들어가도 접촉이 없으면 '벽 없음'
TOOL_ORI = [0.0, 90.0, 0.0]   # 공구 자세 [rx, ry, rz] (ZYZ) → 공구 Z축이 +X(벽)를 향함
READY_J = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]   # 탐색 시작 전 관절 자세 (공구가 앞을 봄)
HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   # 끝나고 돌아갈 홈 자세

GRID_Y = [-150.0, 0.0, 150.0]       # 프로빙 격자 Y 위치들
GRID_Z = [300.0, 400.0, 500.0]      # 프로빙 격자 Z 위치들

#   슬라이딩 스캔 경로 (가로 줄 + 세로 줄 = 격자)
#        ┌──────────────────────┐
#        │    │      │      │   │
#        │────┼──────┼──────┼───│ ← SCAN_ROWS_Z[1]  (가로: Y 방향)
#        │    │      │      │   │
#        │────┼──────┼──────┼───│ ← SCAN_ROWS_Z[0]
#        │    │      │      │   │
#        └──────────────────────┘
#             ↑      ↑      ↑  SCAN_COLS_Y (세로: Z 방향)
SCAN_ROWS_Z = [350.0, 450.0]        # 가로 스캔할 높이(줄)들
SCAN_Y_RANGE = (-150.0, 150.0)      # 가로 스캔 Y 시작 → 끝
SCAN_COLS_Y = [-100.0, 0.0, 100.0]  # 세로 스캔할 Y 위치(열)들
SCAN_Z_RANGE = (300.0, 500.0)       # 세로 스캔 Z 시작 → 끝 (아래 → 위)
SCAN_STEP = 10.0                    # (virtual) 슬라이딩 경로 간격
RVIZ_EVERY = 10                     # (real) 슬라이딩 중 N 개 점마다 RViz 갱신

# =============================================================================
# 속도 / 힘 설정
# =============================================================================
MOVE_VEL, MOVE_ACC = 100.0, 200.0   # 일반 이동 속도 [mm/s], 가속도 [mm/s^2]
PROBE_VEL, PROBE_ACC = 10.0, 50.0   # 벽 쪽으로 '더듬는' 속도 (느릴수록 정확/안전)
SCAN_VEL, SCAN_ACC = 20.0, 50.0     # 슬라이딩 속도
JOINT_VEL, JOINT_ACC = 30.0, 30.0   # 관절 이동 속도 [deg/s]

FORCE_THRESHOLD = 5.0               # 접촉 판정 힘 [N] (이 이상이면 '닿았다')
SCAN_FORCE = 8.0                    # 슬라이딩 때 벽을 누르는 힘 [N]
SCAN_STIFFNESS = [500.0, 3000.0, 3000.0, 200.0, 200.0, 200.0]  # (real) X 만 부드럽게
SAMPLE_PERIOD = 0.02                # 센서 확인 주기 [s] (50Hz)
PROBE_TIMEOUT = 30.0                # 한 점 프로빙 최대 시간 [s]

SAVE_DIR = os.path.expanduser('~/wall_scan_data')

# =============================================================================
# (virtual 전용) 가상 벽 - 에뮬레이터에서 '진짜 벽'처럼 쓰는 모델
# =============================================================================
#   벽 표면 X 위치 = x0 + tilt_y*y + tilt_z*z + (가운데 볼록한 혹)
#   → 일부러 살짝 기울이고 혹을 넣어서, 모델링이 이를 잘 찾는지 확인할 수 있다.
VIRTUAL_WALL = {
    'x0': 550.0,          # 기준 위치
    'tilt_y': 0.03,       # Y 로 1mm 갈 때 X 가 0.03mm 변함 (좌우 기울기 ≈ 1.7°)
    'tilt_z': -0.02,      # Z 로 1mm 갈 때 X 가 -0.02mm 변함 (앞뒤 기울기 ≈ 1.1°)
    'bump_height': 3.0,   # 혹 높이 [mm] (+ 면 로봇 쪽으로 튀어나옴)
    'bump_center': [0.0, 400.0],   # 혹 중심 [y, z]
    'bump_sigma': 60.0,   # 혹 크기 [mm]
    'stiffness': 20.0,    # 벽 강성 [N/mm] (1mm 파고들면 20N → 단단한 벽 흉내)
    'noise': 0.2,         # 힘 센서 노이즈 [N]
}


class VirtualWall:
    """에뮬레이터용 가상 벽. TCP 위치를 주면 '벽이 밀어내는 힘'을 계산해 준다."""

    def __init__(self, cfg):
        self.cfg = cfg
        self.rng = np.random.default_rng()

    def surface_x(self, y, z):
        """(y, z) 위치에서 벽 표면의 X 좌표."""
        c = self.cfg
        by, bz = c['bump_center']
        bump = c['bump_height'] * np.exp(-((y - by) ** 2 + (z - bz) ** 2) / (2 * c['bump_sigma'] ** 2))
        # 혹은 로봇 쪽(-X)으로 튀어나오므로 빼준다
        return c['x0'] + c['tilt_y'] * y + c['tilt_z'] * z - bump

    def force_at(self, pos):
        """TCP 가 벽 속으로 파고든 깊이 × 강성 = 힘. [Fx, Fy, Fz, Mx, My, Mz] 반환."""
        x, y, z = pos[:3]
        penetration = x - self.surface_x(y, z)      # + 이면 벽 안으로 들어간 것
        fx = -self.cfg['stiffness'] * max(0.0, penetration)  # 벽은 -X 로 밀어냄
        noise = self.rng.normal(0.0, self.cfg['noise'], 3)
        return [fx + noise[0], noise[1], noise[2], 0.0, 0.0, 0.0]


# =============================================================================
# 로봇 입출력 래퍼 - 모드(virtual/real) 차이를 이 클래스 안에 모두 숨긴다
# =============================================================================
class RobotIO:
    """DSR_ROBOT2 함수를 감싸서, 탐색 로직이 모드를 신경쓰지 않게 해 준다."""

    def __init__(self, node, dsr, mode):
        self.node = node
        self.dsr = dsr                      # DSR_ROBOT2 모듈
        self.mode = mode                    # 'virtual' or 'real'
        self.virtual_wall = VirtualWall(VIRTUAL_WALL) if mode == 'virtual' else None
        self.force_bias = np.zeros(6)       # (real) 힘 센서 영점
        self.force_on = False               # 힘 제어 켜짐 여부 (finally 에서 해제용)
        self.compliance_on = False

        # DSR_ROBOT2 에는 '정지(stop)' 함수가 없어서 서비스를 직접 만든다
        from dsr_msgs2.srv import MoveStop
        self._MoveStop = MoveStop
        self.stop_cli = node.create_client(MoveStop, 'dsr_controller2/motion/move_stop')

    # ---------------------------------------------------------------- 기본
    def log(self, msg):
        self.node.get_logger().info(msg)

    def wait_for_services(self):
        """사용할 서비스가 모두 뜰 때까지 대기 (안 하면 첫 호출에서 멈출 수 있음)."""
        d = self.dsr
        clients = [d._ros2_set_robot_mode, d._ros2_movej, d._ros2_movel,
                   d._ros2_get_current_posx, d._ros2_check_motion, self.stop_cli]
        if self.mode == 'real':
            clients += [d._ros2_set_current_tool, d._ros2_set_current_tcp,
                        d._ros2_get_tool_force, d._ros2_set_ref_coord,
                        d._ros2_task_compliance_ctrl, d._ros2_set_desired_force,
                        d._ros2_release_force, d._ros2_release_compliance_ctrl]
        for cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                self.log(f'서비스 대기 중: {cli.srv_name}')

    def setup(self):
        """로봇 모드/공구 설정."""
        d = self.dsr
        d.set_robot_mode(d.ROBOT_MODE_AUTONOMOUS)
        if self.mode == 'real':
            # 에뮬레이터에는 등록된 Tool/TCP 가 없을 수 있어서 real 에서만 설정
            d.set_tool(TOOL_NAME)
            d.set_tcp(TCP_NAME)
            d.set_ref_coord(d.DR_BASE)      # 힘 제어 기준 좌표계 = BASE

    # ---------------------------------------------------------------- 이동
    def move_j(self, q):
        self._check(self.dsr.movej(self.dsr.posj(*q), vel=JOINT_VEL, acc=JOINT_ACC), 'movej')

    def move_l(self, xyz, vel=MOVE_VEL, acc=MOVE_ACC):
        """직선 이동 (끝날 때까지 기다림). xyz 만 주면 자세는 TOOL_ORI 사용."""
        self._check(self.dsr.movel(self._posx(xyz), vel=[vel, 30.0], acc=[acc, 60.0],
                                   ref=self.dsr.DR_BASE), 'movel')

    def amove_l(self, xyz, vel, acc):
        """직선 이동 (명령만 보내고 바로 리턴 → 이동 중에 센서를 볼 수 있음)."""
        self._check(self.dsr.amovel(self._posx(xyz), vel=[vel, 30.0], acc=[acc, 60.0],
                                    ref=self.dsr.DR_BASE), 'amovel')

    def stop(self):
        """즉시 정지 (Quick stop, 정지 카테고리 2)."""
        req = self._MoveStop.Request()
        req.stop_mode = self.dsr.DR_QSTOP
        future = self.stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)

    def is_moving(self):
        return self.dsr.check_motion() != self.dsr.DR_STATE_IDLE

    def wait_motion_done(self, timeout=10.0):
        t0 = time.time()
        while self.is_moving() and time.time() - t0 < timeout:
            time.sleep(SAMPLE_PERIOD)

    # ---------------------------------------------------------------- 센서
    def get_pos(self):
        """현재 TCP 위치 [x, y, z] (BASE, mm)."""
        posx, _ = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        return np.array(posx[:3], dtype=float)

    def get_force(self, pos=None):
        """현재 외력 [Fx, Fy, Fz, Mx, My, Mz].
        virtual: 가상 벽으로 계산 / real: 실측값 - 영점."""
        if self.mode == 'virtual':
            return np.array(self.virtual_wall.force_at(pos if pos is not None else self.get_pos()))
        f = self.dsr.get_tool_force(ref=self.dsr.DR_BASE)
        if not isinstance(f, list):     # 서비스 실패 시 -1 이 돌아옴
            raise RuntimeError('get_tool_force 실패')
        return np.array(f, dtype=float) - self.force_bias

    def zero_force(self, samples=10):
        """(real) 접촉 없는 상태에서 여러 번 읽어 평균을 영점으로 저장.
        공구 무게/센서 오프셋 때문에 가만히 있어도 힘이 0 이 아니기 때문."""
        if self.mode != 'real':
            return
        self.force_bias = np.zeros(6)
        vals = [self.get_force() for _ in range(samples)]
        self.force_bias = np.mean(vals, axis=0)

    # ---------------------------------------------------------------- 힘 제어 (real)
    def start_press(self, force):
        """BASE +X 방향으로 force[N] 만큼 누르는 힘 제어 시작."""
        d = self.dsr
        d.task_compliance_ctrl(SCAN_STIFFNESS)
        self.compliance_on = True
        time.sleep(0.2)
        d.set_desired_force([force, 0, 0, 0, 0, 0], [1, 0, 0, 0, 0, 0], mod=d.DR_FC_MOD_REL)
        self.force_on = True

    def release_press(self):
        """힘 제어/컴플라이언스 해제 (켜져 있을 때만)."""
        if self.force_on:
            self.dsr.release_force()
            self.force_on = False
        if self.compliance_on:
            self.dsr.release_compliance_ctrl()
            self.compliance_on = False

    # ---------------------------------------------------------------- 내부
    def _posx(self, xyz):
        return self.dsr.posx(*[float(v) for v in xyz[:3]], *TOOL_ORI)

    @staticmethod
    def _check(ret, name):
        """DSR 함수는 실패하면 음수를 돌려준다 → 예외로 바꿔서 바로 멈추게 함."""
        if isinstance(ret, (int, float)) and ret < 0:
            raise RuntimeError(f'{name} 실패 (ret={ret})')


# =============================================================================
# 벽 탐색 로직 (모드와 무관)
# =============================================================================
class HybridWallExplorer:
    def __init__(self, io, viz=None):
        self.io = io
        self.viz = viz          # WallMarkerPublisher (None 이면 RViz 표시 안 함)
        self.plane = None       # STEP 2 에서 계산한 평면 (RViz 표시용)
        # [{'kind': 'probe'|'scan_h'|'scan_v', 'xyz': [x,y,z], 'force': N}, ...]
        self.points = []

    def log(self, msg):
        self.io.log(msg)

    def update_rviz(self, model=None):
        """지금까지 모은 점 (+ 평면/모델) 을 RViz 로 보낸다. 실패해도 탐색은 계속."""
        if self.viz is None or not self.points:
            return
        try:
            self.viz.publish([p['xyz'] for p in self.points],
                             [p['kind'] for p in self.points],
                             plane=self.plane, model=model)
        except Exception as e:
            self.io.node.get_logger().warn(f'RViz 퍼블리시 실패: {e}')

    # ------------------------------------------------------------ STEP 1
    def probe_point(self, y, z, force_threshold=FORCE_THRESHOLD):
        """(y, z) 위치에서 +X 로 천천히 전진하다 벽에 닿으면 멈추고 접촉점을 반환.

        1) 벽 앞 안전 위치(start)로 이동
        2) 벽 너머(end)까지 '비동기'로 전진 명령
        3) 이동하는 동안 50Hz 로 힘을 확인 → 임계값 넘으면 즉시 정지
        4) start 로 후퇴
        Returns: 접촉점 [x, y, z] 또는 None (끝까지 가도 안 닿음)
        """
        start = [WALL_GUESS_X - APPROACH_GAP, y, z]
        end = [WALL_GUESS_X + SEARCH_DEPTH, y, z]
        self.log(f'[프로빙] y={y:.0f}, z={z:.0f} 탐색 시작')

        self.io.move_l(start)
        self.io.zero_force()                  # 닿기 전에 영점 잡기
        self.io.amove_l(end, PROBE_VEL, PROBE_ACC)

        contact, f_norm = None, 0.0
        t0 = time.time()
        while time.time() - t0 < PROBE_TIMEOUT:
            pos = self.io.get_pos()
            f_norm = float(np.linalg.norm(self.io.get_force(pos)[:3]))  # 힘의 크기
            if f_norm >= force_threshold:
                self.io.stop()                # 닿았다! 바로 정지
                contact = pos
                break
            # 출발 직후 0.5초는 아직 '정지 상태'로 보일 수 있어서 건너뜀
            if time.time() - t0 > 0.5 and not self.io.is_moving():
                break                         # end 까지 갔는데도 안 닿음
            time.sleep(SAMPLE_PERIOD)
        else:
            self.io.stop()
            self.log('  -> 시간 초과로 정지')

        self.io.wait_motion_done()
        self.io.move_l(start, vel=PROBE_VEL * 3)   # 벽에서 떨어지기 (후퇴)

        if contact is None:
            self.log('  -> 접촉 없음 (WALL_GUESS_X / SEARCH_DEPTH 확인)')
            return None
        self.log(f'  -> 접촉! 위치 [{contact[0]:.2f}, {contact[1]:.2f}, {contact[2]:.2f}], 힘 {f_norm:.2f}N')
        self.points.append({'kind': 'probe', 'xyz': contact.tolist(), 'force': f_norm})
        self.update_rviz()
        return contact

    # ------------------------------------------------------------ STEP 2
    def calculate_plane(self):
        """프로빙 점들로 평면/법선 계산 (자세한 모델링은 끝나고 wall_modeling 에서)."""
        pts = [p['xyz'] for p in self.points if p['kind'] == 'probe']
        if len(pts) < 3:
            raise RuntimeError(f'접촉점이 {len(pts)}개뿐이라 평면을 만들 수 없습니다')
        plane = fit_plane(pts)
        n = plane['normal']
        self.log('=== 평면 계산 완료 ===')
        self.log(f'  법선 벡터: [{n[0]:.4f}, {n[1]:.4f}, {n[2]:.4f}]')
        self.log(f'  평면식  : {n[0]:.4f}x + {n[1]:.4f}y + {n[2]:.4f}z + {plane["d"]:.2f} = 0')
        self.log(f'  평탄도  : RMS {plane["rms"]:.3f} mm')
        self.plane = plane
        self.update_rviz()
        return plane

    def wall_x_on_plane(self, plane, y, z):
        """평면식에서 (y, z) 위치의 벽 X 좌표를 계산 (슬라이딩 시작점 잡을 때 사용)."""
        a, b, c = plane['normal']
        return -(b * y + c * z + plane['d']) / a

    # ------------------------------------------------------------ STEP 3
    def sliding_scan(self, plane, start_yz, end_yz, kind):
        """벽을 누르며 (y, z) 시작점 → 끝점까지 직선으로 미끄러지면서 표면 위치를 기록.

        Args:
            start_yz, end_yz: 벽 위 시작/끝 위치 (y, z) [mm]
            kind: 'scan_h' (가로, Y 방향) 또는 'scan_v' (세로, Z 방향)
        """
        (y0, z0), (y1, z1) = start_yz, end_yz
        name = '가로' if kind == 'scan_h' else '세로'
        self.log(f'[슬라이딩-{name}] ({y0:.0f}, {z0:.0f}) → ({y1:.0f}, {z1:.0f})')
        if self.io.mode == 'virtual':
            n = self._sliding_virtual(start_yz, end_yz, kind)
        else:
            n = self._sliding_real(plane, start_yz, end_yz, kind)
        self.update_rviz()
        self.log(f'  -> {n}개 점 기록')

    def _sliding_virtual(self, start_yz, end_yz, kind):
        """(virtual) 가상 벽 표면을 SCAN_FORCE 만큼 파고든 경로로 movel 하며 샘플링.
        에뮬레이터는 힘 제어로 실제 벽을 따라갈 수 없으므로, '힘 제어가 됐다면
        TCP 가 지나갔을 경로'를 직접 계산해서 따라간다."""
        wall = self.io.virtual_wall
        press = SCAN_FORCE / VIRTUAL_WALL['stiffness']        # 누르는 힘만큼 파고드는 깊이
        p0, p1 = np.asarray(start_yz, float), np.asarray(end_yz, float)
        length = float(np.linalg.norm(p1 - p0))
        steps = max(int(round(length / SCAN_STEP)), 1)

        self.io.move_l([wall.surface_x(*p0) - APPROACH_GAP, *p0])
        count = 0
        for t in np.linspace(0.0, 1.0, steps + 1):
            y, z = p0 + t * (p1 - p0)
            target = [wall.surface_x(y, z) + press, y, z]
            self.io.move_l(target, vel=SCAN_VEL, acc=SCAN_ACC)
            pos = self.io.get_pos()
            f = float(np.linalg.norm(self.io.get_force(pos)[:3]))
            # real 과 똑같이 '측정된 TCP 위치'를 그대로 저장 (벽이 단단해 눌린 깊이는 0.4mm 수준)
            self.points.append({'kind': kind, 'xyz': pos.tolist(), 'force': f})
            count += 1
            self.update_rviz()                                # RViz 실시간 표시
        pos = self.io.get_pos()
        self.io.move_l([pos[0] - APPROACH_GAP, pos[1], pos[2]])   # 후퇴
        return count

    def _sliding_real(self, plane, start_yz, end_yz, kind):
        """(real) 힘 제어로 벽을 SCAN_FORCE 만큼 누른 채 amovel 로 미끄러지며 샘플링.

        1) 평면식으로 예상한 벽 앞 10mm 까지 접근
        2) 힘 제어 ON → 로봇이 +X 로 밀려 나가 벽에 붙음 (힘이 찰 때까지 대기)
        3) 벽면을 따라 끝점까지 amovel → 이동 중 위치/힘 기록
           (X 방향은 힘 제어가 담당하므로 가로든 세로든 벽 굴곡을 따라 자동으로 움직임)
        4) 힘 제어 OFF → 후퇴
        """
        (y0, z0), (y1, z1) = start_yz, end_yz
        wx0 = self.wall_x_on_plane(plane, y0, z0)
        wx1 = self.wall_x_on_plane(plane, y1, z1)
        self.io.move_l([wx0 - APPROACH_GAP, y0, z0])
        self.io.zero_force()
        self.io.move_l([wx0 - 10.0, y0, z0], vel=PROBE_VEL * 2)

        count = 0
        try:
            self.io.start_press(SCAN_FORCE)
            # 벽에 닿아서 힘이 80% 이상 찰 때까지 대기 (최대 5초)
            t0 = time.time()
            while abs(self.io.get_force()[0]) < 0.8 * SCAN_FORCE:
                if time.time() - t0 > 5.0:
                    raise RuntimeError('힘 제어로 벽에 닿지 못함 (평면 위치 확인)')
                time.sleep(SAMPLE_PERIOD)

            self.io.amove_l([wx1, y1, z1], SCAN_VEL, SCAN_ACC)
            t0 = time.time()
            while time.time() - t0 < 0.5 or self.io.is_moving():
                pos = self.io.get_pos()
                f = float(np.linalg.norm(self.io.get_force()[:3]))
                if f >= 0.5 * SCAN_FORCE:                     # 붙어 있을 때만 기록
                    self.points.append({'kind': kind, 'xyz': pos.tolist(), 'force': f})
                    count += 1
                    if count % RVIZ_EVERY == 0:
                        self.update_rviz()
                if f > 3 * SCAN_FORCE:                        # 비정상 과부하 → 중단
                    self.io.stop()
                    raise RuntimeError(f'슬라이딩 중 과도한 힘 {f:.1f}N')
                time.sleep(SAMPLE_PERIOD)
        finally:
            self.io.release_press()
            pos = self.io.get_pos()
            self.io.move_l([pos[0] - APPROACH_GAP, pos[1], pos[2]], vel=PROBE_VEL * 3)
        return count

    def check_crossings(self, tol=3.0):
        """가로/세로 스캔이 교차하는 지점에서 두 측정의 X 값을 비교 (정확도 자가 점검).
        같은 벽 위치를 두 방향에서 쟀으므로 차이가 작을수록 측정이 일관적이다."""
        h = np.array([p['xyz'] for p in self.points if p['kind'] == 'scan_h'])
        v = np.array([p['xyz'] for p in self.points if p['kind'] == 'scan_v'])
        if len(h) == 0 or len(v) == 0:
            return
        diffs = []
        for z in SCAN_ROWS_Z:
            for y in SCAN_COLS_Y:
                hh = h[np.hypot(h[:, 1] - y, h[:, 2] - z) < tol + SCAN_STEP / 2]
                vv = v[np.hypot(v[:, 1] - y, v[:, 2] - z) < tol + SCAN_STEP / 2]
                if len(hh) and len(vv):
                    diffs.append(hh[:, 0].mean() - vv[:, 0].mean())
        if diffs:
            d = np.abs(diffs)
            self.log(f'[교차점 점검] {len(d)}곳, 가로-세로 X 차이 평균 {d.mean():.3f} mm, 최대 {d.max():.3f} mm')

    # ------------------------------------------------------------ 전체 실행
    def run_pipeline(self):
        self.log(f'=== 하이브리드 벽 탐색 시작 (mode={self.io.mode}) ===')
        if self.viz is not None:
            self.viz.clear()                  # 이전 탐색 마커 지우기
        self.io.move_j(READY_J)

        # STEP 1: 격자 프로빙
        for z in GRID_Z:
            for y in GRID_Y:
                self.probe_point(y, z)

        # STEP 2: 평면 계산
        plane = self.calculate_plane()

        # STEP 3-a: 가로 슬라이딩 (Y 방향)
        y0, y1 = SCAN_Y_RANGE
        self.log('=== 가로 슬라이딩 스캔 ===')
        for z in SCAN_ROWS_Z:
            self.sliding_scan(plane, (y0, z), (y1, z), 'scan_h')

        # STEP 3-b: 세로 슬라이딩 (Z 방향) - 열마다 방향을 바꿔(지그재그) 이동 거리 단축
        z0, z1 = SCAN_Z_RANGE
        self.log('=== 세로 슬라이딩 스캔 ===')
        for i, y in enumerate(SCAN_COLS_Y):
            a, b = (z0, z1) if i % 2 == 0 else (z1, z0)
            self.sliding_scan(plane, (y, a), (y, b), 'scan_v')

        self.check_crossings()
        self.log('=== 탐색 완료 ===')

    def save(self):
        """탐색 결과를 JSON 으로 저장하고 경로를 반환."""
        os.makedirs(SAVE_DIR, exist_ok=True)
        path = os.path.join(SAVE_DIR, datetime.now().strftime('scan_%Y%m%d_%H%M%S.json'))
        data = {
            'mode': self.io.mode,
            'robot_model': ROBOT_MODEL,
            'created': datetime.now().isoformat(timespec='seconds'),
            'config': {'grid_y': GRID_Y, 'grid_z': GRID_Z, 'scan_rows_z': SCAN_ROWS_Z,
                       'scan_y_range': list(SCAN_Y_RANGE), 'scan_cols_y': SCAN_COLS_Y,
                       'scan_z_range': list(SCAN_Z_RANGE), 'force_threshold': FORCE_THRESHOLD,
                       'scan_force': SCAN_FORCE, 'tool_ori': TOOL_ORI},
            # 가상 모드면 '정답 벽'도 저장 → 모델링 결과와 비교 가능
            'virtual_wall': VIRTUAL_WALL if self.io.mode == 'virtual' else None,
            'points': self.points,
        }
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        self.log(f'탐색 데이터 저장: {path} ({len(self.points)}개 점)')
        return path


# =============================================================================
# main
# =============================================================================
def main(args=None):
    rclpy.init(args=args)

    # DSR_ROBOT2 는 import 순간 DR_init 값을 읽으므로 먼저 설정
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('wall_navigation', namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # 모드 파라미터: --ros-args -p mode:=virtual|real  (기본 virtual = 안전)
    mode = node.declare_parameter('mode', 'virtual').value
    run_modeling = node.declare_parameter('modeling', True).value      # 끝나고 모델링 할지
    show_plot = node.declare_parameter('show_plot', True).value        # 그림 창 띄울지
    use_rviz = node.declare_parameter('rviz', True).value              # RViz 마커 퍼블리시
    frame_id = node.declare_parameter('frame_id', 'base_link').value   # RViz 기준 프레임
    if mode not in ('virtual', 'real'):
        node.get_logger().error(f"mode 는 'virtual' 또는 'real' 이어야 합니다 (입력: {mode})")
        rclpy.shutdown()
        return

    import DSR_ROBOT2 as dsr          # 반드시 __dsr__node 설정 '뒤에' import

    io = RobotIO(node, dsr, mode)
    viz = WallMarkerPublisher(node, topic='wall_markers', frame_id=frame_id) if use_rviz else None
    if viz is not None:
        io.log(f'RViz 마커: {node.resolve_topic_name("wall_markers")} (Fixed Frame: {frame_id})')
    explorer = HybridWallExplorer(io, viz)
    saved = None

    io.wait_for_services()
    if mode == 'real':
        # 실제 로봇은 움직이기 전에 한 번 더 확인받는다 (취소하면 아무 동작도 안 함)
        ans = input('[REAL] 실제 로봇이 벽 쪽으로 움직입니다. 주변 확인 후 y 입력: ')
        if ans.strip().lower() != 'y':
            io.log('취소했습니다.')
            node.destroy_node()
            rclpy.shutdown()
            return

    try:
        io.setup()
        explorer.run_pipeline()
    except KeyboardInterrupt:
        io.log('사용자 중단 (Ctrl+C)')
        io.stop()
    except Exception as e:
        node.get_logger().error(f'에러: {e}')
        io.stop()
    finally:
        # 어떤 경우에도 힘 제어 해제 → 데이터 저장 → 홈 복귀
        try:
            io.release_press()
        except Exception as e:
            node.get_logger().error(f'힘 제어 해제 실패: {e}')
        if explorer.points:
            saved = explorer.save()
        try:
            io.move_j(HOME_J)
        except Exception as e:
            node.get_logger().error(f'홈 복귀 실패: {e}')

    # STEP 4: 모델링 → 최종 모델(평면/곡면/이상치)을 RViz 에도 표시
    #   (그림 창을 띄우면 창을 닫을 때까지 여기서 멈춤 - RViz 마커는 그대로 남아 있음)
    try:
        if saved and run_modeling and len(explorer.points) >= 4:
            # on_model: 그림 창이 뜨기 '전에' RViz 로 먼저 보냄
            model_from_file(saved, show=show_plot,
                            on_model=lambda m: explorer.update_rviz(model=m))
    except Exception as e:
        node.get_logger().error(f'모델링 실패: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
