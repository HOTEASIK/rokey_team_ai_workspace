#!/usr/bin/env python3
"""wall_tap.py 가 쓰는 공통 부분: 탐색 방향/축 설정, 로봇 입출력(SizeIO), 사고 후 홈 복귀.

[좌표]  로봇 BASE 기준, mm.  dir(+x/-x/+y/-y) 를 고르면 configure_direction() 이
        벽 방향 축/부호, 옆 축, 공구 자세를 다시 정한다.
        옆 = LAT_AXIS, 벽 쪽 = APPROACH_AXIS × APPROACH_SIGN, 위 = UP_AXIS(Z)

[모드]  real    : 실측 힘 (get_tool_force - 영점)
        virtual : 에뮬레이터는 외력이 0 이라 VIRTUAL_BOX(가상 벽+바닥)로 힘을 계산
"""
import os
import sys
import time

import numpy as np

from utils.move_stop import MoveStopClient
from utils.service_waiter import ServiceWaiter
from utils.robot import TOOL_NAME, TCP_NAME, HOME_J  # config/robot.yaml robot 섹션

try:
    from navigation.force_guard import ForceGuardError, BAD_STATES
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from force_guard import ForceGuardError, BAD_STATES

# =============================================================================
# 축 설정  (0=X, 1=Y, 2=Z) - configure_direction() 이 다시 정함
# =============================================================================
LAT_AXIS = 1            # 옆 방향 (너비를 재는 축)
APPROACH_AXIS = 0       # 벽을 향하는 축
APPROACH_SIGN = +1.0    # +1: +축 방향으로 가면 벽, -1: -축 방향으로 가면 벽
UP_AXIS = 2             # 위 방향 (높이를 재는 축)
AX_NAME = 'xyz'

# 방향 → (벽 방향 축, 부호, 옆 축, 공구 회전각[deg])
DIRECTIONS = {
    '+x': (0, +1.0, 1, 0.0),
    '-x': (0, -1.0, 1, 180.0),
    '+y': (1, +1.0, 0, 90.0),
    '-y': (1, -1.0, 0, -90.0),
}
DIRECTION = '+x'

# 그리퍼 기울기: 0 = 벽을 수평으로 봄, 90 = 바닥을 수직으로 봄 (wall_tap 이 C.tilt 로 덮어씀)
TILT_DEG = 75.0
# ZYZ [a, b, c]: a = 탐색 방향 회전각, b = 90+TILT, c = 공구 축 둘레 회전
TOOL_ORI = [0.0, 90.0 + TILT_DEG, 0.0]

# 그리퍼: 닫기 = DO1 ON, DO2 OFF
GRIP_DO_CLOSE = (1, 1)      # (포트, 값)
GRIP_DO_OPEN = (2, 0)
GRIP_WAIT_SEC = 1.0         # 그리퍼가 다 닫힐 때까지 대기

# =============================================================================
# 속도
# =============================================================================
MOVE_VELOCITY, MOVE_ACC = 100.0, 200.0     # 일반 직선 이동 [mm/s, mm/s^2]
PROBE_ACC = 50.0                           # 찍기 가속도
JOINT_VELOCITY, JOINT_ACC = 30.0, 30.0     # 관절 이동 [deg/s]
APPROACH_GAP = 40.0         # 사고 후 위/벽 반대쪽으로 빠지는 거리
SAMPLE_PERIOD = 0.02
PATH_MIN_MM = 1.0           # 경로 기록 간격 (이만큼 움직여야 새 점 추가)

SAVE_DIR = os.path.expanduser('~/wall_scan_data')

# (virtual 전용) 가상 박스 벽 + 바닥 (wall_tap 이 vbox_* 로 덮어씀)
VIRTUAL_BOX = {
    'face': 500.0,          # 벽 표면 APPROACH 좌표
    'thickness': 20.0,      # 벽 두께
    'lat': (-250.0, 250.0), # 벽 옆 범위
    'floor': 50.0,          # 바닥 높이
    'top': 350.0,           # 벽 윗면 높이
    'stiffness': 20.0,      # [N/mm]
    'noise': 0.2,           # [N]
}

UNIT = np.eye(3)
APP_DIR = UNIT[APPROACH_AXIS] * APPROACH_SIGN
UP_DIR = UNIT[UP_AXIS]
LAT_DIR = UNIT[LAT_AXIS]


def _tool_dir(ori):
    """ZYZ [a, b, c] (deg) → 공구 Z축 방향 (손가락이 향하는 방향)."""
    a, b = np.radians(ori[0]), np.radians(ori[1])
    return np.array([np.cos(a) * np.sin(b), np.sin(a) * np.sin(b), np.cos(b)])


def configure_direction(name):
    """탐색 방향(+x/-x/+y/-y)에 맞게 축/부호/공구 자세를 다시 정한다 (측정 전에 한 번)."""
    global DIRECTION, APPROACH_AXIS, APPROACH_SIGN, LAT_AXIS, TOOL_ORI, APP_DIR, LAT_DIR
    key = str(name).strip().lower()
    if key in ('x', 'y'):
        key = '+' + key
    if key not in DIRECTIONS:
        raise ValueError(f'dir 은 {list(DIRECTIONS)} 중 하나: {name}')
    APPROACH_AXIS, APPROACH_SIGN, LAT_AXIS, yaw = DIRECTIONS[key]
    DIRECTION = key
    TOOL_ORI = [yaw, 90.0 + TILT_DEG, 0.0]
    APP_DIR = UNIT[APPROACH_AXIS] * APPROACH_SIGN
    LAT_DIR = UNIT[LAT_AXIS]
    return key


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


# =============================================================================
# 로봇 입출력 - 모드(virtual/real) 차이를 여기에 숨긴다
# =============================================================================
class SizeIO:
    """DSR_ROBOT2 래퍼. wall_tap.TapIO 가 '방향 힘 제어' 를 덧붙여 쓴다."""

    on_move = None              # move_l 끝날 때 호출 (경로 기록용 콜백)
    guard = None                # force_guard.ForceGuard (있으면 모든 move_l 을 감시)

    def __init__(self, node, dsr, mode):
        self.node = node
        self.dsr = dsr                      # DSR_ROBOT2 모듈
        self.mode = mode                    # 'virtual' or 'real'
        self.box = VirtualBox(VIRTUAL_BOX) if mode == 'virtual' else None
        self.force_bias = np.zeros(6)       # (real) 힘 센서 영점
        self.force_on = False               # 힘 제어 켜짐 여부 (finally 에서 해제용)
        self.compliance_on = False
        self.stopper = MoveStopClient(node)  # DSR_ROBOT2 에는 정지 함수가 없어서 서비스 직접 사용

    # ---------------------------------------------------------------- 기본
    def wait_for_services(self):
        """사용할 서비스가 모두 뜰 때까지 대기 (안 하면 첫 호출에서 멈출 수 있음)."""
        d = self.dsr
        clients = [d._ros2_set_robot_mode, d._ros2_movej, d._ros2_movel,
                   d._ros2_get_current_posx, d._ros2_check_motion, self.stopper.cli,
                   d._ros2_set_digital_output, d._ros2_set_singularity_handling,
                   d._ros2_ikin, d._ros2_fkin, d._ros2_get_current_solution_space]
        if self.mode == 'real':
            clients += [d._ros2_set_current_tool, d._ros2_set_current_tcp,
                        d._ros2_get_tool_force, d._ros2_set_ref_coord,
                        d._ros2_task_compliance_ctrl, d._ros2_set_desired_force,
                        d._ros2_release_force, d._ros2_release_compliance_ctrl]
        ServiceWaiter(self.node).wait(*clients)

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
        self._check(self.dsr.movej(self.dsr.posj(*q), vel=JOINT_VELOCITY, acc=JOINT_ACC), 'movej')

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
        self._check(self.dsr.movel(self._posx(xyz), vel=[vel, 30.0], acc=[acc, 60.0],
                                   ref=self.dsr.DR_BASE), 'movel')
        if self.on_move is not None:
            self.on_move(xyz)

    def amove_l(self, xyz, vel, acc):
        """직선 이동 (명령만 보내고 바로 리턴 → 이동 중에 센서를 볼 수 있음)."""
        self._check(self.dsr.amovel(self._posx(xyz), vel=[vel, 30.0], acc=[acc, 60.0],
                                    ref=self.dsr.DR_BASE), 'amovel')

    def stop(self):
        """즉시 정지 (Quick stop, 정지 카테고리 2)."""
        self.stopper.stop(self.dsr.DR_QSTOP, timeout_sec=2.0)

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
        """현재 외력 [Fx, Fy, Fz, Mx, My, Mz]. virtual: 가상 박스 / real: 실측값 - 영점."""
        if self.mode == 'virtual':
            return self.box.force_at(pos if pos is not None else self.get_pos())
        f = self.dsr.get_tool_force(ref=self.dsr.DR_BASE)
        if not isinstance(f, list):     # 서비스 실패 시 -1 이 돌아옴
            raise RuntimeError('get_tool_force 실패')
        return np.array(f, dtype=float) - self.force_bias

    def zero_force(self, samples=10):
        """(real) 접촉 없는 상태에서 여러 번 읽어 평균을 영점으로 저장."""
        if self.mode != 'real':
            return
        self.force_bias = np.zeros(6)
        vals = [self.get_force() for _ in range(samples)]
        self.force_bias = np.mean(vals, axis=0)

    def release_press(self):
        """힘 제어/컴플라이언스 해제 (켜져 있을 때만)."""
        if self.force_on:
            self.dsr.release_force()
            self.force_on = False
        if self.compliance_on:
            self.dsr.release_compliance_ctrl()
            self.compliance_on = False

    # ---------------------------------------------------------------- 기타
    def set_singularity(self, var_vel):
        """특이점 처리: True → DR_VAR_VEL (경로 유지, 감속), False → DR_AVOID (기본)."""
        d = self.dsr
        mode = d.DR_VAR_VEL if var_vel else d.DR_AVOID
        ret = d.set_singularity_handling(mode)
        self.node.get_logger().info(
            f'[특이점 처리] {"DR_VAR_VEL (경로 유지)" if var_vel else "DR_AVOID (기본)"}'
            f'{"" if ret == 0 else " - 설정 실패!"}')
        return ret == 0

    def joint_at(self, xyz, sol_space, tol=2.0):
        """역기구학: TCP 위치 → 관절각 [deg] (실패/도달 불가면 None).
        컨트롤러 ikin 은 도달 불가여도 success=true 로 아무 값이나 돌려주므로,
        fkin 으로 다시 계산해서 목표 위치와 tol[mm] 이상 다르면 도달 불가로 본다."""
        q = self.dsr.ikin(self._posx(xyz), sol_space, ref=self.dsr.DR_BASE)
        try:
            q = [float(v) for v in q]           # ROS 응답은 numpy 배열 (list/tuple 아님)
        except TypeError:
            return None
        if len(q) < 6 or not all(np.isfinite(q)):
            return None
        p = self.dsr.fkin(self.dsr.posj(*q[:6]), ref=self.dsr.DR_BASE)
        try:
            p = [float(v) for v in p]
        except TypeError:
            return None
        if len(p) < 3 or np.linalg.norm(np.subtract(p[:3], np.asarray(xyz, float)[:3])) > tol:
            return None
        return q

    def gripper_close(self):
        """그리퍼를 닫아 손가락 끝을 탐침으로 쓴다."""
        self.node.get_logger().info('[그리퍼] 닫기 → 손가락 끝으로 탐색')
        self.dsr.set_digital_output(*GRIP_DO_OPEN)    # 반대쪽 신호 먼저 끄기
        self.dsr.set_digital_output(*GRIP_DO_CLOSE)
        time.sleep(GRIP_WAIT_SEC)

    def _posx(self, xyz):
        return self.dsr.posx(*[float(v) for v in xyz[:3]], *TOOL_ORI)

    @staticmethod
    def _check(ret, name):
        """DSR 함수는 실패하면 음수를 돌려준다 → 예외로 바꿔서 바로 멈추게 함."""
        if isinstance(ret, (int, float)) and ret < 0:
            raise RuntimeError(f'{name} 실패 (ret={ret})')


# =============================================================================
# 사고 후 홈 복귀
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
        # 바닥/벽에 닿은 채로 멈췄을 수 있으므로: 위로 먼저 → 벽 반대쪽으로 → 홈.
        # 닿은 상태에선 힘/모멘트가 남아 있어 감시 이동이 바로 다시 멈추므로, 짧고 느린 비감시 후퇴를 쓴다.
        if not guard.relieve(UP_DIR, dist=APPROACH_GAP):
            raise RuntimeError('위로 후퇴 실패')
        if not guard.relieve(-APP_DIR, dist=APPROACH_GAP):
            raise RuntimeError('벽 반대쪽 후퇴 실패')
        io.move_j(HOME_J)
    except ForceGuardError as e:
        io.node.get_logger().info(f'홈 복귀 중 다시 사고 감지 → 정지: {e}')
    except Exception as e:
        io.node.get_logger().info(f'홈 복귀 실패: {e}')
