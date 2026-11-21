#!/usr/bin/env python3
"""벽 내부 탐색(wall_scanner)이 쓰는 로봇 입출력 + 평면 피팅.

원래 navigation/wall_navigation.py(RobotIO)와 navigation/wall_modeling.py(fit_plane, robust_fit_plane)에 있던 코드다.
팀 저장소에서 두 파일이 '벽 탐색은 wall_tap 하나로 통일'하며 정리됐기 때문에, wall_scanner 가 쓰는 부분만
동작을 바꾸지 않고 이 파일로 옮겼다 (가상 벽 격자, start_press, 기본 Tool 설정 등 안 쓰는 부분은 제외).
"""
import time

import numpy as np

from utils.move_stop import MoveStopClient
from utils.service_waiter import ServiceWaiter

MOVE_VELOCITY, MOVE_ACC = 100.0, 200.0   # 일반 직선 이동 속도 [mm/s], 가속도 [mm/s^2]
JOINT_VELOCITY, JOINT_ACC = 30.0, 30.0   # 관절 이동 속도 [deg/s], 가속도 [deg/s^2]
SAMPLE_PERIOD = 0.02                     # 이동 완료 확인 주기 [s] (50 Hz)


# =============================================================================
# 로봇 입출력 래퍼 - 모드(virtual/real) 차이를 이 클래스 안에 숨긴다
# =============================================================================
class RobotIO:
    """DSR_ROBOT2 함수를 감싸서, 탐색 로직이 모드를 신경쓰지 않게 해 준다.

    자세(_posx), 가상 모드의 힘, Tool/TCP 설정은 하위 클래스(wall_scanner.ScanIO)가 정한다.
    """

    def __init__(self, node, dsr, mode):
        self.node = node
        self.dsr = dsr                      # DSR_ROBOT2 모듈
        self.mode = mode                    # 'virtual' or 'real'
        self.force_bias = np.zeros(6)       # (real) 힘 센서 영점
        self.force_on = False               # 힘 제어 켜짐 여부 (finally 에서 해제용)
        self.compliance_on = False
        # DSR_ROBOT2 에는 '정지(stop)' 함수가 없어서 MoveStop 서비스를 직접 쓴다
        self.stopper = MoveStopClient(node)

    # ---------------------------------------------------------------- 기본
    def wait_for_services(self):
        """사용할 서비스가 모두 뜰 때까지 대기 (안 하면 첫 호출에서 멈출 수 있음)."""
        d = self.dsr
        clients = [d._ros2_set_robot_mode, d._ros2_movej, d._ros2_movel,
                   d._ros2_get_current_posx, d._ros2_check_motion, self.stopper.cli]
        if self.mode == 'real':
            clients += [d._ros2_set_current_tool, d._ros2_set_current_tcp,
                        d._ros2_get_tool_force, d._ros2_set_ref_coord,
                        d._ros2_task_compliance_ctrl, d._ros2_set_desired_force,
                        d._ros2_release_force, d._ros2_release_compliance_ctrl]
        ServiceWaiter(self.node).wait(*clients)

    # ---------------------------------------------------------------- 이동
    def move_j(self, q):
        """관절 이동 movej (끝날 때까지 기다림)."""
        self._check(self.dsr.movej(self.dsr.posj(*q), vel=JOINT_VELOCITY, acc=JOINT_ACC), 'movej')

    def move_l(self, xyz, vel=MOVE_VELOCITY, acc=MOVE_ACC):
        """직선 이동 movel (끝날 때까지 기다림). 자세는 _posx 가 정한다."""
        self._check(self.dsr.movel(self._posx(xyz), vel=[vel, 30.0], acc=[acc, 60.0],
                                   ref=self.dsr.DR_BASE), 'movel')

    def amove_l(self, xyz, vel, acc):
        """직선 이동 amovel (명령만 보내고 바로 리턴 → 이동 중에 센서를 볼 수 있음)."""
        self._check(self.dsr.amovel(self._posx(xyz), vel=[vel, 30.0], acc=[acc, 60.0],
                                    ref=self.dsr.DR_BASE), 'amovel')

    def stop(self):
        """즉시 정지 (Quick stop, 정지 카테고리 2)."""
        self.stopper.stop(self.dsr.DR_QSTOP, timeout_sec=2.0)

    def is_moving(self):
        return self.dsr.check_motion() != self.dsr.DR_STATE_IDLE

    def wait_motion_done(self, timeout=10.0):
        t0 = time.time()
        while self.is_moving() and time.time()-t0 < timeout:
            time.sleep(SAMPLE_PERIOD)

    # ---------------------------------------------------------------- 센서
    def get_pos(self):
        """현재 TCP 위치 [x, y, z] (BASE, mm)."""
        posx, _ = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        return np.array(posx[:3], dtype=float)

    def get_force(self, pos=None):
        """(real) 현재 외력 [Fx, Fy, Fz, Mx, My, Mz] = 실측값 - 영점. virtual 은 하위 클래스가 계산한다."""
        f = self.dsr.get_tool_force(ref=self.dsr.DR_BASE)
        if not isinstance(f, list):     # 서비스 실패 시 -1 이 돌아옴
            raise RuntimeError('get_tool_force 실패')
        return np.array(f, dtype=float)-self.force_bias

    def zero_force(self, samples=10):
        """(real) 접촉 없는 상태에서 여러 번 읽어 평균을 영점으로 저장.
        공구 무게/센서 오프셋 때문에 가만히 있어도 힘이 0 이 아니기 때문."""
        if self.mode != 'real':
            return
        self.force_bias = np.zeros(6)
        vals = [self.get_force() for _ in range(samples)]
        self.force_bias = np.mean(vals, axis=0)

    # ---------------------------------------------------------------- 힘 제어 해제 (real)
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
        raise NotImplementedError('자세는 하위 클래스(ScanIO)가 정한다')

    @staticmethod
    def _check(ret, name):
        """DSR 함수는 실패하면 음수를 돌려준다 → 예외로 바꿔서 바로 멈추게 함."""
        if isinstance(ret, (int, float)) and ret < 0:
            raise RuntimeError(f'{name} 실패 (ret={ret})')


# =============================================================================
# 평면 피팅
# =============================================================================
def fit_plane(points):
    """점들에 가장 잘 맞는 평면 (SVD). 법선은 로봇(원점) 쪽을 향하게 통일.

    Returns:
        dict(normal, centroid, d, rms, max_abs, residuals)   평면: normal·p + d = 0
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        raise ValueError(f'평면 피팅에는 점이 3개 이상 필요합니다 (현재 {len(pts)}개)')
    centroid = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts-centroid)
    normal = vh[2, :]                      # 가장 작은 특이값에 대응하는 벡터
    if np.dot(normal, -centroid) < 0:      # 법선 방향을 로봇(원점) 쪽으로
        normal = -normal
    d = -float(np.dot(normal, centroid))
    residuals = (pts-centroid) @ normal    # 각 점의 평면까지 부호 있는 거리 [mm]
    return {
        'normal': normal,
        'centroid': centroid,
        'd': d,
        'rms': float(np.sqrt(np.mean(residuals**2))),
        'max_abs': float(np.max(np.abs(residuals))),
        'residuals': residuals,
    }


def robust_fit_plane(points, sigma=3.0, max_iter=5, min_tol=2.0):
    """이상치(튀는 점)를 반복적으로 제거하며 평면을 피팅한다.

    - 평면에서 sigma × RMS 보다 멀리 떨어진 점은 이상치로 보고 제외
    - 단, 기준 거리는 최소 min_tol[mm] (점들이 너무 잘 맞을 때 정상점까지 버리는 것 방지)
    - 제외할 점이 더 이상 없거나 점이 4개 미만이 되면 멈춘다.

    Returns:
        (plane_dict, inlier_mask)  inlier_mask[i] = True 이면 사용된 점
    """
    pts = np.asarray(points, dtype=float)
    mask = np.ones(len(pts), dtype=bool)
    plane = fit_plane(pts)
    for _ in range(max_iter):
        dist = np.abs((pts-plane['centroid']) @ plane['normal'])
        tol = max(sigma*plane['rms'], min_tol)
        new_mask = dist <= tol
        if new_mask.sum() < 4 or np.array_equal(new_mask, mask):
            break
        mask = new_mask
        plane = fit_plane(pts[mask])
    plane['residuals_all'] = (pts-plane['centroid']) @ plane['normal']   # 최종 잔차는 전체 점 기준
    return plane, mask
