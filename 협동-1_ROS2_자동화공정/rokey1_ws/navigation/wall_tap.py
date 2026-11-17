#!/usr/bin/env python3
"""두산 M0609 로 벽을 '점 단위로 찍어서(tap)' 높이 / 너비를 재는 노드 (virtual / real).

wall_size.py 는 벽을 누른 채 미끄러지며 재는 방식이고, 이 노드는 매번 떨어졌다가
다시 찍는 방식이다. 느리지만 마찰 영향이 없고 '닿음 / 안 닿음' 판정이 분명하다.

[알고리즘]
  0. 시작 위치(start)로 movel. 그리퍼(닫힘)는 XZ 평면에서 45° 숙임 (벽 쪽 + 아래쪽)
     → 접촉 없는 상태에서 힘 노이즈 측정 (평균 μ, 공분산 Σ) → 접촉 임계값 계산
  1. 바닥  : 순응제어 ON → -Z 로 FORCE[N] 힘 제어 → 막히면 좌표 기록 → 힘/순응 해제 → 후퇴
             → floor_z
  2. 첫 벽 : 바닥 + LIFT 높이에서 +X(벽 방향)로 힘 제어 전진 → 막히면 wall 표면 좌표
  3. 높이  : 벽 앞 STANDOFF 에서 z 를 STEP_Z 씩 올리며 찍기 반복
             → 연속 MISS_CONFIRM 번 안 닿으면 '벽 끝' → (마지막 닿음 ~ 첫 안 닿음) 구간을
               TOL 까지 이분탐색 → top_z,   높이 = top_z - floor_z
             height_lats 로 옆 위치 여러 곳에서 반복 → 중앙값 (이전 끝점을 힌트로 써서 빠르게)
  4. 너비  : 높이의 width_ups 비율 높이(기본 0.5 = 가운데)에서 왼쪽(+Y) → 오른쪽(−Y) 순서로 같은 방법
             (width_first=left. 가동 범위 때문에 기준 자세로 먼저 +Y, 자세를 바꿔 −Y)
             자세(width_ori=lean): 공구 축을 Z축과 수직(수평)으로 두고 벽을 정면으로 봄 (width_yaw=0, 기본)
               - width_yaw=0 이면 양쪽 자세가 같음 [0, 90, 90] → 처음 한 번만 자세를 바꾸고 그대로 양쪽 탐색
               - 아래는 width_yaw=45 로 줄 때 (Z축 둘레로 찍는 쪽으로 45° 기울임)
               - 공구 좌표축을 X/Y/Z 에 맞춤: 공구 z(탐침) = 수평 45°, 공구 y = ±Z, 공구 x = 수평
               - 기준 자세 (+x 벽, 왼쪽(+Y) 찾을 때) task BASE [786.680,-21.090,116.960, 46.56,101.05,93.67],
                 joint [-24.36,51.39,73.72,91.82,68.58,-29.89] → 정리하면 ZYZ [45, 90, 90]
                 (공구 z 가 +X+Y 쪽 = 찾는 쪽) / 오른쪽(−Y)은 Y 대칭 [-45, 90, -90]
               - width_lean=toward(기본): 찾는 쪽으로 기울임 / away: 반대로
             탐침: 그리퍼 끝(등록 TCP)에서 공구 축 방향으로 probe_len(120mm) 더 나간 점을 TCP 로 씀
             → 너비 = 오른쪽 끝 ~ 왼쪽 끝 거리, 여러 높이면 중앙값
  5. 결과 출력 + 저장 → 홈 복귀
     ~/wall_scan_data/tap_*.json        : 설정, 노이즈, 선별 결과, 중앙값/표준편차, 오차, 시간, 찍기 요약
     ~/wall_scan_data/tap_*_trace.json  : 찍기마다 [t, x,y,z, Fx..Mz, 필터된 진행방향 힘] 시계열

[찍기 1회 (probe)]
  ① 시작점(벽/바닥 앞, 접촉 없음)으로 movel → 힘 영점
  ② real   : task_compliance_ctrl(힘 방향 축만 부드럽게) → set_desired_force(방향 × FORCE, REL)
     virtual: 에뮬레이터는 외력 0 → 같은 방향으로 저속 amovel + 가상 박스 벽(VirtualBox)으로 힘 계산
  ③ 접촉 판정 (tactile_exploration_optimization_plan 2.2):
       - 진행 방향 힘 성분을 필터(median / ma / none, filter_win)
       - 진입 임계값 th_in = max(contact, k_sigma × σ_진행방향)   (σ: 0번에서 측정한 노이즈)
       - 해제 임계값 th_out = hyst × th_in  (히스테리시스: 한 번 넘으면 th_out 아래로 떨어져야 해제)
       - th_in 이상이 contact_hold 초 지속 + (real) 위치 정체(STALL) → '닿음'
       - 좌표: real = 판정 순간(힘을 풀기 전), virtual = 접촉 시작 순간 (가상 벽은 계속 뚫고 들어가므로)
     진행 거리 > depth → '안 닿음' (허공으로 계속 나가지 않게)
  ④ release_force → release_compliance_ctrl → 시작점으로 후퇴
  ※ 허공에서 힘 제어는 계속 가속할 수 있음 → MAX_FC_SPEED 를 넘으면 즉시 해제/중단.

[특이점]  45° 는 벽 아래쪽에서 손목 J5≈0 을 지날 수 있다 (wall_size.py 는 그래서 75° 사용).
          각 단계 전에 예상 경로를 역기구학으로 미리 계산해서 |J5| < sing_deg 이면 중단
          (sing_abort:=false 면 경고만). 이동은 DR_VAR_VEL (경로 유지, 감속) 로 처리.

[실행]
  ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m0609
  ros2 run rokey1 wall_tap --ros-args -p mode:=virtual
  ros2 run rokey1 wall_tap --ros-args -p mode:=virtual -p height_lats:="-150,0,150" -p width_ups:="0.25,0.5,0.75"
  ros2 run rokey1 wall_tap --ros-args -p mode:=real -p start:="350,0,200" -p z_limit:=450 \
      -p lat_min:=-320 -p lat_max:=320 -p true_height:=300 -p true_width:=500
  파라미터 조합 반복 실험 (Grid Search): wall_tap_grid.py

[실제 로봇 전 체크리스트]
  - start : 벽보다 확실히 앞, 바닥 위. 이 아래(-Z)로 floor_depth 안에 바닥이 있어야 함
  - start 에서 +X 로 wall_search 안에 벽이 있어야 함
  - z_limit / lat_min~lat_max : 벽보다 넓게, 하지만 작업 영역 안
  - TCP_NAME(wall_navigation.py) 이 '닫힌 손가락 끝' 으로 등록돼 있어야 측정값이 손끝 위치가 됨
  - 처음엔 force 를 작게(5~10N), 작은 범위로 바닥/첫 벽만 먼저 시험
  - 오차를 보려면 실제 치수를 자로 재서 true_height / true_width 로 넣을 것
"""
import csv
import json
import os
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np
import rclpy
import DR_init
from rcl_interfaces.msg import ParameterDescriptor

from utils.shutdown import safe_shutdown

try:
    import navigation.wall_size as ws
    from navigation.wall_size_rviz import WallSizeViz
    from navigation.force_guard import (ForceGuard, ToolCollision, ArmCollision,
                                        ControllerStop, SensorFault)
except ImportError:
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import wall_size as ws
    from wall_size_rviz import WallSizeViz
    from force_guard import ForceGuard, ToolCollision, ArmCollision, ControllerStop, SensorFault

ROBOT_ID = 'dsr01'
ROBOT_MODEL = 'm0609'


# =============================================================================
# 설정 (ROS 파라미터로 바꿀 수 있음, 단위 mm / N / s)
# =============================================================================
class Cfg:
    direction = '+x'            # 벽 방향 (+x/-x/+y/-y)
    tilt = 45.0                 # 그리퍼 기울기 [deg] (0 = 벽을 수평으로 봄, 90 = 바닥을 수직으로 봄)
    start = (700.0, 0.0, 100.0)  # 시작 위치 BASE [x, y, z] (벽 앞, 바닥 위) (실측 셋업 기준)

    force = 10.0                # 힘 제어 지령 크기 [N] (바닥: -Z, 벽: 벽 방향)
    stall_mm = 0.3              # (real) 이 거리 이하로만 움직이면 '정체'
    stall_sec = 0.3             # (real) ... 이 시간 동안
    max_fc_speed = 40.0         # (real) 힘 제어 중 이 속도[mm/s] 넘으면 위험 → 중단
    probe_vel = 10.0            # (virtual) 찍기 전진 속도 [mm/s]
    move_vel = 40.0             # 찍기 사이 이동 속도 [mm/s]
    move_acc = 80.0             # 이동 가속도 상한 [mm/s²]
    approach_vel = 20.0         # 홈 → 시작 위치 첫 이동 속도 [mm/s] (자세가 크게 바뀜 → 천천히)
    approach_acc = 40.0         # 첫 이동 가속도 [mm/s²]
    rot_vel = 15.0              # 공구 회전 속도 [deg/s] (빠르면 힘 추정 오차가 커짐)
    rot_acc = 30.0              # 공구 회전 가속도 [deg/s²]

    # 접촉 판정
    contact = 3.0               # 진입 임계값 하한 [N]
    k_sigma = 5.0               # 진입 임계값 = max(contact, k_sigma × 노이즈 σ)
    hyst = 0.6                  # 해제 임계값 = hyst × 진입 임계값
    contact_hold = 0.1          # 진입 후 이 시간 [s] 이상 유지돼야 '닿음'
    filter = 'median'           # 진행 방향 힘 필터: median | ma | none
    filter_win = 5              # 필터 창 크기 [샘플]
    noise_samples = 50          # 노이즈 측정 샘플 수

    floor_depth = 250.0         # 시작 위치에서 이만큼 내려가도 안 닿으면 바닥 측정 실패
    wall_search = 300.0         # 시작 위치에서 벽 방향으로 이만큼 가도 안 닿으면 벽 없음
    lift = 10.0                 # 바닥에서 이만큼 띄워서 첫 벽 찾기 / 높이 측정 시작
    standoff = 15.0             # 찍기 전 벽 표면 앞 대기 거리
    max_depth = 20.0            # 표면보다 이만큼 더 들어가도 안 닿으면 '안 닿음'
    step_z = 30.0               # 높이 측정 간격
    step_y = 30.0               # 너비 측정 간격
    tol = 1.0                   # 이분탐색 종료 구간 크기
    miss_confirm = 2            # 연속 이만큼 안 닿아야 '벽 끝' (한 번의 오판 방지)

    # 반복 측정 (중앙값)
    height_lats = (0.0,)        # 높이를 잴 옆 위치 (시작 위치 기준 오프셋 [mm])
    width_ups = (0.5,)          # 너비를 잴 높이 (바닥~윗면 비율, 0~1)
    width_z = 0.0               # >0 이면 높이 측정을 건너뛰고 이 BASE z 에서 좌우만 탐색 [mm]
    width_ori = 'lean'          # 너비 측정 자세: lean = 수평 + Z축 둘레로 width_yaw 기울임 / keep = 높이 측정 자세 그대로
    width_first = 'left'        # 너비 먼저 찾을 쪽: left(+Y, 기준 자세) → right(−Y) / right → left
    width_lean = 'toward'       # (lean) toward = 찾는 쪽으로 기울임 (기준 자세: +Y 찾을 때 +Y 쪽) / away = 반대로
    width_yaw = 0.0             # (lean) 벽 방향에서 Z축 둘레로 기울이는 각 [deg] (0 = 벽 정면, 45 = 찍는 쪽으로 45°)
    width_tilt = 0.0            # (lean) 아래로 숙이는 각 [deg] (0 = X-Y 평면과 평행, 기준 자세는 약 11°)
    width_roll = -90.0          # (lean) 공구 축 둘레 회전 (ZYZ c) [deg] (-90 = 공구 y 축이 −Z, 실측 기준 관절의 FK ≈ [0, 90, -90])
    rot_clear = 30.0            # 자세 바꾸기 전에 벽에서 이만큼 더 물러남 [mm]
    # (lean) 너비 자세의 관절 배치 기준 (+x 벽 기준 joint [deg], J1 은 방향에 맞춰 자동 회전). '' = 사용 안 함 (movel)
    #   같은 TCP 자세라도 손목(J4~J6) 배치가 여러 개 → 쪽마다 실측 관절값의 solution space 로 ikin → movej.
    #   실측 (2026-11-17): 두 값 모두 FK 로 공구 z = +X 수평, 공구 y = −Z (ZYZ ≈ [0, 90, -90]) → 자세는 같고 손목만 뒤집힘
    #     오른쪽(−Y): [0, 27.46, 127.68, 176.69, 64.48, 91.43]
    #     왼쪽(+Y)  : [0, 24.63, 127.20, -1.28, -60.84, 270.57]  (받은 값은 J5 +60.84 였으나 그러면 TCP 가 z −89(바닥 아래)
    #                 → 손목 뒤집힘(J4+180, −J5, J6+180) 관계에 맞는 −60.84 로 사용)
    #   (이전 45° 자세용: left (-24.36, 51.39, 73.72, 91.82, 68.58, -29.89), right (23.26, 51.46, 73.76, -91.95, 67.34, 29.97))
    width_ref_j_left = (0.0, 24.63, 127.20, -1.28, -60.84, 90.57)   # J6 270.57 → 90.57 (사용자 요청, 공구 축 둘레 180° → 롤 +90)
    width_roll_left = 90.0      # 왼쪽 탐색 롤 (J6 ≈ 90 → FK [0, 90, +90])
    width_ref_j_right = (0.0, 27.46, 127.68, 176.69, 64.48, 91.43)
    flip_clear = 60.0           # 자세 바꾸는 관절 경로에서 탐침 끝이 바닥 위/벽 앞에 남길 여유 [mm] (모자라면 들어 올리고 물러남)
    reorient_z_max = 600.0      # 자세 바꾸려고 들어 올릴 수 있는 최대 높이 (BASE z) [mm]
    probe_len = 0.0             # 등록 TCP 에서 탐침 끝까지 더할 길이 [mm], 공구 z 방향 (실측: 펜던트 TCP 가 이미 손끝 → 0)
    joint_vel = 10.0            # 자세 바꿀 때 movej 속도 [deg/s]
    joint_acc = 20.0            # movej 가속도 [deg/s²]
    reorient_step = 15.0        # 자세 바꿀 때 이 각도[deg]마다 멈춰서 힘 영점 재설정 (공구 무게 영향 → 가드 오판 방지)
    travel_clear = 15.0         # 찍기 사이 옆/위 이동은 (현재·다음 시작점 중 먼 쪽에서) 이만큼 더 물러나서 [mm]

    z_limit = 450.0             # 높이 측정 상한 (BASE z)
    lat_range = (-320.0, 320.0)  # 너비 측정 옆 방향 한계 (BASE)

    stiff_soft = 500.0          # (real) 힘 방향 축 강성 [N/m]
    stiff_other = 3000.0        # (real) 나머지 축 강성 [N/m]
    stiff_rot = 200.0           # (real) 회전 강성 [Nm/rad]

    sing_deg = 5.0              # |J5| 가 이보다 작으면 특이점 근처
    sing_abort = True           # True: 특이점 근처 경로가 있으면 중단 / False: 경고만

    true_height = 0.0           # 실제 높이 [mm] (0 = 모름, virtual 은 가상 벽 값 자동)
    true_width = 0.0            # 실제 너비 [mm]
    out = ''                    # 결과 파일 경로 ('' = ~/wall_scan_data/tap_<시각>.json)
    tag = ''                    # 실험 조합 이름 (wall_tap_grid 가 넣음)
    log_trace = True            # 찍기마다 힘/위치 시계열 저장
    confirm = True              # (real) 시작 전 y 입력 확인

    # 접촉 없는 이동 가드 (force_guard 참고)
    free_guard = 'baseline'     # abs = |힘| ≥ 15N 이면 충돌 / baseline = 기준선 대비 급변이 유지될 때만 충돌
    free_tau = 0.5              # (baseline) 기준선 시상수 [s]
    free_hold = 0.3             # (baseline) 급변이 이 시간 유지돼야 충돌 [s]
    free_hard = 40.0            # (baseline) 절대 힘 상한 [N] (넘으면 즉시)
    free_mhard = 8.0            # (baseline) 절대 모멘트 상한 [Nm] (넘으면 즉시)
    log_motion = True           # 이동/자세 바꾸기 중 매 샘플 힘·위치·관절을 _motion.csv 로 저장 (원인 분석용)

    # (virtual) 가상 벽 — 실측 셋업에 맞춤 (2026-11-17 real 로그: 바닥 ≈95, 벽 표면 ≈718, 높이 ≈195)
    vbox_face = 720.0           # 벽 표면 x [mm]
    vbox_floor = 95.0           # 바닥 z
    vbox_top = 290.0            # 벽 윗면 z (정답 높이 = top − floor)
    vbox_lat = (-200.0, 200.0)  # 벽 옆 범위 y (정답 너비 = 차이)
    vbox_stiff = 2.0            # 가상 벽/바닥 강성 [N/mm] (wall_size 기본 20 은 접촉 판정 지연 ~0.2s 동안 40N 을 넘음)

    sample = 0.02
    probe_timeout = 15.0        # 찍기 1회 최대 시간 (넘으면 에러: 힘 제어로 안 움직임)


C = Cfg()


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


class TapError(RuntimeError):
    pass


class SignalFilter:
    """스칼라 신호 실시간 필터 (median: 순간 이상치 제거 / ma: 이동평균 / none)."""

    def __init__(self, kind, win):
        self.kind = kind
        self.buf = deque(maxlen=1 if kind == 'none' else max(1, int(win)))

    def __call__(self, v):
        self.buf.append(float(v))
        if self.kind == 'median':
            return float(np.median(self.buf))
        return float(np.mean(self.buf))


def stats(values):
    v = np.asarray([x for x in values if x is not None], float)
    if len(v) == 0:
        return None
    return {'median': float(np.median(v)), 'mean': float(np.mean(v)),
            'std': float(np.std(v)), 'min': float(np.min(v)), 'max': float(np.max(v)), 'n': int(len(v))}


# =============================================================================
# 입출력: wall_size.SizeIO 에 '방향 힘 제어' 만 추가
# =============================================================================
class TapIO(ws.SizeIO):
    MOTION_COLS = ['t', 'seg', 'ctx', 'x', 'y', 'z', 'fx', 'fy', 'fz', 'mx', 'my', 'mz',
                   'f_abs', 'm_abs', 'f_dev', 'm_dev', 'j1', 'j2', 'j3', 'j4', 'j5', 'j6']

    def __init__(self, *a, **k):
        super().__init__(*a, **k)
        self.mlog = []                      # 이동 중 힘 기록 (log_motion)
        self.on_sample = None               # 감시 샘플마다 호출 (pos, phase) → RViz 경로
        self._seg = 0                       # 이동 구간 번호
        self._t_log0 = time.time()

    def _mlog(self, ctx, pos, trip=False):
        """가드가 방금 본 wrench 를 한 줄 기록 (관절값 포함). trip=True: 충돌 판정된 샘플."""
        g = self.guard
        if not C.log_motion or g is None or g.last_w is None:
            return
        try:
            q = [float(v) for v in self.dsr.get_current_posj()[:6]]
        except Exception:
            q = [float('nan')] * 6
        w = [float(v) for v in g.last_w[:6]]
        p = [float(v) for v in (pos if pos is not None else [np.nan] * 3)[:3]]
        self.mlog.append([round(time.time() - self._t_log0, 3), self._seg, ctx + ('!TRIP' if trip else '')]
                         + [round(v, 2) for v in p] + [round(v, 3) for v in w]
                         + [round(float(np.linalg.norm(w[:3])), 2), round(float(np.linalg.norm(w[3:])), 3),
                            round(g.last_dev[0], 2), round(g.last_dev[1], 3)] + [round(v, 2) for v in q])

    def _guarded(self, pos, expect, ctx):
        """가드 검사 + 기록. 충돌이면 그 샘플도 기록하고 예외를 그대로 올림."""
        try:
            self.guard.check(pos, None, expected=expect, context=ctx)
        except Exception:
            self._mlog(ctx, pos, trip=True)
            raise
        self._mlog(ctx, pos)
        if self.on_sample is not None and pos is not None:
            self.on_sample(pos, 'move')

    def zero_force(self, samples=10):
        super().zero_force(samples)
        if self.guard is not None:
            self.guard.free_reset()         # 측정값 기준이 바뀜 → 기준선 다시
            if C.log_motion:
                self.mlog.append([round(time.time() - self._t_log0, 3), self._seg, 'ZERO'] + [''] * 19)

    # 탐침 보정: 컨트롤러 TCP(그리퍼 끝)에서 공구 z 방향으로 probe_len 더 나간 점을 '위치'로 쓴다.
    #   명령: 탐침 끝 목표 → TCP = 목표 − probe_len·공구z   /   읽기: TCP + probe_len·공구z
    #   (컨트롤러 TCP 를 바꾸지 않으므로 티칭 펜던트 등록값을 몰라도 됨. 자세 회전도 탐침 끝 기준)
    def _posx(self, xyz):
        p = np.asarray(xyz, float)[:3] - C.probe_len * ws._tool_dir(ws.TOOL_ORI)
        return self.dsr.posx(*[float(v) for v in p], *ws.TOOL_ORI)

    def get_pos(self):
        posx, _ = self.dsr.get_current_posx(ref=self.dsr.DR_BASE)
        p = np.array(posx[:3], dtype=float)
        return p + C.probe_len * ws._tool_dir(posx[3:6])

    def joint_at(self, xyz, sol_space, tol=2.0):
        """SizeIO.joint_at 과 같음. fkin 결과(컨트롤러 TCP)에 탐침 길이를 더해 목표(탐침 끝)와 비교."""
        q = self.dsr.ikin(self._posx(xyz), sol_space, ref=self.dsr.DR_BASE)
        try:
            q = [float(v) for v in q]
        except TypeError:
            return None
        if len(q) < 6 or not all(np.isfinite(q)):
            return None
        p = self.dsr.fkin(self.dsr.posj(*q[:6]), ref=self.dsr.DR_BASE)
        try:
            p = [float(v) for v in p]
        except TypeError:
            return None
        if len(p) < 6:
            return None
        tip = np.asarray(p[:3]) + C.probe_len * ws._tool_dir(p[3:6])
        if np.linalg.norm(tip - np.asarray(xyz, float)[:3]) > tol:
            return None
        return q

    def amove_l(self, xyz, vel, acc):
        """직선 이동 명령 (회전 속도/가속도를 rot_vel/rot_acc 로, 가속도는 move_acc 이하로)."""
        self._check(self.dsr.amovel(self._posx(xyz), vel=[vel, C.rot_vel],
                                    acc=[min(acc, C.move_acc), C.rot_acc],
                                    ref=self.dsr.DR_BASE), 'amovel')

    def move_l(self, xyz, vel=None, acc=None, expect=()):
        """SizeIO.move_l 과 같음 (가드 감시). 회전이 느려진 만큼 시간 초과를 넉넉히."""
        vel = C.move_vel if vel is None else vel
        acc = C.move_acc if acc is None else acc
        if self.guard is None:
            return self.move_l_unguarded(xyz, vel, acc)
        target = np.asarray(xyz, float)[:3]
        self._seg += 1
        self.amove_l(target, vel, acc)
        timeout = (np.linalg.norm(target - self.get_pos()) / max(vel, 1e-3) * 3.0
                   + 180.0 / max(C.rot_vel, 1e-3) * 1.5 + 5.0)
        t0 = time.time()
        while True:
            pos = self.get_pos()
            self._guarded(pos, expect, 'move' if not expect else 'move_c')
            if time.time() - t0 > 0.3 and not self.is_moving():
                break
            if time.time() - t0 > timeout:
                self.stop()
                raise RuntimeError(f'이동 시간 초과 (목표 {np.round(target, 1).tolist()})')
            time.sleep(ws.SAMPLE_PERIOD)
        if self.on_move is not None:
            self.on_move(xyz)

    def start_force(self, d, start, depth):
        """d 방향으로 밀기 시작. real = 순응제어 + 힘 제어, virtual = 저속 amovel."""
        if self.mode == 'virtual':
            # 가상 접촉은 위치로만 흉내 → 끝점이 팔이 닿지 않는 곳이면 amovel 이 안 움직임 → 닿는 거리까지 줄임
            start = np.asarray(start, float)
            L = depth + 5.0
            try:
                sol = self.dsr.get_solution_space(self.dsr.get_current_posj())
            except Exception:
                sol = None
            if isinstance(sol, int) and sol >= 0:
                while L > 10.0 and self.joint_at(start + d * L, sol) is None:
                    L *= 0.8
            self.amove_l(start + d * L, C.probe_vel, ws.PROBE_ACC)
            return
        dsr = self.dsr
        axis = int(np.argmax(np.abs(d)))          # BASE 축 방향만 지원 (X/Y/Z)
        k = [C.stiff_other] * 3 + [C.stiff_rot] * 3
        k[axis] = C.stiff_soft
        dsr.task_compliance_ctrl(k)
        self.compliance_on = True
        time.sleep(0.2)
        fd = [0.0] * 6
        mask = [0] * 6
        fd[axis] = float(np.sign(d[axis]) * C.force)
        mask[axis] = 1
        dsr.set_desired_force(fd, mask, mod=dsr.DR_FC_MOD_REL)
        self.force_on = True

    def move_j_reorient(self, q, vel=None, acc=None):
        """자세 바꾸기용 관절 이동 (가드 감시). reorient_step 도씩 나눠 움직이고 멈출 때마다 힘 영점을 다시 잡는다.
        자세가 크게 바뀌면 공구 무게가 힘/모멘트 추정값에 다르게 실려(티칭 펜던트 Tool 무게와 실제가 다르면 더 큼)
        접촉이 없어도 수 N / 수 Nm 씩 커져 가드가 '충돌'로 오판한다. 관절 직선 보간이라 경로는 한 번에 movej 와 같음."""
        vel = C.joint_vel if vel is None else vel
        acc = C.joint_acc if acc is None else acc
        q = np.array([float(v) for v in q[:6]])
        cur = np.array([float(v) for v in self.dsr.get_current_posj()[:6]])
        span = float(np.max(np.abs(q - cur)))
        n = max(1, int(np.ceil(span / max(C.reorient_step, 1.0))))
        for i in range(1, n + 1):
            qi = cur + (q - cur) * i / n
            self._seg += 1
            self._check(self.dsr.amovej(self.dsr.posj(*qi.tolist()), vel=vel, acc=acc), 'amovej')
            timeout = span / n / max(vel, 1e-3) * 3.0 + 10.0
            t0 = time.time()
            while True:
                if self.guard is not None:
                    self._guarded(self.get_pos(), (), f'movej {i}/{n}')
                if time.time() - t0 > 0.3 and not self.is_moving():
                    break
                if time.time() - t0 > timeout:
                    self.stop()
                    raise RuntimeError(f'관절 이동 시간 초과 (목표 {np.round(qi, 1).tolist()})')
                time.sleep(ws.SAMPLE_PERIOD)
            time.sleep(0.3)                                 # 멈춘 뒤 흔들림 가라앉히기
            self.zero_force()                               # 이 자세의 공구 무게 영향을 영점으로

    def end_force(self):
        """힘 제어 → 순응제어 순서로 해제 (virtual 은 정지)."""
        if self.mode == 'virtual':
            self.stop()
        else:
            self.release_press()
        self.wait_motion_done(3.0)


# =============================================================================
# 측정 로직
# =============================================================================
class WallTapper:
    def __init__(self, io):
        self.io = io
        self.log = io.node.get_logger()
        self.taps = []              # 찍기 요약
        self.traces = []            # 찍기별 시계열 (taps 와 같은 순서)
        self.floor_z = None
        self.face = None            # 마지막으로 닿은 벽 표면 (벽 방향 거리)
        self.last_onset = None      # 마지막 찍기의 접촉 시작 좌표
        self.lat_c = float(C.start[ws.LAT_AXIS])
        self.noise = None           # {'mean': [6], 'std': [6], 'cov': 3x3}
        self.height_lines = []
        self.width_lines = []
        self.res = {}
        self.phase_time = {}
        self.status, self.error = 'running', None
        self.t_start = None
        self.viz = None             # WallSizeViz (RViz 마커)
        self.path = []              # 탐침 끝 경로 [[x, y, z, phase], ...]
        io.on_sample = self.trace
        io.on_move = lambda xyz: self.trace(xyz, 'move', publish=True)

    # ------------------------------------------------------------ RViz
    def trace(self, pos, phase, publish=False):
        """경로 점 추가 (1mm 이상 움직였을 때) + RViz 갱신 표시."""
        pos = [float(v) for v in pos[:3]]
        if not self.path or np.linalg.norm(np.subtract(pos, self.path[-1][:3])) >= ws.PATH_MIN_MM:
            self.path.append(pos + [phase])
        if self.viz is not None:
            if publish:
                self.publish_viz()
            else:
                self.viz.mark_dirty()

    def publish_viz(self):
        if self.viz is None:
            return
        try:
            self.viz.publish(self.scene(), force=True)
        except Exception as e:
            self.log.warn(f'RViz 퍼블리시 실패: {e}')

    def scene(self, with_tool=True):
        """wall_size_rviz 형식의 장면 (경로, 찍은 점, 높이/너비 선, 바닥, 가상 벽 정답, 현재 공구)."""
        face = self.face if self.face is not None else ws.app_of(C.start)
        pts = [{'kind': 'floor' if t['kind'] == 'floor' else 'probe', 'xyz': t['xyz']}
               for t in list(self.taps) if t['hit'] and t['xyz']]
        lines = []
        if self.floor_z is not None:
            for ln in list(self.height_lines):
                if ln.get('ok'):
                    lines.append({'kind': 'height', 'value': ln['height'],
                                  'p0': ws.make_point(ln['lat'], face, self.floor_z).tolist(),
                                  'p1': ws.make_point(ln['lat'], face, ln['top_z']).tolist()})
        for ln in list(self.width_lines):
            if ln.get('ok'):
                lines.append({'kind': 'width', 'value': ln['width'],
                              'p0': ws.make_point(ln['lat_minus'], face, ln['up']).tolist(),
                              'p1': ws.make_point(ln['lat_plus'], face, ln['up']).tolist()})
        tool = None
        if with_tool and self.path:
            tool = {'pos': self.path[-1][:3], 'dir': ws._tool_dir(ws.TOOL_ORI).tolist()}
        h, w = self.res.get('height'), self.res.get('width')
        return {'axes': {'lat': ws.LAT_AXIS, 'approach': ws.APPROACH_AXIS,
                         'approach_sign': ws.APPROACH_SIGN, 'up': ws.UP_AXIS},
                'path': list(self.path), 'points': pts, 'lines': lines,
                'floor_z': self.floor_z, 'face': face, 'lat_range': list(C.lat_range), 'tool': tool,
                'truth': ws.VIRTUAL_BOX if self.io.mode == 'virtual' else None,
                'summary': {'total_height': h['median'] if h else None,
                            'total_width': w['median'] if w else None}}

    # ------------------------------------------------------------ 노이즈 / 임계값
    def calibrate_noise(self):
        """접촉 없는 정지 상태에서 힘 노이즈 측정 → 접촉 임계값 μ + kσ 의 σ."""
        io = self.io
        io.zero_force()
        vals = []
        for _ in range(C.noise_samples):
            vals.append(np.asarray(io.get_force(), float)[:6])
            time.sleep(C.sample)
        F = np.array(vals)
        self.noise = {'mean': F.mean(axis=0).tolist(), 'std': F.std(axis=0).tolist(),
                      'cov': np.cov(F[:, :3].T).tolist(), 'n': len(F)}
        self.log.info(f'[노이즈] 힘 σ = {np.round(F.std(axis=0)[:3], 2).tolist()} N, '
                      f'모멘트 σ = {np.round(F.std(axis=0)[3:], 3).tolist()} Nm '
                      f'→ 임계값 바닥 {self.threshold(-ws.UP_DIR):.2f}N / 벽 {self.threshold(ws.APP_DIR):.2f}N')

    def threshold(self, d):
        sigma = 0.0
        if self.noise is not None:
            sigma = float(np.sqrt(max(0.0, d @ np.asarray(self.noise['cov']) @ d)))
        th = max(C.contact, C.k_sigma * sigma)
        if self.io.mode == 'real' and th >= 0.8 * C.force:
            raise TapError(f'접촉 임계값 {th:.1f}N 이 지령 힘 {C.force:.0f}N 에 너무 가까움 '
                           '→ force 를 키우거나 k_sigma 를 줄일 것 (노이즈가 큼)')
        return th

    # ------------------------------------------------------------ 찍기 1회
    def probe(self, start, direction, depth, kind):
        """start 에서 direction 으로 depth 까지 힘 제어로 전진. Returns: 닿은 좌표 또는 None."""
        io = self.io
        d = unit(direction)
        start = np.asarray(start, float)
        self._travel_to(start, d)                 # 접촉 없는 이동 (가드: 작은 힘도 충돌)
        io.zero_force()
        th_in = self.threshold(d)
        th_out = C.hyst * th_in
        filt = SignalFilter(C.filter, C.filter_win)

        hit, travel, peak, latency = None, 0.0, 0.0, None
        hist, trace = deque(), []
        t_cross = t_on = onset_pos = t_ref = None
        t0 = time.time()
        io.start_force(d, start, depth)
        try:
            while True:
                now = time.time()
                pos = io.get_pos()
                w = self._sense(pos, d, kind)
                raw = abs(float(np.dot(w[:3], d)))
                f = filt(raw)
                peak = max(peak, raw)
                travel = float(np.dot(pos - start, d))
                if C.log_trace:
                    trace.append([round(now - t0, 4), *np.round(pos, 3).tolist(),
                                  *np.round(w[:6], 3).tolist(), round(f, 3)])

                # 지연시간 기준: 필터 전 신호가 처음 임계값을 넘은 시각
                t_cross = (t_cross or now) if raw >= th_in else None
                # 히스테리시스 접촉 상태
                if t_on is None:
                    if f >= th_in:
                        t_on, onset_pos, t_ref = now, pos.copy(), t_cross or now
                elif f < th_out:
                    t_on = onset_pos = None

                hist.append((now, pos))
                while len(hist) > 2 and now - hist[1][0] >= C.stall_sec:
                    hist.popleft()
                span = now - hist[0][0]
                moved = float(np.linalg.norm(pos - hist[0][1]))
                stalled = span >= C.stall_sec * 0.9 and moved < C.stall_mm

                if t_on is not None and now - t_on >= C.contact_hold and (io.mode == 'virtual' or stalled):
                    hit = onset_pos if io.mode == 'virtual' else pos.copy()   # real: 힘을 풀기 전 좌표
                    self.last_onset = onset_pos.copy()  # 접촉 시작 좌표 (휘는 벽이면 hit 보다 앞)
                    latency = now - t_ref
                    break
                if travel > depth:
                    break                           # 안 닿음
                if io.mode == 'real' and span > 0.05 and moved / span > C.max_fc_speed:
                    raise TapError(f'힘 제어 속도 {moved / span:.0f}mm/s > {C.max_fc_speed:.0f} → 중단 '
                                   '(force 를 줄이거나 stiff_soft 를 높일 것)')
                if now - t0 > C.probe_timeout:
                    if t_on is None and travel > depth - C.max_depth / 2:
                        break                       # 표면보다 max_depth/2 넘게 들어갔는데 힘 없음 → 안 닿음 (끝 너머에서 느리게 진행)
                    raise TapError(f'찍기 시간 초과 ({kind}, 진행 {travel:.1f}/{depth:.0f}mm) '
                                   '→ 힘 제어로 움직이지 않음. force 를 확인')
                time.sleep(C.sample)
        finally:
            io.end_force()

        rec = {'i': len(self.taps), 'kind': kind, 't': round(t0 - (self.t_start or t0), 3),
               'start': start.round(2).tolist(), 'dir': d.round(3).tolist(),
               'hit': hit is not None, 'xyz': None if hit is None else hit.round(2).tolist(),
               'travel': round(travel, 2), 'duration': round(time.time() - t0, 3),
               'th_in': round(th_in, 3), 'peak': round(peak, 3),
               'latency': None if latency is None else round(latency, 3)}
        self.taps.append(rec)
        self.traces.append(trace)
        self.log.info(f'  [{kind}] {"닿음 " + str(rec["xyz"]) if hit is not None else "안 닿음"} '
                      f'(진행 {travel:.1f}mm, 최대 {peak:.1f}N'
                      + (f', 지연 {latency * 1000:.0f}ms)' if latency is not None else ')'))
        # 시작점으로 후퇴 (닿아 있던 방향의 힘은 정상으로 허용)
        io.move_l(start, vel=C.move_vel / 2, expect=(d,))
        return hit

    def _travel_to(self, start, d):
        """다음 찍기 시작점으로 이동. 옆/위로 움직여야 하면 진행 방향(d) 반대로 travel_clear 만큼
        더 물러난 선을 따라 옮긴 뒤 시작점으로 들어간다 (손끝/몸체가 표면을 긁지 않게)."""
        io = self.io
        cur = io.get_pos()
        delta = start - cur
        side = delta - d * float(np.dot(delta, d))
        if C.travel_clear > 0 and np.linalg.norm(side) > 1.0:
            back = min(float(np.dot(cur, d)), float(np.dot(start, d))) - C.travel_clear
            via1 = cur + d * (back - float(np.dot(cur, d)))
            via2 = start + d * (back - float(np.dot(start, d)))
            io.move_l(via1, vel=C.move_vel, expect=(d,))   # 방금 닿았던 쪽에서 떨어지는 중
            io.move_l(via2, vel=C.move_vel)
        io.move_l(start, vel=C.move_vel)

    def _sense(self, pos, d, kind):
        self.trace(pos, 'seek')
        if self.io.guard is None:
            return np.asarray(self.io.get_force(pos), float)
        return np.asarray(self.io.guard.check(pos, None, expected=(d,), context=kind), float)

    def tap_wall(self, lat, up, kind):
        """(옆, 높이) 에서 벽 앞 STANDOFF 부터 벽 방향으로 찍기. 닿으면 표면 갱신."""
        start = ws.make_point(lat, self.face - C.standoff, up)
        self.last_onset = None
        hit = self.probe(start, ws.APP_DIR, C.standoff + C.max_depth, kind)
        if hit is not None:
            # 다음 시작점 기준: 접촉 시작 좌표 (real 은 10N 으로 밀려 들어간 hit 보다 앞 → 휘는 벽에서도 안전)
            ref = self.last_onset if self.last_onset is not None else hit
            self.face = min(ws.app_of(ref), ws.app_of(hit))   # app_of: 클수록 벽 쪽
        return hit is not None

    # ------------------------------------------------------------ 끝점 찾기
    def find_edge(self, tap, s0, step, sign, limit, name, hint=None):
        """s0(닿음 확인된 위치)에서 sign 방향으로 step 씩 찍다가 연속 miss_confirm 번 안 닿으면
        (마지막 닿음, 첫 안 닿음) 구간을 tol 까지 이분탐색. Returns: (끝점, 마지막 닿음, 첫 안 닿음).
        hint: 다른 선에서 찾은 끝점 → 그 한 칸 안쪽이 닿으면 거기서 시작 (찍기 횟수 절약)."""
        if hint is not None:
            inner = hint - sign * step
            if (inner - s0) * sign > 0 and (limit - inner) * sign > 0 and tap(inner):
                s0 = inner
        last_hit, first_miss, misses = s0, None, 0
        s = s0
        while misses < C.miss_confirm:
            s += sign * step
            if (s - limit) * sign > 0:
                if first_miss is None:
                    raise TapError(f'{name}: 한계({limit:.0f})까지 벽 끝을 못 찾음 → 한계 값을 넓힐 것')
                break
            if tap(s):
                last_hit, first_miss, misses = s, None, 0
            else:
                first_miss = s if first_miss is None else first_miss
                misses += 1
        lo, hi = last_hit, first_miss
        while abs(hi - lo) > C.tol:
            mid = (lo + hi) / 2.0
            if tap(mid):
                lo = mid
            else:
                hi = mid
        edge = (lo + hi) / 2.0
        self.log.info(f'[{name}] 끝 = {edge:.1f} (닿음 {lo:.1f} / 안 닿음 {hi:.1f})')
        return edge, lo, hi

    # ------------------------------------------------------------ 특이점 점검
    def check_singularity(self, points, label):
        io = self.io
        try:
            sol = io.dsr.get_current_solution_space()
            if not isinstance(sol, int) or sol < 0 or io.joint_at(io.get_pos(), sol) is None:
                self.log.warn(f'[특이점 점검:{label}] 역기구학 검산 불가 → 건너뜀')
                return
        except Exception as e:
            self.log.warn(f'[특이점 점검:{label}] 실패 → 건너뜀: {e}')
            return
        bad, unreachable = [], []
        for p in points:
            q = io.joint_at(p, sol)
            if q is None:
                unreachable.append(np.round(p, 0).tolist())
            elif abs(q[4]) < C.sing_deg:
                bad.append((np.round(p, 0).tolist(), round(q[4], 1)))
        if unreachable:
            raise TapError(f'[특이점 점검:{label}] 도달 불가 위치 {len(unreachable)}개 '
                           f'(예: {unreachable[0]}) → 범위/시작 위치 확인')
        if bad:
            msg = (f'[특이점 점검:{label}] |J5| < {C.sing_deg}° 위치 {len(bad)}개 '
                   f'(예: {bad[0][0]} J5={bad[0][1]}°) → tilt 를 키우거나 시작 위치를 옮길 것')
            if C.sing_abort:
                raise TapError(msg + ' (sing_abort:=false 로 경고만 가능)')
            self.log.warn(msg)
        else:
            self.log.info(f'[특이점 점검:{label}] OK ({len(points)}점)')

    def _path_points(self, lats, ups):
        pts = []
        for lat in lats:
            for up in ups:
                for app in (self.face - C.standoff, self.face + C.max_depth):
                    pts.append(ws.make_point(lat, app, up))
        return pts

    def _phase(self, name, fn):
        t = time.time()
        try:
            return fn()
        finally:
            self.phase_time[name] = round(time.time() - t, 2)

    # ------------------------------------------------------------ 단계
    def measure_floor(self):
        self.log.info('[1] 바닥 찍기 (-Z)')
        start = np.asarray(C.start, float)
        hit = self.probe(start, -ws.UP_DIR, C.floor_depth, 'floor')
        if hit is None:
            raise TapError(f'시작 위치에서 {C.floor_depth:.0f}mm 내려가도 바닥이 없음')
        self.floor_z = float(hit[ws.UP_AXIS])
        self.log.info(f'    floor_z = {self.floor_z:.1f}')

    def find_wall(self):
        self.log.info('[2] 첫 벽 찾기')
        z0 = self.floor_z + C.lift
        start = ws.make_point(self.lat_c, ws.app_of(C.start), z0)
        hit = self.probe(start, ws.APP_DIR, C.wall_search, 'wall_seek')
        if hit is None:
            raise TapError(f'벽 방향으로 {C.wall_search:.0f}mm 가도 벽이 없음')
        self.face = ws.app_of(hit)
        self.log.info(f'    벽 표면 = {self.face:.1f} (z={z0:.1f})')

    def measure_height(self):
        z0 = self.floor_z + C.lift
        lats = [self.lat_c + o for o in C.height_lats]
        self.log.info(f'[3] 높이 측정 (위로 찍기) - 옆 위치 {[round(v, 1) for v in lats]}')
        ups = np.arange(z0, C.z_limit + 1e-6, max(C.step_z, 20.0))
        self.check_singularity(self._path_points(lats, ups), 'height')
        hint = None
        for lat in lats:
            line = {'lat': lat, 'ok': False}
            self.height_lines.append(line)
            if not (C.lat_range[0] <= lat <= C.lat_range[1]):
                line['reason'] = 'lat_range 밖'
                continue
            if not self.tap_wall(lat, z0, 'height_base'):
                line['reason'] = f'z={z0:.1f} 에서 벽이 안 닿음'
                self.log.warn(f'[높이] 옆 {lat:.1f}: {line["reason"]} → 건너뜀')
                continue
            n0 = len(self.taps)
            top, lo, hi = self.find_edge(lambda z, lat=lat: self.tap_wall(lat, z, 'height'),
                                         z0, C.step_z, +1, C.z_limit, f'높이 @{lat:.0f}', hint)
            hint = top
            line.update(ok=True, top_z=top, height=top - self.floor_z, err=(hi - lo) / 2,
                        taps=len(self.taps) - n0 + 1)
        st = stats([ln.get('height') for ln in self.height_lines if ln['ok']])
        if st is None:
            raise TapError('높이를 잰 선이 하나도 없음')
        self.res['height'] = st
        self.res['top_z'] = self.floor_z + st['median']
        self.log.info(f'    높이 중앙값 {st["median"]:.1f}mm (σ {st["std"]:.2f}, n={st["n"]})')

    def side_ori(self, side_sign):
        """너비 측정 자세 (ZYZ). 공구 축을 X-Y 평면과 평행(width_tilt=0)하게 두고 벽 방향에서
        Z축 둘레로 width_yaw 만큼 돌린다. side_sign: 찾는 쪽의 옆 축 부호.
          width_lean=toward: 찾는 쪽으로 (기준: +x 벽, 왼쪽(+Y) 찾을 때 [45, 90, 90])
          width_lean=away  : 찾는 쪽 반대로
        width_roll=90 → 공구 y 축이 +Z(위), 공구 x 축은 수평 (공구 축이 X/Y/Z 와 평행)."""
        yaw = np.radians(C.width_yaw)
        lean = side_sign * (-1 if C.width_lean == 'away' else 1)
        horiz = np.cos(yaw) * ws.APP_DIR + np.sin(yaw) * lean * ws.LAT_DIR
        a = float(np.degrees(np.arctan2(horiz[1], horiz[0])))
        # 롤도 대칭: 기준 자세(+90)의 거울상은 -90 → 왼쪽 기준 관절(J6 ≈ +30)과 맞아 J6 를 180° 돌리지 않음
        if abs(np.sin(yaw)) < 1e-6:
            roll = C.width_roll                             # 정면: 양쪽 같은 자세 (기준 관절 배치, 공구 y = +Z)
        else:
            roll = C.width_roll * (1 if lean * float(np.dot(np.cross(ws.APP_DIR, ws.LAT_DIR), ws.UP_DIR)) > 0 else -1)
        return [round(a, 3), 90.0 + C.width_tilt, roll]

    def set_tool_ori(self, ori, label, ref_j=None):
        """벽에서 rot_clear 만큼 더 물러난 뒤 그 자리(TCP 고정)에서 공구 자세만 바꾼다.
        ref_j(+x 기준 관절값)가 있으면 그 관절 배치(solution space)로 ikin → movej
        (movel 은 지금 손목 배치를 그대로 끌고 가서 손목이 바닥 쪽으로 갈 수 있음)."""
        io = self.io
        back = io.get_pos() - ws.APP_DIR * C.rot_clear
        io.move_l(back)
        old = list(ws.TOOL_ORI)
        self.log.info(f'[자세] {label}: TOOL_ORI {old} → {list(ori)}')
        if ref_j:
            ref = [float(v) for v in ref_j]
            ref[0] += ws.DIRECTIONS[ws.DIRECTION][3]        # 벽 방향에 맞춰 J1 회전
            sol = io.dsr.get_solution_space(io.dsr.posj(*ref))
            for attempt in range(4):
                q = self._ref_ik(back, ori, sol, ref)
                if q is None:
                    raise TapError(f'[자세] {label}: 기준 관절 배치(sol {sol})로 {np.round(back, 1).tolist()} '
                                   f'{ori} 에 도달 불가 → width_ref_j_* 또는 시작 위치 확인')
                low, far = self._sweep(io.dsr.get_current_posj(), q)
                lift = (self.floor_z + C.flip_clear) - low          # 바닥까지 여유 부족분
                retreat = far - (self.face - C.flip_clear)          # 벽까지 여유 부족분
                self.log.info(f'[자세] 관절 경로 점검: 탐침 끝 최저 z {low:.0f} (바닥 {self.floor_z:.0f}), '
                              f'벽 쪽 최대 {far:.0f} (벽 {self.face:.0f})')
                if lift <= 1.0 and retreat <= 1.0:              # 1mm 이내면 충분 (반복해도 0 으로 수렴 안 함)
                    break
                back = back + ws.UP_DIR * max(lift, 0.0) - ws.APP_DIR * max(retreat, 0.0)
                if float(np.dot(back, ws.UP_DIR)) > C.reorient_z_max:
                    raise TapError(f'[자세] {label}: 손목을 돌리는 동안 바닥에 닿지 않으려면 '
                                   f'z {np.dot(back, ws.UP_DIR):.0f} 까지 올려야 함 (> reorient_z_max {C.reorient_z_max:.0f})')
                self.log.info(f'[자세] 여유 부족 → 올림 {max(lift, 0):.0f}mm / 물러남 {max(retreat, 0):.0f}mm '
                              f'→ {np.round(back, 1).tolist()} 에서 다시 계산')
                io.move_l(back)                             # 아직 이전 자세 (ws.TOOL_ORI = old)
            else:
                raise TapError(f'[자세] {label}: 안전한 자세 바꾸기 위치를 못 찾음')
            ws.TOOL_ORI = list(ori)                         # SizeIO._posx 가 이 값을 사용
            self.log.info(f'[자세] movej (sol {sol}, 기준 {np.round(ref, 1).tolist()}) → '
                          f'{np.round(q, 1).tolist()}')
            io.move_j_reorient(q)
        else:
            ws.TOOL_ORI = list(ori)
            io.move_l(back, vel=C.move_vel / 2)
        io.zero_force()                                     # 자세가 바뀌어 공구 무게 영향이 달라짐 → 다시 영점

    def _ref_ik(self, xyz, ori, sol, ref):
        """ori 자세로 xyz 에 가는 관절값 (sol 배치). J4/J6 은 ±360° 중 기준(ref)에 가까운 값으로
        (예: J6 270 과 −90 은 같은 자세지만 관절 이동량이 다름). 못 가면 None."""
        old = ws.TOOL_ORI
        ws.TOOL_ORI = list(ori)
        try:
            q = self.io.joint_at(xyz, sol)
        finally:
            ws.TOOL_ORI = old
        if q is None:
            return None
        q = list(q)
        for j in (3, 5):
            v = q[j] + 360.0 * round((ref[j] - q[j]) / 360.0)
            if abs(v) <= 360.0:
                q[j] = v
        return q

    def _sweep(self, q0, q1, n=20):
        """q0 → q1 관절 직선 보간 경로의 탐침 끝 (최저 높이, 벽 쪽 최대 좌표). fkin 으로 계산."""
        dsr = self.io.dsr
        q0 = np.array([float(v) for v in q0[:6]])
        q1 = np.array([float(v) for v in q1[:6]])
        low, far = np.inf, -np.inf
        for s in np.linspace(0.0, 1.0, n + 1):
            p = [float(v) for v in dsr.fkin(dsr.posj(*(q0 + (q1 - q0) * s).tolist()), ref=dsr.DR_BASE)]
            tip = np.asarray(p[:3]) + C.probe_len * ws._tool_dir(p[3:6])
            low = min(low, float(np.dot(tip, ws.UP_DIR)))
            far = max(far, ws.app_of(tip))
        return low, far

    def measure_width(self):
        if C.width_z > 0:
            ups = [C.width_z]
        else:
            H = self.res['height']['median']
            ups = [self.floor_z + r * H for r in C.width_ups]
        # 벽을 바라볼 때 오른쪽 = 벽 방향 × 위 (+x 방향이면 −Y)
        r_sign = 1 if float(np.dot(np.cross(ws.APP_DIR, ws.UP_DIR), ws.LAT_DIR)) > 0 else -1
        ax = ws.AX_NAME[ws.LAT_AXIS]
        sides = [(r_sign, 'right', '오른쪽'), (-r_sign, 'left', '왼쪽')]
        if C.width_first == 'left':
            sides.reverse()
        (s0, _, n0_), (_, _, n1_) = sides
        self.log.info(f'[4] 너비 측정 - 높이 {[round(v, 1) for v in ups]}, '
                      f'{n0_}({"+" if s0 > 0 else "-"}{ax}) 먼저 → {n1_}')
        lines = [{'up': z, 'ok': False, 'taps': 0, 'edges': {}} for z in ups]
        faces = {}
        ref_used = None
        for k, (sign, key, name) in enumerate(sides):
            if C.width_ori == 'lean':
                ori = self.side_ori(sign)
                if key == 'left' and abs(C.width_yaw) < 1e-6:
                    ori[2] = C.width_roll_left
                ref_j = tuple(C.width_ref_j_left if key == 'left' else C.width_ref_j_right)
                if np.allclose(ori, ws.TOOL_ORI, atol=1e-3) and (not ref_j or ref_j == ref_used):
                    self.log.info(f'[자세] 너비 {name}: 이전과 같은 자세/관절 배치 {ws.TOOL_ORI} → 그대로 탐색')
                else:
                    self.set_tool_ori(ori, f'너비 {name}: 수평, Z축 둘레 {C.width_yaw:.0f}° ({C.width_lean})',
                                      ref_j)
                    ref_used = ref_j
                self.res[f'tool_ori_width_{key}'] = list(ws.TOOL_ORI)
            limit = C.lat_range[1] if sign > 0 else C.lat_range[0]
            lats = np.arange(self.lat_c, limit + sign * 1e-6, sign * 40.0)
            self.check_singularity(self._path_points(lats, ups), f'width_{key}')
            hint = None
            for i, z in enumerate(ups):
                line = lines[i]
                n0 = len(self.taps)
                if k == 0:                                  # 첫 쪽에서만 가운데를 찍어 그 높이의 벽 표면 확인
                    if not self.tap_wall(self.lat_c, z, 'width_center'):
                        line['reason'] = f'z={z:.1f} 가운데에서 벽이 안 닿음'
                        self.log.warn(f'[너비] {line["reason"]} → 건너뜀')
                        continue
                    faces[i] = self.face
                elif i not in faces:
                    continue
                self.face = faces[i]                        # 가운데 기준으로 시작
                edge, lo, hi = self.find_edge(lambda s, z=z: self.tap_wall(s, z, f'width_{key}'),
                                              self.lat_c, C.step_y, sign, limit, f'너비 {name} @z{z:.0f}', hint)
                hint = edge
                line['edges'][key] = {'edge': edge, 'lo': lo, 'hi': hi}
                line['taps'] += len(self.taps) - n0
        for line in lines:
            e = line['edges']
            if 'right' in e and 'left' in e:
                lo_e, hi_e = sorted([e['right']['edge'], e['left']['edge']])
                line.update(ok=True, lat_minus=lo_e, lat_plus=hi_e,
                            dist_right=abs(e['right']['edge'] - self.lat_c),
                            dist_left=abs(e['left']['edge'] - self.lat_c), width=hi_e - lo_e,
                            err=sum(abs(v['hi'] - v['lo']) for v in e.values()) / 2)
            self.width_lines.append(line)
        st = stats([ln.get('width') for ln in self.width_lines if ln['ok']])
        if st is None:
            raise TapError('너비를 잰 선이 하나도 없음')
        self.res['width'] = st
        self.log.info(f'    너비 중앙값 {st["median"]:.1f}mm (σ {st["std"]:.2f}, n={st["n"]})')

    def run(self):
        self.t_start = time.time()
        self._phase('floor', self.measure_floor)
        self._phase('wall', self.find_wall)
        if C.width_z <= 0:
            self._phase('height', self.measure_height)
        self._phase('width', self.measure_width)
        self.status = 'ok'

    # ------------------------------------------------------------ 결과
    def truth(self):
        th, tw = C.true_height, C.true_width
        if self.io.mode == 'virtual':
            b = ws.VIRTUAL_BOX
            th = th or b['top'] - b['floor']
            tw = tw or b['lat'][1] - b['lat'][0]
        return (th or None), (tw or None)

    def metrics(self):
        th, tw = self.truth()
        hits = [t for t in self.taps if t['hit']]
        lat = [t['latency'] for t in hits if t['latency'] is not None]
        m = {'total_time': round(time.time() - self.t_start, 2) if self.t_start else None,
             'phase_time': self.phase_time, 'n_taps': len(self.taps), 'n_hits': len(hits),
             'latency_mean': float(np.mean(lat)) if lat else None,
             'latency_max': float(np.max(lat)) if lat else None,
             'peak_force_max': max((t['peak'] for t in self.taps), default=None),
             'peak_over_force_mean': (float(np.mean([t['peak'] - C.force for t in hits]))
                                      if hits and self.io.mode == 'real' else None),
             'true_height': th, 'true_width': tw}
        for key, true in (('height', th), ('width', tw)):
            st = self.res.get(key)
            if st is not None and true:
                m[f'{key}_abs_err'] = abs(st['median'] - true)
                m[f'{key}_rel_err'] = abs(st['median'] - true) / true
        return m

    def summary(self):
        r = self.res
        mt = self.metrics()
        lines = ['========== 벽 측정 결과 (tap) ==========',
                 f'  상태         : {self.status}' + (f' ({self.error})' if self.error else '')]
        if self.floor_z is not None:
            lines.append(f'  바닥 z       : {self.floor_z:.1f}')
        for ln in self.height_lines:
            if ln['ok']:
                lines.append(f'  높이 @옆{ln["lat"]:7.1f} : {ln["height"]:.1f} (±{ln["err"]:.1f}, 찍기 {ln["taps"]})')
        for ln in self.width_lines:
            if ln['ok']:
                lines.append(f'  너비 @z{ln["up"]:8.1f} : {ln["width"]:.1f} (±{ln["err"]:.1f}, '
                             f'{ln["lat_minus"]:.1f}~{ln["lat_plus"]:.1f}, 찍기 {ln["taps"]})')
        for key, name in (('height', '높이'), ('width', '너비')):
            st = r.get(key)
            if st:
                true = mt.get(f'true_{key}')
                err = f', 오차 {mt[key + "_abs_err"]:.2f}mm (정답 {true:.1f})' if true else ''
                lines.append(f'  {name} 중앙값  : {st["median"]:.1f} mm (σ {st["std"]:.2f}, n={st["n"]}){err}')
        lines.append(f'  찍기 / 시간  : {mt["n_taps"]}회 / {mt["total_time"]}s {mt["phase_time"]}')
        if mt['latency_mean'] is not None:
            lines.append(f'  접촉 지연    : 평균 {mt["latency_mean"] * 1000:.0f}ms, 최대 {mt["latency_max"] * 1000:.0f}ms'
                         f', 최대 힘 {mt["peak_force_max"]:.1f}N')
        for line in lines:
            self.log.info(line)

    def save(self):
        if C.out:
            path = os.path.expanduser(C.out)
        else:
            path = os.path.join(ws.SAVE_DIR, f'tap_{datetime.now():%Y%m%d_%H%M%S}.json')
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        cfg = {k: getattr(C, k) for k in dir(Cfg) if not k.startswith('_')}
        data = {'tag': C.tag, 'mode': self.io.mode, 'status': self.status, 'error': self.error,
                'time': datetime.now().isoformat(timespec='seconds'),
                'direction': ws.DIRECTION, 'tool_ori': ws.TOOL_ORI, 'config': cfg,
                'noise': self.noise, 'floor_z': self.floor_z, 'face': self.face, 'lat_center': self.lat_c,
                'height_lines': self.height_lines, 'width_lines': self.width_lines,
                'results': self.res, 'metrics': self.metrics(), 'taps': self.taps,
                'truth_box': ws.VIRTUAL_BOX if self.io.mode == 'virtual' else None}
        with open(path, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False, default=float)
        msg = f'저장: {path}'
        if C.log_trace and self.traces:
            tpath = os.path.splitext(path)[0] + '_trace.json'
            with open(tpath, 'w') as f:
                json.dump({'columns': ['t', 'x', 'y', 'z', 'fx', 'fy', 'fz', 'mx', 'my', 'mz', 'f_filt'],
                           'taps': [{'i': t['i'], 'kind': t['kind'], 'samples': s}
                                    for t, s in zip(self.taps, self.traces)]},
                          f, separators=(',', ':'))
            msg += f' (+ 시계열 {os.path.basename(tpath)})'
        if C.log_motion and self.io.mlog:
            mpath = os.path.splitext(path)[0] + '_motion.csv'
            with open(mpath, 'w', newline='') as f:
                wr = csv.writer(f)
                wr.writerow(TapIO.MOTION_COLS)
                wr.writerows(self.io.mlog)
            msg += f' (+ 이동 힘 기록 {os.path.basename(mpath)}, {len(self.io.mlog)}줄)'
        self.log.info(msg)


# =============================================================================
# main
# =============================================================================
def _floats(s):
    return tuple(float(v) for v in str(s).strip('[]() ').split(',') if v.strip())


def _declare(node):
    dyn = ParameterDescriptor(dynamic_typing=True)    # 10 / 10.0 / "10" 모두 허용

    def p(name, default):
        return node.declare_parameter(name, default, dyn).value

    C.direction = str(p('dir', C.direction))
    C.start = _floats(p('start', ','.join(str(v) for v in C.start)))
    if len(C.start) != 3:
        raise ValueError(f'start 는 "x,y,z" 형식: {C.start}')
    for name in ('tilt', 'force', 'stall_mm', 'stall_sec', 'max_fc_speed', 'probe_vel', 'move_vel',
                 'move_acc', 'approach_vel', 'approach_acc', 'rot_vel', 'rot_acc', 'joint_vel', 'joint_acc', 'reorient_step',
                 'contact', 'k_sigma', 'hyst', 'contact_hold',
                 'floor_depth', 'wall_search', 'lift', 'standoff', 'max_depth', 'step_z', 'step_y',
                 'tol', 'z_limit', 'travel_clear', 'probe_len', 'width_yaw', 'width_tilt', 'width_roll', 'rot_clear', 'stiff_soft', 'stiff_other', 'stiff_rot', 'sing_deg', 'probe_timeout',
                 'true_height', 'true_width', 'free_tau', 'free_hold', 'free_hard', 'free_mhard',
                 'flip_clear', 'reorient_z_max', 'vbox_face', 'vbox_floor', 'vbox_top', 'vbox_stiff', 'width_z', 'width_roll_left'):
        setattr(C, name, float(p(name, float(getattr(C, name)))))
    for name in ('miss_confirm', 'filter_win', 'noise_samples'):
        setattr(C, name, int(p(name, getattr(C, name))))
    for name in ('sing_abort', 'log_trace', 'confirm', 'log_motion'):
        v = p(name, getattr(C, name))
        setattr(C, name, v if isinstance(v, bool) else str(v).lower() in ('1', 'true', 'yes'))
    C.filter = str(p('filter', C.filter))
    C.free_guard = str(p('free_guard', C.free_guard))
    if C.free_guard not in ('abs', 'baseline'):
        raise ValueError(f'free_guard 는 abs/baseline: {C.free_guard}')
    C.width_ori = str(p('width_ori', C.width_ori))
    C.width_lean = str(p('width_lean', C.width_lean))
    C.width_first = str(p('width_first', C.width_first))
    if C.width_first not in ('left', 'right'):
        raise ValueError(f'width_first 는 left/right: {C.width_first}')
    if C.width_lean not in ('away', 'toward'):
        raise ValueError(f'width_lean 은 away/toward: {C.width_lean}')
    if C.width_ori not in ('lean', 'keep'):
        raise ValueError(f'width_ori 는 lean/keep: {C.width_ori}')
    for name in ('width_ref_j_left', 'width_ref_j_right'):
        v = _floats(p(name, ','.join(str(x) for x in getattr(C, name))))
        if v and len(v) != 6:
            raise ValueError(f'{name} 는 관절 6개 "j1,...,j6" (또는 "" = 사용 안 함): {v}')
        setattr(C, name, v)
    C.out = str(p('out', C.out))
    C.tag = str(p('tag', C.tag))
    C.height_lats = _floats(p('height_lats', ','.join(str(v) for v in C.height_lats)))
    C.width_ups = _floats(p('width_ups', ','.join(str(v) for v in C.width_ups)))
    C.lat_range = (float(p('lat_min', C.lat_range[0])), float(p('lat_max', C.lat_range[1])))
    C.vbox_lat = (float(p('vbox_lat_min', C.vbox_lat[0])), float(p('vbox_lat_max', C.vbox_lat[1])))
    if C.tol <= 0 or C.step_z <= C.tol or C.step_y <= C.tol:
        raise ValueError('0 < tol < step_z, step_y 이어야 함')
    if C.contact >= C.force or not (0.0 < C.hyst <= 1.0):
        raise ValueError('contact < force, 0 < hyst <= 1 이어야 함')
    if C.filter not in ('median', 'ma', 'none'):
        raise ValueError(f'filter 는 median/ma/none: {C.filter}')
    if not C.height_lats or not C.width_ups or not all(0.0 < r < 1.0 for r in C.width_ups):
        raise ValueError('height_lats 는 1개 이상, width_ups 는 0~1 사이 비율 1개 이상')


def main(args=None):
    rclpy.init(args=args)
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    node = rclpy.create_node('wall_tap', namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    log = node.get_logger()

    mode = node.declare_parameter('mode', 'virtual').value
    try:
        if mode not in ('virtual', 'real'):
            raise ValueError(f'mode 는 virtual/real: {mode}')
        _declare(node)
        ws.TILT_DEG = C.tilt                 # wall_size 의 축/공구 자세 계산을 그대로 사용
        ws.configure_direction(C.direction)
        if mode == 'virtual':
            ws.VIRTUAL_BOX.update(face=C.vbox_face, floor=C.vbox_floor, top=C.vbox_top, lat=C.vbox_lat,
                                  stiffness=C.vbox_stiff)
            log.info(f'[virtual] 가상 벽: 표면 {C.vbox_face:.0f}, 바닥 {C.vbox_floor:.0f}, 윗면 {C.vbox_top:.0f}, '
                     f'옆 {list(C.vbox_lat)} → 정답 높이 {C.vbox_top - C.vbox_floor:.0f} / '
                     f'너비 {C.vbox_lat[1] - C.vbox_lat[0]:.0f}mm')
            if C.true_height <= 0:
                C.true_height = C.vbox_top - C.vbox_floor
            if C.true_width <= 0:
                C.true_width = C.vbox_lat[1] - C.vbox_lat[0]
    except ValueError as e:
        log.error(str(e))
        safe_shutdown(node)
        sys.exit(2)
    log.info(f'[설정] mode={mode}, 방향 {ws.DIRECTION}, tilt {C.tilt:.0f}° → TOOL_ORI={ws.TOOL_ORI}, '
             f'start={list(C.start)}, force {C.force:.0f}N, 간격 z{C.step_z:.0f}/옆{C.step_y:.0f}, '
             f'tol {C.tol}mm, 임계값 max({C.contact}N, {C.k_sigma}σ), 필터 {C.filter}({C.filter_win})'
             + (f', tag={C.tag}' if C.tag else ''))

    try:
        import DSR_ROBOT2 as dsr
    except ImportError as e:
        log.error(f'Error importing DSR_ROBOT2 : {e}')
        safe_shutdown(node)
        sys.exit(2)

    io = TapIO(node, dsr, mode)
    guard = ForceGuard(io)
    use_rviz = node.declare_parameter('rviz', True).value
    frame_id = node.declare_parameter('frame_id', 'base_link').value
    guard.free_mode = C.free_guard
    guard.free_tau, guard.free_hold = C.free_tau, C.free_hold
    guard.free_hard, guard.free_mhard = C.free_hard, C.free_mhard
    m = WallTapper(io)
    if use_rviz:
        m.viz = WallSizeViz(node, topic='wall_size_markers', frame_id=frame_id)
        m.viz.clear()
        m.viz.start_background(m.scene)
        log.info(f'[RViz] 마커: {node.resolve_topic_name("wall_size_markers")} (Fixed Frame: {frame_id}) '
                 '→ RViz 에서 Add > By topic > MarkerArray')
    io.wait_for_services()
    guard.wait_for_services()
    if mode == 'real' and C.confirm:
        ans = input('[REAL] 실제 로봇이 바닥/벽을 찍습니다. 주변 확인 후 y 입력: ')
        if ans.strip().lower() != 'y':
            log.info('취소했습니다.')
            safe_shutdown(node)
            sys.exit(1)

    home_policy = 'auto'
    try:
        io.setup()
        io.set_singularity(True)
        io.gripper_close()
        io.move_j(ws.HOME_J)
        guard.zero()
        io.guard = guard                      # 이제부터 모든 이동/찍기를 감시
        log.info(f'[0] 시작 위치로 movel {list(C.start)}')
        io.move_l(np.asarray(C.start, float), vel=C.approach_vel, acc=C.approach_acc)
        m.check_singularity([np.asarray(C.start, float),
                             ws.make_point(m.lat_c, ws.app_of(C.start), C.start[ws.UP_AXIS] - C.floor_depth)],
                            'floor')
        m.calibrate_noise()
        m.run()
    except KeyboardInterrupt:
        log.info('사용자 중단 (Ctrl+C)')
        m.status, m.error = 'aborted', 'KeyboardInterrupt'
        io.stop()
    except ToolCollision as e:
        log.error(f'[중단] 툴 충돌: {e}')
        m.status, m.error = 'error', f'ToolCollision: {e}'
    except (ArmCollision, ControllerStop, SensorFault) as e:
        log.error(f'[중단] {type(e).__name__}: {e}')
        m.status, m.error = 'error', f'{type(e).__name__}: {e}'
        home_policy = 'confirm'
    except Exception as e:
        log.error(f'[중단] {e}')
        m.status, m.error = 'error', str(e)
        io.stop()
    finally:
        def finish():
            m.summary()
            if m.taps or C.out or io.mlog:
                m.save()

        def stop_viz():
            if m.viz is not None:
                m.viz.stop_background()
                m.publish_viz()            # 마지막 상태 (TRANSIENT_LOCAL 로 남음)

        safe_shutdown(node, ('힘/순응제어 해제 실패', io.release_press),
                      ('결과 저장 실패', finish),
                      ('홈 복귀 실패', lambda: ws.go_home_after_stop(io, guard, home_policy)),
                      ('특이점 처리 복구 실패', lambda: io.set_singularity(False)),
                      ('RViz 정리 실패', stop_viz))
    sys.exit(0 if m.status == 'ok' else 1)


if __name__ == '__main__':
    main()
