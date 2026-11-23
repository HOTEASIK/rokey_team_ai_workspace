#!/usr/bin/env python3
"""[백업: 탐침 교체(mid) 기능 넣기 전 원래 버전 (2026-11-22)]  실행: ros2 run rokey1 wall_tap_backup
두산 M0609 로 벽을 '점 단위로 찍어서(tap)' 높이 / 너비를 재는 노드 (virtual / real).

wall_size.py 는 벽을 누른 채 미끄러지며 재는 방식이고, 이 노드는 매번 떨어졌다가
다시 찍는 방식이다. 느리지만 마찰 영향이 없고 '닿음 / 안 닿음' 판정이 분명하다.

[탐침 프리셋 / 자세 방식] robot.yaml wall_tap.probe (short | long) 로 고름 → probes.<이름> 이 공통 값을 덮어씀
  long  (133.5mm, pose_mode=tilt): 아래 [알고리즘] 그대로 (바닥·벽·높이 = tilt 숙임, 너비만 수평)
  short (26mm,   pose_mode=axis): 숙인 자세는 손목 특이점(J5)이라 (run149)
     바닥 = 수직 아래(floor_ori) → 수직 위로 바닥+lift → 홈 경유로 수평 자세 한 번 (벽 찾기 전, 시작 위치보다 벽 쪽으로 안 감)
     → 벽 찾기·높이·너비 모두 수평 (너비 앞 손목 뒤집기 없음, 오른쪽도 지금 배치로 되면 그대로)
     → 끝/실패 뒤: 벽에서 물러나 벽 윗면 위로 → 천천히 홈.  움직이기 전 점검에 J3 ≤ j3_max 도 포함

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
             (width_first=left. 첫 쪽(+Y)은 그 자리에서 수평 자세로, −Y 는 홈을 멈추지 않고 지나(amovesj) 자세 변경)
             자세(width_ori=lean): 공구 축을 Z축과 수직(수평)으로 두고 벽을 정면으로 봄 (width_yaw=0, 기본)
               - width_yaw=0 이면 양쪽 자세가 같음 [0, 90, 90] → 처음 한 번만 자세를 바꾸고 그대로 양쪽 탐색
               - 아래는 width_yaw=45 로 줄 때 (Z축 둘레로 찍는 쪽으로 45° 기울임)
               - 공구 좌표축을 X/Y/Z 에 맞춤: 공구 z(탐침) = 수평 45°, 공구 y = ±Z, 공구 x = 수평
               - 기준 자세 (+x 벽, 왼쪽(+Y) 찾을 때) task BASE [786.680,-21.090,116.960, 46.56,101.05,93.67],
                 joint [-24.36,51.39,73.72,91.82,68.58,-29.89] → 정리하면 ZYZ [45, 90, 90]
                 (공구 z 가 +X+Y 쪽 = 찾는 쪽) / 오른쪽(−Y)은 Y 대칭 [-45, 90, -90]
               - width_lean=toward(기본): 찾는 쪽으로 기울임 / away: 반대로
             탐침: 그리퍼 끝(등록 TCP)에서 공구 축 방향으로 probe_len(robot.yaml wall_tap.probe_len) 더 나간 점을 TCP 로 씀
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
  - TCP_NAME(config/robot.yaml) 이 '닫힌 손가락 끝' 으로 등록돼 있어야 측정값이 손끝 위치가 됨
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

from utils.config import CFG
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

from utils.robot import ROBOT_ID, ROBOT_MODEL  # config/robot.yaml robot 섹션


# =============================================================================
# 설정 (ROS 파라미터로 바꿀 수 있음, 단위 mm / N / s)
# =============================================================================
class Cfg:
    direction = '+x'            # 벽 방향 (+x/-x/+y/-y)
    tilt = 45.0                 # 그리퍼 기울기 [deg] (0 = 벽을 수평으로 봄, 90 = 바닥을 수직으로 봄)
    # 탐침 프리셋 (robot.yaml wall_tap.probe 로 고르고 wall_tap.probes.<이름> 이 공통 값을 덮어씀)
    probe = ''                  # 고른 프리셋 이름 (기록용)
    # 자세 방식: tilt = 바닥·벽·높이를 tilt 만큼 숙인 자세로, 너비만 수평 (긴 탐침 133.5mm)
    #            axis = 바닥은 수직 아래(floor_ori), 벽·높이·너비는 수평 (짧은 탐침 26mm: 숙인 자세는 J5 특이점, run149)
    pose_mode = 'tilt'
    floor_ori = (0.0, 180.0, 0.0)   # (axis) 바닥 찍기 자세 ZYZ (b=180: 탐침이 수직 아래). a 에 벽 방향 회전이 더해짐
    pick_j = ()                 # (pick_tap) 탐침 집는 관절값. () = pick_tap.PICK_J
    start = (700.0, 0.0, 100.0)  # 시작 위치 BASE [x, y, z] (벽 앞, 바닥 위) (실측 셋업 기준)

    force = 10.0                # 힘 제어 지령 크기 [N] (바닥: -Z, 벽: 벽 방향)
    stall_mm = 0.3              # (real) 이 거리 이하로만 움직이면 '정체'
    stall_sec = 0.3             # (real) ... 이 시간 동안
    flex_mm = 3.0               # (real) 접촉 시작 뒤 이만큼 더 밀려 들어가면 정체 없이도 '닿음' (휘는 도구는 계속 휘며 밀려 정체가 안 걸림, run72). 0 = 정체만
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
    width_contact = 1.5         # (백업에서는 안 씀: robot.yaml 에 있어도 오류 안 나게 이름만)
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
    step_z = 40.0               # 높이 측정 간격
    step_y = 40.0               # 너비 측정 간격
    tol = 1.0                   # 이분탐색 종료 구간 크기
    miss_confirm = 2            # 연속 이만큼 안 닿아야 '벽 끝' (한 번의 오판 방지)

    # 반복 측정 (중앙값)
    height_lats = (0.0,)        # 높이를 잴 옆 위치 (시작 위치 기준 오프셋 [mm])
    width_ups = (0.5,)          # 너비를 잴 높이 (바닥~윗면 비율, 0~1)
    width_z = 0.0               # >0 이면 높이 측정을 건너뛰고 이 BASE z 에서 좌우만 탐색 [mm]
    width_from_top = 30.0       # >0 이면 너비를 벽 윗면에서 이만큼 아래 높이에서 잼 (width_ups 대신) [mm].
                                #   단 그 높이가 가운데(바닥~윗면 0.5)보다 낮으면 가운데에서 잼 (낮은 벽)
    horizontal_tol = 1.0        # 너비 탐색 중 공구(탐침) 축이 수평(Z 축과 수직)에서 벗어나도 되는 각 [deg]
    width_ori = 'lean'          # 너비 측정 자세: lean = 수평 + Z축 둘레로 width_yaw 기울임 / keep = 높이 측정 자세 그대로
    width_first = 'left'        # 너비 먼저 찾을 쪽: left(+Y, 기준 자세) → right(−Y) / right → left
    width_lean = 'toward'       # (lean) toward = 찾는 쪽으로 기울임 (기준 자세: +Y 찾을 때 +Y 쪽) / away = 반대로
    width_yaw = 0.0             # (lean) 벽 방향에서 Z축 둘레로 기울이는 각 [deg] (0 = 벽 정면, 45 = 찍는 쪽으로 45°)
    # (lean) width_yaw 자세로 못 가면 차례로 시도할 기울기 [deg] (찾는 쪽으로). () = 시도 안 함
    #   벽이 가까우면 정면 자세는 팔이 너무 접혀 |J3| > j3_max (run139: 벽 755 에서 J3 136.9~141.6).
    #   찾는 쪽으로 기울이면 팔이 펴짐: 같은 벽에서 15° → J3 ≤ 131, 30° → ≤ 127 (2026-11-21 ikin 계산)
    width_yaw_try = (15.0, 30.0)
    width_switch_lift = 100.0   # 너비 y+ → y− 넘어갈 때 벽에서 물러난 뒤 홈 가기 전에 위로 올리는 높이 [mm] (0 = 안 올림)
    edge_depth_tol = 5.0        # (기울인 너비 자세) 표면보다 이만큼 더 들어가서 닿으면 탐침 몸통이 벽 모서리에 걸린 것 → 안 닿음 [mm]
    width_tilt = 0.0            # (lean) 아래로 숙이는 각 [deg]. 너비 탐색은 탐침이 무조건 수평이어야 해서 0 만 허용
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
    via_dz = 50.0               # (홈 경유 자세 바꾸기) 경유점 높이 후보 간격 [mm]
    via_dx = 10.0               # (홈 경유 자세 바꾸기) 경유점·도착점을 벽 쪽으로 옮기는 후보 간격 [mm] (벽 앞 standoff 까지)
    j3_max = 135.0              # 자세 바꾸기 목표 관절값의 |J3| 상한 [deg] (컨트롤러 한계 ±135 에서 2° 여유. 너비 자세 자체가 J3 ≈ 132 라 130 은 막힘, run64)
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


def apply_yaml(cfg_wall_tap, c=None):
    """robot.yaml wall_tap → Cfg. 공통 값 적용 뒤 probe 로 고른 probes.<이름> 을 덮어씀."""
    c = C if c is None else c
    wt = dict(cfg_wall_tap or {})
    probes = wt.pop('probes', None) or {}
    name = wt.pop('probe', None)
    items = list(wt.items())
    if name:
        if name not in probes:
            raise KeyError(f'robot.yaml wall_tap.probe: {name!r} 가 probes 에 없음 (있는 것: {list(probes)})')
        items += list((probes[name] or {}).items())
        c.probe = str(name)
    for k, v in items:
        if not hasattr(Cfg, k):
            raise KeyError(f'robot.yaml wall_tap.{k}: wall_tap Cfg 에 없는 이름')
        setattr(c, k, tuple(v) if isinstance(v, list) else v)
    return c


apply_yaml(CFG.get('wall_tap'))   # robot.yaml wall_tap 이 기본값 (-p 파라미터가 있으면 그게 우선)


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
            now = np.array([float(v) for v in self.dsr.get_current_posj()[:6]])
            if np.max(np.abs(now - qi)) > 1.0:              # 컨트롤러가 거부(관절 한계 등)해도 ret 은 성공일 수 있음
                raise RuntimeError(f'관절 이동 도착 실패 ({i}/{n}): 현재 {np.round(now, 1).tolist()} / '
                                   f'목표 {np.round(qi, 1).tolist()} → 관절 한계 / NOT REACHABLE 로그 확인')
            time.sleep(0.3)                                 # 멈춘 뒤 흔들림 가라앉히기
            self.zero_force()                               # 이 자세의 공구 무게 영향을 영점으로

    def move_sj(self, qs, vel=None, acc=None):
        """관절 경유점들을 멈추지 않고 지나는 스플라인 이동 (amovesj, 가드 감시). 도착 못 하면 에러."""
        vel = C.joint_vel if vel is None else vel
        acc = C.joint_acc if acc is None else acc
        pts = [np.array([float(v) for v in q[:6]]) for q in qs]
        cur = np.array([float(v) for v in self.dsr.get_current_posj()[:6]])
        span = sum(float(np.max(np.abs(b - a))) for a, b in zip([cur] + pts[:-1], pts))
        self._seg += 1
        self._check(self.dsr.amovesj([self.dsr.posj(*p.tolist()) for p in pts], vel=vel, acc=acc), 'amovesj')
        timeout = span / max(vel, 1e-3) * 3.0 + 10.0
        t0 = time.time()
        started = False
        while True:
            if self.guard is not None:
                self._guarded(self.get_pos(), (), 'movesj')
            moving = self.is_moving()
            started = started or moving
            if (started and not moving) or (not started and time.time() - t0 > 3.0):
                break                                       # 스플라인은 출발이 늦을 수 있음 → 출발 뒤 멈춤 = 끝
            if time.time() - t0 > timeout:
                self.stop()
                raise RuntimeError(f'스플라인 이동 시간 초과 (목표 {np.round(pts[-1], 1).tolist()})')
            time.sleep(ws.SAMPLE_PERIOD)
        q = np.array([float(v) for v in self.dsr.get_current_posj()[:6]])
        if np.max(np.abs(q - pts[-1])) > 1.0:              # 컨트롤러가 거부(NOT REACHABLE 등)해도 ret 은 성공일 수 있음
            raise RuntimeError(f'스플라인 이동 도착 실패: 현재 {np.round(q, 1).tolist()} / 목표 {np.round(pts[-1], 1).tolist()}')
        time.sleep(0.3)                                     # 멈춘 뒤 흔들림 가라앉히기

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
        t_cross = t_on = onset_pos = t_ref = timeout_msg = None
        ever_on = False                           # 한 번이라도 접촉 상태였는지 (시간 초과 원인 표시용)
        t0 = time.time()
        io.start_force(d, start, depth)
        try:
            while True:
                now = time.time()
                pos = io.get_pos()
                w = self._sense(pos, d, kind)
                # 진행 방향을 거슬러 미는 힘만 접촉 (벽/바닥 반력은 항상 −d 쪽, run90: +d 쪽 2.1N 출발 흔들림을 접촉으로 오판)
                raw = max(0.0, -float(np.dot(w[:3], d)))
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

                pushed = float(np.dot(pos - onset_pos, d)) if t_on is not None else 0.0   # 접촉 시작 뒤 더 들어간 거리
                if t_on is not None:
                    ever_on = True
                if t_on is not None and now - t_on >= C.contact_hold and (
                        io.mode == 'virtual' or stalled or (C.flex_mm > 0 and pushed >= C.flex_mm)):
                    # 접촉 시작 좌표 (real 도: 휘는 도구는 멈춘/밀린 위치가 휜 만큼 더 들어가 있음)
                    hit = onset_pos.copy()
                    self.last_onset = onset_pos.copy()
                    latency = now - t_ref
                    break
                if travel > depth and (t_on is None or travel > depth + C.max_depth):
                    break                           # 안 닿음 (힘이 걸린 채면 max_depth 만큼 더 기다림: 탄성 물건이 휘는 중일 수 있음)
                if io.mode == 'real' and span > 0.05 and moved / span > C.max_fc_speed:
                    raise TapError(f'힘 제어 속도 {moved / span:.0f}mm/s > {C.max_fc_speed:.0f} → 중단 '
                                   '(force 를 줄이거나 stiff_soft 를 높일 것)')
                if now - t0 > C.probe_timeout:
                    if t_on is None and travel > depth - C.max_depth / 2:
                        break                       # 표면보다 max_depth/2 넘게 들어갔는데 힘 없음 → 안 닿음 (끝 너머에서 느리게 진행)
                    if kind != 'wall_seek' and travel > depth:
                        # 예상 표면보다 깊이 들어가도록 정체 없음 → 벽 면이 아님 (끝 모서리를 스치며 미끄러짐, run90) → 안 닿음
                        self.log.info(f'  [{kind}] 표면 너머 {travel - depth:.1f}mm 까지 정체 없이 진행 '
                                      f'(최대 {peak:.1f}N) → 모서리 스침으로 보고 안 닿음 처리')
                        break
                    speed = moved / span if span > 0 else 0.0
                    if ever_on:
                        why = '접촉은 감지됐지만 정체 없이 계속 밀려 들어감 (도구 휨 → flex_mm 확인)'
                    elif peak >= th_in:
                        why = '순간 힘만 임계값을 넘음 (필터 후 미달)'
                    else:
                        why = '힘이 임계값까지 안 걸림 (도구 휨/미끄러짐 또는 아무것도 안 닿음)'
                    timeout_msg = (f'찍기 시간 초과 ({kind}, 진행 {travel:.1f}/{depth:.0f}mm, 최대 힘 {peak:.1f}N '
                                   f'/ 임계 {th_in:.1f}N, 최근 속도 {speed:.1f}mm/s, '
                                   f'끝 {np.round(pos, 1).tolist()}) → {why}')
                    break
                time.sleep(C.sample)
        finally:
            io.end_force()

        rec = {'i': len(self.taps), 'kind': kind, 't': round(t0 - (self.t_start or t0), 3),
               'start': start.round(2).tolist(), 'dir': d.round(3).tolist(),
               'hit': hit is not None, 'xyz': None if hit is None else hit.round(2).tolist(),
               'travel': round(travel, 2), 'duration': round(time.time() - t0, 3),
               'th_in': round(th_in, 3), 'peak': round(peak, 3),
               'latency': None if latency is None else round(latency, 3)}
        if timeout_msg:
            rec['timeout'] = timeout_msg
        self.taps.append(rec)
        self.traces.append(trace)                 # 시간 초과여도 시계열을 남김 (원인 분석용)
        if timeout_msg:
            raise TapError(timeout_msg)
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
        if hit is not None and getattr(self, 'edge_guard', False) and kind != 'width_center':
            ref = self.last_onset if self.last_onset is not None else hit
            depth = ws.app_of(ref) - self.face
            if depth > C.edge_depth_tol:
                # 기울인 자세: 탐침 끝이 벽 끝을 지나 표면보다 깊이 들어간 뒤 몸통이 모서리에 닿음 → 벽 없음
                self.log.info(f'    [너비] 표면보다 {depth:.1f}mm 깊이 닿음 (> edge_depth_tol {C.edge_depth_tol:.0f}) '
                              '→ 탐침 몸통이 모서리에 걸림, 안 닿음으로 처리')
                self.taps[-1].update(hit=False, edge_snag=round(depth, 2))   # 벽 4점 계산(explore_server)에도 안 쓰이게
                hit = None
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
        bad, unreachable, j3_bad = [], [], []
        for p in points:
            q = io.joint_at(p, sol)
            if q is None:
                unreachable.append(np.round(p, 0).tolist())
            elif abs(q[4]) < C.sing_deg:
                bad.append((np.round(p, 0).tolist(), round(q[4], 1)))
            elif C.pose_mode == 'axis' and abs(q[2]) > C.j3_max:
                j3_bad.append((np.round(p, 0).tolist(), round(q[2], 1)))
        if j3_bad:
            raise TapError(f'[특이점 점검:{label}] |J3| > j3_max {C.j3_max:.0f}° 위치 {len(j3_bad)}개 '
                           f'(예: {j3_bad[0][0]} J3={j3_bad[0][1]}°) → 시작 위치/벽 거리 확인. 로봇은 그 자리에서 멈춤')
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

    def side_ori(self, side_sign, yaw_deg=None):
        """너비 측정 자세 (ZYZ). 공구 축을 X-Y 평면과 평행(width_tilt=0)하게 두고 벽 방향에서
        Z축 둘레로 width_yaw 만큼 돌린다. side_sign: 찾는 쪽의 옆 축 부호.
          width_lean=toward: 찾는 쪽으로 (기준: +x 벽, 왼쪽(+Y) 찾을 때 [45, 90, 90])
          width_lean=away  : 찾는 쪽 반대로
        width_roll=90 → 공구 y 축이 +Z(위), 공구 x 축은 수평 (공구 축이 X/Y/Z 와 평행)."""
        yaw = np.radians(C.width_yaw if yaw_deg is None else yaw_deg)
        lean = side_sign * (-1 if C.width_lean == 'away' else 1)
        horiz = np.cos(yaw) * ws.APP_DIR + np.sin(yaw) * lean * ws.LAT_DIR
        a = float(np.degrees(np.arctan2(horiz[1], horiz[0])))
        # 롤도 대칭: 기준 자세(+90)의 거울상은 -90 → 왼쪽 기준 관절(J6 ≈ +30)과 맞아 J6 를 180° 돌리지 않음
        if abs(np.sin(yaw)) < 1e-6:
            roll = C.width_roll                             # 정면: 양쪽 같은 자세 (기준 관절 배치, 공구 y = +Z)
        else:
            roll = C.width_roll * (1 if lean * float(np.dot(np.cross(ws.APP_DIR, ws.LAT_DIR), ws.UP_DIR)) > 0 else -1)
        return [round(a, 3), 90.0, roll]                # ZYZ b = 90 → 공구 축이 수평 (Z 축과 수직), a·c 와 상관없음

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
                self._check_j3(q, label, back)                  # 물러날수록 J3 가 커짐 → 한계 넘으면 더 물러나지 않고 멈춤
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

    def _ref_sol(self, ref_j):
        """width_ref_j_* → (벽 방향에 맞춰 J1 을 돌린 기준 관절, 그 solution space)."""
        ref = [float(v) for v in ref_j]
        ref[0] += ws.DIRECTIONS[ws.DIRECTION][3]            # 벽 방향에 맞춰 J1 회전
        return ref, self.io.dsr.get_solution_space(self.io.dsr.posj(*ref))

    def _lift_points(self, lift, label):
        """벽에서 rot_clear 물러난 점 → 위로 lift 올린 점까지의 경로 (5mm 간격, 로봇 안 움직임).
        지금 관절 배치로 도달 가능 + |J3| ≤ j3_max 인 데까지만 올림: 중간에 막히면 그 바로 아래가 끝 (0 이면 안 올림).
        반환: (물러난 점, 올라갈 점들 [마지막이 최종 높이], 막힌 이유 또는 None)."""
        io = self.io
        back = io.get_pos() - ws.APP_DIR * C.rot_clear
        up0 = float(np.dot(back, ws.UP_DIR))
        lift = min(float(lift), C.reorient_z_max - up0)
        if lift <= 0.0:
            return back, [], None
        sol = io.dsr.get_current_solution_space()
        ok, why = [], None
        for h in np.linspace(0.0, lift, max(2, int(np.ceil(lift / 5.0)) + 1))[1:]:
            p = back + ws.UP_DIR * h
            q = io.joint_at(p, sol)
            if q is None:
                why = f'z {np.dot(p, ws.UP_DIR):.0f} 도달 불가 (sol {sol})'
            elif abs(float(q[2])) > C.j3_max:
                why = f'z {np.dot(p, ws.UP_DIR):.0f} 에서 J3 {float(q[2]):.1f}° > j3_max {C.j3_max:.0f}'
            if why:
                break
            ok.append(p)
        if why:
            got = float(np.dot(ok[-1] - back, ws.UP_DIR)) if ok else 0.0
            self.log.warn(f'[자세] {label}: 위로 {lift:.0f}mm 는 못 올림 ({why}) → 관절이 허락하는 {got:.0f}mm 까지만 올리고 홈으로')
        return back, ok, why

    def reorient_via_home(self, ori, label, ref_j, plan=None, lift=0.0, target=None, app_max=None):
        """벽에서 rot_clear 물러남 → 홈 (movej) → ori 자세로 가운데(lat_c) 위 경유점 (movej, 바닥 여유 점검)
        → 수직 하강으로 같은 높이에 도착. 벽 앞에서 손목을 뒤집지 않는다.
        plan: _plan_via 결과 (없으면 여기서 계산, 못 찾으면 TapError). 움직이기 전에 계획을 다 세운다.
        lift: 물러난 뒤 홈 가기 전에 위로 올리는 높이 [mm] (너비 y+ → y−: width_switch_lift)."""
        io = self.io
        if not ref_j:
            raise TapError(f'[자세] {label}: width_ref_j_* 필요 (홈 경유 후 관절 배치)')
        if plan is None:
            plan, why = self._plan_via(ori, ref_j, target, app_max)
            if plan is None:
                raise TapError(f'[자세] {label}: {why}')
        land, via, qv, dz, dx, low, far, n_fail = plan
        back, ups, _ = self._lift_points(lift, label) if lift > 0 else (None, [], None)
        self.log.info(f'[자세] {label}: TOOL_ORI {list(ws.TOOL_ORI)} → {list(ori)}, 후보 {n_fail + 1}개째 '
                      f'(z+{dz:.0f}, 벽쪽+{dx:.0f}) 홈 → 경유점 {np.round(via, 1).tolist()} movej '
                      f'{np.round(qv, 1).tolist()} (경로 최저 z {low:.0f}, 벽쪽 최대 {far:.0f})'
                      + (f' → 수직 하강 {dz:.0f}mm' if dz > 0 else ''))
        io.move_l(io.get_pos() - ws.APP_DIR * C.rot_clear if back is None else back)
        if ups:
            self.log.info(f'[자세] {label}: 홈 가기 전에 위로 {np.dot(ups[-1] - back, ws.UP_DIR):.0f}mm 올림 '
                          f'(z {np.dot(back, ws.UP_DIR):.0f} → {np.dot(ups[-1], ws.UP_DIR):.0f})')
            io.move_l(ups[-1])                              # 자세 그대로 수직 상승 (경로는 _lift_points 로 점검)
        io.move_j_reorient(ws.HOME_J)                       # 지금 자세로 홈까지 (run67 에서 문제없던 구간)
        io.move_j_reorient(qv)                              # 관절 직선 = _sweep 으로 점검한 경로 그대로
        ws.TOOL_ORI = list(ori)                             # SizeIO._posx 가 이 값을 사용
        if dz > 0:
            io.move_l(land)                                 # 새 자세 그대로 수직 하강
        io.zero_force()                                     # 자세가 바뀌어 공구 무게 영향이 달라짐 → 다시 영점

    def _wall_top(self):
        """관절 경로 벽 점검에 쓸 높이: 벽 윗면 + flip_clear (윗면을 모르면 None = 높이 상관없이 점검)."""
        top = self.res.get('top_z')
        return None if top is None else float(top) + C.flip_clear

    def _plan_via(self, ori, ref_j, target=None, app_max=None):
        """홈 경유 자세 바꾸기 계획 (로봇 안 움직임). 지금 위치에서 rot_clear 물러난 높이·거리 기준.
        target: 도착점 (기본: 가운데, 지금 높이, 지금보다 rot_clear 뒤), app_max: 벽 쪽 한계 (기본: 벽 − standoff).
        반환: ((land, via, qv, dz, dx, low, far, 실패 후보 수), None) 또는 (None, 못 찾은 이유)."""
        io = self.io
        if target is None:
            cur = io.get_pos()
            target = ws.make_point(self.lat_c, ws.app_of(cur) - C.rot_clear, float(np.dot(cur, ws.UP_DIR)))
        target = np.asarray(target, float)
        ref, sol = self._ref_sol(ref_j)
        top = self._wall_top()                              # 이 높이보다 위에서는 벽 쪽으로 나가도 괜찮음
        # 홈 → 목표를 관절로 바로 가면 J5 가 +90 → −60 으로 뒤집히며 탐침 끝이 바닥까지 처짐 (run67, z 14 에서 충돌)
        # → 새 자세의 경유점으로 관절 이동 → 경유점에서 수직 직선 하강해 도착점(경유점 아래, 원래 높이)으로.
        #   경유점 후보: 높이 +via_dz 씩(위) × 벽 쪽 +via_dx 씩(팔이 펴져 J3 가 줄어듦, run74: 올리기만 하면 J3 135 초과)
        #   조건: 도착점·경유점 |J3| ≤ j3_max, 홈→경유점 관절 경로가 바닥 위 flip_clear, 벽 앞 standoff 이상 떨어짐
        up0 = float(np.dot(target, ws.UP_DIR))
        app0 = ws.app_of(target)
        if app_max is None:
            app_max = self.face - C.standoff                # 도착점/경로가 벽에 이보다 가까이 가지 않음 (찍기 시작 위치)
        n_dx = max(0, int((app_max - app0) // C.via_dx))
        fails = []
        found = None
        for dz in np.arange(0.0, C.reorient_z_max - up0 + 1e-6, C.via_dz):
            for dx in [i * C.via_dx for i in range(n_dx + 1)]:
                land = target + ws.APP_DIR * dx
                via = land + ws.UP_DIR * dz
                tag = f'z+{dz:.0f}/벽쪽+{dx:.0f}'
                ql = self._ref_ik(land, ori, sol, ref)
                if ql is None or abs(float(ql[2])) > C.j3_max:
                    fails.append(f'{tag}: 도착점 ' + ('도달 불가' if ql is None else f'J3 {ql[2]:.1f}'))
                    continue
                qv = ql if dz == 0 else self._ref_ik(via, ori, sol, ref)
                if qv is None or abs(float(qv[2])) > C.j3_max:
                    fails.append(f'{tag}: 경유점 ' + ('도달 불가' if qv is None else f'J3 {qv[2]:.1f}'))
                    continue
                low, far = self._sweep(ws.HOME_J, qv, wall_top=top)
                if low < self.floor_z + C.flip_clear or far > app_max:
                    fails.append(f'{tag}: 경로 최저 z {low:.0f} / 벽 높이 안 벽쪽 최대 {far:.0f}')
                    continue
                found = (land, via, qv, dz, dx, low, far)
                break
            if found:
                break
        if found is None:
            return None, (f'안전한 경유점 없음 (바닥 {self.floor_z:.0f}+{C.flip_clear:.0f}, 벽 {app_max:.0f}'
                          + (f' (z {top:.0f} 아래)' if top is not None else '') + ', '
                          f'j3_max {C.j3_max:.0f}) 후보 {len(fails)}개: {"; ".join(fails[-3:])}')
        return (*found, len(fails)), None

    def _width_path_bad(self, ori, ref_j, lats, ups):
        """ori 자세·기준 배치로 너비 찍기 위치(벽 앞 standoff ~ 표면+max_depth)에 모두 갈 수 있는지 (로봇 안 움직임).
        문제 없으면 None, 있으면 첫 문제 설명."""
        ref, sol = self._ref_sol(ref_j)
        for p in self._path_points(lats, ups):
            q = self._ref_ik(p, ori, sol, ref)
            where = f'y {float(np.dot(p, ws.LAT_DIR)):.0f}/z {float(np.dot(p, ws.UP_DIR)):.0f}'
            if q is None:
                return f'찍기 위치 {where} 도달 불가'
            if abs(float(q[2])) > C.j3_max:
                return f'찍기 위치 {where} J3 {float(q[2]):.1f}'
            if abs(float(q[4])) < C.sing_deg:
                return f'찍기 위치 {where} J5 {float(q[4]):.1f} (특이점)'
        return None

    def width_pose(self, k, sign, key, name, lats, ups):
        """너비 한쪽의 자세를 고른다 (로봇 안 움직임). (기준 배치) × (width_yaw → width_yaw_try) 순서로
        ① 찍기 위치 전부 도달·|J3| ≤ j3_max·특이점 아님 ② 자세 그대로 넘어가기 또는 홈 경유 계획 가능 인 첫 조합.
        기준 배치: 이쪽 width_ref_j_*. axis 방식 오른쪽은 지금 배치(왼쪽 기준)를 먼저 시도 (되면 홈 경유 없음).
        반환: (ori, 'over' | 'via', plan, yaw, ref_j). 하나도 안 되면 TapError (모든 조합의 이유)."""
        own = tuple(C.width_ref_j_left if key == 'left' else C.width_ref_j_right)
        refs = [own]
        if C.pose_mode == 'axis' and key == 'right' and tuple(C.width_ref_j_left) and own != tuple(C.width_ref_j_left):
            refs = [tuple(C.width_ref_j_left), own]
        j4_now = float(self.io.dsr.get_current_posj()[3])
        yaws = list(dict.fromkeys([float(C.width_yaw), *[float(v) for v in C.width_yaw_try]]))
        fails = []
        for ref_j in refs:
            same_cfg = bool(ref_j) and abs((j4_now - ref_j[3] + 180.0) % 360.0 - 180.0) < 90.0   # 지금 J4 가 기준 배치와 같은 쪽
            tag = '' if len(refs) == 1 else f'배치 J4 {ref_j[3]:.0f}°, '
            for yaw in yaws:
                ori = self.side_ori(sign, yaw)
                if ref_j:
                    # 롤은 기울기와 상관없이 기준 관절 배치(J6)에 맞춤 (거울 롤이면 J6 가 180° 어긋남)
                    ori[2] = C.width_roll_left if ref_j == tuple(C.width_ref_j_left) else C.width_roll
                    bad = self._width_path_bad(ori, ref_j, lats, ups)
                    if bad:
                        fails.append(f'{tag}기울기 {yaw:.0f}°: {bad}')
                        continue
                if np.allclose(ori, ws.TOOL_ORI, atol=1e-3) and same_cfg:
                    # 지금 자세·관절 배치 그대로 → 손목 뒤집기(홈 경유) 없이
                    #   도구는 계속 앞(벽 쪽)을 본 채 위로 띄워 가운데로 넘어감 (run75: 뒤집힌 배치로 내려가다 충돌)
                    try:
                        self._plan_over(f'너비 {name}')
                        return ori, 'over', None, yaw, ref_j
                    except TapError as e:
                        self.log.info(f'[자세] 너비 {name}: 자세 그대로 넘어가기 불가 ({e}) → 홈 경유 시도')
                if k > 0 and not same_cfg and ref_j:
                    # 자세는 같아도 관절 배치가 다름 → 이쪽 기준 배치(width_ref_j_*)로 홈 경유
                    #   (run85: 왼쪽 배치(J4≈-1) 그대로 오른쪽 y −290 까지 가다 J5 가 바닥에 닿음. 오른쪽은 J4≈177 이어야 안전)
                    self.log.info(f'[자세] 너비 {name}: 지금 J4 {j4_now:.1f}° ≠ 기준 {ref_j[3]:.1f}° → 홈 경유로 배치 바꿈')
                if not ref_j:
                    return ori, 'via', None, yaw, ref_j     # reorient_via_home 이 width_ref_j_* 없음 오류를 냄
                plan, why = self._plan_via(ori, ref_j)
                if plan is None:
                    fails.append(f'{tag}기울기 {yaw:.0f}°: {why}')
                    continue
                return ori, 'via', plan, yaw, ref_j
        raise TapError(f'[자세] 너비 {name}: 가능한 자세 없음 (j3_max {C.j3_max:.0f}, 시도 기울기 {yaws}) → '
                       + ' / '.join(fails) + ' → 시작 위치를 벽에서 더 떨어뜨리거나 width_yaw_try 에 더 큰 각 추가')

    def move_over(self, label):
        """자세(TOOL_ORI)·관절 배치를 그대로 두고 첫 쪽 끝 → 둘째 쪽 시작(가운데)으로 옮긴다. 도구는 계속 벽 쪽을 봄.
        벽에서 rot_clear 물러남 → 위로 (벽 윗면 + flip_clear) → 가운데로 옆 이동 → 원래 높이로 수직 하강 (모두 movel).
        움직이기 전에 네 점 모두 지금 관절 배치로 도달 가능 + |J3| ≤ j3_max 인지 확인."""
        pts = self._plan_over(label)
        self.log.info(f'[자세] {label}: 자세 그대로 {list(ws.TOOL_ORI)}, 위로 띄워 넘어감 '
                      f'z {np.dot(pts[0], ws.UP_DIR):.0f} → {np.dot(pts[1], ws.UP_DIR):.0f}, '
                      f'옆 {np.dot(pts[1], ws.LAT_DIR):.0f} → {self.lat_c:.0f}')
        for p in pts:
            self.io.move_l(p)
        self.io.zero_force()

    def _plan_over(self, label):
        """move_over 의 네 점 (로봇 안 움직임). 못 가면 TapError."""
        io = self.io
        cur = io.get_pos()
        up0 = float(np.dot(cur, ws.UP_DIR))
        lat0 = float(np.dot(cur, ws.LAT_DIR))
        app = ws.app_of(cur) - C.rot_clear
        top = self.res.get('top_z', up0)
        z_hi = min(max(up0, top + C.flip_clear), C.reorient_z_max)
        pts = [ws.make_point(lat0, app, up0), ws.make_point(lat0, app, z_hi),
               ws.make_point(self.lat_c, app, z_hi), ws.make_point(self.lat_c, app, up0)]
        sol = io.dsr.get_current_solution_space()
        for p in pts:
            q = io.joint_at(p, sol)
            if q is None:
                raise TapError(f'[자세] {label}: 넘어가는 경로 {np.round(p, 1).tolist()} 에 지금 관절 배치(sol {sol})로 도달 불가')
            self._check_j3(q, label, p)
        return pts

    def _check_j3(self, q, label, xyz):
        """자세 바꾸기 목표 관절값의 J3 가 j3_max 를 넘으면 움직이기 전에 멈춤 (컨트롤러가 일부 구간만 거부해 손목이 반쯤 돈 채 남는 것 방지)."""
        if abs(float(q[2])) > C.j3_max:
            raise TapError(f'[자세] {label}: {np.round(xyz, 1).tolist()} 목표 J3 {float(q[2]):.1f}° '
                           f'(|J3| > j3_max {C.j3_max:.0f}°) → 관절 {np.round(q, 1).tolist()}, 시작 위치/width_ref_j_* 확인')

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

    def _sweep(self, q0, q1, n=20, wall_top=None):
        """q0 → q1 관절 직선 보간 경로의 탐침 끝 (최저 높이, 벽 쪽 최대 좌표). fkin 으로 계산.
        wall_top: 주면 이 높이보다 낮은 점만 벽 쪽 최대에 셈 (벽 윗면보다 높으면 벽 쪽으로 나가도 안 부딪힘,
                  run141·143: 벽 쪽 759~816 은 모두 z 438~561, 벽 윗면 190 위). 그 아래 점이 없으면 -inf."""
        dsr = self.io.dsr
        q0 = np.array([float(v) for v in q0[:6]])
        q1 = np.array([float(v) for v in q1[:6]])
        low, far = np.inf, -np.inf
        for s in np.linspace(0.0, 1.0, n + 1):
            p = [float(v) for v in dsr.fkin(dsr.posj(*(q0 + (q1 - q0) * s).tolist()), ref=dsr.DR_BASE)]
            tcp = np.asarray(p[:3])                          # 그리퍼 손끝 (도구가 위를 향하면 끝보다 낮음)
            tip = tcp + C.probe_len * ws._tool_dir(p[3:6])
            low = min(low, float(np.dot(tip, ws.UP_DIR)), float(np.dot(tcp, ws.UP_DIR)))
            for pt in (tip, tcp):
                if wall_top is None or float(np.dot(pt, ws.UP_DIR)) < wall_top:
                    far = max(far, ws.app_of(pt))
        return low, far

    def width_heights(self):
        """너비를 잴 높이 (BASE z).
          width_z > 0        : 그 높이 하나
          width_from_top > 0 : 윗면 − width_from_top (가운데보다 낮으면 가운데)
          그 외              : 바닥 + width_ups 비율 × 높이"""
        if C.width_z > 0:
            return [float(C.width_z)]
        H = self.res['height']['median']
        if C.width_from_top > 0:
            top = self.floor_z + H
            mid = self.floor_z + 0.5 * H
            z = top - C.width_from_top
            if z < mid:
                self.log.warn(f'[4] 윗면 −{C.width_from_top:.0f}mm (z {z:.1f}) 가 가운데(z {mid:.1f})보다 낮음 '
                              f'(벽 높이 {H:.0f}mm) → 가운데에서 잼')
                z = mid
            return [z]
        return [self.floor_z + r * H for r in C.width_ups]

    def check_horizontal(self, label):
        """지금 공구(탐침) 축이 수평(Z 축과 수직)인지 실제 자세로 확인. 아니면 찍기 전에 멈춤."""
        posx, _ = self.io.dsr.get_current_posx(ref=self.io.dsr.DR_BASE)
        d = ws._tool_dir([float(v) for v in posx[3:6]])
        tilt = float(np.degrees(np.arcsin(np.clip(abs(np.dot(d, ws.UP_DIR)), 0.0, 1.0))))
        if tilt > C.horizontal_tol:
            raise TapError(f'[자세] {label}: 탐침 축이 수평에서 {tilt:.1f}° 기울어짐 (> horizontal_tol '
                           f'{C.horizontal_tol:.1f}°, 자세 {np.round(posx[3:6], 1).tolist()}) → 너비 탐색 안 함')
        self.log.info(f'[자세] {label}: 탐침 축 수평 확인 (기울기 {tilt:.2f}°)')
        return tilt

    def horizontal_ori(self):
        """(axis) 벽·높이·너비 공통 수평 자세 = 너비 왼쪽(+Y 기준 배치) 정면 자세."""
        r_sign = 1 if float(np.dot(np.cross(ws.APP_DIR, ws.UP_DIR), ws.LAT_DIR)) > 0 else -1
        ori = self.side_ori(-r_sign, 0.0)
        ori[2] = C.width_roll_left
        return ori

    def switch_to_horizontal(self):
        """(axis) 바닥을 찍은 뒤: 수직 위로 바닥 + lift → 시작 위치 높이에서 수평 자세로 (홈 경유, 벽 찾기 전이라
        시작 위치보다 벽 쪽으로 안 감) → 노이즈 다시 측정. 움직이기 전에 경유점·벽 찾기 시작점을 계산으로 점검."""
        io = self.io
        ori = self.horizontal_ori()
        ref_j = tuple(C.width_ref_j_left)
        if not ref_j:
            raise TapError('[자세] 수평 자세: width_ref_j_left 필요 (수평 자세 관절 배치)')
        z0 = self.floor_z + C.lift
        app_s = ws.app_of(C.start)
        target = ws.make_point(self.lat_c, app_s, z0)          # = 첫 벽 찾기 시작점
        ref, sol = self._ref_sol(ref_j)
        for p, what in ((target, '벽 찾기 시작점'), (target + ws.APP_DIR * 50.0, '벽 찾기 +50mm')):
            q = self._ref_ik(p, ori, sol, ref)
            if q is None or abs(float(q[2])) > C.j3_max or abs(float(q[4])) < C.sing_deg:
                why = '도달 불가' if q is None else f'J3 {q[2]:.1f} / J5 {q[4]:.1f}'
                raise TapError(f'[자세] 수평 자세: {what} {np.round(p, 1).tolist()} {why} '
                               f'(j3_max {C.j3_max:.0f}, sing_deg {C.sing_deg:.0f}) → start 를 옮길 것. 로봇은 그 자리에서 멈춤')
        plan, why = self._plan_via(ori, ref_j, target=target, app_max=app_s)
        if plan is None:
            raise TapError(f'[자세] 수평 자세: {why}')
        cur = io.get_pos()
        up = ws.make_point(float(np.dot(cur, ws.LAT_DIR)), ws.app_of(cur), max(z0, float(np.dot(cur, ws.UP_DIR))))
        self.log.info(f'[자세] 바닥 찍은 뒤 수직 위로 z {np.dot(cur, ws.UP_DIR):.0f} → {np.dot(up, ws.UP_DIR):.0f} '
                      f'→ 수평 자세 {ori} (벽·높이·너비 공통)')
        io.move_l(up)                                       # 아래 보는 자세 그대로 바닥에서 떨어짐
        self.reorient_via_home(ori, '수평 자세', ref_j, plan, target=target, app_max=app_s)
        self.res['tool_ori_horizontal'] = list(ws.TOOL_ORI)
        self.check_horizontal('수평 자세')
        self.calibrate_noise()                              # 자세가 바뀌어 힘 노이즈도 다름

    def retreat_before_home(self):
        """(axis) 홈 가기 전: 벽에서 rot_clear 물러남 → 벽 윗면 + flip_clear 까지 (관절이 허락하는 데까지) 수직 위로
        → 천천히 홈 (자세 바꾸기 속도). 수평 자세(J5 −)에서 홈(J5 +90)으로 손목이 돌 때 탐침이 벽·바닥에서 멀게."""
        io = self.io
        if np.allclose(ws.TOOL_ORI, self.floor_pose(), atol=1e-3):
            return                                          # 아직 아래 보는 자세 → 기존 홈 복귀로 충분
        cur = io.get_pos()
        top = self.res.get('top_z')
        z_now = float(np.dot(cur, ws.UP_DIR))
        lift = (float(top) + C.flip_clear - z_now) if top is not None else C.width_switch_lift
        back, ups, _ = self._lift_points(max(lift, 0.0), '홈 복귀')
        self.log.info(f'[홈 복귀] 벽에서 {C.rot_clear:.0f}mm 물러남 → 위로 '
                      f'{(np.dot(ups[-1] - back, ws.UP_DIR) if ups else 0.0):.0f}mm → 천천히 홈')
        io.move_l(back)
        if ups:
            io.move_l(ups[-1])
        io.move_j_reorient(ws.HOME_J)

    def floor_pose(self):
        """(axis) 바닥 찍기 자세 = floor_ori + 벽 방향 회전."""
        a, b, c = (float(v) for v in C.floor_ori)
        return [a + ws.DIRECTIONS[ws.DIRECTION][3], b, c]

    def measure_width(self):
        ups = self.width_heights()
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
        for k, (sign, key, name) in enumerate(sides):
            limit = C.lat_range[1] if sign > 0 else C.lat_range[0]
            lats = np.arange(self.lat_c, limit + sign * 1e-6, sign * 40.0)
            self.edge_guard = False
            # lean: 양쪽 모두 수평 자세. 양쪽 다 홈을 거쳐 바꿈
            #   (벽 앞에서 손목을 뒤집으면(set_tool_ori) 여유 확보로 물러나다 J3 한계를 넘고 공구가 바닥에 닿았음, run56)
            #   움직이기 전에 기울기 후보(width_yaw → width_yaw_try) 중 찍기 위치·경유점이 모두 되는 자세를 고름 (run139)
            if C.width_ori == 'lean':
                ori, how, plan, yaw, ref_j = self.width_pose(k, sign, key, name, lats, ups)
                if abs(yaw - C.width_yaw) > 1e-6:
                    self.log.warn(f'[자세] 너비 {name}: 기울기 {C.width_yaw:.0f}° 로는 안 됨 → {yaw:.0f}° 사용 {ori}')
                self.res[f'width_yaw_{key}'] = yaw
                self.edge_guard = abs(yaw) > 1e-6            # 기울이면 탐침 몸통이 모서리에 걸릴 수 있음 (tap_wall)
                if how == 'over':
                    self.move_over(f'너비 {name}')
                else:
                    self.reorient_via_home(ori, f'너비 {name}', ref_j, plan,
                                           lift=C.width_switch_lift if k > 0 else 0.0)   # y+ → y− 넘어갈 때만
            self.res[f'tool_ori_width_{key}'] = list(ws.TOOL_ORI)
            self.res[f'width_tilt_{key}'] = round(self.check_horizontal(f'너비 {name}'), 2)
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
        if C.pose_mode == 'axis':
            self._phase('to_horizontal', self.switch_to_horizontal)
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
    for name in ('tilt', 'force', 'stall_mm', 'stall_sec', 'flex_mm', 'max_fc_speed', 'probe_vel', 'move_vel',
                 'move_acc', 'approach_vel', 'approach_acc', 'rot_vel', 'rot_acc', 'joint_vel', 'joint_acc', 'reorient_step',
                 'contact', 'k_sigma', 'hyst', 'contact_hold',
                 'floor_depth', 'wall_search', 'lift', 'standoff', 'max_depth', 'step_z', 'step_y',
                 'tol', 'z_limit', 'travel_clear', 'probe_len', 'width_yaw', 'width_tilt', 'width_roll', 'rot_clear', 'stiff_soft', 'stiff_other', 'stiff_rot', 'sing_deg', 'probe_timeout',
                 'true_height', 'true_width', 'free_tau', 'free_hold', 'free_hard', 'free_mhard',
                 'flip_clear', 'reorient_z_max', 'j3_max', 'edge_depth_tol', 'width_switch_lift', 'width_from_top', 'horizontal_tol', 'via_dz', 'via_dx', 'vbox_face', 'vbox_floor', 'vbox_top', 'vbox_stiff', 'width_z', 'width_roll_left'):
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
    C.pose_mode = str(p('pose_mode', C.pose_mode))
    if C.pose_mode not in ('tilt', 'axis'):
        raise ValueError(f'pose_mode 는 tilt/axis: {C.pose_mode}')
    if C.width_ori != 'lean':
        raise ValueError(f'width_ori 는 lean 만 가능 (너비 탐색은 탐침이 수평이어야 함): {C.width_ori}')
    if abs(C.width_tilt) > 1e-9:
        raise ValueError(f'width_tilt 는 0 이어야 함 (너비 탐색은 탐침이 수평): {C.width_tilt}')
    C.width_yaw_try = tuple(_floats(p('width_yaw_try', ','.join(str(v) for v in C.width_yaw_try))))
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


def pose_mode_warnings(c=None):
    """탐침 길이와 자세 방식이 안 맞으면 경고 문장들 (멈추지는 않음)."""
    c = C if c is None else c
    out = []
    if c.pose_mode == 'tilt' and 0.0 < c.probe_len <= 60.0:
        out.append(f'[탐침] 길이 {c.probe_len:g} mm 인데 tilt 방식 → 숙인 자세가 손목 특이점(J5)에 가까움 (run149). '
                   'robot.yaml wall_tap.probe 를 짧은 탐침 프리셋(axis)으로')
    if c.pose_mode == 'axis' and c.probe_len >= 100.0:
        out.append(f'[탐침] 길이 {c.probe_len:g} mm 인데 axis 방식 → 수평 자세가 팔을 접어 J3 한계에 가까움. '
                   'robot.yaml wall_tap.probe 를 긴 탐침 프리셋(tilt)으로')
    return out


def go_home(m, io, guard, policy):
    """측정 끝·실패 뒤 홈 복귀. axis 방식이고 자동 복귀면 먼저 벽에서 물러나 벽 위로 올라간 뒤 천천히 홈."""
    if policy == 'auto' and C.pose_mode == 'axis':
        try:
            guard.reset()
            m.retreat_before_home()
        except Exception as e:                  # 실패하면 기존 방식(위 40 → 뒤 40 → 홈)으로
            io.node.get_logger().warn(f'[홈 복귀] 벽 위로 물러나기 실패 → 기존 홈 복귀: {e}')
    ws.go_home_after_stop(io, guard, policy)


def main(args=None, pre=None, post=None):
    """pre(io) : 홈 이동 전에 실행할 동작 (예: pick_tap 의 물건 집기). 감시(guard) 붙기 전에 실행됨.
    post(io): 측정 성공(status ok) + 홈 복귀 뒤 실행할 동작 (예: pick_tap 의 물건 반납)."""
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
        if C.pose_mode == 'axis':            # 바닥은 수직 아래 자세로 시작 (벽 찾기 전에 수평으로 바꿈)
            a, b, c = (float(v) for v in C.floor_ori)
            ws.TOOL_ORI = [a + ws.DIRECTIONS[ws.DIRECTION][3], b, c]
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
    log.info(f'[탐침] 프리셋 {C.probe or "(없음)"} · 길이 {C.probe_len:g} mm · 자세 방식 {C.pose_mode} '
             + ('(바닥 수직 아래 → 벽·높이·너비 수평)' if C.pose_mode == 'axis' else f'(숙임 {C.tilt:.0f}° → 너비만 수평)'))
    for warn in pose_mode_warnings():
        log.warn(warn)
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
        if pre is not None:
            pre(io)
        io.gripper_close()
        io.move_j(ws.HOME_J)
        guard.zero()
        io.guard = guard                      # 이제부터 모든 이동/찍기를 감시
        log.info(f'[0] 시작 위치로 movel {list(C.start)}')
        io.move_l(np.asarray(C.start, float), vel=C.approach_vel, acc=C.approach_acc)
        floor_low = C.start[ws.UP_AXIS] - C.floor_depth
        if C.pose_mode == 'axis':
            floor_low = max(floor_low, 0.0)   # 수직 아래 자세: 바닥(z≥0) 아래는 점검할 필요 없음
        m.check_singularity([np.asarray(C.start, float),
                             ws.make_point(m.lat_c, ws.app_of(C.start), floor_low)],
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
                      ('홈 복귀 실패', lambda: go_home(m, io, guard, home_policy)),
                      ('후처리 실패', lambda: post(io) if post is not None and m.status == 'ok' else None),
                      ('특이점 처리 복구 실패', lambda: io.set_singularity(False)),
                      ('RViz 정리 실패', stop_viz))
    sys.exit(0 if m.status == 'ok' else 1)


if __name__ == '__main__':
    main()
