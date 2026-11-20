"""도구 집기 → 벽 앞으로 → 순응/힘제어 샌딩 → 도구 반납 (MoveIt 미사용, 두산 컨트롤러만)

sanding_server.py 가 이 SandingJob 으로 goal 을 실행합니다. (이 파일은 단독 실행 안 함)
  HMI(hmi_server) / sanding_client → /dsr01/tool_task 액션 → sanding_server → SandingJob
  job = SandingJob(dsr_node, logger, is_canceled=..., on_step=...)
  job.run(tool_posx, start_posx, end_posx, surface_normal, return_tool)
  job.run(tool_posx, None, None, surface_normal, return_tool, start_posj=..., work_vector=...)  # 관절각으로 시작

동작 (2026-11-20 단순화: 경로 관절 검사 없음. 도구 집기는 고정, 거치대~벽 사이 장애물 없음 전제):
  1. 경로 계산 (로봇 안 움직임): 벽 법선으로 자세(벽에 90도), 틈 시작/끝 z 는 바닥 + floor_min_tcp(50) 이상으로
  2. 도구 집기 (고정): 도구 위 → 열기 → 하강 → 닫기 → +100 → +100 → +X 250   (도구 들고 있으면 생략)
  3. 관절 이동으로 몸체 쪽 위(틈 시작 −10mm − body_side, 이동 높이)에서 작업 자세로 → 수직 하강
  4. 벽 쪽으로 수평으로 천천히(contact_speed) → 외력 contact_force 이상인 자리에서 멈춤 = 접촉
     → 원판이 고르게 닿게 자세 맞추기
  5. 순응제어 + 힘제어 ON → 틈 끝까지 좌우(±12.5mm) 왕복하며 연속 이동 (ZIGZAG_PITCH_MM 씩 전진)
  6. 힘제어 OFF → 후퇴 → 몸체 쪽으로 빠지기 → 수직 상승
     → (다음 틈 있으면 도구 든 채 끝) / (마지막이면 관절로 도구 자세 → 도구 반납 → 위로)

안전:
  - 모든 이동은 끝난 뒤 실제 위치로 도착 확인 (컨트롤러는 실패해도 '성공' 응답)
  - 샌딩 중: 외력 > FORCE_LIMIT_N / 벽보다 MAX_DEPTH_MM 이상 들어감 / 도구가 PERP_TOL_DEG 이상 기울어짐
    → 즉시 정지 → 힘제어 해제 → 후퇴
  - 취소 (HMI STOP / E-STOP, 클라이언트 Ctrl+C): 즉시 정지 → 힘제어 해제 → 그 자리에서 멈춤
    HMI 는 STOP 과 E-STOP 모두 '취소' 로 보내므로, 취소 뒤에는 기본으로 움직이지 않음
    (retreat_on_cancel=True 면 예전처럼 벽에서 후퇴)
"""
import math
import time

import rclpy
import DR_init

from dsr_msgs2.srv import MoveStop
from onrobot_rg_msgs.srv import SetCommand

from sanding import surface_math as sm
from sanding.reach_check import ReachChecker, fmt, vec6

from utils.robot import HOME_J, ROBOT_ID, ROBOT_MODEL, TCP_NAME, TOOL_NAME  # config/robot.yaml robot 섹션
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ── 도구 들기 ─────────────────────────────────────────────────────
HOME_POSJ = HOME_J   # 현재 팔 형태로 도구에 못 가면 여기로 먼저 이동
TCP_NAMES = (TCP_NAME, TCP_NAME.replace("_", " "))   # 티칭 펜던트에 등록된 TCP 이름 후보 (도구/작업 위치는 이 TCP 기준으로 교시)
APPROACH_Z = 100.0             # 도구 위 접근 높이
LIFT_Z = 100.0                 # 도구 들어올리기 (천천히)
LIFT_EXTRA_Z = 100.0           # 한 번 더 올리기 (바닥 끌림 방지)
GRAB_DOWN_MM = 5.0             # 도구 잡을 때만 도구 위치보다 이만큼 더 내려가서 잡음 (반납은 원래 위치)
TOOL_OUT_X_MM = 250.0          # 도구 들고 20cm 올라간 뒤 같은 높이로 +X 방향 movel 이동 (그 다음 관절로 자세 꺾기)
                               # (도구가 로봇 뒤쪽이라 도구 바로 위에서 관절로 돌면 팔이 무리 → 로봇 쪽으로 먼저 빼냄)

# ── 벽 접근 / 접촉 탐색 (툴 Z = 벽 쪽 방향 거리) ────────────────────
SEARCH_CHUNK_MM = 30.0         # 접촉 탐색: 한 번에 전진하는 거리 (힘 감지될 때까지 반복)
SEARCH_MAX_MM = 150.0          # 접촉 탐색 안전 한계: 이만큼 전진해도 힘이 안 잡히면 중단 (벽이 없음/위치 잘못 입력)
SEARCH_VELX, SEARCH_ACCX = 15, 30
CONTACT_DETECT_N = 5.0
RETREAT_MM = 60.0
# 틈 접근 (2026-11-20 단순화: 관절 검사 없음)
#   틈 시작에서 몸 쪽으로 START_BACK_MM + body_side 떨어진 곳에서 수직 하강 → 벽 쪽 수평으로 천천히 접촉 탐색
#   (벽 앞에서 바로 수직 하강하면 run112 처럼 충돌감지. 150mm 는 관절 한계 → 80mm, 2026-11-20 컨트롤러 IK 로 확인)
START_BACK_MM = 10.0
EXIT_PULL_MM = 10.0            # 샌딩 성공 후: 벽에서 이만큼만 뗀 뒤 한 번에 빠지며 상승 (robot.yaml sanding.exit_*)
EXIT_VELX, EXIT_ACCX = 60, 60
TOOL_SPIN_DEG = 90.0           # 작업 자세로 꺾을 때 그리퍼를 툴 Z 축으로 이만큼 돌림 (robot.yaml sanding.tool_spin)
BODY_SIDE_MM = 80.0            # robot.yaml sanding.body_side
FLOOR_MIN_TCP_MM = 50.0        # TCP 는 바닥 + 이 값 아래로 안 내려감 (도구 크기 때문에 바닥에 닿지 않게). robot.yaml sanding.floor_min_tcp
ZIGZAG_PITCH_MM = 5.0          # 샌딩: 좌우 한 번 왕복(반)마다 틈 끝 쪽으로 나아가는 거리

# ── 순응제어 / 힘제어 ──────────────────────────────────────────────
# 강성 [x, y, z (N/m, 0~20000), rx, ry, rz (Nm/rad, 0~400)] - 툴 좌표계
COMPLIANCE_STX = [3000, 3000, 200, 400, 400, 400]  # 툴 Z(누르는 방향)만 부드럽게, 회전은 최대
                                                   # (회전 강성을 15 까지 낮춰도 실제 기울기 0.1도 → 수동 순응으론 안 맞음.
                                                   #  대신 아래 '자세 맞추기'로 모멘트를 보고 직접 기울임)
PRESS_FORCE_N = 13.0           # 기본값 (HMI [샌딩 작업 실행] 의 '누르는 힘' 으로 작업마다 바꿀 수 있음)
PRESS_FORCE_MIN_N, PRESS_FORCE_MAX_N = 3.0, 30.0
FORCE_SETTLE_SEC = 0.7
FORCE_LIMIT_N = 80.0           # 누르는 힘 + 사포 마찰 포함 전체 외력 (사포가 걸리면 40 N 은 쉽게 넘음)
MAX_DEPTH_MM = 10.0
CONTACT_EVEN_MM = 10.0
# 자세 맞추기: 벽에 누른 채 멈춰서 모멘트로 압력 중심을 재고, 덜 닿는 쪽이 벽으로 가게 조금씩 기울임
ALIGN_MAX_ITERS = 8
ALIGN_DONE_MM = 8.0            # 압력 중심이 원판 중앙에서 이 안이면 완료
ALIGN_GAIN_DEG_PER_MM = 0.04   # 치우침 1mm 당 기울이는 각도 (30mm 치우침 → 1.2도)
ALIGN_STEP_MAX_DEG = 1.5       # 한 번에 최대
ALIGN_MAX_DEG = 5.0            # 처음 자세에서 최대 (PERP_TOL_DEG 보다 작게)
ALIGN_SETTLE_SEC = 0.6
ALIGN_BACKOFF_MM = 3.0         # 기울이기 전에 벽에서 떨어지는 거리 (닿은 채로 돌리면 가장자리가 긁힘)
ALIGN_SAMPLES = 25         # 모멘트 로그: 압력 중심이 원판 중앙에서 이 안이면 '고르게 닿음'
PERP_TOL_DEG = 6.0             # 샌딩 중 허용 기울기 (회전 순응으로 벽면을 따라 기우는 만큼 허용)

# ── 샌딩 패턴 ─────────────────────────────────────────────────────
SCRUB_STROKE_MM = 25.0         # 시작/종료 위치 왕복 폭 (전체 2.5cm)
SCRUB_VELX, SCRUB_ACCX = 60, 120

# ── 벽 방향 자동 판단 (surface_normal 을 안 줬을 때) ──────────────────
SNAP_AXIS_DEG = 20.0           # 시작 자세 툴 Z 가 베이스 축과 이 각도 안이면 그 축에 정확히 90도로 맞춤

# ── 도구 크기 / 바닥 여유 (원형 도구를 중앙에서 잡음) ──────────────────
TOOL_RADIUS_MM = 45.0          # 원형 도구 반지름 (TCP = 도구 중앙 → 벽 작업 때 가장자리가 TCP 보다 이만큼 아래)
FLOOR_CLEAR_MM = 1.0           # 도구 가장자리와 바닥 사이 여유 → 벽 맨 아래까지 갈리도록 작게 (0 이면 바닥에 닿음)
                               # (너무 낮은 시작/종료/중간 지점만 그 높이로 올림. 벽 윗면은 검사 안 함)

# ── 속도 / 감시 ───────────────────────────────────────────────────
REORIENT_VELJ, REORIENT_ACCJ = 20, 20   # 자세가 크게 바뀌는 관절 이동
VELX, ACCX = 150, 150
SLOW_VELX, SLOW_ACCX = 60, 60
GRIPPER_WAIT = 1.5
WARMUP_SETTLE_S = 2.0         # 컨트롤러 첫 응답 뒤 기다림 (DDS 연결 안정화)
# RG2 열기/닫기: 컨트롤러 디지털 출력 (퍼티 헤라 집기와 같은 순서, putty_demo/pickup_spatula.OPEN/CLOSE)
# (채널, 값) 차례로 출력. OnRobot(/onrobot/sendCommand, pymodbus 필요) 없이 동작한다.
# robot.yaml sanding.gripper: onrobot 이면 예전 방식.
GRIPPER_IO = {"o": ((1, 0), (2, 0), (2, 1)), "c": ((1, 0), (2, 0), (1, 1), (2, 0))}
POLL_DT = 0.02
MOTION_START_TIMEOUT = 0.5
MOTION_TIMEOUT = 60.0
ARRIVE_TOL_MM, ARRIVE_TOL_DEG = 3.0, 2.0
SANDING_TOL_MM = 8.0           # 샌딩 중(힘제어) 도착 허용 오차 - 사포 마찰로 경로보다 몇 mm 뒤처지는 건 정상
WORK_TOL_DEG = 0.5

# 진행 단계 (피드백 step 번호)
PHASES = ["계획", "도구 집기", "작업 위치로 이동", "접촉 탐색", "샌딩", "후퇴", "도구 반납"]


class Abort(Exception):
    """샌딩 중단 (충돌 / 표면 이탈 / 기울어짐 / 접촉 실패) - 후퇴 완료, 도구는 들고 있음"""


class Canceled(Exception):
    pass


class MotionFailed(RuntimeError):
    pass


class PlanError(Exception):
    """계획 단계에서 불가 (로봇 안 움직임)"""


class SandingJob:
    """도구 집기 + 샌딩 + 반납. dsr_node 는 DSR_ROBOT2 가 직접 spin 하는 노드.

    is_canceled() → True 면 이동 중 즉시 정지 / 단계 사이에서 중단
    on_step(phase_index, total, text) → 진행 상황 알림 (액션 피드백 등)
    """

    def __init__(self, dsr_node, logger, is_canceled=lambda: False, on_step=None, retreat_on_cancel=False,
                 go_home=False, floor_z=None, floor_clear=FLOOR_CLEAR_MM, tool_radius=TOOL_RADIUS_MM, virtual=False):
        self.node, self.log = dsr_node, logger
        self.virtual = virtual   # sodvir(에뮬레이터, 외력 0): 접촉 = 목표 위치 도착, 자세 맞추기·힘제어 생략
        self.is_canceled, self.on_step = is_canceled, on_step
        self.retreat_on_cancel = retreat_on_cancel
        self.go_home = go_home   # True 면 항상 홈 자세부터 (False 면 필요할 때만)
        from utils.config import CFG as _CFG
        self.body_side_mm = float((_CFG.get("sanding") or {}).get("body_side", BODY_SIDE_MM))
        if not 0.0 <= self.body_side_mm <= 400.0:
            raise ValueError(f"robot.yaml sanding.body_side 는 0~400 mm: {self.body_side_mm}")
        scfg = _CFG.get("sanding") or {}
        self.contact_speed = float(scfg.get("contact_speed", SEARCH_VELX))     # 접촉 탐색 속도 mm/s
        self.contact_force = float(scfg.get("contact_force", CONTACT_DETECT_N))  # 이 이상이면 접촉 N
        self.contact_max = float(scfg.get("contact_max", SEARCH_MAX_MM))        # 이만큼 가도 안 닿으면 중단 mm
        if not (0 < self.contact_speed <= 30 and 0 < self.contact_force <= 30 and 0 < self.contact_max <= 300):
            raise ValueError("robot.yaml sanding.contact_speed(0~30) / contact_force(0~30) / contact_max(0~300) 확인")
        self.start_back = float(scfg.get("start_back", START_BACK_MM))   # 틈 시작에서 몸체 쪽으로 mm
        if not 0 <= self.start_back <= 100:
            raise ValueError("robot.yaml sanding.start_back(0~100) 확인")
        self.tool_spin = float(scfg.get("tool_spin", TOOL_SPIN_DEG))   # 작업 자세에서 그리퍼를 툴 Z 축으로 돌리는 각 deg
        if not -180 <= self.tool_spin <= 180:
            raise ValueError("robot.yaml sanding.tool_spin(-180~180) 확인")
        self.exit_pull = float(scfg.get("exit_pull", EXIT_PULL_MM))      # 샌딩 후 벽에서 떼는 거리 mm
        self.exit_speed = float(scfg.get("exit_speed", EXIT_VELX))     # 샌딩 후 빠지며 상승 속도 mm/s
        self.exit_acc = float(scfg.get("exit_acc", EXIT_ACCX))         # 그 가속도 mm/s² (작을수록 부드러움)
        if not (0 < self.exit_pull <= 60 and 0 < self.exit_speed <= 150 and 0 < self.exit_acc <= 300):
            raise ValueError("robot.yaml sanding.exit_pull(0~60) / exit_speed(0~150) / exit_acc(0~300) 확인")
        self.floor_min_tcp = float((_CFG.get("sanding") or {}).get("floor_min_tcp", FLOOR_MIN_TCP_MM))
        self.q_out = None        # 도구 집고 +X 로 빼낸 자세의 관절 (반납 때 되돌아감)
        self.floor_z = floor_z   # 작업하는 벽 아래 바닥 높이 (베이스 z, mm). None 이면 바닥 여유 검사 안 함
        self.floor_clear = floor_clear   # 도구 아래끝과 바닥 사이 여유 (mm)
        self.tool_radius = tool_radius   # 원형 도구 반지름 (mm)
        self.press_force = PRESS_FORCE_N   # 샌딩 중 벽 쪽 누르는 힘 (N) - 작업마다 서버가 HMI 값으로 바꿈
        import DSR_ROBOT2 as dr
        from DR_common2 import posj, posx
        self.dr, self.posx, self.posj = dr, posx, posj
        from utils.config import CFG
        self.gripper_mode = str((CFG.get("sanding") or {}).get("gripper", "io")).lower()
        if self.gripper_mode not in ("io", "onrobot"):
            raise ValueError(f"robot.yaml sanding.gripper 는 io 또는 onrobot: {self.gripper_mode}")
        self.gripper_cli = (dsr_node.create_client(SetCommand, "/onrobot/sendCommand")
                            if self.gripper_mode == "onrobot" else None)
        self.stop_cli = dsr_node.create_client(MoveStop, "dsr_controller2/motion/move_stop")
        self.reach = ReachChecker(dr, posx, logger)
        self.force_active = False
        self.phase = 0

    def wait_services(self, timeout=30.0):
        """Wi-Fi + discovery server(다른 노트북의 sodreal) 로 찾으면 몇 초~십수 초 걸려서 넉넉히 기다림"""
        clients = [(self.stop_cli, "move_stop")]
        if self.gripper_cli is not None:
            clients.insert(0, (self.gripper_cli, "/onrobot/sendCommand"))
        for cli, name in clients:
            waited = 0.0
            while not cli.wait_for_service(timeout_sec=5.0):
                waited += 5.0
                if waited >= timeout:
                    raise RuntimeError(f"{name} 서비스가 없습니다 ({timeout:.0f}초 대기). sodreal 실행 / "
                                       f"ROS_DOMAIN_ID·ROS_DISCOVERY_SERVER 가 sodreal 쪽과 같은지 확인")
                self.log.info(f"{name} 서비스 찾는 중... ({waited:.0f}초)")
        self.warm_up(timeout)

    def warm_up(self, timeout=30.0, settle=WARMUP_SETTLE_S):
        """컨트롤러가 실제로 응답하는지 시간 제한을 두고 한 번 물어본다.

        DSR_ROBOT2 함수(release_force, get_current_tcp …)는 시간 제한 없이 응답을 기다린다.
        bringup 직후(dsr_controller2 활성화 전)나 DDS 연결이 덜 된 상태에서 보내면 응답이 사라져 영원히 멈춘다
        (2026-11-20 17:02: bringup 재시작 13초 뒤 goal → '[1/7] 계획'에서 멈춤).
        """
        from dsr_msgs2.srv import GetCurrentPosj
        cli = self.node.create_client(GetCurrentPosj, "dsr_controller2/aux_control/get_current_posj")
        t0 = time.monotonic()
        try:
            while True:
                if cli.wait_for_service(timeout_sec=1.0):
                    fut = cli.call_async(GetCurrentPosj.Request())
                    rclpy.spin_until_future_complete(self.node, fut, timeout_sec=3.0)
                    if fut.done() and fut.result() is not None and fut.result().success:
                        break
                waited = time.monotonic() - t0
                if waited >= timeout:
                    raise RuntimeError(f"컨트롤러가 {timeout:.0f}초 동안 응답하지 않습니다 (get_current_posj). "
                                       "bringup 이 완전히 켜졌는지 확인")
                self.log.info(f"컨트롤러 응답 기다리는 중... ({waited:.0f}초)")
        finally:
            self.node.destroy_client(cli)
        time.sleep(settle)      # 다른 서비스들의 DDS 연결도 마저 잡히도록
        self.log.info(f"컨트롤러 응답 확인 ({time.monotonic() - t0:.1f}초)")

    # ── 기본 ─────────────────────────────────────────────────────
    def step(self, msg, phase=None):
        if phase is not None:
            self.phase = phase
            if self.is_canceled():
                raise Canceled(PHASES[phase - 1])
        self.log.info(f"▶ [{self.phase}/{len(PHASES)}] {msg}")
        if self.on_step:
            self.on_step(self.phase, len(PHASES), msg)

    def alarm(self):
        try:
            a = self.dr.get_last_alarm()
            return f"index={a.index}, {' '.join(x for x in a.param if x)}"
        except Exception:
            return "알람 조회 실패"

    def check_ret(self, ret, what):
        if ret not in (None, 0):
            raise MotionFailed(f"{what} 실패 (ret={ret}, 알람: {self.alarm()})")

    def current(self):
        p, _ = self.dr.get_current_posx(ref=self.dr.DR_BASE)
        if p is None:
            raise RuntimeError("get_current_posx 실패")
        return [float(v) for v in list(p)[:6]]

    def current_q(self):
        q = vec6(self.dr.get_current_posj())
        if q is None:
            raise RuntimeError("get_current_posj 실패")
        return q

    def fk(self, q):
        p = vec6(self.dr.fkin(self.posj(list(q)), ref=self.dr.DR_BASE))
        if p is None:
            raise RuntimeError("fkin 실패")
        return p

    def verify(self, target, what, skip_axis=None, tol_deg=ARRIVE_TOL_DEG, tol_mm=ARRIVE_TOL_MM):
        cur = self.current()
        err = sm.sub(cur[:3], target[:3])
        if skip_axis is not None:
            err = sm.sub(err, sm.scale(skip_axis, sm.dot(err, skip_axis)))
        rc, rt = sm.zyz_to_matrix(cur[3:]), sm.zyz_to_matrix(target[3:])
        c = (sum(rc[i][j] * rt[i][j] for i in range(3) for j in range(3)) - 1.0) / 2.0
        ang = math.degrees(math.acos(max(-1.0, min(1.0, c))))
        if sm.norm(err) > tol_mm or ang > tol_deg:
            raise MotionFailed(f"{what}: 목표 미도착 (오차 {sm.norm(err):.1f}mm / {ang:.1f}도), "
                               f"목표 {fmt(target)}, 현재 {fmt(cur)}, 알람: {self.alarm()}")

    def movel(self, pose, vel, acc, what):
        self.check_ret(self.dr.movel(self.posx(list(pose)), vel=vel, acc=acc, ref=self.dr.DR_BASE), what)
        self.verify(pose, what)

    def movej(self, q, vel, acc, what):
        self.log.info(f"   {what}: q={fmt(q)} ({vel} deg/s)")
        self.check_ret(self.dr.movej(self.posj(list(q)), vel=vel, acc=acc), what)
        time.sleep(0.2)
        q_now = self.current_q()
        if max(abs(a - b) for a, b in zip(q_now, q)) > 1.0:
            raise MotionFailed(f"{what}: 목표 관절 미도착, 목표 {fmt(q)}, 현재 {fmt(q_now)}, 알람: {self.alarm()}")

    def gripper(self, cmd):
        if self.gripper_cli is None:        # 디지털 출력 (기본)
            for channel, value in GRIPPER_IO[cmd]:
                if self.dr.set_digital_output(channel, value) != 0:
                    raise RuntimeError(f"그리퍼 '{cmd}' 실패: 디지털 출력 {channel}번 = {value}")
            self.log.info(f"그리퍼 {'열기' if cmd == 'o' else '닫기'} (디지털 출력 {list(GRIPPER_IO[cmd])})")
            time.sleep(GRIPPER_WAIT)
            return
        req = SetCommand.Request()
        req.command = cmd
        fut = self.gripper_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        if not fut.done() or not fut.result().success:
            raise RuntimeError(f"그리퍼 '{cmd}' 실패")
        time.sleep(GRIPPER_WAIT)

    def stop_motion(self):
        req = MoveStop.Request()
        req.stop_mode = self.dr.DR_QSTOP
        fut = self.stop_cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=2.0)
        if not fut.done() or not fut.result().success:
            self.log.error("move_stop 실패 - 교시기로 로봇을 정지하세요!")
        time.sleep(0.3)

    def force(self):
        f = vec6(self.dr.get_tool_force(ref=self.dr.DR_TOOL))
        if f is None:
            raise RuntimeError("get_tool_force 실패")
        self.last_ft = f   # 툴 좌표계 [Fx, Fy, Fz, Mx, My, Mz] (모멘트 로그용)
        return f[:3]

    def force_avg(self, n=10):
        acc = [0.0, 0.0, 0.0]
        macc = [0.0, 0.0, 0.0]
        for _ in range(n):
            acc = sm.add(acc, self.force())
            macc = sm.add(macc, self.last_ft[3:6])
            time.sleep(POLL_DT)
        self.moment_baseline = sm.scale(macc, 1.0 / n)   # 접촉 전 모멘트 (도구 무게 등) → 샌딩 중 빼고 봄
        return sm.scale(acc, 1.0 / n)

    def contact_sample(self, baseline, tilt):
        """샌딩 중 한 순간의 접촉 상태: (Fz, Mx, My, tilt) - 접촉 전 기준값을 뺀 값"""
        ft = self.last_ft
        mb = getattr(self, "moment_baseline", [0.0, 0.0, 0.0])
        return (ft[2] - baseline[2], ft[3] - mb[0], ft[4] - mb[1], tilt)

    def log_contact(self, what, samples, orient):
        """이동 하나 동안의 모멘트 요약 + 압력 중심 (원판 전체가 고르게 눌리면 압력 중심 ≈ 원판 중앙)
        누르는 힘 Fz 가 점 (x, y) 에 작용하면 Mx = y·Fz, My = -x·Fz → x = -My/Fz, y = Mx/Fz (툴 좌표계)"""
        if not samples:
            return
        n = len(samples)
        fz = sum(s[0] for s in samples) / n
        mx = sum(s[1] for s in samples) / n
        my = sum(s[2] for s in samples) / n
        mx_pk = max(abs(s[1]) for s in samples)
        my_pk = max(abs(s[2]) for s in samples)
        tilt_pk = max(s[3] for s in samples)
        msg = (f"   [모멘트] {what}: Mx 평균 {mx:+.2f} (최대 {mx_pk:.2f}) / My 평균 {my:+.2f} (최대 {my_pk:.2f}) Nm, "
               f"Fz 평균 {fz:+.1f} N, 기울기 최대 {tilt_pk:.1f}도")
        if abs(fz) >= 2.0:
            x_mm, y_mm = -my / fz * 1000.0, mx / fz * 1000.0
            tx, ty, _ = sm.tool_axes(orient)
            cop = sm.add(sm.scale(tx, x_mm), sm.scale(ty, y_mm))   # 베이스 좌표계 방향으로 바꾼 압력 중심 치우침
            off = math.hypot(x_mm, y_mm)
            if off > self.tool_radius:
                state = "원판 밖 - 이동 중 사포 마찰 비틀림이 커서 접촉 판단 불가"
            else:
                state = "고르게 닿음" if off < CONTACT_EVEN_MM else ("한쪽 치우침" if off < self.tool_radius * 0.6 else "한쪽만 닿음 의심")
            msg += (f", 압력중심 치우침 {off:.0f}mm → 베이스 X {cop[0]:+.0f} Y {cop[1]:+.0f} Z {cop[2]:+.0f}mm ({state})")
        else:
            msg += ", 누르는 힘이 작아 압력중심 계산 안 함"
        self.log.info(msg)

    def pressure_center(self, baseline):
        """멈춘 상태에서 평균 낸 (Fz, x, y): 압력 중심 (툴 좌표계 mm). Fz 가 작으면 x, y = None"""
        mb = getattr(self, "moment_baseline", [0.0, 0.0, 0.0])
        fz = mx = my = 0.0
        for _ in range(ALIGN_SAMPLES):
            self.force()
            ft = self.last_ft
            fz += ft[2] - baseline[2]
            mx += ft[3] - mb[0]
            my += ft[4] - mb[1]
            time.sleep(POLL_DT)
        fz, mx, my = fz / ALIGN_SAMPLES, mx / ALIGN_SAMPLES, my / ALIGN_SAMPLES
        if abs(fz) < 2.0:
            return fz, None, None
        return fz, -my / fz * 1000.0, mx / fz * 1000.0

    def seek_contact(self, orient, tz, baseline, what, target=None, direction=None):
        """direction(기본 툴 Z = 벽 쪽)으로 천천히(contact_speed) 전진하다가 외력이 contact_force 이상이면
        그 자리에서 멈춤. SEARCH_CHUNK_MM 씩 이어서 전진 (안전 한계 contact_max). 닿으면 True
        virtual: target(벽 위 점) 으로 직선 이동 후 닿은 것으로 봄"""
        dr = self.dr
        speed = getattr(self, "contact_speed", SEARCH_VELX)
        if self.virtual and target is not None:
            self.movel(list(target) + list(orient), speed, SEARCH_ACCX, f"{what} (가상: 벽 위치로 이동)")
            return True
        limit = self.contact_max
        d = tz if direction is None else direction
        start = self.current()[:3]
        moved = 0.0
        while moved < limit - 1e-6:
            step = min(SEARCH_CHUNK_MM, limit - moved)
            target = sm.add(self.current()[:3], sm.scale(d, step)) + list(orient)
            if self.guarded(lambda t=target: dr.amovel(self.posx(t), vel=speed, acc=SEARCH_ACCX, ref=dr.DR_BASE),
                            baseline, stop_force=self.contact_force, what=what):
                return True
            moved = sm.dot(sm.sub(self.current()[:3], start), d)
            if moved < limit - 1e-6:
                self.log.info(f"   {what}: {moved:.0f}mm 전진, 아직 힘 감지 없음 → 계속 전진")
        return False

    def align_to_wall(self, orient, baseline, tz):
        """힘제어 켜기 전, 벽에 닿은 채 멈춘 상태에서 모멘트로 압력 중심을 재고 원판 전체가 닿게 자세를 맞춤.
        (힘제어 중에는 툴 힘 값이 거의 0 으로 나와 못 씀 → 위치 제어로 닿은 상태에서 잼)
        한 번: 압력 중심 측정 → 벽에서 ALIGN_BACKOFF_MM 뒤로 → 기울이기 → 다시 접촉 탐색.
        압력 중심 r=(x, y) 쪽이 더 세게 닿음 → 툴축 (-y, x) 둘레로 돌리면 r 쪽이 벽에서 멀어지고 반대쪽이 다가감.
        센서 부호가 반대일 수 있으므로 치우침이 오히려 커지면 방향을 뒤집음. 맞춘 자세를 돌려줌."""
        if self.virtual:
            self.log.info("   자세 맞추기: 가상 모드 (외력 0) → 생략")
            return orient
        start = list(orient)

        def tilt_from_start(o):
            return math.degrees(math.acos(max(-1.0, min(1.0, sm.dot(sm.tool_axes(o)[2], sm.tool_axes(start)[2])))))

        sign, prev = 1.0, None
        for it in range(1, ALIGN_MAX_ITERS + 1):
            time.sleep(ALIGN_SETTLE_SEC)
            fz, x, y = self.pressure_center(baseline)
            if x is None:
                self.log.warn(f"   자세 맞추기: 누르는 힘이 작음 (Fz {fz:+.1f} N) - 지금 자세로 진행")
                break
            off = math.hypot(x, y)
            tx, ty, _ = sm.tool_axes(orient)
            cop = sm.add(sm.scale(tx, x), sm.scale(ty, y))
            self.log.info(f"   자세 맞추기 {it}: 압력중심 치우침 {off:.0f}mm (베이스 Y {cop[1]:+.0f} Z {cop[2]:+.0f}), "
                          f"Fz {fz:+.1f} N, 처음 자세 대비 {tilt_from_start(orient):.1f}도")
            if off < ALIGN_DONE_MM:
                self.log.info("   자세 맞추기 완료: 원판 전체가 고르게 닿음")
                break
            if prev is not None and off > prev + 2.0:
                sign = -sign
                self.log.warn("   치우침이 커짐 → 기울이는 방향을 반대로")
            ang = sign * min(ALIGN_STEP_MAX_DEG, ALIGN_GAIN_DEG_PER_MM * off)
            new = sm.rotate_about_axis(orient, sm.add(sm.scale(tx, -y), sm.scale(ty, x)), ang)
            if tilt_from_start(new) > ALIGN_MAX_DEG:
                self.log.warn(f"   자세 맞추기: 처음 자세에서 {ALIGN_MAX_DEG}도 넘게 기울어야 함 - 여기서 멈춤 "
                              "(벽이 많이 틀어졌거나 도구가 그리퍼에서 돌아갔을 수 있음)")
                break
            here = self.current()[:3]
            back = sm.sub(here, sm.scale(tz, ALIGN_BACKOFF_MM))
            self.movel(back + list(orient), SLOW_VELX, SLOW_ACCX, f"자세 맞추기 {it}: 벽에서 {ALIGN_BACKOFF_MM:.0f}mm 뒤로")
            self.movel(back + list(new), SLOW_VELX, SLOW_ACCX, f"자세 맞추기 {it}: {ang:+.1f}도 기울이기")
            orient, prev = new, off
            if not self.seek_contact(new, tz, baseline, f"자세 맞추기 {it}: 다시 접촉"):
                raise Abort(f"자세 맞추기 중 {self.contact_max:.0f}mm 전진해도 벽에 닿지 않았습니다.")
        else:
            self.log.warn(f"   자세 맞추기: {ALIGN_MAX_ITERS}번 해도 {ALIGN_DONE_MM}mm 안으로 안 들어옴 - 지금 자세로 진행")
        return orient

    # ── 감시하며 이동 ─────────────────────────────────────────────
    def guarded(self, start_motion, baseline, target=None, what="이동",
                stop_force=None, limit_force=None, plane=None):
        self.check_ret(start_motion(), what)
        t0 = time.time()
        started = False
        samples = []
        try:
            return self._guarded_loop(t0, started, samples, baseline, target, what, stop_force, limit_force, plane)
        finally:
            if plane is not None:
                try:   # 로그 실패가 원래 오류(중단 사유)를 가리지 않게
                    self.log_contact(what, samples, self.current()[3:])
                except Exception as ex:
                    self.log.warn(f"   [모멘트] 로그 실패: {ex}")

    def _guarded_loop(self, t0, started, samples, baseline, target, what, stop_force, limit_force, plane):
        while True:
            if self.dr.check_motion() == self.dr.DR_STATE_IDLE:
                if started or time.time() - t0 > MOTION_START_TIMEOUT:
                    if stop_force is not None:
                        f = sm.norm(sm.sub(self.force(), baseline))
                        if f >= stop_force:
                            self.log.info(f"   접촉 감지 (이동 종료 시점): {f:.1f} N")
                            return True
                    if target is not None:
                        self.verify(target, what, skip_axis=None if plane is None else plane[1],
                                    tol_deg=ARRIVE_TOL_DEG if plane is None else PERP_TOL_DEG + ARRIVE_TOL_DEG,
                                    tol_mm=ARRIVE_TOL_MM if plane is None else SANDING_TOL_MM)
                    return False
            else:
                started = True

            f = sm.norm(sm.sub(self.force(), baseline))
            if stop_force is not None and f >= stop_force:
                self.stop_motion()
                self.log.info(f"   접촉 감지: 외력 {f:.1f} N")
                return True
            if limit_force is not None and f >= limit_force:
                self.stop_motion()
                fv = sm.sub(self.last_ft[:3], baseline)
                raise Abort(f"외력 {f:.1f} N > {limit_force} N (충돌 의심) - 툴 기준 누르는 방향 Fz {fv[2]:+.1f} N, "
                            f"옆 방향 {math.hypot(fv[0], fv[1]):.1f} N")
            if plane is not None:
                cur = self.current()
                depth = sm.dot(sm.sub(cur[:3], plane[0]), plane[1])
                if depth > MAX_DEPTH_MM:
                    self.stop_motion()
                    raise Abort(f"벽보다 {depth:.1f} mm 더 들어감 (표면 이탈 의심)")
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, sm.dot(sm.tool_axes(cur[3:])[2], plane[1])))))
                samples.append(self.contact_sample(baseline, tilt))
                if tilt > PERP_TOL_DEG:
                    self.stop_motion()
                    raise Abort(f"도구가 벽 수직에서 {tilt:.1f}도 기울어짐 (> {PERP_TOL_DEG}도)")
            if self.is_canceled():
                self.stop_motion()
                raise Canceled("이동 중")
            if time.time() - t0 > MOTION_TIMEOUT:
                self.stop_motion()
                raise Abort(f"{what}: 이동 시간 초과")
            time.sleep(POLL_DT)

    # ── 순응 / 힘제어 ────────────────────────────────────────────
    def force_on(self):
        if self.virtual:
            self.log.info("   힘제어: 가상 모드 → 생략 (위치 제어로 샌딩)")
            return
        dr = self.dr
        dr.set_ref_coord(dr.DR_TOOL)   # 순응/힘 방향을 툴 좌표계로 (툴 Z = 벽 쪽)
        try:
            if dr.task_compliance_ctrl(stx=COMPLIANCE_STX) not in (None, 0):
                raise RuntimeError("task_compliance_ctrl 실패")
            self.force_active = True
            time.sleep(0.3)
            if dr.set_desired_force(fd=[0, 0, float(self.press_force), 0, 0, 0], dir=[0, 0, 1, 0, 0, 0],
                                    mod=dr.DR_FC_MOD_REL) not in (None, 0):
                raise RuntimeError("set_desired_force 실패")
        except BaseException:
            dr.set_ref_coord(dr.DR_BASE)
            raise
        # 힘제어 중에는 set_ref_coord 가 거부됨 (DSR 1903 경고) → 해제 후 force_off 에서 BASE 로 되돌림.
        # 이동/위치 읽기는 전부 ref=DR_BASE 를 직접 넘기므로 영향 없음
        time.sleep(FORCE_SETTLE_SEC)

    def ensure_tcp(self):
        """도구 위치(TOOL_POSX)·작업 위치는 GripperDA_v1 TCP 기준으로 교시한 값 → 작업 전에 Tool/TCP 를 선택.
        (TCP 가 꺼져 있으면 같은 좌표로 보내도 그리퍼 끝이 TCP 길이만큼 더 아래로 가서 도구/거치대에 부딪힘)
        팀 sun_gear(실제 로봇에서 동작)와 같은 순서: 수동 모드 → set_tool → set_tcp → 자동 모드.
        (자동 모드에서는 컨트롤러가 Tool/TCP 변경을 거부함 - ret=-1)"""
        dr = self.dr
        tool_now, tcp_now = str(dr.get_tool()).strip(), str(dr.get_tcp()).strip()
        if tool_now == TOOL_NAME and tcp_now in TCP_NAMES:
            self.log.info(f"   Tool='{tool_now}', TCP='{tcp_now}' 이미 선택됨")
        else:
            tries = [f"처음 Tool='{tool_now}' TCP='{tcp_now}'"]
            tries.append(f"수동모드 ret={dr.set_robot_mode(dr.ROBOT_MODE_MANUAL)}")
            time.sleep(0.5)
            try:
                if tool_now != TOOL_NAME:
                    tries.append(f"set_tool('{TOOL_NAME}') ret={dr.set_tool(TOOL_NAME)}")
                if tcp_now not in TCP_NAMES:
                    for name in TCP_NAMES:
                        ret = dr.set_tcp(name)
                        tries.append(f"set_tcp('{name}') ret={ret}")
                        if ret == 0:
                            break
            finally:
                tries.append(f"자동모드 ret={dr.set_robot_mode(dr.ROBOT_MODE_AUTONOMOUS)}")
                time.sleep(0.5)
            tool_now, tcp_now = str(dr.get_tool()).strip(), str(dr.get_tcp()).strip()
            self.log.info(f"   Tool/TCP 설정: {' / '.join(tries)} → Tool='{tool_now}', TCP='{tcp_now}'")
            if tool_now != TOOL_NAME:
                self.log.warn(f"   Tool '{TOOL_NAME}' 선택 안 됨 → Tool 없이 진행 (충돌 감지가 민감할 수 있음)")
            if tcp_now not in TCP_NAMES:
                raise PlanError(f"TCP 를 '{TCP_NAMES[0]}' 으로 바꾸지 못했습니다 (현재 '{tcp_now}'). 로봇은 움직이지 않았습니다. "
                                "로봇 상태가 STANDBY 인지 확인하세요")
        self.log.info(f"   현재 위치 {fmt(self.current())}, 현재 관절 {fmt(self.current_q())}")

    def reset_control_mode(self):
        """작업 시작 전 컨트롤러에 남아 있을 수 있는 힘제어/순응제어를 해제하고 좌표계를 BASE 로.
        (이전 작업이 SAFE_STOP·서버 종료 등으로 끝나면 순응제어가 켜진 채 남을 수 있음 →
         그 상태로 도구 쪽으로 가면 툴 Z(아래) 강성이 200 N/m 라 팔이 아래로 처져 내리꽂힘)"""
        dr = self.dr
        for name, fn in (("release_force", dr.release_force), ("release_compliance_ctrl", dr.release_compliance_ctrl)):
            try:
                fn()
            except Exception as ex:
                self.log.warn(f"   {name} 실패(무시): {ex}")
        try:
            dr.set_ref_coord(dr.DR_BASE)
        except Exception as ex:
            self.log.warn(f"   set_ref_coord(BASE) 실패(무시): {ex}")
        self.force_active = False
        time.sleep(0.3)

    def force_off(self):
        if not self.force_active:
            return
        try:
            self.dr.release_force()
        finally:
            self.dr.release_compliance_ctrl()
            self.dr.set_ref_coord(self.dr.DR_BASE)
            self.force_active = False

    # ── 1. 경로 계산 ──────────────────────────────────────────────
    @staticmethod
    def auto_normal(tool_z):
        """툴 Z 에 가장 가까운 베이스 축 (±X, ±Y, -Z) 이 SNAP_AXIS_DEG 안이면 그 면의 법선, 아니면 None"""
        axes = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, -1]]
        best = max(axes, key=lambda a: sm.dot(a, tool_z))
        ang = math.degrees(math.acos(max(-1.0, min(1.0, sm.dot(best, tool_z)))))
        return (sm.scale(best, -1.0), ang) if ang <= SNAP_AXIS_DEG else (None, ang)

    @staticmethod
    def scrub_dir(u, v):
        """시작/종료 왕복 방향: 진행(u)·좌우(v) 중 더 수평인 쪽 (위아래로 왕복하지 않음).
        벽에서 Y 로 진행 → u(Y), Z 로 진행 → v(Y). 바닥 작업처럼 둘 다 수평이면 진행 방향 u"""
        return u if abs(u[2]) <= abs(v[2]) + 1e-9 else v

    def plan(self, tool, start_posx, end_posx, normal, start_posj=None, work_vector=None):
        """경로 계산 (관절 검사 없음). 도구 집기는 고정 동작, 거치대~벽 사이에 장애물이 없다는 전제.

        틈 시작 s → 끝 e. 벽 법선(tz, 벽 쪽) 기준으로
          하강 위치 = s − 수평(tz) × (start_back + body_side)  (몸체 쪽에서 수직 하강)
          거기서 벽 쪽으로 수평으로 천천히 → 저항을 받은 자리가 접촉 위치 (위치 목표 없음)
        TCP 는 바닥 + FLOOR_MIN_TCP_MM 아래로 내려가지 않는다 (도구 크기 때문에 바닥에 닿지 않게).
        """
        if start_posj is not None:
            fk = self.fk(start_posj)
            self.log.info(f"시작 관절 {fmt(start_posj, 2)} → TCP {fmt(fk)} (fkin)")
            start_posx = fk
            end_posx = sm.add(fk[:3], list(work_vector)) + fk[3:]
        start_posx, end_posx = list(start_posx), list(end_posx)
        taught_z = sm.tool_axes(start_posx[3:])[2]
        if normal is None or sm.norm(normal) < 1e-6:
            normal, ang = self.auto_normal(taught_z)
            if normal is None:
                self.log.warn(f"툴 Z 가 베이스 축과 {ang:.1f}도 차이 (> {SNAP_AXIS_DEG}도) - 자세 보정 없이 그대로 사용")
        orient = sm.orient_toward_surface(start_posx[3:], normal) if normal is not None else start_posx[3:]
        spin = getattr(self, "tool_spin", TOOL_SPIN_DEG)
        if abs(spin) > 1e-6:   # 그리퍼를 툴 Z(벽 쪽) 축으로 돌림 → 벽에 90도는 그대로
            orient = sm.rotate_about_axis(orient, sm.tool_axes(orient)[2], spin)
        tx, _, tz = sm.tool_axes(orient)

        s = start_posx[:3]
        e = sm.project_to_plane(end_posx[:3], s, tz)
        min_z = None if self.floor_z is None else self.floor_z + self.floor_min_tcp
        for name, p in (("시작", s), ("끝", e)):
            if min_z is not None and p[2] < min_z:
                self.log.warn(f"틈 {name} z={p[2]:.1f} → z={min_z:.1f} 로 올림 (TCP 는 바닥 {self.floor_z:.1f} + "
                              f"{self.floor_min_tcp:.0f}mm 위까지만)")
                p[2] = min_z
        u, v, length = sm.path_frame(s, e, tz, tx)
        # 좌우 왕복 방향 = 벽면 위 수평
        side = sm.unit(sm.cross([0.0, 0.0, 1.0], tz)) if abs(tz[2]) < 0.7 else self.scrub_dir(u, v)
        half = SCRUB_STROKE_MM / 2.0

        transit_z = tool[2] + LIFT_Z + LIFT_EXTRA_Z
        high = [tool[0], tool[1], transit_z] + tool[3:]
        out = [tool[0] + TOOL_OUT_X_MM, tool[1], transit_z] + tool[3:]
        flat = [tz[0], tz[1], 0.0]
        flat = sm.scale(flat, 1.0 / sm.norm(flat)) if sm.norm(flat) > 1e-6 else [0.0, 0.0, 0.0]
        body = self.body_side_mm
        back = getattr(self, "start_back", START_BACK_MM)
        approach = sm.sub(s, sm.scale(flat, back))
        approach_body = sm.sub(approach, sm.scale(flat, body))
        approach_high = [approach_body[0], approach_body[1], transit_z]
        self.log.info(f"작업: 틈 {fmt(s)} → {fmt(e)} ({length:.0f}mm) 를 좌우 ±{half:.1f}mm 왕복하며 연속 이동 | "
                      f"하강 {fmt(approach_high)} → {fmt(approach_body)} → 벽 쪽 수평 탐색"
                      + (f" | TCP 최저 z {min_z:.0f}" if min_z is not None else ""))
        return dict(orient=orient, tz=tz, u=u, s=s, e=e, side=side, half=half, length=length, min_z=min_z,
                    high=high, out=out, body_flat=flat, body_side=body, transit_z=transit_z,
                    approach=approach, approach_body=approach_body, approach_high=approach_high)

    def zigzag(self, s, e, u, side, half, length):
        """s → e 로 ZIGZAG_PITCH_MM 씩 나아가며 좌우(±half) 번갈아 찍은 점들. 마지막은 e (가운데)."""
        n = max(1, int(math.ceil(length / ZIGZAG_PITCH_MM)))
        pts = []
        for k in range(1, n + 1):
            c = sm.add(s, sm.scale(u, length * k / n))
            pts.append(sm.add(c, sm.scale(side, half if k % 2 else -half)))
        pts.append(list(e))
        return pts

    def turn_joints(self, pose, what):
        """작업 자세 pose 의 관절값: 지금 관절과 가장 가까운 해 (sol 0~7). 관절 이동(movej) 목표로만 쓴다."""
        q_now = self.current_q()
        best = None
        for sol in range(8):
            q, why = self.reach.ik(pose, sol)
            if q is None:
                continue
            cost = sum(abs(a - b) for a, b in zip(q, q_now))
            if best is None or cost < best[0]:
                best = (cost, sol, q)
        if best is None:
            raise PlanError(f"{what}: {fmt(pose)} 의 관절값을 못 구함 (body_side / 틈 좌표 확인)")
        self.log.info(f"   {what}: sol {best[1]} q={fmt(best[2])}")
        return best[2]

    # ── 전체 실행 ────────────────────────────────────────────────
    def run(self, tool, start_posx, end_posx, normal=None, return_tool=True, start_posj=None, work_vector=None,
            holding=False):
        """도구 집기(고정) → 몸체 쪽에서 하강 → 틈 시작 −10mm → 접촉 → 틈 끝까지 좌우 왕복하며 연속 샌딩 → 빠지기.
        holding=True: 앞 틈에서 도구를 들고 몸체 쪽 위에 있음 → 집기·자세 꺾기 없이 같은 높이로 이 틈으로 이동
        return_tool=False: 끝나고 몸체 쪽 위에서 도구를 든 채 대기 (다음 틈)"""
        dr = self.dr
        self.step("준비: TCP 확인 + 경로 계산 (로봇 안 움직임)", phase=1)
        self.reset_control_mode()   # 남아 있는 순응/힘제어 해제 (안 하면 첫 이동에서 팔이 처질 수 있음)
        self.ensure_tcp()
        geo = self.plan(tool, start_posx, end_posx, normal, start_posj, work_vector)
        orient, tz = geo["orient"], geo["tz"]

        def pose(p):
            return list(p)[:3] + list(orient)

        def amovel(p, vel, acc):
            return lambda: dr.amovel(self.posx(pose(p)), vel=vel, acc=acc, ref=dr.DR_BASE)

        if holding:
            self.step("도구를 든 채 다음 틈으로 (집기 생략)", phase=2)
            here = self.current()
            self.step(f"이동 높이 z={geo['transit_z']:.0f} → 같은 높이로 이 틈의 몸체 쪽 위", phase=3)
            self.movel([here[0], here[1], geo["transit_z"]] + list(orient), VELX, ACCX, "이동 높이 맞추기")
            self.movel(pose(geo["approach_high"]), VELX, ACCX, "몸체 쪽 위")
        else:
            # 2. 도구 집기 (고정 동작)
            self.step("도구 접근 위치로 이동", phase=2)
            self.movel(sm.add(tool[:3], [0, 0, APPROACH_Z]) + tool[3:], VELX, ACCX, "도구 접근")
            self.step("그리퍼 열기")
            self.gripper("o")
            self.step(f"도구 위치로 하강 (도구 위치보다 {GRAB_DOWN_MM:.0f}mm 더 아래에서 잡기)")
            self.movel(sm.add(tool[:3], [0, 0, -GRAB_DOWN_MM]) + tool[3:], SLOW_VELX, SLOW_ACCX, "도구 잡는 위치")
            self.step("그리퍼 닫기")
            self.gripper("c")
            self.step(f"도구 들어올리기 (+{LIFT_Z:.0f}mm)")
            self.movel(sm.add(tool[:3], [0, 0, LIFT_Z]) + tool[3:], SLOW_VELX, SLOW_ACCX, "들어올리기")
            self.step(f"한 번 더 올리기 (+{LIFT_EXTRA_Z:.0f}mm)")
            self.movel(geo["high"], VELX, ACCX, "더 올리기")
            self.step(f"같은 높이로 +X {TOOL_OUT_X_MM:.0f}mm 이동 (도구 자세 그대로)")
            self.movel(geo["out"], VELX, ACCX, "+X 로 빼내기")
            self.q_out = self.current_q()          # 반납 때 이 관절로 되돌아옴
            # 3. 관절 이동으로 몸체 쪽 위에서 작업 자세(벽에 90도)로
            self.step(f"관절 이동: 몸체 쪽 위에서 작업 자세(벽에 90도, 그리퍼 {getattr(self, 'tool_spin', TOOL_SPIN_DEG):+.0f}도 회전)로",
                      phase=3)
            q_turn = self.turn_joints(pose(geo["approach_high"]), "작업 자세")
            self.movej(q_turn, REORIENT_VELJ, REORIENT_ACCJ, "작업 자세로")
            self.verify(pose(geo["approach_high"]), "작업 자세 (몸체 쪽 위)", tol_deg=WORK_TOL_DEG)
        self.step(f"몸체 쪽({geo['body_side']:.0f}mm)에서 수직으로 내려가기")
        self.movel(pose(geo["approach_body"]), VELX, ACCX, "몸체 쪽 하강")

        err = None
        try:
            # 4. 접촉 탐색: 하강한 곳에서 벽 쪽으로 수평으로 천천히 → 저항(외력 contact_force)을 받은 자리에서 멈춤
            #    (위치 목표로 가면 실제 벽/원판 두께를 몰라 부딪힘: run115 에서 x 790 위치 이동 중 96 N 충돌 감지)
            self.step(f"접촉 탐색: 벽 쪽으로 수평 {self.contact_speed:g}mm/s, 외력 {self.contact_force:g} N 이상이면 "
                      f"그 자리에서 멈춤 (최대 {self.contact_max:.0f}mm)", phase=4)
            time.sleep(0.3)
            baseline = self.force_avg()
            if not self.seek_contact(orient, tz, baseline, "접촉 탐색", target=geo["s"], direction=geo["body_flat"]):
                raise Abort(f"벽 쪽으로 {self.contact_max:.0f}mm 전진해도 저항이 없습니다 (틈 좌표 확인).")
            self.step("자세 맞추기: 원판 전체가 벽에 닿게 (모멘트로 덜 닿는 쪽 판단)")
            orient = self.align_to_wall(orient, baseline, tz)   # 이후 샌딩 동작은 맞춘 자세로
            contact = self.current()[:3]
            self.log.info(f"   벽 접촉 위치: {fmt(contact)} (틈 시작 대비 툴Z {sm.dot(sm.sub(contact, geo['s']), tz):+.1f}mm)")

            # 5. 순응 + 힘제어 → 틈 끝까지 좌우 왕복하며 연속 이동
            self.step(f"순응제어 + 힘제어 ON (벽 쪽 {self.press_force:.1f} N)", phase=5)
            self.force_on()
            shift = sm.sub(contact, geo["s"])
            s, e = contact, sm.add(geo["e"], shift)
            pts = self.zigzag(s, e, geo["u"], geo["side"], geo["half"], geo["length"])
            if geo["min_z"] is not None:
                pts = [[p[0], p[1], max(p[2], geo["min_z"])] for p in pts]
            g = dict(baseline=baseline, limit_force=FORCE_LIMIT_N, plane=(contact, tz))
            self.step(f"샌딩: 틈 끝까지 {len(pts)}구간 (좌우 ±{geo['half']:.1f}mm, {ZIGZAG_PITCH_MM:g}mm 씩 전진)")
            for i, p in enumerate(pts, 1):
                self.guarded(amovel(p, SCRUB_VELX, SCRUB_ACCX), target=pose(p), what=f"샌딩 {i}/{len(pts)}", **g)
        except BaseException as ex:
            err = ex
            raise
        finally:
            # 6. 힘제어 OFF → 후퇴 (성공 / 중단). 취소·이동 실패면 멈춘 자리 그대로 (HMI E-STOP 대비)
            self.phase = 6
            try:
                self.force_off()
            finally:
                retreat = err is None or isinstance(err, Abort) or \
                    (isinstance(err, Canceled) and self.retreat_on_cancel)
                if not retreat:
                    self.log.warn(f"{type(err).__name__} - 순응/힘제어만 해제하고 그 자리에서 멈춥니다 (후퇴 안 함). "
                                  "교시기로 확인하세요.")
                else:
                    # 성공이면 살짝만 떼고 7단계에서 부드럽게 빠짐 (중단이면 60mm 후퇴 후 멈춤)
                    soft = err is None
                    dist = self.exit_pull if soft else RETREAT_MM
                    self.step(f"순응/힘제어 해제, 벽에서 {dist:.0f}mm 후퇴")
                    try:
                        here = self.current()
                        self.movel(sm.sub(here[:3], sm.scale(tz, dist)) + here[3:], SLOW_VELX, SLOW_ACCX,
                                   "벽에서 후퇴")
                    except Exception as ex:
                        self.log.error(f"후퇴 실패: {ex} - 교시기로 확인하세요!")

        # 7. 멈추지 않고 한 번의 비스듬한 직선으로 하강했던 자리 위(이동 높이)까지 빠지며 상승
        #    (예전: 후퇴 60 + 몸체 쪽 80 → x 586~590 관절 한계 1206. 경로가 하강 위치보다 몸체 쪽으로 안 들어감)
        here = self.current()
        top = [geo["approach_body"][0], geo["approach_body"][1], geo["transit_z"]]
        self.step(f"하강했던 자리 위 {fmt(top)} 로 한 번에 빠지며 상승 ({self.exit_speed:g}mm/s)", phase=7)
        self.movel(top + here[3:], self.exit_speed, self.exit_acc, "부드럽게 빠지며 상승")
        if not return_tool:
            self.log.info("완료 (도구 반납 안 함 - 도구를 들고 있음)")
            return
        q_out = getattr(self, "q_out", None)
        if q_out is None:
            raise RuntimeError("도구 반납: 집을 때의 관절값이 없음 (서버 재시작?) - 손으로 반납하세요")
        self.step("관절 이동: 도구 자세로 되돌리기")
        self.movej(q_out, REORIENT_VELJ, REORIENT_ACCJ, "+X 로 빼낸 곳 (도구 자세)")
        self.verify(geo["out"], "+X 로 빼낸 곳 (도구 자세)")
        self.step(f"같은 높이로 -X {TOOL_OUT_X_MM:.0f}mm 이동 → 도구 위")
        self.movel(geo["high"], VELX, ACCX, "도구 위 높은 자세")
        self.step("도구 반납: 하강")
        self.movel(sm.add(tool[:3], [0, 0, LIFT_Z]) + tool[3:], VELX, ACCX, "도구 위 10cm")
        self.movel(tool, SLOW_VELX, SLOW_ACCX, "도구 반납 위치")
        self.step("도구 반납: 그리퍼 열기")
        self.gripper("o")
        self.step(f"위로 빠지기 (+{APPROACH_Z:.0f}mm)")
        self.movel(sm.add(tool[:3], [0, 0, APPROACH_Z]) + tool[3:], SLOW_VELX, SLOW_ACCX, "위로 빠지기")
        self.q_out = None
        self.log.info("완료")
