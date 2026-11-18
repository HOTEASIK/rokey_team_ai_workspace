"""도구 집기 → 벽 앞으로 → 순응/힘제어 샌딩 → 도구 반납 (MoveIt 미사용, 두산 컨트롤러만)

sanding_server.py 가 이 SandingJob 으로 goal 을 실행합니다. (이 파일은 단독 실행 안 함)
  HMI(hmi_server) / sanding_client → /dsr01/tool_task 액션 → sanding_server → SandingJob
  job = SandingJob(dsr_node, logger, is_canceled=..., on_step=...)
  job.run(tool_posx, start_posx, end_posx, surface_normal, return_tool)
  job.run(tool_posx, None, None, surface_normal, return_tool, start_posj=..., work_vector=...)  # 관절각으로 시작

동작:
  1. 계획 (로봇 안 움직임): 시작 posx 의 툴 Z 로 벽 방향 판단 → 표면에 정확히 90도로 자세 보정
     (surface_normal 을 주면 그 방향, 안 주면 툴 Z 에 가장 가까운 베이스 축 ±X/±Y/-Z 로 맞춤)
     경로 전체 관절 검사(ikin/fkin, 같은 sol) + 이동 중 처짐 검사(관절 이동 중 TCP 높이)
  2. (현재 팔 형태로 도구에 못 가면 홈 자세로 먼저 이동) → 도구 접근 → 그리퍼 열기 → 하강 → 닫기 → 10cm 들어올리기 → 10cm 더 올리기 (바닥 끌림 방지)
  3. 그 높이에서 관절로 작업 자세로 꺾기 (도구 위, 팔이 안 닿으면 벽 쪽으로 조금씩 옮긴 곳)
     → 같은 높이로 수평 이동 → 벽 앞에서 Z 로만 내려가 접근 위치
  4. 벽 근처까지 접근 → 툴 Z(벽 쪽)로 천천히 이동하며 힘 감시 → 접촉 시 정지
  5. 순응제어 + 힘제어 ON → 시작 위치 왕복 → 중간 지점 원 샌딩 → 종료 위치 왕복
     (왕복은 항상 벽면 위의 수평 방향(좌우): 진행이 Y 면 Y 로, 진행이 Z(위아래)여도 Y 로)
  6. 힘제어 OFF → 후퇴 → 수직으로 올라감 → 같은 높이로 수평 이동 → 관절로 자세 되돌리기 → 도구 반납 → 10cm 위로

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
from sanding.reach_check import ReachChecker, ReachError, fmt, sample_joint, sample_line, vec6

from utils.robot import ROBOT_ID, ROBOT_MODEL, TOOL_NAME  # config/robot.yaml robot 섹션
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ── 도구 들기 ─────────────────────────────────────────────────────
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]   # 현재 팔 형태로 도구에 못 가면 여기로 먼저 이동
TCP_NAMES = ("GripperDA_v1", "GripperDA v1")   # 티칭 펜던트에 등록된 TCP 이름 후보 (도구/작업 위치는 이 TCP 기준으로 교시)
APPROACH_Z = 100.0             # 도구 위 접근 높이
LIFT_Z = 100.0                 # 도구 들어올리기 (천천히)
LIFT_EXTRA_Z = 100.0           # 한 번 더 올리기 (바닥 끌림 방지)
TURN_FRACTIONS = (0.0, 0.25, 0.5, 0.75, 1.0)   # 자세 꺾을 곳 후보: 도구 위(0) → 벽 앞 위쪽(1) 사이, 가까운 순
TRANSIT_SAG_MM = 30.0          # (홈 이동) 관절 이동 중 TCP 가 출발/도착 높이보다 이 이상 내려가면 불가
TRANSIT_CLEAR_MM = 100.0       # (도구 들고 이동) TCP 가 도구 잡은 높이 + 이 값보다 내려가면 불가 (바닥 끌림 방지)

# ── 벽 접근 / 접촉 탐색 (툴 Z = 벽 쪽 방향 거리) ────────────────────
SURFACE_APPROACH_MM = 60.0
PRE_CONTACT_MM = 20.0
SEARCH_BEYOND_MM = 30.0
SEARCH_VELX, SEARCH_ACCX = 15, 30
CONTACT_DETECT_N = 5.0
RETREAT_MM = 60.0

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
SCRUB_PASSES = 5
SCRUB_VELX, SCRUB_ACCX = 60, 120
ORBIT_RADIUS_MM = 10.0
ORBIT_TURNS = 3
ORBIT_POINTS_PER_TURN = 12
ORBIT_STEP_MM = 30.0           # 원 샌딩 간격 최대값 (시작→종료를 이 값 이하로 균등하게 나눔)
ORBIT_VELX, ORBIT_ACCX = 50, 100

# ── 벽 방향 자동 판단 (surface_normal 을 안 줬을 때) ──────────────────
SNAP_AXIS_DEG = 20.0           # 시작 자세 툴 Z 가 베이스 축과 이 각도 안이면 그 축에 정확히 90도로 맞춤

# ── 도구 크기 / 바닥 여유 (원형 도구를 중앙에서 잡음) ──────────────────
TOOL_RADIUS_MM = 45.0          # 원형 도구 반지름 (TCP = 도구 중앙 → 벽 작업 때 가장자리가 TCP 보다 이만큼 아래)
FLOOR_CLEAR_MM = 1.0           # 도구 가장자리와 바닥 사이 여유 → 벽 맨 아래까지 갈리도록 작게 (0 이면 바닥에 닿음)
                               # (너무 낮은 시작/종료점·원 중심만 그 높이로 올림. 벽 윗면은 검사 안 함)

# ── 속도 / 감시 ───────────────────────────────────────────────────
REORIENT_VELJ, REORIENT_ACCJ = 20, 20   # 자세가 크게 바뀌는 관절 이동
VELX, ACCX = 150, 150
SLOW_VELX, SLOW_ACCX = 60, 60
GRIPPER_WAIT = 1.5
POLL_DT = 0.02
MOTION_START_TIMEOUT = 0.5
MOTION_TIMEOUT = 60.0
ARRIVE_TOL_MM, ARRIVE_TOL_DEG = 3.0, 2.0
SANDING_TOL_MM = 8.0           # 샌딩 중(힘제어) 도착 허용 오차 - 사포 마찰로 경로보다 몇 mm 뒤처지는 건 정상
WORK_TOL_DEG = 0.5
PATH_STEP_MM = 20.0

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
                 go_home=False, floor_z=None, floor_clear=FLOOR_CLEAR_MM, tool_radius=TOOL_RADIUS_MM):
        self.node, self.log = dsr_node, logger
        self.is_canceled, self.on_step = is_canceled, on_step
        self.retreat_on_cancel = retreat_on_cancel
        self.go_home = go_home   # True 면 항상 홈 자세부터 (False 면 필요할 때만)
        self.floor_z = floor_z   # 작업하는 벽 아래 바닥 높이 (베이스 z, mm). None 이면 바닥 여유 검사 안 함
        self.floor_clear = floor_clear   # 도구 아래끝과 바닥 사이 여유 (mm)
        self.tool_radius = tool_radius   # 원형 도구 반지름 (mm)
        self.press_force = PRESS_FORCE_N   # 샌딩 중 벽 쪽 누르는 힘 (N) - 작업마다 서버가 HMI 값으로 바꿈
        import DSR_ROBOT2 as dr
        from DR_common2 import posj, posx
        self.dr, self.posx, self.posj = dr, posx, posj
        self.gripper_cli = dsr_node.create_client(SetCommand, "/onrobot/sendCommand")
        self.stop_cli = dsr_node.create_client(MoveStop, "dsr_controller2/motion/move_stop")
        self.reach = ReachChecker(dr, posx, logger)
        self.force_active = False
        self.phase = 0

    def wait_services(self, timeout=30.0):
        """Wi-Fi + discovery server(다른 노트북의 sodreal) 로 찾으면 몇 초~십수 초 걸려서 넉넉히 기다림"""
        for cli, name in ((self.gripper_cli, "/onrobot/sendCommand"), (self.stop_cli, "move_stop")):
            waited = 0.0
            while not cli.wait_for_service(timeout_sec=5.0):
                waited += 5.0
                if waited >= timeout:
                    raise RuntimeError(f"{name} 서비스가 없습니다 ({timeout:.0f}초 대기). sodreal 실행 / "
                                       f"ROS_DOMAIN_ID·ROS_DISCOVERY_SERVER 가 sodreal 쪽과 같은지 확인")
                self.log.info(f"{name} 서비스 찾는 중... ({waited:.0f}초)")

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

    def align_to_wall(self, orient, baseline, tz, search_end):
        """힘제어 켜기 전, 벽에 닿은 채 멈춘 상태에서 모멘트로 압력 중심을 재고 원판 전체가 닿게 자세를 맞춤.
        (힘제어 중에는 툴 힘 값이 거의 0 으로 나와 못 씀 → 위치 제어로 닿은 상태에서 잼)
        한 번: 압력 중심 측정 → 벽에서 ALIGN_BACKOFF_MM 뒤로 → 기울이기 → 다시 접촉 탐색.
        압력 중심 r=(x, y) 쪽이 더 세게 닿음 → 툴축 (-y, x) 둘레로 돌리면 r 쪽이 벽에서 멀어지고 반대쪽이 다가감.
        센서 부호가 반대일 수 있으므로 치우침이 오히려 커지면 방향을 뒤집음. 맞춘 자세를 돌려줌."""
        dr = self.dr
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
            hit = self.guarded(lambda o=list(new): dr.amovel(self.posx(list(search_end) + o), vel=SEARCH_VELX,
                                                            acc=SEARCH_ACCX, ref=dr.DR_BASE),
                               baseline, stop_force=CONTACT_DETECT_N, what=f"자세 맞추기 {it}: 다시 접촉")
            if not hit:
                raise Abort("자세 맞추기 중 다시 벽에 닿지 않았습니다.")
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

    # ── 1. 계획 ──────────────────────────────────────────────────
    def check_transit(self, qa, qb, what, floor_z=None):
        """관절 이동(qa→qb) 중 TCP 최저 높이 검사 (fkin).
        floor_z 를 주면 그 높이 아래로 내려가면 불가, 안 주면 출발/도착보다 TRANSIT_SAG_MM 이상 처지면 불가"""
        zs = [self.fk(q)[2] for q in sample_joint(qa, qb, 5.0)]
        if floor_z is None:
            floor_z = min(zs[0], zs[-1]) - TRANSIT_SAG_MM
        if min(zs) < floor_z:
            raise PlanError(f"{what}: 관절 이동 중 TCP 가 z={min(zs):.0f}mm 까지 내려감 (한계 z={floor_z:.0f}, "
                            f"출발 {zs[0]:.0f} / 도착 {zs[-1]:.0f}) - 도구가 바닥에 닿을 수 있음")
        self.log.info(f"   {what}: 관절 이동 중 TCP 최저 z={min(zs):.0f}mm (한계 {floor_z:.0f}) OK")

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

    def floor_min_z(self, tz, drop):
        """도구(중앙을 잡은 원판) 가장자리가 바닥 + FLOOR_CLEAR_MM 에 오도록 하는 TCP 최소 z.
        drop = 그 위치에서 하는 동작(왕복/원)이 TCP 를 아래로 내리는 양. 바닥 높이 미설정이면 None"""
        if self.floor_z is None:
            return None
        edge_drop = self.tool_radius * math.sqrt(max(0.0, 1.0 - tz[2] ** 2))   # 벽 작업 = 반지름, 바닥 작업 0
        return self.floor_z + self.floor_clear + edge_drop + drop

    def keep_floor_clear(self, pts, min_z, names):
        """pts 중 min_z 보다 낮은 점만 min_z 로 올림 (충분히 높은 점은 주어진 그대로)"""
        out = []
        for name, p in zip(names, pts):
            if min_z is not None and p[2] < min_z - 1e-6:
                self.log.warn(f"{name} z={p[2]:.1f}mm 는 도구(반지름 {self.tool_radius:.1f}mm)가 바닥(z={self.floor_z:.1f})에 "
                              f"닿음 → z={min_z:.1f}mm 로 맞춤 (도구 아래끝 = 바닥 + {self.floor_clear:.1f}mm)")
                p = [p[0], p[1], min_z]
            out.append(list(p))
        return out

    def plan(self, tool, start_posx, end_posx, normal, start_posj=None, work_vector=None):
        dr = self.dr
        # 시작 TCP + 표면에 90도 자세
        #   start_posj 를 주면: 컨트롤러 fkin 으로 정확한 시작 TCP, 종료 = 시작 + work_vector (real_test 방식)
        if start_posj is not None:
            fk = self.fk(start_posj)
            self.log.info(f"시작 관절 {fmt(start_posj, 2)} → TCP {fmt(fk)} (fkin)")
            start_posx = fk
            end_posx = sm.add(fk[:3], list(work_vector)) + fk[3:]
        start_posx, end_posx = list(start_posx), list(end_posx)
        taught_z = sm.tool_axes(start_posx[3:])[2]
        self.log.info(f"시작 posx {fmt(start_posx, 2)} (툴 Z {fmt(taught_z, 3)})")
        if normal is None or sm.norm(normal) < 1e-6:
            normal, ang = self.auto_normal(taught_z)
            if normal is None:
                self.log.warn(f"툴 Z 가 베이스 축과 {ang:.1f}도 차이 (> {SNAP_AXIS_DEG}도) - 자세 보정 없이 그대로 사용")
            else:
                self.log.info(f"벽 방향 자동 판단: 법선 {fmt(normal, 0)} (툴 Z 와 {ang:.1f}도 차이)")
        if normal is not None:
            orient = sm.orient_toward_surface(start_posx[3:], normal)
            corr = math.degrees(math.acos(max(-1.0, min(1.0, sm.dot(taught_z, sm.tool_axes(orient)[2])))))
            self.log.info(f"표면에 90도로 자세 보정: {corr:.1f}도 → {fmt(orient, 2)}")
        else:
            orient = start_posx[3:]
        tx, _, tz = sm.tool_axes(orient)

        # 작업 경로 (종료 위치는 시작 위치의 표면 평면에 투영)
        s = start_posx[:3]
        e = sm.project_to_plane(end_posx[:3], s, tz)
        u, v, length = sm.path_frame(s, e, tz, tx)
        half = SCRUB_STROKE_MM / 2.0
        sd = self.scrub_dir(u, v)   # 시작/종료 왕복 방향 = 벽면 위 수평 방향
        # 바닥 여유 (원판 도구 가장자리가 바닥 + FLOOR_CLEAR_MM 까지 내려가도록, 동작별로 따로)
        #   시작/종료: 왕복이 아래로 내려가는 만큼만 / 중간 원: 원이 아래로 내려가는 만큼만
        if self.floor_z is None:
            self.log.warn("바닥 높이(floor_z) 미설정 - 도구 바닥 여유 검사를 건너뜁니다.")
        end_min = self.floor_min_z(tz, half * abs(sd[2]))
        s, e = self.keep_floor_clear([s, e], end_min, ["시작", "종료"])
        u, v, length = sm.path_frame(s, e, tz, tx)   # 점을 올렸으면 진행 방향/길이 다시 계산
        sd = self.scrub_dir(u, v)
        end_min = self.floor_min_z(tz, half * abs(sd[2]))
        orbit_min = self.floor_min_z(tz, ORBIT_RADIUS_MM * math.hypot(u[2], v[2]))
        centers = sm.orbit_centers(s, e, u, length, ORBIT_STEP_MM)
        if len(centers) > 2:
            centers = [centers[0]] + self.keep_floor_clear(
                centers[1:-1], orbit_min, [f"원 {k} 중심" for k in range(1, len(centers) - 1)]) + [centers[-1]]
        if end_min is not None:
            self.log.info(f"바닥 여유: TCP 최소 z 시작/종료 {end_min:.1f} · 원 {orbit_min:.1f} "
                          f"(바닥 {self.floor_z:.1f} + 여유 {self.floor_clear:.1f} + 도구 반지름 {self.tool_radius:.1f} "
                          f"→ 도구 아래끝이 바닥 위 {self.floor_clear:.1f}mm 까지)")
        sn = "위아래" if abs(sd[2]) > 0.7 else "좌우"
        self.log.info(f"작업: 시작 {fmt(s)} → 종료 {fmt(e)} ({length:.0f}mm, 진행 {fmt(u, 2)}), "
                      f"시작/종료 {sn} 왕복 ±{half:.1f}mm")

        # 이동 경로: 도구 들고 수직 상승(z=transit_z) → 그 높이에서 관절로 작업 자세 꺾기
        #   → 같은 높이로 수평 이동(벽 쪽) → 벽 앞에서 Z 로만 수직 하강 → 벽 쪽 접근.  돌아올 때는 반대 순서
        #   자세 꺾는 곳 = 도구 위 ~ 벽 앞 위쪽 사이에서 팔이 닿는 가장 도구 쪽 (TURN_FRACTIONS)
        high = sm.add(tool[:3], [0, 0, LIFT_Z + LIFT_EXTRA_Z]) + tool[3:]
        approach = sm.sub(s, sm.scale(tz, SURFACE_APPROACH_MM))
        transit_z = max(high[2], approach[2])
        high = [tool[0], tool[1], transit_z] + tool[3:]
        approach_high = [approach[0], approach[1], transit_z]
        retreat = sm.sub(e, sm.scale(tz, RETREAT_MM))
        retreat_high = [retreat[0], retreat[1], transit_z]
        geo = dict(orient=orient, tz=tz, u=u, v=v, s=s, e=e, centers=centers, half=half, sn=sn, sd=sd,
                   high=high, approach_high=approach_high, approach=approach,
                   pre=sm.sub(s, sm.scale(tz, PRE_CONTACT_MM)),
                   search_end=sm.add(s, sm.scale(tz, SEARCH_BEYOND_MM)), retreat_high=retreat_high)

        # 관절 검사 ① 도구 구간 (movel 은 팔 형태를 못 바꿈 → 현재 팔 형태, 안 되면 홈 자세의 팔 형태)
        _, cur_sol = dr.get_current_posx(ref=dr.DR_BASE)
        cur_sol = int(cur_sol)
        home_sol = int(dr.get_solution_space(self.posj(list(HOME_POSJ))))
        tool_pts = ([sm.add(tool[:3], [0, 0, APPROACH_Z]) + tool[3:]]
                    + sample_line(sm.add(tool[:3], [0, 0, APPROACH_Z]) + tool[3:], tool, PATH_STEP_MM)[1:]
                    + sample_line(tool, high, PATH_STEP_MM)[1:])
        q_tool, why = None, []
        if not self.go_home:
            try:
                _, q_tool = self.reach.solve_path(tool_pts, sols=[cur_sol], what="도구 집기 구간")
            except ReachError as ex:
                why.append(f"현재 팔 형태(sol {cur_sol}): {ex}")
        geo["go_home"] = q_tool is None
        tool_sol = home_sol if q_tool is None else cur_sol
        if q_tool is None:
            try:
                _, q_tool = self.reach.solve_path(tool_pts, sols=[home_sol], what="도구 집기 구간")
            except ReachError as ex:
                why.append(f"홈 자세 팔 형태(sol {home_sol}): {ex}")
                raise PlanError("도구에 갈 수 없음\n  " + "\n  ".join(why) + "\n  → TOOL_POSX(도구 위치/자세) 확인")
            if why:
                self.log.warn(f"{why[0]}\n  → 홈 자세(sol {home_sol})로 먼저 이동한 뒤 도구로 갑니다.")
            self.check_transit(self.current_q(), HOME_POSJ, "현재 자세 → 홈 자세")
        q_high = q_tool[-1]

        # 관절 검사 ② 작업 구간 (같은 sol): 수평 이동 → 수직 하강 → 접근/탐색 → 샌딩 → 후퇴 → 수직 상승 → 수평 복귀
        P = lambda p: list(p) + list(orient)   # noqa: E731
        work = sample_line(P(approach_high), P(approach), PATH_STEP_MM)                    # 수직 하강
        work += sample_line(work[-1], P(geo["search_end"]), PATH_STEP_MM)[1:]
        for i, c in enumerate(centers):
            work += sample_line(work[-1], P(c), PATH_STEP_MM)[1:]
            if i in (0, len(centers) - 1):
                for d in (half, -half, 0.0):
                    work += sample_line(work[-1], P(sm.add(c, sm.scale(sd, d))), PATH_STEP_MM)[1:]
            else:
                for k in range(8):
                    a = 2 * math.pi * k / 8
                    work.append(P(sm.add(c, sm.add(sm.scale(u, ORBIT_RADIUS_MM * math.cos(a)),
                                                   sm.scale(v, ORBIT_RADIUS_MM * math.sin(a))))))
                work.append(P(c))
        work += sample_line(work[-1], P(retreat), PATH_STEP_MM)[1:]
        work += sample_line(work[-1], P(retreat_high), PATH_STEP_MM)[1:]                   # 수직 상승
        # 팔 형태(sol): 시작 관절각을 주면 그 자세와 가장 가까운 것 (교시한 팔 형태),
        #               아니면 도구를 들어 올린 자세(q_high)에서 관절이 가장 적게 바뀌는 것
        q_ref = list(start_posj) if start_posj is not None else q_high
        floor_z = tool[2] + TRANSIT_CLEAR_MM   # 관절 이동 중 TCP 최저 허용 = 도구 잡은 높이 + 10cm
        fails = []
        for f in TURN_FRACTIONS:
            turn = sm.add(high[:3], sm.scale(sm.sub(approach_high, high[:3]), f))
            pts = sample_line(P(turn), P(approach_high), PATH_STEP_MM) if f < 1.0 else [P(approach_high)]
            pts = pts + work[1:]
            if f < 1.0:
                pts += sample_line(pts[-1], P(turn), PATH_STEP_MM)[1:]                     # 수평 복귀
            try:
                sol, qs = self.reach.solve_path(pts, q_ref=q_ref, sols=range(8), what="작업 경로")
                self.check_transit(q_high, qs[0], "자세 꺾기", floor_z)
                self.check_transit(qs[-1], q_high, "자세 되돌리기", floor_z)
            except (ReachError, PlanError) as ex:
                fails.append(f"꺾는 곳 {fmt(turn)} ({f * 100:.0f}%): {str(ex).splitlines()[0]}")
                continue
            break
        else:
            raise PlanError("작업 경로 불가 - 자세 꺾을 곳을 못 찾음\n  " + "\n  ".join(fails)
                            + "\n  → 관절 한계/특이점 때문에 불가")
        for msg in fails:
            self.log.info(f"   (건너뜀) {msg}")
        self.log.info(f"자세 꺾는 곳: {fmt(turn)} (도구 위 → 벽 앞 위쪽 {f * 100:.0f}% 지점), "
                      f"수평 이동 {sm.norm(sm.sub(approach_high, turn)):.0f}mm 후 수직 하강")
        geo.update(q_high=q_high, q_turn=qs[0], turn=turn)
        self.log.info(f"계획 통과: 도구 sol {tool_sol}{' (홈 먼저)' if geo['go_home'] else ''}, 작업 sol {sol}, "
                      f"점 {len(tool_pts) + len(pts)}개, 이동 높이 z={transit_z:.0f}mm")
        return geo

    # ── 전체 실행 ────────────────────────────────────────────────
    def run(self, tool, start_posx, end_posx, normal=None, return_tool=True, start_posj=None, work_vector=None):
        """start_posx/end_posx 로 작업. start_posj + work_vector 를 주면 그걸로 (start/end_posx 무시)"""
        dr = self.dr
        self.step("계획: 벽 방향 + 경로 관절 검사 + 처짐 검사 (로봇 안 움직임)", phase=1)
        # 계획(ikin/fkin)은 컨트롤러의 '현재 TCP' 기준으로 계산됨 → TCP 를 먼저 맞춘 뒤 계획해야 함
        # (TCP 없이 계획하고 나중에 TCP 를 켜면 관절 목표가 TCP 길이(204mm)만큼 어긋남)
        self.reset_control_mode()   # 남아 있는 순응/힘제어 해제 (안 하면 첫 이동에서 팔이 처질 수 있음)
        self.ensure_tcp()
        geo = self.plan(tool, start_posx, end_posx, normal, start_posj, work_vector)
        orient, tz, u, v, half, sn = geo["orient"], geo["tz"], geo["u"], geo["v"], geo["half"], geo["sn"]
        sd = geo["sd"]

        def pose(p):
            return list(p) + list(orient)

        def amovel(p, vel, acc):
            return lambda: dr.amovel(self.posx(pose(p)), vel=vel, acc=acc, ref=dr.DR_BASE)

        def amovesx(pts, vel, acc):
            return lambda: dr.amovesx([self.posx(pose(p)) for p in pts], vel=vel, acc=acc, ref=dr.DR_BASE)

        # 2. (필요하면 홈 자세) → 도구 집기 → 20cm 들기
        if geo["go_home"]:
            self.step(f"홈 자세로 이동 (팔 형태 맞추기, {REORIENT_VELJ} deg/s)", phase=2)
            self.movej(HOME_POSJ, REORIENT_VELJ, REORIENT_ACCJ, "홈 자세")
        self.step("도구 접근 위치로 이동", phase=2)
        self.movel(sm.add(tool[:3], [0, 0, APPROACH_Z]) + tool[3:], VELX, ACCX, "도구 접근")
        self.step("그리퍼 열기")
        self.gripper("o")
        self.step("도구 위치로 하강")
        self.movel(tool, SLOW_VELX, SLOW_ACCX, "도구 위치")
        self.step("그리퍼 닫기")
        self.gripper("c")
        self.step(f"도구 들어올리기 (+{LIFT_Z:.0f}mm)")
        self.movel(sm.add(tool[:3], [0, 0, LIFT_Z]) + tool[3:], SLOW_VELX, SLOW_ACCX, "들어올리기")
        self.step(f"한 번 더 올리기 (+{LIFT_EXTRA_Z:.0f}mm, 바닥 끌림 방지)")
        self.movel(geo["high"], VELX, ACCX, "더 올리기")

        # 3. 높은 곳에서 관절로 자세 꺾기 → 수평 이동 → 수직 하강 → 벽 근처
        self.step("높은 곳에서 관절로 작업 자세(벽에 90도)로 꺾기", phase=3)
        self.movej(geo["q_turn"], REORIENT_VELJ, REORIENT_ACCJ, "자세 꺾기")
        self.verify(pose(geo["turn"]), "자세 꺾기 (90도 자세)", tol_deg=WORK_TOL_DEG)
        self.step(f"같은 높이(z={geo['turn'][2]:.0f}mm)로 수평 이동 → 벽 앞 위쪽")
        self.movel(pose(geo["approach_high"]), VELX, ACCX, "벽 앞 위쪽")
        self.step("수직으로 내려가 벽 앞 접근 위치")
        self.movel(pose(geo["approach"]), VELX, ACCX, "벽 앞 접근 위치")
        self.step(f"벽 {PRE_CONTACT_MM:.0f}mm 앞까지 접근")
        self.movel(pose(geo["pre"]), SLOW_VELX, SLOW_ACCX, "벽 근처")

        err = None
        try:
            # 4. 접촉 탐색
            self.step(f"접촉 탐색: 툴 Z(벽 쪽)로 {SEARCH_VELX}mm/s", phase=4)
            time.sleep(0.3)
            baseline = self.force_avg()
            hit = self.guarded(amovel(geo["search_end"], SEARCH_VELX, SEARCH_ACCX), baseline,
                               stop_force=CONTACT_DETECT_N, what="접촉 탐색")
            if not hit:
                raise Abort(f"시작 위치보다 {SEARCH_BEYOND_MM:.0f}mm 더 들어가도 벽에 닿지 않았습니다.")
            self.step("자세 맞추기: 원판 전체가 벽에 닿게 (모멘트로 덜 닿는 쪽 판단)")
            orient = self.align_to_wall(orient, baseline, tz, geo["search_end"])   # 이후 샌딩 동작은 맞춘 자세로
            contact = self.current()[:3]
            self.log.info(f"   벽 접촉 위치: {fmt(contact)} (시작 위치 대비 툴Z {sm.dot(sm.sub(contact, geo['s']), tz):+.1f}mm)")

            # 5. 순응 + 힘제어 샌딩
            self.step(f"순응제어 + 힘제어 ON (벽 쪽 {self.press_force:.1f} N)", phase=5)
            self.force_on()
            shift = sm.sub(contact, geo["s"])
            centers = [sm.add(c, shift) for c in geo["centers"]]
            g = dict(baseline=baseline, limit_force=FORCE_LIMIT_N, plane=(contact, tz))

            def line(p, vel, acc, what):
                return self.guarded(amovel(p, vel, acc), target=pose(p), what=what, **g)

            def scrub(p, name):
                for i in range(1, SCRUB_PASSES + 1):
                    self.step(f"{name} {sn} 샌딩 {i}/{SCRUB_PASSES} (±{half:.1f}mm)")
                    line(sm.add(p, sm.scale(sd, half)), SCRUB_VELX, SCRUB_ACCX, f"{name} {sn}(+)")
                    line(sm.sub(p, sm.scale(sd, half)), SCRUB_VELX, SCRUB_ACCX, f"{name} {sn}(-)")
                line(p, SCRUB_VELX, SCRUB_ACCX, f"{name} 가운데")

            scrub(centers[0], "시작 위치")
            middle = centers[1:-1]
            for i, c in enumerate(middle, start=1):
                self.step(f"원 샌딩 {i}/{len(middle)} ({sm.dot(sm.sub(c, centers[0]), u):.0f}mm 지점, "
                          f"{ORBIT_TURNS}바퀴)")
                line(c, ORBIT_VELX, ORBIT_ACCX, f"원 {i} 중심")
                line(sm.add(c, sm.scale(u, ORBIT_RADIUS_MM)), ORBIT_VELX, ORBIT_ACCX, f"원 {i} 시작점")
                pts = sm.orbit_points(c, u, v, ORBIT_RADIUS_MM, ORBIT_TURNS, ORBIT_POINTS_PER_TURN)
                self.guarded(amovesx(pts, ORBIT_VELX, ORBIT_ACCX), target=pose(pts[-1]), what=f"원 {i} 샌딩", **g)
            self.step("종료 위치로 이동")
            line(centers[-1], ORBIT_VELX, ORBIT_ACCX, "종료 위치")
            scrub(centers[-1], "종료 위치")

        except BaseException as ex:
            err = ex
            raise
        finally:
            # 6. 힘제어 OFF → 후퇴 (성공 / 중단). 취소·이동 실패면 멈춘 자리 그대로 (HMI E-STOP 대비)
            self.phase = 6
            try:
                self.force_off()
            finally:
                # 후퇴: 정상 완료 / 안전 감시 중단(Abort) 때만. 취소·이동 실패(HMI E-STOP 의 move_stop 등)는 그 자리
                retreat = err is None or isinstance(err, Abort) or \
                    (isinstance(err, Canceled) and self.retreat_on_cancel)
                if not retreat:
                    self.log.warn(f"{type(err).__name__} - 순응/힘제어만 해제하고 그 자리에서 멈춥니다 (후퇴 안 함). "
                                  "교시기로 확인하세요.")
                else:
                    self.step(f"순응/힘제어 해제, 벽에서 {RETREAT_MM:.0f}mm 후퇴")
                    try:
                        here = self.current()
                        self.movel(sm.sub(here[:3], sm.scale(tz, RETREAT_MM)) + here[3:], SLOW_VELX, SLOW_ACCX,
                                   "벽에서 후퇴")
                    except Exception as ex:
                        self.log.error(f"후퇴 실패: {ex} - 교시기로 확인하세요!")

        # 7. 수직 상승 → 도구 위 높은 자세 → 반납 → 10cm 위
        self.step("수직으로 올라가기", phase=7)
        here = self.current()
        self.movel([here[0], here[1], geo["retreat_high"][2]] + here[3:], VELX, ACCX, "수직 상승")
        if not return_tool:
            self.log.info("완료 (도구 반납 안 함 - 도구를 들고 있음)")
            return
        self.step("같은 높이로 수평 이동 → 자세 꺾은 곳")
        self.movel(pose(geo["turn"]), VELX, ACCX, "자세 꺾은 곳")
        self.step("관절로 도구 자세로 되돌리기")
        self.movej(geo["q_high"], REORIENT_VELJ, REORIENT_ACCJ, "도구 위 높은 자세")
        self.verify(geo["high"], "도구 위 높은 자세")
        self.step("도구 반납: 하강")
        self.movel(sm.add(tool[:3], [0, 0, LIFT_Z]) + tool[3:], VELX, ACCX, "도구 위 10cm")
        self.movel(tool, SLOW_VELX, SLOW_ACCX, "도구 반납 위치")
        self.step("도구 반납: 그리퍼 열기")
        self.gripper("o")
        self.step(f"위로 빠지기 (+{APPROACH_Z:.0f}mm)")
        self.movel(sm.add(tool[:3], [0, 0, APPROACH_Z]) + tool[3:], SLOW_VELX, SLOW_ACCX, "위로 빠지기")
        self.log.info("완료")
