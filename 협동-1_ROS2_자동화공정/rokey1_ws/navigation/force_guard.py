#!/usr/bin/env python3
"""암(로봇 팔) / 툴(그리퍼)에 '예상하지 못한 힘'이 걸렸을 때의 감지 + 예외 처리 모듈.

wall_size.py 처럼 벽을 일부러 누르는 탐색에서는 '정상적인 접촉 힘'과 '사고(충돌)'를
구분해야 한다. 이 모듈은 매 샘플마다 힘/토크/로봇 상태를 보고, 사고면 즉시 멈추고
종류별 예외를 던진다. 어떻게 회복할지는 호출한 쪽(wall_size)이 예외 종류로 결정한다.

[감시 대상]
  ① 툴 힘/토크   get_tool_force()      : 그리퍼 손끝/몸체에 걸린 힘 [N], 모멘트 [Nm]
  ② 암 외력 토크 get_external_torque() : 각 관절이 느끼는 외부 토크 [Nm] → 팔 몸통 충돌
  ③ 로봇 상태    get_robot_state()     : 컨트롤러 자체 충돌감지/안전정지/비상정지

[이벤트 분류]                                                  [예외]           [처리]
  TOOL_OVERLOAD : 툴 힘 크기 > TOOL_HARD_LIMIT                 ToolCollision    정지 → 후퇴 → 힘이 빠졌는지 확인
                                                                                 → 빠지면 그 선만 건너뛰고 계속
                                                                                 → 안 빠지면(계속 눌림) 전체 중단
  TOOL_SIDE     : '예상 방향' 이 아닌 방향 힘 > SIDE_LIMIT        ToolCollision    (연속 MAX_TOOL_COLLISIONS 번이면 전체 중단)
                  (예: 벽을 누르는데 옆/위에서 뭔가에 부딪힘)
  TOOL_TORQUE   : 툴 모멘트 > TORQUE_LIMIT                      ToolCollision
                  (손끝이 아니라 그리퍼 몸체/옆면이 걸림 → 지렛대로 큰 모멘트)
  ARM_CONTACT   : 관절 외력 토크 > (기준 + 툴 힘으로 설명되는 몫)  ArmCollision     정지 + 컴플라이언스 해제, 자동 이동 금지
                  (팔 몸통이 사람/물체에 닿음)                                     → 전체 중단, 작업자 확인
  CONTROLLER    : 로봇 상태가 안전정지/비상정지/서보 OFF           ControllerStop   이동 불가 → 알람 기록 후 전체 중단
  SENSOR_FAIL   : 센서 읽기가 연속 실패                          SensorFault      볼 수 없으면 위험 → 정지 후 전체 중단

  - '예상 방향': 호출하는 쪽이 지금 의도한 접촉 방향을 알려준다.
      벽 찾기 → (벽 방향)   벽 따라가기 → (벽 방향, 진행 방향)   일반 이동 → () = 어떤 접촉도 예상 안 함
    예상 방향 성분은 TOOL_HARD_LIMIT 까지 허용, 나머지(수직 성분)는 SIDE_LIMIT 까지만 허용.
    일반 이동(예상 방향 없음)은 FREE_MOVE_LIMIT 만 넘어도 충돌로 본다.
  - WARN_RATIO 이상이면 경고 로그만 (초당 1회).
  - 마지막 방어선은 두산 컨트롤러의 충돌 감지 (CONTROLLER 이벤트로 잡힘).

[가상 모드 테스트]  에뮬레이터는 외력이 없으므로 일부러 사고를 주입해 처리 과정을 볼 수 있다.
  ros2 run rokey1 wall_size --ros-args -p mode:=virtual -p inject:=tool_side -p inject_after:=20.0
    inject = none | tool_side | tool_overload | tool_torque | arm | estop | sensor
    inject_after 초 뒤에 inject_duration 초 동안 해당 외력/상태를 측정값에 더한다.
"""
import time
from dataclasses import dataclass, field, asdict

import numpy as np

# =============================================================================
# 임계값  (처음엔 보수적으로. 실제 로봇에서 정상 탐색 로그를 보고 조정)
# =============================================================================
TOOL_HARD_LIMIT = 40.0      # [N]  툴 힘 크기 상한 (어느 방향이든)
FREE_MOVE_LIMIT = 15.0      # [N]  접촉을 예상하지 않는 일반 이동 중 상한
SIDE_LIMIT = 15.0           # [N]  예상 방향에 수직인 힘 상한
TORQUE_LIMIT = 5.0          # [Nm] 툴 모멘트 크기 상한 (4.0 → 5.0: 이동 중 3.5Nm 안팎이 자주 나와서)
WARN_RATIO = 0.7            # 상한의 이 비율 이상이면 경고

# 접촉 예상 없는 이동의 판정 방식 (free_mode)
#   abs      : |힘| ≥ FREE_MOVE_LIMIT 이면 바로 충돌 (기존, wall_size 기본)
#   baseline : 천천히 따라가는 기준선(시상수 FREE_TAU)과의 '차이' 가 FREE_MOVE_LIMIT(힘) / TORQUE_LIMIT(모멘트)
#              이상으로 FREE_HOLD 동안 유지될 때만 충돌. 자세가 바뀌며 생기는 추정 오차(서서히/한 번 튀고 일정)는
#              기준선이 흡수하고, 계속 커지는 진짜 충돌만 잡는다. 절대값 FREE_HARD(힘) / FREE_MHARD(모멘트) 는 그대로 즉시 정지.
FREE_TAU = 0.5              # [s]  기준선 시상수 (작을수록 계단형 오차를 빨리 흡수, 느린 충돌도 흡수할 위험)
FREE_HOLD = 0.3             # [s]  차이가 이 시간 동안 유지돼야 충돌
FREE_HARD = 40.0            # [N]  baseline 모드에서도 이 이상이면 즉시 충돌
FREE_MHARD = 8.0            # [Nm] baseline 모드에서도 이 이상이면 즉시 충돌
FREE_GAP = 0.5              # [s]  접촉 없는 이동 검사가 이 시간 이상 끊겼으면 새 이동 → 기준선 다시 잡음

# 암 외력 토크: 관절별 기준 상한 [Nm] + 툴 힘이 만드는 몫(|F| × 팔 길이 근사 [m])
#   툴로 벽을 누르면 관절에도 토크가 생기므로, 그만큼은 '정상'으로 빼준다 (보수적 근사).
ARM_TORQUE_BASE = [15.0, 20.0, 15.0, 6.0, 6.0, 6.0]
ARM_TORQUE_LEVER = [0.9, 0.9, 0.6, 0.25, 0.25, 0.25]
ARM_CHECK_PERIOD = 0.1      # [s] 관절 토크 확인 주기 (서비스 호출이라 매 샘플은 무거움)
STATE_CHECK_PERIOD = 0.2    # [s] 로봇 상태 확인 주기

SENSOR_FAIL_LIMIT = 3       # 연속 읽기 실패 허용 횟수
MAX_TOOL_COLLISIONS = 2     # 툴 충돌이 연속 이만큼이면 전체 중단
RELIEVE_MM = 20.0           # 툴 충돌 후 후퇴 거리
RELIEVE_VELOCITY = 15.0          # 후퇴 속도 [mm/s]
CLEAR_FORCE = 5.0           # 후퇴 후 툴 힘이 이 아래로 ...
CLEAR_TORQUE = 1.0          # ... 모멘트가 이 아래로
CLEAR_HOLD = 0.5            # ... 이 시간 동안 유지되면 '힘이 빠졌다' [s]
CLEAR_TIMEOUT = 3.0         # 이 시간 안에 안 빠지면 원인이 남아있음 → 전체 중단 [s]

# 로봇 상태 (dsr_msgs2/srv/GetRobotState.srv)
STATE_NAMES = {0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
               5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMMING', 8: 'RECOVERY',
               9: 'SAFE_STOP2', 10: 'SAFE_OFF2', 15: 'NOT_READY'}
BAD_STATES = {3, 5, 6, 9, 10}


# =============================================================================
# 이벤트 / 예외
# =============================================================================
@dataclass
class ForceEvent:
    kind: str                   # TOOL_OVERLOAD | TOOL_SIDE | TOOL_TORQUE | ARM_CONTACT | CONTROLLER | SENSOR_FAIL
    message: str
    context: str = ''           # 무엇을 하던 중이었나 (seek / follow / move ...)
    value: float = 0.0
    limit: float = 0.0
    pos: list = field(default_factory=list)
    wrench: list = field(default_factory=list)
    ext_torque: list = field(default_factory=list)
    time: float = 0.0

    def to_dict(self):
        return asdict(self)


class ForceGuardError(RuntimeError):
    """force_guard 가 던지는 모든 예외의 부모. .event 에 상세 정보."""
    recoverable = False

    def __init__(self, event):
        super().__init__(f'[{event.kind}] {event.message}')
        self.event = event


class ToolCollision(ForceGuardError):
    """툴(그리퍼)에 예상 밖 힘/토크 → 후퇴 후 계속할 수 있음."""
    recoverable = True


class ArmCollision(ForceGuardError):
    """팔 몸통에 외력 → 사람/물체 가능성. 자동 이동 금지, 전체 중단."""


class ControllerStop(ForceGuardError):
    """컨트롤러가 안전정지/비상정지 → 움직일 수 없음, 전체 중단."""


class SensorFault(ForceGuardError):
    """센서를 못 읽음 → 감시 불가, 전체 중단."""


# =============================================================================
# (virtual 전용) 사고 주입
# =============================================================================
class Injector:
    """에뮬레이터에서 예외 처리를 시험하려고 측정값에 가짜 외력/상태를 더한다."""

    KINDS = ('none', 'tool_side', 'tool_overload', 'tool_torque', 'arm', 'estop', 'sensor')

    def __init__(self, kind='none', after=20.0, duration=1.5, side_axis=2):
        if kind not in self.KINDS:
            raise ValueError(f'inject 는 {self.KINDS} 중 하나')
        self.kind, self.after, self.duration = kind, after, duration
        self.side_axis = side_axis
        self.t0 = time.time()

    def active(self):
        dt = time.time() - self.t0
        return self.kind != 'none' and self.after <= dt <= self.after + self.duration

    def wrench(self, w):
        if not self.active():
            return w
        w = np.array(w, float)
        if self.kind == 'tool_side':
            w[self.side_axis] += 30.0
        elif self.kind == 'tool_overload':
            w[:3] += 60.0 / np.sqrt(3)
        elif self.kind == 'tool_torque':
            w[3] += 8.0
        return w

    def ext_torque(self, t):
        if self.active() and self.kind == 'arm':
            t = np.array(t, float)
            t[1] += 40.0                    # J2 에 40Nm
        return t

    def state(self, s):
        return 6 if (self.active() and self.kind == 'estop') else s

    def sensor_fail(self):
        return self.active() and self.kind == 'sensor'


# =============================================================================
# 감시기
# =============================================================================
class ForceGuard:
    """사용법:
        guard = ForceGuard(io)               # io: wall_size.SizeIO
        guard.zero()                         # 접촉 없는 정지 상태에서 관절 토크 영점
        ...
        w = guard.check(pos, wrench, expected=(APP_DIR,), context='seek')   # 루프마다
        → 문제 있으면 정지하고 ToolCollision / ArmCollision / ... 예외
    """

    def __init__(self, io, injector=None):
        self.io = io
        self.inj = injector or Injector('none')
        self.events = []                    # 기록 (저장용)
        self.tripped = None                 # 마지막으로 발동한 이벤트 (reset 전까지)
        self.tool_streak = 0                # 연속 툴 충돌 수
        self.torque_bias = np.zeros(6)
        self.arm_ok = True                  # 관절 토크 서비스 사용 가능 여부
        self.state_ok = True
        self._fails = 0
        self._t_arm = self._t_state = 0.0
        self._t_warn = 0.0
        self._last_ext = np.zeros(6)
        # 접촉 없는 이동 판정 (free_mode='baseline')
        self.free_mode = 'abs'
        self.free_tau, self.free_hold = FREE_TAU, FREE_HOLD
        self.free_hard, self.free_mhard = FREE_HARD, FREE_MHARD
        self._fbase = None                  # 기준선 wrench (6,)
        self._ft = 0.0                      # 마지막 접촉 없는 이동 검사 시각
        self._fsince = None                 # 차이가 상한을 넘기 시작한 시각
        self.last_w = None                  # 마지막으로 검사한 wrench (기록용, 충돌 순간 포함)
        self.last_dev = (0.0, 0.0)          # 마지막 기준선 차이 (힘 N, 모멘트 Nm)

    # ------------------------------------------------------------ 준비
    def wait_for_services(self):
        d = self.io.dsr
        for cli in (d._ros2_get_external_torque, d._ros2_get_robot_state, d._ros2_get_last_alarm):
            if not cli.wait_for_service(timeout_sec=3.0):
                self.io.node.get_logger().info(f'[가드] 서비스 없음: {cli.srv_name}')
                if cli is d._ros2_get_external_torque:
                    self.arm_ok = False
                if cli is d._ros2_get_robot_state:
                    self.state_ok = False

    def zero(self, samples=10):
        """접촉 없이 정지한 상태에서 관절 외력 토크 영점 (모델 오차/오프셋 제거)."""
        if not self.arm_ok:
            return
        vals = []
        for _ in range(samples):
            t = self._read_ext_raw()
            if t is not None:
                vals.append(t)
            time.sleep(0.01)
        if vals:
            self.torque_bias = np.mean(vals, axis=0)
            self.io.node.get_logger().info(
                f'[가드] 관절 토크 영점: {np.round(self.torque_bias, 2).tolist()}')
        else:
            self.arm_ok = False
            self.io.node.get_logger().info('[가드] 관절 외력 토크를 읽을 수 없음 → 암 충돌 감시 끔 (컨트롤러 감지만 사용)')

    # ------------------------------------------------------------ 메인 검사
    def check(self, pos=None, wrench=None, expected=(), context=''):
        """한 번 검사. wrench 를 주면 그걸 쓰고(중복 읽기 방지), 없으면 직접 읽는다.
        expected: 지금 정상적으로 접촉이 예상되는 방향들 (단위벡터, BASE).
        Returns: 사용한 wrench (6,)."""
        now = time.time()
        if pos is None:
            pos = self._safe(self.io.get_pos, None)
        pos = [] if pos is None else [float(v) for v in pos[:3]]

        # ① 툴 힘/토크
        if self.inj.sensor_fail():
            wrench = None                   # (테스트) 센서 읽기 실패 흉내
        elif wrench is None:
            wrench = self._read_wrench()
        if wrench is None:
            self._fails += 1
            if self._fails >= SENSOR_FAIL_LIMIT:
                self._trip(SensorFault, 'SENSOR_FAIL', f'툴 힘 센서를 {self._fails}번 연속 못 읽음',
                           context, pos)
            return np.zeros(6)
        self._fails = 0
        w = self.inj.wrench(wrench)
        self.last_w = np.asarray(w, float)
        self._check_tool(w, expected, context, pos)

        # ② 암 외력 토크 (주기적으로)
        if self.arm_ok and now - self._t_arm >= ARM_CHECK_PERIOD:
            self._t_arm = now
            self._check_arm(w, context, pos)

        # ③ 로봇 상태 (주기적으로)
        if self.state_ok and now - self._t_state >= STATE_CHECK_PERIOD:
            self._t_state = now
            self._check_state(w, context, pos)
        return w

    def ok_streak(self):
        """선 하나를 문제 없이 끝냈을 때 호출 → 연속 충돌 수 초기화."""
        self.tool_streak = 0

    def free_reset(self):
        """기준선을 버림 (영점을 다시 잡아 측정값이 바뀌었을 때 등). 다음 검사 값이 새 기준선."""
        self._fbase, self._fsince = None, None

    def reset(self):
        """회복 처리를 마친 뒤 호출 (tripped 해제)."""
        self.tripped = None

    # ------------------------------------------------------------ 회복 동작
    def relieve(self, retreat_dir, dist=RELIEVE_MM):
        """툴 충돌 후: 힘을 빼고 retreat_dir 로 천천히 조금 물러난다 (감시 없이 짧게).
        Returns: 성공 여부."""
        io = self.io
        try:
            io.release_press()
            pos = io.get_pos()
            d = np.asarray(retreat_dir, float)
            d = d / np.linalg.norm(d)
            io.node.get_logger().info(f'[가드] 후퇴 {dist:.0f}mm (방향 {np.round(d, 2).tolist()})')
            io.move_l_unguarded(pos + d * dist, vel=RELIEVE_VELOCITY)
            return True
        except Exception as e:
            io.node.get_logger().info(f'[가드] 후퇴 실패: {e}')
            return False

    def wait_clear(self, timeout=CLEAR_TIMEOUT):
        """후퇴 후 툴에 걸린 힘이 사라졌는지 확인 (CLEAR_HOLD 동안 계속 작아야 OK).
        Returns: True = 힘이 빠짐 (계속해도 됨), False = 계속 눌리고 있음 (원인 남아있음)."""
        t0 = time.time()
        t_ok = None
        fn = mn = float('nan')
        while time.time() - t0 < timeout:
            w = None if self.inj.sensor_fail() else self._read_wrench()
            if w is not None:
                w = self.inj.wrench(w)
                fn, mn = float(np.linalg.norm(w[:3])), float(np.linalg.norm(w[3:6]))
                if fn < CLEAR_FORCE and mn < CLEAR_TORQUE:
                    t_ok = t_ok or time.time()
                    if time.time() - t_ok >= CLEAR_HOLD:
                        self.io.node.get_logger().info(
                            f'[가드] 힘이 빠짐 (힘 {fn:.1f}N, 모멘트 {mn:.2f}Nm) → 계속')
                        return True
                else:
                    t_ok = None
            else:
                t_ok = None
            time.sleep(0.02)
        self.io.node.get_logger().info(
            f'[가드] {timeout:.0f}초 동안 힘이 안 빠짐 (힘 {fn:.1f}N, 모멘트 {mn:.2f}Nm)')
        return False

    def last_alarm(self):
        """컨트롤러 마지막 알람 (level/group/index/param) 문자열."""
        try:
            a = self.io.dsr.get_last_alarm()
            if a == -1 or a is None:
                return '알람 없음/읽기 실패'
            return f'level={a.level} group={a.group} index={a.index} param={list(a.param)}'
        except Exception as e:
            return f'알람 읽기 실패: {e}'

    # ------------------------------------------------------------ 내부: 검사
    def _check_tool(self, w, expected, context, pos):
        f, m = np.asarray(w[:3], float), np.asarray(w[3:6], float)
        fn, mn = float(np.linalg.norm(f)), float(np.linalg.norm(m))
        if not expected and self.free_mode == 'baseline':
            self._check_free(np.asarray(w[:6], float), fn, mn, context, pos)
            return
        self._fbase = None                  # 접촉 이동이 끼면 다음 접촉 없는 이동에서 기준선을 새로
        self.last_dev = (0.0, 0.0)

        if expected:
            hard = TOOL_HARD_LIMIT
            # 예상 방향들이 만드는 부분공간에 수직인 성분 = '옆에서 받은 힘'
            E = np.array([np.asarray(e, float) / np.linalg.norm(e) for e in expected]).T
            Q, _ = np.linalg.qr(E)
            side = f - Q @ (Q.T @ f)
            sn = float(np.linalg.norm(side))
        else:
            hard = FREE_MOVE_LIMIT          # 접촉을 예상하지 않음 → 작은 힘도 충돌
            sn = 0.0

        if fn >= hard:
            self._trip(ToolCollision, 'TOOL_OVERLOAD',
                       f'툴 힘 {fn:.1f}N ≥ {hard:.0f}N' + ('' if expected else ' (접촉 예상 없는 이동 중)'),
                       context, pos, w, value=fn, limit=hard)
        if sn >= SIDE_LIMIT:
            self._trip(ToolCollision, 'TOOL_SIDE',
                       f'예상 밖 방향 힘 {sn:.1f}N ≥ {SIDE_LIMIT:.0f}N (옆/위에서 충돌?)',
                       context, pos, w, value=sn, limit=SIDE_LIMIT)
        if mn >= TORQUE_LIMIT:
            self._trip(ToolCollision, 'TOOL_TORQUE',
                       f'툴 모멘트 {mn:.2f}Nm ≥ {TORQUE_LIMIT:.1f}Nm (손끝이 아닌 몸체가 걸림?)',
                       context, pos, w, value=mn, limit=TORQUE_LIMIT)

        ratio = max(fn / hard, sn / SIDE_LIMIT, mn / TORQUE_LIMIT)
        if ratio >= WARN_RATIO and time.time() - self._t_warn > 1.0:
            self._t_warn = time.time()
            self.io.node.get_logger().info(f'[가드] 경고: 힘 {fn:.1f}N / 옆 {sn:.1f}N / 모멘트 {mn:.2f}Nm '
                                           f'(상한의 {ratio * 100:.0f}%, {context})')

    def _check_free(self, w, fn, mn, context, pos):
        """접촉 없는 이동, 기준선 방식. 기준선 = 지수이동평균 (시상수 free_tau)."""
        now = time.time()
        if self._fbase is None or now - self._ft > FREE_GAP:
            self._fbase, self._fsince = w.copy(), None
        dt = max(0.0, now - self._ft)
        self._ft = now
        dev = w - self._fbase
        fd, md = float(np.linalg.norm(dev[:3])), float(np.linalg.norm(dev[3:6]))
        self.last_dev = (fd, md)
        base_s = f'(기준선 대비 힘 {fd:.1f}N / 모멘트 {md:.2f}Nm, 절대 {fn:.1f}N / {mn:.2f}Nm)'

        if fn >= self.free_hard:
            self._trip(ToolCollision, 'TOOL_OVERLOAD', f'툴 힘 {fn:.1f}N ≥ {self.free_hard:.0f}N (접촉 예상 없는 이동 중) {base_s}',
                       context, pos, w, value=fn, limit=self.free_hard)
        if mn >= self.free_mhard:
            self._trip(ToolCollision, 'TOOL_TORQUE', f'툴 모멘트 {mn:.2f}Nm ≥ {self.free_mhard:.1f}Nm (접촉 예상 없는 이동 중) {base_s}',
                       context, pos, w, value=mn, limit=self.free_mhard)
        if fd >= FREE_MOVE_LIMIT or md >= TORQUE_LIMIT:
            if self._fsince is None:
                self._fsince = now
            elif now - self._fsince >= self.free_hold:
                kind, val, lim = (('TOOL_OVERLOAD', fd, FREE_MOVE_LIMIT) if fd / FREE_MOVE_LIMIT >= md / TORQUE_LIMIT
                                  else ('TOOL_TORQUE', md, TORQUE_LIMIT))
                self._trip(ToolCollision, kind, f'기준선 대비 급변이 {self.free_hold:.1f}s 유지 {base_s}',
                           context, pos, w, value=val, limit=lim)
        else:
            self._fsince = None
        # 기준선 갱신 (계단형/서서히 생기는 추정 오차는 free_tau 동안 흡수됨)
        a = min(1.0, dt / max(self.free_tau, 1e-3))
        self._fbase = self._fbase + a * (w - self._fbase)

        ratio = max(fd / FREE_MOVE_LIMIT, md / TORQUE_LIMIT, fn / self.free_hard, mn / self.free_mhard)
        if ratio >= WARN_RATIO and now - self._t_warn > 1.0:
            self._t_warn = now
            self.io.node.get_logger().info(f'[가드] 경고: {base_s} (상한의 {ratio * 100:.0f}%, {context})')

    def _check_arm(self, w, context, pos):
        raw = self._read_ext_raw()
        if raw is None:
            return                          # 관절 토크는 보조 감시 → 한 번 실패는 무시
        t = self.inj.ext_torque(raw - self.torque_bias)
        self._last_ext = t
        fn = float(np.linalg.norm(w[:3]))
        mn = float(np.linalg.norm(w[3:6]))
        allowed = np.array(ARM_TORQUE_BASE) + fn * np.array(ARM_TORQUE_LEVER) + mn
        over = np.abs(t) - allowed
        j = int(np.argmax(over))
        if over[j] > 0:
            self._trip(ArmCollision, 'ARM_CONTACT',
                       f'J{j + 1} 외력 토크 {abs(t[j]):.1f}Nm > 허용 {allowed[j]:.1f}Nm '
                       f'(툴 힘 {fn:.1f}N 로 설명 안 됨 → 팔 몸통 충돌?)',
                       context, pos, w, value=float(abs(t[j])), limit=float(allowed[j]))

    def _check_state(self, w, context, pos):
        try:
            s = self.io.dsr.get_robot_state()
        except Exception:
            return
        s = self.inj.state(s)
        if isinstance(s, int) and s in BAD_STATES:
            alarm = self.last_alarm()
            self._trip(ControllerStop, 'CONTROLLER',
                       f'로봇 상태 {STATE_NAMES.get(s, s)} (컨트롤러 안전정지/비상정지). 알람: {alarm}',
                       context, pos, w, value=float(s))

    # ------------------------------------------------------------ 내부: 발동
    def _trip(self, exc_cls, kind, msg, context, pos, w=None, value=0.0, limit=0.0):
        ev = ForceEvent(kind=kind, message=msg, context=context, value=float(value),
                        limit=float(limit), pos=list(pos),
                        wrench=[float(v) for v in (w if w is not None else [])],
                        ext_torque=[float(v) for v in self._last_ext], time=time.time())
        self.events.append(ev)
        self.tripped = ev
        if exc_cls is ToolCollision:
            self.tool_streak += 1
        self.io.node.get_logger().error(f'[가드] {kind}: {msg}  (작업: {context})')
        # 공통 1차 대응: 즉시 정지 + 힘/컴플라이언스 제어 해제 (밀고 있던 힘을 뺀다)
        if kind != 'CONTROLLER':            # 안전정지 상태에서는 정지 명령도 거부될 수 있음
            self._safe(self.io.stop)
            self._safe(self.io.wait_motion_done, None, 3.0)
        self._safe(self.io.release_press)
        raise exc_cls(ev)

    # ------------------------------------------------------------ 내부: 읽기
    def _read_wrench(self):
        try:
            return np.asarray(self.io.get_force(), float)
        except Exception:
            return None

    def _read_ext_raw(self):
        try:
            t = self.io.dsr.get_external_torque()
            if isinstance(t, (list, tuple)) and len(t) >= 6:
                return np.asarray(t[:6], float)
        except Exception:
            pass
        return None

    @staticmethod
    def _safe(fn, default=None, *args):
        try:
            return fn(*args)
        except Exception:
            return default
