"""유한 거리의 비동기 이동 중 BASE 외력 변화를 감시하는 접촉 탐색."""

# ===== 파일 역할 / 통신 안내 =====
# 내부 공통 모듈: 별도 노드가 아니다. io로 전달받은 함수가 실제 ROS 서비스를 호출한다.
# 탐색: 유한 거리 이동 중 시작 대비 해당 BASE 축 외력 변화량을 감시하고 기준 도달 시 정지한다.
# 바닥 -Z를 찾은 뒤, 누르기 방식(press_method)에 따라 벽 +X를 찾거나 측정된 벽 위치로 바로 간다.
#   direct   : 측정된 벽 X(+1 mm)까지 위치 이동, 먼저 닿으면 그 자리를 벽으로 사용 → 힘제어
#   hybrid   : 힘으로 벽을 찾은 뒤 힘제어
#   position : 힘으로 벽을 찾은 뒤 목표 힘/강성만큼 위치로 밀어 넣기 (set_desired_force 미사용)
#   force    : 예전 방식. 정지 후 set_desired_force만으로 힘 형성 대기
# WallForceHold는 컴플라이언스·목표 힘의 시작/해제를 묶는다. 탐색 감지 기준과 유지 목표 힘은 별개다.
# 외력 [Fx,Fy,Fz,Mx,My,Mz]에서 앞 3개는 N, 뒤 3개는 Nm이다.

import math
import time


# ===== 하이퍼파라미터 =====
# 벽 정체 판정: 이동 명령 중인데 이 시간 동안 X가 거의 늘지 않고 외력이 조금이라도 생겼으면
# 벽에 막힌 것으로 본다. 헤라를 눕히면 날이 휘어 힘이 감지 기준까지 오르지 않을 수 있기 때문이다.
STALL_WINDOW_S = 1.0
STALL_TRAVEL_MM = 0.3
STALL_MIN_FORCE_N = 1.5
# direct: 측정된 벽 X보다 이만큼 더 들어간 곳까지 위치로 민다. 컴플라이언스 상태라 힘 ≈ 강성 × 깊이.
DIRECT_PRESS_MM = 1.0
FLOOR_LIFT_TOLERANCE_MM = 5.0   # 바닥 감지 후 들기(컴플라이언스 중) 도착 허용 오차


# 두산 API set_desired_force와 같은 요청을 만든다. BASE X만 힘제어(dir=[1,0,0,0,0,0])한다.
# mode 1(DR_FC_MOD_REL): 힘제어를 시작한 순간의 센서값을 0으로 보고 그 위에 value만큼 추가로 민다.
# sign: +1이면 BASE +X(도포 벽), -1이면 BASE -X(뜨기 용기 벽)로 민다. value는 힘의 크기(양수)다.
def press_force_request(value, mode=1, sign=1):
    from dsr_msgs2.srv import SetDesiredForce

    value = float(value)
    # 최소값 제한은 두지 않는다(예전 10 N 하한은 임의 규칙이라 제거). 샌딩 하한은 샌딩 쪽에서 검사한다.
    if not math.isfinite(value) or value <= 0:
        raise ValueError('BASE X 힘 목표는 양의 유한한 값이어야 합니다.')

    request = SetDesiredForce.Request()
    if sign not in (1, -1):
        raise ValueError('힘 방향 sign은 +1 또는 -1이어야 합니다.')
    request.fd = [sign*value, 0.0, 0.0, 0.0, 0.0, 0.0]
    request.dir = [1, 0, 0, 0, 0, 0]  # BASE X축 힘 제어 활성화
    request.ref = 0                   # BASE 좌표계
    if mode not in (0, 1):
        raise ValueError('힘 모드는 absolute(0)/relative(1)여야 합니다.')
    request.mod = mode                # 1=상대(기본, 샌딩과 동일). 감시는 설정 직전 Fx를 offset으로 뺀다.
    request.time = 0.3                # 목표 힘까지 올리는 데 걸리는 시간 s
    return request


# 컴플라이언스와 목표 힘의 시작/해제를 묶는 내부 클래스다. 노드가 아니다.
class WallForceHold:
    """정지 상태에서 힘 제어를 시작하고 해제 실패 시에도 나머지 정리를 시도한다."""

    def __init__(self, io, force, stiffness):
        self.io, self.force, self.stiffness = io, force, stiffness
        self.active = False

    def start(self):
        self.active = True  # 응답 실패여도 컨트롤러 적용 여부가 불확실하므로 정리한다
        self.io.compliance(self.stiffness)
        self.io.set_force(self.force)

    def release(self):
        if not self.active:
            return
        try:
            self.io.release_force()
        finally:
            self.io.release_compliance()
            self.active = False


# 탐색 방향·거리·감지 힘·유지 힘 입력을 검사한다.
def validate_search(args):
    if not getattr(args, 'contact_search', False):
        return
    if args.plane != 'yz' or args.normal_sign != 'negative':
        raise ValueError('접촉 탐색은 단일 YZ 벽의 +X 접근에서만 지원합니다.')
    if args.end[2] <= args.start[2] or any(abs(args.end[i]-args.start[i]) > 1e-6 for i in (0, 1)):
        raise ValueError('접촉 탐색 후 도포는 X/Y 고정 +Z 방향이어야 합니다.')
    for name in ('floor_force', 'wall_force', 'search_speed', 'search_distance'):
        value = getattr(args, name)
        if value is None or not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}에 양의 유한한 값을 지정하세요. 힘 기준은 자동 선택하지 않습니다.')
    below = getattr(args, 'floor_below_gap', None)
    if below is not None and (not math.isfinite(below) or below < 0 or
                              getattr(args, 'approach_clearance_z', 0.0)+below <= 0):
        raise ValueError('floor-below-gap 은 0 이상이고, approach-clearance-z 와 합이 0보다 커야 합니다.')
    press = getattr(args, 'press_force', 6.0)
    if not math.isfinite(press) or press <= 0:
        raise ValueError('press-force는 양의 유한한 N 값이어야 합니다.')
    method = getattr(args, 'press_method', 'force')
    if method not in ('force', 'position', 'hybrid', 'direct'):
        raise ValueError('press-method는 direct, hybrid, position, force 중 하나여야 합니다.')
    if method in ('position', 'hybrid', 'direct'):
        k, tol = press_stiffness(args), press_tolerance(args)
        if not math.isfinite(k) or not 100 <= k <= 20000:
            raise ValueError('press-stiffness는 100~20000 N/m여야 합니다.')
        if not math.isfinite(tol) or not 0 < tol < press:
            raise ValueError('press-tolerance는 0보다 크고 목표 힘보다 작아야 합니다.')
        if position_press_limit(args) <= press+tol:
            raise ValueError('force-limit은 목표 힘+press-tolerance보다 커야 합니다.')
    clearance = getattr(args, 'approach_clearance_z', 0.0)
    if not math.isfinite(clearance) or not 0 <= clearance < args.search_distance:
        raise ValueError('approach-clearance-z는 0 이상, 탐색 거리 미만이어야 합니다.')
    backoff = getattr(args, 'wall_backoff', 0.0)
    if not math.isfinite(backoff) or not 0 <= backoff < args.search_distance:
        raise ValueError('wall-backoff는 0 이상, 탐색 거리 미만이어야 합니다.')
    lift = getattr(args, 'floor_lift', 0.0)
    if not math.isfinite(lift) or not 0 <= lift < args.search_distance:
        raise ValueError('floor-lift는 0 이상, 탐색 거리 미만이어야 합니다.')
    confirm = getattr(args, 'contact_confirm', 1)
    if not isinstance(confirm, int) or not 1 <= confirm <= 10:
        raise ValueError('contact-confirm은 1~10 정수여야 합니다.')
    for name in ('force_tolerance', 'force_wait_timeout', 'force_stable_time', 'force_loss_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다.')
    if args.force_tolerance >= press:
        raise ValueError('force-tolerance는 목표 힘보다 작아야 합니다.')
    limit = force_limit(args)
    if method == 'force' and (not math.isfinite(limit) or limit <= press+args.force_tolerance):
        raise ValueError('force-limit은 목표 힘+허용오차보다 커야 합니다.')
    if args.force_stable_time >= args.force_wait_timeout:
        raise ValueError('force-stable-time은 force-wait-timeout보다 작아야 합니다.')


# 위치 누르기 방식에서 쓰는 X 강성과 감시 허용오차/상한.
def press_stiffness(args):
    return getattr(args, 'press_stiffness', 1000.0)


def press_tolerance(args):
    return getattr(args, 'press_tolerance', 4.0)


def position_press_limit(args):
    value = getattr(args, 'force_limit', None)
    if value is None:
        # 헤라 각도가 커지는 동안 날 끝이 벽 쪽으로 돌아 들어가 힘이 서서히 오른다
        # (실측 3 N 목표에서 Z≈254·33°에 10 N). 충돌 수준만 막도록 상한은 최소 20 N.
        return max(getattr(args, 'press_force', 6.0)+2*press_tolerance(args), 20.0)
    return value


def position_press(io, args, clock=time.monotonic, sleep=time.sleep, push=True):
    """벽은 힘으로 '찾기만' 하고, 누르는 양은 위치(깊이)로 정한다.

    1) 컴플라이언스만 켠 채 +X로 이동하다 Fx가 벽 감지 기준(wall_force) 이상 변하면 그곳을 벽으로 본다.
    2) 벽 X에서 깊이(목표 힘/강성)만큼 위치로 더 밀어 넣는다. 스프링이 눌려 그만큼 힘이 생긴다.
    set_desired_force와 '목표 힘 도달 대기'를 쓰지 않는다.
    반환: (실제 TCP, 기준 X = 벽 X + 깊이, 기준 Fx).
    """
    k = press_stiffness(args)
    limit = position_press_limit(args)
    confirm = getattr(args, 'contact_confirm', 1)
    threshold = args.wall_force
    depth = args.press_force/k*1000.0
    travel_max = args.search_distance
    start = tuple(io.read_pose())
    offset = checked_force(io.read_force())[0]
    goal = list(start)
    goal[0] += travel_max
    io.info(f'[위치 누르기] 컴플라이언스 X {k:g} N/m | 벽 감지 {threshold:g} N, +X 최대 '
            f'{travel_max:g} mm, {args.search_speed:g} mm/s | 벽 감지 후 +{depth:.1f} mm 밀기 '
            f'(≈{args.press_force:g} N) | 기준 Fx {offset:+.2f} N')
    hits = 0
    stopped = False
    history = []  # (시각, X) — 벽에 막혀 정체됐는지 판단
    next_log = 0.0
    io.move_async(tuple(goal), args.search_speed)
    started = clock()
    deadline = started+travel_max/args.search_speed*3+5
    try:
        while True:
            pushed = abs(checked_force(io.read_force())[0]-offset)
            if pushed > limit:
                raise RuntimeError(f'벽 찾기 중 힘 상한 초과: {pushed:.2f} N > {limit:g} N')
            hits = hits+1 if pushed >= threshold else 0
            actual = tuple(io.read_pose())
            now = clock()
            history.append((now, actual[0]))
            history = [h for h in history if now-h[0] <= STALL_WINDOW_S]
            # 정체 판정 기준은 파일 위 STALL_* 상수 주석 참고.
            stalled =(now-started >= STALL_WINDOW_S+0.5 and history and
                       now-history[0][0] >= STALL_WINDOW_S*0.9 and
                       actual[0]-history[0][1] < STALL_TRAVEL_MM and pushed >= STALL_MIN_FORCE_N)
            if now >= next_log:
                io.info(f'[벽 찾기] +{actual[0]-start[0]:.1f} mm | 외력 변화 {pushed:.2f} N')
                next_log = now+1.0
            if hits >= confirm or stalled:
                io.stop_and_wait()
                stopped = True
                if stalled and hits < confirm:
                    io.info(f'[벽 찾기] X 정체로 벽 판정 (외력 변화 {pushed:.2f} N < {threshold:g} N, '
                            f'최근 {STALL_WINDOW_S:g} s 이동 {actual[0]-history[0][1]:.2f} mm)')
                break
            if actual[0]-start[0] >= travel_max-0.1:
                raise RuntimeError(f'+X {travel_max:g} mm 안에서 벽(외력 변화 {threshold:g} N)을 찾지 못했습니다. '
                                   '벽이 더 멀거나 시작점이 틀렸습니다.')
            if math.dist(actual[1:3], start[1:3]) > 2.0:
                raise RuntimeError('벽 찾기 중 Y/Z가 2 mm를 벗어났습니다.')
            if clock() >= deadline:
                raise TimeoutError('벽 찾기 시간 초과')
            sleep(0.05)
    finally:
        if not stopped:
            io.stop_and_wait()
    wall = tuple(io.read_pose())
    if not push:
        # hybrid: 위치 밀기 없이 벽에서 멈춘 채 반환. 이후 누르기는 힘제어가 맡는다.
        io.info(f'[벽 감지] 벽 X {wall[0]:.2f} (시작 대비 +{wall[0]-start[0]:.1f} mm)')
        return wall, wall[0], offset
    reference_x = wall[0]+depth
    io.info(f'[위치 누르기] 벽 X {wall[0]:.2f} (시작 대비 +{wall[0]-start[0]:.1f} mm) → '
            f'기준 X {reference_x:.2f}까지 +{depth:.1f} mm 밀기')
    # 벽에 막혀 목표 X에 도달하지 못하는 것이 정상이므로 도착 판정 없이 정지만 기다린다.
    io.move_async((reference_x, *wall[1:]), args.search_speed, '벽 밀어 넣기')
    end = clock()+depth/args.search_speed*3+5
    while True:
        pushed = abs(checked_force(io.read_force())[0]-offset)
        if pushed > limit:
            io.stop_and_wait()
            raise RuntimeError(f'밀어 넣기 중 힘 상한 초과: {pushed:.2f} N > {limit:g} N')
        if io.motion_idle():
            break
        if clock() >= end:
            io.stop_and_wait()
            break
        sleep(0.05)
    actual = tuple(io.read_pose())
    io.info(f'[위치 누르기] 밀기 완료 X {actual[0]:.2f} | 외력 변화 {pushed:.2f} N')
    return actual, reference_x, offset


def direct_press(io, args, wall_x, free_fx=None, clock=time.monotonic, sleep=time.sleep):
    """측정해 둔 벽 X(+DIRECT_PRESS_MM)까지 컴플라이언스 상태로 가되, 먼저 닿으면 그 자리를 벽으로 쓴다.

    헤라를 잡을 때마다 깊이가 조금씩 달라 측정 위치보다 벽이 가까울 수 있다(실측: 접근점에서 이미
    벽에 닿아 3501 충돌 감지). 그래서
      1) free_fx(벽에서 멀리 떨어진 허공의 Fx) 대비 이미 wall_force 이상이면 더 밀지 않는다.
      2) 이동 중 외력 변화가 wall_force 이상이거나 X가 막혀 정체되면 그 자리에서 멈춘다.
    반환: (실제 TCP, 벽 X, 기준 Fx).
    """
    limit = position_press_limit(args)
    threshold = args.wall_force
    confirm = getattr(args, 'contact_confirm', 1)
    start = tuple(io.read_pose())
    offset = checked_force(io.read_force())[0]
    if free_fx is not None and abs(offset-free_fx) >= threshold:
        io.info(f'[바로 시작] 이미 벽에 닿아 있음 (허공 대비 Fx {offset-free_fx:+.2f} N) → '
                f'추가로 밀지 않고 현재 X {start[0]:.2f}를 벽으로 사용')
        return start, start[0], offset
    goal = (wall_x+DIRECT_PRESS_MM, *start[1:])
    io.info(f'[바로 시작] 측정된 벽 X {wall_x:.2f} (+{DIRECT_PRESS_MM:g} mm)까지 {args.search_speed:g} mm/s 이동 '
            f'(+{goal[0]-start[0]:.1f} mm), 먼저 닿으면 그 자리에서 정지 | 기준 Fx {offset:+.2f} N')
    if goal[0]-start[0] > args.search_distance:
        raise RuntimeError(f'측정된 벽까지 {goal[0]-start[0]:.1f} mm로 최대 {args.search_distance:g} mm보다 멉니다. '
                           '시작점 설정을 확인하세요.')
    if goal[0] <= start[0]:
        io.info('[바로 시작] 이미 측정된 벽 위치 이상에 있음 → 이동 없이 시작')
        return start, start[0], offset
    io.move_async(goal, args.search_speed, '측정된 벽 위치로 이동')
    began = clock()
    end = began+abs(goal[0]-start[0])/args.search_speed*3+5
    hits, history, contact = 0, [], False
    pushed = 0.0
    while True:
        pushed = abs(checked_force(io.read_force())[0]-offset)
        if pushed > limit:
            io.stop_and_wait()
            raise RuntimeError(f'벽 위치로 이동 중 힘 상한 초과: {pushed:.2f} N > {limit:g} N')
        actual = tuple(io.read_pose())
        now = clock()
        history = [h for h in history+[(now, actual[0])] if now-h[0] <= STALL_WINDOW_S]
        hits = hits+1 if pushed >= threshold else 0
        stalled = (now-began >= STALL_WINDOW_S+0.5 and now-history[0][0] >= STALL_WINDOW_S*0.9 and
                   actual[0]-history[0][1] < STALL_TRAVEL_MM and pushed >= STALL_MIN_FORCE_N)
        if hits >= confirm or stalled:
            io.stop_and_wait()
            contact = True
            break
        if io.motion_idle():
            break
        if now >= end:
            io.stop_and_wait()
            break
        sleep(0.05)
    actual = tuple(io.read_pose())
    wall = actual[0] if contact else wall_x
    io.info(f'[바로 시작] {"먼저 닿아 정지" if contact else "도착"} X {actual[0]:.2f} | 외력 변화 {pushed:.2f} N'
            f' → 벽 X {wall:.2f}')
    return actual, wall, offset


# 즉시 중단할 절대 상한. 미지정 시 목표+허용오차×3 (기본 6+3=9 N).
def force_limit(args):
    value = getattr(args, 'force_limit', None)
    if value is None:
        return getattr(args, 'press_force', 6.0)+3*args.force_tolerance
    return value


# 조회한 외력 배열이 유효한 6개 유한 수인지 검사한다.
def checked_force(values):
    values = tuple(values)
    if len(values) != 6 or not all(map(math.isfinite, values)):
        raise RuntimeError('유효한 BASE 외력을 읽지 못했습니다.')
    return values


class PressForceMonitor:
    """실측 |BASE Fx|를 감시한다. 힘 조절은 컨트롤러가 맡고 이 클래스는 출발/중단을 판정한다.

    반력 부호를 목표 명령 부호로 단정하지 않는다. 출발 시 확인한 실측 부호를
    저장해 도포 중 반대 부호의 큰 힘을 정상 유지로 오인하지 않는다.
    툴 중량 오차나 접촉 위치를 검증하는 기능은 아니다.
    """

    def __init__(self, target, tolerance, stable_time, loss_timeout, limit=None, offset=0.0,
                 warn_only=False):
        self.target, self.tolerance = target, tolerance
        # warn_only: 위치로 누르는 방식. 힘 부족/부호는 경고만 하고 상한 초과만 중단한다.
        self.warn_only = warn_only
        # 상대 모드: 힘 설정 직전의 Fx를 빼고 '추가로 만든 힘'만 목표와 비교한다.
        self.offset = offset
        # 허용 범위를 잠깐 넘는 과도 힘은 loss_timeout으로 판정하고, 이 상한만 즉시 중단한다.
        self.limit = target+3*tolerance if limit is None else limit
        self.stable_time, self.loss_timeout = stable_time, loss_timeout
        self.since = self.lost_since = None
        self.samples = 0
        self.ready = False
        self.sign = None

    def update(self, force, now):
        fx = checked_force(force)[0]-self.offset
        magnitude = abs(fx)
        if magnitude > self.limit:
            raise RuntimeError(f'힘 상한 초과: |BASE Fx|={magnitude:.3f} N '
                               f'> {self.limit:g} N')
        in_band = abs(magnitude-self.target) <= self.tolerance
        if self.warn_only:
            self.ready = True
            return '유지 확인' if in_band else ('힘 과다(경고)' if magnitude > self.target else '힘 부족(경고)')
        if self.ready:
            if in_band and (1 if fx > 0 else -1) != self.sign:
                raise RuntimeError('도포 중 실측 Fx 부호 반전. 접촉 상태 확인이 필요합니다.')
            if in_band:
                self.lost_since = None
                return '유지 확인'
            if self.lost_since is None:
                self.lost_since = now
            if now-self.lost_since >= self.loss_timeout:
                kind = '과다' if magnitude > self.target else '부족'
                raise RuntimeError(f'도포 힘 {kind} 지속: |BASE Fx|={magnitude:.3f} N, '
                                   f'허용 범위 {self.target-self.tolerance:g}~'
                                   f'{self.target+self.tolerance:g} N')
            return '힘 과다 감시' if magnitude > self.target else '힘 부족 감시'
        sign = 1 if fx > 0 else -1
        if not in_band:
            self.since, self.samples, self.sign = None, 0, None
            return '힘 형성 대기'
        if self.since is None or self.sign != sign:
            self.since, self.samples, self.sign = now, 0, sign
        self.samples += 1
        if now-self.since >= self.stable_time and self.samples >= 3:
            self.ready = True
            return '출발 조건 충족'
        return '힘 안정 확인 중'


def wait_for_press_force(io, monitor, timeout, max_distance,
                         clock=time.monotonic, sleep=time.sleep, origin=None):
    """위치 이동 명령 없이 힘 형성을 기다린다. 실패 시 호출자가 정지·해제한다."""
    origin = tuple(io.read_pose()) if origin is None else tuple(origin)
    deadline = clock()+timeout
    next_log = 0.0
    while True:
        actual = tuple(io.read_pose())
        force = checked_force(io.read_force())
        now = clock()
        if now >= deadline:
            io.log_force('힘 형성 시간 초과·상승 금지', actual=actual, measured=force)
            raise TimeoutError(f'목표 {monitor.target:g} N 힘 형성 시간 초과. '
                               '상향 도포를 시작하지 않습니다.')
        if abs(actual[0]-origin[0]) >= max_distance:
            raise RuntimeError('힘 형성 대기 중 X 이동이 최대 탐색 범위에 도달했습니다.')
        if math.dist(actual[1:3], origin[1:3]) > 2.0:
            raise RuntimeError('힘 형성 대기 중 Y/Z 위치가 2 mm를 벗어났습니다.')
        status = io.read_state()
        if status not in (1, 2):
            raise RuntimeError(f'힘 형성 대기 중 로봇 상태 {status}')
        try:
            verdict = monitor.update(force, now)
        except RuntimeError:
            io.log_force('힘 형성 중 이상·상승 금지', actual=actual, measured=force)
            raise
        if now >= next_log or monitor.ready:
            io.log_force(verdict, actual=actual, measured=force)
            next_log = now+0.5
        if monitor.ready:
            return actual
        sleep(0.05)


# 비동기 이동 후 힘을 반복 조회한다. 접촉/거리 제한/오류 시 정지하며 다음 축으로 진행 여부를 결정한다.
def seek_axis(io, axis, direction, threshold, distance, speed,
              clock=time.monotonic, sleep=time.sleep, confirm=1, stall_mm=0.0, stall_sec=0.0, miss_ok=False):
    """축별 시작 외력 대비 변화량이 기준 이상이면 정지·대기 후 자세를 반환한다.

    confirm: 기준 이상 변화가 연속 몇 번 조회돼야 접촉으로 볼지. 순간 잡음 오판 방지.
    miss_ok: distance 끝까지 접촉이 없으면 실패 대신 그 자리 자세를 반환한다 (바닥 탐색: 틈 Z 아래 한계).
    stall_sec > 0: stall_sec 동안 stall_mm 넘게 더 나아가지 못하면(막힘) 힘이 기준에 못 미쳐도 접촉으로 본다.
      컴플라이언스(강성이 낮음)에서는 막혀도 지령-실제 차이 × 강성 만큼만 힘이 나서 기준에 못 닿는다
      (뜨기 벽 탐색: 강성 1 N/mm, 6.8/10 mm 에서 막혀 2.2 N → 20 s 시간 초과).
    """
    start = tuple(io.read_pose())
    hits = 0
    baseline = checked_force(io.read_force())
    target = list(start)
    target[axis] += direction*distance
    started = False
    stopped = False
    try:
        # 응답 시간 초과에도 명령이 이미 수신되었을 수 있으므로 stop을 시도한다.
        started = True
        io.move_async(tuple(target), speed)
        t0 = clock()
        limit = distance/speed*3+5
        deadline = t0+limit
        next_log = t0
        best, t_best = 0.0, t0      # 가장 멀리 간 거리와 그때 시각 (막힘 판정용)
        while True:
            force = checked_force(io.read_force())
            delta = force[axis]-baseline[axis]
            hits = hits+1 if abs(delta) >= threshold else 0
            if hits >= confirm:
                io.stop_and_wait()
                stopped = True
                return tuple(io.read_pose())
            actual = tuple(io.read_pose())
            travel = direction*(actual[axis]-start[axis])
            now = clock()
            if travel > best+stall_mm:
                best, t_best = travel, now
            if stall_sec > 0 and now-t0 >= stall_sec+0.5 and now-t_best >= stall_sec:
                print(f'[접촉 탐색] 축 {axis} {stall_sec:g} s 동안 {stall_mm:g} mm 넘게 못 나아감 → 막힘 = 접촉 '
                      f'(이동 {travel:.1f} mm, 힘 변화 {delta:+.2f} N)', flush=True)
                io.stop_and_wait()
                stopped = True
                return tuple(io.read_pose())
            if now >= next_log:     # 진단용 출력만 (판정·동작은 그대로)
                print(f'[접촉 탐색] 축 {axis} 이동 {travel:.1f}/{distance:g} mm | 힘 변화 {delta:+.2f} N '
                      f'(기준 {threshold:g} N) | 경과 {now-t0:.1f}/{limit:.0f} s', flush=True)
                next_log = now+0.5
            if travel >= distance-0.1 or (miss_ok and getattr(io, 'motion_idle', lambda: False)()):
                if miss_ok:
                    print(f'[접촉 탐색] 축 {axis} {distance:g} mm 끝까지 접촉 없음 → 그 자리를 접촉점으로 사용 '
                          f'(힘 변화 {delta:+.2f} N)', flush=True)
                    io.stop_and_wait()
                    stopped = True
                    return tuple(io.read_pose())
                raise RuntimeError(f'축 {axis} 탐색 {distance:g} mm 내에서 접촉을 찾지 못했습니다.')
            if clock() >= deadline:
                raise TimeoutError('접촉 탐색 시간 초과')
            sleep(0.05)
    finally:
        if started and not stopped:
            io.stop_and_wait()


# 바닥 -Z→벽 +X 순서이며 마지막 실제 TCP가 도포 경로의 기준이 된다.
def seek_floor_then_wall(io, args):
    """바닥 −Z, 벽 +X 순서. 한 단계라도 실패하면 다음 단계를 수행하지 않는다."""
    confirm = getattr(args, 'contact_confirm', 1)
    # 틈 좌표 Z 를 믿고, 시작 높이(틈 Z + approach_clearance_z)에서 틈 Z − floor_below_gap 까지만 내려간다.
    # 그 안에서 못 닿으면 그 자리(틈 Z − floor_below_gap)를 바닥으로 본다 (2026-11-20 사용자 결정).
    below = getattr(args, 'floor_below_gap', None)
    if below is None:
        distance, miss_ok = args.search_distance, False
    else:
        distance, miss_ok = getattr(args, 'approach_clearance_z', 0.0)+below, True
    io.info(f'바닥 −Z 접촉 탐색 (최대 {distance:g} mm' + (f' = 틈 Z −{below:g} mm, 못 닿으면 그 자리를 바닥으로)'
                                                       if miss_ok else ')'))
    floor = seek_axis(io, 2, -1, args.floor_force, distance, args.search_speed,
                      confirm=confirm, miss_ok=miss_ok)
    io.info(f'바닥 접촉 정지: {floor[:3]}')
    if getattr(io, 'log_force', None):
        io.log_force('바닥 감지 후', actual=floor)
    # 바닥을 누른 채 +X로 끌면 바닥 마찰이 Fx 변화로 잡혀 벽 접촉으로 오판한다. 살짝 띄운 뒤 벽을 찾는다.
    lift = getattr(args, 'floor_lift', 0.0)
    if lift > 0:
        lifted = (*floor[:2], floor[2]+lift, *floor[3:])
        io.info(f'바닥 마찰 오판 방지: +Z {lift:g} mm 들고 벽 탐색')
        # 컴플라이언스가 켜진 채 드는 동작이라, 바닥을 누르던 힘(Fz)·마찰(Fx)이 풀리며 몇 mm 더 밀린다.
        # (run104 Y=120 틈: X +1.7, Z +1.2 → 합 2.1 mm 로 2 mm 검사에 걸림). 이후 기준은 실제 위치를 다시 읽는다.
        io.move_line('바닥 감지 후 들기', lifted, args.search_speed, tolerance=FLOOR_LIFT_TOLERANCE_MM)
        floor = tuple(io.read_pose())
    if getattr(args, 'press_method', 'force') in ('position', 'hybrid', 'direct'):
        # 벽은 위치 누르기 단계에서 컴플라이언스를 켠 채 찾는다. 잡음에 약한 저힘 벽 탐색은 생략.
        return floor
    wall = seek_axis(io, 0, 1, args.wall_force, args.search_distance, args.search_speed,
                     confirm=confirm)
    io.info(f'벽 접촉 정지: {wall[:3]}')
    if getattr(io, 'log_force', None):
        io.log_force('벽 감지 후', actual=wall)
    return wall
