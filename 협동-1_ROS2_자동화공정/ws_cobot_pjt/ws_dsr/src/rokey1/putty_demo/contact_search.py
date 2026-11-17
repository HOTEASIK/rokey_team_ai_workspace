"""유한 거리의 비동기 이동 중 BASE 외력 변화를 감시하는 접촉 탐색."""

# ===== 파일 역할 / 통신 안내 =====
# 내부 공통 모듈: 별도 노드가 아니다. io로 전달받은 함수가 실제 ROS 서비스를 호출한다.
# 탐색: 유한 거리 이동 중 시작 대비 해당 BASE 축 외력 변화량을 감시하고 기준 도달 시 정지.
# 바닥 -Z, 벽 +X를 순서대로 찾는다. 벽 감지 후 추가 위치 이동은 없다.
# WallForceHold는 도포 유지 힘의 시작/해제를 관리한다. 탐색 감지 기준과 유지 목표 힘은 별개다.
# 외력 [Fx,Fy,Fz,Mx,My,Mz]에서 앞 3개는 N, 뒤 3개는 모멘트다.

import math
import time


# BASE X만 힘 제어하도록 dir=[1,0,0,0,0,0], 절대 목표 힘을 요청한다.
def press_force_request(value):
    from dsr_msgs2.srv import SetDesiredForce
    request = SetDesiredForce.Request()
    request.fd = [value, 0.0, 0.0, 0.0, 0.0, 0.0]
    request.dir = [1, 0, 0, 0, 0, 0]
    request.ref, request.mod, request.time = 0, 0, 0.3
    return request


# 컴플라이언스와 목표 힘의 시작/해제를 묶는 내부 클래스다. 노드가 아니다.
class WallForceHold:
    """정지 상태에서 힘 제어를 시작하고 해제 실패 시에도 나머지 정리를 시도한다."""

    def __init__(self, io, force, stiffness):
        self.io, self.force, self.stiffness = io, force, stiffness
        self.active = False

    def start(self):
        self.active = True  # 응답 실패여도 컨트롤러 적용 여부가 불확실하므로 정리한다.
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
    press = getattr(args, 'press_force', 6.0)
    if not math.isfinite(press) or press <= 0:
        raise ValueError('press-force는 양의 유한한 N 값이어야 합니다.')
    for name in ('force_tolerance', 'force_wait_timeout', 'force_stable_time', 'force_loss_timeout'):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f'{name}은 양의 유한한 값이어야 합니다.')
    if args.force_tolerance >= press:
        raise ValueError('force-tolerance는 목표 힘보다 작아야 합니다.')
    if args.force_stable_time >= args.force_wait_timeout:
        raise ValueError('force-stable-time은 force-wait-timeout보다 작아야 합니다.')


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

    def __init__(self, target, tolerance, stable_time, loss_timeout):
        self.target, self.tolerance = target, tolerance
        self.stable_time, self.loss_timeout = stable_time, loss_timeout
        self.since = self.lost_since = None
        self.samples = 0
        self.ready = False
        self.sign = None

    def update(self, force, now):
        fx = checked_force(force)[0]
        magnitude = abs(fx)
        if magnitude > self.target+self.tolerance:
            raise RuntimeError(f'힘 상한 초과: |BASE Fx|={magnitude:.3f} N '
                               f'> {self.target+self.tolerance:g} N')
        in_band = abs(magnitude-self.target) <= self.tolerance
        if self.ready:
            if in_band and (1 if fx > 0 else -1) != self.sign:
                raise RuntimeError('도포 중 실측 Fx 부호 반전. 접촉 상태 확인이 필요합니다.')
            if in_band:
                self.lost_since = None
                return '유지 확인'
            if self.lost_since is None:
                self.lost_since = now
            if now-self.lost_since >= self.loss_timeout:
                raise RuntimeError(f'도포 힘 부족 지속: |BASE Fx|={magnitude:.3f} N, '
                                   f'허용 범위 {self.target-self.tolerance:g}~'
                                   f'{self.target+self.tolerance:g} N')
            return '힘 부족 감시'
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
              clock=time.monotonic, sleep=time.sleep):
    """축별 시작 외력 대비 변화량이 기준 이상이면 정지·대기 후 자세를 반환한다."""
    start = tuple(io.read_pose())
    baseline = checked_force(io.read_force())
    target = list(start)
    target[axis] += direction*distance
    started = False
    stopped = False
    try:
        # 응답 시간 초과에도 명령이 이미 수신되었을 수 있으므로 stop을 시도한다.
        started = True
        io.move_async(tuple(target), speed)
        deadline = clock()+distance/speed*3+5
        while True:
            force = checked_force(io.read_force())
            if abs(force[axis]-baseline[axis]) >= threshold:
                io.stop_and_wait()
                stopped = True
                return tuple(io.read_pose())
            actual = tuple(io.read_pose())
            travel = direction*(actual[axis]-start[axis])
            if travel >= distance-0.1:
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
    io.info('바닥 −Z 접촉 탐색')
    floor = seek_axis(io, 2, -1, args.floor_force, args.search_distance, args.search_speed)
    io.info(f'바닥 접촉 정지: {floor[:3]}')
    if getattr(io, 'log_force', None):
        io.log_force('바닥 감지 후', actual=floor)
    wall = seek_axis(io, 0, 1, args.wall_force, args.search_distance, args.search_speed)
    io.info(f'벽 접촉 정지: {wall[:3]}')
    if getattr(io, 'log_force', None):
        io.log_force('벽 감지 후', actual=wall)
    return wall
