#!/usr/bin/env python3
"""[백업: 탐침으로 높이·너비 모두 측정하던 원래 버전 (2026-11-22)]  실행: ros2 run rokey1 pick_tap_backup
물건 집기 → 관절 0 → 홈 → wall_tap 벽 측정 → 홈 → 물건 반납 → 관절 0 → 홈.

  집기 : 홈 → 관절 0 → 집기점 위(+Z UP) movej → 열기 → 수직 하강 movel → 닫기 → 수직 상승 movel → 관절 0 (movej)
  측정 : wall_tap.main 그대로 (홈 → 벽 측정 → 홈 복귀). 쥔 물건 끝 = 탐침
         탐침 길이는 robot.yaml wall_tap.probe_len 한 곳에서만 정함 (-p probe_len 이 있으면 그게 우선)
  반납 : 측정 성공 + 홈 도착일 때만. 관절 0 → 집기점 위 movej → 수직 하강 → 열기 → 수직 상승 → 관절 0 → 홈
         (측정 실패/사고 정지면 반납 안 함 → 물건은 그리퍼에 남음)

집기점은 관절값(PICK_J)으로 받고, FK 로 TCP 자세를 구해 그 위 UP mm 지점을
같은 solution space 로 IK → 위/아래는 같은 자세의 수직 직선 이동.
그리퍼 신호/속도는 putty_demo/pickup_spatula.py 와 같다.

[실행]
  ros2 run rokey1 pick_tap --ros-args -p mode:=virtual
  ros2 run rokey1 pick_tap --ros-args -p mode:=real -p pick_j:="199.96,28.99,68.54,0,79.30,110.52"
  (wall_tap 파라미터도 같이 줄 수 있음)
"""
import time

import numpy as np

try:
    from navigation import wall_tap_backup as wall_tap
    from navigation.wall_size import HOME_J
except ImportError:
    import wall_tap_backup as wall_tap
    from wall_size import HOME_J

PICK_J = (199.96, 28.99, 68.54, 0.0, 79.30, 110.52)   # 집기 관절값 [deg]. -p pick_j:="j1,...,j6"
UP = 100.0                  # 집기점 위 접근 / 상승 거리 (+Z) [mm]
TRANSFER_VEL = 15.0         # 상승 속도 [mm/s]
PICK_VEL = 5.0              # 집기/반납 하강 속도 [mm/s]
GRIP_WAIT = 1.0             # 그리퍼 동작 대기 [s]
ZERO_J = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
OPEN = ((1, 0), (2, 0), (2, 1))           # pickup_spatula.py 와 같은 DO 순서
CLOSE = ((1, 0), (2, 0), (1, 1), (2, 0))

_spot = {}                  # 집기 때 계산한 자세 (반납에 그대로 사용)


def _plan(io):
    """PICK_J → (집기 TCP 자세, 위 TCP 자세, 위 관절값)."""
    d = io.dsr
    default = tuple(wall_tap.C.pick_j) or PICK_J            # robot.yaml 탐침 프리셋의 pick_j (없으면 PICK_J)
    q = wall_tap._floats(io.node.declare_parameter('pick_j', ','.join(map(str, default)),
                                                   wall_tap.ParameterDescriptor(dynamic_typing=True)).value)
    if len(q) != 6:
        raise ValueError(f'pick_j 는 관절 6개 "j1,...,j6": {q}')
    pick_x = [float(v) for v in d.fkin(d.posj(*q), ref=d.DR_BASE)][:6]
    above_x = pick_x[:2] + [pick_x[2] + UP] + pick_x[3:]
    above_j = io.dsr.ikin(d.posx(*above_x), d.get_solution_space(d.posj(*q)), ref=d.DR_BASE)
    above_j = [float(v) for v in above_j][:6]
    above_j[0] += 360.0 * round((q[0] - above_j[0]) / 360.0)   # J1 을 집기 관절 쪽 해로 (±360 같은 자세)
    if len(above_j) != 6 or max(abs(a - b) for a, b in zip(above_j, q)) > 45.0:
        raise RuntimeError(f'집기점 위 IK 실패/이상: {above_j} (집기 {q})')
    return pick_x, above_x, above_j


def _visit(io, grip_seq, name):
    """위 → 수직 하강 → 그리퍼 → 수직 상승 → 관절 0."""
    d, log = io.dsr, io.node.get_logger()
    pick_x, above_x, above_j = _spot['pose']

    def movel(p, vel, step):
        log.info(f'[{name}] {step}')
        io._check(d.movel(d.posx(*p), vel=[vel, 10.0], acc=[20.0, 20.0], ref=d.DR_BASE), step)

    log.info(f'[{name}] 관절 0 경유 → 집기점 위로 movej {np.round(above_j, 2).tolist()}')
    io.move_j(ZERO_J)
    io.move_j(above_j)
    if grip_seq is CLOSE:
        grip(io, OPEN)
    movel(pick_x, PICK_VEL, '하강')
    grip(io, grip_seq)
    movel(above_x, TRANSFER_VEL, '수직 상승')
    log.info(f'[{name}] 관절 0 경유')
    io.move_j(ZERO_J)


def grip(io, seq):
    for ch, v in seq:
        io.dsr.set_digital_output(ch, v)
    time.sleep(GRIP_WAIT)


def check_probe_len():
    """탐침(쥔 물건)이 있으니 probe_len 이 0 이면 탐침 끝이 아니라 손끝 위치로 벽을 찍게 됨 → 움직이기 전에 멈춤."""
    if wall_tap.C.probe_len <= 0:
        raise RuntimeError(f'탐침 길이 probe_len={wall_tap.C.probe_len} → robot.yaml wall_tap.probe_len 에 '
                           '쥔 물건 끝까지 길이[mm]를 넣으세요. 로봇은 움직이지 않았습니다.')


def pick(io):
    check_probe_len()
    io.node.get_logger().info(f'[집기] 탐침 길이 {wall_tap.C.probe_len:g} mm (robot.yaml wall_tap.probe_len)')
    _spot['pose'] = _plan(io)
    io.move_j(HOME_J)
    _visit(io, CLOSE, '집기')
    # → wall_tap.main: 홈 → 측정 → 홈


def place(io):
    q = io.dsr.get_current_posj()
    if max(abs(float(a) - b) for a, b in zip(q, HOME_J)) > 1.0:
        raise RuntimeError(f'홈이 아님 {np.round(q, 1).tolist()} → 반납 안 함 (물건은 그리퍼에 있음)')
    _visit(io, OPEN, '반납')
    io.move_j(HOME_J)


def main(args=None):
    wall_tap.main(args, pre=pick, post=place)


if __name__ == '__main__':
    main()
