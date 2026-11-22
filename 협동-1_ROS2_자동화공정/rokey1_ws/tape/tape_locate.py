#!/usr/bin/env python3
"""테이프 롤(축 // BASE Y) 중심 위치를 구해서 ~/tape_model/tape_roll.json 에 저장.

[모드]
  teach : 티칭 펜던트로 TCP 를 '롤 폭 가운데의 축 중심'에 맞춘 뒤 실행 → 현재 TCP 좌표를 저장.
          (축 끝에 맞췄다면 --y-offset 으로 롤 폭 가운데까지의 Y 차이[mm]를 준다)
  probe : 저장된 중심(또는 --center x y z)을 대략값으로 삼아, XZ 평면에서
          여러 각도로 롤 표면을 눌러보고 원 피팅 → 중심 x, z 보정 (y 는 대략값 유지).

[실행]
  ros2 run rokey1 tape_locate teach [--y-offset 25]
  ros2 run rokey1 tape_locate probe [--center 500 0 200]

[주의 - probe]
  - 손가락은 닫은 상태로, 손가락 벌림 방향이 Y(롤 축)와 평행하도록 PROBE_ORI 를 맞출 것.
  - TCP 를 손가락 끝 '공'으로 보면, 피팅한 반지름 = 롤 반지름 + 손끝 반지름 이고
    중심은 그대로 맞다. 그래서 반지름은 확인용으로만 쓴다.
"""
import argparse
import json
import math
import os

import numpy as np
import rclpy

from utils.aspects import dsr_main
from utils.move_stop import MoveStopClient

TOOL_NAME = 'Tool Weight'
TCP_NAME = 'GripperDA_v1'

HOME = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
ROLL_R = 20.0                         # 롤 반지름 [mm] (바깥지름 40mm)
MODEL_FILE = os.path.expanduser('~/tape_model/tape_roll.json')

PROBE_ORI = [0.0, 180.0, 0.0]         # TODO: 툴 Z 아래 방향 자세 (rx, ry, rz) 실제 값 확인
PROBE_ANGLES = [-90.0, -45.0, 0.0, 45.0]  # 0 = 꼭대기, 음수 = 로봇 쪽(-X)
START_GAP = 30.0                      # 표면에서 이만큼 떨어진 곳에서 출발 [mm]
MAX_DEPTH = 10.0                      # 표면 안쪽으로 최대 이만큼까지 진행 [mm]
SAFE_DZ = 60.0                        # 이동 시 롤 꼭대기 위 여유 높이 [mm]
PROBE_VELOCITY = [5.0, 5.0]
MOVE_VELOCITY = [50.0, 30.0]
ACC = [100.0, 60.0]
FORCE_THRESHOLD = 3.0                 # 접촉 판정: 힘 변화량 크기 [N]


def fit_circle(xs, zs):
    """최소자승 원 피팅 → (xc, zc, r, rms 오차)."""
    xs, zs = np.asarray(xs), np.asarray(zs)
    A = np.c_[xs, zs, np.ones_like(xs)]
    a, b, c = np.linalg.lstsq(A, xs ** 2 + zs ** 2, rcond=None)[0]
    xc, zc = a / 2, b / 2
    r = math.sqrt(c + xc ** 2 + zc ** 2)
    rms = float(np.sqrt(np.mean((np.hypot(xs - xc, zs - zc) - r) ** 2)))
    return xc, zc, r, rms


def save_model(center, source, extra=None):
    os.makedirs(os.path.dirname(MODEL_FILE), exist_ok=True)
    data = {'center_mm': [round(v, 2) for v in center], 'source': source, **(extra or {})}
    with open(MODEL_FILE, 'w') as f:
        json.dump(data, f, indent=2)
    x, y, z = (v / 1000.0 for v in center)
    print(f'\n저장: {MODEL_FILE}\n{json.dumps(data, indent=2)}')
    print(f'\nros2 launch rokey1 tape_display.launch.py x:={x:.4f} y:={y:.4f} z:={z:.4f}')


def parse_options(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('mode', choices=['teach', 'probe'])
    parser.add_argument('--y-offset', type=float, default=0.0)
    parser.add_argument('--center', type=float, nargs=3)
    return parser.parse_args(rclpy.utilities.remove_ros_args(args)[1:])


@dsr_main('tape_locate')
def run(app, opt):
    node = app.node

    import DSR_ROBOT2 as robot
    from DSR_ROBOT2 import posj, posx

    if opt.mode == 'teach':
        p = robot.get_current_posx()[0]
        save_model([p[0], p[1] + opt.y_offset, p[2]], 'teach')
        return

    # ---------------- probe ----------------
    if opt.center:
        xc, yc, zc = opt.center
    else:
        with open(MODEL_FILE) as f:
            xc, yc, zc = json.load(f)['center_mm']
    node.get_logger().info(f'대략 중심: ({xc:.1f}, {yc:.1f}, {zc:.1f})')

    stopper = MoveStopClient(node)

    def quick_stop():
        stopper.stop(robot.DR_QSTOP)

    def force():
        return np.array(robot.get_tool_force(robot.DR_BASE)[:3])

    def pt(theta_deg, dist):
        """중심에서 theta 방향으로 dist 떨어진 점 (XZ 평면)."""
        t = math.radians(theta_deg)
        return posx(xc + dist * math.sin(t), yc, zc + dist * math.cos(t), *PROBE_ORI)

    robot.set_robot_mode(robot.ROBOT_MODE_MANUAL)
    robot.set_tool(TOOL_NAME)
    robot.set_tcp(TCP_NAME)
    robot.set_robot_mode(robot.ROBOT_MODE_AUTONOMOUS)

    def mv(p):
        if robot.movel(p, vel=MOVE_VELOCITY, acc=ACC) < 0:
            raise RuntimeError(f'movel 실패: {list(p)}')

    # 관절 0 자세(특이점)에서는 movel 이 실패하므로 홈부터 간다
    if robot.movej(posj(*HOME), vel=30, acc=60) < 0:
        raise RuntimeError('movej HOME 실패')

    safe_z = zc + ROLL_R + SAFE_DZ
    hits = []
    for th in PROBE_ANGLES:
        start = pt(th, ROLL_R + START_GAP)
        above = list(start)
        above[2] = safe_z
        mv(posx(*above))
        mv(start)
        robot.wait(0.5)
        base = force()
        robot.amovel(pt(th, ROLL_R - MAX_DEPTH), vel=PROBE_VELOCITY, acc=ACC)
        hit = False
        while robot.check_motion() != robot.DR_STATE_IDLE:
            if np.linalg.norm(force() - base) > FORCE_THRESHOLD:
                quick_stop()
                hit = True
                break
        robot.wait(0.3)
        p = robot.get_current_posx()[0]
        mv(start)
        mv(posx(*above))
        if not hit:
            raise RuntimeError(f'{th}° 에서 접촉 없음 → 대략 중심 확인')
        node.get_logger().info(f'{th:+.0f}° 접촉: x={p[0]:.2f} z={p[2]:.2f}')
        hits.append([th, p[0], p[2]])

    _, xs, zs = zip(*hits)
    fx, fz, fr, rms = fit_circle(xs, zs)
    node.get_logger().info(
        f'피팅: 중심 x={fx:.2f} z={fz:.2f}, r={fr:.2f} (롤 {ROLL_R} + 손끝), rms={rms:.2f}mm')
    save_model([fx, yc, fz], 'probe',
               {'fit_r_mm': round(fr, 2), 'rms_mm': round(rms, 3), 'hits': hits})


def main(args=None):
    # 인자 파싱은 rclpy.init 전에 해서 잘못된 인자면 노드를 띄우지 않는다
    opt = parse_options(args)
    return run(args, opt)


if __name__ == '__main__':
    main()
