"""실시간 6축 관절각 모니터 + 특이점 알람 순간 관절각 기록.

실행: ros2 run rokey_team joint_monitor
      (다른 터미널에서 작업을 돌리는 동안 켜 둔다. 로봇을 움직이지 않는 읽기 전용 노드)

화면 위쪽: 현재 J1~J6 각도와 대표 특이점 지표까지의 여유
  - 손목 특이점: J5 ≈ 0°  (J4·J6 축이 일직선)
  - 팔꿈치 특이점: J3 ≈ 0° (팔이 일자로 펴짐)
  지표는 참고용이다. '진짜 특이점'인지는 컨트롤러 알람(3205 등)이 판정한다.
화면 아래쪽: 컨트롤러 특이점 알람이 온 순간의 관절각 목록
  같은 내용을 ~/.ros/singularity_events.csv 에 누적 저장한다.
"""

import csv
import math
import os
import sys
import time

ROBOT_ID = 'dsr01'
WRIST_WARN_DEG = 10.0   # |J5|가 이보다 작으면 손목 특이점 근처
ELBOW_WARN_DEG = 45.0   # |J3|가 이보다 작으면 팔꿈치 특이점 근처 (실측: J3=39.1°에서 3205 알람)
EVENT_FILE = os.path.expanduser('~/.ros/singularity_events.csv')


def margins(joints):
    """대표 특이점까지 남은 각도(deg)."""
    return {'손목 J5→0°': abs(joints[4]), '팔꿈치 J3→0°': abs(joints[2])}


def is_singularity_alarm(message):
    text = ' '.join((message.msg1, message.msg2, message.msg3)).lower()
    return message.code == 3205 or 'singular' in text


def render(joints, events, last_update):
    lines = ['\033[2J\033[H', '=== 관절각 모니터 (Ctrl+C 종료) ===']
    if joints is None:
        lines.append('joint_states 대기 중...')
    else:
        age = time.monotonic()-last_update
        lines.append('현재 관절각 (deg)' + (f'  ※ {age:.1f}s 동안 갱신 없음' if age > 1.0 else ''))
        lines.append('  ' + '  '.join(f'J{i+1}={v:8.2f}' for i, v in enumerate(joints)))
        lines.append('특이점 지표까지 여유 (참고용)')
        for name, value in margins(joints).items():
            limit = WRIST_WARN_DEG if '손목' in name else ELBOW_WARN_DEG
            flag = '  <<< 근접' if value < limit else ''
            lines.append(f'  {name}: {value:6.1f}°{flag}')
    lines.append('')
    lines.append(f'특이점 알람 기록 ({len(events)}건, 최근 10건) — 파일: {EVENT_FILE}')
    if not events:
        lines.append('  (없음)')
    for stamp, code, joints_at, text in events[-10:]:
        joint_str = ' '.join(f'J{i+1}={v:.1f}' for i, v in enumerate(joints_at)) if joints_at else '관절각 미수신'
        lines.append(f'  {stamp} code={code} [{joint_str}]')
        lines.append(f'      {text[:100]}')
    sys.stdout.write('\n'.join(lines)+'\n')
    sys.stdout.flush()


def main():
    import rclpy
    from sensor_msgs.msg import JointState
    from dsr_msgs2.msg import RobotError

    rclpy.init()
    node = rclpy.create_node('joint_monitor', namespace=ROBOT_ID)
    state = {'joints': None, 'updated': 0.0}
    events = []

    def on_joints(message):
        if len(message.position) >= 6:
            state['joints'] = tuple(math.degrees(p) for p in message.position[:6])
            state['updated'] = time.monotonic()

    def on_error(message):
        if not is_singularity_alarm(message):
            return
        stamp = time.strftime('%H:%M:%S')
        text = ' / '.join(s for s in (message.msg1, message.msg2, message.msg3) if s)
        joints = state['joints']
        events.append((stamp, message.code, joints, text))
        new_file = not os.path.exists(EVENT_FILE)
        with open(EVENT_FILE, 'a', newline='') as handle:
            writer = csv.writer(handle)
            if new_file:
                writer.writerow(['time', 'code', 'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'message'])
            writer.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), message.code,
                             *(f'{v:.2f}' for v in (joints or ('',)*6)), text])

    node.create_subscription(JointState, f'/{ROBOT_ID}/joint_states', on_joints, 10)
    node.create_subscription(RobotError, f'/{ROBOT_ID}/error', on_error, 100)
    next_draw = 0.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_draw:
                render(state['joints'], events, state['updated'])
                next_draw = time.monotonic()+0.5
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
