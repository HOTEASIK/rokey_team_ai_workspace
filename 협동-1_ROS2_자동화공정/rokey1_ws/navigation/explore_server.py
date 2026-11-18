#!/usr/bin/env python3
"""/process/explore 액션 서버: wall_tap 으로 벽을 재고 결과를 Process 양식으로 돌려준다.

  hmi_server(process_runner) ─goal→ explore_server ─ros2 run→ wall_tap (자식 프로세스, 기존 코드 그대로)
                             ←result (wall[12], gaps)── ~/wall_scan_data/explore_run<id>_*.json 읽기

  wall     : top / floor / right / left 각 [x,y,z] (BASE mm). 벽 표면 위, 높이·너비 측정선 기준
  gaps     : robot.yaml explore.gaps (틈 자동 감지가 생기기 전까지 손으로 넣는 값)
  feedback : wall_tap 단계(floor / wall / height / width)마다 1번 → task_log 1줄 (params = robot.yaml wall_tap 값)

로봇은 wall_tap 이 움직이고, 이 서버는 움직이지 않는다.
웹 시작 버튼이 확인 역할이므로 wall_tap 의 y 입력 확인(confirm)은 끈다.
취소(웹 정지/비상정지): wall_tap 일시정지 → move_stop(QSTOP) → 강제 종료 → 힘/순응 해제.
  자동 홈 복귀는 하지 않는다 (정지 뒤 로봇이 스스로 움직이지 않게).

실행 (bringup 과 같은 mode):
  ros2 run rokey1 explore_server --ros-args -p mode:=virtual
"""
import json
import os
import statistics
import time
from datetime import datetime

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from dsr_msgs2.srv import MoveStop, ReleaseComplianceCtrl, ReleaseForce
from rokey_interfaces.action import Process

from navigation.wall_size import DIRECTIONS, SAVE_DIR, UP_AXIS
from utils.config import CFG
from utils.robot import ROBOT_ID
from utils.worker_process import WorkerProcess

DR_QSTOP = 1


def wall_corners(data):
    """wall_tap 결과 JSON → [top, floor, right, left] xyz 12개. 높이/너비가 없으면 ValueError.

    옆/위 좌표는 측정 결과(중앙값), 벽 쪽 좌표는 그 모서리에서 가장 가까운 벽 접촉점의 실측값
    → 벽이 기울어 있으면 4점에 그대로 남는다 (법선은 utils/wall.wall_normal 로 계산).
    """
    app_axis, _, lat_axis, _ = DIRECTIONS[data['direction']]
    hits = [t['xyz'] for t in data['taps'] if t['hit'] and t['kind'] != 'floor']
    lines = [ln for ln in data['width_lines'] if ln['ok']]
    if 'top_z' not in data['results'] or not lines or not hits:
        raise ValueError('높이/너비 결과 없음')

    def pt(lat, up):
        near = min(hits, key=lambda h: (h[lat_axis] - lat) ** 2 + (h[UP_AXIS] - up) ** 2)
        p = [0.0, 0.0, 0.0]
        p[lat_axis], p[app_axis], p[UP_AXIS] = lat, float(near[app_axis]), up
        return p

    z = statistics.median(ln['up'] for ln in lines)
    right = statistics.median(ln['edges']['right']['edge'] for ln in lines)
    left = statistics.median(ln['edges']['left']['edge'] for ln in lines)
    c = data['lat_center']
    return pt(c, data['results']['top_z']) + pt(c, data['floor_z']) + pt(right, z) + pt(left, z)


def manual_gaps():
    """robot.yaml explore.gaps → [sx,sy,sz, ex,ey,ez, ...] 한 줄로."""
    out = []
    for g in CFG['explore']['gaps'] or []:
        if len(g) != 6:
            raise ValueError(f'robot.yaml explore.gaps 는 [sx,sy,sz, ex,ey,ez] 6개씩: {g}')
        out += [float(v) for v in g]
    return out


class ExploreServer(Node):
    def __init__(self):
        super().__init__('explore_server')
        self.mode = self.declare_parameter('mode', 'virtual').value
        self.busy = False
        cb = ReentrantCallbackGroup()
        ctrl = f'/{ROBOT_ID}/dsr_controller2'
        self.cli = {key: self.create_client(srv, f'{ctrl}/{name}', callback_group=cb) for key, srv, name in (
            ('stop', MoveStop, 'motion/move_stop'),
            ('force', ReleaseForce, 'force/release_force'),
            ('compliance', ReleaseComplianceCtrl, 'force/release_compliance_ctrl'))}
        ActionServer(self, Process, '/process/explore', self.execute, callback_group=cb,
                     goal_callback=lambda _: GoalResponse.REJECT if self.busy else GoalResponse.ACCEPT,
                     cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.get_logger().info(f'/process/explore 준비 (mode={self.mode}). goal 전에는 움직이지 않습니다.')

    def execute(self, gh):
        self.busy = True
        try:
            return self._run(gh)
        finally:
            self.busy = False

    def _run(self, gh):
        run_id = gh.request.run_id
        out = os.path.join(SAVE_DIR, f'explore_run{run_id}_{datetime.now():%Y%m%d_%H%M%S}.json')
        worker = WorkerProcess(['ros2', 'run', 'rokey1', 'wall_tap', '--ros-args',
                                '-p', f'mode:={self.mode}', '-p', 'confirm:=false', '-p', 'rviz:=false',
                                '-p', f'out:={out}', '-p', f'tag:=run{run_id}'])
        limit = float(CFG['explore']['timeout_sec'])
        t0 = time.time()
        while worker.poll() is None:
            self._echo(worker)
            if gh.is_cancel_requested or time.time() - t0 > limit:
                self._abort(worker)
                if gh.is_cancel_requested:
                    gh.canceled()
                    return Process.Result(success=False, error_code='CANCELED', message='취소됨')
                gh.abort()
                return Process.Result(success=False, error_code='E_TIMEOUT', message=f'wall_tap {limit:.0f}s 초과')
            time.sleep(0.1)
        self._echo(worker)

        try:
            with open(out) as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            gh.abort()
            return Process.Result(success=False, error_code='E_EXPLORE', message=f'wall_tap 결과 없음: {e}')

        params = json.dumps({k: data['config'].get(k) for k in CFG.get('wall_tap') or {}})
        for name, sec in data['metrics']['phase_time'].items():
            gh.publish_feedback(Process.Feedback(state='RUNNING', step=f'explore_{name}', step_success=True,
                                                 duration_ms=int(sec * 1000), params_json=params))
        try:
            if data['status'] != 'ok':
                raise ValueError(data.get('error') or data['status'])
            wall, gaps = wall_corners(data), manual_gaps()
        except (ValueError, KeyError) as e:
            gh.abort()
            return Process.Result(success=False, error_code='E_EXPLORE', message=str(e))
        gh.succeed()
        r = data['results']
        return Process.Result(success=True, wall=wall, gaps=gaps,
                              message=f'높이 {r["height"]["median"]:.1f} / 너비 {r["width"]["median"]:.1f} mm, '
                                      f'틈 {len(gaps) // 6}개 ({os.path.basename(out)})')

    def _echo(self, worker):
        for line in worker.drain():
            self.get_logger().info(f'[wall_tap] {line}')

    def _abort(self, worker):
        """다음 명령 막기 → 즉시 정지 → 강제 종료(홈 복귀 안 함) → 힘/순응 해제."""
        worker.freeze()
        self._call('stop', MoveStop.Request(stop_mode=DR_QSTOP))
        worker.kill()
        self._call('force', ReleaseForce.Request(time=0.0))
        self._call('compliance', ReleaseComplianceCtrl.Request())

    def _call(self, key, req, timeout=2.0):
        cli = self.cli[key]
        if not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'{cli.srv_name} 서비스 없음')
            return
        fut = cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout:
            time.sleep(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = ExploreServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
