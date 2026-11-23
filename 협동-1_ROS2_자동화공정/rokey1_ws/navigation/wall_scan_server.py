#!/usr/bin/env python3
"""벽 내부 탐색 액션 서버 (팀 공통 Process 액션, task_code=EXPLORE).

액션: /process/inner_explore  (rokey_interfaces/action/Process) ← HMI [전체 공정] INNER_EXPLORE 단계
  Goal    : task_code='INNER_EXPLORE'(또는 'EXPLORE'), run_id, gap_id(-1),
            wall[12] = 외부 탐색 벽 4점 [위, 바닥, 오른쪽, 왼쪽] xyz → 탐색 테두리 (전부 0 이면 wall_scanner.CORNERS)
  Result  : success, error_code('' | CANCELED | E_SINGULAR | E_FORCE | E_TIMEOUT | E_NO_GAP),
            message, wall[12] = 테두리 [위, 바닥, 오른쪽, 왼쪽] 변 중점 xyz,
            gaps[] = 틈 1개당 [sx,sy,sz, ex,ey,ez]
  Feedback: 선(가로 줄/세로 열) 하나 끝날 때마다 1회
            step='explore_line', joint/task/force, params_json={"line","dir","index","done","total","points"}

탐색 설정은 이 노드의 ROS 파라미터 (HMI 는 Goal 직전에 set_parameters 로 바꿈):
  mode(real|virtual), dirs(both|rows|cols), row_step, col_step, margin, floor_detect,
  probe_length, tool_roll(공구 Z축 회전, 기본 90°), tcp_name, go_home(끝나면 홈 복귀),
  scan_method, scan_force, top_force_ratio(맨 위 줄 힘 = scan_force × 비율, 사이 줄은 높이에 비례)
탐지 점은 /dsr01/wall_scan/points 로 HMI 로봇 화면에 표시.

시작 자세: 빈 그리퍼(닫힘) 끝을 탐침으로 쓰고 홈에서 시작. real 은 홈이 아니면 움직이지 않음.
취소: 이동 중이면 즉시 정지 → 순응제어 해제 → 지금까지 점 분석·저장 → 벽에서 후퇴 (홈 복귀는 안 함)

실행:
  ros2 run rokey1 wall_scan_server --ros-args -p mode:=real
  HMI 벽면 탐색 탭 [벽 내부 탐색 서버] → [벽 내부 탐색 실행]
"""
import json
import threading
import time

import rclpy
import DR_init
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from rokey_interfaces.action import Process

from utils import config

try:
    from navigation import wall_scanner as ws
except ImportError:
    import os
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import wall_scanner as ws

ACTION_NAME = '/process/inner_explore'
TASK_CODES = ('INNER_EXPLORE', 'EXPLORE')


def error_code(message):
    """예외 메시지 → Process 결과 error_code."""
    text = message.lower()
    if '특이점' in message or '도달 불가' in message or '3205' in message:
        return 'E_SINGULAR'
    if '힘' in message or 'force' in text:
        return 'E_FORCE'
    if '시간 초과' in message or 'timeout' in text:
        return 'E_TIMEOUT'
    return 'E_FORCE'


def goal_corners(goal):
    """goal.wall [위, 바닥, 오른쪽, 왼쪽] xyz → 네 모서리 (wall_scanner.wall_edges 의 반대). 비었으면 고정값."""
    w = [float(v) for v in getattr(goal, 'wall', [])]
    if len(w) != 12 or not any(abs(v) > 1e-9 for v in w):
        return ws.CORNERS
    top, floor, right, left = w[0:3], w[3:6], w[6:9], w[9:12]
    if not (left[1] > right[1] and top[2] > floor[2]):
        raise ValueError(f'goal.wall 이 잘못됨 (왼쪽 Y > 오른쪽 Y, 위 Z > 바닥 Z 여야 함): {w}')
    return {'bottom_left': [left[0], left[1], floor[2]], 'top_left': [left[0], left[1], top[2]],
            'top_right': [right[0], right[1], top[2]], 'bottom_right': [right[0], right[1], floor[2]]}


class WallScanServer:
    def __init__(self):
        # 로봇 제어 노드: executor 에 넣지 않음 (DSR_ROBOT2 가 직접 spin) — 샌딩 서버와 같은 구조
        self.dsr_node = rclpy.create_node('wall_scan_dsr', namespace=ws.ROBOT_ID)
        # 클래스 안에서 DR_init.__dsr__node = ... 로 쓰면 파이썬이 이름을 _WallScanServer__dsr__node 로
        # 바꿔(name mangling) DSR_ROBOT2 가 노드를 못 받는다 → setattr 로 정확한 이름에 넣는다 (샌딩 서버와 동일)
        setattr(DR_init, '__dsr__id', ws.ROBOT_ID)
        setattr(DR_init, '__dsr__model', ws.ROBOT_MODEL)
        setattr(DR_init, '__dsr__node', self.dsr_node)
        import DSR_ROBOT2 as dsr        # 반드시 __dsr__node 설정 뒤
        self.dsr = dsr

        # 액션 노드
        self.node = rclpy.create_node('wall_scan_server', namespace=ws.ROBOT_ID)
        self.log = self.node.get_logger()
        n = self.node
        n.declare_parameter('mode', 'real')
        n.declare_parameter('dirs', 'both')
        n.declare_parameter('row_step', ws.ROW_STEP)
        n.declare_parameter('col_step', ws.ROW_STEP)
        n.declare_parameter('margin', ws.MARGIN)
        n.declare_parameter('floor_detect', True)
        n.declare_parameter('probe_length', ws.PROBE_LENGTH)
        n.declare_parameter('tool_roll', ws.TOOL_ROLL_DEG)
        n.declare_parameter('scan_method', ws.SCAN_METHOD)
        n.declare_parameter('scan_force', ws.SCAN_FORCE)
        n.declare_parameter('top_force_ratio', ws.TOP_FORCE_RATIO)
        n.declare_parameter('tcp_name', ws.TCP_NAME)
        n.declare_parameter('go_home', True)
        self.publisher = n.create_publisher(String, ws.SCAN_TOPIC, 10)
        self._gh = None
        self._busy = False
        self._lock = threading.Lock()
        self.action_server = ActionServer(
            n, Process, ACTION_NAME, execute_callback=self.execute,
            goal_callback=self.accept, cancel_callback=self.cancel,
            callback_group=ReentrantCallbackGroup())

    def param(self, name):
        return self.node.get_parameter(name).value

    def settings(self):
        values = {k: self.param(k) for k in ('mode', 'dirs', 'row_step', 'col_step', 'margin',
                                              'floor_detect', 'probe_length', 'tool_roll', 'tcp_name',
                                              'go_home', 'scan_method', 'scan_force', 'top_force_ratio')}
        if values['mode'] not in ('real', 'virtual') or values['dirs'] not in ('rows', 'cols', 'both'):
            raise ValueError(f"mode={values['mode']}, dirs={values['dirs']} 값이 잘못됐습니다.")
        for key in ('row_step', 'col_step', 'margin', 'probe_length', 'tool_roll', 'scan_force'):
            values[key] = float(values[key])
        # robot.yaml inner_explore 가 있으면 그 값이 우선 (goal 마다 파일을 새로 읽음 → 서버 재시작 없이 반영)
        #   vertical_scan: false → 가로 줄만 (dirs=rows, 상하 탐색 끔) / true → 가로+세로 (dirs=both)
        #   scan_force   : 누르는 힘 [N] (11-20 실물 성공 3.5, 기본 2.0 은 틈 신호가 약함)
        try:
            ie = config.load().get('inner_explore') or {}
        except Exception as exc:
            self.log.warn(f'robot.yaml 읽기 실패 → ROS 파라미터 사용: {exc}')
            ie = {}
        if 'vertical_scan' in ie:
            values['dirs'] = 'both' if ie['vertical_scan'] else 'rows'
        if ie.get('scan_force') is not None:
            values['scan_force'] = float(ie['scan_force'])
        # HMI 에서 칸을 비우면 0 이 온다 → 비었거나 0 이하면 1.0 (보정 없음)
        values['top_force_ratio'] = ws.top_force_ratio_or_default(values['top_force_ratio'])
        return values

    # ------------------------------------------------------------ 액션 콜백
    def accept(self, goal):
        if goal.task_code not in TASK_CODES:
            self.log.error(f'task_code={goal.task_code!r} (INNER_EXPLORE / EXPLORE 만 처리)')
            return GoalResponse.REJECT
        with self._lock:
            if self._busy:
                self.log.warn('이미 탐색 중 - 새 goal 거절')
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    def cancel(self, _handle):
        self.log.warn('취소 요청 - 이동 중이면 즉시 정지합니다.')
        return CancelResponse.ACCEPT

    def feedback(self, scanner, line, done, total, points, started):
        if self._gh is None:
            return
        fb = Process.Feedback()
        fb.state, fb.step, fb.step_success = 'RUNNING', 'explore_line', points > 0
        fb.error_code = '' if points > 0 else 'E_NO_GAP'
        fb.fail_reason = '' if points > 0 else f'{ws.line_name(line)} 벽에 닿지 못함'
        try:
            fb.joint = [float(v) for v in scanner.io.get_joints()]
            fb.task = [float(v) for v in scanner.io.get_pos()]+[float(v) for v in scanner.io.tool_ori]
            fb.force = [float(v) for v in scanner.io.get_force()]
        except Exception:
            pass
        fb.duration_ms = int((time.monotonic()-started)*1000)
        fb.params_json = json.dumps({'line': line['line'], 'dir': line['dir'], 'index': line['index'],
                                     'done': done, 'total': total, 'points': points}, ensure_ascii=False)
        fb.contact_ratio = -1.0
        self._gh.publish_feedback(fb)
        self.log.info(f'[피드백] {done}/{total} {ws.line_name(line)} {points}점')

    def execute(self, handle):
        self._gh = handle
        goal = handle.request
        result = Process.Result()
        started = time.monotonic()
        scanner = None
        lines = []
        corners = ws.CORNERS
        try:
            corners = goal_corners(goal)
            cfg = self.settings()
            self.log.info(f'[{goal.task_code} run_id={goal.run_id}] {cfg} | 테두리 '
                          f'{"고정값" if corners is ws.CORNERS else "외부 탐색 벽"} {corners}')
            io = ws.ScanIO(self.dsr_node, self.dsr, cfg['mode'], ws.TOOL_NAME, cfg['tcp_name'],
                           corners, cfg['probe_length'], cfg['tool_roll'])
            io.node = self.node     # 로그는 액션 노드로
            scanner = ws.WallScanner(
                io, corners, cfg['row_step'], cfg['col_step'], cfg['dirs'], cfg['margin'],
                cfg['floor_detect'], self.publisher,
                scan_method=cfg['scan_method'], scan_force=cfg['scan_force'],
                top_force_ratio=cfg['top_force_ratio'],
                is_canceled=lambda: handle.is_cancel_requested,
                on_line=lambda line, done, total, pts: self.feedback(scanner, line, done, total, pts, started))
            io.wait_for_services()
            io.setup()
            lines = scanner.run()
            result.success = True
        except ws.Canceled:
            result.success, result.error_code = False, 'CANCELED'
            result.message = '취소됨'
        except Exception as exc:
            result.success, result.message = False, str(exc)
            result.error_code = error_code(str(exc))
            self.log.error(f'탐색 오류: {exc}')
            if scanner is not None:
                scanner.io.stop()
        finally:
            analysis, path = None, None
            if scanner is not None:
                try:
                    scanner.io.release_press()
                except Exception as exc:
                    self.log.error(f'순응/힘 제어 해제 실패: {exc}')
                try:
                    analysis, path = scanner.finish(lines)
                except Exception as exc:
                    self.log.error(f'분석/저장 실패: {exc}')
                try:
                    # 출발 전 점검 실패 등으로 홈을 떠나지 않았으면 움직이지 않는다
                    if scanner.retreat() and result.success and self.param('go_home'):
                        scanner.io.move_j(ws.HOME_J)
                except Exception as exc:
                    self.log.error(f'후퇴/홈 복귀 실패: {exc}')
            result.wall = [float(v) for p in ws.wall_edges(corners) for v in p]
            # 여러 줄에서 확인된 세로 틈만 (매니저가 DB gap 테이블에 넣고 그 자리에 다음 작업을 실행하므로).
            # 걸림 추정(obstacle_suspects)은 확인되지 않은 위치라 JSON 에만 기록
            result.gaps = [float(v) for seg in ws.vertical_gap_lines(analysis) for v in seg]
            gaps = len(result.gaps)//6
            if result.success:
                result.message = (f'탐색 완료: 세로 틈 {gaps}곳 (걸림 추정 {len(scanner.obstacles)}곳은 기록만), '
                                  f'기록 {len(scanner.samples)}점'
                                  + (f', 저장 {path}' if path else ''))
            if handle.is_cancel_requested:
                handle.canceled()
            elif result.success:
                handle.succeed()
            else:
                handle.abort()
            self._gh = None
            with self._lock:
                self._busy = False
        # rclpy 는 같은 줄에서 로그 수준이 바뀌면 예외를 낸다(실패 뒤 성공 시 결과 전달 실패) → 두 줄로 나눔
        if result.success:
            self.log.info(f'결과: {result.message}')
        else:
            self.log.error(f'결과: {result.message}')
        return result

    def destroy(self):
        self.action_server.destroy()
        self.node.destroy_node()
        self.dsr_node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = WallScanServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(server.node)
    server.log.info(f'벽 내부 탐색 액션 서버: {ACTION_NAME} (Process, task_code=INNER_EXPLORE) | '
                    f'mode={server.param("mode")} | 탐지 점 /{ws.ROBOT_ID}/{ws.SCAN_TOPIC}')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
