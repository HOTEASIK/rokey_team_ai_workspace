"""퍼티 작업 액션 서버: /dsr01/putty_task (rokey_interfaces/action/PuttyTask).

서버 시작만으로 움직이지 않는다. Goal 한 개씩 기존 작업을 실행한다.
작업 노드를 격리해 동기 이동 서비스 대기 중에도 피드백/취소를 처리한다.
취소는 작업 프로세스 정지 → 로봇 정지 확인 → 작업 종료 → 재정지/힘 해제
확인 순서다. 확인에 실패하면 aborted로 반환하고 새 Goal을 차단한다.
"""

import math
import fcntl
import os
import signal
import threading
import time
import json
import re

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from dsr_msgs2.srv import (
    GetRobotSystem, GetRobotState, GetCurrentPosx, GetCurrentPosj, GetToolForce,
    MoveStop, ReleaseForce, ReleaseComplianceCtrl,
)
from rokey_interfaces.action import PuttyTask, Process

from utils.worker_process import WorkerProcess
from .action_protocol import worker_command, process_putty_goal, process_error_code
from utils.robot import ROBOT_ID  # config/robot.yaml robot 섹션


class ProcessTaskView:
    """팀 Goal을 기존 도포 실행기에 전달한다. DB 기록은 매니저만 담당한다.

    PUTTY는 현재 준비된 헤라로 세로 홈을 도포하는 putty_scrape 한 단계다.
    초기화/집기/스쿱/반납을 새로 자동 실행하지 않는다.
    """

    def __init__(self, handle, request):
        self.handle, self.request = handle, request
        self.status = None
        self.latest = None
        self.fx = []
        self.worker_started = False

    @property
    def is_cancel_requested(self):
        return self.handle.is_cancel_requested

    def publish_feedback(self, feedback):
        self.latest = feedback  # 실시간 내부 피드백. DB용 완료 feedback과 구분한다.

    def observe_line(self, line):
        if '단계=상향 도포 중' not in line:
            return
        match = re.search(r'실측BASE_Fxyz=\[([^\]]+)\]', line)
        if match:
            try:
                value = float(match[1].split(',')[0])
                if math.isfinite(value):
                    self.fx.append(value)
            except ValueError:
                pass

    def succeed(self):
        self.status = 'succeeded'

    def abort(self):
        self.status = 'aborted'

    def canceled(self):
        self.status = 'canceled'


class PuttyTaskServer(Node):
    def __init__(self):
        super().__init__('putty_task_server', namespace=ROBOT_ID)
        # HMI/launch에서 같은 서버를 중복 실행해 두 Goal이 동시에 움직이지 않게 한다.
        # 런타임 잠금이며 사용자 환경 설정 파일은 아니다.
        self.server_lock = open(f'/tmp/putty_action_{os.getuid()}_{ROBOT_ID}_'
                                f'{self.context.get_domain_id()}.lock', 'a')
        try:
            fcntl.flock(self.server_lock.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.server_lock.close()
            self.destroy_node()
            raise RuntimeError('이미 이 PC/ROS domain에서 퍼티 액션 서버가 실행 중입니다.') from exc
        self.group = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.busy = False
        self.finishing = False
        self.faulted = False
        self.spatula_held = False   # /process/putty_pickup 성공 ~ putty_return 성공 사이 = 헤라를 쥐고 있음
        self.scoop_pass = None      # 다음 뜨기 회차. 집기 성공 때 robot.yaml scoop_pass 로, 뜨기 성공마다 +1 (4 다음 1)
        self.cancel_event = threading.Event()
        self.shutdown_event = threading.Event()
        self.prefix = f'/{ROBOT_ID}/dsr_controller2'
        self.process_virtual_mode = bool(self.declare_parameter('process_virtual_mode', False).value)
        self.control_clients = {
            'system': self.create_client(GetRobotSystem, f'{self.prefix}/system/get_robot_system'),
            'state': self.create_client(GetRobotState, f'{self.prefix}/system/get_robot_state'),
            'pose': self.create_client(GetCurrentPosx, f'{self.prefix}/aux_control/get_current_posx'),
            'joints': self.create_client(GetCurrentPosj, f'{self.prefix}/aux_control/get_current_posj'),
            'force': self.create_client(GetToolForce, f'{self.prefix}/aux_control/get_tool_force'),
            'stop': self.create_client(MoveStop, f'{self.prefix}/motion/move_stop'),
            'release': self.create_client(ReleaseForce, f'{self.prefix}/force/release_force'),
            'release_comp': self.create_client(ReleaseComplianceCtrl,
                                                f'{self.prefix}/force/release_compliance_ctrl'),
        }
        self.action = ActionServer(
            self, PuttyTask, 'putty_task', execute_callback=self.execute,
            goal_callback=self.accept_goal, cancel_callback=self.cancel_goal,
            callback_group=self.group,
        )
        self.process_action = ActionServer(
            self, Process, '/process/putty', execute_callback=self.execute_process,
            goal_callback=self.accept_process_goal, cancel_callback=self.cancel_goal,
            callback_group=self.group,
        )

        # 전체 공정 퍼티 준비/정리: /process/putty_pickup, /process/putty_scoop, /process/putty_return
        self.process_step_actions = [ActionServer(
            self, Process, f'/process/{code.lower()}', execute_callback=self.execute_process,
            goal_callback=lambda request, code=code: (self.reserve_goal() if request.task_code == code
                                                      else GoalResponse.REJECT),
            cancel_callback=self.cancel_goal, callback_group=self.group)
            for code in ('PUTTY_PICKUP', 'PUTTY_SCOOP', 'PUTTY_RETURN')]

    def accept_goal(self, goal):
        try:
            worker_command(goal)
        except (ValueError, TypeError) as exc:
            self.get_logger().error(f'Goal 거절: {exc}')
            return GoalResponse.REJECT
        return self.reserve_goal()

    def reserve_goal(self):
        with self.lock:
            if self.busy or self.faulted or self.shutdown_event.is_set():
                self.get_logger().warning('실행 중/종료 중 또는 정리 실패 상태이므로 Goal 거절')
                return GoalResponse.REJECT
            self.busy, self.finishing = True, False
            self.cancel_event.clear()
        return GoalResponse.ACCEPT

    def accept_process_goal(self, request):
        if request.task_code != 'PUTTY':
            return GoalResponse.REJECT
        # 좌표/설정 오류는 Result로 반환해 매니저가 DB에 원인을 기록하게 한다.
        return self.reserve_goal()

    def cancel_goal(self, handle):
        with self.lock:
            if not self.busy or self.finishing:
                return CancelResponse.REJECT
            self.cancel_event.set()
        self.get_logger().warning('액션 취소 수신: 이동 중에도 정지 절차를 시작합니다.')
        return CancelResponse.ACCEPT

    def check_cancel(self):
        if self.cancel_event.is_set() or self.shutdown_event.is_set():
            raise InterruptedError('작업 취소 또는 서버 종료 요청')

    def call(self, name, request, timeout=5.0, cancellable=True):
        """Action executor의 다른 스레드가 서비스 응답을 처리한다. 중첩 spin은 하지 않는다."""
        client = self.control_clients[name]
        deadline = time.monotonic()+timeout
        while not client.service_is_ready():
            if cancellable:
                self.check_cancel()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{name} 서비스 없음')
            time.sleep(0.02)
        if cancellable:
            self.check_cancel()
        future = client.call_async(request)
        try:
            while not future.done():
                if cancellable:
                    self.check_cancel()
                if time.monotonic() >= deadline:
                    raise TimeoutError(f'{name} 서비스 응답 시간 초과')
                time.sleep(0.02)
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError(f'{name} 서비스 실패')
            return result
        finally:
            if not future.done():
                future.cancel()

    def stop_robot(self):
        request = MoveStop.Request()
        request.stop_mode = 3  # 기존 DR_HOLD 유지
        self.call('stop', request, timeout=2, cancellable=False)
        deadline = time.monotonic()+5
        while time.monotonic() < deadline:
            state = self.call('state', GetRobotState.Request(), timeout=0.5,
                              cancellable=False).robot_state
            if state == 1:
                return
            if state != 2:
                raise RuntimeError(f'정지 확인 실패: robot_state={state}')
            time.sleep(0.05)
        raise TimeoutError('STANDBY 정지 확인 시간 초과')

    def cleanup_worker(self, worker, task):
        """작업 종료만을 로봇 정지로 간주하지 않는다. 실패도 전부 모아 결과에 남긴다."""
        errors = []
        operations = [('작업 일시 정지', worker.freeze), ('로봇 정지', self.stop_robot),
                      ('작업 프로세스 종료', worker.terminate), ('종료 후 로봇 정지 재확인', self.stop_robot)]
        # 힘제어나 컴플라이언스를 쓰는 작업. return은 실물 바닥 탐색, scoop은 실물 벽 긁기 중 켠다.
        if task in ('apply', 'return', 'scoop'):
            request = ReleaseForce.Request()
            request.time = 0.3
            operations += [
                ('목표 힘 해제', lambda: self.call('release', request, timeout=2, cancellable=False)),
                ('컴플라이언스 해제', lambda: self.call('release_comp', ReleaseComplianceCtrl.Request(),
                                                      timeout=2, cancellable=False)),
                ('힘 해제 후 정지 확인', self.stop_robot),
            ]
        for label, operation in operations:
            try:
                operation()
            except Exception as exc:
                errors.append(f'{label}: {exc}')
                self.get_logger().error(errors[-1])
        if errors:
            self.faulted = True
            raise RuntimeError('정지/정리 확인 실패. 새 Goal 차단: '+'; '.join(errors))

    def feedback(self, handle, stage, started, pending, samples, log_line='', progress=-1.0):
        # 피드백 전송 실패가 정지/힘 해제 절차를 건너뛰게 해서는 안 된다.
        try:
            self._feedback(handle, stage, started, pending, samples, log_line, progress)
        except Exception as exc:
            self.get_logger().warning(f'액션 피드백 조회/전송 실패: {exc}')

    def _feedback(self, handle, stage, started, pending, samples, log_line='', progress=-1.0):
        """이동 서비스와 독립적으로 위치/힘을 비동기 조회한다. 오래된 값은 valid=false."""
        now = time.monotonic()
        for key, typ in (('pose', GetCurrentPosx), ('force', GetToolForce)):
            entry = pending.get(key)
            if entry:
                future, sent = entry
                if future.done():
                    try:
                        response = future.result()
                        if response is None or not response.success:
                            raise ValueError('조회 실패')
                        values = (response.task_pos_info[0].data[:6] if key == 'pose'
                                  else response.tool_force)
                        if len(values) != 6 or not all(map(math.isfinite, values)):
                            raise ValueError('유효하지 않은 센서값')
                        samples[key] = (list(values), now)
                    except Exception:
                        samples.pop(key, None)
                    pending.pop(key, None)
                elif now-sent > 0.5:
                    future.cancel()
                    pending.pop(key, None)
                    samples.pop(key, None)
            if key not in pending and self.control_clients[key].service_is_ready():
                request = typ.Request()
                request.ref = 0
                pending[key] = (self.control_clients[key].call_async(request), now)
        fb = PuttyTask.Feedback()
        fb.task, fb.stage, fb.elapsed_sec = handle.request.task, stage, now-started
        fb.progress, fb.log_line = float(progress), log_line
        for key, valid, data in (('pose', 'pose_valid', 'tcp_pose'),
                                 ('force', 'force_valid', 'tool_force')):
            entry = samples.get(key)
            fresh = entry is not None and now-entry[1] <= 0.5
            setattr(fb, valid, fresh)
            if fresh:
                setattr(fb, data, entry[0])
        handle.publish_feedback(fb)

    def execute(self, handle, release_slot=True):
        worker = None
        pending, samples = {}, {}
        started = time.monotonic()
        result = PuttyTask.Result()
        result.exit_code = -1
        stage, last_line = '컨트롤러 확인', ''
        try:
            command = worker_command(handle.request)
            self.feedback(handle, stage, started, pending, samples)
            system = self.call('system', GetRobotSystem.Request()).robot_system
            if system != int(handle.request.virtual_mode):
                raise RuntimeError('Goal의 real/virtual 모드와 연결된 컨트롤러가 다릅니다.')
            if self.call('state', GetRobotState.Request()).robot_state != 1:
                raise RuntimeError('로봇이 STANDBY가 아니므로 작업을 시작하지 않습니다.')
            self.check_cancel()
            worker = WorkerProcess(command, grasp_confirmed=handle.request.tool_held_confirmed)
            if isinstance(handle, ProcessTaskView):
                handle.worker_started = True
            next_feedback = 0.0
            while True:
                self.check_cancel()
                for line in worker.drain():
                    if isinstance(handle, ProcessTaskView):
                        handle.observe_line(line)
                    last_line = line
                    self.get_logger().info(f'[{handle.request.task}] {line}')
                    stage = line.split('단계=', 1)[1].split(' |', 1)[0] if '단계=' in line else line
                if time.monotonic() >= next_feedback:
                    self.feedback(handle, stage, started, pending, samples, last_line)
                    next_feedback = time.monotonic()+0.2
                exit_code = worker.poll()
                if exit_code is not None:
                    worker.reader.join(timeout=1)
                    for line in worker.drain():
                        if isinstance(handle, ProcessTaskView):
                            handle.observe_line(line)
                        last_line = line
                        self.get_logger().info(f'[{handle.request.task}] {line}')
                    result.exit_code = exit_code
                    if exit_code != 0:
                        raise RuntimeError(f'작업 실패(exit={exit_code}): {last_line}')
                    break
                time.sleep(0.02)
            self.check_cancel()
            if self.call('state', GetRobotState.Request()).robot_state != 1:
                raise RuntimeError('작업 종료 후 로봇 정지를 확인하지 못했습니다.')
            with self.lock:
                self.check_cancel()
                self.finishing = True
            self.feedback(handle, '완료', started, pending, samples, last_line, progress=1.0)
            result.success, result.stop_confirmed = True, True
            result.message = f'{handle.request.task} 완료'
            handle.succeed()
        except Exception as exc:
            canceled = self.cancel_event.is_set() or handle.is_cancel_requested
            message = str(exc)
            cleanup_ok = True
            if worker is not None:
                self.feedback(handle, '정지·힘 해제 확인 중', started, pending, samples, message)
                try:
                    self.cleanup_worker(worker, handle.request.task)
                    result.stop_confirmed = True
                    result.exit_code = worker.poll()
                except Exception as cleanup_exc:
                    cleanup_ok = False
                    message += f' | {cleanup_exc}'
            result.success = False
            result.message = ('취소 완료: ' if canceled and cleanup_ok else '작업 중단: ')+message
            with self.lock:
                self.finishing = True
            if canceled and cleanup_ok and handle.is_cancel_requested:
                handle.canceled()
            else:
                handle.abort()
            self.get_logger().warning(result.message)
        finally:
            for future, _ in pending.values():
                if not future.done():
                    future.cancel()
            if release_slot:
                with self.lock:
                    self.busy = False
        return result

    def process_snapshot(self):
        """완료/정지 후 조회. 누락 데이터를 0으로 꾸미지 않고 NaN으로 표시한다."""
        result = {}
        for key, typ in (('joints', GetCurrentPosj), ('pose', GetCurrentPosx), ('force', GetToolForce)):
            try:
                request = typ.Request()
                if key != 'joints':
                    request.ref = 0
                response = self.call(key, request, timeout=0.5, cancellable=False)
                values = (response.pos if key == 'joints' else response.task_pos_info[0].data[:6]
                          if key == 'pose' else response.tool_force)
                if len(values) != 6 or not all(map(math.isfinite, values)):
                    raise ValueError('센서값 없음')
                result[key] = [float(v) for v in values]
            except Exception:
                result[key] = [float('nan')] * 6
        return result

    def execute_process(self, handle):
        """공통 Process 결과/완료 피드백. 기존 실행기의 중간 취소·정리 절차를 재사용한다."""
        started = time.monotonic()
        code = handle.request.task_code
        is_apply = code == 'PUTTY'
        step = 'putty_scrape' if is_apply else code.lower()     # task_log.step (= robot.yaml 키 이름)
        axis = 'fx'
        result = Process.Result()
        view = None
        settings = None
        inner = None
        try:
            from utils.config import CFG
            settings = CFG
            request = process_putty_goal(handle.request, CFG, self.process_virtual_mode, held=self.spatula_held,
                                         scoop_pass=getattr(self, 'scoop_pass', None))
            view = ProcessTaskView(handle, request)
            scoop_label = ''
            if code == 'PUTTY_SCOOP':
                from .scoop_putty import SCOOP_PASS_STEP_MM
                n = getattr(self, 'scoop_pass', None) or int(CFG['force']['putty_scoop']['scoop_pass'])
                step_mm = float(CFG['force']['putty_scoop'].get('scoop_pass_step', SCOOP_PASS_STEP_MM))
                scoop_label = f'{n}회차 뜨기 (끝점 +X {(n-1)*step_mm:g} mm)'
                self.get_logger().info(f'[scoop] {scoop_label} 시작')
            inner = self.execute(view, release_slot=False)
            result.success = inner.success
            result.message = f'[{scoop_label}] {inner.message}' if scoop_label else inner.message
            if inner.success and code in ('PUTTY_PICKUP', 'PUTTY_RETURN'):
                self.spatula_held = code == 'PUTTY_PICKUP'
                self.scoop_pass = int(CFG['force']['putty_scoop']['scoop_pass']) if code == 'PUTTY_PICKUP' else None
            if inner.success and code == 'PUTTY_SCOOP':
                # 다음 뜨기는 몸체 쪽(+X)으로 한 칸. 용기 끝(SCOOP_MAX_PASS)을 넘으면 다시 1회차
                from .scoop_putty import SCOOP_MAX_PASS
                done = getattr(self, 'scoop_pass', None) or int(CFG['force']['putty_scoop']['scoop_pass'])
                self.scoop_pass = done % SCOOP_MAX_PASS + 1
            result.error_code = ('' if inner.success else 'CANCELED' if view.status == 'canceled'
                                 else process_error_code(inner.message))
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            result.error_code = 'CANCELED' if handle.is_cancel_requested else process_error_code(str(exc))
        try:
            from utils.force_stats import summarize
            sample = self.process_snapshot()
            cfg = ((settings['force'].get(step) or settings['putty'].get(step.replace('putty_', '')) or {})
                   if settings else {})   # putty_scrape / putty_scoop → force, putty_pickup / putty_return → putty
            params = dict(cfg)
            params.update(run_id=int(handle.request.run_id), gap_id=int(handle.request.gap_id),
                          start_point=list(handle.request.start_point), end_point=list(handle.request.end_point),
                          worker_started=bool(view and view.worker_started),
                          sample_phase='after_work_or_stop',
                          joint_valid=all(map(math.isfinite, sample['joints'])),
                          task_valid=all(map(math.isfinite, sample['pose'])),
                          force_valid=all(map(math.isfinite, sample['force'])))
            trace = view.fx if view else []
            target = float(cfg['target'][0]) if is_apply else 0.0   # 힘 목표는 도포만
            tolerance = float(cfg.get('tol') or 0) if is_apply else 0.0
            _, stats = summarize(trace, target, tolerance, axis=axis)
            ratio = -1.0
            if trace and is_apply:
                ratio = sum(abs(abs(f)-target) <= tolerance for f in trace)/len(trace)
                data = json.loads(stats)
                data.update(contact_ratio_metric='abs_base_'+axis, target=target, tolerance=tolerance)
                stats = json.dumps(data, allow_nan=False)
            params.update(force_axis=axis, target_force=target,
                          vel=float(cfg.get('apply_speed' if is_apply else 'transfer_speed') or 0))
            params['target_'+axis] = target
            feedback = Process.Feedback()
            feedback.state = 'RUNNING' if result.success else 'PAUSED' if result.error_code == 'CANCELED' else 'ERROR'
            feedback.step, feedback.step_success = step, result.success
            feedback.error_code, feedback.fail_reason = result.error_code, '' if result.success else result.message
            feedback.joint, feedback.task, feedback.force = sample['joints'], sample['pose'], sample['force']
            feedback.duration_ms = int((time.monotonic()-started)*1000)
            feedback.params_json = json.dumps(params, ensure_ascii=False, allow_nan=False)
            feedback.contact_ratio, feedback.force_stats_json = float(ratio), stats
            handle.publish_feedback(feedback)
        except Exception as exc:
            self.get_logger().error(f'Process 완료 피드백 전송 실패: {exc}')
            result.success, result.error_code = False, 'E_TIMEOUT'
            result.message += f' | 완료 피드백 실패: {exc}'
        finally:
            with self.lock:
                self.finishing = True
            try:
                if result.success:
                    handle.succeed()
                elif result.error_code == 'CANCELED' and handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
            finally:
                with self.lock:
                    self.busy = False
        return result


def main(args=None):
    # SIGINT 후에도 정지 서비스 응답을 받을 수 있도록 ROS context를 먼저 종료하지 않는다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = PuttyTaskServer()
    except Exception:
        rclpy.shutdown()
        raise
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    previous = {}
    def request_shutdown(signum, frame):
        node.shutdown_event.set()
        node.cancel_event.set()
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.signal(signum, request_shutdown)
    node.get_logger().info('퍼티 액션 준비: /dsr01/putty_task, /process/putty_pickup · putty_scoop · putty · '
                           'putty_return. Goal 전에는 움직이지 않습니다.')
    try:
        while not node.shutdown_event.is_set() or node.busy:
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.shutdown()
        node.action.destroy()
        node.process_action.destroy()
        for action in node.process_step_actions:
            action.destroy()
        node.destroy_node()
        node.server_lock.close()
        rclpy.shutdown()
        for signum, handler in previous.items():
            signal.signal(signum, handler)


if __name__ == '__main__':
    main()
