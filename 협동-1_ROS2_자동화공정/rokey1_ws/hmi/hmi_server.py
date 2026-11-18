#!/usr/bin/env python3
"""rokey1 웹 HMI 서버 노드.

웹 브라우저(HMI 화면)  ⇄  HTTP / SSE  ⇄  hmi_server(ROS2 노드)  ⇄  rokey1 작업 노드 / DSR 컨트롤러

[하는 일]
  1) 정적 웹 파일(index.html, main.js, styles.css) 제공
  2) 작업 실행/정지 : navigation / putty_demo / sanding 노드를 `ros2 run rokey1 ...` 로 띄우고
                      출력(로그)을 실시간으로 화면에 전달. y/Enter 같은 입력(stdin)도 전달.
  3) sanding goal   : /dsr01/tool_task 액션에 직접 goal 을 보내고 feedback(step/stage) 을 진행률로 표시
  3-1) 공정(DB)     : [공정] 탭 작업(kind=db)은 /process/* 액션을 직접 호출하고 결과를 MySQL 에 기록
                      (hmi/process_runner.py, DB 에 쓰는 곳은 hmi_server 하나뿐)
  4) 로봇 상태      : DSR 서비스(get_robot_state / get_current_posx / get_tool_force) 를 주기적으로 조회,
                      /dsr01/joint_states, /dsr01/error 구독
  5) 비상 정지      : move_stop(DR_QSTOP) → 실행 중인 작업 프로세스 강제 종료(SIGKILL, 자동 홈 복귀 방지)
                      → 힘/컴플라이언스 제어 해제. RESET 전까지 새 작업 시작 금지.
     ※ 소프트웨어 정지일 뿐 하드웨어 비상정지 버튼을 대신하지 않는다.

[실행]
  ros2 run rokey1 hmi_server                                  # http://127.0.0.1:8080
  ros2 run rokey1 hmi_server --ros-args -p port:=8081 -p host:=0.0.0.0   # 다른 PC 에서 접속 허용

[API]
  GET  /api/tasks                         작업 카탈로그
  GET  /api/state                         상태 스냅샷
  GET  /api/events                        SSE (state / log / run 이벤트)
  POST /api/start   {task_id, mode, params}
  POST /api/stop    {run_id?}             run_id 없으면 전부 정지
  POST /api/stdin   {run_id, text}
  POST /api/estop   /   POST /api/estop/reset
  POST /api/speed   {pct, unlocked}       속도 % (기본 100, 안전 해제 시 최대 200)

[속도 제어]
  ≤100% : 컨트롤러 속도 오버라이드(motion/change_operation_speed) → 실행 중인 동작에도 바로 적용
  >100% : '안전 해제' 상태에서만 허용. 새로 시작하는 작업의 속도 파라미터(scale 항목)를 배율만큼 키움
          (컨트롤러 오버라이드는 100% 가 최대). 비상 정지 시 자동으로 100% · 잠금으로 복귀.
"""
import itertools
import json
import mimetypes
import os
import queue
import re
import select
import signal
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from hmi.tasks import MODES, TASK_MAP, apply_speed, catalog, normalize_params, parse_posx

try:
    from dsr_msgs2.msg import RobotError
    from dsr_msgs2.srv import (ChangeOperationSpeed, GetCurrentPosx, GetRobotState, GetToolForce,
                               MoveStop, ReleaseComplianceCtrl, ReleaseForce)
    HAVE_DSR = True
except ImportError:
    HAVE_DSR = False

try:
    from rclpy.action import ActionClient
    from rokey_interfaces.action import ToolTask
    HAVE_TOOL_TASK = True
except ImportError:
    HAVE_TOOL_TASK = False

try:
    from hmi.process_runner import ProcessRunner
except ImportError as e:      # pymysql / rokey_interfaces 없음
    ProcessRunner = None
    PROCESS_IMPORT_ERROR = str(e)

try:
    from onrobot_rg_msgs.srv import SetCommand
    HAVE_ONROBOT = True
except ImportError:
    HAVE_ONROBOT = False


DR_QSTOP = 1
ROBOT_STATES = {
    0: 'INITIALIZING', 1: 'STANDBY', 2: 'MOVING', 3: 'SAFE_OFF', 4: 'TEACHING',
    5: 'SAFE_STOP', 6: 'EMERGENCY_STOP', 7: 'HOMING', 8: 'RECOVERY', 9: 'SAFE_STOP2',
    10: 'SAFE_OFF2', 15: 'NOT_READY',
}
BAD_STATES = {3, 5, 6, 9, 10}
LOG_LEVEL_RE = re.compile(r'\[(DEBUG|INFO|WARN|WARNING|ERROR|FATAL)\]')
ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')
STOP_ESCALATE = ((8.0, signal.SIGTERM), (15.0, signal.SIGKILL))
SPEED_MIN, SPEED_DEFAULT, SPEED_SAFE_MAX, SPEED_UNLOCKED_MAX = 10, 100, 100, 200
STATE_PERIOD = 0.2   # 화면으로 로봇 상태를 보내는 주기 [s] (5 Hz)


def now():
    return time.time()


# ============================================================================ 이벤트 허브
class EventHub:
    """SSE 구독자에게 이벤트를 뿌리고, 최근 로그를 보관한다."""

    def __init__(self, history=400):
        self.lock = threading.Lock()
        self.subscribers = []
        self.logs = deque(maxlen=history)

    def subscribe(self):
        q = queue.Queue(maxsize=2000)
        with self.lock:
            self.subscribers.append(q)
            backlog = list(self.logs)
        return q, backlog

    def unsubscribe(self, q):
        with self.lock:
            if q in self.subscribers:
                self.subscribers.remove(q)

    def publish(self, event, data):
        msg = {'event': event, 'data': data}
        with self.lock:
            if event == 'log':
                self.logs.append(msg)
            subs = list(self.subscribers)
        for q in subs:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass

    def log(self, level, source, message, state=None, run_id=None, raw=None, popup=None):
        """popup=False : 작업자가 직접 누른 동작의 결과처럼 화면 팝업이 필요 없는 경고."""
        level = {'WARNING': 'WARN', 'FATAL': 'ERROR', 'DEBUG': 'INFO'}.get(level, level)
        if state is None:
            state = {'ERROR': 'FAULT', 'WARN': 'WATCH'}.get(level, 'OK')
        self.publish('log', {'t': now(), 'type': level, 'source': source,
                             'message': message, 'state': state, 'run_id': run_id,
                             'raw': raw if raw is not None else message,
                             'popup': (level in ('WARN', 'ERROR') and state != 'INPUT') if popup is None else popup})


# ============================================================================ 작업 실행
class Run:
    _ids = itertools.count(1)

    def __init__(self, task, mode, params, motion):
        self.id = next(Run._ids)
        self.task = task
        self.mode = mode
        self.params = params
        self.motion = motion
        self.state = 'starting'          # starting | running | done | failed | stopped | killed
        self.started = now()
        self.ended = None
        self.returncode = None
        self.step = 0
        self.total = 0
        self.stage = ''
        self.command = ''
        self.proc = None
        self.goal_handle = None
        self.stop_requested = False
        self.awaiting_input = False
        self.lock = threading.Lock()

    @property
    def active(self):
        return self.state in ('starting', 'running')

    def to_json(self):
        end = self.ended or now()
        return {
            'run_id': self.id, 'task_id': self.task['id'], 'label': self.task['label'],
            'group': self.task['group'], 'kind': self.task['kind'], 'mode': self.mode,
            'state': self.state, 'started': self.started, 'elapsed': round(end - self.started, 1),
            'returncode': self.returncode, 'step': self.step, 'total': self.total,
            'stage': self.stage, 'command': self.command, 'motion': self.motion,
            'pid': self.proc.pid if self.proc else None, 'awaiting_input': self.awaiting_input,
        }


class TaskManager:
    def __init__(self, hub, ros):
        self.hub = hub
        self.ros = ros
        self.lock = threading.Lock()
        self.runs = []                 # 최근 실행 목록 (끝난 것 포함)
        self.estop = False
        self.speed_pct = SPEED_DEFAULT
        self.speed_unlocked = False
        self.runner = None

    # ------------------------------------------------------------------ 속도
    def speed_json(self):
        return {'pct': self.speed_pct, 'unlocked': self.speed_unlocked,
                'max': SPEED_UNLOCKED_MAX if self.speed_unlocked else SPEED_SAFE_MAX,
                'min': SPEED_MIN, 'default': SPEED_DEFAULT}

    def set_speed(self, pct, unlocked):
        unlocked = bool(unlocked)
        if unlocked and self.estop:
            raise RuntimeError('비상 정지 상태에서는 속도 안전 해제를 할 수 없습니다.')
        pct = int(round(float(pct)))
        limit = SPEED_UNLOCKED_MAX if unlocked else SPEED_SAFE_MAX
        if not SPEED_MIN <= pct <= limit:
            raise ValueError(f'속도는 {SPEED_MIN}~{limit}% 사이여야 합니다'
                             + ('' if unlocked else ' (100% 초과는 안전 해제 필요)'))
        changed_lock = unlocked != self.speed_unlocked
        self.speed_pct, self.speed_unlocked = pct, unlocked
        if changed_lock:
            self.hub.log('WARN' if unlocked else 'INFO', 'SPEED',
                         '속도 안전 해제 · 최대 200%' if unlocked else '속도 안전 잠금 · 최대 100%',
                         state='WATCH' if unlocked else 'OK', popup=False)
        self.hub.log('WARN' if pct > SPEED_SAFE_MAX else 'INFO', 'SPEED', f'속도 설정 {pct}%', popup=False)
        self.ros.set_operation_speed(min(pct, SPEED_SAFE_MAX))
        self.hub.publish('speed', self.speed_json())

    # ------------------------------------------------------------------ 조회
    def active_runs(self):
        with self.lock:
            return [r for r in self.runs if r.active]

    def get(self, run_id):
        with self.lock:
            for r in self.runs:
                if r.id == run_id:
                    return r
        return None

    def snapshot(self):
        with self.lock:
            return [r.to_json() for r in self.runs[-20:]]

    # ------------------------------------------------------------------ 시작
    def start(self, task_id, mode, raw_params):
        task = TASK_MAP.get(task_id)
        if task is None:
            raise ValueError(f'알 수 없는 작업: {task_id}')
        if mode not in MODES:
            raise ValueError(f'mode 는 {MODES} 중 하나')
        if self.estop:
            raise RuntimeError('비상 정지 상태입니다. 비상 정지 해제 후 시작하세요.')
        params = apply_speed(task, normalize_params(task, raw_params), self.speed_pct)
        motion = task['motion'](mode)

        with self.lock:
            for r in self.runs:
                if not r.active:
                    continue
                if r.task['id'] == task_id:
                    raise RuntimeError(f'{task["label"]} 은(는) 이미 실행 중입니다 (실행 #{r.id}).')
                if motion and r.motion:
                    raise RuntimeError(f'로봇 동작 작업이 이미 실행 중입니다: {r.task["label"]} (실행 #{r.id}). '
                                       '먼저 정지하세요.')
            run = Run(task, mode, params, motion)
            self.runs.append(run)
            self.runs = self.runs[-40:]

        if task['kind'] == 'action':
            self._start_action(run)
        elif task['kind'] == 'db':
            if self.runner is None:
                with self.lock:
                    self.runs.remove(run)
                raise RuntimeError(f'DB 공정 실행 불가: {PROCESS_IMPORT_ERROR}')
            threading.Thread(target=self._run_db, args=(run,), daemon=True).start()
        else:
            commands = task['build'](mode, params)
            threading.Thread(target=self._run_commands, args=(run, commands), daemon=True).start()
        if motion:
            self.ros.set_operation_speed(min(self.speed_pct, SPEED_SAFE_MAX))
        self.hub.log('INFO', 'HMI', f'시작: {task["label"]} · 모드={"가상" if mode == "virtual" else "실제 로봇"}'
                                    f' · 속도 {self.speed_pct}% · 실행 #{run.id}', run_id=run.id)
        self._changed()
        return run

    def _start_action(self, run):
        try:
            self.ros.send_tool_task(run)
        except Exception as e:
            with self.lock:
                if run in self.runs:
                    self.runs.remove(run)
            raise RuntimeError(str(e)) from e

    def _changed(self):
        self.hub.publish('runs', self.snapshot())

    def _run_db(self, run):
        run.state = 'running'
        self._changed()
        try:
            result = self.runner.run(run, lambda: self.estop)
        except Exception as e:
            self.hub.log('ERROR', 'DB', f'공정 실행 오류: {e}', run_id=run.id)
            result = 'E_DB'
        if result == 'OK':
            self._finish(run, 'done')
        elif run.stop_requested:
            self._finish(run, 'killed' if self.estop else 'stopped')
        else:
            self._finish(run, 'failed')

    def _finish(self, run, state, returncode=None):
        with run.lock:
            if not run.active:
                return
            run.state = state
            run.returncode = returncode
            run.ended = now()
            run.awaiting_input = False
        level = 'INFO' if state in ('done', 'stopped') else 'ERROR' if state == 'failed' else 'WARN'
        rc = f' · 종료코드 {returncode}' if returncode is not None else ''
        state_ko = {'done': '완료', 'failed': '실패', 'stopped': '정지', 'killed': '강제 종료'}.get(state, state)
        self.hub.log(level, 'HMI', f'{run.task["label"]} {state_ko}{rc} · 실행 #{run.id}',
                     state=state.upper()[:7], run_id=run.id)
        self._changed()

    # ------------------------------------------------------------------ 프로세스
    def _run_commands(self, run, commands):
        total = len(commands)
        run.total = total if total > 1 else 0
        for i, argv in enumerate(commands, 1):
            if run.stop_requested:
                break
            run.step = i if total > 1 else 0
            run.command = ' '.join(argv)
            run.stage = argv[3] if len(argv) > 3 else argv[0]
            rc = self._spawn_and_wait(run, argv)
            if run.stop_requested:
                break
            if rc != 0:
                self._finish(run, 'failed', rc)
                return
        if run.stop_requested:
            self._finish(run, 'killed' if self.estop else 'stopped', run.returncode)
        else:
            self._finish(run, 'done', 0)

    def _spawn_and_wait(self, run, argv):
        env = dict(os.environ, PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0',
                   RCUTILS_LOGGING_BUFFERED_STREAM='0')
        source = run.task['label']
        self.hub.log('INFO', source, '$ ' + ' '.join(argv), state='EXEC', run_id=run.id)
        try:
            proc = subprocess.Popen(argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, env=env, start_new_session=True)
        except FileNotFoundError as e:
            self.hub.log('ERROR', source, f'실행 실패: {e} (ROS2 환경을 source 했는지 확인)', run_id=run.id)
            return 127
        with run.lock:
            run.proc = proc
            run.state = 'running'
        self._changed()
        self._pump_output(run, proc, source)
        rc = proc.wait()
        run.returncode = rc
        with run.lock:
            run.proc = None
        return rc

    def _pump_output(self, run, proc, source):
        """stdout 을 줄 단위로 로그로 보낸다. 줄바꿈 없이 멈춘 출력(input() 프롬프트)도 보낸다."""
        fd = proc.stdout.fileno()
        buf = b''
        while True:
            ready, _, _ = select.select([fd], [], [], 0.3)
            if ready:
                chunk = os.read(fd, 4096)
                if not chunk:
                    break
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    self._emit_line(run, source, line)
            elif buf:
                self._emit_line(run, source, buf, prompt=True)
                buf = b''
        if buf:
            self._emit_line(run, source, buf)

    def _emit_line(self, run, source, raw, prompt=False):
        text = ANSI_RE.sub('', raw.decode('utf-8', 'replace')).rstrip()
        if not text.strip():
            return
        m = LOG_LEVEL_RE.search(text)
        level = m.group(1) if m else 'INFO'
        if not m and ('Traceback' in text or 'Error' in text or '실패' in text or '중단' in text):
            level = 'ERROR' if 'Traceback' in text or 'Error' in text else 'WARN'
        # "[INFO] [1731712345.123] [dsr01.wall_size]: msg" → "msg"
        msg = re.sub(r'^\[[A-Z]+\]\s*\[[\d.]+\]\s*\[[^\]]*\]:\s*', '', text)
        if prompt:
            run.awaiting_input = True
            self.hub.log('WARN', source, msg, state='INPUT', run_id=run.id, raw=text)
            self._changed()
        else:
            self.hub.log(level, source, msg, run_id=run.id, raw=text)

    def send_stdin(self, run_id, text):
        run = self.get(run_id)
        if run is None or run.proc is None or run.proc.stdin is None:
            raise RuntimeError('입력을 받을 실행 중인 프로세스가 없습니다.')
        run.proc.stdin.write((text + '\n').encode())
        run.proc.stdin.flush()
        run.awaiting_input = False
        self.hub.log('INFO', 'OPERATOR', f'> {text}', state='INPUT', run_id=run.id)
        self._changed()

    # ------------------------------------------------------------------ 정지
    def stop(self, run_id=None, sig=signal.SIGINT):
        targets = [r for r in self.active_runs() if run_id is None or r.id == run_id]
        if not targets:
            raise RuntimeError('정지할 실행 중인 작업이 없습니다.')
        for run in targets:
            run.stop_requested = True
            if run.task['kind'] in ('action', 'db'):
                self.ros.cancel_goal(run)
            elif run.proc is not None:
                self._signal(run, sig)
                if sig == signal.SIGINT:
                    threading.Thread(target=self._escalate, args=(run, run.proc), daemon=True).start()
            else:
                self._finish(run, 'stopped')
            self.hub.log('WARN', 'HMI', f'정지 요청: {run.task["label"]} ({signal.Signals(sig).name}) · 실행 #{run.id}',
                         state='STOP', run_id=run.id, popup=False)
        return len(targets)

    def _signal(self, run, sig):
        proc = run.proc
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            pass

    def _escalate(self, run, proc):
        t0 = now()
        for after, sig in STOP_ESCALATE:
            while now() - t0 < after:
                if proc.poll() is not None:
                    return
                time.sleep(0.2)
            if proc.poll() is None and run.proc is proc:
                self.hub.log('WARN', 'HMI', f'{run.task["label"]} 이(가) 종료되지 않아 {sig.name} 전송', run_id=run.id)
                self._signal(run, sig)

    # ------------------------------------------------------------------ 비상 정지
    def emergency_stop(self):
        self.estop = True
        self.hub.log('ERROR', 'SAFETY', '비상 정지 · move_stop(DR_QSTOP) 요청', state='STOPPED')
        self.ros.move_stop()
        for run in self.active_runs():
            run.stop_requested = True
            if run.task['kind'] in ('action', 'db'):
                self.ros.cancel_goal(run)
                self._finish(run, 'killed')
            else:
                self._signal(run, signal.SIGKILL)
        threading.Timer(0.3, self.ros.release_force).start()
        if self.speed_unlocked or self.speed_pct > SPEED_SAFE_MAX:
            self.speed_unlocked = False
            self.speed_pct = min(self.speed_pct, SPEED_SAFE_MAX)
            self.hub.log('WARN', 'SPEED', f'비상 정지 → 속도 안전 잠금 ({self.speed_pct}%)')
            self.hub.publish('speed', self.speed_json())
        self._changed()

    def reset_estop(self):
        self.estop = False
        self.hub.log('WARN', 'SAFETY', '비상 정지 해제 · 작업자 확인 완료', state='ARMED', popup=False)

    def shutdown(self):
        for run in self.active_runs():
            run.stop_requested = True
            self._signal(run, signal.SIGINT)


# ============================================================================ ROS2 노드
class HmiNode(Node):
    def __init__(self, hub):
        super().__init__('rokey1_hmi')
        self.hub = hub
        self.robot_id = self.declare_parameter('robot_id', 'dsr01').value
        self.host = self.declare_parameter('host', '127.0.0.1').value
        self.port = int(self.declare_parameter('port', 8080).value)
        self.web_root = self.declare_parameter('web_root', '').value
        poll_hz = float(self.declare_parameter('poll_hz', 5.0).value)

        rid = self.robot_id
        ctrl = f'/{rid}/dsr_controller2'
        self.cb = ReentrantCallbackGroup()
        self.lock = threading.Lock()
        self.robot = {'state': None, 'state_name': 'UNKNOWN', 'posx': None, 'force': None,
                      'joints': None, 'joint_hz': 0.0, 'last_update': None}
        self._joint_times = deque(maxlen=50)
        self._pending = set()
        self.manager = None

        self.create_subscription(JointState, f'/{rid}/joint_states', self._on_joints, 10,
                                 callback_group=self.cb)
        self.dsr_clients = {}
        if HAVE_DSR:
            self.create_subscription(RobotError, self.declare_parameter('error_topic', f'/{rid}/error').value,
                                     self._on_error, 10, callback_group=self.cb)
            for key, srv, name in (
                ('state', GetRobotState, 'system/get_robot_state'),
                ('posx', GetCurrentPosx, 'aux_control/get_current_posx'),
                ('force', GetToolForce, 'aux_control/get_tool_force'),
                ('move_stop', MoveStop, 'motion/move_stop'),
                ('release_force', ReleaseForce, 'force/release_force'),
                ('release_compliance', ReleaseComplianceCtrl, 'force/release_compliance_ctrl'),
                ('speed', ChangeOperationSpeed, 'motion/change_operation_speed'),
            ):
                self.dsr_clients[key] = self.create_client(srv, f'{ctrl}/{name}', callback_group=self.cb)
        else:
            self.get_logger().warn('dsr_msgs2 를 불러오지 못해 로봇 상태 조회/정지 기능이 꺼집니다.')

        self.gripper_cli = (self.create_client(SetCommand, '/onrobot/sendCommand', callback_group=self.cb)
                            if HAVE_ONROBOT else None)
        self.tool_task = (ActionClient(self, ToolTask, f'/{rid}/tool_task', callback_group=self.cb)
                          if HAVE_TOOL_TASK else None)
        self.create_timer(1.0 / max(poll_hz, 0.2), self._poll, callback_group=self.cb)

    # ------------------------------------------------------------------ 구독
    def _on_joints(self, msg):
        t = now()
        self._joint_times.append(t)
        with self.lock:
            self.robot['joints'] = [round(p * 57.29577951308232, 2) for p in msg.position[:6]]
            self.robot['last_update'] = t

    def _on_error(self, msg):
        level = {1: 'INFO', 2: 'WARN'}.get(msg.level, 'ERROR')
        group = {1: 'SYSTEM', 2: 'MOTION', 3: 'TP', 4: 'INVERTER', 5: 'SAFETY'}.get(msg.group, 'ROBOT')
        text = ' / '.join(s for s in (msg.msg1, msg.msg2, msg.msg3) if s)
        self.hub.log(level, f'DSR-{group}', f'[{msg.code}] {text}')

    # ------------------------------------------------------------------ 주기 조회
    def _call(self, key, request, handler):
        cli = self.dsr_clients.get(key)
        if cli is None or key in self._pending or not cli.service_is_ready():
            return
        self._pending.add(key)
        future = cli.call_async(request)

        def done(f):
            self._pending.discard(key)
            try:
                res = f.result()
            except Exception:
                return
            if res is not None:
                handler(res)
        future.add_done_callback(done)

    def _poll(self):
        if not HAVE_DSR:
            return
        self._call('state', GetRobotState.Request(), self._on_state)
        req = GetCurrentPosx.Request()
        req.ref = 0
        self._call('posx', req, self._on_posx)
        req = GetToolForce.Request()
        req.ref = 0
        self._call('force', req, self._on_force)

    def _on_state(self, res):
        if not res.success:
            return
        s = int(res.robot_state)
        with self.lock:
            prev = self.robot['state']
            self.robot['state'] = s
            self.robot['state_name'] = ROBOT_STATES.get(s, f'STATE_{s}')
            self.robot['last_update'] = now()
        if prev != s:
            level = 'ERROR' if s in BAD_STATES else 'INFO'
            self.hub.log(level, 'DSR', f'로봇 상태 변경 → {ROBOT_STATES.get(s, s)}',
                         state='FAULT' if s in BAD_STATES else 'OK')

    def _on_posx(self, res):
        if res.success and res.task_pos_info:
            with self.lock:
                self.robot['posx'] = [round(v, 2) for v in list(res.task_pos_info[0].data)[:6]]

    def _on_force(self, res):
        if res.success:
            with self.lock:
                self.robot['force'] = [round(v, 2) for v in res.tool_force]

    # ------------------------------------------------------------------ 상태
    def links(self):
        def ready(key):
            cli = self.dsr_clients.get(key)
            return bool(cli and cli.service_is_ready())
        return {
            'controller': ready('state') or ready('move_stop'),
            'gripper': bool(self.gripper_cli and self.gripper_cli.service_is_ready()),
            'tool_task': bool(self.tool_task and self.tool_task.server_is_ready()),
            'dsr_msgs': HAVE_DSR, 'tool_task_msgs': HAVE_TOOL_TASK,
        }

    def robot_snapshot(self):
        t = now()
        times = [x for x in self._joint_times if t - x < 2.0]
        with self.lock:
            data = dict(self.robot)
        data['joint_hz'] = round(len(times) / 2.0, 1)
        data['stale'] = data['last_update'] is None or t - data['last_update'] > 3.0
        return data

    # ------------------------------------------------------------------ 정지
    def move_stop(self):
        cli = self.dsr_clients.get('move_stop')
        if cli is None or not cli.service_is_ready():
            self.hub.log('ERROR', 'SAFETY', 'move_stop 서비스 없음 → 로봇에 정지 명령을 보내지 못했습니다! '
                                            '티칭 펜던트/하드웨어 비상정지를 사용하세요.', state='FAULT')
            return
        req = MoveStop.Request()
        req.stop_mode = DR_QSTOP
        cli.call_async(req).add_done_callback(
            lambda f: self.hub.log('WARN', 'SAFETY', f'move_stop 응답: 성공={getattr(f.result(), "success", None)}'))

    def set_operation_speed(self, pct):
        """컨트롤러 전체 속도 오버라이드 (1~100%). 실행 중인 동작에도 적용된다."""
        cli = self.dsr_clients.get('speed')
        if cli is None or not cli.service_is_ready():
            self.hub.log('WARN', 'SPEED', 'change_operation_speed 서비스 없음 → 컨트롤러 속도는 바뀌지 않았습니다 '
                                          '(100% 초과 배율은 새로 시작하는 작업에 적용)', popup=False)
            return
        req = ChangeOperationSpeed.Request()
        req.speed = int(max(1, min(100, pct)))

        def done(f):
            try:
                ok = f.result().success
            except Exception:
                ok = False
            if not ok:
                self.hub.log('ERROR', 'SPEED', f'컨트롤러 속도 {req.speed}% 설정 실패')
        cli.call_async(req).add_done_callback(done)

    def release_force(self):
        for key, srv in (('release_force', ReleaseForce if HAVE_DSR else None),
                         ('release_compliance', ReleaseComplianceCtrl if HAVE_DSR else None)):
            cli = self.dsr_clients.get(key)
            if srv is None or cli is None or not cli.service_is_ready():
                continue
            req = srv.Request()
            if hasattr(req, 'time'):
                req.time = 0.0
            cli.call_async(req)
        self.hub.log('INFO', 'SAFETY', '힘/컴플라이언스 제어 해제 요청 전송')

    # ------------------------------------------------------------------ sanding 액션
    def send_tool_task(self, run):
        if self.tool_task is None:
            raise RuntimeError('rokey_interfaces 를 불러오지 못했습니다 (colcon build 확인).')
        if not self.tool_task.server_is_ready():
            raise RuntimeError('/dsr01/tool_task 액션 서버가 없습니다. 먼저 [샌딩 액션 서버] 를 시작하세요.')
        p = run.params
        goal = ToolTask.Goal()
        goal.tool_id = int(p['tool_id'])
        goal.start_posx = parse_posx(p['start_posx'])
        goal.end_posx = parse_posx(p['end_posx'])
        goal.return_tool = bool(p['return_tool'])
        run.command = (f'tool_task goal tool_id={goal.tool_id} start={list(goal.start_posx)} '
                       f'end={list(goal.end_posx)} return_tool={goal.return_tool}')

        def on_feedback(msg):
            fb = msg.feedback
            run.step, run.total, run.stage = fb.step, fb.total_steps, fb.stage
            self.hub.log('INFO', 'SANDING', f'[{fb.step}/{fb.total_steps}] {fb.stage}', run_id=run.id)
            self.manager._changed()

        def on_goal(f):
            handle = f.result()
            if not handle.accepted:
                self.hub.log('ERROR', 'SANDING', 'goal 거절됨 (서버가 다른 작업 중일 수 있음)', run_id=run.id)
                self.manager._finish(run, 'failed')
                return
            run.goal_handle = handle
            run.state = 'running'
            self.manager._changed()
            if run.stop_requested:
                self.cancel_goal(run)
            handle.get_result_async().add_done_callback(on_result)

        def on_result(f):
            res = f.result().result
            self.hub.log('INFO' if res.success else 'ERROR', 'SANDING', f'결과: {res.message}', run_id=run.id)
            if run.stop_requested:
                self.manager._finish(run, 'killed' if self.manager.estop else 'stopped')
            else:
                self.manager._finish(run, 'done' if res.success else 'failed')

        self.tool_task.send_goal_async(goal, feedback_callback=on_feedback).add_done_callback(on_goal)

    def cancel_goal(self, run):
        if run.goal_handle is not None:
            run.goal_handle.cancel_goal_async()


# ============================================================================ HTTP
def find_web_root(param):
    if param:
        return os.path.abspath(os.path.expanduser(param))
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'web')
    if os.path.isdir(here):
        return here
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('rokey1'), 'hmi_web')
    except Exception:
        return here


def make_handler(app):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):   # 접근 로그는 생략
            pass

        # -------------------------------------------------------------- 응답
        def _json(self, code, body):
            data = json.dumps(body, ensure_ascii=False).encode()
            self.send_response(code)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            n = int(self.headers.get('Content-Length') or 0)
            if n > 65536:
                raise ValueError('요청이 너무 큽니다')
            return json.loads(self.rfile.read(n) or b'{}') if n else {}

        # -------------------------------------------------------------- GET
        def do_GET(self):
            path = self.path.split('?', 1)[0]
            if path == '/api/tasks':
                return self._json(200, {'tasks': catalog(), 'modes': list(MODES)})
            if path == '/api/state':
                return self._json(200, app.state())
            if path == '/api/events':
                return self._sse()
            return self._static(path)

        def _static(self, path):
            if path in ('', '/'):
                path = '/index.html'
            root = app.web_root
            full = os.path.realpath(os.path.join(root, path.lstrip('/')))
            if not full.startswith(os.path.realpath(root) + os.sep) or not os.path.isfile(full):
                return self._json(404, {'error': 'not found'})
            with open(full, 'rb') as f:
                data = f.read()
            ctype = mimetypes.guess_type(full)[0] or 'application/octet-stream'
            if ctype.startswith('text/') or ctype.endswith('javascript'):
                ctype += '; charset=utf-8'
            self.send_response(200)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            self.wfile.write(data)

        def _sse(self):
            q, backlog = app.hub.subscribe()
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()

            def send(msg):
                payload = json.dumps(msg['data'], ensure_ascii=False)
                self.wfile.write(f'event: {msg["event"]}\ndata: {payload}\n\n'.encode())
                self.wfile.flush()
            try:
                send({'event': 'backlog', 'data': [m['data'] for m in backlog]})
                send({'event': 'state', 'data': app.state()})
                # 로봇 상태(관절각·TCP)는 로그가 계속 들어와도 STATE_PERIOD 마다 반드시 보낸다.
                # (예전에는 0.5초 동안 다른 이벤트가 없을 때만 보내서, 작업 중 로그가 많으면 화면이 멈췄음)
                next_state = time.monotonic() + STATE_PERIOD
                while app.running:
                    try:
                        send(q.get(timeout=max(0.0, next_state - time.monotonic())))
                    except queue.Empty:
                        pass
                    if time.monotonic() >= next_state:
                        send({'event': 'state', 'data': app.state()})
                        next_state = time.monotonic() + STATE_PERIOD
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                app.hub.unsubscribe(q)
                self.close_connection = True

        # -------------------------------------------------------------- POST
        def do_POST(self):
            path = self.path.split('?', 1)[0]
            try:
                body = self._body()
                m = app.manager
                if path == '/api/start':
                    run = m.start(body.get('task_id'), body.get('mode', 'virtual'), body.get('params'))
                    return self._json(200, {'ok': True, 'run': run.to_json()})
                if path == '/api/stop':
                    rid = body.get('run_id')
                    n = m.stop(int(rid) if rid is not None else None)
                    return self._json(200, {'ok': True, 'stopped': n})
                if path == '/api/stdin':
                    m.send_stdin(int(body.get('run_id')), str(body.get('text', '')))
                    return self._json(200, {'ok': True})
                if path == '/api/estop':
                    m.emergency_stop()
                    return self._json(200, {'ok': True})
                if path == '/api/speed':
                    m.set_speed(body.get('pct', SPEED_DEFAULT), body.get('unlocked', False))
                    return self._json(200, {'ok': True, 'speed': m.speed_json()})
                if path == '/api/estop/reset':
                    m.reset_estop()
                    return self._json(200, {'ok': True})
                return self._json(404, {'ok': False, 'error': 'not found'})
            except (ValueError, RuntimeError, TypeError) as e:
                app.hub.log('WARN', 'HMI', f'요청 거부: {e}', state='REJECT', popup=False)
                return self._json(400, {'ok': False, 'error': str(e)})

    return Handler


class HmiApp:
    def __init__(self):
        self.hub = EventHub()
        self.node = HmiNode(self.hub)
        self.manager = TaskManager(self.hub, self.node)
        self.node.manager = self.manager
        if ProcessRunner is not None:
            self.manager.runner = ProcessRunner(self.node, self.hub)
        self.web_root = find_web_root(self.node.web_root)
        self.running = True

    def state(self):
        return {
            't': now(), 'estop': self.manager.estop, 'runs': self.manager.snapshot(),
            'speed': self.manager.speed_json(),
            'robot': self.node.robot_snapshot(), 'links': self.node.links(),
            'robot_id': self.node.robot_id,
        }


def main(args=None):
    # rclpy 가 SIGTERM 을 받으면 ROS 만 끄고 웹 서버는 계속 돌아서 8080 을 잡고 있었다 → 신호는 직접 처리
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    app = HmiApp()
    log = app.node.get_logger()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(app.node)
    threading.Thread(target=executor.spin, daemon=True).start()

    server = ThreadingHTTPServer((app.node.host, app.node.port), make_handler(app))
    server.daemon_threads = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    log.info(f'HMI web root : {app.web_root}')
    log.info(f'HMI 접속 주소 : http://{"localhost" if app.node.host in ("0.0.0.0", "127.0.0.1") else app.node.host}'
             f':{app.node.port}')
    app.hub.log('INFO', 'HMI', f'hmi_server 시작 · port {app.node.port}')
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        app.running = False
        app.manager.shutdown()
        server.server_close()
        executor.shutdown()
        app.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
