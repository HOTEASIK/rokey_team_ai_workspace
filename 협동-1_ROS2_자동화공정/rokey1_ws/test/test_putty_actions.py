"""로봇/ROS graph 없이 액션 취소·결과·중복 목표·HMI 연결을 검사한다."""

import sys
import threading
import time
from types import MethodType, SimpleNamespace

import pytest
from rclpy.action import GoalResponse, CancelResponse

from putty_demo.action_protocol import worker_command
from putty_demo import putty_task_server as module
from putty_demo.putty_task_server import PuttyTaskServer


def goal(task='apply', virtual=False, confirmed=False, arguments=None):
    if arguments is None:
        arguments = ['--floor-force', '2.3', '--wall-force', '2.3', '--press-force', '10'] if task == 'apply' else []
    return SimpleNamespace(task=task, virtual_mode=virtual, tool_held_confirmed=confirmed,
                           arguments=arguments)


@pytest.mark.parametrize('task', ['initialize', 'pickup', 'scoop', 'apply', 'return'])
def test_worker_commands_keep_original_nodes_and_explicit_execution(task):
    command = worker_command(goal(task, virtual=True))
    assert command[:4] == [sys.executable, '-u', '-m', 'putty_demo.action_worker']
    assert command[4] == task
    assert ('execute_motion:=true' if task == 'initialize' else '--execute-motion') in command


@pytest.mark.parametrize('options', [['--ros-args'], ['--extra-approach', '7'],
                                    ['--execute-motion'], ['--press-force', 'nan'],
                                    ['--press-force', '6', '--press-force', '8']])
def test_unapproved_or_invalid_options_rejected(options):
    with pytest.raises(ValueError):
        worker_command(goal(arguments=options))


def test_real_apply_requires_contact_search_settings():
    with pytest.raises(ValueError):
        worker_command(goal(arguments=[]))
    assert '--contact-search' in worker_command(goal())


def test_real_return_and_unconfirmed_scoop_do_not_start():
    with pytest.raises(ValueError):
        worker_command(goal('return'))
    with pytest.raises(ValueError):
        worker_command(goal('scoop'))
    assert 'scoop' in worker_command(goal('scoop', confirmed=True))


def fake_server():
    server = SimpleNamespace(
        lock=threading.Lock(), busy=True, finishing=False, faulted=False,
        cancel_event=threading.Event(), shutdown_event=threading.Event(), events=[],
    )
    logger = SimpleNamespace(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None)
    server.get_logger = lambda: logger
    server.check_cancel = MethodType(PuttyTaskServer.check_cancel, server)
    server.reserve_goal = MethodType(PuttyTaskServer.reserve_goal, server)
    server.cancel_goal = MethodType(PuttyTaskServer.cancel_goal, server)
    server.stop_robot = lambda: server.events.append('robot_stop_confirmed')
    def call(name, request, **kwargs):
        server.events.append(name)
        return SimpleNamespace(success=True, robot_system=0, robot_state=1)
    server.call = call
    server.feedback = lambda *a, **kw: None
    server.cleanup_worker = MethodType(PuttyTaskServer.cleanup_worker, server)
    return server


class Handle:
    def __init__(self):
        self.request = goal()
        self.is_cancel_requested = False
        self.terminal = None
    def succeed(self):
        self.terminal = 'succeeded'
    def canceled(self):
        self.terminal = 'canceled'
    def abort(self):
        self.terminal = 'aborted'


class FakeWorker:
    def __init__(self, server, handle, cancel=False, exit_code=0):
        self.server, self.handle, self.cancel = server, handle, cancel
        self.exit_code = exit_code
        self.reader = SimpleNamespace(join=lambda **kw: None)
    def drain(self):
        return []
    def poll(self):
        if self.cancel:
            self.handle.is_cancel_requested = True
            assert self.server.cancel_goal(self.handle) == CancelResponse.ACCEPT
            return None  # 기존 이동 서비스가 아직 완료되지 않은 상태
        return self.exit_code
    def freeze(self):
        self.server.events.append('freeze')
    def terminate(self):
        self.server.events.append('terminate')
        self.cancel, self.exit_code = False, 130


def test_cancel_during_unfinished_motion_stops_before_canceled_result(monkeypatch):
    server, handle = fake_server(), Handle()
    worker = FakeWorker(server, handle, cancel=True)
    monkeypatch.setattr(module, 'WorkerProcess', lambda *a, **kw: worker)
    result = PuttyTaskServer.execute(server, handle)
    assert handle.terminal == 'canceled'
    assert not result.success and result.stop_confirmed
    assert server.events.index('freeze') < server.events.index('robot_stop_confirmed')
    assert server.events.index('robot_stop_confirmed') < server.events.index('terminate')
    assert server.events.index('terminate') < server.events.index('release')
    assert 'release_comp' in server.events
    assert not server.busy and not server.faulted


def test_cleanup_failure_aborts_and_latches_new_goal_rejection(monkeypatch):
    server, handle = fake_server(), Handle()
    worker = FakeWorker(server, handle, cancel=True)
    monkeypatch.setattr(module, 'WorkerProcess', lambda *a, **kw: worker)
    original_call = server.call
    def fail_release(name, request, **kw):
        if name == 'release':
            raise TimeoutError('force release failed')
        return original_call(name, request, **kw)
    server.call = fail_release
    result = PuttyTaskServer.execute(server, handle)
    assert handle.terminal == 'aborted'
    assert not result.stop_confirmed and server.faulted
    assert 'release_comp' in server.events  # 한 해제가 실패해도 나머지 정리를 실행
    assert PuttyTaskServer.accept_goal(server, goal()) == GoalResponse.REJECT


def test_normal_result_requires_standby_and_does_not_interrupt_motion(monkeypatch):
    server, handle = fake_server(), Handle()
    monkeypatch.setattr(module, 'WorkerProcess', lambda *a, **kw: FakeWorker(server, handle))
    result = PuttyTaskServer.execute(server, handle)
    assert handle.terminal == 'succeeded'
    assert result.success and result.stop_confirmed and result.exit_code == 0
    assert 'robot_stop_confirmed' not in server.events


def test_worker_failure_stops_and_releases_before_abort(monkeypatch):
    server, handle = fake_server(), Handle()
    monkeypatch.setattr(module, 'WorkerProcess',
                        lambda *a, **kw: FakeWorker(server, handle, exit_code=1))
    result = PuttyTaskServer.execute(server, handle)
    assert handle.terminal == 'aborted' and not result.success
    assert result.stop_confirmed
    assert 'robot_stop_confirmed' in server.events and 'release_comp' in server.events


def test_mode_mismatch_never_starts_worker(monkeypatch):
    server, handle = fake_server(), Handle()
    handle.request.virtual_mode = True
    def forbidden(*a, **kw):
        pytest.fail('모드가 다르면 작업 프로세스를 시작하면 안 된다.')
    monkeypatch.setattr(module, 'WorkerProcess', forbidden)
    result = PuttyTaskServer.execute(server, handle)
    assert handle.terminal == 'aborted' and not result.success


def test_busy_server_rejects_concurrent_goal():
    server = fake_server()
    assert PuttyTaskServer.accept_goal(server, goal()) == GoalResponse.REJECT
    server.busy = False
    assert PuttyTaskServer.accept_goal(server, goal()) == GoalResponse.ACCEPT
    assert PuttyTaskServer.accept_goal(server, goal()) == GoalResponse.REJECT


def test_service_wait_remains_cancellable_without_waiting_for_motion_completion():
    server = fake_server()
    future = SimpleNamespace(done=lambda: False, cancel=lambda: server.events.append('future_cancel'))
    server.control_clients = {'state': SimpleNamespace(service_is_ready=lambda: True,
                                               call_async=lambda request: future)}
    timer = threading.Timer(0.05, server.cancel_event.set)
    timer.start()
    started = time.monotonic()
    try:
        with pytest.raises(InterruptedError):
            PuttyTaskServer.call(server, 'state', object(), timeout=5)
    finally:
        timer.join()
    assert time.monotonic()-started < 1
    assert 'future_cancel' in server.events


def test_hmi_putty_tasks_use_actions_and_sanding_is_preserved():
    from hmi.tasks import TASK_MAP, putty_arguments
    for name in ('initialize_device', 'pickup_spatula', 'scoop_putty',
                 'apply_putty_plane', 'return_spatula', 'putty_cycle'):
        assert TASK_MAP[name]['kind'] == 'action'
        assert TASK_MAP[name]['putty_tasks']
        assert TASK_MAP[name]['build'] is None
    assert TASK_MAP['sanding_goal']['kind'] == 'action'
    assert putty_arguments('apply', dict(floor_force=2.3, wall_force=2.3,
                                       press_force=6, apply_speed=10))[-2:] == ['--apply-speed', '10']


def test_worker_can_be_interrupted_while_its_operation_is_still_running():
    """ROS/로봇 없이 실제 자식 프로세스의 SIGSTOP→SIGINT 종료를 확인한다."""
    from utils.worker_process import WorkerProcess
    script = ('import signal,time,sys; '
              'signal.signal(signal.SIGINT, lambda *a: sys.exit(130)); '
              'print("ready", flush=True); time.sleep(60)')
    worker = WorkerProcess([sys.executable, '-u', '-c', script])
    try:
        deadline = time.monotonic()+3
        lines = []
        while 'ready' not in lines and time.monotonic() < deadline:
            lines.extend(worker.drain())
            time.sleep(0.01)
        assert 'ready' in lines and worker.poll() is None
        worker.freeze()
        worker.terminate()
        assert worker.poll() == 130
    finally:
        worker.terminate()


def test_server_constructor_preserves_ros_node_read_only_clients_property(monkeypatch, tmp_path):
    """실제 Node 상속 속성은 유지하고 ROS 통신만 대체해 시작 단계 충돌을 검출한다."""
    from rclpy.node import Node

    def init_without_middleware(self, *args, **kwargs):
        self._context = SimpleNamespace(get_domain_id=lambda: 221)
        self._clients = []
    def make_client(self, service_type, path):
        client = SimpleNamespace(path=path)
        self._clients.append(client)
        return client
    monkeypatch.setattr(Node, '__init__', init_without_middleware)
    monkeypatch.setattr(Node, 'create_client', make_client)
    monkeypatch.setattr(Node, 'declare_parameter', lambda *a: SimpleNamespace(value=False))
    monkeypatch.setattr(module, 'open', lambda *a, **kw: (tmp_path/'server.lock').open('a'),
                        raising=False)
    monkeypatch.setattr(module, 'ActionServer', lambda *a, **kw: SimpleNamespace())
    server = PuttyTaskServer()
    try:
        assert set(server.control_clients) == {'system', 'state', 'pose', 'joints', 'force',
                                               'stop', 'release', 'release_comp'}
        assert len(list(server.clients)) == 8
        assert isinstance(Node.clients, property) and Node.clients.fset is None
    finally:
        server.server_lock.close()


def test_team_process_uses_same_apply_worker_and_preserves_force_settings():
    from copy import deepcopy
    from utils.config import CFG
    from putty_demo.action_protocol import process_putty_goal
    cfg = deepcopy(CFG)
    cfg['force']['putty_scrape']['target'][0] = 10.0  # 유효 요청용 테스트 값, 운영 설정 변경 없음
    request = SimpleNamespace(task_code='PUTTY', start_point=[789., -15., 27.],
                              end_point=[789., -15., 207.])
    converted = process_putty_goal(request, cfg)
    command = worker_command(converted)
    assert command[4] == 'apply'
    assert command[command.index('--force-mode')+1] == 'relative'
    assert command[command.index('--press-force')+1] == '10.0'
    request.end_point[0] += 1
    with pytest.raises(ValueError, match='E_NO_GAP'):
        process_putty_goal(request, cfg)


def test_existing_six_newton_minimum_conflict_is_rejected_before_worker():
    with pytest.raises(ValueError, match='10 N'):
        worker_command(goal(arguments=['--floor-force', '2.3', '--wall-force', '2.3']))


def test_process_trace_only_collects_finite_upward_force_samples():
    view = module.ProcessTaskView(SimpleNamespace(is_cancel_requested=False), goal())
    view.observe_line('단계=힘 형성 대기 | 실측BASE_Fxyz=[10, 0, 0]')
    view.observe_line('단계=상향 도포 중 | 실측BASE_Fxyz=[-10.2, 1, 2]')
    view.observe_line('단계=상향 도포 중 | 실측BASE_Fxyz=[nan, 1, 2]')
    assert view.fx == [-10.2]
    view.succeed()
    assert view.status == 'succeeded'


def test_process_and_existing_goals_share_busy_lock():
    server = fake_server()
    server.busy = False
    assert PuttyTaskServer.accept_process_goal(server, SimpleNamespace(task_code='PUTTY')) == GoalResponse.ACCEPT
    assert PuttyTaskServer.accept_goal(server, goal()) == GoalResponse.REJECT


def test_team_force_stats_can_record_base_x_without_relabeling_as_z():
    import json
    from utils.force_stats import summarize
    ratio, stats = summarize([9., 10., 11.], 10., 1., axis='fx')
    assert ratio == 1.0
    data = json.loads(stats)
    assert data['fx_mean'] == 10 and 'fz_mean' not in data


def test_process_result_and_completed_feedback_use_existing_executor(monkeypatch):
    from rokey_interfaces.action import PuttyTask
    server = fake_server()
    server.process_virtual_mode = False
    server.process_snapshot = lambda: dict(joints=[0.]*6, pose=[1.]*6, force=[2.]*6)
    def execute(view, release_slot):
        assert release_slot is False
        view.worker_started = True
        view.fx = [6., -6.]
        view.succeed()
        result = PuttyTask.Result()
        result.success, result.message = True, 'completed'
        return result
    server.execute = execute
    monkeypatch.setattr(module, 'process_putty_goal', lambda *a: goal())
    handle = Handle()
    handle.request = SimpleNamespace(task_code='PUTTY', run_id=1, gap_id=2, start_point=[1.,2.,3.], end_point=[1.,2.,4.])
    feedback = []
    handle.publish_feedback = feedback.append
    result = PuttyTaskServer.execute_process(server, handle)
    assert result.success and handle.terminal == 'succeeded'
    assert len(feedback) == 1 and feedback[0].step == 'putty_scrape'
    assert feedback[0].contact_ratio == 1.0
    assert not server.busy


def test_process_invalid_configuration_reports_force_error_without_worker(monkeypatch):
    server = fake_server()
    server.process_virtual_mode = False
    server.process_snapshot = lambda: dict(joints=[float('nan')]*6, pose=[float('nan')]*6,
                                           force=[float('nan')]*6)
    def invalid(*a):
        raise ValueError('E_FORCE: invalid force')
    monkeypatch.setattr(module, 'process_putty_goal', invalid)
    server.execute = lambda *a: pytest.fail('must not start worker')
    handle = Handle()
    handle.request = SimpleNamespace(task_code='PUTTY', run_id=1, gap_id=2, start_point=[1.,2.,3.], end_point=[1.,2.,4.])
    feedback = []
    handle.publish_feedback = feedback.append
    result = PuttyTaskServer.execute_process(server, handle)
    assert not result.success and result.error_code == 'E_FORCE'
    assert handle.terminal == 'aborted' and not server.busy
    assert '"worker_started": false' in feedback[0].params_json
    assert '"force_valid": false' in feedback[0].params_json
