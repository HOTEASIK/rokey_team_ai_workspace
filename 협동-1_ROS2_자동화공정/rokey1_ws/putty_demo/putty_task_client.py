"""퍼티 액션 CLI. Ctrl+C/SIGTERM은 Action Cancel을 보내고 정리 결과를 기다린다."""

import argparse
import signal
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from rokey_interfaces.action import PuttyTask

from .action_protocol import worker_command
from .action_worker import MODULES
from utils.robot import ROBOT_ID  # config/robot.yaml robot 섹션


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('task', choices=MODULES)
    parser.add_argument('--virtual-mode', action='store_true')
    parser.add_argument('--tool-held-confirmed', action='store_true', help='사용자가 실제 헤라 파지를 확인함')
    # '--' 뒤에는 작업 노드의 기존 설정 옵션을 그대로 전달한다.
    import sys
    tokens = list(sys.argv[1:] if argv is None else argv)
    split = tokens.index('--') if '--' in tokens else len(tokens)
    args = parser.parse_args(tokens[:split])
    goal = PuttyTask.Goal()
    goal.task, goal.virtual_mode = args.task, args.virtual_mode
    goal.tool_held_confirmed = args.tool_held_confirmed
    goal.arguments = tokens[split+1:]
    try:
        worker_command(goal)
    except (ValueError, TypeError) as exc:
        parser.error(str(exc))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('putty_task_client', namespace=ROBOT_ID)
    client = ActionClient(node, PuttyTask, f'/{ROBOT_ID}/putty_task')
    cancel = threading.Event()
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda *_: cancel.set())
    try:
        deadline = time.monotonic()+10
        while not client.wait_for_server(timeout_sec=0.1):
            if cancel.is_set():
                return 130
            if time.monotonic() > deadline:
                raise TimeoutError('퍼티 액션 서버가 없습니다. putty_task_server를 먼저 실행하세요.')

        def feedback(message):
            fb = message.feedback
            pose = str([round(v, 2) for v in fb.tcp_pose]) if fb.pose_valid else '조회 대기'
            force = str([round(v, 2) for v in fb.tool_force[:3]]) if fb.force_valid else '조회 대기'
            node.get_logger().info(f'[{fb.task}] {fb.elapsed_sec:.1f}s | {fb.stage}'
                                   f' | BASE TCP={pose} | BASE Fxyz={force}')

        sent = client.send_goal_async(goal, feedback_callback=feedback)
        # Goal 수락이 늦어도 Future를 버리지 않는다. 취소 요청을 기억했다가 수락 후 전달한다.
        while not sent.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        handle = sent.result()
        if handle is None or not handle.accepted:
            node.get_logger().error('Goal 거절: 서버 로그에서 실행 중/인자/정리 실패 여부를 확인하세요.')
            return 1
        result_future = handle.get_result_async()
        cancel_sent = False
        cancel_deadline = None
        while not result_future.done():
            if cancel.is_set() and not cancel_sent:
                handle.cancel_goal_async()
                cancel_sent = True
                cancel_deadline = time.monotonic()+30
                node.get_logger().warning('액션 취소 전송. 로봇 정지·힘 해제 결과를 기다립니다.')
            if cancel_deadline is not None and time.monotonic() > cancel_deadline:
                raise TimeoutError('취소 결과를 확인하지 못했습니다. 로봇 정지를 별도로 확인하세요.')
            rclpy.spin_once(node, timeout_sec=0.1)
        response = result_future.result()
        result = response.result
        node.get_logger().info(f'액션 결과 status={response.status} | {result.message}'
                               f' | 정지확인={result.stop_confirmed} | exit={result.exit_code}')
        if response.status == GoalStatus.STATUS_SUCCEEDED and result.success:
            return 0
        if response.status == GoalStatus.STATUS_CANCELED:
            return 130
        return 1
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
