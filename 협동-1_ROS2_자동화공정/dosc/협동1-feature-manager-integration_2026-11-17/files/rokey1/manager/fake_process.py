"""매니저 테스트용 가짜 공정 서버 (로봇 안 움직임).
/process/explore, /process/putty, /process/sanding 을 모두 받아서
피드백 2번 보낸 뒤 성공. EXPLORE 는 벽 1개 + 틈 1개를 돌려준다.

실행: ros2 run rokey1 fake_process
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rokey_interfaces.action import Process
from utils.force_stats import summarize


class Fake(Node):
    def __init__(self):
        super().__init__('fake_process')
        for t in ('explore', 'putty', 'sanding'):
            ActionServer(self, Process, f'/process/{t}', self.execute)

    def execute(self, gh):
        g = gh.request
        for step in ('a', 'b'):
            if gh.is_cancel_requested:
                gh.canceled()
                return Process.Result(success=False, error_code='CANCELED')
            time.sleep(1.0)
            ratio, stats = summarize([-9.5, -10.2, -11.0, -6.0], target=-10.0, tol=2.0)
            gh.publish_feedback(Process.Feedback(
                state='RUNNING', step=f'{g.task_code.lower()}_{step}', step_success=True,
                duration_ms=1000, params_json='{"target_fz": -10, "vel": 30}',
                contact_ratio=ratio, force_stats_json=stats))
        gh.succeed()
        res = Process.Result(success=True)
        if g.task_code == 'EXPLORE':
            res.wall = [400.0, 0.0, 600.0, 400.0, 0.0, 100.0, 400.0, -300.0, 350.0, 400.0, 300.0, 350.0]
            res.gaps = [400.0, -200.0, 350.0, 400.0, 200.0, 350.0]
        return res


def main():
    rclpy.init()
    ex = MultiThreadedExecutor()
    ex.add_node(Fake())
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
