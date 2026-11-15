from dsr_msgs2.srv import MoveStop
import rclpy


class MoveStopClient:
    """
    dsr_controller2/motion/move_stop 서비스로 로봇 모션을 정지시키는 클래스.

    DSR_ROBOT2 에는 정지(stop) 함수가 없어서 서비스를 직접 호출한다.
    DSR_ROBOT2 는 모듈 최상단에서 import 할 수 없으므로 stop_mode 는 인자로 받는다.

    사용 예:
        stopper = MoveStopClient(node)
        ServiceWaiter(node).wait(stopper.cli)
        stopper.stop(DSR_ROBOT2.DR_QSTOP)                   # 응답까지 대기
        stopper.stop(DSR_ROBOT2.DR_QSTOP, timeout_sec=2.0)  # 최대 2초 대기
    """

    def __init__(self, node):
        self.node = node
        self.cli = node.create_client(MoveStop, 'dsr_controller2/motion/move_stop')

    def stop(self, stop_mode, timeout_sec=None):
        """정지 요청을 보내고 응답(또는 timeout_sec)까지 기다린다. 응답이 없으면 None."""
        req = MoveStop.Request()
        req.stop_mode = stop_mode
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        return future.result()
