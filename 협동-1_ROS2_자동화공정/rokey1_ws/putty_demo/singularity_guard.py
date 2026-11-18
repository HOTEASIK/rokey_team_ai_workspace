"""지원되는 특이점 정책 API와 실시간 알람을 사용한다. 자동 회복 이동은 수행하지 않는다.

현재 V2 드라이버에는 get_force_control_state 서비스가 없다.
알람 부재를 '현재 자세가 안전하다'는 상태 조회로 해석하지 않는다.
컨트롤러의 SINGULARITY_ERROR 정책을 함께 설정해 힘 제어 진입도 검사한다.
"""

import threading
import time


class SingularityDetected(RuntimeError):
    pass


class SingularityLatch:
    """한 번 감지되면 이 실행에서는 해제하지 않는다. 이탈 알람이 와도 자동 재개하지 않는다."""

    def __init__(self):
        self.event = threading.Event()
        self.reason = ''

    def observe(self, message):
        if message.group != 2:
            return False
        text = ' / '.join((message.msg1, message.msg2, message.msg3))
        normalized = ''.join(text.lower().split()).replace('→', '->')
        # 3205는 영역 전이 알림이다. 명시적인 이탈만 무시하고, 불명확하면 중단한다.
        if message.code == 3205:
            if 'inside->outside' in normalized and 'outside->inside' not in normalized:
                return False
        elif not (message.level >= 2 and 'singular' in normalized):
            return False
        if self.event.is_set():
            return False
        self.reason = f'특이점 알람 code={message.code}: {text}'
        self.event.set()
        return True

    def check(self):
        if self.event.is_set():
            raise SingularityDetected(self.reason+' | 정지·힘 해제, 자동 회복/재시도 없음')


class SingularityWatch:
    """별도 노드/executor로 이동 서비스 응답 대기 중에도 알람을 받는다."""

    def __init__(self, robot_id):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from dsr_msgs2.msg import RobotError

        self.rclpy = rclpy
        self.topic = f'/{robot_id}/error'
        self.node = rclpy.create_node('putty_singularity_monitor', namespace=robot_id)
        self.latch = SingularityLatch()
        self.stopping = threading.Event()
        self.failure = None
        self.last_spin = time.monotonic()
        self.subscription = self.node.create_subscription(RobotError, self.topic, self._alarm, 100)
        self.executor = SingleThreadedExecutor(context=self.node.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _alarm(self, message):
        if self.latch.observe(message):
            self.node.get_logger().error(f'[특이점 감지] {self.latch.reason}')

    def _spin(self):
        try:
            while not self.stopping.is_set() and self.rclpy.ok():
                self.executor.spin_once(timeout_sec=0.05)
                self.last_spin = time.monotonic()
        except Exception as exc:
            self.failure = str(exc)

    def wait_ready(self, timeout=5.0):
        deadline = time.monotonic()+timeout
        while self.node.count_publishers(self.topic) == 0:
            self.check(require_publisher=False)
            if time.monotonic() >= deadline:
                raise RuntimeError(f'특이점 알람 publisher 없음: {self.topic}. 이동하지 않습니다.')
            time.sleep(0.05)
        self.check()

    def check(self, require_publisher=True):
        self.latch.check()
        if self.failure or not self.thread.is_alive() or time.monotonic()-self.last_spin > 1.0:
            raise RuntimeError(f'특이점 알람 감시 중단: {self.failure or "executor 응답 없음"}')
        if require_publisher and self.node.count_publishers(self.topic) == 0:
            raise RuntimeError(f'특이점 알람 publisher 연결 소실: {self.topic}')

    def close(self):
        self.stopping.set()
        self.executor.wake()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('특이점 감시 스레드 종료 확인 실패')
        self.executor.shutdown()
        self.node.destroy_node()


def start_singularity_watch(robot_id):
    return SingularityWatch(robot_id)
