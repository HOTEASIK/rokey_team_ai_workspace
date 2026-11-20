"""
공통 관심사(cross-cutting concern)를 데코레이터로 분리한 AOP 모듈.

- dsr_main : DSR 노드 라이프사이클 (init → 노드 생성 → 예외 로깅 → 정리 → shutdown)
- timed    : 함수 실행 시간 로깅

사용 예:
    @dsr_main('rokey_move')
    def main(app):
        from DSR_ROBOT2 import movej
        app.on_shutdown('Failed to move home', lambda: movej(home, vel=30, acc=30))
        ...  # 핵심 로직만 작성
"""
import functools
import time
import traceback

import rclpy
import DR_init

from utils.shutdown import safe_shutdown

from utils.robot import ROBOT_ID as DEFAULT_ROBOT_ID, ROBOT_MODEL as DEFAULT_ROBOT_MODEL  # config/robot.yaml


class DsrApp:
    """dsr_main 이 본문에 넘겨주는 실행 컨텍스트."""

    def __init__(self, node, args):
        self.node = node
        self.args = args
        self.logger = node.get_logger()
        self._cleanups = []

    def on_shutdown(self, message, fn):
        """종료 시 실행할 정리 동작을 등록한다 (등록 순서대로 실행, 실패해도 계속)."""
        self._cleanups.append((message, fn))


def dsr_main(node_name, robot_id=DEFAULT_ROBOT_ID, robot_model=DEFAULT_ROBOT_MODEL):
    """
    DSR 노드 main 을 감싸는 데코레이터.

    - rclpy.init, DR_init 설정, 노드 생성
    - KeyboardInterrupt → info 'Program Stopped'
    - ImportError / 그 외 예외 → error 로그(+traceback), 종료 코드 1
    - 무조건 등록된 정리 동작 → destroy_node → rclpy.shutdown (safe_shutdown)

    감싼 함수는 main(args=None) 형태가 되어 console_scripts 에 그대로 쓸 수 있다.
    args 뒤에 넘긴 추가 인자는 body(app, ...) 로 그대로 전달된다.
    """
    def decorator(body):
        @functools.wraps(body)
        def main(args=None, *extra, **kwargs):
            rclpy.init(args=args)
            DR_init.__dsr__id = robot_id
            DR_init.__dsr__model = robot_model
            node = rclpy.create_node(node_name, namespace=robot_id)
            DR_init.__dsr__node = node

            app = DsrApp(node, args)
            try:
                return body(app, *extra, **kwargs)
            except KeyboardInterrupt:
                app.logger.info('Program Stopped')
            except ImportError as e:
                app.logger.error(f'Error importing DSR_ROBOT2 : {e}')
                return 1
            except Exception as e:
                app.logger.error(f'Robot Error: {e}\n{traceback.format_exc()}')
                return 1
            finally:
                safe_shutdown(node, *app._cleanups)
        return main
    return decorator


def timed(logger, label=None):
    """함수 실행 시간을 '[Timing] <label> elapsed time: x.xxxx sec' 로 로깅한다."""
    def decorator(fn):
        name = label or fn.__name__

        @functools.wraps(fn)
        def wrapper(*args, **kwargs):
            start = time.perf_counter()
            try:
                return fn(*args, **kwargs)
            finally:
                elapsed = time.perf_counter() - start
                logger.info(f'[Timing] {name} elapsed time: {elapsed:.4f} sec')
        return wrapper
    return decorator
