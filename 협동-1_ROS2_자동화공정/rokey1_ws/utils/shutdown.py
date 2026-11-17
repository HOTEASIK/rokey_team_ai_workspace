import rclpy


def run_cleanups(node, *cleanups):
    """
    정리 동작을 순서대로 실행한다. 하나가 실패해도 다음 정리는 계속한다.

    cleanup 은 콜백, 또는 (실패 메시지, 콜백) 튜플이다.
    실패하면 '<실패 메시지>: <예외>' 를 error 로그로 남긴다.

    사용 예:
        run_cleanups(node, ('힘 제어 해제 실패', io.release_press))
    """
    logger = node.get_logger() if node is not None else rclpy.logging.get_logger('shutdown')
    for cleanup in cleanups:
        if isinstance(cleanup, tuple):
            message, fn = cleanup
        else:
            message, fn = f'{getattr(cleanup, "__name__", cleanup)} failed', cleanup
        try:
            fn()
        except Exception as e:
            logger.error(f'{message}: {e}')


def safe_shutdown(node, *cleanups):
    """
    정리 동작 → destroy_node → rclpy.shutdown 순서로 프로그램을 끝낸다.

    각 cleanup 이 실패해도 destroy_node / rclpy.shutdown 은 반드시 실행된다.
    node 가 None 이면 destroy_node 는 건너뛴다.

    사용 예:
        try:
            ...  # 작업
        except KeyboardInterrupt:
            node.get_logger().info("Program Stopped")
        finally:
            safe_shutdown(node, ('Failed to move home',
                                 lambda: movej(homej, vel=VELOCITY, acc=ACC)))
    """
    run_cleanups(node, *cleanups)
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
