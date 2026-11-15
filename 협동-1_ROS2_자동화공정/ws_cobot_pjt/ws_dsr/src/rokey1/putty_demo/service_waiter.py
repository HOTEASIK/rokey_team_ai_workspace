class ServiceWaiter:
    """DSR 서비스가 준비될 때까지 기다리는 클래스.

    서비스 디스커버리가 끝나기 전에 요청을 보내면 응답이 오지 않아 무한 대기하므로,
    사용할 함수의 서비스 클라이언트를 넘겨 준비될 때까지 기다린다.

    사용 예:
        waiter = ServiceWaiter(node)
        waiter.wait(DSR_ROBOT2._ros2_movej, DSR_ROBOT2._ros2_movel)
    """

    def __init__(self, node, timeout_sec=1.0):
        self.node = node
        self.timeout_sec = timeout_sec

    def wait(self, *clients):
        """넘겨받은 서비스 클라이언트가 모두 준비될 때까지 대기한다."""
        for cli in clients:
            while not cli.wait_for_service(timeout_sec=self.timeout_sec):
                self.node.get_logger().info(f"Waiting for service {cli.srv_name} ...")
