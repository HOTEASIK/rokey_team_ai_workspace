from utils.aspects import dsr_main
from utils.service_waiter import ServiceWaiter

# ===== 로봇 설정 =====
# 로봇 ID / 모델은 dsr_main 기본값(dsr01, m0609). 다르면 dsr_main(..., robot_id=, robot_model=)
VELOCITY, ACC = 30, 30   # 속도, 가속도
CYCLE = 1                # 반복 횟수


# 노드 생성 / 예외 로깅 / 종료 정리(destroy_node, rclpy.shutdown)는 dsr_main 이 처리한다
@dsr_main("node_name")   # TODO: 노드 이름
def main(app):
    node = app.node

    import DSR_ROBOT2
    from DSR_ROBOT2 import (
        set_tool,
        set_tcp,
        movej,
        movel,
        # TODO: 사용할 함수 추가
    )
    from DR_common2 import posx, posj

    # ===== 서비스 준비될 때까지 대기 =====
    # 사용하는 함수의 서비스 클라이언트를 적는다
    #   set_tool -> _ros2_set_current_tool,  set_tcp -> _ros2_set_current_tcp
    #   movej    -> _ros2_movej,             movel   -> _ros2_movel
    ServiceWaiter(node).wait(
        DSR_ROBOT2._ros2_set_current_tool,
        DSR_ROBOT2._ros2_set_current_tcp,
        DSR_ROBOT2._ros2_movej,
        DSR_ROBOT2._ros2_movel,
        # TODO: 추가로 사용하는 함수의 서비스 클라이언트
    )

    # ===== 도구 / TCP 설정 =====
    set_tool("")   # TODO: 도구 무게 이름
    set_tcp("")    # TODO: TCP 이름

    # ===== 포즈 정의 =====
    homej = posj([0.0, 0.0, 90.0, 0.0, 90.0, 0.0])
    # TODO: posj1 = posj([J1, J2, J3, J4, J5, J6])
    # TODO: posx1 = posx([X, Y, Z, A, B, C])

    # ===== 종료 시 정리 동작 (예외/Ctrl+C 에도 실행) =====
    app.on_shutdown("Failed to move home", lambda: movej(homej, vel=VELOCITY, acc=ACC))

    movej(homej, vel=VELOCITY, acc=ACC)

    for i in range(CYCLE):
        node.get_logger().info(f"Cycle {i+1}")

        # ===== 동작 작성 =====
        # TODO: movej(posj1, vel=VELOCITY, acc=ACC)
        # TODO: movel(posx1, vel=VELOCITY, acc=ACC)
        pass


if __name__ == "__main__":
    main()
