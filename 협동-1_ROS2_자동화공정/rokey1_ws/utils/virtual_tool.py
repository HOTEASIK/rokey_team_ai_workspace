"""가상(에뮬레이터) 로봇에 TCP 등록·선택 (virtual 전용).

실제 로봇은 티칭 펜던트에 TCP 'GripperDA_v1' 이 등록돼 있지만, 에뮬레이터는 새로 켤 때마다 비어 있다
→ 퍼티 집기(pickup_spatula)가 "TCP GripperDA_v1 적용 서비스 실패" 로 멈춤 (E_GRIP).
그래서 virtual 공정 시작 전에 같은 이름으로 등록하고 선택해 둔다.

  값: robot.yaml virtual.tcp_pos (에뮬레이터용 TCP 오프셋). 실물 GripperDA_v1 과 같아야 한다:
      오프셋이 다르면 같은 TCP 목표에 대한 관절값(ikin)이 달라져 특이점 검사에 걸린다
      (2026-11-21 오프셋 0 → 퍼티 뜨기 J5 7.6° 로 E_SINGULAR). 실측 관절·TCP 쌍 2개에서 역산 ≈ (0, 0, 203.5).
  순서: 컨트롤러가 가상(robot_system=1)인지 확인 → 아니면 거부 (실물 로봇 설정은 절대 안 건드림)
        → 현재 TCP 조회 → 이미 같은 이름이고 force=False 면 아무것도 안 함
        → 수동 모드 → (같은 이름이 있으면) config_delete_tcp → config_create_tcp(오프셋)
        → set_current_tcp → 자동 모드 → 다시 조회해서 확인
        (등록된 TCP 의 오프셋은 조회할 수 없어서, 값을 확실히 맞추려면 force=True 로 지우고 다시 등록)
  로봇은 움직이지 않는다.
  ※ Tool 은 다루지 않는다: 에뮬레이터는 Tool 이 없을 때 get_current_tool 에 응답하지 않고,
    그동안 컨트롤러의 다른 서비스 응답까지 늦어진다 (2026-11-21 확인).

  HMI: 가상 모드 [공정] 시작 때 자동 실행 (hmi_server)
  단독: ros2 run rokey1 virtual_tool          (bringup mode:=virtual 이 떠 있어야 함, 항상 다시 등록)
"""
import time

ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS = 0, 1   # DRFC.py
ROBOT_SYSTEM_VIRTUAL = 1                          # get_robot_system: 0 실물, 1 가상


def _call(node, client, request, label, timeout=5.0, spin=True):
    """서비스 호출. spin=True: 여기서 직접 spin (단독 실행) / False: 다른 스레드의 executor 가 spin 중 (HMI).
    (node.executor 로 판단하면 안 됨: spin_until_future_complete 한 번 뒤에는 전역 executor 가 붙어 있음)"""
    import rclpy
    if not client.wait_for_service(timeout_sec=3.0):
        raise RuntimeError(f'{label}: 서비스 {client.srv_name} 없음 (bringup 확인)')
    future = client.call_async(request)
    t0 = time.time()
    while not future.done() and time.time() - t0 < timeout:
        if spin:
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.1)
        else:
            time.sleep(0.02)
    if not future.done():
        raise RuntimeError(f'{label}: 응답 없음 ({timeout:.0f}s)')
    return future.result()


def make_clients(node, robot_id, callback_group=None):
    from dsr_msgs2.srv import (ConfigCreateTcp, ConfigDeleteTcp, GetCurrentTcp, GetRobotSystem, SetCurrentTcp,
                               SetRobotMode)
    ctrl = f'/{robot_id}/dsr_controller2'
    return {key: node.create_client(srv, f'{ctrl}/{name}', callback_group=callback_group)
            for key, srv, name in (('system', GetRobotSystem, 'system/get_robot_system'),
                                   ('get_tcp', GetCurrentTcp, 'tcp/get_current_tcp'),
                                   ('new_tcp', ConfigCreateTcp, 'tcp/config_create_tcp'),
                                   ('del_tcp', ConfigDeleteTcp, 'tcp/config_delete_tcp'),
                                   ('set_tcp', SetCurrentTcp, 'tcp/set_current_tcp'),
                                   ('mode', SetRobotMode, 'system/set_robot_mode'))}


def ensure_virtual_tcp(node, cfg, log=print, clients=None, spin=True, force=False):
    """에뮬레이터에 robot.yaml robot.tcp 를 등록·선택. 반환: 바꿨으면 True, 이미 맞았으면 False.
    가상 컨트롤러가 아니거나 실패하면 RuntimeError. clients: make_clients() 결과 (없으면 여기서 만듦).
    spin=False: node 를 다른 스레드 executor 가 이미 spin 중 (hmi_server).
    force=True: 이름이 같아도 지우고 robot.yaml 오프셋으로 다시 등록."""
    from dsr_msgs2.srv import (ConfigCreateTcp, ConfigDeleteTcp, GetCurrentTcp, GetRobotSystem, SetCurrentTcp,
                               SetRobotMode)
    robot = cfg['robot']
    tcp_name = robot['tcp']
    pos = [float(v) for v in (cfg.get('virtual') or {}).get('tcp_pos') or [0.0] * 6]
    if len(pos) != 6:
        raise RuntimeError(f'robot.yaml virtual.tcp_pos 는 6개 [x,y,z,a,b,c]: {pos}')
    c = clients or make_clients(node, robot['id'])

    system = _call(node, c['system'], GetRobotSystem.Request(), '컨트롤러 종류 조회', spin=spin).robot_system
    if system != ROBOT_SYSTEM_VIRTUAL:
        raise RuntimeError(f'연결된 컨트롤러가 가상이 아님 (robot_system={system}) → 가상 TCP 등록 안 함. '
                           'bringup 모드를 확인하세요.')

    def current():
        return _call(node, c['get_tcp'], GetCurrentTcp.Request(), '현재 TCP 조회', spin=spin).info.strip()

    tcp = current()
    if tcp == tcp_name and not force:
        log(f'[가상 TCP] 이미 {tcp_name!r}')
        return False
    log(f'[가상 TCP] 현재 {tcp!r} → {tcp_name!r} 오프셋 {pos} 로 등록 · 선택')
    # 자동 모드에서는 TCP 변경이 거부됨 → 수동 → 등록·선택 → 자동 (putty_motion / sanding_job 과 같은 순서)
    _call(node, c['mode'], SetRobotMode.Request(robot_mode=ROBOT_MODE_MANUAL), '수동 모드', spin=spin)
    time.sleep(0.5)
    try:
        # 같은 이름이 이미 있으면(오프셋이 다를 수 있음) 지우고 다시 등록. 없으면 삭제는 실패해도 됨
        _call(node, c['del_tcp'], ConfigDeleteTcp.Request(name=tcp_name), 'TCP 삭제', spin=spin)
        if not _call(node, c['new_tcp'], ConfigCreateTcp.Request(name=tcp_name, pos=pos), 'TCP 등록',
                     spin=spin).success:
            raise RuntimeError(f'TCP {tcp_name!r} 등록 실패 (오프셋 {pos})')
        _call(node, c['set_tcp'], SetCurrentTcp.Request(name=tcp_name), 'TCP 선택', spin=spin)
    finally:
        _call(node, c['mode'], SetRobotMode.Request(robot_mode=ROBOT_MODE_AUTONOMOUS), '자동 모드', spin=spin)
        time.sleep(0.5)
    tcp = current()
    if tcp != tcp_name:
        raise RuntimeError(f'가상 TCP 선택 실패: 지금 {tcp!r} (필요: {tcp_name!r})')
    log(f'[가상 TCP] 완료: {tcp!r}')
    return True


def main(args=None):
    import rclpy
    from utils.config import load
    rclpy.init(args=args)
    node = rclpy.create_node('virtual_tool')
    try:
        ensure_virtual_tcp(node, load(), log=lambda m: node.get_logger().info(m), force=True)
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
