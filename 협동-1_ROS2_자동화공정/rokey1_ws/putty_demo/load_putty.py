"""판 접촉 TCP 자세를 유지하며 BASE -Z 힘 / +X 위치로 퍼티를 묻힌다.

별도 작업: scoop_putty는 변경하지 않는다. --transfer-z 지정 시 상승·자세 정렬·XY 이송·판 위 하강 후 실행한다.
좌표는 헤라 날이 닿았을 때 GripperDA_v1의 BASE TCP 자세이며 길이 보정하지 않는다.
"""

import argparse
import json
import math
import sys
import time
from types import SimpleNamespace

from .contact_search import checked_force, seek_axis
from .putty_geometry import EXPECTED_TCP, ROBOT_ID, orientation_error
from .singularity_guard import start_singularity_watch

# ===== 하이퍼파라미터 =====
CONTACT_POSE = (-431.88, -1.52, 123.68, 8.37, 178.03, 7.62)
# ===== 로봇 초기화 설정 =====
# GripperDA_v1 / BASE. 이송 높이는 운영자 지정. 충돌 회피 경로 자동 계획은 아니다.


def main():
    try:
        args = make_parser().parse_args()
        validate(args)
        run_robot(args)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f'판 퍼티 묻히기 중단: {exc}', file=sys.stderr, flush=True)
        return 1


def make_parser():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--execute-motion', action='store_true', required=True)
    parser.add_argument('--contact-pose', nargs=6, type=float, default=CONTACT_POSE)
    # 힘/속도/거리/강성은 현장에서 결정: 임의 운영 기본값 없이 필수 입력.
    for name in ('approach-height', 'distance', 'search-distance', 'search-speed',
                 'contact-force', 'hold-force', 'force-limit', 'force-tolerance',
                 'force-wait-timeout', 'force-stable-time', 'force-loss-timeout',
                 'scrape-speed', 'lift', 'lift-speed', 'acceleration'):
        parser.add_argument('--'+name, type=float, required=True)
    parser.add_argument('--joint-speed', type=float, help='이송 관절 속도 deg/s, 자동 이송 시 필수')
    parser.add_argument('--joint-acceleration', type=float, help='이송 관절 가속도 deg/s², 자동 이송 시 필수')
    parser.add_argument('--transfer-z', type=float, help='자동 이송 BASE TCP 높이 mm. 미지정 시 판 위 준비 위치 필요')
    parser.add_argument('--transfer-speed', type=float, help='자동 이송 속도 mm/s. transfer-z와 함께 필수')
    parser.add_argument('--force-stiffness', nargs=6, type=float, required=True)
    return parser


def validate(args):
    numbers = [v for k, v in vars(args).items() if isinstance(v, (int, float)) and not isinstance(v, bool)]
    if not all(math.isfinite(v) and v > 0 for v in numbers):
        raise ValueError('거리·속도·힘·감시 시간은 양의 유한한 값이어야 합니다.')
    if (args.transfer_z is None) != (args.transfer_speed is None):
        raise ValueError('자동 이송은 transfer-z와 transfer-speed를 함께 지정하세요.')
    if args.transfer_z is not None and (args.joint_speed is None or args.joint_acceleration is None):
        raise ValueError('자동 관절 이송은 --joint-speed/--joint-acceleration이 필요합니다.')
    if args.transfer_z is not None and args.transfer_z <= approach_pose(args)[2]:
        raise ValueError('이송 TCP 높이는 판 위 접근 TCP 높이보다 높아야 합니다.')
    if len(args.contact_pose) != 6 or not all(map(math.isfinite, args.contact_pose)):
        raise ValueError('접촉 TCP XYZABC 6개가 필요합니다.')
    if not all(math.isfinite(v) and v > 0 for v in args.force_stiffness):
        raise ValueError('강성 6개는 양의 유한한 값이어야 합니다.')
    if not 30 <= args.distance <= 40:
        raise ValueError('이번 판 긁기는 +X 30~40 mm만 허용합니다.')
    if not args.approach_height < args.search_distance <= 20:
        raise ValueError('접근 높이 < 최대 탐색 거리 <= 20 mm가 필요합니다.')
    if not args.force_tolerance < args.hold_force or not args.hold_force+args.force_tolerance < args.force_limit:
        raise ValueError('허용오차 < 유지 힘, 유지 힘+허용오차 < 힘 상한이어야 합니다.')
    if args.contact_force >= args.force_limit or args.force_stable_time >= args.force_wait_timeout:
        raise ValueError('접촉 힘/안정 시간이 감시 한도보다 작아야 합니다.')


def approach_pose(args):
    p = list(args.contact_pose)
    p[2] += args.approach_height
    return tuple(p)


def clockwise_xy_waypoint(start, target, offset):
    """시작-목표 직선의 BASE -Z 쪽으로 부풀린 XY 경유점."""
    dx, dy = target[0]-start[0], target[1]-start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-6:
        raise ValueError('시계방향 이송을 위한 XY 이동 거리가 없습니다.')
    x = (start[0]+target[0])*0.5 - dy/length*offset
    y = (start[1]+target[1])*0.5 + dx/length*offset
    return (x, y, target[2], *target[3:])


def execute(io, args):
    """IO 주입 가능한 작업 순서. Z는 힘, X는 위치. 센서 값으로 출발/유지 판정한다."""
    actual = tuple(io.read_pose())
    above = approach_pose(args)
    if math.dist(actual[:3], above[:3]) > 2 or orientation_error(actual[3:], above[3:]) > 2:
        if args.transfer_z is None:
            raise ValueError(f'판 위 준비 위치에 먼저 배치하거나 --transfer-z/--transfer-speed를 지정하세요: TCP {above}.')
    moved = False
    compliance = False
    try:
        if args.transfer_z is not None:
            height = max(actual[2], args.transfer_z)
            initial_joints = io.read_joints()
            moved = True
            io.log('판 이송 전 수직 상승 (MoveLine)', actual, checked_force(io.read_force()))
            io.move_line((*actual[:2], height, *actual[3:]), args.transfer_speed)
            raised_joints = io.read_joints()
            target = (*above[:2], height, *above[3:])
            waypoint = clockwise_xy_waypoint(actual, target, 100.0)
            io.log('판 이송 시계방향 경유점 (MoveJoint)', waypoint, checked_force(io.read_force()))
            io.move_joint_pose(waypoint)
            io.log('판 위 목표점 관절 이송 (MoveJoint)', target, checked_force(io.read_force()))
            io.move_joint_pose(target)
            io.move_line(above, args.transfer_speed)
        moved = True  # 응답 실패 시에도 컨트롤러가 수신했을 가능성이 있어 정지한다.
        def search_force():
            values = checked_force(io.read_force())
            if abs(values[2]) > args.force_limit:
                raise RuntimeError('E_FORCE: 접촉 탐색 중 Fz 힘 상한 초과')
            return values
        search_io = SimpleNamespace(read_pose=io.read_pose, read_force=search_force,
                                    move_async=io.move_async, stop_and_wait=io.stop_and_wait)
        contact = seek_axis(search_io, 2, -1, args.contact_force, args.search_distance, args.search_speed)
        io.log('접촉 감지', contact, checked_force(io.read_force()))
        compliance = True
        io.compliance(args.force_stiffness)
        io.set_force(args.hold_force)
        io.log('힘 설정 응답 확인', io.read_pose(), checked_force(io.read_force()))
        stable_since = None
        stable_samples = 0
        sign = None
        deadline = time.monotonic()+args.force_wait_timeout
        while True:
            now = time.monotonic()
            p, f = tuple(io.read_pose()), checked_force(io.read_force())
            check_contact_bounds(p, contact, args)
            if abs(p[0]-contact[0]) > 2:
                raise RuntimeError('힘 형성 대기 중 X 위치 이탈')
            if abs(f[2]) > args.force_limit:
                raise RuntimeError('E_FORCE: BASE Fz 힘 상한 초과')
            in_band = abs(abs(f[2])-args.hold_force) <= args.force_tolerance
            current_sign = 1 if f[2] > 0 else -1
            if not in_band or sign != current_sign:
                stable_since, stable_samples = None, 0
            sign = current_sign
            if in_band:
                stable_since = now if stable_since is None else stable_since
                stable_samples += 1
            io.log('힘 형성 대기', p, f)
            if stable_since is not None and now-stable_since >= args.force_stable_time and stable_samples >= 3:
                break
            if now >= deadline:
                raise TimeoutError('E_FORCE: -Z 힘 형성 시간 초과. 긁기를 시작하지 않습니다.')
            if io.read_state() not in (1, 2):
                raise RuntimeError('E_FORCE: 힘 형성 중 로봇 상태 이상')
            time.sleep(0.05)
        io.log('힘 안정 확인 완료', p, f)
        start = p
        target = (start[0]+args.distance, start[1], start[2], *args.contact_pose[3:])
        io.move_async(target, args.scrape_speed)
        deadline = time.monotonic()+args.distance/args.scrape_speed*3+5
        lost = None
        while True:
            now = time.monotonic()
            p, f = tuple(io.read_pose()), checked_force(io.read_force())
            check_contact_bounds(p, start, args)
            in_band = abs(abs(f[2])-args.hold_force) <= args.force_tolerance
            if abs(f[2]) > args.force_limit or (in_band and (1 if f[2] > 0 else -1) != sign):
                raise RuntimeError('E_FORCE: 긁기 힘 상한 초과 또는 부호 반전')
            lost = None if in_band else now if lost is None else lost
            if lost is not None and now-lost >= args.force_loss_timeout:
                raise RuntimeError('E_FORCE: 긁기 중 유지 힘 이탈 지속')
            io.log('판 긁기 중', p, f)
            state = io.read_state()
            if state not in (1, 2):
                raise RuntimeError('긁기 중 로봇 상태 이상')
            if state == 1:
                if abs(p[0]-target[0]) <= 1 and abs(p[1]-target[1]) <= 1 and orientation_error(p[3:], target[3:]) <= 2:
                    break
            if not start[0]-2 <= p[0] <= target[0]+2 or now >= deadline:
                raise RuntimeError('긁기 X 범위/시간 초과')
            time.sleep(0.05)
        io.stop_and_wait()
        io.release()
        compliance = False
        p = tuple(io.read_pose())
        io.move_line((*p[:2], p[2]+args.lift, *args.contact_pose[3:]), args.lift_speed)
        if args.transfer_z is not None:
            p = tuple(io.read_pose())
            io.move_line((*p[:2], max(height, p[2]), *p[3:]), args.transfer_speed)
            io.log('시작 위치로 관절 복귀 (MoveJoint)', actual, checked_force(io.read_force()))
            io.move_joint(raised_joints)
            io.move_joint(initial_joints)

    finally:
        if moved:
            # 정지 실패 시에도 힘 해제를 시도하며, 정리 실패는 결과 성공으로 취급하지 않는다.
            errors = []
            try:
                io.stop_and_wait()
            except Exception as exc:
                errors.append(str(exc))
            if compliance:
                try:
                    io.release()
                except Exception as exc:
                    errors.append(str(exc))
            if errors:
                raise RuntimeError('정지/정리 확인 실패: '+'; '.join(errors))


def check_contact_bounds(p, origin, args):
    if abs(p[2]-origin[2]) > args.search_distance or abs(p[1]-origin[1]) > 2:
        raise RuntimeError('E_FORCE: 접촉 중 Z/Y 허용 이동 범위 초과')
    if orientation_error(p[3:], args.contact_pose[3:]) > 2:
        raise RuntimeError('접촉 중 고정 자세 이탈')


def run_robot(args):
    import rclpy
    from dsr_msgs2 import srv
    from utils.singularity import apply as apply_singularity
    from utils.shutdown import safe_shutdown
    rclpy.init(args=[])
    node = rclpy.create_node('load_putty', namespace=ROBOT_ID)
    watch = None
    cleaning = False
    names = {'joints': ('aux_control/get_current_posj', srv.GetCurrentPosj),
             'solution': ('aux_control/get_current_solution_space', srv.GetCurrentSolutionSpace),
             'ikin': ('motion/ikin', srv.Ikin),
             'joint': ('motion/move_joint', srv.MoveJoint),
             'pose': ('aux_control/get_current_posx', srv.GetCurrentPosx),
             'force': ('aux_control/get_tool_force', srv.GetToolForce),
             'state': ('system/get_robot_state', srv.GetRobotState),
             'tcp': ('tcp/get_current_tcp', srv.GetCurrentTcp),
             'move': ('motion/move_line', srv.MoveLine), 'stop': ('motion/move_stop', srv.MoveStop),
             'compliance': ('force/task_compliance_ctrl', srv.TaskComplianceCtrl),
             'desired': ('force/set_desired_force', srv.SetDesiredForce),
             'release': ('force/release_force', srv.ReleaseForce),
             'release_comp': ('force/release_compliance_ctrl', srv.ReleaseComplianceCtrl),
             'avoid': ('motion/set_singularity_handling', srv.SetSingularityHandling),
             'force_policy': ('motion/set_singular_handling_force', srv.SetSingularHandlingForce)}
    clients = {k: node.create_client(t, f'/{ROBOT_ID}/dsr_controller2/{path}') for k, (path, t) in names.items()}
    def call(key, timeout=2, **fields):
        if watch and not cleaning:
            watch.check()
        client = clients[key]
        if not client.wait_for_service(timeout_sec=timeout):
            raise TimeoutError(f'{key} 서비스 없음')
        future = client.call_async(names[key][1].Request(**fields))
        deadline = time.monotonic()+timeout
        while not future.done():
            if watch and not cleaning:
                watch.check()
            if time.monotonic() >= deadline:
                raise TimeoutError(f'{key} 서비스 시간 초과')
            rclpy.spin_until_future_complete(node, future, timeout_sec=0.05)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError(f'{key} 서비스 실패')
        return result
    def pose():
        p = tuple(call('pose', ref=0).task_pos_info[0].data[:6])
        if len(p) != 6 or not all(map(math.isfinite, p)):
            raise RuntimeError('유효한 TCP 조회 실패')
        return p
    def move(p, speed, sync_type):
        call('move', timeout=2 if sync_type else 30, pos=list(p), vel=[speed, speed],
             acc=[args.acceleration, args.acceleration], ref=0, mode=0, sync_type=sync_type)
        if not sync_type:
            actual = pose()
            if math.dist(actual[:3], p[:3]) > 2 or orientation_error(actual[3:], p[3:]) > 2:
                raise RuntimeError('직선 이동 목표 미도착')
    def joints():
        q = tuple(call('joints').pos)
        if len(q) != 6 or not all(map(math.isfinite, q)):
            raise RuntimeError('관절 조회 실패')
        return q
    def move_joint(q):
        if len(q) != 6 or not all(map(math.isfinite, q)):
            raise ValueError('유효한 목표 관절 6개가 필요합니다.')
        call('joint', timeout=60, pos=list(q), vel=args.joint_speed,
             acc=args.joint_acceleration, mode=0, sync_type=0)
        if max(abs(a-b) for a,b in zip(joints(), q)) > 1:
            raise RuntimeError('관절 이동 목표 미도착')
    def move_joint_pose(p):
        solution = call('solution').sol_space
        q = call('ikin', pos=list(p), sol_space=solution, ref=0).conv_posj
        move_joint(q)
        current = pose()
        if math.dist(current[:3], p[:3]) > 2 or orientation_error(current[3:], p[3:]) > 2:
            raise RuntimeError('판 위 관절 이송 TCP 목표 미도착')
    def stop():
        nonlocal cleaning
        previous, cleaning = cleaning, True
        try:
            call('stop', stop_mode=3)
            deadline = time.monotonic()+5
            while call('state').robot_state != 1:
                if time.monotonic() > deadline:
                    raise TimeoutError('정지 확인 실패')
                time.sleep(0.05)
        finally:
            cleaning = previous
    def release():
        nonlocal cleaning
        previous, cleaning = cleaning, True
        try:
            try:
                call('release', time=0.3)
            finally:
                call('release_comp')
        finally:
            cleaning = previous
    last_log = [0.]
    def log(stage, p, f):
        if time.monotonic()-last_log[0] >= 0.5 or stage != '판 긁기 중' and stage != '힘 형성 대기':
            print('[판힘진단] '+json.dumps(dict(stage=stage, tcp=list(p), force=list(f),
                  target_base_fz=-args.hold_force), ensure_ascii=False), flush=True)
            last_log[0] = time.monotonic()
    try:
        if call('tcp').info != EXPECTED_TCP:
            raise ValueError(f'{EXPECTED_TCP} TCP를 먼저 선택하세요.')
        if call('state').robot_state != 1:
            raise ValueError('STANDBY에서 시작해야 합니다.')
        watch = start_singularity_watch(ROBOT_ID)
        watch.wait_ready()
        apply_singularity(SimpleNamespace(set_singularity_handling=lambda mode: (call('avoid', mode=mode), 0)[1]))
        call('force_policy', mode=0)
        io = SimpleNamespace(read_joints=joints, move_joint=move_joint, move_joint_pose=move_joint_pose, read_pose=pose, read_force=lambda: call('force', ref=0).tool_force,
             read_state=lambda: call('state').robot_state, move_async=lambda p,s: move(p,s,1),
             move_line=lambda p,s: move(p,s,0), stop_and_wait=stop, release=release, log=log,
             compliance=lambda st: call('compliance', stx=list(st), ref=0, time=0.3),
             set_force=lambda f: call('desired', fd=[0.,0.,-f,0.,0.,0.], dir=[0,0,1,0,0,0], ref=0, mod=0, time=0.3))
        execute(io, args)
        print('판 퍼티 묻히기 완료. 퍼티 부착량 자동 판정은 하지 않습니다.', flush=True)
    finally:
        if watch:
            watch.close()
        safe_shutdown(node)


if __name__ == '__main__':
    raise SystemExit(main())
