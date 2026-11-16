#!/usr/bin/env python3
"""M0609 를 홈 자세(관절)로 이동시키는 노드.

[동작]
  1) 현재 관절 각도 조회 (get_current_posj)
  2) 이미 홈이면 아무것도 하지 않고 종료
  3) 아니면 move_joint(동기) 로 홈 자세 이동 → 도착 확인
  가상(에뮬레이터) / 실제 로봇 모두 같은 DSR 서비스를 사용한다.

[실행]
  ros2 run rokey1 homeset
  ros2 run rokey1 homeset --ros-args -p vel:=20.0 -p acc:=20.0
  ros2 run rokey1 homeset --ros-args -p home:="0,0,90,0,90,0" -p mode:=real

[파라미터]
  robot_id  (dsr01)            DSR 네임스페이스
  mode      (virtual)          virtual | real  (로그 표시용, 동작은 같음)
  home      (0,0,90,0,90,0)    홈 관절 각도 J1..J6 [deg]
  vel       (30.0)             관절 속도 [deg/s]  (HMI 속도 200% 에서 최대 60)
  acc       (30.0)             관절 가속도 [deg/s^2]
  tolerance (0.5)              홈 도착 판정 허용 오차 [deg]
  timeout   (10.0)             서비스 대기 시간 [s]
"""
import math
import sys

import rclpy
from rclpy.node import Node

DEFAULT_HOME = '0,0,90,0,90,0'
MAX_VEL = 90.0      # 안전 상한 [deg/s]
MAX_ACC = 90.0      # 안전 상한 [deg/s^2]
MOVE_TIMEOUT = 120.0


def parse_home(text):
    vals = [float(v) for v in str(text).replace('[', '').replace(']', '').split(',') if v.strip()]
    if len(vals) != 6 or not all(map(math.isfinite, vals)):
        raise ValueError(f'home 은 관절 6개 값이어야 합니다: {text}')
    return vals


class HomeSet(Node):
    def __init__(self):
        super().__init__('homeset')
        self.robot_id = self.declare_parameter('robot_id', 'dsr01').value
        self.mode = self.declare_parameter('mode', 'virtual').value
        self.home = parse_home(self.declare_parameter('home', DEFAULT_HOME).value)
        self.vel = float(self.declare_parameter('vel', 30.0).value)
        self.acc = float(self.declare_parameter('acc', 30.0).value)
        self.tol = float(self.declare_parameter('tolerance', 0.5).value)
        self.timeout = float(self.declare_parameter('timeout', 10.0).value)

        if self.vel > MAX_VEL or self.acc > MAX_ACC:
            self.get_logger().warn(f'속도/가속도가 상한을 넘어 제한합니다 (vel≤{MAX_VEL}, acc≤{MAX_ACC})')
        self.vel = min(max(self.vel, 1.0), MAX_VEL)
        self.acc = min(max(self.acc, 1.0), MAX_ACC)

        from dsr_msgs2.srv import GetCurrentPosj, MoveJoint
        self.GetCurrentPosj, self.MoveJoint = GetCurrentPosj, MoveJoint
        ctrl = f'/{self.robot_id}/dsr_controller2'
        self.posj_cli = self.create_client(GetCurrentPosj, f'{ctrl}/aux_control/get_current_posj')
        self.move_cli = self.create_client(MoveJoint, f'{ctrl}/motion/move_joint')

    # ------------------------------------------------------------------ 서비스
    def _wait_services(self):
        for cli in (self.posj_cli, self.move_cli):
            if not cli.wait_for_service(timeout_sec=self.timeout):
                raise RuntimeError(f'서비스 없음: {cli.srv_name} (dsr_bringup2 실행 여부 확인)')

    def _call(self, cli, req, timeout, what):
        future = cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise RuntimeError(f'{what} 응답 시간 초과 ({timeout:.0f}s)')
        res = future.result()
        if res is None:
            raise RuntimeError(f'{what} 응답 없음')
        return res

    def read_joints(self):
        res = self._call(self.posj_cli, self.GetCurrentPosj.Request(), 5.0, '관절 조회')
        pos = list(res.pos)
        if not res.success or len(pos) != 6 or not all(map(math.isfinite, pos)):
            raise RuntimeError('현재 관절 자세를 읽지 못했습니다.')
        return pos

    def at_home(self, pos):
        return all(abs(a - b) <= self.tol for a, b in zip(pos, self.home))

    # ------------------------------------------------------------------ 실행
    def run(self):
        log = self.get_logger()
        log.info(f'홈 이동 시작 · 모드={self.mode} · 목표 {self.home} · vel={self.vel:.1f} acc={self.acc:.1f}')
        self._wait_services()

        pos = self.read_joints()
        log.info(f'현재 관절: {[round(v, 2) for v in pos]}')
        if self.at_home(pos):
            log.info('이미 홈 위치입니다.')
            return

        req = self.MoveJoint.Request()
        req.pos = [float(v) for v in self.home]
        req.vel = self.vel
        req.acc = self.acc
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0         # 절대 좌표
        req.blend_type = 0
        req.sync_type = 0    # 이동이 끝난 뒤 응답
        res = self._call(self.move_cli, req, MOVE_TIMEOUT, '홈 이동')
        if not res.success:
            raise RuntimeError('move_joint 실패 (컨트롤러 상태/알람 확인)')

        pos = self.read_joints()
        if not self.at_home(pos):
            raise RuntimeError(f'홈 도착을 확인하지 못했습니다: {[round(v, 2) for v in pos]}')
        log.info('홈 위치 도착 완료')


def main(args=None):
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = HomeSet()
        node.run()
    except KeyboardInterrupt:
        code = 130
    except ImportError as e:
        print(f'[ERROR] dsr_msgs2 를 불러오지 못했습니다: {e}', flush=True)
        code = 1
    except Exception as e:  # noqa: BLE001 - 사용자에게 원인 표시
        if node is not None:
            node.get_logger().error(f'홈 이동 실패: {e}')
        else:
            print(f'[ERROR] 홈 이동 실패: {e}', flush=True)
        code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == '__main__':
    sys.exit(main())
