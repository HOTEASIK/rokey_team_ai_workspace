"""퍼티 경로의 공통 좌표·회전 계산. 단독 실행 노드가 아니다."""

# ===== 파일 역할 / 통신 안내 =====
# 내부 공통 모듈: ROS 노드를 생성하거나 서비스 요청을 전송하지 않는다.
# 입력: BASE 좌표, ZYZ 각도, Tool 오프셋. 출력: 회전축/ABC/날 기준점 및 TCP 경로.
# 위치 단위 mm, 각도 단위 degree. Tool 오프셋은 회전해서 BASE 좌표로 변환한다.
# 날 기준점 = TCP + 회전된 Tool 오프셋이므로, TCP = 날 기준점 - 회전된 오프셋이다.

import math

# 헤라 방향 계산의 기준 자세(TCP ABC, deg).
# 뜻: TCP가 이 자세일 때 쥔 헤라가 '퍼티 면 = BASE +X, 날 선 = BASE Y, 손잡이 = 위(+Z)'가 된다.
# 도포 경로(apply_putty_plane.surface_frame)는 이 관계를 이용해 원하는 헤라 방향 → TCP 자세를 계산한다.
#
# 2026-11-19 보관대를 로봇 뒤로 옮기면서 헤라가 반대로 쥐어져, 집기 자세(pickup_spatula.PICKUP_ABC)에서
# 퍼티 면이 -X를 본다. 그래서 집기 자세를 BASE Z축으로 180° 돌린 자세가 기준이다(ZYZ에서 A+180°).
# 결과적으로 도포 자세는 예전보다 손목(J6)이 약 180° 돌아간 자세가 되고, 퍼티 면이 +X 벽을 향한다.
BLADE_REFERENCE_ABC = (136.31+180.0-360.0, -179.38, 136.56)   # = (-43.69, -179.38, 136.56)

BLADE_WIDTH, BLADE_THICKNESS = 100.0, 6.0   # 헤라 날 폭·두께 mm (실측 전 가정값)
SAMPLES = 21                # 각도 변화 도포 경로의 자세 개수
from utils.robot import ROBOT_ID, TCP_NAME  # config/robot.yaml robot 섹션
EXPECTED_TCP = TCP_NAME
TRANSFER_Z = 245.14         # 공용 이송 높이(TCP Z) mm


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def zyz_axes(abc):
    """두산 ZYZ 자세를 BASE에서 표현한 Tool X/Y/Z 축으로 변환한다."""
    a, b, c = map(math.radians, abc)
    ca, sa, cb, sb, cc, sc = (math.cos(a), math.sin(a), math.cos(b),
                             math.sin(b), math.cos(c), math.sin(c))
    return ((ca*cb*cc-sa*sc, sa*cb*cc+ca*sc, -sb*cc),
            (-ca*cb*sc-sa*cc, -sa*cb*sc+ca*cc, sb*sc),
            (ca*sb, sa*sb, cb))


def rotate_base_y(vector, degrees):
    angle = math.radians(degrees)
    x, y, z = vector
    return (math.cos(angle)*x + math.sin(angle)*z,
            y, -math.sin(angle)*x + math.cos(angle)*z)


def axes_to_zyz(axes):
    x, y, z = axes
    b = math.acos(max(-1.0, min(1.0, z[2])))
    if abs(math.sin(b)) < 1e-8:
        # Euler 표현만의 경계: 같은 회전을 C=0인 표현으로 반환한다.
        a = math.atan2(x[1], x[0]) if z[2] > 0 else math.atan2(-x[1], -x[0])
        return (math.degrees(a), math.degrees(b), 0.0)
    return tuple(map(math.degrees, (math.atan2(z[1], z[0]), b,
                                    math.atan2(y[2], -x[2]))))


def format_values(values):
    return ', '.join(f'{value:.2f}' for value in values)


def orientation_error(actual, target):
    # ZYZ Euler 표현의 ±180도 경계를 회전행렬로 비교한다.
    def matrix(angles):
        a, b, c = map(math.radians, angles)
        ca, sa, cb, sb, cc, sc = (math.cos(a), math.sin(a), math.cos(b),
                                 math.sin(b), math.cos(c), math.sin(c))
        return (ca*cb*cc-sa*sc, -ca*cb*sc-sa*cc, ca*sb,
                sa*cb*cc+ca*sc, -sa*cb*sc+ca*cc, sa*sb,
                -sb*cc, sb*sc, cb)
    cosine = (sum(a*b for a, b in zip(matrix(actual), matrix(target))) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
