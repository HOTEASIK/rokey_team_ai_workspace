"""퍼티 경로의 공통 좌표·회전 계산. 단독 실행 노드가 아니다."""

# ===== 파일 역할 / 통신 안내 =====
# 내부 공통 모듈: ROS 노드를 생성하거나 서비스 요청을 전송하지 않는다.
# 입력: BASE 좌표, ZYZ 각도, Tool 오프셋. 출력: 회전축/ABC/날 기준점 및 TCP 경로.
# 위치 단위 mm, 각도 단위 degree. Tool 오프셋은 회전해서 BASE 좌표로 변환한다.
# 날 기준점 = TCP + 회전된 Tool 오프셋이므로, TCP = 날 기준점 - 회전된 오프셋이다.

import math

from .pickup_spatula import PICKUP_ABC

START_XY = (695.42, -223.46)
END_XY = (574.26, -227.09)
START_TILT, END_TILT = 45.0, 30.0
UP = (0.0, 0.0, 1.0)
BLADE_WIDTH, BLADE_LENGTH, BLADE_THICKNESS = 100.0, 70.0, 6.0
HANDLE_RADIUS = 4.0
SURFACE_Z = 35.0
EDGE_OFFSET = (0.0, 0.0, 126.0)
SAMPLES = 21
ROBOT_ID = 'dsr01'
EXPECTED_TCP = 'GripperDA_v1'
TRANSFER_Z = 245.14


def translate(point, direction, distance):
    return tuple(p + d * distance for p, d in zip(point, direction))


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


def build_stroke(surface_z, edge_offset, samples, pickup_abc=PICKUP_ABC, normal_x=-1.0):
    """픽업 면 법선 BASE ±X를 Y축 회전으로 -Z 쪽으로 눕힌다."""
    length = math.dist(START_XY, END_XY)
    travel = (*((end - start) / length for start, end in zip(START_XY, END_XY)), 0.0)
    initial_axes = zyz_axes(pickup_abc)
    poses = []
    for i in range(samples):
        u = i / (samples - 1)
        progress = u * u * (3 - 2 * u)
        tilt = START_TILT - (START_TILT - END_TILT) * progress ** 1.6
        # 면의 초기 방향에 맞춰 BASE Y축으로 회전한다.
        rotation = normal_x * (90.0 - tilt)
        axes = tuple(rotate_base_y(axis, rotation) for axis in initial_axes)
        handle = rotate_base_y(UP, rotation)
        width = (0.0, 1.0, 0.0)
        normal = rotate_base_y((normal_x, 0.0, 0.0), rotation)
        edge = translate((*START_XY, surface_z), travel, length * progress)
        tcp = edge
        for axis, offset in zip(axes, edge_offset):
            tcp = translate(tcp, axis, -offset)
        poses.append((edge, tcp, axes_to_zyz(axes), tilt, handle, width, normal))
    return poses


def format_values(values):
    return ', '.join(f'{value:.2f}' for value in values)


# 접근→도포→후퇴 순서의 (설명, TCP XYZABC, 속도)를 yield한다. 계산만 수행한다.
def application_moves(current, poses, transfer_z, transfer_speed, apply_speed):
    """현재 위치에서 상승, 자세 변경, 접근, 도포 후 수직 후퇴한다."""
    height = max(transfer_z, current[2], *(pose[1][2] for pose in poses))
    _, start, abc, *_ = poses[0]
    yield '이송 높이로 상승', (*current[:2], height, *current[3:]), transfer_speed
    yield '픽업 자세에서 BASE Y축으로 눕히기', (*current[:2], height, *abc), transfer_speed
    yield '도포 시작점 위로 이동', (*start[:2], height, *abc), transfer_speed
    yield '시험판에 접근', (*start, *abc), apply_speed
    for index, (_, tcp, angles, *_rest) in enumerate(poses[1:], 2):
        yield f'도포 {index}/{len(poses)}', (*tcp, *angles), apply_speed
    _, end, abc, *_ = poses[-1]
    yield '도포 후 수직 상승', (*end[:2], height, *abc), apply_speed


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
