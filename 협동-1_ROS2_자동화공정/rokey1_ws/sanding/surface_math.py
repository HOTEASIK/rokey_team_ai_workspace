"""샌딩 경로 계산용 순수 수학 함수 (ROS / 로봇 없이 테스트 가능)

좌표: 베이스 좌표계, 위치 mm, 자세는 두산 posx 기본값인 Euler ZYZ (deg).
툴 Z 축 = TCP 가 향하는 방향 = 표면을 누르는 방향.
  바닥 : 툴 Z = 베이스 -Z (아래)
  벽   : 툴 Z = 벽 쪽 수평 방향
"""
import math

EPS = 1e-9


# ── 벡터 ──────────────────────────────────────────────────────────
def add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def scale(a, s):
    return [a[0] * s, a[1] * s, a[2] * s]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    n = norm(a)
    if n < EPS:
        raise ValueError("길이가 0인 벡터는 정규화할 수 없습니다.")
    return scale(a, 1.0 / n)


# ── 회전 ──────────────────────────────────────────────────────────
def zyz_to_matrix(euler_deg):
    """Euler ZYZ (deg) → 3x3 회전 행렬 (행 리스트). 열 i = 툴 축 i 의 베이스 방향."""
    a, b, c = (math.radians(v) for v in euler_deg)
    ca, sa = math.cos(a), math.sin(a)
    cb, sb = math.cos(b), math.sin(b)
    cc, sc = math.cos(c), math.sin(c)
    return [
        [ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
        [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
        [-sb * cc, sb * sc, cb],
    ]


def matrix_to_zyz(m):
    """3x3 회전 행렬 → Euler ZYZ (deg)."""
    r13, r23, r33 = m[0][2], m[1][2], m[2][2]
    sb = math.hypot(r13, r23)
    b = math.atan2(sb, r33)
    if sb > 1e-6:
        a = math.atan2(r23, r13)
        c = math.atan2(m[2][1], -m[2][0])
    elif r33 > 0:  # b ≈ 0 : a + c 만 의미 있음
        a = math.atan2(m[1][0], m[0][0])
        c = 0.0
    else:          # b ≈ 180 : a - c 만 의미 있음
        a = math.atan2(-m[0][1], -m[0][0])
        c = 0.0
    return [math.degrees(a), math.degrees(b), math.degrees(c)]


def column(m, i):
    return [m[0][i], m[1][i], m[2][i]]


def tool_axes(euler_deg):
    """(툴 X, 툴 Y, 툴 Z) 의 베이스 기준 단위 벡터"""
    m = zyz_to_matrix(euler_deg)
    return column(m, 0), column(m, 1), column(m, 2)


def _axis_angle_matrix(axis, ang):
    x, y, z = unit(axis)
    c, s = math.cos(ang), math.sin(ang)
    t = 1 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def _matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def orient_toward_surface(euler_deg, surface_normal):
    """툴 Z 가 표면 쪽(-surface_normal)을 향하도록 자세를 최소 회전으로 바꿈.

    surface_normal : 표면에서 로봇 쪽으로 나오는 법선 (크기 무관)
      바닥            → [0, 0, 1]
      +X 쪽 앞 벽     → [-1, 0, 0]
      -Y 쪽 옆 벽     → [0, 1, 0]
    원래 자세(euler_deg)에서 최소한으로만 돌리므로 그리퍼 롤은 최대한 유지됩니다.
    """
    r = zyz_to_matrix(euler_deg)
    z_old = column(r, 2)
    z_new = scale(unit(surface_normal), -1.0)
    axis = cross(z_old, z_new)
    s, c = norm(axis), dot(z_old, z_new)
    if s < 1e-9:
        if c > 0:  # 이미 같은 방향
            return list(euler_deg)
        axis = column(r, 0)  # 정반대: 툴 X 축 기준 180도
    rot = _axis_angle_matrix(axis, math.atan2(s, c))
    return matrix_to_zyz(_matmul(rot, r))


# ── 샌딩 경로 ─────────────────────────────────────────────────────
def project_to_plane(p, plane_point, n):
    """점 p 를 (plane_point, 법선 n) 평면에 투영"""
    return sub(p, scale(n, dot(sub(p, plane_point), n)))


def path_frame(start, end, tool_z, tool_x):
    """표면 위 진행 방향 u, 좌우 방향 v (둘 다 표면 평면 안의 단위 벡터)와 거리.

    start / end 는 이미 같은 평면 위에 있어야 합니다.
    start == end 이면 진행 방향은 툴 X 를 평면에 투영한 방향.
    """
    d = sub(end, start)
    length = norm(d)
    if length > 1e-3:
        u = scale(d, 1.0 / length)
    else:
        u = unit(project_to_plane(tool_x, [0, 0, 0], tool_z))
        length = 0.0
    v = unit(cross(tool_z, u))
    return u, v, length


def orbit_centers(start, end, u, length, step):
    """start → end 를 step 간격으로 나눈 원 중심점들 (start, end 포함)"""
    n = max(1, int(math.ceil(length / step - 1e-6))) if length > 0 else 0
    return [add(start, scale(u, length * i / n)) for i in range(n + 1)] if n else [list(start)]


def orbit_points(center, u, v, radius, turns, points_per_turn):
    """center 주위 원 경로 점들. 첫 점(각도 0)은 포함하지 않고 마지막에 다시 각도 0 으로 돌아옴."""
    pts = []
    total = turns * points_per_turn
    for i in range(1, total + 1):
        a = 2.0 * math.pi * i / points_per_turn
        pts.append(add(center, add(scale(u, radius * math.cos(a)), scale(v, radius * math.sin(a)))))
    return pts
