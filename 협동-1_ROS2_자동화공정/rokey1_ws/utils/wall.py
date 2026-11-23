"""벽 4점(DB wall / Process goal.wall) → 벽 법선. 퍼티·샌딩 공용 (ROS / 로봇 없이 테스트 가능)."""
import math


def wall_normal(wall):
    """[top, floor, right, left] xyz 12개 → 벽 법선 (벽 → 로봇 방향, 단위벡터 [x, y, z]).
    (오른쪽-왼쪽) × (위-바닥), 로봇 베이스(원점) 쪽을 향하게 부호를 맞춘다.
    4점이 없거나(전부 0 포함) 한 줄 위에 있으면 None."""
    w = [float(v) for v in wall]
    if len(w) != 12:
        return None
    top, floor, right, left = w[0:3], w[3:6], w[6:9], w[9:12]
    a = [right[i] - left[i] for i in range(3)]
    b = [top[i] - floor[i] for i in range(3)]
    n = [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
    length = math.sqrt(sum(v * v for v in n))
    if length < 1e-6:
        return None
    center = [(top[i] + floor[i] + right[i] + left[i]) / 4.0 for i in range(3)]
    if sum(n[i] * center[i] for i in range(3)) > 0:     # 원점(로봇)에서 벽 쪽을 향하면 뒤집기
        length = -length
    return [v / length for v in n]
