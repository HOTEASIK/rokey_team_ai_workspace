import numpy as np
import random


# 1. 탐색 공간 설정

WIDTH = 0.505       # 가로 505mm = 0.505m
HEIGHT = 0.3      # 세로 300mm = 0.3m

CENTER = (0.0, 0.0)

# 직사각형의 벽 위치
WALLS = {
    "right": WIDTH / 2,
    "left": -WIDTH / 2,
    "top": HEIGHT / 2,
    "bottom": -HEIGHT / 2
}


# 2. 가상의 벽 접촉 좌표 생성

def create_wall_points(wall_name, wall_position):

    points = []

    # 벽을 따라 15개의 접촉 좌표 생성
    for i in range(15):

        # 벽을 따라 움직이는 좌표
        t = -0.12 + i * 0.24 / 14

        if wall_name == "right" or wall_name == "left":
            x = wall_position
            y = t

        else:
            x = t
            y = wall_position

        # 실제 센서 오차를 가정
        x += np.random.normal(0, 0.003)
        y += np.random.normal(0, 0.003)

        points.append([x, y])

    # 이상치 3개 추가
    for i in range(3):
        points.append([
            random.uniform(-0.3, 0.3),
            random.uniform(-0.2, 0.2)
        ])

    return np.array(points)


# 3. 두 점으로 직선 만들기

def make_line(p1, p2):

    x1, y1 = p1
    x2, y2 = p2

    a = y1 - y2
    b = x2 - x1
    c = x1 * y2 - x2 * y1

    norm = np.sqrt(a**2 + b**2)

    if norm < 1e-10:
        return None

    return np.array([a / norm, b / norm, c / norm])


# 4. 점과 직선 사이 거리 계산

def calculate_distance(points, line):

    a, b, c = line

    distances = np.abs(
        a * points[:, 0] +
        b * points[:, 1] + c
    )

    return distances


# 5. RANSAC 실행

def ransac(points, iterations=300, threshold=0.008):

    best_inliers = []

    for i in range(iterations):

        # 무작위로 두 점 선택
        p1, p2 = random.sample(list(points), 2)

        # 직선 생성
        line = make_line(p1, p2)

        if line is None:
            continue

        # 모든 점과 직선 사이 거리 계산
        distances = calculate_distance(points, line)

        # 직선에 가까운 점 선택
        inliers = np.where(distances < threshold)[0]

        # 가장 많은 점을 가진 직선 선택
        if len(inliers) > len(best_inliers):
            best_inliers = inliers

    if len(best_inliers) < 2:
        return None

    # 최종 Inlier들로 직선 다시 계산
    selected = points[best_inliers]

    center = np.mean(selected, axis=0)

    # PCA 방식으로 직선 방향 계산
    _, _, vh = np.linalg.svd(selected - center)

    direction = vh[0]

    normal = np.array([-direction[1], direction[0]])

    a, b = normal
    c = -(a * center[0] + b * center[1])

    return a, b, c, selected


# 6. 전체 탐색 및 벽 추정

def main():

    print(" 공간 탐색 시작")

    print(f"탐색 공간: {WIDTH}m x {HEIGHT}m")
    print(f"중심점: {CENTER}")

    for wall_name, wall_position in WALLS.items():

        print(f"\n[{wall_name}] 벽 탐색")

        # 가상의 접촉 좌표 생성
        points = create_wall_points(
            wall_name,
            wall_position
        )

        # RANSAC 실행
        result = ransac(points)

        if result is None:
            print("벽 추정 실패")
            continue

        a, b, c, inliers = result

        print(f"추정 직선: {a:.4f}x + {b:.4f}y + {c:.4f} = 0")
        print(f"사용된 접촉점: {len(inliers)}개")
        print(f"전체 측정점: {len(points)}개")

        # 벽의 위치 계산
        if abs(b) > abs(a):
            position = -c / a
            print(f"추정 X 위치: {position:.4f}m")

        else:
            position = -c / b
            print(f"추정 Y 위치: {position:.4f}m")

    print(" 공간 탐색 종료")


if __name__ == "__main__":
    main()