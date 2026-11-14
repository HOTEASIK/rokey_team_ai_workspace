#!/usr/bin/env python3
"""벽면 탐색 결과(접촉점들)로 벽을 '모델링'하는 코드.

[무엇을 하나?]
  wall_navigation.py 가 저장한 탐색 데이터(JSON)를 읽어서
    1) 평면 피팅      : 벽을 평면 ax + by + cz + d = 0 으로 근사 (SVD 최소자승법)
    2) 이상치 제거    : 평면에서 너무 먼 점(3σ 밖)은 빼고 다시 피팅
    3) 평탄도 계산    : 각 점이 평면에서 얼마나 튀어나오고/들어갔는지 (RMS, 최대값)
    4) 곡면 피팅      : 평면 기준 편차를 2차 곡면 w = f(u, v) 로 근사 (휘어짐 확인용)
    5) 시각화         : 3D 점 + 평면 + 법선, 편차 히트맵, 편차 히스토그램 → PNG 저장
    6) 결과 저장      : 모델 결과를 *_model.json 으로 저장
  를 수행한다.

[좌표계 설명]
  - 모든 좌표는 로봇 BASE 기준 [mm].
  - 평면 위에 2D 좌표계 (u, v) 를 만든다.
      u : 벽 위에서 '수평' 방향  (BASE 의 Z 축과 법선에 모두 수직)
      v : 벽 위에서 '위쪽' 방향
      w : 평면에서 법선 방향으로 떨어진 거리 (+ 이면 로봇 쪽으로 튀어나온 것)

[실행]
  # 가장 최근 탐색 데이터로 모델링
  ros2 run rokey1 wall_modeling
  # 특정 파일 지정 / 그림 창 띄우지 않기
  ros2 run rokey1 wall_modeling ~/wall_scan_data/scan_20261112_101010.json --no-show
  # RViz 에 모델 표시 (Ctrl+C 까지 /dsr01/wall_markers 퍼블리시, Fixed Frame = base_link)
  ros2 run rokey1 wall_modeling --rviz --no-show

[점 종류]
  probe  : 격자 프로빙 접촉점
  scan_h : 가로(Y 방향) 슬라이딩 점
  scan_v : 세로(Z 방향) 슬라이딩 점
  scan   : (예전 데이터) 가로 슬라이딩 점
"""
import argparse
import glob
import json
import os

import numpy as np

# 탐색 데이터가 저장되는 기본 폴더 (wall_navigation.py 와 같아야 함)
DEFAULT_DATA_DIR = os.path.expanduser('~/wall_scan_data')


# =============================================================================
# 1. 평면 피팅
# =============================================================================
def fit_plane(points):
    """점들에 가장 잘 맞는 평면을 SVD 로 구한다.

    원리:
      - 점들의 중심(centroid)을 빼면, 평면 방향으로는 넓게 퍼지고
        법선 방향으로는 거의 퍼지지 않는다.
      - SVD 를 하면 '가장 적게 퍼진 방향' = 가장 작은 특이값의 벡터 = 법선.

    Args:
        points: (N, 3) 배열 [mm], 최소 3개 (권장 4개 이상)
    Returns:
        dict(normal, centroid, d, rms, max_abs, residuals)
    """
    pts = np.asarray(points, dtype=float)
    if len(pts) < 3:
        raise ValueError(f'평면 피팅에는 점이 3개 이상 필요합니다 (현재 {len(pts)}개)')

    centroid = pts.mean(axis=0)
    _, _, vh = np.linalg.svd(pts - centroid)
    normal = vh[2, :]                      # 가장 작은 특이값에 대응하는 벡터

    # 법선 방향을 '로봇(원점) 쪽'으로 통일한다.
    # → w 가 + 이면 벽이 로봇 쪽으로 튀어나온 것
    if np.dot(normal, -centroid) < 0:
        normal = -normal

    d = -float(np.dot(normal, centroid))   # ax + by + cz + d = 0 의 d
    residuals = (pts - centroid) @ normal  # 각 점의 평면까지 부호 있는 거리 [mm]

    return {
        'normal': normal,
        'centroid': centroid,
        'd': d,
        'rms': float(np.sqrt(np.mean(residuals ** 2))),
        'max_abs': float(np.max(np.abs(residuals))),
        'residuals': residuals,
    }


def robust_fit_plane(points, sigma=3.0, max_iter=5, min_tol=2.0):
    """이상치(튀는 점)를 반복적으로 제거하며 평면을 피팅한다.

    - 평면에서 sigma × RMS 보다 멀리 떨어진 점은 이상치로 보고 제외
    - 단, 기준 거리는 최소 min_tol[mm] (점들이 너무 잘 맞을 때 정상점까지 버리는 것 방지)
    - 제외할 점이 더 이상 없거나 점이 4개 미만이 되면 멈춘다.

    Returns:
        (plane_dict, inlier_mask)  inlier_mask[i] = True 이면 사용된 점
    """
    pts = np.asarray(points, dtype=float)
    mask = np.ones(len(pts), dtype=bool)
    plane = fit_plane(pts)

    for _ in range(max_iter):
        # 현재 평면 기준으로 '전체 점'의 거리 계산
        dist = np.abs((pts - plane['centroid']) @ plane['normal'])
        tol = max(sigma * plane['rms'], min_tol)
        new_mask = dist <= tol
        if new_mask.sum() < 4 or np.array_equal(new_mask, mask):
            break
        mask = new_mask
        plane = fit_plane(pts[mask])

    # 최종 잔차는 '전체 점' 기준으로 다시 계산 (그림/통계에 쓰기 위해)
    plane['residuals_all'] = (pts - plane['centroid']) @ plane['normal']
    return plane, mask


# =============================================================================
# 2. 평면 위 2D 좌표계 (u, v, w)
# =============================================================================
def plane_basis(normal):
    """평면 위에서 쓸 단위벡터 u(수평), v(위쪽) 를 만든다."""
    n = np.asarray(normal, dtype=float)
    up = np.array([0.0, 0.0, 1.0])          # BASE Z 축 = 위쪽
    u = np.cross(up, n)
    if np.linalg.norm(u) < 1e-6:            # 벽이 바닥처럼 누워있는 특수한 경우
        u = np.cross([1.0, 0.0, 0.0], n)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)                      # n, u 에 모두 수직 → 벽 위의 '위쪽'
    v /= np.linalg.norm(v)
    if v[2] < 0:                            # v 는 항상 위(+Z)를 보도록
        u, v = -u, -v
    return u, v


def to_plane_coords(points, plane):
    """BASE 좌표 (x, y, z) → 평면 좌표 (u, v, w) 로 변환."""
    pts = np.asarray(points, dtype=float) - plane['centroid']
    u_axis, v_axis = plane_basis(plane['normal'])
    return pts @ u_axis, pts @ v_axis, pts @ plane['normal']


# =============================================================================
# 3. 2차 곡면 피팅 (벽이 휘었는지 확인)
# =============================================================================
def fit_quadratic_surface(u, v, w):
    """w = c0 + c1*u + c2*v + c3*u^2 + c4*u*v + c5*v^2 를 최소자승법으로 피팅.

    - c3, c5 가 크면 벽이 그 방향으로 오목/볼록하게 휘어 있다는 뜻.
    - 점이 6개 미만이면 계수를 구할 수 없으므로 None 반환.
    """
    u, v, w = map(np.asarray, (u, v, w))
    if len(u) < 6:
        return None
    A = np.column_stack([np.ones_like(u), u, v, u ** 2, u * v, v ** 2])
    coeffs, *_ = np.linalg.lstsq(A, w, rcond=None)
    fit_rms = float(np.sqrt(np.mean((A @ coeffs - w) ** 2)))
    return {'coeffs': coeffs.tolist(), 'rms': fit_rms}


# =============================================================================
# 4. 전체 모델 만들기
# =============================================================================
def build_wall_model(points):
    """접촉점들로 벽 모델(평면 + 평탄도 + 곡면)을 만든다.

    Args:
        points: (N, 3) BASE 좌표 [mm]
    Returns:
        model dict (JSON 저장 가능한 형태)
    """
    pts = np.asarray(points, dtype=float)
    plane, inliers = robust_fit_plane(pts)
    n = plane['normal']
    a, b, c = n

    # 벽 기울기 해석 (사람이 이해하기 쉬운 각도)
    #  - yaw  : 위에서 봤을 때 벽 법선이 -X 축에서 좌우로 돌아간 각도
    #  - pitch: 벽이 앞/뒤로 기울어진 각도 (0 이면 완전히 수직인 벽)
    yaw_deg = float(np.degrees(np.arctan2(n[1], -n[0])))
    pitch_deg = float(np.degrees(np.arcsin(np.clip(n[2], -1.0, 1.0))))

    u, v, w = to_plane_coords(pts[inliers], plane)
    quad = fit_quadratic_surface(u, v, w)

    return {
        'num_points': int(len(pts)),
        'num_inliers': int(inliers.sum()),
        'inlier_mask': inliers.tolist(),
        'plane': {
            'normal': n.tolist(),                       # 단위 법선 (로봇 쪽 방향)
            'centroid': plane['centroid'].tolist(),     # 벽 중심 [mm]
            'equation': [float(a), float(b), float(c), plane['d']],
            'equation_str': f'{a:.4f}x + {b:.4f}y + {c:.4f}z + {plane["d"]:.2f} = 0',
            'yaw_deg': yaw_deg,
            'pitch_deg': pitch_deg,
        },
        'flatness': {
            'rms_mm': plane['rms'],                     # 평면에서의 평균적인 벗어남
            'max_abs_mm': plane['max_abs'],             # 가장 많이 벗어난 점
            'peak_to_valley_mm': float(w.max() - w.min()) if len(w) else 0.0,
        },
        'quadratic_surface': quad,
        'extent_uv_mm': {                               # 벽에서 탐색한 영역 크기
            'u': [float(u.min()), float(u.max())],
            'v': [float(v.min()), float(v.max())],
        },
    }


def compare_with_truth(model, truth):
    """가상 모드일 때: 실제(가상) 벽과 모델을 비교해 정확도를 출력용 dict 로 반환."""
    if not truth:
        return None
    # 가상 벽: x = x0 + tilt_y*y + tilt_z*z  →  x - tilt_y*y - tilt_z*z - x0 = 0
    true_n = np.array([1.0, -truth['tilt_y'], -truth['tilt_z']])
    true_n /= np.linalg.norm(true_n)
    est_n = np.array(model['plane']['normal'])
    angle = float(np.degrees(np.arccos(np.clip(abs(true_n @ est_n), -1.0, 1.0))))
    # 모델 중심점에서 진짜 벽까지의 X 방향 거리
    cx, cy, cz = model['plane']['centroid']
    true_x = truth['x0'] + truth['tilt_y'] * cy + truth['tilt_z'] * cz
    return {'normal_angle_error_deg': angle, 'center_x_error_mm': float(cx - true_x)}


# =============================================================================
# 5. 시각화
# =============================================================================
def plot_model(points, kinds, model, png_path, show=True, title=''):
    """3D 그림 + 편차 히트맵 + 히스토그램을 하나의 PNG 로 저장한다."""
    import matplotlib
    if not show:
        matplotlib.use('Agg')               # 화면 없이 파일로만 저장
    import matplotlib.pyplot as plt

    pts = np.asarray(points, dtype=float)
    kinds = np.asarray(kinds)
    inl = np.asarray(model['inlier_mask'], dtype=bool)
    plane = {
        'normal': np.array(model['plane']['normal']),
        'centroid': np.array(model['plane']['centroid']),
    }
    u, v, w = to_plane_coords(pts, plane)

    fig = plt.figure(figsize=(16, 5.5))
    fig.suptitle(title or 'Wall model', fontsize=13)

    # --- (1) 3D: 접촉점 + 피팅 평면 + 법선 ----------------------------------
    ax = fig.add_subplot(1, 3, 1, projection='3d')
    for kind, color, marker in (('probe', 'tab:red', 'o'), ('scan', 'tab:blue', '.'),
                                ('scan_h', 'tab:blue', '.'), ('scan_v', 'tab:orange', '.')):
        sel = (kinds == kind) & inl
        if sel.any():
            ax.scatter(pts[sel, 0], pts[sel, 1], pts[sel, 2], c=color, marker=marker,
                       s=40 if kind == 'probe' else 10, label=f'{kind} ({sel.sum()})')
    if (~inl).any():
        ax.scatter(pts[~inl, 0], pts[~inl, 1], pts[~inl, 2], c='k', marker='x',
                   s=40, label=f'outlier ({(~inl).sum()})')

    # 평면을 (u, v) 범위만큼 사각형 격자로 그린다
    u_ax, v_ax = plane_basis(plane['normal'])
    uu, vv = np.meshgrid(np.linspace(u.min(), u.max(), 10), np.linspace(v.min(), v.max(), 10))
    grid = plane['centroid'] + uu[..., None] * u_ax + vv[..., None] * v_ax
    ax.plot_surface(grid[..., 0], grid[..., 1], grid[..., 2], alpha=0.25, color='tab:green')

    # 법선 화살표 (벽 크기의 1/3 길이)
    c = plane['centroid']
    L = 0.33 * max(np.ptp(u), np.ptp(v), 1.0)
    ax.quiver(c[0], c[1], c[2], *(plane['normal'] * L), color='tab:green', linewidth=2)
    ax.set_xlabel('X [mm]')
    ax.set_ylabel('Y [mm]')
    ax.set_zlabel('Z [mm]')
    ax.set_title('Contact points & fitted plane')
    ax.legend(loc='upper left', fontsize=8)
    _set_equal_3d(ax, pts)

    # --- (2) 편차 히트맵: 벽을 정면에서 본 모습 ------------------------------
    ax2 = fig.add_subplot(1, 3, 2)
    lim = max(np.abs(w[inl]).max(), 0.1)
    sc = ax2.scatter(u[inl], v[inl], c=w[inl], cmap='coolwarm', vmin=-lim, vmax=lim, s=30)
    ax2.scatter(u[~inl], v[~inl], c='k', marker='x', s=40)
    fig.colorbar(sc, ax=ax2, label='deviation w [mm] (+ = toward robot)')
    ax2.set_xlabel('u (horizontal on wall) [mm]')
    ax2.set_ylabel('v (up on wall) [mm]')
    ax2.set_title('Deviation from plane (front view)')
    ax2.set_aspect('equal', adjustable='datalim')
    ax2.grid(alpha=0.3)

    # --- (3) 편차 히스토그램 -------------------------------------------------
    ax3 = fig.add_subplot(1, 3, 3)
    ax3.hist(w[inl], bins=20, color='tab:blue', alpha=0.8)
    f = model['flatness']
    ax3.set_title(f'RMS {f["rms_mm"]:.2f} mm  /  max {f["max_abs_mm"]:.2f} mm')
    ax3.set_xlabel('deviation w [mm]')
    ax3.set_ylabel('count')
    ax3.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    print(f'[모델링] 그림 저장: {png_path}')
    if show:
        plt.show()
    plt.close(fig)


def _set_equal_3d(ax, pts):
    """3D 축 비율을 1:1:1 로 맞춘다 (안 하면 평면이 휘어 보임)."""
    center = (pts.max(axis=0) + pts.min(axis=0)) / 2
    r = max(np.ptp(pts, axis=0).max() / 2, 1.0)
    ax.set_xlim(center[0] - r, center[0] + r)
    ax.set_ylim(center[1] - r, center[1] + r)
    ax.set_zlim(center[2] - r, center[2] + r)


# =============================================================================
# 6. 파일 입출력 + 실행
# =============================================================================
def latest_scan_file(data_dir=DEFAULT_DATA_DIR):
    """data_dir 에서 가장 최근 scan_*.json (모델 결과 파일 제외) 을 찾는다."""
    files = [f for f in glob.glob(os.path.join(data_dir, 'scan_*.json'))
             if not f.endswith('_model.json')]
    if not files:
        raise FileNotFoundError(f'{data_dir} 에 scan_*.json 파일이 없습니다. 먼저 wall_navigation 을 실행하세요.')
    return max(files, key=os.path.getmtime)


def model_from_file(json_path, show=True, on_model=None):
    """탐색 데이터 파일 → 모델링 → 결과 출력/저장. 모델 dict 를 반환.

    on_model: 모델이 만들어진 직후(그림 창 띄우기 전) 호출할 함수 (예: RViz 퍼블리시)
    """
    with open(json_path) as f:
        data = json.load(f)

    pts = [p['xyz'] for p in data['points']]
    kinds = [p['kind'] for p in data['points']]
    model = build_wall_model(pts)
    model['source'] = os.path.abspath(json_path)
    model['mode'] = data.get('mode')
    model['truth_check'] = compare_with_truth(model, data.get('virtual_wall'))

    print_model(model)

    base = os.path.splitext(json_path)[0]
    with open(base + '_model.json', 'w') as f:
        json.dump(model, f, indent=2, ensure_ascii=False)
    print(f'[모델링] 모델 저장: {base}_model.json')
    if on_model is not None:
        on_model(model)

    title = f'Wall model ({data.get("mode", "?")} mode) - {os.path.basename(json_path)}'
    plot_model(pts, kinds, model, base + '_model.png', show=show, title=title)
    return model


def print_model(model):
    """모델 결과를 사람이 읽기 좋게 출력."""
    p, f = model['plane'], model['flatness']
    print('=============== 벽 모델링 결과 ===============')
    print(f' 사용한 점      : {model["num_inliers"]} / {model["num_points"]} (나머지는 이상치)')
    print(f' 평면 방정식    : {p["equation_str"]}')
    print(f' 법선 벡터      : [{p["normal"][0]:.4f}, {p["normal"][1]:.4f}, {p["normal"][2]:.4f}]')
    print(f' 벽 중심 [mm]   : [{p["centroid"][0]:.1f}, {p["centroid"][1]:.1f}, {p["centroid"][2]:.1f}]')
    print(f' 벽 기울기      : 좌우(yaw) {p["yaw_deg"]:.2f}°, 앞뒤(pitch) {p["pitch_deg"]:.2f}°')
    print(f' 평탄도         : RMS {f["rms_mm"]:.3f} mm, 최대 {f["max_abs_mm"]:.3f} mm, '
          f'P-V {f["peak_to_valley_mm"]:.3f} mm')
    q = model.get('quadratic_surface')
    if q:
        c = q['coeffs']
        print(f' 2차 곡면       : u² {c[3]:.2e}, uv {c[4]:.2e}, v² {c[5]:.2e} (잔차 RMS {q["rms"]:.3f} mm)')
    t = model.get('truth_check')
    if t:
        print(f' [가상벽 비교]  : 법선 오차 {t["normal_angle_error_deg"]:.3f}°, '
              f'중심 X 오차 {t["center_x_error_mm"]:.2f} mm')
    print('==============================================')


def main(args=None):
    parser = argparse.ArgumentParser(description='벽 탐색 데이터로 벽 모델링')
    parser.add_argument('file', nargs='?', help='scan_*.json 경로 (생략하면 가장 최근 파일)')
    parser.add_argument('--dir', default=DEFAULT_DATA_DIR, help='탐색 데이터 폴더')
    parser.add_argument('--no-show', action='store_true', help='그림 창을 띄우지 않고 PNG 만 저장')
    parser.add_argument('--rviz', action='store_true', help='RViz 에 모델 마커 퍼블리시 (Ctrl+C 로 종료)')
    parser.add_argument('--frame', default='base_link', help='RViz 기준 프레임 (기본 base_link)')
    # ros2 run 이 붙이는 --ros-args 이후 인자는 무시
    argv = args if args is not None else __import__('sys').argv[1:]
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]
    opt = parser.parse_args(argv)

    path = opt.file or latest_scan_file(opt.dir)
    print(f'[모델링] 입력 파일: {path}')
    model = model_from_file(path, show=not opt.no_show)
    if opt.rviz:
        try:
            from navigation.wall_rviz import spin_publish_model
        except ImportError:
            import sys
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from wall_rviz import spin_publish_model
        spin_publish_model(path, model, frame_id=opt.frame)


if __name__ == '__main__':
    main()
