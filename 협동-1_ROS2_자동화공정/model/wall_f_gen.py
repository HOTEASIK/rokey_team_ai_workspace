#!/usr/bin/env python3
"""wall_f.urdf 생성기

wall1(단일 벽) / wall2(세로 홈) / wall3(상하 단차)를 합친 최종 벽 모델(wall_f)을 만든다.

개선 사항 반영
  1. 각 벽(패널)을 독립 링크(객체)로 분리하고, 좌우(x)·상하(z) prismatic 조인트를 달아
     joint_state_publisher_gui 슬라이더로 움직일 수 있게 한다.
  2. 각 패널의 8개 모서리(앞면 4 + 뒷면 4)를 45° 모따기(chamfer)할 수 있게 한다.
     모따기 형상은 STL 메시로 생성한다 (meshes/ 폴더).
  3. 위 값들을 전부 파라미터(CLI 인자 / JSON / launch 인자)로 넘길 수 있게 한다.

좌표계 (패널 로컬)
  x : 좌(-) / 우(+)   (벽 가로)
  y : 앞(-) / 뒤(+)   (벽 두께)  ← 앞면 = -y 면
  z : 하(-) / 상(+)   (벽 높이)

사용 예
  python3 wall_f_gen.py                          # 기본값(wall3 형태 + 2cm 모따기)으로 wall_f.urdf 생성
  python3 wall_f_gen.py --chamfer 0.03           # 모든 패널, 8모서리 3cm 모따기
  python3 wall_f_gen.py --set ul.front_top=0.05 --set lr.back_left=0.02
  python3 wall_f_gen.py --set ll.dy=0.08 --set ur.dz=0.1
  python3 wall_f_gen.py --params my_wall.json    # JSON 파라미터 파일
"""

import argparse
import copy
import json
import math
import os
import struct
import sys

import numpy as np
from scipy.spatial import ConvexHull, HalfspaceIntersection

HERE = os.path.dirname(os.path.abspath(__file__))

EDGES = ['front_top', 'front_bottom', 'front_left', 'front_right',
         'back_top', 'back_bottom', 'back_left', 'back_right']

PANEL_NAMES = {'ul': '상단 왼쪽', 'ur': '상단 오른쪽', 'll': '하단 왼쪽', 'lr': '하단 오른쪽'}

# ---------------------------------------------------------------- 기본 파라미터
# wall3 와 동일한 배치: 전체 5.0 x 0.2 x 2.0, 가운데 0.005 세로 홈, 하단 패널 y=+0.05 단차
DEFAULT_PARAMS = {
    'wall_xyz': [2.0, 0.0, 0.0],   # 월드 기준 벽 위치 (wall1~3 과 동일)
    'wall_rpy': [0.0, 0.0, 0.0],
    'total_width': 5.0,
    'thickness': 0.2,
    'total_height': 2.0,
    'groove': 0.005,               # 좌/우 패널 사이 세로 홈 폭
    'split_height': 1.0,           # 하단 패널 높이 (상단 = total_height - split_height)
    'step': 0.05,                  # 하단 패널 y 단차
    'move_limit_x': 1.0,           # 좌우 슬라이더 범위 ±
    'move_limit_z': 1.0,           # 상하 슬라이더 범위 ±
    'mass_total': 100.0,
    'panels': {
        # dx/dy/dz : 기본 위치에서의 추가 오프셋 (좌우 / 앞뒤 / 상하). 슬라이더는 이 위치 기준 ±이동
        # chamfer  : 8모서리 모따기 크기 [m], 0 이면 모따기 없음
        name: {'dx': 0.0, 'dy': 0.0, 'dz': 0.0,
               'chamfer': {e: 0.02 for e in EDGES}}
        for name in PANEL_NAMES
    },
}


# ---------------------------------------------------------------- 형상
def chamfered_box(sx, sy, sz, ch):
    """크기 (sx, sy, sz) 박스의 앞/뒤면 8모서리를 45° 모따기한 볼록 다면체.

    반환: (vertices Nx3, faces Mx3) - 바깥쪽을 향하는 삼각형
    """
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    # 모따기 유효성 검사
    for side, lim in (('top', sz), ('bottom', sz), ('left', sx), ('right', sx)):
        s = ch['front_' + side] + ch['back_' + side]
        if s >= sy + 1e-9:
            raise ValueError(f'front_{side} + back_{side} = {s} 가 두께 {sy} 이상입니다.')
    for e in EDGES:
        if ch[e] < 0:
            raise ValueError(f'{e} 모따기는 음수일 수 없습니다.')
        if ch[e] >= min(sx, sz) / 2:
            raise ValueError(f'{e} 모따기 {ch[e]} 가 너무 큽니다.')

    # 반공간: A·p + b <= 0 형식 (scipy 규약)
    hs = [
        [1, 0, 0, -hx], [-1, 0, 0, -hx],
        [0, 1, 0, -hy], [0, -1, 0, -hy],
        [0, 0, 1, -hz], [0, 0, -1, -hz],
    ]
    face_n = {'front': (np.array([0, -1, 0]), hy), 'back': (np.array([0, 1, 0]), hy)}
    side_n = {'top': (np.array([0, 0, 1]), hz), 'bottom': (np.array([0, 0, -1]), hz),
              'left': (np.array([-1, 0, 0]), hx), 'right': (np.array([1, 0, 0]), hx)}
    for e in EDGES:
        c = ch[e]
        if c <= 0:
            continue
        f, s = e.split('_')
        nf, df = face_n[f]
        ns, ds = side_n[s]
        n = nf + ns
        hs.append([*n, -(df + ds - c)])
    hs = np.array(hs, dtype=float)

    hi = HalfspaceIntersection(hs, np.zeros(3))
    pts = hi.intersections
    # 중복 꼭짓점 제거
    pts = np.unique(np.round(pts, 9), axis=0)
    hull = ConvexHull(pts)
    faces = []
    for simplex in hull.simplices:
        a, b, c = pts[simplex]
        n = np.cross(b - a, c - a)
        if np.dot(n, (a + b + c) / 3) < 0:  # 중심(원점) 기준 바깥을 향하도록
            simplex = simplex[[0, 2, 1]]
        faces.append(simplex)
    return pts, np.array(faces)


def write_stl(path, verts, faces, name='wall_panel'):
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b'\0'))
        f.write(struct.pack('<I', len(faces)))
        for tri in faces:
            a, b, c = verts[tri]
            n = np.cross(b - a, c - a)
            ln = np.linalg.norm(n)
            n = n / ln if ln > 0 else n
            f.write(struct.pack('<12fH', *n, *a, *b, *c, 0))


# ---------------------------------------------------------------- URDF
def fmt(*v):
    return ' '.join(f'{x:.6g}' for x in v)


def box_inertia(m, sx, sy, sz):
    return (m * (sy**2 + sz**2) / 12, m * (sx**2 + sz**2) / 12, m * (sx**2 + sy**2) / 12)


def panel_layout(p):
    """패널별 (크기, 기본 위치) 계산."""
    W, T, H = p['total_width'], p['thickness'], p['total_height']
    g, hs, step = p['groove'], p['split_height'], p['step']
    pw = (W - g) / 2                 # 패널 가로 (기본 2.4975)
    cx = (pw + g) / 2                # 패널 중심 x (기본 1.25125)
    hu = H - hs                      # 상단 패널 높이
    return {
        'ul': ((pw, T, hu), (-cx, 0.0, hs + hu / 2)),
        'ur': ((pw, T, hu), (cx, 0.0, hs + hu / 2)),
        'll': ((pw, T, hs), (-cx, step, hs / 2)),
        'lr': ((pw, T, hs), (cx, step, hs / 2)),
    }


def build(p, out_urdf, mesh_dir):
    os.makedirs(mesh_dir, exist_ok=True)
    layout = panel_layout(p)
    vol_total = sum(s[0] * s[1] * s[2] for s, _ in layout.values())
    lx, lz = p['move_limit_x'], p['move_limit_z']

    out = ['<?xml version="1.0"?>',
           '<!-- 이 파일은 wall_f_gen.py 로 자동 생성됩니다. 직접 수정하지 말고 파라미터를 바꿔 다시 생성하세요. -->',
           '<robot name="wall_f">', '',
           '  <material name="gray"><color rgba="0.7 0.7 0.7 1.0"/></material>',
           '  <material name="dark_gray"><color rgba="0.55 0.55 0.55 1.0"/></material>', '',
           '  <link name="world"/>', '',
           '  <!-- 벽 전체 기준 링크 -->',
           '  <link name="wall_base"/>',
           '  <joint name="wall_fixed_joint" type="fixed">',
           '    <parent link="world"/>',
           '    <child link="wall_base"/>',
           f'    <origin xyz="{fmt(*p["wall_xyz"])}" rpy="{fmt(*p["wall_rpy"])}"/>',
           '  </joint>', '']

    for name, label in PANEL_NAMES.items():
        pp = p['panels'][name]
        (sx, sy, sz), (bx, by, bz) = layout[name]
        ch = pp['chamfer']
        verts, faces = chamfered_box(sx, sy, sz, ch)
        stl = os.path.join(mesh_dir, f'wall_f_{name}.stl')
        write_stl(stl, verts, faces, f'wall_f_{name}')
        uri = 'file://' + os.path.abspath(stl)
        m = p['mass_total'] * sx * sy * sz / vol_total
        ixx, iyy, izz = box_inertia(m, sx, sy, sz)
        mat = 'gray' if name.startswith('u') else 'dark_gray'
        ch_txt = ', '.join(f'{e}={ch[e]:g}' for e in EDGES)

        out += [
            f'  <!-- ==================== {label} 패널 ({name}) ==================== -->',
            f'  <!-- 크기 {fmt(sx, sy, sz)} / 모따기: {ch_txt} -->',
            f'  <link name="{name}_slider"/>',
            f'  <joint name="{name}_x_joint" type="prismatic">  <!-- 좌우 이동 -->',
            '    <parent link="wall_base"/>',
            f'    <child link="{name}_slider"/>',
            f'    <origin xyz="{fmt(bx + pp["dx"], by + pp["dy"], bz + pp["dz"])}" rpy="0 0 0"/>',
            '    <axis xyz="1 0 0"/>',
            f'    <limit lower="{-lx:g}" upper="{lx:g}" effort="100" velocity="0.5"/>',
            '  </joint>',
            f'  <link name="{name}_link">',
            '    <visual>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <geometry><mesh filename="{uri}"/></geometry>',
            f'      <material name="{mat}"/>',
            '    </visual>',
            '    <collision>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <geometry><mesh filename="{uri}"/></geometry>',
            '    </collision>',
            '    <inertial>',
            '      <origin xyz="0 0 0" rpy="0 0 0"/>',
            f'      <mass value="{m:.6g}"/>',
            f'      <inertia ixx="{ixx:.6g}" ixy="0" ixz="0" iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/>',
            '    </inertial>',
            '  </link>',
            f'  <joint name="{name}_z_joint" type="prismatic">  <!-- 상하 이동 -->',
            f'    <parent link="{name}_slider"/>',
            f'    <child link="{name}_link"/>',
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            '    <axis xyz="0 0 1"/>',
            f'    <limit lower="{-lz:g}" upper="{lz:g}" effort="100" velocity="0.5"/>',
            '  </joint>',
            f'  <gazebo reference="{name}_link"><static>true</static></gazebo>',
            '',
        ]
    out.append('</robot>')
    xml = '\n'.join(out) + '\n'
    if out_urdf:
        with open(out_urdf, 'w', encoding='utf-8') as f:
            f.write(xml)
    return xml


# ---------------------------------------------------------------- 파라미터 처리
def deep_update(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            deep_update(dst[k], v)
        else:
            dst[k] = v


def apply_set(p, expr):
    """'ul.front_top=0.05', 'll.dy=0.1', 'all.chamfer=0.03', 'groove=0.01' 형식."""
    key, val = expr.split('=', 1)
    val = json.loads(val)
    parts = key.strip().split('.')
    if len(parts) == 1:
        if parts[0] not in p:
            raise KeyError(f'알 수 없는 파라미터: {parts[0]}')
        p[parts[0]] = val
        return
    target, field = parts
    names = list(PANEL_NAMES) if target == 'all' else [target]
    for n in names:
        if n not in PANEL_NAMES:
            raise KeyError(f'알 수 없는 패널: {n} (ul, ur, ll, lr, all)')
        pp = p['panels'][n]
        if field == 'chamfer':
            pp['chamfer'] = {e: float(val) for e in EDGES}
        elif field in ('front', 'back'):
            for s in ('top', 'bottom', 'left', 'right'):
                pp['chamfer'][f'{field}_{s}'] = float(val)
        elif field in EDGES:
            pp['chamfer'][field] = float(val)
        elif field in ('dx', 'dy', 'dz'):
            pp[field] = float(val)
        else:
            raise KeyError(f'알 수 없는 패널 필드: {field}')


def make_params(params_file=None, chamfer=None, sets=()):
    p = copy.deepcopy(DEFAULT_PARAMS)
    if params_file:
        with open(params_file, encoding='utf-8') as f:
            deep_update(p, json.load(f))
    if chamfer is not None:
        apply_set(p, f'all.chamfer={chamfer}')
    for s in sets:
        apply_set(p, s)
    return p


def main(argv=None):
    ap = argparse.ArgumentParser(description='wall_f.urdf 생성기')
    ap.add_argument('--params', help='JSON 파라미터 파일')
    ap.add_argument('--chamfer', type=float, help='모든 패널 8모서리 모따기 [m]')
    ap.add_argument('--set', action='append', default=[], metavar='KEY=VAL',
                    help='예: ul.front_top=0.05, ll.dz=0.1, all.back=0.01, groove=0.01')
    ap.add_argument('-o', '--output', default=os.path.join(HERE, 'wall_f.urdf'))
    ap.add_argument('--mesh-dir', default=os.path.join(HERE, 'meshes'))
    ap.add_argument('--dump-params', action='store_true', help='최종 파라미터 JSON 출력')
    a = ap.parse_args(argv)

    p = make_params(a.params, a.chamfer, a.set)
    if a.dump_params:
        print(json.dumps(p, indent=2, ensure_ascii=False))
    build(p, a.output, a.mesh_dir)
    print(f'생성 완료: {a.output}  (메시: {a.mesh_dir})')


if __name__ == '__main__':
    sys.exit(main())
