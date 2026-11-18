#!/usr/bin/env python3
"""wall_tap.py 의 '탐색 경로 / 측정 결과' 를 RViz2 MarkerArray 로 보여주는 모듈.

[보이는 것]  (ns 별로 RViz 에서 켜고 끌 수 있음)
  path      : TCP 가 지나간 경로 (선)
                회색 = 일반 이동, 노랑 = 벽/바닥 찾기(전진), 초록 = 벽 따라가기
  contact   : 벽/바닥 '막힘' 지점 (빨간 큰 점)
  follow    : 따라가며 벽에 닿아 있던 점 (파랑 = 옆으로, 주황 = 위로)
  collision : 툴 충돌(force_guard)이 감지된 위치 (검은 큰 점)
  measure   : 선 측정 결과 (자홍 = 높이, 청록 = 너비) + 길이 텍스트
  floor     : 측정한 바닥 (반투명 회색 판)
  wall_est  : 지금까지 측정으로 추정한 벽 (반투명 초록 판) + 총 높이/너비 텍스트
  truth     : (virtual) 가상 벽 정답 (빨간 테두리)
  tool      : 현재 그리퍼 (45° 기울어진 막대 + 손가락 끝 점)

[사용]  wall_tap 이 탐색 중 /dsr01/wall_size_markers 로 퍼블리시 (rviz:=true)

[RViz]  Fixed Frame = base_link,  Add → By topic → /dsr01/wall_size_markers (MarkerArray)

[scene 형식]  (wall_tap.WallTapper.scene() 가 만듦)
  axes      : {'lat','approach','approach_sign','up'}  축 번호
  path      : [[x, y, z, phase], ...]   phase = 'move' | 'seek' | 'follow'
  points    : [{'kind': 'probe'|'scan_h'|'scan_v'|'floor', 'xyz': [...]}, ...]
  lines     : [{'kind': 'height'|'width', 'p0': [...], 'p1': [...], 'value': mm}, ...]
  floor_z, face, lat_range : 바닥 높이 / 벽 표면(벽 방향 거리) / 옆 범위
  tool      : {'pos': [...], 'dir': [...]} 또는 None
  truth     : 가상 벽 설정 또는 None
  좌표 단위는 mm, RViz 는 m → 내부에서 /1000 변환.
"""
import threading
import time

import numpy as np

MM = 0.001

PHASE_COLOR = {
    'move':   (0.60, 0.60, 0.60, 0.8),
    'seek':   (1.00, 0.85, 0.10, 1.0),
    'follow': (0.10, 0.80, 0.30, 1.0),
}
POINT_STYLE = {   # kind → (ns, 색, 지름[m])
    'probe':  ('contact', (0.90, 0.10, 0.10, 1.0), 0.012),
    'floor':  ('contact', (0.50, 0.10, 0.60, 1.0), 0.012),
    'scan_h': ('follow',  (0.15, 0.40, 0.95, 1.0), 0.005),
    'scan_v': ('follow',  (1.00, 0.55, 0.05, 1.0), 0.005),
    'collision': ('collision', (0.05, 0.05, 0.05, 1.0), 0.02),   # force_guard 가 잡은 툴 충돌 위치
}
LINE_COLOR = {
    'height': (0.90, 0.10, 0.80, 1.0),
    'width':  (0.05, 0.80, 0.90, 1.0),
}


class WallSizeViz:
    """기존 rclpy 노드에 붙여 쓰는 MarkerArray 퍼블리셔 (탐색 경로 + 측정 결과)."""

    def __init__(self, node, topic='wall_size_markers', frame_id='base_link', min_period=0.2):
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        self._Marker, self._MarkerArray = Marker, MarkerArray
        self._Point, self._Color = Point, ColorRGBA
        self.node = node
        self.frame_id = frame_id
        self.min_period = min_period          # 실시간 퍼블리시 최소 간격 [s]
        self._last = 0.0
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = node.create_publisher(MarkerArray, topic, qos)
        self._lock = threading.Lock()
        self._dirty = False
        self._thread = None

    # ------------------------------------------------------------ 백그라운드 퍼블리시
    #   마커 만들기(경로 점 수천 개)는 수십 ms 가 걸릴 수 있다. 이걸 힘 감시 루프 안에서
    #   하면 센서 확인이 늦어져 벽을 더 깊이 누르게 되므로, 루프는 mark_dirty() 만 하고
    #   실제 퍼블리시는 이 스레드가 period 마다 한다.
    def start_background(self, scene_fn, period=0.2):
        def loop():
            while self._thread is not None:
                if self._dirty:
                    self._dirty = False
                    try:
                        self.publish(scene_fn(), force=True)
                    except Exception as e:
                        self.node.get_logger().warn(f'RViz 퍼블리시 실패: {e}')
                time.sleep(period)
        self._thread = threading.Thread(target=loop, daemon=True)
        self._thread.start()

    def mark_dirty(self):
        self._dirty = True

    def stop_background(self):
        self._thread = None

    # ------------------------------------------------------------ 공개 API
    def clear(self):
        m = self._Marker()
        m.header.frame_id = self.frame_id
        m.action = self._Marker.DELETEALL
        self.pub.publish(self._MarkerArray(markers=[m]))

    def publish(self, scene, force=False):
        """scene 을 MarkerArray 로 퍼블리시. force=False 면 min_period 마다 한 번만."""
        now = time.time()
        if not force and now - self._last < self.min_period:
            return False
        with self._lock:
            self._last = now
            self.pub.publish(self._MarkerArray(markers=self.build(scene)))
        return True

    def build(self, scene):
        ax = scene['axes']
        mk = []
        mk += self._path(scene.get('path', []))
        mk += self._points(scene.get('points', []))
        mk += self._lines(scene.get('lines', []), ax)
        if scene.get('floor_z') is not None:
            mk.append(self._floor(scene, ax))
        mk += self._wall_est(scene, ax)
        if scene.get('truth'):
            mk.append(self._truth(scene['truth'], ax))
        if scene.get('tool'):
            mk += self._tool(scene['tool'])
        return mk

    # ------------------------------------------------------------ 요소별
    def _path(self, path):
        """경로: phase 별 색의 LINE_LIST (연속한 두 점 = 한 선분, 뒤 점의 phase 색)."""
        m = self._base('path', 0, self._Marker.LINE_LIST)
        m.scale.x = 0.002
        for a, b in zip(path[:-1], path[1:]):
            c = self._color(PHASE_COLOR.get(b[3], PHASE_COLOR['move']))
            m.points += [self._pt(a[:3]), self._pt(b[:3])]
            m.colors += [c, c]
        return [m]

    def _points(self, points):
        out = []
        for i, (kind, (ns, color, size)) in enumerate(POINT_STYLE.items()):
            m = self._base(ns, i, self._Marker.SPHERE_LIST)
            m.scale.x = m.scale.y = m.scale.z = size
            m.color = self._color(color)
            m.points = [self._pt(p['xyz']) for p in points if p['kind'] == kind]
            out.append(m)
        return out

    def _lines(self, lines, ax):
        out = []
        for i, ln in enumerate(lines):
            color = LINE_COLOR.get(ln['kind'], (1, 1, 1, 1))
            # 벽 표면보다 3mm 앞(로봇 쪽)에 그려서 벽 판에 가려지지 않게
            off = -np.eye(3)[ax['approach']] * ax['approach_sign'] * 3.0
            p0, p1 = np.asarray(ln['p0']) + off, np.asarray(ln['p1']) + off
            m = self._base('measure', 2 * i, self._Marker.LINE_STRIP)
            m.scale.x = 0.003
            m.color = self._color(color)
            m.points = [self._pt(p0), self._pt(p1)]
            out.append(m)
            label = ('H ' if ln['kind'] == 'height' else 'W ') + f"{ln['value']:.1f}"
            out.append(self._text('measure', 2 * i + 1, (p0 + p1) / 2 + off * 3, label, color, 0.012))
        return out

    def _floor(self, scene, ax):
        lo, hi = scene.get('lat_range', (-250.0, 250.0))
        face = scene.get('face', 0.0)
        a0, a1 = face - 200.0, face
        z = scene['floor_z']
        center = self._grid(ax, (lo + hi) / 2, (a0 + a1) / 2, z - 1.0)
        size = self._grid(ax, hi - lo, a1 - a0, 2.0, signed=False)
        m = self._base('floor', 0, self._Marker.CUBE)
        self._set_box(m, center, size)
        m.color = self._color((0.5, 0.5, 0.5, 0.25))
        return m

    def _wall_est(self, scene, ax):
        """측정된 선들로 벽 범위를 추정해 반투명 판으로 표시."""
        lines = scene.get('lines', [])
        if not lines or scene.get('floor_z') is None:
            return []
        L, U = ax['lat'], ax['up']
        lats = [ln[p][L] for ln in lines for p in ('p0', 'p1')]
        tops = [ln['p1'][U] for ln in lines if ln['kind'] == 'height']
        tops += [ln['p0'][U] for ln in lines if ln['kind'] == 'width']
        lo, hi, floor = min(lats), max(lats), scene['floor_z']
        top = max(tops)
        if hi - lo < 1.0:          # 높이 선 하나뿐이면 얇은 판
            lo, hi = lo - 5.0, hi + 5.0
        face = scene.get('face', 0.0)
        center = self._grid(ax, (lo + hi) / 2, face + 1.0, (floor + top) / 2)
        size = self._grid(ax, hi - lo, 2.0, top - floor, signed=False)
        m = self._base('wall_est', 0, self._Marker.CUBE)
        self._set_box(m, center, size)
        m.color = self._color((0.2, 0.9, 0.3, 0.25))
        s = scene.get('summary') or {}
        h, w = s.get('total_height'), s.get('total_width')
        txt = 'H {} / W {} mm'.format(f'{h:.1f}' if h is not None else '-',
                                      f'{w:.1f}' if w is not None else '-')
        t = self._text('wall_est', 1, self._grid(ax, (lo + hi) / 2, face, top + 30.0), txt,
                       (1, 1, 1, 1), 0.02)
        return [m, t]

    def _truth(self, box, ax):
        """가상 벽 정답의 앞면 테두리."""
        lo, hi = box['lat']
        f, fl, tp = box['face'], box['floor'], box['top']
        c = [self._grid(ax, lo, f, fl), self._grid(ax, hi, f, fl),
             self._grid(ax, hi, f, tp), self._grid(ax, lo, f, tp)]
        m = self._base('truth', 0, self._Marker.LINE_STRIP)
        m.scale.x = 0.0015
        m.color = self._color((1.0, 0.2, 0.2, 0.9))
        m.points = [self._pt(p) for p in c + [c[0]]]
        return m

    def _tool(self, tool, length=120.0):
        """현재 그리퍼: 손가락 끝(TCP) 에서 공구 축 반대로 length 만큼 막대."""
        tip = np.asarray(tool['pos'], float)
        d = np.asarray(tool['dir'], float)
        d = d / np.linalg.norm(d)
        body = self._base('tool', 0, self._Marker.ARROW)
        body.scale.x, body.scale.y, body.scale.z = 0.012, 0.02, 0.02
        body.color = self._color((0.85, 0.85, 0.9, 0.9))
        body.points = [self._pt(tip - d * length), self._pt(tip)]
        dot = self._base('tool', 1, self._Marker.SPHERE)
        dot.pose.position = self._pt(tip)
        dot.pose.orientation.w = 1.0
        dot.scale.x = dot.scale.y = dot.scale.z = 0.01
        dot.color = self._color((1.0, 1.0, 0.0, 1.0))
        return [body, dot]

    # ------------------------------------------------------------ 헬퍼
    @staticmethod
    def _grid(ax, lat, a, up, signed=True):
        """(옆, 벽 방향 거리, 위) → BASE xyz [mm]."""
        p = np.zeros(3)
        p[ax['lat']], p[ax['up']] = lat, up
        p[ax['approach']] = a * (ax['approach_sign'] if signed else 1.0)
        return p

    def _set_box(self, m, center, size):
        m.pose.position = self._pt(center)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = [max(abs(float(v)) * MM, 1e-4) for v in size]

    def _base(self, ns, mid, mtype):
        m = self._Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, self._Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _text(self, ns, mid, pos, text, color, height):
        m = self._base(ns, mid, self._Marker.TEXT_VIEW_FACING)
        m.pose.position = self._pt(pos)
        m.scale.z = height
        m.color = self._color(color)
        m.text = text
        return m

    def _pt(self, xyz):
        p = self._Point()
        p.x, p.y, p.z = (float(v) * MM for v in xyz[:3])
        return p

    def _color(self, rgba):
        c = self._Color()
        c.r, c.g, c.b, c.a = (float(v) for v in rgba)
        return c
