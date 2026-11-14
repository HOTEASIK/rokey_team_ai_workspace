#!/usr/bin/env python3
"""벽 탐색 결과를 RViz2 에 MarkerArray 로 보여주는 모듈.

[보이는 것]  (네임스페이스(ns)별로 RViz 에서 켜고 끌 수 있음)
  probe      : 프로빙 접촉점 (빨간 큰 점)
  scan_h     : 가로(Y 방향) 슬라이딩 점 (파란 점)
  scan_v     : 세로(Z 방향) 슬라이딩 점 (주황 점)
  outlier    : 모델링에서 이상치로 빠진 점 (검은 점)
  plane      : 피팅된 벽 평면 (반투명 초록 판)
  normal     : 벽 법선 화살표 (로봇 쪽 방향)
  surface    : 2차 곡면 모델 메쉬 (편차를 색으로: 파랑=들어감, 빨강=튀어나옴)
  info       : 평면식 / 평탄도 텍스트

[RViz 설정]
  Fixed Frame = base_link
  Add → By topic → /dsr01/wall_markers (MarkerArray)
  ※ 탐색이 끝난 뒤 RViz 를 켰다면, Display 의 Topic → Durability Policy 를
    'Transient Local' 로 바꾸거나 `ros2 run rokey1 wall_modeling --rviz` 로 다시 띄우면 된다.

좌표 단위: 탐색 데이터는 mm, RViz 는 m → 내부에서 /1000 변환.
"""
import numpy as np

MM = 0.001   # mm → m

# kind → (ns, 색 RGBA, 점 지름[m])
POINT_STYLE = {
    'probe':  ('probe',  (0.90, 0.10, 0.10, 1.0), 0.014),
    'scan_h': ('scan_h', (0.15, 0.40, 0.95, 1.0), 0.006),
    'scan_v': ('scan_v', (1.00, 0.55, 0.05, 1.0), 0.006),
    'scan':   ('scan_h', (0.15, 0.40, 0.95, 1.0), 0.006),   # 예전 데이터 호환 (가로 스캔만 있던 시절)
}


def _coolwarm(t):
    """t ∈ [-1, 1] → (r, g, b). -1 파랑, 0 흰색, +1 빨강."""
    t = float(np.clip(t, -1.0, 1.0))
    if t < 0:
        return (1.0 + t * 0.8, 1.0 + t * 0.6, 1.0)
    return (1.0, 1.0 - t * 0.6, 1.0 - t * 0.8)


def _quat_from_matrix(R):
    """회전행렬(열 = x, y, z 축) → 쿼터니언 (x, y, z, w)."""
    m = np.asarray(R, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (m[2, 1] - m[1, 2]) / s
        y, z = (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        w, x = (m[2, 1] - m[1, 2]) / s, 0.25 * s
        y, z = (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        w, x = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s
        y, z = 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        w, x = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s
        y, z = (m[1, 2] + m[2, 1]) / s, 0.25 * s
    return x, y, z, w


class WallMarkerPublisher:
    """기존 rclpy 노드에 붙여서 쓰는 MarkerArray 퍼블리셔."""

    def __init__(self, node, topic='wall_markers', frame_id='base_link'):
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        from visualization_msgs.msg import Marker, MarkerArray
        from geometry_msgs.msg import Point
        from std_msgs.msg import ColorRGBA
        self._Marker, self._MarkerArray = Marker, MarkerArray
        self._Point, self._Color = Point, ColorRGBA

        self.node = node
        self.frame_id = frame_id
        # TRANSIENT_LOCAL: 나중에 접속한 RViz 도 마지막 메시지를 받을 수 있게
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = node.create_publisher(MarkerArray, topic, qos)

    # ------------------------------------------------------------ 공개 API
    def clear(self):
        """RViz 에 남아있는 이전 탐색 마커를 모두 지운다."""
        m = self._Marker()
        m.header.frame_id = self.frame_id
        m.action = self._Marker.DELETEALL
        self.pub.publish(self._MarkerArray(markers=[m]))

    def publish(self, points, kinds, plane=None, model=None):
        """현재까지의 점들 (+ 평면 / 모델) 을 한 번에 퍼블리시.

        Args:
            points: (N, 3) [mm] BASE 좌표
            kinds : 각 점의 종류 ('probe' / 'scan_h' / 'scan_v')
            plane : fit_plane() 결과 (탐색 중 평면 표시용, 선택)
            model : build_wall_model() 결과 (최종 모델 표시용, 선택)
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 3)
        kinds = np.asarray(kinds)
        markers = []
        inl = np.ones(len(pts), dtype=bool)
        if model is not None and len(model.get('inlier_mask', [])) == len(pts):
            inl = np.asarray(model['inlier_mask'], dtype=bool)

        # 1) 점들 (종류별)
        for kind, (ns, color, size) in POINT_STYLE.items():
            sel = (kinds == kind) & inl
            if kind == 'scan' and not sel.any():
                continue
            markers.append(self._sphere_list(ns, pts[sel], color, size,
                                             mid=list(POINT_STYLE).index(kind)))
        markers.append(self._sphere_list('outlier', pts[~inl], (0.05, 0.05, 0.05, 1.0), 0.012))

        # 2) 평면 + 법선
        if model is not None:
            p = model['plane']
            normal, centroid = np.array(p['normal']), np.array(p['centroid'])
            ext = model.get('extent_uv_mm')
        elif plane is not None:
            normal, centroid, ext = np.asarray(plane['normal']), np.asarray(plane['centroid']), None
        else:
            normal = None
        if normal is not None:
            u_ax, v_ax = self._basis(normal)
            if ext is None:      # 탐색 중에는 지금까지 점들의 범위로
                rel = pts - centroid
                ext = {'u': [float((rel @ u_ax).min()), float((rel @ u_ax).max())],
                       'v': [float((rel @ v_ax).min()), float((rel @ v_ax).max())]}
            markers.append(self._plane_box(centroid, normal, u_ax, v_ax, ext))
            markers.append(self._arrow(centroid, normal))

        # 3) 최종 모델: 곡면 메쉬 + 점 색칠(편차) + 텍스트
        if model is not None:
            markers.append(self._surface_mesh(model))
            markers.append(self._info_text(model, centroid, normal))

        msg = self._MarkerArray(markers=markers)
        self.pub.publish(msg)

    # ------------------------------------------------------------ 마커 생성
    def _base(self, ns, mid, mtype):
        m = self._Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, self._Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def _pt(self, xyz_mm):
        return self._Point(x=float(xyz_mm[0]) * MM, y=float(xyz_mm[1]) * MM, z=float(xyz_mm[2]) * MM)

    def _sphere_list(self, ns, pts, color, size, mid=0):
        m = self._base(ns, mid, self._Marker.SPHERE_LIST)
        m.scale.x = m.scale.y = m.scale.z = size
        m.color = self._Color(r=color[0], g=color[1], b=color[2], a=color[3])
        m.points = [self._pt(p) for p in pts]
        if not m.points:            # 빈 리스트면 지워서 이전 표시가 남지 않게
            m.action = self._Marker.DELETE
        return m

    @staticmethod
    def _basis(normal):
        # wall_modeling.plane_basis 와 같은 규칙 (u=수평, v=위쪽)
        n = np.asarray(normal, dtype=float)
        u = np.cross([0.0, 0.0, 1.0], n)
        if np.linalg.norm(u) < 1e-6:
            u = np.cross([1.0, 0.0, 0.0], n)
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        v /= np.linalg.norm(v)
        if v[2] < 0:
            u, v = -u, -v
        return u, v

    def _plane_box(self, centroid, normal, u_ax, v_ax, ext, margin=20.0):
        m = self._base('plane', 0, self._Marker.CUBE)
        u0, u1 = ext['u'][0] - margin, ext['u'][1] + margin
        v0, v1 = ext['v'][0] - margin, ext['v'][1] + margin
        center = centroid + 0.5 * (u0 + u1) * u_ax + 0.5 * (v0 + v1) * v_ax
        m.pose.position = self._pt(center)
        # 박스 x축=u, y축=v, z축=법선 (오른손 좌표계가 되도록 n = u × v 확인)
        n = np.asarray(normal, dtype=float)
        if np.dot(np.cross(u_ax, v_ax), n) < 0:
            n = -n
        qx, qy, qz, qw = _quat_from_matrix(np.column_stack([u_ax, v_ax, n]))
        m.pose.orientation.x, m.pose.orientation.y = qx, qy
        m.pose.orientation.z, m.pose.orientation.w = qz, qw
        m.scale.x, m.scale.y, m.scale.z = (u1 - u0) * MM, (v1 - v0) * MM, 0.002
        m.color = self._Color(r=0.2, g=0.8, b=0.3, a=0.30)
        return m

    def _arrow(self, centroid, normal, length=100.0):
        m = self._base('normal', 0, self._Marker.ARROW)
        m.points = [self._pt(centroid), self._pt(centroid + np.asarray(normal) * length)]
        m.scale.x, m.scale.y, m.scale.z = 0.006, 0.014, 0.02   # 몸통/머리 지름, 머리 길이
        m.color = self._Color(r=0.1, g=0.7, b=0.2, a=1.0)
        return m

    def _surface_mesh(self, model, n_grid=25):
        """2차 곡면 w = f(u, v) 를 삼각형 메쉬로. 색 = 평면 대비 편차."""
        m = self._base('surface', 0, self._Marker.TRIANGLE_LIST)
        quad = model.get('quadratic_surface')
        if not quad:
            m.action = self._Marker.DELETE
            return m
        c = quad['coeffs']
        normal = np.array(model['plane']['normal'])
        centroid = np.array(model['plane']['centroid'])
        u_ax, v_ax = self._basis(normal)
        ext = model['extent_uv_mm']
        us = np.linspace(ext['u'][0], ext['u'][1], n_grid)
        vs = np.linspace(ext['v'][0], ext['v'][1], n_grid)
        U, V = np.meshgrid(us, vs)
        W = c[0] + c[1] * U + c[2] * V + c[3] * U ** 2 + c[4] * U * V + c[5] * V ** 2
        lim = max(np.abs(W).max(), 0.5)
        P = centroid + U[..., None] * u_ax + V[..., None] * v_ax + W[..., None] * normal

        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = self._Color(r=1.0, g=1.0, b=1.0, a=0.85)

        def vert(i, j):
            m.points.append(self._pt(P[i, j]))
            r, g, b = _coolwarm(W[i, j] / lim)
            m.colors.append(self._Color(r=r, g=g, b=b, a=0.85))

        for i in range(n_grid - 1):
            for j in range(n_grid - 1):
                for (a, b) in ((i, j), (i + 1, j), (i + 1, j + 1), (i, j), (i + 1, j + 1), (i, j + 1)):
                    vert(a, b)
        return m

    def _info_text(self, model, centroid, normal):
        m = self._base('info', 0, self._Marker.TEXT_VIEW_FACING)
        p, f = model['plane'], model['flatness']
        top = np.array(centroid) + np.array([0.0, 0.0, model['extent_uv_mm']['v'][1] + 60.0])
        m.pose.position = self._pt(top)
        m.scale.z = 0.025
        m.color = self._Color(r=1.0, g=1.0, b=1.0, a=1.0)
        m.text = (f'{p["equation_str"]}\n'
                  f'yaw {p["yaw_deg"]:.2f}deg  pitch {p["pitch_deg"]:.2f}deg\n'
                  f'RMS {f["rms_mm"]:.2f}mm  P-V {f["peak_to_valley_mm"]:.2f}mm  '
                  f'({model["num_inliers"]}/{model["num_points"]} pts)')
        return m


def spin_publish_model(json_path, model, topic='wall_markers', frame_id='base_link',
                       namespace='dsr01', period=1.0):
    """저장된 탐색 데이터 + 모델을 RViz 에 Ctrl+C 할 때까지 계속 퍼블리시 (wall_modeling --rviz)."""
    import json
    import rclpy

    with open(json_path) as f:
        data = json.load(f)
    pts = [p['xyz'] for p in data['points']]
    kinds = [p['kind'] for p in data['points']]

    rclpy.init()
    node = rclpy.create_node('wall_model_rviz', namespace=namespace)
    pub = WallMarkerPublisher(node, topic=topic, frame_id=frame_id)
    pub.clear()
    node.create_timer(period, lambda: pub.publish(pts, kinds, model=model))
    pub.publish(pts, kinds, model=model)
    full = node.resolve_topic_name(topic)
    node.get_logger().info(f'RViz 퍼블리시 중: {full} (frame={frame_id}) - Ctrl+C 로 종료')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
