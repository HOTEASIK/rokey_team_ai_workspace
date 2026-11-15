#!/usr/bin/env python3
import math
import random

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .exploration import (
    CENTER,
    HEIGHT,
    WALLS,
    WIDTH,
    create_wall_points,
    ransac,
)
from .gap_detection import GapDetector


class NavigationRvizSimNode(Node):
    def __init__(self):
        super().__init__('navigation_rviz_sim')

        self.exploration_pub = self.create_publisher(
            MarkerArray, 'exploration_markers', 10
        )
        self.gap_pub = self.create_publisher(
            MarkerArray, 'gap_markers', 10
        )

        # Exploration data (computed once)
        self.exploration_data = self._run_exploration()

        # Gap detection simulation state
        self.gap_detector = GapDetector(
            calibration_count=10,
            threshold=0.8,
            confirm_count=3,
        )
        self.gap_step = 0
        self.gap_max_steps = 101
        self.gap_samples = []  # (position_mm, force, is_gap)

        self.timer = self.create_timer(0.5, self._on_timer)
        self.get_logger().info('RViz simulation node started')
        self.get_logger().info(
            f'Exploration space: {WIDTH}m x {HEIGHT}m, center={CENTER}'
        )

    def _run_exploration(self):
        data = {}
        for wall_name, wall_position in WALLS.items():
            points = create_wall_points(wall_name, wall_position)
            result = ransac(points)
            inliers = np.empty((0, 2))
            line = None
            if result is not None:
                a, b, c, inlier_pts = result
                inliers = inlier_pts
                line = (a, b, c)
            data[wall_name] = {
                'points': points,
                'inliers': inliers,
                'line': line,
                'wall_position': wall_position,
            }
            self.get_logger().info(
                f'Wall {wall_name}: total={len(points)}, inliers={len(inliers)}, '
                f'line={"ok" if line else "fail"}'
            )
        return data

    def _on_timer(self):
        self._publish_exploration()
        self._step_gap_simulation()
        self._publish_gap()

    # ---------------- Exploration markers ----------------

    def _publish_exploration(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = 'map'
        mid = 0

        # Workspace boundary
        boundary = Marker()
        boundary.header.frame_id = frame
        boundary.header.stamp = stamp
        boundary.ns = 'exploration_boundary'
        boundary.id = mid; mid += 1
        boundary.type = Marker.LINE_STRIP
        boundary.action = Marker.ADD
        boundary.scale.x = 0.003
        boundary.color.r = 1.0
        boundary.color.g = 1.0
        boundary.color.b = 1.0
        boundary.color.a = 1.0
        hx, hy = WIDTH / 2.0, HEIGHT / 2.0
        corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)]
        for x, y in corners:
            boundary.points.append(Point(x=x, y=y, z=0.0))
        markers.markers.append(boundary)

        wall_colors = {
            'right': (1.0, 0.2, 0.2),
            'left': (0.2, 0.4, 1.0),
            'top': (0.2, 1.0, 0.2),
            'bottom': (1.0, 0.7, 0.2),
        }

        for wall_name, info in self.exploration_data.items():
            color = wall_colors.get(wall_name, (0.8, 0.8, 0.8))
            pts = info['points']
            inliers = info['inliers']
            line = info['line']

            # All measured points
            m_pts = Marker()
            m_pts.header.frame_id = frame
            m_pts.header.stamp = stamp
            m_pts.ns = f'{wall_name}_points'
            m_pts.id = mid; mid += 1
            m_pts.type = Marker.SPHERE_LIST
            m_pts.action = Marker.ADD
            m_pts.scale.x = 0.008
            m_pts.scale.y = 0.008
            m_pts.scale.z = 0.008
            m_pts.color.r = 0.6
            m_pts.color.g = 0.6
            m_pts.color.b = 0.6
            m_pts.color.a = 0.7
            for p in pts:
                m_pts.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.0))
            markers.markers.append(m_pts)

            # Inlier points
            if len(inliers) > 0:
                m_in = Marker()
                m_in.header.frame_id = frame
                m_in.header.stamp = stamp
                m_in.ns = f'{wall_name}_inliers'
                m_in.id = mid; mid += 1
                m_in.type = Marker.SPHERE_LIST
                m_in.action = Marker.ADD
                m_in.scale.x = 0.01
                m_in.scale.y = 0.01
                m_in.scale.z = 0.01
                m_in.color.r = color[0]
                m_in.color.g = color[1]
                m_in.color.b = color[2]
                m_in.color.a = 1.0
                for p in inliers:
                    m_in.points.append(Point(x=float(p[0]), y=float(p[1]), z=0.005))
                markers.markers.append(m_in)

            # Fitted line (clipped to workspace)
            if line is not None:
                a, b, c = line
                seg = self._line_segment_in_rect(a, b, c, hx, hy)
                if seg is not None:
                    (x1, y1), (x2, y2) = seg
                    m_line = Marker()
                    m_line.header.frame_id = frame
                    m_line.header.stamp = stamp
                    m_line.ns = f'{wall_name}_line'
                    m_line.id = mid; mid += 1
                    m_line.type = Marker.LINE_LIST
                    m_line.action = Marker.ADD
                    m_line.scale.x = 0.004
                    m_line.color.r = color[0]
                    m_line.color.g = color[1]
                    m_line.color.b = color[2]
                    m_line.color.a = 1.0
                    m_line.points.append(Point(x=x1, y=y1, z=0.01))
                    m_line.points.append(Point(x=x2, y=y2, z=0.01))
                    markers.markers.append(m_line)

                    # Label
                    m_text = Marker()
                    m_text.header.frame_id = frame
                    m_text.header.stamp = stamp
                    m_text.ns = f'{wall_name}_label'
                    m_text.id = mid; mid += 1
                    m_text.type = Marker.TEXT_VIEW_FACING
                    m_text.action = Marker.ADD
                    m_text.pose.position.x = (x1 + x2) / 2.0
                    m_text.pose.position.y = (y1 + y2) / 2.0
                    m_text.pose.position.z = 0.04
                    m_text.pose.orientation.w = 1.0
                    m_text.scale.z = 0.03
                    m_text.color.r = color[0]
                    m_text.color.g = color[1]
                    m_text.color.b = color[2]
                    m_text.color.a = 1.0
                    m_text.text = wall_name
                    markers.markers.append(m_text)

        self.exploration_pub.publish(markers)

    def _line_segment_in_rect(self, a, b, c, hx, hy):
        # Intersect ax + by + c = 0 with rectangle [-hx,hx]x[-hy,hy]
        pts = []
        eps = 1e-9
        # x = -hx, x = hx
        if abs(b) > eps:
            for x in (-hx, hx):
                y = -(a * x + c) / b
                if -hy - 1e-6 <= y <= hy + 1e-6:
                    pts.append((x, max(-hy, min(hy, y))))
        # y = -hy, y = hy
        if abs(a) > eps:
            for y in (-hy, hy):
                x = -(b * y + c) / a
                if -hx - 1e-6 <= x <= hx + 1e-6:
                    pts.append((max(-hx, min(hx, x)), y))
        # unique
        uniq = []
        for p in pts:
            if not any(math.hypot(p[0]-q[0], p[1]-q[1]) < 1e-6 for q in uniq):
                uniq.append(p)
        if len(uniq) >= 2:
            # farthest pair
            best = (uniq[0], uniq[1])
            best_d = -1.0
            for i in range(len(uniq)):
                for j in range(i + 1, len(uniq)):
                    d = math.hypot(uniq[i][0]-uniq[j][0], uniq[i][1]-uniq[j][1])
                    if d > best_d:
                        best_d = d
                        best = (uniq[i], uniq[j])
            return best
        return None

    # ---------------- Gap detection markers ----------------

    def _step_gap_simulation(self):
        if self.gap_step >= self.gap_max_steps:
            if self.gap_step == self.gap_max_steps:
                self.gap_detector.finish()
                self.get_logger().info(
                    f'Gap simulation finished. gaps={self.gap_detector.gaps}'
                )
                self.gap_step += 1
            return

        position = self.gap_step * 0.1  # mm
        force = 5.0
        if 4.0 <= position <= 4.5:
            force = 3.5
        force += random.gauss(0, 0.05)

        before = len(self.gap_detector.gaps)
        detected = self.gap_detector.gap_detected
        self.gap_detector.add_sample(position, force)
        is_gap = self.gap_detector.gap_detected or len(self.gap_detector.gaps) > before

        self.gap_samples.append((position, force, self.gap_detector.gap_detected))
        self.gap_step += 1

    def _publish_gap(self):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame = 'map'
        mid = 1000

        # Draw gap scan path as a line along y = -HEIGHT/2 - 0.05 (below workspace)
        base_y = -HEIGHT / 2.0 - 0.06
        base_z = 0.0
        scale_x = 0.03  # 10mm -> 0.3m along x for visibility

        # Baseline path
        path = Marker()
        path.header.frame_id = frame
        path.header.stamp = stamp
        path.ns = 'gap_path'
        path.id = mid; mid += 1
        path.type = Marker.LINE_STRIP
        path.action = Marker.ADD
        path.scale.x = 0.003
        path.color.r = 0.7
        path.color.g = 0.7
        path.color.b = 0.7
        path.color.a = 1.0
        for i in range(self.gap_max_steps):
            pos_mm = i * 0.1
            path.points.append(Point(x=pos_mm * scale_x, y=base_y, z=base_z))
        markers.markers.append(path)

        # Samples: color by force (normal vs gap region)
        if self.gap_samples:
            pts_marker = Marker()
            pts_marker.header.frame_id = frame
            pts_marker.header.stamp = stamp
            pts_marker.ns = 'gap_samples'
            pts_marker.id = mid; mid += 1
            pts_marker.type = Marker.SPHERE_LIST
            pts_marker.action = Marker.ADD
            pts_marker.scale.x = 0.006
            pts_marker.scale.y = 0.006
            pts_marker.scale.z = 0.006

            # We need per-point colors -> use points/colors arrays
            for pos_mm, force, in_gap in self.gap_samples:
                pts_marker.points.append(
                    Point(x=pos_mm * scale_x, y=base_y, z=(force - 3.0) * 0.02)
                )
                c = ColorRGBA()
                if in_gap:
                    c.r, c.g, c.b, c.a = 1.0, 0.2, 0.2, 1.0
                else:
                    c.r, c.g, c.b, c.a = 0.2, 0.8, 1.0, 1.0
                pts_marker.colors.append(c)
            markers.markers.append(pts_marker)

        # Detected gap segments
        for idx, (start_mm, end_mm) in enumerate(self.gap_detector.gaps):
            seg = Marker()
            seg.header.frame_id = frame
            seg.header.stamp = stamp
            seg.ns = 'gap_detected'
            seg.id = mid; mid += 1
            seg.type = Marker.LINE_STRIP
            seg.action = Marker.ADD
            seg.scale.x = 0.01
            seg.color.r = 1.0
            seg.color.g = 1.0
            seg.color.b = 0.0
            seg.color.a = 1.0
            seg.points.append(Point(x=start_mm * scale_x, y=base_y, z=0.05))
            seg.points.append(Point(x=end_mm * scale_x, y=base_y, z=0.05))
            markers.markers.append(seg)

            txt = Marker()
            txt.header.frame_id = frame
            txt.header.stamp = stamp
            txt.ns = 'gap_detected_text'
            txt.id = mid; mid += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x = (start_mm + end_mm) / 2.0 * scale_x
            txt.pose.position.y = base_y
            txt.pose.position.z = 0.08
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.025
            txt.color.r = 1.0
            txt.color.g = 1.0
            txt.color.b = 0.0
            txt.color.a = 1.0
            txt.text = f'gap {idx+1}: {start_mm:.1f}-{end_mm:.1f}mm'
            markers.markers.append(txt)

        # Label
        label = Marker()
        label.header.frame_id = frame
        label.header.stamp = stamp
        label.ns = 'gap_label'
        label.id = mid; mid += 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = 0.15
        label.pose.position.y = base_y
        label.pose.position.z = 0.12
        label.pose.orientation.w = 1.0
        label.scale.z = 0.03
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = 'Gap detection (force over position)'
        markers.markers.append(label)

        self.gap_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationRvizSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
