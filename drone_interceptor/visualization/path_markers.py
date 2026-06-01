from __future__ import annotations

from visualization_msgs.msg import Marker

from drone_interceptor.core.math_utils import Array3
from drone_interceptor.visualization.rviz_markers import make_color, make_point


def build_line_strip_marker(
    *,
    stamp,
    frame_id: str,
    namespace: str,
    marker_id: int,
    points: list[Array3],
    line_width: float,
    color: tuple[float, float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = line_width
    marker.color = make_color(*color)
    marker.points.extend(make_point(point) for point in points)
    return marker
