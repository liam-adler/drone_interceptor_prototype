from __future__ import annotations

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from drone_interceptor.core.math_utils import Array3


def make_color(r: float, g: float, b: float, a: float) -> ColorRGBA:
    color = ColorRGBA()
    color.r = r
    color.g = g
    color.b = b
    color.a = a
    return color


def make_point(coords: Array3) -> Point:
    point = Point()
    point.x = float(coords[0])
    point.y = float(coords[1])
    point.z = float(coords[2])
    return point


def build_sphere_marker(
    *,
    stamp,
    frame_id: str,
    namespace: str,
    marker_id: int,
    position: Array3,
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0),
    scale: float,
    color: tuple[float, float, float, float],
) -> Marker:
    marker = Marker()
    marker.header.stamp = stamp
    marker.header.frame_id = frame_id
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.pose.position.x = float(position[0])
    marker.pose.position.y = float(position[1])
    marker.pose.position.z = float(position[2])
    marker.pose.orientation.x = float(orientation[0])
    marker.pose.orientation.y = float(orientation[1])
    marker.pose.orientation.z = float(orientation[2])
    marker.pose.orientation.w = float(orientation[3])
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color = make_color(*color)
    return marker
