import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, TransformStamped
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class TargetDynamicsNode(Node):
    def __init__(self):
        super().__init__("target_dynamics_node")

        self.state_publisher_ = self.create_publisher(
            Odometry,
            "/target/state",
            10
        )

        self.marker_publisher_ = self.create_publisher(
            Marker,
            "/target/marker",
            10
        )

        self.path_marker_publisher_ = self.create_publisher(
            Marker,
            "/target/path_marker",
            10
        )

        self.desired_heading_sub = self.create_subscription(
            Vector3,
            "/target/desired_heading",
            self.desired_heading_callback,
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.dt = 0.5
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.position = np.array([0.0, 0.0, 2.0], dtype=float)
        self.velocity = np.array([1.0, 0.5, 0.1], dtype=float)

        self.path_points = []
        self.interpolation_steps = 5
        self.max_path_points = 1000

        self.v_floor = 0.1
        self.v_max = 30.0  # m/s

        self.a_long_max = 5.0   # not used yet
        self.a_lat_max = 3.0    # used for turn-rate limit
        self.a_vert_max = 3.0   # not used yet

        self.desired_heading = np.array([1.0, 1.0, 0.3], dtype=float)
        self.desired_heading = self.desired_heading / np.linalg.norm(self.desired_heading)

        self.compute_heading_from_velocity()

    def desired_heading_callback(self, msg):
        desired_heading = np.array([msg.x, msg.y, msg.z], dtype=float)
        norm = np.linalg.norm(desired_heading)

        if norm < 1e-6:
            return

        self.desired_heading = desired_heading / norm

    def compute_heading_from_velocity(self):
        speed = np.linalg.norm(self.velocity)

        if speed < self.v_floor:
            self.speed = 0.0
            self.heading = np.zeros(3, dtype=float)
            return

        self.speed = speed
        self.heading = self.velocity / speed

    def turn_towards_desired_heading(self):
        self.compute_heading_from_velocity()

        if self.speed < self.v_floor:
            return

        dot = np.dot(self.heading, self.desired_heading)
        dot = np.clip(dot, -1.0, 1.0)

        theta = np.arccos(dot)

        if theta < 1e-6:
            return

        omega_max = self.a_lat_max / max(self.speed, self.v_floor)
        dtheta_max = omega_max * self.dt

        alpha = min(1.0, dtheta_max / theta)

        new_heading = (1.0 - alpha) * self.heading + alpha * self.desired_heading
        new_heading_norm = np.linalg.norm(new_heading)

        if new_heading_norm < 1e-6:
            return

        self.heading = new_heading / new_heading_norm
        self.velocity = self.speed * self.heading

    def update_path_points(self, old_position, new_position):
        for i in range(1, self.interpolation_steps + 1):
            alpha = i / self.interpolation_steps
            interpolated_position = (1.0 - alpha) * old_position + alpha * new_position

            point = Point()
            point.x = float(interpolated_position[0])
            point.y = float(interpolated_position[1])
            point.z = float(interpolated_position[2])

            self.path_points.append(point)

        if len(self.path_points) > self.max_path_points:
            self.path_points = self.path_points[-self.max_path_points:]

    def publish_odometry(self):
        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "target"

        msg.pose.pose.position.x = float(self.position[0])
        msg.pose.pose.position.y = float(self.position[1])
        msg.pose.pose.position.z = float(self.position[2])
        msg.pose.pose.orientation.w = 1.0

        msg.twist.twist.linear.x = float(self.velocity[0])
        msg.twist.twist.linear.y = float(self.velocity[1])
        msg.twist.twist.linear.z = float(self.velocity[2])

        self.state_publisher_.publish(msg)

    def publish_target_marker(self):
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"

        marker.ns = "target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.position[0])
        marker.pose.position.y = float(self.position[1])
        marker.pose.position.z = float(self.position[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2

        self.marker_publisher_.publish(marker)

    def publish_path_marker(self):
        marker = Marker()

        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "world"

        marker.ns = "target_path"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.08

        marker.color.a = 1.0
        marker.color.r = 0.2
        marker.color.g = 0.8
        marker.color.b = 1.0

        marker.points = self.path_points

        self.path_marker_publisher_.publish(marker)

    def publish_target_tf(self):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "target"

        transform.transform.translation.x = float(self.position[0])
        transform.transform.translation.y = float(self.position[1])
        transform.transform.translation.z = float(self.position[2])

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)

    def timer_callback(self):
        old_position = self.position.copy()

        self.turn_towards_desired_heading()

        self.position += self.dt * self.velocity

        self.update_path_points(old_position, self.position)

        self.publish_odometry()
        self.publish_target_marker()
        self.publish_path_marker()
        self.publish_target_tf()


def main(args=None):
    rclpy.init(args=args)

    node = TargetDynamicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()