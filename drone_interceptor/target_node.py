import rclpy
import numpy as np
from rclpy.node import Node

from nav_msgs.msg import Odometry

class TargetNode(Node):
    def __init__(self):
        super().__init__("target_node")
        self.publisher_ = self.create_publisher(Odometry, "/target/state", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.dt = 0.5
        
        self.position = np.array([0.0, 0.0, 2.0], dtype=float)

        self.velocity = np.array([1.0, 0.5, 0.1], dtype=float)

        self.v_floor = 0.1
        self.v_max = 30.0  # m/s

        self.a_long_max = 5.0   # acceleration along current flight direction
        self.a_lat_max = 3.0    # acceleration perpendicular to current flight direction
        self.a_vert_max = 3.0   # vertical acceleration limit

        self.desired_heading = np.array([1.0, 1.0, 0.3], dtype=float)

        self.desired_heading = self.desired_heading / np.linalg.norm(self.desired_heading)

        self.compute_heading_from_velocity()



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

    def timer_callback(self):
        self.turn_towards_desired_heading()

        self.position += self.dt * self.velocity

        msg = Odometry()

        msg.pose.pose.position.x = float(self.position[0])
        msg.pose.pose.position.y = float(self.position[1])
        msg.pose.pose.position.z = float(self.position[2])

        msg.twist.twist.linear.x = float(self.velocity[0])
        msg.twist.twist.linear.y = float(self.velocity[1])
        msg.twist.twist.linear.z = float(self.velocity[2])

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.child_frame_id = "target"

        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = TargetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()