import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32


class TargetBehaviorNode(Node):
    def __init__(self):
        super().__init__("target_behavior_node")

        self.publisher_ = self.create_publisher(
            Vector3,
            "/target/desired_heading",
            10
        )

        self.speed_publisher_ = self.create_publisher(
            Float32,
            "/target/desired_speed",
            10
        )

        self.timer_period = 4
        self.timer = self.create_timer(
            self.timer_period,
            self.timer_callback
        )

    def timer_callback(self):
        direction = np.array([
            np.random.uniform(-1.0, 1.0),
            np.random.uniform(-1.0, 1.0),
            np.random.uniform(-0.2, 0.2),
        ])

        if np.linalg.norm(direction) < 1e-6:
            return

        heading_msg = Vector3()
        heading_msg.x = float(direction[0])
        heading_msg.y = float(direction[1])
        heading_msg.z = float(direction[2])

        speed_msg = Float32()
        speed_msg.data = float(np.random.uniform(1.0, 8.0))

        self.publisher_.publish(heading_msg)
        self.speed_publisher_.publish(speed_msg)


def main(args=None):
    rclpy.init(args=args)

    node = TargetBehaviorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()