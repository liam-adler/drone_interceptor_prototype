#!/usr/bin/env python3

from __future__ import annotations

import os
from pathlib import Path
import tempfile

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "drone_interceptor_mpl"),
)

import matplotlib  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float32  # noqa: E402

matplotlib.use("Agg")

from drone_interceptor.evaluation.metrics import DistanceMetrics  # noqa: E402
from drone_interceptor.evaluation.result_logger import (  # noqa: E402
    write_distance_csv,
    write_distance_plot,
    write_summary_json,
)


class ResultsLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("results_logger_node")

        self.declare_parameter("distance_topic", "/intercept/distance")
        self.declare_parameter("controller_label", "baseline")
        self.declare_parameter("capture_radius", 0.5)
        self.declare_parameter("report_period", 5.0)
        self.declare_parameter("output_dir", "results/drone_interceptor")
        self.declare_parameter("run_label", "baseline_run")
        self.declare_parameter("profile_name", "unknown")
        self.declare_parameter("controller_mode", "unknown")
        self.declare_parameter("threat_response", True)
        self.declare_parameter("spawn_distance", 0.0)
        self.declare_parameter("random_seed", 0)

        distance_topic = self.get_string_parameter("distance_topic")
        self.controller_label = self.get_string_parameter("controller_label")
        self.capture_radius = self.get_float_parameter("capture_radius")
        report_period = self.get_float_parameter("report_period")
        self.output_dir = Path(self.get_string_parameter("output_dir")).expanduser()
        self.run_label = self.get_string_parameter("run_label")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.metrics = DistanceMetrics(capture_radius=self.capture_radius)
        self.capture_announced = False
        self.summary_logged = False

        self.create_subscription(
            Float32,
            distance_topic,
            self.distance_callback,
            10,
        )

        self.report_timer = self.create_timer(report_period, self.report_timer_callback)

        self.get_logger().info(
            "Results logger started for "
            f"controller='{self.controller_label}' on topic '{distance_topic}'"
        )

    def distance_callback(self, msg: Float32) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        distance = float(msg.data)

        elapsed, is_first_sample, capture_just_reached = self.metrics.add_sample(
            timestamp=now,
            distance=distance,
        )

        if is_first_sample:
            self.get_logger().info(
                f"[{self.controller_label}] first distance sample: {distance:.3f} m"
            )

        if capture_just_reached:
            if not self.capture_announced:
                self.capture_announced = True
                self.get_logger().info(
                    f"[{self.controller_label}] capture reached in "
                    f"{elapsed:.2f} s at distance {distance:.3f} m"
                )

    def report_timer_callback(self) -> None:
        if (
            self.metrics.first_sample_time is None
            or self.metrics.last_distance is None
            or self.metrics.min_distance is None
        ):
            return

        elapsed = self.metrics.compute_elapsed_time()
        capture_text = (
            f"{self.metrics.capture_time:.2f} s"
            if self.metrics.capture_time is not None
            else "not yet"
        )
        self.get_logger().info(
            f"[{self.controller_label}] elapsed={elapsed:.1f}s "
            f"current={self.metrics.last_distance:.3f}m "
            f"min={self.metrics.min_distance:.3f}m "
            f"capture={capture_text}"
        )

    def log_summary(self) -> None:
        if self.summary_logged or self.metrics.first_sample_time is None:
            return

        self.summary_logged = True
        elapsed = self.metrics.compute_elapsed_time()

        capture_status = (
            f"yes ({self.metrics.capture_time:.2f} s)"
            if self.metrics.capture_time is not None
            else "no"
        )
        min_distance = (
            self.metrics.min_distance
            if self.metrics.min_distance is not None
            else float("nan")
        )
        last_distance = (
            self.metrics.last_distance
            if self.metrics.last_distance is not None
            else float("nan")
        )

        self.get_logger().info(
            f"[{self.controller_label}] final summary: "
            f"captured={capture_status}, "
            f"capture_events={self.metrics.capture_event_count}, "
            f"min_distance={min_distance:.3f} m, "
            f"final_distance={last_distance:.3f} m, "
            f"elapsed={elapsed:.2f} s, "
            f"samples={self.metrics.sample_count}"
        )
        self.write_artifacts(
            elapsed=elapsed,
            min_distance=min_distance,
            last_distance=last_distance,
        )

    def write_artifacts(
        self,
        elapsed: float,
        min_distance: float,
        last_distance: float,
    ) -> None:
        csv_path = self.output_dir / f"{self.run_label}.csv"
        json_path = self.output_dir / f"{self.run_label}.json"
        png_path = self.output_dir / f"{self.run_label}.png"

        summary = self.build_summary(
            elapsed=elapsed,
            min_distance=min_distance,
            last_distance=last_distance,
        )
        write_distance_csv(csv_path, self.metrics.samples)
        write_summary_json(json_path, summary)
        write_distance_plot(
            png_path,
            run_label=self.run_label,
            capture_radius=self.capture_radius,
            samples=self.metrics.samples,
            capture_time=self.metrics.capture_time,
        )

        self.get_logger().info(
            f"[{self.controller_label}] wrote artifacts: "
            f"{csv_path}, {json_path}, {png_path}"
        )

    def build_summary(
        self,
        elapsed: float,
        min_distance: float,
        last_distance: float,
    ) -> dict[str, object]:
        return {
            "controller_label": self.controller_label,
            "controller_mode": self.get_string_parameter("controller_mode"),
            "profile_name": self.get_string_parameter("profile_name"),
            "threat_response": self.get_bool_parameter("threat_response"),
            "spawn_distance_m": self.get_float_parameter("spawn_distance"),
            "random_seed": self.get_int_parameter("random_seed"),
            "captured": self.metrics.capture_time is not None,
            "capture_time_s": self.metrics.capture_time,
            "capture_event_count": self.metrics.capture_event_count,
            "min_distance_m": min_distance,
            "final_distance_m": last_distance,
            "elapsed_s": elapsed,
            "sample_count": self.metrics.sample_count,
            "distance_topic": self.get_string_parameter("distance_topic"),
            "capture_radius_m": self.capture_radius,
            "run_label": self.run_label,
        }

    def get_bool_parameter(self, name: str) -> bool:
        return bool(self.get_parameter(name).value)

    def get_float_parameter(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def get_int_parameter(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def get_string_parameter(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def destroy_node(self) -> bool:
        self.log_summary()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ResultsLoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
