#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
import os
from pathlib import Path
import tempfile
from typing import Optional

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "drone_interceptor_mpl"),
)

import matplotlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

matplotlib.use("Agg")
import matplotlib.pyplot as plt


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

        self.first_sample_time: Optional[float] = None
        self.last_sample_time: Optional[float] = None
        self.last_distance: Optional[float] = None
        self.min_distance: Optional[float] = None
        self.capture_time: Optional[float] = None
        self.capture_announced = False
        self.summary_logged = False
        self.sample_count = 0
        self.samples: list[tuple[float, float]] = []

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

        if self.first_sample_time is None:
            self.first_sample_time = now
            self.get_logger().info(
                f"[{self.controller_label}] first distance sample: {distance:.3f} m"
            )

        self.last_sample_time = now
        self.last_distance = distance
        self.sample_count += 1
        self.samples.append((now - self.first_sample_time, distance))

        if self.min_distance is None or distance < self.min_distance:
            self.min_distance = distance

        if self.capture_time is None and distance <= self.capture_radius:
            self.capture_time = now - self.first_sample_time
            if not self.capture_announced:
                self.capture_announced = True
                self.get_logger().info(
                    f"[{self.controller_label}] capture reached in "
                    f"{self.capture_time:.2f} s at distance {distance:.3f} m"
                )

    def report_timer_callback(self) -> None:
        if self.first_sample_time is None or self.last_distance is None:
            return

        elapsed = self.compute_elapsed_time()
        capture_text = (
            f"{self.capture_time:.2f} s" if self.capture_time is not None else "not yet"
        )
        self.get_logger().info(
            f"[{self.controller_label}] elapsed={elapsed:.1f}s "
            f"current={self.last_distance:.3f}m "
            f"min={self.min_distance:.3f}m "
            f"capture={capture_text}"
        )

    def log_summary(self) -> None:
        if self.summary_logged or self.first_sample_time is None:
            return

        self.summary_logged = True
        elapsed = self.compute_elapsed_time()

        capture_status = (
            f"yes ({self.capture_time:.2f} s)"
            if self.capture_time is not None
            else "no"
        )
        min_distance = self.min_distance if self.min_distance is not None else float("nan")
        last_distance = (
            self.last_distance if self.last_distance is not None else float("nan")
        )

        self.get_logger().info(
            f"[{self.controller_label}] final summary: "
            f"captured={capture_status}, "
            f"min_distance={min_distance:.3f} m, "
            f"final_distance={last_distance:.3f} m, "
            f"elapsed={elapsed:.2f} s, "
            f"samples={self.sample_count}"
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

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["time_s", "distance_m"])
            writer.writerows(self.samples)

        summary = self.build_summary(
            elapsed=elapsed,
            min_distance=min_distance,
            last_distance=last_distance,
        )
        with json_path.open("w", encoding="utf-8") as json_file:
            json.dump(summary, json_file, indent=2)

        self.write_plot(png_path)

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
            "captured": self.capture_time is not None,
            "capture_time_s": self.capture_time,
            "min_distance_m": min_distance,
            "final_distance_m": last_distance,
            "elapsed_s": elapsed,
            "sample_count": self.sample_count,
            "distance_topic": self.get_string_parameter("distance_topic"),
            "capture_radius_m": self.capture_radius,
            "run_label": self.run_label,
        }

    def write_plot(self, png_path: Path) -> None:
        times = [sample[0] for sample in self.samples]
        distances = [sample[1] for sample in self.samples]

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.plot(times, distances, color="#1f77b4", linewidth=2.0, label="distance")
        ax.axhline(
            self.capture_radius,
            color="#d62728",
            linestyle="--",
            linewidth=1.5,
            label="capture radius",
        )
        ax.fill_between(
            times,
            0.0,
            self.capture_radius,
            color="#d62728",
            alpha=0.08,
        )
        if self.capture_time is not None:
            capture_distance = self.capture_radius
            for time_value, distance_value in self.samples:
                if time_value >= self.capture_time:
                    capture_distance = distance_value
                    break

            ax.scatter(
                [self.capture_time],
                [capture_distance],
                color="#2ca02c",
                s=70,
                zorder=5,
                label="capture",
            )
            ax.axvline(
                self.capture_time,
                color="#2ca02c",
                linestyle=":",
                linewidth=1.4,
                alpha=0.9,
            )
        ax.set_title(f"Intercept Distance - {self.run_label}")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Distance [m]")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(png_path, dpi=160)
        plt.close(fig)

    def compute_elapsed_time(self) -> float:
        if self.first_sample_time is None or self.last_sample_time is None:
            return 0.0
        return self.last_sample_time - self.first_sample_time

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
