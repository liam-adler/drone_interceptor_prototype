from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class DistanceMetrics:
    capture_radius: float
    first_sample_time: float | None = None
    last_sample_time: float | None = None
    last_distance: float | None = None
    min_distance: float | None = None
    capture_time: float | None = None
    sample_count: int = 0
    samples: list[tuple[float, float]] = field(default_factory=list)

    def add_sample(self, *, timestamp: float, distance: float) -> tuple[float, bool, bool]:
        is_first_sample = self.first_sample_time is None
        if is_first_sample:
            self.first_sample_time = timestamp

        assert self.first_sample_time is not None
        elapsed = timestamp - self.first_sample_time

        self.last_sample_time = timestamp
        self.last_distance = distance
        self.sample_count += 1
        self.samples.append((elapsed, distance))

        if self.min_distance is None or distance < self.min_distance:
            self.min_distance = distance

        capture_just_reached = False
        if self.capture_time is None and distance <= self.capture_radius:
            self.capture_time = elapsed
            capture_just_reached = True

        return elapsed, is_first_sample, capture_just_reached

    def compute_elapsed_time(self) -> float:
        if self.first_sample_time is None or self.last_sample_time is None:
            return 0.0
        return self.last_sample_time - self.first_sample_time
