from __future__ import annotations


def classify_failure_reason(*, captured: bool) -> str | None:
    if captured:
        return None
    return "capture_not_reached"
