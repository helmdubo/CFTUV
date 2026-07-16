"""Общая deterministic chart-space решётка для admission и PyVoronoi."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, isfinite, sqrt

from .constants import DECAL_WELD_DISTANCE


DIAGRAM_INT_LIMIT = (1 << 31) - 1
DIAGRAM_INT_SAFETY_MARGIN = 0.8
DIAGRAM_RELATIVE_QUANTUM = 1e-4


class DiagramTransformError(ValueError):
    def __init__(self, details):
        self.details = str(details)
        super().__init__(self.details)


@dataclass(frozen=True)
class DiagramTransform:
    """Детерминированная граница chart-space -> Boost integer space."""

    center: tuple[float, float]
    scale: float
    inv_scale: float
    quantum: float
    max_abs_input: float
    int_safety_margin: float

    def quantize(self, point):
        return tuple(
            self.center[axis]
            + round(
                (float(point[axis]) - self.center[axis]) / self.quantum
            )
            * self.quantum
            for axis in range(2)
        )

    def to_diagram(self, point):
        quantized = self.quantize(point)
        return (
            quantized[0] - self.center[0],
            quantized[1] - self.center[1],
        )

    def from_diagram(self, point):
        return (
            float(point[0]) + self.center[0],
            float(point[1]) + self.center[1],
        )


def build_diagram_transform(points):
    """Подбирает precision, не выводя centred input из Boost int range."""

    points = tuple(
        (float(point[0]), float(point[1])) for point in points
    )
    if not points or any(
        not all(isfinite(value) for value in point) for point in points
    ):
        raise DiagramTransformError("diagram input is empty or non-finite")
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    extent_x = max_x - min_x
    extent_y = max_y - min_y
    diagonal = sqrt(extent_x * extent_x + extent_y * extent_y)
    if not isfinite(diagonal) or diagonal <= 0.0:
        raise DiagramTransformError(f"degenerate chart extent={diagonal!r}")
    center = ((min_x + max_x) * 0.5, (min_y + max_y) * 0.5)
    desired_quantum = min(
        DECAL_WELD_DISTANCE * 0.1,
        diagonal * DIAGRAM_RELATIVE_QUANTUM,
    )
    if not isfinite(desired_quantum) or desired_quantum <= 0.0:
        raise DiagramTransformError(
            f"invalid desired quantum={desired_quantum!r}"
        )
    guard_margin = max(diagonal, desired_quantum * 32.0)
    max_abs_input = max(
        abs(min_x - guard_margin - center[0]),
        abs(max_x + guard_margin - center[0]),
        abs(min_y - guard_margin - center[1]),
        abs(max_y + guard_margin - center[1]),
    )
    safe_integer = DIAGRAM_INT_LIMIT * DIAGRAM_INT_SAFETY_MARGIN
    maximum_scale = min(safe_integer / max_abs_input, safe_integer)
    required_scale = float(ceil(1.0 / desired_quantum))
    preferred_scale = float(ceil(10.0 / desired_quantum))
    if (
        not isfinite(maximum_scale)
        or not isfinite(required_scale)
        or required_scale < 1.0
        or required_scale > maximum_scale
    ):
        raise DiagramTransformError(
            (
                f"extent={diagonal:.12g} desired_quantum="
                f"{desired_quantum:.12g} required_scale="
                f"{required_scale:.12g} maximum_scale="
                f"{maximum_scale:.12g} max_abs_input="
                f"{max_abs_input:.12g}"
            )
        )
    scale = float(int(min(preferred_scale, maximum_scale)))
    inv_scale = 1.0 / scale
    quantum_steps = max(1, int(desired_quantum * scale))
    quantum = quantum_steps * inv_scale
    return DiagramTransform(
        center=center,
        scale=scale,
        inv_scale=inv_scale,
        quantum=quantum,
        max_abs_input=max_abs_input,
        int_safety_margin=DIAGRAM_INT_SAFETY_MARGIN,
    )
