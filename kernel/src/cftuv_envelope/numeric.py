"""Числовые value objects без Blender/mathutils и без округления identity."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from math import gcd, isfinite

from .ids import SymbolicScalarId


class MetricSpace(str, Enum):
    SOURCE_LOCAL_INTRINSIC = "SOURCE_LOCAL_INTRINSIC"


class ExactAngleSymbol(str, Enum):
    PI_OVER_3 = "PI_OVER_3"


class SurfaceCoordinateUnavailableReason(str, Enum):
    EC0_COORDINATE_FREE_FIXTURE_V5 = "EC0_COORDINATE_FREE_FIXTURE_V5"


@dataclass(frozen=True, slots=True)
class LocalPoint3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("LocalPoint3V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class LocalVector3V1:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("LocalVector3V1 requires finite components")


@dataclass(frozen=True, slots=True)
class UvPoint2V1:
    u: float
    v: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.u, self.v)):
            raise ValueError("UvPoint2V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class LocalLengthV1:
    value: Decimal
    metric_space: MetricSpace = MetricSpace.SOURCE_LOCAL_INTRINSIC

    def __post_init__(self) -> None:
        if not isinstance(self.value, Decimal) or not self.value.is_finite():
            raise ValueError("LocalLengthV1 requires a finite Decimal")
        if self.value < 0:
            raise ValueError("LocalLengthV1 cannot be negative")


@dataclass(frozen=True, slots=True)
class SymbolicLocalLengthV1:
    """Явная coordinate-free величина только для принятого EC0 oracle."""

    symbol: SymbolicScalarId
    metric_space: MetricSpace = MetricSpace.SOURCE_LOCAL_INTRINSIC


MetricLengthV1 = LocalLengthV1 | SymbolicLocalLengthV1


@dataclass(frozen=True, slots=True)
class ExactAngleV1:
    symbol: ExactAngleSymbol


@dataclass(frozen=True, slots=True)
class ExactRatioV1:
    numerator: int
    denominator: int

    def __post_init__(self) -> None:
        if self.denominator <= 0:
            raise ValueError("ExactRatioV1 denominator must be positive")
        if self.numerator < 0:
            raise ValueError("ExactRatioV1 numerator cannot be negative")
        if gcd(self.numerator, self.denominator) != 1:
            raise ValueError("ExactRatioV1 must be reduced")


@dataclass(frozen=True, slots=True)
class UnavailableSourcePositionV1:
    reason: SurfaceCoordinateUnavailableReason


SourcePositionV1 = LocalPoint3V1 | UnavailableSourcePositionV1

