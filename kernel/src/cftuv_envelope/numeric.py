"""Числовые value objects без Blender/mathutils и без округления identity."""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal
from enum import Enum
from fractions import Fraction
from math import gcd, isfinite

from .ids import SymbolicScalarId


def canonical_primitive_normal(vector):
    """Каноническая примитивная форма нормали: целые, gcd = 1, знак объявлен.

    Нормаль задаёт ПЛОСКОСТЬ, а плоскость не меняется от умножения нормали на
    ненулевое число. Поэтому запись нормали обязана быть канонической, иначе
    один и тот же геометрический факт печатается разными числами в зависимости
    от того, каким выводом его получили, и дайджест начинает различать то, что
    геометрически неразличимо.

    Знак: первая ненулевая координата положительна. Ориентация карты этим не
    теряется — она живёт отдельно, в `chart_orientation`, и выводится из знака
    площади граней в координатах карты. Привязывать знак нормали ещё и к обходу
    граней значило бы хранить одну величину дважды.

    Живёт в `numeric`, а не рядом с построителем метрики, потому что её читают
    оба конца: построитель — чтобы записать нормаль, валидатор — чтобы
    проверить, что записанная нормаль канонична. Общий предок у них только
    здесь.
    """

    denominator = 1
    for item in vector:
        denominator = denominator * item.denominator // gcd(
            denominator, item.denominator
        )
    integers = [int(item * denominator) for item in vector]
    divisor = 0
    for item in integers:
        divisor = gcd(divisor, abs(item))
    if divisor == 0:
        raise ValueError("patch polygons do not define a non-zero plane normal")
    integers = [item // divisor for item in integers]
    if next(item for item in integers if item) < 0:
        integers = [-item for item in integers]
    return tuple(Fraction(item) for item in integers)


class MetricSpace(str, Enum):
    SOURCE_LOCAL_INTRINSIC = "SOURCE_LOCAL_INTRINSIC"


class ExactAngleSymbol(str, Enum):
    PI_OVER_2 = "PI_OVER_2"
    PI_OVER_3 = "PI_OVER_3"
    PI_OVER_4 = "PI_OVER_4"
    PI_OVER_5 = "PI_OVER_5"
    PI_OVER_6 = "PI_OVER_6"


class SurfaceCoordinateUnavailableReason(str, Enum):
    EC0_COORDINATE_FREE_FIXTURE_V5 = "EC0_COORDINATE_FREE_FIXTURE_V5"


class IntervalEndpointKind(str, Enum):
    OPEN = "OPEN"
    CLOSED = "CLOSED"


@dataclass(frozen=True, slots=True)
class LocalPoint2V1:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x, self.y)):
            raise ValueError("LocalPoint2V1 requires finite coordinates")


@dataclass(frozen=True, slots=True)
class LocalVector2V1:
    x: float
    y: float

    def __post_init__(self) -> None:
        if not all(isfinite(value) for value in (self.x, self.y)):
            raise ValueError("LocalVector2V1 requires finite components")


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
class LocalCoordinateV1:
    """Signed metric coordinate for semantic station/transverse facts."""

    value: Decimal
    metric_space: MetricSpace = MetricSpace.SOURCE_LOCAL_INTRINSIC

    def __post_init__(self) -> None:
        if not isinstance(self.value, Decimal) or not self.value.is_finite():
            raise ValueError("LocalCoordinateV1 requires a finite Decimal")


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
class CertifiedDecimalIntervalV1:
    """Exact decimal bounds supplied by an external measurement authority."""

    lower: Decimal
    upper: Decimal
    lower_kind: IntervalEndpointKind
    upper_kind: IntervalEndpointKind
    absolute_error_bound: Decimal

    def __post_init__(self) -> None:
        values = (self.lower, self.upper, self.absolute_error_bound)
        if not all(isinstance(value, Decimal) and value.is_finite() for value in values):
            raise ValueError("CertifiedDecimalIntervalV1 requires finite Decimals")
        if self.lower > self.upper:
            raise ValueError("CertifiedDecimalIntervalV1 lower bound exceeds upper bound")
        if self.absolute_error_bound < 0:
            raise ValueError("CertifiedDecimalIntervalV1 error bound cannot be negative")
        if self.lower == self.upper and (
            self.lower_kind is IntervalEndpointKind.OPEN
            or self.upper_kind is IntervalEndpointKind.OPEN
        ):
            raise ValueError("A zero-width certified interval must be closed")


@dataclass(frozen=True, slots=True)
class UnavailableSourcePositionV1:
    reason: SurfaceCoordinateUnavailableReason


SourcePositionV1 = LocalPoint3V1 | UnavailableSourcePositionV1
