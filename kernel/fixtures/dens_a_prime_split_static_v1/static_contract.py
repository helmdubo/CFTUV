"""Proof-only closed decoder and recomposer for conveyor static state.

The module deliberately has no product imports.  Product objects may cross
the decoder boundary as opaque, exactly typed values, but no live preparation
object is accepted or reconstructed here.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass
from decimal import Decimal, InvalidOperation
from fractions import Fraction
from typing import Any


STATIC_TYPE = "ConveyorStaticPreparationV1"
STATIC_SCHEMA_VERSION = "cftuv.envelope.conveyor_static_preparation.v1"
STATIC_FIELDS = (
    "$type",
    "schema_version",
    "outcome",
    "regions",
    "lattice",
    "arrival_laws",
    "law_names",
)
FORBIDDEN_STATIC_FIELDS = frozenset(
    {
        "compilation",
        "context",
        "domain",
        "requested_alpha",
        "counters",
        "detail",
        "timings",
        "decal_request_id",
    }
)
OUTCOME_VALUES = frozenset(
    {
        "EXACT",
        "PLAN_IS_NOT_COMPILED",
        "DOMAIN_FRAME_IS_UNAVAILABLE",
        "DOMAIN_HAS_NO_REGIONS",
        "DOMAIN_POINT_IS_NOT_RATIONAL",
        "ARRIVAL_LAW_IS_NOT_RATIONAL",
        "NO_STRIP_SOURCE_IN_THE_PLAN",
        "CHART_LATTICE_IS_NOT_DECLARED",
        "DOMAIN_GEOMETRY_REFUSED",
        "BRIDGE_DID_NOT_MAP",
        "SKELETON_DID_NOT_CLOSE",
        "FACES_DID_NOT_ASSEMBLE",
        "PREPARATION_IS_NOT_EXACT",
        "COVERAGE_DID_NOT_CLOSE",
        "SHARED_ENVELOPE_MIXED_ALPHA_UNPROVEN",
    }
)


class StaticContractReject(ValueError):
    """Именованный fail-closed отказ proof-контракта."""

    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


def _reject(code: str) -> None:
    raise StaticContractReject(code)


def _exact_product_type(value: object, module: str, name: str) -> bool:
    cls = type(value)
    return cls.__module__ == module and cls.__name__ == name


def _validate_outcome(value: object) -> None:
    if not _exact_product_type(
        value,
        "cftuv_envelope.wavefront.conveyor",
        "ConveyorOutcome",
    ):
        _reject("STATIC_OUTCOME_ENUM_MISMATCH")
    if getattr(value, "value", None) not in OUTCOME_VALUES:
        _reject("STATIC_OUTCOME_ENUM_MISMATCH")


def _validate_regions(value: object) -> None:
    if type(value) is not tuple:
        _reject("STATIC_REGIONS_TYPE_MISMATCH")
    if any(
        not _exact_product_type(
            item,
            "cftuv_envelope.wavefront.conveyor",
            "PreparedRegionV1",
        )
        for item in value
    ):
        _reject("STATIC_REGIONS_TYPE_MISMATCH")


def _validate_lattice(value: object) -> None:
    if value is None:
        return
    if not _exact_product_type(
        value,
        "cftuv_envelope.robust.grid",
        "GridSpecV1",
    ):
        _reject("STATIC_LATTICE_TYPE_MISMATCH")


def _validate_arrival_laws(
    value: object,
    law_names: object,
) -> None:
    if type(value) is not tuple:
        _reject("STATIC_ARRIVAL_LAWS_TYPE_MISMATCH")
    if type(law_names) is not tuple or any(
        type(item) is not str or not item for item in law_names
    ):
        _reject("STATIC_LAW_NAMES_TYPE_MISMATCH")
    names: list[str] = []
    expected_fields = (
        "name",
        "normal_x",
        "normal_y",
        "constant",
        "speed_squared",
    )
    for law in value:
        if not _exact_product_type(
            law,
            "cftuv_envelope.wavefront.bridge",
            "PlainArrivalLawV1",
        ):
            _reject("STATIC_ARRIVAL_LAW_TYPE_MISMATCH")
        if not is_dataclass(law) or tuple(
            item.name for item in fields(law)
        ) != expected_fields:
            _reject("STATIC_ARRIVAL_LAW_SCHEMA_MISMATCH")
        if type(law.name) is not str or not law.name:
            _reject("STATIC_ARRIVAL_LAW_SCHEMA_MISMATCH")
        rationals = (
            law.normal_x,
            law.normal_y,
            law.constant,
            law.speed_squared,
        )
        if any(type(item) is not Fraction for item in rationals):
            _reject("STATIC_ARRIVAL_LAW_SCHEMA_MISMATCH")
        normal_squared = law.normal_x**2 + law.normal_y**2
        if normal_squared <= 0 or law.speed_squared <= 0:
            _reject("STATIC_ARRIVAL_LAW_MISMATCH")
        names.append(law.name)
    if len(names) != len(set(names)):
        _reject("STATIC_ARRIVAL_LAW_MISMATCH")
    if tuple(names) != law_names:
        _reject("STATIC_ARRIVAL_LAW_MISMATCH")


@dataclass(frozen=True, slots=True)
class ConveyorStaticPreparationV1:
    """Единственный декодированный источник статических полей."""

    outcome: object
    regions: tuple[object, ...]
    lattice: object | None
    arrival_laws: tuple[object, ...]
    law_names: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ConveyorStaticRecompositionV1:
    """Статические поля DTO плюс объявленная per-request alpha."""

    outcome: object
    regions: tuple[object, ...]
    lattice: object | None
    arrival_laws: tuple[object, ...]
    law_names: tuple[str, ...]
    alpha: Decimal


def decode_conveyor_static(
    record: dict[str, Any],
) -> ConveyorStaticPreparationV1:
    """Декодировать только exact closed `ConveyorStaticPreparationV1`."""

    if type(record) is not dict:
        _reject("STATIC_DTO_NOT_RECORD")
    actual = set(record)
    expected = set(STATIC_FIELDS)
    missing = expected - actual
    if missing:
        _reject("STATIC_REQUIRED_FIELD_MISSING")
    extra = actual - expected
    if extra:
        if extra.issubset(FORBIDDEN_STATIC_FIELDS):
            _reject("STATIC_EXTRA_FIELD")
        _reject("STATIC_UNKNOWN_FIELD")
    if record["$type"] != STATIC_TYPE:
        _reject("FORGED_STATIC_TYPE")
    if record["schema_version"] != STATIC_SCHEMA_VERSION:
        _reject("UNKNOWN_STATIC_SCHEMA")
    _validate_outcome(record["outcome"])
    _validate_regions(record["regions"])
    _validate_lattice(record["lattice"])
    _validate_arrival_laws(
        record["arrival_laws"],
        record["law_names"],
    )
    return ConveyorStaticPreparationV1(
        outcome=record["outcome"],
        regions=record["regions"],
        lattice=record["lattice"],
        arrival_laws=record["arrival_laws"],
        law_names=record["law_names"],
    )


def _declared_alpha(alpha: object) -> Decimal:
    if isinstance(alpha, bool) or not isinstance(alpha, (Decimal, int, str)):
        _reject("STATIC_ALPHA_TYPE_MISMATCH")
    try:
        value = alpha if isinstance(alpha, Decimal) else Decimal(str(alpha))
    except (InvalidOperation, ValueError):
        _reject("STATIC_ALPHA_VALUE_MISMATCH")
    if not value.is_finite() or value < 0:
        _reject("STATIC_ALPHA_VALUE_MISMATCH")
    return value


def recompose_conveyor_static(
    static_preparation: ConveyorStaticPreparationV1,
    alpha: object,
) -> ConveyorStaticRecompositionV1:
    """Recompose from exactly `(decoded static DTO, alpha)`."""

    if type(static_preparation) is not ConveyorStaticPreparationV1:
        _reject("STATIC_RECOMPOSE_DECODED_REQUIRED")
    return ConveyorStaticRecompositionV1(
        outcome=static_preparation.outcome,
        regions=static_preparation.regions,
        lattice=static_preparation.lattice,
        arrival_laws=static_preparation.arrival_laws,
        law_names=static_preparation.law_names,
        alpha=_declared_alpha(alpha),
    )
