"""Proof-only wire decoder and recomposer for conveyor static state.

The decoder accepts exact built-in JSON value types only.  Product instances,
custom mappings, subclasses and objects that spoof Python type metadata never
cross the boundary.  Decoded values are proof-local and deeply immutable.
"""

from __future__ import annotations

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation
from fractions import Fraction
import json
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
REGION_FIELDS = frozenset(
    {
        "$type",
        "ambiguous_owner_spans",
        "bridge",
        "bridge_outcome",
        "degraded_miter_corners",
        "face_outcome",
        "owner_by_edge",
        "partition",
        "region_id",
        "skeleton",
        "skeleton_outcome",
        "wall_edge_count",
        "wall_spans",
    }
)
LATTICE_FIELDS = frozenset({"$type", "scale", "magnitude_bound"})
ARRIVAL_LAW_FIELDS = frozenset(
    {
        "$type",
        "name",
        "normal_x",
        "normal_y",
        "constant",
        "speed_squared",
    }
)
FRACTION_FIELDS = frozenset({"$type", "numerator", "denominator"})


class StaticContractReject(ValueError):
    """Именованный fail-closed отказ proof-контракта."""

    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


def _reject(code: str) -> None:
    raise StaticContractReject(code)


def _invalid_json_constant(_value: str) -> None:
    raise ValueError("non-standard JSON constant")


def _exact_keys(
    value: dict[str, Any],
    expected: frozenset[str],
    code: str,
) -> None:
    if any(type(key) is not str for key in value):
        _reject("STATIC_FIELD_NAME_NOT_EXACT_STRING")
    if set(value) != expected:
        _reject(code)


@dataclass(frozen=True, slots=True)
class FrozenWireRecordV1:
    """Канонически упорядоченная immutable JSON mapping."""

    items: tuple[tuple[str, object], ...]


def _freeze_wire(value: object) -> object:
    if type(value) is dict:
        if any(type(key) is not str for key in value):
            _reject("STATIC_FIELD_NAME_NOT_EXACT_STRING")
        return FrozenWireRecordV1(
            tuple(
                (key, _freeze_wire(value[key]))
                for key in sorted(value)
            )
        )
    if type(value) is list:
        return tuple(_freeze_wire(item) for item in value)
    if (
        value is None
        or type(value) is str
        or type(value) is int
        or type(value) is bool
    ):
        return value
    _reject("STATIC_WIRE_VALUE_TYPE_MISMATCH")


def _thaw_wire(value: object) -> object:
    if type(value) is FrozenWireRecordV1:
        return {key: _thaw_wire(item) for key, item in value.items}
    if type(value) is tuple:
        return [_thaw_wire(item) for item in value]
    return value


@dataclass(frozen=True, slots=True)
class ConveyorOutcomeValueV1:
    """Proof-local outcome; никогда не является product Enum."""

    value: str


@dataclass(frozen=True, slots=True)
class GridSpecValueV1:
    scale: int
    magnitude_bound: int


@dataclass(frozen=True, slots=True)
class PlainArrivalLawValueV1:
    name: str
    normal_x: Fraction
    normal_y: Fraction
    constant: Fraction
    speed_squared: Fraction


@dataclass(frozen=True, slots=True)
class ConveyorStaticPreparationV1:
    """Единственный декодированный источник статических полей."""

    outcome: ConveyorOutcomeValueV1
    regions: tuple[FrozenWireRecordV1, ...]
    lattice: GridSpecValueV1 | None
    arrival_laws: tuple[PlainArrivalLawValueV1, ...]
    law_names: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class ConveyorStaticRecompositionV1:
    """Статические поля DTO плюс объявленная per-request alpha."""

    outcome: ConveyorOutcomeValueV1
    regions: tuple[FrozenWireRecordV1, ...]
    lattice: GridSpecValueV1 | None
    arrival_laws: tuple[PlainArrivalLawValueV1, ...]
    law_names: tuple[str, ...]
    alpha: Decimal


def _decode_outcome(value: object) -> ConveyorOutcomeValueV1:
    if type(value) is not dict:
        _reject("STATIC_OUTCOME_NOT_EXACT_DICT")
    _exact_keys(
        value,
        frozenset({"$enum_type", "value"}),
        "STATIC_OUTCOME_ENUM_SCHEMA_MISMATCH",
    )
    if (
        type(value["$enum_type"]) is not str
        or type(value["value"]) is not str
    ):
        _reject("STATIC_OUTCOME_ENUM_STRING_TYPE_MISMATCH")
    if value["$enum_type"] != "ConveyorOutcome":
        _reject("STATIC_OUTCOME_ENUM_MISMATCH")
    if value["value"] not in OUTCOME_VALUES:
        _reject("STATIC_OUTCOME_ENUM_MISMATCH")
    return ConveyorOutcomeValueV1(value=value["value"])


def _decode_regions(value: object) -> tuple[FrozenWireRecordV1, ...]:
    if type(value) is not list:
        _reject("STATIC_REGIONS_TYPE_MISMATCH")
    output = []
    for region in value:
        if type(region) is not dict:
            _reject("STATIC_REGION_NOT_EXACT_DICT")
        _exact_keys(
            region,
            REGION_FIELDS,
            "STATIC_REGION_SCHEMA_MISMATCH",
        )
        if type(region["$type"]) is not str:
            _reject("STATIC_REGION_TYPE_NOT_EXACT_STRING")
        if region["$type"] != "PreparedRegionV1":
            _reject("STATIC_REGION_TYPE_MISMATCH")
        frozen = _freeze_wire(region)
        if type(frozen) is not FrozenWireRecordV1:
            _reject("STATIC_REGION_NOT_EXACT_DICT")
        output.append(frozen)
    return tuple(output)


def _decode_lattice(value: object) -> GridSpecValueV1 | None:
    if value is None:
        return None
    if type(value) is not dict:
        _reject("STATIC_LATTICE_TYPE_MISMATCH")
    _exact_keys(value, LATTICE_FIELDS, "STATIC_LATTICE_SCHEMA_MISMATCH")
    if type(value["$type"]) is not str:
        _reject("STATIC_LATTICE_TYPE_NOT_EXACT_STRING")
    if value["$type"] != "GridSpecV1":
        _reject("STATIC_LATTICE_TYPE_MISMATCH")
    if (
        type(value["scale"]) is not int
        or type(value["magnitude_bound"]) is not int
        or value["scale"] <= 0
        or value["magnitude_bound"] < 0
    ):
        _reject("STATIC_LATTICE_SCHEMA_MISMATCH")
    return GridSpecValueV1(
        scale=value["scale"],
        magnitude_bound=value["magnitude_bound"],
    )


def _decode_fraction(value: object) -> Fraction:
    if type(value) is not dict:
        _reject("STATIC_ARRIVAL_LAW_FRACTION_TYPE_MISMATCH")
    _exact_keys(
        value,
        FRACTION_FIELDS,
        "STATIC_ARRIVAL_LAW_FRACTION_SCHEMA_MISMATCH",
    )
    if type(value["$type"]) is not str:
        _reject("STATIC_ARRIVAL_LAW_FRACTION_SCHEMA_MISMATCH")
    if value["$type"] != "ExactFractionV1":
        _reject("STATIC_ARRIVAL_LAW_FRACTION_SCHEMA_MISMATCH")
    numerator = value["numerator"]
    denominator = value["denominator"]
    if (
        type(numerator) is not int
        or type(denominator) is not int
        or denominator <= 0
    ):
        _reject("STATIC_ARRIVAL_LAW_FRACTION_SCHEMA_MISMATCH")
    result = Fraction(numerator, denominator)
    if (
        result.numerator != numerator
        or result.denominator != denominator
    ):
        _reject("STATIC_ARRIVAL_LAW_FRACTION_NOT_CANONICAL")
    return result


def _decode_arrival_laws(
    value: object,
    law_names: object,
) -> tuple[PlainArrivalLawValueV1, ...]:
    if type(value) is not list:
        _reject("STATIC_ARRIVAL_LAWS_TYPE_MISMATCH")
    if type(law_names) is not list or any(
        type(item) is not str or not item for item in law_names
    ):
        _reject("STATIC_LAW_NAMES_TYPE_MISMATCH")
    output = []
    for law in value:
        if type(law) is not dict:
            _reject("STATIC_ARRIVAL_LAW_NOT_EXACT_DICT")
        _exact_keys(
            law,
            ARRIVAL_LAW_FIELDS,
            "STATIC_ARRIVAL_LAW_SCHEMA_MISMATCH",
        )
        if (
            type(law["$type"]) is not str
            or law["$type"] != "PlainArrivalLawV1"
            or type(law["name"]) is not str
            or not law["name"]
        ):
            _reject("STATIC_ARRIVAL_LAW_SCHEMA_MISMATCH")
        decoded = PlainArrivalLawValueV1(
            name=law["name"],
            normal_x=_decode_fraction(law["normal_x"]),
            normal_y=_decode_fraction(law["normal_y"]),
            constant=_decode_fraction(law["constant"]),
            speed_squared=_decode_fraction(law["speed_squared"]),
        )
        if (
            decoded.normal_x**2 + decoded.normal_y**2 <= 0
            or decoded.speed_squared <= 0
        ):
            _reject("STATIC_ARRIVAL_LAW_MISMATCH")
        output.append(decoded)
    names = tuple(item.name for item in output)
    if len(names) != len(set(names)) or names != tuple(law_names):
        _reject("STATIC_ARRIVAL_LAW_MISMATCH")
    return tuple(output)


def _decode_wire_record(
    record: object,
) -> ConveyorStaticPreparationV1:
    if any(type(key) is not str for key in record):
        _reject("STATIC_FIELD_NAME_NOT_EXACT_STRING")
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
    if type(record["$type"]) is not str:
        _reject("STATIC_TYPE_NOT_EXACT_STRING")
    if record["$type"] != STATIC_TYPE:
        _reject("FORGED_STATIC_TYPE")
    if type(record["schema_version"]) is not str:
        _reject("STATIC_SCHEMA_NOT_EXACT_STRING")
    if record["schema_version"] != STATIC_SCHEMA_VERSION:
        _reject("UNKNOWN_STATIC_SCHEMA")
    outcome = _decode_outcome(record["outcome"])
    regions = _decode_regions(record["regions"])
    lattice = _decode_lattice(record["lattice"])
    arrival_laws = _decode_arrival_laws(
        record["arrival_laws"],
        record["law_names"],
    )
    return ConveyorStaticPreparationV1(
        outcome=outcome,
        regions=regions,
        lattice=lattice,
        arrival_laws=arrival_laws,
        law_names=tuple(record["law_names"]),
    )


def decode_conveyor_static(
    payload: bytes,
) -> ConveyorStaticPreparationV1:
    """Декодировать только canonical UTF-8 JSON `bytes`."""

    if type(payload) is not bytes:
        _reject("STATIC_DTO_BYTES_REQUIRED")
    try:
        record = json.loads(
            payload,
            parse_constant=_invalid_json_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
        _reject("STATIC_DTO_JSON_INVALID")
    if type(record) is not dict:
        _reject("STATIC_DTO_NOT_EXACT_DICT")
    canonical = json.dumps(
        record,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    if canonical != payload:
        _reject("STATIC_DTO_JSON_NOT_CANONICAL")
    return _decode_wire_record(record)


def _fraction_to_wire(value: Fraction) -> dict[str, object]:
    return {
        "$type": "ExactFractionV1",
        "numerator": value.numerator,
        "denominator": value.denominator,
    }


def _static_fields_to_wire(
    outcome: ConveyorOutcomeValueV1,
    regions: tuple[FrozenWireRecordV1, ...],
    lattice: GridSpecValueV1 | None,
    arrival_laws: tuple[PlainArrivalLawValueV1, ...],
    law_names: tuple[str, ...],
) -> dict[str, object]:
    return {
        "$type": STATIC_TYPE,
        "schema_version": STATIC_SCHEMA_VERSION,
        "outcome": {
            "$enum_type": "ConveyorOutcome",
            "value": outcome.value,
        },
        "regions": [_thaw_wire(item) for item in regions],
        "lattice": (
            None
            if lattice is None
            else {
                "$type": "GridSpecV1",
                "scale": lattice.scale,
                "magnitude_bound": lattice.magnitude_bound,
            }
        ),
        "arrival_laws": [
            {
                "$type": "PlainArrivalLawV1",
                "name": law.name,
                "normal_x": _fraction_to_wire(law.normal_x),
                "normal_y": _fraction_to_wire(law.normal_y),
                "constant": _fraction_to_wire(law.constant),
                "speed_squared": _fraction_to_wire(
                    law.speed_squared
                ),
            }
            for law in arrival_laws
        ],
        "law_names": list(law_names),
    }


def conveyor_static_to_wire(
    static_preparation: ConveyorStaticPreparationV1,
) -> dict[str, object]:
    if type(static_preparation) is not ConveyorStaticPreparationV1:
        _reject("STATIC_WIRE_ENCODE_DECODED_REQUIRED")
    return _static_fields_to_wire(
        static_preparation.outcome,
        static_preparation.regions,
        static_preparation.lattice,
        static_preparation.arrival_laws,
        static_preparation.law_names,
    )


def recomposition_static_to_wire(
    recomposed: ConveyorStaticRecompositionV1,
) -> dict[str, object]:
    if type(recomposed) is not ConveyorStaticRecompositionV1:
        _reject("STATIC_WIRE_ENCODE_RECOMPOSED_REQUIRED")
    return _static_fields_to_wire(
        recomposed.outcome,
        recomposed.regions,
        recomposed.lattice,
        recomposed.arrival_laws,
        recomposed.law_names,
    )


def _declared_alpha(alpha: object) -> Decimal:
    if type(alpha) not in {Decimal, int, str}:
        _reject("STATIC_ALPHA_TYPE_MISMATCH")
    try:
        value = alpha if type(alpha) is Decimal else Decimal(str(alpha))
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
