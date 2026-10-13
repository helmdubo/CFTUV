"""Закрытый reader сырых facts offline-корпуса; не читает symbolic AST."""

from dataclasses import fields
from fractions import Fraction

from cftuv_envelope.reference import density_oracle_v1 as facts


def _record(cls, value, converters=None):
    if type(value) is not dict or set(value) != {field.name for field in fields(cls)}:
        raise ValueError("DENSITY_RAW_WIRE_RECORD_FIELDS:" + cls.__name__)
    converters = converters or {}
    return cls(**{key: converters[key](item) if key in converters else _primitive(item)
                  for key, item in value.items()})


def _primitive(value):
    if value is None or type(value) in (str, int, bool):
        return value
    if type(value) is list:
        return tuple(_primitive(item) for item in value)
    raise TypeError("DENSITY_RAW_WIRE_NON_PRIMITIVE")


def _q(value):
    if type(value) is not list or len(value) != 2 or any(type(item) is not int for item in value):
        raise TypeError("DENSITY_RAW_WIRE_RATIONAL_PAIR_REQUIRED")
    if value[1] <= 0:
        raise ValueError("DENSITY_RAW_WIRE_POSITIVE_DENOMINATOR_REQUIRED")
    return Fraction(*value)


def _many(convert, value):
    if type(value) is not list:
        raise TypeError("DENSITY_RAW_WIRE_ARRAY_REQUIRED")
    return tuple(convert(item) for item in value)


def _point(value):
    return _record(facts.RawDensityPointV1, value, {"x": _q, "y": _q})


def _metric(value):
    return _record(facts.RawDensityMetricV1, value, {
        "gram": lambda items: _many(_q, items), "inverse_gram": lambda items: _many(_q, items),
    })


def _incident(value):
    return _record(facts.RawDensityIncidentV1, value, {
        "start": _point, "end": _point,
        "owner_face_cycles": lambda items: _many(lambda cycle: _many(_point, cycle), items),
    })


def _geometry(value):
    return _record(facts.RawDensityGeometryV1, value, {
        "metric": _metric, "incoming": _incident, "outgoing": _incident,
    })


def _canonical(value):
    return _record(facts.RawDensityCanonicalFactsV1, value, {
        "source_reflex_interval": lambda items: _many(_q, items),
        "canonical_excess_over_pi": lambda item: None if item is None else _q(item),
    })


def read_raw_spec(value) -> facts.RawDensitySpecInput:
    return _record(facts.RawDensitySpecInput, value, {
        "source_geometry": _geometry, "evaluation_geometry": _geometry,
        "selection": lambda item: _record(facts.RawDensitySelectionV1, item), "canonical": _canonical,
    })
