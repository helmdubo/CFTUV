"""Строгая JSON-проволока Density: дерево SymPy без переупорядочивания и память входа."""

from __future__ import annotations

import json
from dataclasses import dataclass, is_dataclass
from decimal import Decimal
from enum import Enum
from fractions import Fraction

import sympy as sp
from mpmath import iv

from cftuv_envelope import codec
from cftuv_envelope.reference.density_shell_events import DensityFanBindingMemoEntry
from cftuv_envelope.reference.density_shell_v2 import (
    DensityShellOutcomeBundleV2, DensityShellOutcomeCode, DensityShellRationalMetadataV2, SHELL_SCALE,
)
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.planar_types import ExactPlanarVector, ExactScalar


class CorpusWireError(ValueError):
    """Вход вне объявленного формата корпуса; выполнение не подменяется успехом."""


# Только эти узлы разрешены; никакого eval/sympify строки или произвольного импорта.
EXPRESSION_KINDS = {
    "Add": sp.Add, "Mul": sp.Mul, "Pow": sp.Pow,
    "sin": sp.sin, "cos": sp.cos, "tan": sp.tan,
    "atan": sp.atan, "atan2": sp.atan2,
}
MEMO_FIELDS = (
    "expressions", "dual_dots", "crosses", "intervals", "signs", "sreprs",
    "subturns", "covector_rows", "fan_bindings", "native_gram",
)
ENUM_TYPES = {
    value.__name__: value
    for module in codec._PUBLIC_MODULES
    for value in vars(module).values()
    if isinstance(value, type) and issubclass(value, Enum)
}


@dataclass(frozen=True)
class ExpressionVector:
    x: sp.Expr
    y: sp.Expr

    def expressions(self, transaction_memo=None):
        return self.x, self.y


def json_key(value) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def require_keys(value, keys, label: str) -> None:
    if type(value) is not dict or set(value) != set(keys):
        raise CorpusWireError(f"{label}: expected exactly {sorted(keys)}")


def _shell_rational(body):
    if len(body) != 4 or any(type(item) is not int for item in body[1:3]) or body[2] <= 0:
        raise CorpusWireError("density_shell_rational_v2 requires reduced integers and a positive denominator")
    numerator, denominator, grid_units = body[1:]
    rational = Fraction(numerator, denominator)
    if (rational.numerator, rational.denominator) != (numerator, denominator):
        raise CorpusWireError("density_shell_rational_v2 must be reduced")
    expected = numerator * (SHELL_SCALE // denominator) if SHELL_SCALE % denominator == 0 else None
    if (grid_units is not None and type(grid_units) is not int) or grid_units != expected:
        raise CorpusWireError("density_shell_rational_v2 grid units disagree with the rational")
    return DensityShellRationalMetadataV2(numerator, denominator, grid_units)


def _shell_outcomes(body, wire):
    if len(body) != 4 or type(body[1]) is not list or any(type(code) is not str for code in body[1]):
        raise CorpusWireError("density_shell_outcomes_v2 requires code array, rational metadata and minimum width")
    try:
        codes = tuple(DensityShellOutcomeCode(code) for code in body[1])
    except ValueError as error:
        raise CorpusWireError("density_shell_outcomes_v2 unknown code") from error
    rational = wire.unvalue(body[2])
    if rational is not None and type(rational) is not DensityShellRationalMetadataV2:
        raise CorpusWireError("density_shell_outcomes_v2 rational metadata has a different type")
    expected, minimum = (DensityShellOutcomeCode.APPLIED,), None
    if rational is not None and rational.grid_units is not None:
        expected += (DensityShellOutcomeCode.GRID_POINT,)
        if rational.denominator & (rational.denominator - 1):
            expected += (DensityShellOutcomeCode.NON_DYADIC_GRID_STRAW,)
            minimum = 2
    if codes != expected or body[3] != minimum or (body[3] is not None and type(body[3]) is not int):
        raise CorpusWireError("density_shell_outcomes_v2 codes/minimum width disagree with metadata")
    return DensityShellOutcomeBundleV2(codes, rational, minimum)


def _fan_binding_entry(body, wire):
    if len(body) != 3:
        raise CorpusWireError("fan_binding_memo_v2 requires a result and outcome bundle")
    result, bundle = wire.unvalue(body[1]), wire.unvalue(body[2])
    if type(result) is not tuple or len(result) != 2 or type(result[0]) is not tuple:
        raise CorpusWireError("fan_binding_memo_v2 result must be the binding two-tuple")
    if type(bundle) is not tuple:
        raise CorpusWireError("fan_binding_memo_v2 outcomes must be an immutable tuple")
    seen = set()
    for pair in bundle:
        if type(pair) is not tuple or len(pair) != 2:
            raise CorpusWireError("fan_binding_memo_v2 outcomes must be value-wire/outcomes pairs")
        value_wire, outcomes = pair
        if (not _is_shell_value_wire(value_wire)
                or type(outcomes) is not DensityShellOutcomeBundleV2 or value_wire in seen):
            raise CorpusWireError("fan_binding_memo_v2 requires distinct integer value wires and typed outcomes")
        rational = outcomes.rational
        if rational is not None:
            expected = (1, rational.numerator, rational.denominator) if rational.numerator else ()
            if value_wire != expected:
                raise CorpusWireError("fan_binding_memo_v2 rational metadata disagrees with its value wire")
        elif not value_wire or value_wire[::3] == (1,):
            raise CorpusWireError("fan_binding_memo_v2 rational value wire requires rational metadata")
        seen.add(value_wire)
    return DensityFanBindingMemoEntry(result, bundle)


def _is_shell_value_wire(value):
    if type(value) is not tuple or len(value) % 3 or any(type(part) is not int for part in value):
        return False
    previous = 0
    for at in range(0, len(value), 3):
        radicand, numerator, denominator = value[at:at + 3]
        if radicand <= previous or not numerator or denominator <= 0:
            return False
        rational = Fraction(numerator, denominator)
        if (rational.numerator, rational.denominator) != (numerator, denominator):
            return False
        previous = radicand
    return True


def _legacy_fan_bindings(values):
    # Только reader старого корпуса: продуктовый memo всегда хранит один новый тип.
    result = {}
    for key, value in values.items():
        if type(value) is tuple and len(value) == 2 and type(value[0]) is tuple:
            value = DensityFanBindingMemoEntry(value, ())
        if type(value) is not DensityFanBindingMemoEntry:
            raise CorpusWireError("metric.memo.fan_bindings requires typed entries or legacy binding two-tuples")
        result[key] = value
    return result


class Wire:
    """Общие узлы DAG; порядок args и невычисленные Pow остаются порядком входа."""

    def __init__(self, nodes=None) -> None:
        self.nodes = [] if nodes is None else nodes
        self._index = {json_key(node): at for at, node in enumerate(self.nodes)}
        self._decoded: dict = {}

    def expression(self, value) -> int:
        if isinstance(value, sp.Rational):
            node = ["Q", int(value.p), int(value.q)]
        elif value is sp.pi:
            node = ["pi"]
        elif value.func.__name__ in EXPRESSION_KINDS and value.func is EXPRESSION_KINDS[value.func.__name__]:
            node = [value.func.__name__, *[self.expression(arg) for arg in value.args]]
        else:
            raise CorpusWireError(f"unsupported exact-expression node: {type(value).__name__}")
        key = json_key(node)
        if key not in self._index:
            self._index[key] = len(self.nodes)
            self.nodes.append(node)
        return self._index[key]

    def unexpression(self, index: int):
        if type(index) is not int or not 0 <= index < len(self.nodes):
            raise CorpusWireError(f"invalid expression reference: {index!r}")
        if index in self._decoded:
            return self._decoded[index]
        node = self.nodes[index]
        if type(node) is not list or not node:
            raise CorpusWireError("expression node must be a nonempty array")
        tag = node[0]
        if tag == "Q" and len(node) == 3:
            if any(type(item) is not int for item in node[1:]) or node[2] <= 0:
                raise CorpusWireError("Q requires integer numerator and positive denominator")
            result = sp.Rational(node[1], node[2])
            if ["Q", int(result.p), int(result.q)] != node:
                raise CorpusWireError("Q must be reduced")
        elif node == ["pi"]:
            result = sp.pi
        elif tag in EXPRESSION_KINDS:
            if any(type(at) is not int or not 0 <= at < index for at in node[1:]):
                raise CorpusWireError("expression children must refer to earlier nodes")
            arity = {"Pow": 2, "sin": 1, "cos": 1, "tan": 1, "atan": 1, "atan2": 2}.get(tag)
            if (arity is not None and len(node) != arity + 1) or (arity is None and len(node) < 3):
                raise CorpusWireError(f"invalid {tag} arity")
            result = EXPRESSION_KINDS[tag](*[self.unexpression(at) for at in node[1:]], evaluate=False)
            if result.func is not EXPRESSION_KINDS[tag]:
                raise CorpusWireError(f"{tag} did not preserve its node")
        else:
            raise CorpusWireError(f"unsupported expression tag: {tag!r}")
        self._decoded[index] = result
        return result

    def value(self, value):
        if value is None or type(value) in (int, str, bool):
            return value
        if isinstance(value, sp.Expr):
            return ["expr", self.expression(value)]
        if isinstance(value, Enum):
            return ["enum", type(value).__name__, value.value]
        if isinstance(value, Fraction):
            return ["fraction", value.numerator, value.denominator]
        if isinstance(value, Decimal):
            return ["decimal", str(value)]
        if type(value) is DensityFanBindingMemoEntry:
            return ["fan_binding_memo_v2", self.value(value.result), self.value(value.shell_outcomes)]
        if type(value) is DensityShellOutcomeBundleV2:
            return ["density_shell_outcomes_v2", [code.value for code in value.codes],
                    self.value(value.rational), value.legacy_projected_min_width_units]
        if type(value) is DensityShellRationalMetadataV2:
            return ["density_shell_rational_v2", value.numerator, value.denominator, value.grid_units]
        if isinstance(value, ExactPlanarVector):
            return ["vector_text", value.x.expression, value.y.expression]
        if hasattr(value, "expressions"):
            return ["vector_expr", *[self.expression(item) for item in value.expressions()]]
        if value is iv.pi:
            # iv.pi — объект с динамическим _mpi_: границы читаются при текущей iv.prec.
            # Заморозить их при записи (обычно 53), значит изменить последующее вычисление при 160.
            return ["interval_constant", "pi"]
        if type(value) is type(iv.pi):
            raise CorpusWireError("unsupported dynamic interval constant; only iv.pi is declared")
        if hasattr(value, "_mpi_"):
            return ["interval", [list(endpoint) for endpoint in value._mpi_]]
        if type(value) is dict:
            return ["map", [[self.value(key), self.value(item)] for key, item in value.items()]]
        if isinstance(value, tuple):
            return ["tuple", [self.value(item) for item in value]]
        if type(value) is list:
            return ["list", [self.value(item) for item in value]]
        if is_dataclass(value) and type(value).__name__ in codec._TYPE_REGISTRY:
            return ["contract", type(value).__name__, codec.to_canonical_data(value)]
        raise CorpusWireError(f"unsupported memo value: {type(value).__name__}")

    def unvalue(self, value):
        if value is None or type(value) in (int, str, bool):
            return value
        if type(value) is not list or not value or type(value[0]) is not str:
            raise CorpusWireError("typed value must be a tagged array")
        tag = value[0]
        if tag == "fan_binding_memo_v2":
            return _fan_binding_entry(value, self)
        if tag == "density_shell_outcomes_v2":
            return _shell_outcomes(value, self)
        if tag == "density_shell_rational_v2":
            return _shell_rational(value)
        if tag == "expr" and len(value) == 2:
            return self.unexpression(value[1])
        if tag == "enum" and len(value) == 3 and value[1] in ENUM_TYPES:
            return ENUM_TYPES[value[1]](value[2])
        if tag == "fraction" and len(value) == 3:
            if any(type(item) is not int for item in value[1:]) or value[2] <= 0:
                raise CorpusWireError("fraction requires integer numerator and positive denominator")
            return Fraction(value[1], value[2])
        if tag == "decimal" and len(value) == 2 and type(value[1]) is str:
            result = Decimal(value[1])
            if not result.is_finite():
                raise CorpusWireError("non-finite decimal")
            return result
        if tag == "vector_text" and len(value) == 3 and all(type(item) is str for item in value[1:]):
            return ExactPlanarVector(ExactScalar(value[1]), ExactScalar(value[2]))
        if tag == "vector_expr" and len(value) == 3:
            return ExpressionVector(self.unexpression(value[1]), self.unexpression(value[2]))
        if value == ["interval_constant", "pi"]:
            return iv.pi
        if tag == "interval" and len(value) == 2:
            endpoints = value[1]
            if type(endpoints) is not list or len(endpoints) != 2 or any(
                type(endpoint) is not list or len(endpoint) != 4 or any(type(part) is not int for part in endpoint)
                for endpoint in endpoints
            ):
                raise CorpusWireError("interval requires two exact mpf endpoint tuples")
            return iv.make_mpf(tuple(tuple(endpoint) for endpoint in endpoints))
        if tag in ("tuple", "list", "map") and len(value) == 2 and type(value[1]) is list:
            if tag == "map":
                if any(type(pair) is not list or len(pair) != 2 for pair in value[1]):
                    raise CorpusWireError("map requires key/value pairs")
                pairs = [(self.unvalue(key), self.unvalue(item)) for key, item in value[1]]
                result = dict(pairs)
                if len(result) != len(pairs):
                    raise CorpusWireError("duplicate map key")
                return result
            items = [self.unvalue(item) for item in value[1]]
            return tuple(items) if tag == "tuple" else items
        if tag == "contract" and len(value) == 3 and value[1] in codec._TYPE_REGISTRY:
            return codec._decode_as(value[2], codec._TYPE_REGISTRY[value[1]])
        raise CorpusWireError(f"unsupported typed value: {tag!r}")

    def metric(self, metric) -> dict:
        return {
            "gram": [[self.expression(item) for item in row] for row in metric.gram],
            "inverse": [[self.expression(item) for item in row] for row in metric.inverse_gram],
            "sign": metric.owner_orientation_sign,
            "memo": {field: self.value(getattr(metric._density_exact_memo, field)) for field in MEMO_FIELDS},
        }

    def unmetric(self, body):
        require_keys(body, ("gram", "inverse", "sign", "memo"), "metric")
        require_keys(body["memo"], MEMO_FIELDS, "metric.memo")
        if type(body["sign"]) is not int or body["sign"] not in (-1, 1):
            raise CorpusWireError("metric.sign must be -1 or +1")
        for name in ("gram", "inverse"):
            if type(body[name]) is not list or len(body[name]) != 2 or any(type(row) is not list or len(row) != 2 for row in body[name]):
                raise CorpusWireError(f"metric.{name} must be 2 by 2")
        metric = ExactPlanarMetric(
            tuple(tuple(self.unexpression(item) for item in row) for row in body["gram"]),
            tuple(tuple(self.unexpression(item) for item in row) for row in body["inverse"]), body["sign"],
        )
        for field, value in body["memo"].items():
            decoded = self.unvalue(value)
            if type(decoded) is not dict:
                raise CorpusWireError(f"metric.memo.{field} must be a map")
            if field == "fan_bindings":
                decoded = _legacy_fan_bindings(decoded)
            setattr(metric._density_exact_memo, field, decoded)
        return metric
