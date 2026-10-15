"""DENSITY_SOURCE_CONSTRUCTION_V1: numeric scalar-island CANDIDATE only.

All fresh numeric arithmetic is RadicalSumV2 arithmetic. The separate literal
decision view has a fixed construction profile, not the ambient SymPy shape.
No opaque Add/Mul ring, trigonometric construction, sign controller or product
dispatch is implemented here. V2 sign/enclosure never supplies sign admission.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .radical_v2 import RadicalInverseClassLimit, RadicalSumV2

DENSITY_SOURCE_CONSTRUCTION_V1 = "DENSITY_SOURCE_CONSTRUCTION_V1"
LiteralNode = tuple[str | int, ...]


class DensityScalarProfileCode(str, Enum):
    ROOT_REQUIRES_NONNEGATIVE_RATIONAL = "DENSITY_SCALAR_V2_ROOT_REQUIRES_NONNEGATIVE_RATIONAL"
    INVERSE_CLASS_LIMIT = "DENSITY_SCALAR_V2_INVERSE_CLASS_LIMIT"


class DensityScalarProfileUnsupported(ValueError):
    """Internal candidate profile boundary; never a product geometry refusal."""

    def __init__(self, code: DensityScalarProfileCode, detail: str) -> None:
        super().__init__(f"{code.value}: {detail}")
        self.code = code
        self.detail = detail


@dataclass(frozen=True, slots=True)
class ScalarLiteralAstV1:
    """Native-density-wire-compatible, child-before-parent rows; immutable."""

    canon: str
    nodes: tuple[LiteralNode, ...]
    root: int


def _rational(value: Fraction | int) -> Fraction:
    if type(value) is int or isinstance(value, Fraction):
        return Fraction(value)
    raise TypeError("ScalarV2 requires Python int/Fraction or ScalarV2, not a symbolic AST")


def _scalar(value: ScalarV2 | Fraction | int) -> ScalarV2:
    return value if isinstance(value, ScalarV2) else ScalarV2.rational(value)


class _LiteralBuilder:
    """Interning belongs to one encoding, with first-occurrence insertion order."""

    def __init__(self) -> None:
        self.nodes: list[LiteralNode] = []
        self.index: dict[LiteralNode, int] = {}

    def node(self, *items: str | int) -> int:
        row = tuple(items)
        if row not in self.index:
            self.index[row] = len(self.nodes)
            self.nodes.append(row)
        return self.index[row]

    def term(self, term: tuple[int, int, int]) -> int:
        radicand, sign, denominator = term
        if radicand == 1:
            return self.node("Q", sign, denominator)
        coefficient = None
        if (sign, denominator) != (1, 1):
            coefficient = self.node("Q", sign, denominator)
        base = self.node("Q", radicand, 1)
        half = self.node("Q", 1, 2)
        root = self.node("Pow", base, half)
        return root if coefficient is None else self.node("Mul", coefficient, root)


def scalar_literal_ast(value: RadicalSumV2) -> ScalarLiteralAstV1:
    """Fixed raw literal profile of a canonical value, without symbolic evaluation.

    Empty value -> Q(0,1); rational -> Q(n,d). Irrational (m,s,Q) ->
    Mul(Q(s,Q), Pow(Q(m,1), Q(1,2))), omitting coefficient exactly when s=Q=1.
    Coefficient row is emitted before root rows. Add children follow increasing
    canonical m' order (rational first); one term has no unary Add. Canonical
    zero terms are already dropped by V2. Nodes are interned on first encounter,
    children before parent; a repeated half/coefficient is shared. Identity,
    zero and ordering conventions therefore depend on the whole value only.
    """

    builder = _LiteralBuilder()
    children = tuple(builder.term(term) for term in value.terms)
    if not children:
        root = builder.node("Q", 0, 1)
    elif len(children) == 1:
        root = children[0]
    else:
        root = builder.node("Add", *children)
    return ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1, tuple(builder.nodes), root)


@dataclass(frozen=True, slots=True)
class ScalarV2:
    """An owned numeric island; operations return fresh immutable V2 values.

    Operation Work is not a Density budget and is not accumulated here. The
    RadicalSumV2 primitives remain the independently priced numeric oracle.
    """

    value: RadicalSumV2

    def __post_init__(self) -> None:
        if not isinstance(self.value, RadicalSumV2):
            raise TypeError("ScalarV2 requires an actual RadicalSumV2 value")

    @staticmethod
    def rational(value: Fraction | int) -> ScalarV2:
        return ScalarV2(RadicalSumV2.rational(_rational(value)))

    @staticmethod
    def sqrt_rational(value: Fraction | int) -> ScalarV2:
        return ScalarV2.rational(value).sqrt()

    def literal_ast(self) -> ScalarLiteralAstV1:
        return scalar_literal_ast(self.value)

    def add(self, other: ScalarV2 | Fraction | int) -> ScalarV2:
        return ScalarV2(self.value.add(_scalar(other).value)[0])

    def sub(self, other: ScalarV2 | Fraction | int) -> ScalarV2:
        return ScalarV2(self.value.sub(_scalar(other).value)[0])

    def mul(self, other: ScalarV2 | Fraction | int) -> ScalarV2:
        return ScalarV2(self.value.mul(_scalar(other).value)[0])

    def neg(self) -> ScalarV2:
        return ScalarV2(self.value.neg())

    def scaled(self, factor: Fraction | int) -> ScalarV2:
        return ScalarV2(self.value.scaled(_rational(factor)))

    def inverse(self) -> ScalarV2:
        try:
            return ScalarV2(self.value.inverse()[0])
        except RadicalInverseClassLimit as error:
            raise DensityScalarProfileUnsupported(
                DensityScalarProfileCode.INVERSE_CLASS_LIMIT, error.detail,
            ) from error

    def div(self, other: ScalarV2 | Fraction | int) -> ScalarV2:
        return self.mul(_scalar(other).inverse())

    def integer_power(self, exponent: int) -> ScalarV2:
        if type(exponent) is not int:
            raise TypeError("ScalarV2 power exponent must be a Python integer")
        # Отрицательная степень сначала инвертирует базу; предел не обходится.
        base = self.inverse() if exponent < 0 else self
        exponent = abs(exponent)
        result = ScalarV2.rational(1)
        while exponent:
            if exponent & 1:
                result = result.mul(base)
            exponent >>= 1
            if exponent:
                base = base.mul(base)
        return result

    def sqrt(self) -> ScalarV2:
        rational = self.value.as_rational()
        if rational is None or rational < 0:
            raise DensityScalarProfileUnsupported(
                DensityScalarProfileCode.ROOT_REQUIRES_NONNEGATIVE_RATIONAL,
                "sqrt requires the whole scalar value to be a nonnegative rational; opaque roots remain outside this layer",
            )
        return ScalarV2(RadicalSumV2.sqrt_of_rational(rational)[0])

    __add__ = add
    __sub__ = sub
    __mul__ = mul
    __neg__ = neg
    __truediv__ = div
    __pow__ = integer_power
