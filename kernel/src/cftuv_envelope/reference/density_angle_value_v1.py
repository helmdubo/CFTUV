"""Локальный A2 value gateway; owned literal admission остаётся отдельным.

Caller регистрирует луч только после доказанных exact principal facts и связи
его source angle с atom. Этот модуль не выводит admission из математической
оболочки и не обещает фиксированную ширину произвольной композиции.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .density_angle_v1 import AngleAtomV1, FractionIntervalV1, RayEnclosureV1
from .density_construct_v1 import (
    DensityConstructArenaV1, DensityConstructProfileUnsupported, OwnedExprV1,
)
from .density_trig_v1 import principal_pi_multiple


DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1 = "DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1"
VALUE_BITS = 160


class AngleValueErrorCode(str, Enum):
    FOREIGN_ARENA = "FOREIGN_ARENA"
    INVALID_REGISTRATION = "INVALID_REGISTRATION"
    CONFLICTING_REGISTRATION = "CONFLICTING_REGISTRATION"
    UNREGISTERED_OPAQUE = "UNREGISTERED_OPAQUE"
    UNSUPPORTED_POWER = "UNSUPPORTED_POWER"
    DENOMINATOR_CONTAINS_ZERO = "DENOMINATOR_CONTAINS_ZERO"
    NO_CANONICAL_RECIPE = "NO_CANONICAL_RECIPE"


class AngleValueError(ValueError):
    """Явная неспособность experimental value-path; не продуктовый UNDECIDABLE."""

    def __init__(self, code: AngleValueErrorCode):
        self.code = code
        super().__init__(code.value)


@dataclass(frozen=True, slots=True)
class PrincipalComponentV1:
    atom: AngleAtomV1
    ordinal: int
    sine: bool
    quarter_turns: int = 0


def canonical_pi_recipe(multiple: Fraction) -> tuple[AngleAtomV1, int] | None:
    """Детерминированный порядок (n,r,s,j), без float/поиска произвольных углов."""
    for n in range(2, 7):
        for ratio in (Fraction(0), Fraction(1, 4), Fraction(1, 2), Fraction(3, 4)):
            for sign in ((0,) if not ratio else (-1, 1)):
                angle = principal_pi_multiple(ratio, sign)
                for j in range(1, n):
                    if angle * Fraction(j, n) == multiple:
                        return AngleAtomV1(n, ratio, sign), j
    return None


def interval_divide(left: FractionIntervalV1, right: FractionIntervalV1) -> FractionIntervalV1:
    if right.lower <= 0 <= right.upper:
        raise AngleValueError(AngleValueErrorCode.DENOMINATOR_CONTAINS_ZERO)
    reciprocal = FractionIntervalV1(1 / right.upper, 1 / right.lower)
    return left.mul(reciprocal)


def interval_integer_power(value: FractionIntervalV1, exponent: int) -> FractionIntervalV1:
    if exponent < 0:
        value = interval_divide(FractionIntervalV1.point(1), value)
    remaining, result = abs(exponent), FractionIntervalV1.point(1)
    while remaining:
        if remaining & 1:
            result = result.mul(value)
        remaining >>= 1
        if remaining:
            value = value.square()
    return result


class AngleValueRegistryV1:
    """Одна owned arena; metadata/memo не переживают компиляцию автоматически."""

    def __init__(self, arena: DensityConstructArenaV1):
        self.arena = arena
        self._components: dict[int, PrincipalComponentV1] = {}
        self._rays: dict[tuple[AngleAtomV1, int], RayEnclosureV1] = {}
        self._memo: dict[int, FractionIntervalV1] = {}

    def _form(self, expression: OwnedExprV1):
        try:
            return self.arena.owned_form(expression)
        except DensityConstructProfileUnsupported as error:
            raise AngleValueError(AngleValueErrorCode.FOREIGN_ARENA) from error

    def _canonical_multiple(self, angle: OwnedExprV1) -> Fraction | None:
        # Только закрытый owned template Q*pi, не parser generic/source AST.
        form = self._form(angle)
        if form.kind == "pi":
            return Fraction(1)
        if form.kind == "Numeric" and form.payload.value.as_rational() == 0:
            return Fraction(0)
        if form.kind == "Mul" and len(form.children) == 2:
            coefficient, pi = map(self._form, form.children)
            if coefficient.kind == "Numeric" and pi.kind == "pi":
                return coefficient.payload.value.as_rational()
        return None

    def register_canonical_ray(self, cosine: OwnedExprV1, sine: OwnedExprV1,
                               multiple: Fraction) -> None:
        self._form(cosine)
        self._form(sine)
        recipe = canonical_pi_recipe(multiple)
        if recipe is None:
            raise AngleValueError(AngleValueErrorCode.NO_CANONICAL_RECIPE)
        self.register_ray(cosine, sine, *recipe)

    def register_ray(self, cosine: OwnedExprV1, sine: OwnedExprV1,
                     atom: AngleAtomV1, ordinal: int) -> None:
        self.register_ray_with_quarter_turns(cosine, sine, atom, ordinal, 0)

    def register_ray_with_quarter_turns(self, cosine: OwnedExprV1, sine: OwnedExprV1,
                                       atom: AngleAtomV1, ordinal: int, quarter_turns: int) -> None:
        """Только прямой source ray и наблюдённая literal-компонента phi-pi/2."""
        cosine_form, sine_form = self._form(cosine), self._form(sine)
        if (not isinstance(atom, AngleAtomV1) or type(ordinal) is not int
                or not 0 < ordinal < atom.subdivision or type(quarter_turns) is not int
                or quarter_turns not in (0, -1)):
            raise AngleValueError(AngleValueErrorCode.INVALID_REGISTRATION)
        forms = (cosine_form, sine_form)
        if any(form.kind not in ("Numeric", kind)
               for form, kind in zip(forms, ("cos", "sin"), strict=True)):
            raise AngleValueError(AngleValueErrorCode.INVALID_REGISTRATION)
        if all(form.kind != "Numeric" for form in forms):
            if cosine_form.children != sine_form.children:
                raise AngleValueError(AngleValueErrorCode.INVALID_REGISTRATION)
        opaque = next((form for form in forms if form.kind != "Numeric"), None)
        if opaque is not None:
            multiple = self._canonical_multiple(opaque.children[0])
            if multiple is not None:
                unshifted = multiple - Fraction(quarter_turns, 2)
                source_angle = principal_pi_multiple(atom.ratio, atom.dot_sign)
                if source_angle is None or source_angle * Fraction(ordinal, atom.subdivision) != unshifted:
                    raise AngleValueError(AngleValueErrorCode.INVALID_REGISTRATION)
                recipe = canonical_pi_recipe(unshifted)
                if recipe is None:
                    raise AngleValueError(AngleValueErrorCode.NO_CANONICAL_RECIPE)
                atom, ordinal = recipe
        additions = []
        for expression, form, is_sine in zip((cosine, sine), forms, (False, True), strict=True):
            if form.kind == "Numeric":
                continue
            component = PrincipalComponentV1(atom, ordinal, is_sine, quarter_turns)
            previous = self._components.get(expression.node)
            if previous is not None and previous != component:
                raise AngleValueError(AngleValueErrorCode.CONFLICTING_REGISTRATION)
            additions.append((expression.node, component))
        self._components.update(additions)
        self._memo.clear()

    def enclosure(self, expression: OwnedExprV1) -> FractionIntervalV1:
        form = self._form(expression)
        if expression.node in self._memo:
            return self._memo[expression.node]
        if form.kind == "Numeric":
            (lower, upper), _ = form.payload.value.enclosure(VALUE_BITS)
            value = FractionIntervalV1(lower, upper)
        elif expression.node in self._components:
            component = self._components[expression.node]
            key = (component.atom, component.ordinal)
            if key not in self._rays:
                self._rays[key] = component.atom.ray(component.ordinal)
            ray = self._rays[key]
            if component.quarter_turns == -1:
                value = FractionIntervalV1.point(0).sub(ray.cosine) if component.sine else ray.sine
            else:
                value = ray.sine if component.sine else ray.cosine
        elif form.kind in ("Add", "Mul"):
            value = FractionIntervalV1.point(0 if form.kind == "Add" else 1)
            for child in form.children:
                next_value = self.enclosure(child)
                value = value.add(next_value) if form.kind == "Add" else value.mul(next_value)
        elif form.kind == "IntegerPow":
            value = interval_integer_power(self.enclosure(form.children[0]), form.payload)
        elif form.kind == "OpaquePow":
            exponent = Fraction(form.payload)
            if exponent.denominator != 2 or exponent.numerator < 0:
                raise AngleValueError(AngleValueErrorCode.UNSUPPORTED_POWER)
            root = self.enclosure(form.children[0]).positive_sqrt(VALUE_BITS)
            value = interval_integer_power(root, exponent.numerator)
        else:
            raise AngleValueError(AngleValueErrorCode.UNREGISTERED_OPAQUE)
        self._memo[expression.node] = value
        return value
