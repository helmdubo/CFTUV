"""Конечные тригонометрические islands именованного owned source canon.

Таблица даёт Numeric компонентам в Q(sqrt(2),sqrt(3),sqrt(5)); конечный old
special whitelist сохраняет explicit positive half-roots. Остальные углы
остаются raw opaque, включая pi/16. Здесь нет sign/admission controller.
"""

from __future__ import annotations

from enum import Enum
from fractions import Fraction

from .density_construct_v1 import DensityConstructArenaV1, OwnedExprV1
from .density_scalar_v2 import ScalarV2


class TrigV1ErrorCode(str, Enum):
    RATIO_OUTSIDE_CLOSED_UNIT_INTERVAL = "RATIO_OUTSIDE_CLOSED_UNIT_INTERVAL"
    INCONSISTENT_DOT_SIGN = "INCONSISTENT_DOT_SIGN"
    NON_RATIONAL_INPUT = "NON_RATIONAL_INPUT"
    INVALID_SUBDIVISION = "INVALID_SUBDIVISION"
    INVALID_ORDINAL = "INVALID_ORDINAL"
    UNSUPPORTED_DENSITY_Q = "UNSUPPORTED_DENSITY_Q"


class TrigV1Error(ValueError):
    """Внутренняя ошибка API; не именованный отказ геометрии продукта."""

    def __init__(self, code: TrigV1ErrorCode):
        self.code = code
        super().__init__(code.value)


def _fraction(value: Fraction | int) -> Fraction:
    if type(value) not in (Fraction, int):
        raise TrigV1Error(TrigV1ErrorCode.NON_RATIONAL_INPUT)
    return Fraction(value)


def principal_pi_multiple(ratio: Fraction | int, dot_sign: int) -> Fraction | None:
    ratio = _fraction(ratio)
    if not 0 <= ratio <= 1:
        raise TrigV1Error(TrigV1ErrorCode.RATIO_OUTSIDE_CLOSED_UNIT_INTERVAL)
    if type(dot_sign) is not int or not (
            (not ratio and dot_sign == 0) or (ratio and dot_sign in (-1, 1))):
        raise TrigV1Error(TrigV1ErrorCode.INCONSISTENT_DOT_SIGN)
    if not ratio:
        return Fraction(1, 2)
    # cos(theta)^2: только пять объявленных значений, без распознавания иных AST.
    positive = {Fraction(1, 4): Fraction(1, 3), Fraction(1, 2): Fraction(1, 4),
                Fraction(3, 4): Fraction(1, 6), Fraction(1): Fraction(0)}.get(ratio)
    if positive is None:
        return None
    return positive if dot_sign > 0 else 1 - positive


# (радиканд, числитель коэффициента, знаменатель). None означает raw opaque.
_QUADRANT_TABLE = (
    (0, 1, ((1, 1, 1),), ()),
    (1, 2, (), ((1, 1, 1),)),
    (1, 3, ((1, 1, 2),), ((3, 1, 2),)),
    (1, 4, ((2, 1, 2),), ((2, 1, 2),)),
    (1, 6, ((3, 1, 2),), ((1, 1, 2),)),
    (1, 12, ((6, 1, 4), (2, 1, 4)), ((6, 1, 4), (2, -1, 4))),
    (5, 12, ((6, 1, 4), (2, -1, 4)), ((6, 1, 4), (2, 1, 4))),
    (1, 5, ((1, 1, 4), (5, 1, 4)), None),
    (2, 5, ((1, -1, 4), (5, 1, 4)), None),
    (1, 10, None, ((1, -1, 4), (5, 1, 4))),
    (3, 10, None, ((1, 1, 4), (5, 1, 4))),
)


def _recipe_scalar(recipe, sign: int) -> ScalarV2:
    scalar = ScalarV2.rational(0)
    for radicand, numerator, denominator in recipe:
        root = ScalarV2.rational(1) if radicand == 1 else ScalarV2.sqrt_rational(radicand)
        scalar = scalar.add(root.scaled(Fraction(sign * numerator, denominator)))
    return scalar


def _numeric_components(multiple: Fraction) -> tuple[ScalarV2 | None, ScalarV2 | None]:
    phase, cosine_sign, sine_sign = _quadrant(multiple)
    for numerator, denominator, cosine, sine in _QUADRANT_TABLE:
        if phase == Fraction(numerator, denominator):
            return (None if cosine is None else _recipe_scalar(cosine, cosine_sign),
                    None if sine is None else _recipe_scalar(sine, sine_sign))
    return None, None


def _quadrant(multiple: Fraction) -> tuple[Fraction, int, int]:
    phase, cosine_sign, sine_sign = multiple % 2, 1, 1
    if phase > 1:
        phase, sine_sign = 2 - phase, -1
    if phase > Fraction(1, 2):
        phase, cosine_sign = 1 - phase, -1
    return phase, cosine_sign, sine_sign


# Только actual old-special ключи: не общий half-angle алгоритм.
_COMPOSED_RECIPES = (
    ((1, 30), (1, 5), (1, 6), -1), ((1, 24), (1, 6), (1, 8), -1),
    ((1, 20), (1, 4), (1, 5), -1), ((1, 15), (1, 6), (1, 10), -1),
    ((2, 15), (1, 3), (1, 5), -1), ((3, 20), (1, 4), (1, 10), -1),
    ((5, 24), (1, 3), (1, 8), -1), ((4, 15), (1, 6), (1, 10), 1),
    ((7, 15), (1, 6), (3, 10), 1), ((9, 20), (1, 4), (1, 5), 1),
)

# Конечные дополнения уже явных ключей; не рекурсивное замыкание таблицы.
_COMPLEMENT_RECIPES = (
    ((7, 30), (4, 15)), ((7, 24), (5, 24)), ((7, 20), (3, 20)),
    ((11, 30), (2, 15)), ((13, 30), (1, 15)), ((11, 24), (1, 24)),
)


def _fifth_root(arena: DensityConstructArenaV1, positive: bool) -> OwnedExprV1:
    root_five = arena.numeric(ScalarV2.sqrt_rational(5))
    radicand = arena.add(arena.rational(10), arena.scaled(root_five, 2 if positive else -2))
    return arena.scaled(arena.sqrt(radicand), Fraction(1, 4))


def _eighth_pair(arena: DensityConstructArenaV1) -> tuple[OwnedExprV1, OwnedExprV1]:
    root_two = arena.numeric(ScalarV2.sqrt_rational(2))
    cosine = arena.scaled(arena.sqrt(arena.add(arena.rational(2), root_two)), Fraction(1, 2))
    sine = arena.scaled(arena.sqrt(arena.sub(arena.rational(2), root_two)), Fraction(1, 2))
    return cosine, sine


def _combine_pair(arena: DensityConstructArenaV1, left, right, sign: int):
    ca, sa = left
    cb, sb = right
    cosine = arena.sub(arena.mul(ca, cb), arena.scaled(arena.mul(sa, sb), sign))
    sine = arena.add(arena.mul(sa, cb), arena.scaled(arena.mul(ca, sb), sign))
    return cosine, sine


def _finite_quadrant_pair(arena: DensityConstructArenaV1, phase: Fraction):
    cosine, sine = _numeric_components(phase)
    if cosine is not None and sine is not None:
        return arena.numeric(cosine), arena.numeric(sine)
    if phase in (Fraction(1, 8), Fraction(3, 8)):
        pair = _eighth_pair(arena)
        return pair if phase == Fraction(1, 8) else tuple(reversed(pair))
    if phase in (Fraction(1, 5), Fraction(2, 5)):
        return arena.numeric(cosine), _fifth_root(arena, phase == Fraction(2, 5))
    if phase in (Fraction(1, 10), Fraction(3, 10)):
        return _fifth_root(arena, phase == Fraction(1, 10)), arena.numeric(sine)
    for target, original in _COMPLEMENT_RECIPES:
        if phase == Fraction(*target):
            return tuple(reversed(_finite_quadrant_pair(arena, Fraction(*original))))
    for target, left, right, sign in _COMPOSED_RECIPES:
        if phase == Fraction(*target):
            return _combine_pair(arena, _finite_quadrant_pair(arena, Fraction(*left)),
                                 _finite_quadrant_pair(arena, Fraction(*right)), sign)
    return None


def owned_rational_pi_trig(arena: DensityConstructArenaV1, multiple: Fraction | int
                           ) -> tuple[OwnedExprV1, OwnedExprV1]:
    multiple = _fraction(multiple)
    cosine, sine = _numeric_components(multiple)
    phase, cosine_sign, sine_sign = _quadrant(multiple)
    if cosine is None or sine is None:
        finite = _finite_quadrant_pair(arena, phase)
        if finite is not None:
            return arena.scaled(finite[0], cosine_sign), arena.scaled(finite[1], sine_sign)
    # Отсутствующая компонента сохраняет ORIGINAL multiple*pi, без opaque identities.
    angle = None if cosine is not None and sine is not None else arena.scaled(arena.pi(), multiple)
    return (arena.numeric(cosine) if cosine is not None else arena.raw_cos(angle),
            arena.numeric(sine) if sine is not None else arena.raw_sin(angle))


def owned_principal_ray(arena: DensityConstructArenaV1, ratio: Fraction | int, dot_sign: int,
                        subdivision: int, ordinal: int
                        ) -> tuple[OwnedExprV1, OwnedExprV1] | None:
    if type(subdivision) is not int or not 2 <= subdivision <= 6:
        raise TrigV1Error(TrigV1ErrorCode.INVALID_SUBDIVISION)
    if type(ordinal) is not int or not 0 <= ordinal <= subdivision:
        raise TrigV1Error(TrigV1ErrorCode.INVALID_ORDINAL)
    multiple = principal_pi_multiple(ratio, dot_sign)
    if multiple is None:
        return None
    return owned_rational_pi_trig(arena, multiple * Fraction(ordinal, subdivision))


def owned_exact_q_trig(arena: DensityConstructArenaV1, q: int
                       ) -> tuple[OwnedExprV1, OwnedExprV1]:
    if type(q) is not int or not 2 <= q <= 6:
        raise TrigV1Error(TrigV1ErrorCode.UNSUPPORTED_DENSITY_Q)
    return owned_rational_pi_trig(arena, Fraction(1, q))
