"""`RadicalSumV2`: the value-canonical sum of square roots of rationals, the ORACLE that the Rust `RadicalSum` mirrors bit for bit.

WHAT. A value is `sum_k t_k`, one term `t_k` per SQUARE CLASS (two roots share a class when their quotient is rational), and
every term is stored by its VALUE alone (amendment A0 of the RadicalSumV2 design): a term `t = c * sqrt(r)` with `t^2 = P/Q` in
lowest terms is the integer triple `(m', s, Q)` with `m' = P*Q` and `s = sign(t)`, so that `t = s * sqrt(m') / Q`. The rational
class is the triple `(1, n, d)` (`t = n/d`, lowest terms, `d > 0`). The triples are sorted by `m'`; two classes never share an
`m'` (`m'^2` is a square), so the order is total and the rational term, if any, comes first. Nothing here depends on how the
value was built: the representative of a class, the term order, the integer form, every enclosure, the text and the wire are
functions of the value only. `RadicalSumV1` (`native_exact.py`, pinned for the Rust stage `effective_alpha`) keeps the FIRST
radicand that entered a class, so its terms depend on the order of operations; nothing in this module touches V1 or its
consumers (slice S2: V2 lives beside V1; consumers switch in R2, with the Rust mirror and a new pin, in one release).

CLASSES WITHOUT FACTORISATION. Two irrational terms share a class iff `m'_1 * m'_2` is a perfect square: one `isqrt`
(`CLASS_MERGE_CHECKS`). A merge goes into the smaller `m'` (`sqrt(m_big) = isqrt(m_small*m_big)/m_small * sqrt(m_small)`), and
the merged class is normalised back to its value-canonical triple; a class that cancels disappears.

ZERO (Besicovitch 1940). Every `m'` is `j^2 * s` with `s` squarefree, and two canonical classes have different `s` (else
`m'_1 m'_2` would be a square). The square roots of distinct squarefree positive integers are linearly independent over Q
(A. S. Besicovitch, "On the linear independence of fractional powers of integers", J. London Math. Soc. 15 (1940) 3-6), so
`sum c_k sqrt(m'_k) = sum (c_k j_k) sqrt(s_k)` is zero iff every coefficient is zero: a value is zero iff it has no terms, and
two values are equal iff their term tuples are equal (`==` and `hash` are the dataclass's). No enclosure, no conjugate norm.

SIGN. No terms: 0. One term: its sign. Two terms of opposite signs: compare the squares `t^2` in Q (they are never equal:
different classes). Three or more: the integer enclosure (as in V1) on the rungs 64, then min(128, bound), min(256, bound), ...
bits, where the bound is a COMPUTED separation bound (`separation_bits`, computed only when the 64-bit rung fails) instead of
V1's fixed 4096: at the bound the sign is certain, so `RadicalSignUndecided` is only the outcome of an explicit price cap
(`precision_cap_bits`), never of the mathematics. The bound proves termination, not affordability: it is exponential in the
number of classes (a 28-class sign of the kernel suite has a bound of 8.6e10 bits, decided at the first rung like every
recorded sign), so a cap for many classes is an owner's price decision, not a property of this type. The value-canonical term
also tightens every enclosure: `s sqrt(m')/Q` is off by `1/Q` units of the last bit where V1's `c sqrt(r)` is off by `|c|`.

    The bound. Clear denominators: `y = L x = sum_{i=1..k} a_i sqrt(m_i)`, integers `a_i != 0`, distinct classes, `k >= 2`.
    Let `F(z) = prod_{e in {+1,-1}^(k-1)} (a_1 z_1 + sum_{i>=2} e_i a_i z_i)` over indeterminates `z_i`. Flipping `z_i` (i >= 2)
    permutes the factors; flipping `z_1` multiplies each of the `2^(k-1)` (an even number of) factors by -1 after re-indexing.
    So `F` is invariant under every sign flip, hence `F` is in `Z[z_1^2, ..., z_k^2]` and `N = F(sqrt m)` is an integer. Each
    factor at `z = sqrt m` is a sum over the same distinct classes with nonzero coefficients `+-a_i`: nonzero by Besicovitch.
    So `|N| >= 1` and, `y` being the factor with all signs `+`, `|y| >= 1 / U^e` with `U = sum |a_i| (isqrt(m_i) + 1) >=
    sum |a_i| sqrt(m_i)` (a bound on every factor) and `e = 2^(k-1) - 1`; thus `|x| >= 1 / (L U^e)`. The enclosure of `2^b y`
    has width `W = sum_{irrational} |a_i|`, so once `2^b > W U^e` the enclosure cannot hold zero. `separation_bits` takes
    `b = bitlen(W) + e * bitlen(U)` (then `W < 2^bitlen(W)`, `U^e < 2^(e bitlen(U))`). This is the BFMSS form `u^-(D-1)`
    (Burnikel, Fleischer, Mehlhorn, Schirra, ESA 2001; Li and Yap, SODA 2001) with the half-conjugate product of degree
    `D = 2^(k-1)` instead of the full `2^k`, proved here directly rather than quoted.

INVERSE (amendment A3). One term: `1/(s sqrt(m')/Q) = s sqrt(m')/(m'/Q)` (same `m'`). Two or three classes: conjugate the
term of the LARGEST `m'` (the last in `m'` order), `x^-1 = conj(x) * (rest^2 - pivot^2)^-1`, recursively; the norm has at most
two classes after the first step and is nonzero (both factors are nonzero sums over distinct classes). Every product of sums
made by the conjugation counts one `CONJUGATION_PRODUCTS`. More than three classes: `RadicalInverseClassLimit`
(`EXACT_NATIVE_INVERSE_CLASS_LIMIT`, a price limit like V1's, now with its own name).

TEXT (amendment A5). `Integer(n)` / `Rational(p, q)` for a rational value, `[-]Sqrt(Rational(P, Q))` for one irrational term
(identical to `native_exact.canonical_text`, EXACT_SCALAR_TEXT_CANON_V2), `Sum(<term>, <term>, ...)` in `m'` order for more.

PRICE. `NativeAlgebraWorkV1`: `CLASS_MERGE_CHECKS` (perfect-square decisions on a product of two radicands, including the
rationality of a root or of a product of two roots), `CONJUGATION_PRODUCTS`, `ENCLOSURE_ROUNDS` (integer enclosures
evaluated). No cap. Every operation that can do work RETURNS `(answer, work)`: nothing here writes a process counter, and a
refusal carries the work done before it (`.work`). The counting rules ARE the contract a mirror reproduces:
* `add`/`sub`: a term of the right operand whose `m'` equals a left one merges free; otherwise it asks the not yet merged
  irrational terms of the left operand in `m'` order, one check each, until one is its class; rational terms are free;
* `mul`: a rational operand only scales (free); else, products left-outer/right-inner: two equal radicands give a rational
  free, two different irrational ones cost one check (is `(a/g)(b/g)` a square); an irrational product then joins the class
  slots: an equal radicand free, else one check per slot in creation order until its class; slots merge into the smaller radicand;
* `from_terms` (and `from_v1`, `value_of_text`): one check per radicand other than 0 and 1 (is it a square), then as `mul`'s slots;
  `sqrt_of_rational(p/q)`: one check unless `p q = 1`;
* `inverse`: the work of its products and sums plus two `CONJUGATION_PRODUCTS` per conjugation; `div`: `inverse` + `mul`;
* `signum`: one `ENCLOSURE_ROUNDS` per rung evaluated; `enclosure(bits)`: one; `neg`, `scaled`, readers, text, wire: free.

No sympy, no factorisation, no float, no hash-ordered iteration: the answer, the work and every text are the same under any
`PYTHONHASHSEED`. The only import of `native_exact` (which imports sympy) is inside `to_v1`.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
from math import gcd, isqrt, lcm
import re

RADICAL_SUM_V2 = "RADICAL_SUM_V2"
EXACT_NATIVE_OUTSIDE_FIELD = "EXACT_NATIVE_OUTSIDE_FIELD"
EXACT_NATIVE_SIGN_UNDECIDED = "EXACT_NATIVE_SIGN_UNDECIDED"
EXACT_NATIVE_INVERSE_CLASS_LIMIT = "EXACT_NATIVE_INVERSE_CLASS_LIMIT"

#: More classes than this are not inverted by conjugation (a cost limit, not mathematics).
INVERSE_CLASS_LIMIT = 3
#: The first rung of the sign ladder; each next rung doubles, the last one is the separation bound.
SIGN_FIRST_BITS = 64
#: The articles of `NativeAlgebraWorkV1`, in the order of its wire.
ARTICLES = ("CLASS_MERGE_CHECKS", "CONJUGATION_PRODUCTS", "ENCLOSURE_ROUNDS")


@dataclass(frozen=True, slots=True)
class NativeAlgebraWorkV1:
    """The price of one exact operation, by article; operations RETURN it (a counts delta), the caller adds it where it wants."""

    class_merge_checks: int = 0
    conjugation_products: int = 0
    enclosure_rounds: int = 0

    def __add__(self, other: "NativeAlgebraWorkV1") -> "NativeAlgebraWorkV1":
        if not isinstance(other, NativeAlgebraWorkV1):
            return NotImplemented
        if other is NO_WORK:
            return self
        if self is NO_WORK:
            return other
        return NativeAlgebraWorkV1(
            self.class_merge_checks + other.class_merge_checks,
            self.conjugation_products + other.conjugation_products,
            self.enclosure_rounds + other.enclosure_rounds,
        )

    def articles(self) -> dict[str, int]:
        return dict(zip(ARTICLES, self.wire()))

    def wire(self) -> tuple[int, int, int]:
        return (self.class_merge_checks, self.conjugation_products, self.enclosure_rounds)


NO_WORK = NativeAlgebraWorkV1()


_SMALL_CHECKS = tuple(NativeAlgebraWorkV1(count) for count in range(1, 33))


def _checks(count: int) -> NativeAlgebraWorkV1:
    if not count:
        return NO_WORK
    return _SMALL_CHECKS[count - 1] if count <= 32 else NativeAlgebraWorkV1(count)


class RadicalRefusal(ValueError):
    """A named refusal of the V2 algebra; the code is `.code`, the work done before it is `.work`."""

    code = ""

    def __init__(self, detail: str = "", work: NativeAlgebraWorkV1 = NO_WORK) -> None:
        super().__init__(f"{self.code}: {detail}" if detail else self.code)
        self.detail = detail
        self.work = work


class RadicalOutsideField(RadicalRefusal):
    """The value is not a sum of square roots of rationals (a root of a negative number)."""

    code = EXACT_NATIVE_OUTSIDE_FIELD


class RadicalSignUndecided(RadicalRefusal):
    """The caller's precision cap is below the separation bound: a PRICE outcome, the sign itself is always decidable."""

    code = EXACT_NATIVE_SIGN_UNDECIDED


class RadicalInverseClassLimit(RadicalRefusal):
    """More than `INVERSE_CLASS_LIMIT` classes to invert by conjugation."""

    code = EXACT_NATIVE_INVERSE_CLASS_LIMIT


class RadicalSeparationBoundViolated(RuntimeError):
    """The enclosure at the separation bound still holds zero: the proof or the code is broken. Never an answer."""


# --------------------------------------------------------------------------
# terms: (m', s, Q) for an irrational class, (1, n, d) for the rational one
# --------------------------------------------------------------------------

Term = tuple[int, int, int]


def _reduced(numerator: int, denominator: int) -> tuple[int, int]:
    """`numerator/denominator` in lowest terms with a positive denominator (`denominator != 0`)."""

    if denominator < 0:
        numerator, denominator = -numerator, -denominator
    common = gcd(numerator, denominator)
    if common != 1:
        return numerator // common, denominator // common
    return numerator, denominator


def _irrational_term(radicand: int, numerator: int, denominator: int) -> Term | None:
    """The value-canonical triple of `numerator/denominator * sqrt(radicand)` (`radicand` not a square, `denominator > 0`); `None` for zero.

    With `numerator/denominator = n/d` in lowest terms, `t^2 = n^2 r / d^2` and `gcd(n^2 r, d^2) = gcd(r, d^2) = h`, so
    `P = n^2 (r/h)`, `Q = d^2/h` are coprime (`gcd(r/h, d^2/h) = 1`, `gcd(n, d) = 1`): `m' = P Q`, `s = sign(n)`.
    """

    if not numerator:
        return None
    common = gcd(numerator, denominator)
    if common != 1:
        numerator //= common
        denominator //= common
    square = denominator * denominator
    shared = gcd(radicand, square)
    quotient = square // shared
    return (numerator * numerator * (radicand // shared) * quotient, 1 if numerator > 0 else -1, quotient)


def _class_root(keep: int, new: int) -> int:
    """`isqrt(keep * new)` when the product is a perfect square (one class), else 0 (the radicands are positive)."""

    product = keep * new
    root = isqrt(product)
    return root if root * root == product else 0


def _merge_into(slot: list, radicand: int, numerator: int, denominator: int, root: int) -> None:
    """Adds `numerator/denominator * sqrt(radicand)` to the class `slot = [r, n, d]`, `root = isqrt(r * radicand)`, on the smaller radicand."""

    keep, n, d = slot
    if radicand < keep:
        # sqrt(keep) = root / radicand * sqrt(radicand)
        n, d = n * root, d * radicand
        slot[0] = radicand
    else:
        # sqrt(radicand) = root / keep * sqrt(keep)
        numerator, denominator = numerator * root, denominator * keep
    slot[1], slot[2] = _reduced(n * denominator + numerator * d, d * denominator)


def _accumulate(slots: list, index: dict, radicand: int, numerator: int, denominator: int) -> int:
    """Adds an irrational `numerator/denominator * sqrt(radicand)` to the class slots (creation order); returns the class checks made.

    An equal radicand merges without a check; otherwise every slot, in creation order, is asked once until one is the same class.
    """

    slot = index.get(radicand)
    if slot is not None:
        n, d = slot[1], slot[2]
        slot[1], slot[2] = _reduced(n * denominator + numerator * d, d * denominator)
        return 0
    checks = 0
    for slot in slots:
        checks += 1
        keep = slot[0]
        root = _class_root(keep, radicand)
        if root:
            _merge_into(slot, radicand, numerator, denominator, root)
            if slot[0] != keep:
                del index[keep]
                index[slot[0]] = slot
            return checks
    slot = [radicand, numerator, denominator]
    slots.append(slot)
    index[radicand] = slot
    return checks


def _freeze(rational_numerator: int, rational_denominator: int, slots: list) -> tuple[Term, ...]:
    """The canonical terms of a rational part and class slots: each class normalised to its triple, zero classes dropped, `m'` order."""

    terms = []
    for radicand, numerator, denominator in slots:
        term = _irrational_term(radicand, numerator, denominator)
        if term is not None:
            terms.append(term)
    terms.sort()
    if rational_numerator:
        terms.insert(0, (1, *_reduced(rational_numerator, rational_denominator)))
    return tuple(terms)


def _negated(terms: tuple[Term, ...]) -> tuple[Term, ...]:
    return tuple([(radicand, -numerator, denominator) for radicand, numerator, denominator in terms])


def _combine(left: Term, right: Term, root: int) -> Term | None:
    """`left + right` of two irrational terms of ONE class: equal `m'`, or `root = isqrt(a b)` and the sum on the smaller radicand."""

    a, n1, d1 = left
    b, n2, d2 = right
    if a == b:
        return _irrational_term(a, n1 * d2 + n2 * d1, d1 * d2)
    if a < b:
        # sqrt(b) = root / a * sqrt(a)
        return _irrational_term(a, n1 * d2 * a + n2 * root * d1, d1 * d2 * a)
    return _irrational_term(b, n2 * d1 * b + n1 * root * d2, d1 * d2 * b)


def _add_one(left: Term, right: Term) -> tuple[tuple[Term, ...], int]:
    """`left + right` of two single canonical terms (the common case): the same `m'`, two classes, or one class by one check."""

    a, n1, d1 = left
    b, n2, d2 = right
    if a == 1 and b == 1:
        numerator, denominator = _reduced(n1 * d2 + n2 * d1, d1 * d2)
        return (((1, numerator, denominator),) if numerator else ()), 0
    if a == b:
        term = _combine(left, right, 0)
        return ((term,) if term is not None else ()), 0
    if a == 1 or b == 1:
        return ((left, right) if a < b else (right, left)), 0
    root = _class_root(a, b)
    if not root:
        return ((left, right) if a < b else (right, left)), 1
    term = _combine(left, right, root)
    return ((term,) if term is not None else ()), 1


def _add_terms(left: tuple[Term, ...], right: tuple[Term, ...], sign: int = 1) -> tuple[tuple[Term, ...], int]:
    """`left + sign * right` of two canonical term tuples, and the class checks.

    The terms of each operand are pairwise different classes, so a term of `right` meets at most one term of `left` (an equal
    `m'` first, without a check; else the open terms of `left` in `m'` order, one check each), and a term of `left` that met
    one is never asked again; a term that met none is kept as it is (already canonical).
    """

    if not right:
        return left, 0
    if sign < 0:
        right = _negated(right)
    if not left:
        return right, 0
    if len(left) == 1 and len(right) == 1:
        return _add_one(left[0], right[0])
    rational_n, rational_d = 0, 1
    if left[0][0] == 1:
        rational_n, rational_d = left[0][1], left[0][2]
        left = left[1:]
    open_terms = list(left)
    position = {term[0]: index for index, term in enumerate(open_terms)}
    out: list = []
    checks = 0
    for term in right:
        radicand = term[0]
        if radicand == 1:
            rational_n, rational_d = _reduced(rational_n * term[2] + term[1] * rational_d, rational_d * term[2])
            continue
        index = position.get(radicand)
        root = 0
        if index is None:
            for at, candidate in enumerate(open_terms):
                if candidate is None:
                    continue
                checks += 1
                root = _class_root(candidate[0], radicand)
                if root:
                    index = at
                    break
        if index is None:
            out.append(term)
            continue
        merged = _combine(open_terms[index], term, root)
        open_terms[index] = None
        if merged is not None:
            out.append(merged)
    out.extend(term for term in open_terms if term is not None)
    out.sort()
    if rational_n:
        out.insert(0, (1, rational_n, rational_d))
    return tuple(out), checks


def _scaled_terms(terms: tuple[Term, ...], numerator: int, denominator: int) -> tuple[Term, ...]:
    """`terms * numerator/denominator` (nonzero, `denominator > 0`): every class stays a class, its `m'` moves, the order is re-sorted."""

    out = []
    for radicand, n, d in terms:
        if radicand == 1:
            out.append((1, *_reduced(n * numerator, d * denominator)))
        else:
            out.append(_irrational_term(radicand, n * numerator, d * denominator))
    out.sort()
    return tuple(out)


def _is_rational_terms(terms: tuple[Term, ...]) -> bool:
    return len(terms) == 1 and terms[0][0] == 1


def _mul_terms(left: tuple[Term, ...], right: tuple[Term, ...]) -> tuple[tuple[Term, ...], int]:
    """`left * right`, and the class checks: products left-outer, right-inner, each into the class slots.

    A rational operand only scales the other (no class can merge). `sqrt(a) sqrt(b) = gcd * sqrt((a/g)(b/g))`; whether that
    radicand is a square is one class check; equal radicands give the rational `a` without one.
    """

    if not left or not right:
        return (), 0
    if _is_rational_terms(left):
        return _scaled_terms(right, left[0][1], left[0][2]), 0
    if _is_rational_terms(right):
        return _scaled_terms(left, right[0][1], right[0][2]), 0
    rational_n, rational_d = 0, 1
    slots: list = []
    index: dict = {}
    checks = 0
    for a, n1, d1 in left:
        for b, n2, d2 in right:
            numerator, denominator = n1 * n2, d1 * d2
            if a == 1:
                radicand = b
            elif b == 1:
                radicand = a
            elif a == b:
                radicand = 1
                numerator *= a
            else:
                common = gcd(a, b)
                radicand = (a // common) * (b // common)
                numerator *= common
                checks += 1
                root = isqrt(radicand)
                if root * root == radicand:
                    numerator *= root
                    radicand = 1
            if radicand == 1:
                rational_n, rational_d = _reduced(rational_n * denominator + numerator * rational_d, rational_d * denominator)
            else:
                checks += _accumulate(slots, index, radicand, numerator, denominator)
    return _freeze(rational_n, rational_d, slots), checks


def _inverse_terms(terms: tuple[Term, ...]) -> tuple[tuple[Term, ...], NativeAlgebraWorkV1]:
    count = len(terms)
    if not count:
        raise ZeroDivisionError("division by an exact zero")
    if count == 1:
        radicand, numerator, denominator = terms[0]
        if radicand == 1:
            return ((1, denominator if numerator > 0 else -denominator, abs(numerator)),), NO_WORK
        return ((radicand, numerator, radicand // denominator),), NO_WORK
    if count > INVERSE_CLASS_LIMIT:
        raise RadicalInverseClassLimit(f"{count} classes, limit {INVERSE_CLASS_LIMIT}")
    pivot, numerator, denominator = terms[-1]
    rest = terms[:-1]
    conjugate = rest + ((pivot, -numerator, denominator),)
    square, first = _mul_terms(rest, rest)
    # pivot^2 = m'/Q^2 = P/Q (lowest terms)
    norm, second = _add_terms(square, ((1, -(pivot // denominator), denominator),))
    inverse_norm, inner = _inverse_terms(norm)
    result, third = _mul_terms(conjugate, inverse_norm)
    return result, NativeAlgebraWorkV1(first + second + third, 2) + inner


def _integer_form(terms: tuple[Term, ...]) -> tuple[int, list[tuple[int, int]]]:
    """`(L, [(m', a)])` with `t = a sqrt(m') / L` (irrational) or `t = a / L` (rational), `L` the least common denominator."""

    common = 1
    for _, _, denominator in terms:
        if denominator != 1:
            common = lcm(common, denominator)
    return common, [(radicand, numerator * (common // denominator)) for radicand, numerator, denominator in terms]


def _integer_enclosure(items: list[tuple[int, int]], bits: int) -> tuple[int, int]:
    """Integer bounds of `2^bits * sum a sqrt(m')`, outward: `isqrt` is the lower node of each root, the next node the upper."""

    scale = 1 << bits
    shift = 2 * bits
    low = high = 0
    for radicand, numerator in items:
        if radicand == 1:
            exact = numerator * scale
            low += exact
            high += exact
            continue
        floor_root = isqrt(radicand << shift)
        if numerator > 0:
            low += numerator * floor_root
            high += numerator * (floor_root + 1)
        else:
            low += numerator * (floor_root + 1)
            high += numerator * floor_root
    return low, high


def separation_bits(items: list[tuple[int, int]]) -> int:
    """Bits at which the integer enclosure of `sum a sqrt(m')` (>= 2 distinct classes, all `a != 0`) cannot hold zero (module docstring)."""

    upper = 0
    width = 0
    for radicand, numerator in items:
        magnitude = abs(numerator)
        if radicand == 1:
            upper += magnitude
        else:
            upper += magnitude * (isqrt(radicand) + 1)
            width += magnitude
    exponent = (1 << (len(items) - 1)) - 1
    return width.bit_length() + exponent * upper.bit_length()


def _square_of(term: Term) -> tuple[int, int]:
    """`t^2` as `(numerator, denominator)`: `n^2/d^2` for the rational term, `m'/Q^2` for an irrational one."""

    radicand, numerator, denominator = term
    return (numerator * numerator if radicand == 1 else radicand), denominator * denominator


def _signum_terms(terms: tuple[Term, ...], cap: int | None) -> tuple[int, int]:
    """`(sign, enclosure rounds)`."""

    count = len(terms)
    if not count:
        return 0, 0
    if count == 1:
        return (1 if terms[0][1] > 0 else -1), 0
    if count == 2:
        first_sign = 1 if terms[0][1] > 0 else -1
        second_sign = 1 if terms[1][1] > 0 else -1
        if first_sign == second_sign:
            return first_sign, 0
        (p1, q1), (p2, q2) = _square_of(terms[0]), _square_of(terms[1])
        # Equal squares would mean one class; the classes are different.
        return (first_sign if p1 * q2 > p2 * q1 else second_sign), 0
    _, items = _integer_form(terms)
    # Rungs: 64, then min(128, bound), min(256, bound), ...; the bound is computed only when the first rung fails (a rung at or
    # above the bound always decides, so a bound below 64 bits can never be the reason the first rung failed).
    required = None
    rung = SIGN_FIRST_BITS
    rounds = 0
    while True:
        if cap is not None and rung > cap:
            raise RadicalSignUndecided(
                f"{count} classes: the separation bound needs {required or 'more than ' + str(cap)} bits, the cap is {cap}",
                NativeAlgebraWorkV1(enclosure_rounds=rounds),
            )
        rounds += 1
        low, high = _integer_enclosure(items, rung)
        if low > 0:
            return 1, rounds
        if high < 0:
            return -1, rounds
        if required is None:
            required = separation_bits(items)
        if rung >= required:
            raise RadicalSeparationBoundViolated(f"{count} classes, enclosure [{low}, {high}] at {rung} bits, bound {required}")
        rung = min(2 * rung, required)


# --------------------------------------------------------------------------
# the type
# --------------------------------------------------------------------------


Priced = tuple["RadicalSumV2", NativeAlgebraWorkV1]


def _coerce(other: object) -> "RadicalSumV2":
    if type(other) is RadicalSumV2:
        return other
    if isinstance(other, (int, Fraction)) and not isinstance(other, bool):
        return RadicalSumV2.rational(other)
    raise TypeError(f"cannot read {type(other).__name__} as a RadicalSumV2")


@dataclass(frozen=True, slots=True)
class RadicalSumV2:
    """`sum s_k sqrt(m'_k) / Q_k` (+ a rational `n/d` as the term `(1, n, d)`): value-canonical integer triples sorted by `m'`.

    The terms are a function of the value: `==` and `hash` compare and hash them (exact value equality, Besicovitch).
    Construct through `zero`, `rational`, `sqrt_of_rational`, `from_terms`, `value_of_wire`, `value_of_text` or an operation;
    the bare constructor trusts its argument.
    """

    terms: tuple[Term, ...]

    # ---- construction ------------------------------------------------------

    @staticmethod
    def zero() -> "RadicalSumV2":
        return _ZERO

    @staticmethod
    def rational(value: Fraction | int) -> "RadicalSumV2":
        value = Fraction(value)
        return RadicalSumV2(((1, value.numerator, value.denominator),)) if value else _ZERO

    @staticmethod
    def sqrt_of_rational(value: Fraction | int) -> Priced:
        """`sqrt(p/q) = sqrt(p q) / q`: the term `(p q, 1, q)` (`p/q` lowest), or a rational when `p q` is a square (one class check)."""

        value = Fraction(value)
        if value < 0:
            raise RadicalOutsideField("root of a negative number")
        if not value:
            return _ZERO, NO_WORK
        numerator, denominator = value.numerator, value.denominator
        radicand = numerator * denominator
        if radicand == 1:
            return _ONE, NO_WORK
        root = isqrt(radicand)
        if root * root == radicand:
            return RadicalSumV2.rational(Fraction(root, denominator)), _checks(1)
        return RadicalSumV2(((radicand, 1, denominator),)), _checks(1)

    @staticmethod
    def from_terms(pairs) -> Priced:
        """`sum c * sqrt(r)` over `(r, c)` pairs (`r >= 0` integer, `c` int or Fraction) in ANY representation, read by value.

        A radicand that is a perfect square (one class check unless it is 1) goes to the rational part; the others are merged
        by class (`_accumulate`) and normalised. A negative radicand is `RadicalOutsideField`.
        """

        rational_n, rational_d = 0, 1
        slots: list = []
        index: dict = {}
        checks = 0
        for radicand, coefficient in pairs:
            coefficient = Fraction(coefficient)
            if radicand < 0:
                raise RadicalOutsideField("root of a negative number", _checks(checks))
            if not coefficient or not radicand:
                continue
            numerator, denominator = coefficient.numerator, coefficient.denominator
            if radicand != 1:
                checks += 1
                root = isqrt(radicand)
                if root * root != radicand:
                    checks += _accumulate(slots, index, radicand, numerator, denominator)
                    continue
                numerator *= root
            rational_n, rational_d = _reduced(rational_n * denominator + numerator * rational_d, rational_d * denominator)
        return RadicalSumV2(_freeze(rational_n, rational_d, slots)), _checks(checks)

    # ---- reading -----------------------------------------------------------

    @property
    def is_zero(self) -> bool:
        """The exact zero test (Besicovitch): no terms."""

        return not self.terms

    def as_rational(self) -> Fraction | None:
        """The value if it is rational, else `None` (an irrational class never cancels to a rational)."""

        terms = self.terms
        if not terms:
            return Fraction(0)
        if len(terms) == 1 and terms[0][0] == 1:
            return Fraction(terms[0][1], terms[0][2])
        return None

    def pairs(self) -> tuple[tuple[int, Fraction], ...]:
        """The terms as V1-shaped `(radicand, coefficient)` pairs: `(m', s/Q)`, `(1, n/d)` (what `float_filter._measure` reads)."""

        return tuple((radicand, Fraction(numerator, denominator)) for radicand, numerator, denominator in self.terms)

    # ---- arithmetic (priced) -------------------------------------------------

    def neg(self) -> "RadicalSumV2":
        return RadicalSumV2(_negated(self.terms))

    __neg__ = neg

    def scaled(self, factor: Fraction | int) -> "RadicalSumV2":
        """`self * factor` for a rational factor: no class can merge, so it costs nothing."""

        factor = Fraction(factor)
        if not factor or not self.terms:
            return _ZERO
        return RadicalSumV2(_scaled_terms(self.terms, factor.numerator, factor.denominator))

    def add(self, other) -> Priced:
        terms, checks = _add_terms(self.terms, _coerce(other).terms)
        return RadicalSumV2(terms), _checks(checks)

    def sub(self, other) -> Priced:
        terms, checks = _add_terms(self.terms, _coerce(other).terms, -1)
        return RadicalSumV2(terms), _checks(checks)

    def mul(self, other) -> Priced:
        terms, checks = _mul_terms(self.terms, _coerce(other).terms)
        return RadicalSumV2(terms), _checks(checks)

    def inverse(self) -> Priced:
        """`1 / self`; `ZeroDivisionError` for the exact zero; more than three classes: `RadicalInverseClassLimit`."""

        terms, work = _inverse_terms(self.terms)
        return RadicalSumV2(terms), work

    def div(self, other) -> Priced:
        inverse, work = _coerce(other).inverse()
        value, product = self.mul(inverse)
        return value, work + product

    # ---- decisions ---------------------------------------------------------

    def signum(self, precision_cap_bits: int | None = None) -> tuple[int, NativeAlgebraWorkV1]:
        """The exact sign and its price; `precision_cap_bits` is the only way to get `RadicalSignUndecided`."""

        sign, rounds = _signum_terms(self.terms, precision_cap_bits)
        return sign, (NativeAlgebraWorkV1(enclosure_rounds=rounds) if rounds else NO_WORK)

    def enclosure(self, bits: int) -> tuple[tuple[Fraction, Fraction], NativeAlgebraWorkV1]:
        """Outward `[low, high]`: each term `s sqrt(P/Q)` within `[isqrt(PQ 4^bits), +1] / (Q 2^bits)`, summed over the common denominator."""

        common, items = _integer_form(self.terms)
        low, high = _integer_enclosure(items, bits)
        denominator = common << bits
        return (Fraction(low, denominator), Fraction(high, denominator)), NativeAlgebraWorkV1(enclosure_rounds=1)

    # ---- identity ----------------------------------------------------------

    def text(self) -> str:
        return canonical_text(self)

    def wire(self) -> tuple[int, ...]:
        return terms_wire(self)

    def __repr__(self) -> str:
        return f"RadicalSumV2({canonical_text(self)})"


_ZERO = RadicalSumV2(())
_ONE = RadicalSumV2(((1, 1, 1),))


# --------------------------------------------------------------------------
# wire and text
# --------------------------------------------------------------------------


def terms_wire(value: RadicalSumV2) -> tuple[int, ...]:
    """The canonical terms as one flat tuple of ints `(m', n, d, ...)` in `m'` order; the empty tuple is zero."""

    flat: list[int] = []
    for term in value.terms:
        flat.extend(term)
    return tuple(flat)


def value_of_wire(wire: tuple[int, ...]) -> RadicalSumV2:
    """The inverse of `terms_wire`, for a wire `terms_wire` wrote: the SHAPE is checked (`ValueError`), the class distinctness is trusted.

    Shape: triples; `m'` strictly increasing; `(1, n, d)` with `n != 0`, `d > 0`, `gcd(n, d) = 1`; `(m', +-1, Q)` with
    `m' > 1`, `Q | m'`, `gcd(m'/Q, Q) = 1`.
    """

    if len(wire) % 3:
        raise ValueError(f"{RADICAL_SUM_V2} wire of {len(wire)} ints is not a list of triples")
    terms = tuple((wire[at], wire[at + 1], wire[at + 2]) for at in range(0, len(wire), 3))
    previous = 0
    for radicand, numerator, denominator in terms:
        if radicand <= previous or denominator <= 0 or not numerator:
            raise ValueError(f"{RADICAL_SUM_V2} wire {wire} is not canonical")
        if radicand == 1:
            if gcd(numerator, denominator) != 1:
                raise ValueError(f"{RADICAL_SUM_V2} wire {wire} has an unreduced rational term")
        elif abs(numerator) != 1 or radicand % denominator or gcd(radicand // denominator, denominator) != 1:
            raise ValueError(f"{RADICAL_SUM_V2} wire {wire} has a non-canonical irrational term")
        previous = radicand
    return RadicalSumV2(terms)


def _rational_text(numerator: int, denominator: int) -> str:
    return f"Integer({numerator})" if denominator == 1 else f"Rational({numerator}, {denominator})"


def _term_text(term: Term) -> str:
    radicand, numerator, denominator = term
    if radicand == 1:
        return _rational_text(numerator, denominator)
    sign = "-" if numerator < 0 else ""
    return f"{sign}Sqrt(Rational({radicand // denominator}, {denominator}))"


def canonical_text(value: RadicalSumV2) -> str:
    """`Integer(0)`; one term: its EXACT_SCALAR_TEXT_CANON_V2 text; more: `Sum(<term>, ...)` in `m'` order. A function of the value."""

    terms = value.terms
    if not terms:
        return "Integer(0)"
    if len(terms) == 1:
        return _term_text(terms[0])
    return "Sum(" + ", ".join(_term_text(term) for term in terms) + ")"


_TERM_TEXT = re.compile(
    r"Integer\((?P<int>-?\d+)\)"
    r"|Rational\((?P<num>-?\d+), (?P<den>\d+)\)"
    r"|(?P<neg>-)?Sqrt\(Rational\((?P<p>\d+), (?P<q>\d+)\)\)"
)


def _text_pairs(body: str) -> list[tuple[int, Fraction]]:
    pairs = []
    at = 0
    while True:
        match = _TERM_TEXT.match(body, at)
        if match is None:
            raise ValueError(f"{RADICAL_SUM_V2} cannot read {body!r} at {at}")
        if match.group("int") is not None:
            pairs.append((1, Fraction(int(match.group("int")))))
        elif match.group("num") is not None:
            pairs.append((1, Fraction(int(match.group("num")), int(match.group("den")))))
        else:
            p, q = int(match.group("p")), int(match.group("q"))
            if not q:
                raise ValueError(f"{RADICAL_SUM_V2} text {body!r} divides by zero")
            # sqrt(p/q) = sqrt(p q) / q
            pairs.append((p * q, Fraction(-1 if match.group("neg") else 1, q)))
        at = match.end()
        if at == len(body):
            return pairs
        if not body.startswith(", ", at):
            raise ValueError(f"{RADICAL_SUM_V2} cannot read {body!r} at {at}")
        at += 2


def value_of_text(text: str) -> Priced:
    """The value of a text made of the `canonical_text` shapes, read BY VALUE (`from_terms`); `canonical_text` of it gives a canonical text back."""

    if text.startswith("Sum(") and text.endswith(")"):
        return RadicalSumV2.from_terms(_text_pairs(text[4:-1]))
    pairs = _text_pairs(text)
    if len(pairs) != 1:
        raise ValueError(f"{RADICAL_SUM_V2} text {text!r}: several terms outside Sum(...)")
    return RadicalSumV2.from_terms(pairs)


# --------------------------------------------------------------------------
# V1 <-> V2, by value
# --------------------------------------------------------------------------


def from_v1(value) -> Priced:
    """The V2 value of a `RadicalSumV1` (its `(radicand, Fraction)` terms, any representatives), read by value."""

    return RadicalSumV2.from_terms(value.terms)


def to_v1(value: RadicalSumV2):
    """A `RadicalSumV1` of the same value whose representatives are the canonical `m'` (V1's invariant holds: distinct classes, sorted)."""

    from .native_exact import RadicalSumV1

    return RadicalSumV1(value.pairs())
