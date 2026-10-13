"""Owned finite construction CANDIDATE, not a product construction/sign canon.

Actual ScalarV2 islands stay atomic. Formal operations use a closed literal-tree
order and actual ScalarV2 monomial coefficient collection. Export is a read-only decision
view; subsequent arithmetic consumes owned handles, never an exported SymPy AST.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .density_scalar_v2 import DENSITY_SOURCE_CONSTRUCTION_V1, ScalarLiteralAstV1, ScalarV2
from .radical_v2 import RadicalSumV2

MAX_INTEGER_POWER = 2**20


class DensityConstructProfileCode(str, Enum):
    NEGATIVE_RATIONAL_ROOT = "DENSITY_CONSTRUCT_V1_NEGATIVE_RATIONAL_ROOT"
    NEGATIVE_OPAQUE_POWER = "DENSITY_CONSTRUCT_V1_NEGATIVE_OPAQUE_POWER"
    POWER_PROFILE = "DENSITY_CONSTRUCT_V1_POWER_PROFILE"
    WIRE_PROFILE = "DENSITY_CONSTRUCT_V1_WIRE_PROFILE"
    ARENA_MISMATCH = "DENSITY_CONSTRUCT_V1_ARENA_MISMATCH"
    RATIONAL_NORM_REQUIRED = "DENSITY_CONSTRUCT_V1_RATIONAL_NORM_REQUIRED"
    PARTIAL_DOMAIN_DISCARD = "DENSITY_CONSTRUCT_V1_PARTIAL_DOMAIN_DISCARD"
    EXPANSION_LIMIT = "DENSITY_CONSTRUCT_V1_EXPANSION_LIMIT"
    OPAQUE_CHILD_CHANGES = "DENSITY_CONSTRUCT_V1_OPAQUE_CHILD_CHANGES"


class DensityConstructProfileUnsupported(ValueError):
    """Internal candidate boundary, never a geometry refusal or UNDECIDABLE."""

    def __init__(self, code: DensityConstructProfileCode, detail: str) -> None:
        super().__init__(f"{code.value}: {detail}")
        self.code, self.detail = code, detail


@dataclass(frozen=True, slots=True)
class OwnedExprV1:
    arena: DensityConstructArenaV1
    node: int
    canon: str = DENSITY_SOURCE_CONSTRUCTION_V1


@dataclass(frozen=True, slots=True)
class _NonzeroWitnessV1:
    """Internal capability issued only by the explicit old-admission gateway."""

    expression: OwnedExprV1
    sign: int


@dataclass(frozen=True, slots=True)
class _Node:
    kind: str
    children: tuple[int, ...] = ()
    payload: ScalarV2 | Fraction | int | None = None


@dataclass(frozen=True, slots=True)
class OwnedFormV1:
    """Read-only semantic node view; literal AST rows are not arithmetic input."""

    kind: str
    children: tuple[OwnedExprV1, ...]
    payload: ScalarV2 | Fraction | int | None


@dataclass(frozen=True, slots=True)
class ExpandLimitsV1:
    """Explicit internal resource profile, unrelated to product DensityWork.

    Zero caps are valid profile inputs; lazy decision success never invokes the
    expander. Term choices/products are checked before allocation. Rational-bit
    caps apply to whole-rational values produced during expansion multiplication.
    """

    max_terms: int
    max_polynomial_exponent: int
    max_rational_power_bits: int

    def __post_init__(self) -> None:
        for value in (self.max_terms, self.max_polynomial_exponent, self.max_rational_power_bits):
            if type(value) is not int or value < 0:
                raise ValueError("Expansion caps must be nonnegative Python integers")


def _fraction(value: Fraction | int) -> Fraction:
    if type(value) is int or isinstance(value, Fraction):
        return Fraction(value)
    raise TypeError("Owned construction requires a Python int/Fraction")


def _power_exponent(exponent: int) -> int:
    if type(exponent) is not int:
        raise TypeError("Owned integer power requires a Python integer")
    if abs(exponent) > MAX_INTEGER_POWER:
        raise DensityConstructProfileUnsupported(
            DensityConstructProfileCode.POWER_PROFILE, "integer exponent exceeds |e| <= 2**20",
        )
    return exponent


def _tree_key(tree: tuple) -> tuple:
    """Closed Basic.compare order: class, content/arity, recursive children."""
    tag, *args = tree
    if tag == "Q":
        n, d = args
        code = 0 if n == 0 else 1 if (n, d) == (1, 1) else 2 if (n, d) == (1, 2) else 5 if (n, d) == (-1, 1) else 7 if d == 1 else 8
        return (code, "", n, d)
    code = {"pi": 11, "Pow": 15, "Mul": 16, "Add": 17}.get(tag, 255)
    return (code, tag if code == 255 else "", len(args), tuple(_tree_key(arg) for arg in args))


class DensityConstructArenaV1:
    """One append-only transaction; no process cache, SymPy or root registry.

    Numeric islands are never flattened by their literal Add/Mul. Formal Add
    collects actual ScalarV2 coefficients of identical opaque monomials. Mul combines all
    numeric factors by V2 and collects opaque nonnegative integer powers only.
    Ordering compares deterministic literal trees, never arena allocation IDs.
    """

    def __init__(self) -> None:
        self._nodes: list[_Node] = []
        self._index: dict[_Node, int] = {}
        self._trees: dict[int, tuple] = {}
        self._partial: set[int] = set()

    def _handle(self, node: int) -> OwnedExprV1:
        return OwnedExprV1(self, node)

    def _check(self, expr: OwnedExprV1) -> _Node:
        if (not isinstance(expr, OwnedExprV1) or expr.arena is not self
                or expr.canon != DENSITY_SOURCE_CONSTRUCTION_V1
                or type(expr.node) is not int or not 0 <= expr.node < len(self._nodes)):
            raise DensityConstructProfileUnsupported(
                DensityConstructProfileCode.ARENA_MISMATCH, "foreign, invalid or mismatched-canon handle",
            )
        return self._nodes[expr.node]

    def _intern(self, kind: str, children=(), payload=None) -> OwnedExprV1:
        node = _Node(kind, tuple(children), payload)
        if node not in self._index:
            self._index[node] = len(self._nodes)
            self._nodes.append(node)
            if ((kind in ("IntegerPow", "OpaquePow") and payload < 0)
                    or any(child in self._partial for child in node.children)):
                self._partial.add(self._index[node])
        return self._handle(self._index[node])

    def _check_discard(self, expr: OwnedExprV1) -> None:
        self._check(expr)
        if expr.node in self._partial:
            raise DensityConstructProfileUnsupported(
                DensityConstructProfileCode.PARTIAL_DOMAIN_DISCARD,
                "simplification would discard an unproved partial reciprocal domain",
            )

    def numeric(self, scalar: ScalarV2) -> OwnedExprV1:
        if not isinstance(scalar, ScalarV2):
            raise TypeError("numeric requires an actual ScalarV2")
        return self._intern("Numeric", payload=scalar)

    def rational(self, value: Fraction | int) -> OwnedExprV1:
        return self.numeric(ScalarV2.rational(_fraction(value)))

    def scalar(self, expr: OwnedExprV1) -> ScalarV2 | None:
        node = self._check(expr)
        return node.payload if node.kind == "Numeric" else None

    def owned_form(self, expr: OwnedExprV1) -> OwnedFormV1:
        node = self._check(expr)
        return OwnedFormV1(node.kind, tuple(self._handle(child) for child in node.children), node.payload)

    def has_partial_domain(self, expr: OwnedExprV1) -> bool:
        self._check(expr)
        return expr.node in self._partial

    def whole_rational(self, expr: OwnedExprV1) -> Fraction | None:
        scalar = self.scalar(expr)
        return None if scalar is None else scalar.value.as_rational()

    def pi(self) -> OwnedExprV1:
        return self._intern("pi")

    def _opaque(self, kind: str, *args: OwnedExprV1) -> OwnedExprV1:
        for arg in args:
            self._check(arg)
        return self._intern(kind, (arg.node for arg in args))

    def raw_sin(self, arg: OwnedExprV1) -> OwnedExprV1:
        return self._opaque("sin", arg)

    def raw_cos(self, arg: OwnedExprV1) -> OwnedExprV1:
        return self._opaque("cos", arg)

    def raw_atan2(self, y: OwnedExprV1, x: OwnedExprV1) -> OwnedExprV1:
        return self._opaque("atan2", y, x)

    def literal_tree(self, expr: OwnedExprV1) -> tuple:
        self._check(expr)
        if expr.node not in self._trees:
            literal = self.export_raw(expr)
            trees = []
            for tag, *args in literal.nodes:
                trees.append((tag, *args) if tag == "Q" else (tag, *(trees[index] for index in args)))
            self._trees[expr.node] = trees[literal.root]
        return self._trees[expr.node]

    def _order(self, expr: OwnedExprV1) -> tuple:
        return _tree_key(self.literal_tree(expr))

    def _raw_add(self, args: tuple[OwnedExprV1, ...]) -> OwnedExprV1:
        if not args:
            return self.rational(0)
        if len(args) == 1:
            return args[0]
        return self._intern("Add", (arg.node for arg in args))

    def _raw_mul(self, coefficient: ScalarV2 | Fraction, remainder: OwnedExprV1) -> OwnedExprV1:
        """Reassembly preserves coefficient*FormalAdd without distribution."""
        coefficient = coefficient if isinstance(coefficient, ScalarV2) else ScalarV2.rational(coefficient)
        if coefficient.value.is_zero:
            self._check_discard(remainder)
            return self.rational(0)
        if coefficient.value.as_rational() == 1:
            return remainder
        node = self._check(remainder)
        if node.kind == "Numeric":
            return self.numeric(node.payload.mul(coefficient))
        head = self.numeric(coefficient)
        if node.kind == "Mul":
            return self.mul(head, *(self._handle(child) for child in node.children))
        factors = [head, remainder]
        if coefficient.value.as_rational() is None:
            factors.sort(key=self._order)
        return self._intern("Mul", (factor.node for factor in factors))

    def _split_coefficient(self, expr: OwnedExprV1) -> tuple[ScalarV2, OwnedExprV1]:
        node = self._check(expr)
        if node.kind == "Mul" and node.children:
            coefficient, tail = ScalarV2.rational(1), []
            for child in node.children:
                scalar = self.scalar(self._handle(child))
                if scalar is None:
                    tail.append(child)
                else:
                    coefficient = coefficient.mul(scalar)
            if len(tail) != len(node.children):
                if not tail:
                    return coefficient, self.rational(1)
                if len(tail) == 1:
                    return coefficient, self._handle(tail[0])
                return coefficient, self._intern("Mul", tail)
        return ScalarV2.rational(1), expr

    def add(self, *args: OwnedExprV1) -> OwnedExprV1:
        for arg in args:
            self._check(arg)
        queue = list(args)
        numeric = ScalarV2.rational(0)
        groups: dict[tuple, tuple[ScalarV2, OwnedExprV1]] = {}
        while queue:
            arg = queue.pop(0)
            node = self._check(arg)
            if node.kind == "Add":
                queue[0:0] = [self._handle(child) for child in node.children]
            elif node.kind == "Numeric":
                numeric = numeric.add(node.payload)
            else:
                coefficient, remainder = self._split_coefficient(arg)
                key = self.literal_tree(remainder)
                previous = groups.get(key, (ScalarV2.rational(0), remainder))
                groups[key] = (previous[0].add(coefficient), previous[1])
        for coefficient, remainder in groups.values():
            if coefficient.value.is_zero:
                self._check_discard(remainder)
        terms = [self._raw_mul(coefficient, remainder)
                 for coefficient, remainder in groups.values() if not coefficient.value.is_zero]
        if not numeric.value.is_zero:
            terms.append(self.numeric(numeric))
        return self._raw_add(tuple(sorted(terms, key=self._order)))

    def _rational_add_branch(self, left: OwnedExprV1, right: OwnedExprV1) -> OwnedExprV1 | None:
        coefficient = self.whole_rational(left)
        if coefficient is None:
            return None
        node = self._check(right)
        if node.kind == "Add":
            return self.add(*(self._raw_mul(coefficient, self._handle(child)) for child in node.children))
        inner, remainder = self._split_coefficient(right)
        rational = inner.value.as_rational()
        if rational is not None and rational != 1 and self._check(remainder).kind == "Add":
            return self._raw_mul(coefficient * rational, remainder)
        return None

    def mul(self, *args: OwnedExprV1) -> OwnedExprV1:
        for arg in args:
            self._check(arg)
        if len(args) == 2:
            branch = self._rational_add_branch(args[0], args[1])
            if branch is None:
                branch = self._rational_add_branch(args[1], args[0])
            if branch is not None:
                return branch
        queue = list(args)
        numeric = ScalarV2.rational(1)
        powers: dict[tuple, tuple[OwnedExprV1, int]] = {}
        positive_halves: dict[tuple, OwnedExprV1] = {}
        atomic = []
        while True:
            while queue:
                arg = queue.pop(0)
                node = self._check(arg)
                if node.kind == "Mul":
                    queue[0:0] = [self._handle(child) for child in node.children]
                elif node.kind == "Numeric":
                    numeric = numeric.mul(node.payload)
                elif node.kind == "OpaquePow":
                    exponent = node.payload
                    if exponent > 0 and exponent.denominator == 2:
                        key = self.literal_tree(arg)
                        if key in positive_halves:
                            del positive_halves[key]
                            base = self._handle(node.children[0])
                            queue.insert(0, self.integer_power(base, exponent.numerator))
                        else:
                            positive_halves[key] = arg
                    else:
                        atomic.append(arg)
                else:
                    base, exponent = (self._handle(node.children[0]), node.payload) if node.kind == "IntegerPow" else (arg, 1)
                    if exponent < 0:
                        atomic.append(arg)
                        continue
                    key = self.literal_tree(base)
                    previous = powers.get(key, (base, 0))
                    powers[key] = (previous[0], _power_exponent(previous[1] + exponent))
            # Сначала полная сумма степеней: промежуточная чётность не меняет канон.
            for key, (base, exponent) in sorted(tuple(powers.items()), key=lambda item: self._order(item[1][0])):
                node = self._check(base)
                if (node.kind == "OpaquePow" and node.payload > 0
                        and node.payload.denominator == 2 and exponent % 2 == 0):
                    del powers[key]
                    queue.append(self.integer_power(base, exponent))
            if not queue:
                break
        if numeric.value.is_zero:
            for arg in args:
                self._check_discard(arg)
            return self.numeric(numeric)
        factors = atomic + list(positive_halves.values()) + [base if exponent == 1 else self._intern("IntegerPow", (base.node,), exponent)
                            for base, exponent in powers.values()]
        rational = numeric.value.as_rational()
        if rational != 1:
            factors.append(self.numeric(numeric))
        factors.sort(key=self._order)
        if rational is not None and rational != 1:
            coefficient = self.numeric(numeric)
            factors.remove(coefficient)
            factors.insert(0, coefficient)
        if not factors:
            return self.rational(1)
        if len(factors) == 1:
            return factors[0]
        return self._intern("Mul", (arg.node for arg in factors))

    def scaled(self, expr: OwnedExprV1, factor: Fraction | int) -> OwnedExprV1:
        self._check(expr)
        return self.mul(self.rational(_fraction(factor)), expr)

    def neg(self, expr: OwnedExprV1) -> OwnedExprV1:
        return self.scaled(expr, -1)

    def sub(self, left: OwnedExprV1, right: OwnedExprV1) -> OwnedExprV1:
        self._check(left)
        self._check(right)
        return self.add(left, self.neg(right))

    def integer_power(self, expr: OwnedExprV1, exponent: int) -> OwnedExprV1:
        node = self._check(expr)
        exponent = _power_exponent(exponent)
        if node.kind == "Numeric":
            return self.numeric(node.payload.integer_power(exponent))
        if exponent < 0:
            raise DensityConstructProfileUnsupported(
                DensityConstructProfileCode.NEGATIVE_OPAQUE_POWER, "fresh opaque reciprocal is outside the finite profile",
            )
        if exponent == 0:
            self._check_discard(expr)
            return self.rational(1)
        if exponent == 1:
            return expr
        if (node.kind == "OpaquePow" and node.payload > 0
                and node.payload.denominator == 2 and exponent % 2 == 0):
            # Только точное тождество одинакового главного положительного half-root.
            folded = _power_exponent(node.payload.numerator * (exponent // 2))
            return self.integer_power(self._handle(node.children[0]), folded)
        return self._intern("IntegerPow", (expr.node,), exponent)

    def div(self, left: OwnedExprV1, right: OwnedExprV1) -> OwnedExprV1:
        self._check(left)
        self._check(right)
        return self.mul(left, self.integer_power(right, -1))

    def reciprocal(self, expr: OwnedExprV1) -> OwnedExprV1:
        """Partial formal reciprocal; no proof of an opaque nonzero domain.

        Numeric input delegates to actual inverse BASE first. Opaque reciprocal
        remains atomic; dropping its domain is a typed internal profile error.
        A future product producer must certify its denominator before division.
        """
        node = self._check(expr)
        if node.kind == "Numeric":
            return self.numeric(node.payload.inverse())
        return self._intern("IntegerPow", (expr.node,), -1)

    def partial_div(self, left: OwnedExprV1, right: OwnedExprV1) -> OwnedExprV1:
        self._check(left)
        self._check(right)
        return self.mul(left, self.reciprocal(right))

    def _reciprocal_with_nonzero_witness(self, expr: OwnedExprV1, witness: _NonzeroWitnessV1) -> OwnedExprV1:
        """Resolve only the introduced reciprocal obligation after old proof.

        The local proof does not remove any pre-existing partial descendants.
        Numeric inverse still uses the whole BASE first and its class limit.
        Literal/value identity is unchanged; proof metadata is arena-local.
        """
        self._check(expr)
        if (not isinstance(witness, _NonzeroWitnessV1) or witness.expression != expr
                or type(witness.sign) is not int or witness.sign not in (-1, 1)):
            raise ValueError("DENSITY_CONSTRUCT_V1_NONZERO_WITNESS_MISMATCH")
        inherited = self.has_partial_domain(expr)
        reciprocal = self.reciprocal(expr)
        if not inherited:
            self._partial.discard(reciprocal.node)
        return reciprocal

    def sqrt(self, expr: OwnedExprV1) -> OwnedExprV1:
        node = self._check(expr)
        rational = self.whole_rational(expr)
        if rational is not None:
            if rational < 0:
                raise DensityConstructProfileUnsupported(
                    DensityConstructProfileCode.NEGATIVE_RATIONAL_ROOT, "real sqrt has a negative rational argument",
                )
            return self.numeric(node.payload.sqrt())
        return self._intern("OpaquePow", (expr.node,), Fraction(1, 2))

    def raw_pow(self, expr: OwnedExprV1, exponent: Fraction | int) -> OwnedExprV1:
        self._check(expr)
        exponent = _fraction(exponent)
        _power_exponent(exponent.numerator)
        if exponent.denominator == 1:
            return self.integer_power(expr, exponent.numerator)
        if exponent.denominator != 2:
            raise DensityConstructProfileUnsupported(
                DensityConstructProfileCode.POWER_PROFILE, "only integer/half-integer powers are admitted",
            )
        if self.whole_rational(expr) is not None:
            return self.integer_power(self.sqrt(expr), exponent.numerator)
        if exponent < 0:
            raise DensityConstructProfileUnsupported(
                DensityConstructProfileCode.NEGATIVE_OPAQUE_POWER, "fresh negative opaque half-power is outside the finite profile",
            )
        return self._intern("OpaquePow", (expr.node,), exponent)

    def export_raw(self, expr: OwnedExprV1) -> ScalarLiteralAstV1:
        self._check(expr)
        rows, index, visited = [], {}, {}

        def emit(row):
            row = tuple(row)
            if row not in index:
                index[row] = len(rows)
                rows.append(row)
            return index[row]

        def visit(identifier):
            if identifier in visited:
                return visited[identifier]
            node = self._nodes[identifier]
            if node.kind == "Numeric":
                literal = node.payload.literal_ast()
                remap = []
                for tag, *args in literal.nodes:
                    remap.append(emit((tag, *args) if tag == "Q" else (tag, *(remap[child] for child in args))))
                result = remap[literal.root]
            else:
                children = tuple(visit(child) for child in node.children)
                if node.kind in ("IntegerPow", "OpaquePow"):
                    exponent = Fraction(node.payload)
                    exponent_id = emit(("Q", exponent.numerator, exponent.denominator))
                    result = emit(("Pow", *children, exponent_id))
                else:
                    result = emit((node.kind, *children))
            visited[identifier] = result
            return result

        root = visit(expr.node)
        return ScalarLiteralAstV1(DENSITY_SOURCE_CONSTRUCTION_V1, tuple(rows), root)

    def finite_expand(self, expr: OwnedExprV1, limits: ExpandLimitsV1) -> OwnedExprV1:
        """Finite polynomial distribution only, before read-only export.

        Numeric sums expose their canonical summand VALUES only where a product
        or polynomial power needs choices. Numeric literal coefficient/root Mul
        nodes are never generic factors. Opaque child changes are a profile gap.
        Left factor choices are outer, right choices inner; normalization then
        applies the same finite constructor. No generic factorization or cancel.
        """
        self._check(expr)
        if not isinstance(limits, ExpandLimitsV1):
            raise TypeError("finite_expand requires explicit ExpandLimitsV1")
        return _FiniteExpansionV1(self, limits).expand(expr)

    def export_expanded_raw(self, expr: OwnedExprV1, limits: ExpandLimitsV1) -> ScalarLiteralAstV1:
        return self.export_raw(self.finite_expand(expr, limits))

    def import_raw(self, literal: ScalarLiteralAstV1) -> OwnedExprV1:
        """Structural whitelist decoder. Closed numeric children use actual V2.

        Opaque input operator order is retained; it is not fresh normalization.
        Imported opaque negative powers remain read-only syntax. Numeric inverse
        limits and exact zero errors propagate, never become opaque fallback.
        """
        if not isinstance(literal, ScalarLiteralAstV1) or literal.canon != DENSITY_SOURCE_CONSTRUCTION_V1:
            raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "unknown wire canon/type")
        if type(literal.root) is not int or not 0 <= literal.root < len(literal.nodes):
            raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "invalid root")
        values = []
        for row in literal.nodes:
            if not isinstance(row, tuple) or not row or not isinstance(row[0], str):
                raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "malformed row")
            tag, *args = row
            if tag == "Q":
                if len(args) != 2 or any(type(x) is not int for x in args) or args[1] <= 0:
                    raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "invalid rational")
                rational = Fraction(*args)
                if (rational.numerator, rational.denominator) != tuple(args):
                    raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "non-reduced rational")
                values.append(self.rational(rational))
                continue
            arity = {"pi": 0, "sin": 1, "cos": 1, "atan2": 2, "Pow": 2}.get(tag)
            if tag not in ("Add", "Mul") and arity is None:
                raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, f"unsupported tag {tag}")
            if (arity is not None and len(args) != arity) or (tag in ("Add", "Mul") and len(args) < 2):
                raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "invalid arity")
            if any(type(child) is not int or not 0 <= child < len(values) for child in args):
                raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "forward/invalid child")
            children = tuple(values[child] for child in args)
            values.append(self._import_operator(tag, children))
        return values[literal.root]

    def _import_operator(self, tag: str, children: tuple[OwnedExprV1, ...]) -> OwnedExprV1:
        if tag in ("Add", "Mul"):
            if all(self.scalar(child) is not None for child in children):
                return self.add(*children) if tag == "Add" else self.mul(*children)
            return self._intern(tag, (child.node for child in children))
        if tag == "Pow":
            base, power = children
            exponent = self.whole_rational(power)
            if exponent is None or exponent.denominator not in (1, 2):
                raise DensityConstructProfileUnsupported(DensityConstructProfileCode.WIRE_PROFILE, "unsupported exponent")
            _power_exponent(exponent.numerator)
            if self.scalar(base) is not None and (exponent.denominator == 1 or self.whole_rational(base) is not None):
                return self.raw_pow(base, exponent)
            kind = "IntegerPow" if exponent.denominator == 1 else "OpaquePow"
            return self._intern(kind, (base.node,), exponent.numerator if kind == "IntegerPow" else exponent)
        return self._intern(tag, (child.node for child in children))

    def to_sympy_readonly(self, expr: OwnedExprV1):
        """Optional decision adapter; every non-rational constructor is raw.

        The returned object is for old readers/intervals or serialization only.
        It is never input to a new arithmetic operation, expand or cancel.
        """
        import sympy as sp

        values = []
        constructors = {"Add": sp.Add, "Mul": sp.Mul, "Pow": sp.Pow,
                        "sin": sp.sin, "cos": sp.cos, "atan2": sp.atan2}
        literal = self.export_raw(expr)
        for tag, *args in literal.nodes:
            if tag == "Q":
                values.append(sp.Rational(*args))
            elif tag == "pi":
                values.append(sp.pi)
            else:
                values.append(constructors[tag](*(values[child] for child in args), evaluate=False))
        return values[literal.root]


class _FiniteExpansionV1:
    def __init__(self, arena: DensityConstructArenaV1, limits: ExpandLimitsV1) -> None:
        self.arena, self.limits = arena, limits
        self.memo: dict[tuple[int, bool], tuple[OwnedExprV1, ...]] = {}

    def _term_count(self, count: int) -> None:
        if count > self.limits.max_terms:
            raise DensityConstructProfileUnsupported(DensityConstructProfileCode.EXPANSION_LIMIT,
                                                    "finite term choices exceed max_terms")

    def _rational_bits(self, expr: OwnedExprV1) -> None:
        rational = self.arena.whole_rational(expr)
        if rational is not None and max(abs(rational.numerator).bit_length(), rational.denominator.bit_length()) > self.limits.max_rational_power_bits:
            raise DensityConstructProfileUnsupported(DensityConstructProfileCode.EXPANSION_LIMIT,
                                                    "generated rational product exceeds max_rational_power_bits")

    def expand(self, expr: OwnedExprV1) -> OwnedExprV1:
        return self.arena.add(*self.choices(expr, False))

    def product(self, left: tuple[OwnedExprV1, ...], right: tuple[OwnedExprV1, ...]) -> tuple[OwnedExprV1, ...]:
        self._term_count(len(left) * len(right))
        terms = []
        for a in left:
            for b in right:
                term = self.arena.mul(a, b)
                self._rational_bits(term)
                terms.append(term)
        return tuple(terms)

    def choices(self, expr: OwnedExprV1, multiplicative: bool) -> tuple[OwnedExprV1, ...]:
        key = (expr.node, multiplicative)
        if key in self.memo:
            return self.memo[key]
        node = self.arena._check(expr)
        if node.kind == "Numeric":
            if multiplicative and len(node.payload.value.terms) > 1:
                result = tuple(self.arena.numeric(ScalarV2(RadicalSumV2((term,)))) for term in node.payload.value.terms)
            else:
                result = (expr,)
        elif node.kind == "Add":
            terms = []
            for child in node.children:
                choices = self.choices(self.arena._handle(child), False)
                self._term_count(len(terms)+len(choices))
                terms.extend(choices)
            result = tuple(terms)
        elif node.kind == "Mul":
            result = (self.arena.rational(1),)
            for child in node.children:
                result = self.product(result, self.choices(self.arena._handle(child), True))
        elif node.kind == "IntegerPow" and node.payload >= 0:
            base, exponent = self.arena._handle(node.children[0]), node.payload
            if exponent == 0:
                result = (self.arena.integer_power(base, 0),)
            elif exponent == 1:
                result = self.choices(base, False)
            else:
                if exponent > self.limits.max_polynomial_exponent:
                    raise DensityConstructProfileUnsupported(DensityConstructProfileCode.EXPANSION_LIMIT,
                                                            "polynomial exponent exceeds max_polynomial_exponent")
                choices = self.choices(base, True)
                result = (self.arena.rational(1),)
                for _ in range(exponent):
                    result = self.product(result, choices)
        else:
            # Неполиномиальные листья не перестраиваются после изменения детей.
            for child in node.children:
                old = self.arena._handle(child)
                expanded = self.expand(old)
                if self.arena.literal_tree(old) != self.arena.literal_tree(expanded):
                    raise DensityConstructProfileUnsupported(DensityConstructProfileCode.OPAQUE_CHILD_CHANGES,
                                                            "finite expansion would change an opaque leaf child")
            result = (expr,)
        self._term_count(len(result))
        self.memo[key] = result
        return result


@dataclass(frozen=True, slots=True)
class OwnedVec2V1:
    x: OwnedExprV1
    y: OwnedExprV1


def owned_dot(arena: DensityConstructArenaV1, gram: tuple[OwnedExprV1, OwnedExprV1, OwnedExprV1],
              left: OwnedVec2V1, right: OwnedVec2V1) -> OwnedExprV1:
    g00, g01, g11 = gram
    return arena.add(arena.mul(g00, left.x, right.x),
                     arena.mul(g01, arena.add(arena.mul(left.x, right.y), arena.mul(left.y, right.x))),
                     arena.mul(g11, left.y, right.y))


def owned_cross(arena: DensityConstructArenaV1, left: OwnedVec2V1,
                right: OwnedVec2V1, owner_sign: int) -> OwnedExprV1:
    if type(owner_sign) is not int or owner_sign not in (-1, 1):
        raise ValueError("owner_sign must be an admitted +/-1 source fact")
    return arena.scaled(arena.sub(arena.mul(left.x, right.y), arena.mul(left.y, right.x)), owner_sign)


def owned_rational_norm_unit(arena: DensityConstructArenaV1,
                             gram: tuple[OwnedExprV1, OwnedExprV1, OwnedExprV1],
                             vector: OwnedVec2V1) -> OwnedVec2V1:
    norm = owned_dot(arena, gram, vector, vector)
    if arena.whole_rational(norm) is None:
        raise DensityConstructProfileUnsupported(DensityConstructProfileCode.RATIONAL_NORM_REQUIRED,
                                                "whole norm must be rational in this first consumer")
    root = arena.sqrt(norm)
    inverse = arena.integer_power(root, -1)
    return OwnedVec2V1(arena.mul(vector.x, inverse), arena.mul(vector.y, inverse))


def owned_oriented_basis(arena: DensityConstructArenaV1,
                         gram: tuple[OwnedExprV1, OwnedExprV1, OwnedExprV1],
                         vector: OwnedVec2V1, owner_sign: int) -> tuple[OwnedVec2V1, OwnedVec2V1]:
    if type(owner_sign) is not int or owner_sign not in (-1, 1):
        raise ValueError("owner_sign must be an admitted +/-1 source fact")
    g00, g01, g11 = gram
    incoming = owned_rational_norm_unit(arena, gram, vector)
    normal = OwnedVec2V1(arena.scaled(arena.neg(arena.add(arena.mul(g01, vector.x), arena.mul(g11, vector.y))), owner_sign),
                         arena.scaled(arena.add(arena.mul(g00, vector.x), arena.mul(g01, vector.y)), owner_sign))
    return incoming, owned_rational_norm_unit(arena, gram, normal)


def owned_principal_linear_combination(arena: DensityConstructArenaV1, angle: OwnedExprV1,
                                       incoming: OwnedVec2V1, normal: OwnedVec2V1) -> OwnedVec2V1:
    """Closed owned ray construction; angle/admission remain caller facts.

    No trig identity, opaque-ray norm simplification, sign or value enclosure is
    inferred. Sin/Cos remain raw even for zero/rational/pi arguments.
    """
    cosine, sine = arena.raw_cos(angle), arena.raw_sin(angle)
    return OwnedVec2V1(arena.add(arena.mul(cosine, incoming.x), arena.mul(sine, normal.x)),
                       arena.add(arena.mul(cosine, incoming.y), arena.mul(sine, normal.y)))
