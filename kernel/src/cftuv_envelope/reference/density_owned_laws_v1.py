"""Owned leaf laws for the shared Density controller CANDIDATE.

Arithmetic stays in one explicit arena. Literal ordered iv160/finite quadratic
admission and A2 mathematical value enclosures are separate gateways. This is
not a product dispatch, generic trig simplifier, or inverse-profile bypass.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from decimal import Decimal, localcontext
from fractions import Fraction
from types import SimpleNamespace

from .density_angle_v1 import AngleAtomV1
from .density_angle_value_v1 import AngleValueRegistryV1
from .density_attempt_memo_v1 import DensityAttemptMemoV1, DensityAttemptMemoSnapshotV1
from .density_construct_v1 import DensityConstructArenaV1, OwnedExprV1, _NonzeroWitnessV1
from .density_oracle_v1 import DensityOracleLimitsV1
from .density_scalar_v2 import ScalarV2
from .density_shell_v2 import _shell, _outcome_bundle
from .density_trig_v1 import owned_exact_q_trig, owned_principal_ray


@dataclass(frozen=True, slots=True)
class OwnedScalarExprV1:
    """Bound operator facade; only int/Fraction or same-laws operands allowed."""

    laws: OwnedDensityLawsV1
    value: OwnedExprV1

    def _other(self, value):
        return self.laws.scalar(value).value

    def __add__(self, other):
        return self.laws.wrap(self.laws.arena.add(self.value, self._other(other)))

    __radd__ = __add__

    def __mul__(self, other):
        return self.laws.wrap(self.laws.arena.mul(self.value, self._other(other)))

    __rmul__ = __mul__

    def __sub__(self, other):
        return self.laws.wrap(self.laws.arena.sub(self.value, self._other(other)))

    def __rsub__(self, other):
        return self.laws.scalar(other).__sub__(self)

    def __neg__(self):
        return self.laws.wrap(self.laws.arena.neg(self.value))

    def __pow__(self, exponent):
        return self.laws.wrap(self.laws.arena.raw_pow(self.value, exponent))

    def __truediv__(self, other):
        denominator = self.laws.scalar(other)
        return self.laws.divide_nonzero(self, denominator)

    def __rtruediv__(self, other):
        return self.laws.scalar(other).__truediv__(self)


@dataclass(frozen=True, slots=True)
class OwnedDensityVectorV1:
    x: OwnedScalarExprV1
    y: OwnedScalarExprV1

    def expressions(self, transaction_memo=None):
        return self.x, self.y


@dataclass(slots=True)
class OwnedDensityMetricV1:
    gram: tuple
    inverse_gram: tuple
    owner_orientation_sign: int
    metric_id: str | None = None
    _density_exact_memo: object = field(default_factory=lambda: SimpleNamespace(
        dual_dots={}, crosses={}, signs={}, intervals={}, subturns={},
        covector_rows={}, fan_bindings={},
    ))

    def density_expressions(self, vector):
        return vector.expressions()


class OwnedDensityLawsV1:
    def __init__(self, arena: DensityConstructArenaV1, limits: DensityOracleLimitsV1,
                 *, attempt_snapshot=DensityAttemptMemoSnapshotV1()):
        if not isinstance(arena, DensityConstructArenaV1) or not isinstance(limits, DensityOracleLimitsV1):
            raise TypeError("Owned laws require an explicit arena and profile")
        self.arena, self.limits = arena, limits
        self.values = AngleValueRegistryV1(arena)
        self.preparation_cap, self.search_cap, self.box_cap = limits.preparation_cap, limits.search_cap, limits.box_cap
        self.budgets, self.shell_outcomes = [], []
        self.sign_memo, self.literal_intervals = {}, {}
        self._raw_metrics = {}
        # Память успешных вееров общая для математически равных metric wrappers.
        self.attempt_memo = DensityAttemptMemoV1(attempt_snapshot)

    def wrap(self, expression):
        self.arena.owned_form(expression)
        return OwnedScalarExprV1(self, expression)

    def scalar(self, value):
        if isinstance(value, OwnedScalarExprV1):
            if value.laws is not self:
                raise ValueError("DENSITY_OWNED_LAWS_V1_FOREIGN_TRANSACTION")
            return value
        if isinstance(value, OwnedExprV1):
            return self.wrap(value)
        if type(value) is int or isinstance(value, Fraction):
            return self.wrap(self.arena.rational(value))
        raise TypeError("No symbolic AST or evaluated scalar may enter owned arithmetic")

    def metric(self, gram, inverse, owner_sign):
        if type(owner_sign) is not int or owner_sign not in (-1, 1):
            raise ValueError("Owner orientation must be +/-1")
        def matrix(values):
            a, b, c = (self.scalar(value) for value in values)
            return ((a, b), (b, c))
        return OwnedDensityMetricV1(matrix(gram), matrix(inverse), owner_sign)

    def vector(self, x, y, metric=None):
        return OwnedDensityVectorV1(self.scalar(x), self.scalar(y))

    def dot_g(self, metric, left, right):
        lx, ly = left.expressions()
        rx, ry = right.expressions()
        a, b = metric.gram
        return lx*(a[0]*rx+a[1]*ry)+ly*(b[0]*rx+b[1]*ry)

    def unit_g(self, metric, vector):
        squared = self.dot_g(metric, vector, vector)
        self.fraction(squared)
        root = self.sqrt(squared)
        return self.vector(vector.x/root, vector.y/root)

    def covector(self, metric, normal):
        x, y = normal.expressions()
        a, b = metric.gram
        return self.vector(a[0]*x+a[1]*y, b[0]*x+b[1]*y)

    def primal(self, metric, covector):
        x, y = covector.expressions()
        a, b = metric.inverse_gram
        return self.vector(a[0]*x+a[1]*y, b[0]*x+b[1]*y)

    def _owner_normal_g(self, metric, tangent, owner_left):
        x, y = self.covector(metric, tangent).expressions()
        side = metric.owner_orientation_sign*(1 if owner_left else -1)
        return self.vector(-side*y, side*x)

    def _raw_incident_normal(self, metric, incident, use_direction_declarations):
        from .common import ReferenceGeometryError
        from .contracts import ReferenceOutcome

        def invalid(detail):
            raise ReferenceGeometryError(ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED, detail)
        tangent = self.unit_g(metric, self.vector(incident.end.x-incident.start.x, incident.end.y-incident.start.y))
        candidates = []
        if use_direction_declarations:
            for declaration in dict(incident.source_direction_declaration).get("records", ()):
                record = dict(declaration)
                if record["kind"] == "CERTIFIED_AFFINE_V2" and record["reference_metric_id"] != metric.metric_id:
                    invalid("affine support direction references another metric")
                if record["kind"] in ("CERTIFIED_PLANAR_V1", "CERTIFIED_AFFINE_V2"):
                    candidates.append(self.vector(Fraction(*record["x"]), Fraction(*record["y"])))
        analysis = self.unit_g(metric, candidates[0]) if candidates else None
        for candidate in candidates[1:]:
            normal = self.unit_g(metric, candidate)
            if self.sign(self.oriented_cross(metric, analysis, normal)) != 0 or self.sign(self.dot_g(metric, analysis, normal)) <= 0:
                invalid(f"conflicting certified support directions for {incident.chain_use_id}")
        cycle_normals = []
        for cycle, ordinal in zip(incident.owner_face_cycles, incident.owner_face_edge_ordinals):
            twice_area = self.scalar(0)
            for start, end in zip(cycle, cycle[1:]+cycle[:1]):
                twice_area = twice_area+self.scalar(start.x)*end.y-self.scalar(start.y)*end.x
            cycle_area_sign = self.sign(twice_area/2)
            start, end = cycle[ordinal], cycle[(ordinal+1) % len(cycle)]
            edge = self.vector(end.x-start.x, end.y-start.y)
            same_direction = self.sign(self.dot_g(metric, tangent, edge)) > 0
            coordinate_left = (cycle_area_sign > 0) == same_direction
            owner_left = coordinate_left == (metric.owner_orientation_sign > 0)
            cycle_normals.append(self._owner_normal_g(metric, tangent, owner_left))
        face = None
        if cycle_normals:
            first = cycle_normals[0]
            if all(self.sign(self.oriented_cross(metric, first, item)) == 0
                   and self.sign(self.dot_g(metric, first, item)) > 0 for item in cycle_normals[1:]):
                face = first
        if face is None:
            left = incident.owner_left
            if left is None and incident.chain_use_orientation in ("A_START_TO_END", "B_START_TO_END"):
                left = True
            if left is not None:
                face = self._owner_normal_g(metric, tangent, left)
        normal = None
        if analysis is not None:
            if self.sign(self.dot_g(metric, analysis, tangent)) == 0:
                normal = analysis
            elif self.sign(self.oriented_cross(metric, analysis, tangent)) == 0:
                normal = face
            else:
                invalid(f"certified support direction is neither tangent nor normal for {incident.physical_edge_id}")
        normal = face if normal is None else normal
        if normal is None:
            invalid(f"owner-interior normal is ambiguous for {incident.physical_edge_id}")
        return self.unit_g(metric, normal)

    def raw_geometry(self, raw, *, use_direction_declarations):
        """First genuine rawfact producer: normals derived inside owned math."""
        from .density_oracle_v1 import RawDensityGeometryV1

        if not isinstance(raw, RawDensityGeometryV1):
            raise TypeError("Owned raw geometry requires a validated declaration record")
        g00, g01, g11 = raw.metric.gram
        i00, i01, i11 = raw.metric.inverse_gram
        if (g00 <= 0 or g00*g11-g01*g01 <= 0 or g00*i00+g01*i01 != 1
                or g00*i01+g01*i11 != 0 or g01*i00+g11*i01 != 0 or g01*i01+g11*i11 != 1):
            raise ValueError("DENSITY_ORACLE_V1_INVALID_GRAM_FACTS")
        key = raw.metric, use_direction_declarations
        metric = self._raw_metrics.get(key)
        if metric is None:
            metric = self.metric(raw.metric.gram, raw.metric.inverse_gram, raw.metric.owner_orientation_sign)
            metric.metric_id = raw.metric.metric_id
            self._raw_metrics[key] = metric
        incoming = self._raw_incident_normal(metric, raw.incoming, use_direction_declarations)
        outgoing = self._raw_incident_normal(metric, raw.outgoing, use_direction_declarations)
        return metric, incoming, outgoing

    def fraction(self, expression):
        scalar = self.scalar(expression)
        value = self.arena.whole_rational(scalar.value)
        if value is None:
            raise ValueError("DENSITY_OWNED_LAWS_V1_RATIONAL_REQUIRED")
        return value

    def preparation_budget(self, factory):
        budget = factory(self.preparation_cap)
        self.budgets.append(("PREPARATION", budget))
        return budget

    def search_budget(self, factory):
        budget = factory(self.search_cap)
        self.budgets.append(("SEARCH", budget))
        return budget

    def sign(self, expression, metric=None):
        from mpmath import iv
        from .._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
        from .common import ReferenceGeometryError
        from .contracts import ReferenceOutcome
        from .planar_types import ExactQuadraticFieldUnsupported, exact_quadratic_value

        expression = self.scalar(expression)
        rational = self.arena.whole_rational(expression.value)
        if rational is not None:
            return (rational > 0)-(rational < 0)
        key = expression.value.node
        if key in self.sign_memo:
            return self.sign_memo[key]
        raw = self.arena.to_sympy_readonly(expression.value)
        saved = iv.prec
        iv.prec = 160
        try:
            try:
                box = density_interval_enclosure(raw, self.literal_intervals)
            except (DensityIntervalEnclosureUnsupported, ArithmeticError, TypeError, ValueError):
                box = None
            result = 1 if box is not None and box.a > 0 else -1 if box is not None and box.b < 0 else None
        finally:
            iv.prec = saved
        if result is None:
            expanded = self.arena.finite_expand(expression.value, self.limits.finite_expand)
            try:
                result = exact_quadratic_value(self.arena.to_sympy_readonly(expanded)).sign()
            except ExactQuadraticFieldUnsupported:
                raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE,
                    "Density A exact sign is not certified without generic factorization") from None
        self.sign_memo[key] = result
        return result

    def divide_nonzero(self, numerator, denominator):
        numerator, denominator = self.scalar(numerator), self.scalar(denominator)
        sign = self.sign(denominator)
        if sign == 0:
            raise ZeroDivisionError("Owned certified denominator is exactly zero")
        if self.arena.scalar(denominator.value) is not None:
            # Numeric inverse BASE first: даже нулевой числитель не обходит cap.
            return self.wrap(self.arena.partial_div(numerator.value, denominator.value))
        # Нулевой числитель допустим только ПОСЛЕ старого nonzero-предиката;
        # обычный mul всё ещё охраняет унаследованные partial descendants.
        reciprocal = self.arena._reciprocal_with_nonzero_witness(
            denominator.value, _NonzeroWitnessV1(denominator.value, sign))
        return self.wrap(self.arena.mul(numerator.value, reciprocal))

    def dual_dot(self, metric, left, right):
        lx, ly = metric.density_expressions(left)
        rx, ry = metric.density_expressions(right)
        key = (lx, ly, rx, ry)
        memo = metric._density_exact_memo.dual_dots
        if key not in memo:
            a, b = metric.inverse_gram
            memo[key] = lx*(a[0]*rx+a[1]*ry) + ly*(b[0]*rx+b[1]*ry)
            memo[(rx, ry, lx, ly)] = memo[key]
        return memo[key]

    def oriented_cross(self, metric, left, right):
        lx, ly = metric.density_expressions(left)
        rx, ry = metric.density_expressions(right)
        key = (lx, ly, rx, ry)
        memo = metric._density_exact_memo.crosses
        if key not in memo:
            memo[key] = metric.owner_orientation_sign*(lx*ry-ly*rx)
            memo[(rx, ry, lx, ly)] = -memo[key]
        return memo[key]

    def sqrt(self, value):
        return self.wrap(self.arena.sqrt(self.scalar(value).value))

    def quarter_turn(self, metric, vector, orientation):
        x, y = metric.density_expressions(vector)
        a, b = metric.inverse_gram
        scale = self.scalar(orientation*metric.owner_orientation_sign)/self.sqrt(a[0]*b[1]-a[1]*b[0])
        return self.vector(-scale*(b[0]*x+b[1]*y), scale*(a[0]*x+a[1]*y))

    def exact_q_trig(self, q):
        return tuple(map(self.wrap, owned_exact_q_trig(self.arena, q)))

    def rotate_q(self, metric, vector, orientation, q):
        c, s = self.exact_q_trig(q)
        perpendicular = self.quarter_turn(metric, vector, orientation)
        x, y = metric.density_expressions(vector)
        return self.vector(c*x+s*perpendicular.x, c*y+s*perpendicular.y)

    def subturn_boundary_vectors(self, metric, ideal, ordinal, orientation, q):
        from .density_projective import _expected_orientation
        sign = _expected_orientation(orientation)
        return (self.rotate_q(metric, ideal[ordinal-1], sign, q),
                self.rotate_q(metric, ideal[ordinal+1], -sign, q))

    def residual_sign(self, metric, dot, left, right, q):
        from .density_residual import _filtered_sign
        views = [self.arena.to_sympy_readonly(self.scalar(value).value) for value in (dot, left, right)]
        raw_metric = SimpleNamespace(_density_exact_memo=SimpleNamespace(intervals=self.literal_intervals))
        filtered = _filtered_sign(raw_metric, *views, q)
        if filtered is not None:
            return filtered
        norms, square = left*right, dot*dot
        if q == 5:
            return self.sign(8*square-(3+self.sqrt(5))*norms)
        return self.sign({3: 4, 4: 2, 6: 4}[q]*square-{3: 1, 4: 1, 6: 3}[q]*norms)

    def subturn(self, metric, left, right, q):
        left_values, right_values = left.expressions(), right.expressions()
        key, reverse = (*left_values, *right_values, q), (*right_values, *left_values, q)
        memo = metric._density_exact_memo.subturns
        if key in memo:
            return memo[key]
        dot = self.dual_dot(metric, left, right)
        sign = self.sign(dot)
        if q == 2:
            result = sign >= 0
        elif sign <= 0:
            result = False
        else:
            result = self.residual_sign(metric, dot, self.dual_dot(metric, left, left), self.dual_dot(metric, right, right), q) >= 0
        memo[key] = memo[reverse] = result
        return result

    def subturn_boundary(self, metric, left, right, q):
        dot = self.dual_dot(metric, left, right)
        sign = self.sign(dot)
        if sign < 0:
            return False
        norm_left, norm_right = self.dual_dot(metric, left, left), self.dual_dot(metric, right, right)
        if q == 2:
            return self.sign(dot) == 0
        return self.residual_sign(metric, dot, norm_left, norm_right, q) == 0

    def inside_ordinal(self, metric, candidate, previous, ideal, following):
        px, py = metric.density_expressions(previous)
        ix, iy = metric.density_expressions(ideal)
        fx, fy = metric.density_expressions(following)
        return (self.sign(self.dual_dot(metric, candidate, self.vector(ix-px, iy-py))) > 0
                and self.sign(self.dual_dot(metric, candidate, self.vector(ix-fx, iy-fy))) > 0)

    def strict_between(self, lower, value, upper, metric=None):
        return (self.sign(value-lower, metric) > 0
                and self.sign(upper-value, metric) > 0)

    def literal_unit(self, value):
        rational = self.arena.whole_rational(self.scalar(value).value)
        return int(rational) if rational in (-1, 1) else None

    def slope(self, vector, use_x, metric=None):
        x, y = vector.expressions()
        denominator = x if use_x else y
        if self.sign(denominator) == 0:
            from .density_fan_types import AdaptiveDensityFanInvalid
            raise AdaptiveDensityFanInvalid("fan chart denominator is zero")
        return self.divide_nonzero(y if use_x else x, denominator)

    def chart_for_window(self, metric, ideal, left, right):
        from .density_fan_types import DensityWindowChartUnrepresentable
        ix, iy = ideal.expressions()
        preferred = self.sign(ix*ix-iy*iy) >= 0
        for use_x in (preferred, not preferred):
            signs = tuple(self.sign(vector.expressions()[0 if use_x else 1]) for vector in (left, ideal, right))
            if 0 not in signs and len(set(signs)) == 1:
                return use_x, signs[0]
        raise DensityWindowChartUnrepresentable("DENSITY_WINDOW_CHART_UNREPRESENTABLE")

    def window_neighbours(self, ideal, ordinal):
        from .adaptive_density_band import WINDOW_LAW_NARROW_BAND, ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT
        if ideal.window_law != WINDOW_LAW_NARROW_BAND:
            return ideal[ordinal-1], ideal[ordinal+1]
        if ordinal not in ideal.band_cache:
            center, metric = ideal[ordinal], ideal.metric
            t = ADAPTIVE_FAN_NARROW_BAND_HALF_TANGENT
            c, s = (1-t*t)/(1+t*t), 2*t/(1+t*t)
            results = []
            for neighbour, sign in ((ideal[ordinal-1], -1), (ideal[ordinal+1], 1)):
                dot = self.dual_dot(metric, center, neighbour)
                norms = self.dual_dot(metric, center, center)*self.dual_dot(metric, neighbour, neighbour)
                if self.sign(dot) > 0 and self.sign(dot*dot-c*c*norms) >= 0:
                    results.append(neighbour)
                else:
                    perpendicular = self.quarter_turn(metric, center, ideal.band_orientation)
                    results.append(self.vector(c*center.x+sign*s*perpendicular.x, c*center.y+sign*s*perpendicular.y))
            ideal.band_cache[ordinal] = tuple(results)
        return ideal.band_cache[ordinal]

    def window_triple(self, ideal, ordinal):
        previous, following = self.window_neighbours(ideal, ordinal)
        return previous, ideal[ordinal], following

    def principal_pair(self, ratio: Fraction, dot_sign: int, n: int, ordinal: int):
        pair = owned_principal_ray(self.arena, ratio, dot_sign, n, ordinal)
        if pair is None:
            cosine = self.arena.scaled(self.arena.sqrt(self.arena.rational(ratio)), dot_sign)
            sine = self.arena.sqrt(self.arena.rational(1-ratio))
            if ordinal == 0:
                pair = self.arena.rational(1), self.arena.rational(0)
            elif ordinal == n:
                pair = cosine, sine
            else:
                angle = self.arena.scaled(self.arena.raw_atan2(sine, cosine), Fraction(ordinal, n))
                pair = self.arena.raw_cos(angle), self.arena.raw_sin(angle)
        cosine, sine = pair
        forms = [self.arena.owned_form(component) for component in (cosine, sine)]
        if all(form.kind in ("Numeric", kind) for form, kind in zip(forms, ("cos", "sin"))):
            if any(form.kind != "Numeric" for form in forms) and 0 < ordinal < n:
                self.values.register_ray(cosine, sine, AngleAtomV1(n, ratio, dot_sign), ordinal)
        return self.wrap(cosine), self.wrap(sine)

    def unit_covector(self, metric, vector):
        norm = self.dual_dot(metric, vector, vector)
        rational = self.fraction(norm)
        if rational <= 0:
            raise ZeroDivisionError("Owned covector norm is not positive")
        root = self.sqrt(norm)
        return self.vector(vector.x/root, vector.y/root)

    def turn_witness(self, metric, left, right, turn_sign):
        from .common import ReferenceGeometryError
        from .contracts import ReferenceOutcome

        if self.sign(self.oriented_cross(metric, left, right)) != turn_sign:
            raise ReferenceGeometryError(ReferenceOutcome.PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED,
                                         "ordered support normals do not realize the certified owner-sector turn")
        dot = self.dual_dot(metric, left, right)
        norms = self.dual_dot(metric, left, left)*self.dual_dot(metric, right, right)
        return self.sign(dot), self.fraction(dot*dot/norms)

    def turn_count_comparison(self, cosine_squared, count, q):
        steps = Fraction(count+1, q)
        if steps >= 1:
            return None
        if steps.denominator == 5:
            threshold = (3+(1 if steps.numerator in (1, 4) else -1)*self.sqrt(5))/8
        else:
            threshold = {Fraction(1, 6): Fraction(3, 4), Fraction(5, 6): Fraction(3, 4),
                         Fraction(1, 4): Fraction(1, 2), Fraction(3, 4): Fraction(1, 2),
                         Fraction(1, 3): Fraction(1, 4), Fraction(2, 3): Fraction(1, 4),
                         Fraction(1, 2): Fraction(0)}[steps]
        return self.sign(self.scalar(cosine_squared)-threshold)

    def verify_raw_selection(self, raw, canonical):
        selection = raw.selection
        if (selection.selection_policy_id != "HUBER_EMANATED_COUNT_DENSITY_A_V1"
                or selection.max_subturn_value_id != f"LINEAR_REFLEX_DENSITY_{selection.q-2}_V1"):
            raise ValueError("DENSITY_RAW_SELECTION_POLICY_MISMATCH")
        lower, upper = raw.canonical.source_reflex_interval
        lower_open = dict(dict(raw.canonical.source_reflex_declaration).get("reflex_excess_over_pi", ())).get("lower_kind") == "OPEN"
        if canonical is not None:
            lower = upper = canonical
            lower_open = False
        bucket = None
        for candidate in range(1, selection.q+1):
            comparison = self.sign(self.scalar(lower-Fraction(candidate-1, selection.q)))
            if ((comparison > 0 or comparison == 0 and lower_open)
                    and self.sign(self.scalar(upper-Fraction(candidate, selection.q))) <= 0):
                bucket = candidate
                break
        if bucket != selection.bucket_c:
            raise ValueError("DENSITY_RAW_SELECTION_BUCKET_UNPROVEN")

    def turn_count_feasible(self, sign, cosine_squared, count, q):
        steps = Fraction(count+1, q)
        if steps >= 1:
            return True
        if steps == Fraction(1, 2):
            return sign >= 0
        comparison = self.turn_count_comparison(cosine_squared, count, q)
        return sign > 0 and comparison >= 0 if steps < Fraction(1, 2) else sign >= 0 or comparison <= 0

    def turn_count_exact_limit(self, sign, cosine_squared, count, q):
        steps = count+1
        return (steps < q and self.turn_count_comparison(cosine_squared, count, q) == 0
                and sign == (1 if 2*steps < q else -1 if 2*steps > q else 0))

    def rational_direction(self, vector):
        """Primitive covector or a three-state exact radical-field decision."""
        from math import isqrt

        x, y = (self.arena.scalar(component.value) for component in vector.expressions())
        if x is None or y is None:
            return None, None
        if x.value.is_zero:
            if y.value.is_zero:
                return None, False
            return (0, self.sign(vector.y)), True
        if y.value.is_zero:
            return (self.sign(vector.x), 0), True
        rad_x, coeff_x = x.value.pairs()[0]
        ratio = None
        for rad_y, coeff_y in y.value.pairs():
            root = isqrt(rad_x*rad_y)
            if root*root == rad_x*rad_y:
                candidate = coeff_y/coeff_x*Fraction(root, rad_x)
                if y.sub(x.scaled(candidate)).value.is_zero:
                    ratio = candidate
                break
        if ratio is None:
            return None, False
        sign = self.sign(vector.x)
        return (sign*ratio.denominator, sign*ratio.numerator), True

    def canonical_fan(self, metric, incoming, outgoing, turn_sign, count, canonical):
        step = Fraction(canonical, count+1)
        if step.numerator != 1 or step.denominator not in (2, 3, 4, 5, 6):
            from .common import ReferenceGeometryError
            from .contracts import ReferenceOutcome
            raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CANONICAL_ANGLE_RESTORATION_INVALID,
                                         f"canonical subturn is not a single exact rotation: {step} of pi")
        left, right = self.unit_covector(metric, incoming), self.unit_covector(metric, outgoing)
        self.turn_witness(metric, left, right, turn_sign)
        rays = []
        for _ in range(count):
            left = self.rotate_q(metric, left, turn_sign, step.denominator)
            rays.append(left)
        return self.unit_covector(metric, incoming), *rays, right

    def rotation_fan(self, metric, incoming, outgoing, turn_sign, rotations):
        left, right = self.unit_covector(metric, incoming), self.unit_covector(metric, outgoing)
        self.turn_witness(metric, left, right, turn_sign)
        perpendicular = self.quarter_turn(metric, left, turn_sign)
        rays = []
        for a, b in rotations:
            norm = self.sqrt(a*a+b*b)
            rays.append(self.vector((a*left.x+b*perpendicular.x)/norm,
                                    (a*left.y+b*perpendicular.y)/norm))
        return left, *rays, right

    def principal_fan(self, metric, incoming, outgoing, turn_sign, hidden_count):
        """Fresh numeric endpoint normalization and complete owned ray rows.

        This consumes raw covector coordinates, not old constructed ideal ASTs.
        Turn sign is independent of the metric owner orientation; atan2 uses
        positive principal sine while the perpendicular basis carries turn.
        """
        if type(turn_sign) is not int or turn_sign not in (-1, 1):
            raise ValueError("Turn orientation must be +/-1")
        if type(hidden_count) is not int or hidden_count < 0:
            raise ValueError("Hidden count must be a nonnegative integer")
        left, right = self.unit_covector(metric, incoming), self.unit_covector(metric, outgoing)
        self.turn_witness(metric, left, right, turn_sign)
        if hidden_count == 0:
            return left, right
        dot = self.dual_dot(metric, left, right)
        ratio = self.fraction(dot*dot)
        dot_sign = self.sign(dot)
        normal = self.quarter_turn(metric, left, turn_sign)
        rays = []
        for ordinal in range(1, hidden_count+1):
            c, s = self.principal_pair(ratio, dot_sign, hidden_count+1, ordinal)
            rays.append(self.vector(c*left.x+s*normal.x, c*left.y+s*normal.y))
        return left, *rays, right

    def envelope(self, expression, metric=None):
        from mpmath import iv
        from .._density_policy import DensityIntervalEnclosureUnsupported, density_interval_enclosure
        from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
        from .density_binding_types import DirectionBindingCertificateUnproven, BINDING_INSIDE_OWN_ORDINAL_WINDOW
        expression = self.scalar(expression)
        saved = iv.prec
        iv.prec = 160
        try:
            # Admission первым: A2 enclosure не исправляет отказ literal iv160.
            density_interval_enclosure(self.arena.to_sympy_readonly(expression.value), self.literal_intervals)
        except (DensityIntervalEnclosureUnsupported, ArithmeticError, ValueError, TypeError):
            raise DirectionBindingCertificateUnproven(BINDING_INSIDE_OWN_ORDINAL_WINDOW) from None
        finally:
            iv.prec = saved
        bounds = self.values.enclosure(expression.value)
        shell = _shell((bounds.lower, bounds.upper))
        numeric = self.arena.scalar(expression.value)
        if numeric is not None:
            self.shell_outcomes.append((numeric.value.wire(), _outcome_bundle(numeric.value)))
        return CertifiedDecimalIntervalV1(shell.lower, shell.upper, IntervalEndpointKind.CLOSED,
                                          IntervalEndpointKind.CLOSED, shell.absolute_error_bound)

    @staticmethod
    def width(lower: Decimal, upper: Decimal):
        with localcontext() as context:
            context.prec = 28
            return upper-lower
