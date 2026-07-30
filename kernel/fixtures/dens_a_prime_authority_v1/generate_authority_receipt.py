"""Генератор proof-only расписки DENS-A′: конечная власть направлений."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import time
from fractions import Fraction
from math import gcd, isqrt
from pathlib import Path

import sympy as sp
from mpmath import iv, libmp

from cftuv_envelope.reference.planar_types import interval_enclosure


BASE_SHA = "b5b97e43ee7ca4d30a540f42ff36cf01852c2911"
PRODUCT_OIDS = {
    "cftuv": "01f510088b50c425570cc2e42c13e400ca4875f3",
    "kernel_src": "ed33d3f43635a5acbe5b8695dfed9b2e12396c6b",
    "tools": "ec5439a5fc8ea46784d4621ac763f941b4833c36",
}
METRIC_SOURCES = (
    {
        "metric_id": "FIELD_WEIGHTED",
        "path": "kernel/fixtures/building_002_weighted_normals_v1/analysis_snapshot.json",
        "sha256": "e5bbf6b3210ee195c227b4f09ac776418dd1f723592cefdbefe1acfd844693a3",
        "reference_metric_id": "reference-metric:ac669896fbe12ebd1c3714f4",
        "patch_domain_id": "host-v0:patch-domain:e6497c2fff9e1e4d3ac2806c",
    },
    {
        "metric_id": "FIELD_FULL",
        "path": "kernel/fixtures/building_002_full_selection_v1/analysis_snapshot.json",
        "sha256": "5c759cfc80548072eb3918f5fdc114d6f93ad077b70a5f0c663f10ede2f41d16",
        "reference_metric_id": "reference-metric:749dbc5d924dc816c848e90a",
        "patch_domain_id": "host-v0:patch-domain:bf66337616136188f314fb69",
    },
)
DECIMAL_SCALE = 10**27
REAL_Q = (2, 3, 4, 6)


def _canonical_bytes(value: object) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n"
    ).encode("utf-8")


def _git(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        ("git", "-C", str(repo), *args), text=True
    ).strip()


def _fraction_record(value: Fraction | int | sp.Rational) -> list[int]:
    value = Fraction(value)
    return [value.numerator, value.denominator]


def _fraction(value: list[int]) -> Fraction:
    return Fraction(value[0], value[1])


def _matrix_record(matrix) -> list[list[list[int]]]:
    return [
        [_fraction_record(matrix[row][column]) for column in range(2)]
        for row in range(2)
    ]


def _matrix_from_record(record):
    return tuple(
        tuple(sp.Rational(*record[row][column]) for column in range(2))
        for row in range(2)
    )


def _exact_rational(record) -> Fraction:
    return Fraction(record["numerator"], record["denominator"])


def _read_metric(repo: Path, source: dict) -> dict:
    path = repo / source["path"]
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != source["sha256"]:
        raise RuntimeError(f"UNKNOWN_METRIC_FIXTURE:{source['metric_id']}")
    payload = json.loads(raw)
    matches = [
        item
        for item in payload["surface_metric_descriptors"]
        if item["reference_metric_id"]["value"] == source["reference_metric_id"]
        and item["patch_domain_id"]["value"] == source["patch_domain_id"]
    ]
    if len(matches) != 1:
        raise RuntimeError(f"UNKNOWN_METRIC_DESCRIPTOR:{source['metric_id']}")
    descriptor = matches[0]
    gram_raw = descriptor["exact_gram_matrix"]
    inverse_raw = descriptor["exact_inverse_gram_matrix"]
    gram = (
        (
            _exact_rational(gram_raw["m00"]),
            _exact_rational(gram_raw["m01"]),
        ),
        (
            _exact_rational(gram_raw["m10"]),
            _exact_rational(gram_raw["m11"]),
        ),
    )
    inverse = (
        (
            _exact_rational(inverse_raw["m00"]),
            _exact_rational(inverse_raw["m01"]),
        ),
        (
            _exact_rational(inverse_raw["m10"]),
            _exact_rational(inverse_raw["m11"]),
        ),
    )
    return {
        **source,
        "blob_oid": _git(repo, "rev-parse", f"HEAD:{source['path']}"),
        "gram": _matrix_record(gram),
        "inverse_gram": _matrix_record(inverse),
    }


def _mirror_metric(metric: dict) -> dict:
    def mirror(record):
        result = json.loads(json.dumps(record))
        result[0][1][0] *= -1
        result[1][0][0] *= -1
        return result

    return {
        **metric,
        "metric_id": f"{metric['metric_id']}_MIRROR",
        "mirror_of": metric["metric_id"],
        "gram": mirror(metric["gram"]),
        "inverse_gram": mirror(metric["inverse_gram"]),
    }


def _dot(matrix, left, right):
    return sp.simplify(
        left[0] * (matrix[0][0] * right[0] + matrix[0][1] * right[1])
        + left[1] * (matrix[1][0] * right[0] + matrix[1][1] * right[1])
    )


def _unit(matrix, vector):
    length = sp.sqrt(_dot(matrix, vector, vector))
    return (
        sp.simplify(vector[0] / length),
        sp.simplify(vector[1] / length),
    )


def _sign(value) -> int:
    value = sp.simplify(value)
    if value == 0 or value.is_zero is True:
        return 0
    if value.is_positive is True:
        return 1
    if value.is_negative is True:
        return -1
    numeric = sp.N(value, 100)
    if numeric > 0:
        return 1
    if numeric < 0:
        return -1
    raise RuntimeError(f"UNDECIDABLE_SIGN:{value}")


def _perpendicular(matrix, vector, orientation: int):
    a, b = matrix[0]
    _, c = matrix[1]
    candidate = (
        sp.simplify(-(b * vector[0] + c * vector[1])),
        sp.simplify(a * vector[0] + b * vector[1]),
    )
    if _sign(vector[0] * candidate[1] - vector[1] * candidate[0]) != orientation:
        candidate = (-candidate[0], -candidate[1])
    return _unit(matrix, candidate)


def _ideal_sequence(matrix, start, end, delta, count: int, orientation: int):
    start_unit = _unit(matrix, start)
    if sp.simplify(delta - sp.pi) == 0:
        perpendicular = _perpendicular(matrix, start_unit, orientation)
    else:
        end_unit = _unit(matrix, end)
        cosine = sp.cos(delta)
        sine = sp.sin(delta)
        perpendicular = (
            sp.simplify((end_unit[0] - cosine * start_unit[0]) / sine),
            sp.simplify((end_unit[1] - cosine * start_unit[1]) / sine),
        )
        if _sign(
            start_unit[0] * perpendicular[1]
            - start_unit[1] * perpendicular[0]
        ) != orientation:
            raise RuntimeError("ORIENTATION_CONSTRUCTION_MISMATCH")
    result = []
    for ordinal in range(count + 1):
        angle = sp.simplify(delta * ordinal / count)
        result.append(
            (
                sp.simplify(
                    sp.cos(angle) * start_unit[0]
                    + sp.sin(angle) * perpendicular[0]
                ),
                sp.simplify(
                    sp.cos(angle) * start_unit[1]
                    + sp.sin(angle) * perpendicular[1]
                ),
            )
        )
    return tuple(result)


def _envelope(expression) -> tuple[Fraction, Fraction]:
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = interval_enclosure(sp.simplify(expression))
    finally:
        iv.prec = saved
    lower, upper = (
        Fraction(*libmp.to_rational(item)) for item in enclosure._mpi_
    )
    lower_units = lower.numerator * DECIMAL_SCALE // lower.denominator
    upper_units = -((-upper.numerator * DECIMAL_SCALE) // upper.denominator)
    return (
        Fraction(lower_units, DECIMAL_SCALE),
        Fraction(upper_units, DECIMAL_SCALE),
    )


def _authority_budget(width: Fraction) -> int:
    if width <= 0:
        raise RuntimeError("NON_POSITIVE_CERTIFIED_WIDTH")
    estimate = (width.denominator + width.numerator - 1) // width.numerator
    budget = isqrt(estimate)
    while Fraction(budget * budget) * width < 1:
        budget += 1
    while budget > 1 and Fraction((budget - 1) ** 2) * width >= 1:
        budget -= 1
    return budget


def _fractions_in_open_window(
    lower: Fraction, upper: Fraction, budget: int
) -> list[Fraction]:
    result: set[Fraction] = set()
    for denominator in range(1, budget + 1):
        first = lower.numerator * denominator // lower.denominator - 1
        last = -((-upper.numerator * denominator) // upper.denominator) + 1
        for numerator in range(first, last + 1):
            if gcd(abs(numerator), denominator) != 1:
                continue
            candidate = Fraction(numerator, denominator)
            if lower < candidate < upper:
                result.add(candidate)
    return sorted(result)


def _candidate_vector(
    slope: Fraction, use_x: bool, denominator_sign: int
) -> tuple[int, int]:
    numerator = slope.numerator * denominator_sign
    denominator = slope.denominator * denominator_sign
    return (
        (denominator, numerator) if use_x else (numerator, denominator)
    )


def _subturn_admissible(matrix, left, right, q: int) -> bool:
    dot = _dot(matrix, left, right)
    if _sign(dot) < 0:
        return False
    cosine_squared = sp.simplify(sp.cos(sp.pi / q) ** 2)
    residual = sp.simplify(
        dot**2
        - cosine_squared
        * _dot(matrix, left, left)
        * _dot(matrix, right, right)
    )
    return _sign(residual) >= 0


def _objective(matrix, ideal, candidate):
    return sp.simplify(
        _dot(matrix, ideal, ideal)
        - _dot(matrix, ideal, candidate) ** 2
        / _dot(matrix, candidate, candidate)
    )


def _canonical_expr(value) -> str:
    return sp.srepr(sp.simplify(value))


def _build_window(
    matrix,
    ideal,
    left_ideal,
    right_ideal,
    ordinal: int,
    q: int,
) -> dict:
    use_x = _sign(ideal[0] ** 2 - ideal[1] ** 2) >= 0
    denominator = ideal[0] if use_x else ideal[1]
    denominator_sign = _sign(denominator)
    midpoint_left = (
        sp.simplify(left_ideal[0] + ideal[0]),
        sp.simplify(left_ideal[1] + ideal[1]),
    )
    midpoint_right = (
        sp.simplify(ideal[0] + right_ideal[0]),
        sp.simplify(ideal[1] + right_ideal[1]),
    )

    def slope(vector):
        return sp.simplify(
            (vector[1] if use_x else vector[0])
            / (vector[0] if use_x else vector[1])
        )

    slope_a = slope(midpoint_left)
    slope_b = slope(midpoint_right)
    lower_exact, upper_exact = (
        (slope_a, slope_b)
        if _sign(slope_b - slope_a) > 0
        else (slope_b, slope_a)
    )
    lower_envelope = _envelope(lower_exact)
    upper_envelope = _envelope(upper_exact)
    certified_lower = lower_envelope[1]
    certified_upper = upper_envelope[0]
    width = certified_upper - certified_lower
    budget = _authority_budget(width)
    all_slopes = _fractions_in_open_window(
        certified_lower, certified_upper, budget
    )
    candidate_rows = []
    for candidate_slope in all_slopes:
        vector = _candidate_vector(candidate_slope, use_x, denominator_sign)
        left_ok = _subturn_admissible(matrix, left_ideal, vector, q)
        right_ok = _subturn_admissible(matrix, vector, right_ideal, q)
        objective = _objective(matrix, ideal, vector)
        candidate_rows.append(
            {
                "slope": _fraction_record(candidate_slope),
                "primitive_vector": list(vector),
                "subturn_left_le_pi_over_q": left_ok,
                "subturn_right_le_pi_over_q": right_ok,
                "admissible": left_ok and right_ok,
                "gram_distance_squared": _canonical_expr(objective),
            }
        )
    admissible = [item for item in candidate_rows if item["admissible"]]
    winner = None
    disposition = "EMPTY_AUTHORITY"
    if admissible:
        winner = admissible[0]
        tie = False
        for candidate in admissible[1:]:
            current_value = sp.sympify(winner["gram_distance_squared"], evaluate=False)
            candidate_value = sp.sympify(
                candidate["gram_distance_squared"], evaluate=False
            )
            comparison = _sign(candidate_value - current_value)
            if comparison < 0:
                winner = candidate
                tie = False
            elif comparison == 0:
                tie = True
                if _fraction(candidate["slope"]) > _fraction(winner["slope"]):
                    winner = candidate
        disposition = "HALF_UP_EXACT_TIE" if tie else "STRICT_GRAM_MINIMUM"
    denominator_counts = []
    for denominator in range(1, budget + 1):
        denominator_counts.append(
            sum(
                1
                for item in candidate_rows
                if item["slope"][1] == denominator
            )
        )
    return {
        "ordinal": ordinal,
        "left_ideal_vector": [_canonical_expr(item) for item in left_ideal],
        "ideal_vector": [_canonical_expr(item) for item in ideal],
        "right_ideal_vector": [_canonical_expr(item) for item in right_ideal],
        "dominant_component": "X" if use_x else "Y",
        "denominator_sign": denominator_sign,
        "lower_exact_srepr": _canonical_expr(lower_exact),
        "upper_exact_srepr": _canonical_expr(upper_exact),
        "lower_envelope": [
            _fraction_record(lower_envelope[0]),
            _fraction_record(lower_envelope[1]),
        ],
        "upper_envelope": [
            _fraction_record(upper_envelope[0]),
            _fraction_record(upper_envelope[1]),
        ],
        "certified_open_window": [
            _fraction_record(certified_lower),
            _fraction_record(certified_upper),
        ],
        "certified_window_width_lower_bound": _fraction_record(width),
        "authority_budget": budget,
        "authority_minimality_witness": {
            "b_minus_one_fails": (
                budget == 1 or Fraction((budget - 1) ** 2) * width < 1
            ),
            "b_passes": Fraction(budget * budget) * width >= 1,
        },
        "candidate_count_by_denominator": denominator_counts,
        "candidates": candidate_rows,
        "admissible_count": len(admissible),
        "minimum_candidate_denominator": (
            min(item["slope"][1] for item in candidate_rows)
            if candidate_rows
            else None
        ),
        "selected_slope": winner["slope"] if winner else None,
        "selection_disposition": disposition,
    }


def _owner_values(q: int, m: int) -> dict:
    u = Fraction(m, q)
    c = m
    emanated = max(2, c)
    hidden = emanated - 1
    return {
        "density": q - 2,
        "q": q,
        "u": _fraction_record(u),
        "C": c,
        "E": emanated,
        "H": hidden,
        "n": hidden + 1,
        "selection_boundary": (
            "MAX_CLOSED_ENDPOINT" if m == q else "LOWER_OPEN_UPPER_CLOSED_POINT"
        ),
    }


def _canonical_threshold_metric(q: int, m: int, orientation: int) -> dict:
    cosine = sp.cos(sp.pi * sp.Rational(m, q))
    cosine_squared = sp.simplify(cosine**2)
    if cosine_squared == 0:
        matrix = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
        end = (0, orientation)
    elif cosine_squared == 1:
        matrix = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
        end = (-1 if _sign(cosine) < 0 else 1, 0)
    else:
        rational = Fraction(cosine_squared)
        numerator, denominator = rational.numerator, rational.denominator
        off_diagonal = orientation * _sign(cosine) * numerator
        matrix = (
            (sp.Integer(1), sp.Integer(off_diagonal)),
            (
                sp.Integer(off_diagonal),
                sp.Integer(numerator * denominator),
            ),
        )
        end = (0, orientation)
    return {
        "metric_id": f"CANONICAL_Q{q}_M{m}_{'CCW' if orientation > 0 else 'CW'}",
        "inverse_gram": _matrix_record(matrix),
        "construction": "CANONICAL_MINIMAL_THRESHOLD_EQUATION_V1",
        "start": [1, 0],
        "end": list(end),
    }


def _identity_threshold_metric(q: int, m: int, orientation: int) -> dict | None:
    if q != 4 or m not in (1, 3):
        return None
    return {
        "metric_id": "IDENTITY",
        "inverse_gram": _matrix_record(((1, 0), (0, 1))),
        "construction": "FIXED_FAMILY_IDENTITY",
        "start": [1, 0],
        "end": [1 if m == 1 else -1, orientation],
    }


def _right_angle_metric(metric: dict, orientation: int) -> dict:
    matrix = _matrix_from_record(metric["inverse_gram"])
    a, b = matrix[0]
    end = (-b * orientation, a * orientation)
    end_fraction = (Fraction(end[0]), Fraction(end[1]))
    common = gcd(abs(end_fraction[0].numerator), abs(end_fraction[1].numerator))
    if common:
        end_fraction = (
            end_fraction[0] / common,
            end_fraction[1] / common,
        )
    scale = end_fraction[0].denominator * end_fraction[1].denominator
    end_integer = (
        int(end_fraction[0] * scale),
        int(end_fraction[1] * scale),
    )
    common = gcd(abs(end_integer[0]), abs(end_integer[1]))
    end_integer = (end_integer[0] // common, end_integer[1] // common)
    return {
        "metric_id": metric["metric_id"],
        "inverse_gram": metric["inverse_gram"],
        "construction": "FIXED_FAMILY_DUAL_ORTHOGONAL_BASE_BOUND",
        "start": [1, 0],
        "end": list(end_integer),
    }


def _threshold_realizations(
    q: int, m: int, orientation: int, fixed_metrics: list[dict]
) -> list[dict]:
    if 2 * m == q or m == q:
        return [
            (
                _right_angle_metric(metric, orientation)
                if m != q
                else {
                    "metric_id": metric["metric_id"],
                    "inverse_gram": metric["inverse_gram"],
                    "construction": "FIXED_FAMILY_OPPOSITE_BASE_BOUND",
                    "start": [1, 0],
                    "end": [-1, 0],
                }
            )
            for metric in fixed_metrics
        ]
    identity = _identity_threshold_metric(q, m, orientation)
    return [identity or _canonical_threshold_metric(q, m, orientation)]


def _build_real_row(
    q: int,
    m: int,
    orientation: int,
    realization: dict,
) -> dict:
    owner = _owner_values(q, m)
    matrix = _matrix_from_record(realization["inverse_gram"])
    delta = sp.pi * sp.Rational(m, q)
    ideal = _ideal_sequence(
        matrix,
        realization["start"],
        realization["end"],
        delta,
        owner["E"],
        orientation,
    )
    windows = [
        _build_window(
            matrix,
            ideal[ordinal],
            ideal[ordinal - 1],
            ideal[ordinal + 1],
            ordinal,
            q,
        )
        for ordinal in range(1, owner["E"])
    ]
    return {
        "row_id": (
            f"q{q}_m{m}_{'ccw' if orientation > 0 else 'cw'}_"
            f"{realization['metric_id'].lower()}"
        ),
        **owner,
        "orientation": "CCW" if orientation > 0 else "CW",
        "metric_id": realization["metric_id"],
        "metric_construction": realization["construction"],
        "inverse_gram": realization["inverse_gram"],
        "base_bound_start": realization["start"],
        "base_bound_end": realization["end"],
        "chebyshev_root_law": f"T_{owner['E']}(x)=cos({m}*pi/{q})",
        "windows": windows,
        "empty_window_count": sum(
            item["admissible_count"] == 0 for item in windows
        ),
    }


def _q5_selection_rows() -> list[dict]:
    neighbors = {
        1: ((6, 1), (4, 1)),
        2: ((3, 1), (2, 1)),
        3: ((2, 1), (3, 2)),
        4: ((4, 3), (6, 5)),
    }
    rows = []
    for m in range(1, 5):
        point = _owner_values(5, m)
        below, above = neighbors[m]
        rows.append(
            {
                "boundary": _fraction_record(Fraction(m, 5)),
                "point_certificate": {
                    **point,
                    "outcome": "SELECTION_SUCCESS_CLOSED_UPPER",
                    "product_geometry": "UNREPRESENTABLE_BY_THEOREM",
                    "product_windows": [],
                },
                "straddling_interval": {
                    "lower": _fraction_record(Fraction(m, 5) - Fraction(1, 100)),
                    "upper": _fraction_record(Fraction(m, 5) + Fraction(1, 100)),
                    "outcome": "ANGULAR_PROFILE_SELECTION_UNCERTAIN",
                },
                "real_product_neighbors": {
                    "below": {
                        "u": _fraction_record(Fraction(below[1], below[0])),
                        "comparison": "STRICTLY_BELOW",
                        "real_threshold_family": f"q{below[0]}_m{below[1]}",
                    },
                    "above": {
                        "u": _fraction_record(Fraction(above[1], above[0])),
                        "comparison": "STRICTLY_ABOVE",
                        "real_threshold_family": f"q{above[0]}_m{above[1]}",
                    },
                },
            }
        )
    return rows


def _simple_authority_window(lower: Fraction, upper: Fraction) -> dict:
    width = upper - lower
    budget = _authority_budget(width)
    candidates = _fractions_in_open_window(lower, upper, budget)
    return {
        "open_window": [_fraction_record(lower), _fraction_record(upper)],
        "width": _fraction_record(width),
        "authority_budget": budget,
        "candidates": [_fraction_record(item) for item in candidates],
        "outcome": "SUCCESS" if candidates else "EMPTY_AUTHORITY",
    }


def _farey_falsification() -> dict:
    return {
        "scope": "SYNTHETIC_FALSIFICATION_ONLY_NOT_PRODUCT_THRESHOLD",
        "threshold": _simple_authority_window(Fraction(1, 5), Fraction(1, 4)),
        "below": _simple_authority_window(Fraction(19, 100), Fraction(6, 25)),
        "above": _simple_authority_window(Fraction(21, 100), Fraction(13, 50)),
        "theorem": (
            "(1/B,1/(B-1)) are Farey neighbors of order B; "
            "strict interior is empty for q<=B"
        ),
    }


def _objective_value(matrix, ideal, slope: Fraction):
    vector = (slope.denominator, slope.numerator)
    return _objective(matrix, ideal, vector)


def _gram_witness(metrics_by_id: dict[str, dict]) -> dict:
    lower, upper = Fraction(39, 200), Fraction(51, 200)
    authority = _simple_authority_window(lower, upper)
    if authority["candidates"] != [[1, 5], [1, 4]]:
        raise RuntimeError("GRAM_WITNESS_CANDIDATES_DRIFT")
    ideal = (sp.Integer(40), sp.Integer(9))
    rows = []
    for metric_id in ("FIELD_WEIGHTED", "FIELD_FULL", "FIELD_FULL_MIRROR"):
        matrix = _matrix_from_record(metrics_by_id[metric_id]["inverse_gram"])
        values = [
            _objective_value(matrix, ideal, Fraction(1, 5)),
            _objective_value(matrix, ideal, Fraction(1, 4)),
        ]
        winner = Fraction(1, 5) if _sign(values[0] - values[1]) < 0 else Fraction(1, 4)
        rows.append(
            {
                "metric_id": metric_id,
                "objectives": [_canonical_expr(item) for item in values],
                "winner": _fraction_record(winner),
                "disposition": "STRICT_GRAM_MINIMUM",
            }
        )
    identity = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    left = _unit(identity, (5, 1))
    right = _unit(identity, (4, 1))
    tie_ideal = (
        sp.simplify(left[0] + right[0]),
        sp.simplify(left[1] + right[1]),
    )
    tie_values = [
        _objective_value(identity, tie_ideal, Fraction(1, 5)),
        _objective_value(identity, tie_ideal, Fraction(1, 4)),
    ]
    if _sign(tie_values[0] - tie_values[1]) != 0:
        raise RuntimeError("GRAM_TIE_NOT_EXACT")
    return {
        "authority": authority,
        "ideal_vector": [_canonical_expr(item) for item in ideal],
        "metric_switch_rows": rows,
        "switch_proven": len({tuple(item["winner"]) for item in rows}) > 1,
        "exact_tie": {
            "metric_id": "IDENTITY",
            "ideal_vector": [_canonical_expr(item) for item in tie_ideal],
            "objectives": [_canonical_expr(item) for item in tie_values],
            "winner": [1, 4],
            "disposition": "HALF_UP_EXACT_TIE",
        },
    }


def _candidate_extension_witness() -> dict:
    base = _simple_authority_window(Fraction(1, 5), Fraction(27, 100))
    extended = _simple_authority_window(Fraction(39, 200), Fraction(51, 200))
    if base["candidates"] != [[1, 4]]:
        raise RuntimeError("EXTENSION_BASE_CANDIDATES_DRIFT")
    if extended["candidates"] != [[1, 5], [1, 4]]:
        raise RuntimeError("EXTENSION_CANDIDATES_DRIFT")
    matrix = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    ideal = (sp.Integer(200), sp.Integer(41))
    extended_values = [
        _objective_value(matrix, ideal, Fraction(1, 5)),
        _objective_value(matrix, ideal, Fraction(1, 4)),
    ]
    if _sign(extended_values[0] - extended_values[1]) >= 0:
        raise RuntimeError("EXTENSION_BETTER_CANDIDATE_DRIFT")
    return {
        "scope": "SYNTHETIC_MONOTONE_AUTHORITY_EXTENSION",
        "metric_id": "IDENTITY",
        "ideal_vector": [_canonical_expr(item) for item in ideal],
        "base": {
            **base,
            "winner": [1, 4],
        },
        "extended": {
            **extended,
            "objectives": [_canonical_expr(item) for item in extended_values],
            "old_winner": [1, 4],
            "winner": [1, 5],
        },
    }


def _operand_bits(value) -> int:
    if isinstance(value, dict):
        return max((_operand_bits(item) for item in value.values()), default=0)
    if isinstance(value, list):
        return max((_operand_bits(item) for item in value), default=0)
    if isinstance(value, int):
        return abs(value).bit_length()
    if isinstance(value, str):
        integers = [int(item) for item in value.replace("-", " ").split() if item.isdigit()]
        return max((abs(item).bit_length() for item in integers), default=0)
    return 0


def build_receipt(repo: Path) -> dict:
    started = time.perf_counter()
    for name, expected in PRODUCT_OIDS.items():
        path = "kernel/src" if name == "kernel_src" else name
        actual = _git(repo, "rev-parse", f"HEAD:{path}")
        if actual != expected:
            raise RuntimeError(f"UNKNOWN_PRODUCT_OID:{name}:{actual}")
    if (
        subprocess.run(
            ("git", "-C", str(repo), "merge-base", "--is-ancestor", BASE_SHA, "HEAD"),
            check=False,
        ).returncode
        != 0
    ):
        raise RuntimeError("UNKNOWN_PRODUCT_ANCESTRY")

    sources = [_read_metric(repo, source) for source in METRIC_SOURCES]
    identity = {
        "metric_id": "IDENTITY",
        "source": "SYNTHETIC_RATIONAL_IDENTITY",
        "gram": _matrix_record(((1, 0), (0, 1))),
        "inverse_gram": _matrix_record(((1, 0), (0, 1))),
    }
    fixed_metrics = [identity]
    for source in sources:
        fixed_metrics.extend((source, _mirror_metric(source)))
    metrics_by_id = {item["metric_id"]: item for item in fixed_metrics}

    rows = []
    for q in REAL_Q:
        for m in range(1, q):
            for orientation in (1, -1):
                for realization in _threshold_realizations(
                    q, m, orientation, fixed_metrics
                ):
                    rows.append(
                        _build_real_row(q, m, orientation, realization)
                    )
    for orientation in (1, -1):
        for realization in _threshold_realizations(
            6, 6, orientation, fixed_metrics
        ):
            rows.append(_build_real_row(6, 6, orientation, realization))

    receipt = {
        "schema": "cftuv.envelope.dens_a_prime_authority_proof.v1",
        "scope": "PROOF_ONLY_NO_PRODUCT_AUTHORITY",
        "source_authority": {
            "base_sha": BASE_SHA,
            "product_tree_oids": PRODUCT_OIDS,
            "metric_sources": sources,
        },
        "policy": {
            "owner_law": (
                "u=(phi-pi)/pi; q=d+2; C=ceil(q*u) lower-OPEN/upper-CLOSED; "
                "E=max(2,C); H=max(1,C-1); n=H+1"
            ),
            "candidate_authority": (
                "B(w)=min{b>=1:b^2*w>=1}; every reduced p/q strictly inside "
                "the certified window with q<=B; then exact dual-Gram subturn"
            ),
            "product_window": (
                "dominant-component slope; OPEN dual-unit Gram midpoint window; "
                "outward 27-digit envelopes; width=upper.lower-lower.upper"
            ),
            "point_threshold_only": "EXCLUDED_FROM_FUTURE_PRODUCT_CONTRACT",
        },
        "q5_unrepresentability_theorem": {
            "statement": (
                "For rational SPD dual Gram D and rational BASE_BOUND a,b, "
                "cos^2(delta)=(aDb)^2/((aDa)(bDb)) is rational. "
                "At internal q=5 thresholds cos^2 is (3±sqrt(5))/8, irrational."
            ),
            "product_consequence": (
                "exact q5 threshold geometry is forbidden; point behavior is "
                "selection-certificate-owned and has no product ideal window"
            ),
            "algebraic_non_base_bound_contract": "REJECTED",
        },
        "fixed_metric_family": fixed_metrics,
        "selection_point_matrix": [
            _owner_values(q, m)
            for q in range(2, 7)
            for m in range(1, q + 1)
        ],
        "real_product_window_matrix": rows,
        "q5_selection_matrix": _q5_selection_rows(),
        "farey_falsification_witness": _farey_falsification(),
        "gram_objective_witness": _gram_witness(metrics_by_id),
        "candidate_extension_witness": _candidate_extension_witness(),
        "red_controls": [
            {"name": name, "expected": "REJECT"}
            for name in (
                "UNREACHABLE_H",
                "NON_COPRIME",
                "BOUNDARY_CANDIDATE",
                "NON_MINIMUM",
                "FORGED_OLD_WINNER_AFTER_BETTER",
                "FAKE_TIE",
                "DUPLICATE",
                "ORDER",
                "CROSS",
                "UNKNOWN_PRODUCT_OID",
                "UNKNOWN_FIXTURE",
            )
        ],
    }
    receipt["stats"] = {
        "real_rows": len(rows),
        "real_windows": sum(len(item["windows"]) for item in rows),
        "empty_real_windows": sum(
            window["admissible_count"] == 0
            for row in rows
            for window in row["windows"]
        ),
        "q5_point_rows": len(receipt["q5_selection_matrix"]),
        "candidate_rows": sum(
            len(window["candidates"])
            for row in rows
            for window in row["windows"]
        ),
        "max_operand_bits": _operand_bits(receipt),
        "wall_seconds_budget": "30.000000000",
    }
    receipt["stats"]["certified_bit_cost_units"] = (
        receipt["stats"]["candidate_rows"] + receipt["stats"]["real_windows"]
    ) * receipt["stats"]["max_operand_bits"] ** 2
    receipt["generation_observation"] = {
        "operation": "REFERENCE_GENERATION_WALL_CLOCK_REPORTED_OUTSIDE_CANONICAL_BYTES",
        "deterministic_receipt": True,
    }
    build_receipt.last_wall_seconds = time.perf_counter() - started
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=Path(__file__).parents[3])
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).with_name("dens_a_prime_authority_receipt.json"),
    )
    args = parser.parse_args()
    receipt = build_receipt(args.repo.resolve())
    args.output.write_bytes(_canonical_bytes(receipt))
    print(
        json.dumps(
            {
                "output": str(args.output),
                "sha256": hashlib.sha256(args.output.read_bytes()).hexdigest(),
                "stats": receipt["stats"],
                "wall_seconds": f"{build_receipt.last_wall_seconds:.9f}",
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
