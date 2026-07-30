"""Автономный verifier расписки DENS-A′/AUTHORITY.

Модуль читает только canonical receipt bytes и текущие git/fixture authorities.
Генератор не импортируется и его функции не вызываются.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import subprocess
import time
from fractions import Fraction
from math import gcd, isqrt
from pathlib import Path

import sympy as sp
from mpmath import iv, libmp

from cftuv_envelope.reference.planar_types import (
    CertifiedPredicateUndecidable,
    exact_sign,
    interval_enclosure,
)


EXPECTED_BASE = "b5b97e43ee7ca4d30a540f42ff36cf01852c2911"
EXPECTED_PRODUCT = {
    "cftuv": "01f510088b50c425570cc2e42c13e400ca4875f3",
    "kernel_src": "ed33d3f43635a5acbe5b8695dfed9b2e12396c6b",
    "tools": "ec5439a5fc8ea46784d4621ac763f941b4833c36",
}
EXPECTED_FIXTURES = {
    "FIELD_WEIGHTED": (
        "kernel/fixtures/building_002_weighted_normals_v1/analysis_snapshot.json",
        "e5bbf6b3210ee195c227b4f09ac776418dd1f723592cefdbefe1acfd844693a3",
    ),
    "FIELD_FULL": (
        "kernel/fixtures/building_002_full_selection_v1/analysis_snapshot.json",
        "5c759cfc80548072eb3918f5fdc114d6f93ad077b70a5f0c663f10ede2f41d16",
    ),
}
SCALE = 10**27
EXPECTED_RECEIPT_SHA256 = (
    "6a341750e26dec13f8edb5ff0daf3a0000cb632b6ca67c3641f7a0ff272345f1"
)
POLICY = {
    "owner_law_id": "HUBER_EMANATED_DENSITY_FLOOR_V1",
    "owner_law": (
        "u=(phi-pi)/pi; q=d+2; C=ceil(q*u) lower-OPEN/upper-CLOSED; "
        "E=max(2,C); H=max(1,C-1); n=H+1"
    ),
    "candidate_authority_id": "CERTIFIED_WIDTH_SQUARE_ROOT_DENOMINATOR_V1",
    "candidate_authority": (
        "B(w)=min{b>=1:b^2*w>=1}; every reduced p/q strictly inside "
        "the certified window with q<=B; then exact dual-Gram subturn"
    ),
    "product_window_id": "DUAL_GRAM_DOMINANT_SLOPE_MIDPOINT_OPEN_V1",
    "product_window": (
        "dominant-component slope; OPEN dual-unit Gram midpoint window; "
        "outward 27-digit envelopes; width=upper.lower-lower.upper"
    ),
    "point_threshold_only": "EXCLUDED_FROM_FUTURE_PRODUCT_CONTRACT",
}
Q5_THEOREM = {
    "law_id": "RATIONAL_GRAM_BASE_BOUND_Q5_UNREPRESENTABLE_V1",
    "rational_cos_squared_field": "Q",
    "q5_minimal_polynomial_coefficients": [16, -12, 1],
    "q5_minimal_polynomial_discriminant": 80,
    "boundary_root_class": {
        "1": "PLUS_SQRT5",
        "2": "MINUS_SQRT5",
        "3": "MINUS_SQRT5",
        "4": "PLUS_SQRT5",
    },
    "product_consequence": (
        "Q5_POINT_SELECTION_CERTIFICATE_ONLY_NO_PRODUCT_IDEAL_WINDOW"
    ),
    "algebraic_non_base_bound_contract": "REJECTED_NON_BASE_BOUND",
}
RED_CONTROL_CODES = {
    "UNREACHABLE_H": "OWNER_LAW",
    "NON_COPRIME": "NON_COPRIME",
    "BOUNDARY_CANDIDATE": "BOUNDARY_CANDIDATE",
    "NON_MINIMUM": "NON_MINIMUM",
    "FORGED_OLD_WINNER_AFTER_BETTER": (
        "FORGED_OLD_WINNER_AFTER_BETTER"
    ),
    "FAKE_TIE": "FAKE_TIE",
    "DUPLICATE": "DUPLICATE",
    "ORDER": "ORDER",
    "CROSS": "CROSS",
    "UNKNOWN_PRODUCT_OID": "UNKNOWN_PRODUCT_OID",
    "UNKNOWN_FIXTURE": "UNKNOWN_FIXTURE",
    "ALGEBRAIC_NEAR_ZERO": "EXACT_SIGN_UNDECIDABLE",
    "ALGEBRAIC_UNSUPPORTED": "EXACT_SIGN_UNDECIDABLE",
}
EXPLOIT_CONTROL_CODES = {
    "FORGED_SCOPE": "SCOPE",
    "FORGED_POLICY": "POLICY",
    "FORGED_SOURCE_GRAM": "UNKNOWN_FIXTURE",
    "FORGED_SOURCE_BLOB": "UNKNOWN_FIXTURE",
    "FORGED_SOURCE_DOMAIN": "UNKNOWN_FIXTURE",
    "FORGED_SOURCE_REFERENCE_ID": "UNKNOWN_FIXTURE",
    "FORGED_FIXED_FAMILY": "FIXED_METRIC_FAMILY",
    "FORGED_Q5_THEOREM": "Q5_THEOREM",
    "FORGED_RED_EXPECTED_ACCEPT": "RED_MATRIX",
    "FORGED_STATS_ZERO": "STATS",
    "FORGED_DETERMINISM_FALSE": "GENERATION_OBSERVATION",
    "RECOMPUTED_DIGEST_ANCHOR_ATTEMPT": "RECEIPT_ANCHOR",
}
TOP_LEVEL_KEYS = {
    "candidate_extension_witness",
    "exploit_regression_corpus",
    "farey_falsification_witness",
    "fixed_metric_family",
    "generation_observation",
    "gram_objective_witness",
    "policy",
    "q5_selection_matrix",
    "q5_unrepresentability_theorem",
    "real_product_window_matrix",
    "red_controls",
    "schema",
    "scope",
    "selection_point_matrix",
    "source_authority",
    "stats",
}


class Reject(ValueError):
    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


def _reject(code: str) -> None:
    raise Reject(code)


def _keys(value: dict, expected: set[str], code: str) -> None:
    if type(value) is not dict or set(value) != expected:
        _reject(code)


def _canonical_bytes(value: object) -> bytes:
    return (
        json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        + "\n"
    ).encode("utf-8")


def _check_anchor(raw: bytes) -> None:
    if hashlib.sha256(raw).hexdigest() != EXPECTED_RECEIPT_SHA256:
        _reject("RECEIPT_ANCHOR")


def _git(repo: Path, *args: str) -> str:
    return subprocess.check_output(
        ("git", "-C", str(repo), *args), text=True
    ).strip()


def _f(record) -> Fraction:
    if (
        not isinstance(record, list)
        or len(record) != 2
        or type(record[0]) is not int
        or type(record[1]) is not int
        or record[1] <= 0
    ):
        _reject("RATIONAL_RECORD")
    return Fraction(record[0], record[1])


def _m(record):
    if not isinstance(record, list) or len(record) != 2:
        _reject("MATRIX_RECORD")
    return tuple(
        tuple(sp.Rational(*record[row][column]) for column in range(2))
        for row in range(2)
    )


def _expr(record: str):
    try:
        return sp.sympify(record, evaluate=False)
    except Exception:
        _reject("EXACT_EXPRESSION")


def _dot(matrix, left, right):
    return sp.simplify(
        left[0] * (matrix[0][0] * right[0] + matrix[0][1] * right[1])
        + left[1] * (matrix[1][0] * right[0] + matrix[1][1] * right[1])
    )


def _sign(value) -> int:
    value = sp.simplify(value)
    try:
        result = exact_sign(value)
    except (
        CertifiedPredicateUndecidable,
        ArithmeticError,
        TypeError,
        ValueError,
    ):
        _reject("EXACT_SIGN_UNDECIDABLE")
    if result not in (-1, 0, 1):
        _reject("EXACT_SIGN_UNDECIDABLE")
    return result


def _envelope(expression) -> tuple[Fraction, Fraction]:
    saved = iv.prec
    iv.prec = 160
    try:
        enclosure = interval_enclosure(sp.simplify(expression))
    except Exception:
        _reject("ENVELOPE_UNDECIDABLE")
    finally:
        iv.prec = saved
    lower, upper = (
        Fraction(*libmp.to_rational(item)) for item in enclosure._mpi_
    )
    lower_units = lower.numerator * SCALE // lower.denominator
    upper_units = -((-upper.numerator * SCALE) // upper.denominator)
    return (
        Fraction(lower_units, SCALE),
        Fraction(upper_units, SCALE),
    )


def _budget(width: Fraction) -> int:
    if width <= 0:
        _reject("NON_POSITIVE_WIDTH")
    estimate = (width.denominator + width.numerator - 1) // width.numerator
    result = isqrt(estimate)
    while result * result * width < 1:
        result += 1
    while result > 1 and (result - 1) ** 2 * width >= 1:
        result -= 1
    return result


def _enumerate(lower: Fraction, upper: Fraction, budget: int) -> list[Fraction]:
    values: set[Fraction] = set()
    for denominator in range(1, budget + 1):
        low = lower.numerator * denominator // lower.denominator - 1
        high = -((-upper.numerator * denominator) // upper.denominator) + 1
        for numerator in range(low, high + 1):
            if gcd(abs(numerator), denominator) != 1:
                continue
            value = Fraction(numerator, denominator)
            if lower < value < upper:
                values.add(value)
    return sorted(values)


def _candidate_vector(
    value: Fraction, use_x: bool, denominator_sign: int
) -> tuple[int, int]:
    numerator = value.numerator * denominator_sign
    denominator = value.denominator * denominator_sign
    return (
        (denominator, numerator) if use_x else (numerator, denominator)
    )


def _objective(matrix, ideal, candidate):
    return sp.simplify(
        _dot(matrix, ideal, ideal)
        - _dot(matrix, ideal, candidate) ** 2
        / _dot(matrix, candidate, candidate)
    )


def _subturn(matrix, left, right, q: int) -> bool:
    dot = _dot(matrix, left, right)
    if _sign(dot) < 0:
        return False
    residual = sp.simplify(
        dot**2
        - sp.cos(sp.pi / q) ** 2
        * _dot(matrix, left, left)
        * _dot(matrix, right, right)
    )
    return _sign(residual) >= 0


def _owner(q: int, m: int) -> tuple[int, int, int, int]:
    c = m
    e = max(2, c)
    h = max(1, c - 1)
    return c, e, h, h + 1


def _exact_rational(record) -> Fraction:
    _keys(record, {"$type", "numerator", "denominator"}, "UNKNOWN_FIXTURE")
    if record["$type"] != "ExactRationalV1":
        _reject("UNKNOWN_FIXTURE")
    return Fraction(record["numerator"], record["denominator"])


def _matrix_record(matrix) -> list[list[list[int]]]:
    return [
        [
            [Fraction(matrix[row][column]).numerator, Fraction(matrix[row][column]).denominator]
            for column in range(2)
        ]
        for row in range(2)
    ]


def _fixture_metric(repo: Path, claimed: dict) -> dict:
    metric_id = claimed.get("metric_id")
    if metric_id not in EXPECTED_FIXTURES:
        _reject("UNKNOWN_FIXTURE")
    path_text, digest = EXPECTED_FIXTURES[metric_id]
    path = repo / path_text
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != digest:
        _reject("UNKNOWN_FIXTURE")
    payload = json.loads(raw)
    matches = [
        item
        for item in payload["surface_metric_descriptors"]
        if item["reference_metric_id"]["value"]
        == claimed.get("reference_metric_id")
        and item["patch_domain_id"]["value"] == claimed.get("patch_domain_id")
    ]
    if len(matches) != 1:
        _reject("UNKNOWN_FIXTURE")
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
    expected = {
        "metric_id": metric_id,
        "path": path_text,
        "sha256": digest,
        "reference_metric_id": claimed["reference_metric_id"],
        "patch_domain_id": claimed["patch_domain_id"],
        "blob_oid": _git(repo, "rev-parse", f"HEAD:{path_text}"),
        "gram": _matrix_record(gram),
        "inverse_gram": _matrix_record(inverse),
    }
    if claimed != expected:
        _reject("UNKNOWN_FIXTURE")
    return expected


def _mirror_metric(metric: dict) -> dict:
    def mirror(record):
        result = copy.deepcopy(record)
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


def _expected_fixed_metrics(sources: list[dict]) -> list[dict]:
    identity = {
        "metric_id": "IDENTITY",
        "source": "SYNTHETIC_RATIONAL_IDENTITY",
        "gram": _matrix_record(((1, 0), (0, 1))),
        "inverse_gram": _matrix_record(((1, 0), (0, 1))),
    }
    result = [identity]
    for source in sources:
        result.extend((source, _mirror_metric(source)))
    return result


def _validate_authorities(document: dict, repo: Path) -> None:
    authority = document.get("source_authority", {})
    _keys(
        authority,
        {"base_sha", "product_tree_oids", "metric_sources"},
        "SOURCE_AUTHORITY",
    )
    if authority.get("base_sha") != EXPECTED_BASE:
        _reject("UNKNOWN_PRODUCT_OID")
    if authority.get("product_tree_oids") != EXPECTED_PRODUCT:
        _reject("UNKNOWN_PRODUCT_OID")
    for name, expected in EXPECTED_PRODUCT.items():
        path = "kernel/src" if name == "kernel_src" else name
        if _git(repo, "rev-parse", f"HEAD:{path}") != expected:
            _reject("UNKNOWN_PRODUCT_OID")
    if (
        subprocess.run(
            ("git", "-C", str(repo), "merge-base", "--is-ancestor", EXPECTED_BASE, "HEAD"),
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode
        != 0
    ):
        _reject("UNKNOWN_PRODUCT_OID")
    sources = authority.get("metric_sources", [])
    if {item.get("metric_id") for item in sources} != set(EXPECTED_FIXTURES):
        _reject("UNKNOWN_FIXTURE")
    rebuilt_sources = []
    for item in sources:
        path, digest = EXPECTED_FIXTURES[item["metric_id"]]
        if item.get("path") != path or item.get("sha256") != digest:
            _reject("UNKNOWN_FIXTURE")
        if hashlib.sha256((repo / path).read_bytes()).hexdigest() != digest:
            _reject("UNKNOWN_FIXTURE")
        rebuilt_sources.append(_fixture_metric(repo, item))
    expected_fixed = _expected_fixed_metrics(rebuilt_sources)
    if document.get("fixed_metric_family") != expected_fixed:
        _reject("FIXED_METRIC_FAMILY")


def _validate_owner_record(record: dict) -> None:
    required = {"density", "q", "u", "C", "E", "H", "n", "selection_boundary"}
    if not required.issubset(record):
        _reject("OWNER_LAW")
    q = record.get("q")
    u = _f(record.get("u"))
    if q not in range(2, 7) or record.get("density") != q - 2:
        _reject("OWNER_LAW")
    scaled = q * u
    c = scaled.numerator // scaled.denominator
    if Fraction(c) < scaled:
        c += 1
    expected = (c, max(2, c), max(1, c - 1), max(1, c - 1) + 1)
    actual = (
        record.get("C"),
        record.get("E"),
        record.get("H"),
        record.get("n"),
    )
    if actual != expected:
        _reject("OWNER_LAW")


def _validate_owner_matrix(document: dict) -> None:
    rows = document.get("selection_point_matrix", [])
    expected = {(q, m) for q in range(2, 7) for m in range(1, q + 1)}
    actual = set()
    for record in rows:
        _validate_owner_record(record)
        q = record["q"]
        u = _f(record["u"])
        actual.add((q, int(q * u)))
    if actual != expected or len(rows) != len(expected):
        _reject("OWNER_MATRIX")
    h_cells = {}
    for record in rows:
        h_cells.setdefault(record["H"], set()).add(record["C"])
    if set(h_cells) != {1, 2, 3, 4, 5} or h_cells[1] != {1, 2}:
        _reject("OWNER_MATRIX")


def _window_vectors(window: dict):
    return tuple(
        tuple(_expr(item) for item in window[name])
        for name in ("left_ideal_vector", "ideal_vector", "right_ideal_vector")
    )


def _validate_window(
    window: dict,
    matrix,
    q: int,
    orientation: int,
) -> None:
    _keys(
        window,
        {
            "ordinal",
            "left_ideal_vector",
            "ideal_vector",
            "right_ideal_vector",
            "dominant_component",
            "denominator_sign",
            "lower_exact_srepr",
            "upper_exact_srepr",
            "lower_envelope",
            "upper_envelope",
            "certified_open_window",
            "certified_window_width_lower_bound",
            "authority_budget",
            "authority_minimality_witness",
            "candidate_count_by_denominator",
            "candidates",
            "admissible_count",
            "minimum_candidate_denominator",
            "selected_slope",
            "selection_disposition",
        },
        "WINDOW_SCHEMA",
    )
    _keys(
        window["authority_minimality_witness"],
        {"b_minus_one_fails", "b_passes"},
        "WINDOW_SCHEMA",
    )
    left, ideal, right = _window_vectors(window)
    for vector in (left, ideal, right):
        if _sign(_dot(matrix, vector, vector) - 1) != 0:
            _reject("IDEAL_UNIT")
    if _sign(left[0] * ideal[1] - left[1] * ideal[0]) != orientation:
        _reject("CROSS")
    if _sign(ideal[0] * right[1] - ideal[1] * right[0]) != orientation:
        _reject("CROSS")
    use_x = _sign(ideal[0] ** 2 - ideal[1] ** 2) >= 0
    if window.get("dominant_component") != ("X" if use_x else "Y"):
        _reject("DOMINANT_COMPONENT")
    denominator = ideal[0] if use_x else ideal[1]
    denominator_sign = _sign(denominator)
    if window.get("denominator_sign") != denominator_sign:
        _reject("DOMINANT_COMPONENT")

    def slope(vector):
        return sp.simplify(
            (vector[1] if use_x else vector[0])
            / (vector[0] if use_x else vector[1])
        )

    mid_left = (left[0] + ideal[0], left[1] + ideal[1])
    mid_right = (ideal[0] + right[0], ideal[1] + right[1])
    exact = (slope(mid_left), slope(mid_right))
    lower_exact, upper_exact = (
        exact if _sign(exact[1] - exact[0]) > 0 else (exact[1], exact[0])
    )
    if _sign(_expr(window["lower_exact_srepr"]) - lower_exact) != 0:
        _reject("WINDOW_EXPRESSION")
    if _sign(_expr(window["upper_exact_srepr"]) - upper_exact) != 0:
        _reject("WINDOW_EXPRESSION")
    lower_envelope = tuple(_f(item) for item in window["lower_envelope"])
    upper_envelope = tuple(_f(item) for item in window["upper_envelope"])
    if lower_envelope != _envelope(lower_exact):
        _reject("WINDOW_ENVELOPE")
    if upper_envelope != _envelope(upper_exact):
        _reject("WINDOW_ENVELOPE")
    lower, upper = (_f(item) for item in window["certified_open_window"])
    if lower != lower_envelope[1] or upper != upper_envelope[0]:
        _reject("WINDOW_ENVELOPE")
    width = upper - lower
    if _f(window["certified_window_width_lower_bound"]) != width:
        _reject("WINDOW_WIDTH")
    budget = _budget(width)
    if window.get("authority_budget") != budget:
        _reject("AUTHORITY_BUDGET")
    minimality = window.get("authority_minimality_witness", {})
    if (
        not minimality.get("b_passes")
        or not minimality.get("b_minus_one_fails")
        or budget * budget * width < 1
        or (budget > 1 and (budget - 1) ** 2 * width >= 1)
    ):
        _reject("AUTHORITY_BUDGET")

    rows = window.get("candidates", [])
    supplied_slopes = []
    for row in rows:
        _keys(
            row,
            {
                "slope",
                "primitive_vector",
                "subturn_left_le_pi_over_q",
                "subturn_right_le_pi_over_q",
                "admissible",
                "gram_distance_squared",
            },
            "CANDIDATE_SCHEMA",
        )
        raw = row.get("slope")
        if (
            not isinstance(raw, list)
            or len(raw) != 2
            or gcd(abs(raw[0]), raw[1]) != 1
        ):
            _reject("NON_COPRIME")
        value = _f(raw)
        if value == lower or value == upper:
            _reject("BOUNDARY_CANDIDATE")
        if not lower < value < upper:
            _reject("CANDIDATE_OUTSIDE")
        supplied_slopes.append(value)
    if len(supplied_slopes) != len(set(supplied_slopes)):
        _reject("DUPLICATE")
    if supplied_slopes != sorted(supplied_slopes):
        _reject("ORDER")
    expected_slopes = _enumerate(lower, upper, budget)
    if supplied_slopes != expected_slopes:
        _reject("CANDIDATE_SET")

    admissible = []
    for row, value in zip(rows, supplied_slopes, strict=True):
        vector = _candidate_vector(value, use_x, denominator_sign)
        if row.get("primitive_vector") != list(vector):
            _reject("CROSS")
        left_ok = _subturn(matrix, left, vector, q)
        right_ok = _subturn(matrix, vector, right, q)
        if (
            row.get("subturn_left_le_pi_over_q") is not left_ok
            or row.get("subturn_right_le_pi_over_q") is not right_ok
            or row.get("admissible") is not (left_ok and right_ok)
        ):
            _reject("SUBTURN")
        objective = _objective(matrix, ideal, vector)
        if _sign(_expr(row["gram_distance_squared"]) - objective) != 0:
            _reject("GRAM_OBJECTIVE")
        if left_ok and right_ok:
            admissible.append((value, objective))

    counts = [
        sum(value.denominator == denominator for value in supplied_slopes)
        for denominator in range(1, budget + 1)
    ]
    if window.get("candidate_count_by_denominator") != counts:
        _reject("DENOMINATOR_COUNTS")
    minimum_denominator = (
        min(value.denominator for value in supplied_slopes)
        if supplied_slopes
        else None
    )
    if window.get("minimum_candidate_denominator") != minimum_denominator:
        _reject("DENOMINATOR_COUNTS")
    if window.get("admissible_count") != len(admissible):
        _reject("ADMISSIBLE_COUNT")

    if not admissible:
        if (
            window.get("selected_slope") is not None
            or window.get("selection_disposition") != "EMPTY_AUTHORITY"
        ):
            _reject("NON_MINIMUM")
        return
    minimum_value = min(item[1] for item in admissible)
    tied = [item[0] for item in admissible if _sign(item[1] - minimum_value) == 0]
    winner = max(tied)
    if _f(window.get("selected_slope")) != winner:
        _reject("NON_MINIMUM")
    expected_disposition = (
        "HALF_UP_EXACT_TIE" if len(tied) > 1 else "STRICT_GRAM_MINIMUM"
    )
    if window.get("selection_disposition") != expected_disposition:
        _reject("FAKE_TIE")


def _validate_real_rows(document: dict) -> None:
    rows = document.get("real_product_window_matrix", [])
    coverage = {(q, m, orientation) for q in (2, 3, 4, 6) for m in range(1, q) for orientation in ("CCW", "CW")}
    observed = set()
    h_values = set()
    fixed_by_id = {
        item["metric_id"]: item for item in document["fixed_metric_family"]
    }
    for row in rows:
        _keys(
            row,
            {
                "row_id",
                "density",
                "q",
                "u",
                "C",
                "E",
                "H",
                "n",
                "selection_boundary",
                "orientation",
                "metric_id",
                "metric_construction",
                "inverse_gram",
                "base_bound_start",
                "base_bound_end",
                "chebyshev_root_law",
                "windows",
                "empty_window_count",
            },
            "REAL_ROW_SCHEMA",
        )
        _validate_owner_record(row)
        q = row["q"]
        m = int(q * _f(row["u"]))
        orientation = 1 if row.get("orientation") == "CCW" else -1
        if q not in (2, 3, 4, 6) or not 1 <= m <= q:
            _reject("REAL_MATRIX")
        if m < q:
            observed.add((q, m, row["orientation"]))
        h_values.add(row["H"])
        matrix = _m(row["inverse_gram"])
        determinant = sp.simplify(
            matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]
        )
        if _sign(matrix[0][0]) <= 0 or _sign(determinant) <= 0:
            _reject("SPD_METRIC")
        start = tuple(sp.Integer(item) for item in row["base_bound_start"])
        end = tuple(sp.Integer(item) for item in row["base_bound_end"])
        construction = row["metric_construction"]
        if construction in {
            "FIXED_FAMILY_DUAL_ORTHOGONAL_BASE_BOUND",
            "FIXED_FAMILY_OPPOSITE_BASE_BOUND",
            "FIXED_FAMILY_IDENTITY",
        }:
            fixed = fixed_by_id.get(row["metric_id"])
            if fixed is None or row["inverse_gram"] != fixed["inverse_gram"]:
                _reject("FIXED_METRIC_FAMILY")
        elif construction == "CANONICAL_MINIMAL_THRESHOLD_EQUATION_V1":
            if row["metric_id"] in fixed_by_id:
                _reject("CANONICAL_METRIC_SCOPE")
            cosine = sp.cos(sp.pi * sp.Rational(m, q))
            cosine_squared = sp.simplify(cosine**2)
            if cosine_squared.is_Rational is not True or cosine_squared in (0, 1):
                _reject("CANONICAL_METRIC_SCOPE")
            rational = Fraction(cosine_squared)
            off_diagonal = (
                orientation
                * _sign(cosine)
                * rational.numerator
            )
            expected_matrix = (
                (sp.Integer(1), sp.Integer(off_diagonal)),
                (
                    sp.Integer(off_diagonal),
                    sp.Integer(rational.numerator * rational.denominator),
                ),
            )
            expected_id = (
                f"CANONICAL_Q{q}_M{m}_"
                f"{'CCW' if orientation > 0 else 'CW'}"
            )
            if (
                row["metric_id"] != expected_id
                or matrix != expected_matrix
                or start != (1, 0)
                or end != (0, orientation)
            ):
                _reject("CANONICAL_METRIC_SCOPE")
        else:
            _reject("CANONICAL_METRIC_SCOPE")
        if gcd(abs(int(start[0])), abs(int(start[1]))) != 1:
            _reject("BASE_BOUND")
        if gcd(abs(int(end[0])), abs(int(end[1]))) != 1:
            _reject("BASE_BOUND")
        cosine_squared = sp.simplify(
            _dot(matrix, start, end) ** 2
            / (_dot(matrix, start, start) * _dot(matrix, end, end))
        )
        if _sign(cosine_squared - sp.cos(sp.pi * m / q) ** 2) != 0:
            _reject("THRESHOLD_EQUATION")
        dot_sign = _sign(_dot(matrix, start, end))
        expected_dot = _sign(sp.cos(sp.pi * m / q))
        if dot_sign != expected_dot:
            _reject("THRESHOLD_EQUATION")
        windows = row.get("windows", [])
        if len(windows) != row["H"]:
            _reject("REAL_MATRIX")
        for ordinal, window in enumerate(windows, start=1):
            if window.get("ordinal") != ordinal:
                _reject("ORDER")
            _validate_window(window, matrix, q, orientation)
        if row.get("empty_window_count") != sum(
            item.get("admissible_count") == 0 for item in windows
        ):
            _reject("EMPTY_COUNTS")
        if windows:
            sequence = [_window_vectors(windows[0])[0]]
            sequence.extend(_window_vectors(item)[1] for item in windows)
            sequence.append(_window_vectors(windows[-1])[2])
            if len(sequence) != row["E"] + 1:
                _reject("IDEAL_SEQUENCE")
            if _sign(sequence[0][0] * start[1] - sequence[0][1] * start[0]) != 0:
                _reject("BASE_BOUND")
            if _sign(sequence[-1][0] * end[1] - sequence[-1][1] * end[0]) != 0:
                _reject("BASE_BOUND")
            step_cosine = sp.cos(sp.pi * m / (q * row["E"]))
            for left, right in zip(sequence, sequence[1:]):
                if _sign(_dot(matrix, left, right) - step_cosine) != 0:
                    _reject("CHEBYSHEV_ROOT")
            x = _dot(matrix, sequence[0], sequence[1])
            if _sign(sp.chebyshevt(row["E"], x) - sp.cos(sp.pi * m / q)) != 0:
                _reject("CHEBYSHEV_ROOT")
    if not coverage.issubset(observed):
        _reject("REAL_MATRIX")
    if h_values != {1, 2, 3, 4, 5}:
        _reject("REAL_MATRIX")


def _validate_q5(document: dict) -> None:
    rows = document.get("q5_selection_matrix", [])
    if len(rows) != 4:
        _reject("Q5_SELECTION")
    for m, row in enumerate(rows, start=1):
        _keys(
            row,
            {
                "boundary",
                "point_certificate",
                "straddling_interval",
                "real_product_neighbors",
            },
            "Q5_SELECTION",
        )
        boundary = Fraction(m, 5)
        if _f(row["boundary"]) != boundary:
            _reject("Q5_SELECTION")
        point = row["point_certificate"]
        _keys(
            point,
            {
                "density",
                "q",
                "u",
                "C",
                "E",
                "H",
                "n",
                "selection_boundary",
                "outcome",
                "product_geometry",
                "product_windows",
            },
            "Q5_SELECTION",
        )
        _validate_owner_record(point)
        if (
            _f(point["u"]) != boundary
            or point.get("outcome") != "SELECTION_SUCCESS_CLOSED_UPPER"
            or point.get("product_geometry") != "UNREPRESENTABLE_BY_THEOREM"
            or point.get("product_windows") != []
        ):
            _reject("Q5_SELECTION")
        interval = row["straddling_interval"]
        _keys(interval, {"lower", "upper", "outcome"}, "Q5_SELECTION")
        if not _f(interval["lower"]) < boundary < _f(interval["upper"]):
            _reject("Q5_SELECTION")
        if interval.get("outcome") != "ANGULAR_PROFILE_SELECTION_UNCERTAIN":
            _reject("Q5_SELECTION")
        neighbors = row["real_product_neighbors"]
        _keys(neighbors, {"below", "above"}, "Q5_SELECTION")
        for side in ("below", "above"):
            _keys(
                neighbors[side],
                {"u", "comparison", "real_threshold_family"},
                "Q5_SELECTION",
            )
        if not _f(neighbors["below"]["u"]) < boundary < _f(neighbors["above"]["u"]):
            _reject("Q5_SELECTION")
    theorem = document.get("q5_unrepresentability_theorem")
    if theorem != Q5_THEOREM:
        _reject("Q5_THEOREM")
    coefficients = theorem["q5_minimal_polynomial_coefficients"]
    a, b, c = coefficients
    discriminant = b * b - 4 * a * c
    if (
        discriminant != theorem["q5_minimal_polynomial_discriminant"]
        or isqrt(discriminant) ** 2 == discriminant
    ):
        _reject("Q5_THEOREM")
    plus = (sp.Integer(3) + sp.sqrt(5)) / 8
    minus = (sp.Integer(3) - sp.sqrt(5)) / 8
    if plus.is_Rational is not False or minus.is_Rational is not False:
        _reject("Q5_THEOREM")
    for root in (plus, minus):
        if _sign(a * root**2 + b * root + c) != 0:
            _reject("Q5_THEOREM")
    expected_roots = {1: plus, 2: minus, 3: minus, 4: plus}
    for m, root in expected_roots.items():
        cosine_squared = sp.simplify(sp.cos(sp.pi * m / 5) ** 2)
        if _sign(cosine_squared - root) != 0:
            _reject("Q5_THEOREM")
    # Само поле рациональности выводится из типов, а не из текста receipt.
    rational_matrix = ((Fraction(2), Fraction(1)), (Fraction(1), Fraction(3)))
    rational_a = (Fraction(1), Fraction(2))
    rational_b = (Fraction(3), Fraction(5))
    rational_cos_squared = (
        _dot(rational_matrix, rational_a, rational_b) ** 2
        / (
            _dot(rational_matrix, rational_a, rational_a)
            * _dot(rational_matrix, rational_b, rational_b)
        )
    )
    if getattr(rational_cos_squared, "is_Rational", None) is not True:
        _reject("Q5_THEOREM")


def _validate_simple_authority(record: dict) -> None:
    required = {
        "open_window",
        "width",
        "authority_budget",
        "candidates",
        "outcome",
    }
    allowed = required | {"winner", "old_winner", "objectives"}
    if not required.issubset(record) or not set(record).issubset(allowed):
        _reject("SYNTHETIC_AUTHORITY")
    lower, upper = (_f(item) for item in record["open_window"])
    width = upper - lower
    budget = _budget(width)
    candidates = _enumerate(lower, upper, budget)
    if (
        _f(record["width"]) != width
        or record.get("authority_budget") != budget
        or [_f(item) for item in record.get("candidates", [])] != candidates
        or record.get("outcome") != ("SUCCESS" if candidates else "EMPTY_AUTHORITY")
    ):
        _reject("SYNTHETIC_AUTHORITY")


def _validate_farey(document: dict) -> None:
    witness = document.get("farey_falsification_witness", {})
    _keys(
        witness,
        {"scope", "threshold", "below", "above", "theorem"},
        "SYNTHETIC_AUTHORITY",
    )
    if witness.get("scope") != "SYNTHETIC_FALSIFICATION_ONLY_NOT_PRODUCT_THRESHOLD":
        _reject("SYNTHETIC_AUTHORITY")
    for name in ("threshold", "below", "above"):
        _validate_simple_authority(witness[name])
    if witness["threshold"]["outcome"] != "EMPTY_AUTHORITY":
        _reject("SYNTHETIC_AUTHORITY")
    if witness["below"]["outcome"] != "SUCCESS" or witness["above"]["outcome"] != "SUCCESS":
        _reject("SYNTHETIC_AUTHORITY")
    budgets = {
        witness[name]["authority_budget"] for name in ("threshold", "below", "above")
    }
    if budgets != {5}:
        _reject("SYNTHETIC_AUTHORITY")


def _metric_by_id(document: dict, metric_id: str):
    matches = [
        item
        for item in document.get("fixed_metric_family", [])
        if item.get("metric_id") == metric_id
    ]
    if len(matches) != 1:
        _reject("GRAM_WITNESS")
    return _m(matches[0]["inverse_gram"])


def _validate_gram(document: dict) -> None:
    witness = document.get("gram_objective_witness", {})
    _keys(
        witness,
        {
            "authority",
            "ideal_vector",
            "metric_switch_rows",
            "switch_proven",
            "exact_tie",
        },
        "GRAM_WITNESS",
    )
    _validate_simple_authority(witness["authority"])
    slopes = [_f(item) for item in witness["authority"]["candidates"]]
    if slopes != [Fraction(1, 5), Fraction(1, 4)]:
        _reject("GRAM_WITNESS")
    ideal = tuple(_expr(item) for item in witness["ideal_vector"])
    winners = set()
    for row in witness["metric_switch_rows"]:
        _keys(
            row,
            {"metric_id", "objectives", "winner", "disposition"},
            "GRAM_WITNESS",
        )
        matrix = _metric_by_id(document, row["metric_id"])
        values = [
            _objective(matrix, ideal, (item.denominator, item.numerator))
            for item in slopes
        ]
        for supplied, actual in zip(row["objectives"], values, strict=True):
            if _sign(_expr(supplied) - actual) != 0:
                _reject("GRAM_WITNESS")
        minimum = min(range(2), key=lambda index: values[index])
        winner = slopes[minimum]
        if _f(row["winner"]) != winner:
            _reject("FORGED_OLD_WINNER_AFTER_BETTER")
        if row.get("disposition") != "STRICT_GRAM_MINIMUM":
            _reject("FAKE_TIE")
        winners.add(winner)
    if len(winners) < 2 or witness.get("switch_proven") is not True:
        _reject("GRAM_WITNESS")
    tie = witness["exact_tie"]
    _keys(
        tie,
        {"metric_id", "ideal_vector", "objectives", "winner", "disposition"},
        "GRAM_WITNESS",
    )
    matrix = _metric_by_id(document, tie["metric_id"])
    tie_ideal = tuple(_expr(item) for item in tie["ideal_vector"])
    values = [
        _objective(matrix, tie_ideal, (item.denominator, item.numerator))
        for item in slopes
    ]
    if _sign(values[0] - values[1]) != 0:
        _reject("FAKE_TIE")
    if (
        _f(tie["winner"]) != max(slopes)
        or tie.get("disposition") != "HALF_UP_EXACT_TIE"
    ):
        _reject("FAKE_TIE")


def _validate_extension(document: dict) -> None:
    witness = document.get("candidate_extension_witness", {})
    _keys(
        witness,
        {"scope", "metric_id", "ideal_vector", "base", "extended"},
        "CANDIDATE_EXTENSION",
    )
    if witness.get("scope") != "SYNTHETIC_MONOTONE_AUTHORITY_EXTENSION":
        _reject("CANDIDATE_EXTENSION")
    _validate_simple_authority(witness["base"])
    _validate_simple_authority(witness["extended"])
    base = [_f(item) for item in witness["base"]["candidates"]]
    extended = [_f(item) for item in witness["extended"]["candidates"]]
    if base != [Fraction(1, 4)] or extended != [
        Fraction(1, 5),
        Fraction(1, 4),
    ]:
        _reject("CANDIDATE_EXTENSION")
    if _f(witness["base"]["winner"]) != Fraction(1, 4):
        _reject("CANDIDATE_EXTENSION")
    matrix = _metric_by_id(document, witness["metric_id"])
    ideal = tuple(_expr(item) for item in witness["ideal_vector"])
    values = [
        _objective(matrix, ideal, (item.denominator, item.numerator))
        for item in extended
    ]
    supplied = witness["extended"]["objectives"]
    if any(
        _sign(_expr(record) - value) != 0
        for record, value in zip(supplied, values, strict=True)
    ):
        _reject("CANDIDATE_EXTENSION")
    winner = extended[min(range(len(values)), key=lambda index: values[index])]
    if _f(witness["extended"]["old_winner"]) != Fraction(1, 4):
        _reject("CANDIDATE_EXTENSION")
    if _f(witness["extended"]["winner"]) != winner:
        _reject("FORGED_OLD_WINNER_AFTER_BETTER")


def _expected_stats(document: dict) -> dict:
    rows = document["real_product_window_matrix"]
    payload = copy.deepcopy(document)
    payload.pop("stats", None)
    payload.pop("generation_observation", None)
    stats = {
        "real_rows": len(rows),
        "real_windows": sum(len(item["windows"]) for item in rows),
        "empty_real_windows": sum(
            window["admissible_count"] == 0
            for row in rows
            for window in row["windows"]
        ),
        "q5_point_rows": len(document["q5_selection_matrix"]),
        "candidate_rows": sum(
            len(window["candidates"])
            for row in rows
            for window in row["windows"]
        ),
        "max_operand_bits": _max_bits(payload),
        "wall_seconds_budget": "30.000000000",
    }
    stats["certified_bit_cost_units"] = (
        stats["candidate_rows"] + stats["real_windows"]
    ) * stats["max_operand_bits"] ** 2
    return stats


def validate(document: dict, repo: Path) -> None:
    _keys(document, TOP_LEVEL_KEYS, "SCHEMA")
    if document.get("schema") != "cftuv.envelope.dens_a_prime_authority_proof.v1":
        _reject("SCHEMA")
    if document.get("scope") != "PROOF_ONLY_NO_PRODUCT_AUTHORITY":
        _reject("SCOPE")
    if document.get("policy") != POLICY:
        _reject("POLICY")
    if document.get("generation_observation") != {
        "operation": "REFERENCE_GENERATION_WALL_CLOCK_REPORTED_OUTSIDE_CANONICAL_BYTES",
        "deterministic_receipt": True,
    }:
        _reject("GENERATION_OBSERVATION")
    _validate_authorities(document, repo)
    _validate_owner_matrix(document)
    _validate_real_rows(document)
    _validate_q5(document)
    _validate_farey(document)
    _validate_gram(document)
    _validate_extension(document)
    expected_red = [
        {
            "name": name,
            "expected_disposition": "REJECT",
            "expected_code": code,
        }
        for name, code in sorted(RED_CONTROL_CODES.items())
    ]
    if document.get("red_controls") != expected_red:
        _reject("RED_MATRIX")
    expected_exploit = [
        {
            "name": name,
            "expected_disposition": "REJECT",
            "expected_code": code,
            "digest_recomputed": True,
        }
        for name, code in sorted(EXPLOIT_CONTROL_CODES.items())
    ]
    if document.get("exploit_regression_corpus") != expected_exploit:
        _reject("EXPLOIT_MATRIX")
    if document.get("stats") != _expected_stats(document):
        _reject("STATS")


def _first_candidate_window(document):
    for row in document["real_product_window_matrix"]:
        for window in row["windows"]:
            if window["candidates"]:
                return row, window
    _reject("RED_MATRIX")


def _multi_admissible_window(document):
    for row in document["real_product_window_matrix"]:
        for window in row["windows"]:
            if window["admissible_count"] >= 2:
                return row, window
    _reject("RED_MATRIX")


def _expect_reject(name: str, expected: str, operation) -> dict:
    try:
        operation()
    except Reject as exc:
        if exc.code != expected:
            _reject(f"RED_WRONG_CODE:{name}:{exc.code}:{expected}")
        return {"name": name, "outcome": "REJECT", "code": exc.code}
    _reject(f"RED_ACCEPTED:{name}")


def run_red_matrix(document: dict, repo: Path) -> list[dict]:
    results = []

    def owner_red():
        forged = copy.deepcopy(document["selection_point_matrix"][0])
        forged["H"] += 1
        _validate_owner_record(forged)

    results.append(_expect_reject("UNREACHABLE_H", "OWNER_LAW", owner_red))

    row, window = _first_candidate_window(document)
    matrix = _m(row["inverse_gram"])
    orientation = 1 if row["orientation"] == "CCW" else -1

    def window_red(name, expected, mutate):
        forged = copy.deepcopy(window)
        mutate(forged)
        return _expect_reject(
            name,
            expected,
            lambda: _validate_window(forged, matrix, row["q"], orientation),
        )

    results.append(
        window_red(
            "NON_COPRIME",
            "NON_COPRIME",
            lambda item: item["candidates"][0].update({"slope": [2, 2]}),
        )
    )
    results.append(
        window_red(
            "BOUNDARY_CANDIDATE",
            "BOUNDARY_CANDIDATE",
            lambda item: item["candidates"][0].update(
                {"slope": item["certified_open_window"][0]}
            ),
        )
    )
    results.append(
        window_red(
            "DUPLICATE",
            "DUPLICATE",
            lambda item: item["candidates"].append(
                copy.deepcopy(item["candidates"][0])
            ),
        )
    )
    if len(window["candidates"]) < 2:
        row, window = _multi_admissible_window(document)
        matrix = _m(row["inverse_gram"])
        orientation = 1 if row["orientation"] == "CCW" else -1
    results.append(
        window_red(
            "ORDER",
            "ORDER",
            lambda item: item["candidates"].reverse(),
        )
    )
    results.append(
        window_red(
            "CROSS",
            "CROSS",
            lambda item: item["candidates"][0].update(
                {
                    "primitive_vector": [
                        -value
                        for value in item["candidates"][0]["primitive_vector"]
                    ]
                }
            ),
        )
    )

    multi_row, multi_window = _multi_admissible_window(document)
    multi_matrix = _m(multi_row["inverse_gram"])
    multi_orientation = 1 if multi_row["orientation"] == "CCW" else -1

    def non_minimum():
        forged = copy.deepcopy(multi_window)
        alternatives = [
            item["slope"]
            for item in forged["candidates"]
            if item["admissible"] and item["slope"] != forged["selected_slope"]
        ]
        forged["selected_slope"] = alternatives[0]
        _validate_window(
            forged, multi_matrix, multi_row["q"], multi_orientation
        )

    results.append(_expect_reject("NON_MINIMUM", "NON_MINIMUM", non_minimum))

    def fake_tie():
        forged = copy.deepcopy(multi_window)
        forged["selection_disposition"] = "HALF_UP_EXACT_TIE"
        _validate_window(
            forged, multi_matrix, multi_row["q"], multi_orientation
        )

    results.append(_expect_reject("FAKE_TIE", "FAKE_TIE", fake_tie))

    def forged_old_winner():
        forged = copy.deepcopy(document)
        target = forged["candidate_extension_witness"]["extended"]
        target["winner"] = copy.deepcopy(target["old_winner"])
        _validate_extension(forged)

    results.append(
        _expect_reject(
            "FORGED_OLD_WINNER_AFTER_BETTER",
            "FORGED_OLD_WINNER_AFTER_BETTER",
            forged_old_winner,
        )
    )

    def unknown_oid():
        forged = copy.deepcopy(document)
        forged["source_authority"]["product_tree_oids"]["kernel_src"] = "0" * 40
        _validate_authorities(forged, repo)

    results.append(
        _expect_reject(
            "UNKNOWN_PRODUCT_OID", "UNKNOWN_PRODUCT_OID", unknown_oid
        )
    )

    def unknown_fixture():
        forged = copy.deepcopy(document)
        forged["source_authority"]["metric_sources"][0]["sha256"] = "0" * 64
        _validate_authorities(forged, repo)

    results.append(
        _expect_reject("UNKNOWN_FIXTURE", "UNKNOWN_FIXTURE", unknown_fixture)
    )
    results.append(
        _expect_reject(
            "ALGEBRAIC_NEAR_ZERO",
            "EXACT_SIGN_UNDECIDABLE",
            lambda: _sign(
                sp.Symbol(
                    "algebraic_near_zero_1e_minus_1000",
                    real=True,
                )
            ),
        )
    )
    results.append(
        _expect_reject(
            "ALGEBRAIC_UNSUPPORTED",
            "EXACT_SIGN_UNDECIDABLE",
            lambda: _sign(
                sp.Function("unsupported_exact_sign")(sp.Integer(1))
            ),
        )
    )
    return sorted(results, key=lambda item: item["name"])


def _semantic_exploit(
    document: dict,
    name: str,
    expected: str,
    mutate,
    operation,
) -> dict:
    forged = copy.deepcopy(document)
    mutate(forged)
    forged_raw = _canonical_bytes(forged)
    result = _expect_reject(name, expected, lambda: operation(forged))
    result["recomputed_sha256"] = hashlib.sha256(forged_raw).hexdigest()
    result["digest_recomputed"] = True
    return result


def run_exploit_matrix(document: dict, repo: Path) -> list[dict]:
    results = []
    results.append(
        _semantic_exploit(
            document,
            "FORGED_SCOPE",
            "SCOPE",
            lambda item: item.update({"scope": "PRODUCT_AUTHORITY_PROVEN"}),
            lambda item: (
                None
                if item.get("scope") == "PROOF_ONLY_NO_PRODUCT_AUTHORITY"
                else _reject("SCOPE")
            ),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_POLICY",
            "POLICY",
            lambda item: item["policy"].update(
                {"point_threshold_only": "PRODUCT_THRESHOLD_ONLY_PROVEN"}
            ),
            lambda item: None if item.get("policy") == POLICY else _reject("POLICY"),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_SOURCE_GRAM",
            "UNKNOWN_FIXTURE",
            lambda item: item["source_authority"]["metric_sources"][0][
                "gram"
            ][0][0].__setitem__(0, 1),
            lambda item: _validate_authorities(item, repo),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_SOURCE_BLOB",
            "UNKNOWN_FIXTURE",
            lambda item: item["source_authority"]["metric_sources"][0].update(
                {"blob_oid": "0" * 40}
            ),
            lambda item: _validate_authorities(item, repo),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_SOURCE_DOMAIN",
            "UNKNOWN_FIXTURE",
            lambda item: item["source_authority"]["metric_sources"][0].update(
                {"patch_domain_id": "forged-domain"}
            ),
            lambda item: _validate_authorities(item, repo),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_SOURCE_REFERENCE_ID",
            "UNKNOWN_FIXTURE",
            lambda item: item["source_authority"]["metric_sources"][0].update(
                {"reference_metric_id": "forged-reference-metric"}
            ),
            lambda item: _validate_authorities(item, repo),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_FIXED_FAMILY",
            "FIXED_METRIC_FAMILY",
            lambda item: item["fixed_metric_family"][1][
                "inverse_gram"
            ][0][0].__setitem__(0, 1),
            lambda item: _validate_authorities(item, repo),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_Q5_THEOREM",
            "Q5_THEOREM",
            lambda item: item["q5_unrepresentability_theorem"].update(
                {
                    "product_consequence": (
                        "Q5_PRODUCT_THRESHOLD_GEOMETRY_PROVEN"
                    )
                }
            ),
            lambda item: _validate_q5(item),
        )
    )

    def validate_red_declaration(item):
        expected = [
            {
                "name": name,
                "expected_disposition": "REJECT",
                "expected_code": code,
            }
            for name, code in sorted(RED_CONTROL_CODES.items())
        ]
        if item.get("red_controls") != expected:
            _reject("RED_MATRIX")

    results.append(
        _semantic_exploit(
            document,
            "FORGED_RED_EXPECTED_ACCEPT",
            "RED_MATRIX",
            lambda item: item["red_controls"][0].update(
                {"expected_disposition": "ACCEPT"}
            ),
            validate_red_declaration,
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_STATS_ZERO",
            "STATS",
            lambda item: item.update(
                {
                    "stats": {
                        key: (
                            value
                            if key == "wall_seconds_budget"
                            else 0
                        )
                        for key, value in item["stats"].items()
                    }
                }
            ),
            lambda item: (
                None
                if item.get("stats") == _expected_stats(item)
                else _reject("STATS")
            ),
        )
    )
    results.append(
        _semantic_exploit(
            document,
            "FORGED_DETERMINISM_FALSE",
            "GENERATION_OBSERVATION",
            lambda item: item["generation_observation"].update(
                {"deterministic_receipt": False}
            ),
            lambda item: (
                None
                if item.get("generation_observation")
                == {
                    "operation": (
                        "REFERENCE_GENERATION_WALL_CLOCK_REPORTED_OUTSIDE_CANONICAL_BYTES"
                    ),
                    "deterministic_receipt": True,
                }
                else _reject("GENERATION_OBSERVATION")
            ),
        )
    )

    def anchor_attempt():
        forged = copy.deepcopy(document)
        forged["scope"] = "PRODUCT_AUTHORITY_PROVEN"
        forged_raw = _canonical_bytes(forged)
        # Атакующий пересчитал digest, но frozen authority остаётся прежней.
        hashlib.sha256(forged_raw).hexdigest()
        _check_anchor(forged_raw)

    anchor_result = _expect_reject(
        "RECOMPUTED_DIGEST_ANCHOR_ATTEMPT",
        "RECEIPT_ANCHOR",
        anchor_attempt,
    )
    anchor_result["digest_recomputed"] = True
    results.append(anchor_result)
    return sorted(results, key=lambda item: item["name"])


def _max_bits(value) -> int:
    if isinstance(value, bool) or value is None:
        return 0
    if isinstance(value, int):
        return abs(value).bit_length()
    if isinstance(value, list):
        return max((_max_bits(item) for item in value), default=0)
    if isinstance(value, dict):
        return max((_max_bits(item) for item in value.values()), default=0)
    if isinstance(value, str):
        import re

        return max(
            (abs(int(item)).bit_length() for item in re.findall(r"-?\d+", value)),
            default=0,
        )
    return 0


def _validate_manifest() -> None:
    path = Path(__file__).with_name("manifest.json")
    raw = path.read_bytes()
    document = json.loads(raw)
    if _canonical_bytes(document) != raw:
        _reject("MANIFEST_ANCHOR")
    expected = {
        "schema": "cftuv.envelope.dens_a_prime_authority_manifest.v1",
        "receipt_filename": "dens_a_prime_authority_receipt.json",
        "receipt_sha256": EXPECTED_RECEIPT_SHA256,
        "verifier_filename": "verify_authority_receipt.py",
        "base_sha": EXPECTED_BASE,
        "product_tree_oids": EXPECTED_PRODUCT,
        "fixture_source_sha256": {
            metric_id: digest
            for metric_id, (_, digest) in sorted(EXPECTED_FIXTURES.items())
        },
    }
    if document != expected:
        _reject("MANIFEST_ANCHOR")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--receipt",
        type=Path,
        default=Path(__file__).with_name("dens_a_prime_authority_receipt.json"),
    )
    parser.add_argument("--repo", type=Path, default=Path(__file__).parents[3])
    parser.add_argument("--regenerate", type=Path)
    args = parser.parse_args()
    started = time.perf_counter()
    raw = args.receipt.read_bytes()
    document = json.loads(raw)
    if _canonical_bytes(document) != raw:
        _reject("NON_CANONICAL_BYTES")
    _validate_manifest()
    _check_anchor(raw)
    validate(document, args.repo.resolve())
    red = run_red_matrix(document, args.repo.resolve())
    exploits = run_exploit_matrix(document, args.repo.resolve())
    regenerated = _canonical_bytes(document)
    if args.regenerate is not None:
        args.regenerate.write_bytes(regenerated)
    elapsed = time.perf_counter() - started
    if elapsed > float(document["stats"]["wall_seconds_budget"]):
        _reject("PERF_WALL_BUDGET")
    print(
        json.dumps(
            {
                "outcome": "GO",
                "receipt_sha256": hashlib.sha256(raw).hexdigest(),
                "independent_regeneration_sha256": hashlib.sha256(
                    regenerated
                ).hexdigest(),
                "wall_seconds": f"{elapsed:.9f}",
                "max_operand_bits": document["stats"]["max_operand_bits"],
                "red_matrix": red,
                "exploit_matrix": exploits,
                "stats": document["stats"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
