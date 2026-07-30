"""Proof-only probe for the SEM-CLB-02-R Gate C-prime factor basis.

The probe observes the accepted p011 product at the exact 75f product tree.
It does not replace factorisation in the product.  Instead it proves that every
dynamic conjugation radicand in the motorcycle and coverage operations is a
squarefree product of primes already present in the domain's primitive support
lines.
"""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from fractions import Fraction
import hashlib
import json
from math import prod
from pathlib import Path
import subprocess
import sys
import time
from typing import Any, Callable


SCHEMA = "cftuv.envelope.sem_clb_02_gate_c_prime_basis.v1"
PROOF_BASE_REVISION = "75fdf8b07a483ce57f62d2fa95fc033a3e019281"
EXPECTED_KERNEL_SRC_TREE = "49181785c45c5dd0600130eb7aec994ab83631d3"
FIXTURE_ID = "building_all_seams_patch_011_lost_resolved_v1"
EXPECTED_DIVISION_SCOPES = {"MOTORCYCLE": 18, "COVERAGE": 96}
EXPECTED_DIVISIONS_WITHOUT_PICK = (
    "COVERAGE:026",
    "COVERAGE:039",
    "COVERAGE:040",
    "COVERAGE:050",
    "COVERAGE:054",
    "COVERAGE:058",
    "COVERAGE:088",
    "COVERAGE:090",
)
EXPECTED_RATIONAL_DIVISORS = {
    "COVERAGE:026": 20692074496,
    "COVERAGE:039": -24520815600,
    "COVERAGE:040": 24520815600,
    "COVERAGE:050": 20692074496,
    "COVERAGE:054": 51628736512,
    "COVERAGE:058": 51628736512,
    "COVERAGE:088": 158822,
    "COVERAGE:090": 158822,
}
EXPECTED_PICK_ITERATIONS = {"MOTORCYCLE": 28, "COVERAGE": 168}
EXPECTED_RADICAND_OCCURRENCES = {"MOTORCYCLE": 50, "COVERAGE": 338}
EXPECTED_UNIQUE_RADICANDS = {"MOTORCYCLE": 11, "COVERAGE": 47}
EXPECTED_UNION_UNIQUE_RADICANDS = 53
EXPECTED_PRIMITIVE_Q_COUNT = 54
EXPECTED_DISTINCT_PRIMITIVE_Q_COUNT = 21
EXPECTED_PRIME_UNIVERSE_COUNT = 39
EXPECTED_HEAVY_CACHE_MISSES = 68
EXPECTED_HEAVY_CACHE_MISS_STAGES = {"PREPARE": 52, "COVERAGE": 16}
EXPECTED_COMPILE_CANONICAL_BYTES_LENGTH = 785100
EXPECTED_COMPILE_CANONICAL_SHA256 = (
    "ea9bbcd7f2ae8fba5bf60eb499b28c78c8bba5b160cb75c1e21ec12baca6f7b6"
)
EXPECTED_SKELETON_SEMANTIC_DIGEST = (
    "6683f9d2951b6ed27a7d90518ea7982c7eb8961a2d7976201fb39d87c481439b"
)
EXPECTED_PREPARE_CANONICAL_BYTES_LENGTH = 7136
EXPECTED_PREPARE_CANONICAL_SHA256 = (
    "3438d829e3afb53d03de3d65170232c9b6782799dbbbd511f6de42c90ea8205d"
)
EXPECTED_COVERAGE_CANONICAL_BYTES_LENGTH = 24828
EXPECTED_COVERAGE_CANONICAL_SHA256 = (
    "a2859ebb358ac38f61e0d048f4b92194578095c63adf5eb9155d9e4b89dcb067"
)
EXPECTED_COVERAGE_AREA_TERMS_LENGTH = 2046
EXPECTED_COVERAGE_AREA_TERMS_SHA256 = (
    "5f163fd1ca7ad92a85cdac88f0fcac51b6a0237a0ee12249d4166d40fa9066b8"
)
EXPECTED_POLYGON_DOUBLED_AREA = 3009689813344
EXPECTED_PREPARE_COUNTERS = {
    "CONVEYOR_AMBIGUOUS_OWNER_EDGES": 5,
    "CONVEYOR_ARRIVAL_LAWS": 32,
    "CONVEYOR_BOUND_FAN_DIRECTIONS": 22,
    "CONVEYOR_DEGRADED_MITER_CORNERS": 0,
    "CONVEYOR_DOMAIN_EDGES": 32,
    "CONVEYOR_DOMAIN_REGIONS": 1,
    "CONVEYOR_FACES": 54,
    "CONVEYOR_FAN_EDGES": 22,
    "CONVEYOR_FAN_SUPPORTS": 22,
    "CONVEYOR_LATTICE_SCALE": 131072,
    "CONVEYOR_MITERED_CORNERS": 0,
    "CONVEYOR_RATIONAL_VERTEX_FANS": 22,
    "CONVEYOR_RESCALED_ARRIVAL_LAWS": 43,
    "CONVEYOR_SKELETON_NODES": 62,
    "CONVEYOR_SOURCE_EDGES": 32,
    "CONVEYOR_WALL_EDGES": 0,
    "CONVEYOR_WEIGHTED_SOURCE_EDGES": 32,
}
EXPECTED_SKELETON_COUNTERS = {
    "coincident_split_targets": 6,
    "discarded_stale_candidates": 297,
    "edge_events": 15,
    "motorcycle_march_steps": 157,
    "motorcycle_trace_crashes": 9,
    "motorcycle_trace_pairs": 27,
    "motorcycle_traces": 22,
    "motorcycle_unbounded_traces": 0,
    "motorcycle_wall_crashes": 22,
    "motorcycle_wall_tests": 2278,
    "multi_participant_nodes": 10,
    "peaks": 10,
    "refused_filter_beyond_trace": 1307,
    "refused_filter_edge_is_own": 148,
    "refused_filter_event_in_the_past": 2826,
    "refused_filter_point_outside_front": 1223,
    "refused_filter_solo_vertex": 0,
    "refused_filter_span_does_not_collapse": 0,
    "refused_filter_span_is_born_zero": 22,
    "refused_filter_triple_never_concurrent": 824,
    "refused_no_rule_joint_is_antiparallel": 4,
    "refused_no_rule_joint_is_codirectional": 0,
    "refused_no_rule_joint_is_codirectional_at_different_speeds": 0,
    "refused_no_rule_meeting_not_reconnectable": 33,
    "refused_no_rule_span_vanished": 0,
    "refused_no_rule_triple_always_concurrent": 307,
    "resting_steiner_vertices": 9,
    "ridges": 33,
    "split_candidates_beyond_trace": 1307,
    "split_candidates_examined": 6648,
    "split_events": 47,
    "split_search_exhaustive_segments": 0,
    "split_search_exhaustive_vertices": 9,
    "start_events": 0,
    "switch_events": 0,
    "vertex_meeting_events": 0,
}
EXPECTED_COVERAGE_COUNTERS = {
    "CONVEYOR_COVERAGE_TERMS": 31,
    "CONVEYOR_COVERED_FACES": 54,
    "CONVEYOR_COVERED_REGIONS": 1,
    "CONVEYOR_NAMED_OWNERS": 27,
    "CONVEYOR_WALL_EDGES": 0,
}


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path(__file__).resolve().parents[3],
    )
    parser.add_argument(
        "--fixture-root",
        type=Path,
        default=Path(__file__).resolve().parent / "cases",
    )
    parser.add_argument("--output-report", type=Path, required=True)
    return parser.parse_args()


def _git(source_root: Path, *arguments: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(source_root.resolve()), *arguments],
        text=True,
    ).strip()


def _activate_source(source_root: Path):
    for module_name in tuple(sys.modules):
        if (
            module_name == "cftuv_envelope"
            or module_name.startswith("cftuv_envelope.")
        ):
            del sys.modules[module_name]
    source = str(source_root.resolve() / "kernel" / "src")
    if source in sys.path:
        sys.path.remove(source)
    sys.path.insert(0, source)
    import cftuv_envelope as kernel

    return kernel


def _canonical_json(payload: Any, *, pretty: bool = False) -> bytes:
    if pretty:
        text = json.dumps(
            payload,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
        )
        return (text + "\n").encode("utf-8")
    return json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _fraction(value: Fraction | int) -> list[int]:
    rational = Fraction(value)
    return [rational.numerator, rational.denominator]


def _sqrt_sum(value) -> list[list[Any]]:
    return [
        [radicand, _fraction(coefficient)]
        for radicand, coefficient in value.terms
    ]


def _q_integer_radicand(q: int | Fraction) -> int:
    value = Fraction(q)
    if value < 0:
        raise RuntimeError(f"negative primitive support q: {value}")
    if value == 0:
        return 0
    return value.numerator * value.denominator


def _prepare_projection(prepared, skeleton_digest: Callable[[Any], str]) -> dict:
    regions = []
    for region in prepared.regions:
        skeleton = region.skeleton
        partition = region.partition
        regions.append(
            {
                "region_id": region.region_id,
                "bridge_outcome": region.bridge_outcome.value,
                "skeleton_outcome": (
                    None
                    if region.skeleton_outcome is None
                    else region.skeleton_outcome.value
                ),
                "face_outcome": (
                    None
                    if region.face_outcome is None
                    else region.face_outcome.value
                ),
                "skeleton_digest": (
                    None if skeleton is None else skeleton_digest(skeleton)
                ),
                "skeleton_levels": None if skeleton is None else skeleton.levels,
                "skeleton_node_count": (
                    None if skeleton is None else len(skeleton.nodes)
                ),
                "skeleton_counters": (
                    [] if skeleton is None else [list(item) for item in skeleton.counters]
                ),
                "face_count": (
                    None if partition is None else len(partition.faces)
                ),
                "owner_by_edge": [
                    [list(edge), owner] for edge, owner in region.owner_by_edge
                ],
                "wall_spans": [list(edge) for edge in region.wall_spans],
            }
        )
    return {
        "outcome": prepared.outcome.value,
        "detail": prepared.detail,
        "lattice_scale": (
            None if prepared.lattice is None else prepared.lattice.scale
        ),
        "law_names": list(prepared.law_names),
        "counters": [list(item) for item in prepared.counters],
        "regions": regions,
    }


def _coverage_projection(coverage) -> dict:
    return {
        "outcome": coverage.outcome.value,
        "detail": coverage.detail,
        "alpha": _fraction(coverage.alpha),
        "lattice_alpha": _fraction(coverage.lattice_alpha),
        "doubled_area": _sqrt_sum(coverage.doubled_area),
        "polygon_doubled_area": coverage.polygon_doubled_area,
        "counters": [list(item) for item in coverage.counters],
        "regions": [
            {
                "region_id": region.region_id,
                "outcome": region.outcome.value,
                "doubled_area": _sqrt_sum(region.doubled_area),
                "polygon_doubled_area": region.polygon_doubled_area,
                "faces": [
                    {
                        "owner": list(face.owner),
                        "envelope_spec_id": face.envelope_spec_id,
                        "envelope_instance_id": face.envelope_instance_id,
                        "doubled_area": _sqrt_sum(face.doubled_area),
                    }
                    for face in region.faces
                ],
            }
            for region in coverage.regions
        ],
    }


class BasisProbe:
    """Scoped observer over the unchanged exact-square-root implementation."""

    def __init__(self, exact_module) -> None:
        self.exact = exact_module
        self.original_pick_prime = exact_module._pick_prime
        self.original_prime_support = exact_module.prime_support
        self.original_divide = exact_module.SqrtSumV1.__truediv__
        self.operation: str | None = None
        self.division_stack: list[str] = []
        self.division_counts = {"MOTORCYCLE": 0, "COVERAGE": 0}
        self.division_trace: list[dict[str, Any]] = []
        self.pick_counts = {"MOTORCYCLE": 0, "COVERAGE": 0}
        self.pick_by_division: dict[str, int] = {}
        self.pick_trace: list[dict[str, Any]] = []
        self.prime_support_inputs = {"MOTORCYCLE": 0, "COVERAGE": 0}
        self.observed_radicands = {"MOTORCYCLE": [], "COVERAGE": []}
        self.support_rows: dict[int, dict[str, Any]] = {}
        self.cache_miss_records: list[tuple[str, int]] = []
        self.cache_miss_seen: set[int] = set()
        self.stage: str | None = None
        self.universe: tuple[int, ...] = ()
        self.primitive_q_records: list[dict[str, Any]] = []
        self.primitive_q_histogram: list[dict[str, Any]] = []
        self.primitive_q_count = 0
        self.counters = {
            "fallback": 0,
            "unsupported": 0,
            "min_mismatch": 0,
            "prime_support_mismatch": 0,
            "non_squarefree": 0,
            "product_mismatch": 0,
            "pick_outside_division": 0,
        }

    @contextmanager
    def scoped(self, operation: str):
        previous = self.operation
        if previous is not None:
            raise RuntimeError(f"nested proof operation: {previous} -> {operation}")
        self.operation = operation
        try:
            yield
        finally:
            self.operation = previous

    @contextmanager
    def staged(self, stage: str):
        previous = self.stage
        if previous is not None:
            raise RuntimeError(f"nested proof stage: {previous} -> {stage}")
        self.stage = stage
        try:
            yield
        finally:
            self.stage = previous

    def build_universe(self, polygon) -> None:
        if self.universe:
            raise RuntimeError("primitive prime universe was built twice")
        q_values = [q for _, _, q in polygon.edges()]
        q_values.extend(line.q for _, _, line in polygon.fan_edges())
        self.primitive_q_count = len(q_values)
        histogram = Counter(Fraction(q) for q in q_values)
        self.primitive_q_histogram = [
            {
                "q": _fraction(q),
                "integer_radicand": _q_integer_radicand(q),
                "count": count,
            }
            for q, count in sorted(histogram.items())
        ]
        transformed = sorted({_q_integer_radicand(q) for q in q_values if q})
        primes: set[int] = set()
        for radicand in transformed:
            factors = self.exact._factorize(radicand)
            reconstructed = prod(
                prime ** power for prime, power in factors.items()
            )
            if reconstructed != radicand:
                raise RuntimeError(
                    f"old exact factorization did not reconstruct {radicand}"
                )
            primes.update(
                prime for prime, power in factors.items() if power % 2
            )
            self.primitive_q_records.append(
                {
                    "integer_radicand": radicand,
                    "factors": [
                        [prime, power]
                        for prime, power in sorted(factors.items())
                    ],
                }
            )
        self.universe = tuple(sorted(primes))
        if not self.universe:
            raise RuntimeError("primitive support-line prime universe is empty")

    def reconstruct_support(self, radicand: int) -> tuple[int, ...]:
        if radicand <= 1:
            return ()
        support = tuple(
            prime for prime in self.universe if radicand % prime == 0
        )
        if prod(support) != radicand:
            self.counters["fallback"] += 1
            self.counters["product_mismatch"] += 1
        if any(radicand % (prime * prime) == 0 for prime in support):
            self.counters["non_squarefree"] += 1
        return support

    def prime_support(self, radicand: int) -> tuple[int, ...]:
        operation = self.operation
        if operation is not None:
            self.prime_support_inputs[operation] += 1
        before = self.original_prime_support.cache_info()
        current = self.original_prime_support(radicand)
        after = self.original_prime_support.cache_info()
        if after.misses == before.misses + 1:
            if radicand in self.cache_miss_seen:
                raise RuntimeError(f"repeat cache miss for {radicand}")
            self.cache_miss_seen.add(radicand)
            self.cache_miss_records.append((self.stage or "UNSCOPED", radicand))
        if (
            operation is not None
        ):
            reconstructed = self.reconstruct_support(radicand)
            if current != reconstructed:
                self.counters["prime_support_mismatch"] += 1
            row = {
                "radicand": radicand,
                "current_support": list(current),
                "reconstructed_support": list(reconstructed),
            }
            prior = self.support_rows.setdefault(radicand, row)
            if prior != row:
                raise RuntimeError(
                    f"non-deterministic support observation for {radicand}"
                )
        return current

    def pick_prime(self, terms: dict[int, Fraction]) -> int | None:
        operation = self.operation
        if operation is None:
            return self.original_pick_prime(terms)
        self.pick_counts[operation] += 1
        division_id = self.division_stack[-1] if self.division_stack else None
        if division_id is None:
            self.counters["pick_outside_division"] += 1
        else:
            self.pick_by_division[division_id] = (
                self.pick_by_division.get(division_id, 0) + 1
            )

        # Сначала исполняется продукт ровно в прежнем порядке. Проверяющий
        # путь не имеет права прогреть его unbounded cache до настоящего вызова.
        current_min = self.original_pick_prime(terms)
        reconstructed_primes: set[int] = set()
        for radicand, coefficient in terms.items():
            if not coefficient or radicand == 1:
                continue
            if radicand < 1:
                self.counters["unsupported"] += 1
                continue
            self.observed_radicands[operation].append(radicand)
            reconstructed_primes.update(self.reconstruct_support(radicand))
        reconstructed_min = (
            min(reconstructed_primes) if reconstructed_primes else None
        )
        if current_min != reconstructed_min:
            self.counters["min_mismatch"] += 1
        self.pick_trace.append(
            {
                "operation": operation,
                "division_id": division_id,
                "division_iteration": (
                    None
                    if division_id is None
                    else self.pick_by_division[division_id]
                ),
                "radicands": sorted(
                    radicand
                    for radicand, coefficient in terms.items()
                    if coefficient and radicand > 1
                ),
                "current_min": current_min,
                "reconstructed_min": reconstructed_min,
            }
        )
        return current_min

    def divide(self, left, right):
        operation = self.operation
        if operation is None:
            return self.original_divide(left, right)
        self.division_counts[operation] += 1
        division_id = f"{operation}:{self.division_counts[operation]:03d}"
        rational = right.as_rational()
        self.division_trace.append(
            {
                "division_id": division_id,
                "operation": operation,
                "divisor_terms": _sqrt_sum(right),
                "divisor_radicands": [
                    radicand
                    for radicand, coefficient in right.terms
                    if coefficient and radicand > 1
                ],
                "divisor_is_rational": rational is not None,
                "divisor_rational": (
                    None if rational is None else _fraction(rational)
                ),
            }
        )
        self.division_stack.append(division_id)
        try:
            return self.original_divide(left, right)
        finally:
            popped = self.division_stack.pop()
            if popped != division_id:
                raise RuntimeError("division scope stack was corrupted")

    def receipt(self) -> dict[str, Any]:
        observed = {
            operation: {
                "count": len(values),
                "unique_count": len(set(values)),
                "unique": sorted(set(values)),
            }
            for operation, values in self.observed_radicands.items()
        }
        divisions_without_picks = [
            f"{operation}:{ordinal:03d}"
            for operation, count in self.division_counts.items()
            for ordinal in range(1, count + 1)
            if f"{operation}:{ordinal:03d}" not in self.pick_by_division
        ]
        return {
            "primitive_support_line_q_count": self.primitive_q_count,
            "primitive_support_line_q_histogram": self.primitive_q_histogram,
            "distinct_primitive_support_line_q_count": len(
                self.primitive_q_records
            ),
            "primitive_support_line_q": self.primitive_q_records,
            "prime_universe_count": len(self.universe),
            "prime_universe": list(self.universe),
            "division_scopes": dict(self.division_counts),
            "division_trace": self.division_trace,
            "pick_iterations": dict(self.pick_counts),
            "pick_iterations_by_division": [
                [key, value] for key, value in sorted(self.pick_by_division.items())
            ],
            "pick_trace": self.pick_trace,
            "divisions_without_pick": divisions_without_picks,
            "prime_support_inputs": dict(self.prime_support_inputs),
            "support_rows": [
                self.support_rows[radicand]
                for radicand in sorted(self.support_rows)
            ],
            "observed_radicands": observed,
            "cache_misses": len(self.cache_miss_records),
            "cache_misses_by_stage": {
                stage: sum(
                    1 for record_stage, _ in self.cache_miss_records
                    if record_stage == stage
                )
                for stage in ("PREPARE", "COVERAGE")
            },
            "cache_miss_radicands": [
                [stage, radicand]
                for stage, radicand in sorted(self.cache_miss_records)
            ],
            "heavy_definition": (
                "first global product prime_support evaluation of a nontrivial "
                "radicand in the fresh p011 prepare+coverage process; on 75f "
                "this is exactly an uncached call to old exact _factorize. "
                "It is intentionally broader than the two target operation "
                "scopes: PREPARE has 52 misses and COVERAGE adds 16"
            ),
            "heavy_count": len(self.cache_miss_records),
            "counters": dict(self.counters),
        }


def _install_probe(
    probe: BasisProbe,
    skeleton_module,
    motorcycle_module,
    conveyor_module,
):
    original_motorcycle = skeleton_module.build_motorcycle_graph
    original_as_sqrt_sum = motorcycle_module._as_sqrt_sum
    original_coverage = conveyor_module.coverage_at
    exact_module = probe.exact

    def observed_motorcycle(polygon):
        probe.build_universe(polygon)
        return original_motorcycle(polygon)

    def observed_as_sqrt_sum(time_value):
        with probe.scoped("MOTORCYCLE"):
            return original_as_sqrt_sum(time_value)

    def observed_coverage(partition, alpha):
        with probe.scoped("COVERAGE"):
            return original_coverage(partition, alpha)

    def observed_divide(left, right):
        return probe.divide(left, right)

    skeleton_module.build_motorcycle_graph = observed_motorcycle
    motorcycle_module._as_sqrt_sum = observed_as_sqrt_sum
    conveyor_module.coverage_at = observed_coverage
    exact_module._pick_prime = probe.pick_prime
    exact_module.prime_support = probe.prime_support
    exact_module.SqrtSumV1.__truediv__ = observed_divide

    def restore() -> None:
        skeleton_module.build_motorcycle_graph = original_motorcycle
        motorcycle_module._as_sqrt_sum = original_as_sqrt_sum
        conveyor_module.coverage_at = original_coverage
        exact_module._pick_prime = probe.original_pick_prime
        exact_module.prime_support = probe.original_prime_support
        exact_module.SqrtSumV1.__truediv__ = probe.original_divide

    return restore


def _fail_closed(probe_receipt: dict[str, Any]) -> None:
    if probe_receipt["division_scopes"] != EXPECTED_DIVISION_SCOPES:
        raise RuntimeError(
            "division scope mismatch: "
            f"{probe_receipt['division_scopes']} != {EXPECTED_DIVISION_SCOPES}"
        )
    if probe_receipt["pick_iterations"] != EXPECTED_PICK_ITERATIONS:
        raise RuntimeError(
            "pick iteration mismatch: "
            f"{probe_receipt['pick_iterations']} != {EXPECTED_PICK_ITERATIONS}"
        )
    if (
        len(probe_receipt["pick_trace"])
        != sum(EXPECTED_PICK_ITERATIONS.values())
        or any(
            row["current_min"] != row["reconstructed_min"]
            for row in probe_receipt["pick_trace"]
        )
    ):
        raise RuntimeError("stepwise pick trace is incomplete or disagrees")
    occurrences = {
        operation: record["count"]
        for operation, record in probe_receipt["observed_radicands"].items()
    }
    if occurrences != EXPECTED_RADICAND_OCCURRENCES:
        raise RuntimeError(
            "radicand occurrence mismatch: "
            f"{occurrences} != {EXPECTED_RADICAND_OCCURRENCES}"
        )
    unique = {
        operation: record["unique_count"]
        for operation, record in probe_receipt["observed_radicands"].items()
    }
    if unique != EXPECTED_UNIQUE_RADICANDS:
        raise RuntimeError(
            f"unique radicand mismatch: {unique} != {EXPECTED_UNIQUE_RADICANDS}"
        )
    union_unique = len(
        {
            radicand
            for record in probe_receipt["observed_radicands"].values()
            for radicand in record["unique"]
        }
    )
    if union_unique != EXPECTED_UNION_UNIQUE_RADICANDS:
        raise RuntimeError(
            "union unique radicand mismatch: "
            f"{union_unique} != {EXPECTED_UNION_UNIQUE_RADICANDS}"
        )
    if (
        len(probe_receipt["support_rows"])
        != EXPECTED_UNION_UNIQUE_RADICANDS
        or any(
            row["current_support"] != row["reconstructed_support"]
            for row in probe_receipt["support_rows"]
        )
    ):
        raise RuntimeError("stepwise support rows are incomplete or disagree")
    if (
        probe_receipt["primitive_support_line_q_count"]
        != EXPECTED_PRIMITIVE_Q_COUNT
        or probe_receipt["distinct_primitive_support_line_q_count"]
        != EXPECTED_DISTINCT_PRIMITIVE_Q_COUNT
    ):
        raise RuntimeError("primitive support-line q counts differ")
    if (
        sum(
            row["count"]
            for row in probe_receipt["primitive_support_line_q_histogram"]
        )
        != EXPECTED_PRIMITIVE_Q_COUNT
    ):
        raise RuntimeError("primitive support-line q histogram does not reconcile")
    if probe_receipt["prime_universe_count"] != EXPECTED_PRIME_UNIVERSE_COUNT:
        raise RuntimeError(
            "prime universe size mismatch: "
            f"{probe_receipt['prime_universe_count']} "
            f"!= {EXPECTED_PRIME_UNIVERSE_COUNT}"
        )
    if tuple(probe_receipt["divisions_without_pick"]) != (
        EXPECTED_DIVISIONS_WITHOUT_PICK
    ):
        raise RuntimeError(
            "rational division reconciliation mismatch: "
            f"{probe_receipt['divisions_without_pick']} "
            f"!= {EXPECTED_DIVISIONS_WITHOUT_PICK}"
        )
    division_by_id = {
        row["division_id"]: row for row in probe_receipt["division_trace"]
    }
    if any(
        not division_by_id[division_id]["divisor_is_rational"]
        or division_by_id[division_id]["divisor_radicands"]
        or division_by_id[division_id]["divisor_terms"]
        != [[1, [value, 1]]]
        for division_id, value in EXPECTED_RATIONAL_DIVISORS.items()
    ):
        raise RuntimeError("zero-pick divisions are not exactly rational")
    if probe_receipt["cache_misses"] != EXPECTED_HEAVY_CACHE_MISSES:
        raise RuntimeError(
            "heavy/cache-miss mismatch: "
            f"{probe_receipt['cache_misses']} != {EXPECTED_HEAVY_CACHE_MISSES}"
        )
    if (
        probe_receipt["cache_misses_by_stage"]
        != EXPECTED_HEAVY_CACHE_MISS_STAGES
    ):
        raise RuntimeError(
            "cache-miss stage mismatch: "
            f"{probe_receipt['cache_misses_by_stage']} "
            f"!= {EXPECTED_HEAVY_CACHE_MISS_STAGES}"
        )
    if any(probe_receipt["counters"].values()):
        raise RuntimeError(
            f"basis proof counters are nonzero: {probe_receipt['counters']}"
        )


def main() -> None:
    arguments = _arguments()
    source_root = arguments.source_root.resolve()
    revision = _git(source_root, "rev-parse", "HEAD")
    product_tree = _git(source_root, "rev-parse", "HEAD:kernel/src")
    if revision != PROOF_BASE_REVISION:
        raise SystemExit(
            f"UNKNOWN_PROOF_BASE_REVISION: {revision} != {PROOF_BASE_REVISION}"
        )
    if product_tree != EXPECTED_KERNEL_SRC_TREE:
        raise SystemExit(
            f"UNKNOWN_KERNEL_SRC_PRODUCT_TREE: {product_tree}"
        )

    fixture_dir = arguments.fixture_root.resolve() / FIXTURE_ID
    kernel = _activate_source(source_root)
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (fixture_dir / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (fixture_dir / "decal_request.json").read_bytes()
    )
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id
    compiled = kernel.compile_reference_envelopes(snapshot, request, domain_id)
    if compiled.outcome.value != "EXACT" or compiled.compilation is None:
        raise RuntimeError(f"p011 compile is not EXACT: {compiled.outcome.value}")
    compile_bytes = kernel.canonical_json_bytes(compiled.compilation)
    if (
        len(compile_bytes) != EXPECTED_COMPILE_CANONICAL_BYTES_LENGTH
        or _sha256(compile_bytes) != EXPECTED_COMPILE_CANONICAL_SHA256
    ):
        raise RuntimeError("p011 compile canonical bytes differ from external 75f")

    import cftuv_envelope.exact_sqrt_sum as exact_module
    import cftuv_envelope.wavefront.conveyor as conveyor_module
    import cftuv_envelope.wavefront.digest as digest_module
    import cftuv_envelope.wavefront.motorcycle as motorcycle_module
    import cftuv_envelope.wavefront.skeleton as skeleton_module

    probe = BasisProbe(exact_module)
    restore = _install_probe(
        probe, skeleton_module, motorcycle_module, conveyor_module
    )
    started = time.perf_counter()
    try:
        with probe.staged("PREPARE"):
            prepared = conveyor_module.prepare_conveyor(
                snapshot, request, patch_domain_id=domain_id
            )
        with probe.staged("COVERAGE"):
            coverage = conveyor_module.conveyor_coverage(prepared)
    finally:
        restore()
    elapsed = time.perf_counter() - started

    if prepared.outcome.value != "EXACT":
        raise RuntimeError(f"p011 prepare is not EXACT: {prepared.outcome.value}")
    if coverage.outcome.value != "EXACT":
        raise RuntimeError(f"p011 coverage is not EXACT: {coverage.outcome.value}")
    if prepared.compilation is None:
        raise RuntimeError("p011 prepared compilation is absent")
    prepared_compile_bytes = kernel.canonical_json_bytes(prepared.compilation)
    if prepared_compile_bytes != compile_bytes:
        raise RuntimeError("compile canonical bytes changed during observed run")
    if dict(prepared.counters) != EXPECTED_PREPARE_COUNTERS:
        raise RuntimeError("p011 prepare counters differ from frozen 75f baseline")
    region = prepared.regions[0]
    if region.skeleton is None:
        raise RuntimeError("p011 skeleton is absent")
    if region.skeleton.outcome.value != "EXACT":
        raise RuntimeError(
            f"p011 skeleton is not EXACT: {region.skeleton.outcome.value}"
        )
    if len(region.skeleton.nodes) != 62 or region.skeleton.levels != 218:
        raise RuntimeError(
            "p011 skeleton size differs from frozen 75f baseline"
        )
    if dict(region.skeleton.counters) != EXPECTED_SKELETON_COUNTERS:
        raise RuntimeError(
            "p011 skeleton counters differ from frozen 75f baseline"
        )
    if region.partition is None or len(region.partition.faces) != 54:
        raise RuntimeError("p011 face count differs from frozen 75f baseline")
    if dict(coverage.counters) != EXPECTED_COVERAGE_COUNTERS:
        raise RuntimeError("p011 coverage counters differ from frozen 75f baseline")
    skeleton_digest = digest_module.semantic_digest(region.skeleton)
    if skeleton_digest != EXPECTED_SKELETON_SEMANTIC_DIGEST:
        raise RuntimeError("p011 skeleton digest differs from external 75f")

    prepare_projection = _prepare_projection(
        prepared, digest_module.semantic_digest
    )
    coverage_projection = _coverage_projection(coverage)
    prepare_bytes = _canonical_json(prepare_projection)
    coverage_bytes = _canonical_json(coverage_projection)
    area_terms_bytes = _canonical_json(_sqrt_sum(coverage.doubled_area))
    if (
        len(prepare_bytes) != EXPECTED_PREPARE_CANONICAL_BYTES_LENGTH
        or _sha256(prepare_bytes) != EXPECTED_PREPARE_CANONICAL_SHA256
    ):
        raise RuntimeError("p011 prepare canonical bytes differ from external 75f")
    if (
        len(coverage_bytes) != EXPECTED_COVERAGE_CANONICAL_BYTES_LENGTH
        or _sha256(coverage_bytes) != EXPECTED_COVERAGE_CANONICAL_SHA256
    ):
        raise RuntimeError("p011 coverage canonical bytes differ from external 75f")
    if (
        len(area_terms_bytes) != EXPECTED_COVERAGE_AREA_TERMS_LENGTH
        or _sha256(area_terms_bytes) != EXPECTED_COVERAGE_AREA_TERMS_SHA256
        or len(coverage.doubled_area.terms) != 31
        or coverage.polygon_doubled_area != EXPECTED_POLYGON_DOUBLED_AREA
    ):
        raise RuntimeError("p011 coverage area differs from external 75f")
    probe_receipt = probe.receipt()
    _fail_closed(probe_receipt)
    payload = {
        "schema": SCHEMA,
        "outcome": "EXACT_DOMAIN_PRIME_BASIS_PROVEN",
        "proof_base_revision": revision,
        "kernel_src_product_tree_oid": product_tree,
        "fixture_id": FIXTURE_ID,
        "fixture_files": {
            name: _sha256((fixture_dir / name).read_bytes())
            for name in (
                "analysis_snapshot.json",
                "decal_request.json",
                "host_mesh.json",
                "manifest.json",
            )
        },
        "product_preservation": {
            "compile_outcome": compiled.outcome.value,
            "compile_canonical_bytes_length": len(compile_bytes),
            "compile_canonical_sha256": _sha256(compile_bytes),
            "prepare_outcome": prepared.outcome.value,
            "prepare_canonical_bytes_length": len(prepare_bytes),
            "prepare_canonical_sha256": _sha256(prepare_bytes),
            "prepare_counters": [list(item) for item in prepared.counters],
            "skeleton_outcome": region.skeleton.outcome.value,
            "skeleton_semantic_digest": skeleton_digest,
            "skeleton_levels": region.skeleton.levels,
            "skeleton_node_count": len(region.skeleton.nodes),
            "skeleton_counters": [
                list(item) for item in region.skeleton.counters
            ],
            "face_count": len(region.partition.faces),
            "coverage_outcome": coverage.outcome.value,
            "coverage_canonical_bytes_length": len(coverage_bytes),
            "coverage_canonical_sha256": _sha256(coverage_bytes),
            "coverage_area_terms_bytes_length": len(area_terms_bytes),
            "coverage_area_terms_sha256": _sha256(area_terms_bytes),
            "coverage_counters": [list(item) for item in coverage.counters],
        },
        "basis_probe": probe_receipt,
        "timing_policy": (
            "wall-clock seconds are printed beside the receipt hash but are "
            "excluded from canonical bytes so independent runs must match"
        ),
    }
    encoded = _canonical_json(payload, pretty=True)
    arguments.output_report.parent.mkdir(parents=True, exist_ok=True)
    arguments.output_report.write_bytes(encoded)
    print(
        json.dumps(
            {
                "outcome": payload["outcome"],
                "receipt_sha256": _sha256(encoded),
                "wall_seconds": elapsed,
                "division_scopes": probe_receipt["division_scopes"],
                "pick_iterations": probe_receipt["pick_iterations"],
                "observed_radicands": {
                    key: {
                        "count": value["count"],
                        "unique_count": value["unique_count"],
                    }
                    for key, value in probe_receipt[
                        "observed_radicands"
                    ].items()
                },
                "cache_misses": probe_receipt["cache_misses"],
                "heavy_count": probe_receipt["heavy_count"],
                "counters": probe_receipt["counters"],
            },
            sort_keys=True,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
