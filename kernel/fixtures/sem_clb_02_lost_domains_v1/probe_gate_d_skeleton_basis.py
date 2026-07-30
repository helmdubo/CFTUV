"""Proof-only Gate D probe for the p006 skeleton ``event_point`` divisions.

The product tree is left byte-for-byte at 75f.  The only product binding
replaced by this probe is ``wavefront.skeleton.event_point``.  For each call
through that binding the two coordinate divisions are replayed with a prime
universe frozen from the complete ordinary and fan support-line input before
the first scoped event.  The replay is compared to the unchanged product
division and checked by exact multiplication.
"""

from __future__ import annotations

import argparse
from collections import Counter
from fractions import Fraction
import hashlib
import json
from math import prod
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


SCHEMA = "cftuv.envelope.sem_clb_02_gate_d_skeleton_basis.v1"
PROOF_BASE_REVISION = "bff2260e6f24f05556d118b9cf9ba17d4df1b752"
PRODUCT_BASE_REVISION = "75fdf8b07a483ce57f62d2fa95fc033a3e019281"
EXPECTED_KERNEL_SRC_TREE = "49181785c45c5dd0600130eb7aec994ab83631d3"
FIXTURE_ID = "building_all_seams_patch_006_lost_resolved_v1"

EXPECTED_COMPILE_LENGTH = 871002
EXPECTED_COMPILE_SHA256 = (
    "d42164dea338aade99125704ebc576e2ac3fcca79c514954ef5a56109ad141f8"
)
EXPECTED_PREPARE_LENGTH = 7651
EXPECTED_PREPARE_SHA256 = (
    "2512c0390e6e3f67ae9c5ed605c12289e24f3b71a25ae15dc8b3a414b09ccc5e"
)
EXPECTED_SKELETON_DIGEST = (
    "d2e21b47e48bae7fbeb411f418fa216e9ae478b4523468a090ca869ea82e9280"
)
EXPECTED_COVERAGE_LENGTH = 32926
EXPECTED_COVERAGE_SHA256 = (
    "7e192d10d1b9c279dd0137df33479d2337fadb26cefae7e6462b6124d8351e02"
)
EXPECTED_AREA_TERMS_LENGTH = 2429
EXPECTED_AREA_TERMS_SHA256 = (
    "1bac89bf33ae1017c6edb7aa76ace1f86d5371c8ddd038566ad36259a7b36b5d"
)
EXPECTED_POLYGON_DOUBLED_AREA = 4917431057152
EXPECTED_PRIMITIVE_SUPPORT_LINES = 63
EXPECTED_PRIMITIVE_KIND_COUNTS = {"FAN": 26, "ORDINARY": 37}
EXPECTED_DISTINCT_PRIMITIVE_Q = 24
EXPECTED_PRIME_UNIVERSE_COUNT = 40
EXPECTED_EVENT_CALLS = 5265
EXPECTED_COORDINATE_DIVISIONS = 10530
EXPECTED_COORDINATE_COUNTS = {"x": 5265, "y": 5265}
EXPECTED_PICK_ITERATIONS = 18344
EXPECTED_SUPPORT_CHECKS = 31830
EXPECTED_UNIQUE_DYNAMIC_RADICANDS = 49
EXPECTED_UNIQUE_PICKED_PRIMES = 14
EXPECTED_PICKS_PER_DIVISION = {"0": 696, "1": 1340, "2": 8478, "3": 16}
EXPECTED_UNIQUE_DIVISOR_TERM_TUPLES = 2300
EXPECTED_UNIQUE_RESULT_TERM_TUPLES = 5089

EXPECTED_PREPARE_COUNTERS = {
    "CONVEYOR_AMBIGUOUS_OWNER_EDGES": 9,
    "CONVEYOR_ARRIVAL_LAWS": 37,
    "CONVEYOR_BOUND_FAN_DIRECTIONS": 26,
    "CONVEYOR_DEGRADED_MITER_CORNERS": 0,
    "CONVEYOR_DOMAIN_EDGES": 37,
    "CONVEYOR_DOMAIN_REGIONS": 1,
    "CONVEYOR_FACES": 63,
    "CONVEYOR_FAN_EDGES": 26,
    "CONVEYOR_FAN_SUPPORTS": 26,
    "CONVEYOR_LATTICE_SCALE": 524288,
    "CONVEYOR_MITERED_CORNERS": 0,
    "CONVEYOR_RATIONAL_VERTEX_FANS": 26,
    "CONVEYOR_RESCALED_ARRIVAL_LAWS": 51,
    "CONVEYOR_SKELETON_NODES": 73,
    "CONVEYOR_SOURCE_EDGES": 37,
    "CONVEYOR_WALL_EDGES": 0,
    "CONVEYOR_WEIGHTED_SOURCE_EDGES": 37,
}
EXPECTED_SKELETON_COUNTERS = {
    "coincident_split_targets": 11,
    "discarded_stale_candidates": 371,
    "edge_events": 16,
    "motorcycle_march_steps": 441,
    "motorcycle_trace_crashes": 11,
    "motorcycle_trace_pairs": 26,
    "motorcycle_traces": 26,
    "motorcycle_unbounded_traces": 0,
    "motorcycle_wall_crashes": 26,
    "motorcycle_wall_tests": 2659,
    "multi_participant_nodes": 9,
    "peaks": 9,
    "refused_filter_beyond_trace": 2345,
    "refused_filter_edge_is_own": 190,
    "refused_filter_event_in_the_past": 3854,
    "refused_filter_point_outside_front": 1463,
    "refused_filter_solo_vertex": 0,
    "refused_filter_span_does_not_collapse": 0,
    "refused_filter_span_is_born_zero": 26,
    "refused_filter_triple_never_concurrent": 717,
    "refused_no_rule_joint_is_antiparallel": 3,
    "refused_no_rule_joint_is_codirectional": 0,
    "refused_no_rule_joint_is_codirectional_at_different_speeds": 0,
    "refused_no_rule_meeting_not_reconnectable": 44,
    "refused_no_rule_span_vanished": 0,
    "refused_no_rule_triple_always_concurrent": 456,
    "resting_steiner_vertices": 11,
    "ridges": 43,
    "split_candidates_beyond_trace": 2345,
    "split_candidates_examined": 9035,
    "split_events": 57,
    "split_search_exhaustive_segments": 0,
    "split_search_exhaustive_vertices": 25,
    "start_events": 0,
    "switch_events": 0,
    "vertex_meeting_events": 0,
}
EXPECTED_COVERAGE_COUNTERS = {
    "CONVEYOR_COVERAGE_TERMS": 34,
    "CONVEYOR_COVERED_FACES": 63,
    "CONVEYOR_COVERED_REGIONS": 1,
    "CONVEYOR_NAMED_OWNERS": 28,
    "CONVEYOR_WALL_EDGES": 0,
}


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-root", type=Path, default=Path(__file__).resolve().parents[3]
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
        ["git", "-C", str(source_root), *arguments], text=True
    ).strip()


def _activate_source(source_root: Path):
    for module_name in tuple(sys.modules):
        if module_name == "cftuv_envelope" or module_name.startswith(
            "cftuv_envelope."
        ):
            del sys.modules[module_name]
    source = str(source_root / "kernel" / "src")
    if source in sys.path:
        sys.path.remove(source)
    sys.path.insert(0, source)
    import cftuv_envelope as kernel

    return kernel


def _canonical_json(payload: Any, *, pretty: bool = False) -> bytes:
    options = {
        "ensure_ascii": False,
        "sort_keys": True,
    }
    if pretty:
        text = json.dumps(payload, indent=2, **options)
        return (text + "\n").encode("utf-8")
    return json.dumps(payload, separators=(",", ":"), **options).encode("utf-8")


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _fraction(value: Fraction | int) -> list[int]:
    rational = Fraction(value)
    return [rational.numerator, rational.denominator]


def _sqrt_sum(value) -> list[list[Any]]:
    return [[radicand, _fraction(coefficient)] for radicand, coefficient in value.terms]


def _line(line) -> list[Any]:
    return [line.a, line.b, line.c, _fraction(line.q)]


def _prepare_projection(prepared, skeleton_digest) -> dict[str, Any]:
    return {
        "outcome": prepared.outcome.value,
        "detail": prepared.detail,
        "lattice_scale": None if prepared.lattice is None else prepared.lattice.scale,
        "law_names": list(prepared.law_names),
        "counters": [list(item) for item in prepared.counters],
        "regions": [
            {
                "region_id": region.region_id,
                "bridge_outcome": region.bridge_outcome.value,
                "skeleton_outcome": (
                    None
                    if region.skeleton_outcome is None
                    else region.skeleton_outcome.value
                ),
                "face_outcome": (
                    None if region.face_outcome is None else region.face_outcome.value
                ),
                "skeleton_digest": (
                    None if region.skeleton is None else skeleton_digest(region.skeleton)
                ),
                "skeleton_levels": (
                    None if region.skeleton is None else region.skeleton.levels
                ),
                "skeleton_node_count": (
                    None if region.skeleton is None else len(region.skeleton.nodes)
                ),
                "skeleton_counters": (
                    []
                    if region.skeleton is None
                    else [list(item) for item in region.skeleton.counters]
                ),
                "face_count": (
                    None if region.partition is None else len(region.partition.faces)
                ),
                "owner_by_edge": [
                    [list(edge), owner] for edge, owner in region.owner_by_edge
                ],
                "wall_spans": [list(edge) for edge in region.wall_spans],
            }
            for region in prepared.regions
        ],
    }


def _coverage_projection(coverage) -> dict[str, Any]:
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


class SkeletonBasisProbe:
    """Replays only divisions reached through ``skeleton.event_point``."""

    def __init__(self, exact_module) -> None:
        self.exact = exact_module
        self.original_divide = exact_module.SqrtSumV1.__truediv__
        self.original_pick = exact_module._pick_prime
        self.original_support = exact_module.prime_support
        self.universe: tuple[int, ...] = ()
        self.primitive_rows: tuple[dict[str, Any], ...] = ()
        self.universe_digest = ""
        self.event_calls = 0
        self.coordinate_divisions = 0
        self.coordinate_counts: Counter[str] = Counter()
        self.pick_iterations = 0
        self.picked_primes: set[int] = set()
        self.picks_per_division: Counter[int] = Counter()
        self.support_checks = 0
        self.unique_radicands: set[int] = set()
        self.dynamic_support_rows: dict[int, dict[str, Any]] = {}
        self.unique_divisor_terms: set[tuple[Any, ...]] = set()
        self.unique_result_terms: set[tuple[Any, ...]] = set()
        self.event_stream = hashlib.sha256()
        self.division_stream = hashlib.sha256()
        self.pick_stream = hashlib.sha256()
        self.quotient_stream = hashlib.sha256()
        self.event_stream_records = 0
        self.division_stream_records = 0
        self.pick_stream_records = 0
        self.quotient_stream_records = 0
        self.witnesses_by_pick_count: dict[int, dict[str, Any]] = {}
        self.first_event_witness: dict[str, Any] | None = None
        self.last_event_witness: dict[str, Any] | None = None
        self.active_event: dict[str, Any] | None = None
        self.failures = {
            "scope_leak": 0,
            "universe_missing": 0,
            "universe_rebuilt": 0,
            "factor_miss": 0,
            "factor_product_mismatch": 0,
            "non_squarefree_dynamic": 0,
            "support_mismatch": 0,
            "minimum_mismatch": 0,
            "quotient_mismatch": 0,
            "quotient_identity_mismatch": 0,
            "coordinate_count_mismatch": 0,
        }

    def freeze_universe(self, polygon) -> None:
        if self.universe:
            self.failures["universe_rebuilt"] += 1
            raise RuntimeError("prime universe was frozen twice")
        rows: list[dict[str, Any]] = []
        for ordinal, (start, end, q) in enumerate(polygon.edges()):
            rows.append(
                {
                    "kind": "ORDINARY",
                    "ordinal": ordinal,
                    "start": list(start),
                    "end": list(end),
                    "q": _fraction(q),
                }
            )
        for point, ordinal, line in polygon.fan_edges():
            rows.append(
                {
                    "kind": "FAN",
                    "ordinal": ordinal,
                    "point": list(point),
                    "line": _line(line),
                    "q": _fraction(line.q),
                }
            )
        rows.sort(key=lambda row: _canonical_json(row))
        integer_radicands = sorted(
            {
                Fraction(*row["q"]).numerator * Fraction(*row["q"]).denominator
                for row in rows
            }
        )
        factors_by_radicand: dict[int, dict[int, int]] = {}
        for radicand in integer_radicands:
            factors = self.exact._factorize(radicand) if radicand else {}
            if prod(prime**power for prime, power in factors.items()) != radicand:
                self.failures["factor_product_mismatch"] += 1
            factors_by_radicand[radicand] = factors
        primes: set[int] = set()
        expanded: list[dict[str, Any]] = []
        for row in rows:
            q = Fraction(*row["q"])
            if q < 0:
                raise RuntimeError(f"negative primitive q: {q}")
            radicand = q.numerator * q.denominator
            factors = factors_by_radicand[radicand]
            odd = [prime for prime, power in sorted(factors.items()) if power % 2]
            primes.update(odd)
            expanded.append(
                {
                    **row,
                    "canonical_integer_radicand_p_times_r": radicand,
                    "factorization": [
                        [prime, power] for prime, power in sorted(factors.items())
                    ],
                    "odd_exponent_primes": odd,
                }
            )
        self.primitive_rows = tuple(expanded)
        self.universe = tuple(sorted(primes))
        if not self.universe:
            raise RuntimeError("empty primitive prime universe")
        self.universe_digest = _sha256(
            _canonical_json(
                {
                    "primitive_support_lines": self.primitive_rows,
                    "prime_universe": self.universe,
                }
            )
        )

    @staticmethod
    def _feed(stream, payload: Any) -> None:
        encoded = _canonical_json(payload)
        stream.update(len(encoded).to_bytes(8, "big"))
        stream.update(encoded)

    def _reconstruct_support(self, radicand: int) -> tuple[int, ...]:
        support = tuple(prime for prime in self.universe if radicand % prime == 0)
        if prod(support) != radicand:
            self.failures["factor_miss"] += 1
        if any(radicand % (prime * prime) == 0 for prime in support):
            self.failures["non_squarefree_dynamic"] += 1
        return support

    def _optimized_divide(self, numerator, denominator, division) -> Any:
        current_numerator, current_denominator = numerator, denominator
        iteration = 0
        while True:
            rational = current_denominator.as_rational()
            if rational is not None:
                quotient = current_numerator.scaled(Fraction(1) / rational)
                division["terminal_rational"] = _fraction(rational)
                return quotient
            iteration += 1
            radicands = sorted(
                radicand
                for radicand, coefficient in current_denominator.terms
                if coefficient and radicand > 1
            )
            reconstructed: dict[int, tuple[int, ...]] = {}
            legacy: dict[int, tuple[int, ...]] = {}
            for radicand in radicands:
                self.support_checks += 1
                self.unique_radicands.add(radicand)
                reconstructed[radicand] = self._reconstruct_support(radicand)
                legacy[radicand] = self.original_support(radicand)
                if reconstructed[radicand] != legacy[radicand]:
                    self.failures["support_mismatch"] += 1
                row = {
                    "radicand": radicand,
                    "legacy_support": list(legacy[radicand]),
                    "reconstructed_support": list(reconstructed[radicand]),
                }
                prior = self.dynamic_support_rows.setdefault(radicand, row)
                if prior != row:
                    raise RuntimeError(
                        f"non-deterministic support row for {radicand}"
                    )
            legacy_min = self.original_pick(current_denominator.as_map())
            reconstructed_primes = {
                prime
                for support in reconstructed.values()
                for prime in support
            }
            reconstructed_min = (
                min(reconstructed_primes) if reconstructed_primes else None
            )
            if legacy_min != reconstructed_min:
                self.failures["minimum_mismatch"] += 1
            self.pick_iterations += 1
            self.picked_primes.add(reconstructed_min)
            iteration_record = {
                "event_call": self.event_calls + 1,
                "coordinate": division["coordinate"],
                "iteration": iteration,
                "denominator_terms": _sqrt_sum(current_denominator),
                "current_radicands": radicands,
                "legacy_support": [
                    [radicand, list(legacy[radicand])] for radicand in radicands
                ],
                "reconstructed_support": [
                    [radicand, list(reconstructed[radicand])]
                    for radicand in radicands
                ],
                "legacy_min": legacy_min,
                "reconstructed_min": reconstructed_min,
            }
            division["iterations"].append(iteration_record)
            self._feed(self.pick_stream, iteration_record)
            self.pick_stream_records += 1
            outside, inside = self.exact._split_by_prime(
                current_denominator.as_map(), reconstructed_min
            )
            root = self.exact.SqrtSumV1.radical(1, reconstructed_min)
            conjugate = self.exact.SqrtSumV1._from_map(outside) - (
                self.exact.SqrtSumV1._from_map(inside) * root
            )
            current_numerator = current_numerator * conjugate
            current_denominator = current_denominator * conjugate

    def divide(self, numerator, denominator):
        if self.active_event is None:
            self.failures["scope_leak"] += 1
            return self.original_divide(numerator, denominator)
        index = len(self.active_event["coordinate_divisions"])
        coordinate = ("x", "y")[index] if index < 2 else f"extra_{index}"
        division = {
            "event_call": self.event_calls + 1,
            "coordinate": coordinate,
            "raw_numerator_terms": _sqrt_sum(numerator),
            "raw_denominator_terms": _sqrt_sum(denominator),
            "iterations": [],
        }
        optimized = self._optimized_divide(numerator, denominator, division)
        legacy = self.original_divide(numerator, denominator)
        division["optimized_quotient_terms"] = _sqrt_sum(optimized)
        division["legacy_quotient_terms"] = _sqrt_sum(legacy)
        division["quotient_times_denominator_terms"] = _sqrt_sum(
            optimized * denominator
        )
        self._feed(
            self.division_stream,
            {
                "event_call": division["event_call"],
                "coordinate": coordinate,
                "raw_numerator_terms": division["raw_numerator_terms"],
                "raw_denominator_terms": division["raw_denominator_terms"],
            },
        )
        self.division_stream_records += 1
        self._feed(
            self.quotient_stream,
            {
                "event_call": division["event_call"],
                "coordinate": coordinate,
                "optimized_quotient_terms": division[
                    "optimized_quotient_terms"
                ],
                "legacy_quotient_terms": division["legacy_quotient_terms"],
                "quotient_times_denominator_terms": division[
                    "quotient_times_denominator_terms"
                ],
            },
        )
        self.quotient_stream_records += 1
        if optimized.terms != legacy.terms:
            self.failures["quotient_mismatch"] += 1
        if (optimized * denominator).terms != numerator.terms:
            self.failures["quotient_identity_mismatch"] += 1
        self.active_event["coordinate_divisions"].append(division)
        self.coordinate_divisions += 1
        self.coordinate_counts[coordinate] += 1
        self.picks_per_division[len(division["iterations"])] += 1
        self.unique_divisor_terms.add(denominator.terms)
        self.unique_result_terms.add(legacy.terms)
        self.witnesses_by_pick_count.setdefault(
            len(division["iterations"]), division
        )
        return legacy

    def event_point(self, original_event_point, first, second, event_time):
        if not self.universe:
            self.failures["universe_missing"] += 1
            raise RuntimeError("event_point entered before universe freeze")
        if self.active_event is not None:
            raise RuntimeError("nested skeleton event_point")
        record = {
            "event_call": self.event_calls + 1,
            "first": _line(first),
            "second": _line(second),
            "time_dividend": _fraction(event_time.dividend),
            "time_divisor_terms": _sqrt_sum(event_time.divisor),
            "coordinate_divisions": [],
        }
        self.active_event = record
        product_class = self.exact.SqrtSumV1

        def observed_divide(numerator, denominator):
            return self.divide(numerator, denominator)

        product_class.__truediv__ = observed_divide
        try:
            point = original_event_point(first, second, event_time)
        finally:
            product_class.__truediv__ = self.original_divide
            self.active_event = None
        if len(record["coordinate_divisions"]) != 2:
            self.failures["coordinate_count_mismatch"] += 1
        record["result"] = {"x": _sqrt_sum(point.x), "y": _sqrt_sum(point.y)}
        self._feed(self.event_stream, record)
        self.event_stream_records += 1
        bounded_witness = {
            "event_call": record["event_call"],
            "first": record["first"],
            "second": record["second"],
            "time_dividend": record["time_dividend"],
            "time_divisor_terms": record["time_divisor_terms"],
            "result": record["result"],
        }
        if self.first_event_witness is None:
            self.first_event_witness = bounded_witness
        self.last_event_witness = bounded_witness
        self.event_calls += 1
        return point

    def receipt(self) -> dict[str, Any]:
        return {
            "scope": "cftuv_envelope.wavefront.skeleton.event_point binding only",
            "primitive_support_lines": list(self.primitive_rows),
            "primitive_support_line_count": len(self.primitive_rows),
            "primitive_support_line_kind_counts": dict(
                sorted(Counter(row["kind"] for row in self.primitive_rows).items())
            ),
            "distinct_primitive_q_count": len(
                {tuple(row["q"]) for row in self.primitive_rows}
            ),
            "prime_universe": list(self.universe),
            "prime_universe_count": len(self.universe),
            "prime_universe_digest": self.universe_digest,
            "event_calls": self.event_calls,
            "coordinate_divisions": self.coordinate_divisions,
            "coordinate_counts": dict(sorted(self.coordinate_counts.items())),
            "pick_iterations": self.pick_iterations,
            "picked_primes": sorted(self.picked_primes),
            "picks_per_division": {
                str(key): value
                for key, value in sorted(self.picks_per_division.items())
            },
            "support_checks": self.support_checks,
            "unique_dynamic_radicand_count": len(self.unique_radicands),
            "unique_dynamic_radicands": sorted(self.unique_radicands),
            "dynamic_support_rows": [
                self.dynamic_support_rows[key]
                for key in sorted(self.dynamic_support_rows)
            ],
            "unique_divisor_term_tuple_count": len(self.unique_divisor_terms),
            "unique_result_term_tuple_count": len(self.unique_result_terms),
            "factor_misses": self.failures["factor_miss"],
            "ordered_streams": {
                "events": {
                    "records": self.event_stream_records,
                    "sha256": self.event_stream.hexdigest(),
                },
                "raw_coordinate_divisions": {
                    "records": self.division_stream_records,
                    "sha256": self.division_stream.hexdigest(),
                },
                "conjugation_picks": {
                    "records": self.pick_stream_records,
                    "sha256": self.pick_stream.hexdigest(),
                },
                "quotient_checks": {
                    "records": self.quotient_stream_records,
                    "sha256": self.quotient_stream.hexdigest(),
                },
            },
            "bounded_witnesses": {
                "first_event": self.first_event_witness,
                "last_event": self.last_event_witness,
                "first_division_by_pick_count": [
                    {
                        "pick_count": key,
                        "division": self.witnesses_by_pick_count[key],
                    }
                    for key in sorted(self.witnesses_by_pick_count)
                ],
            },
            "failure_counters": dict(self.failures),
        }


def _authoritative_polygon(
    conveyor_module, bridge_module, snapshot, request, domain_id
):
    inputs, refusal = conveyor_module._prepare_inputs(
        snapshot, request, domain_id, conveyor_module._Clock()
    )
    if inputs is None:
        raise RuntimeError(f"p006 independent input preparation refused: {refusal}")
    _, _, domain, reading, lattice, _ = inputs
    if len(domain.domain_regions) != 1:
        raise RuntimeError("p006 no longer has exactly one region")
    loops, issue = conveyor_module._region_loops(domain.domain_regions[0])
    if loops is None:
        raise RuntimeError(f"p006 region loops unavailable: {issue}")
    report = bridge_module.bridge_arrival_laws(
        loops,
        reading.laws,
        lattice=lattice,
        weighted_fronts=True,
        vertex_fans=tuple(
            bridge_module.VertexFanLawV1(fan.name, fan.point, fan.supports)
            for fan in reading.fans
        ),
    )
    if report.polygon is None:
        raise RuntimeError(f"p006 bridge did not produce polygon: {report.outcome}")
    return report.polygon


def _assert_product(
    kernel,
    digest_module,
    compiled,
    compile_bytes,
    prepared,
    coverage,
) -> dict[str, Any]:
    if compiled.outcome.value != "EXACT" or prepared.outcome.value != "EXACT":
        raise RuntimeError("p006 compile/prepare is not EXACT")
    if coverage.outcome.value != "EXACT":
        raise RuntimeError("p006 coverage is not EXACT")
    region = prepared.regions[0]
    skeleton = region.skeleton
    if skeleton is None or skeleton.outcome.value != "EXACT":
        raise RuntimeError("p006 skeleton is not EXACT")
    if region.partition is None or len(region.partition.faces) != 63:
        raise RuntimeError("p006 face partition differs")
    prepare_bytes = _canonical_json(
        _prepare_projection(prepared, digest_module.semantic_digest)
    )
    coverage_bytes = _canonical_json(_coverage_projection(coverage))
    area_bytes = _canonical_json(_sqrt_sum(coverage.doubled_area))
    checks = (
        (len(compile_bytes), EXPECTED_COMPILE_LENGTH, "compile length"),
        (_sha256(compile_bytes), EXPECTED_COMPILE_SHA256, "compile sha"),
        (len(prepare_bytes), EXPECTED_PREPARE_LENGTH, "prepare length"),
        (_sha256(prepare_bytes), EXPECTED_PREPARE_SHA256, "prepare sha"),
        (len(coverage_bytes), EXPECTED_COVERAGE_LENGTH, "coverage length"),
        (_sha256(coverage_bytes), EXPECTED_COVERAGE_SHA256, "coverage sha"),
        (len(area_bytes), EXPECTED_AREA_TERMS_LENGTH, "area length"),
        (_sha256(area_bytes), EXPECTED_AREA_TERMS_SHA256, "area sha"),
        (
            digest_module.semantic_digest(skeleton),
            EXPECTED_SKELETON_DIGEST,
            "skeleton digest",
        ),
        (skeleton.levels, 266, "skeleton levels"),
        (len(skeleton.nodes), 73, "skeleton node count"),
        (coverage.polygon_doubled_area, EXPECTED_POLYGON_DOUBLED_AREA, "polygon area"),
    )
    for observed, expected, name in checks:
        if observed != expected:
            raise RuntimeError(f"{name} differs: {observed} != {expected}")
    if dict(prepared.counters) != EXPECTED_PREPARE_COUNTERS:
        raise RuntimeError("p006 prepare counters differ")
    if dict(skeleton.counters) != EXPECTED_SKELETON_COUNTERS:
        raise RuntimeError("p006 event/skeleton counters differ")
    if dict(coverage.counters) != EXPECTED_COVERAGE_COUNTERS:
        raise RuntimeError("p006 coverage counters differ")
    if prepared.compilation is None:
        raise RuntimeError("p006 prepared compilation absent")
    if kernel.canonical_json_bytes(prepared.compilation) != compile_bytes:
        raise RuntimeError("p006 prepared compilation changed")
    return {
        "compile_outcome": compiled.outcome.value,
        "compile_canonical_bytes_length": len(compile_bytes),
        "compile_canonical_sha256": _sha256(compile_bytes),
        "prepare_outcome": prepared.outcome.value,
        "prepare_canonical_bytes_length": len(prepare_bytes),
        "prepare_canonical_sha256": _sha256(prepare_bytes),
        "prepare_counters": [list(item) for item in prepared.counters],
        "skeleton_outcome": skeleton.outcome.value,
        "skeleton_semantic_digest": digest_module.semantic_digest(skeleton),
        "skeleton_levels": skeleton.levels,
        "skeleton_node_count": len(skeleton.nodes),
        "skeleton_event_counters": [list(item) for item in skeleton.counters],
        "face_count": len(region.partition.faces),
        "coverage_outcome": coverage.outcome.value,
        "coverage_canonical_bytes_length": len(coverage_bytes),
        "coverage_canonical_sha256": _sha256(coverage_bytes),
        "coverage_area_terms_bytes_length": len(area_bytes),
        "coverage_area_terms_sha256": _sha256(area_bytes),
        "coverage_counters": [list(item) for item in coverage.counters],
    }


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
        raise SystemExit(f"UNKNOWN_KERNEL_SRC_PRODUCT_TREE: {product_tree}")

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
    if compiled.compilation is None:
        raise RuntimeError(f"p006 compile failed: {compiled.outcome.value}")
    compile_bytes = kernel.canonical_json_bytes(compiled.compilation)

    import cftuv_envelope.exact_sqrt_sum as exact_module
    import cftuv_envelope.wavefront.bridge as bridge_module
    import cftuv_envelope.wavefront.conveyor as conveyor_module
    import cftuv_envelope.wavefront.digest as digest_module
    import cftuv_envelope.wavefront.skeleton as skeleton_module

    probe = SkeletonBasisProbe(exact_module)
    polygon = _authoritative_polygon(
        conveyor_module, bridge_module, snapshot, request, domain_id
    )
    probe.freeze_universe(polygon)

    original_event_point = skeleton_module.event_point

    def observed_event_point(first, second, event_time):
        return probe.event_point(
            original_event_point, first, second, event_time
        )

    # Единственная подмена продуктового binding: motorcycle, event_time,
    # coverage и прочие деления остаются на исходных функциях.
    skeleton_module.event_point = observed_event_point
    started = time.perf_counter()
    try:
        prepared = conveyor_module.prepare_conveyor(
            snapshot, request, patch_domain_id=domain_id
        )
        coverage = conveyor_module.conveyor_coverage(prepared)
    finally:
        skeleton_module.event_point = original_event_point
        exact_module.SqrtSumV1.__truediv__ = probe.original_divide
    elapsed = time.perf_counter() - started

    product = _assert_product(
        kernel,
        digest_module,
        compiled,
        compile_bytes,
        prepared,
        coverage,
    )
    proof = probe.receipt()
    expected = {
        "primitive_support_line_count": EXPECTED_PRIMITIVE_SUPPORT_LINES,
        "primitive_support_line_kind_counts": EXPECTED_PRIMITIVE_KIND_COUNTS,
        "distinct_primitive_q_count": EXPECTED_DISTINCT_PRIMITIVE_Q,
        "prime_universe_count": EXPECTED_PRIME_UNIVERSE_COUNT,
        "event_calls": EXPECTED_EVENT_CALLS,
        "coordinate_divisions": EXPECTED_COORDINATE_DIVISIONS,
        "coordinate_counts": EXPECTED_COORDINATE_COUNTS,
        "pick_iterations": EXPECTED_PICK_ITERATIONS,
        "picks_per_division": EXPECTED_PICKS_PER_DIVISION,
        "support_checks": EXPECTED_SUPPORT_CHECKS,
        "unique_dynamic_radicand_count": EXPECTED_UNIQUE_DYNAMIC_RADICANDS,
        "unique_divisor_term_tuple_count": EXPECTED_UNIQUE_DIVISOR_TERM_TUPLES,
        "unique_result_term_tuple_count": EXPECTED_UNIQUE_RESULT_TERM_TUPLES,
    }
    for name, expected_value in expected.items():
        if proof[name] != expected_value:
            raise RuntimeError(
                f"Gate D {name} differs: {proof[name]} != {expected_value}"
            )
    if len(proof["picked_primes"]) != EXPECTED_UNIQUE_PICKED_PRIMES:
        raise RuntimeError("Gate D picked-prime count differs")
    if len(proof["dynamic_support_rows"]) != EXPECTED_UNIQUE_DYNAMIC_RADICANDS:
        raise RuntimeError("Gate D support rows do not reconcile")
    streams = proof["ordered_streams"]
    expected_stream_counts = {
        "events": EXPECTED_EVENT_CALLS,
        "raw_coordinate_divisions": EXPECTED_COORDINATE_DIVISIONS,
        "conjugation_picks": EXPECTED_PICK_ITERATIONS,
        "quotient_checks": EXPECTED_COORDINATE_DIVISIONS,
    }
    if any(
        streams[name]["records"] != count
        for name, count in expected_stream_counts.items()
    ):
        raise RuntimeError("Gate D ordered stream counts do not reconcile")
    if sum(
        int(pick_count) * count
        for pick_count, count in proof["picks_per_division"].items()
    ) != EXPECTED_PICK_ITERATIONS:
        raise RuntimeError("Gate D pick histogram does not reconcile")
    if set(probe.witnesses_by_pick_count) != {0, 1, 2, 3}:
        raise RuntimeError("Gate D bounded pick witnesses are incomplete")
    if any(proof["failure_counters"].values()):
        raise RuntimeError(
            f"Gate D failure counters are nonzero: {proof['failure_counters']}"
        )

    payload = {
        "schema": SCHEMA,
        "outcome": "EXACT_SKELETON_EVENT_POINT_PRIME_BASIS_PROVEN",
        "proof_base_revision": revision,
        "product_base_revision": PRODUCT_BASE_REVISION,
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
        "product_preservation": product,
        "skeleton_basis_probe": proof,
        "timing_policy": (
            "wall-clock seconds are printed beside the receipt hash but excluded "
            "from canonical bytes so independent processes must match"
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
                "event_calls": proof["event_calls"],
                "coordinate_divisions": proof["coordinate_divisions"],
                "pick_iterations": proof["pick_iterations"],
                "support_checks": proof["support_checks"],
                "unique_dynamic_radicands": proof[
                    "unique_dynamic_radicand_count"
                ],
                "prime_universe_count": proof["prime_universe_count"],
                "prime_universe_digest": proof["prime_universe_digest"],
                "failure_counters": proof["failure_counters"],
            },
            sort_keys=True,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
