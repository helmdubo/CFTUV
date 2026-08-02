"""Красные ворота P0-2b: exact-time component применяется транзакционно."""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from cftuv_envelope.wavefront.event_time import EventPointV1, ZERO_TIME
from cftuv_envelope.wavefront.events import (
    CandidateEventV1,
    EventKind,
    EventQueueV1,
)
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.proof import (
    ProofObligationBranch,
    ProofObligationDisposition,
)
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront import superlevel as superlevel_module
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from wavefront_cases import named_corpus, partial_source_corpus


_REPRO_PATH = (
    Path(__file__).parents[2]
    / "artifacts"
    / "p0_2_same_time_closure"
    / "repro_superlevel_order_dependence.py"
)
_SPEC = importlib.util.spec_from_file_location("p0_2_d_repro", _REPRO_PATH)
assert _SPEC is not None and _SPEC.loader is not None
_REPRO = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_REPRO)

_CORPUS = dict(named_corpus()) | dict(partial_source_corpus())
_D_CASES = tuple(_REPRO.CASES)
_PRODUCT_AXIS_CASES = (
    "ell_12_source_edges_0_1",
    "ell_12_source_edges_4_5",
    "ell_12_source_without_the_reflex_pair",
    "staircase_source_edges_0_1",
    "staircase_source_edges_6_7",
)
_PRE_TRANSACTION_COVERAGE = {
    "ell_12_source_edges_0_1": (
        "dfcd1e1eed4f0991b326c2362ba4521b993c1b09283a18cda198b740a3d1e922",
        "018a5a576ad485ff864ab6729577ac525e28db8e1e975293b37c6a413a0b2e98",
        "5137ff2996c83b2dd1eab12ad240e3c910fdb74f152b909cff62cf8dbf461026",
    ),
    "ell_12_source_edges_4_5": (
        "9f6c130f24fa59b402255de2e9a3e5408899530b8454d03af45412d0d00d8108",
        "bbdb7fbee231a7c13d8dd2f9dd1222277f5a8007ba0d8bc6f185ad7de693512b",
        "c621290d02ff43bac9dabc36ad5c9d40e4b65f93bc1a3bf5fad50477098a725d",
    ),
    "ell_12_source_without_the_reflex_pair": (
        "fd20bd7950b754b64792133452091e1b0a929dd30754fdfcd1fc9b3935e4027d",
        "74cb5696b1b61d2b210481a72b089d5bc190cd9e5dc85b728dd93462e1f5cda1",
        "169ad2910f66dbb44328a21dcdd66c79de53d7755372955db73e3fef80bd5154",
    ),
    "staircase_source_edges_0_1": (
        "a6497bccf935014c358b8a0e3c19377f7d8f8e65f3b25e5d36f47b9b61e6cf86",
        "8aaa05830557a093fa9e0d9c8c2d4d25003ff5c2881a743fcdc2f0baa2e02ad7",
        "4b2d5d50deb6e3956c5cc51f6f293be8763fb0b701a5b04088e282ec39d07b27",
    ),
    "staircase_source_edges_6_7": (
        "bf30d3e8cc7be59a9ff72ebaae19a66c3a7bdff045b0da6fb51c5fc4a90d30b1",
        "cc8e7b99ae786af02cd94bef74a26ee12042b82a5944caa282bf1e25db176c40",
        "24b11d705594f4ac8c6a638d1a1d41afd59a326323d0551e8220dbcb9d299d34",
    ),
}

_PROJECTION_KEYS = (
    "outcome",
    "semantic_digest",
    "topology_after_each_superlevel",
    "state_after_each_superlevel",
    "partition_sha256",
    "partition_outcome",
    "ownership_sha256",
    "coverage_by_alpha",
    "coverage_sha256",
    "proof_status",
    "proof_verdict_sha256",
    "proof_geometric_projection",
    "partition_geometric_projection",
    "coverage_geometric_projection",
)
_INPUT_PROJECTION_KEYS = tuple(
    key
    for key in _PROJECTION_KEYS
    if key
    not in {
        "partition_sha256",
        "coverage_by_alpha",
        "coverage_sha256",
    }
)


def _event_order(event: CandidateEventV1) -> tuple:
    return (
        event.kind.value,
        event.point.x.terms,
        event.point.y.terms,
        event.vertex,
        event.peer,
        event.edge,
    )


def _edge_first(builder, level) -> None:
    """Исторический counterfactual до транзакции, permutation после неё."""

    transaction = getattr(
        superlevel_module, "apply_superlevel_transaction", None
    )
    if transaction is not None:
        ordered = tuple(
            sorted(
                level,
                key=lambda event: (
                    event.kind is not EventKind.EDGE,
                    _event_order(event),
                ),
            )
        )
        transaction(builder, ordered)
        return
    _REPRO._apply_level_edge_first(builder, level)


_PRODUCTION_APPLY = skeleton_module._Builder._apply_level


def _input_geometric(builder, level) -> None:
    _PRODUCTION_APPLY(builder, tuple(sorted(level, key=_event_order)))


def _input_geometric_reverse(builder, level) -> None:
    _PRODUCTION_APPLY(
        builder, tuple(sorted(level, key=_event_order, reverse=True))
    )


def _run(
    polygon: PolygonV1,
    *,
    apply=None,
    reverse_heap_packet: bool = False,
) -> dict:
    original_apply = skeleton_module._Builder._apply_level
    original_discharge = (
        skeleton_module._Builder._discharge_observed_obligations
    )
    original_pop = EventQueueV1.pop_level
    superlevels = []

    def observed_discharge(builder):
        original_discharge(builder)
        if not superlevel_module.has_same_time_residual(
            builder.queue, builder.now
        ):
            superlevels.append(_REPRO._superlevel_state(builder))

    def permuted_pop(queue):
        packet = original_pop(queue)
        return tuple(reversed(packet)) if reverse_heap_packet else packet

    skeleton_module._Builder._apply_level = apply or original_apply
    skeleton_module._Builder._discharge_observed_obligations = (
        observed_discharge
    )
    EventQueueV1.pop_level = permuted_pop
    try:
        skeleton = build_skeleton(polygon)
    finally:
        EventQueueV1.pop_level = original_pop
        skeleton_module._Builder._apply_level = original_apply
        skeleton_module._Builder._discharge_observed_obligations = (
            original_discharge
        )
    record = _REPRO._final_record(polygon, skeleton, superlevels)
    record["proof_geometric_projection"] = _REPRO._proof_projection(
        skeleton.proof_obligations
    )
    partition = build_faces(polygon, skeleton)
    partition_record = _REPRO._partition_record(partition)
    geometric_faces = []
    for face in partition_record["faces"]:
        geometric = dict(face)
        geometric.pop("start")
        geometric.pop("end")
        geometric_faces.append(geometric)
    geometric_faces.sort(key=repr)
    record["partition_geometric_projection"] = {
        key: value
        for key, value in partition_record.items()
        if key != "faces"
    } | {"faces": geometric_faces}
    coverage_records = []
    for alpha in (Fraction(1, 4), Fraction(1), Fraction(3)):
        coverage = _REPRO._coverage_record(coverage_at(partition, alpha))
        coverage["faces"].sort(key=repr)
        coverage_records.append(coverage)
    record["coverage_geometric_projection"] = coverage_records
    return record


def _input_permutations(polygon: PolygonV1) -> dict[str, PolygonV1]:
    variants = {
        "outer_rotate_1": PolygonV1(
            polygon.outer.rotated(1), polygon.holes, polygon.vertex_fans
        ),
        "outer_rotate_last": PolygonV1(
            polygon.outer.rotated(-1), polygon.holes, polygon.vertex_fans
        ),
    }
    if polygon.holes:
        variants["hole_rotate_1"] = PolygonV1(
            polygon.outer,
            tuple(hole.rotated(1) for hole in polygon.holes),
            polygon.vertex_fans,
        )
    if len(polygon.holes) > 1:
        variants["hole_order_reverse"] = PolygonV1(
            polygon.outer,
            tuple(reversed(polygon.holes)),
            polygon.vertex_fans,
        )
    return variants


def _differences(
    reference: dict,
    candidate: dict,
    keys: tuple[str, ...] = _PROJECTION_KEYS,
) -> tuple[str, ...]:
    return tuple(
        key for key in keys if reference[key] != candidate[key]
    )


@pytest.mark.parametrize("name", _D_CASES)
def test_d_config_is_transaction_invariant_under_every_schedule(name):
    polygon = _CORPUS[name]
    reference = _run(polygon)
    variants = {
        "edge_first": _run(polygon, apply=_edge_first),
        "heap_packet_reverse": _run(polygon, reverse_heap_packet=True),
        "input_geometric": _run(polygon, apply=_input_geometric),
        "input_geometric_reverse": _run(
            polygon, apply=_input_geometric_reverse
        ),
    }
    input_variants = {
        schedule: _run(candidate)
        for schedule, candidate in _input_permutations(polygon).items()
    }
    failures = {
        schedule: _differences(reference, candidate)
        for schedule, candidate in variants.items()
        if _differences(reference, candidate)
    }
    failures.update(
        {
            schedule: _differences(
                reference, candidate, _INPUT_PROJECTION_KEYS
            )
            for schedule, candidate in input_variants.items()
            if _differences(reference, candidate, _INPUT_PROJECTION_KEYS)
        }
    )
    assert failures == {}, (name, failures)


@pytest.mark.parametrize("name", _PRODUCT_AXIS_CASES)
def test_main_product_axes_converge_without_moving_coverage(name):
    polygon = _CORPUS[name]
    split_first = _run(polygon)
    edge_first = _run(polygon, apply=_edge_first)
    expected_coverage = _PRE_TRANSACTION_COVERAGE[name]
    assert tuple(
        item["sha256"] for item in split_first["coverage_by_alpha"]
    ) == expected_coverage
    assert tuple(
        item["sha256"] for item in edge_first["coverage_by_alpha"]
    ) == expected_coverage
    assert (
        split_first["partition_sha256"],
        split_first["ownership_sha256"],
    ) == (
        edge_first["partition_sha256"],
        edge_first["ownership_sha256"],
    )


@pytest.mark.skip(
    reason="AUTH_PENDING_STAIRCASE_3_4_T4_MIXED_REPLACEMENT"
)
def test_staircase_3_4_names_the_midrun_split_split_component(monkeypatch):
    planner = getattr(
        superlevel_module, "plan_superlevel_components", None
    )
    assert planner is not None, "SUPERLEVEL_TRANSACTION_PLANNER_UNAVAILABLE"
    observed = []

    def measured(incidents):
        components = planner(incidents)
        observed.extend(components)
        return components

    monkeypatch.setattr(
        superlevel_module, "plan_superlevel_components", measured
    )
    skeleton = build_skeleton(_CORPUS["staircase_source_edges_3_4"])
    split_split = [
        component
        for component in observed
        if component.event_kinds == (EventKind.SPLIT, EventKind.SPLIT)
    ]
    assert len(split_split) == 1
    component = split_split[0]
    assert _REPRO._time(component.time) == [
        [8, 1],
        [[1, [1, 1]]],
    ]
    assert component.point_keys == (((), ()),)
    assert component.resolution.value == "SPLIT"
    assert skeleton.counter("same_time_residual_after_level") == 0


@dataclass
class _FakeBuilder:
    """Минимальный adapter: unresolvable обязан отказать до мутации."""

    vertices: list
    edges: list
    counters: dict
    refusal: SkeletonOutcome | None = None

    def __post_init__(self):
        self.debts = []
        self.mutations = []

    def _edge_event_is_live(self, event):
        return True

    def _split_is_live(self, event):
        return True

    def _position(self, vertex, time):
        return vertex.point

    def _proof_edge_endpoint_ids(self, edge_id):
        return (3, 4) if edge_id == 4 else ()

    def _edge_keys(self, *edge_ids):
        return tuple(self.edges[index].key for index in edge_ids)

    def _record_obligation(self, **fields):
        self.debts.append(fields)

    def _apply_multi_edge(self, events):
        self.mutations.append(("EDGE", events))

    def _separate_vertex_meetings(self, events):
        self.mutations.append(("SPLIT", events))
        return {}, list(events)


def _fake_builder() -> _FakeBuilder:
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    edges = [SimpleNamespace(key=(index, 0, index + 1, 0)) for index in range(5)]
    vertices = [
        SimpleNamespace(
            ident=index,
            alive=True,
            prev=(index - 1) % 5,
            next=(index + 1) % 5,
            prev_edge=index,
            next_edge=(index + 1) % 5,
            birth=ZERO_TIME,
            point=point,
            reflex=False,
            sliding=None,
        )
        for index in range(5)
    ]
    return _FakeBuilder(
        vertices,
        edges,
        {
            "discarded_stale_candidates": 0,
            "unsupported_event_kind_dropped": 0,
            "superlevel_unresolvable_components": 0,
        },
    )


def test_artificial_unresolvable_component_refuses_without_order_choice():
    transaction = getattr(
        superlevel_module, "apply_superlevel_transaction", None
    )
    assert transaction is not None, "SUPERLEVEL_TRANSACTION_APPLIER_UNAVAILABLE"
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    level = (
        CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 0, 1, -1),
        CandidateEventV1(EventKind.SPLIT, ZERO_TIME, point, 2, -1, 4),
    )
    projections = []
    for packet in (level, tuple(reversed(level))):
        builder = _fake_builder()
        transaction(builder, packet)
        projections.append(
            (
                builder.refusal,
                tuple(
                    (
                        debt["cause"],
                        debt["disposition"],
                        debt["participant_edge_keys"],
                        debt["target_edge_keys"],
                    )
                    for debt in builder.debts
                ),
                tuple(builder.mutations),
            )
        )
    assert projections[0] == projections[1]
    outcome, debts, mutations = projections[0]
    assert outcome is SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    assert mutations == ()
    assert len(debts) == 1
    assert debts[0][:2] == (
        ProofObligationBranch.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
        ProofObligationDisposition.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
    )
