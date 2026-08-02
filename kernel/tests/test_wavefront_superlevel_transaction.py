"""Красные ворота P0-2b: exact-time component применяется транзакционно."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from fractions import Fraction
import hashlib
import importlib.util
import inspect
import json
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
from cftuv_envelope.wavefront.digest import node_record, semantic_digest
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.proof import (
    ProofObligationBranch,
    ProofObligationDisposition,
    ProofStatus,
)
from cftuv_envelope.wavefront import proof as proof_module
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront import superlevel as superlevel_module
from cftuv_envelope.wavefront.skeleton import (
    SkeletonOutcome,
    SplitSearch,
    build_skeleton,
)
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from wavefront_cases import named_corpus, partial_source_corpus, star


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
_ONE_POINT_COMPONENT_EDGES = {
    "ell_12_source_edges_0_1": (0, 1, 2, 3),
    "ell_12_source_edges_4_5": (2, 3, 4, 5),
    "ell_12_source_without_the_reflex_pair": (0, 1, 2, 3, 4, 5),
    "staircase_source_edges_0_1": (0, 1, 2, 3),
    "staircase_source_edges_6_7": (4, 5, 6, 7),
}
_DYNAMIC_SAME_TIME_CASES = (
    "ell_12_source_edge_1",
    "staircase_source_edge_1",
    "staircase_source_edge_3",
    "staircase_source_edges_2_3",
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

_STAR_16_SEED_11_DIGEST = (
    "b9beaca9b8a5ee662c1b3efde38f60dbbc6a746796dc98e2b55c8bf0cf8b19ef"
)
_STAR_16_SEED_11_LEGACY_MIXED_LOCUS_SHA256 = (
    "1c43a5139d6a6d1c842b9c5bc25aa54579c854b721bd00dfc48fc6a5f517292e",
    "5e7d38ff4748e0cde87ab5ca322ebd50b08cbd0c63bbe844aac201920e4dfa53",
    "896d1368d96cb210f374f8160670430d541c90a09592fd3cf4770b8751a1a226",
    "f65f136824f2627cd3702e78a02c83202eace6a412fe5cb1c65da9b6b04689b5",
    "f6adc2cc745f05f6e21053d5edbc118cf176b625596fb3e0e42ff810fe5ed433",
    "fa165fd8dc568b49893a138ab92aa1fc80da04a806cf01a4225d2518294c1598",
    "fa251d1555cdc3fd62f68127ca1c81e9d5a758539fce95c7aa93c9f2b602f9a2",
)
_STAR_16_SEED_11_LEGACY_NODE_SHA256 = (
    "4854194e27f8890cee247d408ee3eec53d656debd3f160d450f906cec41a91d5",
    "48838fbefb2560ed11824142d52fea66a4d996f5a8d084ca03e9f1ecd13c09a9",
    "4ae78998362f8d9399557e8f32335a5ed899582ddf3c0710c90b7c6c0a02e229",
    "7b55eea3745386a6e9c7bba4aa3af6fcb272ec692b2380fb12e21c8ab6ac83dd",
    "be074c5caeccb7aa08c3ce915814ae2b0b799ae50834f97ec60697940730cd55",
    "df17ba3fa4df4532af2c2c56a81c2bbd23d00ef92776542245f83cf211057991",
    "fe195c761d41e498acc0821d9da1722986477bdae78503740bd5a8bbeb170aef",
)
_STAR_16_SEED_11_PRODUCT_AUTHORITY = {
    "partition_sha256": (
        "023e0331bdff31172d18cac96b6c69de327a72b32d358e674bdebfb8a3eb6dfb"
    ),
    "ownership_sha256": (
        "c0441480559baa6d506561b611a1d02943909b29735e099d2b44f1faf1affd86"
    ),
    "coverage_sha256": (
        "7833d6cc6c70c77710c5e4f53ea599886f9190c593f43aaaf6e809b7c505ac17",
        "3870ea85ddbc936fb003b6c4c2d6c28a31cc9e2de7b64eb52fcd5f9bad3902a9",
        "e779f3cbf5d97dda812604ad628858f7e85173561e319404e436e601266ed144",
    ),
}
_STAR_16_SEED_11_LEGACY_PROOF = Counter(
    {
        (
            "NO_RULE_MEETING_NOT_RECONNECTABLE",
            "UNPROVEN_FALLBACK_APPLIED",
        ): 8,
        (
            "NO_RULE_TRIPLE_ALWAYS_CONCURRENT",
            "DISCHARGED_BY_PROVEN_SAME_TIME_EVENT",
        ): 51,
    }
)
_P0_2_PROOF_FINALIZE_SOURCE_SHA256 = (
    "6549c735573bf91c527793c76d9cea9aa735f2dcf973f94e03b55e2ed6fffa40"
)
_P0_2_INCOMPLETE_DISPOSITIONS = (
    "EVENT_ACCEPTED_WITH_UNPROVEN_SPAN",
    "SURVIVED_PAST_EVENT_TIME",
    "UNPROVEN_FALLBACK_APPLIED",
    "UNSUPPORTED_EVENT_KIND_DROPPED",
)

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


def _sha256_json(value) -> str:
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


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


@pytest.mark.parametrize("name", _PRODUCT_AXIS_CASES)
def test_main_product_has_one_union_incidence_component_node(name):
    polygon = _CORPUS[name]
    skeleton = build_skeleton(polygon)
    mixed = [
        node
        for node in skeleton.nodes
        if node.kind is EventKind.MULTIWAY
        and node.kinds == (EventKind.EDGE, EventKind.SPLIT)
    ]
    assert len(mixed) == 1
    node = mixed[0]
    points = polygon.outer.points
    expected = tuple(
        sorted(
            points[index] + points[(index + 1) % len(points)]
            for index in _ONE_POINT_COMPONENT_EDGES[name]
        )
    )
    assert node.participants == expected
    assert node.incidences == (expected,)


def test_staircase_3_4_names_the_t4_mixed_component(monkeypatch):
    planner = superlevel_module.plan_superlevel_components
    collector = superlevel_module.collect_superlevel_snapshot
    observed = []
    packets = []

    def measured(snapshot):
        components = planner(snapshot)
        observed.extend(components)
        return components

    def measured_packet(builder, level):
        packets.append(
            (
                _REPRO._time(level[0].time),
                tuple(superlevel_module._event_point_key(event) for event in level),
            )
        )
        return collector(builder, level)

    monkeypatch.setattr(
        superlevel_module, "plan_superlevel_components", measured
    )
    monkeypatch.setattr(
        superlevel_module, "collect_superlevel_snapshot", measured_packet
    )
    skeleton = build_skeleton(_CORPUS["staircase_source_edges_3_4"])
    mixed = [
        component
        for component in observed
        if component.event_kinds == (EventKind.EDGE, EventKind.SPLIT)
        and component.point_keys
        == (
            (
                ((1, Fraction(4)),),
                ((1, Fraction(4)),),
            ),
        )
    ]
    assert len(mixed) == 1
    component = mixed[0]
    assert _REPRO._time(component.time) == [
        [4, 1],
        [[1, [1, 1]]],
    ]
    assert component.resolution.value == "BOUNDARY_PORT_PAIRING"
    assert component.participants == (
        (4, 8, 4, 12),
        (8, 4, 8, 8),
        (8, 8, 4, 8),
        (12, 4, 8, 4),
    )
    assert skeleton.counter("same_time_residual_after_level") == 0
    assert not any(
        time == [[8, 1], [[1, [1, 1]]]] and ((), ()) in points
        for time, points in packets
    ), "DEPENDENT_STALE_T8_ORIGIN_PACKET_PRESENT"


def test_staircase_edge_6_regenerates_live_split_after_t4_rewire(monkeypatch):
    """Every edge whose live span changed at t4 must regenerate candidates."""

    original_close = skeleton_module._Builder._close_short_lavs
    after_t4 = []

    def measured_close(builder):
        if _REPRO._time(builder.now) == [[4, 1], [[1, [1, 1]]]]:
            for entry in builder.queue._heap:
                event = entry.event
                if event.kind is not EventKind.SPLIT:
                    continue
                vertex = builder.vertices[event.vertex]
                after_t4.append(
                    (
                        _REPRO._time(event.time),
                        superlevel_module._event_point_key(event),
                        builder._edge_keys(vertex.prev_edge, vertex.next_edge),
                        builder._edge_keys(event.edge),
                        builder._split_is_live(event),
                    )
                )
        original_close(builder)

    monkeypatch.setattr(skeleton_module._Builder, "_close_short_lavs", measured_close)
    build_skeleton(_CORPUS["staircase_source_edge_6"])
    assert (
        [[8, 1], [[1, [1, 1]]]],
        (((1, Fraction(8)),), ((1, Fraction(4)),)),
        ((12, 4, 8, 4), (8, 4, 8, 8)),
        ((4, 12, 0, 12),),
        True,
    ) in after_t4


def test_cross_node_projection_is_byte_identical_to_pretransaction_anchor():
    skeleton = build_skeleton(_CORPUS["cross"])
    projection = [node_record(node) for node in skeleton.nodes]
    projection_sha256 = hashlib.sha256(
        json.dumps(
            projection, sort_keys=True, separators=(",", ":")
        ).encode("utf-8")
    ).hexdigest()
    assert semantic_digest(skeleton) == (
        "a5205821ef102efb12752307e917214b535e60a8379f938d481fc19aadbc74cf"
    )
    assert projection_sha256 == (
        "dc60f47f5c0c285a54daeb85d84857da6db8f5e8605a16d8354d042723984376"
    )


def test_close_and_proof_wait_for_dynamic_same_time_fixed_point(monkeypatch):
    original_apply = skeleton_module._Builder._apply_level
    original_close = skeleton_module._Builder._close_short_lavs
    original_discharge = (
        skeleton_module._Builder._discharge_observed_obligations
    )
    observed_dynamic = set()
    active_case = [None]

    def measured_apply(builder, level):
        original_apply(builder, level)
        if superlevel_module.has_same_time_residual(builder.queue, builder.now):
            observed_dynamic.add(active_case[0])

    def guarded_close(builder):
        assert not superlevel_module.has_same_time_residual(
            builder.queue, builder.now
        )
        original_close(builder)

    def guarded_discharge(builder):
        assert not superlevel_module.has_same_time_residual(
            builder.queue, builder.now
        )
        original_discharge(builder)

    monkeypatch.setattr(skeleton_module._Builder, "_apply_level", measured_apply)
    monkeypatch.setattr(
        skeleton_module._Builder, "_close_short_lavs", guarded_close
    )
    monkeypatch.setattr(
        skeleton_module._Builder,
        "_discharge_observed_obligations",
        guarded_discharge,
    )
    for name in _DYNAMIC_SAME_TIME_CASES:
        active_case[0] = name
        build_skeleton(_CORPUS[name])
    assert observed_dynamic == set(_DYNAMIC_SAME_TIME_CASES)


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


def _fake_indistinguishable_twins() -> _FakeBuilder:
    """Два LAV с одинаковой геометрией: runtime id не разрешает ничью."""

    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    owner_keys = (
        (0, 0, 1, 0),
        (1, 0, 0, 1),
        (0, 1, 0, 0),
    )
    edges = [
        SimpleNamespace(key=owner_keys[index % 3]) for index in range(6)
    ]
    vertices = [
        SimpleNamespace(
            ident=index,
            alive=True,
            prev=(index // 3) * 3 + (index - 1) % 3,
            next=(index // 3) * 3 + (index + 1) % 3,
            prev_edge=(index // 3) * 3 + (index - 1) % 3,
            next_edge=index,
            birth=ZERO_TIME,
            point=point,
            reflex=False,
            sliding=None,
        )
        for index in range(6)
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


def test_star_16_seed_11_conditional_auth_oracle():
    """Пять условий AUTH для принятого mixed-component ответа b9be."""

    # Старые якоря сняты на P0-2 parent a091028: новый код их не вычисляет.
    polygon = star(16, 11)
    assert polygon is not None
    skeleton = build_skeleton(polygon, split_search=SplitSearch.MOTORCYCLE)
    assert skeleton.outcome is SkeletonOutcome.EXACT

    records = tuple(node_record(node) for node in skeleton.nodes)
    mixed = tuple(
        record
        for record in records
        if record["kind"] == EventKind.MULTIWAY.value
        and record["kinds"]
        == [EventKind.EDGE.value, EventKind.SPLIT.value]
    )

    # (1) Все семь старых exact (t, point, participants) loci сохранены байт-в-байт.
    loci = tuple(
        {
            key: record[key]
            for key in (
                "time_dividend",
                "time_divisor",
                "point_x",
                "point_y",
                "participants",
            )
        }
        for record in mixed
    )
    assert len(loci) == 7
    assert tuple(sorted(_sha256_json(locus) for locus in loci)) == (
        _STAR_16_SEED_11_LEGACY_MIXED_LOCUS_SHA256
    )

    # (2) Разбиение, ownership и coverage на всех трёх alpha — старые байты.
    product = _run(polygon)
    assert product["partition_sha256"] == (
        _STAR_16_SEED_11_PRODUCT_AUTHORITY["partition_sha256"]
    )
    assert product["ownership_sha256"] == (
        _STAR_16_SEED_11_PRODUCT_AUTHORITY["ownership_sha256"]
    )
    assert tuple(
        record["sha256"] for record in product["coverage_by_alpha"]
    ) == _STAR_16_SEED_11_PRODUCT_AUTHORITY["coverage_sha256"]
    assert tuple(
        tuple(record["alpha"]) for record in product["coverage_by_alpha"]
    ) == ((1, 4), (1, 1), (3, 1))

    # (3) Оба split search и все schedule/input permutations дают b9be.
    digests = {
        f"search_{search.value}": semantic_digest(
            build_skeleton(polygon, split_search=search)
        )
        for search in SplitSearch
    }
    digests.update(
        {
            "schedule_edge_first": _run(
                polygon, apply=_edge_first
            )["semantic_digest"],
            "schedule_heap_reverse": _run(
                polygon, reverse_heap_packet=True
            )["semantic_digest"],
            "schedule_geometric": _run(
                polygon, apply=_input_geometric
            )["semantic_digest"],
            "schedule_geometric_reverse": _run(
                polygon, apply=_input_geometric_reverse
            )["semantic_digest"],
        }
    )
    digests.update(
        {
            f"input_{name}": _run(candidate)["semantic_digest"]
            for name, candidate in _input_permutations(polygon).items()
        }
    )
    assert set(digests.values()) == {_STAR_16_SEED_11_DIGEST}, digests

    # (4) Единственная node-record дельта: SPLIT становится mixed MULTIWAY.
    # kinds/incidences/converging_vertices — обязательная запись этого kind;
    # после её удаления полный record совпадает со старым frozen record.
    legacy_records = []
    for record in mixed:
        assert record["converging_vertices"] == 2
        assert record["incidences"] == [record["participants"]]
        legacy = dict(record)
        legacy["kind"] = EventKind.SPLIT.value
        legacy["converging_vertices"] = 1
        del legacy["kinds"]
        del legacy["incidences"]
        legacy_records.append(_sha256_json(legacy))
    assert tuple(sorted(legacy_records)) == _STAR_16_SEED_11_LEGACY_NODE_SHA256
    assert sum(record["kind"] == EventKind.SPLIT.value for record in records) == 1

    # (5) Ровно восемь старых named meeting fallback исчезли; COMPLETE по-прежнему
    # есть отрицание неизменного множества incomplete-dispositions.
    current_proof = Counter(
        (obligation.cause.value, obligation.disposition.value)
        for obligation in skeleton.proof_obligations
    )
    assert _STAR_16_SEED_11_LEGACY_PROOF == Counter(
        {
            (
                "NO_RULE_MEETING_NOT_RECONNECTABLE",
                "UNPROVEN_FALLBACK_APPLIED",
            ): 8,
            (
                "NO_RULE_TRIPLE_ALWAYS_CONCURRENT",
                "DISCHARGED_BY_PROVEN_SAME_TIME_EVENT",
            ): 51,
        }
    )
    assert current_proof == Counter(
        {
            (
                "NO_RULE_TRIPLE_ALWAYS_CONCURRENT",
                "DISCHARGED_BY_PROVEN_SAME_TIME_EVENT",
            ): 13,
        }
    )
    assert not any(
        obligation.cause.value == "NO_RULE_MEETING_NOT_RECONNECTABLE"
        for obligation in skeleton.proof_obligations
    )
    assert not any(
        obligation.disposition
        is ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED
        for obligation in skeleton.proof_obligations
    )
    assert skeleton.counter("refused_no_rule_meeting_not_reconnectable") == 0
    assert skeleton.counter("superlevel_contact_junction_resolutions") == 13
    assert skeleton.counter("superlevel_contact_junction_resolutions") >= 8
    finalize_source = inspect.getsource(
        proof_module.ProofLedger.finalize
    ).replace("\r\n", "\n")
    assert hashlib.sha256(finalize_source.encode("utf-8")).hexdigest() == (
        _P0_2_PROOF_FINALIZE_SOURCE_SHA256
    )
    current_incomplete = tuple(
        sorted(
            disposition.value
            for disposition in proof_module._INCOMPLETE_DISPOSITIONS
        )
    )
    additive = (
        ProofObligationDisposition.SUPERLEVEL_COMPONENT_UNRESOLVABLE.value
    )
    assert tuple(
        value for value in current_incomplete if value != additive
    ) == _P0_2_INCOMPLETE_DISPOSITIONS
    assert current_incomplete == tuple(
        sorted(_P0_2_INCOMPLETE_DISPOSITIONS + (additive,))
    )
    assert ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED in (
        proof_module._INCOMPLETE_DISPOSITIONS
    )
    assert all(
        obligation.disposition not in proof_module._INCOMPLETE_DISPOSITIONS
        for obligation in skeleton.proof_obligations
    )
    assert skeleton.proof_status is ProofStatus.COMPLETE


def test_independent_same_point_incidents_remain_separate_and_canonical():
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    events = (
        CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 0, 1, -1),
        CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 3, 4, -1),
    )
    vertices = tuple(
        superlevel_module._VertexSnapshot(
            ident=index,
            prev=(index // 3) * 3 + (index - 1) % 3,
            next=(index // 3) * 3 + (index + 1) % 3,
            prev_edge=index,
            next_edge=(index + 1) % 3 + (index // 3) * 3,
            alive=True,
            incoming_ray=(-1, 0),
            outgoing_ray=(1, 0),
        )
        for index in range(6)
    )
    incidents = tuple(
        superlevel_module.SuperlevelIncidentV1(
            event=event,
            vertex_ids=(offset, offset + 1, offset + 2),
            edge_occurrences=(offset, offset + 1, offset + 2),
            participants=((offset, 0, offset + 1, 0),),
            target_participants=((offset, 0, offset + 1, 0),),
            point_key=((), ()),
        )
        for event, offset in zip(events, (0, 3))
    )

    def planned(packet):
        snapshot = superlevel_module.SuperlevelSnapshotV1(
            incidents=packet,
            vertices=vertices,
            unsupported=(),
            stale_candidates=0,
        )
        return superlevel_module.plan_superlevel_components(snapshot)

    forward = planned(incidents)
    backward = planned(tuple(reversed(incidents)))
    assert forward == backward
    assert len(forward) == 2
    assert all(len(plan.edge_contacts) == 1 for plan in forward)


def test_indistinguishable_occurrence_twins_refuse_before_runtime_tie():
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    packet = (
        CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 0, 1, -1),
        CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 3, 4, -1),
    )
    projections = []
    for events in (packet, tuple(reversed(packet))):
        builder = _fake_indistinguishable_twins()
        superlevel_module.apply_superlevel_transaction(builder, events)
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
    assert projections[0][0] is SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    assert projections[0][2] == ()
