"""Красные ворота P0-2b: exact-time component применяется транзакционно."""

from __future__ import annotations

from collections import Counter
import copy
from dataclasses import dataclass, replace
from fractions import Fraction
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.wavefront.event_time import (
    EventPointV1,
    ZERO_TIME,
    compare_times,
)
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
from cftuv_envelope.wavefront import superlevel_closure as closure_module
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.skeleton import (
    SkeletonOutcome,
    SplitSearch,
    build_skeleton,
)
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1
from wavefront_cases import named_corpus, partial_source_corpus, star
from weighted_wall_differential_cases import weighted_wall_differential_corpus


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
_FIELD_CASES = (
    "building_all_seams_patch_001_lost_resolved_v1",
    "building_all_seams_patch_006_lost_resolved_v1",
    "building_all_seams_patch_011_lost_resolved_v1",
    "building_all_seams_patch_105_lost_resolved_v1",
)
#: Терминал конвейера на полевом случае. Честный именованный отказ — такая же
#: приёмка, как EXACT, и закрепляется он ТАК ЖЕ строго: имя, а не «не EXACT».
#: Ценность теневой сверки от терминала не зависит и на отказном пути даже
#: выше: равенство разреженного и плотного снимков обязано держаться и там,
#: где фронт не закрылся, иначе отказ был бы артефактом снимка.
#: Корень отказа и открытый счёт зеркальной чётности имён — в записи поставки
#: P0-2B-FINISH в `DECISIONS.md`.
_FIELD_TERMINAL = {
    "building_all_seams_patch_001_lost_resolved_v1": "SKELETON_DID_NOT_CLOSE",
    "building_all_seams_patch_006_lost_resolved_v1": "SKELETON_DID_NOT_CLOSE",
    "building_all_seams_patch_011_lost_resolved_v1": "SKELETON_DID_NOT_CLOSE",
    "building_all_seams_patch_105_lost_resolved_v1": "EXACT",
}
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


def _collect_eager_snapshot(builder, level):
    """До sparse-query oracle: все live LAV positions одного уровня."""

    time = level[0].time if level else ZERO_TIME
    point_keys = []
    for vertex in builder.vertices:
        if not vertex.alive:
            point_keys.append(None)
            continue
        point = builder._position(vertex, time)
        point_keys.append(
            None if point is None else (point.x.terms, point.y.terms)
        )
    occurrences = {}
    owner_counts = Counter()
    for vertex in builder.vertices:
        if not vertex.alive:
            continue
        end = builder.vertices[vertex.next]
        if end.alive:
            owner_counts[vertex.next_edge] += 1
            occurrences[vertex.next_edge] = (
                builder.edges[vertex.next_edge].key,
                point_keys[vertex.ident],
                point_keys[end.ident],
            )
    vertices = tuple(
        superlevel_module._vertex_snapshot(
            builder, vertex, point_keys[vertex.ident], occurrences
        )
        for vertex in builder.vertices
    )
    incidents = []
    unsupported = []
    stale = 0
    for event in level:
        if event.kind not in {EventKind.SPLIT, EventKind.EDGE}:
            unsupported.append(event)
            continue
        live = (
            builder._edge_event_is_live(event)
            if event.kind is EventKind.EDGE
            else builder._split_is_live(event)
        )
        if live:
            incidents.append(superlevel_module._incident(builder, event, vertices))
        else:
            stale += 1
    return superlevel_module.SuperlevelSnapshotV1(
        incidents=tuple(incidents),
        vertices=vertices,
        unsupported=tuple(unsupported),
        stale_candidates=stale,
        duplicate_live_owner_edge_ids=tuple(
            sorted(edge_id for edge_id, count in owner_counts.items() if count != 1)
        ),
    )


def _final_semantic_axes(polygon, skeleton):
    record = _REPRO._final_record(polygon, skeleton, [])
    record.pop("state_after_each_superlevel")
    record.pop("topology_after_each_superlevel")
    record["levels"] = skeleton.levels
    record["counters"] = skeleton.counters
    record["nodes"] = tuple(
        sorted(
            (node_record(node) for node in skeleton.nodes),
            key=lambda item: json.dumps(item, sort_keys=True),
        )
    )
    return record


def _shadow_collector(sparse_collector, label):
    def collect(builder, level):
        sparse = sparse_collector(builder, level)
        eager = _collect_eager_snapshot(builder, level)
        assert sparse.incidents == eager.incidents, (label, "incidents")
        assert sparse.unsupported == eager.unsupported, (label, "unsupported")
        assert sparse.stale_candidates == eager.stale_candidates, (
            label,
            "stale_candidates",
        )
        assert sparse.duplicate_live_owner_edge_ids == (
            eager.duplicate_live_owner_edge_ids
        ), (label, "duplicate_live_owner_edge_ids")
        assert sparse.duplicate_live_owner_edge_ids == (), (
            label,
            "duplicate_live_owner_edge_ids_nonzero",
        )
        sparse_plans = superlevel_module.plan_superlevel_components(sparse)
        eager_plans = superlevel_module.plan_superlevel_components(eager)
        assert sparse_plans == eager_plans, (label, "plans")
        assert superlevel_module._commit_prestate(sparse, sparse_plans) == (
            superlevel_module._commit_prestate(eager, eager_plans)
        ), (label, "commit_prestate")
        return sparse

    return collect


def _assert_shadow_corpus(cases, monkeypatch):
    sparse_collector = superlevel_module.collect_superlevel_snapshot
    for name, polygon in cases:
        for search in SplitSearch:
            label = (name, search.value)
            monkeypatch.setattr(
                superlevel_module,
                "collect_superlevel_snapshot",
                _shadow_collector(sparse_collector, label),
            )
            sparse = build_skeleton(polygon, split_search=search)
            monkeypatch.setattr(
                superlevel_module,
                "collect_superlevel_snapshot",
                _collect_eager_snapshot,
            )
            eager = build_skeleton(polygon, split_search=search)
            assert _final_semantic_axes(polygon, sparse) == (
                _final_semantic_axes(polygon, eager)
            ), (label, "final_semantic_axes")


def test_sparse_shadow_matches_eager_on_all_63_named_cases_both_modes(
    monkeypatch,
):
    cases = tuple(named_corpus()) + tuple(partial_source_corpus())
    assert len(cases) == 63
    _assert_shadow_corpus(cases, monkeypatch)


def test_sparse_shadow_matches_eager_on_p0_3_corpus_both_modes(monkeypatch):
    weighted = weighted_wall_differential_corpus()
    assert len(weighted) == 23
    _assert_shadow_corpus(
        tuple((case.name, case.polygon) for case in weighted),
        monkeypatch,
    )


@pytest.mark.parametrize("case_name", _FIELD_CASES)
def test_sparse_shadow_matches_eager_on_sem_clb_field_both_modes(
    case_name,
    monkeypatch,
):
    root = (
        Path(__file__).parents[1]
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
        / "cases"
        / case_name
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    sparse_collector = superlevel_module.collect_superlevel_snapshot
    monkeypatch.setattr(
        superlevel_module,
        "collect_superlevel_snapshot",
        _shadow_collector(sparse_collector, (case_name, "prepare")),
    )
    prepared = prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    assert prepared.outcome.value == _FIELD_TERMINAL[case_name], prepared.detail
    (region,) = prepared.regions
    polygon = region.bridge.polygon
    assert polygon is not None
    for search in SplitSearch:
        label = (case_name, search.value)
        monkeypatch.setattr(
            superlevel_module,
            "collect_superlevel_snapshot",
            _shadow_collector(sparse_collector, label),
        )
        sparse = build_skeleton(polygon, split_search=search)
        monkeypatch.setattr(
            superlevel_module,
            "collect_superlevel_snapshot",
            _collect_eager_snapshot,
        )
        eager = build_skeleton(polygon, split_search=search)
        assert _final_semantic_axes(polygon, sparse) == (
            _final_semantic_axes(polygon, eager)
        ), (label, "final_semantic_axes")


def _differences(
    reference: dict,
    candidate: dict,
    keys: tuple[str, ...] = _PROJECTION_KEYS,
) -> tuple[str, ...]:
    return tuple(
        key for key in keys if reference[key] != candidate[key]
    )


@pytest.mark.parametrize("name", _D_CASES)
def test_sparse_snapshot_matches_eager_oracle_under_corpus_permutations(
    name, monkeypatch
):
    polygon = _CORPUS[name]
    sparse_collector = superlevel_module.collect_superlevel_snapshot
    schedules = {
        "production": {},
        "edge_first": {"apply": _edge_first},
        "heap_reverse": {"reverse_heap_packet": True},
        "geometric": {"apply": _input_geometric},
        "geometric_reverse": {"apply": _input_geometric_reverse},
    }
    polygons = {"original": polygon} | _input_permutations(polygon)
    for polygon_name, candidate in polygons.items():
        for schedule_name, options in schedules.items():
            monkeypatch.setattr(
                superlevel_module,
                "collect_superlevel_snapshot",
                sparse_collector,
            )
            sparse = _run(candidate, **options)
            monkeypatch.setattr(
                superlevel_module,
                "collect_superlevel_snapshot",
                _collect_eager_snapshot,
            )
            eager = _run(candidate, **options)
            assert not _differences(sparse, eager), (
                name,
                polygon_name,
                schedule_name,
                _differences(sparse, eager),
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


def test_symbolic_closure_absorbs_cross_full_q1_t8_before_commit(monkeypatch):
    polygon = next(
        case.polygon
        for case in weighted_wall_differential_corpus()
        if case.name == "cross_full_q_1"
    )
    original = superlevel_module.apply_superlevel_transaction
    t8_packets = []

    def observed(builder, level):
        if _REPRO._time(level[0].time) == [[8, 1], [[1, [1, 1]]]]:
            t8_packets.append(
                tuple(
                    (
                        event.kind.value,
                        event.vertex,
                        event.peer,
                        event.edge,
                        superlevel_module._event_point_key(event),
                    )
                    for event in level
                )
            )
        original(builder, level)
        if t8_packets:
            assert not superlevel_module.has_same_time_residual(
                builder.queue, builder.now
            )

    monkeypatch.setattr(
        superlevel_module, "apply_superlevel_transaction", observed
    )
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert len(t8_packets) == 1


def test_runtime_commit_allocates_real_ids_for_dead_symbolic_births(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_runtime_commit as runtime

    polygon = next(
        case.polygon
        for case in weighted_wall_differential_corpus()
        if case.name == "cross_full_q_1"
    )
    original = runtime.materialize_symbolic_runtime_commit
    rows = []

    def observed(builder, snapshot, plan):
        before_vertices = len(builder.vertices)
        before_nodes = len(builder._node_vertex_ids)
        original(builder, snapshot, plan)
        allocated = len(builder.vertices) - before_vertices
        dead_births = sum(not vertex.alive for vertex in plan.births)
        assert allocated == len(plan.births)
        assert all(
            0 <= ident < len(builder.vertices)
            for ids in builder._node_vertex_ids[before_nodes:]
            for ident in ids
        )
        rows.append((allocated, dead_births))

    monkeypatch.setattr(runtime, "materialize_symbolic_runtime_commit", observed)
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert any(dead_births for _, dead_births in rows)


def test_forced_runtime_prevalidation_refusal_preserves_topology(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_runtime_commit as runtime

    def fingerprint(builder):
        return (
            tuple(
                (
                    vertex.ident,
                    vertex.prev,
                    vertex.next,
                    vertex.prev_edge,
                    vertex.next_edge,
                    vertex.birth,
                    vertex.point,
                    vertex.reflex,
                    vertex.alive,
                    vertex.sliding,
                )
                for vertex in builder.vertices
            ),
            tuple((edge.ident, edge.line, edge.span) for edge in builder.edges),
        )

    reason = "SYMBOLIC_RUNTIME_FORCED_PREVALIDATION_REFUSAL"
    monkeypatch.setattr(
        runtime,
        "plan_symbolic_runtime_commit",
        lambda *args: (None, reason),
    )
    original = superlevel_module.apply_superlevel_transaction
    observed = []

    def guarded(builder, level):
        before = fingerprint(builder)
        original(builder, level)
        observed.append((before, fingerprint(builder)))

    monkeypatch.setattr(
        superlevel_module, "apply_superlevel_transaction", guarded
    )
    skeleton = build_skeleton(_CORPUS["cross"])
    assert skeleton.outcome is SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    assert observed and all(before == after for before, after in observed)
    assert skeleton.counter("superlevel_unresolvable_components") == 1
    assert skeleton.counter(f"superlevel_unresolvable_reason::{reason}") == 1
    assert any(
        obligation.cause
        is ProofObligationBranch.SUPERLEVEL_COMPONENT_UNRESOLVABLE
        for obligation in skeleton.proof_obligations
    )


def test_affine_span_ambiguity_is_the_reachable_q08_last_guard(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_runtime_commit as runtime
    from cftuv_envelope.wavefront.poststate_span import (
        PoststateSpanClassificationV1,
        PoststateSpanDisposition,
    )
    from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

    zero = SqrtSumV1.rational(0)
    ambiguous = PoststateSpanClassificationV1(
        PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN,
        zero,
        zero,
        1,
    )
    monkeypatch.setattr(
        runtime,
        "poststate_span_classifications",
        lambda *args: (
            runtime.SymbolicPoststateSpanV1(
                None, None, ambiguous, (), ()
            ),
        ),
    )
    before_after = []
    original = superlevel_module.apply_superlevel_transaction

    def topology(builder):
        return (
            tuple(
                (v.prev, v.next, v.prev_edge, v.next_edge, v.alive)
                for v in builder.vertices
            ),
            tuple((e.ident, e.key, e.line, e.span) for e in builder.edges),
        )

    def guarded(builder, level):
        before = topology(builder)
        original(builder, level)
        before_after.append((before, topology(builder)))

    monkeypatch.setattr(
        superlevel_module, "apply_superlevel_transaction", guarded
    )
    skeleton = build_skeleton(_CORPUS["cross"])
    reason = "SYMBOLIC_POSTSTATE_SPAN_AFFINE_CLASSIFICATION_UNPROVEN"
    assert skeleton.outcome is (
        SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    )
    assert before_after and all(a == b for a, b in before_after)
    assert skeleton.counter(
        f"superlevel_unresolvable_reason::{reason}"
    ) == 1


def test_runtime_commit_rejects_in_range_wrong_physical_edge(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_runtime_commit as runtime
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer
    from cftuv_envelope.wavefront.symbolic_component import clone_overlay

    polygon = next(
        case.polygon
        for case in weighted_wall_differential_corpus()
        if case.name == "cross_full_q_1"
    )
    original = superlevel_module.apply_superlevel_transaction

    class _GateReached(Exception):
        pass

    def observed(builder, level):
        if superlevel_module._time_key(level[0].time)[0] != Fraction(8):
            return original(builder, level)
        snapshot = superlevel_module.collect_superlevel_snapshot(builder, level)
        budget = 2 * len(snapshot.vertices) + len(snapshot.incidents)
        closure = outer.plan_symbolic_superlevel_closure(
            builder,
            snapshot,
            outer_budget=budget,
            junction_budget=budget,
        )
        assert closure.unresolved_reason is None
        tampered = clone_overlay(closure.overlay)
        active = next(
            binding
            for binding in tampered.spans.values()
            if binding.start is not None and binding.end is not None
        )
        wrong = next(
            edge.ident
            for edge in builder.edges
            if edge.key != builder.edges[active.physical_edge_id].key
        )
        for leaf, binding in tuple(tampered.spans.items()):
            if binding.physical_edge_id == active.physical_edge_id:
                tampered.spans[leaf] = replace(
                    binding, physical_edge_id=wrong
                )
        before = (
            tuple(
                (v.prev, v.next, v.prev_edge, v.next_edge, v.alive)
                for v in builder.vertices
            ),
            tuple((e.ident, e.key, e.line, e.span) for e in builder.edges),
        )
        plan, reason = runtime.plan_symbolic_runtime_commit(
            builder, snapshot, replace(closure, overlay=tampered)
        )
        after = (
            tuple(
                (v.prev, v.next, v.prev_edge, v.next_edge, v.alive)
                for v in builder.vertices
            ),
            tuple((e.ident, e.key, e.line, e.span) for e in builder.edges),
        )
        assert plan is None
        assert reason == "SYMBOLIC_RUNTIME_EDGE_AUTHORITY_MISMATCH"
        assert before == after
        raise _GateReached

    monkeypatch.setattr(
        superlevel_module, "apply_superlevel_transaction", observed
    )
    with pytest.raises(_GateReached):
        build_skeleton(polygon)


def test_poststate_past_edge_symptom_is_classified_as_opening(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_runtime_commit as runtime
    from cftuv_envelope.wavefront.poststate_span import (
        PoststateSpanDisposition,
    )

    case_name = "building_all_seams_patch_001_lost_resolved_v1"
    root = (
        Path(__file__).parents[1]
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
        / "cases"
        / case_name
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    raw_extra = (-100088, 1159360, -219120, 1193292)
    edge_49 = (-85508, 517564, -51032, 507740)
    edge_45 = (-60168, 615468, -60168, 615468, 1)
    edge_25 = (-33500, 140620, -119984, 527392)
    original_plan = runtime.plan_symbolic_runtime_commit
    causal_receipts = []

    def observed_plan(builder, frozen, closure):
        by_key = {edge.key: edge.ident for edge in builder.edges}
        classifications = tuple(
            item
            for item in runtime.poststate_span_classifications(
                builder, frozen, closure.overlay
            )
            if {
                by_key.get(key) for key in item.participant_edge_keys
            } == {49, 45, 25}
        )
        plan, reason = original_plan(builder, frozen, closure)
        if not classifications:
            return plan, reason
        assert len(classifications) == 1
        classification = classifications[0]
        affine = classification.classification
        assert affine.disposition is PoststateSpanDisposition.OPENING
        assert affine.birth_length.sign() > 0
        assert affine.slope.sign() > 0
        assert affine.orientation_sign == -1
        assert reason is None and plan is not None

        # Q-08's exact past event remains a diagnostic consequence, not an
        # input to the affine predicate.  The raw EDGE and the newborn
        # adjacency have distinct stable identities.
        raw_edges = tuple(
            incident for incident in frozen.incidents
            if incident.event.kind is EventKind.EDGE
        )
        raw = next(
            incident for incident in raw_edges
            if (incident.event.vertex, incident.event.peer) == (140, 123)
        )
        assert (raw.event.vertex, raw.event.peer) == (140, 123)
        assert set(raw.participants) == {raw_extra, edge_49, edge_45}
        past = next(
            item
            for item in runtime.changed_poststate_past_edge_events(
                builder, frozen, closure.overlay
            )
            if {by_key.get(key) for key in item.participant_edge_keys}
            == {49, 45, 25}
        )
        assert set(raw.participants) != set(past.participant_edge_keys)
        assert set(past.participant_edge_keys) == {
            edge_49, edge_45, edge_25
        }
        assert compare_times(raw.event.time, past.event_time) > 0
        assert past.event_point is not None
        assert raw.point_key != (
            past.event_point.x.terms,
            past.event_point.y.terms,
        )
        assert past.event_time.sign < 0
        assert past.vertex_ref == classification.vertex_ref
        assert past.peer_ref == classification.peer_ref
        assert past.vertex_ref.kind == "BIRTH"
        assert past.peer_ref.kind == "EXISTING"
        assert compare_times(past.event_time, past.now) < 0
        assert compare_times(past.event_time, past.vertex_birth) < 0
        assert compare_times(past.event_time, past.peer_birth) < 0

        # The birth is materialized and targeted scheduling runs.  Its empty
        # queue result is therefore not a lost enqueue.
        shadow = copy.deepcopy(builder)
        enqueued_for = []
        shadow_enqueue_for = shadow._enqueue_for

        def track_enqueue(vertex):
            enqueued_for.append(vertex.ident)
            shadow_enqueue_for(vertex)

        shadow._enqueue_for = track_enqueue
        assert runtime.materialize_symbolic_runtime_commit(
            shadow, frozen, plan
        ) is None
        born_ids = {
            symbolic.ref: ident
            for symbolic, ident in zip(
                plan.births,
                range(len(builder.vertices), len(shadow.vertices)),
            )
        }
        born_id = born_ids[classification.vertex_ref]
        assert born_id in enqueued_for
        shadow.queue = EventQueueV1()
        shadow._enqueue_edge_event(shadow.vertices[born_id])
        assert len(shadow.queue) == 0
        causal_receipts.append((classification, past))
        return plan, reason

    monkeypatch.setattr(
        runtime, "plan_symbolic_runtime_commit", observed_plan
    )

    # The final unresolved LAV has no stale queue item and a full once-only
    # reseed remains empty.  This freezes the fourth Q-08 negative statement
    # while leaving its later repair outside P0-2c.
    original_finish = skeleton_module._Builder._finish
    full_reseed_receipts = []

    def capture_full_reseed(builder, outcome, levels):
        if outcome is SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED:
            alive = tuple(
                vertex for vertex in builder.vertices if vertex.alive
            )
            initial_queue_size = len(builder.queue)
            builder.queue = EventQueueV1()
            for vertex in alive:
                builder._enqueue_for(vertex)
            full_reseed_receipts.append(
                (
                    tuple(vertex.ident for vertex in alive),
                    initial_queue_size,
                    len(builder.queue),
                )
            )
        return original_finish(builder, outcome, levels)

    monkeypatch.setattr(
        skeleton_module._Builder, "_finish", capture_full_reseed
    )
    prepared = prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    assert prepared.outcome.value == "SKELETON_DID_NOT_CLOSE"
    assert len(causal_receipts) == 1
    assert full_reseed_receipts == [((102, 137, 145, 148), 0, 0)]


def test_split_family_normal_form_materializes_comb_2_once(monkeypatch):
    polygon = _CORPUS["comb_2"]
    original = closure_module.plan_split_materialization
    captured = []

    def observed(snapshot):
        result = original(snapshot)
        if any(len(family.contacts) > 1 for family in result.families):
            reverse = original(
                replace(
                    snapshot,
                    incidents=tuple(reversed(snapshot.incidents)),
                )
            )
            captured.append((result, reverse))
        return result

    monkeypatch.setattr(
        closure_module, "plan_split_materialization", observed
    )
    skeleton = build_skeleton(polygon)

    assert len(captured) == 1
    forward, backward = captured[0]
    assert forward == backward
    assert forward.unresolved_reason is None
    assert len(forward.families) == 1
    family = forward.families[0]
    assert tuple(
        contact.key.point_key[0] for contact in family.contacts
    ) == (
        ((1, Fraction(18)),),
        ((1, Fraction(30)),),
    )
    assert len(family.segments) == 3
    assert tuple(
        (
            segment.start,
            segment.end,
        )
        for segment in family.segments
    ) == (
        (
            None,
            family.contacts[0].key,
        ),
        (family.contacts[0].key, family.contacts[1].key),
        (family.contacts[1].key, None),
    )
    assert len(family.births) == 4
    assert skeleton.outcome is SkeletonOutcome.EXACT
    assert semantic_digest(skeleton) == (
        "9a1dffb281c70512a809e6428f31e01cdbd58ad0497f5b7512b5685aec8070c2"
    )


def test_cross_t8_edge_generations_are_stable_under_permutations(monkeypatch):
    """Первое EDGE-поколение наблюдается без выбора порядка свёртки."""

    from cftuv_envelope.wavefront import (
        superlevel_fixed_point as split_fixed_point,
    )
    from cftuv_envelope.wavefront import symbolic_edge_closure
    from cftuv_envelope.wavefront import symbolic_edge_fixed_point
    from cftuv_envelope.wavefront import symbolic_split_endpoint
    from cftuv_envelope.wavefront.symbolic_overlay import SymbolicOverlayV1

    polygon = next(
        case.polygon
        for case in weighted_wall_differential_corpus()
        if case.name == "cross_full_q_1"
    )
    original = superlevel_module.apply_superlevel_transaction
    captured = []

    class _GateReached(Exception):
        pass

    def observed(builder, level):
        if superlevel_module._time_key(level[0].time)[0] != Fraction(8):
            return original(builder, level)
        snapshots = (
            superlevel_module.collect_superlevel_snapshot(builder, level),
            superlevel_module.collect_superlevel_snapshot(
                builder, tuple(reversed(level))
            ),
        )
        for snapshot in snapshots:
            split = split_fixed_point.plan_symbolic_split_fixed_point(
                builder, snapshot, budget=1
            )
            assert split.unresolved_reason is None
            assert split.iterations == 1
            assert len(split.added_contacts) == 1
            split_contact = split.added_contacts[0]
            assert split_contact.key.emitter.kind == "EXISTING"
            assert split_contact.key.family.occurrence[0] == (4, 8, 0, 8)
            assert split_contact.key.point_key == (
                ((1, Fraction(6)),),
                ((1, Fraction(6)),),
            )
            assert split.overlay is not None
            geometry_projection = tuple(sorted((
                (
                    vertex.point.x.terms,
                    vertex.point.y.terms,
                    vertex.prev_leaf.family.participant_keys,
                    vertex.next_leaf.family.participant_keys,
                )
                for vertex in split.overlay.vertices.values()
                if vertex.alive
            ), key=repr))
            leaf_projection = tuple(sorted((
                (leaf.family.participant_keys, leaf.occurrence)
                for leaf in split.overlay.spans
            ), key=repr))
            # Дайджест проекции переснят: "не этим числом". Сдвиг дала плотная
            # гидратация — в F0 больше нет line-only портов, и множество живых
            # вершин overlay стало полным. Предмет теста при этом ПРОВЕРЕН, а не
            # ослаблен: цикл идёт по прямому и развёрнутому пакету, и обе
            # стороны обязаны дать ОДНО и то же число — стабильность под
            # перестановками и есть утверждение.
            assert hashlib.sha256(repr(geometry_projection).encode()).hexdigest() == (
                "cd72b1fc91830d68cbee219e8c8172dcd8d022d04974a5df58065a37077dbca8"
            )
            # Та же пересъёмка и по той же причине, только адреснее: проекция
            # листьев несёт САМО вхождение вместе с концами, а плотная
            # гидратация как раз концы и заполнила. Стабильность под
            # перестановками проверяется тем же циклом.
            assert hashlib.sha256(repr(leaf_projection).encode()).hexdigest() == (
                "3a21d9b0703fa2918890a2e0c03e5b68a22083dac0f9bace2e42914ab5be9571"
            )
            assert all(
                len(vertex.ref.key) == 2
                for vertex in split.overlay.vertices.values()
                if vertex.ref.kind == "EXISTING"
            )
            overlays = (
                split.overlay,
                SymbolicOverlayV1(
                    dict(reversed(tuple(split.overlay.vertices.items()))),
                    dict(reversed(tuple(split.overlay.spans.items()))),
                    set(split.overlay.changed),
                    split.overlay.time,
                ),
            )
            for overlay in overlays:
                endpoint = symbolic_split_endpoint.plan_endpoint_generations(
                    builder, overlay, budget=1
                )
                assert endpoint.unresolved_reason is None
                assert endpoint.generations == ()
                assert endpoint.overlay is not None
                assert symbolic_edge_fixed_point.overlay_signature(
                    endpoint.overlay
                ) == symbolic_edge_fixed_point.overlay_signature(overlay)
                contacts = (
                    symbolic_edge_closure.discover_symbolic_edge_contacts(
                        builder, endpoint.overlay
                    )
                )
                assert len(contacts) == 1
                fixed = symbolic_edge_fixed_point.plan_symbolic_edge_generations(
                    builder, endpoint.overlay, budget=3
                )
                assert fixed.unresolved_reason is None
                assert fixed.overlay is not None
                assert tuple(len(item.contacts) for item in fixed.generations) == (
                    1,
                    1,
                    1,
                )
                assert not symbolic_edge_closure.discover_symbolic_edge_contacts(
                    builder, fixed.overlay
                )
                captured.append(
                    (
                        tuple(
                            tuple(contact.key for contact in item.contacts)
                            for item in fixed.generations
                        ),
                        symbolic_edge_fixed_point.overlay_signature(fixed.overlay),
                    )
                )
        raise _GateReached

    monkeypatch.setattr(
        superlevel_module, "apply_superlevel_transaction", observed
    )
    with pytest.raises(_GateReached):
        build_skeleton(polygon)

    assert captured and all(result == captured[0] for result in captured)
    trace, _ = captured[0]
    assert sum(map(len, trace)) == 3
    assert len({contact.point_key for generation in trace for contact in generation}) == 1
    key = trace[0][0]
    assert key.point_key == (
        ((1, Fraction(14, 3)),),
        ((1, Fraction(6)),),
    )
    assert tuple(key.__dataclass_fields__) == (
        "time_key",
        "point_key",
        "start",
        "end",
        "prev_family",
        "shared_family",
        "next_family",
    )
    assert tuple(key.start.__dataclass_fields__) == ("kind", "key")
    assert tuple(key.end.__dataclass_fields__) == ("kind", "key")
    assert "runtime_id" not in repr(key)


def test_nested_split_contacts_rebind_to_one_root_family(monkeypatch):
    from cftuv_envelope.wavefront import superlevel_fixed_point as fixed
    from cftuv_envelope.wavefront.symbolic_overlay import (
        JunctionRefV1,
        SymbolicOverlayV1,
        SymbolicSpanBindingV1,
        SymbolicVertexV1,
    )

    class Builder:
        def __init__(self):
            coordinates = ((0, -1), (1, -1), (2, -1), (-1, 0), (3, 0))
            self.points = tuple(
                EventPointV1(SqrtSumV1.rational(x), SqrtSumV1.rational(y))
                for x, y in coordinates
            )
            keys = (
                (0, -1, 0, -1),
                (1, -1, 1, -1),
                (2, -1, 2, -1),
                (-1, 0, 3, 0),
                (3, 0, -1, 0),
            )
            self.edges = [
                SimpleNamespace(
                    key=key,
                    line=SimpleNamespace(a=0, b=1),
                )
                for key in keys
            ]
            topology = (
                (0, 0, 0, 0),
                (1, 1, 1, 1),
                (2, 2, 2, 2),
                (4, 4, 4, 3),
                (3, 3, 3, 4),
            )
            self.vertices = [
                SimpleNamespace(
                    ident=index,
                    prev=prev,
                    next=next_,
                    prev_edge=prev_edge,
                    next_edge=next_edge,
                    alive=True,
                    reflex=True,
                    sliding=None,
                )
                for index, (prev, next_, prev_edge, next_edge)
                in enumerate(topology)
            ]

        def _position(self, vertex, time):
            return self.points[vertex.ident]

        def _proof_edge_endpoint_ids(self, edge_id):
            return (3, 4) if edge_id == 3 else ()

        def _edge_keys(self, *edge_ids):
            return tuple(self.edges[index].key for index in edge_ids)

    builder = Builder()
    required = {edge.key for edge in builder.edges}
    point_keys, occurrences, duplicate = superlevel_module._sparse_occurrences(
        builder, ZERO_TIME, required
    )
    assert not duplicate
    vertices = tuple(
        superlevel_module._vertex_snapshot(
            builder,
            vertex,
            point_keys[vertex.ident],
            occurrences,
        )
        for vertex in builder.vertices
    )
    root_occurrence = occurrences[3]
    root_family = closure_module.SpanFamilyRefV1(
        root_occurrence, (builder.edges[3].key,)
    )
    initial_event = CandidateEventV1(
        EventKind.SPLIT, ZERO_TIME, builder.points[0], 0, -1, 3
    )
    initial = superlevel_module._incident(builder, initial_event, vertices)
    snapshot = superlevel_module.SuperlevelSnapshotV1(
        (initial,), vertices, (), 0
    )
    cut, dropped, valid = superlevel_module._split_cut_plans(
        (initial,), vertices
    )
    assert valid and dropped == 0
    normal = closure_module._normal_form(cut[0], {initial.event: initial})
    assert normal is not None

    candidate_points = {1: builder.points[1], 2: builder.points[2]}
    refs = {
        ident: JunctionRefV1(
            "EXISTING", superlevel_module._port_identity(vertices[ident])
        )
        for ident in (1, 2)
    }

    def candidate(view, emitter_ref, leaf, *, now, **kwargs):
        ident = next(index for index, ref in refs.items() if ref == emitter_ref)
        return SimpleNamespace(
            candidate=SimpleNamespace(
                time=ZERO_TIME,
                point=candidate_points[ident],
                at_start=False,
                at_end=False,
            )
        )

    monkeypatch.setattr(fixed, "exact_overlay_view", lambda *args: None)
    monkeypatch.setattr(fixed, "evaluate_split_candidate", candidate)

    def discover(ident, leaf, reverse, expected_reason=None):
        emitter_occurrence = occurrences[ident]
        emitter_family = closure_module.SpanFamilyRefV1(
            emitter_occurrence, (builder.edges[ident].key,)
        )
        emitter_leaf = closure_module.SegmentRefV1(
            emitter_family, None, None, emitter_occurrence
        )
        emitter = SymbolicVertexV1(
            refs[ident], None, None, emitter_leaf, emitter_leaf,
            ZERO_TIME, builder.points[ident], None, frozenset(),
            runtime_id=ident,
        )
        bindings = (
            (leaf, SymbolicSpanBindingV1(leaf, 3, None, None)),
            (
                emitter_leaf,
                SymbolicSpanBindingV1(emitter_leaf, ident, None, None),
            ),
        )
        overlay = SymbolicOverlayV1(
            {emitter.ref: emitter},
            dict(reversed(bindings) if reverse else bindings),
            {leaf},
            ZERO_TIME,
        )
        contacts, reason = fixed._latent_contacts(builder, overlay)
        if expected_reason is not None:
            assert contacts == () and reason == expected_reason
            return None
        assert reason is None and len(contacts) == 1
        return contacts[0]

    root_leaf = closure_module.SegmentRefV1(
        root_family, None, None, root_occurrence
    )
    root_first = discover(1, root_leaf, False)
    first_forward = discover(1, normal.segments[-1], False)
    first_reverse = discover(1, normal.segments[-1], True)
    assert root_first.key == first_forward.key
    assert root_first.key.participants == first_forward.key.participants
    assert first_forward == first_reverse
    first_compiled, reason = fixed._compile_contacts(
        builder, snapshot, (first_forward,)
    )
    assert reason is None
    first_cut, _, valid = superlevel_module._split_cut_plans(
        (initial, *first_compiled), vertices
    )
    assert valid
    first_normal = closure_module._normal_form(
        first_cut[0],
        {item.event: item for item in (initial, *first_compiled)},
    )
    child_leaf = first_normal.segments[-1]
    assert child_leaf.occurrence != root_occurrence

    second_forward = discover(2, child_leaf, False)
    second_reverse = discover(2, child_leaf, True)
    assert second_forward == second_reverse
    assert second_forward.key.family == root_family
    projections = []
    for contacts in (
        (first_forward, second_forward),
        (second_reverse, first_reverse),
    ):
        compiled, reason = fixed._compile_contacts(builder, snapshot, contacts)
        assert reason is None
        assert all(item.target_occurrence == root_occurrence for item in compiled)
        assert all(item.target_occurrence != child_leaf.occurrence for item in compiled)
        planned, dropped, valid = superlevel_module._split_cut_plans(
            (initial, *compiled), vertices
        )
        assert valid and dropped == 0 and len(planned) == 1
        flattened = closure_module._normal_form(
            planned[0],
            {item.event: item for item in (initial, *compiled)},
        )
        projections.append(flattened)
    assert projections[0] == projections[1]
    assert len(projections[0].contacts) == 3
    assert len(projections[0].segments) == 4
    assert projections[0].family == root_family

    for at_start, at_end in ((True, False), (False, True)):
        def endpoint_candidate(view, emitter_ref, leaf, *, now, **kwargs):
            return SimpleNamespace(
                candidate=SimpleNamespace(
                    time=ZERO_TIME,
                    point=builder.points[1],
                    at_start=at_start,
                    at_end=at_end,
                )
            )

        monkeypatch.setattr(fixed, "evaluate_split_candidate", endpoint_candidate)
        discover(
            1,
            root_leaf,
            False,
            "SYMBOLIC_SPLIT_ENDPOINT_JUNCTION_REQUIRED",
        )


def _symbolic_edge_fixture(routes):
    from cftuv_envelope.wavefront import symbolic_edge_closure
    from cftuv_envelope.wavefront.symbolic_overlay import (
        JunctionRefV1,
        SymbolicOverlayV1,
        SymbolicSpanBindingV1,
        SymbolicVertexV1,
    )

    point_key = (((1, Fraction(3)),), ((1, Fraction(5)),))
    point = EventPointV1(SqrtSumV1(point_key[0]), SqrtSumV1(point_key[1]))
    labels = sorted({label for _, edges in routes for label in edges})
    leaves = {}
    for index, label in enumerate(labels):
        occurrence = ((index, index + 1), point_key, point_key)
        family = closure_module.SpanFamilyRefV1(
            occurrence, ((index, index + 1),)
        )
        leaves[label] = closure_module.SegmentRefV1(
            family, None, None, occurrence
        )
    refs = {
        name: JunctionRefV1("TEST", (name,))
        for names, _ in routes
        for name in names
    }
    vertices = {}
    starts = {}
    ends = {}
    for names, edge_labels in routes:
        for index, name in enumerate(names):
            ref = refs[name]
            previous = refs[names[index - 1]]
            following = refs[names[(index + 1) % len(names)]]
            prev_leaf = leaves[edge_labels[index - 1]]
            next_leaf = leaves[edge_labels[index]]
            vertices[ref] = SymbolicVertexV1(
                ref,
                previous,
                following,
                prev_leaf,
                next_leaf,
                ZERO_TIME,
                point,
                None,
                frozenset({("TEST", name)}),
            )
            starts.setdefault(next_leaf, ref)
            ends.setdefault(prev_leaf, ref)
    spans = {
        leaf: SymbolicSpanBindingV1(
            leaf, index, starts.get(leaf), ends.get(leaf)
        )
        for index, leaf in enumerate(leaves.values())
    }
    overlay = SymbolicOverlayV1(
        vertices, spans, set(leaves.values()), ZERO_TIME
    )

    def contact(start_name):
        start = vertices[refs[start_name]]
        end = vertices[start.next]
        key = symbolic_edge_closure.SymbolicEdgeContactKeyV1(
            superlevel_module._time_key(ZERO_TIME),
            point_key,
            start.ref,
            end.ref,
            start.prev_leaf.family,
            start.next_leaf.family,
            end.next_leaf.family,
        )
        return symbolic_edge_closure.SymbolicEdgeContactV1(
            key,
            start.prev_leaf,
            start.next_leaf,
            end.next_leaf,
            False,
            tuple(
                sorted(
                    {
                        participant
                        for leaf in (
                            start.prev_leaf,
                            start.next_leaf,
                            end.next_leaf,
                        )
                        for participant in leaf.family.participant_keys
                    }
                )
            ),
        )

    return overlay, contact


def test_same_generation_edge_component_is_one_order_free_delta():
    from cftuv_envelope.wavefront import symbolic_edge_fixed_point as fixed
    from cftuv_envelope.wavefront.symbolic_overlay import SymbolicOverlayV1

    overlay, contact = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    contacts = (contact("A"), contact("B"))
    permuted_overlay = SymbolicOverlayV1(
        dict(reversed(tuple(overlay.vertices.items()))),
        dict(reversed(tuple(overlay.spans.items()))),
        set(overlay.changed),
        overlay.time,
    )
    before = fixed.overlay_signature(overlay)
    results = []
    for candidate_overlay in (overlay, permuted_overlay):
        for order in (contacts, tuple(reversed(contacts))):
            generation, reason = fixed.normalize_edge_generation(
                candidate_overlay, order
            )
            assert reason is None
            assert generation is not None
            assert len(generation.deltas) == 1
            result, reason = fixed.apply_edge_generation(
                candidate_overlay, generation
            )
            assert reason is None
            results.append((generation, fixed.overlay_signature(result)))
    assert all(result == results[0] for result in results)
    assert fixed.overlay_signature(overlay) == before
    assert sorted(row[0].kind for row in results[0][1][0]) == [
        "EDGE_JUNCTION",
        "TEST",
    ]


def test_ambiguous_edge_component_ports_refuse_without_mutation():
    from cftuv_envelope.wavefront import symbolic_edge_fixed_point as fixed

    overlay, contact = _symbolic_edge_fixture(
        (
            (("P", "A", "B"), ("L0", "L1", "L2")),
            (("Q", "C", "D"), ("L0", "L3", "L4")),
        )
    )
    before = fixed.overlay_signature(overlay)
    generation, reason = fixed.normalize_edge_generation(
        overlay, (contact("A"), contact("C"))
    )
    assert generation is None
    assert reason == "SYMBOLIC_EDGE_PORT_MATCHING_AMBIGUOUS"
    assert fixed.overlay_signature(overlay) == before


def test_edge_generation_refuses_duplicate_active_span_owner():
    from cftuv_envelope.wavefront import symbolic_edge_fixed_point as fixed

    overlay, contact = _symbolic_edge_fixture(
        (
            (("D", "A", "B", "C"), ("L0", "L1", "L2", "L3")),
            (("X", "Y"), ("L0", "L4")),
        )
    )
    before = fixed.overlay_signature(overlay)
    generation, reason = fixed.normalize_edge_generation(
        overlay, (contact("A"), contact("B"))
    )
    assert reason is None
    result, reason = fixed.apply_edge_generation(overlay, generation)
    assert result is None
    assert reason == "SYMBOLIC_EDGE_SPAN_OWNER_AMBIGUOUS"
    assert fixed.overlay_signature(overlay) == before


def test_endpoint_start_end_aliases_have_one_stable_junction_delta(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_edge_fixed_point as edge_fixed
    from cftuv_envelope.wavefront import symbolic_split_endpoint as endpoint
    from cftuv_envelope.wavefront.symbolic_overlay import (
        SymbolicOverlayV1,
        SymbolicSpanBindingV1,
    )

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    vertices = dict(overlay.vertices)
    vertices[refs["A"]] = replace(vertices[refs["A"]], runtime_id=0)
    start_leaf = vertices[refs["B"]].next_leaf
    end_leaf = closure_module.SegmentRefV1(
        start_leaf.family, None, None, ("END_ALIAS",)
    )
    spans = dict(overlay.spans)
    spans[end_leaf] = SymbolicSpanBindingV1(
        end_leaf,
        spans[start_leaf].physical_edge_id,
        refs["C"],
        refs["B"],
    )
    builder = SimpleNamespace(
        vertices=[SimpleNamespace(reflex=True, sliding=None)]
    )
    point = vertices[refs["B"]].point
    monkeypatch.setattr(endpoint, "exact_overlay_view", lambda *args: None)
    results = []
    for target, at_start, at_end in (
        (start_leaf, True, False),
        (end_leaf, False, True),
    ):
        def candidate(view, emitter_ref, leaf, *, now, **kwargs):
            return SimpleNamespace(
                candidate=None if leaf != target else SimpleNamespace(
                    time=ZERO_TIME,
                    point=point,
                    at_start=at_start,
                    at_end=at_end,
                )
            )

        monkeypatch.setattr(endpoint, "evaluate_split_candidate", candidate)
        for reverse in (False, True):
            candidate_overlay = SymbolicOverlayV1(
                dict(reversed(tuple(vertices.items()))) if reverse else dict(vertices),
                dict(reversed(tuple(spans.items()))) if reverse else dict(spans),
                {start_leaf, end_leaf},
                ZERO_TIME,
            )
            before = edge_fixed.overlay_signature(candidate_overlay)
            contacts, reason = endpoint.discover_endpoint_contacts(
                builder, candidate_overlay
            )
            assert reason is None and len(contacts) == 1
            generation, reason = endpoint.normalize_endpoint_generation(
                candidate_overlay, tuple(reversed(contacts))
            )
            assert reason is None and len(generation.deltas) == 1
            result, reason = endpoint.apply_endpoint_generation(
                candidate_overlay, generation
            )
            assert reason is None
            fixed = endpoint.plan_endpoint_generations(
                builder, candidate_overlay, budget=1
            )
            assert fixed.unresolved_reason is None
            assert len(fixed.generations) == 1
            assert fixed.overlay is not None
            assert edge_fixed.overlay_signature(
                fixed.overlay
            ) == edge_fixed.overlay_signature(result)
            assert edge_fixed.overlay_signature(candidate_overlay) == before
            results.append(
                (contacts, generation, edge_fixed.overlay_signature(result))
            )
    assert all(result == results[0] for result in results)
    assert any(
        row[0].kind == "ENDPOINT_JUNCTION"
        for row in results[0][2][0]
    )


def test_ambiguous_endpoint_ports_refuse_without_mutation():
    from cftuv_envelope.wavefront import symbolic_edge_fixed_point as edge_fixed
    from cftuv_envelope.wavefront import symbolic_split_endpoint as endpoint
    from cftuv_envelope.wavefront.symbolic_overlay import SymbolicOverlayV1

    overlay, _ = _symbolic_edge_fixture(
        (
            (("P", "A", "B"), ("L0", "L1", "L2")),
            (("Q", "C", "D"), ("L0", "L3", "L4")),
        )
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    family = overlay.vertices[refs["A"]].prev_leaf.family
    point = overlay.vertices[refs["A"]].point
    point_key = (point.x.terms, point.y.terms)
    contacts = tuple(
        endpoint.EndpointContactKeyV1(
            superlevel_module._time_key(ZERO_TIME),
            point_key,
            refs[emitter],
            refs[target],
            family,
            family.participant_keys,
        )
        for emitter, target in (("A", "B"), ("C", "D"))
    )
    variants = (
        overlay,
        SymbolicOverlayV1(
            dict(reversed(tuple(overlay.vertices.items()))),
            dict(reversed(tuple(overlay.spans.items()))),
            set(overlay.changed),
            overlay.time,
        ),
    )
    for candidate_overlay in variants:
        before = edge_fixed.overlay_signature(candidate_overlay)
        for order in (contacts, tuple(reversed(contacts))):
            generation, reason = endpoint.normalize_endpoint_generation(
                candidate_overlay, order
            )
            assert generation is None
            assert reason == "SYMBOLIC_ENDPOINT_PORT_MATCHING_AMBIGUOUS"
            assert edge_fixed.overlay_signature(candidate_overlay) == before


def test_unified_mixed_edge_endpoint_component_is_one_order_free_delta():
    from cftuv_envelope.wavefront import symbolic_junction_fixed_point as mixed
    from cftuv_envelope.wavefront import symbolic_split_endpoint as endpoint
    from cftuv_envelope.wavefront.symbolic_overlay import SymbolicOverlayV1

    overlay, contact = _symbolic_edge_fixture(
        ((("D", "A", "B", "C"), ("L0", "L1", "L2", "L3")),)
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    edge = mixed._edge_contact(contact("A"))
    target = overlay.vertices[refs["B"]].next_leaf
    point = overlay.vertices[refs["B"]].point
    key = endpoint.EndpointContactKeyV1(
        superlevel_module._time_key(ZERO_TIME),
        (point.x.terms, point.y.terms),
        refs["D"], refs["B"], target.family,
        target.family.participant_keys,
    )
    endpoint_contact = mixed._endpoint_contact(overlay, key)
    permuted = SymbolicOverlayV1(
        dict(reversed(tuple(overlay.vertices.items()))),
        dict(reversed(tuple(overlay.spans.items()))),
        set(overlay.changed), overlay.time,
    )
    results = []
    for candidate in (overlay, permuted):
        for contacts in ((edge, endpoint_contact), (endpoint_contact, edge)):
            generation, reason = mixed.normalize_junction_generation(
                SimpleNamespace(), candidate, contacts
            )
            assert reason is None and len(generation.deltas) == 1
            assert {item.kind for item in generation.contacts} == {
                "EDGE", "ENDPOINT"
            }
            result, reason = mixed.apply_junction_generation(
                candidate, generation
            )
            assert reason is None
            results.append((generation, mixed.overlay_signature(result)))
    assert all(result == results[0] for result in results)


def test_initial_edge_seed_is_not_lost_on_unchanged_overlay(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_junction_fixed_point as mixed

    overlay, contact = _symbolic_edge_fixture(
        ((("D", "A", "B", "C"), ("L0", "L1", "L2", "L3")),)
    )
    seed = mixed._edge_contact(contact("A"))
    monkeypatch.setattr(
        mixed, "discover_junction_contacts", lambda *args: ((), None)
    )
    fixed = mixed.plan_symbolic_junction_generations(
        SimpleNamespace(), overlay, seeds=(seed,), budget=1
    )
    assert fixed.unresolved_reason is None
    assert len(fixed.generations) == 1
    assert fixed.generations[0].contacts == (seed,)
    assert fixed.overlay is not None


def test_existing_symbolic_ref_is_invariant_under_exact_endpoint_hydration():
    edge_a = (0, 0, 4, 0)
    edge_b = (4, 0, 4, 4)
    point = (((1, Fraction(4)),), ((1, Fraction(0)),))
    sparse = SimpleNamespace(
        prev_occurrence=(edge_a, None, point),
        next_occurrence=(edge_b, point, None),
    )
    hydrated = SimpleNamespace(
        prev_occurrence=(edge_a, (((1, Fraction(0)),), point[1]), point),
        next_occurrence=(edge_b, point, (point[0], ((1, Fraction(4)),))),
    )
    assert superlevel_module._port_identity(sparse) == (
        edge_a, edge_b
    )
    assert superlevel_module._port_identity(sparse) == (
        superlevel_module._port_identity(hydrated)
    )


def test_adjacent_symbolic_deltas_compose_with_reciprocal_alive_ports():
    from cftuv_envelope.wavefront import symbolic_component as component

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    first, reason = component.normalize_dead_component(
        overlay,
        contact_keys=(("K1",),),
        dead_refs=(refs["D"],),
        point_key=(((1, Fraction(3)),), ((1, Fraction(5)),)),
        birth_kind="JUNCTION",
        stale_reason="STALE",
        ambiguity_reason="AMBIGUOUS",
    )
    assert reason is None
    second, reason = component.normalize_dead_component(
        overlay,
        contact_keys=(("K2",),),
        dead_refs=(refs["A"], refs["B"]),
        point_key=(((1, Fraction(4)),), ((1, Fraction(5)),)),
        birth_kind="JUNCTION",
        stale_reason="STALE",
        ambiguity_reason="AMBIGUOUS",
    )
    assert reason is None
    result, reason = component.apply_component_deltas(
        overlay,
        (first, second),
        collision_reason="SYMBOLIC_JUNCTION_COMPONENT_DELTAS_OVERLAP",
    )
    assert reason is None and result is not None
    alive = {ref: vertex for ref, vertex in result.vertices.items()
             if vertex.alive}
    assert len(alive) == 3
    assert all(
        vertex.prev in alive and vertex.next in alive
        and alive[vertex.prev].next == ref
        and alive[vertex.next].prev == ref
        for ref, vertex in alive.items()
    )


def test_overlay_signature_covers_every_candidate_affecting_field():
    from cftuv_envelope.wavefront.symbolic_component import (
        clone_overlay, overlay_signature,
    )

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    baseline = overlay_signature(overlay)
    ref = next(iter(overlay.vertices))
    leaf = next(iter(overlay.spans))
    variants = []
    candidate = clone_overlay(overlay)
    candidate.spans[leaf] = replace(
        candidate.spans[leaf], physical_edge_id=999
    )
    variants.append(candidate)
    candidate = clone_overlay(overlay)
    candidate.vertices[ref].point = EventPointV1(
        SqrtSumV1.rational(99), SqrtSumV1.rational(101)
    )
    variants.append(candidate)
    candidate = clone_overlay(overlay)
    candidate.vertices[ref].birth = CandidateEventV1(
        EventKind.EDGE, ZERO_TIME,
        candidate.vertices[ref].point, 0, 0, -1,
    ).time
    candidate.vertices[ref].birth = replace(
        candidate.vertices[ref].birth, dividend=Fraction(1)
    )
    variants.append(candidate)
    candidate = clone_overlay(overlay)
    candidate.vertices[ref].sliding = SqrtSumV1.rational(7)
    variants.append(candidate)
    candidate = clone_overlay(overlay)
    candidate.vertices[ref].provenance = frozenset({("CHANGED",)})
    variants.append(candidate)
    assert all(overlay_signature(item) != baseline for item in variants)


def test_seed_and_discovered_contact_metadata_conflict_is_named(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_junction_fixed_point as mixed
    from cftuv_envelope.wavefront.symbolic_junction_contacts import edge_contact

    overlay, make_contact = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    seed = edge_contact(replace(
        make_contact("A"), span_unproven=True,
        participant_keys=((111,),),
    ))
    discovered = edge_contact(replace(
        make_contact("A"), span_unproven=False,
        participant_keys=((222,),),
    ))
    monkeypatch.setattr(
        mixed, "discover_junction_contacts",
        lambda *args: ((discovered,), None),
    )
    fixed = mixed.plan_symbolic_junction_generations(
        SimpleNamespace(), overlay, seeds=(seed,), budget=1
    )
    assert fixed.unresolved_reason == (
        "SYMBOLIC_JUNCTION_CONTACT_METADATA_CONFLICT"
    )


def test_cross_t8_raw_f0_admits_external_and_symbolic_born_emitters(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer
    from cftuv_envelope.wavefront.symbolic_component import clone_overlay
    from cftuv_envelope.wavefront.symbolic_f0_overlay import build_f0_overlay
    from cftuv_envelope.wavefront.symbolic_overlay import JunctionRefV1

    polygon = next(
        case.polygon for case in weighted_wall_differential_corpus()
        if case.name == "cross_full_q_1"
    )
    original = superlevel_module.apply_superlevel_transaction

    class _GateReached(Exception):
        pass

    def observed(builder, level):
        if superlevel_module._time_key(level[0].time)[0] != Fraction(8):
            return original(builder, level)
        snapshot = superlevel_module.collect_superlevel_snapshot(builder, level)
        raw = build_f0_overlay(builder, snapshot, level[0].time)
        assert raw is not None
        live = {vertex.ident for vertex in builder.vertices if vertex.alive}
        admitted = {
            vertex.runtime_id for vertex in raw.vertices.values()
            if vertex.runtime_id is not None
        }
        assert admitted == live
        assert {2, 5, 8, 11}.issubset(admitted)
        assert all(
            vertex.prev is not None and vertex.next is not None
            for vertex in raw.vertices.values()
        )
        changed = clone_overlay(raw)
        changed.changed = set(changed.spans)
        evaluated = set()

        def no_candidate(view, emitter_ref, leaf, *, now, **kwargs):
            evaluated.add(emitter_ref)
            return SimpleNamespace(candidate=None)

        monkeypatch.setattr(outer, "evaluate_split_candidate", no_candidate)
        assert outer.discover_interior_split_contacts(builder, changed) == ((), None)
        evaluated_runtime = {
            vertex.runtime_id for vertex in changed.vertices.values()
            if vertex.ref in evaluated
        }
        assert {2, 5, 8, 11}.issubset(evaluated_runtime)

        reflex = changed.vertices[next(
            ref for ref, vertex in changed.vertices.items()
            if vertex.runtime_id == 2
        )]
        target = next(
            leaf for leaf in changed.spans
            if leaf not in (reflex.prev_leaf, reflex.next_leaf)
        )
        changed.changed = {target}
        born_ref = JunctionRefV1("BIRTH", (("SYNTHETIC_REFLEX",),))
        born = replace(
            reflex, ref=born_ref, runtime_id=None,
            provenance=frozenset({("SYNTHETIC_REFLEX",)}),
        )
        changed.vertices[born_ref] = born

        def born_candidate(view, emitter_ref, leaf, *, now, **kwargs):
            return SimpleNamespace(candidate=(
                None if emitter_ref != born_ref or leaf != target
                else SimpleNamespace(
                    time=now,
                    point=born.point,
                    at_start=False,
                    at_end=False,
                )
            ))

        monkeypatch.setattr(outer, "evaluate_split_candidate", born_candidate)
        contacts, reason = outer.discover_interior_split_contacts(builder, changed)
        assert reason is None
        assert len(contacts) == 1
        assert contacts[0].key.emitter == born_ref
        raise _GateReached

    monkeypatch.setattr(superlevel_module, "apply_superlevel_transaction", observed)
    with pytest.raises(_GateReached):
        build_skeleton(polygon)


def test_interior_discovery_rejects_identity_equal_leaf_aliases(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    emitter = refs["A"]
    target = next(
        leaf for leaf in overlay.spans
        if leaf not in (
            overlay.vertices[emitter].prev_leaf,
            overlay.vertices[emitter].next_leaf,
        )
    )
    alias = replace(target, start=("ALIASED_CHILD",))
    overlay.spans[alias] = replace(overlay.spans[target], leaf=alias)
    overlay.changed = {target, alias}
    point = overlay.vertices[emitter].point
    evaluated = []

    monkeypatch.setattr(
        outer,
        "is_symbolic_split_emitter",
        lambda builder, candidate_overlay, vertex: vertex.ref == emitter,
    )

    def same_candidate(view, emitter_ref, leaf, *, now, **kwargs):
        evaluated.append(leaf)
        return SimpleNamespace(candidate=SimpleNamespace(
            time=now,
            point=point,
            at_start=False,
            at_end=False,
        ))

    monkeypatch.setattr(outer, "evaluate_split_candidate", same_candidate)
    line = SimpleNamespace(a=0, b=1)
    builder = SimpleNamespace(
        _prime_universe=(),
        edges=tuple(
            SimpleNamespace(line=line, span=(0, 0, 1, 0))
            for _ in overlay.spans
        ),
    )
    contacts, reason = outer.discover_interior_split_contacts(builder, overlay)
    assert contacts == ()
    assert reason == "SYMBOLIC_INTERIOR_SPLIT_CONTACT_METADATA_CONFLICT"
    assert set(evaluated) == {target, alias}


def test_symbolic_born_interior_split_applies_two_reciprocal_births():
    from cftuv_envelope.wavefront import symbolic_mixed_generation as mixed
    from cftuv_envelope.wavefront import superlevel_fixed_point as split_fixed
    from cftuv_envelope.wavefront.symbolic_overlay import (
        JunctionRefV1, SymbolicOverlayV1,
    )

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    old, born_ref = refs["A"], JunctionRefV1("BIRTH", (("CASCADE",),))
    vertices = dict(overlay.vertices)
    born = replace(vertices.pop(old), ref=born_ref, runtime_id=None)
    vertices[born_ref] = born
    for ref, vertex in tuple(vertices.items()):
        vertices[ref] = replace(
            vertex,
            prev=born_ref if vertex.prev == old else vertex.prev,
            next=born_ref if vertex.next == old else vertex.next,
        )
    spans = {
        leaf: replace(
            binding,
            start=born_ref if binding.start == old else binding.start,
            end=born_ref if binding.end == old else binding.end,
        )
        for leaf, binding in overlay.spans.items()
    }
    candidate = SymbolicOverlayV1(
        vertices, spans, set(overlay.changed), overlay.time
    )
    target = next(
        leaf for leaf in candidate.spans
        if leaf not in (born.prev_leaf, born.next_leaf)
    )
    point = EventPointV1(SqrtSumV1.rational(7), SqrtSumV1.rational(11))
    point_key = (point.x.terms, point.y.terms)
    key = split_fixed.SymbolicSplitContactKeyV1(
        superlevel_module._time_key(ZERO_TIME),
        point_key,
        born_ref,
        target.family,
        tuple(sorted({
            participant
            for leaf in (born.prev_leaf, born.next_leaf, target)
            for participant in leaf.family.participant_keys
        })),
    )
    contact = split_fixed.SymbolicSplitContactV1(
        key, ZERO_TIME, point, SqrtSumV1.rational(7), target
    )
    expanded, generation, reason = mixed.normalize_mixed_generation(
        SimpleNamespace(), candidate, (), (contact,)
    )
    assert reason is None and generation is not None
    result, reason = mixed.apply_mixed_generation(expanded, generation)
    assert reason is None and result is not None
    assert not result.vertices[born_ref].alive
    split_births = [
        vertex for vertex in result.vertices.values()
        if vertex.alive and vertex.ref.kind == "INTERIOR_SPLIT"
    ]
    assert len(split_births) == 2
    alive = {ref: vertex for ref, vertex in result.vertices.items()
             if vertex.alive}
    assert all(
        alive[vertex.prev].next == ref and alive[vertex.next].prev == ref
        for ref, vertex in alive.items()
    )


def test_sparse_line_only_child_accepts_next_interior_contact_with_unique_owners():
    from cftuv_envelope.wavefront import symbolic_mixed_generation as mixed
    from cftuv_envelope.wavefront import superlevel_fixed_point as split_fixed
    from cftuv_envelope.wavefront.symbolic_overlay import (
        SymbolicOverlayV1, SymbolicSpanBindingV1,
    )

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    emitter = overlay.vertices[refs["A"]]
    target = next(
        leaf for leaf in overlay.spans
        if leaf not in (emitter.prev_leaf, emitter.next_leaf)
    )
    binding = overlay.spans[target]
    sparse = closure_module.SegmentRefV1(
        target.family, target.start, target.end,
        (target.occurrence[0], None, None),
    )
    vertices = dict(overlay.vertices)
    vertices[binding.start] = replace(
        vertices[binding.start], next_leaf=sparse
    )
    vertices[binding.end] = replace(
        vertices[binding.end], prev_leaf=sparse
    )
    spans = dict(overlay.spans)
    del spans[target]
    spans[sparse] = SymbolicSpanBindingV1(
        sparse, binding.physical_edge_id, binding.start, binding.end
    )
    candidate = SymbolicOverlayV1(
        vertices, spans, {sparse}, overlay.time
    )

    def contact(ref, leaf, x):
        point = EventPointV1(
            SqrtSumV1.rational(x), SqrtSumV1.rational(11)
        )
        key = split_fixed.SymbolicSplitContactKeyV1(
            superlevel_module._time_key(ZERO_TIME),
            (point.x.terms, point.y.terms),
            ref,
            leaf.family,
            leaf.family.participant_keys,
        )
        return split_fixed.SymbolicSplitContactV1(
            key, ZERO_TIME, point, SqrtSumV1.rational(x), leaf
        )

    expanded, generation, reason = mixed.normalize_mixed_generation(
        SimpleNamespace(), candidate, (),
        (contact(emitter.ref, sparse, 7),),
    )
    assert reason is None
    first, reason = mixed.apply_mixed_generation(expanded, generation)
    assert reason is None and first is not None
    children = tuple(
        leaf for leaf in first.spans
        if leaf.family == sparse.family and leaf != sparse
    )
    assert len(children) == 2
    assert {child.occurrence[1:] for child in children} == {
        (None, (((1, Fraction(7)),), ((1, Fraction(11)),))),
        ((((1, Fraction(7)),), ((1, Fraction(11)),)), None),
    }
    child = next(item for item in children if item.occurrence[1] is None)
    next_emitter = next(
        vertex for vertex in first.vertices.values()
        if vertex.alive
        and child not in (vertex.prev_leaf, vertex.next_leaf)
    )
    expanded, generation, reason = mixed.normalize_mixed_generation(
        SimpleNamespace(), first, (),
        (contact(next_emitter.ref, child, 5),),
    )
    assert reason is None
    second, reason = mixed.apply_mixed_generation(expanded, generation)
    assert reason is None and second is not None
    alive = {ref: vertex for ref, vertex in second.vertices.items()
             if vertex.alive}
    assert all(
        alive[vertex.prev].next == ref and alive[vertex.next].prev == ref
        for ref, vertex in alive.items()
    )
    starts = {}
    ends = {}
    for ref, vertex in alive.items():
        starts.setdefault(vertex.next_leaf, []).append(ref)
        ends.setdefault(vertex.prev_leaf, []).append(ref)
    assert all(len(owners) == 1 for owners in (*starts.values(), *ends.values()))


def test_same_point_multi_emitter_interior_split_refuses_before_subdivision():
    from cftuv_envelope.wavefront import symbolic_mixed_generation as mixed
    from cftuv_envelope.wavefront import superlevel_fixed_point as split_fixed
    from cftuv_envelope.wavefront.symbolic_component import overlay_signature

    overlay, _ = _symbolic_edge_fixture((
        (("A", "B", "C", "D", "E", "F"),
         ("L0", "L1", "L2", "L3", "L4", "L5")),
    ))
    refs = {ref.key[0]: ref for ref in overlay.vertices}
    target = overlay.vertices[refs["C"]].next_leaf
    point = EventPointV1(SqrtSumV1.rational(7), SqrtSumV1.rational(11))
    point_key = (point.x.terms, point.y.terms)

    def contact(emitter, projection):
        key = split_fixed.SymbolicSplitContactKeyV1(
            superlevel_module._time_key(ZERO_TIME),
            point_key,
            refs[emitter],
            target.family,
            target.family.participant_keys,
        )
        return split_fixed.SymbolicSplitContactV1(
            key, ZERO_TIME, point, SqrtSumV1.rational(projection), target
        )

    before = overlay_signature(overlay)
    before_spans = set(overlay.spans)
    expanded, generation, reason = mixed.normalize_mixed_generation(
        SimpleNamespace(), overlay, (), (contact("A", 7), contact("F", 8))
    )
    assert expanded is None and generation is None
    assert reason == (
        "SYMBOLIC_INTERIOR_SPLIT_POINT_MULTIPLICITY_UNRESOLVABLE"
    )
    assert overlay_signature(overlay) == before
    assert set(overlay.spans) == before_spans
    assert not any(
        leaf.occurrence[1:] == (point_key, point_key)
        for leaf in overlay.spans
    )


def test_trace_bound_authority_changes_signature_and_candidate_view():
    from cftuv_envelope.wavefront.symbolic_component import (
        clone_overlay, overlay_signature,
    )
    from cftuv_envelope.wavefront.symbolic_overlay import exact_overlay_view

    @dataclass(frozen=True)
    class _Trace:
        crash_time: object | None

        def bounds_time(self, time):
            if self.crash_time is None:
                return False
            return self.crash_time.canonical() == time.canonical()

    overlay, _ = _symbolic_edge_fixture(
        ((('D', 'A', 'B', 'C'), ('L0', 'L1', 'L2', 'L3')),)
    )
    ref = next(iter(overlay.vertices))
    unavailable = clone_overlay(overlay)
    bounded = clone_overlay(overlay)
    bounded.vertices[ref].trace = _Trace(ZERO_TIME)
    no_crash = clone_overlay(overlay)
    no_crash.vertices[ref].trace = _Trace(None)
    assert len({
        overlay_signature(unavailable),
        overlay_signature(bounded),
        overlay_signature(no_crash),
    }) == 3
    builder = SimpleNamespace(_prime_universe=())
    assert exact_overlay_view(
        builder, unavailable
    ).trace_bounds(ref, ZERO_TIME) is None
    assert exact_overlay_view(
        builder, bounded
    ).trace_bounds(ref, ZERO_TIME) is True
    assert exact_overlay_view(
        builder, no_crash
    ).trace_bounds(ref, ZERO_TIME) is False


def test_cross_t8_edge_and_6_6_split_share_initial_mixed_generation(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer

    polygon = next(
        case.polygon for case in weighted_wall_differential_corpus()
        if case.name == "cross_full_q_1"
    )
    original = superlevel_module.apply_superlevel_transaction
    original_f0 = outer.build_f0_overlay

    class _GateReached(Exception):
        pass

    def observed(builder, level):
        if superlevel_module._time_key(level[0].time)[0] != Fraction(8):
            return original(builder, level)
        snapshot = superlevel_module.collect_superlevel_snapshot(builder, level)
        results = []
        for incidents in (
            snapshot.incidents, tuple(reversed(snapshot.incidents))
        ):
            f0_calls = 0

            def counted_f0(*args):
                nonlocal f0_calls
                f0_calls += 1
                return original_f0(*args)

            monkeypatch.setattr(outer, "build_f0_overlay", counted_f0)
            fixed = outer.plan_symbolic_superlevel_closure(
                builder,
                replace(snapshot, incidents=incidents),
                outer_budget=8,
                junction_budget=8,
            )
            assert f0_calls == 1
            assert fixed.unresolved_reason is None
            assert fixed.materialization is not None
            mixed_plans = [
                plan for plan in fixed.materialization.plans
                if plan.event_kinds == (EventKind.EDGE, EventKind.SPLIT)
            ]
            assert len(mixed_plans) == 1
            plan = mixed_plans[0]
            cut_points = {
                superlevel_module._event_point_key(event)
                for cut in plan.split_cuts for event in cut.events
            }
            assert (
                ((1, Fraction(6)),), ((1, Fraction(6)),)
            ) in cut_points
            assert any(
                contact.point_key == (
                    ((1, Fraction(2)),), ((1, Fraction(6)),)
                )
                for contact in plan.edge_contacts
            )
            assert all(
                not generation.interior_contacts
                for generation in fixed.junction.generations
            )
            results.append((
                fixed.materialization,
                outer.overlay_signature(fixed.overlay),
                fixed.canonical_batch_count,
            ))
        assert results[0] == results[1]
        raise _GateReached

    monkeypatch.setattr(superlevel_module, "apply_superlevel_transaction", observed)
    with pytest.raises(_GateReached):
        build_skeleton(polygon)


def test_outer_coordinator_replays_one_mixed_generation_from_same_f0(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_junction_fixed_point as mixed
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer

    overlay, _ = _symbolic_edge_fixture(
        ((("D", "A", "B", "C"), ("L0", "L1", "L2", "L3")),)
    )
    contact = SimpleNamespace(key=("STABLE_INTERIOR",))
    rebuild_sizes = []
    mixed_calls = 0

    def rebuild(builder, snapshot, contacts, time, **kwargs):
        rebuild_sizes.append(len(contacts))
        return SimpleNamespace(plans=()), overlay, None

    def plan_mixed(*args, **kwargs):
        nonlocal mixed_calls
        mixed_calls += 1
        return mixed.SymbolicJunctionFixedPointV1(
            (SimpleNamespace(),), overlay, (outer.overlay_signature(overlay),)
        ), (contact,)

    monkeypatch.setattr(outer, "build_f0_overlay", lambda *args: overlay)
    monkeypatch.setattr(outer, "initial_interior_contacts", lambda snapshot: ((), None))
    monkeypatch.setattr(outer, "_rebuild_splits", rebuild)
    monkeypatch.setattr(
        outer, "discover_interior_split_contacts", lambda *args: ((), None)
    )
    monkeypatch.setattr(outer, "plan_mixed_generations", plan_mixed)
    snapshot = SimpleNamespace(
        incidents=(SimpleNamespace(event=SimpleNamespace(time=ZERO_TIME)),)
    )
    fixed = outer.plan_symbolic_superlevel_closure(
        SimpleNamespace(), snapshot, outer_budget=1, junction_budget=1
    )
    assert fixed.unresolved_reason is None
    assert fixed.outer_iterations == 1
    assert fixed.split_contacts == (contact,)
    assert fixed.canonical_batch_count == 2
    # Stable full-component oracle одновременно является final materialization;
    # второй planning pass не должен создавать скрытый альтернативный закон.
    assert rebuild_sizes == [0]
    assert mixed_calls == 2


def test_outer_coordinator_names_changed_signature_on_stable_replay(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_junction_fixed_point as mixed
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer
    from cftuv_envelope.wavefront.symbolic_component import clone_overlay

    overlay, _ = _symbolic_edge_fixture(
        ((("D", "A", "B", "C"), ("L0", "L1", "L2", "L3")),)
    )
    contact = SimpleNamespace(key=("STABLE_INTERIOR",))
    mixed_calls = 0

    def plan_mixed(*args, **kwargs):
        nonlocal mixed_calls
        mixed_calls += 1
        result = clone_overlay(overlay)
        if mixed_calls == 2:
            next(iter(result.vertices.values())).alive = False
        return (mixed.SymbolicJunctionFixedPointV1(
            (SimpleNamespace(),), result, ()
        ), (contact,))

    monkeypatch.setattr(outer, "initial_interior_contacts", lambda snapshot: ((), None))
    monkeypatch.setattr(outer, "build_f0_overlay", lambda *args: overlay)
    monkeypatch.setattr(
        outer, "_rebuild_splits",
            lambda *args, **kwargs: (SimpleNamespace(plans=()), overlay, None),
    )
    monkeypatch.setattr(
        outer, "discover_interior_split_contacts", lambda *args: ((), None)
    )
    monkeypatch.setattr(outer, "plan_mixed_generations", plan_mixed)
    snapshot = SimpleNamespace(
        incidents=(SimpleNamespace(event=SimpleNamespace(time=ZERO_TIME)),)
    )
    fixed = outer.plan_symbolic_superlevel_closure(
        SimpleNamespace(), snapshot, outer_budget=1, junction_budget=1
    )
    assert fixed.unresolved_reason == (
        "SYMBOLIC_SUPERLEVEL_REPEATED_CONTACT_SET_CHANGED_SIGNATURE"
    )
    assert mixed_calls == 2


def test_real_e2_s4_packets_use_one_permutation_free_junction_batch(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_superlevel_coordinator as outer
    from cftuv_envelope.wavefront import superlevel_fixed_point as old_split
    from cftuv_envelope.wavefront import symbolic_junction_fixed_point as mixed
    from cftuv_envelope.wavefront.symbolic_f0_overlay import build_f0_overlay

    cases = {
        "cross": dict(named_corpus())["cross"],
        "u_shape": dict(named_corpus())["u_shape"],
        "staircase": dict(partial_source_corpus())[
            "staircase_source_edges_3_4"
        ],
    }
    original = superlevel_module.apply_superlevel_transaction
    rows = []

    def observed(builder, level):
        snapshot = superlevel_module.collect_superlevel_snapshot(builder, level)
        kinds = Counter(item.event.kind for item in snapshot.incidents)
        if kinds == Counter({EventKind.EDGE: 2, EventKind.SPLIT: 4}):
            projections = []
            for incidents in (
                snapshot.incidents, tuple(reversed(snapshot.incidents))
            ):
                candidate = replace(snapshot, incidents=incidents)
                fixed = outer.plan_symbolic_superlevel_closure(
                    builder, candidate, outer_budget=4, junction_budget=8
                )
                assert fixed.unresolved_reason is None
                assert fixed.overlay is not None and fixed.junction is not None
                assert fixed.canonical_batch_count == 1
                assert len(fixed.junction.generations) == 0
                assert fixed.materialization is not None
                assert all(
                    plan.resolution
                    is not superlevel_module.SuperlevelResolution.UNRESOLVABLE
                    for plan in fixed.materialization.plans
                )
                projections.append((
                    fixed.split_contacts,
                    fixed.junction.generations,
                    outer.overlay_signature(fixed.overlay),
                    fixed.canonical_batch_count,
                ))
            assert projections[0] == projections[1]
            rows.append((case_name, search.value,
                         superlevel_module._time_key(level[0].time)))
            if case_name == "cross" and level[0].time.canonical().dividend == 2:
                raw = build_f0_overlay(builder, snapshot, level[0].time)
                assert raw is not None and raw.changed == set()
                exact = {
                    occurrence
                    for vertex in snapshot.vertices
                    for occurrence in (
                        vertex.prev_occurrence, vertex.next_occurrence
                    )
                    if occurrence is not None
                }
                raw_exact = {
                    leaf.occurrence for leaf in raw.spans
                    if None not in leaf.occurrence[1:]
                }
                # Прежде здесь стояло `any(None in leaf.occurrence[1:])` —
                # свидетель РАЗРЕЖЕННОСТИ снимка: хотя бы один пролёт F0
                # оставался line-only, без вычисленных концов. Плотная
                # гидратация упразднила это по закону: снимок пакета несёт
                # ПОЛНОЕ множество ключей живых вхождений, и негидратированных
                # пролётов не остаётся ни одного. Свидетель снят, а утверждение
                # усилено обратным: множества совпадают ЦЕЛИКОМ, без остатка в
                # line-only.
                assert raw_exact == exact
                assert all(
                    None not in leaf.occurrence[1:] for leaf in raw.spans
                )
                post = old_split.plan_symbolic_split_fixed_point(
                    builder, snapshot, budget=1
                )
                assert post.unresolved_reason is None and post.overlay is not None
                seeds, reason = mixed.initial_junction_seeds(
                    snapshot, post.overlay
                )
                assert seeds == ()
                assert reason == "SYMBOLIC_INITIAL_EDGE_BINDING_UNRESOLVABLE"
        return original(builder, level)

    monkeypatch.setattr(
        superlevel_module, "apply_superlevel_transaction", observed
    )
    for case_name, polygon in cases.items():
        for search in SplitSearch:
            build_skeleton(polygon, split_search=search)
    # У `cross` было по ДВА пакета на путь поиска, стало по одному, и это
    # ровно предмет карточки, а не потеря событий: до quotient один и тот же
    # локус предъявлялся дважды и дренировался двумя пакетами; после
    # факторизации локус один — и пакет один. Число пакетов у остальных фигур
    # не двинулось, а главное утверждение теста («батч один и от перестановок
    # не зависит», `projections[0] == projections[1]` выше) проверяется на
    # КАЖДОМ пакете и не ослаблено.
    assert Counter((name, search) for name, search, _ in rows) == Counter({
        ("cross", "MOTORCYCLE"): 1,
        ("cross", "EXHAUSTIVE"): 1,
        ("u_shape", "MOTORCYCLE"): 1,
        ("u_shape", "EXHAUSTIVE"): 1,
        ("staircase", "MOTORCYCLE"): 1,
        ("staircase", "EXHAUSTIVE"): 1,
    })


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


def test_duplicate_live_owner_refuses_after_retaining_packet_observations(
    monkeypatch,
):
    builder = _fake_builder()
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    level = (
        CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 0, 1, -1),
    )
    unsupported = CandidateEventV1(
        EventKind.START, ZERO_TIME, point, 2, -1, 4
    )
    vertices = tuple(
        superlevel_module._VertexSnapshot(
            ident=index,
            prev=(index - 1) % 3,
            next=(index + 1) % 3,
            prev_edge=index,
            next_edge=0 if index < 2 else 2,
            alive=True,
            incoming_ray=(-1, 0),
            outgoing_ray=(1, 0),
        )
        for index in range(3)
    )
    snapshot = superlevel_module.SuperlevelSnapshotV1(
        incidents=(),
        vertices=vertices,
        unsupported=(unsupported,),
        stale_candidates=2,
        duplicate_live_owner_edge_ids=(0,),
    )
    monkeypatch.setattr(
        superlevel_module,
        "collect_superlevel_snapshot",
        lambda candidate_builder, candidate_level: snapshot,
    )
    superlevel_module.apply_superlevel_transaction(builder, level)

    assert builder.refusal is SkeletonOutcome.SUPERLEVEL_COMPONENT_UNRESOLVABLE
    assert builder.counters["discarded_stale_candidates"] == 2
    assert builder.counters["unsupported_event_kind_dropped"] == 1
    assert builder.counters["superlevel_unresolvable_components"] == 1
    assert builder.mutations == []
    assert tuple(debt["cause"] for debt in builder.debts) == (
        ProofObligationBranch.UNSUPPORTED_EVENT_KIND,
        ProofObligationBranch.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
    )
    assert builder.debts[-1]["participant_edge_keys"] == (
        builder.edges[0].key,
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


def test_two_port_meeting_has_unique_cross_pair_even_with_equal_rays():
    vertices = tuple(
        superlevel_module._VertexSnapshot(
            ident=index,
            prev=index + 2,
            next=index + 4,
            prev_edge=index,
            next_edge=index + 1,
            alive=True,
            incoming_ray=(1, 0),
            outgoing_ray=(0, 1),
        )
        for index in range(2)
    )
    assert superlevel_module._reconnect_snapshot((0, 1), vertices) == (
        (0, 1),
        (1, 0),
    )


def _snapshot_vertex(
    ident,
    *,
    prev,
    next_,
    prev_occurrence,
    next_occurrence,
    alive=True,
    incoming_ray=(-1, 0),
    outgoing_ray=(1, 0),
):
    return superlevel_module._VertexSnapshot(
        ident=ident,
        prev=prev,
        next=next_,
        prev_edge=ident,
        next_edge=ident,
        alive=alive,
        incoming_ray=incoming_ray,
        outgoing_ray=outgoing_ray,
        point_key=None,
        prev_occurrence=prev_occurrence,
        next_occurrence=next_occurrence,
    )


def _boundary_birth(seed, prev_occurrence, next_occurrence, *, replaces=()):
    point_key = ((0, Fraction(seed + 1)), ())
    return superlevel_module.BoundaryBirthV1(
        point_key=point_key,
        prev_occurrence=prev_occurrence,
        next_occurrence=next_occurrence,
        key=(seed, point_key, prev_occurrence, next_occurrence),
        replaces=replaces,
    )


def _wired_cycle(size, *, reverse=False):
    occurrences = tuple(("owner", index, index + 1) for index in range(size))
    births = tuple(
        _boundary_birth(
            index,
            occurrences[index],
            occurrences[(index + 1) % size],
        )
        for index in range(size)
    )
    if reverse:
        births = tuple(reversed(births))
    rewritten, wiring, _, components, valid = superlevel_module._wire_births(
        births,
        set(),
        (),
        (),
    )
    return rewritten, wiring, components, valid


def test_interior_split_refuses_zero_duplicate_or_unchanged_target_occurrence():
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    point_key = (point.x.terms, point.y.terms)
    other = ((1, Fraction(1)), ())
    target = ("owner", point_key, other)
    vertex = _snapshot_vertex(
        0,
        prev=0,
        next_=0,
        prev_occurrence=("prev",),
        next_occurrence=("next",),
    )
    incident = superlevel_module.SuperlevelIncidentV1(
        event=CandidateEventV1(EventKind.SPLIT, ZERO_TIME, point, 0, -1, 3),
        vertex_ids=(0,),
        edge_occurrences=(),
        participants=((0, 0, 1, 0),),
        target_participants=((3, 0, 1, 0),),
        point_key=point_key,
        target_projection=SqrtSumV1.zero(),
        target_occurrence=target,
    )
    plans, dropped, valid = superlevel_module._split_cut_plans(
        (incident,), (vertex,)
    )
    assert (plans, dropped, valid) == ((), 0, False)

    duplicate = superlevel_module.SuperlevelIncidentV1(
        event=CandidateEventV1(EventKind.SPLIT, ZERO_TIME, point, 1, -1, 3),
        vertex_ids=(1,),
        edge_occurrences=(),
        participants=((1, 0, 1, 0),),
        target_participants=((3, 0, 1, 0),),
        point_key=point_key,
        target_projection=SqrtSumV1.zero(),
        target_occurrence=("owner", ((1, Fraction(-1)), ()), other),
    )
    interior_first = superlevel_module.SuperlevelIncidentV1(
        event=incident.event,
        vertex_ids=incident.vertex_ids,
        edge_occurrences=incident.edge_occurrences,
        participants=incident.participants,
        target_participants=incident.target_participants,
        point_key=incident.point_key,
        target_projection=incident.target_projection,
        target_occurrence=duplicate.target_occurrence,
    )
    second_vertex = _snapshot_vertex(
        1,
        prev=1,
        next_=1,
        prev_occurrence=("prev-1",),
        next_occurrence=("next-1",),
    )
    assert superlevel_module._split_cut_plans(
        (interior_first, duplicate), (vertex, second_vertex)
    )[2] is False

    start = ((1, Fraction(-1)), ())
    end = ((1, Fraction(2)), ())
    second_point = EventPointV1(
        SqrtSumV1.rational(1), SqrtSumV1.zero()
    )
    second_key = (second_point.x.terms, second_point.y.terms)
    interior_target = ("owner", start, end)

    def interior(event_point, event_vertex, projection, key):
        return superlevel_module.SuperlevelIncidentV1(
            event=CandidateEventV1(
                EventKind.SPLIT,
                ZERO_TIME,
                event_point,
                event_vertex,
                -1,
                3,
            ),
            vertex_ids=(event_vertex,),
            edge_occurrences=(),
            participants=((event_vertex, 0, 1, 0),),
            target_participants=((3, 0, 1, 0),),
            point_key=key,
            target_projection=SqrtSumV1.rational(projection),
            target_occurrence=interior_target,
        )

    valid_incidents = (
        interior(point, 0, 0, point_key),
        interior(second_point, 1, 1, second_key),
    )
    forward = superlevel_module._split_cut_plans(
        valid_incidents, (vertex, second_vertex)
    )
    backward = superlevel_module._split_cut_plans(
        tuple(reversed(valid_incidents)), (vertex, second_vertex)
    )
    assert forward == backward
    plans, dropped, valid = forward
    assert valid and dropped == 0 and len(plans) == 1
    assert plans[0].segment_occurrences == (
        ("owner", start, point_key),
        ("owner", point_key, second_key),
        ("owner", second_key, end),
    )


def test_terminal_two_birth_cycle_is_unique_reciprocal_and_permutation_invariant():
    projections = []
    for reverse in (False, True):
        births, wiring, components, valid = _wired_cycle(2, reverse=reverse)
        assert valid
        projections.append(
            superlevel_module._terminal_two_birth_cycles(
                tuple(reversed(births)) if reverse else births,
                tuple(reversed(components)) if reverse else components,
                tuple(reversed(wiring)) if reverse else wiring,
            )
        )
    assert len(projections[0]) == 1
    assert {frozenset(item) for item in projections[0]} == {
        frozenset(projections[1][0])
    }

    for size in (1, 3):
        births, wiring, components, valid = _wired_cycle(size)
        assert valid
        assert superlevel_module._terminal_two_birth_cycles(
            births, components, wiring
        ) == ()


def test_terminal_cycle_rejects_nonreciprocal_existing_anchor_and_live_alias():
    births, wiring, components, valid = _wired_cycle(2)
    assert valid
    key, predecessor, successor = wiring[0]
    nonreciprocal = (
        (key, predecessor, superlevel_module.VertexReferenceV1(existing=9)),
        *wiring[1:],
    )
    assert superlevel_module._terminal_two_birth_cycles(
        births, components, nonreciprocal
    ) == ()

    anchored = (
        (
            key,
            superlevel_module.VertexReferenceV1(existing=8),
            successor,
        ),
        *wiring[1:],
    )
    assert superlevel_module._terminal_two_birth_cycles(
        births, components, anchored
    ) == ()

    alias = _snapshot_vertex(
        0,
        prev=0,
        next_=0,
        prev_occurrence=births[1].next_occurrence,
        next_occurrence=births[0].prev_occurrence,
    )
    assert superlevel_module._wire_births(
        births, set(), (alias,), ()
    )[4] is False

    noncycle_births = (
        births[0],
        _boundary_birth(7, births[0].next_occurrence, ("not-reciprocal",)),
    )
    birth_keys = tuple(birth.key for birth in noncycle_births)
    symbolic = tuple(
        (
            key,
            superlevel_module.VertexReferenceV1(
                birth_key=birth_keys[1 - index]
            ),
            superlevel_module.VertexReferenceV1(
                birth_key=birth_keys[1 - index]
            ),
        )
        for index, key in enumerate(birth_keys)
    )
    assert superlevel_module._terminal_two_birth_cycles(
        noncycle_births, (birth_keys,), symbolic
    ) == ()


def test_duplicate_birth_port_or_key_refuses_without_symbolic_wiring_leak():
    first = _boundary_birth(0, ("a",), ("b",))
    same_port = _boundary_birth(1, ("a",), ("c",))
    same_key = superlevel_module.BoundaryBirthV1(
        point_key=first.point_key,
        prev_occurrence=("d",),
        next_occurrence=("e",),
        key=first.key,
    )
    for births in ((first, same_port), (first, same_key)):
        rewritten, wiring, rewrites, components, valid = (
            superlevel_module._wire_births(births, set(), (), ())
        )
        assert rewritten
        assert (wiring, rewrites, components, valid) == ((), (), (), False)


def test_reciprocal_birth_cycle_remains_live_through_commit(monkeypatch):
    # ДВЕРЬ ПЕРЕЕХАЛА, свойство осталось. Коммит идёт не через `_commit_plans`,
    # а через `materialize_symbolic_runtime_commit`; наблюдатель перенесён на
    # неё. Предмет прежний: терминальный взаимный двухрождённый цикл обязан
    # ПЕРЕЖИТЬ коммит живым и взаимно связанным.
    from cftuv_envelope.wavefront import (
        symbolic_runtime_commit as runtime_module,
    )

    original = runtime_module.materialize_symbolic_runtime_commit
    observations = []

    def observed_commit(builder, snapshot, plan):
        first_new = len(builder.vertices)
        ridges_before = builder.counters["ridges"]
        terminal_count = sum(
            len(component.terminal_birth_cycles)
            for component in plan.closure.materialization.plans
        )
        reason = original(builder, snapshot, plan)
        if not terminal_count:
            return reason
        born = builder.vertices[first_new:]
        terminal = [
            vertex
            for vertex in born
            if vertex.birth == builder.now
            and vertex.next >= first_new
            and builder.vertices[vertex.next].next == vertex.ident
        ]
        observations.append(
            (
                terminal_count,
                len(terminal),
                builder.counters["ridges"] - ridges_before,
                all(vertex.alive for vertex in terminal),
                all(
                    builder.vertices[vertex.next].prev == vertex.ident
                    for vertex in terminal
                ),
            )
        )

        return reason

    monkeypatch.setattr(
        runtime_module,
        "materialize_symbolic_runtime_commit",
        observed_commit,
    )
    skeleton = build_skeleton(_CORPUS["axis_rectangle"])
    assert skeleton.outcome is SkeletonOutcome.EXACT
    # Свидетель обязан быть непустым: наблюдатель, который никого не поймал,
    # свойства не проверяет.
    assert observations
    assert observations == [(1, 2, 0, True, True)]
    assert skeleton.counter("ridges") == 1


def test_reciprocal_birth_cycle_detector_is_diagnostic_only(
    monkeypatch,
):
    polygon = _CORPUS["axis_rectangle"]
    original_detector = superlevel_module._terminal_two_birth_cycles
    original_close = skeleton_module._Builder._close_short_lavs
    detections = []
    fixed_point_pairs = []

    def observed_detector(births, components, wiring):
        cycles = original_detector(births, components, wiring)
        detections.append(len(cycles))
        return cycles

    def observed_close(builder):
        pairs = set()
        for vertex in builder.vertices:
            if not vertex.alive or vertex.birth != builder.now:
                continue
            peer = builder.vertices[vertex.next]
            if (
                peer.alive
                and peer.birth == builder.now
                and peer.ident != vertex.ident
                and builder.vertices[peer.next].ident == vertex.ident
            ):
                pairs.add(tuple(sorted((vertex.ident, peer.ident))))
        fixed_point_pairs.extend(sorted(pairs))
        original_close(builder)

    monkeypatch.setattr(
        superlevel_module, "_terminal_two_birth_cycles", observed_detector
    )
    monkeypatch.setattr(
        skeleton_module._Builder, "_close_short_lavs", observed_close
    )
    diagnosed = build_skeleton(polygon)
    diagnosed_partition = build_faces(polygon, diagnosed)
    diagnosed_coverage = tuple(
        _REPRO._coverage_record(coverage_at(diagnosed_partition, alpha))
        for alpha in (Fraction(1, 4), Fraction(1), Fraction(3))
    )

    monkeypatch.setattr(
        superlevel_module,
        "_terminal_two_birth_cycles",
        lambda births, components, wiring: (),
    )
    monkeypatch.setattr(
        skeleton_module._Builder, "_close_short_lavs", original_close
    )
    undiagnosed = build_skeleton(polygon)
    undiagnosed_partition = build_faces(polygon, undiagnosed)
    undiagnosed_coverage = tuple(
        _REPRO._coverage_record(coverage_at(undiagnosed_partition, alpha))
        for alpha in (Fraction(1, 4), Fraction(1), Fraction(3))
    )

    assert sum(detections) == 1
    assert fixed_point_pairs == [(4, 5)]
    assert diagnosed.outcome is undiagnosed.outcome is SkeletonOutcome.EXACT
    assert diagnosed.nodes == undiagnosed.nodes
    assert diagnosed.levels == undiagnosed.levels
    assert diagnosed.counters == undiagnosed.counters
    assert semantic_digest(diagnosed) == semantic_digest(undiagnosed)
    assert _REPRO._partition_record(diagnosed_partition) == (
        _REPRO._partition_record(undiagnosed_partition)
    )
    assert diagnosed_coverage == undiagnosed_coverage
    assert diagnosed.proof_status is undiagnosed.proof_status is ProofStatus.COMPLETE


def test_mixed_edge_chain_and_interior_split_compose_unique_ports():
    point = EventPointV1(SqrtSumV1.zero(), SqrtSumV1.zero())
    point_key = (point.x.terms, point.y.terms)
    edge_event = CandidateEventV1(EventKind.EDGE, ZERO_TIME, point, 0, 1, -1)
    split_event = CandidateEventV1(EventKind.SPLIT, ZERO_TIME, point, 1, -1, 2)
    endpoint_event = CandidateEventV1(
        EventKind.SPLIT, ZERO_TIME, point, 1, -1, 0
    )
    outer_prev = ("P", "p0", "p1")
    collapsed = ("C", "c0", "c1")
    outer_next = ("N", "n0", "n1")
    target = ("T", "t0", "t1")
    target_left = ("T", "t0", point_key)
    target_right = ("T", point_key, "t1")
    other = ("O", "o0", "o1")
    vertices = (
        _snapshot_vertex(
            0, prev=4, next_=1,
            prev_occurrence=outer_prev, next_occurrence=collapsed,
        ),
        _snapshot_vertex(
            1, prev=0, next_=5,
            prev_occurrence=collapsed, next_occurrence=outer_next,
        ),
        _snapshot_vertex(
            2, prev=3, next_=3,
            prev_occurrence=other, next_occurrence=target,
        ),
        _snapshot_vertex(
            3, prev=2, next_=2,
            prev_occurrence=target, next_occurrence=other,
        ),
        _snapshot_vertex(
            4, prev=5, next_=0,
            prev_occurrence=other, next_occurrence=outer_prev,
        ),
        _snapshot_vertex(
            5, prev=1, next_=4,
            prev_occurrence=outer_next, next_occurrence=other,
        ),
    )
    edge_incident = superlevel_module.SuperlevelIncidentV1(
        event=edge_event,
        vertex_ids=(0, 1, 4, 5),
        edge_occurrences=(0, 1),
        participants=((10, 0, 11, 0),),
        target_participants=((10, 0, 11, 0),),
        point_key=point_key,
    )
    split_incident = superlevel_module.SuperlevelIncidentV1(
        event=split_event,
        vertex_ids=(1, 2, 3),
        edge_occurrences=(1, 2),
        participants=((10, 0, 11, 0),),
        target_participants=((20, 0, 21, 0),),
        point_key=point_key,
        target_projection=SqrtSumV1.zero(),
        emitter_key=(collapsed[0], outer_next[0]),
        target_occurrence=target,
    )
    endpoint_incident = superlevel_module.SuperlevelIncidentV1(
        event=endpoint_event,
        vertex_ids=(0, 1, 4),
        edge_occurrences=(0, 1),
        participants=((10, 0, 11, 0),),
        target_participants=((10, 0, 11, 0),),
        point_key=point_key,
        met_vertex_id=0,
        met_adjacent=True,
        target_projection=SqrtSumV1.zero(),
        target_start_id=4,
        target_end_id=0,
        emitter_key=(collapsed[0], outer_next[0]),
        target_occurrence=outer_prev,
    )
    canonical_packet = (
        edge_incident,
        split_incident,
        endpoint_incident,
        endpoint_incident,
        endpoint_incident,
        endpoint_incident,
    )
    projections = []
    for packet in (
        canonical_packet,
        tuple(reversed(canonical_packet)),
    ):
        snapshot = superlevel_module.SuperlevelSnapshotV1(
            incidents=packet,
            vertices=vertices,
            unsupported=(),
            stale_candidates=0,
        )
        (plan,) = superlevel_module.plan_superlevel_components(snapshot)
        projections.append(plan)

    assert projections[0] == projections[1]
    plan = projections[0]
    assert plan.resolution is (
        superlevel_module.SuperlevelResolution.BOUNDARY_PORT_PAIRING
    )
    assert plan.dead_vertex_ids == (0, 1)
    assert plan.edge_contacts[0].chains == ((0, 1),)
    assert plan.edge_contacts[0].events == (edge_event, endpoint_event)
    assert plan.edge_contacts[0].births == ()
    assert {
        (birth.prev_occurrence, birth.next_occurrence)
        for birth in plan.births
    } == {
        (outer_prev, target_right),
        (target_left, outer_next),
    }
    assert len(plan.birth_wiring) == 2

    # ОТРИЦАТЕЛЬНЫЙ КОНТРОЛЬ по AUTH Q-10-ADD: рантаймовый id ничью не
    # разрешает. Инцидент, отличающийся ТОЛЬКО `target_end_id` — рантаймовым
    # номером встреченной вершины, — несёт ТОТ ЖЕ зародыш (то же каноническое
    # время, ту же точку, тот же набор концов), а значит обязан дать ТОТ ЖЕ
    # исход. Разный исход означал бы, что ответ зависит от нумерации вершин,
    # то есть от порядка применения, — ровно то, ради чего заведена
    # транзакция.
    #
    # SUPERSEDED: до Q-10-ADD этот блок требовал обратного — что
    # `target_end_id=4` ВЫВОДИТ инцидент из поглощения (`wrong_endpoint in
    # remaining`). То ожидание описывало предикат поглощения, который решал
    # тождество рантаймовым id (`met_adjacent` + boundary endpoint +
    # принадлежность рантаймового edge id). Q-10-ADD отменил этот предикат
    # нормативно: тождество — канонический t, каноническая точка и
    # канонический НАБОР КОНЦОВ, где конец = (ключ вхождения, примитивный
    # целочисленный луч). Старое ожидание сохранено здесь прозой, потому что
    # тест не должен молча поменять смысл.
    same_germ = replace(endpoint_incident, target_end_id=4)
    baseline = superlevel_module._edge_contact_plans(
        (edge_incident,), (endpoint_incident, split_incident), vertices
    )
    renumbered = superlevel_module._edge_contact_plans(
        (edge_incident,), (same_germ, split_incident), vertices
    )
    assert baseline[2] and renumbered[2]
    assert renumbered[0][0].events == baseline[0][0].events
    assert renumbered[0][0].births == baseline[0][0].births
    assert renumbered[0][0].chains == baseline[0][0].chains
    assert tuple(item.event for item in renumbered[1]) == tuple(
        item.event for item in baseline[1]
    )
    # Настоящий interior-разрез поглощён НЕ был ни в одном из прогонов: его
    # рассекаемое вхождение не плечо этого локуса.
    assert split_incident.event not in renumbered[0][0].events
