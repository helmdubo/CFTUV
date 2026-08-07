from __future__ import annotations

import hashlib
import json
from functools import lru_cache
from pathlib import Path

import cftuv_envelope as kernel
import pytest
from cftuv_envelope import exact_sqrt_sum as exact_sqrt_sum_module
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor
from cftuv_envelope.wavefront import coverage as coverage_module
from cftuv_envelope.wavefront import event_time as event_time_module
from cftuv_envelope.wavefront import (
    exact_candidate_view as candidate_view_module,
)
from cftuv_envelope.wavefront import motorcycle as motorcycle_module
from cftuv_envelope.wavefront import skeleton as skeleton_module
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.skeleton import (
    CandidateRefusal,
    refusal_counter,
)
from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1


KERNEL_ROOT = Path(__file__).parents[1]
SEM_CLB_ROOT = (
    KERNEL_ROOT / "fixtures" / "sem_clb_02_lost_domains_v1"
)
CASE_ROOT = SEM_CLB_ROOT / "cases"
RECEIPT_PATH = SEM_CLB_ROOT / "gate_a_triple_prime_receipt.json"
RECEIPT_SHA256 = (
    "bf1864fe80ec66ecc926ec02b82a6204a25e3b7f63032adff3fb769227ebb3b4"
)
EXPECTED_R_MIN = {
    "building_all_seams_patch_001_lost_resolved_v1": 2,
    "building_all_seams_patch_006_lost_resolved_v1": 3,
    "building_all_seams_patch_011_lost_resolved_v1": 2,
    "building_all_seams_patch_105_lost_resolved_v1": 1,
}
EXPECTED_K_SEQUENCES = {
    "building_all_seams_patch_001_lost_resolved_v1": (
        (0, 3, 8),
        (0, 1, 3, 4),
        (0, 5, 8),
    ),
    "building_all_seams_patch_006_lost_resolved_v1": (
        (0, 1, 3, 5, 7, 8),
        (0, 271129, 524288),
    ),
    "building_all_seams_patch_011_lost_resolved_v1": (
        (0, 5, 8),
        (0, 1, 3, 4),
    ),
    "building_all_seams_patch_105_lost_resolved_v1": (
        (0, 1, 2),
    ),
}
#: Терминал полевого случая. EXACT — фронт закрылся; иначе — ЧЕСТНЫЙ
#: ИМЕНОВАННЫЙ отказ, и он такая же приёмка, как EXACT: тихого исчезновения
#: нет, геометрия не лжёт. Три случая стоят на отказе по закону живых
#: переворотов пролёта (`SYMBOLIC_POSTSTATE_SPAN_INVERTED` у 006 и 011,
#: незакрывшийся фронт у 001) — корень измерен и лежит ВЫШЕ этой карточки:
#: ход рассекаемого пролёта в композиторе канонический (+1 в 138 замерах,
#: -1 ни разу), а переворот приходит от живых пролётов. Известный открытый
#: счёт — зеркальная чётность ИМЁН отказа (001 и 006 меняются именами при
#: отражении); он вынесен в карточку FIELD-REFUSAL-MIRROR-PARITY, см. запись
#: поставки P0-2B-FINISH в `DECISIONS.md` и зонд
#: `kernel/artifacts/p0_2_superlevel_transaction/
#: probe_field_mirror_covariance.py`.
#: Цена прохода по простому базису. Единица измерения сменилась вместе со
#: слоем: точки считает символический путь, и цена у него своя. Ноль откатов
#: (`incomplete_*`) — не число прохода, а СВОЙСТВО, и оно тут же рядом.
FROZEN_PATCH011_COVERAGE_TERMINAL = "EXACT"
FROZEN_PATCH011_OPTIMIZED_CALLS = {"MOTORCYCLE": 18, "COVERAGE": 96}

# ПЕРЕСЪЁМКА 2026-08-04, «НЕ ЭТИМ ЧИСЛОМ». Терминал patch011 переехал с
# именованного отказа на EXACT (закон места рождённого порта,
# symbolic_overlay._born_place), поэтому покрытие ТЕПЕРЬ СЧИТАЕТСЯ — и
# числа прохода выросли ровно потому, что прогон доходит до конца, а не
# обрывается отказом. Предмет обоих тестов — «базис полон и к легаси-делению
# не откатывается НИ РАЗУ» — НЕ СДВИНУЛСЯ: `incomplete_picks` и
# `incomplete_supports` остались нулями (измерено: при пересъёмке эти два
# ключа единственные, что не изменились).
FROZEN_PATCH006_PASS = {
    "event_calls": 22407,
    "coordinate_divisions": 44814,
    "pick_iterations": 78832,
    "support_checks": 138678,
    # НОЛЬ ОТКАТОВ — это свойство, а не цена: базис полон, легаси-деление не
    # срабатывает ни разу.
    "incomplete_picks": 0,
    "incomplete_supports": 0,
}

EXPECTED_PUBLIC_WAVEFRONT = {
    # ПЕРЕСЪЁМКА 2026-08-04, «НЕ ЭТИМ ЧИСЛОМ». Три полевых патча стояли
    # именованными отказами не потому, что их геометрия недостижима, а
    # потому что порт, РОЖДЁННЫЙ локусом ровно в `now`, спрашивал о своём
    # месте закон движения — тот честно отвечал «места нет» на двух
    # совпавших антипараллельных прямых. Закон примирения продолжен на
    # концы пролёта (symbolic_overlay._born_place): место такого порта
    # называет сам локус. Доказательство, что переехало НЕ число, а
    # предмет: `node_count`, `face_count`, `born_zero_refusal_count` и
    # `sliding_seed_count` НЕ СДВИНУЛИСЬ НИ НА ЕДИНИЦУ у всех четырёх
    # случаев; сдвинулись только терминал (отказ → EXACT) и `skeleton_levels`
    # — прогон, обрывавшийся отказом, теперь доходит до конца.
    # patch006 при этом попал в числа, измеренные ещё при консервации стека
    # как ПРАВИЛЬНЫЙ ответ (241 уровень, 73 узла, 63 грани, COMPLETE) —
    # тогда они достигались снятием latent-discovery, то есть без законного
    # механизма; теперь достигнуты законом. `proof_status` всех четырёх —
    # COMPLETE (было INCOMPLETE), покрытие EXACT.
    "building_all_seams_patch_001_lost_resolved_v1": {
        "conveyor_terminal": "EXACT",
        "terminal": "EXACT",
        "sliding_seed_count": 4,
        "skeleton_levels": 197,
        "node_count": 62,
        "face_count": 54,
        "born_zero_refusal_count": 22,
    },
    "building_all_seams_patch_006_lost_resolved_v1": {
        "conveyor_terminal": "EXACT",
        "terminal": "EXACT",
        "sliding_seed_count": 5,
        "skeleton_levels": 241,
        "node_count": 73,
        "face_count": 63,
        "born_zero_refusal_count": 26,
    },
    "building_all_seams_patch_011_lost_resolved_v1": {
        "conveyor_terminal": "EXACT",
        "terminal": "EXACT",
        "sliding_seed_count": 3,
        "skeleton_levels": 202,
        "node_count": 62,
        "face_count": 54,
        "born_zero_refusal_count": 22,
    },
    "building_all_seams_patch_105_lost_resolved_v1": {
        "conveyor_terminal": "EXACT",
        "terminal": "EXACT",
        "sliding_seed_count": 1,
        # AUTH_PENDING_Q06: локальный targeted ratchet, не для commit/push.
        "skeleton_levels": 12,
        "node_count": 6,
        "face_count": 8,
        "born_zero_refusal_count": 1,
    },
}
EXPECTED_PATCH006_SKELETON_PRIME_UNIVERSE = (
    2,
    13,
    17,
    29,
    37,
    41,
    53,
    89,
    113,
    181,
    197,
    293,
    617,
    877,
    941,
    1789,
    1901,
    5417,
    8209,
    78241,
    80629,
    85093,
    262313,
    355777,
    4610729,
    12310913,
    13285057,
    45517601,
    69827269,
    251194849,
    433707749,
    4814427773,
    7830174716261,
    13258330511417,
    24615899915773,
    34915153375909,
    82022315028289,
    1208514780081437,
    2665193254113601,
    37366821091976921,
)


def _receipt():
    return json.loads(RECEIPT_PATH.read_text(encoding="utf-8"))


@lru_cache(maxsize=None)
def _load_case(name: str):
    root = CASE_ROOT / name
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    result = kernel.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is kernel.ReferenceOutcome.EXACT
    assert result.compilation is not None
    return snapshot, request, domain, result.compilation


def _ratio_text(value) -> str:
    if value.denominator == 1:
        return str(value.numerator)
    return f"{value.numerator}/{value.denominator}"


def _point_text(value) -> list[str]:
    return [_ratio_text(value.x), _ratio_text(value.y)]


def _node_sequence(chain) -> tuple[int, ...]:
    return (
        0,
        *(item.selected_k for item in chain.internal_assignments),
        chain.refined_endpoint_span_k,
    )


def test_accepted_a3_receipt_bytes_and_coverage_are_frozen():
    payload = RECEIPT_PATH.read_bytes()
    receipt = _receipt()
    aggregate = receipt["aggregate"]

    assert hashlib.sha256(payload).hexdigest() == RECEIPT_SHA256
    assert aggregate["domain_r_min"] == EXPECTED_R_MIN
    assert aggregate["fixture_count"] == 4
    assert aggregate["declared_straight_chain_count"] == 8
    assert aggregate["internal_vertex_count"] == 13
    assert aggregate["non_chain_vertex_count"] == 83
    assert aggregate["clamped_internal_steps_at_r_min"] == 1
    assert aggregate["unclamped_internal_steps_at_r_min"] == 12
    assert receipt["failures"] == []


def test_product_v2_matches_every_selected_a3_exact_assignment_row():
    receipt_by_case = {
        item["fixture_id"]: item for item in _receipt()["fixtures"]
    }
    internal_row_count = 0
    non_chain_count = 0
    observed_sequences = {}

    for case_name, expected_r in EXPECTED_R_MIN.items():
        _, _, _, compilation = _load_case(case_name)
        binding = compilation.evaluation_geometry_binding
        assert type(binding) is (
            kernel.ChainStraightEvaluationGeometryBindingV2
        )
        fixture = receipt_by_case[case_name]
        selected_attempt = next(
            item
            for item in fixture["attempts"]
            if item["r"] == fixture["r_min"]
        )
        expected_chains = {
            item["physical_chain_id"]: item
            for item in selected_attempt["chain_attempts"]
        }
        product_chains = {
            item.physical_chain_id.value: item
            for item in binding.straight_chain_bindings
        }
        authorities = {
            item.source_vertex_id.value: item
            for item in binding.vertex_authorities
        }

        assert binding.refinement_power == expected_r
        assert binding.base_lattice_scale == fixture["base_scale_S"]
        assert binding.lattice_scale == selected_attempt[
            "refined_scale_S_prime"
        ]
        assert product_chains.keys() == expected_chains.keys()
        observed_sequences[case_name] = tuple(
            _node_sequence(product_chains[chain_id])
            for chain_id in sorted(product_chains)
        )

        for chain_id, expected in expected_chains.items():
            chain = product_chains[chain_id]
            assignment = expected["canonical_gram_assignment"]
            assert [item.value for item in chain.ordered_source_vertex_ids] == (
                expected["ordered_source_vertex_ids"]
            )
            assert list(chain.primitive_direction) == expected[
                "primitive_direction"
            ]
            assert chain.base_endpoint_span_k == expected[
                "base_endpoint_span_K"
            ]
            assert chain.refined_endpoint_span_k == expected["K_prime"]
            assert list(_node_sequence(chain)) == assignment["k_sequence"]
            assert len(chain.internal_assignments) == len(
                assignment["steps"]
            )

            for product_row, expected_row in zip(
                chain.internal_assignments,
                assignment["steps"],
                strict=True,
            ):
                internal_row_count += 1
                authority = authorities[
                    product_row.source_vertex_id.value
                ]
                assert product_row.ordinal == expected_row["ordinal"]
                assert product_row.source_vertex_id.value == expected_row[
                    "source_vertex_id"
                ]
                assert product_row.lower_k == expected_row["lower"]
                assert product_row.upper_k == expected_row["upper"]
                assert _ratio_text(product_row.projection_k_gram) == (
                    expected_row["projection_k_gram"]
                )
                assert product_row.unconstrained_canonical_k == (
                    expected_row["unconstrained_canonical_k"]
                )
                assert product_row.selected_k == expected_row["selected_k"]
                assert product_row.clamped == expected_row["CLAMPED"]
                assert product_row.disposition.value == expected_row[
                    "boundary_disposition"
                ]
                assert list(product_row.assigned_refined_node) == (
                    expected_row["assigned_refined_node"]
                )
                assert _point_text(
                    product_row.exact_offset_from_source
                ) == expected_row["exact_displacement"]
                assert _ratio_text(
                    product_row.exact_gram_displacement_squared
                ) == expected_row["exact_gram_displacement_squared"]
                assert _ratio_text(
                    product_row.half_step_gram_squared_bound
                ) == expected_row[
                    "half_S_prime_step_gram_squared_bound"
                ]
                assert _ratio_text(
                    product_row.exact_longitudinal_displacement_k
                ) == expected_row[
                    "exact_longitudinal_displacement_k"
                ]
                assert _ratio_text(
                    product_row.longitudinal_half_step_bound_k
                ) == "1/2"
                assert _point_text(
                    authority.assigned_domain_coordinate
                ) == expected_row["assigned_coordinate"]
                assert _point_text(
                    authority.source_domain_coordinate
                ) == expected_row["source_coordinate"]

        expected_deficits = {
            item["physical_chain_id"]: (
                expected_r - 1,
                item["base_endpoint_span_K"] * (1 << (expected_r - 1)),
                item["vertex_count"] - 1,
            )
            for item in expected_chains.values()
            if item["base_endpoint_span_K"] * (1 << (expected_r - 1))
            < item["vertex_count"] - 1
        }
        assert {
            item.physical_chain_id.value: (
                item.previous_refinement_power,
                item.previous_endpoint_span_k,
                item.capacity_required,
            )
            for item in binding.previous_refinement_capacity_deficits
        } == expected_deficits
        assert all(
            item.exact_deficit
            == item.capacity_required - item.previous_endpoint_span_k
            for item in binding.previous_refinement_capacity_deficits
        )

        product_non_chain = tuple(
            item
            for item in binding.vertex_authorities
            if item.authority
            is kernel.ChainStraightVertexAuthorityV2.BASE_BOUND_NON_CHAIN_V1
        )
        non_chain_count += len(product_non_chain)
        assert len(product_non_chain) == fixture["non_chain_vertex_count"]
        assert all(
            item.assigned_domain_coordinate == item.base_bound_coordinate
            and not item.physical_chain_ids
            for item in product_non_chain
        )

    assert observed_sequences == EXPECTED_K_SEQUENCES
    assert internal_row_count == 13
    assert non_chain_count == 83


def test_patch105_has_the_sole_exact_named_clamp():
    all_clamps = []
    for case_name in EXPECTED_R_MIN:
        _, _, _, compilation = _load_case(case_name)
        binding = compilation.evaluation_geometry_binding
        all_clamps.extend(
            item
            for chain in binding.straight_chain_bindings
            for item in chain.internal_assignments
            if item.clamped
        )

    (clamp,) = all_clamps
    assert clamp.source_vertex_id.value.endswith(":building:189")
    assert _ratio_text(clamp.projection_k_gram) == (
        "578633553271514/339894311513453"
    )
    assert clamp.unconstrained_canonical_k == 2
    assert (clamp.lower_k, clamp.upper_k, clamp.selected_k) == (1, 1, 1)
    assert clamp.disposition is (
        kernel.ChainStraightAssignmentDispositionV2.CLAMPED_CONSTRAINT_EXCESS_ALLOWED
    )
    assert _point_text(clamp.exact_offset_from_source) == [
        "-27745503/451280896",
        "-130742673/451280896",
    ]
    assert _ratio_text(clamp.exact_gram_displacement_squared) == (
        "8887499590674321/288230376151711744"
    )
    assert _ratio_text(clamp.half_step_gram_squared_bound) == (
        "18014398510213009/1152921504606846976"
    )
    assert _ratio_text(clamp.exact_longitudinal_displacement_k) == (
        "238739241758061/339894311513453"
    )


def test_point_contact_v1_binding_and_raw_topology_anchor_are_unchanged():
    root = KERNEL_ROOT / "fixtures" / "building_002_point_contact_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    result = kernel.compile_reference_envelopes(
        snapshot,
        request,
        domain.patch_domain_id,
    )
    assert result.outcome is kernel.ReferenceOutcome.EXACT
    compilation = result.compilation
    binding = compilation.evaluation_geometry_binding

    assert type(binding) is kernel.EvaluationGeometryBindingV1
    assert hashlib.sha256(
        kernel.EvaluationGeometryBindingCodecV1.dumps(binding)
    ).hexdigest() == (
        "fe440be52dd6a4f386126a707a7528d749afd5921534abef9574b073fd1cf2d7"
    )
    evaluated = kernel.evaluate_reference_raw_coverage(
        compilation,
        request.requested_alpha,
    )
    assert evaluated.outcome is kernel.ReferenceOutcome.EXACT
    raw = evaluated.raw_coverage
    assert raw.semantic_digest == (
        "402ec97a93567aad5890786f75079b2114b1ba397819904cdb665049b31715eb"
    )
    assert (
        len(raw.point_contacts),
        len(raw.loops),
        len(raw.regions),
    ) == (0, 1, 1)


@pytest.mark.parametrize(
    "case_name",
    tuple(EXPECTED_PUBLIC_WAVEFRONT),
)
def test_public_wavefront_classifies_exact_straight_seed_vertices(
    case_name,
):
    expected = EXPECTED_PUBLIC_WAVEFRONT[case_name]
    snapshot, request, domain, _ = _load_case(case_name)
    prepared = prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    (region,) = prepared.regions
    if expected["terminal"] != "EXACT":
        # Терминал — именованный отказ. Проверяется он ТАК ЖЕ строго, как
        # EXACT: имя закреплено, а не «не EXACT».
        assert prepared.outcome.value == "SKELETON_DID_NOT_CLOSE"
        assert region.skeleton is not None
        assert region.skeleton.outcome.value == expected["terminal"]
        assert not region.is_exact
        return
    assert prepared.outcome.value == "EXACT", prepared.detail
    assert region.is_exact
    assert region.bridge.polygon is not None
    assert region.skeleton is not None
    assert region.partition is not None

    seed = skeleton_module._Builder(region.bridge.polygon)
    sliding = [
        vertex for vertex in seed.vertices if vertex.sliding is not None
    ]
    assert len(sliding) == expected["sliding_seed_count"]
    assert {vertex.ident for vertex in sliding} == seed.sliding_vertices
    assert region.skeleton.levels == expected["skeleton_levels"]
    assert len(region.skeleton.nodes) == expected["node_count"]
    assert len(region.partition.faces) == expected["face_count"]
    assert region.skeleton.counter(
        refusal_counter(CandidateRefusal.FILTER_SPAN_IS_BORN_ZERO)
    ) == expected["born_zero_refusal_count"]

    coverage = conveyor_coverage(prepared)
    assert coverage.outcome.value == "EXACT", coverage.detail
    (covered_region,) = coverage.regions
    assert covered_region.outcome.value == "EXACT"


def test_patch006_skeleton_event_points_use_the_frozen_prime_basis(
    monkeypatch,
):
    case_name = "building_all_seams_patch_006_lost_resolved_v1"
    snapshot, request, domain, _ = _load_case(case_name)
    observed_universes = []
    event_calls = 0
    coordinate_divisions = 0
    pick_iterations = 0
    support_checks = 0
    incomplete_picks = 0
    incomplete_supports = 0
    coordinate_depth = 0
    original_build = skeleton_module._prime_universe_from_q_values
    original_event = skeleton_module._event_point_with_prime_universe
    original_divide = event_time_module._divide_with_prime_universe
    original_pick = exact_sqrt_sum_module._pick_prime_from_universe
    original_support = exact_sqrt_sum_module._support_from_prime_universe

    def capture_build(q_values):
        universe = original_build(q_values)
        observed_universes.append(universe)
        return universe

    def capture_event(first, second, time, prime_universe):
        nonlocal event_calls
        event_calls += 1
        assert observed_universes
        assert prime_universe is observed_universes[0]
        return original_event(first, second, time, prime_universe)

    def capture_divide(numerator, denominator, prime_universe):
        nonlocal coordinate_depth, coordinate_divisions
        coordinate_divisions += 1
        coordinate_depth += 1
        try:
            return original_divide(
                numerator,
                denominator,
                prime_universe,
            )
        finally:
            coordinate_depth -= 1

    def capture_pick(terms, prime_universe):
        nonlocal pick_iterations, incomplete_picks
        prime = original_pick(terms, prime_universe)
        if coordinate_depth:
            pick_iterations += 1
            if prime is None:
                incomplete_picks += 1
        return prime

    def capture_support(radicand, prime_universe):
        nonlocal support_checks, incomplete_supports
        support = original_support(radicand, prime_universe)
        if coordinate_depth:
            support_checks += 1
            if support is None:
                incomplete_supports += 1
        return support

    monkeypatch.setattr(
        skeleton_module,
        "_prime_universe_from_q_values",
        capture_build,
    )
    monkeypatch.setattr(
        skeleton_module,
        "_event_point_with_prime_universe",
        capture_event,
    )
    # НОВАЯ ДВЕРЬ. Предмет теста — не дверь, а СВОЙСТВО: событийные точки
    # живут в замороженном простом базисе. Символический слой считает их через
    # `exact_candidate_view.position`, мимо старой двери в `skeleton`, поэтому
    # свидетель перенесён сюда. Ниже стоит `event_calls > 0`: инструмент,
    # который никого не поймал, свойства не проверяет.
    monkeypatch.setattr(
        candidate_view_module,
        "_event_point_with_prime_universe",
        capture_event,
    )
    monkeypatch.setattr(
        event_time_module,
        "_divide_with_prime_universe",
        capture_divide,
    )
    monkeypatch.setattr(
        exact_sqrt_sum_module,
        "_pick_prime_from_universe",
        capture_pick,
    )
    monkeypatch.setattr(
        exact_sqrt_sum_module,
        "_support_from_prime_universe",
        capture_support,
    )

    prepared = prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    assert prepared.outcome.value == EXPECTED_PUBLIC_WAVEFRONT[case_name][
        "conveyor_terminal"
    ], prepared.detail
    (region,) = prepared.regions
    assert region.skeleton is not None
    assert region.skeleton.outcome.value == (
        EXPECTED_PUBLIC_WAVEFRONT[case_name]["terminal"]
    )
    assert observed_universes == [
        EXPECTED_PATCH006_SKELETON_PRIME_UNIVERSE
    ]
    # Свидетель обязан быть непустым: `capture_event` сверяет у КАЖДОГО вызова,
    # что базис — тот самый замороженный объект.
    assert event_calls > 0
    # Sparse superlevel snapshot is byte-equivalent to the eager oracle; the
    # old exact count remains a hard upper bound, while the new count freezes
    # the factual cost of the query-oriented implementation.
    measured_pass = {
        "event_calls": event_calls,
        "coordinate_divisions": coordinate_divisions,
        "pick_iterations": pick_iterations,
        "support_checks": support_checks,
        "incomplete_picks": incomplete_picks,
        "incomplete_supports": incomplete_supports,
    }
    assert measured_pass == FROZEN_PATCH006_PASS, measured_pass


def test_patch011_prime_basis_is_complete_and_never_falls_back(monkeypatch):
    case_name = "building_all_seams_patch_011_lost_resolved_v1"
    snapshot, request, domain, _ = _load_case(case_name)
    observed_universes = {"MOTORCYCLE": [], "COVERAGE": []}
    optimized_calls = {"MOTORCYCLE": 0, "COVERAGE": 0}
    fallback_calls = []
    original_legacy = SqrtSumV1.__truediv__
    original_optimized = (
        exact_sqrt_sum_module._divide_with_prime_universe
    )
    original_motorcycle_build = (
        motorcycle_module._prime_universe_from_q_values
    )
    original_coverage_build = coverage_module._prime_universe_from_q_values

    def capture_universe(operation, original):
        def capture(q_values):
            universe = original(q_values)
            observed_universes[operation].append(universe)
            return universe

        return capture

    def observe_division(operation):
        def divide(numerator, denominator, prime_universe):
            optimized_calls[operation] += 1

            def track_fallback(left, right):
                fallback_calls.append((operation, left, right))
                return original_legacy(left, right)

            with monkeypatch.context() as local_patch:
                local_patch.setattr(
                    SqrtSumV1,
                    "__truediv__",
                    track_fallback,
                )
                return original_optimized(
                    numerator,
                    denominator,
                    prime_universe,
                )

        return divide

    monkeypatch.setattr(
        motorcycle_module,
        "_prime_universe_from_q_values",
        capture_universe("MOTORCYCLE", original_motorcycle_build),
    )
    monkeypatch.setattr(
        coverage_module,
        "_prime_universe_from_q_values",
        capture_universe("COVERAGE", original_coverage_build),
    )
    monkeypatch.setattr(
        motorcycle_module,
        "_divide_with_prime_universe",
        observe_division("MOTORCYCLE"),
    )
    monkeypatch.setattr(
        coverage_module,
        "_divide_with_prime_universe",
        observe_division("COVERAGE"),
    )

    prepared = prepare_conveyor(
        snapshot,
        request,
        patch_domain_id=domain.patch_domain_id,
    )
    # Терминал закреплён именем; предмет теста — «базис ПОЛОН и к легаси-делению
    # не откатывается НИ РАЗУ» — от терминала не зависит и проверяется ниже без
    # послаблений.
    assert prepared.outcome.value == EXPECTED_PUBLIC_WAVEFRONT[case_name][
        "conveyor_terminal"
    ], prepared.detail
    coverage = conveyor_coverage(prepared)
    assert coverage.outcome.value == FROZEN_PATCH011_COVERAGE_TERMINAL
    assert len(observed_universes["MOTORCYCLE"]) == 1
    assert len(observed_universes["MOTORCYCLE"][0]) == 39
    # ПЕРЕСЪЁМКА 2026-08-04. Прежняя строка утверждала ОТСУТСТВИЕ базиса
    # покрытия — «на отказном терминале покрытие не считается вовсе» — и
    # прямо обещала заметить, если покрытие начнёт считаться. Оно начало:
    # терминал переехал на EXACT. Утверждение перевёрнуто на своё истинное
    # содержание, а предмет теста сохранён: базис покрытия ПОЛОН и к
    # легаси-делению не откатывается ни разу (ноль ниже).
    assert len(observed_universes["COVERAGE"]) == 1
    assert len(observed_universes["COVERAGE"][0]) == 39
    # Свидетель непустой — инструмент, который никого не поймал, свойства не
    # проверяет.
    assert optimized_calls["MOTORCYCLE"] > 0
    assert optimized_calls == FROZEN_PATCH011_OPTIMIZED_CALLS
    # ГЛАВНОЕ утверждение теста и оно без послаблений: к легаси-делению путь не
    # откатывается НИ РАЗУ.
    assert fallback_calls == []
