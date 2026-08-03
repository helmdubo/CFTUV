"""Закон разреза физической цепочки по точному излому.

У хоста нет представления излома ВНУТРИ `BoundaryChain`: «прямота» — это
отсутствие записи об угле, а не утверждение о геометрии. Изломанный
seam-маршрут поэтому объявлялся ядру одной прямой цепью, и ядро честно
отказывало `SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR`, не доходя до
скелета. Здесь проверяется, что чинятся ПРЕДЪЯВЛЯЕМЫЕ ФАКТЫ, а не закон:
`kernel/src` в этой карточке не менялся ни строкой.
"""

from __future__ import annotations

from fractions import Fraction
import json
import math
import sys
from pathlib import Path

import pytest
import sympy

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

import cftuv_envelope as kernel  # noqa: E402
from cftuv import envelope_request_export  # noqa: E402
from cftuv.envelope_debug_profile import (  # noqa: E402
    ANGULAR_STAGE_COUNTERS,
    SEAM_PARTITION_COUNTERS,
    EnvelopeDebugProfileBuilderV1,
)
from cftuv.envelope_host_adapter import (  # noqa: E402
    EnvelopeDebugHostOutcome,
    build_envelope_analysis_snapshot,
    build_envelope_decal_request,
)
from cftuv.envelope_request_export import (  # noqa: E402
    _exact_kink_vertex_ids,
    _exact_source_positions,
)
from cftuv.envelope_topology_export import (  # noqa: E402
    build_envelope_topology_export,
)

from envelope_fixture_bundles import (  # noqa: E402
    SEM_CLB_CASE_ROOT,
    U_ROUTE,
    bundle_from_exported_snapshot,
    host_exported_snapshot_paths,
    host_number,
    planar_quad_bundle,
    u_route_bundle,
    u_route_positions,
)


FIELD_CASE = (
    Path(__file__).resolve().parents[1]
    / "kernel"
    / "fixtures"
    / "sem_clb_02_lost_domains_v1"
    / "cases"
    / "building_001_single_edge_patch_000_named_outcome_v1"
)
# Цепочка, прямая в источнике и изломанная НА РЕШЁТКЕ: её разрезать нельзя,
# иначе привязка `ChainStraightEvaluationGeometryBindingV2` теряет предмет.
LATTICE_BENT_CHAIN_ID = "host-v0:physical-chain:722892ff0c5479743b66eacc"

# Перепись разрезов по корпусу, замороженная числом. Ноль означает «закон
# смотрел и не нашёл», а не «закон не включался»; шесть разрезов на
# `patch_001`/`patch_006` — вырожденные проекцией 3D-изломы порядка 1e-5°
# (домены и без них дают EXACT), они объявлены здесь, а не растворены в
# «примерно ноль».
EXPECTED_CUTS_BY_FIXTURE = {
    "building_001_single_edge_patch_000_named_outcome_v1": 1,
    "building_002_single_edge_patch_000_named_outcome_v1": 0,
    "building_003_single_edge_patch_000_named_outcome_v1": 0,
    "building_all_seams_patch_001_lost_resolved_v1": 1,
    "building_all_seams_patch_006_lost_resolved_v1": 5,
    "building_all_seams_patch_011_lost_resolved_v1": 0,
    "building_all_seams_patch_105_lost_resolved_v1": 0,
    "building_002_full_selection_v1": 0,
    "building_002_point_contact_v1": 0,
    "building_002_weighted_normals_v1": 0,
    "building_patch10_density4_v1": 0,
}
# Из них подлинными изломами ЧАРТА — теми, на которых ядро и отказывает —
# является ровно один во всём корпусе.
EXPECTED_CHART_KINKS = {
    "building_001_single_edge_patch_000_named_outcome_v1": [
        ("host-v0:physical-chain:9d42cda796feb85637960858", 5)
    ],
}


def _counters(profile) -> dict[str, float]:
    return {item.name: item.value for item in profile.snapshot().counters}


def _reexport(snapshot, *, profile=None):
    bundle, patch_id = bundle_from_exported_snapshot(snapshot)
    return build_envelope_analysis_snapshot(
        bundle,
        included_patch_ids=frozenset({patch_id}),
        profile=profile,
    )


def _compile(snapshot, selected_edge_ids, alpha=0.3):
    request = build_envelope_decal_request(
        snapshot, frozenset(selected_edge_ids), alpha
    )
    (domain,) = snapshot.patch_domains
    return kernel.compile_reference_envelopes(
        snapshot, request, domain.patch_domain_id
    )


def _field_snapshot():
    return kernel.AnalysisSnapshotCodecV1.loads(
        (FIELD_CASE / "analysis_snapshot.json").read_bytes()
    )


# --------------------------------------------------------------------------
# T1–T4: закон чинит именно то, ради чего заведён
# --------------------------------------------------------------------------


def test_field_fixture_stops_failing_the_kernel_linearity_law():
    """T1. `building.001` патч 0: излом 0.017° режется, домен доезжает до EXACT.

    Байты фикстуры заморожены и пересобираются только из Blender, поэтому
    факты хоста восстанавливаются из самого снапшота и заново прогоняются
    через экспортёр. Восстановление проверяемо: число `CornerRelationV1`
    совпадает с фикстурой, и 27 из 28 идентичностей цепочек воспроизводятся
    ПОБИТОВО — расходится ровно та одна, которая и подлежит разрезу.
    """

    frozen = _field_snapshot()
    assert _compile(frozen, {4}).outcome is (
        kernel.ReferenceOutcome.SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR
    ), "фикстура обязана падать на замороженных байтах — иначе предмет исчез"

    profile = EnvelopeDebugProfileBuilderV1("building.001", "EXACT")
    rebuilt = _reexport(frozen, profile=profile)

    assert len(rebuilt.corner_relations) == len(frozen.corner_relations)
    before = {item.physical_chain_id.value for item in frozen.physical_chains}
    after = {item.physical_chain_id.value for item in rebuilt.physical_chains}
    assert len(before - after) == 1
    assert len(after) == len(before) + 1
    assert _counters(profile)["SEAM_PARTITION_KINK_CUT_VERTICES"] == 1
    assert _compile(rebuilt, {4}).outcome is kernel.ReferenceOutcome.EXACT


def test_source_straight_chain_bent_only_on_the_lattice_is_never_cut():
    """T2. Излом решётки — не излом источника, и цепочка остаётся одной.

    `722892ff…` прямая ТОЧНО в источнике и перестаёт быть прямой только
    после привязки к решётке. Разрезать её означало бы отнять предмет у
    `ChainStraightEvaluationGeometryBindingV2` — привязки, которая как раз и
    существует, чтобы такую цепочку восстановить, а не расщепить.
    """

    frozen = _field_snapshot()
    chain = next(
        item
        for item in frozen.physical_chains
        if item.physical_chain_id.value == LATTICE_BENT_CHAIN_ID
    )
    assert len(chain.ordered_source_vertex_ids) == 3

    rebuilt = _reexport(frozen)
    survivor = next(
        item
        for item in rebuilt.physical_chains
        if item.physical_chain_id.value == LATTICE_BENT_CHAIN_ID
    )
    assert survivor.ordered_source_vertex_ids == chain.ordered_source_vertex_ids

    binding = _compile(rebuilt, {4}).compilation.evaluation_geometry_binding
    assert type(binding) is kernel.ChainStraightEvaluationGeometryBindingV2
    bound = next(
        item
        for item in binding.straight_chain_bindings
        if item.physical_chain_id.value == LATTICE_BENT_CHAIN_ID
    )
    # Внутренняя вершина осталась внутренней: у разрезанной цепочки внутренних
    # вершин не бывает, и привязка V2 просто не имела бы что восстанавливать.
    assert len(bound.internal_assignments) == 1
    assert bound.ordered_source_vertex_ids == chain.ordered_source_vertex_ids


def test_u_route_field_anchor_compiles_and_covers_every_turn():
    """T3. Полевой U-маршрут `walls.001`: четыре подлинных излома по 45°.

    Маршрут 1→8→9→10→11→0 объявлен ОДНОЙ цепочкой — тот самый дефект. После
    разреза домен даёт EXACT, а покрытие вокруг поворотов непрерывно: в
    каждой вершине разреза стоит ровно один `AngularEnvelopeSpec` (веер), и
    ни одного `CapEnvelopeSpec` — колпачки остаются только на физических
    концах маршрута.
    """

    # Фикстура — это ЯКОРЬ, а не его пересказ: координаты читаются из слепка
    # владельца, и здесь проверяется, что читается именно тот маршрут.
    position = u_route_positions()
    route = [position[item] for item in U_ROUTE]
    steps = [
        tuple(b - a for a, b in zip(start, end, strict=True))
        for start, end in zip(route, route[1:])
    ]
    lengths = [math.dist((0.0, 0.0, 0.0), step) for step in steps]
    assert [round(item, 2) for item in lengths] == [3.38, 0.08, 1.72, 0.08, 3.38]
    for incoming, outgoing in zip(steps, steps[1:]):
        cosine = sum(
            a * b for a, b in zip(incoming, outgoing, strict=True)
        ) / (math.dist((0.0, 0.0, 0.0), incoming) * math.dist((0.0, 0.0, 0.0), outgoing))
        assert round(math.degrees(math.acos(cosine)), 3) == 45.0

    profile = EnvelopeDebugProfileBuilderV1("walls.001", "EXACT")
    snapshot = build_envelope_analysis_snapshot(u_route_bundle(), profile=profile)
    counters = _counters(profile)
    assert counters["SEAM_PARTITION_KINK_CHAIN_USES"] == 1
    assert counters["SEAM_PARTITION_KINK_CUT_VERTICES"] == 4
    assert counters["ANGULAR_CUT_REFLEX_RELATIONS"] == 4
    assert counters["ANGULAR_CUT_CHART_COLLINEAR"] == 0

    result = _compile(snapshot, {4, 5, 6, 7, 8})
    assert result.outcome is kernel.ReferenceOutcome.EXACT

    cut_vertices = {
        f"host-vertex:{snapshot.source_revision.value}:{item}"
        for item in U_ROUTE[1:-1]
    }
    corner_vertex_by_relation = {
        item.corner_relation_id: item.source_vertex_id.value
        for item in snapshot.corner_relations
    }
    angular_at = [
        corner_vertex_by_relation[item.source_relation_id]
        for item in result.compilation.envelope_specs
        if type(item).__name__ == "AngularEnvelopeSpec"
    ]
    assert sorted(angular_at) == sorted(cut_vertices)
    assert not {
        item.physical_terminal_source_vertex_id.value
        for item in result.compilation.envelope_specs
        if type(item).__name__ == "CapEnvelopeSpec"
    } & cut_vertices

    # Разобранный руками маршрут даёт ТОТ ЖЕ набор физических цепочек:
    # разрез не изобретает топологию, а восстанавливает объявленную.
    manual = build_envelope_analysis_snapshot(u_route_bundle(split_route=True))
    assert {item.physical_chain_id.value for item in snapshot.physical_chains} == {
        item.physical_chain_id.value for item in manual.physical_chains
    }


def test_both_export_paths_agree_on_physical_chain_identity():
    """T4. Второй потребитель цепочек обязан прийти к тому же набору.

    `build_envelope_topology_export` и `build_envelope_analysis_snapshot` —
    два независимых пути к одним и тем же `PhysicalChainId`. Разойдись они,
    и сцена топологии показывала бы владельцу не те цепочки, которые уехали
    в ядро.
    """

    for bundle in (
        u_route_bundle(),
        planar_quad_bundle(midpoint=(2.0, 0.5, 0.0)),
        planar_quad_bundle(),
    ):
        export = build_envelope_topology_export(bundle)
        snapshot = build_envelope_analysis_snapshot(bundle)
        from_topology = {
            envelope_request_export._typed_value(
                "physical-chain",
                export.source_revision_value,
                bool(record.chain.is_closed),
                record.canonical_edge_ids,
                record.canonical_vertex_ids,
            )
            for record in export.host_chains
        }
        assert from_topology == {
            item.physical_chain_id.value for item in snapshot.physical_chains
        }


# --------------------------------------------------------------------------
# N1–N7: отрицательные ворота
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "path", host_exported_snapshot_paths(), ids=lambda path: path.parent.name
)
def test_kink_law_never_fires_where_the_chart_is_already_linear(path: Path):
    """N1. Регрессионный ноль: закон включается только при найденном изломе.

    По всему корпусу host-выпущенных фикстур закон режет ровно ОДНУ цепочку
    с подлинным изломом чарта — ту, ради которой заведён. Остальные разрезы
    — вырожденные проекцией 3D-изломы (порядка 1e-5°), они законны и учтены
    отдельным счётчиком; цепочек, прямых в 3D, закон не трогает.
    """

    snapshot = kernel.AnalysisSnapshotCodecV1.loads(path.read_bytes())
    bundle, _ = bundle_from_exported_snapshot(snapshot)
    positions = _exact_source_positions(bundle)
    frame = next(iter(snapshot.surface_metric_descriptors))
    chart = {
        item.source_vertex_id.value: (
            Fraction(
                item.domain_coordinate.x.numerator,
                item.domain_coordinate.x.denominator,
            ),
            Fraction(
                item.domain_coordinate.y.numerator,
                item.domain_coordinate.y.denominator,
            ),
        )
        for item in frame.exact_source_vertex_coordinates
    }

    chart_kinks = []
    cut_total = 0
    for chain in snapshot.physical_chains:
        vertices = [host_number(item) for item in chain.ordered_source_vertex_ids]
        cuts = _exact_kink_vertex_ids(
            tuple(vertices), bool(chain.is_closed), positions
        )
        cut_total += len(cuts)
        for index in range(1, len(vertices) - 1):
            if vertices[index] not in cuts:
                continue
            points = [
                chart[chain.ordered_source_vertex_ids[offset].value]
                for offset in (index - 1, index, index + 1)
            ]
            left = (points[1][0] - points[0][0], points[1][1] - points[0][1])
            right = (points[2][0] - points[1][0], points[2][1] - points[1][1])
            if left[0] * right[1] != left[1] * right[0]:
                chart_kinks.append((chain.physical_chain_id.value, vertices[index]))

    assert cut_total == EXPECTED_CUTS_BY_FIXTURE[path.parent.name]
    assert chart_kinks == EXPECTED_CHART_KINKS.get(path.parent.name, [])


@pytest.mark.parametrize(
    "case",
    sorted(SEM_CLB_CASE_ROOT.iterdir()),
    ids=lambda case: case.name,
)
def test_no_domain_of_the_corpus_loses_its_outcome_to_the_law(case: Path):
    """N1. Ни один домен корпуса не теряет исход; ровно один его получает.

    Это и есть предметный смысл «регрессионного нуля»: закон не имеет права
    ухудшить ни одного домена, который и без него доезжал до EXACT, — включая
    два домена, где он делает ЛИШНИЕ разрезы по вырожденным проекцией
    3D-изломам.
    """

    frozen = kernel.AnalysisSnapshotCodecV1.loads(
        (case / "analysis_snapshot.json").read_bytes()
    )
    manifest = json.loads((case / "manifest.json").read_text(encoding="utf-8"))
    (domain,) = frozen.patch_domains
    before = kernel.compile_reference_envelopes(
        frozen,
        kernel.DecalRequestCodecV1.loads((case / "decal_request.json").read_bytes()),
        domain.patch_domain_id,
    ).outcome
    after = _compile(
        _reexport(frozen),
        manifest["effective_domain_physical_edge_ids"],
        float(manifest["requested_alpha"]),
    ).outcome

    assert after is kernel.ReferenceOutcome.EXACT
    if case.name.startswith("building_001_single_edge"):
        assert before is (
            kernel.ReferenceOutcome.SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR
        )
    else:
        assert before is kernel.ReferenceOutcome.EXACT


def test_clean_domain_bytes_do_not_move_when_the_law_finds_no_kink(monkeypatch):
    """N1. Ноль байтов, а не «почти ноль»: снапшот чистого домена неподвижен."""

    def build(bundle):
        return kernel.AnalysisSnapshotCodecV1.dumps(
            build_envelope_analysis_snapshot(bundle)
        )

    clean = (planar_quad_bundle(), u_route_bundle(split_route=True))
    with_law = [build(item) for item in clean]
    monkeypatch.setattr(
        envelope_request_export,
        "_exact_kink_vertex_ids",
        lambda *args, **kwargs: frozenset(),
    )
    assert [build(item) for item in clean] == with_law


def test_reflex_kink_without_a_certified_measure_is_a_named_refusal(monkeypatch):
    """N2. Без сертифицированной меры — ИМЯ отказа, а не наивный разрез.

    Наивный разрез рефлексного излома превращает честный отказ в два
    `CapEnvelopeSpec` вместо веера — тихую потерю. Поэтому при недоступной
    мере не выпускается ничего.
    """

    from cftuv_envelope.reference import angle_measure

    def unavailable(*args, **kwargs):
        raise angle_measure.CertifiedAngleUnavailable(
            "constructed: no certified reflex measure"
        )

    monkeypatch.setattr(
        envelope_request_export, "certified_reflex_measure", unavailable
    )
    with pytest.raises(Exception) as error:
        build_envelope_analysis_snapshot(u_route_bundle())
    assert error.value.outcome is (
        EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE
    )


def test_reflex_cut_vertex_carries_a_fan_and_never_two_caps():
    """N3. В вершине разреза рефлексного излома — ноль колпачков, один веер."""

    snapshot = build_envelope_analysis_snapshot(u_route_bundle())
    result = _compile(snapshot, {4, 5, 6, 7, 8})
    kinds = [type(item).__name__ for item in result.compilation.envelope_specs]
    assert kinds.count("AngularEnvelopeSpec") == 4
    assert kinds.count("CapEnvelopeSpec") == 2
    assert kinds.count("StripEnvelopeSpec") == 5
    seeds = [type(item).__name__ for item in result.compilation.seeds]
    assert seeds.count("CornerSeedV1") == 4
    assert seeds.count("CapSeedV1") == 2


def test_one_thousandth_of_a_degree_is_a_kink_because_there_is_no_threshold():
    """N4. Порог 30° из полевого анализа НЕ наследуется законом разреза.

    Закон ядра точный, значит и предъявляемый ему факт обязан быть точным:
    любая величина, отличная от нуля в рационалах, — излом.
    """

    offset = 2.0 * math.tan(math.radians(0.001) / 2.0)
    bundle = planar_quad_bundle(midpoint=(2.0, offset, 0.0))
    profile = EnvelopeDebugProfileBuilderV1("v0-quad", "EXACT")
    snapshot = build_envelope_analysis_snapshot(bundle, profile=profile)

    assert _counters(profile)["SEAM_PARTITION_KINK_CUT_VERTICES"] == 1
    assert len(snapshot.physical_chains) == 5
    assert _compile(snapshot, {0}).outcome is kernel.ReferenceOutcome.EXACT


def test_the_decimal_string_twin_disagrees_and_the_product_follows_the_kernel():
    """N5. `sympy.Rational(str(float))` — ДРУГОЙ рационал того же float.

    На построенной тройке двойник объявляет цепочку прямой, а закон ядра —
    изломанной. Продукт обязан идти за `float.as_integer_ratio()`: иначе он
    объявил бы прямой ту цепочку, которую ядро прямой не считает, и вернул
    бы ровно тот отказ, ради которого карточка заведена.
    """

    points = ((0.0, 0.0, 0.0), (0.1, 0.3, 0.0), (0.3, 0.9, 0.0))

    def cross(law):
        rows = [tuple(law(value) for value in point) for point in points]
        left = tuple(a - b for a, b in zip(rows[1], rows[0], strict=True))
        right = tuple(a - b for a, b in zip(rows[2], rows[1], strict=True))
        return left[0] * right[1] - left[1] * right[0]

    binary = cross(lambda value: Fraction(*float(value).as_integer_ratio()))
    decimal = cross(lambda value: Fraction(sympy.Rational(str(float(value)))))
    assert decimal == 0
    assert binary != 0

    positions = dict(enumerate(
        tuple(Fraction(*float(value).as_integer_ratio()) for value in point)
        for point in points
    ))
    assert _exact_kink_vertex_ids((0, 1, 2), False, positions) == frozenset({1})


def test_disabling_the_partition_returns_exactly_the_kernel_refusal(monkeypatch):
    """N6. Выключенный закон возвращает исход РОВНО в прежнее имя."""

    monkeypatch.setattr(
        envelope_request_export,
        "_exact_kink_vertex_ids",
        lambda *args, **kwargs: frozenset(),
    )
    snapshot = build_envelope_analysis_snapshot(u_route_bundle())
    assert _compile(snapshot, {4, 5, 6, 7, 8}).outcome is (
        kernel.ReferenceOutcome.SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR
    )

    rebuilt = _reexport(_field_snapshot())
    assert _compile(rebuilt, {4}).outcome is (
        kernel.ReferenceOutcome.SOURCE_DECLARED_STRAIGHT_CHAIN_IS_NOT_LINEAR
    )


def test_kink_degenerate_in_the_chart_is_still_cut_and_is_counted():
    """N7. Излом в 3D, коллинеарный в чарте: режется и объявляется числом.

    Проекция near-planar чарта аффинна, поэтому 3D-коллинеарность ВЛЕЧЁТ
    коллинеарность чарта, но не наоборот. Сверх-разбиение вырожденной
    проекции законно — оба закона линейности от него не страдают — и не
    прячется: `ANGULAR_CUT_CHART_COLLINEAR` считает такие разрезы отдельно
    от подлинных поворотов.
    """

    bundle = planar_quad_bundle(midpoint=(2.0, 0.0, 1e-6))
    profile = EnvelopeDebugProfileBuilderV1("v0-quad", "EXACT")
    snapshot = build_envelope_analysis_snapshot(bundle, profile=profile)

    counters = _counters(profile)
    assert counters["SEAM_PARTITION_KINK_CUT_VERTICES"] == 1
    assert counters["ANGULAR_CUT_VERTICES_CONSIDERED"] == 1
    assert counters["ANGULAR_CUT_CHART_COLLINEAR"] == 1
    assert counters["ANGULAR_CUT_REFLEX_RELATIONS"] == 0
    assert len(snapshot.physical_chains) == 5
    assert _compile(snapshot, {0}).outcome is kernel.ReferenceOutcome.EXACT


def test_every_declared_counter_is_published_even_as_zero():
    """Ноль стадии объявлен перечнем, а не подразумевается её молчанием."""

    profile = EnvelopeDebugProfileBuilderV1("v0-quad", "EXACT")
    build_envelope_analysis_snapshot(planar_quad_bundle(), profile=profile)
    published = _counters(profile)
    assert set(ANGULAR_STAGE_COUNTERS) | set(SEAM_PARTITION_COUNTERS) <= set(
        published
    )
    # Патч прямой: все числа разреза — объявленные нули, а углы петли считаны.
    assert published["ANGULAR_CORNERS_CONSIDERED"] == 4
    for name in (*SEAM_PARTITION_COUNTERS, "ANGULAR_CUT_VERTICES_CONSIDERED",
                 "ANGULAR_CUT_CHART_COLLINEAR", "ANGULAR_CUT_REFLEX_RELATIONS"):
        assert published[name] == 0
