from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

from cftuv.envelope_debug_profile import (
    EnvelopeDebugProfileBuilderV1,
    EnvelopeDomainStage,
    EnvelopeDomainStageReceiptV1,
)
from cftuv.envelope_debug_renderer import (
    ENVELOPE_DEBUG_LABEL_LAYER,
    ENVELOPE_DEBUG_LAYER_STYLES,
    ENVELOPE_DEBUG_REFUSED_LAYER,
    _render_topology_scene,
    envelope_debug_object_name,
    envelope_debug_profile_text_name,
    envelope_debug_text_name,
    refused_receipts,
    render_refused_domains,
    visibility_from_settings,
)
from cftuv.envelope_topology_debug import (
    ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1,
    EnvelopeTopologyDebugPairV1,
    EnvelopeTopologyDebugPathV1,
    EnvelopeTopologyDebugSceneV1,
    EnvelopeTopologyPairKind,
    EnvelopeTopologyPathKind,
)


REQUIRED_LAYERS = {
    "ENV_93_QUEUE_REFUSED",
    "ENV_00_PATCH_DOMAIN",
    "ENV_01_HOLES",
    "ENV_02_BARRIERS",
    "ENV_10_PHYSICAL_CHAINS",
    "ENV_11_CHAIN_USES",
    "ENV_12_SELECTED_SOURCES",
    "ENV_13_PATCH_PAIRS",
    "ENV_14_SEAM_SELF_PAIRS",
    "ENV_20_SOURCE_SUPPORTS",
    "ENV_21_MOVING_FRONTS",
    "ENV_22_HIDDEN_SUPPORTS",
    "ENV_30_ENVELOPE_INSTANCES",
    "ENV_40_RAW_COVERAGE",
    "ENV_53_POINT_CONTACTS",
    "ENV_54_BOUNDARY_OCCURRENCES",
    "ENV_50_INTERACTION_COMPONENTS",
    "ENV_51_FRONT_READINGS",
    "ENV_52_EQUALITY_LOCI",
    "ENV_60_RESOLVED_COVERAGE",
    "ENV_61_RETAINED_REGIONS",
    "ENV_62_REMOVED_REGIONS",
    "ENV_70_EVENT_ANCHORS",
    "ENV_80_DIAGNOSTICS",
    "ENV_LABELS",
}

# Слои движка QUEUE. Отдельным множеством, а не влитые в предыдущее: на
# движке LEGACY они в GP-объекте НЕ создаются вовсе, и разделение здесь —
# то же утверждение, только исполняемое.
QUEUE_LAYERS = {
    "ENV_90_QUEUE_SKELETON",
    "ENV_91_QUEUE_WALLS",
} | {f"ENV_92_QUEUE_OWNER_{slot:02d}" for slot in range(8)}


def test_v0b_layer_contract_is_complete_and_named():
    assert set(ENVELOPE_DEBUG_LAYER_STYLES) == REQUIRED_LAYERS | QUEUE_LAYERS
    assert ENVELOPE_DEBUG_LABEL_LAYER == "ENV_LABELS"
    assert envelope_debug_object_name("Cube") == "CFTUV_DEBUG_Envelope_Cube"
    assert (
        envelope_debug_text_name("Cube")
        == "CFTUV_EnvelopeDebug_Cube.json"
    )
    assert (
        envelope_debug_profile_text_name("Cube")
        == "CFTUV_EnvelopeProfile_Cube.json"
    )


def test_refused_layer_reuses_the_barrier_red_and_lifts_above_every_other():
    """Цвет отказа взят из палитры, а не изобретён, и это утверждение исполняемое.

    Порядок ключей задаёт высоту подъёма над поверхностью, поэтому слой стоит
    ПОСЛЕДНИМ: вставка в середину сдвинула бы подъём всех следующих слоёв.
    """

    color, line_width = ENVELOPE_DEBUG_LAYER_STYLES[ENVELOPE_DEBUG_REFUSED_LAYER]
    assert color == ENVELOPE_DEBUG_LAYER_STYLES["ENV_02_BARRIERS"][0]
    assert line_width > 0
    assert list(ENVELOPE_DEBUG_LAYER_STYLES)[-1] == ENVELOPE_DEBUG_REFUSED_LAYER
    # Слой отказа не принадлежит движку QUEUE, несмотря на имя: METRIC_REJECTED
    # бывает и на LEGACY, поэтому он создаётся всегда.
    assert ENVELOPE_DEBUG_REFUSED_LAYER not in QUEUE_LAYERS


def test_visibility_groups_map_to_runtime_layers_without_geometry_changes():
    settings = SimpleNamespace(
        envelope_debug_show_domains=False,
        envelope_debug_show_chains=True,
        envelope_debug_show_supports=False,
        envelope_debug_show_envelopes=True,
        envelope_debug_show_raw=False,
        envelope_debug_show_readings=True,
        envelope_debug_show_equality=False,
        envelope_debug_show_resolved=True,
        envelope_debug_show_diagnostics=False,
        envelope_debug_show_refused=False,
        envelope_debug_show_labels=True,
    )
    visibility = visibility_from_settings(settings)
    assert visibility[ENVELOPE_DEBUG_REFUSED_LAYER] is False
    assert visibility["ENV_00_PATCH_DOMAIN"] is False
    assert visibility["ENV_10_PHYSICAL_CHAINS"] is True
    assert visibility["ENV_12_SELECTED_SOURCES"] is True
    assert visibility["ENV_13_PATCH_PAIRS"] is True
    assert visibility["ENV_14_SEAM_SELF_PAIRS"] is True
    assert visibility["ENV_20_SOURCE_SUPPORTS"] is False
    assert visibility["ENV_40_RAW_COVERAGE"] is False
    assert visibility["ENV_53_POINT_CONTACTS"] is False
    assert visibility["ENV_54_BOUNDARY_OCCURRENCES"] is False
    assert visibility["ENV_51_FRONT_READINGS"] is True
    assert visibility["ENV_52_EQUALITY_LOCI"] is False
    assert visibility["ENV_60_RESOLVED_COVERAGE"] is True
    assert visibility["ENV_80_DIAGNOSTICS"] is False
    assert visibility["ENV_LABELS"] is True


def _receipt(domain_id, stage):
    return EnvelopeDomainStageReceiptV1(
        0,
        domain_id,
        stage,
        stage.value,
        "measured",
    )


def _profile_with(build_kind, *receipts):
    profile = EnvelopeDebugProfileBuilderV1("Mesh", build_kind)
    for receipt in receipts:
        profile.set_receipt(receipt)
    return profile.snapshot()


def test_only_domains_short_of_coverage_count_as_refused():
    snapshot = _profile_with(
        "EXACT_REFERENCE",
        _receipt("domain-a", EnvelopeDomainStage.RESOLVED),
        _receipt("domain-b", EnvelopeDomainStage.METRIC_REJECTED),
        _receipt("domain-c", EnvelopeDomainStage.QUEUE_RESOLVED),
        _receipt("domain-d", EnvelopeDomainStage.QUEUE_PREPARE_REJECTED),
    )

    refused = refused_receipts(snapshot)

    assert [item.patch_domain_id for item in refused] == [
        "domain-b",
        "domain-d",
    ]
    assert [item.outcome for item in refused] == [
        "METRIC_REJECTED",
        "QUEUE_PREPARE_REJECTED",
    ]


def test_console_receipt_prefix_matches_resolved_stages(capsys):
    """`QUEUE_RESOLVED: EXACT` в консоли печатается как RESOLVED, не REJECTED.

    Полевой факт, которым это оплачено: на walls.001 домен стены построился
    (EXACT, Resolved 1/2), но в консоли его строка начиналась с «REJECTED» —
    успех, объявленный отказом. Числа покрытия из сообщения расписки при
    этом обязаны остаться на экране.
    """

    from cftuv.envelope_debug_renderer import _print_profile

    snapshot = _profile_with(
        "QUEUE",
        _receipt("domain-a", EnvelopeDomainStage.QUEUE_RESOLVED),
        _receipt("domain-b", EnvelopeDomainStage.QUEUE_PREPARE_REJECTED),
    )
    capsys.readouterr()
    _print_profile(snapshot)
    printed = capsys.readouterr().out

    lines = [line.strip() for line in printed.splitlines()]
    resolved = [line for line in lines if line.startswith("RESOLVED ")]
    rejected = [line for line in lines if line.startswith("REJECTED ")]
    assert len(resolved) == 1
    assert "QUEUE_RESOLVED" in resolved[0]
    assert "domain-a"[-24:] in resolved[0]
    assert len(rejected) == 1
    assert "QUEUE_PREPARE_REJECTED" in rejected[0]
    assert "domain-b"[-24:] in rejected[0]
    # Сообщение расписки успеха (числа покрытия) не потеряно печатью.
    assert lines[lines.index(resolved[0]) + 1] == "measured"


def test_topology_only_build_refuses_nothing():
    """`TOPOLOGY_READY` на топологическом прогоне — успех, а не отказ.

    Без этого различия чисто топологическая кнопка красила бы красным КАЖДЫЙ
    домен каждого прогона, и слой отказа перестал бы что-либо означать.
    """

    snapshot = _profile_with(
        "TOPOLOGY",
        _receipt("domain-a", EnvelopeDomainStage.TOPOLOGY_READY),
        _receipt("domain-b", EnvelopeDomainStage.TOPOLOGY_READY),
    )

    assert refused_receipts(snapshot) == ()
    assert refused_receipts(None) == ()
    # На постадийном прогоне та же стадия — уже отказ: домен не дошёл дальше.
    staged = _profile_with(
        "EXACT_REFERENCE",
        _receipt("domain-a", EnvelopeDomainStage.TOPOLOGY_READY),
    )
    assert len(refused_receipts(staged)) == 1


class _RecordingWriter:
    """Писатель штрихов без Blender: он и есть измерение этого теста."""

    def __init__(self):
        self.paths = []

    def add_path(self, layer_name, points, *, line_width, cyclic=False):
        self.paths.append((layer_name, tuple(points), line_width, cyclic))
        return len(self.paths) - 1


def _loop_path(semantic_id, domain_id, kind, style_key):
    return EnvelopeTopologyDebugPathV1(
        semantic_id=semantic_id,
        patch_domain_id=domain_id,
        kind=kind,
        local_points=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)),
        host_vertex_ids=(0, 1, 2),
        host_edge_ids=(0, 1, 2),
        closed=True,
        directed=False,
        selected=False,
        style_key=style_key,
        label=f"Patch {domain_id} OUTER",
        boundary_loop_id=semantic_id,
        display_normal=(0.0, 0.0, 1.0),
    )


def _topology_scene():
    return EnvelopeTopologyDebugSceneV1(
        ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1,
        "rev",
        ("domain-a", "domain-b"),
        (0,),
        (
            _loop_path(
                "loop-a",
                "domain-a",
                EnvelopeTopologyPathKind.PATCH_OUTER_LOOP,
                "ENV_00_PATCH_DOMAIN",
            ),
            _loop_path(
                "loop-b",
                "domain-b",
                EnvelopeTopologyPathKind.PATCH_OUTER_LOOP,
                "ENV_00_PATCH_DOMAIN",
            ),
            _loop_path(
                "hole-b",
                "domain-b",
                EnvelopeTopologyPathKind.PATCH_HOLE_LOOP,
                "ENV_01_HOLES",
            ),
        ),
        (),
        (),
    )


def test_refused_domain_contour_carries_the_outcome_name():
    """Контур отказавшего домена рисуется, и имя исхода стоит в самом штрихе.

    Полевой факт, которым это оплачено: домен отказывал
    `QUEUE_PREPARE_REJECTED`, а во вьюпорте выглядел ровно как отсутствующий —
    владелец читал «строит с ошибкой на одну сторону».
    """

    writer = _RecordingWriter()
    stroke_map = []
    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }

    point_count = render_refused_domains(
        _topology_scene(),
        refused_receipts(
            _profile_with(
                "QUEUE",
                _receipt("domain-a", EnvelopeDomainStage.QUEUE_RESOLVED),
                _receipt(
                    "domain-b",
                    EnvelopeDomainStage.QUEUE_PREPARE_REJECTED,
                ),
            )
        ),
        writer,
        layer_ordinals,
        stroke_map,
    )

    # Только отказавший домен, и обе его петли: внешняя и дырка.
    assert [item[0] for item in writer.paths] == [
        ENVELOPE_DEBUG_REFUSED_LAYER,
        ENVELOPE_DEBUG_REFUSED_LAYER,
    ]
    assert all(item[3] is True for item in writer.paths)
    assert point_count == 6
    assert {item["patch_domain_id"] for item in stroke_map} == {"domain-b"}
    assert {item["kind"] for item in stroke_map} == {
        "PATCH_OUTER_LOOP",
        "PATCH_HOLE_LOOP",
    }
    for item in stroke_map:
        assert item["stage"] == "REFUSED"
        assert item["outcome"] == "QUEUE_PREPARE_REJECTED"
        assert item["domain_stage"] == "QUEUE_PREPARE_REJECTED"
        assert "QUEUE_PREPARE_REJECTED" in item["label"]


def test_nothing_is_drawn_without_a_refusal_or_without_topology():
    writer = _RecordingWriter()
    stroke_map = []
    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }

    assert render_refused_domains(
        _topology_scene(), (), writer, layer_ordinals, stroke_map
    ) == 0
    assert render_refused_domains(
        None,
        refused_receipts(
            _profile_with(
                "QUEUE",
                _receipt("domain-b", EnvelopeDomainStage.METRIC_REJECTED),
            )
        ),
        writer,
        layer_ordinals,
        stroke_map,
    ) == 0
    assert writer.paths == []
    assert stroke_map == []


def _chain_path(local_points, *, closed=False):
    return EnvelopeTopologyDebugPathV1(
        semantic_id="chain-a",
        patch_domain_id="domain-a",
        kind=EnvelopeTopologyPathKind.PHYSICAL_CHAIN,
        local_points=local_points,
        host_vertex_ids=tuple(range(len(local_points))),
        host_edge_ids=tuple(range(len(local_points) - 1)),
        closed=closed,
        directed=False,
        selected=False,
        style_key="ENV_10_PHYSICAL_CHAINS",
        label="chain",
        physical_chain_id="physical-chain:a",
        display_normal=(0.0, 0.0, 1.0),
    )


def _pair_scene(local_points, *, kind, closed=False):
    domains = (
        ("domain-a", "domain-b")
        if kind is EnvelopeTopologyPairKind.PATCH
        else ("domain-a", "domain-a")
    )
    return EnvelopeTopologyDebugSceneV1(
        ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1,
        "rev",
        ("domain-a", "domain-b"),
        (0,),
        (_chain_path(local_points, closed=closed),),
        (
            EnvelopeTopologyDebugPairV1(
                pair_id="pair-a",
                kind=kind,
                physical_chain_id="physical-chain:a",
                chain_use_ids=("use-a", "use-b"),
                patch_domain_ids=domains,
                host_edge_ids=(0, 1),
            ),
        ),
        (),
    )


def _pair_marker_centers(scene, layer_name):
    writer = _RecordingWriter()
    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }
    _render_topology_scene(scene, writer, layer_ordinals, [])
    centers = []
    for name, points, _width, _cyclic in writer.paths:
        if name != layer_name:
            continue
        first, second = points
        centers.append(tuple((first[axis] + second[axis]) / 2.0 for axis in range(3)))
    return centers


def _chain_vertices(scene):
    """Вершины цепи в тех же координатах, в которых её рисует слой цепей."""

    writer = _RecordingWriter()
    layer_ordinals = {
        name: ordinal
        for ordinal, name in enumerate(ENVELOPE_DEBUG_LAYER_STYLES)
    }
    _render_topology_scene(scene, writer, layer_ordinals, [])
    for name, points, _width, _cyclic in writer.paths:
        if name == "ENV_10_PHYSICAL_CHAINS":
            return [tuple(point) for point in points]
    raise AssertionError("слой цепей не нарисован")


def test_pair_marker_avoids_every_chain_vertex_on_a_two_edge_chain():
    """Крестик пары — не junction-пометка, и это утверждение исполняемое.

    Полевое наблюдение владельца: на цепи из двух коллинеарных рёбер крестик
    `ENV_13_PATCH_PAIRS` садился ровно на внутреннюю вершину, то есть маскировал
    запись о ЦЕПИ под семантику УГЛА. Причина была в том, что место маркера
    бралось счётом вершин (`points[len(points) // 2]`), а не длиной цепи.

    Проверяется случай РАВНЫХ половин — тот самый, на котором и середина дуги
    села бы на вершину: ребро шва, разбитое вершиной пополам.
    """

    scene = _pair_scene(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        kind=EnvelopeTopologyPairKind.PATCH,
    )
    vertices = _chain_vertices(scene)
    centers = _pair_marker_centers(scene, "ENV_13_PATCH_PAIRS")

    # Крестик — два штриха, и оба стоят в ОДНОЙ точке.
    assert len(centers) == 2
    assert centers[0] == centers[1]
    marker = centers[0]
    assert all(
        marker[:2] != vertex[:2] for vertex in vertices
    ), (marker, vertices)
    # И он лежит на самой цепи, а не рядом с ней: середина второго ребра.
    assert marker[:2] == (1.5, 0.0)


def test_pair_marker_follows_length_and_not_the_vertex_count():
    """Место маркера определяет ДЛИНА цепи, а не то, где её разбили вершины.

    Одна и та же геометрия, разбитая по-разному, даёт разное место у прежнего
    правила и одно и то же — у нового: лишняя вершина не двигает длину.
    """

    coarse = _pair_scene(
        ((0.0, 0.0, 0.0), (4.0, 0.0, 0.0)),
        kind=EnvelopeTopologyPairKind.PATCH,
    )
    fine = _pair_scene(
        (
            (0.0, 0.0, 0.0),
            (0.5, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (4.0, 0.0, 0.0),
        ),
        kind=EnvelopeTopologyPairKind.PATCH,
    )

    assert _pair_marker_centers(coarse, "ENV_13_PATCH_PAIRS")[0][:2] == (2.0, 0.0)
    # Середина дуги (2.0) лежит внутри третьего ребра 1.0 -> 4.0; маркер идёт в
    # его середину. Прежнее правило поставило бы его в вершину (1.0, 0.0).
    marker = _pair_marker_centers(fine, "ENV_13_PATCH_PAIRS")[0]
    assert marker[:2] == (2.5, 0.0)
    assert all(
        marker[:2] != vertex[:2] for vertex in _chain_vertices(fine)
    )


def test_seam_self_pairs_use_the_same_placement_rule():
    """SEAM_SELF идёт тем же кодом: два правила разошлись бы молча."""

    scene = _pair_scene(
        ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
        kind=EnvelopeTopologyPairKind.SEAM_SELF,
    )
    centers = _pair_marker_centers(scene, "ENV_14_SEAM_SELF_PAIRS")

    assert len(centers) == 2
    assert centers[0][:2] == (1.5, 0.0)
    assert _pair_marker_centers(scene, "ENV_13_PATCH_PAIRS") == []


def test_a_closed_chain_counts_its_closing_span():
    """У замкнутой цепи дуга — периметр, а не путь до последней вершины."""

    square = (
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
        (0.0, 2.0, 0.0),
    )
    scene = _pair_scene(
        square, kind=EnvelopeTopologyPairKind.PATCH, closed=True
    )
    marker = _pair_marker_centers(scene, "ENV_13_PATCH_PAIRS")[0]

    # Периметр 8, середина дуги 4 — конец второго ребра; маркер уходит в
    # середину ТРЕТЬЕГО, то есть снова не на вершину.
    assert marker[:2] == (1.0, 2.0)
    assert all(
        marker[:2] != vertex[:2] for vertex in _chain_vertices(scene)
    )


def test_renderer_does_not_import_legacy_or_production_geometry():
    path = (
        Path(__file__).resolve().parents[1]
        / "cftuv"
        / "envelope_debug_renderer.py"
    )
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add(node.module or "")
    assert not imports & {
        "cftuv.decal_voronoi",
        "cftuv.decals",
        "decal_voronoi",
        "decals",
        "cftuv_envelope.contracts.geometry_batch",
        "cftuv_envelope.contracts.ownership",
    }
