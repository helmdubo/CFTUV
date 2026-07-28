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
    envelope_debug_object_name,
    envelope_debug_profile_text_name,
    envelope_debug_text_name,
    refused_receipts,
    render_refused_domains,
    visibility_from_settings,
)
from cftuv.envelope_topology_debug import (
    ENVELOPE_TOPOLOGY_DEBUG_SCENE_SCHEMA_V1,
    EnvelopeTopologyDebugPathV1,
    EnvelopeTopologyDebugSceneV1,
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
