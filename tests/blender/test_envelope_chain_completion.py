"""Blender 4.3 background smoke: дополнение выделения и видимый отказ домена.

Три утверждения, и каждое стоит на числе, а не на виде картинки:

1. выделено ОДНО ребро двухрёберного шва — билд FINISHED, выделение дополнено
   (счётчики, диагностика сцены, строка панели), обе стороны шва в отчёте, а
   выделение владельца во вьюпорте вернулось ИСХОДНЫМ;
2. домен, не дошедший до покрытия, получает контур в слое
   `ENV_93_QUEUE_REFUSED` с именем исхода в самом штрихе — отказ видно во
   вьюпорте, а не только строкой статуса;
3. чисто топологический прогон не объявляет отказавшим НИ ОДИН домен: там
   `TOPOLOGY_READY` — это успех, и без этого различия слой красил бы всё.

Прогон: blender --background --factory-startup --python <этот файл>
Последняя строка при успехе: ENVELOPE_CHAIN_COMPLETION_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

import bmesh
import bpy


REPO_ROOT = Path(__file__).resolve().parents[2]
for path in (REPO_ROOT, REPO_ROOT / "kernel" / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_envelope_debug_bridge import (  # noqa: E402
    REQUIRED_LAYERS,
    _build_two_patch_multi_edge_seam,
    _build_two_patch_seam,
    _enter_edge_selection,
    _layer_name,
    _reset_scene,
    _sidecar_payload,
)


REFUSED_LAYER = "ENV_93_QUEUE_REFUSED"
COMPLETION_DIAGNOSTIC = "PARTIAL_CHAIN_SELECTION_COMPLETED"


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _gp_object(source_obj):
    return bpy.data.objects["CFTUV_DEBUG_Envelope_" + source_obj.name]


def _layer_names(source_obj):
    return {
        _layer_name(layer) for layer in _gp_object(source_obj).data.layers
    }


def _profile_counters(source_obj):
    profile = json.loads(
        bpy.data.texts[
            "CFTUV_EnvelopeProfile_" + source_obj.name + ".json"
        ].as_string()
    )
    counters = {}
    for item in profile["counters"]:
        if item["patch_domain_id"] is None:
            counters[item["name"]] = item["value"]
    return counters


def _refused_strokes(payload):
    return [
        item
        for item in payload["strokes"]
        if item.get("layer") == REFUSED_LAYER
    ]


def _selected_edge_indices(source_obj):
    mesh = bmesh.from_edit_mesh(source_obj.data)
    mesh.edges.ensure_lookup_table()
    return {edge.index for edge in mesh.edges if edge.select}


def _run_one_edge_of_a_two_edge_chain_builds_both_sides():
    _reset_scene()
    source_obj, seam_edges = _build_two_patch_multi_edge_seam()
    requested = (seam_edges[0],)
    _enter_edge_selection(source_obj, requested)

    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )

    payload = _sidecar_payload(source_obj)
    # Обе стороны шва: два домена и пара ChainUse на полной цепочке из двух рёбер.
    assert len(payload["patch_domain_ids"]) == 2, payload["patch_domain_ids"]
    patch_pairs = [
        item
        for item in payload["topology_pairs"]
        if item["kind"] == "PATCH"
        and tuple(sorted(item["host_edge_ids"])) == seam_edges
    ]
    assert len(patch_pairs) == 1, payload["topology_pairs"]
    assert len(patch_pairs[0]["patch_domain_ids"]) == 2

    # Дополнение объявлено: диагностика сцены, счётчики и строка панели.
    completed = [
        item
        for item in payload["topology_selection_diagnostics"]
        if item["code"] == COMPLETION_DIAGNOSTIC
    ]
    assert len(completed) == 1, payload["topology_selection_diagnostics"]
    assert str(seam_edges[1]) in completed[0]["message"], completed[0]
    counters = _profile_counters(source_obj)
    assert counters["SELECTION_COMPLETED_CHAINS"] == 1, counters
    assert counters["SELECTION_COMPLETED_EDGES"] == 1, counters
    assert counters["SELECTED_CHAINS"] == 1, counters
    assert counters["SELECTED_DOMAINS"] == 2, counters
    summary = _settings().envelope_debug_stage_summary
    assert "Selection completed: 1 chains, +1 edges" in summary, summary

    # Выделение источника — ПОЛНАЯ цепочка, а не одно ребро.
    selected_sources = [
        item
        for item in payload["strokes"]
        if item.get("kind") == "SELECTED_SOURCE"
    ]
    assert selected_sources
    for item in selected_sources:
        assert tuple(sorted(item["host_edge_ids"])) == seam_edges, item

    # Оба домена дошли до покрытия, поэтому слой отказа существует и ПУСТ.
    receipts = json.loads(_gp_object(source_obj)["stage_receipts"])
    assert {item["stage"] for item in receipts} == {"RESOLVED"}, receipts
    assert REFUSED_LAYER in _layer_names(source_obj)
    assert _refused_strokes(payload) == []
    assert payload["stage_outcomes"]["REFUSED"] == 0

    # Выделение владельца восстановлено ИСХОДНЫМ, а не дополненным.
    assert _selected_edge_indices(source_obj) == set(requested)


def _run_refused_domain_gets_a_contour_and_a_name():
    _reset_scene()
    # 1e-2, а НЕ 1e-3: половина ячейки привязки источника на этом патче крупнее
    # тысячной, и отклонение в 1e-3 привязка кладёт в плоскость точно — домен
    # проходит, и отказу, который надо нарисовать, взяться неоткуда. Число
    # измерено, а не подобрано: тот же порог держит
    # `test_a_deviation_below_half_a_cell_is_absorbed_by_the_source_snap`.
    source_obj = _build_two_patch_seam(
        nonplanar_second_patch=True,
        second_patch_offset=0.01,
    )

    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )

    payload = _sidecar_payload(source_obj)
    receipts = json.loads(_gp_object(source_obj)["stage_receipts"])
    refused_receipts = [
        item for item in receipts if item["stage"] != "RESOLVED"
    ]
    assert len(refused_receipts) == 1, receipts
    refused_domain = refused_receipts[0]["patch_domain_id"]
    # Стадия и исход — разные величины: стадия говорит, ГДЕ домен встал, исход —
    # ЧЕМ он отказал. В штрих обязаны попасть оба.
    assert refused_receipts[0]["stage"] == "METRIC_REJECTED", refused_receipts
    outcome = refused_receipts[0]["outcome"]
    assert outcome == "RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED", (
        refused_receipts[0]
    )

    strokes = _refused_strokes(payload)
    assert strokes, payload["stage_outcomes"]
    assert {item["patch_domain_id"] for item in strokes} == {refused_domain}
    for item in strokes:
        assert item["stage"] == "REFUSED"
        assert item["outcome"] == outcome
        assert item["domain_stage"] == "METRIC_REJECTED"
        assert outcome in item["label"], item
    assert payload["stage_outcomes"]["REFUSED"] == 1

    # Чекбокс работает как у остальных слоёв и гасит ТОЛЬКО слой отказа.
    settings = _settings()
    settings.envelope_debug_show_refused = False
    layers = {
        _layer_name(layer): layer
        for layer in _gp_object(source_obj).data.layers
    }
    assert layers[REFUSED_LAYER].hide
    assert not layers["ENV_00_PATCH_DOMAIN"].hide
    settings.envelope_debug_show_refused = True
    assert not layers[REFUSED_LAYER].hide


def _run_topology_only_build_refuses_nothing():
    _reset_scene()
    source_obj = _build_two_patch_seam()

    assert bpy.ops.hotspotuv.build_envelope_topology_debug() == {"FINISHED"}

    payload = _sidecar_payload(source_obj)
    receipts = json.loads(_gp_object(source_obj)["stage_receipts"])
    assert {item["stage"] for item in receipts} == {"TOPOLOGY_READY"}
    assert _layer_names(source_obj) == REQUIRED_LAYERS
    assert _refused_strokes(payload) == []
    assert payload["stage_outcomes"]["REFUSED"] == 0


def _main():
    import cftuv

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован установленной копией
        pass
    assert hasattr(
        bpy.ops.hotspotuv,
        "build_exact_reference_envelope_debug",
    )
    _run_one_edge_of_a_two_edge_chain_builds_both_sides()
    _run_refused_domain_gets_a_contour_and_a_name()
    _run_topology_only_build_refuses_nothing()
    print("ENVELOPE_CHAIN_COMPLETION_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
