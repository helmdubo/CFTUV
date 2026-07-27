"""Blender 4.3 background smoke: движок QUEUE в Envelope-debug.

Три утверждения, и каждое стоит на числе, а не на виде картинки:

1. при умолчании (LEGACY) набор слоёв, sidecar и счётчик подготовки очереди
   ТЕ ЖЕ, что были: движок не включается сам;
2. при QUEUE слои очереди созданы, грани разложены по слоям своих владельцев,
   и соответствие слот -> цвет -> spec_id -> instance_id лежит в sidecar;
3. смена alpha не пересобирает подготовку — по счётчику кэша сессии, а не по
   ощущению скорости.

Прогон: blender --background --factory-startup --python <этот файл>
Последняя строка при успехе: ENVELOPE_QUEUE_BLENDER_SMOKE_OK
"""

from __future__ import annotations

from fractions import Fraction
import json
from pathlib import Path
import sys

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
    _build_two_patch_seam,
    _layer_name,
    _reset_scene,
    _sidecar_payload,
)


QUEUE_LAYERS = {
    "ENV_90_QUEUE_SKELETON",
    "ENV_91_QUEUE_WALLS",
} | {f"ENV_92_QUEUE_OWNER_{slot:02d}" for slot in range(8)}


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _controller():
    return bpy.context.window_manager._cftuv_envelope_debug_session


def _gp_object(source_obj):
    return bpy.data.objects["CFTUV_DEBUG_Envelope_" + source_obj.name]


def _layer_names(source_obj):
    return {
        _layer_name(layer) for layer in _gp_object(source_obj).data.layers
    }


def _queue_strokes(payload):
    return [
        item
        for item in payload["strokes"]
        if item.get("stage") == "QUEUE"
    ]


def _run_legacy_default_creates_no_queue_layer():
    """Умолчание не включает очередь: ни слоя, ни раздела, ни подготовки."""

    _reset_scene()
    source_obj = _build_two_patch_seam()
    settings = _settings()
    assert settings.envelope_debug_engine == "LEGACY"
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )
    assert _layer_names(source_obj) == REQUIRED_LAYERS
    payload = _sidecar_payload(source_obj)
    assert "queue" not in payload
    assert not _queue_strokes(payload)
    assert _controller().build_counts["CONVEYOR_PREPARATION"] == 0
    assert settings.envelope_debug_queue_timing == ""
    # Ползунок на движке LEGACY не считает и не перерисовывает НИЧЕГО.
    settings.envelope_debug_alpha = 0.31
    assert settings.envelope_debug_queue_timing == ""
    assert _controller().build_counts["CONVEYOR_PREPARATION"] == 0
    return source_obj


def _run_queue_engine_paints_owners():
    _reset_scene()
    source_obj = _build_two_patch_seam()
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )

    assert _layer_names(source_obj) == REQUIRED_LAYERS | QUEUE_LAYERS
    payload = _sidecar_payload(source_obj)
    queue = payload["queue"]
    assert queue["engine"] == "QUEUE"
    assert queue["palette_wrapped"] == 0
    assert queue["domains"]
    for domain in queue["domains"]:
        assert domain["preparation_outcome"] == "EXACT", domain
        assert domain["coverage_outcome"] == "EXACT", domain

    slot_by_spec = {
        item["envelope_spec_id"]: item["palette_slot"]
        for item in queue["owners"]
    }
    layer_by_spec = {
        item["envelope_spec_id"]: item["layer"] for item in queue["owners"]
    }
    assert slot_by_spec
    faces = [
        item
        for item in _queue_strokes(payload)
        if item["kind"] == "QUEUE_COVERAGE_FACE"
    ]
    assert faces
    for face in faces:
        assert face["layer"] == layer_by_spec[face["envelope_spec_id"]]
        assert face["palette_slot"] == slot_by_spec[face["envelope_spec_id"]]
        assert face["envelope_instance_id"]
    assert any(
        item["layer"] == "ENV_90_QUEUE_SKELETON"
        for item in _queue_strokes(payload)
    )
    assert any(
        item["layer"] == "ENV_91_QUEUE_WALLS"
        for item in _queue_strokes(payload)
    )
    receipts = json.loads(_gp_object(source_obj)["stage_receipts"])
    assert {item["stage"] for item in receipts} == {"QUEUE_RESOLVED"}
    assert "QUEUE" in settings.envelope_debug_queue_timing
    return source_obj, payload


def _run_alpha_change_reuses_the_preparation(source_obj, payload):
    settings = _settings()
    controller = _controller()
    before = controller.build_counts["CONVEYOR_PREPARATION"]
    assert before > 0
    faces_before = [
        item
        for item in _queue_strokes(payload)
        if item["kind"] == "QUEUE_COVERAGE_FACE"
    ]

    settings.envelope_debug_alpha = 0.45
    assert controller.build_counts["CONVEYOR_PREPARATION"] == before
    assert "alpha redraw" in settings.envelope_debug_queue_timing

    updated = _sidecar_payload(source_obj)
    # Ядро хранит alpha точной дробью, а не десятичной записью, и сверять её с
    # литералом 9/20 НЕЛЬЗЯ — это измерено: `FloatProperty` одинарной точности,
    # поэтому 0.45 доезжает как 0.44999998807907104. Утверждение поэтому иное и
    # более сильное: в sidecar лежит РОВНО то число, которое стоит в панели, —
    # тем же `str(float(...))`, которым его берёт `build_envelope_decal_request`.
    assert float(
        Fraction(updated["queue"]["domains"][0]["alpha"])
    ) == float(settings.envelope_debug_alpha)
    faces_after = [
        item
        for item in _queue_strokes(updated)
        if item["kind"] == "QUEUE_COVERAGE_FACE"
    ]
    assert faces_after
    # Цвет владельца при перетаскивании не прыгает: слот стоит на spec_id.
    assert {
        (item["envelope_spec_id"], item["palette_slot"])
        for item in faces_after
    } == {
        (item["envelope_spec_id"], item["palette_slot"])
        for item in faces_before
    }
    # Слои эталонного пути перерисовка очереди не трогает.
    assert _layer_names(source_obj) == REQUIRED_LAYERS | QUEUE_LAYERS
    assert any(
        item.get("stage") == "TOPOLOGY" for item in updated["strokes"]
    )


def _profile_counter_values(name):
    profile = json.loads(
        bpy.data.texts[
            "CFTUV_EnvelopeProfile_EnvelopeTwoPatch.json"
        ].as_string()
    )
    return sorted(
        item["value"]
        for item in profile["counters"]
        if item["name"] == name
    )


def _run_repeat_press_hits_the_preparation_cache():
    """Вторая кнопка на том же выделении берёт подготовку из кэша сессии."""

    controller = _controller()
    before = controller.build_counts["CONVEYOR_PREPARATION"]
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )
    assert controller.build_counts["CONVEYOR_PREPARATION"] == before
    assert _profile_counter_values("CONVEYOR_PREPARATION_CACHE_HIT") == [1, 1]
    assert _profile_counter_values("CONVEYOR_PREPARATION_CACHE_MISS") == [0, 0]


def _visibility_toggle_hides_only_queue(source_obj):
    settings = _settings()
    settings.envelope_debug_show_queue = False
    layers = {
        _layer_name(layer): layer
        for layer in _gp_object(source_obj).data.layers
    }
    assert all(layers[name].hide for name in QUEUE_LAYERS)
    assert not layers["ENV_00_PATCH_DOMAIN"].hide
    settings.envelope_debug_show_queue = True
    assert not layers["ENV_90_QUEUE_SKELETON"].hide


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
    _run_legacy_default_creates_no_queue_layer()
    source_obj, payload = _run_queue_engine_paints_owners()
    _run_alpha_change_reuses_the_preparation(source_obj, payload)
    _run_repeat_press_hits_the_preparation_cache()
    _visibility_toggle_hides_only_queue(source_obj)
    print("ENVELOPE_QUEUE_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
