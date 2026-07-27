"""Контракт движка QUEUE в хосте: палитра, числа, исходы и кэш подготовки.

Тесты работают БЕЗ Blender: движок отображает контракты ядра, и всё, что
проверяется здесь, проверяемо без него. То, что требует bpy (слои GP, панель,
ползунок), проверяется в `tests/blender/test_envelope_debug_queue.py`.
"""

from __future__ import annotations

from fractions import Fraction
import math
from pathlib import Path
import sys

import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = REPO_ROOT / "kernel" / "src"
KERNEL_TESTS = REPO_ROOT / "kernel" / "tests"
for path in (KERNEL_SRC, KERNEL_TESTS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv.envelope_queue_export import (  # noqa: E402
    QUEUE_OWNER_LAYERS,
    QUEUE_OWNER_SLOT_COUNT,
    QUEUE_SKELETON_LAYER,
    QUEUE_WALL_LAYER,
    build_queue_palette,
    build_queue_scene,
    queue_scene_payload,
    queue_timing_text,
    recompute_queue_coverage,
    run_queue_domain,
    sqrt_sum_float,
)


# Вход измерения — полевая фикстура `building_002_point_contact_v1`, та же,
# на которой ядро проверяет вход очереди. Синтетический `straight_snapshot`
# здесь не годится и это ИЗМЕРЕНО, а не предположено: его кадр —
# `PlanarPatchFrameV1`, у которого закон решётки не объявлен, и подготовка
# честно отвечает `CHART_LATTICE_IS_NOT_DECLARED`.
FIXTURE = (
    REPO_ROOT / "kernel" / "fixtures" / "building_002_point_contact_v1"
)


def _field_inputs():
    import cftuv_envelope as kernel

    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (FIXTURE / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (FIXTURE / "decal_request.json").read_bytes()
    )
    return snapshot, request


def _run(alpha: str, *, preparation_provider=None):
    snapshot, request = _field_inputs()
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id.value
    _prepared, domain = run_queue_domain(
        7,
        domain_id,
        snapshot,
        request,
        alpha,
        preparation_provider=preparation_provider,
    )
    return domain


def test_palette_is_deterministic_and_counts_its_own_overflow():
    palette, wrapped = build_queue_palette(["b", "a", "a", "c"])
    assert palette == (("a", 0), ("b", 1), ("c", 2))
    assert wrapped == 0

    names = [f"spec-{index:02d}" for index in range(11)]
    palette, wrapped = build_queue_palette(reversed(names))
    slots = dict(palette)
    assert [slots[name] for name in names] == [0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2]
    # Переполнение названо числом: три владельца получили чужой цвет.
    assert wrapped == 11 - QUEUE_OWNER_SLOT_COUNT


def test_sqrt_sum_number_comes_from_the_enclosure_not_from_parts():
    from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1

    value = SqrtSumV1.rational(Fraction(3, 4)) + SqrtSumV1.radical(2, 3)
    low, high = value.enclosure(64)
    number = sqrt_sum_float(value)
    # Оболочка на 64 битах уже, чем шаг double, поэтому границы сравниваются с
    # допуском самого double, а не с нулём: иначе тест мерил бы округление
    # печати, а не то, откуда взято число.
    assert float(low) - 1e-12 <= number <= float(high) + 1e-12
    assert number == pytest.approx(0.75 + 2.0 * math.sqrt(3.0), abs=1e-12)
    assert sqrt_sum_float(SqrtSumV1.zero()) == 0.0


def test_queue_domain_carries_owners_and_reaches_exact():
    domain = _run("0.25")

    assert domain.preparation_outcome == "EXACT"
    assert domain.coverage_outcome == "EXACT"
    # Те же числа, которыми ядро описывает эту фикстуру: 3 источника, 9 стен,
    # 3 грани. Числа взяты прогоном, а не из памяти.
    assert len(domain.law_names) == 3
    assert len(domain.faces) == 3
    assert domain.regions[0].wall_edge_count == 9
    assert domain.lattice_scale == 65536
    assert all(face.envelope_spec_id for face in domain.faces)
    assert all(len(face.points) >= 3 for face in domain.faces)
    assert len(
        [item for item in domain.segments if item.layer == QUEUE_WALL_LAYER]
    ) == 9
    assert any(
        item.layer == QUEUE_SKELETON_LAYER for item in domain.segments
    )
    # У полевой фикстуры скорости фронтов НЕ единичные, и мост говорит это
    # именем, а не молчанием.
    assert "NON_UNIT_SPEED_LAWS=3" in domain.regions[0].findings


def test_coverage_grows_with_alpha_and_geometry_does_not_move():
    small = _run("0.05")
    large = _run("0.5")

    small_area = sum(face.doubled_area for face in small.faces)
    large_area = sum(face.doubled_area for face in large.faces)
    assert small_area < large_area
    # Скелет alpha-независим: те же дуги и те же стены при обеих alpha.
    assert [item.points for item in small.segments] == [
        item.points for item in large.segments
    ]


def test_alpha_change_on_a_warm_preparation_does_not_prepare_again():
    calls = []

    def provider(_patch_id, _domain_id, _edges, snapshot, request):
        from cftuv_envelope.wavefront import prepare_conveyor

        if not calls:
            calls.append(prepare_conveyor(snapshot, request))
        return calls[0]

    first = _run("0.05", preparation_provider=provider)
    assert len(calls) == 1

    scene = recompute_queue_coverage(
        ((first.patch_id, first.patch_domain_id, first.preparation),),
        "0.5",
    )
    assert len(calls) == 1
    assert scene.domains[0].coverage_outcome == "EXACT"
    assert sum(
        face.doubled_area for face in scene.domains[0].faces
    ) > sum(face.doubled_area for face in first.faces)
    assert scene.domains[0].prepare_seconds == 0.0
    assert "prepare 0 ms" in queue_timing_text(scene)


def test_scene_payload_maps_slot_colour_spec_and_instance():
    scene = build_queue_scene((_run("0.25"),))
    payload = queue_scene_payload(scene)

    assert payload["engine"] == "QUEUE"
    assert payload["palette_wrapped"] == 0
    assert payload["owners"]
    for owner in payload["owners"]:
        assert owner["layer"] == QUEUE_OWNER_LAYERS[owner["palette_slot"]]
        assert len(owner["color"]) == 4
        assert owner["envelope_instance_id"]
        # Имя экземпляра зависит от alpha, имя спеки — нет; в цвет идёт спека.
        assert owner["envelope_spec_id"] != owner["envelope_instance_id"]
    assert payload["domains"][0]["coverage_outcome"] == "EXACT"
    assert payload["domains"][0]["regions"][0]["bridge_outcome"] == "EXACT"


def test_owner_colour_survives_an_alpha_change():
    first = build_queue_scene((_run("0.05"),))
    second = build_queue_scene((_run("0.5"),))

    assert first.palette == second.palette


def test_float_alpha_is_refused_by_name_at_the_kernel_boundary():
    snapshot, request = _field_inputs()
    domain_id = next(iter(snapshot.patch_domains)).patch_domain_id.value
    with pytest.raises(TypeError) as failure:
        run_queue_domain(1, domain_id, snapshot, request, 0.25)
    assert "float" in str(failure.value)
