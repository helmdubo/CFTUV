"""Blender 4.5 background smoke: пул доменов QUEUE даёт тот же ответ.

Два утверждения, и оба стоят на числах, а не на виде картинки:

1. кнопка при `envelope_debug_workers = 2` строит ТОТ ЖЕ sidecar, слои и
   квитанции, что и при `0`, с точностью до секунд: воркеры пула (подпроцессы
   интерпретатора Blender) считают домен тем же `run_queue_domain`;
2. пул действительно работал, а не молча уступил последовательному пути:
   счётчики `ENVELOPE_DOMAIN_POOL_*` лежат в профиле кнопки, задач отправлено
   столько, сколько доменов, и ни одна не упала.

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_DOMAIN_POOL_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import json
from pathlib import Path
import re
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
    _build_two_patch_seam,
    _reset_scene,
    _sidecar_payload,
)


PROFILE_TEXT = "CFTUV_EnvelopeProfile_EnvelopeTwoPatch.json"
POOL_COUNTERS = (
    "ENVELOPE_DOMAIN_POOL_WORKERS",
    "ENVELOPE_DOMAIN_POOL_DISPATCHED",
    "ENVELOPE_DOMAIN_POOL_TASK_FALLBACK",
    "ENVELOPE_DOMAIN_POOL_UNAVAILABLE",
)
_MS = re.compile(r"\d+(?:\.\d+)? ms")


def _settings():
    return bpy.context.scene.hotspotuv_settings


def _timing_free(payload):
    """Sidecar без секунд.

    Записи доменов очереди несут их в четырёх полях, а сообщение квитанции —
    текстом (`prepare 9.8 ms`).
    """

    payload = json.loads(json.dumps(payload))
    for domain in payload["queue"]["domains"]:
        for key in (
            "prepare_seconds",
            "coverage_seconds",
            "contour_seconds",
            "timings",
        ):
            domain.pop(key)
    for receipt in payload["stage_receipts"]:
        receipt["message"] = _MS.sub("<ms>", receipt["message"])
    return payload


def _first_difference(left, right, path="payload"):
    """Первое расхождение двух JSON-деревьев: путь и обе стороны, а не `!=`."""

    if type(left) is not type(right):
        return f"{path}: {left!r} != {right!r}"
    if isinstance(left, dict):
        for key in sorted(set(left) | set(right)):
            if key not in left or key not in right:
                return f"{path}.{key}: present on one side only"
            found = _first_difference(left[key], right[key], f"{path}.{key}")
            if found:
                return found
        return None
    if isinstance(left, list):
        if len(left) != len(right):
            return f"{path}: length {len(left)} != {len(right)}"
        for index, (a, b) in enumerate(zip(left, right)):
            found = _first_difference(a, b, f"{path}[{index}]")
            if found:
                return found
        return None
    return None if left == right else f"{path}: {left!r} != {right!r}"


def _receipts(source_obj):
    gp_object = bpy.data.objects["CFTUV_DEBUG_Envelope_" + source_obj.name]
    return [
        {**item, "message": _MS.sub("<ms>", item["message"])}
        for item in json.loads(gp_object["stage_receipts"])
    ]


def _pool_counters():
    profile = json.loads(bpy.data.texts[PROFILE_TEXT].as_string())
    return {
        item["name"]: item["value"]
        for item in profile["counters"]
        if item["name"] in POOL_COUNTERS and item["patch_domain_id"] is None
    }


def _build(workers):
    _reset_scene()
    controller = bpy.context.window_manager._cftuv_envelope_debug_session
    if controller is not None:
        controller.clear()
    source_obj = _build_two_patch_seam()
    settings = _settings()
    settings.envelope_debug_engine = "QUEUE"
    settings.envelope_debug_alpha = 0.25
    settings.envelope_debug_workers = workers
    assert (
        bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
        == {"FINISHED"}
    )
    return (
        _timing_free(_sidecar_payload(source_obj)),
        _receipts(source_obj),
        _pool_counters(),
        settings.envelope_debug_queue_timing,
    )


def _main():
    import cftuv

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован установленной копией
        pass
    settings = _settings()
    from cftuv.envelope_domain_pool import (
        DEFAULT_POOL_WORKERS,
        resolve_python_executable,
    )

    assert settings.envelope_debug_workers == DEFAULT_POOL_WORKERS
    assert 1 <= DEFAULT_POOL_WORKERS <= 8

    # В Blender 2.92+ `sys.executable` — интерпретатор, иначе пул не стартует.
    print("pool interpreter:", resolve_python_executable())

    sequential_payload, sequential_receipts, sequential_counters, timing = (
        _build(0)
    )
    assert sequential_counters == {}, sequential_counters
    assert "pool" not in timing, timing

    pooled_payload, pooled_receipts, pooled_counters, pooled_timing = _build(2)

    domains = sequential_payload["queue"]["domains"]
    assert domains and all(
        item["preparation_outcome"] == "EXACT"
        and item["coverage_outcome"] == "EXACT"
        for item in domains
    ), domains
    difference = _first_difference(pooled_payload, sequential_payload)
    assert difference is None, difference
    assert pooled_receipts == sequential_receipts
    assert {item["stage"] for item in pooled_receipts} == {"QUEUE_RESOLVED"}

    assert set(pooled_counters) == set(POOL_COUNTERS), pooled_counters
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_WORKERS"] == 2, pooled_counters
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_DISPATCHED"] == len(domains)
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_TASK_FALLBACK"] == 0
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_UNAVAILABLE"] == 0
    assert "pool wall" in pooled_timing and "2 workers" in pooled_timing
    print("pooled timing:", pooled_timing)

    # Вернуть последовательный режим: воркеры остановлены, а не оставлены.
    from cftuv import envelope_domain_pool

    _settings().envelope_debug_workers = 0
    _build(0)
    assert envelope_domain_pool._POOL is None

    print("ENVELOPE_DOMAIN_POOL_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
