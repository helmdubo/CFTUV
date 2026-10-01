"""Blender 4.5 background smoke: пул доменов QUEUE даёт тот же ответ.

Два утверждения, и оба стоят на числах, а не на виде картинки:

1. кнопка при `envelope_debug_workers = 2` строит ТОТ ЖЕ sidecar, слои и
   квитанции, что и при `0`, с точностью до секунд: воркеры пула (подпроцессы
   интерпретатора Blender) считают домен тем же `run_queue_domain`;
2. пул действительно работал, а не молча уступил последовательному пути:
   счётчики `ENVELOPE_DOMAIN_POOL_*` лежат в профиле кнопки, задач отправлено
   столько, сколько доменов, и ни одна не упала;
3. ТЁПЛАЯ кнопка и ползунок alpha, чьё покрытие кэшированных подготовок идёт в
   воркерах (WARM-COVERAGE-PARALLEL), дают тот же sidecar, что и в родителе, а
   пул назван счётчиком `ENVELOPE_DOMAIN_POOL_COVERAGE_DISPATCHED` и хвостом
   строки панели. Порог малой партии на время проверки снят: двухпатчевый шов
   стоил бы пулу больше, чем самому покрытию;
4. настройка «Worker Python» (WORKER-PYTHON): свойство пристёгнуто к
   зарегистрированному классу предпочтений, а внешний CPython, если он есть,
   отвечает побитово так же; расхождение окружений названо и не выключает пул.

Прогон (без `--factory-startup`: sympy в 4.5 живёт в профиле пользователя):
blender --background --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_DOMAIN_POOL_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import json
import os
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
    "ENVELOPE_DOMAIN_POOL_COVERAGE_DISPATCHED",
    "ENVELOPE_DOMAIN_POOL_PYTHON_VERSION",
    "ENVELOPE_DOMAIN_POOL_EXTERNAL_PYTHON",
    "ENVELOPE_DOMAIN_POOL_INTERPRETER_FALLBACK",
    "ENVELOPE_DOMAIN_POOL_INTERPRETER_REASON",
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


def _warm_and_slide(workers):
    """Холодная кнопка, тёплая кнопка, два шага ползунка: ответ каждого шага."""

    from cftuv import envelope_queue_pool

    cold = _build(workers)
    source_obj = bpy.data.objects["EnvelopeTwoPatch"]
    settings = _settings()
    original = envelope_queue_pool.COVERAGE_POOL_MIN_BYTES
    envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = 0
    try:
        assert (
            bpy.ops.hotspotuv.build_exact_reference_envelope_debug()
            == {"FINISHED"}
        )
        warm = (
            _timing_free(_sidecar_payload(source_obj)),
            _pool_counters(),
            settings.envelope_debug_queue_timing,
        )
        slides = []
        for alpha in (0.4, 0.3):
            settings.envelope_debug_alpha = alpha
            slides.append(
                (
                    _timing_free(_sidecar_payload(source_obj)),
                    settings.envelope_debug_queue_timing,
                )
            )
    finally:
        envelope_queue_pool.COVERAGE_POOL_MIN_BYTES = original
    return cold, warm, slides


def _check_warm_and_slide(domains):
    sequential_cold, sequential_warm, sequential_slides = _warm_and_slide(0)
    assert sequential_warm[1] == {}, sequential_warm[1]
    assert "pool" not in sequential_warm[2], sequential_warm[2]
    for _, text in sequential_slides:
        assert "pool" not in text, text

    pooled_cold, pooled_warm, pooled_slides = _warm_and_slide(2)
    difference = _first_difference(pooled_warm[0], sequential_warm[0])
    assert difference is None, difference
    counters = pooled_warm[1]
    assert counters["ENVELOPE_DOMAIN_POOL_COVERAGE_DISPATCHED"] == domains
    assert counters["ENVELOPE_DOMAIN_POOL_DISPATCHED"] == domains
    assert counters["ENVELOPE_DOMAIN_POOL_WORKERS"] == 2, counters
    assert counters["ENVELOPE_DOMAIN_POOL_TASK_FALLBACK"] == 0
    assert counters["ENVELOPE_DOMAIN_POOL_UNAVAILABLE"] == 0
    assert "pool wall" in pooled_warm[2] and "2 workers" in pooled_warm[2]
    for (payload, text), (reference, _) in zip(pooled_slides, sequential_slides):
        difference = _first_difference(payload, reference)
        assert difference is None, difference
        assert "pool wall" in text and "2 workers" in text, text
    print("pooled warm timing:", pooled_warm[2])
    print("pooled slider timing:", pooled_slides[-1][1])


def _external_python():
    """Внешний CPython для проверки: переменная окружения либо стандартное место."""

    candidates = (os.environ.get("CFTUV_TEST_EXTERNAL_PYTHON", ""), "C:/Python313/python.exe")
    return next((item for item in candidates if item and Path(item).is_file()), "")


def _set_worker_python(value):
    """Предпочтение аддона «Worker Python» тем же путём, каким его выставит владелец."""

    from cftuv.envelope_worker_python import (
        install_worker_python_preference,
        read_worker_python,
    )

    install_worker_python_preference()  # `register` зовёт это сам; повтор ничего не делает
    addons = bpy.context.preferences.addons
    entry = addons.get("cftuv") or addons.new()
    entry.module = "cftuv"
    assert hasattr(entry.preferences, "worker_python"), "preference not attached"
    entry.preferences.worker_python = value
    assert read_worker_python() == value


def _check_external_python(reference_payload, domains):
    external = _external_python()
    if not external:
        print("external interpreter: skipped (no standalone CPython found)")
        return
    _set_worker_python(external)
    try:
        payload, _, counters, timing = _build(2)
    finally:
        _set_worker_python("")
    assert counters["ENVELOPE_DOMAIN_POOL_UNAVAILABLE"] == 0, counters
    assert counters["ENVELOPE_DOMAIN_POOL_WORKERS"] == 2, counters
    if counters["ENVELOPE_DOMAIN_POOL_INTERPRETER_FALLBACK"]:
        # Окружение внешнего не сошлось с Blender: исход назван, воркеры идут
        # на встроенном (а не на последовательном пути), и причина в панели.
        assert counters["ENVELOPE_DOMAIN_POOL_EXTERNAL_PYTHON"] == 0, counters
        assert counters["ENVELOPE_DOMAIN_POOL_INTERPRETER_REASON"] > 0, counters
        assert "external Python rejected" in timing and "(bundled)" in timing, timing
    else:
        assert counters["ENVELOPE_DOMAIN_POOL_EXTERNAL_PYTHON"] == 1, counters
        assert counters["ENVELOPE_DOMAIN_POOL_DISPATCHED"] == domains, counters
        assert "(external)" in timing, timing
        difference = _first_difference(payload, reference_payload)
        assert difference is None, difference
    print("external interpreter timing:", timing)


def _check_unusable_python():
    """Несуществующий путь: исход назван, а воркеры идут на встроенном Python."""

    _set_worker_python("C:/nowhere/cftuv/python.exe")
    try:
        _, _, counters, timing = _build(2)
        sidecar = json.dumps(_sidecar_payload(bpy.data.objects["EnvelopeTwoPatch"]))
    finally:
        _set_worker_python("")
    assert counters["ENVELOPE_DOMAIN_POOL_INTERPRETER_FALLBACK"] == 1, counters
    assert counters["ENVELOPE_DOMAIN_POOL_INTERPRETER_REASON"] == 1, counters
    assert counters["ENVELOPE_DOMAIN_POOL_EXTERNAL_PYTHON"] == 0, counters
    assert counters["ENVELOPE_DOMAIN_POOL_UNAVAILABLE"] == 0, counters
    assert counters["ENVELOPE_DOMAIN_POOL_WORKERS"] == 2, counters
    assert "path is not a usable Python executable" in timing, timing
    assert "(bundled)" in timing and "pool wall" in timing, timing
    assert "ENVELOPE_DOMAIN_POOL_INTERPRETER_UNUSABLE" in sidecar
    print("unusable interpreter timing:", timing)


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
    version = sys.version_info
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_EXTERNAL_PYTHON"] == 0
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_INTERPRETER_FALLBACK"] == 0
    assert pooled_counters["ENVELOPE_DOMAIN_POOL_PYTHON_VERSION"] == (
        version[0] * 10000 + version[1] * 100 + version[2]
    ), pooled_counters
    assert "(bundled)" in pooled_timing, pooled_timing
    print("pooled timing:", pooled_timing)

    _check_warm_and_slide(len(domains))
    _check_external_python(sequential_payload, len(domains))
    _check_unusable_python()

    # Вернуть последовательный режим: воркеры остановлены, а не оставлены.
    from cftuv import envelope_domain_pool

    _settings().envelope_debug_workers = 0
    _build(0)
    assert envelope_domain_pool._POOL is None

    print("ENVELOPE_DOMAIN_POOL_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
