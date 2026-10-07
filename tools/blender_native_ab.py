"""A/B нативного ядра против Python-эталона на полевых случаях: исход, дайджест батча, цена, секунды — по каждому домену.

    blender -b E:\\testscene.blend --python-exit-code 1 --python tools/blender_native_ab.py -- \\
        [--strict | --diagnostic] [--expect-build-id tree|none|<id>] \\
        [--cases mesh:alpha:density:stretch,...] [--steps 3] [--width-step 0.01] [--workers 0] \\
        [--native-path <каталог с cftuv_native>] [--root <дерево>] [--out <json>]

Каждый случай (по умолчанию 22 полевых — рецепт `field_rel.py`: меш, alpha, плотность веера, допуск растяжения) считается кнопкой
(`run_production`, настройка `Kernel backend` читается из свойства сцены, как у оператора) на ряде ширин `alpha + width_step * n`,
`n = 0..steps`, ДВАЖДЫ: бэкендом `PYTHON` и бэкендом `NATIVE`. Кэши сессии, память стадии резки и (при `--workers`) пул воркеров между
двумя прогонами сбрасываются, порядок чередуется от случая к случаю (процессные `lru_cache` ядра иначе грели бы второго). Домены
сравниваются попарно по номеру патча: исход, дайджест содержимого батча и семантический дайджест — это ОТВЕТ (расхождение — код
возврата 1); шесть статей `EXACT_WORK_*` — ЦЕНА (расхождение тоже код 1: нативное ядро обязано стоить столько же, сколько эталон);
секунды и кто на самом деле посчитал (запись бэкенда домена: `native` / `python` / `mixed`, названный откат) — в таблице и в отчёте.

РЕЖИМЫ. `--strict` (по умолчанию; приёмка Rust) отказывает КОДОМ 2, не сравнивая ничего «на нуле», если: расширения нет (`UNAVAILABLE`),
порт устарел (`stale(...)`) или не `available`, сборка не та (`--expect-build-id`: `tree` по умолчанию — id собранного из ЭТОГО дерева,
считает `tools/native_build_id.py`; `none` отключает сверку и записывается в отчёт; либо точный id), на каком-либо домене случился
откат, кроме названного разрешённым `NATIVE_NOT_REACHED`, у разрешённого `NATIVE_NOT_REACHED` нет честного объяснения (результат из кэша,
шаблон шага ширины `FAST_HIT` или отказ до ядра: иначе «домен прошёл, а Rust не вызван»), либо случай не позвал ни одной нативной
операции. Различие ответа или цены, как и раньше, — код 1 (сильнее отказа). `--diagnostic` — прежний режим без отказов приёмки: без расширения
статус `UNAVAILABLE` и код 0, устаревший порт сравнивается (откаты названы в таблице), сборку не сверяет (её id лежит в `native_status`). Последняя строка:
`NATIVE_AB_OK|NATIVE_AB_UNAVAILABLE|NATIVE_AB_FAILED|NATIVE_AB_REFUSED ...`; в отчёте `strict.violations` — именованные отказы.

ЧТО ИЗМЕРЯЕТСЯ. Это кнопка (`run_production` + упаковка массивов меша `build_mesh_arrays`), а не живое превью ширины. На каждый шаг ширины и
бэкенд пишутся ДВЕ метрики, которые при пуле процессов различны: сумма секунд доменов (`seconds` строки домена; `speedup`) и стена
взаимодействия (`wall_seconds` прогона плюс `pack_seconds`; `wall_speedup`); по стенам считаются p50/p95/p99/max (nearest-rank, число выборок
`n` рядом: при `n < 100` p99 равно max), отдельно по всем шагам и по тёплым (шаг ≥ 1: перетаскивание ширины после холодного первого). Далее: число
настоящих нативных и питоновых вызовов и доменов с откатом (`calls`), упаковка (`pack_seconds`, плюс `pool`: байты и разбор пикла пула) и доля
нативного счёта в меше по числу доменов и по площади упакованных граней (`share`). Запись настоящего меша в Blender (apply) НЕ измеряется и названа
`NOT_MEASURED`: она добавляла бы объекты в открытую сцену. Доля ТОЧНОГО меша живого превью (сертификат интервала ширины) — другой вопрос и
другой инструмент, здесь её нет.

Диспетчеры покрытия и резки подключены в самом ядре (`cut_domain` зовёт `backend.clip_compute`), поэтому заказ `NATIVE` ничего не ставит; домен, который не
позвал ни одной нативной операции (резка из памяти стадии, покрытие из шаблона шага ширины), назван `NATIVE_NOT_REACHED`. Идентичность нативной сборки
(`native_build_id`) лежит в `native_status` отчёта.

Blender только headless, без `--factory-startup`, без сохранения .blend. Аддон берётся из дерева репозитория (установленный снимается).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
#: 22 полевых случая `mesh:alpha:density:stretch` (рецепт `field_rel.py`).
FIELD_CASES = (
    "2:0.25:2:20,2.001:0.25:2:20,building:0.25:2:20,building.001:0.25:2:20,building.002:0.25:2:20,building.003:0.25:2:20,"
    "building.004:0.25:2:20,CFTUV_D_PERIODIC_CYLINDER:0.25:2:20,half_sphere:0.25:2:20,rounded_wall.001:0.25:2:20,"
    "rounded_wall_noise_top:0.25:2:20,sagging_wall:0.25:2:20,wall_noise_top:0.25:2:20,walls.001:0.25:2:20,walls.002:0.25:2:20,"
    "walls.003:0.25:2:20,walls.006:0.25:2:20,sagging_wall:0.987:2:42,rounded_wall_noise_top:0.5:2:42,"
    "rounded_wall_noise_top:0.2239:2:42,sagging_wall:0.2239:2:42,building:0.2239:2:42"
)
PRICE_PREFIX = "EXACT_WORK_"
STATUS_OK, STATUS_UNAVAILABLE, STATUS_FAILED, STATUS_REFUSED = "OK", "UNAVAILABLE", "FAILED", "REFUSED"
#: Коды возврата: различие ответа или цены (и упавший случай) сильнее отказа приёмки.
EXIT_OK, EXIT_FAILED, EXIT_REFUSED = 0, 1, 2
#: Единственный откат, который строгий режим допускает: заказан нативный бэкенд, а домен не позвал ни одной операции (`BackendOutcomeV1.NATIVE_NOT_REACHED`).
#: Допущен только с объяснением (`unexplained_not_reached`), иначе это молчаливое отсутствие Rust.
ALLOWED_FALLBACKS = frozenset({"NATIVE_NOT_REACHED"})
#: `cftuv_envelope.materialize.step.PATH_FAST` и `cftuv.envelope_production_export.MATERIALIZED` (равенство держит тест).
STEP_FAST = "FAST_HIT"
MATERIALIZED = "MATERIALIZED"
PLACEMENT_CACHED = "cache"
#: Классы домена в нативном прогоне: откуда взят его ответ.
SHARE_CLASSES = ("native", "python", "not_reached", "cache", "refused")
#: Названное отсутствие измерения (молчаливо не пропадает).
NOT_MEASURED = {
    "apply_seconds": "NOT_MEASURED: запись меша в Blender (`write_decal_object`) добавила бы объекты в открытую сцену; измерены прогон и упаковка массивов"
}


# --------------------------------------------------------------------------
# Чистая часть: строка домена, сравнение, сводка, таблица (без Blender, под тестом)
# --------------------------------------------------------------------------


def domain_row(result) -> dict:
    """Строка домена: ответ (исход, дайджесты), цена (`EXACT_WORK_*`), секунды и запись бэкенда."""

    counters = dict(result.counters)
    record = getattr(result, "backend_record", None)
    batch = result.batch
    return {
        "patch_id": int(result.patch_id),
        "outcome": str(result.outcome),
        "content_digest": result.content_digest,
        "semantic_digest": "" if batch is None else batch.semantic_digest.value,
        "prices": {name: value for name, value in sorted(counters.items()) if name.startswith(PRICE_PREFIX)},
        "seconds": float(result.seconds),
        "placement": str(result.placement),
        "step_path": str(result.step_path),
        "ran": "python" if record is None else record.ran,
        "native_calls": 0 if record is None else record.native_calls,
        "python_calls": 0 if record is None else record.python_calls,
        "fallbacks": [] if record is None else list(record.outcomes),
        "area": None,
    }


def compare_domains(python_rows, native_rows) -> dict:
    """`{answer: [...], price: [...], missing: [...]}`: различия двух прогонов по номеру патча (пусто — побитово то же)."""

    first = {item["patch_id"]: item for item in python_rows}
    second = {item["patch_id"]: item for item in native_rows}
    found: dict = {"answer": [], "price": [], "missing": []}
    for patch in sorted(first.keys() | second.keys()):
        if patch not in first or patch not in second:
            found["missing"].append(f"patch {patch}: only in {'python' if patch in first else 'native'}")
            continue
        one, other = first[patch], second[patch]
        for name in ("outcome", "content_digest", "semantic_digest"):
            if one[name] != other[name]:
                found["answer"].append(f"patch {patch}: {name} {one[name]!r} != {other[name]!r}")
        if one["prices"] != other["prices"]:
            names = sorted(name for name in one["prices"].keys() | other["prices"].keys() if one["prices"].get(name) != other["prices"].get(name))
            found["price"].append(f"patch {patch}: {', '.join(names)}")
    return found


def classify_row(row: dict) -> str:
    """Откуда ответ домена в нативном прогоне: `cache` (результат прежнего прогона), `refused` (в мешe его нет), `native`, `python` либо `not_reached`."""

    if row["placement"] == PLACEMENT_CACHED:
        return "cache"
    if row["outcome"] != MATERIALIZED:
        return "refused"
    if row["ran"] in ("native", "mixed"):
        return "native"
    if row["python_calls"] or any(name not in ALLOWED_FALLBACKS for name in row["fallbacks"]):
        return "python"
    return "not_reached"


def unexplained_not_reached(rows) -> list:
    """Номера патчей, которые не позвали ни одной операции, хотя их ответ не из кэша и не из шаблона шага ширины (`FAST_HIT`) и не отказ до ядра."""

    return sorted(row["patch_id"] for row in rows if classify_row(row) == "not_reached" and row["step_path"] != STEP_FAST)


def share_of(rows) -> dict:
    """Состав меша по классам: число доменов и площадь упакованных граней (`area` строки; нет площади — домен назван в `area_unknown_domains`)."""

    domains = {name: 0 for name in SHARE_CLASSES}
    area = {name: 0.0 for name in SHARE_CLASSES}
    unknown = 0
    for row in rows:
        kind = classify_row(row)
        domains[kind] += 1
        if row.get("area") is not None:
            area[kind] += float(row["area"])
        elif kind != "refused":
            unknown += 1
    in_mesh = [name for name in SHARE_CLASSES if name != "refused"]
    return {
        "domains": domains,
        "area": {name: round(value, 6) for name, value in area.items()},
        "area_unknown_domains": unknown,
        "mesh_domains": sum(domains[name] for name in in_mesh),
        "mesh_area": round(sum(area[name] for name in in_mesh), 6),
    }


def _seconds(value):
    return None if value is None else round(float(value), 3)


def _interaction(run) -> float | None:
    """Стена взаимодействия шага: прогон кнопки плюс упаковка массивов меша (нет любого слагаемого — нет и суммы)."""

    if not run or run.get("wall_seconds") is None or run.get("pack_seconds") is None:
        return None
    return float(run["wall_seconds"]) + float(run["pack_seconds"])


def summarize_width(width, python_rows, native_rows, python_run=None, native_run=None) -> dict:
    """Одна ширина случая: различия, секунды доменов и стена двух бэкендов, кто посчитал в нативном прогоне, откаты, вызовы, состав меша.

    `python_run`/`native_run` — `{"wall_seconds", "pack_seconds", "pool"}` прогона (нет — метрики стены названы `None`, остальное считается по строкам).
    """

    computed = [item for item in native_rows if item["placement"] != PLACEMENT_CACHED]
    python_seconds = sum(item["seconds"] for item in python_rows if item["placement"] != PLACEMENT_CACHED)
    native_seconds = sum(item["seconds"] for item in computed)
    ran = {"native": 0, "python": 0, "mixed": 0}
    fallbacks: dict = {}
    for item in computed:
        ran[item["ran"]] += 1
        for name in item["fallbacks"]:
            fallbacks.setdefault(name, []).append(item["patch_id"])
    differences = compare_domains(python_rows, native_rows)
    python_wall, native_wall = _interaction(python_run), _interaction(native_run)
    return {
        "width": width,
        "domains": len(native_rows),
        "differences": differences,
        "python_seconds": round(python_seconds, 3),
        "native_seconds": round(native_seconds, 3),
        "speedup": round(python_seconds / native_seconds, 2) if native_seconds else None,
        "python_interaction": _seconds(python_wall),
        "native_interaction": _seconds(native_wall),
        "wall_speedup": round(python_wall / native_wall, 2) if python_wall is not None and native_wall else None,
        "python_run": dict(python_run or {}),
        "native_run": dict(native_run or {}),
        "ran": ran,
        "calls": {
            "native": sum(item["native_calls"] for item in computed),
            "python": sum(item["python_calls"] for item in computed),
            "fallback_domains": sum(1 for item in computed if item["fallbacks"]),
        },
        "fallbacks": {name: sorted(set(found)) for name, found in sorted(fallbacks.items())},
        "unexplained_not_reached": unexplained_not_reached(computed),
        "share": share_of(native_rows),
    }


def has_differences(summary: dict) -> bool:
    return any(summary["differences"][kind] for kind in ("answer", "price", "missing"))


def _fallback_text(fallbacks: dict) -> str:
    return "; ".join(f"{name}: {len(patches)}" for name, patches in fallbacks.items())


def _cell(value, template: str = "{:.2f}") -> str:
    return "-" if value is None else template.format(value)


def format_table(report: dict) -> str:
    """Таблица случаев и ширин: различия ответа и цены, секунды доменов и стена взаимодействия, исполнитель и откаты."""

    header = ("case", "width", "doms", "ans", "price", "py s", "nat s", "x", "py wall", "nat wall", "wall x", "native/python/mixed", "fallbacks")
    rows = [header]
    for case in report.get("cases", ()):
        if "failure" in case:
            rows.append((case["case"], *("-",) * (len(header) - 2), "FAILED: " + case["failure"].strip().splitlines()[-1][:60]))
            continue
        for item in case["widths"]:
            differences = item["differences"]
            ran = item["ran"]
            rows.append(
                (
                    case["case"],
                    f"{item['width']:g}",
                    str(item["domains"]),
                    str(len(differences["answer"]) + len(differences["missing"])),
                    str(len(differences["price"])),
                    f"{item['python_seconds']:.2f}",
                    f"{item['native_seconds']:.2f}",
                    _cell(item["speedup"]),
                    _cell(item.get("python_interaction")),
                    _cell(item.get("native_interaction")),
                    _cell(item.get("wall_speedup")),
                    f"{ran['native']}/{ran['python']}/{ran['mixed']}",
                    _fallback_text(item["fallbacks"]),
                )
            )
    widths = [max(len(row[column]) for row in rows) for column in range(len(header))]
    lines = ["  ".join(cell.ljust(widths[column]) for column, cell in enumerate(row)).rstrip() for row in rows]
    lines.insert(1, "  ".join("-" * width for width in widths))
    return "\n".join(lines)


# --------------------------------------------------------------------------
# Чистая часть: задержка взаимодействия, площадь, итоги
# --------------------------------------------------------------------------


def percentile(values, fraction: float):
    """Nearest-rank: наименьшее значение, не меньше `fraction` выборки (пусто — `None`); при `n < 100` p99 совпадает с max."""

    ordered = sorted(values)
    if not ordered:
        return None
    return ordered[max(1, math.ceil(fraction * len(ordered))) - 1]


def latency_summary(values) -> dict:
    """`{n, p50, p95, p99, max}` секунд (пустая выборка: `n = 0`, остальное `None`)."""

    values = [float(item) for item in values]
    return {
        "n": len(values),
        "p50": _seconds(percentile(values, 0.5)),
        "p95": _seconds(percentile(values, 0.95)),
        "p99": _seconds(percentile(values, 0.99)),
        "max": _seconds(max(values)) if values else None,
    }


def latency_of(cases) -> dict:
    """Стена взаимодействия по шагам ширины: `{бэкенд: {all: ..., warm: ...}}`; `warm` — шаги после первого холодного (перетаскивание ширины)."""

    found: dict = {backend: {"all": [], "warm": []} for backend in ("python", "native")}
    for case in cases:
        for item in case.get("widths", ()):
            for backend in found:
                value = item.get(f"{backend}_interaction")
                if value is None:
                    continue
                found[backend]["all"].append(value)
                if item.get("step", 0) >= 1:
                    found[backend]["warm"].append(value)
    return {backend: {kind: latency_summary(values) for kind, values in groups.items()} for backend, groups in found.items()}


def format_latency(latency: dict) -> list:
    """Строки `LATENCY <бэкенд> <all|warm> n=.. p50=.. p95=.. p99=.. max=..` (секунды стены взаимодействия)."""

    lines = []
    for backend, groups in latency.items():
        for kind, item in groups.items():
            numbers = " ".join(f"{name}={_cell(item[name], '{:.3f}')}" for name in ("p50", "p95", "p99", "max"))
            lines.append(f"LATENCY {backend} {kind} n={item['n']} {numbers}")
    return lines


def polygon_area(positions, loop) -> float:
    """Площадь плоской (по Ньюэллу: и почти плоской) грани `loop` по позициям меша."""

    sx = sy = sz = 0.0
    for index in range(len(loop)):
        ax, ay, az = positions[loop[index]]
        bx, by, bz = positions[loop[(index + 1) % len(loop)]]
        sx += ay * bz - az * by
        sy += az * bx - ax * bz
        sz += ax * by - ay * bx
    return 0.5 * math.sqrt(sx * sx + sy * sy + sz * sz)


def area_by_domain(arrays) -> dict:
    """`{номер патча: площадь его граней в упакованном меше}` по `MeshArraysV1` (`positions`, `faces`, `face_domain`)."""

    areas: dict = {}
    for loop, patch in zip(arrays.faces, arrays.face_domain):
        areas[patch] = areas.get(patch, 0.0) + polygon_area(arrays.positions, loop)
    return areas


def unavailable_report(status: dict, root: str, cases) -> dict:
    """Отчёт без сравнения: нативного ядра нет (названный статус), прогонять нечего."""

    return {"status": STATUS_UNAVAILABLE, "native_status": status, "root": root, "cases": [], "planned": list(cases)}


def totals_of(cases) -> dict:
    totals = {
        "cases": 0,
        "domains": 0,
        "answer": 0,
        "price": 0,
        "python_seconds": 0.0,
        "native_seconds": 0.0,
        "native": 0,
        "python": 0,
        "mixed": 0,
        "native_calls": 0,
        "python_calls": 0,
        "fallback_domains": 0,
        "python_interaction": 0.0,
        "native_interaction": 0.0,
        "mesh_domains": 0,
        "native_domains_in_mesh": 0,
        "mesh_area": 0.0,
        "native_area": 0.0,
        "area_unknown_domains": 0,
    }
    for case in cases:
        totals["cases"] += 1
        for item in case.get("widths", ()):
            differences = item["differences"]
            totals["domains"] += item["domains"]
            totals["answer"] += len(differences["answer"]) + len(differences["missing"])
            totals["price"] += len(differences["price"])
            totals["python_seconds"] += item["python_seconds"]
            totals["native_seconds"] += item["native_seconds"]
            for name in ("native", "python", "mixed"):
                totals[name] += item["ran"][name]
            for name, value in item.get("calls", {}).items():
                totals[name if name == "fallback_domains" else f"{name}_calls"] += value
            for backend in ("python", "native"):
                totals[f"{backend}_interaction"] += item.get(f"{backend}_interaction") or 0.0
            share = item.get("share")
            if share:
                totals["mesh_domains"] += share["mesh_domains"]
                totals["native_domains_in_mesh"] += share["domains"]["native"]
                totals["mesh_area"] += share["mesh_area"]
                totals["native_area"] += share["area"]["native"]
                totals["area_unknown_domains"] += share["area_unknown_domains"]
    return totals


def native_share(totals: dict) -> dict:
    """Доля меша, ответ которого посчитало нативное ядро: по числу доменов и по площади (`None`, если знаменателя нет)."""

    return {
        "domains": round(totals["native_domains_in_mesh"] / totals["mesh_domains"], 4) if totals["mesh_domains"] else None,
        "area": round(totals["native_area"] / totals["mesh_area"], 4) if totals["mesh_area"] else None,
        "area_unknown_domains": totals["area_unknown_domains"],
    }


# --------------------------------------------------------------------------
# Чистая часть: строгий режим (приёмка)
# --------------------------------------------------------------------------


def _port_state_name(state: str) -> str:
    if state == "unavailable":
        return "STRICT_PORT_UNAVAILABLE"
    if state.startswith("stale"):
        return "STRICT_PORT_STALE"
    return "STRICT_PORT_NOT_AVAILABLE"


def port_violations(status: dict, expected_build_id) -> list:
    """Отказы, видные до первого случая: порт не `available` (нет расширения, устарел, чужой интерпретатор) и сборка не та (если ждали конкретную)."""

    found = []
    for operation in ("coverage", "clip"):
        state = str(status.get(operation, "unavailable"))
        if state != "available":
            detail = f" ({status['detail']})" if status.get("detail") else ""
            found.append(f"{_port_state_name(state)}: {operation}={state}{detail}")
    if expected_build_id:
        actual = str(status.get("build_id", "")).lower()
        if actual != expected_build_id.lower():
            found.append(f"STRICT_BUILD_ID_MISMATCH: expected {expected_build_id}, loaded {actual or '<none>'}")
    return found


def width_violations(case: str, item: dict) -> list:
    """Отказы одной ширины: откат кроме разрешённого, `NATIVE_NOT_REACHED` без объяснения."""

    found = []
    for name, patches in item["fallbacks"].items():
        if name not in ALLOWED_FALLBACKS:
            found.append(f"STRICT_UNEXPECTED_FALLBACK: {name} case {case} width {item['width']:g} patches {patches[:8]}")
    if item.get("unexplained_not_reached"):
        found.append(
            f"STRICT_NOT_REACHED_UNEXPLAINED: case {case} width {item['width']:g} patches {item['unexplained_not_reached'][:8]} "
            "called no operation and are neither cached, nor from the width-step template, nor refused"
        )
    return found


def case_violations(case: dict) -> list:
    """Отказы случая: откаты по ширинам и случай, ни разу не позвавший нативную операцию (ничего не проверено)."""

    found = []
    for item in case.get("widths", ()):
        found.extend(width_violations(case["case"], item))
    widths = case.get("widths", ())
    if widths and not any(item["calls"]["native"] for item in widths):
        found.append(f"STRICT_CASE_NEVER_NATIVE: case {case['case']} made no native call at any width: nothing about the port was checked")
    return found


def strict_violations(report: dict, expected_build_id, expectation_error: str | None = None) -> list:
    """Все именованные отказы приёмки по отчёту (пусто — строгий режим пройден; различия ответа и цены в него не входят: у них код 1)."""

    found = port_violations(report["native_status"], expected_build_id)
    if expectation_error:
        found.append(expectation_error)
    ran = report["status"] != STATUS_UNAVAILABLE and "skipped" not in report
    if ran and not report["cases"]:
        found.append("STRICT_NO_CASES: no case was run")
    for case in report["cases"]:
        found.extend(case_violations(case))
    return found


def finalize(report: dict, *, strict: bool, expected_build_id=None, expectation_error: str | None = None) -> dict:
    """Статус отчёта и блок `strict`: `FAILED` (ответ, цена, упавший случай) сильнее `REFUSED` (отказ приёмки), `UNAVAILABLE` остаётся только диагностике."""

    violations = strict_violations(report, expected_build_id, expectation_error) if strict else []
    totals = report.get("totals") or {}
    failed = any("failure" in case for case in report["cases"]) or bool(totals.get("answer")) or bool(totals.get("price"))
    if failed:
        report["status"] = STATUS_FAILED
    elif violations:
        report["status"] = STATUS_REFUSED
    report["strict"] = {
        "enabled": strict,
        "expected_build_id": expected_build_id,
        "build_id_checked": bool(expected_build_id),
        "allowed_fallbacks": sorted(ALLOWED_FALLBACKS),
        "violations": violations,
    }
    return report


def exit_code(report: dict) -> int:
    """`FAILED` — 1, `REFUSED` — 2, иначе 0 (`OK`, диагностическое `UNAVAILABLE`)."""

    return {STATUS_FAILED: EXIT_FAILED, STATUS_REFUSED: EXIT_REFUSED}.get(report["status"], EXIT_OK)


def resolve_expected_build_id(spec, strict: bool, root: Path, tree_id=None) -> tuple:
    """`(id | None, ошибка | None)`: `tree` — id сборки ЭТОГО дерева, `none` — не сверять, иначе точный id; по умолчанию `tree` строго, `none` в диагностике."""

    chosen = spec if spec is not None else ("tree" if strict else "none")
    if chosen == "none":
        return None, None
    if chosen != "tree":
        return chosen.strip().lower(), None
    try:
        return (tree_id or _tree_build_id)(root), None
    except Exception as exc:  # noqa: BLE001 - не смогли посчитать ожидание: названный отказ, а не молчаливое «не сверяли»
        return None, f"STRICT_BUILD_ID_EXPECTATION_UNKNOWN: cannot compute the id of {Path(root) / 'native'}: {type(exc).__name__}: {exc}"


def _tree_build_id(root: Path) -> str:
    import importlib.util

    path = Path(root) / "tools" / "native_build_id.py"
    spec = importlib.util.spec_from_file_location("cftuv_native_build_id_tool", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.tree_parts(Path(root) / "native")["id"]


def final_line(report: dict) -> str:
    status = report["status"]
    strict = report.get("strict")
    tail = "" if not strict else f" strict={'on' if strict['enabled'] else 'off'} violations={len(strict['violations'])}" + (
        f" first={strict['violations'][0].split(':')[0]}" if strict["violations"] else ""
    )
    if "totals" not in report:
        native = report["native_status"]
        return f"NATIVE_AB_{status} coverage={native['coverage']} clip={native['clip']} detail={native['detail']!r}{tail}"
    totals = report["totals"]
    interaction = ""
    p95 = {backend: report.get("latency", {}).get(backend, {}).get("warm", {}).get("p95") for backend in ("python", "native")}
    if any(value is not None for value in p95.values()):
        interaction = f" warm_p95_python={_cell(p95['python'], '{:.3f}')} warm_p95_native={_cell(p95['native'], '{:.3f}')}"
    return (
        f"NATIVE_AB_{status} cases={totals['cases']} domains={totals['domains']} answer_differences={totals['answer']} "
        f"price_differences={totals['price']} python_seconds={totals['python_seconds']:.1f} native_seconds={totals['native_seconds']:.1f} "
        f"native_domains={totals['native']} python_domains={totals['python']} mixed_domains={totals['mixed']} "
        f"native_calls={totals['native_calls']} python_calls={totals['python_calls']} fallback_domains={totals['fallback_domains']}"
        f"{interaction}{tail}"
    )


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "A/B нативного ядра против Python-эталона. По умолчанию СТРОГО (приёмка): нет расширения, устаревший порт, не та сборка, "
            "неразрешённый откат или нативное ядро ни разу не вызвано — код 2; различие ответа или цены — код 1; чисто — 0. "
            "--diagnostic: прежнее поведение (без расширения статус UNAVAILABLE и код 0)."
        ),
        epilog="Коды возврата: 0 OK/диагностический UNAVAILABLE, 1 различие ответа или цены либо упавший случай, 2 отказ строгого режима.",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--strict", dest="strict", action="store_true", default=True, help="приёмка (по умолчанию): отказ кодом 2 по именованным причинам")
    mode.add_argument("--diagnostic", dest="strict", action="store_false", help="диагностика: без отказов приёмки, как раньше")
    parser.add_argument(
        "--expect-build-id",
        default=None,
        help="`tree` (по умолчанию строго): id сборки из дерева --root (tools/native_build_id.py); `none` (по умолчанию в диагностике): не сверять; либо точный id",
    )
    parser.add_argument("--root", default=str(ROOT))
    parser.add_argument("--cases", default=FIELD_CASES)
    parser.add_argument("--steps", type=int, default=3)
    parser.add_argument("--width-step", type=float, default=0.01)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument("--native-path", default="")
    parser.add_argument("--out", default="")
    return parser.parse_args(argv)


# --------------------------------------------------------------------------
# Blender: дерево, меш, прогон кнопки
# --------------------------------------------------------------------------


def _arguments():
    return parse_arguments(sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else [])


def path_with_tree(current, root: Path, native_path: str = "") -> list:
    """`sys.path` после загрузки дерева: каталог нативного порта (если назван) ВПЕРЕДИ всего, затем хост и ядро дерева, затем прежнее.

    Каталог несёт только пакет `cftuv_native`, поэтому деревья `cftuv` и `cftuv_envelope` он не затеняет, а установленное колесо
    `cftuv_native` (site-packages) не затеняет заказанный каталог: раньше каталог дописывался в КОНЕЦ, и колесо побеждало.
    """

    named = [str(native_path)] if native_path else []
    tree = [str(root), str(root / "kernel" / "src")]
    return [*named, *tree, *(item for item in current if item not in (*named, *tree))]


def _load_tree(root: Path, native_path: str) -> None:
    installed = sys.modules.get("cftuv")
    if installed is not None:
        try:
            installed.unregister()
        except Exception as exc:  # noqa: BLE001 - диагностика окружения, не причина отказа
            print("installed unregister failed:", type(exc).__name__, exc)
    for name in tuple(sys.modules):
        if name in {"cftuv", "cftuv_envelope"} or name.startswith(("cftuv.", "cftuv_envelope.")):
            del sys.modules[name]
    sys.path[:] = path_with_tree(sys.path, root, native_path)
    import cftuv
    import cftuv_envelope

    assert Path(cftuv.__file__).resolve().parent == (root / "cftuv").resolve()
    assert Path(cftuv_envelope.__file__).resolve().parent == (root / "kernel" / "src" / "cftuv_envelope").resolve()
    cftuv.register()


def _select_seams(obj) -> list:
    import bmesh
    import bpy

    bpy.context.view_layer.objects.active = obj
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for other in bpy.context.selected_objects:
        other.select_set(False)
    obj.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, True, False)
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    chosen = []
    for edge in bm.edges:
        edge.select_set(bool(edge.seam))
        if edge.seam:
            chosen.append(edge.index)
    bmesh.update_edit_mesh(obj.data)
    return chosen


def _fresh_state(controller, workers: int) -> None:
    """Кэши сессии, память стадии резки и пул воркеров сброшены: прогон одного бэкенда не читает память другого."""

    from cftuv.envelope_domain_pool import shutdown_domain_pool
    from cftuv_envelope.materialize.clip_memo import MEMO

    controller.clear()
    MEMO.clear()
    MEMO.reset_stats()
    if workers:
        shutdown_domain_pool()


def _pool_numbers(run) -> dict:
    """Числа пула прогона: байты туда и обратно и разбор пикла в родителе (секунды); `None` — прогон их не записал (воркеров нет)."""

    from cftuv import envelope_production_export as export

    unpickle = run.counter(export.PRODUCTION_POOL_UNPICKLE_WALL_US)
    return {
        "bytes_sent": run.counter(export.PRODUCTION_POOL_BYTES_SENT),
        "bytes_received": run.counter(export.PRODUCTION_POOL_BYTES_RECEIVED),
        "unpickle_wall_seconds": None if unpickle is None else round(unpickle / 1e6, 4),
    }


def _pack(run) -> tuple:
    """`(секунды упаковки, {патч: площадь}, ошибка | None)`: тот же `build_mesh_arrays`, что зовёт кнопка, на результатах прогона."""

    from cftuv.envelope_production_mesh import DEFAULT_DECAL_OFFSET, build_mesh_arrays

    started = time.perf_counter()
    try:
        arrays = build_mesh_arrays(run.results, DEFAULT_DECAL_OFFSET)
    except Exception as exc:  # noqa: BLE001 - упаковка не должна ронять сравнение: причина названа в отчёте
        return time.perf_counter() - started, {}, f"PACK_FAILED: {type(exc).__name__}: {exc}"
    seconds = time.perf_counter() - started
    return seconds, area_by_domain(arrays), None


def _run_backend(controller, spec: str, backend: str, args) -> list:
    """Шаги ширины случая под заказанным бэкендом: `[{width, rows, wall_seconds, pack_seconds, pool, ...}, ...]`."""

    import bmesh
    import bpy

    from cftuv.analysis import build_analysis_bundle
    from cftuv.analysis_surface import source_revision_from_bmesh
    from cftuv.envelope_kernel_backend import kernel_backend_of
    from cftuv.envelope_production_export import run_production
    from cftuv.envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

    mesh_name, alpha_text, density, stretch = spec.split(":")
    settings = bpy.context.scene.hotspotuv_settings
    mesh_settings = bpy.context.scene.hotspotuv_decal_mesh
    mesh_settings.kernel_backend = backend  # тем же путём, каким его выставит панель
    assert kernel_backend_of(mesh_settings) == backend
    _fresh_state(controller, args.workers)
    obj = bpy.data.objects[mesh_name]
    selected = _select_seams(obj)
    source_bm = bmesh.from_edit_mesh(obj.data)
    source_bm.faces.ensure_lookup_table()
    face_indices = tuple(face.index for face in source_bm.faces)
    revision = source_revision_from_bmesh(source_bm, obj, face_indices)
    object_key, data_key = int(obj.as_pointer()), int(obj.data.as_pointer())
    bundle = controller.get_analysis_bundle(
        object_key, data_key, revision, lambda: build_analysis_bundle(source_bm, face_indices, obj)
    )
    slide = envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance)
    widths = []
    for number in range(args.steps + 1):
        width = round(float(alpha_text) + args.width_step * number, 6)
        started = time.perf_counter()
        run = run_production(
            controller,
            bundle,
            frozenset(selected),
            width,
            source_object_key=object_key,
            source_data_key=data_key,
            density=density,
            developable_stretch_budget=envelope_stretch_budget(int(stretch)),
            silhouette_uv_slide=slide,
            workers=args.workers,
            kernel_backend=mesh_settings.kernel_backend,
        )
        wall = time.perf_counter() - started
        assert run.kernel_backend == backend
        pack_seconds, areas, pack_error = _pack(run)  # до строк: упаковка читает результаты так же, как кнопка (отложенные разворачиваются в ней)
        rows = [domain_row(item) for item in run.results]
        for row in rows:
            row["area"] = areas.get(row["patch_id"])
        widths.append(
            {
                "width": width,
                "rows": rows,
                "wall_seconds": wall,
                "pack_seconds": pack_seconds,
                "pack_error": pack_error,
                "production_wall_seconds": float(run.wall_seconds),
                "pool": _pool_numbers(run),
            }
        )
    if bpy.context.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    return widths


def _run_numbers(item: dict) -> dict:
    return {key: value for key, value in item.items() if key not in ("width", "rows")}


def _case(controller, spec: str, order: int, args) -> dict:
    """Один случай: оба бэкенда (порядок чередуется), сравнение по ширинам."""

    sequence = ("PYTHON", "NATIVE") if order % 2 == 0 else ("NATIVE", "PYTHON")
    runs: dict = {}
    seconds: dict = {}
    for backend in sequence:
        started = time.perf_counter()
        runs[backend] = _run_backend(controller, spec, backend, args)
        seconds[backend] = round(time.perf_counter() - started, 2)
    python, native = runs["PYTHON"], runs["NATIVE"]
    assert [item["width"] for item in python] == [item["width"] for item in native]
    widths = []
    for index, (one, other) in enumerate(zip(python, native)):
        summary = summarize_width(one["width"], one["rows"], other["rows"], _run_numbers(one), _run_numbers(other))
        summary["step"] = index
        widths.append(summary)
    return {"case": spec, "order": list(sequence), "wall_seconds": seconds, "widths": widths}


def _controller(bpy):
    from cftuv.envelope_debug_session import WINDOW_MANAGER_SESSION_ATTRIBUTE, EnvelopeDebugSessionController

    controller = getattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(bpy.context.window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    return controller


def _run_cases(cases, status, root: Path, args) -> dict:
    import bpy

    controller = _controller(bpy)
    results = []
    for order, spec in enumerate(cases):
        try:
            case = _case(controller, spec, order, args)
        except Exception:  # noqa: BLE001 - причина идёт в отчёт и в код возврата
            case = {"case": spec, "failure": traceback.format_exc()}
            print(case["failure"], flush=True)
        results.append(case)
        print("AB_CASE", json.dumps({key: value for key, value in case.items() if key != "widths"}), flush=True)
    totals = totals_of(results)
    return {
        "status": STATUS_OK,
        "native_status": status.as_record(),
        "root": str(root),
        "workers": args.workers,
        "steps": args.steps,
        "width_step": args.width_step,
        "cases": results,
        "totals": totals,
        "native_share": native_share(totals),
        "latency": latency_of(results),
        "not_measured": NOT_MEASURED,
    }


def _print_report(report: dict) -> None:
    print(format_table(report), flush=True)
    for line in format_latency(report["latency"]):
        print(line, flush=True)
    totals, share = report["totals"], report["native_share"]
    print(
        f"CALLS native={totals['native_calls']} python={totals['python_calls']} fallback_domains={totals['fallback_domains']} | "
        f"SHARE native domains={share['domains']} area={share['area']} (area unknown for {share['area_unknown_domains']} domains) | "
        f"INTERACTION python={totals['python_interaction']:.1f}s native={totals['native_interaction']:.1f}s | "
        f"{NOT_MEASURED['apply_seconds'].split(':')[0]} apply",
        flush=True,
    )
    if totals["native"] + totals["mixed"] == 0:
        print("NOTE: no native operation was called (see the fallbacks column)", flush=True)
    for case in report["cases"]:
        for item in case.get("widths", ()):
            for kind in ("answer", "price", "missing"):
                for line in item["differences"][kind][:5]:
                    print(f"DIFFERENT[{kind}] {case['case']} width {item['width']:g}: {line}", flush=True)
    for line in report["strict"]["violations"]:
        print(f"STRICT_VIOLATION {line}", flush=True)


def main() -> int:
    args = _arguments()
    root = Path(args.root).resolve()
    _load_tree(root, args.native_path)

    from cftuv_envelope.backend import UNAVAILABLE, native_status

    status = native_status()
    print("native status:", json.dumps(status.as_record()), flush=True)
    cases = [item for item in args.cases.split(",") if item]
    expected, expectation_error = resolve_expected_build_id(args.expect_build_id, args.strict, root)
    preflight = port_violations(status.as_record(), expected) if args.strict else []
    no_native = status.coverage == UNAVAILABLE and status.clip == UNAVAILABLE
    if no_native or preflight or (args.strict and expectation_error):
        report = unavailable_report(status.as_record(), str(root), cases)  # нечего сравнивать (диагностика) либо приёмка отказана до прогона
        if args.strict:
            report["skipped"] = "STRICT_PREFLIGHT"
    else:
        report = _run_cases(cases, status, root, args)
    finalize(report, strict=args.strict, expected_build_id=expected, expectation_error=expectation_error)
    if "totals" in report:
        _print_report(report)
    else:
        for line in report["strict"]["violations"]:
            print(f"STRICT_VIOLATION {line}", flush=True)
    if args.out:
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    try:
        from cftuv.envelope_domain_pool import shutdown_domain_pool

        shutdown_domain_pool()
    except Exception as exc:  # noqa: BLE001 - уборка не меняет итог
        print("pool shutdown:", type(exc).__name__, exc)
    print(final_line(report), flush=True)
    return exit_code(report)


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
