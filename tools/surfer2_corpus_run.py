"""Прогон корпуса против exact-Surfer2. ОТДЕЛЬНЫЙ скрипт, а не тест.

Почему отдельный, а не `kernel/tests/test_*.py`. Бинаря Surfer2 в дереве нет и
не будет (GPL-3, clean-room), в CI он собирается по расписке
`artifacts/surfer2_r0_spike/build_receipt.json`. Тест, требующий бинаря,
красил бы чистый клон в красный — то есть наказывал бы за отсутствие того,
чего в репозитории нет по решению. Поэтому здесь скрипт с `--binary`, а
ИМЕНОВАННЫЙ отказ `SURFER2_BINARY_IS_NOT_AVAILABLE` — штатный выход
(rc = 0), пока не сказано `--require-binary`. Контракт самого раннера
(генератор GraphML, разбор OBJ, допуск, фильтр внутренности) проверяется
юнитами без бинаря — `tests/test_surfer2_oracle.py`.

Проверка личности бинаря идёт ДО корпуса и до импорта ядра: подменённый
double-сборкой оракул обязан остановить прогон, а не украсить расписку.

Расписка кладётся в `artifacts/surfer2_r0_spike/corpus_report.json` — НОВЫМ
именем рядом со спайковым `diff_smoke_report.json`, который не
переписывается: он свидетельство своего прогона и своей даты.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = REPO_ROOT / "artifacts" / "surfer2_r0_spike" / "corpus_report.json"

sys.path.insert(0, str(Path(__file__).resolve().parent))

from surfer2_oracle import (  # noqa: E402
    AdmissionOutcome,
    ComparisonOutcome,
    DEFAULT_RECEIPT,
    OracleOutcome,
    PointPlacement,
    ToleranceModelV1,
    graphml_document,
    interior_nodes,
    kernel_node_rows,
    match_nodes,
    place_point,
    rings_of_polygon,
    run_surfer,
    verify_binary,
)


#: Именованный пропуск: бинаря нет. Не исход оракула — исход СТЕНДА, поэтому
#: имя своё, а не из `OracleOutcome`.
BINARY_IS_NOT_AVAILABLE = "SURFER2_BINARY_IS_NOT_AVAILABLE"

#: Пустая выборка `--case` и частичный прогон поверх полной расписки —
#: отдельные имена: молча зелёный прогон ни одной фигуры и молча урезанная
#: расписка одинаково выглядят как успех.
CORPUS_SELECTION_IS_EMPTY = "CORPUS_SELECTION_IS_EMPTY"
PARTIAL_RUN_WOULD_OVERWRITE_THE_FULL_RECEIPT = (
    "PARTIAL_RUN_WOULD_OVERWRITE_THE_FULL_RECEIPT"
)

DECLARED_INTERSECTION = (
    "плоский PSLG на целочисленной решётке",
    "фронт идёт от КАЖДОГО ребра: стен (q = 0) нет",
    "веса рациональны как евклидова скорость sqrt(q)/|d|",
    "вееров вогнутых вершин нет: рёбра нулевой длины в GraphML не выразимы",
    "сравниваются СОБЫТИЯ (t > 0); вершины входа отбрасываются с обеих сторон",
)


def _kernel_paths() -> None:
    for relative in ("kernel/src", "kernel/tests"):
        path = str(REPO_ROOT / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def weighted_corpus():
    """Рационально-взвешенные входы. Решение по весам уже принято (R0-СПАЙК).

    Оракул работает на ПЕРЕСЕЧЕНИИ рациональных весов: вес Surfer2 — евклидова
    скорость, наше `q` — её квадрат в единицах нормали длины `|d|`, поэтому
    передаётся вход ровно тогда, когда `sqrt(q)/|d|` рационален. Здесь взяты
    два таких: целый вес и дробный. Иррациональные `sqrt(q)` в корпус НЕ
    добавлены и добавлены быть не могут — они вне области по построению, и
    весовой дифференциал на них остаётся in-house (P0-3).

    Числа подобраны так, чтобы ответ отличался от невзвешенного: у квадрата с
    ускоренным нижним ребром узел уезжает вверх и распадается надвое, то есть
    вес меняет ТОПОЛОГИЮ ответа, а не только координаты.
    """

    from cftuv_envelope.wavefront.polygon import LoopV1, PolygonV1

    square = ((0, 0), (8, 0), (8, 8), (0, 8))
    ell = ((0, 0), (12, 0), (12, 6), (6, 6), (6, 12), (0, 12))
    return (
        # w = 2 на нижнем ребре: q = w^2 * |d|^2 = 4 * 64.
        ("weighted_square_8_bottom_w2", PolygonV1(LoopV1(square, (256, 64, 64, 64)))),
        # w = 3/2 на нижнем ребре: q = (9/4) * 64 = 144.
        (
            "weighted_square_8_bottom_w3_2",
            PolygonV1(LoopV1(square, (144, 64, 64, 64))),
        ),
        # w = 3/2 на двух рёбрах длины 12 и 6: q = (9/4)*144 и (9/4)*36.
        (
            "weighted_ell_12_w3_2_on_two_edges",
            PolygonV1(
                LoopV1(ell, (324, 36, 36, 81, 36, 144)),
            ),
        ),
    )


def _scales(rings, kernel_rows) -> tuple[float, float]:
    """Масштаб допуска берётся с НАШЕЙ стороны, не из ответа оракула.

    Невязка шестизначной печати пропорциональна модулю числа, поэтому масштаб
    нужен — но взять его по напечатанному выходу было бы дырой: оракул задавал
    бы себе допуск сам. Дыра не гипотетическая. `--component=-1` считает скелет
    ВСЕХ компонент, включая внешнюю неограниченную, и её вершины уезжают далеко
    за габарит фигуры: у `field_building_002_scale_64` печать доходит до 14519
    при габарите входа 1139, то есть допуск раздулся бы в 13 раз ровно на самой
    трудной фигуре корпуса.

    Поэтому масштаб — это максимум модуля по КООРДИНАТАМ ВХОДА и по узлам
    НАШЕГО скелета (включая время). Обе величины известны до всякого разговора
    с оракулом, и подвинуть их он не может.
    """

    input_scale = max(
        max(abs(float(x)), abs(float(y)))
        for ring in rings
        for x, y in ring.points
    )
    kernel_scale = max(
        (
            max(abs(row["x"]), abs(row["y"]), abs(row["t"]))
            for row in kernel_rows
        ),
        default=0.0,
    )
    return float(input_scale), float(kernel_scale)


def _component_cross_check(binary, graphml, interior):
    """Второй, независимый способ выделить внутренность: `--component=0`.

    Согласие двух способов — контроль МЕТОДИКИ, а не источник ответа. Прогон
    может и не состояться: на трёхвершинных входах Surfer2 падает по SIGSEGV
    (воспроизводится на их же `test-data/convex03.graphml`), и это записывается
    отдельным исходом, а не «методики разошлись».
    """

    result = run_surfer(binary, graphml, component="0")
    if result.outcome is not OracleOutcome.EXACT:
        return f"n/a: {result.outcome.value} rc={result.rc}"
    component_nodes = tuple(
        sorted((n["x"], n["y"], n["t"]) for n in result.nodes if n["t"] > 0)
    )
    return component_nodes == interior


def _reading(events: int, in_region: int, in_outer: int) -> str:
    """Что за область оказалась под номером компоненты. Читается по узлам."""

    if events == 0:
        return "NO_FINITE_EVENT_NODES"
    if in_region == events:
        return "REGION_WITH_HOLES"
    if in_outer == events and in_region == 0:
        return "HOLE_INTERIOR"
    if in_outer == 0:
        return "OUTSIDE_THE_OUTER_RING"
    return "MIXED"


def _component_regions(binary, graphml, rings, tolerance, limit=4):
    """Как Surfer2 нумерует компоненты КОЛЬЦЕВОГО входа — измерено, не додумано.

    Карточка спрашивает прямо: чем у них оказывается кольцевая область. Ответ
    добывается прогоном по номерам компонент и ЧТЕНИЕМ узлов геометрией, а не
    чтением их исходников: номер компоненты — недокументированная деталь
    (`tag_components` сеет 0 слева от ПЕРВОГО ребра списка), и опираться на
    неё в сверке нельзя. Здесь она только измеряется и кладётся в расписку.
    """

    outer_only = (rings[0],)
    probes = {}
    for component in range(limit):
        result = run_surfer(binary, graphml, component=str(component))
        if result.outcome is not OracleOutcome.EXACT:
            probes[str(component)] = {
                "outcome": result.outcome.value,
                "rc": result.rc,
            }
            continue
        events = [node for node in result.nodes if node["t"] > 0]
        in_region = sum(
            1
            for node in events
            if place_point(node["x"], node["y"], rings, tolerance)
            is PointPlacement.INSIDE
        )
        in_outer = sum(
            1
            for node in events
            if place_point(node["x"], node["y"], outer_only, tolerance)
            is PointPlacement.INSIDE
        )
        probes[str(component)] = {
            "outcome": result.outcome.value,
            "vertices_total": len(result.nodes),
            "event_nodes": len(events),
            "event_nodes_in_region": in_region,
            "event_nodes_inside_outer_ring": in_outer,
            "reading": _reading(len(events), in_region, in_outer),
        }
    return probes


def _compare(kernel_rows, interior, ambiguous, kernel_outcome, tolerance):
    """Вердикт по одной фигуре. Каждая ветка — своё имя, тихой нет.

    `kernel_outcome` — исход НАШЕГО построения, и сравнивается он по `.value`,
    а не по `str()`: форма печати перечисления менялась между версиями Python,
    и сверка, опирающаяся на неё, разъехалась бы молча.
    """

    match = match_nodes(
        tuple((row["x"], row["y"], row["t"]) for row in kernel_rows),
        interior,
        tolerance,
    )
    if kernel_outcome.value != "EXACT":
        verdict = ComparisonOutcome.KERNEL_SKELETON_IS_NOT_EXACT
    elif ambiguous:
        verdict = ComparisonOutcome.INTERIOR_FILTER_IS_AMBIGUOUS
    elif match.kernel_only or match.surfer_only:
        verdict = ComparisonOutcome.NODES_DISAGREE
    else:
        verdict = ComparisonOutcome.NODES_AGREE_WITHIN_TOLERANCE
    return verdict, match


def _surfer_record(oracle) -> dict:
    """Конверт оракула БЕЗ сырых узлов: они ниже уже разобраны и отфильтрованы.

    Ключ `nodes` выброшен, а не обнулён: `None` в конверте оракула означает
    «ответа не было», и записать им «ответ есть, но лежит в другом поле»
    значило бы завести вторую, ложную форму отказа.
    """

    return {key: value for key, value in oracle.payload().items() if key != "nodes"}


def run_case(polygon, binary, tolerance_model, *, cross_check=True):
    """Одна фигура: допуск, оракул, фильтр, ядро, сверка. Всё в расписку."""

    from cftuv_envelope.wavefront import build_skeleton

    admission, detail, rings = rings_of_polygon(polygon)
    if admission is not AdmissionOutcome.ADMITTED:
        return {"admission": admission.value, "admission_detail": detail}

    # Ядро считается ПЕРВЫМ: из него и из входа берётся масштаб допуска, и
    # тогда допуск не зависит от того, что напечатал оракул.
    skeleton = build_skeleton(polygon)
    kernel_rows = kernel_node_rows(skeleton)
    input_scale, kernel_scale = _scales(rings, kernel_rows)
    scale = max(input_scale, kernel_scale)
    tolerance = tolerance_model.for_scale(scale)

    graphml = graphml_document(rings)
    oracle = run_surfer(binary, graphml)
    record = {
        "admission": admission.value,
        "edge_weights": [
            [str(weight) for weight in ring.weights] for ring in rings
        ],
        "ring_sizes": [len(ring.points) for ring in rings],
        "surfer": _surfer_record(oracle),
    }
    if oracle.outcome is not OracleOutcome.EXACT:
        record["comparison"] = ComparisonOutcome.ORACLE_DID_NOT_ANSWER.value
        return record

    interior, ambiguous = interior_nodes(oracle.nodes, rings, tolerance)
    verdict, match = _compare(
        kernel_rows, interior, ambiguous, skeleton.outcome, tolerance
    )
    record.update(
        {
            "scale_input": input_scale,
            "scale_kernel": kernel_scale,
            "scale_printed_observed": max(
                (
                    max(abs(n["x"]), abs(n["y"]), abs(n["t"]))
                    for n in oracle.nodes
                ),
                default=0.0,
            ),
            "tolerance_abs": tolerance,
            "tolerance_floor_binds": tolerance_model.floor_binds(scale),
            "surfer_vertices_total": len(oracle.nodes),
            "surfer_interior_nodes": [list(node) for node in interior],
            "surfer_boundary_ambiguous": list(ambiguous),
            "kernel_outcome": str(skeleton.outcome),
            "kernel_levels": skeleton.levels,
            "kernel_nodes": list(kernel_rows),
            "comparison": verdict.value,
            "matched": len(match.pairs),
            "kernel_only": [kernel_rows[i] for i in match.kernel_only],
            "surfer_only": [list(interior[j]) for j in match.surfer_only],
            "max_residual_abs": match.max_residual,
            "max_residual_relative": (
                match.max_residual / scale if scale else 0.0
            ),
        }
    )
    if cross_check:
        record["component0_agrees_with_geometric_filter"] = _component_cross_check(
            binary, graphml, interior
        )
        if len(rings) > 1:
            record["ring_component_probe"] = _component_regions(
                binary, graphml, rings, tolerance
            )
    return record


AGREE = ComparisonOutcome.NODES_AGREE_WITHIN_TOLERANCE.value


def _verdict(case: dict) -> str:
    """Имя исхода фигуры: сверка, а у не допущенной — причина не-допуска."""

    return case.get("comparison", case["admission"])


def _totals(cases: dict) -> dict:
    compared = [case for case in cases.values() if _verdict(case) == AGREE]
    not_admitted = [
        name
        for name, case in cases.items()
        if case["admission"] != AdmissionOutcome.ADMITTED.value
    ]
    return {
        "cases_total": len(cases),
        "cases_admitted": len(cases) - len(not_admitted),
        "cases_agreeing": len(compared),
        "cases_not_admitted": sorted(not_admitted),
        "nodes_matched": sum(case["matched"] for case in compared),
        "max_residual_abs": max(
            (case["max_residual_abs"] for case in compared), default=0.0
        ),
        "max_residual_relative": max(
            (case["max_residual_relative"] for case in compared), default=0.0
        ),
        "outcomes": {
            value: sum(1 for case in cases.values() if _verdict(case) == value)
            for value in sorted({_verdict(case) for case in cases.values()})
        },
    }


def _findings(cases: dict) -> list[dict]:
    """Всё, что не «узлы совпали». Находка называется, а не тонет в числах."""

    findings = []
    for name, case in sorted(cases.items()):
        verdict = _verdict(case)
        if verdict == AGREE:
            continue
        findings.append(
            {
                "case": name,
                "outcome": verdict,
                "detail": case.get("admission_detail")
                or case.get("surfer", {}).get("detail", ""),
                "kernel_only": case.get("kernel_only", []),
                "surfer_only": case.get("surfer_only", []),
                "boundary_ambiguous": case.get("surfer_boundary_ambiguous", []),
            }
        )
    return findings


def _selected(names, wanted):
    return [entry for entry in names if not wanted or entry[0] in wanted]


def _parse(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--binary",
        type=Path,
        default=os.environ.get("SURFER2_BINARY"),
        help="путь к внешнему бинарю surfer (или переменная SURFER2_BINARY)",
    )
    parser.add_argument("--receipt", type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--case", action="append", default=[])
    parser.add_argument(
        "--require-binary",
        action="store_true",
        help="отсутствие бинаря — ОТКАЗ, а не именованный пропуск",
    )
    parser.add_argument("--no-weighted", action="store_true")
    parser.add_argument("--no-cross-check", action="store_true")
    return parser.parse_args(argv)


def _skip(arguments, detail: str) -> int:
    payload = {
        "outcome": BINARY_IS_NOT_AVAILABLE,
        "detail": detail,
        "how_to_build": "artifacts/surfer2_r0_spike/build_receipt.json",
        "report_written": False,
    }
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 1 if arguments.require_binary else 0


def main(argv=None) -> int:
    arguments = _parse(argv)
    if arguments.binary is None:
        return _skip(arguments, "--binary не задан и SURFER2_BINARY пуста")
    identity = verify_binary(Path(arguments.binary), arguments.receipt)
    if identity.outcome is OracleOutcome.SURFER2_ORACLE_BINARY_IS_MISSING:
        return _skip(arguments, f"{arguments.binary}: файла нет")
    if not identity.verified:
        print(json.dumps({"binary_identity": identity.payload()}, indent=2,
                         ensure_ascii=False))
        return 1

    # Частичный прогон поверх ПОЛНОЙ расписки — тихая её порча: файл остался,
    # фигур в нём стало меньше, а ворота этого не видят. Требуем свой `--out`.
    if arguments.case and arguments.out == DEFAULT_OUT:
        print(json.dumps({
            "outcome": PARTIAL_RUN_WOULD_OVERWRITE_THE_FULL_RECEIPT,
            "detail": f"выбрано {len(arguments.case)} фигур; задайте --out",
        }, indent=2, ensure_ascii=False))
        return 1

    _kernel_paths()
    from wavefront_cases import named_corpus

    started = time.monotonic()
    tolerance_model = ToleranceModelV1()
    entries = list(_selected(named_corpus(), set(arguments.case)))
    if not arguments.no_weighted:
        entries += _selected(weighted_corpus(), set(arguments.case))
    if not entries:
        print(json.dumps({
            "outcome": CORPUS_SELECTION_IS_EMPTY,
            "detail": f"--case {arguments.case} не совпал ни с одной фигурой",
        }, indent=2, ensure_ascii=False))
        return 1

    cases = {
        name: run_case(
            polygon,
            Path(arguments.binary),
            tolerance_model,
            cross_check=not arguments.no_cross_check,
        )
        for name, polygon in entries
    }
    report = {
        "role": "R0 corpus receipt: exact-Surfer2 oracle vs CFTUV wavefront kernel",
        "produced_by": "tools/surfer2_corpus_run.py",
        "corpus": [
            "kernel/tests/wavefront_cases.py::named_corpus",
            "tools/surfer2_corpus_run.py::weighted_corpus",
        ],
        "declared_model_intersection": list(DECLARED_INTERSECTION),
        "binary_identity": identity.payload(),
        "tolerance_model": tolerance_model.payload(),
        "totals": _totals(cases),
        "findings": _findings(cases),
        "cases": cases,
    }
    arguments.out.parent.mkdir(parents=True, exist_ok=True)
    arguments.out.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    totals = report["totals"]
    print(
        json.dumps(
            {
                "out": str(arguments.out),
                "elapsed_seconds": round(time.monotonic() - started, 1),
                "totals": totals,
                "findings": [item["outcome"] for item in report["findings"]],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    # Не допущенный вход — объявленный отсев, а не провал: он назван причиной
    # и той же причиной исключён из ворот. Провал — это допущенная фигура,
    # которая НЕ сошлась.
    return 0 if totals["cases_agreeing"] == totals["cases_admitted"] else 1


if __name__ == "__main__":
    sys.exit(main())
