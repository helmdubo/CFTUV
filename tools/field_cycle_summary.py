"""Сводка полевого прогона: JSON ворот -> короткая таблица в консоль.

Ворота (`run_envelope_mr1_building_gate.py`) пишут полный JSON — он и есть
свидетельство. Но решение «стало лучше или хуже» принимается по нескольким
числам, и выуживать их из мегабайта глазами — ровно тот ручной шаг, на котором
цикл перестаёт быть циклом. Сводка печатает: домены и их исходы, Resolved N/M,
секунды по стадиям. Ничего не вычисляет заново — только читает файл ворот.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path


def _queue_column(domain: dict) -> str:
    """Колонка QUEUE рядом с RAW. Отсутствие стадии печатается, а не молчит."""

    queue = domain.get("queue")
    if queue is None:
        return "  | QUEUE -"
    line = f"  | {queue.get('stage'):<22}"
    prepare = queue.get("prepare_seconds")
    coverage = queue.get("coverage_seconds")
    contour = queue.get("contour_seconds")
    if prepare is not None:
        line += f" prep {prepare:.3f} c"
    if coverage is not None:
        line += f" cov {coverage + (contour or 0.0):.3f} c"
    line += f" faces {queue.get('faces', 0)}"
    return line


def _queue_totals(domains) -> tuple[int, int, float, float]:
    ready = 0
    prepare = 0.0
    coverage = 0.0
    counted = 0
    for domain in domains:
        queue = domain.get("queue")
        if queue is None:
            continue
        counted += 1
        prepare += queue.get("prepare_seconds") or 0.0
        coverage += (queue.get("coverage_seconds") or 0.0) + (
            queue.get("contour_seconds") or 0.0
        )
        if queue.get("stage") == "QUEUE_RESOLVED":
            ready += 1
    return ready, counted, prepare, coverage


def summarise(path: Path) -> int:
    payload = json.loads(path.read_text(encoding="utf-8"))
    print(f"файл    : {path}")
    print(f"сцена   : {payload.get('source_file')}")
    print(f"меш     : {payload.get('source_object')}, статус {payload.get('status')}")
    unchanged = payload.get("source_mesh_unchanged")
    if unchanged is not None:
        print(f"меш не изменён прогоном: {unchanged}")

    exit_code = 0
    for run in payload.get("runs", ()):
        domains = run.get("domains", [])
        ready = [d for d in domains if d.get("stage") == "RAW_READY"]
        queue_ready, queue_seen, prepare, coverage = _queue_totals(domains)
        print()
        print(
            f"scope {run.get('scope')}: RAW {len(ready)}/{len(domains)}, "
            f"QUEUE {queue_ready}/{len(domains)}, "
            f"topology {run.get('topology_elapsed_seconds', 0.0):.2f} c"
        )
        if queue_seen:
            print(
                f"  QUEUE итого: prepare {prepare:.2f} c, "
                f"coverage {coverage:.2f} c, всего {prepare + coverage:.2f} c"
            )
        change = run.get("queue_alpha_change") or {}
        if change.get("domains"):
            print(
                f"  QUEUE смена alpha на {change.get('alpha')} по тёплому "
                f"кэшу: {change['domains']} доменов за "
                f"{change.get('elapsed_seconds', 0.0) * 1000.0:.0f} мс"
            )
            if change.get("palette_wrapped"):
                print(
                    "  палитра владельцев переполнена: "
                    f"{change['palette_wrapped']} сверх восьми"
                )
        for domain in domains:
            runtime = domain.get("runtime") or {}
            performance = runtime.get("performance") or {}
            seconds = performance.get("raw_coverage_seconds")
            line = (
                f"  patch {domain.get('patch_id'):>3}  "
                f"{domain.get('stage'):<16} {domain.get('outcome')}"
            )
            if seconds is not None:
                line += f"  raw {seconds:.3f} c"
            print(line + _queue_column(domain))
            if domain.get("stage") not in ("RAW_READY",):
                message = (domain.get("message") or "").splitlines()
                if message:
                    print(f"        {message[0][:120]}")
                exit_code = 1
            queue = domain.get("queue")
            if queue is not None and queue.get("stage") != "QUEUE_RESOLVED":
                print(
                    f"        QUEUE {queue.get('preparation_outcome')}/"
                    f"{queue.get('coverage_outcome')}: "
                    f"{(queue.get('detail') or '')[:120]}"
                )
                exit_code = 1
    return exit_code


if __name__ == "__main__":
    target = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    if target is None or not target.exists():
        raise SystemExit("нужен путь к JSON полевых ворот")
    sys.exit(summarise(target))
