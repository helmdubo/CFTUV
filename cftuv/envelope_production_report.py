"""Строки владельцу и свидетельства продуктового прогона: статус, консоль, квитанция, JSON-свидетельство.

Чистый текст по результатам домена (`ProductionDomainResultV1`) и квитанции записи меша: ни кэшей сессии, ни пула, ни ядра. Вынесено из
`envelope_production_export.py` ради бюджета размера модуля (поведение и имена прежние: `envelope_production_export` их переэкспортирует).
Строка времени прогона (`production_timing_text`) осталась там: она читает счётчики прогона, которые называет тот модуль.
"""

from __future__ import annotations

import json
from pathlib import Path

from .envelope_production_weld import (
    COUNTER_FACES_OFF_PLANE_AFTER_OFFSET,
    COUNTER_MAX_OFF_PLANE_AFTER_OFFSET,
    weld_console_lines,
)
from .envelope_stretch_lines import developable_stretch_lines


def refused_outcome_counts(results) -> dict[str, int]:
    """`{исход: сколько доменов}` по отказам, порядок — по имени исхода."""

    counts: dict[str, int] = {}
    for item in results:
        if not item.is_materialized:
            counts[item.outcome] = counts.get(item.outcome, 0) + 1
    return dict(sorted(counts.items()))


def _outcome_counts(rows) -> dict[str, int]:
    counts: dict[str, int] = {}
    for _patch, _domain, outcome, _detail in rows:
        counts[outcome] = counts.get(outcome, 0) + 1
    return dict(sorted(counts.items()))


def _status_text(written: int, rows, warnings=()) -> str:
    """`MATERIALIZED n / refused m (OUTCOME x2, OTHER)` и, если есть, `| warnings`."""

    rows = tuple(rows)
    text = f"MATERIALIZED {written} / refused {len(rows)}"
    counts = _outcome_counts(rows)
    if counts:
        text += " (" + ", ".join(
            name if count == 1 else f"{name} x{count}"
            for name, count in counts.items()
        ) + ")"
    names: dict[str, int] = {}
    for _patch, outcome, _detail in warnings:
        names[outcome] = names.get(outcome, 0) + 1
    if names:
        text += " | warnings: " + ", ".join(
            name if count == 1 else f"{name} x{count}"
            for name, count in sorted(names.items())
        )
    return text


def _refused_rows(results):
    return [
        (item.patch_id, item.domain_id, item.outcome, item.detail)
        for item in results
        if not item.is_materialized
    ]


def production_status_text(results) -> str:
    """Строка по результатам ПРОДУКТОВОГО пути (до записи меша)."""

    results = tuple(results)
    return _status_text(
        sum(1 for item in results if item.is_materialized), _refused_rows(results)
    )


def receipt_status_text(receipt) -> str:
    """Строка панели по КВИТАНЦИИ записи: сколько домен лежит в меше, а остальное названо.

    Пропуск писателя (`ADAPTER_*`) стоит в ней наравне с отказом продуктового
    пути: домен, которого нет в меше, не может молчать по любой из причин.
    """

    return _status_text(len(receipt.domains), receipt.skipped, receipt.warnings)


def receipt_report_level(receipt) -> str:
    """`WARNING`, если хоть один домен не в меше (по любой причине) либо есть находка; иначе `INFO`."""

    return "WARNING" if receipt.skipped or receipt.warnings else "INFO"


def _row_line(kind, patch_id, domain_id, outcome, detail) -> str:
    return (
        f"[CFTUV][Production] {kind} patch {patch_id} "
        f"(domain ...{str(domain_id)[-6:]}): {outcome}"
        + (f": {detail}" if detail else "")
    )


def production_console_lines(results) -> list[str]:
    """Каждый отказанный домен — строкой с исходом и деталью; затем итог."""

    results = tuple(results)
    lines = [
        _row_line("REFUSED", *row) for row in _refused_rows(results)
    ]
    lines.append(f"[CFTUV][Production] {production_status_text(results)}")
    return lines


def diagnostic_summary_lines(results) -> list[str]:
    """Диагностики батчей (`NEAR_PLANAR_...`, `U_RESTARTS_...`) по именам: сколько доменов и каких."""

    found: dict[str, list[int]] = {}
    for item in results:
        for line in item.diagnostics:
            found.setdefault(line.split(":", 1)[0], []).append(item.patch_id)
    lines = []
    for name, patches in sorted(found.items()):
        shown = sorted(set(patches))
        tail = ", ".join(str(item) for item in shown[:12])
        more = f", ... (+{len(shown) - 12})" if len(shown) > 12 else ""
        lines.append(
            f"[CFTUV][Production] DIAGNOSTIC {name}: {len(patches)} in "
            f"{len(shown)} domains (patch {tail}{more})"
        )
    return lines


def receipt_console_lines(receipt, results) -> list[str]:
    """Консольная сводка квитанции: каждый пропущенный домен, предупреждения, диагностики, итог."""

    lines = [_row_line("REFUSED", *row) for row in receipt.skipped]
    for patch_id, outcome, detail in receipt.warnings:
        where = "mesh" if patch_id is None else f"patch {patch_id}"
        lines.append(
            f"[CFTUV][Production] WARNING {where}: {outcome}"
            + (f": {detail}" if detail else "")
        )
    lines.extend(diagnostic_summary_lines(results))
    lines.extend(developable_stretch_lines(results))
    lines.extend(weld_console_lines(getattr(receipt, "weld_counters", ())))
    offset = dict(getattr(receipt, "offset_counters", ()) or ())
    if offset.get(COUNTER_FACES_OFF_PLANE_AFTER_OFFSET):
        lines.append(
            f"[CFTUV][Production] OFFSET: {offset[COUNTER_FACES_OFF_PLANE_AFTER_OFFSET]} faces of 4+ vertices "
            f"leave their plane after the offset, at most {offset[COUNTER_MAX_OFF_PLANE_AFTER_OFFSET] / 1e6:.3f} mm "
            "(recorded, not judged)"
        )
    lines.append(f"[CFTUV][Production] {receipt_status_text(receipt)}")
    return lines


def export_production_json(results, directory, *, label: str = "production") -> Path:
    """Батчи MATERIALIZED-доменов и сводка в `directory`: свидетельство для зонда.

    Батч идёт кодеком ядра (`GeometryBatchCodecV1`): канонические байты, их же
    читает `GeometryBatchCodecV1.loads`. Сводка — исход, детали и дайджесты
    каждого домена, всё без секунд (сравнимо между прогонами и воркерами).
    """

    from cftuv_envelope import GeometryBatchCodecV1

    folder = Path(directory)
    folder.mkdir(parents=True, exist_ok=True)
    rows = []
    for item in results:
        row = {
            "patch_id": item.patch_id,
            "domain_id": item.domain_id,
            "outcome": item.outcome,
            "detail": item.detail,
            "content_digest": item.content_digest,
            "counters": dict(item.counters),
            "diagnostics": list(item.diagnostics),
            "normal": None if item.normal is None else list(item.normal),
            "offset_normal_law": item.offset_normal_law,
            "offset_normals_digest": item.offset_normals_digest,
            "decal_topology_law": item.decal_topology_law,
            "alpha_interval": None if item.alpha_interval is None else item.alpha_interval.as_record(),
            "structure_digest": item.structure_digest,
        }
        if item.is_materialized:
            name = f"{label}_patch{item.patch_id:04d}.geometry_batch.json"
            (folder / name).write_bytes(GeometryBatchCodecV1.dumps(item.batch))
            row["batch_file"] = name
            row["semantic_digest"] = item.batch.semantic_digest.value
        rows.append(row)
    summary = folder / f"{label}_summary.json"
    summary.write_text(
        json.dumps(
            {"label": label, "domains": rows, "status": production_status_text(results)},
            ensure_ascii=False,
            indent=1,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return summary


__all__ = (
    "diagnostic_summary_lines",
    "export_production_json",
    "production_console_lines",
    "production_status_text",
    "receipt_console_lines",
    "receipt_report_level",
    "receipt_status_text",
    "refused_outcome_counts",
)
