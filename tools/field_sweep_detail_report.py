"""Разбор ПРИЧИН полевых отказов очереди: (исход, деталь) -> счёт -> домены.

Зачем инструмент. Полевые расписки называют исход домена (`PLAN_IS_NOT_COMPILED`
на 12 доменах меша «2» и 5 на `building`), но ПРИЧИНУ несёт соседнее поле —
`detail`, и оно нигде не сводится. Пока причины не сведены, строка открытых
счетов «`PLAN_IS_NOT_COMPILED`: причины не разобраны» закрывается только чтением
мегабайтного JSON глазами, то есть тем самым ручным шагом, на котором цикл
перестаёт быть циклом (та же причина, по которой существует
`field_cycle_summary.py`). Один прогон этого инструмента по расписке владельца
даёт таблицу «исход × деталь × счёт» и поимённый список доменов каждой группы —
вход для FIELD-800S-STAGE-ATTRIBUTION.

Откуда берётся `detail`. Ядро кладёт в него ИМЯ исхода той ступени, на которой
путь остановился: `PLAN_IS_NOT_COMPILED` несёт имя исхода компиляции эталона
(`kernel/src/cftuv_envelope/wavefront/conveyor.py:1200-1202`), `DOMAIN_FRAME_IS_
UNAVAILABLE` — имя диагностики полезной нагрузки, и так далее. Хост переносит
поле в запись домена без изменений (`cftuv/envelope_queue_export.py:1036-1038`
для отказа, `:940` для успеха) и печатает его в sidecar
(`queue_domain_payload`, `cftuv/envelope_queue_export.py:1030-1045`).

Что инструмент НЕ делает: не нормализует деталь. Часть исходов кладёт в деталь
не одно имя класса, а идентичность: `DOMAIN_HAS_NO_REGIONS` и
`NO_STRIP_SOURCE_IN_THE_PLAN` кладут туда `patch_domain_id`, а
`BRIDGE_DID_NOT_MAP` и `SKELETON_DID_NOT_CLOSE` — «регион: исход ступени»
(`conveyor.py:1015-1024`). Поэтому группа размером 1 здесь — не дефект таблицы,
а свойство исхода. Склеивать такие детали «похожестью» значило бы придумать
класс, которого ядро не называло; читать их глазами по общей части имени
владелец может и так, а инструмент не должен утверждать больше, чем измерил.

Читаемые формы (распознаются по полю `schema`, а при его отсутствии — по
структуре):

* sidecar кнопки — `cftuv.envelope.staged_debug_runtime_sidecar.v1`,
  раздел `queue.domains[*]` (`cftuv/envelope_debug_renderer.py:1348`);
* расписка прямых ворот полевого цикла —
  `cftuv.envelope.runtime_metric_building_gate.v2`, домены
  `runs[*].domains[*].queue` (`tools/run_envelope_mr1_building_gate.py:474-477`);
* расписка свипа кнопки — `cftuv.envelope.density_button_sweep.v3`. Деталей она
  НЕ несёт вовсе: `parity_projection` собирается из исходов и дайджеста
  (`tools/blender_field_sweep.py:504-527`). Это называется по имени
  (`SWEEP_RECEIPT_CARRIES_NO_DETAIL`) вместе с путями прямых расписок, где
  деталь есть, а не превращается в пустую таблицу.

Имена ступеней и ключи полей скопированы из продукта, а не сочинены: равенство
копий их источнику проверяется `tests/test_field_sweep_detail_report.py`.
Инструмент — только stdlib: расписку читают и там, где ни Blender, ни аддон не
установлены.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from dataclasses import dataclass
import json
from pathlib import Path
import sys


REPORT_SCHEMA = "cftuv.envelope.field_detail_report.v1"

#: Схемы читаемых расписок. Значения — копии констант продукта:
#: `cftuv/envelope_debug_renderer.py:1319`, `tools/run_envelope_mr1_building_
#: gate.py:15`, `tools/blender_field_sweep.py:21`.
SIDECAR_SCHEMA = "cftuv.envelope.staged_debug_runtime_sidecar.v1"
GATE_SCHEMA = "cftuv.envelope.runtime_metric_building_gate.v2"
SWEEP_SCHEMA = "cftuv.envelope.density_button_sweep.v3"

#: Исход-успех. Ровно то значение, с которым сравнивает хост
#: (`cftuv/envelope_queue_export.py:_queue_stage`).
EXACT = "EXACT"

#: Ступени домена очереди — копии `EnvelopeDomainStage`
#: (`cftuv/envelope_debug_profile.py:24-26`).
QUEUE_PREPARE_REJECTED = "QUEUE_PREPARE_REJECTED"
QUEUE_COVERAGE_REJECTED = "QUEUE_COVERAGE_REJECTED"
QUEUE_RESOLVED = "QUEUE_RESOLVED"

#: Деталь пустая: домен прошёл (у `EXACT` деталь пуста по построению) либо
#: ступень отказала, не назвав себя. Печатается именем, а не пустотой: пустая
#: ячейка в таблице неотличима от «графа нет».
DETAIL_IS_EMPTY = "DETAIL_IS_EMPTY"

#: Именованные ответы инструмента на вход, из которого причин не достать.
SWEEP_RECEIPT_CARRIES_NO_DETAIL = "SWEEP_RECEIPT_CARRIES_NO_DETAIL"
SOURCE_IS_NOT_A_FIELD_RECEIPT = "SOURCE_IS_NOT_A_FIELD_RECEIPT"
SOURCE_CARRIES_NO_QUEUE_DOMAINS = "SOURCE_CARRIES_NO_QUEUE_DOMAINS"
NO_QUEUE_DOMAINS_IN_INPUT = "NO_QUEUE_DOMAINS_IN_INPUT"
SOURCE_IS_NOT_READABLE = "SOURCE_IS_NOT_READABLE"

#: Ширина колонки детали в текстовой таблице. Полная деталь всегда лежит в
#: `--json`; та же величина усечения, что у `field_cycle_summary.py`.
DETAIL_TEXT_WIDTH = 120


@dataclass(frozen=True)
class DomainRecord:
    """Один домен одной расписки: где мерено, чем кончилось и почему."""

    source: str
    mesh: str
    density: int | None
    scope: str
    patch_id: int | None
    patch_domain_id: str
    preparation_outcome: str
    coverage_outcome: str
    detail: str

    @property
    def stage(self) -> str:
        """Ступень домена — тем же правилом, что у хоста.

        Копия `_queue_stage` (`cftuv/envelope_queue_export.py:1155-1163`):
        своё правило разошлось бы с колонкой, с которой таблицу сверяют.
        """

        if self.preparation_outcome != EXACT:
            return QUEUE_PREPARE_REJECTED
        if self.coverage_outcome != EXACT:
            return QUEUE_COVERAGE_REJECTED
        return QUEUE_RESOLVED

    @property
    def outcome(self) -> str:
        """Исход, которым домен отказал; `EXACT` — если не отказал.

        Отказавшая подготовка отвечает за себя сама, покрытие в этом случае
        лишь пересказывает её имя (`PREPARATION_IS_NOT_EXACT`) — брать его
        значило бы потерять ступень, на которой всё остановилось.
        """

        if self.preparation_outcome != EXACT:
            return self.preparation_outcome
        if self.coverage_outcome not in (EXACT, ""):
            return self.coverage_outcome
        return EXACT

    @property
    def reason(self) -> str:
        return self.detail or DETAIL_IS_EMPTY


@dataclass(frozen=True)
class SourceReading:
    """Прочитанная расписка: её домены и её именованный ответ, если их нет."""

    path: str
    schema: str
    note: str
    records: tuple[DomainRecord, ...]


def _int_or_none(value) -> int | None:
    return int(value) if isinstance(value, int) else None


def _queue_record(
    source: str,
    mesh: str,
    density: int | None,
    scope: str,
    patch_id,
    patch_domain_id,
    queue: dict,
) -> DomainRecord:
    """Ключи читаются ровно те, что пишет `queue_domain_payload`."""

    return DomainRecord(
        source=source,
        mesh=str(mesh),
        density=density,
        scope=scope,
        patch_id=_int_or_none(patch_id),
        patch_domain_id=str(patch_domain_id),
        preparation_outcome=str(queue.get("preparation_outcome") or ""),
        coverage_outcome=str(queue.get("coverage_outcome") or ""),
        detail=str(queue.get("detail") or ""),
    )


def _sidecar_records(payload: dict, source: str) -> tuple[DomainRecord, ...]:
    """Домены sidecar'а кнопки.

    `object_name` — имя ОТЛАДОЧНОГО объекта (рендерер пишет
    `ENVELOPE_DEBUG_GP_PREFIX + source`), и переименовывать его обратно здесь
    нечем: правило именования принадлежит хосту. В таблицу идёт то имя, которое
    расписка несёт.

    Плотность у sidecar'а `None`, и это не пропуск: `_staged_sidecar` её не
    пишет вовсе (ручка запроса лежит в расписке свипа, `request_policy_knobs`).
    Подставить сюда «1» значило бы назвать плотность, которой файл не называл.
    """

    queue = payload.get("queue") or {}
    mesh = payload.get("object_name") or ""
    return tuple(
        _queue_record(
            source,
            mesh,
            None,
            "",
            domain.get("patch_id"),
            domain.get("patch_domain_id"),
            domain,
        )
        for domain in queue.get("domains") or ()
        if isinstance(domain, dict)
    )


def _gate_records(payload: dict, source: str) -> tuple[DomainRecord, ...]:
    """Домены расписки прямых ворот: раздел `queue` внутри строки домена.

    Домены без раздела `queue` пропускаются молча только в одном смысле — их
    не было в прогоне (движок `raw`); отсутствие ВСЕХ разделов называется
    вызывающему через `SOURCE_CARRIES_NO_QUEUE_DOMAINS`.
    """

    mesh = payload.get("source_object") or ""
    density = _int_or_none(payload.get("effective_density"))
    records: list[DomainRecord] = []
    for run in payload.get("runs") or ():
        if not isinstance(run, dict):
            continue
        scope = str(run.get("scope") or "")
        for domain in run.get("domains") or ():
            queue = domain.get("queue") if isinstance(domain, dict) else None
            if not isinstance(queue, dict):
                continue
            records.append(
                _queue_record(
                    source,
                    mesh,
                    density,
                    scope,
                    domain.get("patch_id"),
                    domain.get("patch_domain_id"),
                    queue,
                )
            )
    return tuple(records)


def _sweep_note(payload: dict) -> str:
    """Свип детали не несёт — но несёт пути расписок, у которых она есть."""

    receipts: list[str] = []
    for density in payload.get("densities") or ():
        if not isinstance(density, dict):
            continue
        # `direct_button_parity` — расписка сверки кнопки с прямыми воротами
        # (`tools/blender_field_sweep.py:857-874`); её `direct_receipt` и есть
        # файл, в котором деталь лежит.
        for parity in density.get("direct_button_parity") or ():
            path = parity.get("direct_receipt") if isinstance(parity, dict) else None
            if path:
                receipts.append(str(path))
    listed = ", ".join(sorted(set(receipts)))
    return (
        f"{SWEEP_RECEIPT_CARRIES_NO_DETAIL}: расписка свипа сводит исходы и "
        "дайджесты, детали в ней нет по построению; передайте прямые расписки"
        + (f" — {listed}" if listed else "")
    )


def read_source(path: Path) -> SourceReading:
    """Одна расписка -> её домены. Нераспознанная форма называется по имени."""

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        return SourceReading(str(path), "", f"{SOURCE_IS_NOT_READABLE}: {exc}", ())
    if not isinstance(payload, dict):
        return SourceReading(str(path), "", SOURCE_IS_NOT_A_FIELD_RECEIPT, ())
    schema = str(payload.get("schema") or "")
    if schema == SWEEP_SCHEMA or "densities" in payload:
        return SourceReading(str(path), schema, _sweep_note(payload), ())
    if schema == SIDECAR_SCHEMA or "queue" in payload:
        records = _sidecar_records(payload, str(path))
    elif schema == GATE_SCHEMA or "runs" in payload:
        records = _gate_records(payload, str(path))
    else:
        return SourceReading(str(path), schema, SOURCE_IS_NOT_A_FIELD_RECEIPT, ())
    note = "" if records else SOURCE_CARRIES_NO_QUEUE_DOMAINS
    return SourceReading(str(path), schema, note, records)


def read_paths(paths) -> tuple[SourceReading, ...]:
    """Файлы и каталоги -> прочитанные расписки. Каталог обходится рекурсивно."""

    readings: list[SourceReading] = []
    for item in paths:
        path = Path(item)
        if path.is_dir():
            readings.extend(
                read_source(found) for found in sorted(path.rglob("*.json"))
            )
        else:
            readings.append(read_source(path))
    return tuple(readings)


def _domain_order(record: DomainRecord):
    """Порядок доменов в группе — номером патча, а не строкой.

    Строковая сортировка ставит `10.0` между `1.0` и `2.0`, и список из
    двенадцати доменов перестаёт читаться как список патчей поля.
    """

    return (record.patch_id is None, record.patch_id or 0, record.patch_domain_id)


def _mesh_order(key: tuple[str, int | None]):
    """Меш по имени, плотности по возрастанию, неназванная плотность — последней."""

    mesh, density = key
    return (mesh, density is None, -1 if density is None else density)


def group_records(records) -> list[dict]:
    """Группы (исход, деталь) по мешу и плотности, от частых к редким.

    Сортировка: счёт по убыванию, дальше имя исхода и деталь — чтобы две
    расписки одного прогона давали побайтово одну таблицу.
    """

    by_mesh: dict[tuple[str, int | None], list[DomainRecord]] = defaultdict(list)
    for record in records:
        by_mesh[(record.mesh, record.density)].append(record)
    meshes = []
    for (mesh, density), items in sorted(
        by_mesh.items(), key=lambda pair: _mesh_order(pair[0])
    ):
        groups: dict[tuple[str, str], list[DomainRecord]] = defaultdict(list)
        for record in items:
            groups[(record.outcome, record.reason)].append(record)
        meshes.append(
            {
                "mesh": mesh,
                "effective_density": density,
                "domain_count": len(items),
                "refused_count": sum(
                    1 for item in items if item.stage != QUEUE_RESOLVED
                ),
                "groups": [
                    {
                        "outcome": outcome,
                        "detail": detail,
                        "count": len(members),
                        "stages": sorted({item.stage for item in members}),
                        "patch_domain_ids": [
                            item.patch_domain_id
                            for item in sorted(members, key=_domain_order)
                        ],
                    }
                    for (outcome, detail), members in sorted(
                        groups.items(),
                        key=lambda pair: (-len(pair[1]), pair[0][0], pair[0][1]),
                    )
                ],
            }
        )
    return meshes


def total_groups(records) -> list[dict]:
    """Те же группы поверх всех расписок: сколько всего и на каких мешах."""

    counts: Counter[tuple[str, str]] = Counter()
    meshes: dict[tuple[str, str], Counter[str]] = defaultdict(Counter)
    for record in records:
        key = (record.outcome, record.reason)
        counts[key] += 1
        meshes[key][record.mesh] += 1
    return [
        {
            "outcome": outcome,
            "detail": detail,
            "count": count,
            "meshes": dict(sorted(meshes[(outcome, detail)].items())),
        }
        for (outcome, detail), count in sorted(
            counts.items(), key=lambda pair: (-pair[1], pair[0][0], pair[0][1])
        )
    ]


def build_report(readings) -> dict:
    """Расписка инструмента: источники, группы по мешам, общий итог."""

    records = tuple(record for reading in readings for record in reading.records)
    return {
        "schema": REPORT_SCHEMA,
        "sources": [
            {
                "path": reading.path,
                "source_schema": reading.schema,
                "domain_count": len(reading.records),
                "note": reading.note,
            }
            for reading in readings
        ],
        "domain_count": len(records),
        "refused_count": sum(1 for item in records if item.stage != QUEUE_RESOLVED),
        "meshes": group_records(records),
        "totals": total_groups(records),
    }


def _detail_column(detail: str) -> str:
    if len(detail) <= DETAIL_TEXT_WIDTH:
        return detail
    return detail[: DETAIL_TEXT_WIDTH - 1] + "…"


def _group_members(group: dict) -> str:
    """Кто именно в группе: домены поимённо у меша, меши со счётом — в итоге."""

    if "patch_domain_ids" in group:
        return "домены: " + ", ".join(group["patch_domain_ids"])
    return "меши: " + ", ".join(
        f"{name} {count}" for name, count in group["meshes"].items()
    )


def _print_groups(groups, indent: str) -> None:
    for group in groups:
        print(
            f"{indent}{group['count']:>5}  {group['outcome']:<34} "
            f"{_detail_column(group['detail'])}"
        )
        # У прошедших доменов перечислять нечего: их имена не вопрос. Строка
        # `EXACT` остаётся в таблице ради знаменателя — «12 из 32» без «32»
        # не число.
        if group["outcome"] == EXACT:
            continue
        print(f"{indent}       {_group_members(group)}")


def print_report(report: dict) -> None:
    """Текстовая таблица. Ничего не считает заново — печатает ту же расписку."""

    for source in report["sources"]:
        line = f"источник: {source['path']}  доменов {source['domain_count']}"
        if source["source_schema"]:
            line += f"  [{source['source_schema']}]"
        print(line)
        if source["note"]:
            print(f"          {source['note']}")
    for mesh in report["meshes"]:
        density = mesh["effective_density"]
        print()
        print(
            f"меш {mesh['mesh']}"
            + ("" if density is None else f", плотность {density}")
            + f": доменов {mesh['domain_count']}, "
            f"отказавших {mesh['refused_count']}"
        )
        print("   счёт  исход                              деталь")
        _print_groups(mesh["groups"], "  ")
    print()
    print(
        f"ИТОГО: доменов {report['domain_count']}, "
        f"отказавших {report['refused_count']}"
    )
    print("   счёт  исход                              деталь")
    _print_groups(report["totals"], "  ")


def _parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Группировка доменов полевых расписок очереди по (исход, деталь)."
        )
    )
    parser.add_argument(
        "paths",
        nargs="+",
        help="пути к расписке/sidecar или каталогу с ними (обход рекурсивный)",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="печатать расписку инструмента JSON'ом вместо таблицы",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    parsed = _parse_arguments(sys.argv[1:] if argv is None else argv)
    readings = read_paths(parsed.paths)
    report = build_report(readings)
    if parsed.json:
        print(json.dumps(report, ensure_ascii=False, indent=1, sort_keys=True))
    else:
        print_report(report)
    if report["domain_count"] == 0:
        # Пустая таблица с кодом 0 неотличима от «отказов нет». Вход, из
        # которого не вышло ни одного домена, обязан называть себя.
        print(NO_QUEUE_DOMAINS_IN_INPUT, file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
