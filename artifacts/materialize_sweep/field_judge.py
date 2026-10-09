"""Судья полевых случаев: два прогона `tools/blender_field_cases.py` по спецификации ожидаемого изменения (`expected_change.py`, инструмент `field`).

Полевой случай — настоящая кнопка на меше сцены (`меш:alpha:плотность:растяжение`). Запись прогона — `{"cases": [ {case, operator, status, verts, edges, faces, mesh_digest,
geometry_sha256, ...} ]}`. Судья приводит её к виду `runs[плотность].domains[номер случая]`, общему с `sweep.py` и `gate.py`, и зовёт тот же `expected_change.evaluate`:
без спецификации любое расхождение ответа — `UNEXPECTED`; со спецификацией разрешено ровно то, что она объявила для объявленных случаев (`domains.where` по полю `case`).

    python field_judge.py compare base.json new.json                               # любое расхождение - UNEXPECTED
    python field_judge.py compare base.json new.json --spec cone_angle_numeric_windows
    python field_judge.py compare --list-specs

Последняя строка вывода — один из трёх вердиктов `IDENTICAL`, `EXPECTED-CHANGE (spec X): N domains`, `UNEXPECTED (spec X): K problems; first: ...`; код возврата 0, 0, 1.
Время кнопки и прочая цена в ответ не входят.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import expected_change  # noqa: E402

#: Дайджесты строки (`allow.digests` спецификации) и прочие скалярные поля ответа (`allow.fields`).
DIGEST_FIELDS = ("geometry_sha256", "mesh_digest", "owner_labels_sha256", "owner_partition_sha256")
ANSWER_FIELDS = ("case", "domain_outcomes", "edges", "face_sizes", "faces", "op_error", "operator", "refused", "src_faces", "status", "verts")
VOCABULARY = expected_change.Vocabulary(digests=frozenset(DIGEST_FIELDS), fields=frozenset(ANSWER_FIELDS), counters=frozenset())


def _value(value):
    """Список и словарь сравниваются по записи JSON (порядок ключей не ответ)."""

    return json.dumps(value, sort_keys=True, ensure_ascii=False) if isinstance(value, (list, dict)) else value


def row_view(row: dict) -> expected_change.RowView:
    """Ответ случая: поля и дайджесты; цена (секунды) не входит. `ok` — оператор закончился `FINISHED`."""

    fields = {name: _value(row.get(name)) for name in ANSWER_FIELDS + DIGEST_FIELDS}
    return expected_change.RowView(fields=fields, counters={}, ok="FINISHED" in (row.get("operator") or ()) and not row.get("failure"))


def pair_views(base_row: dict, new_row: dict):
    return row_view(base_row), row_view(new_row)


def runs_of(records: list) -> list:
    """Записи прогонов -> `{"runs": {плотность: {"domains": {номер: строка}}}}`; номер случая един для обеих записей (отсортированное объединение случаев)."""

    cases = sorted({row["case"] for record in records for row in record["cases"]})
    number = {case: str(index) for index, case in enumerate(cases)}
    aligned = []
    for record in records:
        runs: dict = {}
        for row in record["cases"]:
            density = row["case"].split(":")[2]
            runs.setdefault(density, {"domains": {}})["domains"][number[row["case"]]] = row
        aligned.append({"runs": runs})
    return aligned


def compare(base: dict, new: dict, spec=None, partial: bool = False) -> expected_change.Report:
    aligned = runs_of([base, new])
    return expected_change.evaluate(aligned, ["base", "new"], spec, pair_views, VOCABULARY, partial)


def load_spec(references):
    """Спецификация по имени либо пути; список (`--spec a --spec b`) склеивается: группы друг за другом, `no_refusal` у любой - у всех. Пусто - `None`."""

    if not references:
        return None
    if isinstance(references, str):
        references = [references]
    specs = [expected_change.load_spec(reference, "field", VOCABULARY) for reference in references]
    if len(specs) == 1:
        return specs[0]
    require = expected_change.Require(no_refusal=any(item.require.no_refusal for item in specs))
    return expected_change.Spec(
        "+".join(item.name for item in specs), "field", " | ".join(item.law for item in specs), " ".join(item.about for item in specs),
        tuple(group for item in specs for group in item.groups), expected_change.AllowList(), require,
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    comparer = sub.add_parser("compare")
    comparer.add_argument("paths", nargs="*")
    comparer.add_argument("--spec", action="append", default=None, help="спецификация ожидаемого изменения: имя из specs/ либо путь к .json; можно несколько")
    comparer.add_argument("--partial", action="store_true", help="прогон части случаев: объявленные случаи вне записей - примечание")
    comparer.add_argument("--list-specs", action="store_true", help="перечислить сохранённые спецификации")
    arguments = parser.parse_args(argv)
    if arguments.list_specs:
        expected_change.print_stored_specs("field")
        return 0
    if len(arguments.paths) != 2:
        parser.error("compare needs two records: base.json new.json")
    records = [json.loads(Path(path).read_text(encoding="utf-8")) for path in arguments.paths]
    try:
        spec = load_spec(arguments.spec)
    except expected_change.SpecError as error:
        print(error)
        return 2
    report = compare(records[0], records[1], spec, arguments.partial)
    expected_change.print_report(report)
    return report.exit_code


if __name__ == "__main__":
    sys.exit(main())
