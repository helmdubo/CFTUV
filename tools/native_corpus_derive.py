"""Производные записи корпуса: настоящие вызовы с урезанным потолком бюджета (исчерпание, называемое ядром, и частичное состояние).

    "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe" tools/native_corpus_derive.py \\
        [--corpus <каталог корпуса>] [--per-group 3] [--shares 0.3,0.7] [--min-spent 8]

Поле выгрузки (`native_corpus_export.py`) не знает отказов по бюджету: ни одна его запись не кончилась исключением, а цена исчерпания
(`exhaustion_detail`: число единиц по статьям на момент отказа, стадия, операция, радиканд), частичная память канонизации и
частичный счёт знаков — часть равенства, которое обязана держать нативная реализация. Здесь по каждой группе (операция, меш) берутся
записи с наибольшей ценой (не меньше `--min-spent` единиц) и каждая переисполняется эталоном с потолком `потрачено_до + доля * цена`
(доля < 1: потолок заведомо ниже цены вызова). Вход тот же (пикл поля), состояние до то же, кроме потолка; ИСХОД — тот, что дал
эталон на ЭТОЙ версии питона (запускайте под питоном 3.11 продукта, как и выгрузку): он не измерен полем, а выведен из него.
Записи лежат в `records/_derived/`, в индексе помечены `derived` и в статистику замера не входят (проверяются так же, как прочие).
Повторный запуск заменяет прежние производные записи.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт

import native_bench  # noqa: E402  (путь к mpmath/sympy под питоном Blender; импортирует `native_corpus`)
import native_corpus as nc  # noqa: E402


def spent_by(record) -> int:
    """Сколько единиц потратил записанный вызов."""

    before, expected = record.before(), record.expected()
    return sum(after - was for after, was in zip(expected.after.budget["articles"], before.budget["articles"]))


def starved_before(before: nc.StateV1, spent: int, share: float) -> nc.StateV1:
    """Состояние до с потолком `потрачено_до + int(доля * цена)`: ниже цены вызова, пока доля < 1."""

    cap = sum(before.budget["articles"]) + int(spent * share)
    return nc.bounded_before(before, cap)


def _derive_one(root: Path, row: dict, share: float, number: int, preset: int) -> dict:
    record = nc.read_record(root / row["path"])
    op = record.op
    before = starved_before(record.before(), spent_by(record), share)
    outcome = nc.execute(nc.prepare_call(op, record.call_blob, before))
    meta = {key: value for key, value in row.items() if key not in ("path", "bytes")}
    meta.update(
        id=f"d{number:04d}-{op}",
        seconds=outcome.seconds,
        outcome=f"raised:{outcome.exception[0]}" if outcome.exception else nc.outcome_label(op, outcome.result, None),
        exception=outcome.exception,
        derived={"from": row["id"], "cap": before.budget["cap"], "share": share},
        **nc.result_shape(op, outcome.result),
    )
    relative = f"records/_derived/{row['mesh']}/{meta['id']}-from-{row['id'].split('-')[0]}.rec"
    size = nc.write_record(root / relative, meta, nc.make_payload(op, before, record.call_blob, outcome), preset)
    return {**meta, "path": relative, "bytes": size}


def derive_records(root: Path, *, per_group: int = 3, shares=(0.3, 0.7), min_spent: int = 8, preset: int = nc.DEFAULT_PRESET) -> list:
    """Строит производные записи корпуса `root`, обновляет `index.json`; возвращает строки производных записей."""

    index = nc.load_index(root)
    nc.remove_indexed_derived(root, index["records"])
    base = [row for row in index["records"] if not row.get("derived")]
    groups: dict = {}
    for row in base:
        spent = spent_by(nc.read_record(root / row["path"]))
        if spent >= min_spent:
            groups.setdefault((row["op"], row["mesh"]), []).append((-spent, row["id"], row))
    derived = []
    for _key, items in sorted(groups.items()):
        for _spent, _id, row in sorted(items, key=lambda item: item[:2])[:per_group]:
            for share in shares:
                derived.append(_derive_one(root, row, share, len(derived) + 1, preset))
    index["records"] = base + derived
    index["records_count"] = len(base)
    index["derived_count"] = len(derived)
    index["total_bytes"] = sum(row["bytes"] for row in base) + sum(row["bytes"] for row in derived)
    (root / "index.json").write_text(json.dumps(index, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    return derived


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--corpus", default="")
    parser.add_argument("--per-group", type=int, default=3)
    parser.add_argument("--shares", default="0.3,0.7")
    parser.add_argument("--min-spent", type=int, default=8)
    args = parser.parse_args()
    root = Path(args.corpus) if args.corpus else native_bench.default_corpus()
    derived = derive_records(
        root, per_group=args.per_group, shares=[float(item) for item in args.shares.split(",")], min_spent=args.min_spent
    )
    raised = sum(1 for row in derived if row["outcome"].startswith("raised:"))
    print(f"NATIVE_CORPUS_DERIVE_OK derived={len(derived)} raised={raised} bytes={sum(row['bytes'] for row in derived)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
