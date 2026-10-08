"""Производные записи корпуса скелета: настоящие вызовы с урезанным потолком бюджета (исчерпание, называемое ядром, и частичное состояние).

    python tools/native_skeleton_derive.py --corpus field|synthetic|<каталог> [--per-mesh 6] [--shares 0.02,0.05,...] [--min-spent 8] [--max-seconds 3]

Исчерпание бюджета — не единичная ветка, а десяток разных моментов: во внутренностях `_prime_universe_from_q_values` (факторизация набора `q`), посреди марша мотоцикла,
в куче (знак внутри `heappush`), в гидратации места, в сопряжении при знаке. Каждый момент оставляет своё состояние: частичную память канонизации, частичные статьи, `superlevel`,
частичный `SIGN_COUNTS`, текст `exhaustion_detail` с названием операции и радиканда. Полевой и синтетический корпуса сами по себе исчерпанием не кончаются ни разу.
Здесь по каждой группе (меш) берётся `--per-mesh` записей, равномерно по рангу цены (не только самые тяжёлые: ряд мелких доменов даёт больше РАЗНЫХ моментов исчерпания за то же время),
и каждая переисполняется эталоном с потолком `потрачено_до + доля * цена` для каждой доли ряда `--shares` (доля < 1: потолок заведомо ниже цены вызова). Вход тот же (пикл), состояние
до то же, кроме потолка; ИСХОД — тот, что дал эталон на ЭТОЙ версии питона (запись выведена из поля, а не измерена в нём). Записи — `records/_derived/`, в индексе помечены `derived`; повторный
запуск заменяет прежние производные записи. Итог печатает, на каких операциях называемого отказа исчерпание остановилось.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402

DEFAULT_SHARES = (0.01, 0.03, 0.07, 0.12, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 0.97)


def spent_by(record) -> int:
    """Сколько единиц потратил записанный вызов (0 у вызова без бюджета)."""

    before, expected = record.before(), record.expected()
    if before.budget is None:
        return 0
    return sum(after - was for after, was in zip(expected.after.budget["articles"], before.budget["articles"]))


def _derive_one(root: Path, row: dict, cap: int, tag: dict, number: int, preset: int) -> dict:
    """Одна производная запись: тот же вход и состояние до, потолок `cap`; `tag` — как он выбран (доля либо целевая трата)."""

    record = sc.read(root, row)
    before = record.before()
    before = nc.bounded_before(before, cap)
    call = nc.prepare_call(record.op, record.call_blob, before)
    outcome = nc.execute(call)
    meta = {key: value for key, value in row.items() if key not in ("path", "bytes", "live_equal", "label", "test")}
    meta.update(
        id=f"d{number:05d}-{record.op}",
        seconds=outcome.seconds,
        outcome=f"raised:{outcome.exception[0]}" if outcome.exception else nc.outcome_label(record.op, outcome.result, None),
        exception=outcome.exception,
        derived={"from": row["id"], "cap": cap, **tag},
        **nc.result_shape(record.op, outcome.result),
    )
    relative = f"records/_derived/{row['mesh']}/{meta['id']}-from-{row['id'].split('-')[0]}.rec"
    size = nc.write_record(root / relative, meta, nc.make_payload(record.op, before, record.call_blob, outcome), preset)
    return {**meta, "path": relative, "bytes": size}


def spend_trace(record) -> list:
    """Лента трат записи: `[(операция, потрачено ПОСЛЕ траты)]` в порядке исполнения (потолок записи не мешает: он не сработал в записи)."""

    log: list = []
    original = exact.ExactWorkBudgetV1._enforce

    def traced(self, operation, radicand):
        log.append((operation.value, self.spent))
        return original(self, operation, radicand)

    exact.ExactWorkBudgetV1._enforce = traced
    try:
        nc.execute(nc.prepare_call(record.op, record.call_blob, record.before()))
    finally:
        exact.ExactWorkBudgetV1._enforce = original
    return log


def targeted_caps(trace: list, occurrences: int) -> list:
    """Потолки, при которых отказ падает ровно на выбранной трате: `[(потолок, метка)]` по каждой операции ленты.

    `потрачено_после - 1` — отказ именно на этой трате (всё раньше проходит); `потрачено_после` — граница: эта трата проходит (`spent <= cap`), падает следующая."""

    by_operation: dict = {}
    for position, (operation, spent) in enumerate(trace):
        by_operation.setdefault(operation, []).append((position, spent))
    caps = []
    for operation, items in sorted(by_operation.items()):
        count = min(occurrences, len(items))
        for number in range(count):
            position, spent = items[int(number * len(items) / count)] if count > 1 else items[0]
            caps.append((spent - 1, {"target": operation, "occurrence": position, "boundary": False}))
        position, spent = items[len(items) // 2]
        caps.append((spent, {"target": operation, "occurrence": position, "boundary": True}))
    return caps


def _spread(items: list, count: int) -> list:
    """`count` записей, равномерных по рангу цены (от самой тяжёлой по убыванию шага): `items` — `(цена, id, строка)` по убыванию цены."""

    if len(items) <= count:
        return items
    step = len(items) / count
    return [items[int(index * step)] for index in range(count)]


def derive_records(
    root: Path, *, per_mesh: int = 6, shares=DEFAULT_SHARES, min_spent: int = 8, max_seconds: float = 3.0, occurrences: int = 5, preset: int = nc.DEFAULT_PRESET
) -> list:
    """Строит производные записи корпуса `root`, обновляет `index.json`; возвращает строки производных записей."""

    index = sc.load_index(root)
    nc.remove_indexed_derived(root, index["records"])
    base = [row for row in index["records"] if row.get("derived") is None]
    groups: dict = {}
    for row in base:
        if row["op"] != nc.OP_SKELETON or row["seconds"] > max_seconds or not row["budget"]:
            continue
        spent = spent_by(sc.read(root, row))
        if spent >= min_spent:
            groups.setdefault(row["mesh"], []).append((-spent, row["id"], row))
    derived: list = []
    for _mesh, items in sorted(groups.items()):
        for _spent, _id, row in _spread(sorted(items, key=lambda item: item[:2]), per_mesh):
            record = sc.read(root, row)
            before = record.before()
            spent = spent_by(record)
            for share in shares:
                cap = sum(before.budget["articles"]) + int(spent * share)
                derived.append(_derive_one(root, row, cap, {"share": share}, len(derived) + 1, preset))
            for cap, tag in targeted_caps(spend_trace(record), occurrences):
                derived.append(_derive_one(root, row, cap, {"share": None, **tag}, len(derived) + 1, preset))
    index["records"] = base + derived
    index["records_count"] = len(base)
    index["derived_count"] = len(derived)
    index["total_bytes"] = sum(row["bytes"] for row in base) + sum(row["bytes"] for row in derived)
    (root / "index.json").write_text(json.dumps(index, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    return derived


def exhaustion_operations(derived: list) -> dict:
    """Операции называемого отказа, на которых остановились производные записи: из текста исключения (`operation=...`)."""

    counts: Counter = Counter()
    for row in derived:
        if row["exception"]:
            found = re.search(r"operation=([A-Z_]+)", row["exception"][1]) or re.search(r"\b(PRIME_UNIVERSE|COPRIME_BASIS|PRIMALITY|POLLARD_RHO_BRENT|SQUAREFREE_SPLIT|PRIME_SUPPORT|EXACT_POSITION)\b", row["exception"][1])
            counts[found.group(1) if found else row["exception"][0]] += 1
    return dict(counts)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--corpus", required=True)
    parser.add_argument("--per-mesh", type=int, default=6)
    parser.add_argument("--shares", default=",".join(str(item) for item in DEFAULT_SHARES))
    parser.add_argument("--min-spent", type=int, default=8)
    parser.add_argument("--max-seconds", type=float, default=3.0)
    parser.add_argument("--occurrences", type=int, default=5)
    arguments = parser.parse_args(argv)
    root = sc.matching(arguments.corpus) if arguments.corpus in sc.KINDS else Path(arguments.corpus)
    if root is None:
        raise SystemExit(f"NATIVE_SKELETON_DERIVE_FAILED {sc.describe_missing(arguments.corpus)}")
    derived = derive_records(
        root, per_mesh=arguments.per_mesh, shares=[float(item) for item in arguments.shares.split(",")], min_spent=arguments.min_spent,
        max_seconds=arguments.max_seconds, occurrences=arguments.occurrences,
    )
    raised = sum(1 for row in derived if row["outcome"].startswith("raised:"))
    print(json.dumps({"exhaustion_operations": exhaustion_operations(derived), "outcomes": dict(Counter(row["outcome"] for row in derived))}))
    print(f"NATIVE_SKELETON_DERIVE_OK derived={len(derived)} raised={raised} bytes={sum(row['bytes'] for row in derived)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
