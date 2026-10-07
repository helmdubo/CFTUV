"""Целая операция скелета через вставку (`cftuv_native.build_skeleton`) против эталона: общий прогонщик тестов и замера.

    set PYTHONSAFEPATH=1
    python tools/native_skeleton_whole.py bench [--corpus field|<каталог>] [--repeat 3] [--meshes a,b] [--limit N] [--heavy 5] [--out <json>]
    python tools/native_skeleton_whole.py check [--corpus field|synthetic|<каталог>] [--stride N] [--live]
    "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe" tools/native_skeleton_whole.py bench ...   (питон 3.11 продукта;
        расширение: `pip install --target ~/.cftuv-native/py311-site <колесо>` и `PYTHONPATH=~/.cftuv-native/py311-site`)

ВСТАВКА — граница, какой её увидит сеанс Python: сигнатура `skeleton.build_skeleton(polygon, *, split_search, work_budget, dense_hydration)`, результат — настоящий
`SkeletonV1`, а всё остальное — настоящие объекты процесса: статьи `ExactWorkBudgetV1` и строка `superlevel`, `SIGN_COUNTS`, `UNBUDGETED_WORK`, четыре таблицы памяти
канонизации (порядок вставки входит), настоящие исключения. Оба пути стартуют с ОДНОГО восстановленного состояния ДО (`native_corpus.prepare_call`); сравнение — то же
`native_corpus.compare_outcomes` (результат каноническим кодом: `int` и `Fraction` различны; исключение `(класс, текст)`; цена; память с порядком; знаки; неоплаченное).
Эталон — записанный исход записи, либо живой вызов ядра (`live`).

ОТКАЗ ПОРТА (`cftuv_native.NATIVE_REFUSALS`) — не расхождение, а второй вид исхода: он обязан оставить ВСЁ состояние как до вызова (сверяется), после чего ядро на тех же бюджете и
таблицах обязано дать записанный исход (сверяется): так продукт и откатывается. Число отказов и их причины считает прогонщик; тесты называют допустимые.

ЗАМЕР (`bench`): на каждой полевой записи (не производной) `repeat` проходов: эталон (живой вызов на восстановленном состоянии) и нативная вставка (на ОДНОМ сеансе на процесс, как в
продукте; классы привязаны один раз и в замер не входят), оба на одном и том же состоянии ДО; каждый нативный вызов сверяется с эталонным. Время нативного вызова — стенка вокруг вставки:
перевод полигона, синхронизация памяти, вычисление (GIL отпущен), построение результата, журнал памяти, статьи, счётчики; раскладка — `last_skeleton_timings`.
"""

from __future__ import annotations

import argparse
import gc
import json
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт

import native_bench as nb  # noqa: E402,F401  (путь к mpmath/sympy под питоном Blender; импортирует `native_corpus`)
import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402

#: Названные отказы порта (`cftuv_native.NATIVE_REFUSALS`), по именам классов.
REFUSALS = ("NativePortStale", "NativeUnsupportedPython", "NativePortUnsupported", "NativeDivisionDiverged")
PARTS = ("sync", "args", "compute", "result", "log", "post", "other", "total")


@dataclass
class Run:
    """Исход одного вызова на обоих путях: расхождения, секунды эталона и вставки, отказ порта (имя, текст), метка исхода эталона."""

    differences: list
    oracle_seconds: float
    native_seconds: float
    label: str
    refused: tuple | None = None
    expected: object = None
    actual: object = None
    parts: dict = field(default_factory=dict)

    @property
    def equal(self) -> bool:
        return not self.differences


def label_of(expected: "nc.Outcome") -> str:
    return f"raised:{expected.exception[0]}" if expected.exception else str(expected.result.outcome.value)


def _untouched(before: "nc.StateV1", after: "nc.StateV1") -> list:
    """Расхождения между состоянием ДО вызова и состоянием после отказа порта: пусто — не тронуто."""

    found = nc.compare_outcomes(nc.OP_SKELETON, before, nc.Outcome(None, None, before, {}), nc.Outcome(None, None, after, {}))
    return [nc.Difference(f"refusal-left-state.{item.field}", item.detail) for item in found]


class WholeRunner:
    """`cftuv_native.build_skeleton` (вставка) на настоящем состоянии процесса против записанного (или живого) исхода эталона. Один `mirror` живёт между вызовами, как в сеансе."""

    def __init__(self, mirror=None) -> None:
        import cftuv_native

        self.native = cftuv_native
        self.mirror = cftuv_native.new_mirror() if mirror is None else mirror

    def timings(self) -> dict:
        """Раскладка последнего вызова вставки (секунды): синхронизация, разбор аргументов, вычисление, результат, журнал, статьи и счётчики, остаток, стенка."""

        sync, called, post, total, arguments, compute, result, log = self.mirror.last_skeleton_timings
        parts = {"sync": sync, "args": arguments, "compute": compute, "result": result, "log": log, "post": post}
        parts = {name: value * 1e-9 for name, value in parts.items()}
        parts["total"] = total * 1e-9
        parts["other"] = max(0.0, parts["total"] - sum(parts.values()))
        return parts

    def compare(self, record: "nc.Record", *, live: bool = False, before: "nc.StateV1 | None" = None) -> Run:
        before = record.before() if before is None else before
        expected = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)) if live else record.expected()
        call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)
        actual = nc.execute(call, function=self.mirror.build_skeleton)
        label = label_of(expected)
        if actual.exception is not None and actual.exception[0] in REFUSALS:
            differences = _untouched(before, actual.after)
            fallback = nc.execute(call)
            differences += nc.compare_outcomes(nc.OP_SKELETON, before, expected, fallback)
            return Run(differences, expected.seconds, actual.seconds, label, refused=actual.exception, expected=expected, actual=fallback)
        differences = nc.compare_outcomes(nc.OP_SKELETON, before, expected, actual)
        return Run(differences, expected.seconds, actual.seconds, label, expected=expected, actual=actual, parts=self.timings())


def explain(runs: list, limit: int = 12) -> str:
    lines = []
    for name, run in runs:
        for item in run.differences:
            lines.append(f"{name}: {item}" + (f" (the port refused first: {run.refused})" if run.refused else ""))
    return "\n".join(lines[:limit]) + (f"\n... and {len(lines) - limit} more" if len(lines) > limit else "")


def percentile(values: list, fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        return 0.0
    return ordered[min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))]


def summarize(values: list) -> dict:
    return {"n": len(values), "p50": percentile(values, 0.5), "p95": percentile(values, 0.95), "max": max(values, default=0.0)}


def _ms(seconds: float) -> str:
    return f"{seconds * 1e3:9.2f}"


def timing_table(rows: list, left: str = "oracle", right: str = "native") -> str:
    """`rows`: `(группа, секунды левого, секунды правого)` -> таблица p50/p95/max по группам и отношение процентилей."""

    groups: dict = {}
    for group, first, second in rows:
        groups.setdefault(group, ([], []))
        groups[group][0].append(first)
        groups[group][1].append(second)
    lines = [f"{'group':28s} {'n':>4s} | {left} ms p50 / p95 / max | {right} ms p50 / p95 / max | x(p50) x(p95) x(max)"]
    for group in sorted(groups):
        first, second = summarize(groups[group][0]), summarize(groups[group][1])
        ratios = [(first[key] / second[key]) if second[key] else float("inf") for key in ("p50", "p95", "max")]
        lines.append(
            f"{group:28s} {first['n']:4d} | {_ms(first['p50'])} {_ms(first['p95'])} {_ms(first['max'])} | "
            f"{_ms(second['p50'])} {_ms(second['p95'])} {_ms(second['max'])} | {ratios[0]:6.1f} {ratios[1]:6.1f} {ratios[2]:6.1f}"
        )
    return "\n".join(lines)


# --------------------------------------------------------------------------
# корпус
# --------------------------------------------------------------------------


def corpus_root(spec: str) -> Path:
    """`field`/`synthetic` — корпус под ядро процесса; иначе путь к каталогу корпуса."""

    if spec in sc.KINDS:
        found = sc.matching(spec)
        if found is None:
            raise SystemExit(f"NATIVE_SKELETON_WHOLE_FAILED {sc.describe_missing(spec)}")
        return found
    path = Path(spec)
    # the directory of the kernel's field corpus (where `native_bench_native.py` takes its coverage and clip records from) holds the skeleton corpus beside them
    return path / sc.FIELD_DIR if (path / sc.FIELD_DIR / "index.json").exists() else path


# --------------------------------------------------------------------------
# замер
# --------------------------------------------------------------------------


def measure_record(runner: WholeRunner, record: "nc.Record", repeat: int) -> dict:
    """Одна запись: `repeat` проходов (эталон, затем вставка, оба на состоянии ДО), медиана секунд, раскладка вставки; каждый вызов сверяется."""

    before = record.before()
    oracle, native, parts = [], [], {name: [] for name in PARTS}
    differences: list = []
    refused = None
    for index in range(repeat):
        gc.collect()
        expected = nc.execute(nc.prepare_call(nc.OP_SKELETON, record.call_blob, before))
        call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, before)
        gc.collect()
        actual = nc.execute(call, function=runner.mirror.build_skeleton)
        if actual.exception is not None and actual.exception[0] in REFUSALS:
            refused = actual.exception
            differences.append({"repeat": index + 1, "fields": [f"port refused: {actual.exception}"]})
            continue
        found = nc.compare_outcomes(nc.OP_SKELETON, before, expected, actual)
        if found:
            differences.append({"repeat": index + 1, "fields": [str(item) for item in found]})
        oracle.append(expected.seconds)
        native.append(actual.seconds)
        for name, value in runner.timings().items():
            parts[name].append(value)

    def median(values: list) -> float:
        ordered = sorted(values)
        return ordered[len(ordered) // 2] if ordered else float("nan")

    return {
        "id": record.meta["id"], "mesh": record.meta["mesh"], "patch": record.meta.get("patch_id"), "domain_id": record.meta.get("domain_id"), "outcome": record.meta["outcome"],
        "polygon_vertices": record.meta.get("polygon_vertices"), "fan_supports": record.meta.get("fan_supports"), "nodes": record.meta.get("nodes"), "levels": record.meta.get("levels"),
        "oracle": median(oracle), "native": median(native), "parts": {name: median(values) for name, values in parts.items()},
        "oracle_all": oracle, "native_all": native, "differences": differences, "refused": refused,
    }


def text_report(results: list, heavy: int) -> str:
    rows = [(item["mesh"], item["oracle"], item["native"]) for item in results if item["native"] == item["native"]]
    rows += [("ALL", item["oracle"], item["native"]) for item in results if item["native"] == item["native"]]
    lines = ["whole build_skeleton, field records (median of the passes per record; p50 / p95 / max over records):", timing_table(rows)]
    top = sorted(results, key=lambda item: item["oracle"], reverse=True)[:heavy]
    lines.append("")
    lines.append(f"the {heavy} heaviest domains (ms): oracle | native | x | native by part: " + " ".join(PARTS))
    for item in top:
        parts = " ".join(f"{item['parts'][name] * 1e3:7.2f}" for name in PARTS)
        ratio = item["oracle"] / item["native"] if item["native"] else float("inf")
        lines.append(f"  {item['mesh']:24s} {str(item['domain_id'])[-8:]:>8s} v={item['polygon_vertices']!s:>3s} fans={item['fan_supports']!s:>3s} nodes={item['nodes']!s:>3s} {item['oracle'] * 1e3:10.1f} | {item['native'] * 1e3:9.2f} | {ratio:6.1f} | {parts}")
    return "\n".join(lines)


def run_bench(arguments) -> tuple:
    import cftuv_native

    status = cftuv_native.native_status()["skeleton"]
    if status != "available":
        print(f"NATIVE_SKELETON_WHOLE_FAILED the native skeleton port is {status}: refusing to measure it")
        raise SystemExit(2)
    root = corpus_root(arguments.corpus)
    meshes = set(filter(None, arguments.meshes.split(",")))
    rows = [row for row in sc.rows_of(root, derived=False) if not row["exception"] and (not meshes or row["mesh"] in meshes)]
    rows = rows[: arguments.limit] if arguments.limit else rows
    runner = WholeRunner()
    # classes bound once per process in a product, not a cost of the first call
    runner.mirror.skeleton_raw_layouts()
    results = []
    for number, row in enumerate(rows, 1):
        results.append(measure_record(runner, sc.read(root, row), arguments.repeat))
        if results[-1]["differences"]:
            print(f"MISMATCH {row['id']} {row['mesh']}: {results[-1]['differences'][0]}", flush=True)
        if number % 25 == 0:
            print(f"  {number}/{len(rows)} records", flush=True)
    bad = [item for item in results if item["differences"]]
    report = {
        "python": sys.version.split()[0], "native_version": cftuv_native.native_version(), "native_build_id": cftuv_native.native_build_id(), "corpus": str(root),
        "records": len(results), "repeat": arguments.repeat, "calls_checked": len(results) * arguments.repeat,
        "mismatches": [{"id": item["id"], "differences": item["differences"]} for item in bad], "per_record": results,
    }
    return report, len(bad)


def run_check(arguments) -> int:
    import cftuv_native

    status = cftuv_native.native_status()["skeleton"]
    if status != "available":
        print(f"NATIVE_SKELETON_WHOLE_FAILED the native skeleton port is {status}")
        return 2
    root = corpus_root(arguments.corpus)
    rows = sc.rows_of(root)[:: arguments.stride]
    runner = WholeRunner()
    bad = refusals = 0
    for number, row in enumerate(rows, 1):
        run = runner.compare(sc.read(root, row), live=arguments.live)
        refusals += run.refused is not None
        if not run.equal:
            bad += 1
            print(f"MISMATCH {row['id']} {row['mesh']}:\n{explain([(row['id'], run)], 6)}", flush=True)
        if number % 200 == 0:
            print(f"  {number}/{len(rows)}", flush=True)
    print(f"NATIVE_SKELETON_WHOLE_{'FAILED' if bad else 'OK'} records={len(rows)} mismatches={bad} port_refusals={refusals} python={sys.version.split()[0]}")
    return 1 if bad else 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    bench = commands.add_parser("bench")
    bench.add_argument("--corpus", default="field")
    bench.add_argument("--repeat", type=int, default=3)
    bench.add_argument("--meshes", default="")
    bench.add_argument("--limit", type=int, default=0)
    bench.add_argument("--heavy", type=int, default=5)
    bench.add_argument("--out", default="")
    check = commands.add_parser("check")
    check.add_argument("--corpus", default="field")
    check.add_argument("--stride", type=int, default=1)
    check.add_argument("--live", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.command == "check":
        return run_check(arguments)
    report, bad = run_bench(arguments)
    print(text_report(report["per_record"], arguments.heavy))
    print(f"NATIVE_SKELETON_WHOLE_{'FAILED' if bad else 'OK'} op=skeleton records={report['records']} native_calls_checked={report['calls_checked']} mismatches={bad} python={report['python']}")
    if arguments.out:
        Path(arguments.out).write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return 1 if bad else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
