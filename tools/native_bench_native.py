"""Замер нативного `coverage._coverage_at` против эталона на Python на корпусе вызовов (`tools/native_corpus_export.py`).

    set PYTHONSAFEPATH=1
    python tools/native_bench_native.py [--corpus <каталог корпуса>] [--repeat 3] [--meshes building,...] [--limit N] [--out <json>]
    "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe" tools/native_bench_native.py ...   (питон 3.11 продукта;
        расширение: `pip install --target ~/.cftuv-native/py311-site <колесо>` и `PYTHONPATH=~/.cftuv-native/py311-site`)

Для каждой записи покрытия цепочка шагов идёт от ОДНОГО состояния («до» записи): шаг 0 — сама запись (ХОЛОДНЫЙ вызов: разбиение переводится в
нативную сессию, `store` — промах, если он пуст), шаги 1.. — тот же `partition` с другими alpha на той же сессии, том же бюджете и том же `store`
(ТЁПЛЫЕ шаги: попадание в `store`, память канонизации тёплая; именно их делает ползунок ширины). Эталон и нативная сторона идут по цепочке по очереди,
каждая от восстановленного состояния; время шага — одна операция (`time.perf_counter` вокруг вызова), снимок состояния снимается ВНЕ замера.

КАЖДЫЙ нативный вызов сверяется с эталоном точно (`native_corpus.compare_outcomes`: результат с различием `int`/`Fraction`, исключение, цена,
память с порядком, счётчики знаков, неоплаченное, `store`); расхождение — отказ замера (код 1). Нет расширения `cftuv_native` — отказ (код 2), отката на
питон нет. Время нативного вызова раскладывается (миллисекунды):

* `partition` — перевод разбиения в сессию (один раз на разбиение; только холодный вызов);
* `args` — на вызов: синхронизация памяти и бюджета в шиме (`sync`) и разбор аргументов в расширении (alpha, поиск в `store`, заголовок стоимости);
* `compute` — сама операция внутри Rust (замер внутри расширения, GIL отпущен);
* `result` — на возврат: построение `CoverageV1` и запись `store` в Rust (`build`) и разбор ответа, журнал памяти, статьи бюджета, счётчики в шиме (`post`);
* `other` — остальное: разбор аргументов PyO3, возврат кортежа, накладные `perf_counter`;
* `total` — стенка всего вызова, как её видит вызывающий.
"""

from __future__ import annotations

import argparse
import gc
import json
import statistics
import sys
import time
from fractions import Fraction
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт

import native_bench as nb  # noqa: E402  (путь к mpmath/sympy под питоном Blender; импортирует `native_corpus`)
import native_corpus as nc  # noqa: E402

#: Множители alpha тёплых шагов: ползунок ширины около записанной alpha (вперёд и назад, мелкие и крупные шаги).
WARM_FACTORS = (Fraction(15, 16), Fraction(17, 16), Fraction(7, 8), Fraction(9, 8), Fraction(3, 4), Fraction(5, 4), Fraction(1, 2))
PARTS = ("partition", "args", "compute", "result", "other", "total")


def load_extension():
    """Нативный шим либо отказ замера: тихого отката на питон нет."""

    try:
        import cftuv_native
    except ModuleNotFoundError as error:
        if error.name != "cftuv_native":
            raise
        print("NATIVE_BENCH_NATIVE_FAILED the cftuv_native extension is not importable (python tools/native_build.py; for 3.11: pip install --target ~/.cftuv-native/py311-site <wheel>)")
        raise SystemExit(2)
    return cftuv_native


def _run(function, call):
    """Результат либо исключение операции (тип и текст) и секунды вокруг одного вызова."""

    gc.collect()
    started = time.perf_counter()
    try:
        result, error = function(call), None
    except Exception as exc:  # noqa: BLE001 - исключение операции — часть её исхода
        result, error = None, (type(exc).__qualname__, str(exc))
    return result, error, time.perf_counter() - started


def oracle_chain(blob, before, alphas) -> list:
    """Эталон по цепочке alpha на одном бюджете и одном `store`: `[(Outcome, секунды)]`."""

    call = nc.prepare_call(nc.OP_COVERAGE, blob, before)
    partition = call.args[0]
    steps = []
    for alpha in alphas:
        step = nc.Call(nc.OP_COVERAGE, (partition, alpha), {}, call.budget, call.store)
        result, error, seconds = _run(lambda item: nc.ORACLE[nc.OP_COVERAGE](item.args[0], item.args[1], item.budget, item.store), step)
        steps.append((nc.Outcome(result, error, nc.capture_state(call.budget, call.store), {}, seconds), seconds))
    return steps


def native_chain(mirror, blob, before, alphas) -> list:
    """Нативная сторона по той же цепочке: `[(Outcome, секунды, части времени в секундах)]`."""

    call = nc.prepare_call(nc.OP_COVERAGE, blob, before)
    partition = call.args[0]
    steps = []
    for alpha in alphas:
        step = nc.Call(nc.OP_COVERAGE, (partition, alpha), {}, call.budget, call.store)
        result, error, seconds = _run(lambda item: mirror.coverage_at(item.args[0], item.args[1], item.budget, item.store), step)
        timings = mirror.last_timings
        parts = {"partition": timings[4], "args": timings[0] + timings[5], "compute": timings[6], "result": timings[7] + timings[2]}
        parts = {name: value * 1e-9 for name, value in parts.items()}
        parts["other"] = max(0.0, seconds - sum(parts.values()))
        parts["total"] = seconds
        if error is not None:  # a raised call leaves the timings of the call before it
            parts = {name: 0.0 for name in PARTS} | {"total": seconds}
        steps.append((nc.Outcome(result, error, nc.capture_state(call.budget, call.store), {}, seconds), seconds, parts))
    return steps


def chain_alphas(alpha) -> list:
    """Шаг 0 — alpha записи (как есть), дальше — тёплые шаги вокруг неё."""

    return [alpha] + [alpha * factor for factor in WARM_FACTORS]


def measure_record(extension, root: Path, row: dict, repeat: int) -> dict:
    """Одна запись: `repeat` проходов цепочки эталоном и нативной стороной, медиана по проходам для каждого шага; сверка КАЖДОГО шага."""

    record = nc.read_record(root / row["path"])
    before = record.before()
    alpha = nc.decode_call(nc.OP_COVERAGE, record.call_blob, None, None).args[1]
    alphas = chain_alphas(alpha)
    oracle_seconds = [[] for _ in alphas]
    native_seconds = [[] for _ in alphas]
    native_parts = [{name: [] for name in PARTS} for _ in alphas]
    differences: list = []
    raised = [False] * len(alphas)
    for index in range(repeat):
        expected = oracle_chain(record.call_blob, before, alphas)
        mirror = extension.new_mirror()
        actual = native_chain(mirror, record.call_blob, before, alphas)
        for step, ((want, seconds_o), (got, seconds_n, parts)) in enumerate(zip(expected, actual)):
            raised[step] = raised[step] or want.exception is not None
            oracle_seconds[step].append(seconds_o)
            native_seconds[step].append(seconds_n)
            for name in PARTS:
                native_parts[step][name].append(parts[name])
            found = nc.compare_outcomes(nc.OP_COVERAGE, before, want, got)
            if found:
                differences.append({"repeat": index + 1, "step": step, "fields": [str(item) for item in found][:4]})
    steps = [
        {
            "oracle": statistics.median(oracle_seconds[step]),
            "native": statistics.median(native_seconds[step]),
            "parts": {name: statistics.median(native_parts[step][name]) for name in PARTS},
            "raised": raised[step],
        }
        for step in range(len(alphas))
    ]
    return {"id": row["id"], "mesh": row["mesh"], "patch_id": row["patch_id"], "alpha": row["alpha"], "faces": row.get("faces"), "steps": steps, "differences": differences}


def aggregate(results: list) -> dict:
    """`{холодный|тёплый: {меш|ALL: {эталон, нативный, ускорение, части}}}` по записям: p50/p95/max."""

    table: dict = {"cold": {}, "warm": {}}
    for result in results:
        cold = [result["steps"][0]] if not result["steps"][0]["raised"] else []
        warm = [step for step in result["steps"][1:] if not step["raised"]]
        for mesh in (result["mesh"], "ALL"):
            table["cold"].setdefault(mesh, []).extend(cold)
            table["warm"].setdefault(mesh, []).extend(warm)
    return {kind: {mesh: _summary(items) for mesh, items in sorted(meshes.items())} for kind, meshes in table.items()}


def _summary(items: list) -> dict:
    parts = {name: nb.summarize([item["parts"][name] for item in items]) for name in PARTS}
    return {
        "n": len(items),
        "oracle": nb.summarize([item["oracle"] for item in items]),
        "native": nb.summarize([item["native"] for item in items]),
        "speedup_p50": nb.percentile([item["oracle"] / item["native"] for item in items], 0.5),
        "speedup_min": min(item["oracle"] / item["native"] for item in items),
        "parts": parts,
    }


def heaviest(results: list, count: int = 10) -> list:
    """Самые тяжёлые записи по времени эталона на холодном вызове."""

    ranked = sorted((item for item in results if not item["steps"][0]["raised"]), key=lambda item: -item["steps"][0]["oracle"])[:count]
    rows = []
    for item in ranked:
        warm = [step for step in item["steps"][1:] if not step["raised"]] or item["steps"][1:]
        rows.append({
            "id": item["id"], "mesh": item["mesh"], "patch_id": item["patch_id"], "faces": item["faces"],
            "cold_oracle": item["steps"][0]["oracle"], "cold_native": item["steps"][0]["native"], "cold_parts": item["steps"][0]["parts"],
            "warm_oracle": statistics.median(step["oracle"] for step in warm), "warm_native": statistics.median(step["native"] for step in warm),
            "warm_parts": {name: statistics.median(step["parts"][name] for step in warm) for name in PARTS},
        })
    return rows


def _ms(seconds: float) -> str:
    return f"{seconds * 1e3:8.3f}"


def text_report(report: dict) -> str:
    lines = [f"python {report['python']}  native {report['native_version']}  corpus {report['corpus']}  records {report['records']}  repeat {report['repeat']}  warm steps/record {len(WARM_FACTORS)}", ""]
    for kind in ("cold", "warm"):
        lines.append(f"{kind.upper()} ({'first call on a partition: conversion + store miss' if kind == 'cold' else 'later alphas on the same session: converted partition, store hit'}); times in ms")
        lines.append(f"{'mesh':<26}{'n':>5}  {'oracle p50':>10}{'p95':>9}{'max':>9}  {'native p50':>10}{'p95':>9}{'max':>9}  {'speedup p50':>11}{'min':>7}")
        for mesh, row in report["stats"][kind].items():
            o, n = row["oracle"], row["native"]
            lines.append(f"{mesh:<26}{row['n']:>5}  {_ms(o['p50']):>10}{_ms(o['p95']):>9}{_ms(o['max']):>9}  {_ms(n['p50']):>10}{_ms(n['p95']):>9}{_ms(n['max']):>9}  {row['speedup_p50']:>10.1f}x{row['speedup_min']:>6.1f}x")
        lines.append(f"  native split p50/p95/max (ms), ALL:  " + "   ".join(f"{name} {_ms(row['p50']).strip()}/{_ms(row['p95']).strip()}/{_ms(row['max']).strip()}" for name, row in report["stats"][kind]["ALL"]["parts"].items()))
        lines.append("")
    lines.append("heaviest records (by the oracle's cold time); ms")
    lines.append(f"{'record':<34}{'mesh':<24}{'faces':>6}  {'cold oracle':>11}{'native':>9}{'x':>7}   {'warm oracle':>11}{'native':>9}{'x':>7}   warm split: " + " ".join(PARTS))
    for row in report["heaviest"]:
        split = " ".join(_ms(row["warm_parts"][name]).strip() for name in PARTS)
        lines.append(
            f"{row['id']:<34}{row['mesh']:<24}{row['faces'] or 0:>6}  {_ms(row['cold_oracle']):>11}{_ms(row['cold_native']):>9}{row['cold_oracle'] / row['cold_native']:>6.1f}x   "
            f"{_ms(row['warm_oracle']):>11}{_ms(row['warm_native']):>9}{row['warm_oracle'] / row['warm_native']:>6.1f}x   {split}"
        )
    return "\n".join(lines)


def dump_partition(root: Path, row: dict, path: Path) -> None:
    """Разбиение и alpha записи в буфере границы (`cftuv_native.codec`) для `cargo run --release --example coverage_profile -p cftuv-core -- <файл>`.

    Содержимое: `[alpha, [[[ [x, y], ... ], [a, b, c, q]], ... грани]]`, `x`, `y` — суммы корней с типами коэффициентов (`int`/`Fraction`).
    """

    from cftuv_native import codec

    record = nc.read_record(root / row["path"])
    partition, alpha = nc.decode_call(nc.OP_COVERAGE, record.call_blob, None, None).args
    faces = [[[list(point) for point in face.points], [face.line.a, face.line.b, face.line.c, face.line.q]] for face in partition.faces]
    path.write_bytes(codec.encode_value([alpha, faces]))
    print(f"dumped {row['id']}: {len(faces)} faces, {path.stat().st_size} bytes -> {path}")


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dump", default="", help="id записи: записать её разбиение и alpha в файл `--dump-to` и выйти")
    parser.add_argument("--dump-to", default="")
    parser.add_argument("--corpus", default="")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--meshes", default="")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--out", default="")
    return parser.parse_args()


def main() -> int:
    args = _arguments()
    extension = load_extension()
    root = Path(args.corpus) if args.corpus else nb.default_corpus()
    index = nc.load_index(root)
    if args.dump:
        dump_partition(root, next(row for row in index["records"] if row["id"] == args.dump), Path(args.dump_to))
        return 0
    meshes = set(filter(None, args.meshes.split(",")))
    rows = [row for row in index["records"] if row["op"] == nc.OP_COVERAGE and not row.get("derived") and (not meshes or row["mesh"] in meshes) and not row["exception"]]
    rows = rows[: args.limit] if args.limit else rows
    results = []
    for number, row in enumerate(rows, 1):
        results.append(measure_record(extension, root, row, args.repeat))
        if results[-1]["differences"]:
            first = results[-1]["differences"][0]
            print(f"MISMATCH {row['id']} {row['mesh']} step {first['step']} repeat {first['repeat']}: {first['fields']}", flush=True)
        if number % 50 == 0:
            print(f"  {number}/{len(rows)} records", flush=True)
    mismatches = [item for item in results if item["differences"]]
    report = {
        "python": sys.version.split()[0], "native_version": extension.native_version(), "corpus": str(root), "records": len(results), "repeat": args.repeat,
        "calls_checked": sum(len(item["steps"]) for item in results) * args.repeat, "stats": aggregate(results), "heaviest": heaviest(results),
        "mismatches": [{"id": item["id"], "differences": item["differences"]} for item in mismatches], "per_record": results,
    }
    if args.out:
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(text_report(report))
    print(f"NATIVE_BENCH_NATIVE_{'FAILED' if mismatches else 'OK'} records={len(results)} native_calls_checked={report['calls_checked']} mismatches={len(mismatches)}")
    return 1 if mismatches else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
