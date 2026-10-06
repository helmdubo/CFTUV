"""Эталонный замер ядра питона на корпусе вызовов (`tools/native_corpus_export.py`): p50/p95/max по операции и мешу, тяжёлые домены.

    set PYTHONSAFEPATH=1
    python tools/native_bench.py [--corpus <каталог корпуса>] [--repeat 5] [--ops coverage_at,clip_geometry] \\
        [--meshes building,...] [--limit N] [--out <json>]
    "C:/Program Files/Blender Foundation/Blender 4.5/4.5/python/bin/python.exe" tools/native_bench.py ...   (питон 3.11 продукта)

Для каждой записи состояние процесса восстанавливается (`native_corpus.prepare_call`: память канонизации с порядком, бюджет,
знаки, неоплаченное, `store`; чистые кэши сброшены), вход собирается из пикла заново, операция эталона (ядро питона) зовётся
`--repeat` раз, а время берётся как медиана повторов (`time.perf_counter` вокруг одной только операции). Каждый повтор сверяется с
ЗАПИСАННЫМ исходом точно (`outcome_digest`: результат, исключение, цена, память с порядком, знаки, неоплаченное, `store`);
расхождение — проблема детерминизма и называется (запись, повтор, поле). Код возврата 0 только при нуле расхождений.

Версия питона в исход НЕ входит: сортировка `clip._ordered` и свёртка нормали смещения — явная семантика CPython 3.11 ядра (`_cpython311.py`), и запись,
снятая под питоном 3.11 (Blender), под 3.13 даёт тот же результат, ту же цену и те же счётчики; расхождение по любому полю — ошибка детерминизма.

Сторонние пакеты ядра (`mpmath`, `sympy`) под питоном Blender 4.5 не вшиты: каталог пользовательских модулей Blender
добавляется В КОНЕЦ `sys.path` (установленная копия ядра там не должна перекрыть дерево репозитория).
"""

from __future__ import annotations

import argparse
import gc
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт


def _third_party_path() -> None:
    try:
        import mpmath  # noqa: F401
    except ModuleNotFoundError:
        modules = Path(os.environ.get("APPDATA", "")) / "Blender Foundation" / "Blender" / "4.5" / "scripts" / "modules"
        if modules.is_dir():
            sys.path.append(str(modules))


_third_party_path()

import native_corpus as nc  # noqa: E402


def percentile(values: list, fraction: float) -> float:
    """Ранг по ближайшему: `ceil(доля * n)`-й по возрастанию."""

    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def summarize(values: list) -> dict:
    return {
        "n": len(values),
        "p50": percentile(values, 0.5),
        "p95": percentile(values, 0.95),
        "max": max(values),
        "total": sum(values),
    }


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--corpus", default="")
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--ops", default=",".join(nc.OPERATIONS))
    parser.add_argument("--meshes", default="")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--out", default="")
    parser.add_argument("--allow-identity-mismatch", action="store_true")
    return parser.parse_args()


def default_corpus() -> Path:
    """Корпус под отпечаток кода ЭТОГО ядра (HEAD ветки уходит вперёд коммитами нативного кода, ядро питона — нет); нет его — каталог по HEAD (замер откажет)."""

    return nc.matching_corpus() or nc.corpus_directory(nc.git_head())


def measure_record(root: Path, row: dict, repeat: int) -> dict:
    """Один повтор за другим: время операции и сверка с записью. Расхождения — по полям, с номером повтора."""

    record = nc.read_record(root / row["path"])
    op, before, expected = record.op, record.before(), record.expected()
    reference = nc.outcome_digest(op, before, expected)
    seconds: list = []
    differences: list = []
    for index in range(repeat):
        call = nc.prepare_call(op, record.call_blob, before)
        gc.collect()
        outcome = nc.execute(call)
        seconds.append(outcome.seconds)
        if nc.outcome_digest(op, before, outcome) != reference:
            found = nc.compare_outcomes(op, before, expected, outcome)
            differences.append({"repeat": index + 1, "fields": [str(item) for item in found] or ["digest differs, comparison empty"]})
    ordered = sorted(seconds)
    return {
        "id": row["id"], "op": op, "mesh": row["mesh"], "alpha": row["alpha"], "patch_id": row["patch_id"],
        "derived": row.get("derived") is not None, "outcome": row["outcome"],
        "recorded_seconds": row["seconds"], "median": ordered[len(ordered) // 2], "min": ordered[0], "first": seconds[0],
        "seconds": seconds, "differences": differences,
    }


def aggregate(results: list) -> dict:
    """`{операция: {меш | "ALL": {n, p50, p95, max, total, recorded: {...}}}}` по медианам повторов и по записанным секундам."""

    table: dict = {}
    for result in results:
        if result["derived"]:
            continue  # производные записи (урезанный потолок) проверяются, но в статистику времени не входят
        for mesh in (result["mesh"], "ALL"):
            table.setdefault(result["op"], {}).setdefault(mesh, []).append(result)
    return {
        op: {
            mesh: {**summarize([item["median"] for item in items]), "recorded": summarize([item["recorded_seconds"] for item in items])}
            for mesh, items in sorted(meshes.items())
        }
        for op, meshes in table.items()
    }


def heaviest_domains(index: dict, results: list) -> list:
    """Домены, ради которых сделана нативная сборка: `rounded_wall_noise_top` патч 2 (каждая ширина) и пять самых тяжёлых `building`.

    `building` ранжируется дважды: по времени домена и по времени двух операций (`ranked_by`): тяжёлый домен не обязан быть
    тяжёлым для них.
    """

    by_domain: dict = {}
    for item in results:
        if not item["derived"]:
            by_domain.setdefault((item["mesh"], item["alpha"], item["patch_id"]), {})[item["op"]] = item
    chosen: dict = {}
    for row in index["domains"]:
        if row["mesh"] == "rounded_wall_noise_top" and row["patch_id"] == 2:
            chosen[(row["mesh"], row["alpha"], row["patch_id"])] = (row, ["rounded_wall_noise_top patch 2"])
    building = [row for row in index["domains"] if row["mesh"] == "building"]
    for label, key in (("building top5 by domain time", lambda row: -row["seconds_net"]),
                       ("building top5 by coverage+clip time", lambda row: -sum(row["op_seconds"].values()))):
        for row in sorted(building, key=key)[:5]:
            chosen.setdefault((row["mesh"], row["alpha"], row["patch_id"]), (row, []))[1].append(label)
    found = []
    for identity, (row, labels) in chosen.items():
        replayed = by_domain.get(identity, {})
        recorded_ops = sum(row["op_seconds"].values())
        stages = sorted(row.get("stages", {}).items(), key=lambda item: -item[1])[:4]
        found.append({
            "mesh": row["mesh"], "patch_id": row["patch_id"], "alpha": row["alpha"], "domain_seconds": row["seconds_net"],
            "recorded": {op: row["op_seconds"][op] for op in nc.OPERATIONS}, "ranked_by": labels,
            "recorded_share": recorded_ops / row["seconds_net"] if row["seconds_net"] else 0.0,
            "replayed": {op: replayed[op]["median"] for op in nc.OPERATIONS if op in replayed},
            "top_stages": [[name, round(seconds, 4)] for name, seconds in stages],
        })
    return found


def mismatch_fields(mismatches: list) -> dict:
    """Расхождения по именам полей: сколько записей затронуто каждым полем (первый повтор записи)."""

    counts: dict = {}
    for item in mismatches:
        for field in {text.split(":")[0] for text in item["differences"][0]["fields"]}:
            counts[field] = counts.get(field, 0) + 1
    return dict(sorted(counts.items()))


def _text_table(report: dict) -> str:
    lines = [f"python {report['python']}  corpus {report['corpus']}  records {report['records']}  repeat {report['repeat']}", ""]
    lines.append(f"{'op':<14}{'mesh':<26}{'n':>5}{'p50 ms':>10}{'p95 ms':>10}{'max ms':>10}{'total s':>9}   recorded p50/p95/max ms (3.11 Blender run)")
    for op, meshes in report["stats"].items():
        for mesh, row in meshes.items():
            recorded = row["recorded"]
            lines.append(
                f"{op:<14}{mesh:<26}{row['n']:>5}{row['p50'] * 1e3:>10.2f}{row['p95'] * 1e3:>10.2f}{row['max'] * 1e3:>10.2f}"
                f"{row['total']:>9.2f}   {recorded['p50'] * 1e3:.2f}/{recorded['p95'] * 1e3:.2f}/{recorded['max'] * 1e3:.2f}"
            )
    lines += ["", "heaviest domains (recorded in Blender; share = (coverage + clip) / domain time of the same run)"]
    lines.append(f"{'mesh':<26}{'patch':>6}{'alpha':>10}{'domain s':>10}{'coverage s':>12}{'clip s':>9}{'share':>8}   replay coverage/clip s   top stages (s)")
    for row in report["heaviest"]:
        replayed = row["replayed"]
        lines.append(
            f"{row['mesh']:<26}{row['patch_id']:>6}{row['alpha']:>10}{row['domain_seconds']:>10.3f}{row['recorded'][nc.OP_COVERAGE]:>12.3f}"
            f"{row['recorded'][nc.OP_CLIP]:>9.3f}{row['recorded_share']:>8.1%}   "
            f"{replayed.get(nc.OP_COVERAGE, float('nan')):.3f}/{replayed.get(nc.OP_CLIP, float('nan')):.3f}   "
            + " ".join(f"{name}={seconds:.2f}" for name, seconds in row["top_stages"])
        )
    return "\n".join(lines)


def main() -> int:
    args = _arguments()
    root = Path(args.corpus) if args.corpus else default_corpus()
    index = nc.load_index(root)
    identity = nc.clip_memo.kernel_code_identity()
    if identity != index["kernel_identity"] and not args.allow_identity_mismatch:
        print(f"NATIVE_BENCH_FAILED kernel identity {identity} differs from the corpus {index['kernel_identity']}")
        return 2
    ops = set(args.ops.split(","))
    meshes = set(filter(None, args.meshes.split(",")))
    rows = [row for row in index["records"] if row["op"] in ops and (not meshes or row["mesh"] in meshes)]
    rows = rows[: args.limit] if args.limit else rows
    results = []
    for number, row in enumerate(rows, 1):
        results.append(measure_record(root, row, args.repeat))
        if results[-1]["differences"]:
            first = results[-1]["differences"][0]
            print(f"MISMATCH {row['id']} {row['mesh']} patch {row['patch_id']} alpha {row['alpha']} "
                  f"(repeats {len(results[-1]['differences'])}/{args.repeat}): {first['fields']}", flush=True)
        if number % 50 == 0:
            print(f"  {number}/{len(rows)} records", flush=True)
    mismatches = [{"id": item["id"], "mesh": item["mesh"], "alpha": item["alpha"], "patch_id": item["patch_id"], "differences": item["differences"]}
                  for item in results if item["differences"]]
    report = {
        "python": sys.version.split()[0], "corpus": str(root), "records": len(results), "derived": sum(1 for item in results if item["derived"]), "repeat": args.repeat,
        "kernel_identity": identity, "corpus_python": index["python"], "stats": aggregate(results),
        "heaviest": heaviest_domains(index, results), "mismatches": mismatches,
        "mismatch_fields": mismatch_fields(mismatches), "per_record": results,
    }
    if args.out:
        Path(args.out).write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(_text_table(report))
    derived = [item for item in results if item["derived"]]
    print(f"derived records (checked, not timed): {len(derived)}, raised {sum(1 for item in derived if item['outcome'].startswith('raised:'))}")
    print(f"mismatch fields: {report['mismatch_fields']}")
    print(f"NATIVE_BENCH_{'FAILED' if mismatches else 'OK'} records={len(results)} mismatches={len(mismatches)}")
    return 1 if mismatches else 0


if __name__ == "__main__":
    code = main()
    if code:
        raise SystemExit(code)
