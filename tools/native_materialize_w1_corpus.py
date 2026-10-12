"""Корпус W1 цепи MATERIALIZE (`mesh.exact_mesh`, `mesh.canonical_triangles`): воспроизведение записей эталоном и нативной операцией, точное сравнение.

    python tools/native_materialize_w1_corpus.py replay <каталог корпуса> [--limit N] [--ops exact_mesh,canonical_triangles]
    python tools/native_materialize_w1_corpus.py bench  <каталог корпуса> [--repeat 5]
    python tools/native_materialize_w1_corpus.py seed   <каталог корпуса> [<ещё>...] --out tests/data/native_materialize_w1_kernel_suite.recs.xz [--max-bytes 1200000]

Запись W1 (`native_corpus.Recorder`, `tools/native_corpus_export.py --operations w1`) держит вход вызова (слитые грани домена, таблица станций, строки `notes`), состояние процесса ДО и ПОСЛЕ
(бюджет, шесть статей, память канонизации с порядком, `SIGN_COUNTS`) и исход (результат либо исключение). Воспроизведение: эталон на состоянии ДО (он обязан дать записанное: детерминизм
и неизменность эталона), затем нативная операция на том же состоянии; `native_corpus.compare_outcomes` сверяет результат (тип и значение каждого узла: `int` и `Fraction` различны, `float` по `hex`),
исключение вместе с пробегами конфликта станций, цену, память, знаки и строки `notes`. Отказ ПОРТА (`NATIVE_REFUSALS`) считается отдельно и обязан оставить состояние как до вызова.

`bench` меряет время эталона и нативной операции на записях ТЕПЛЫМИ (повтор того же вызова на восстановленном состоянии), с разбивкой нативного времени на преобразование входа, счёт и построение
результата (`last_w1_timings`).
"""

from __future__ import annotations

import argparse
import lzma
import pickle
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))
sys.path.insert(0, str(ROOT / "tools"))

import native_corpus as nc  # noqa: E402
import cftuv_native as native  # noqa: E402

PORT_REFUSALS = tuple(cls.__qualname__ for cls in native.NATIVE_REFUSALS)


def native_function(op: str):
    """The native entry that answers the call of `op` with the oracle's signature."""

    return native.exact_mesh if op == nc.OP_MESH else native.canonical_triangles


def records(root: Path, ops=nc.W1_OPERATIONS, limit: int | None = None):
    """`(row, Record)` of the corpus in file order, the original records only (not the derived ones), at most `limit`."""

    index = nc.load_index(Path(root))
    count = 0
    for row in index["records"]:
        if row.get("derived") is not None or row["op"] not in ops:
            continue
        yield row, nc.read_record(Path(root) / row["path"])
        count += 1
        if limit is not None and count >= limit:
            return


def state_unchanged(before: nc.StateV1, after: nc.StateV1) -> list:
    """What a refusal of the PORT must not have moved: articles, counters, the four memory tables with their order."""

    return [
        *nc._compare_budget(before, before, after),
        *nc._compare_counters(before, before, after),
        *nc._compare_tables(before, after),
    ]


class Replay:
    """The outcome of ONE record replayed by the oracle and by the native operation."""

    __slots__ = ("op", "oracle_diffs", "native_diffs", "refusal", "refusal_diffs", "oracle_seconds", "native_seconds", "timings", "expected_exception")

    def __init__(self, op: str) -> None:
        self.op = op
        self.oracle_diffs: list = []
        self.native_diffs: list = []
        self.refusal: str | None = None
        self.refusal_diffs: list = []
        self.oracle_seconds = 0.0
        self.native_seconds = 0.0
        self.timings: tuple = ()
        self.expected_exception = None


def replay_record(record: nc.Record, function=None) -> Replay:
    """Oracle then native on the state before the call; the oracle must reproduce the record, the native must reproduce the oracle (or refuse by name, touching nothing)."""

    op = record.op
    function = function or native_function(op)
    before, expected = record.before(), record.expected()
    result = Replay(op)
    result.expected_exception = expected.exception
    oracle = nc.execute(nc.prepare_call(op, record.call_blob, before))
    result.oracle_diffs = nc.compare_outcomes(op, before, expected, oracle)
    result.oracle_seconds = oracle.seconds
    call = nc.prepare_call(op, record.call_blob, before)
    actual = nc.execute(call, function)
    result.native_seconds = actual.seconds
    result.timings = native.last_w1_timings()
    if actual.exception is not None and actual.exception[0] in PORT_REFUSALS:
        result.refusal = f"{actual.exception[0]}: {actual.exception[1][:120]}"
        result.refusal_diffs = state_unchanged(before, actual.after) + (
            [nc.Difference("notes", "changed by a refusal of the port")] if nc.observe(call).get("notes") != nc.observe(nc.prepare_call(op, record.call_blob, before)).get("notes") else []
        )
        return result
    result.native_diffs = nc.compare_outcomes(op, before, oracle, actual)
    return result


def run_replay(root: Path, ops=nc.W1_OPERATIONS, limit: int | None = None, quiet: bool = False) -> dict:
    """Replays the corpus; `{records, ok, mismatches: [...], refusals: {...}, oracle_drift: [...], seconds: {...}}`."""

    summary = {"records": 0, "ok": 0, "mismatches": [], "refusals": {}, "refusal_state_moved": [], "oracle_drift": [], "oracle_seconds": 0.0, "native_seconds": 0.0, "by_op": {}}
    for row, record in records(root, ops, limit):
        replayed = replay_record(record)
        summary["records"] += 1
        per_op = summary["by_op"].setdefault(replayed.op, {"records": 0, "ok": 0, "refused": 0, "raised": 0})
        per_op["records"] += 1
        per_op["raised"] += replayed.expected_exception is not None
        if replayed.oracle_diffs:
            summary["oracle_drift"].append((row["id"], [str(item) for item in replayed.oracle_diffs[:3]]))
        if replayed.refusal is not None:
            summary["refusals"][replayed.refusal] = summary["refusals"].get(replayed.refusal, 0) + 1
            per_op["refused"] += 1
            if replayed.refusal_diffs:
                summary["refusal_state_moved"].append((row["id"], [str(item) for item in replayed.refusal_diffs[:3]]))
            continue
        if replayed.native_diffs:
            summary["mismatches"].append((row["id"], [str(item) for item in replayed.native_diffs[:4]]))
            continue
        summary["ok"] += 1
        per_op["ok"] += 1
        summary["oracle_seconds"] += replayed.oracle_seconds
        summary["native_seconds"] += replayed.native_seconds
        if not quiet and summary["records"] % 50 == 0:
            print(f"  .. {summary['records']} records", flush=True)
    return summary


def run_bench(root: Path, repeat: int = 5, ops=nc.W1_OPERATIONS, limit: int | None = None) -> dict:
    """Warm timings per operation: the best of `repeat` replays of every record, oracle and native, plus the native time split (seconds)."""

    totals = {op: {"oracle": 0.0, "native": 0.0, "seam": 0.0, "arguments": 0.0, "compute": 0.0, "result": 0.0, "log": 0.0, "records": 0} for op in ops}
    for _row, record in records(root, ops, limit):
        op = record.op
        before = record.before()
        best_oracle = best_native = float("inf")
        best_split: tuple = ()
        for _ in range(repeat):
            call = nc.prepare_call(op, record.call_blob, before)
            started = time.perf_counter()
            try:
                nc.invoke(call)
            except Exception:  # noqa: BLE001 - an exception of the oracle is its timed outcome too
                pass
            best_oracle = min(best_oracle, time.perf_counter() - started)
        refused = False
        for _ in range(repeat):
            call = nc.prepare_call(op, record.call_blob, before)
            started = time.perf_counter()
            try:
                nc.invoke(call, native_function(op))
            except native.NATIVE_REFUSALS:
                refused = True
                break
            except Exception:  # noqa: BLE001
                pass
            elapsed = time.perf_counter() - started
            if elapsed < best_native:
                best_native, best_split = elapsed, native.last_w1_timings()
        if refused or not best_split:
            continue
        row = totals[op]
        row["records"] += 1
        row["oracle"] += best_oracle
        row["native"] += best_native
        # (sync in, native call, post, total, tables, arguments, compute, result, memory log) nanoseconds
        row["seam"] += best_split[3] / 1e9
        row["arguments"] += (best_split[4] + best_split[5]) / 1e9
        row["compute"] += best_split[6] / 1e9
        row["result"] += best_split[7] / 1e9
        row["log"] += best_split[8] / 1e9
    return totals


SEED_SCHEMA = "cftuv.native-materialize-w1-seed.v1"


def features(row: dict, record: nc.Record) -> frozenset:
    """What a record shows that another may not: the operation, the way it ended, every number of the tally, and (for a budget) the unit where it was cut."""

    expected = record.expected()
    found = {("op", record.op), ("label", str(row.get("label", "")).split("-")[0])}
    if expected.exception is not None:
        found.add(("raised", expected.exception[0], expected.exception[1][:70] if "EXHAUSTED" not in expected.exception[1] else "exhausted"))
        if "operation=" in expected.exception[1]:  # a budget cut at a charge site: the operation that was charged, and the stage of the cap
            found.add(("exhausted", expected.exception[1].split("operation=", 1)[1].split()[0]))
    if expected.result is not None and record.op == nc.OP_MESH:
        found.update(("tally", name) for name, _value in expected.result.tally.items())
        found.add(("polygons", min(8, max((len(polygon) for face in expected.result.polygons for polygon in face), default=0))))
        found.add(("notes", bool(expected.observed.get("notes") not in (None, "[]"))))
    if expected.result is not None and record.op == nc.OP_CANONICAL:
        found.add(("faces", min(6, len(expected.result))))
    return frozenset(found)


def write_seed(roots: list, out: Path, max_bytes: int) -> dict:
    """The smallest records that between them show every feature of the corpora `roots` (greedy cover, cheapest first), as one xz file: the tracked seed of the strict tests."""

    candidates = []
    for root in roots:
        for row, record in records(Path(root), nc.W1_OPERATIONS):
            candidates.append((row.get("bytes", 0), row["id"], features(row, record), row, record))
    index = nc.load_index(Path(roots[0]))
    for root in roots[1:]:
        index["records"] = index["records"] + nc.load_index(Path(root))["records"]
    # derived records (budgets) are rows too; they carry `derived`, which `records()` skips, so they come in here
    for root in roots:
        for row in nc.load_index(Path(root))["records"]:
            if row.get("derived") is not None and row["op"] in nc.W1_OPERATIONS:
                record = nc.read_record(Path(root) / row["path"])
                candidates.append((row.get("bytes", 0), row["id"], features(row, record), row, record))
    covered: set = set()
    chosen = []
    size = 0
    pool = sorted(candidates, key=lambda item: (item[0], item[1]))
    while True:
        best = None
        for item in pool:
            gain = len(item[2] - covered)
            if gain and (best is None or gain / max(item[0], 1) > best[0]):
                best = (gain / max(item[0], 1), item)
        if best is None:
            break
        item = best[1]
        if size + item[0] > max_bytes:
            pool = [other for other in pool if other is not item]
            continue
        chosen.append(item)
        covered |= item[2]
        size += item[0]
        pool = [other for other in pool if other is not item]
    body = [{"meta": item[4].meta, "payload": item[4].payload} for item in chosen]
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(lzma.compress(pickle.dumps({"schema": SEED_SCHEMA, "records": body}, protocol=5), format=lzma.FORMAT_XZ, preset=9))
    return {"records": len(chosen), "features": len(covered), "uncovered": len({feature for item in candidates for feature in item[2]} - covered), "bytes": out.stat().st_size}


def read_seed(path: Path) -> list:
    """The records of a seed file as `native_corpus.Record`."""

    document = pickle.loads(lzma.decompress(Path(path).read_bytes()))
    if document.get("schema") != SEED_SCHEMA:
        raise nc.CorpusError(f"{path}: not a W1 seed (schema {document.get('schema')!r})")
    return [nc.Record(item["meta"], item["payload"]) for item in document["records"]]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("replay", "bench", "seed"))
    parser.add_argument("corpus", nargs="+")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--max-bytes", type=int, default=1_200_000)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--ops", default=",".join(nc.W1_OPERATIONS))
    arguments = parser.parse_args(argv)
    ops = tuple(item for item in arguments.ops.split(",") if item)
    if arguments.command == "seed":
        print(write_seed([Path(item) for item in arguments.corpus], arguments.out, arguments.max_bytes))
        return 0
    root = Path(arguments.corpus[0])
    if arguments.command == "replay":
        summary = run_replay(root, ops, arguments.limit)
        print(f"records {summary['records']} ok {summary['ok']} mismatches {len(summary['mismatches'])} refused {sum(summary['refusals'].values())} oracle_drift {len(summary['oracle_drift'])} state_moved {len(summary['refusal_state_moved'])}")
        for name, per_op in summary["by_op"].items():
            print(f"  {name}: {per_op}")
        for reason, count in summary["refusals"].items():
            print(f"  refusal x{count}: {reason}")
        for record_id, diffs in summary["mismatches"][:10]:
            print(f"  MISMATCH {record_id}: {diffs}")
        for record_id, diffs in summary["oracle_drift"][:5]:
            print(f"  ORACLE_DRIFT {record_id}: {diffs}")
        if summary["native_seconds"]:
            print(f"  replay seconds oracle {summary['oracle_seconds']:.3f} native {summary['native_seconds']:.3f} ratio {summary['oracle_seconds'] / summary['native_seconds']:.2f}")
        return 0 if not (summary["mismatches"] or summary["refusal_state_moved"] or summary["oracle_drift"]) else 1
    totals = run_bench(root, arguments.repeat, ops, arguments.limit)
    for op, row in totals.items():
        if not row["records"]:
            continue
        print(
            f"{op}: {row['records']} records | oracle {row['oracle'] * 1000:.1f} ms native {row['native'] * 1000:.1f} ms ratio {row['oracle'] / row['native']:.2f} | "
            f"seam {row['seam'] * 1000:.1f} (arguments {row['arguments'] * 1000:.1f}, compute {row['compute'] * 1000:.1f}, result {row['result'] * 1000:.1f}, log {row['log'] * 1000:.1f}) ms"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
