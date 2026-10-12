"""Корпус W1 цепи MATERIALIZE (`mesh.exact_mesh`, `mesh.canonical_triangles`): воспроизведение записей эталоном и нативной операцией, точное сравнение.

    python tools/native_materialize_w1_corpus.py replay <каталог корпуса> [--limit N] [--ops exact_mesh,canonical_triangles]
    python tools/native_materialize_w1_corpus.py bench  <каталог корпуса> [--repeat 5]

Запись W1 (`native_corpus.Recorder`, `tools/native_corpus_export.py --operations w1`) держит вход вызова (слитые грани домена, таблица станций, строки `notes`), состояние процесса ДО и ПОСЛЕ
(бюджет, шесть статей, память канонизации с порядком, `SIGN_COUNTS`) и исход (результат либо исключение). Воспроизведение: эталон на состоянии ДО (он обязан дать записанное: детерминизм
и неизменность эталона), затем нативная операция на том же состоянии; `native_corpus.compare_outcomes` сверяет результат (тип и значение каждого узла: `int` и `Fraction` различны, `float` по `hex`),
исключение вместе с пробегами конфликта станций, цену, память, знаки и строки `notes`. Отказ ПОРТА (`NATIVE_REFUSALS`) считается отдельно и обязан оставить состояние как до вызова.

`bench` меряет время эталона и нативной операции на записях ТЕПЛЫМИ (повтор того же вызова на восстановленном состоянии), с разбивкой нативного времени на преобразование входа, счёт и построение
результата (`last_w1_timings`).
"""

from __future__ import annotations

import argparse
import statistics
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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("replay", "bench"))
    parser.add_argument("corpus")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--repeat", type=int, default=5)
    parser.add_argument("--ops", default=",".join(nc.W1_OPERATIONS))
    arguments = parser.parse_args(argv)
    ops = tuple(item for item in arguments.ops.split(",") if item)
    root = Path(arguments.corpus)
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
