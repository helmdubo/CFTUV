"""Замер целой snap_embedding_certificate с конверсией, без memo-обёртки.

Корпус декодируется до таймера; внутри таймера публичный вызов шима, включая
проверку пина, чтение Python-объектов, Rust и создание Python-сертификата.
"""

from __future__ import annotations

import argparse
import json
import statistics
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))

import cftuv_native
import native_embedding_corpus as corpus


def quantiles(values):
    ordered = sorted(values)
    return {"p50": statistics.median(ordered), "p95": ordered[max(0, (95 * len(ordered) + 99) // 100 - 1)], "max": ordered[-1]}


def elapsed(function, arguments):
    start = time.perf_counter_ns()
    result = function(*arguments)
    return time.perf_counter_ns() - start, result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--corpus", type=Path, default=corpus.corpus_directory())
    parser.add_argument("--repeat", type=int, default=7)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stress-file", type=Path)
    parser.add_argument("--cold", action="store_true", help="отдельные новые процессы для первого вызова на крупнейшей записи каждого источника")
    parser.add_argument("--cold-record", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--cold-id", help=argparse.SUPPRESS)
    parser.add_argument("--cold-backend", choices=("python", "native"), help=argparse.SUPPRESS)
    options = parser.parse_args()
    if options.cold_record:
        record = next(row for row in corpus.read_records(options.cold_record) if row["id"] == options.cold_id)
        arguments = corpus.decode_call(record)
        function = corpus.ORACLE if options.cold_backend == "python" else cftuv_native.snap_embedding_certificate
        # Ни status/build_id, ни пробного вызова до первого измерения.
        duration, found = elapsed(function, arguments)
        assert corpus.certificate_fields(found) == corpus.answer_of(record)[1]
        print(json.dumps({"ms": duration / 1e6, "backend": options.cold_backend, "id": options.cold_id}))
        return 0
    if options.repeat < 1:
        parser.error("--repeat must be positive")
    report = {"python": sys.version, "native_origin": cftuv_native.__file__, "build_id": cftuv_native.native_build_id(), "oracle_digest": corpus.oracle_digest(), "repeat": options.repeat, "timer": "warm whole leaf including conversion; corpus decode and output comparison excluded; cold first call in a fresh process when requested (startup/import/decode excluded)", "meshes": {}}
    paths = sorted((options.corpus / "field").glob("*.recs.xz"))
    if options.stress_file:
        paths.append(options.stress_file)
    for path in paths:
        records = corpus.read_records(path)
        rows = []
        for record in records:
            if record["answer"][0] != "ok":
                raise ValueError(f"benchmark requires successful field input: {record['id']}")
            arguments = corpus.decode_call(record)
            wanted = corpus.ORACLE(*arguments)
            assert cftuv_native.snap_embedding_certificate(*arguments) == wanted
            python_times, native_times = [], []
            for iteration in range(options.repeat):
                calls = ((corpus.ORACLE, python_times), (cftuv_native.snap_embedding_certificate, native_times))
                for function, times in (calls if iteration % 2 == 0 else calls[::-1]):
                    duration, found = elapsed(function, arguments)
                    assert found == wanted
                    times.append(duration / 1e6)
            rows.append({"id": record["id"], "source": record["source"], "count": record["count"], "vertices": len(arguments[0]), "alias": arguments[0] is arguments[1], "violations": {name: getattr(wanted, name) for name in corpus.VIOLATION_FIELDS}, "python_ms": python_times, "native_ms": native_times})
        med_python = [statistics.median(row["python_ms"]) for row in rows]
        med_native = [statistics.median(row["native_ms"]) for row in rows]
        aggregate_python = sum(value * row["count"] for row, value in zip(rows, med_python))
        aggregate_native = sum(value * row["count"] for row, value in zip(rows, med_native))
        summary = {"records": len(rows), "calls": sum(row["count"] for row in rows), "python_ms": quantiles(med_python), "native_ms": quantiles(med_native), "python_all_samples_ms": quantiles([value for row in rows for value in row["python_ms"]]), "native_all_samples_ms": quantiles([value for row in rows for value in row["native_ms"]]), "speedup": quantiles([a / b for a, b in zip(med_python, med_native)]), "aggregate_python_ms": aggregate_python, "aggregate_native_ms": aggregate_native, "aggregate_speedup": aggregate_python / aggregate_native}
        mesh = path.name.removesuffix(".recs.xz")
        if options.cold:
            largest = max(rows, key=lambda row: row["vertices"])
            cold = {}
            for backend in ("python", "native"):
                command = [sys.executable, str(Path(__file__).resolve()), "--out", str(options.out), "--cold-record", str(path), "--cold-id", largest["id"], "--cold-backend", backend]
                cold[backend] = json.loads(subprocess.check_output(command, cwd=ROOT, text=True))
            summary["cold_first_call"] = cold
        report["meshes"][mesh] = {**summary, "rows": rows}
        print(mesh, json.dumps(summary), flush=True)
        options.out.write_text(json.dumps(report, indent=1), encoding="utf-8")
    if not report["meshes"]:
        raise ValueError("no field corpus")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
