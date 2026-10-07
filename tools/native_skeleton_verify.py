"""Воспроизведение корпуса скелета эталоном и доказательство детерминизма: повтор, зерно хэша, версия питона, аудит каноники.

    python tools/native_skeleton_verify.py replay --corpus field|synthetic|<каталог> [--repeat 2] [--audit keep|off|on]
        [--hash-seed-note] [--meshes a,b] [--ids 000001,...] [--limit N] [--max-seconds S] [--out digests.json]
    python tools/native_skeleton_verify.py compare a.json b.json [...]
    python tools/native_skeleton_verify.py matrix --corpus field|synthetic [--out DIR] [--max-seconds S]

`replay`: каждая запись воспроизводится эталоном (ядро питона) от записанного состояния ДО и сверяется с записанным исходом точно
(`native_corpus.compare_outcomes`: результат каноническим кодом, исключение, цена, память с порядком, знаки, неоплаченное). `--repeat N`
повторяет воспроизведение в этом же процессе: все повторы обязаны дать одну подпись (`outcome_digest`). Выход — JSON подписей по записям
и свод по мешам (число, секунды). `compare`: подписи нескольких прогонов обязаны совпасть запись в запись (прогоны разных зёрен хэша
и версий питона). `matrix`: прогон на питоне dev-venv (3.13) и питоне Blender (3.11) с двумя зёрнами хэша (0 и 12345 — одно общее, другое
разное), затем `compare` всех файлов; код возврата 0 только при нуле расхождений. Вывод `replay` называет записи, чей исход зависит от зерна хэша.

`--audit`: аудит каноники сумм корней в процессе воспроизведения: `keep` — как записано, `off` — продуктовое значение, `on` — тестовое;
исход (всё, что сравнивается) обязан от него не зависеть.
"""

from __future__ import annotations

import argparse
import dataclasses
import gc
import json
import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))  # `PYTHONSAFEPATH=1` каталог скрипта в путь не кладёт


def _third_party_path() -> None:
    """Питон Blender не несёт `mpmath`/`sympy` (их тянет `native_corpus` через модули ядра): каталог модулей Blender — В КОНЕЦ пути."""

    try:
        import mpmath  # noqa: F401
    except ModuleNotFoundError:
        modules = Path(os.environ.get("APPDATA", "")) / "Blender Foundation" / "Blender" / "4.5" / "scripts" / "modules"
        if modules.is_dir():
            sys.path.append(str(modules))


_third_party_path()

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402


def corpus_root(spec: str) -> Path:
    """`field`/`synthetic` — корпус под ядро процесса; иначе путь к каталогу корпуса."""

    if spec in sc.KINDS:
        found = sc.matching(spec)
        if found is None:
            raise SystemExit(f"NATIVE_SKELETON_VERIFY_FAILED {sc.describe_missing(spec)}")
        return found
    return Path(spec)


def selected_rows(root: Path, arguments) -> list:
    rows = sc.rows_of(root)
    if arguments.meshes:
        wanted = set(arguments.meshes.split(","))
        rows = [row for row in rows if row["mesh"] in wanted]
    if arguments.ids:
        wanted_ids = set(arguments.ids.split(","))
        rows = [row for row in rows if row["id"].split("-")[0] in wanted_ids or row["id"] in wanted_ids]
    if arguments.max_seconds:
        rows = [row for row in rows if row["seconds"] <= arguments.max_seconds]
    return rows[: arguments.limit] if arguments.limit else rows


def _audited(before: nc.StateV1, audit: str) -> nc.StateV1:
    if audit == "keep":
        return before
    return dataclasses.replace(before, canonical_audit=(audit == "on"))


def replay_one(root: Path, row: dict, repeat: int, audit: str) -> dict:
    """Одна запись: `repeat` воспроизведений, подписи, расхождения с записанным исходом, секунды."""

    record = sc.read(root, row)
    before, expected = record.before(), record.expected()
    reference = nc.outcome_digest(record.op, before, expected)
    state = _audited(before, audit)
    digests, seconds, differences = [], [], []
    for index in range(repeat):
        call = nc.prepare_call(record.op, record.call_blob, state)
        gc.collect()
        outcome = nc.execute(call)
        seconds.append(outcome.seconds)
        digests.append(nc.outcome_digest(record.op, before, outcome))
        if digests[-1] != reference:
            found = nc.compare_outcomes(record.op, before, expected, outcome)
            differences.append({"repeat": index + 1, "fields": [str(item) for item in found] or ["digest differs, comparison empty"]})
    return {
        "id": row["id"], "mesh": row["mesh"], "outcome": row["outcome"], "derived": row.get("derived") is not None,
        "expected_digest": reference, "digests": digests, "stable": len(set(digests)) == 1, "equal": not differences,
        "recorded_seconds": row["seconds"], "seconds": seconds, "differences": differences,
    }


def summary_of(results: list) -> dict:
    by_mesh: dict = {}
    for item in results:
        entry = by_mesh.setdefault(item["mesh"], {"records": 0, "seconds_min": 0.0, "recorded_seconds": 0.0, "unequal": 0, "unstable": 0})
        entry["records"] += 1
        if not item["derived"]:
            entry["seconds_min"] += min(item["seconds"])
            entry["recorded_seconds"] += item["recorded_seconds"]
        entry["unequal"] += int(not item["equal"])
        entry["unstable"] += int(not item["stable"])
    return {
        "python": sys.version.split()[0],
        "hash_seed": os.environ.get("PYTHONHASHSEED"),
        "records": len(results),
        "unequal": sum(1 for item in results if not item["equal"]),
        "unstable": sum(1 for item in results if not item["stable"]),
        "by_mesh": by_mesh,
    }


def command_replay(arguments) -> int:
    root = corpus_root(arguments.corpus)
    rows = selected_rows(root, arguments)
    results = []
    for number, row in enumerate(rows, 1):
        item = replay_one(root, row, arguments.repeat, arguments.audit)
        results.append(item)
        if not item["equal"] or not item["stable"]:
            print(f"DIFFERS {item['id']} {item['mesh']}: {item['differences'][:1]}", flush=True)
        if number % 25 == 0:
            print(f"  replayed {number}/{len(rows)}", flush=True)
    summary = summary_of(results)
    if arguments.out:
        Path(arguments.out).write_text(json.dumps({"summary": summary, "records": results}, indent=0), encoding="utf-8")
    print(json.dumps(summary))
    ok = not summary["unequal"] and not summary["unstable"]
    print(("NATIVE_SKELETON_REPLAY_OK" if ok else "NATIVE_SKELETON_REPLAY_FAILED"), summary["records"], summary["python"])
    return 0 if ok else 1


def compare_files(paths) -> dict:
    """Подписи нескольких прогонов запись в запись: `{расхождения: [...], записей: n, прогонов: m}` (общие записи, остальные названы)."""

    runs = [json.loads(Path(path).read_text(encoding="utf-8")) for path in paths]
    tables = [{item["id"]: item for item in run["records"]} for run in runs]
    common = set(tables[0])
    for table in tables[1:]:
        common &= set(table)
    differing = []
    for record_id in sorted(common):
        values = {tuple(table[record_id]["digests"]) for table in tables}
        if len({digest for value in values for digest in value}) > 1:
            differing.append({"id": record_id, "mesh": tables[0][record_id]["mesh"]})
    return {
        "runs": [run["summary"]["python"] + "/seed=" + str(run["summary"]["hash_seed"]) for run in runs],
        "records_common": len(common),
        "only_in_some": sorted(set().union(*tables) - common),
        "differing": differing,
    }


def command_compare(arguments) -> int:
    report = compare_files(arguments.files)
    print(json.dumps(report, indent=1))
    ok = not report["differing"] and not report["only_in_some"]
    print("NATIVE_SKELETON_COMPARE_OK" if ok else "NATIVE_SKELETON_COMPARE_FAILED", report["records_common"], len(report["runs"]))
    return 0 if ok else 1


def _interpreters() -> list:
    import native_catchup as catchup

    return [("py313", catchup.venv_python(), {}), ("py311", catchup.BLENDER_PYTHON, {"PYTHONPATH": str(catchup.PY311_SITE)})]


def command_matrix(arguments) -> int:
    """Четыре прогона (питон 3.13 и 3.11 x зёрна хэша 0 и 12345) параллельно, затем `compare`; `--audit` задаёт аудит каноники в прогонах (по умолчанию как записано)."""

    out = Path(arguments.out)
    out.mkdir(parents=True, exist_ok=True)
    running = []
    for label, interpreter, environment in _interpreters():
        for seed in ("0", "12345"):
            path = out / f"digests_{label}_seed{seed}.json"
            command = [str(interpreter), str(Path(__file__).resolve()), "replay", "--corpus", arguments.corpus, "--repeat", "2", "--audit", arguments.audit, "--out", str(path)]
            if arguments.max_seconds:
                command += ["--max-seconds", str(arguments.max_seconds)]
            merged = {**os.environ, "PYTHONSAFEPATH": "1", "PYTHONHASHSEED": seed, **environment}
            print("+", " ".join(command), flush=True)
            log = open(out / f"replay_{label}_seed{seed}.log", "w", encoding="utf-8")
            running.append((label, seed, path, log, subprocess.Popen(command, env=merged, cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT)))
    for label, seed, _path, log, process in running:
        code = process.wait()
        log.close()
        if code:
            print(f"replay failed: {label} seed {seed} (exit {code}); see {log.name}", flush=True)
    present = [path for _label, _seed, path, _log, _process in running if path.exists()]
    return command_compare(argparse.Namespace(files=present))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    replay = sub.add_parser("replay")
    replay.add_argument("--corpus", required=True)
    replay.add_argument("--repeat", type=int, default=1)
    replay.add_argument("--audit", choices=("keep", "off", "on"), default="keep")
    replay.add_argument("--meshes", default="")
    replay.add_argument("--ids", default="")
    replay.add_argument("--limit", type=int, default=0)
    replay.add_argument("--max-seconds", type=float, default=0.0)
    replay.add_argument("--out", default="")
    compare = sub.add_parser("compare")
    compare.add_argument("files", nargs="+")
    matrix = sub.add_parser("matrix")
    matrix.add_argument("--corpus", required=True)
    matrix.add_argument("--out", required=True)
    matrix.add_argument("--max-seconds", type=float, default=0.0)
    matrix.add_argument("--audit", choices=("keep", "off", "on"), default="keep")
    arguments = parser.parse_args(argv)
    return {"replay": command_replay, "compare": command_compare, "matrix": command_matrix}[arguments.command](arguments)


if __name__ == "__main__":
    sys.exit(main())
