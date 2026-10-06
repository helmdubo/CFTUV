"""Покрытие строк эталона резки: какие ветки `clip_geometry` и её швов доходят до исполнения.

Инвентаризация корпусов (полевой и синтетический): «ветка достигнута» — строка тела функции эталона выполнена хоть раз. Трассировщик
смотрит только файлы списка `WATCHED` (остальные кадры отдают `None`), поэтому цена невелика. Вывод — по функциям: число строк,
невыполненные строки; по ним видно, чего корпусу не хватает (свес, шовное подавление, отказы, нецелая карта, сопряжение).

    python tools/native_clip_coverage.py field [--limit N]             # повтор полевых записей `clip_geometry` под трассировкой
    python tools/native_clip_coverage.py synthetic [--out DIR]         # повтор синтетических записей `clip_geometry`
    python tools/native_clip_coverage.py report FILE [FILE ...]        # объединение файлов покрытия, невыполненные строки по функциям

Плагин `native_clip_synthetic` пишет покрытие и во время прогона тестов ядра (переменная `CFTUV_CLIP_COVERAGE=<файл>`).
"""

from __future__ import annotations

import argparse
import ast
import glob
import json
import os
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (str(ROOT / "tools"), str(ROOT / "kernel" / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

KERNEL = ROOT / "kernel" / "src" / "cftuv_envelope"
#: Файл эталона -> функции, чьи строки считаются (`None` — все): то, что делает единица замены.
WATCHED = {
    "materialize/clip.py": None,
    "materialize/clip_cells.py": None,
    "materialize/clip_snap.py": None,
    "materialize/tessellate.py": {"_ear_contains_vertex", "triangulate_exact", "convex_quad_ring", "has_right_turn"},
    "wavefront/faces.py": {"orientation", "shoelace_sign", "doubled_shoelace"},
    "float_filter.py": {"_measure", "centre_and_bound", "orientation_sign", "line_estimate", "polygon_sign"},
    "materialize/lift_surface.py": {
        "_edge_value", "lift_known", "lift_in", "stretch_square", "window", "_window", "line_value", "values_in",
        "replay_lifted", "_lipschitz_square", "_upper_root", "_down", "_up",
    },
    "materialize/offset_normal.py": {"blend", "_length", "_unit", "_dot"},
    "materialize/lift.py": {"sqrt_sum_binary64"},
}


def _normal(path: Path) -> str:
    return os.path.normpath(str(path)).lower()


class Tracer:
    """`sys.settrace` с фильтром по файлам: хранит выполненные номера строк по относительному имени файла."""

    def __init__(self) -> None:
        self.hits: dict[str, set] = defaultdict(set)
        self._names = {_normal(KERNEL / relative): relative for relative in WATCHED}

    def __call__(self, frame, event, arg):
        name = self._names.get(os.path.normpath(frame.f_code.co_filename).lower())
        if name is None:
            return None
        hits = self.hits[name]
        hits.add(frame.f_lineno)

        def local(frame, event, arg):
            if event == "line":
                hits.add(frame.f_lineno)
            return local

        return local

    def start(self) -> None:
        sys.settrace(self)

    def stop(self) -> None:
        sys.settrace(None)

    def dump(self, path: Path) -> None:
        Path(path).write_text(json.dumps({key: sorted(value) for key, value in self.hits.items()}), encoding="utf-8")


def replay(paths, tracer: Tracer) -> int:
    import native_corpus as nc

    count = 0
    for path in paths:
        record = nc.read_record(path)
        call = nc.prepare_call(nc.OP_CLIP, record.call_blob, record.before())
        tracer.start()
        try:
            nc.ORACLE[nc.OP_CLIP](call.args[0], call.budget, **call.kwargs)
        except Exception:  # noqa: BLE001 - отказ эталона тоже путь
            pass
        finally:
            tracer.stop()
        count += 1
    return count


def field_records(limit: int | None = None) -> list:
    base = Path(os.environ.get("CFTUV_NATIVE_CORPUS") or "E:/cftuv_native_corpus") / "c68b1df2"
    paths = sorted(glob.glob(str(base / "records" / "*" / "*clip_geometry*.rec")))
    return paths[:limit] if limit else paths


def synthetic_records(out: Path | None = None) -> list:
    import native_clip_synthetic as synthetic

    base = out or synthetic.default_out()
    return sorted(glob.glob(str(base / "records" / "*" / "*clip_geometry*.rec")))


def merge(files) -> dict:
    merged: dict[str, set] = defaultdict(set)
    for file in files:
        for name, lines in json.loads(Path(file).read_text(encoding="utf-8")).items():
            merged[name].update(lines)
    return merged


def uncovered(hits: dict) -> dict:
    """`{файл: [(функция, строка def, число операторов, [невыполненные строки])]}` по функциям списка `WATCHED`."""

    report: dict = {}
    for relative, wanted in WATCHED.items():
        source = (KERNEL / relative).read_text(encoding="utf-8")
        tree = ast.parse(source)
        covered = set(hits.get(relative, ()))
        rows = []
        for node in ast.walk(tree):
            if not isinstance(node, ast.FunctionDef) or (wanted is not None and node.name not in wanted):
                continue
            skip = set()
            first = node.body[0] if node.body else None
            if isinstance(first, ast.Expr) and isinstance(getattr(first, "value", None), ast.Constant) and isinstance(first.value.value, str):
                skip = set(range(first.lineno, first.end_lineno + 1))
            statements = set()
            for sub in ast.walk(node):
                if isinstance(sub, ast.stmt) and sub is not node:
                    if isinstance(sub, ast.Expr) and isinstance(getattr(sub, "value", None), ast.Constant) and isinstance(sub.value.value, str):
                        continue
                    statements.add(sub.lineno)
            statements -= skip
            rows.append((node.name, node.lineno, len(statements), sorted(line for line in statements if line not in covered)))
        report[relative] = sorted(rows, key=lambda row: row[1])
    return report


def summary(report: dict) -> dict:
    total = sum(row[2] for rows in report.values() for row in rows)
    missed = sum(len(row[3]) for rows in report.values() for row in rows)
    return {"statements": total, "uncovered": missed, "covered_percent": round(100.0 * (total - missed) / total, 1) if total else 0.0}


def print_report(report: dict, only_missing: bool = True) -> None:
    for relative, rows in report.items():
        print(f"== {relative}")
        for name, line, count, missing in rows:
            if only_missing and not missing:
                continue
            print(f"  {name:30s} L{line:<5d} statements={count:3d} uncovered={len(missing):3d} {missing}")
    print(summary(report))


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("field", "synthetic", "report"))
    parser.add_argument("files", nargs="*")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--dump", type=Path, default=None)
    arguments = parser.parse_args(argv)
    if arguments.command == "report":
        print_report(uncovered(merge(arguments.files)))
        return 0
    tracer = Tracer()
    paths = field_records(arguments.limit) if arguments.command == "field" else synthetic_records(arguments.out)
    count = replay(paths, tracer)
    if arguments.dump:
        tracer.dump(arguments.dump)
    print(f"replayed {count} records")
    print_report(uncovered(dict(tracer.hits)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
