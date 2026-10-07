"""Покрытие строк эталона скелета: какие ветки 31 файла единицы замены доходят до исполнения на корпусах и тестах ядра.

Инвентаризация корпусов (полевой, синтетический): «ветка достигнута» — оператор тела функции эталона выполнен хоть раз. Считаются только функции,
достижимые из `build_skeleton` ПО ИМЕНАМ (статическое замыкание, `reachable_functions`; завышает, поэтому мёртвое семейство последовательного
применения, до которого `run` не доходит, в «недостижимых» остаётся честно мёртвым). Трассировщик — `sys.monitoring` (питон 3.12+: событие строки
отключается после первого попадания, цена около нуля) либо `sys.settrace` (3.11). Вывод — по функциям: число операторов, невыполненные строки.

    python tools/native_skeleton_coverage.py corpus field|synthetic|<каталог> [--limit N] [--dump FILE]   # повтор записей корпуса под трассировкой
    python tools/native_skeleton_coverage.py report FILE [FILE ...]                                       # объединение файлов, невыполненные строки по функциям
    python tools/native_skeleton_coverage.py named [field|synthetic|<каталог> ...]                        # достигнутые ИМЕНОВАННЫЕ ветки по записям (без повтора)

Плагин `native_skeleton_synthetic` пишет покрытие и во время прогона тестов ядра (`CFTUV_SKELETON_COVERAGE=<файл>`).
"""

from __future__ import annotations

import argparse
import ast
import json
import os
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
for _path in (str(ROOT / "tools"), str(ROOT / "kernel" / "src")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

KERNEL = ROOT / "kernel" / "src" / "cftuv_envelope"
#: Файлы единицы замены (PORT-строки карты порта) плюс числовой фундамент, на котором живёт сопряжение.
WATCHED = (
    "wavefront/skeleton.py", "wavefront/superlevel.py", "wavefront/superlevel_snapshot.py", "wavefront/superlevel_germ.py",
    "wavefront/superlevel_closure.py", "wavefront/superlevel_fixed_point.py", "wavefront/symbolic_superlevel_coordinator.py",
    "wavefront/symbolic_overlay.py", "wavefront/symbolic_component.py", "wavefront/symbolic_mixed_generation.py",
    "wavefront/symbolic_junction_contacts.py", "wavefront/symbolic_junction_normalize.py", "wavefront/symbolic_junction_fixed_point.py",
    "wavefront/symbolic_edge_closure.py", "wavefront/symbolic_edge_fixed_point.py", "wavefront/symbolic_split_endpoint.py",
    "wavefront/symbolic_f0_overlay.py", "wavefront/symbolic_initial_composition.py", "wavefront/symbolic_sparse_ports.py",
    "wavefront/symbolic_runtime_commit.py", "wavefront/motorcycle.py", "wavefront/cell_grid.py", "wavefront/events.py",
    "wavefront/event_time.py", "wavefront/exact_candidate_view.py", "wavefront/candidate_law.py", "wavefront/candidate_refusal.py",
    "wavefront/poststate_span.py", "wavefront/proof.py", "wavefront/exact_identity.py", "wavefront/digest.py",
    "exact_sqrt_sum.py",
)
#: Корни статического замыкания: единица замены.
ROOTS = (("wavefront/skeleton.py", "build_skeleton"),)
_IGNORED_NAMES = frozenset({"get", "append", "add", "update", "items", "values", "keys", "pop", "extend", "sort", "setdefault", "join", "format", "replace"})


def _normal(path) -> str:
    return os.path.normpath(str(path)).lower()


class Tracer:
    """Выполненные номера строк по относительному имени файла из `WATCHED`."""

    def __init__(self) -> None:
        self.hits: dict[str, set] = defaultdict(set)
        self._names = {_normal(KERNEL / relative): relative for relative in WATCHED}
        self._monitoring = hasattr(sys, "monitoring")

    # ---- sys.monitoring (3.12+) -------------------------------------------------

    def _on_start(self, code, offset):
        if _normal(code.co_filename) in self._names:
            sys.monitoring.set_local_events(self._tool, code, sys.monitoring.events.LINE)
        return sys.monitoring.DISABLE

    def _on_line(self, code, line):
        name = self._names.get(_normal(code.co_filename))
        if name is not None:
            self.hits[name].add(line)
        return sys.monitoring.DISABLE

    def _start_monitoring(self) -> None:
        self._tool = sys.monitoring.COVERAGE_ID
        sys.monitoring.use_tool_id(self._tool, "native_skeleton_coverage")
        sys.monitoring.register_callback(self._tool, sys.monitoring.events.PY_START, self._on_start)
        sys.monitoring.register_callback(self._tool, sys.monitoring.events.LINE, self._on_line)
        sys.monitoring.set_events(self._tool, sys.monitoring.events.PY_START)
        # код, который уже выполнялся до старта (импортированные функции), попадает в трассировку при следующем входе: PY_START не отключён для него
        sys.monitoring.restart_events()

    # ---- sys.settrace (3.11) ------------------------------------------------------

    def __call__(self, frame, event, arg):
        name = self._names.get(_normal(frame.f_code.co_filename))
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
        if self._monitoring:
            self._start_monitoring()
        else:
            sys.settrace(self)

    def stop(self) -> None:
        if self._monitoring:
            sys.monitoring.set_events(self._tool, 0)
            sys.monitoring.register_callback(self._tool, sys.monitoring.events.PY_START, None)
            sys.monitoring.register_callback(self._tool, sys.monitoring.events.LINE, None)
            sys.monitoring.free_tool_id(self._tool)
        else:
            sys.settrace(None)

    def dump(self, path: Path) -> None:
        Path(path).write_text(json.dumps({key: sorted(value) for key, value in self.hits.items()}), encoding="utf-8")


# --------------------------------------------------------------------------
# статическая достижимость и невыполненные операторы
# --------------------------------------------------------------------------


def _function_table() -> dict:
    """`{(файл, квалифицированное имя): (первая строка, имена вызовов, узел)}` по всем функциям файлов `WATCHED`."""

    table: dict = {}

    def visit(node, relative: str, prefix: str) -> None:
        for child in ast.iter_child_nodes(node):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)):
                names = set()
                for sub in ast.walk(child):
                    if isinstance(sub, ast.Name) and isinstance(sub.ctx, ast.Load):
                        names.add(sub.id)
                    elif isinstance(sub, ast.Attribute):
                        names.add(sub.attr)
                table[(relative, prefix + child.name)] = (child.lineno, names - _IGNORED_NAMES, child)
                visit(child, relative, prefix + child.name + ".<locals>.")
            elif isinstance(child, ast.ClassDef):
                visit(child, relative, prefix + child.name + ".")
            else:
                visit(child, relative, prefix)

    for relative in WATCHED:
        visit(ast.parse((KERNEL / relative).read_text(encoding="utf-8")), relative, "")
    return table


def reachable_functions(table: dict | None = None) -> set:
    """Замыкание по ИМЕНАМ от `build_skeleton` (завышение): вызов, ссылка на функцию, атрибут; вложенные функции — вместе с родителем."""

    table = table or _function_table()
    by_simple: dict = defaultdict(list)
    for key in table:
        by_simple[key[1].split(".")[-1]].append(key)
    hooks = ("__init__", "__post_init__", "__hash__", "__lt__")
    seen: set = set()
    stack = [key for key in table if (key[0], key[1]) in set(ROOTS)]
    while stack:
        key = stack.pop()
        if key in seen:
            continue
        seen.add(key)
        for name in table[key][1]:
            stack.extend(target for target in by_simple.get(name, ()) if target not in seen)
            stack.extend(other for other in by_simple.get(hooks[0], ()) if other[1].split(".")[0] == name and other not in seen)
            for hook in hooks[1:]:
                stack.extend(other for other in by_simple.get(hook, ()) if other[1].split(".")[0] == name and other not in seen)
        prefix = key[1] + ".<locals>."
        stack.extend(other for other in table if other[0] == key[0] and other[1].startswith(prefix) and other not in seen)
    return seen


def _own_statements(function) -> set:
    """Номера строк операторов функции БЕЗ вложенных функций и классов (те — отдельные записи) и без строки документации."""

    lines: set = set()

    def walk(parent) -> None:
        for child in ast.iter_child_nodes(parent):
            if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda)):
                continue
            if isinstance(child, ast.stmt):
                is_text = isinstance(child, ast.Expr) and isinstance(getattr(child, "value", None), ast.Constant) and isinstance(child.value.value, str)
                if not is_text:
                    lines.add(child.lineno)
            walk(child)

    walk(function)
    return lines


def uncovered(hits: dict) -> dict:
    """`{файл: [(функция, строка def, число операторов, [невыполненные строки], достижима по именам)]}` по функциям `WATCHED`."""

    table = _function_table()
    live = reachable_functions(table)
    report: dict = {relative: [] for relative in WATCHED}
    for (relative, name), (line, _names, node) in table.items():
        statements = _own_statements(node)
        covered = set(hits.get(relative, ()))
        report[relative].append((name, line, len(statements), sorted(item for item in statements if item not in covered), (relative, name) in live))
    return {relative: sorted(rows, key=lambda row: row[1]) for relative, rows in report.items()}


def summary(report: dict, *, live_only: bool = True) -> dict:
    rows = [row for items in report.values() for row in items if row[4] or not live_only]
    total = sum(row[2] for row in rows)
    missed = sum(len(row[3]) for row in rows)
    never = sum(1 for row in rows if row[2] and len(row[3]) == row[2])
    return {
        "functions": len(rows), "statements": total, "uncovered": missed, "never_called_functions": never,
        "covered_percent": round(100.0 * (total - missed) / total, 1) if total else 0.0,
    }


def print_report(report: dict, only_missing: bool = True) -> None:
    for relative, rows in report.items():
        shown = [row for row in rows if row[4] and (row[3] or not only_missing)]
        if not shown:
            continue
        print(f"== {relative}")
        for name, line, count, missing, _live in shown:
            print(f"  {name:38s} L{line:<5d} statements={count:3d} uncovered={len(missing):3d} {missing[:40]}")
    print(json.dumps(summary(report)))


# --------------------------------------------------------------------------
# повтор корпуса под трассировкой и опись именованных веток
# --------------------------------------------------------------------------


def corpus_paths(spec: str) -> list:
    import native_skeleton_corpus as sc

    root = sc.matching(spec) if spec in sc.KINDS else Path(spec)
    if root is None:
        raise SystemExit(f"NATIVE_SKELETON_COVERAGE_FAILED {sc.describe_missing(spec)}")
    return [(root, row) for row in sc.rows_of(root)]


def replay(items, tracer: Tracer) -> int:
    import native_corpus as nc

    count = 0
    tracer.start()
    try:
        for root, row in items:
            record = nc.read_record(root / row["path"])
            call = nc.prepare_call(nc.OP_SKELETON, record.call_blob, record.before())
            try:
                nc.invoke(call)
            except Exception:  # noqa: BLE001 - отказ эталона тоже путь
                pass
            count += 1
    finally:
        tracer.stop()
    return count


def merge(files) -> dict:
    merged: dict = defaultdict(set)
    for file in files:
        for name, lines in json.loads(Path(file).read_text(encoding="utf-8")).items():
            merged[name].update(lines)
    return merged


def named_branches(roots) -> dict:
    """Какие именованные ветки достигнуты записями корпусов: исходы, исключения, ненулевые счётчики, сопряжение, исчерпание, режимы вызова."""

    import native_corpus as nc
    import native_skeleton_corpus as sc

    outcomes: Counter = Counter()
    exceptions: Counter = Counter()
    counters: Counter = Counter()
    modes: Counter = Counter()
    conjugation = 0
    records = 0
    for root in roots:
        for row in sc.rows_of(root):
            record = sc.read(root, row)
            before, expected = record.before(), record.expected()
            records += 1
            outcomes[row["outcome"]] += 1
            modes[f"split_search={row['split_search']}"] += 1
            modes["dense_hydration"] += int(row["dense_hydration"])
            modes["budget=None"] += int(not row["budget"])
            modes["warm_memory"] += int(bool(before.known_primes or before.factorization))
            modes["level_budget_pinned_below_default"] += int(expected.result is not None and expected.result.levels >= row["level_budget"])
            if expected.exception:
                exceptions[expected.exception[0]] += 1
                modes["internal_error_of_the_oracle"] += int(sc.is_internal_error(expected.exception))
            if expected.result is not None:
                for name, value in expected.result.counters:
                    counters[name] += int(value > 0)
                for obligation in expected.result.proof_obligations:
                    counters[f"obligation::{obligation.cause.value}/{obligation.disposition.value}"] += 1
            conjugation += int(expected.after.sign_counts.get("closed_by_conjugation", 0) - before.sign_counts.get("closed_by_conjugation", 0) > 0)
    del nc
    return {
        "records": records, "outcomes": dict(outcomes), "exceptions": dict(exceptions), "records_with_conjugation_signs": conjugation,
        "modes": dict(modes), "counters_nonzero_in_records": dict(sorted(counters.items())),
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("corpus", "report", "named"))
    parser.add_argument("items", nargs="*")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--dump", type=Path, default=None)
    arguments = parser.parse_args(argv)
    if arguments.command == "report":
        print_report(uncovered(merge(arguments.items)))
        return 0
    if arguments.command == "named":
        import native_skeleton_corpus as sc

        roots = [sc.matching(spec) if spec in sc.KINDS else Path(spec) for spec in (arguments.items or ["field", "synthetic"])]
        print(json.dumps(named_branches([root for root in roots if root is not None]), indent=1))
        return 0
    paths = []
    for spec in arguments.items:
        paths.extend(corpus_paths(spec))
    paths = paths[: arguments.limit] if arguments.limit else paths
    tracer = Tracer()
    count = replay(paths, tracer)
    if arguments.dump:
        tracer.dump(arguments.dump)
    print(f"replayed {count} records")
    print_report(uncovered(dict(tracer.hits)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
