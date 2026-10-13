"""Шов `frame` (допуск плоской рамки EXPORT) на уровне ЗАПИСАННЫХ вызовов: запись вызовов построителя ядра из корпуса задач, сверка нативной операции с эталоном, зерно.

    python tools/native_frame_corpus.py record <каталог корпуса задач> <файл вызовов.xz> [--name ЧАСТЬ_ИМЕНИ]
    python tools/native_frame_corpus.py replay <каталог корпуса задач | файл вызовов.xz> [--name ЧАСТЬ_ИМЕНИ]
    python tools/native_frame_corpus.py replay <...> --write-seed tests/data/native_frame_seed.xz   # зерно: самые маленькие вызовы каждого исхода

Корпус задач - тот же, что у `snapshot_check` (`tools/native_snapshot_check_corpus.py`, рекордер в `tools/blender_tree_ab.py`, `CFTUV_SNAPSHOT_RECORD_DIR`): каждая задача с выгрузкой идёт через
`task_inputs` воркера под стадиями `PYTHON`, и каждый вызов `cftuv_envelope.build_rational_affine_planar_metric` из шва `envelope_request_export._rational_affine_metric` записывается своими
именованными аргументами ядра (только объекты ядра: вершины, грани, треугольники, законы, допуск, полоса), как он ушёл эталону.

Сверка одного вызова (`compare`): эталон - построитель ядра с ЗАПИСАННЫМИ аргументами; порт - `cftuv_native.frame_metric` с теми аргументами, которые ему даёт шов. Исходов ровно два:
`equal` (порт построил метрику, и она равна метрике эталона - `==` и хеш, - а процессные счётчики символьного бэкенда сдвинуты так же) и `declined` (порт отказал по имени,
`NativePortUnsupported`: шов гонит эталон как раньше). Всё остальное - НАРУШЕНИЕ: порт ответил там, где эталон отказал или исключился, ответ иной, счётчики иные, порт поднял не отказ.
Память сертификата вложения (`_embedding._memo`) нативный путь не заполняет (решение FRAME): её счётчики не сравниваются.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import io
import lzma
import pickle
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CALLS_SCHEMA = "cftuv.native.frame.calls.v1"
SEED_SCHEMA = "cftuv.native.frame.seed.v1"
EQUAL = "equal"
DECLINED = "declined"
VIOLATION = "violation"


def _paths() -> None:
    for path in (ROOT / "kernel" / "src", ROOT / "tools"):
        if str(path) not in sys.path:
            sys.path.insert(0, str(path))


# --------------------------------------------------------------------------
# запись
# --------------------------------------------------------------------------


def record_calls(directory, name: str = "", limit: int | None = None) -> list:
    """`[(метка, аргументы ядра)]` вызовов построителя из шва `frame`, задачи корпуса под стадиями `PYTHON` (порт молчит, каждый вызов доходит до эталона)."""

    _paths()
    import native_snapshot_check_corpus as corpus

    corpus.install_host_stubs()
    import cftuv_envelope as kernel
    from cftuv.envelope_export_input import task_inputs
    from cftuv.envelope_kernel_backend import entered_backend

    original = kernel.build_rational_affine_planar_metric
    calls: list = []
    label = [""]

    def recording(**arguments):
        for key in ("source_vertices", "source_faces", "surface_triangles"):  # the containers are kept as the seam passed them (the port reads their exact class); only a one-shot iterator is materialized
            if arguments.get(key) is not None and not isinstance(arguments[key], (tuple, list, frozenset, set)):
                arguments[key] = tuple(arguments[key])
        calls.append((label[0], dict(arguments)))
        return original(**arguments)

    kernel.build_rational_affine_planar_metric = recording
    try:
        for path in corpus.corpus_files(directory, name):
            for case, task in corpus.read_tasks(path):
                if limit is not None and len(calls) >= limit:
                    return calls
                label[0] = f"{case}:{task.domain_id}"
                with entered_backend("PYTHON"), contextlib.redirect_stdout(io.StringIO()):
                    try:
                        task_inputs(dataclasses.replace(task, cold=None))
                    except Exception:  # noqa: BLE001 - исключение задачи - её исход; записанные вызовы остаются
                        pass
    finally:
        kernel.build_rational_affine_planar_metric = original
    return calls


def write_calls(path, calls: list, schema: str = CALLS_SCHEMA) -> None:
    with lzma.open(path, "wb", preset=6) as handle:
        pickle.dump({"schema": schema, "calls": calls}, handle, protocol=5)


def read_calls(path, schema: str = CALLS_SCHEMA) -> list:
    """`[(метка, аргументы ядра)]` файла вызовов (`record`) либо зерна (`SEED_SCHEMA`)."""

    _paths()
    import cftuv_envelope  # noqa: F401 - классы аргументов

    with lzma.open(path, "rb") as handle:
        document = pickle.load(handle)  # noqa: S301 - файл, записанный этим инструментом
    assert document["schema"] == schema, (document["schema"], schema)
    return document["calls"]


def load(source, name: str = "") -> list:
    """Вызовы из файла (`record`, зерно) либо записанные заново из каталога корпуса задач."""

    path = Path(source)
    if path.is_file():
        for schema in (CALLS_SCHEMA, SEED_SCHEMA):
            try:
                return [row for row in read_calls(path, schema) if name in row[0]]
            except AssertionError:
                continue
        raise SystemExit(f"{path}: not a calls file nor a seed")
    return record_calls(path, name)


# --------------------------------------------------------------------------
# сверка
# --------------------------------------------------------------------------


def native_arguments(arguments: dict) -> dict:
    """Аргументы порта, как их даёт шов (`_rational_affine_metric`): закон масштаба - `None`, если эталону он не назван."""

    import cftuv_envelope as kernel

    return {
        "source_vertices": arguments["source_vertices"],
        "source_faces": arguments["source_faces"],
        "owner_patch_id": arguments["owner_patch_id"],
        "source_revision": arguments["source_revision"],
        "patch_domain_id": arguments["patch_domain_id"],
        "source_lineage": arguments.get("source_lineage", frozenset()),
        "grid_policy": arguments.get("grid_policy", kernel.GridSnappingLawV1.UNSNAPPED_EXACT_V1),
        "grid_scale_law": arguments.get("grid_scale_law"),
    }


def _counters() -> tuple:
    from cftuv_envelope.reference import planar_types, symbolic_backend

    return dict(symbolic_backend.BACKEND_COUNTS), dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)


def _delta(before: tuple, after: tuple) -> tuple:
    return tuple(tuple(sorted((key, value - old.get(key, 0)) for key, value in new.items() if value != old.get(key, 0))) for old, new in zip(before, after))


def oracle(arguments: dict) -> tuple:
    """`(метрика | None, исключение | None, сдвиг счётчиков)` построителя ядра на записанных аргументах."""

    import cftuv_envelope as kernel

    before = _counters()
    try:
        result, raised = kernel.build_rational_affine_planar_metric(**arguments), None
    except Exception as error:  # noqa: BLE001 - исключение эталона - тоже исход
        result, raised = None, f"{type(error).__name__}: {str(error)[:200]}"
    return result, raised, _delta(before, _counters())


def compare(native, arguments: dict) -> tuple:
    """`(исход, подробность)`: `equal`, `declined` (причина порта) либо `violation` (что разошлось)."""

    before = _counters()
    try:
        got = native.frame_metric(**native_arguments(arguments))
    except native.NativePortUnsupported as error:
        return DECLINED, str(error)
    except Exception as error:  # noqa: BLE001 - порт обязан либо ответить, либо отказать по имени
        return VIOLATION, f"the port raised {type(error).__name__}: {str(error)[:200]}"
    moved = _delta(before, _counters())
    result, raised, expected = oracle(arguments)
    if raised is not None:
        return VIOLATION, f"the port answered where the oracle raised {raised}"
    if got != result or hash(got) != hash(result):
        return VIOLATION, f"the port answered otherwise: {str(got)[:300]} != {str(result)[:300]}"
    if moved != expected:
        return VIOLATION, f"the counters moved otherwise: port {moved}, oracle {expected}"
    return EQUAL, ""


def canonical(value):
    """Значение без порядка обхода множеств (он зависит от соли хешей процесса): запись - имя класса и поля, множество - отсортированные элементы."""

    import enum

    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        return (type(value).__name__, tuple((field.name, canonical(getattr(value, field.name))) for field in dataclasses.fields(value)))
    if isinstance(value, (frozenset, set)):
        return ("set", tuple(sorted((canonical(item) for item in value), key=repr)))
    if isinstance(value, (tuple, list)):
        return tuple(canonical(item) for item in value)
    if isinstance(value, enum.Enum):
        return (type(value).__name__, value.value)
    return value


def digest(value) -> str:
    """sha256 канонической записи (`canonical`): одна и та же метрика в любом процессе."""

    import hashlib

    return hashlib.sha256(repr(canonical(value)).encode("utf-8")).hexdigest()


def reason_of(detail: str) -> str:
    """`frame declined: GRID: ...` -> `GRID`."""

    parts = detail.split(":")
    return parts[1].strip() if len(parts) > 2 else detail[:40]


def replay(native, calls: list, quiet: bool = False) -> dict:
    """Все вызовы; `{calls, equal, declined, violations, by_case, ...}`."""

    summary = {"calls": 0, "equal": 0, "declined": Counter(), "violations": [], "by_case": {}, "oracle_raised": 0, "seconds": 0.0}
    started = time.perf_counter()
    for label, arguments in calls:
        outcome, detail = compare(native, arguments)
        summary["calls"] += 1
        row = summary["by_case"].setdefault(label.split(":")[0], Counter())
        row["calls"] += 1
        row[outcome] += 1
        if outcome == EQUAL:
            summary["equal"] += 1
        elif outcome == DECLINED:
            summary["declined"][reason_of(detail)] += 1
            if oracle(arguments)[1] is not None:
                summary["oracle_raised"] += 1
        else:
            summary["violations"].append((label, detail))
        if not quiet and summary["calls"] % 250 == 0:
            print(f"  {summary['calls']} calls, {len(summary['violations'])} violations", flush=True)
    summary["seconds"] = time.perf_counter() - started
    return summary


def describe(summary: dict) -> str:
    lines = [
        f"calls {summary['calls']} equal {summary['equal']} declined {sum(summary['declined'].values())} {dict(summary['declined'])} (the oracle raised on {summary['oracle_raised']} of them) "
        f"VIOLATIONS {len(summary['violations'])} in {summary['seconds']:.1f} s"
    ]
    lines.extend(f"  {case}: {dict(row)}" for case, row in sorted(summary["by_case"].items()))
    lines.extend(f"VIOLATION {label}: {detail}" for label, detail in summary["violations"][:10])
    return "\n".join(lines)


# --------------------------------------------------------------------------
# зерно
# --------------------------------------------------------------------------


def size_of(arguments: dict) -> int:
    return len(arguments["source_vertices"]) + len(arguments["source_faces"])


def category_of(native, arguments: dict) -> str:
    """Исход вызова для выборки зерна: `accepted` (с числом восстановленных прямых углов, `>1` проб масштаба), `declined REASON`, `declined REASON raised` (эталон исключился)."""

    outcome, detail = compare(native, arguments)
    if outcome == EQUAL:
        metric = oracle(arguments)[0]
        grid = metric.grid_certificate
        return f"accepted corners={min(grid.restored_right_corners, 2)} trials={min(len(grid.scale_trials), 2)}"
    if outcome == DECLINED:
        return f"declined {reason_of(detail)}" + (" raised" if oracle(arguments)[1] is not None else "")
    return "violation"


def select_seed(native, calls: list, per_category: int = 3, per_case: int = 2, large: int = 30) -> list:
    """Самые МАЛЕНЬКИЕ вызовы каждой категории, не больше `per_case` одного случая на категорию; и по одному принятому вызову от `large` вершин и граней с каждого случая (сертификат вложения на многих гранях)."""

    rows = sorted(((size_of(arguments), label, arguments) for label, arguments in calls), key=lambda row: (row[0], row[1]))
    taken: Counter = Counter()
    per_label: Counter = Counter()
    chosen = []
    for size, label, arguments in rows:
        category = category_of(native, arguments)
        case = label.split(":")[0]
        if size >= large and category.startswith("accepted") and not taken[("large", case)]:
            taken[("large", case)] += 1
            chosen.append((f"{label} [{category} large, {size}]", arguments))
            continue
        if taken[category] >= per_category or per_label[(category, case)] >= per_case:
            continue
        taken[category] += 1
        per_label[(category, case)] += 1
        chosen.append((f"{label} [{category}, {size}]", arguments))
    return chosen


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("record", "replay"))
    parser.add_argument("source", help="каталог корпуса задач либо файл вызовов (`record`) / зерна")
    parser.add_argument("output", nargs="?", default="", help="`record`: файл вызовов")
    parser.add_argument("--name", default="")
    parser.add_argument("--write-seed", default="")
    arguments = parser.parse_args(argv)
    _paths()
    if arguments.command == "record":
        calls = record_calls(arguments.source, arguments.name)
        write_calls(arguments.output, calls)
        print(f"{len(calls)} calls -> {arguments.output} ({Path(arguments.output).stat().st_size} bytes)")
        return 0
    import cftuv_native

    calls = load(arguments.source, arguments.name)
    if arguments.write_seed:
        chosen = select_seed(cftuv_native, calls)
        write_calls(arguments.write_seed, chosen, SEED_SCHEMA)
        for label, _arguments in chosen:
            print(f"  {label}")
        print(f"seed: {len(chosen)} calls, {Path(arguments.write_seed).stat().st_size} bytes")
        return 0
    summary = replay(cftuv_native, calls)
    print(describe(summary))
    return 1 if summary["violations"] else 0


if __name__ == "__main__":
    sys.exit(main())
