"""Корпус стадии EXPORT воркера (`task_inputs`) и сверка `snapshot_check`: запись задач с выгрузкой, воспроизведение эталоном и нативной стадией, точное сравнение.

    python tools/native_snapshot_check_corpus.py replay <каталог корпуса> [--limit N] [--name ЧАСТЬ_ИМЕНИ]
    python tools/native_snapshot_check_corpus.py bench  <каталог корпуса> [--repeat 3]
    python tools/native_snapshot_check_corpus.py seam   <каталог корпуса> [--name ЧАСТЬ_ИМЕНИ] [--repeat 2]
    python tools/native_snapshot_check_corpus.py seam-once <каталог корпуса> --stage PYTHON|NATIVE [--name ЧАСТЬ_ИМЕНИ]   # один проход, по процессу на стадию
    python tools/native_snapshot_check_corpus.py refusals <каталог корпуса> [--name ЧАСТЬ_ИМЕНИ] [--tasks 40] [--seed 1]   # отказы выгрузки сквозь воркера: снапшот подменён мутантом на обоих швах

Запись (`Recorder`, ставится рабочим процессом `tools/blender_tree_ab.py` при `CFTUV_SNAPSHOT_RECORD_DIR=<каталог>` и `--workers >= 2`): в родителе `DomainPool.run` обёрнут, и каждая задача С ВЫГРУЗКОЙ
(`DomainTaskV1.export`: `HostExportInputV1` + допуски + выделение; и остаток, который вызывающий достраивает за пулом, `LateTasksV1.provide`) пишется ровно так, как уходит воркеру, - в `<каталог>/<случай>.tasks.xz` (поток pickle `(случай, задача)` под lzma; повтор того же
входа не пишется). Вход `task_inputs` - это и есть задача: воспроизведение зовёт ту же функцию воркера в этом процессе, без Blender и без пула.

Воспроизведение: `task_inputs(задача)` дважды - стадия `snapshot_check` заказана на `PYTHON` (эталон: валидаторы Python) и на `NATIVE`. Оракул - канонические байты снапшота
(`AnalysisSnapshotCodecV1.dumps`), равенство запроса, исход отказа выгрузки (имя исхода, текст, домен) и приращения процессных счётчиков символьного бэкенда (`symbolic_backend.BACKEND_COUNTS`,
`planar_types.SYMBOLIC_FALLBACK_COUNTS`): всё это обязано совпасть побитово. Задача, которую порт отказал по имени, считается эталоном - она в сводке как `declined` с причиной, не как расхождение.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import hashlib
import io
import lzma
import os
import pickle
import sys
import time
import types
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RECORD_ENVIRONMENT = "CFTUV_SNAPSHOT_RECORD_DIR"
TASK_SUFFIX = ".tasks.xz"


# --------------------------------------------------------------------------
# запись
# --------------------------------------------------------------------------


class Recorder:
    """Задачи с выгрузкой, ушедшие в пул, - в файл случая."""

    def __init__(self, directory, case: str) -> None:
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.case = case
        self.path = self.directory / f"{case}{TASK_SUFFIX}"
        self.handle = lzma.open(self.path, "wb", preset=3)
        self.seen: set = set()
        self.count = 0
        self.raw_bytes = 0
        self._original = None

    def install(self) -> None:
        from cftuv import envelope_domain_pool as pool_module

        recorder = self
        original = pool_module.DomainPool.run
        self._original = original

        def run(pool, tasks, cancel=None, late=None):
            recorder.note(tasks)
            if late is not None:  # the rest of the tasks is built behind the pool (SCAN_BEHIND_THE_POOL_V1): note them when the caller provides them
                provide = late.provide

                def noting():
                    rest = provide()
                    recorder.note(rest)
                    return rest

                late = dataclasses.replace(late, provide=noting)
            return original(pool, tasks, cancel, late)

        pool_module.DomainPool.run = run

    def note(self, tasks) -> None:
        for task in tasks:
            if getattr(task, "export", None) is None:
                continue
            blob = pickle.dumps(task, protocol=5)
            key = (task.patch_id, hashlib.sha256(pickle.dumps(task.export, protocol=5)).hexdigest())
            if key in self.seen:
                continue
            self.seen.add(key)
            pickle.dump((self.case, blob), self.handle, protocol=5)
            self.count += 1
            self.raw_bytes += len(blob)

    def close(self) -> dict:
        self.handle.close()
        if self._original is not None:
            from cftuv import envelope_domain_pool as pool_module

            pool_module.DomainPool.run = self._original
        return {"case": self.case, "tasks": self.count, "raw_bytes": self.raw_bytes, "file_bytes": self.path.stat().st_size}


def read_tasks(path):
    """`(случай, задача)` файла в порядке записи."""

    with lzma.open(path, "rb") as handle:
        while True:
            try:
                case, blob = pickle.load(handle)
            except EOFError:
                return
            yield case, pickle.loads(blob)


def corpus_files(directory, name: str = "") -> list:
    return sorted(path for path in Path(directory).glob(f"*{TASK_SUFFIX}") if name in path.name)


# --------------------------------------------------------------------------
# воспроизведение
# --------------------------------------------------------------------------


def install_host_stubs() -> None:
    """Пакет хоста импортируется вне Blender, как воркер пула (обычный интерпретатор): запись задачи не несёт `mathutils`, но модули могут её просить."""

    sys.path.insert(0, str(ROOT))
    sys.path.insert(0, str(ROOT / "kernel" / "src"))
    for name in ("bpy", "bmesh", "mathutils"):
        if name not in sys.modules:
            sys.modules[name] = types.ModuleType(name)


@dataclasses.dataclass
class Outcome:
    """Что `task_inputs` сделал с задачей под одним порядком стадии."""

    snapshot_sha: str | None = None
    request: object | None = None
    refusal: tuple | None = None
    exception: str | None = None
    counters: tuple = ()
    ledger: tuple = ()
    seconds: float = 0.0
    snapshot_clean: bool = False

    def comparable(self) -> tuple:
        return (self.snapshot_sha, self.request, self.refusal, self.exception, self.counters)


def run_task(task, master: str) -> Outcome:
    """`task_inputs(задача)` под блоком ядра с заказом `master` (`PYTHON` | `NATIVE`)."""

    import cftuv_envelope as kernel
    from cftuv.envelope_export_input import TaskInputsV1, task_inputs
    from cftuv.envelope_kernel_backend import entered_backend
    from cftuv_envelope.reference import planar_types, symbolic_backend

    task = dataclasses.replace(task, cold=None)
    symbolic_backend.reset_backend_counts()
    symbolic_before = dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    outcome = Outcome()
    started = time.perf_counter()
    with entered_backend(master) as ledger, contextlib.redirect_stdout(io.StringIO()):  # the worker profile prints its stages
        try:
            inputs = task_inputs(task)
        except Exception as error:  # noqa: BLE001 - исключение эталона - тоже исход
            outcome.exception = f"{type(error).__name__}: {error}"
            inputs = None
        record = None if ledger is None else ledger.record()
    outcome.seconds = time.perf_counter() - started
    outcome.counters = (
        tuple(sorted(symbolic_backend.BACKEND_COUNTS.items())),
        tuple(sorted((key, value - symbolic_before.get(key, 0)) for key, value in planar_types.SYMBOLIC_FALLBACK_COUNTS.items() if value != symbolic_before.get(key, 0))),
    )
    if record is not None:
        outcome.ledger = (record.snapshot_check_native_calls, record.snapshot_check_python_calls, record.snapshot_check_outcomes)
    if inputs is None:
        return outcome
    if isinstance(inputs, TaskInputsV1):
        outcome.snapshot_sha = hashlib.sha256(kernel.AnalysisSnapshotCodecV1.dumps(inputs.snapshot)).hexdigest()
        outcome.request = inputs.request
        outcome.snapshot_clean = inputs.check is not None
        return outcome
    refusal = inputs.refusal
    outcome.refusal = (str(getattr(refusal.outcome, "value", refusal.outcome)), refusal.message, refusal.patch_domain_id)
    if inputs.snapshot is not None:
        outcome.snapshot_sha = hashlib.sha256(kernel.AnalysisSnapshotCodecV1.dumps(inputs.snapshot)).hexdigest()
    return outcome


def replay(directory, limit: int | None = None, name: str = "", quiet: bool = False) -> dict:
    """Все задачи корпуса эталоном и нативной стадией; `{tasks, equal, differences, declined, attested, ...}`."""

    install_host_stubs()
    os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
    summary = {"tasks": 0, "equal": 0, "differences": [], "declined": Counter(), "python_seconds": 0.0, "native_seconds": 0.0, "by_case": {}, "refusals": 0}
    for path in corpus_files(directory, name):
        for case, task in read_tasks(path):
            if limit is not None and summary["tasks"] >= limit:
                return summary
            oracle = run_task(task, "PYTHON")
            native = run_task(task, "NATIVE")
            summary["tasks"] += 1
            row = summary["by_case"].setdefault(case, {"tasks": 0, "equal": 0})
            row["tasks"] += 1
            summary["python_seconds"] += oracle.seconds
            summary["native_seconds"] += native.seconds
            summary["refusals"] += oracle.refusal is not None or oracle.exception is not None
            if oracle.comparable() == native.comparable():
                summary["equal"] += 1
                row["equal"] += 1
            else:
                summary["differences"].append((case, task.domain_id, _describe(oracle, native)))
            if native.ledger and native.ledger[1]:
                summary["declined"][native.ledger[2]] += 1
            if not quiet and summary["tasks"] % 200 == 0:
                print(f"  {summary['tasks']} tasks, {len(summary['differences'])} differences", flush=True)
    return summary


def _describe(oracle: Outcome, native: Outcome) -> list:
    names = ("snapshot_sha", "request", "refusal", "exception", "counters")
    return [(name, str(getattr(oracle, name))[:160], str(getattr(native, name))[:160]) for name in names if getattr(oracle, name) != getattr(native, name)]


def bench(directory, repeat: int = 3, name: str = "") -> dict:
    """Время `task_inputs` целиком (EXPORT воркера) эталоном и нативной стадией, на тёплом процессе: лучший из `repeat` кругов по корпусу."""

    install_host_stubs()
    os.environ["CFTUV_SNAPSHOT_BYTE_CHECK"] = "all"
    tasks = [task for path in corpus_files(directory, name) for _case, task in read_tasks(path)]
    result = {}
    for master in ("PYTHON", "NATIVE", "PYTHON", "NATIVE")[: 2 * min(repeat, 2)]:
        best = None
        for _ in range(repeat):
            started = time.perf_counter()
            for task in tasks:
                run_task(task, master)
            spent = time.perf_counter() - started
            best = spent if best is None else min(best, spent)
        result[master] = min(result.get(master, best), best)
    result["tasks"] = len(tasks)
    return result


def seam_bench(directory, name: str = "", repeat: int = 2) -> dict:
    """ШОВ как его гонит воркер: `reject_exported_snapshot` и `request_snapshot_issues` над снапшотами корпуса, стадия `PYTHON` либо `NATIVE`; лучший из кругов, процесс прогрет прогоном эталона.

    Снапшоты берутся из `task_inputs` под эталоном (один раз). Время - только шов, без выгрузки: это число «скорость шва» (эталонные валидаторы против нативного ответа с откатами на эталон).
    """

    install_host_stubs()
    import cftuv_envelope as kernel
    from cftuv import envelope_angle_certificate as certificate
    from cftuv import envelope_snapshot_check as check
    from cftuv.envelope_export_input import TaskInputsV1, task_inputs
    from cftuv.envelope_kernel_backend import entered_backend

    snapshots = []
    with contextlib.redirect_stdout(io.StringIO()):
        for path in corpus_files(directory, name):
            for _case, task in read_tasks(path):
                with entered_backend("PYTHON"):
                    inputs = task_inputs(dataclasses.replace(task, cold=None))
                if isinstance(inputs, TaskInputsV1):
                    snapshots.append((inputs.snapshot, task.export.developable_stretch_budget))
    result = {"snapshots": len(snapshots)}
    for master in ("PYTHON", "NATIVE", "PYTHON", "NATIVE"):
        best = None
        for _ in range(repeat):
            started = time.perf_counter()
            with entered_backend(master) as ledger:
                for snapshot, budget in snapshots:
                    certificate.reject_exported_snapshot(kernel, snapshot)
                    check.request_snapshot_issues(kernel, snapshot, budget)
            spent = time.perf_counter() - started
            best = spent if best is None else min(best, spent)
        result[master] = min(result.get(master, best), best)
        if ledger is not None:
            record = ledger.record()
            result["native_calls"], result["python_calls"] = record.snapshot_check_native_calls, record.snapshot_check_python_calls
    result["speedup"] = result["PYTHON"] / result["NATIVE"]
    return result


def seam_once(directory, master: str, name: str = "", warm: int = 8) -> dict:
    """ШОВ одним проходом, как в воркере: каждый снапшот встречается ОДИН раз (на тёплом интерпретаторе: `warm` первых снапшотов в замер не входят), стадия `master`.

    Повтор круга по тем же снапшотам кормит кэши sympy и занижает цену эталона; один проход - то, что видит воркер. Сравнивать нужно ДВА процесса (`PYTHON` и `NATIVE`) на одном корпусе.
    """

    install_host_stubs()
    import cftuv_envelope as kernel
    from cftuv import envelope_angle_certificate as certificate
    from cftuv import envelope_snapshot_check as check
    from cftuv.envelope_export_input import TaskInputsV1, task_inputs
    from cftuv.envelope_kernel_backend import entered_backend

    snapshots = []
    with contextlib.redirect_stdout(io.StringIO()):
        for path in corpus_files(directory, name):
            for _case, task in read_tasks(path):
                with entered_backend("PYTHON"):
                    inputs = task_inputs(dataclasses.replace(task, cold=None))
                if isinstance(inputs, TaskInputsV1):
                    snapshots.append((inputs.snapshot, task.export.developable_stretch_budget))
    # the snapshots above were VALIDATED once already (by `task_inputs`): their expressions are in the sympy caches. A fresh copy of every snapshot is what the worker would meet; a pickle round trip makes
    # new objects (the caches of the validators key by object), but the Fraction/sympy value caches stay: the honest cold figure needs a fresh process per pass, which `master` provides.
    fresh = [(pickle.loads(pickle.dumps(snapshot, protocol=5)), budget) for snapshot, budget in snapshots]
    result = {"snapshots": len(fresh), "master": master}
    with entered_backend("PYTHON"):
        for snapshot, budget in fresh[:warm]:
            certificate.reject_exported_snapshot(kernel, snapshot)
            check.request_snapshot_issues(kernel, snapshot, budget)
    timed = fresh[warm:]
    started = time.perf_counter()
    with entered_backend(master) as ledger:
        for snapshot, budget in timed:
            certificate.reject_exported_snapshot(kernel, snapshot)
            check.request_snapshot_issues(kernel, snapshot, budget)
    result["seconds"] = time.perf_counter() - started
    result["timed"] = len(timed)
    if ledger is not None:
        record = ledger.record()
        result["native_calls"], result["python_calls"] = record.snapshot_check_native_calls, record.snapshot_check_python_calls
    return result


def refusal_replay(directory, name: str = "", tasks: int = 40, seed: int = 1) -> dict:
    """ОТКАЗЫ выгрузки сквозь воркера: задачи корпуса, у которых снапшот ПЕРЕД проверкой подменён мутантом (по одному на класс замечания, `targeted_mutants`), под стадией `PYTHON` и `NATIVE`.

    Подмена стоит на обоих швах (`reject_exported_snapshot` и `request_snapshot_issues`), так что `task_inputs` идёт настоящим путём: проверка объекта, затем проверка под допуском запроса, отказ выгрузки с именем исхода
    и ТЕКСТОМ, домен, процессные счётчики. Сравнивается всё, что сравнивает `replay`: порт не принимает того, что Python отказывает, и отказ выдаёт эталон тем же текстом.
    """

    import random

    install_host_stubs()
    sys.path.insert(0, str(ROOT / "tools"))
    import native_snapshot_check_mutations as mutations
    from cftuv import envelope_angle_certificate as certificate
    from cftuv import envelope_snapshot_check as check

    rng = random.Random(seed)
    items = [(case, task) for path in corpus_files(directory, name) for case, task in read_tasks(path)]
    rng.shuffle(items)
    reject, issues = certificate.reject_exported_snapshot, check.request_snapshot_issues
    summary = {"tasks": 0, "mutants": 0, "equal": 0, "differences": [], "refused": 0, "raised": 0, "clean": 0, "outcomes": Counter(), "native_clean": 0, "declined": Counter()}
    try:
        for case, task in items[:tasks]:
            seen: list = []

            def recording(kernel, snapshot, _seen=seen):
                _seen.append(snapshot)
                return reject(kernel, snapshot)

            certificate.reject_exported_snapshot = recording
            run_task(task, "PYTHON")
            certificate.reject_exported_snapshot = reject
            if not seen:
                continue
            summary["tasks"] += 1
            for label, mutant in mutations.targeted_mutants(seen[0], rng):
                certificate.reject_exported_snapshot = lambda kernel, snapshot, _m=mutant: reject(kernel, _m)
                check.request_snapshot_issues = lambda kernel, snapshot, budget, _m=mutant: issues(kernel, _m, budget)
                oracle = run_task(task, "PYTHON")
                native = run_task(task, "NATIVE")
                summary["mutants"] += 1
                if oracle.comparable() == native.comparable():
                    summary["equal"] += 1
                else:
                    summary["differences"].append((case, task.domain_id, label, _describe(oracle, native)))
                if oracle.refusal is not None:
                    summary["refused"] += 1
                    summary["outcomes"][oracle.refusal[0]] += 1
                elif oracle.exception is not None:
                    summary["raised"] += 1
                else:
                    summary["clean"] += 1
                    summary["native_clean"] += bool(native.snapshot_clean)
                if native.ledger and native.ledger[1]:
                    summary["declined"][native.ledger[2]] += 1
    finally:
        certificate.reject_exported_snapshot, check.request_snapshot_issues = reject, issues
    return summary


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("replay", "bench", "seam", "seam-once", "refusals"))
    parser.add_argument("directory")
    parser.add_argument("--limit", type=int, default=None)
    parser.add_argument("--name", default="")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--stage", choices=("PYTHON", "NATIVE"), default="NATIVE")
    parser.add_argument("--tasks", type=int, default=40)
    parser.add_argument("--seed", type=int, default=1)
    arguments = parser.parse_args(argv)
    if arguments.command == "refusals":
        summary = refusal_replay(arguments.directory, arguments.name, arguments.tasks, arguments.seed)
        print(f"tasks {summary['tasks']} mutants {summary['mutants']} equal {summary['equal']} differences {len(summary['differences'])}; the oracle refused {summary['refused']} (outcomes {dict(summary['outcomes'])}), raised {summary['raised']}, passed {summary['clean']}")
        print(f"the native stage attested {summary['native_clean']} of the {summary['clean']} the oracle passed; declined by the port: {dict(summary['declined'])}")
        for difference in summary["differences"][:10]:
            print("DIFFERENCE", difference)
        return 1 if summary["differences"] else 0
    if arguments.command == "seam-once":
        print(seam_once(arguments.directory, arguments.stage, arguments.name))
        return 0
    if arguments.command == "seam":
        print(seam_bench(arguments.directory, arguments.name, arguments.repeat))
        return 0
    if arguments.command == "bench":
        print(bench(arguments.directory, arguments.repeat, arguments.name))
        return 0
    summary = replay(arguments.directory, arguments.limit, arguments.name)
    print(f"tasks {summary['tasks']} equal {summary['equal']} differences {len(summary['differences'])} refusals-of-the-oracle {summary['refusals']}")
    print(f"python {summary['python_seconds']:.2f} s native-stage {summary['native_seconds']:.2f} s; declined by the port: {dict(summary['declined'])}")
    for case, row in sorted(summary["by_case"].items()):
        print(f"  {case}: {row['equal']}/{row['tasks']}")
    for case, domain, difference in summary["differences"][:10]:
        print("DIFFERENCE", case, domain, difference)
    return 1 if summary["differences"] else 0


if __name__ == "__main__":
    sys.exit(main())
