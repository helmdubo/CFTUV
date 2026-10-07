"""Синтетический корпус скелета: вызовы `build_skeleton` из тестов ядра и сгенерированные полигоны, и опись достигнутых веток.

Полевой корпус (163 вызова пяти мешей) не доходит до ветвей: сопряжение при знаке, исчерпание бюджета, граница уровней, плотная гидратация,
вызов без бюджета, теплая память, режим полного перебора, лучи при многостороннем слиянии. Их строят тесты ядра на именованных полигонах
(`kernel/tests/test_wavefront_*.py` и те, что проходят через `prepare_conveyor`) и генератор (`native_skeleton_generated.py`). Этот модуль —
плагин pytest (`-p native_skeleton_synthetic`): пока тесты идут, он ставит записывающую обёртку на `build_skeleton` в КАЖДОМ модуле, который
импортировал функцию по имени (`wavefront`, `conveyor`, модули тестов), а сам эталон остаётся нетронутым.

Сгенерированные записи и производные с урезанным потолком ДОПИСЫВАЮТСЯ отдельными командами (`native_skeleton_generated.py`, `native_skeleton_derive.py`).

Запись вызова — как в полевом корпусе (`native_corpus.Recorder`): состояние ДО, пикл входов (полигон, бюджет постоянным идентификатором, режим
поиска, плотная гидратация, ДЕЙСТВУЮЩАЯ граница уровней), исход эталона и состояние ПОСЛЕ. Исход считается ПОСЛЕ прогона тестов, заново, чистым эталоном
от записанного состояния (тест мог подменить внутренности ядра: запись описывает вызов, а не ход теста), с продуктовым значением аудита каноники
(выключен; набор тестов его включает, и сверка `--audit on` в `native_skeleton_verify.py` доказывает, что исход от аудита не зависит). Одинаковые вызовы (тот же полигон,
режим, бюджет и память) пишутся один раз; `live_equal` в строке индекса — совпал ли исход, увиденный тестом, с исходом чистого эталона (нет — тест подменял ядро).

    python tools/native_skeleton_synthetic.py build [--out DIR] [--files a.py,b.py]    # прогон тестов ядра с плагином (подпроцесс), индекс
    python tools/native_skeleton_synthetic.py inventory [--out DIR]                    # опись корпуса
    python tools/native_skeleton_synthetic.py rewrite --src RAW [--out DIR]            # переписать построенный корпус с холодной памятью и без повторов
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import hashlib
import json
import os
import subprocess
import sys
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
for _path in (str(TOOLS), str(ROOT / "kernel" / "src"), str(ROOT / "kernel" / "tests")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import pytest  # noqa: E402

import native_corpus as nc  # noqa: E402
import native_skeleton_corpus as sc  # noqa: E402

OUT_ENVIRONMENT = "CFTUV_SYNTHETIC_SKELETON_OUT"
FILES_ENVIRONMENT = "CFTUV_SYNTHETIC_SKELETON_FILES"
COVERAGE_ENVIRONMENT = "CFTUV_SKELETON_COVERAGE"
INDEX_SCHEMA = "cftuv.native-corpus.synthetic-skeleton.v1"

#: Тесты ядра, которые доходят до `build_skeleton` (прямо или через `prepare_conveyor`). Плагин пишет только то, что вызвано.
TEST_FILES = (
    "test_adaptive_density_fan_authority.py",
    "test_alpha_interval.py",
    "test_building_002_point_contact_fixture.py",
    "test_canonical_fan_rays.py",
    "test_chain_station_plan.py",
    "test_chain_straight_evaluation_geometry_binding.py",
    "test_contact_candidates_memo.py",
    "test_contact_prefilter.py",
    "test_convex_partition.py",
    "test_conveyor_preparation_pickle.py",
    "test_corner_fold.py",
    "test_corner_join.py",
    "test_density_exact_limit_lift.py",
    "test_developable_band.py",
    "test_developable_band_ring.py",
    "test_evaluation_binding_noise.py",
    "test_event_time_memo.py",
    "test_exact_identity_shadow.py",
    "test_exact_work_budget.py",
    "test_exact_work_budget_coverage.py",
    "test_fan_narrow_band.py",
    "test_interval_step.py",
    "test_join_bend_density_subturn.py",
    "test_join_station_conflict.py",
    "test_lazy_hydration_shadow.py",
    "test_materialization_speed_memos.py",
    "test_materialize_memo.py",
    "test_near_planar_reduced_frame.py",
    "test_overlay_signature_memo.py",
    "test_plain_affine_skip.py",
    "test_price_without_history.py",
    "test_sem_clb_02_chain_straight_regression.py",
    "test_silhouette_topology.py",
    "test_surface_flat_reduction.py",
    "test_wavefront_conveyor.py",
    "test_wavefront_coverage.py",
    "test_wavefront_degenerate_event.py",
    "test_wavefront_differential.py",
    "test_wavefront_dissolve_straight.py",
    "test_wavefront_event_queue.py",
    "test_wavefront_faces.py",
    "test_wavefront_faces_crowded.py",
    "test_wavefront_mitered_standard.py",
    "test_wavefront_motorcycle_graph.py",
    "test_wavefront_partial_source.py",
    "test_wavefront_poststate_span.py",
    "test_wavefront_proof_obligations.py",
    "test_wavefront_same_time_closure.py",
    "test_wavefront_superlevel_transaction.py",
    "test_wavefront_vertex_fan.py",
    "test_wavefront_weighted_wall_differential.py",
)


def default_out() -> Path:
    return Path(os.environ.get(OUT_ENVIRONMENT) or sc.default_out("synthetic"))


# --------------------------------------------------------------------------
# плагин pytest: записывающая обёртка на время каждого теста
# --------------------------------------------------------------------------


def cold_state(before: nc.StateV1) -> nc.StateV1:
    """Состояние ДО вызова с ХОЛОДНОЙ памятью: таблицы пусты, знаки и неоплаченное нулевые, аудит выключен (продуктовое значение).

    Набор тестов гонит вызовы в памяти, оставшейся от соседних тестов (тысячи записей по четырём таблицам: ~50 КБ на запись, 240 МБ на корпус), и эта память — случайность порядка тестов,
    а не условие вызова. Бюджет (потолок, статьи, стадия, идентичность) остаётся как был: он — вход вызова. Тёплая и насыщенная память записывается СГЕНЕРИРОВАННЫМИ вызовами
    (`native_skeleton_generated.py`: варианты `warm` и `saturated`), где она задана намеренно."""

    return dataclasses.replace(
        before, known_primes=[], factorization=[], squarefree=[], prime_support=[], sign_counts={key: 0 for key in before.sign_counts},
        unbudgeted=(0,) * 6, store=None, canonical_audit=False,
    )


def _dedupe_key(blob: bytes, before: nc.StateV1) -> tuple:
    shape = json.dumps([before.budget, before.identity_mode], default=str, sort_keys=True)
    return (hashlib.sha1(blob).digest(), hashlib.sha1(shape.encode()).digest())


def _live_view(result, error) -> tuple:
    """Что увидел тест, без цены и памяти: `("raised", (класс, текст))` либо `("result", sha256 ответа)`."""

    if error is not None:
        return ("raised", (type(error).__qualname__, str(error)))
    return ("result", nc.answer_digest(nc.OP_SKELETON, result))


class _Collector:
    """Что собрано за сессию: различные вызовы `build_skeleton` (вход, холодное состояние до, что увидел тест)."""

    def __init__(self, out: Path) -> None:
        self.out = out
        self.pending: list[dict] = []
        self.seen: set = set()
        self.test = ""
        self.calls = 0
        self.duplicates = 0
        self.skipped: Counter = Counter()

    def begin(self, args: tuple, kwargs: dict):
        """Снимок входа ДО вызова; `None` — вызов корпус не несёт или он уже записан (тот же полигон, режим и бюджет)."""

        self.calls += 1
        try:
            call = nc.unpack_call(nc.OP_SKELETON, args, kwargs)
            blob = nc.encode_call(call)
            budget = nc.budget_state(call.budget)
            before = cold_state(nc.StateV1([], [], [], [], budget, dict(nc.exact.SIGN_COUNTS), (0,) * 6, None, False, nc.exact_identity.identity_mode().value))
        except Exception as exc:  # noqa: BLE001 - вызов, который корпус не несёт, учитывается, а не роняет тест
            self.skipped[type(exc).__name__] += 1
            return None
        key = _dedupe_key(blob, before) + (json.dumps(sorted(call.kwargs.items(), key=lambda item: item[0]), default=str).encode(),)
        if key in self.seen:
            self.duplicates += 1
            return None
        self.seen.add(key)
        return {"before": before, "blob": blob, "test": self.test}

    def finish(self, entry: dict, result, error) -> None:
        """Что увидел тест, и запись в очередь."""

        try:
            live = _live_view(result, error)
        except Exception as exc:  # noqa: BLE001
            live = None
            self.skipped[f"live: {type(exc).__name__}"] += 1
        self.pending.append({"test": entry["test"], "before": entry["before"], "blob": entry["blob"], "live": live})


_COLLECTOR: _Collector | None = None
_TRACER = None
_ALIASES: list | None = None


def pytest_configure(config) -> None:
    global _COLLECTOR, _TRACER
    _COLLECTOR = _Collector(default_out())
    if os.environ.get(COVERAGE_ENVIRONMENT):
        import native_skeleton_coverage as coverage

        _TRACER = coverage.Tracer()
        _TRACER.start()


def _aliases_of(original) -> list:
    """`[(модуль, имя)]`: все модули, держащие функцию ПО ИМЕНИ (`from ...skeleton import build_skeleton`): подмена атрибута `skeleton` их не задевает."""

    global _ALIASES
    if _ALIASES is None:
        _ALIASES = [
            (module, attribute)
            for module in list(sys.modules.values())
            for attribute, value in list(getattr(module, "__dict__", {}).items())
            if value is original
        ]
    return _ALIASES


@contextlib.contextmanager
def _capturing(collector: _Collector):
    """Обёртка захвата входов на время теста; снимается в точности."""

    import cftuv_envelope.wavefront.skeleton as skeleton

    original = nc.ORACLE[nc.OP_SKELETON]
    current = skeleton.build_skeleton

    def build_skeleton(*args, **kwargs):
        entry = collector.begin(args, kwargs)
        if entry is None:
            return current(*args, **kwargs)
        try:
            result = current(*args, **kwargs)
        except Exception as exc:
            collector.finish(entry, None, exc)
            raise
        collector.finish(entry, result, None)
        return result

    aliases = _aliases_of(original)
    for module, attribute in aliases:
        setattr(module, attribute, build_skeleton)
    try:
        yield
    finally:
        for module, attribute in aliases:
            setattr(module, attribute, original)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_call(item):
    collector = _COLLECTOR
    collector.test = item.nodeid
    with _capturing(collector):
        yield


def pytest_sessionfinish(session, exitstatus) -> None:
    if _TRACER is not None:
        _TRACER.stop()
        _TRACER.dump(Path(os.environ[COVERAGE_ENVIRONMENT]))
    collector = _COLLECTOR
    if collector is not None:
        write_corpus(collector, exitstatus)


# --------------------------------------------------------------------------
# запись корпуса после прогона
# --------------------------------------------------------------------------


def _group_of(test: str) -> str:
    """Каталог записи: файл теста без `test_` и расширения."""

    name = Path(test.split("::")[0]).stem
    return name[5:] if name.startswith("test_") else name


def _replay(call: nc.Call):
    """Чистый эталон на вызове: `(результат, исключение, секунды)`."""

    started = time.perf_counter()
    try:
        result, error = nc.invoke(call), None
    except Exception as exc:  # noqa: BLE001 - исключение — часть исхода
        result, error = None, exc
    return result, error, time.perf_counter() - started


def write_items(out: Path, items: list, description: dict, extra: dict) -> dict:
    """Пишет корпус `out` из `items` (`test`, холодное `before`, `blob`, `live`): чистый эталон считает исход каждой различной записи; возвращает индекс."""

    out.mkdir(parents=True, exist_ok=True)
    recorder = nc.Recorder(out, description, preset=3, max_bytes=nc.DEFAULT_MAX_BYTES, operations=nc.SKELETON_OPERATIONS)
    seen: set = set()
    differs = duplicates = 0
    for item in items:
        before = item["before"]
        recorder.context.update(mesh=_group_of(item["test"]), mesh_digest="", alpha=None, patch_id=None, domain_id=None)
        call = nc.prepare_call(nc.OP_SKELETON, item["blob"], before)
        key = _dedupe_key(item["blob"], before) + (json.dumps(sorted(call.kwargs.items(), key=lambda pair: pair[0]), default=str).encode(),)
        if key in seen:
            duplicates += 1
            continue
        seen.add(key)
        result, error, seconds = _replay(call)
        recorder._write(call, before, item["blob"], result, error, seconds)
        row = recorder.rows[-1]
        row.update(label="test", test=item["test"], **item.get("row", {}))
        if item.get("live") is not None:
            row["live_equal"] = item["live"] == _live_view(result, error)
            differs += int(not row["live_equal"])
    document = {
        "schema": INDEX_SCHEMA, **description, "records_count": len(recorder.rows), "total_bytes": recorder.bytes, "labels": {"test": len(recorder.rows)},
        "outcomes": dict(Counter(row["outcome"] for row in recorder.rows)), "test_live_differs": differs, "test_duplicates_after_normalization": duplicates,
        "records": recorder.rows, "domains": [], **extra,
    }
    (out / "index.json").write_text(json.dumps(document, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    return document


def write_corpus(collector: _Collector, exitstatus) -> None:
    tests = [item for item in os.environ.get(FILES_ENVIRONMENT, "").split(",") if item] or list(TEST_FILES)
    description = nc.run_description({"corpus": "synthetic_skeleton", "tests": tests, "pytest_exit": int(exitstatus)})
    extra = {"test_calls_seen": collector.calls, "test_duplicates": collector.duplicates, "skipped": dict(collector.skipped)}
    document = write_items(collector.out, collector.pending, description, extra)
    print(
        f"synthetic skeleton corpus: {document['records_count']} records from tests ({collector.calls} calls, {collector.duplicates} duplicates, "
        f"{document['test_live_differs']} differ from what the test saw) {document['total_bytes']} bytes -> {collector.out}"
    )


def rewrite(source: Path, out: Path) -> dict:
    """Переписывает уже построенный корпус `source` с холодной памятью и без повторов (исход каждой записи считается заново чистым эталоном)."""

    index = sc.load_index(source)
    items = []
    for row in sc.rows_of(source, derived=False):
        record = sc.read(source, row)
        items.append({"test": row["test"], "before": cold_state(record.before()), "blob": record.call_blob, "live": None, "row": {"live_equal_warm": row.get("live_equal")}})
    description = {key: index[key] for key in ("python", "kernel_identity", "git_head")} | {"corpus": "synthetic_skeleton", "tests": index.get("tests", []), "pytest_exit": index.get("pytest_exit")}
    return write_items(out, items, description, {"test_calls_seen": index.get("test_calls_seen"), "test_duplicates": index.get("test_duplicates"), "rewritten_from": str(source)})


# --------------------------------------------------------------------------
# команды
# --------------------------------------------------------------------------


def build(out: Path, files=TEST_FILES, extra_arguments=()) -> int:
    environment = dict(os.environ)
    environment[OUT_ENVIRONMENT] = str(out)
    environment[FILES_ENVIRONMENT] = ",".join(files)
    existing = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [str(TOOLS), str(ROOT / "kernel" / "src"), str(ROOT / "kernel" / "tests"), existing]))
    environment["PYTHONSAFEPATH"] = "1"
    paths = [str(ROOT / "kernel" / "tests" / name) for name in files]
    command = [sys.executable, "-m", "pytest", "-q", "-p", "native_skeleton_synthetic", "-p", "no:cacheprovider", *extra_arguments, *paths]
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, env=environment, cwd=str(ROOT / "kernel")).returncode


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("build", "inventory", "rewrite"))
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--src", type=Path, default=None)
    parser.add_argument("--files", default="")
    arguments = parser.parse_args(argv)
    out = arguments.out or default_out()
    if arguments.command == "inventory":
        print(json.dumps(sc.inventory(out), indent=1))
        return 0
    if arguments.command == "rewrite":
        document = rewrite(arguments.src, out)
        print(f"NATIVE_SKELETON_SYNTHETIC_REWRITE_OK {document['records_count']} {document['total_bytes']}")
        return 0
    files = tuple(item for item in arguments.files.split(",") if item) or TEST_FILES
    return build(out, files)


if __name__ == "__main__":
    sys.exit(main())
