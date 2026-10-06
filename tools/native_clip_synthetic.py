"""Второй (СИНТЕТИЧЕСКИЙ) корпус вызовов резки: записи `clip_geometry` и швов из тестов ядра, и инвентаризация веток.

Полевой корпус (117 вызовов `clip_geometry`) не доходит до веток: `by_faces=False`, законы QUAD_STRIPS/TRIANGLES, прямые вершины,
свес, шовное и «вершина не в углу» подавление, отказы, `doubled_shoelace`, нецелая карта, знаки сопряжением. Их строят тесты ядра на
синтетических плоскостях (`kernel/tests/test_clip_*.py` и те, что проходят через `materialize_domain`). Этот модуль — плагин pytest
(`-p native_clip_synthetic`): пока тесты идут, он ставит записывающие обёртки на `clip_geometry`, `_cut_by_faces`, `ClipStageV1`
(стадия по треугольникам без `cells` — это ровно `clip_geometry(by_faces=False)`) и на швы (`native_clip_seams.SeamRecorder`).

Запись вызова — как в полевом корпусе (`native_corpus.Recorder`): состояние ДО, пикл входов, исход эталона и состояние ПОСЛЕ. Исход
считается ПОСЛЕ прогона тестов, заново, из пикла и состояния «до» (тест мог сделать между созданием стадии и `run` что угодно:
запись описывает вызов `clip_geometry` от записанного состояния, а не ход теста). Вызовы швов пишутся пиклом по тесту.

    python tools/native_clip_synthetic.py build [--out DIR]      # прогон тестов ядра с плагином (подпроцесс), индекс
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import json
import lzma
import os
import pickle
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

OUT_ENVIRONMENT = "CFTUV_SYNTHETIC_CLIP_OUT"
INDEX_SCHEMA = "cftuv.native-corpus.synthetic-clip.v1"

#: Тесты ядра, которые доходят до резки (`clip_geometry`, стадия, швы). Остальные тесты ядра не пишутся: плагин пишет только то, что вызвано.
TEST_FILES = (
    "test_clip_law.py",
    "test_clip_faces_law.py",
    "test_clip_plan_inert.py",
    "test_clip_snap.py",
    "test_clip_speed_paths.py",
    "test_materialize_tessellate.py",
    "test_materialize_lift_surface.py",
    "test_materialize_source_lift.py",
    "test_materialize_domain.py",
    "test_planar_polygon_law.py",
    "test_decal_topology_law.py",
    "test_fan_face_law.py",
    "test_convex_partition.py",
    "test_offset_normal_opposition.py",
    "test_materialize_full_path.py",
    "test_silhouette_topology.py",
    "test_chain_station_plan.py",
    "test_station_plan_silhouette.py",
    "test_developable_materialize.py",
    "test_near_planar_surface_law.py",
)


def default_out() -> Path:
    """`<полевой корпус ЭТОГО ядра>/synthetic_clip`; полевого корпуса под это ядро нет — `<база>/<HEAD>/synthetic_clip` (синтетический корпус не зависит от поля)."""

    base = nc.matching_corpus() or nc.corpus_directory(nc.git_head())
    return Path(os.environ.get(OUT_ENVIRONMENT) or base / "synthetic_clip")


def _safe(text: str) -> str:
    return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in text)[:120]


# --------------------------------------------------------------------------
# плагин pytest: записывающие обёртки на время каждого теста
# --------------------------------------------------------------------------


class _Collector:
    """Что собрано за сессию: вызовы `clip_geometry` (вход + состояние до) и вызовы швов по тестам."""

    def __init__(self, out: Path) -> None:
        self.out = out
        self.pending: list[dict] = []
        self.seam_files: list[dict] = []
        self.depth = 0
        self.test = ""
        self.skipped: Counter = Counter()

    def capture(self, label: str, plane, budget, kwargs: dict) -> None:
        try:
            before = nc.capture_state(budget, None)
            blob = nc.encode_call(nc.Call(nc.OP_CLIP, (plane,), kwargs, budget, None))
        except Exception as exc:  # noqa: BLE001 - вызов, который корпус не несёт, учитывается, а не роняет тест
            self.skipped[f"{label}: {type(exc).__name__}"] += 1
            return
        self.pending.append({"test": self.test, "label": label, "before": before, "blob": blob})


_COLLECTOR: _Collector | None = None


_TRACER = None
COVERAGE_ENVIRONMENT = "CFTUV_CLIP_COVERAGE"


def pytest_configure(config) -> None:
    global _COLLECTOR, _TRACER
    _COLLECTOR = _Collector(default_out())
    if os.environ.get(COVERAGE_ENVIRONMENT):
        import native_clip_coverage as coverage

        _TRACER = coverage.Tracer()
        _TRACER.start()


def _clip_kwargs(points, cycles, polygons, law, seam, fans, flows, by_faces, inert=frozenset()) -> dict:
    """Входы `clip_geometry`; `inert` (пары плана станций цепей) пишется, только когда он есть: записи без плана остаются в прежней форме."""

    kwargs = {"points": points, "cycles": cycles, "polygons": polygons, "law": law, "seam": seam, "fans": fans, "flows": flows, "by_faces": by_faces}
    if inert:
        kwargs["inert"] = inert
    return kwargs


_ALIASES: list | None = None


def _aliases_of(*originals) -> list:
    """`[(модуль теста, имя, номер оригинала)]`: модули тестов, которые импортировали функцию резки ПО ИМЕНЕ (`from ...clip import _cut_by_faces`).

    Подмена атрибута модуля `clip` их не задевает, поэтому обёртка ставится и на эти имена: иначе вызовы, которые тесты делают напрямую
    (`test_clip_plan_inert.py`), в корпус не попадут. Список строится один раз за прогон: все модули тестов уже импортированы сбором.
    """

    global _ALIASES
    if _ALIASES is None:
        found = []
        for module in list(sys.modules.values()):
            name = getattr(module, "__name__", "") or ""
            if not name.startswith("test_"):
                continue
            for attribute, value in list(vars(module).items()):
                for number, original in enumerate(originals):
                    if value is original:
                        found.append((module, attribute, number))
        _ALIASES = found
    return _ALIASES


@contextlib.contextmanager
def _capturing(collector: _Collector):
    """Обёртки захвата входов на время теста; снимаются в точности."""

    import cftuv_envelope.materialize.clip as clip
    import cftuv_envelope.materialize.clip_memo as clip_memo

    original_geometry, original_faces = clip.clip_geometry, clip._cut_by_faces
    original_init, original_run = clip.ClipStageV1.__init__, clip.ClipStageV1.run
    clip_memo.MEMO.enabled = False

    def geometry(plane, budget, *, points, cycles, polygons, law, seam, fans, flows, by_faces, inert=frozenset()):
        if collector.depth == 0:
            collector.capture("clip_geometry", plane, budget, _clip_kwargs(points, cycles, polygons, law, seam, fans, flows, by_faces, inert))
        collector.depth += 1
        try:
            return original_geometry(plane, budget, points=points, cycles=cycles, polygons=polygons, law=law, seam=seam, fans=fans, flows=flows, by_faces=by_faces, inert=inert)
        finally:
            collector.depth -= 1

    def cut_by_faces(plane, budget, points, cycles, polygons, law, seam, fans, flows=None, inert=frozenset()):
        if collector.depth == 0:
            collector.capture("_cut_by_faces", plane, budget, _clip_kwargs(points, cycles, polygons, law, seam, fans, flows, True, inert))
        collector.depth += 1
        try:
            return original_faces(plane, budget, points, cycles, polygons, law, seam, fans, flows, inert)
        finally:
            collector.depth -= 1

    def init(self, plane, budget, points, cells=None, shared=None):
        self._synthetic = None
        if collector.depth == 0 and cells is None and shared is None:
            try:
                self._synthetic = {"before": nc.capture_state(budget, None), "plane": plane, "budget": budget, "points": points, "runs": 0}
            except Exception as exc:  # noqa: BLE001
                collector.skipped[f"ClipStageV1: {type(exc).__name__}"] += 1
        return original_init(self, plane, budget, points, cells, shared)

    def run(self, cycles, polygons, law, seam=frozenset(), fans=None, cuts=None):
        info = getattr(self, "_synthetic", None)
        if info is not None and collector.depth == 0 and info["runs"] == 0:
            info["runs"] += 1
            try:
                kwargs = _clip_kwargs(info["points"], cycles, polygons, law, seam, fans, self.flows, False)
                blob = nc.encode_call(nc.Call(nc.OP_CLIP, (info["plane"],), kwargs, info["budget"], None))
                collector.pending.append({"test": collector.test, "label": "ClipStageV1.run", "before": info["before"], "blob": blob})
            except Exception as exc:  # noqa: BLE001
                collector.skipped[f"ClipStageV1.run: {type(exc).__name__}"] += 1
        collector.depth += 1
        try:
            return original_run(self, cycles, polygons, law, seam, fans, cuts)
        finally:
            collector.depth -= 1

    aliases = _aliases_of(original_geometry, original_faces)
    wrappers = (geometry, cut_by_faces)
    clip.clip_geometry, clip._cut_by_faces = geometry, cut_by_faces
    clip.ClipStageV1.__init__, clip.ClipStageV1.run = init, run
    for module, attribute, number in aliases:
        setattr(module, attribute, wrappers[number])
    try:
        yield
    finally:
        for module, attribute, number in aliases:
            setattr(module, attribute, (original_geometry, original_faces)[number])
        clip.clip_geometry, clip._cut_by_faces = original_geometry, original_faces
        clip.ClipStageV1.__init__, clip.ClipStageV1.run = original_init, original_run


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_call(item):
    """На время вызова теста: записывающие обёртки швов и захват входов `clip_geometry`; всё собранное кладётся в сборщик."""

    import native_clip_seams as seams

    collector = _COLLECTOR
    recorder = seams.SeamRecorder(seams.Sampling(head=150, stride=25, cap=500))
    collector.test = item.nodeid
    with recorder.installed(), _capturing(collector):
        yield
    if recorder.calls:
        collector.seam_files.append({"test": item.nodeid, "calls": recorder.calls, "seen": dict(recorder.seen)})


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


def write_corpus(collector: _Collector, exitstatus) -> None:
    out = collector.out
    out.mkdir(parents=True, exist_ok=True)
    description = nc.run_description({"corpus": "synthetic_clip", "tests": list(TEST_FILES), "pytest_exit": int(exitstatus)})
    recorder = nc.Recorder(out, description, preset=3, max_bytes=nc.DEFAULT_MAX_BYTES)
    labels: Counter = Counter()
    for item in collector.pending:
        recorder.context.update(mesh=item["test"], mesh_digest="", alpha=None, patch_id=None, domain_id=None)
        call = nc.prepare_call(nc.OP_CLIP, item["blob"], item["before"])
        started = time.perf_counter()
        try:
            result, error = nc.invoke(call), None
        except Exception as exc:  # noqa: BLE001 - исключение — часть исхода
            result, error = None, exc
        seconds = time.perf_counter() - started
        recorder._write(call, item["before"], item["blob"], result, error, seconds)
        recorder.rows[-1]["label"] = item["label"]
        labels[item["label"]] += 1
    import native_clip_generated as generated

    generated_outcomes = generated.generate(recorder)
    labels["generated"] = sum(generated_outcomes.values())
    seam_rows = []
    seams_dir = out / "seams"
    seams_dir.mkdir(parents=True, exist_ok=True)
    for number, entry in enumerate(collector.seam_files, 1):
        path = seams_dir / f"{number:04d}-{_safe(entry['test'])}.seams"
        body = lzma.compress(pickle.dumps(entry["calls"], protocol=5), format=lzma.FORMAT_XZ, preset=3)
        path.write_bytes(body)
        per = Counter(call.seam for call in entry["calls"])
        seam_rows.append({"test": entry["test"], "path": f"seams/{path.name}", "bytes": len(body), "calls": dict(per), "seen": entry["seen"]})
    document = {
        "schema": INDEX_SCHEMA,
        **description,
        "records_count": len(recorder.rows),
        "labels": dict(labels),
        "skipped": dict(collector.skipped),
        "generated_outcomes": dict(generated_outcomes),
        "outcomes": dict(Counter(row["outcome"] for row in recorder.rows)),
        "seam_files": seam_rows,
        "seam_calls": dict(sum((Counter(row["calls"]) for row in seam_rows), Counter())),
        "records": recorder.rows,
    }
    (out / "index.json").write_text(json.dumps(document, ensure_ascii=False, indent=0, sort_keys=True) + "\n", encoding="utf-8")
    print(f"synthetic clip corpus: {len(recorder.rows)} clip records, {sum(document['seam_calls'].values())} seam calls -> {out}")


def read_seams(path: Path) -> list:
    return pickle.loads(lzma.decompress(Path(path).read_bytes()))


def load_index(out: Path | None = None) -> dict:
    return json.loads(((out or default_out()) / "index.json").read_text(encoding="utf-8"))


# --------------------------------------------------------------------------
# команды
# --------------------------------------------------------------------------


def build(out: Path, files=TEST_FILES, extra_arguments=()) -> int:
    environment = dict(os.environ)
    environment[OUT_ENVIRONMENT] = str(out)
    existing = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [str(TOOLS), str(ROOT / "kernel" / "src"), str(ROOT / "kernel" / "tests"), existing]))
    environment["PYTHONSAFEPATH"] = "1"
    paths = [str(ROOT / "kernel" / "tests" / name) for name in files]
    command = [sys.executable, "-m", "pytest", "-q", "-p", "native_clip_synthetic", "-p", "no:cacheprovider", *extra_arguments, *paths]
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, env=environment, cwd=str(ROOT / "kernel")).returncode


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("build",))
    parser.add_argument("--out", type=Path, default=None)
    arguments = parser.parse_args(argv)
    return build(arguments.out or default_out())


if __name__ == "__main__":
    sys.exit(main())
