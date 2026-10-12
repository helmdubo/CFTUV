"""Второй (СИНТЕТИЧЕСКИЙ) корпус W1 цепи MATERIALIZE: вызовы `mesh.exact_mesh` и `mesh.canonical_triangles` из тестов ядра, и их переисполнение эталоном.

Полевой корпус (`building`, `cover.008`: `native_corpus_export.py --operations w1`, `blender_tree_ab.py` с `CFTUV_W1_RECORD_DIR`) не доходит до веток: слияние названий вершин (`SOURCE_VERTEX_NAME_DROPPED`),
слипшиеся имена, конфликт станций, стык разомкнутого кольца, законы QUAD_STRIPS и TRIANGLES, веер от вершины, непростой контур, отказы разбиения, жадное слияние. Их строят тесты ядра на
синтетических доменах (`materialize_domain`). Этот модуль — плагин pytest (`-p native_materialize_w1_synthetic`): пока тесты идут, он ставит записывающие обёртки на имена `exact_mesh`
и `canonical_triangles` в модуле `domain` (так их зовёт диспетчер бэкенда) и кладёт вход вызова и состояние ДО. Исход считается ПОСЛЕ прогона тестов, заново, эталоном из пикла и состояния «до»
(как в `native_clip_synthetic.py`): запись описывает вызов от записанного состояния, а не ход теста.

    python tools/native_materialize_w1_synthetic.py build [--out DIR] [--processes N]     # прогон тестов ядра с плагином (подпроцесс), запись корпуса
    python tools/native_materialize_w1_synthetic.py mutate DIR [--out DIR2] [--per-record N]   # производные: законы, таблица, грани и бюджет (см. `mutations`)

`mutate` берёт настоящие вызовы (записи корпуса `DIR`) и меняет в них ровно одну вещь, чтобы дойти до ветки, которую ни поле, ни тесты не строят (каждая мутация названа и каждый её исход — тоже);
исход считает ЭТАЛОН на мутированном входе, поэтому запись верна по построению, и от порта требуется то же самое либо названный отказ, не тронувший состояние.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import json
import os
import pickle
import shutil
import subprocess
import sys
import tempfile
import time
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
for _path in (str(TOOLS), str(ROOT / "kernel" / "src"), str(ROOT / "kernel" / "tests")):
    if _path not in sys.path:
        sys.path.insert(0, _path)

import native_corpus as nc  # noqa: E402

OUT_ENVIRONMENT = "CFTUV_SYNTHETIC_W1_OUT"
#: Если задан, плагин не пишет корпус, а кладёт собранное в этот файл (процесс параллельной сборки, `build --processes N`).
ITEMS_ENVIRONMENT = "CFTUV_SYNTHETIC_W1_ITEMS"
INDEX_SCHEMA = "cftuv.native-corpus.synthetic-w1.v1"

#: Тесты ядра, которые доходят до `materialize_domain` (и потому до обоих входов W1).
TEST_FILES = (
    "test_materialize_domain.py",
    "test_materialize_full_path.py",
    "test_planar_polygon_law.py",
    "test_decal_topology_law.py",
    "test_fan_face_law.py",
    "test_convex_partition.py",
    "test_join_station_conflict.py",
    "test_materialize_chord_station.py",
    "test_plain_affine_skip.py",
    "test_silhouette_topology.py",
    "test_chain_station_plan.py",
    "test_station_plan_silhouette.py",
    "test_developable_materialize.py",
    "test_near_planar_surface_law.py",
    "test_clip_law.py",
    "test_clip_faces_law.py",
    "test_materialize_source_lift.py",
    "test_materialize_lift_surface.py",
    "test_offset_normal_opposition.py",
    "test_corner_join.py",
)


def default_out() -> Path:
    base = nc.matching_corpus() or nc.corpus_directory(nc.git_head())
    return Path(os.environ.get(OUT_ENVIRONMENT) or base / "synthetic_w1")


# --------------------------------------------------------------------------
# плагин pytest
# --------------------------------------------------------------------------


class _Collector:
    def __init__(self, out: Path) -> None:
        self.out = out
        self.pending: list[dict] = []
        self.test = ""
        self.skipped: Counter = Counter()

    def capture(self, op: str, args: tuple) -> None:
        try:
            call = nc.unpack_call(op, args, {})
            before = nc.capture_state(call.budget, None)
            blob = nc.encode_call(call)
        except Exception as exc:  # noqa: BLE001 - вызов, который корпус не несёт, учитывается, а не роняет тест
            self.skipped[f"{op}: {type(exc).__name__}"] += 1
            return
        self.pending.append({"test": self.test, "op": op, "before": before, "blob": blob})


_COLLECTOR: _Collector | None = None


def pytest_configure(config) -> None:
    global _COLLECTOR
    _COLLECTOR = _Collector(default_out())


@contextlib.contextmanager
def _capturing(collector: _Collector):
    import cftuv_envelope.materialize.domain as domain

    original_mesh, original_canonical = domain.exact_mesh, domain.canonical_triangles

    def exact_mesh(*args):
        collector.capture(nc.OP_MESH, args)
        return original_mesh(*args)

    def canonical_triangles(*args):
        collector.capture(nc.OP_CANONICAL, args)
        return original_canonical(*args)

    domain.exact_mesh, domain.canonical_triangles = exact_mesh, canonical_triangles
    try:
        yield
    finally:
        domain.exact_mesh, domain.canonical_triangles = original_mesh, original_canonical


import pytest  # noqa: E402


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_call(item):
    collector = _COLLECTOR
    collector.test = item.nodeid
    with _capturing(collector):
        yield


def pytest_sessionfinish(session, exitstatus) -> None:
    collector = _COLLECTOR
    if collector is None:
        return
    target = os.environ.get(ITEMS_ENVIRONMENT)
    if target:
        Path(target).write_bytes(pickle.dumps({"pending": collector.pending, "skipped": dict(collector.skipped), "exit": int(exitstatus)}, protocol=5))
        print(f"synthetic W1 part: {len(collector.pending)} calls -> {target}")
        return
    write_records(collector.out, collector.pending, collector.skipped, exitstatus)


# --------------------------------------------------------------------------
# запись корпуса
# --------------------------------------------------------------------------


def replay_and_write(recorder: nc.Recorder, item: dict, label: str) -> None:
    call = nc.prepare_call(item["op"], item["blob"], item["before"])
    started = time.perf_counter()
    try:
        result, error = nc.invoke(call), None
    except Exception as exc:  # noqa: BLE001 - исключение — часть исхода
        result, error = None, exc
    seconds = time.perf_counter() - started
    recorder._write(call, item["before"], item["blob"], result, error, seconds)
    recorder.rows[-1]["label"] = label


def write_records(out: Path, pending: list, skipped, exitstatus) -> None:
    out.mkdir(parents=True, exist_ok=True)
    description = nc.run_description({"corpus": "synthetic_w1", "tests": list(TEST_FILES), "pytest_exit": int(exitstatus)})
    recorder = nc.Recorder(out, description, preset=3, max_bytes=nc.DEFAULT_MAX_BYTES, operations=nc.W1_OPERATIONS)
    labels: Counter = Counter()
    for item in pending:
        recorder.context.update(mesh=item["test"], mesh_digest="", alpha=None, patch_id=None, domain_id=None)
        replay_and_write(recorder, item, "test")
        labels["test"] += 1
    recorder.write_index({"schema": INDEX_SCHEMA, "labels": dict(labels), "skipped": dict(skipped), "outcomes": dict(Counter(row["outcome"] for row in recorder.rows))})
    print(f"synthetic W1 corpus: {len(recorder.rows)} records -> {out}")


def _environment(out: Path) -> dict:
    environment = dict(os.environ)
    environment[OUT_ENVIRONMENT] = str(out)
    existing = environment.get("PYTHONPATH", "")
    environment["PYTHONPATH"] = os.pathsep.join(filter(None, [str(TOOLS), str(ROOT / "kernel" / "src"), str(ROOT / "kernel" / "tests"), existing]))
    environment["PYTHONSAFEPATH"] = "1"
    return environment


def _command(files, extra_arguments=()) -> list:
    paths = [str(ROOT / "kernel" / "tests" / name) for name in files if (ROOT / "kernel" / "tests" / name).is_file()]
    return [sys.executable, "-m", "pytest", "-q", "-p", "native_materialize_w1_synthetic", "-p", "no:cacheprovider", *extra_arguments, *paths]


def merge_parts(parts: list, files) -> tuple:
    position = {name: number for number, name in enumerate(files)}

    def place(entry: dict) -> int:
        return position[Path(entry["test"].split("::")[0]).name]

    pending = sorted((entry for part in parts for entry in part["pending"]), key=place)
    skipped: Counter = Counter()
    for part in parts:
        skipped.update(part["skipped"])
    return pending, skipped, max(part["exit"] for part in parts)


def build_parallel(out: Path, files, processes: int, extra_arguments=()) -> int:
    groups = nc.split_files([name for name in files if (ROOT / "kernel" / "tests" / name).is_file()], processes)
    folder = Path(tempfile.mkdtemp(prefix="w1-parts-"))
    running = []
    for number, group in enumerate(groups, 1):
        environment = _environment(out)
        environment[ITEMS_ENVIRONMENT] = str(folder / f"part-{number}.pkl")
        command = _command(group, extra_arguments)
        print(f"+ part {number}/{len(groups)} ({len(group)} files)", flush=True)
        running.append((number, subprocess.Popen(command, env=environment, cwd=str(ROOT / "kernel"), stdout=open(folder / f"part-{number}.log", "wb"), stderr=subprocess.STDOUT)))
    codes = [process.wait() for _, process in running]
    parts = []
    for number, _ in running:
        part = folder / f"part-{number}.pkl"
        if not part.is_file():
            print(f"NATIVE_W1_SYNTHETIC_PART_FAILED part {number} left no result; its log: {folder / f'part-{number}.log'}")
            return 1
        parts.append(pickle.loads(part.read_bytes()))
    shutil.rmtree(folder, ignore_errors=True)
    pending, skipped, exit_code = merge_parts(parts, [name for name in TEST_FILES if (ROOT / "kernel" / "tests" / name).is_file()])
    write_records(out, pending, skipped, exit_code)
    return max(codes + [exit_code])


def build(out: Path, files=TEST_FILES, extra_arguments=(), processes: int = 1) -> int:
    if processes > 1:
        return build_parallel(out, files, processes, extra_arguments)
    command = _command(files, extra_arguments)
    print("+", " ".join(command), flush=True)
    return subprocess.run(command, env=_environment(out), cwd=str(ROOT / "kernel")).returncode


# --------------------------------------------------------------------------
# производные записи: мутации настоящих вызовов
# --------------------------------------------------------------------------


def mutations(op: str, args: tuple):
    """`(имя, новые аргументы)` — по одной мутации за раз. Аргументы — те, что хранит запись (`native_corpus.unpack_call`); исходный вход не меняется."""

    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1

    if op == nc.OP_CANONICAL:
        frame_faces, cycles, reverse = args
        yield "reverse-flipped", (frame_faces, cycles, not reverse)
        if cycles and len(cycles[0]) >= 4:
            broken = [list(cycle) for cycle in cycles]
            broken[0] = broken[0][:-1]  # a contour that lost a vertex: the areas do not close
            yield "contour-lost-a-vertex", (frame_faces, broken, reverse)
            twisted = [list(cycle) for cycle in cycles]
            twisted[0][1], twisted[0][2] = twisted[0][2], twisted[0][1]  # a twisted contour
            yield "contour-twisted", (frame_faces, twisted, reverse)
        return
    frame_faces, region_ids, layout, table, notes, chord_names, lattice_alpha, chart_cw, law, exact_plane = args
    base = (frame_faces, region_ids, layout, table, notes, chord_names, lattice_alpha, chart_cw, law, exact_plane)

    def with_(**changes):
        names = ("frame_faces", "region_ids", "layout", "table", "notes", "chord_names", "lattice_alpha", "chart_cw", "law", "exact_plane")
        values = dict(zip(names, base))
        values.update(changes)
        return tuple(values[name] for name in names)

    for other in (DecalTopologyLawV1.TRIANGLES_V1, DecalTopologyLawV1.QUAD_STRIPS_V1, DecalTopologyLawV1.PLANAR_POLYGONS_V1):
        if other is not law:
            yield f"law-{other.value}", with_(law=other)
    yield "exact-plane-flipped", with_(exact_plane=not exact_plane)
    yield "chart-cw-flipped", with_(chart_cw=not chart_cw)
    yield "triangles-on-a-curved-plane", with_(law=DecalTopologyLawV1.QUAD_STRIPS_V1, exact_plane=False)
    if table.joins:
        yield "no-joins", with_(table=dataclasses.replace(table, joins={}))
    runs = {}
    for frame_face in frame_faces:
        runs.setdefault(frame_face.run.run_id, frame_face.run)
    if table.joins and not table.frame_of_run:
        frames = {run_id: f"frame:{number % 2}" for number, run_id in enumerate(sorted(runs))}
        yield "frames-split-the-runs", with_(table=dataclasses.replace(table, frame_of_run=frames))
    if table.node_vertex_ids:
        ids = dict(table.node_vertex_ids)
        keys = list(ids)
        first = keys[0]
        yield "vertex-name-collision", with_(table=dataclasses.replace(table, node_vertex_ids={**ids, keys[-1]: ids[first]})) if len(keys) > 1 else with_()
        renamed = {key: f"{value}~{number % 3}" for number, (key, value) in enumerate(ids.items())}
        yield "vertex-names-differ-by-region", with_(table=dataclasses.replace(table, node_vertex_ids=renamed))
    if len(frame_faces) > 1:
        swapped = list(frame_faces)
        swapped[0] = dataclasses.replace(swapped[0], run=frame_faces[-1].run)
        yield "run-of-another-face", with_(frame_faces=swapped)
        lost = list(frame_faces)
        lost[0] = dataclasses.replace(lost[0], face=dataclasses.replace(lost[0].face, points=lost[0].face.points[:-1]) if len(lost[0].face.points) > 3 else lost[0].face)
        yield "face-lost-a-point", with_(frame_faces=lost)
        twisted = list(frame_faces)
        points = list(twisted[0].face.points)
        if len(points) >= 4:
            points[1], points[2] = points[2], points[1]
            twisted[0] = dataclasses.replace(twisted[0], face=dataclasses.replace(twisted[0].face, points=tuple(points)))
            yield "face-twisted", with_(frame_faces=twisted)
    for index, frame_face in enumerate(frame_faces):
        if len(getattr(frame_face.face, "parts", ())) >= 2:
            parts = list(frame_face.face.parts)
            parts[0] = dataclasses.replace(parts[0], points=parts[0].points[:-1] if len(parts[0].points) > 3 else parts[0].points)
            changed = list(frame_faces)
            changed[index] = dataclasses.replace(frame_face, face=dataclasses.replace(frame_face.face, parts=tuple(parts)))
            yield "part-lost-a-point", with_(frame_faces=changed)
            break
    for index, frame_face in enumerate(frame_faces):
        if frame_face.is_fan:
            changed = list(frame_faces)
            changed[index] = dataclasses.replace(frame_face, face=dataclasses.replace(frame_face.face, owner=(10**6, 10**6) + tuple(frame_face.face.owner[2:])))
            yield "fan-apex-outside-the-contour", with_(frame_faces=changed)
            break
    for index, frame_face in enumerate(frame_faces):
        points = list(frame_face.face.points)
        if frame_face.is_fan and len(points) > 3:
            # a fan cell of more than three points: the polygon route, the apex route (plane off), the apex elsewhere, a twisted cell
            yield "fan-cell-on-a-curved-plane", with_(exact_plane=False)
            twisted = list(frame_faces)
            swapped = list(points)
            swapped[1], swapped[2] = swapped[2], swapped[1]
            twisted[index] = dataclasses.replace(frame_face, face=dataclasses.replace(frame_face.face, points=tuple(swapped)))
            yield "fan-cell-twisted", with_(frame_faces=twisted)
            yield "fan-cell-twisted-on-a-curved-plane", with_(frame_faces=twisted, exact_plane=False)
            node = tuple(coordinate.as_rational() for coordinate in points[1])
            if all(value is not None and value.denominator == 1 for value in node):
                moved = list(frame_faces)
                moved[index] = dataclasses.replace(frame_face, face=dataclasses.replace(frame_face.face, owner=(int(node[0]), int(node[1])) + tuple(frame_face.face.owner[2:])))
                yield "fan-apex-moved-into-the-contour", with_(frame_faces=moved, exact_plane=False)
            break
    for index, frame_face in enumerate(frame_faces):
        points = list(frame_face.face.points)
        if not frame_face.is_fan and len(points) >= 5:
            twisted = list(frame_faces)
            swapped = list(points)
            swapped[1], swapped[3] = swapped[3], swapped[1]
            twisted[index] = dataclasses.replace(frame_face, face=dataclasses.replace(frame_face.face, points=tuple(swapped)))
            yield "strip-twisted", with_(frame_faces=twisted)
            break
    if table.node_vertex_ids and len(set(region_ids)) >= 1 and len(frame_faces) >= 2:
        names = [f"{region}#{number % 2}" for number, region in enumerate(region_ids)]
        nodes = {}
        for (region, node), vertex_id in table.node_vertex_ids.items():
            nodes[(f"{region}#0", node)] = vertex_id
            nodes[(f"{region}#1", node)] = f"{vertex_id}~b"
        yield "regions-split-and-renamed", with_(region_ids=names, table=dataclasses.replace(table, node_vertex_ids=nodes))


def derive_budgets(roots: list, out: Path, *, shares=(0.05, 0.2, 0.4, 0.6, 0.8, 0.95), exhaustive_below: int = 40) -> dict:
    """Records of the same calls with a ceiling below their cost: every unit of the ceiling up to `exhaustive_below` units (every charging site of a small call), the shares above it."""

    out.mkdir(parents=True, exist_ok=True)
    recorder = nc.Recorder(out, nc.run_description({"corpus": "w1_budgets"}), preset=3, max_bytes=nc.DEFAULT_MAX_BYTES, operations=nc.W1_OPERATIONS)
    tally: Counter = Counter()
    for root in roots:
        for row, record in _records(Path(root)):
            before, expected = record.before(), record.expected()
            if before.budget is None:
                continue
            spent = sum(after - was for after, was in zip(expected.after.budget["articles"], before.budget["articles"]))
            if spent < 1:
                continue
            caps = range(spent) if spent <= exhaustive_below else sorted({int(spent * share) for share in shares})
            for cap in caps:
                starved = nc.bounded_before(before, sum(before.budget["articles"]) + cap)
                recorder.context.update(mesh=row.get("mesh", ""), mesh_digest="", alpha=None, patch_id=None, domain_id=None)
                replay_and_write(recorder, {"op": record.op, "blob": record.call_blob, "before": starved}, f"budget-{cap}-of-{spent}")
                tally["budget"] += 1
    recorder.write_index({"schema": INDEX_SCHEMA, "labels": dict(tally), "outcomes": dict(Counter(row["outcome"] for row in recorder.rows))})
    return dict(tally)


def _records(root: Path):
    index = nc.load_index(root)
    for row in index["records"]:
        if row.get("derived") is None and row["op"] in nc.W1_OPERATIONS:
            yield row, nc.read_record(root / row["path"])


def derive_mutations(roots: list, out: Path, *, per_record: int = 4, per_name: int = 60, stride: int = 1) -> dict:
    """One record per mutation: a mutation is yielded only by a call it applies to, and a name is taken from the calls that yield it until it has `per_name` records (at most `per_record` per call)."""

    out.mkdir(parents=True, exist_ok=True)
    recorder = nc.Recorder(out, nc.run_description({"corpus": "w1_mutations"}), preset=3, max_bytes=nc.DEFAULT_MAX_BYTES, operations=nc.W1_OPERATIONS)
    tally: Counter = Counter()
    seen: Counter = Counter()
    for root in roots:
        for number, (row, record) in enumerate(_records(Path(root))):
            if number % stride:
                continue
            before = record.before()
            call = nc.prepare_call(record.op, record.call_blob, before)
            chosen = [(name, args) for name, args in mutations(record.op, call.args) if seen[name] < per_name]
            for name, args in chosen[:per_record]:
                mutated = nc.Call(record.op, args, {}, call.budget, None)
                try:
                    blob = nc.encode_call(mutated)
                except Exception as exc:  # noqa: BLE001 - a mutation the corpus cannot carry is counted, not hidden
                    tally[f"unpicklable:{name}:{type(exc).__name__}"] += 1
                    continue
                recorder.context.update(mesh=row.get("mesh", ""), mesh_digest="", alpha=None, patch_id=None, domain_id=None)
                replay_and_write(recorder, {"op": record.op, "blob": blob, "before": before}, f"mutation-{name}")
                seen[name] += 1
                tally[name] += 1
    recorder.write_index({"schema": INDEX_SCHEMA, "labels": dict(tally), "outcomes": dict(Counter(row["outcome"] for row in recorder.rows))})
    return dict(tally)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("command", choices=("build", "mutate", "budgets"))
    parser.add_argument("corpus", nargs="*", help="mutate / budgets: the corpora whose calls are the base")
    parser.add_argument("--out", type=Path, default=None)
    parser.add_argument("--processes", type=int, default=1)
    parser.add_argument("--per-record", type=int, default=4)
    parser.add_argument("--per-name", type=int, default=60)
    parser.add_argument("--stride", type=int, default=1)
    arguments = parser.parse_args(argv)
    if arguments.command == "build":
        return build(arguments.out or default_out(), processes=arguments.processes)
    roots = [Path(item) for item in arguments.corpus]
    if arguments.command == "mutate":
        result = derive_mutations(roots, arguments.out or default_out().parent / "w1_mutations", per_record=arguments.per_record, per_name=arguments.per_name, stride=arguments.stride)
    else:
        result = derive_budgets(roots, arguments.out or default_out().parent / "w1_budgets")
    print(json.dumps(result, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
