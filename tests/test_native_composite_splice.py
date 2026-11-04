"""COMPOSITE_SPLICE_NATIVE_V1: сварка группы составного образца и грани/швы перекладываемых доменов нативно из байтов видов против пути питона.

Держится одно: при заказе `NATIVE` составной образец потокового такта РАВЕН образцу пути питона до бита - позиции и UV (байты `float64`), раскладка (грани, владельцы, домены,
швы), строки доменов (номера вершин, начала петель, соседи по стене), дайджест раскладки. Чего порт не повторил бы, он отказывает по имени, и отвечает путь питона.
"""

from __future__ import annotations

import dataclasses
import math
import random
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv import envelope_composite_native as composite_native  # noqa: E402
from cftuv import envelope_mesh_assembly as assembly  # noqa: E402
from cftuv import envelope_view_bytes as codec  # noqa: E402
from cftuv import envelope_width_composite as composite  # noqa: E402
from cftuv import envelope_width_mesh_preview as preview  # noqa: E402
from cftuv.envelope_production_weld import weld_vertices  # noqa: E402
from cftuv.envelope_width_rebase import rebased_view  # noqa: E402
from test_envelope_width_incremental import OFFSET, _no_wall_split_in_the_composite_path, _press, _quarantine, _Session, clipped, flat, unfolded  # noqa: E402,F401
from test_native_mesh_buffers import _scene  # noqa: E402

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "mesh_splice_weld"), reason="расширение cftuv_native не собрано (или собрано без mesh_splice_weld)"
)


def _same_bits(mine, theirs) -> None:
    """Два образца - один ответ побитово."""

    assert mine.positions.dtype == theirs.positions.dtype and mine.positions.tobytes() == theirs.positions.tobytes()
    assert mine.uvs.tobytes() == theirs.uvs.tobytes()
    for name in ("face_starts", "face_vertices", "face_domain", "face_owner", "seams"):
        one, other = getattr(mine.image, name), getattr(theirs.image, name)
        assert one.dtype == other.dtype and one.shape == other.shape and one.tobytes() == other.tobytes(), name
    assert mine.layout_digest == theirs.layout_digest and mine.vertex_count == theirs.vertex_count
    assert len(mine.domains) == len(theirs.domains)
    for one, other in zip(mine.domains, theirs.domains):
        assert (one.patch_id, one.token, one.loop_start, one.loop_count, one.partners) == (other.patch_id, other.token, other.loop_start, other.loop_count, other.partners)
        assert one.vertices.dtype == other.vertices.dtype and one.vertices.tobytes() == other.vertices.tobytes()
    assert sorted(mine.origins) == sorted(theirs.origins)


def _compose_both(monkeypatch, displayed, model, results, alpha, splice):
    calls = []
    for name in ("group_weld", "rebuilt_pieces"):
        original = getattr(composite_native, name)

        def spy(*args, _original=original, _name=name, **kwargs):
            found = _original(*args, **kwargs)
            calls.append((_name, found is not None))
            return found

        monkeypatch.setattr(composite_native, name, spy)
    monkeypatch.setattr(assembly, "FALLBACKS", {})
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "PYTHON")
    expected = composite.compose_sample(displayed, model, results, alpha=alpha, offset=OFFSET, splice=splice)
    assert calls == []
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "NATIVE")
    found = composite.compose_sample(displayed, model, results, alpha=alpha, offset=OFFSET, splice=splice)
    return expected, found, calls


@needs_native
@pytest.mark.parametrize(
    "case",
    [
        "clipped 1.999 structure",
        "clipped 1.999 weld",
        "unfolded 0.27 weld",
        "flat quarantined 2 weld",
        "clipped quarantined 4 weld",
        "unfolded quarantined 1 weld",
        "clipped 0.2502 plain",
        "unfolded 0.27 plain",
        "flat quarantined 0 plain",
    ],
)
def test_the_native_composite_is_the_python_composite_bit_for_bit(unfolded, clipped, flat, case, monkeypatch):
    words = case.split()
    bundle = {"unfolded": unfolded, "clipped": clipped, "flat": flat}[words[0]]
    session = _Session(bundle)
    alpha = 0.2502
    if words[1] == "quarantined":
        _quarantine(session, int(words[2]))
    else:
        alpha = float(words[1])
    if words[-1] == "weld":
        monkeypatch.setattr(composite, "_same_partition", lambda first, second: False)  # сварка группы «легла иначе»: перекладка
    c = session.controller
    plan = dataclasses.replace(session.plan(alpha), splice=words[-1] != "plain")
    run = _press(c, bundle, alpha, lean=True, skipped=plan.served)
    expected, found, calls = _compose_both(monkeypatch, c.width_displayed, c.width_model, run.results, alpha, plan.splice)
    assert not isinstance(expected, str) and not isinstance(found, str), (expected, found)
    assert expected.spliced == found.spliced and (expected.recomputed, expected.group, expected.adopted_vertices) == (found.recomputed, found.group, found.adopted_vertices)
    _same_bits(found.sample, expected.sample)
    assert calls and all(done for _name, done in calls), calls
    if words[-1] != "plain":
        assert found.spliced and ("rebuilt_pieces", True) in calls
    assert assembly.FALLBACKS == {}


# --------------------------------------------------------------------------
# Сами операции порта на случайных видах против пути питона
# --------------------------------------------------------------------------


def _group(seed: int):
    """Виды случайной сцены (`test_native_mesh_buffers._scene`) байтами и сдвиги половины из них: наклоны со знаками, нулями и `-0.0`, ширина сертификата - та же или другая."""

    rng = random.Random(10_000 + seed)
    results, offset = _scene(seed)
    datas = [item.view_bytes for item in results if getattr(item, "view_bytes", None) is not None and not codec.view_bytes_failed(item.view_bytes)]
    alpha = rng.choice((0.25, 0.031, 1.5))
    steps = []
    for data in datas:
        view = codec.decode_view(data)
        if rng.random() < 0.4:
            steps.append(None)
            continue
        grid = (0.0, -0.0, 1.0, -2.5, 1e-3, 3.0e2)
        positions = np.array([rng.choice(grid) if rng.random() < 0.5 else rng.uniform(-3.0, 3.0) for _ in range(3 * len(view.vertices.positions))], dtype=np.float64)
        loops = np.array([rng.choice(grid) if rng.random() < 0.5 else rng.uniform(-3.0, 3.0) for _ in range(2 * len(view.uvs))], dtype=np.float64)
        at = alpha if rng.random() < 0.25 else rng.choice((0.2, 0.26, 1.0 / 3.0))
        if at == alpha and rng.random() < 0.5:
            positions[0] = math.inf  # нулевой сдвиг вид не трогает вовсе: наклон, который сдвиг сделал бы не конечным, не читается
        steps.append((positions.tobytes(), loops.tobytes(), at))
    return datas, steps, alpha, offset


def _python_views(datas, steps, alpha):
    views = []
    for data, step in zip(datas, steps):
        view = codec.decode_view(data)
        if step is not None:
            view = rebased_view(view, SimpleNamespace(alpha=step[2], position_slopes=step[0], loop_slopes=step[1]), alpha)
        views.append(view)
    return views


def _native(operation, *arguments):
    answer, fallbacks = __import__("cftuv_envelope.backend", fromlist=["mesh_assembly_compute"]).mesh_assembly_compute(operation, *arguments)
    assert fallbacks == {}, fallbacks
    return answer


@needs_native
@pytest.mark.parametrize("seed", range(200))
def test_the_native_group_weld_is_the_python_weld_of_the_rebased_views(seed):
    datas, steps, alpha, offset = _group(seed)
    if not datas:
        return
    views = _python_views(datas, steps, alpha)
    expected = weld_vertices([view.vertices for view in views], offset)
    inputs = dict(enumerate(zip(datas, steps)))
    found = composite_native.group_weld(inputs, list(range(len(datas))), alpha, offset)
    assert found is not None
    assert found.positions.tobytes() == np.asarray(expected.positions, dtype=np.float64).reshape(-1, 3).tobytes()
    assert [item.tolist() for item in found.index] == [list(item) for item in expected.index]
    assert [tuple(pair) for pair in found.partners] == [tuple(pair) for pair in expected.partners]


@needs_native
@pytest.mark.parametrize("seed", range(200))
def test_the_native_rebuilt_faces_and_seams_are_the_python_ones(seed):
    datas, steps, alpha, _offset = _group(seed)
    if not datas:
        return
    rng = random.Random(20_000 + seed)
    views = dict(enumerate(_python_views(datas, steps, alpha)))
    rebuilt = set(range(len(datas)))
    fresh = {number: (None, views[number]) for number in rebuilt if steps[number] is None}
    total = sum(len(view.vertices.positions) for view in views.values())
    vertices_of = {number: np.array([rng.randrange(max(1, total // 2)) for _ in view.vertices.positions], dtype=np.int64) for number, view in views.items()}
    displayed = SimpleNamespace(domains=[SimpleNamespace(patch_id=7 * number + rng.choice((0, 1))) for number in rebuilt])
    expected = composite._python_pieces(displayed, rebuilt, views, fresh, vertices_of)
    inputs = dict(enumerate(zip(datas, steps)))
    found = composite_native.rebuilt_pieces(inputs, rebuilt, alpha, vertices_of, lambda number: displayed.domains[number].patch_id)
    assert found is not None
    for number in sorted(rebuilt):
        mine, theirs = found[0][number], expected[0][number]
        assert mine[0].tolist() == theirs[0].tolist() and mine[1].tolist() == theirs[1].tolist()
        if theirs[2] is not None:
            assert mine[2].dtype == theirs[2].dtype and mine[2].tolist() == theirs[2].tolist()
    assert np.concatenate([*found[1], found[2]]).tolist() == np.concatenate([*expected[1], expected[2]]).tolist()
    assert found[2].tolist() == expected[2].tolist()


def test_the_random_groups_move_views_and_fold_seams():
    """Случайные группы не пусты: есть сдвинутые виды (и с нулевым сдвигом), сварка находит общие вершины, складки есть."""

    moved = same = folds = merged = 0
    for seed in range(200):
        datas, steps, alpha, offset = _group(seed)
        if not datas:
            continue
        moved += sum(1 for step in steps if step is not None and step[2] != alpha)
        same += sum(1 for step in steps if step is not None and step[2] == alpha)
        views = _python_views(datas, steps, alpha)
        merged += dict(weld_vertices([view.vertices for view in views], offset).counters)["ADAPTER_WELD_VERTICES_MERGED"]
        displayed = SimpleNamespace(domains=[SimpleNamespace(patch_id=number) for number in range(len(views))])
        vertices_of = {number: np.arange(len(view.vertices.positions), dtype=np.int64) for number, view in enumerate(views)}
        folds += len(composite._python_pieces(displayed, set(range(len(views))), dict(enumerate(views)), {}, vertices_of)[2])
    assert moved > 100 and same > 10 and merged > 50 and folds > 10, (moved, same, merged, folds)


@needs_native
def test_what_the_port_would_not_repeat_it_declines_by_name():
    datas, steps, alpha, offset = _group(3)
    assert datas
    law = composite_native.WELD_LAW
    first = codec.decode_view(datas[0])
    positions, loops = np.ones(3 * len(first.vertices.positions)).tobytes(), np.ones(2 * len(first.uvs)).tobytes()
    assert alpha != 0.2 and loops
    for arguments in (
        ([b"not a view"], [None], alpha, offset, law),
        (datas[:1], [(positions[:-8], loops, 0.2)], alpha, offset, law),
        (datas[:1], [(positions, loops[:-8], 0.2)], alpha, offset, law),
        (datas[:1], [(positions + b"\0" * 8, loops, 0.2)], alpha, offset, law),
        (datas[:1], [(positions, loops + b"\0" * 8, 0.2)], alpha, offset, law),
        (datas, steps, -1.0, offset, law),
        (datas, steps, alpha, 1, law),
        (datas, steps[:-1], alpha, offset, law),
        (datas[:1], [(np.full(3 * len(codec.decode_view(datas[0]).vertices.positions), math.inf).tobytes(), np.zeros(2 * len(codec.decode_view(datas[0]).uvs)).tobytes(), 0.2)], alpha, offset, law),
    ):
        answer, fallbacks = __import__("cftuv_envelope.backend", fromlist=["mesh_assembly_compute"]).mesh_assembly_compute(composite_native.SPLICE_WELD_OPERATION, *arguments)
        assert answer is None and list(fallbacks) == [("NATIVE_PORT_UNSUPPORTED", composite_native.SPLICE_WELD_OPERATION)], fallbacks
    view = codec.decode_view(datas[0])
    short = np.zeros(max(0, len(view.vertices.positions) - 1), dtype=np.int64).tobytes()
    negative = np.full(len(view.vertices.positions), -1, dtype=np.int64).tobytes()
    for mapping in (short, negative, b"\x00" * 3):
        answer, fallbacks = __import__("cftuv_envelope.backend", fromlist=["mesh_assembly_compute"]).mesh_assembly_compute(
            composite_native.SPLICE_REBUILT_OPERATION, [datas[0]], [None], alpha, [mapping], [0]
        )
        assert answer is None and list(fallbacks) == [("NATIVE_PORT_UNSUPPORTED", composite_native.SPLICE_REBUILT_OPERATION)], fallbacks


@needs_native
def test_a_view_that_travels_as_an_object_is_a_named_fallback_and_the_python_answer(clipped, monkeypatch):
    session = _Session(clipped)
    c = session.controller
    alpha = 1.999
    plan = dataclasses.replace(session.plan(alpha), splice=True)
    run = _press(c, clipped, alpha, lean=True, skipped=plan.served)
    results = []
    for item in run.results:
        copy = item.with_changes()
        if copy.__dict__.get("view_bytes") is not None:
            object.__setattr__(copy, "view", copy.view)
            copy.__dict__.pop("view_bytes")
        results.append(copy)
    expected, found, calls = _compose_both(monkeypatch, c.width_displayed, c.width_model, results, alpha, True)
    _same_bits(found.sample, expected.sample)
    assert calls == [] and (composite_native.OUTCOME_VIEW_NOT_BYTES, composite_native.SPLICE_WELD_OPERATION) in assembly.FALLBACKS


def test_the_slopes_of_a_certificate_reach_the_port_as_the_bytes_numpy_reads():
    """Наклоны сертификата - байты либо `array("d")` (так их пишет ядро): порт получает те же байты `float64`, что читает `np.frombuffer` пути питона; иное - откат по имени."""

    from array import array

    values = array("d", [1.5, -0.0, 2.0e-3])
    assert composite_native._slope_bytes(values) == np.frombuffer(values, dtype=np.float64).tobytes()
    assert composite_native._slope_bytes(values.tobytes()) == values.tobytes()
    assert composite_native._slope_bytes(array("f", [1.0])) is None and composite_native._slope_bytes([1.0]) is None
    origin = SimpleNamespace(__dict__={}, parametric=None)
    certificate = SimpleNamespace(markers=(), vertex_count=1, position_slopes=values, loop_slopes=array("d"), alpha=0.25)

    class _Origin:
        parametric = certificate

    origin = _Origin()
    origin.view_bytes = b"VIEW"
    displayed = SimpleNamespace(origins={4: origin}, domains=[SimpleNamespace(patch_id=4)])
    assert composite_native.group_inputs(displayed, {}, [0]) == {0: (b"VIEW", (values.tobytes(), b"", 0.25))}
    certificate.loop_slopes = [0.0]
    assert composite_native.group_inputs(displayed, {}, [0]) is None

def _view_bytes(patch_id, positions, normals, refs, walls, faces, uvs):
    from cftuv.envelope_production_view import DomainViewV1, structure_token
    from cftuv.envelope_production_weld import DomainVerticesV1

    view = DomainViewV1(None, DomainVerticesV1(patch_id, positions, normals, refs, walls), faces, uvs, tuple(0 for _ in faces), (), (), "rev:splice")
    return codec.encode_view(view, structure_token(view))


@needs_native
def test_the_native_weld_keeps_the_wall_partners_in_their_order():
    """Узлы стены двух вершин источника: у сварки два соседа по стене, и порт отдаёт их в том же порядке, что путь питона (`_splice_rows` строит из него соседей домена)."""

    datas = []
    for corner, (x, y) in enumerate(((0.0, 0.0), (5.0, 1.0))):
        for number, normal in enumerate(((0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (0.6, 0.0, 0.8))):
            points = ((x, y, 0.0), (x + 1.0, y, 0.0), (x + 2.0, y, 0.0))
            datas.append(_view_bytes(3 * corner + number, points, (normal,) * 3, (f"location:src:host:{corner}", None, None), ((0, 1, 2),), ((0, 1, 2),), ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0))))
    steps = [None] * len(datas)
    expected = weld_vertices([codec.decode_view(data).vertices for data in datas], 0.01)
    assert len(expected.partners) >= 2
    found = composite_native.group_weld(dict(enumerate(zip(datas, steps))), list(range(len(datas))), 0.25, 0.01)
    assert [tuple(pair) for pair in found.partners] == [tuple(pair) for pair in expected.partners]
    assert found.positions.tobytes() == np.asarray(expected.positions, dtype=np.float64).reshape(-1, 3).tobytes()


@needs_native
def test_a_fold_between_a_recomputed_and_a_moved_domain_is_judged_on_the_moved_uvs():
    """Общее ребро посчитанного домена и сдвинутого: UV сдвинутого на ширине такта (`uv + (l - uv) * (dt / alpha)`) равны UV посчитанного - складки нет, иначе есть; порт судит так же."""

    alpha, at = 0.25, 0.2
    uvs0 = np.array([[0.1, 0.2], [0.7, 0.3], [0.4, 0.9]])
    loops = np.array([[0.3, -0.5], [1.25, 0.0625], [-0.75, 2.0]])
    moved = uvs0 + (loops - uvs0) * ((alpha - at) / alpha)
    points = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
    normals = ((0.0, 0.0, 1.0),) * 3
    for fresh_uvs, folds in ((moved, 0), (moved + 1e-3, 3)):
        served = _view_bytes(1, points, normals, (None,) * 3, (), ((0, 1, 2),), tuple(map(tuple, uvs0.tolist())))
        recomputed = _view_bytes(2, points, normals, (None,) * 3, (), ((0, 1, 2),), tuple(map(tuple, fresh_uvs.tolist())))
        datas = [served, recomputed]
        steps = [(np.zeros(9).tobytes(), loops.tobytes(), at), None]
        vertices_of = {0: np.arange(3, dtype=np.int64), 1: np.arange(3, dtype=np.int64)}
        displayed = SimpleNamespace(domains=[SimpleNamespace(patch_id=1), SimpleNamespace(patch_id=2)])
        views = dict(enumerate(_python_views(datas, steps, alpha)))
        expected = composite._python_pieces(displayed, {0, 1}, views, {1: (None, views[1])}, vertices_of)
        found = composite_native.rebuilt_pieces(dict(enumerate(zip(datas, steps))), {0, 1}, alpha, vertices_of, lambda number: displayed.domains[number].patch_id)
        assert len(expected[2]) == folds and found[2].tolist() == expected[2].tolist()
    infinite = (np.zeros(9).tobytes(), np.full(6, math.inf).tobytes(), at)
    answer, fallbacks = __import__("cftuv_envelope.backend", fromlist=["mesh_assembly_compute"]).mesh_assembly_compute(
        composite_native.SPLICE_REBUILT_OPERATION, [served], [infinite], alpha, [np.arange(3, dtype=np.int64).tobytes()], [1]
    )
    assert answer is None and list(fallbacks) == [("NATIVE_PORT_UNSUPPORTED", composite_native.SPLICE_REBUILT_OPERATION)]