"""MESH_ASSEMBLY_BUFFERS_V1 (срез B4a): массивы меша декали нативной сборкой прямо из байтов видов (`cftuv_native.mesh_assemble`) против эталона `build_mesh_arrays`.

Держится одно: при заказе `NATIVE` массивы РАВНЫ эталону `PYTHON` до бита - каждое поле `MeshArraysV1` (кортежи разворачиваются из буферов при первом чтении, `-0.0` и
`float.hex` те же), дайджест и отчёты, предупреждения с их текстами; буферы - те же числа, что кортежи. Чего порт не повторил бы, он отказывает по имени, и отвечает
эталон: вид объектом, а не байтами, байты не этой версии или обрезанные, не конечное число.
"""

from __future__ import annotations

import dataclasses
import math
import random
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv import envelope_mesh_assembly as assembly  # noqa: E402
from cftuv import envelope_view_bytes as codec  # noqa: E402
from cftuv.envelope_production_mesh import MeshArraysV1, build_mesh_arrays  # noqa: E402
from cftuv.envelope_production_view import DomainViewV1, structure_token  # noqa: E402
from cftuv.envelope_production_weld import DomainVerticesV1  # noqa: E402

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "mesh_assemble"), reason="расширение cftuv_native не собрано (или собрано без mesh_assemble)"
)


def _exact(value):
    """Значение, где каждое число - его `float.hex` (`-0.0` и `0.0` различны, как биты меша)."""

    if isinstance(value, float):
        return value.hex()
    if isinstance(value, (tuple, list)):
        return tuple(_exact(item) for item in value)
    return value


def _fields(arrays: MeshArraysV1) -> dict:
    return {item.name: _exact(getattr(arrays, item.name)) for item in dataclasses.fields(arrays) if item.name != "buffers"}


class _Result:
    """Материализованный результат домена, каким его видит писатель меша: вид байтами (`view_bytes`), разбор - при чтении `view`."""

    outcome = "MATERIALIZED"
    detail = ""
    is_materialized = True

    def __init__(self, patch_id, data, *, normal=(0.0, 0.0, 1.0), flipped=0, law="QUAD_STRIPS_V1"):
        self.patch_id = patch_id
        self.domain_id = f"domain:{patch_id}"
        self.view_bytes = data
        self.source_normal = normal
        self.counters = (("MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE", flipped),)
        self.decal_topology_law = law

    @property
    def view(self):
        return codec.decode_view(self.view_bytes)


class _Refused:
    is_materialized = False

    def __init__(self, patch_id):
        self.patch_id, self.domain_id, self.outcome, self.detail = patch_id, f"domain:{patch_id}", "MATERIALIZE_REFUSED", "named"


def _unit(rng):
    while True:
        vector = tuple(rng.uniform(-1.0, 1.0) for _ in range(3))
        length = math.sqrt(sum(item * item for item in vector))
        if length > 0.1:
            return tuple(item / length for item in vector)


def _scene(seed: int):
    """Домены со всем, что сборка обязана повторить: общие вершины исходника (сварка, расхождение на ulp, митра), стены, грани 3-6 углов, швы, UV с `-0.0` и ulp-скачками, цепи."""

    rng = random.Random(seed)
    anchors = [tuple(rng.uniform(-2.0, 2.0) for _ in range(3)) for _ in range(rng.randint(1, 6))]
    shared = [_unit(rng) for _ in anchors]
    grid = [0.0, -0.0, 0.5, 1.0, math.nextafter(1.0, 2.0), 0.25, 1.0 / 3.0, 2.0]
    results = []
    for patch_id in sorted(rng.sample(range(40), rng.randint(1, 6))):
        positions, normals, refs = [], [], []
        for number, point in enumerate(anchors):
            if rng.random() < 0.35:
                continue
            if rng.random() < 0.1:
                point = (math.nextafter(point[0], 9.0), point[1], point[2])
            positions.append(point)
            normals.append(shared[number] if rng.random() < 0.6 else _unit(rng))
            refs.append(f"location:src:host:{number}")
        for _ in range(rng.randint(3, 12)):
            positions.append(tuple(rng.uniform(-2.0, 2.0) * rng.choice((1.0, 1e-3, 37.5)) for _ in range(3)))
            normals.append(_unit(rng))
            refs.append(rng.choice((None, None, f"location:node:{rng.randint(0, 5)}", "местный·ключ")))
        count = len(positions)
        walls = tuple(tuple(rng.sample(range(count), k=min(count, rng.randint(2, 4)))) for _ in range(rng.randint(0, 2)))
        faces = tuple(tuple(rng.sample(range(count), k=min(count, rng.choice((3, 4, 4, 5, 6))))) for _ in range(rng.randint(1, 10)))
        uvs = tuple((rng.choice(grid), rng.choice(grid)) if rng.random() < 0.8 else (rng.random(), rng.random()) for loop in faces for _ in loop)
        owners = tuple(rng.randint(-2, 9) for _ in faces)
        seams = tuple(tuple(rng.sample(range(count), 2)) for _ in range(rng.randint(0, 4)))
        pool = [f"src:{index}" for index in range(5)] + [f"clip:{index}" for index in range(2)] + ["node:1"]
        chains = tuple(
            (f"boundary:{rng.choice(('SOURCE', 'SOURCE', 'WALL', 'INTERFACE'))}:{patch_id}:{index}", tuple(rng.choice(pool) for _ in range(rng.randint(0, 5))))
            for index in range(rng.randint(0, 3))
        )
        vertices = DomainVerticesV1(patch_id, tuple(positions), tuple(normals), tuple(refs), walls)
        view = DomainViewV1(None, vertices, faces, uvs, owners, seams, chains, f"rev:{seed}")
        results.append(_Result(patch_id, codec.encode_view(view, structure_token(view)), normal=rng.choice(((0.0, 0.0, 1.0), None, (0.0, 0.0, 0.0))), flipped=rng.choice((0, 0, 2))))
    if rng.random() < 0.3:
        results.append(_Refused(99))
    if rng.random() < 0.3:
        failed = DomainViewV1(("ADAPTER_NON_FINITE_BATCH", "vertex k:7 - ∞"), None, (), (), (), (), (), f"rev:{seed}")
        results.append(_Result(98, codec.encode_view(failed, "")))
    rng.shuffle(results)
    return results, rng.choice((0.0, 0.001, 0.0125, 1.0))


def _both(monkeypatch, results, offset, reports=True):
    monkeypatch.setattr(assembly, "FALLBACKS", {})
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "PYTHON")
    expected = build_mesh_arrays(results, offset, reports=reports)
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "NATIVE")
    found = build_mesh_arrays(results, offset, reports=reports)
    return expected, found


@needs_native
@pytest.mark.parametrize("seed", range(300))
def test_random_scenes_assemble_as_the_reference(monkeypatch, seed):
    results, offset = _scene(seed)
    reports = seed % 4 != 3
    expected, found = _both(monkeypatch, results, offset, reports)

    assert expected.buffers is None and found.buffers is not None
    assert assembly.FALLBACKS == {}
    assert _fields(found) == _fields(expected)
    assert found == expected
    assert (found.vertex_count, found.face_count, found.loop_count, found.seam_count, found.corner_counts()) == (
        expected.vertex_count, expected.face_count, expected.loop_count, expected.seam_count, expected.corner_counts()
    )


def test_the_random_scenes_reach_the_weld_and_the_reports():
    """The scenes are not trivial: the reference welds, misses by an ulp, falls back from the miter, marks folds, conflicts and reports seam junctions."""

    import os

    seen = set()
    old = os.environ.get(assembly.MESH_ASSEMBLY_ENVIRONMENT)
    os.environ[assembly.MESH_ASSEMBLY_ENVIRONMENT] = "PYTHON"
    try:
        for seed in range(300):
            results, offset = _scene(seed)
            arrays = build_mesh_arrays(results, offset)
            counters = dict(arrays.weld_counters)
            seen.update(name for name, value in counters.items() if value)
            seen.update(name for name, value in arrays.seam_counters if value)
            seen.update(name for _patch, name, _detail in arrays.warnings)
            seen.add("skipped" if arrays.skipped else "none")
    finally:
        if old is None:
            os.environ.pop(assembly.MESH_ASSEMBLY_ENVIRONMENT, None)
        else:
            os.environ[assembly.MESH_ASSEMBLY_ENVIRONMENT] = old
    for name in (
        "ADAPTER_WELD_GROUPS", "ADAPTER_WELD_VERTICES_MERGED", "ADAPTER_WELD_POSITION_MISMATCH", "ADAPTER_WELD_SEAMS_MARKED",
        "ADAPTER_WELD_HALF_EDGE_CONFLICT", "ADAPTER_SEAM_T_JUNCTIONS", "ADAPTER_SOURCE_NORMAL_UNKNOWN", "skipped",
    ):
        assert any(name in item for item in seen), name


@needs_native
def test_the_tuples_are_built_on_first_read_and_the_buffers_are_the_same_numbers(monkeypatch):
    results, offset = _scene(11)
    expected, found = _both(monkeypatch, results, offset)

    for name in ("positions", "faces", "uvs", "face_domain", "face_owner", "seam_edges", "domain_vertex_index"):
        with pytest.raises(AttributeError):
            object.__getattribute__(found, name)
    buffers = found.buffers
    assert buffers.positions_array.tolist() == [list(point) for point in expected.positions]
    assert buffers.uvs_array.tolist() == [list(uv) for uv in expected.uvs]
    assert buffers.face_vertices.tolist() == [item for loop in expected.faces for item in loop]
    assert buffers.face_domain_array.tolist() == list(expected.face_domain) and buffers.face_owner_array.tolist() == list(expected.face_owner)
    assert [buffers.domain_index(number).tolist() for number in range(len(found.domain_keys))] == [list(item) for item in expected.domain_vertex_index]
    assert _exact(found.positions) == _exact(expected.positions)
    assert object.__getattribute__(found, "positions") is found.positions


@needs_native
def test_a_view_that_is_an_object_is_a_named_fallback_and_the_reference_answers(monkeypatch):
    results, offset = _scene(5)
    target = next(item for item in results if isinstance(item, _Result))
    carried = _Result(target.patch_id, None)
    del carried.view_bytes
    carried.__class__ = type("_ObjectResult", (_Result,), {"view": target.view})
    results = [carried if item is target else item for item in results]
    expected, found = _both(monkeypatch, results, offset)

    assert found.buffers is None and _fields(found) == _fields(expected)
    assert list(assembly.FALLBACKS) == [("NATIVE_PORT_UNSUPPORTED", "mesh_assemble")]


@needs_native
@pytest.mark.parametrize(
    "spoil",
    [
        pytest.param(lambda data: data[:-3], id="truncated"),
        pytest.param(lambda data: data[:4] + b"\x02\x00" + data[6:], id="other-version"),
        pytest.param(lambda data: b"XXXX" + data[4:], id="other-magic"),
    ],
)
def test_bytes_the_port_does_not_read_are_declined_by_name(spoil):
    from cftuv.envelope_production_weld import WELD_LAW

    results, _offset = _scene(3)
    data = next(item.view_bytes for item in results if isinstance(item, _Result) and not codec.view_bytes_failed(item.view_bytes))
    with pytest.raises(cftuv_native.NativePortUnsupported, match="mesh_assembly: "):
        cftuv_native.mesh_assemble([spoil(data)], [1], 0.01, WELD_LAW, True)


@needs_native
def test_a_non_finite_number_is_declined_and_the_reference_answers(monkeypatch):
    vertices = DomainVerticesV1(1, ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, math.inf, 0.0)), ((0.0, 0.0, 1.0),) * 3, (None,) * 3, ())
    view = DomainViewV1(None, vertices, ((0, 1, 2),), ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)), (0,), (), (), "rev")
    results = [_Result(1, codec.encode_view(view, structure_token(view)))]
    expected, found = _both(monkeypatch, results, 0.01)

    assert found.buffers is None and _fields(found) == _fields(expected)
    assert next(iter(assembly.FALLBACKS)) == ("NATIVE_PORT_UNSUPPORTED", "mesh_assemble")


@needs_native
def test_real_domains_of_the_kernel_assemble_as_the_reference(monkeypatch):
    from cftuv.envelope_production_export import deferred_result, produce_domain
    from content_fixtures import cold_domain, with_revision
    from envelope_fixture_bundles import quad_row_bundle

    source = with_revision(quad_row_bundle(5, lifted_corner=1.0), "row")
    results = []
    for patch in range(5):
        deferred, prepared, _revision, _request = cold_domain(source, patch, frozenset(range(5)), alpha=0.25)
        results.append(deferred_result(produce_domain(patch, deferred.domain_id, prepared, "0.25")))
    expected, found = _both(monkeypatch, results, 0.001)

    assert found.buffers is not None and assembly.FALLBACKS == {}
    assert found.face_count and _fields(found) == _fields(expected)

@needs_native
def test_the_header_is_read_as_decode_view_reads_it(monkeypatch):
    """The revision is the text the header names (not the first text, not the outcome slot), and a failed view is declined, never assembled."""

    import struct

    from cftuv.envelope_production_weld import WELD_LAW

    results, _offset = _scene(8)
    data = next(item.view_bytes for item in results if isinstance(item, _Result) and not codec.view_bytes_failed(item.view_bytes))
    texts = struct.unpack_from("<I", data, 16 + 4 * 9)[0]
    spoiled = bytearray(data)
    struct.pack_into("<I", spoiled, 16 + 4 * 11, texts - 1)  # the revision names the last text
    struct.pack_into("<I", spoiled, 16 + 4 * 12, 0)
    spoiled = bytes(spoiled)
    answer = cftuv_native.mesh_assemble([spoiled], [1], 0.01, WELD_LAW, False)
    assert answer[11] == (codec.decode_view(spoiled).source_revision,) and texts > 1
    failed = bytearray(data)
    struct.pack_into("<H", failed, 6, 1)
    with pytest.raises(cftuv_native.NativePortUnsupported, match="failed view"):
        cftuv_native.mesh_assemble([bytes(failed)], [1], 0.01, WELD_LAW, False)