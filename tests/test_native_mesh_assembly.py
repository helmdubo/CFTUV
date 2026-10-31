"""The native checks of the assembled decal mesh (MESH_ASSEMBLY_NATIVE_V1, `mesh_assembly.rs`) against the oracle `cftuv_envelope.mesh_checks`: the same answer bit for bit, or a named decline.

The values: random meshes (welded fans, quads lifted off their plane, shared and reversed edges, UV jumps of one ulp, `-0.0` against `0.0`), and one input per decline. The host
dispatch (`cftuv/envelope_mesh_assembly.py`) is checked both ways of `CFTUV_MESH_ASSEMBLY_BACKEND` and through a decline: the counters are the oracle's and the fallback is named.
"""

from __future__ import annotations

import math
import random
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv_envelope import mesh_checks  # noqa: E402

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "mesh_off_plane"), reason="расширение cftuv_native не собрано (или собрано без mesh_off_plane)"
)


def _bits(value):
    return struct.pack("<d", value)


def _mesh(seed: int):
    rng = random.Random(seed)
    count = rng.randint(4, 40)
    scale = rng.choice((1e-3, 1.0, 37.5, 1e4))
    positions = [tuple(rng.uniform(-1.0, 1.0) * scale for _ in range(3)) for _ in range(count)]
    for index in rng.sample(range(count), k=count // 4):  # vertices on the plane z = c: planar quads measure noise or exact zero
        positions[index] = (positions[index][0], positions[index][1], 0.25 * scale)
    faces = []
    for _ in range(rng.randint(1, 30)):
        size = rng.choice((3, 4, 4, 5, 6))
        faces.append(tuple(rng.sample(range(count), k=min(size, count))))
    if faces and rng.random() < 0.5:  # a reversed and a repeated directed edge
        faces.append(tuple(reversed(faces[0])))
        faces.append(faces[0])
    uvs, face_domain = [], []
    grid = [rng.choice((0.0, -0.0, 0.5, 1.0, math.nextafter(1.0, 2.0))) for _ in range(8)]
    for loop in faces:
        face_domain.append(rng.choice((0, 1, 2)))
        for vertex in loop:
            uvs.append((grid[vertex % 8], grid[(vertex * 3 + 1) % 8]) if rng.random() < 0.8 else (rng.random(), rng.random()))
    return positions, faces, uvs, face_domain


@needs_native
@pytest.mark.parametrize("seed", range(300))
def test_random_meshes_answer_as_the_oracle(seed):
    positions, faces, uvs, face_domain = _mesh(seed)

    worst, off = cftuv_native.mesh_off_plane(positions, faces)
    expected_worst, expected_off = mesh_checks.off_plane_measure(positions, faces)
    assert (_bits(worst), off) == (_bits(expected_worst), expected_off)
    assert cftuv_native.mesh_cross_seams(faces, uvs, face_domain) == mesh_checks.cross_domain_seams(faces, uvs, face_domain)
    assert cftuv_native.mesh_half_edges(faces) == mesh_checks.half_edge_conflicts(faces)


@needs_native
def test_minus_zero_uv_is_the_same_uv_and_one_ulp_is_a_seam():
    faces = [(0, 1, 2), (1, 0, 3)]
    same = [(0.0, 0.0), (1.0, 0.0), (0.5, 1.0), (1.0, -0.0), (-0.0, 0.0), (0.5, -1.0)]
    jump = [(0.0, 0.0), (1.0, 0.0), (0.5, 1.0), (math.nextafter(1.0, 2.0), 0.0), (0.0, 0.0), (0.5, -1.0)]

    assert cftuv_native.mesh_cross_seams(faces, same, [0, 1]) == mesh_checks.cross_domain_seams(faces, same, [0, 1]) == ()
    assert cftuv_native.mesh_cross_seams(faces, jump, [0, 1]) == mesh_checks.cross_domain_seams(faces, jump, [0, 1]) == ((0, 1),)


@needs_native
def test_a_deviation_of_half_a_nanometre_rounds_to_even_and_is_not_counted():
    half = 0.5e-9
    quad = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0), (0.0, 1.0, half)]
    for face in ((0, 1, 2, 3), (0, 1, 2, 4)):
        assert cftuv_native.mesh_off_plane(quad, [face]) == mesh_checks.off_plane_measure(quad, [face])


@needs_native
@pytest.mark.parametrize(
    "call",
    [
        lambda: cftuv_native.mesh_off_plane([(0, 0, 0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0), (0.0, 1.0, 0.0)], [(0, 1, 2, 3)]),
        lambda: cftuv_native.mesh_off_plane([(math.inf, 0.0, 0.0)] * 4, [(0, 1, 2, 3)]),
        lambda: cftuv_native.mesh_off_plane([(0.0, 0.0, 0.0)] * 4, [(0, 1, 2, -1)]),
        lambda: cftuv_native.mesh_off_plane([(0.0, 0.0, 0.0)] * 4, [(0, 1, 2, 9)]),
        lambda: cftuv_native.mesh_cross_seams([(0, 1, 2)], [(0.0, 0.0)] * 2, [0]),
        lambda: cftuv_native.mesh_cross_seams([(0, 1, 2)], [(0.0, 0.0)] * 3, ["patch"]),
        lambda: cftuv_native.mesh_half_edges([(0, 1, True)]),
    ],
)
def test_what_the_port_does_not_carry_is_declined_by_name(call):
    with pytest.raises(cftuv_native.NativePortUnsupported, match="mesh_assembly: "):
        call()


@needs_native
@pytest.mark.parametrize("backend", ["PYTHON", "NATIVE"])
def test_the_host_answers_the_same_both_ways_and_names_a_fallback(monkeypatch, backend):
    from cftuv import envelope_mesh_assembly as assembly

    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, backend)
    monkeypatch.setattr(assembly, "FALLBACKS", {})
    positions, faces, uvs, face_domain = _mesh(7)
    worst, off = mesh_checks.off_plane_measure(positions, faces)

    assert assembly.off_plane_after_offset(positions, faces) == (
        ("ADAPTER_MAX_OFF_PLANE_AFTER_OFFSET_NANOMETRES", round(worst * 10**9)),
        ("ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET", off),
    )
    assert assembly.cross_domain_seams(faces, uvs, face_domain) == mesh_checks.cross_domain_seams(faces, uvs, face_domain)
    assert assembly.half_edge_conflicts(faces) == mesh_checks.half_edge_conflicts(faces)
    assert assembly.FALLBACKS == {}
    # an int coordinate: the port declines, the oracle answers, and the fallback is named once per reason
    lifted = [(0, 0, 0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.5), (0.0, 1.0, 0.0)]
    assert assembly.off_plane_after_offset(lifted, [(0, 1, 2, 3)])[1] == ("ADAPTER_FACES_OFF_PLANE_AFTER_OFFSET", 1)
    assert list(assembly.FALLBACKS) == ([("NATIVE_PORT_UNSUPPORTED", "mesh_assembly")] if backend == "NATIVE" else [])


def test_an_unknown_backend_is_refused_by_name(monkeypatch):
    from cftuv import envelope_mesh_assembly as assembly

    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "RUST")
    with pytest.raises(ValueError, match="CFTUV_MESH_ASSEMBLY_BACKEND"):
        assembly.mesh_assembly_backend()
