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
from types import SimpleNamespace

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
def test_a_deviation_of_exactly_half_a_nanometre_rounds_to_even_and_is_not_counted():
    # the loop (o, a, b, c, b) has the normal a x b exactly (the two fan terms of c cancel), so c deviates by exactly its height: 5e-10 * 1e9 == 0.5, and
    # round(0.5) == 0 (half to even) leaves the face uncounted, while 1.5 nanometres rounds to 2 and counts
    for height, counted in ((5e-10, 0), (1.5e-9, 1), (2.5e-9, 1)):
        points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.25, 0.25, height)]
        expected = mesh_checks.off_plane_measure(points, [(0, 1, 2, 3, 2)])
        assert expected == (height, counted)
        assert cftuv_native.mesh_off_plane(points, [(0, 1, 2, 3, 2)]) == expected


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
    assert list(assembly.FALLBACKS) == ([("NATIVE_PORT_UNSUPPORTED", "mesh_off_plane")] if backend == "NATIVE" else [])


def test_an_unknown_backend_is_refused_by_name(monkeypatch):
    from cftuv import envelope_mesh_assembly as assembly

    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "RUST")
    with pytest.raises(ValueError, match="CFTUV_MESH_ASSEMBLY_BACKEND"):
        assembly.mesh_assembly_backend()


# --------------------------------------------------------------------------
# B2: the weld (`mesh_weld.weld_core`)
# --------------------------------------------------------------------------


#: The host's weld law; the B2 tests read it, not a copy of its numbers.
from cftuv.envelope_production_weld import WELD_LAW as LAW  # noqa: E402


def _exact(value):
    """The value with every float as its hex (`-0.0` and `0.0` differ, as the bits of the mesh do)."""

    if isinstance(value, float):
        return value.hex()
    if isinstance(value, (tuple, list)):
        return tuple(_exact(item) for item in value)
    return value


def _unit(rng):
    while True:
        vector = tuple(rng.uniform(-1.0, 1.0) for _ in range(3))
        length = math.sqrt(sum(item * item for item in vector))
        if length > 0.1:
            return tuple(item / length for item in vector)


def _weld_scene(seed: int):
    from cftuv.envelope_production_weld import DomainVerticesV1

    rng = random.Random(seed)
    anchors = [tuple(rng.uniform(-2.0, 2.0) for _ in range(3)) for _ in range(rng.randint(1, 6))]
    directions = [_unit(rng) for _ in anchors]
    shared = [_unit(rng) for _ in anchors]
    domains = []
    for ordinal in range(rng.randint(1, 5)):
        positions, normals, refs, walls = [], [], [], []

        def vertex(point, normal, ref):
            positions.append(point)
            normals.append(normal)
            refs.append(ref)
            return len(positions) - 1

        for _ in range(rng.randint(0, 4)):
            vertex(tuple(rng.uniform(-2.0, 2.0) for _ in range(3)), _unit(rng), rng.choice((None, f"location:node:{rng.randint(0, 9)}", "location:src:lonely")))
        for number, point in enumerate(anchors):
            if rng.random() < 0.3:
                continue
            if rng.random() < 0.15:  # a position one ulp away: the group does not weld (a mismatch)
                point = (math.nextafter(point[0], 9.0), point[1], point[2])
            choice = rng.random()
            normal = shared[number] if choice < 0.5 else (tuple(-item for item in shared[number]) if choice < 0.6 else _unit(rng))
            if rng.random() < 0.1:
                normal = tuple(0.0 * item if index == 0 else item for index, item in enumerate(normal))  # a -0.0 against a 0.0
            anchor = vertex(point, normal, f"location:src:host:{number}")
            if rng.random() < 0.6:  # a wall out of the anchor along the shared direction (a twin when two domains place the node alike)
                step = rng.choice((0.25, 0.5, 0.5))
                heading = directions[number] if rng.random() < 0.85 else _unit(rng)
                node = vertex(tuple(a + step * b for a, b in zip(point, heading)), _unit(rng) if rng.random() < 0.5 else shared[number], None if rng.random() < 0.8 else "location:node:9")
                chain = [anchor, node] if rng.random() < 0.5 else [node, anchor]
                if rng.random() < 0.3:
                    chain.append(vertex(tuple(rng.uniform(-2.0, 2.0) for _ in range(3)), _unit(rng), None))
                walls.append(tuple(chain))
        domains.append(DomainVerticesV1(ordinal, tuple(positions), tuple(normals), tuple(refs), tuple(walls)))
    return domains, rng.choice((0.0, 0.001, 0.0125, 1.0))


@needs_native
@pytest.mark.parametrize("seed", range(400))
def test_random_welds_answer_as_the_oracle(seed):
    from cftuv_envelope import mesh_weld

    domains, offset = _weld_scene(seed)
    assert _exact(cftuv_native.mesh_weld(domains, offset, LAW)) == _exact(mesh_weld.weld_core(domains, offset, LAW))


def _with_faces(domains, seed: int) -> list:
    from cftuv.envelope_production_weld import DomainVerticesV1

    rng = random.Random(seed + 1000)
    found = []
    for domain in domains:
        size = len(domain.positions)
        rings = [tuple(rng.sample(range(size), min(size, rng.choice((3, 4, 5))))) for _ in range(rng.randint(0, 4))] if size >= 3 else []
        # every wall node gets a ring in most scenes, so both answers of the shape check are reached
        for wall in domain.walls:
            if size >= 3 and rng.random() < 0.8:
                others = [item for item in range(size) if item not in wall]
                rings.append((*wall[:2], *rng.sample(others, min(len(others), 2))))
        found.append(DomainVerticesV1(domain.patch_id, domain.positions, domain.normals, domain.refs, domain.walls, faces=tuple(ring for ring in rings if len(ring) >= 3)))
    return found


@needs_native
@pytest.mark.parametrize("seed", range(400))
def test_random_mean_level_welds_answer_as_the_oracle(seed):
    from cftuv.envelope_production_weld import WELD_LAW_MEAN_LEVEL
    from cftuv_envelope import mesh_weld

    domains, offset = _weld_scene(seed)
    domains = _with_faces(domains, seed)
    assert _exact(cftuv_native.mesh_weld(domains, offset, WELD_LAW_MEAN_LEVEL)) == _exact(mesh_weld.weld_core(domains, offset, WELD_LAW_MEAN_LEVEL))


def test_the_random_mean_level_welds_reach_both_answers():
    from cftuv.envelope_production_weld import WELD_LAW_MEAN_LEVEL
    from cftuv_envelope import mesh_weld

    means = refused = 0
    for seed in range(400):
        domains, offset = _weld_scene(seed)
        answer = mesh_weld.weld_core(_with_faces(domains, seed), offset, WELD_LAW_MEAN_LEVEL)
        means += len(answer[-2])
        refused += answer[-1]
    assert means and refused


def test_the_random_welds_reach_every_branch():
    from cftuv_envelope import mesh_weld

    seen = set()
    for seed in range(400):
        domains, offset = _weld_scene(seed)
        answer = mesh_weld.weld_core(domains, offset, LAW)
        (_positions, _index, groups, merged, mismatches, _gaps, fallbacks, _factor, _reasons, lifted, welded, alone, wall_fallbacks, *_rest) = answer
        seen.update(name for name, value in (("groups", groups), ("merged", merged), ("mismatches", mismatches), ("fallbacks", fallbacks), ("lifted", lifted - 2 * welded), ("welded", welded), ("alone", alone), ("wall_fallbacks", wall_fallbacks)) if value)
    assert seen == {"groups", "merged", "mismatches", "fallbacks", "lifted", "welded", "alone", "wall_fallbacks"}


@needs_native
def test_three_normals_and_a_dependent_set_eliminate_as_the_oracle():
    from cftuv_envelope import mesh_weld

    up, side, front = (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0)
    tilted = (0.0, math.sqrt(0.5), math.sqrt(0.5))
    for normals in ([up, side, front], [up, side, front, tilted], [up, up, side], [up, (0.0, 0.0, -1.0)], [up, (0.0, -0.0, 1.0)], [side, (math.nextafter(-1.0, 0.0), 0.0, 0.0)]):
        from cftuv.envelope_production_weld import DomainVerticesV1

        domains = [DomainVerticesV1(index, ((0.5, 0.5, 0.0),), (normal,), ("location:src:host:1",)) for index, normal in enumerate(normals)]
        assert _exact(cftuv_native.mesh_weld(domains, 0.01, LAW)) == _exact(mesh_weld.weld_core(domains, 0.01, LAW))


@needs_native
@pytest.mark.parametrize(
    "domains",
    [
        [SimpleNamespace(positions=((0, 0.0, 0.0),), normals=((0.0, 0.0, 1.0),), refs=(None,), walls=())],
        [SimpleNamespace(positions=((0.0, 0.0, 0.0),), normals=((0.0, 0.0, 1.0),), refs=(7,), walls=())],
        [SimpleNamespace(positions=((0.0, 0.0, 0.0),), normals=((0.0, 0.0, 1.0),), refs=(None,), walls=((0, 3),))],
        [SimpleNamespace(positions=((0.0, 0.0, 0.0),), normals=(), refs=(None,), walls=())],
    ],
)
def test_a_weld_the_port_does_not_carry_is_declined(domains):
    with pytest.raises(cftuv_native.NativePortUnsupported, match="mesh_assembly: "):
        cftuv_native.mesh_weld(domains, 0.01, LAW)


# --------------------------------------------------------------------------
# B3: the reports in one read (`mesh_reports`)
# --------------------------------------------------------------------------


def _chains(seed: int):
    rng = random.Random(seed)
    pool = [f"src:{index}" for index in range(rng.randint(2, 9))]
    domains = []
    for number in range(rng.randint(0, 5)):
        chains = []
        for index in range(rng.randint(0, 4)):
            kind = rng.choice(("SOURCE", "SOURCE", "SOURCE", "WALL", "INTERFACE"))
            run = [rng.choice(pool)]
            for _ in range(rng.randint(0, 6)):
                choice = rng.random()
                run.append(rng.choice(pool) if choice < 0.55 else (f"clip:{rng.randint(0, 3)}" if choice < 0.7 else f"node:{rng.randint(0, 9)}"))
            chains.append((f"boundary:{kind}:{number}:{index}", tuple(run)))
        domains.append(tuple(sorted(chains)))
    return domains


def _reference_reports(positions, faces, uvs, face_domain, face_owner, seams, chains):
    return (
        mesh_checks.arrays_digest(positions, faces, uvs, face_domain, face_owner, seams),
        mesh_checks.seam_report_numbers(chains),
        mesh_checks.off_plane_measure(positions, faces),
        mesh_checks.half_edge_conflicts(faces),
    )


@needs_native
@pytest.mark.parametrize("seed", range(400))
def test_random_reports_answer_as_the_oracle(seed):
    positions, faces, uvs, face_domain = _mesh(seed)
    rng = random.Random(seed)
    if rng.random() < 0.3:  # numbers whose repr is not the shortest decimal of a short literal: exponents, tiny and huge values, -0.0
        positions = [tuple(rng.choice((value, -0.0, 1e-17 * value, 1e22 * value, value / 3.0)) for value in point) for point in positions]
    face_owner = [rng.randint(-3, 40) for _ in faces]
    seams = sorted({tuple(sorted(rng.sample(range(len(positions)), 2))) for _ in range(rng.randint(0, 5))})
    chains = _chains(seed)
    expected = _reference_reports(positions, faces, uvs, face_domain, face_owner, seams, chains)
    assert _exact(cftuv_native.mesh_reports(positions, faces, uvs, face_domain, face_owner, seams, chains)) == _exact(expected)


def test_the_random_chains_reach_every_branch_of_the_seam_report():
    junctions = clips = 0
    for seed in range(400):
        found = mesh_checks.seam_report_numbers(_chains(seed))
        junctions += found[0] > 0
        clips += found[1] > 0
    assert junctions and clips


def _named(group):
    return tuple(sorted((f"boundary:SOURCE:0:{number}", tuple(f"src:{key}" for key in keys)) for number, keys in enumerate(group)))


def _places(genuine):
    mine, theirs = [], []
    for index in range(6):
        a, b, m, c = (f"{index}{tag}" for tag in "abmc")
        mine.append([a, b])
        theirs.extend(([a, c, b] if genuine else [a, b], [b, m, a]))
    return [_named(mine), _named(theirs)]


@needs_native
def test_the_t_junction_scenes_of_the_seam_report_answer_as_the_oracle():
    """The scenes of the host seam tests: the hexagon inside a ring (walls003: a path around an edge is no T-junction) and six places, clean and genuine."""

    hexagon = [[20, 19], [13, 14], [14, 21, 20], [19, 18, 13]]
    ring = [[15, 14], [13, 12], [0, 23, 22, 15], [12, 17, 16, 1], [14, 21, 20, 19, 18, 13], [1, 4, 5, 6, 7, 0]]
    scenes = {"walls003": [_named(hexagon), _named(ring)], "clean": _places(False), "genuine": _places(True)}
    expected = {"walls003": (0, 0), "clean": (0, 0), "genuine": (6, 0)}
    for name, chains in scenes.items():
        assert mesh_checks.seam_report_numbers(chains) == expected[name], name
        assert cftuv_native.mesh_reports([], [], [], [], [], [], chains)[1] == expected[name], name


def test_the_default_backend_is_native_and_python_is_the_named_fallback(monkeypatch):
    """MESH_ASSEMBLY_NATIVE_DEFAULT_V1: without the variable the product assembles natively; `PYTHON` isolates the frozen reference."""

    from cftuv import envelope_mesh_assembly as assembly

    monkeypatch.delenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, raising=False)
    assert assembly.DEFAULT_MESH_ASSEMBLY_BACKEND == "NATIVE" and assembly.mesh_assembly_backend() == "NATIVE"
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, "python")
    assert assembly.mesh_assembly_backend() == "PYTHON"
