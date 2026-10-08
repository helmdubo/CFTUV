"""Blender-free generator of calls of `_embedding._compute_source_snap_embedding_certificate`: planar and spatial polygon meshes on grids, forced degeneracies, snaps, many widths of number.

    cases(count, seed) -> iterator of (label, arguments)    arguments = (before, after, faces, intended_corners, unclassifiable_corners, snapping_law)

Every case is a pure function of `(seed, index)`. A case is a MESH (faces of vertex ids and physical-edge ids, positions before the snap), a NUMBER STYLE (how a lattice coordinate becomes the
exact value the host would hold) and a SNAP (what `after` is). The mixture is chosen so that each counter of the certificate is non-zero in many cases:

* meshes: quad grids with holes (planar in an axis plane or a tilted rational plane, or spatial with a height field), triangle fans, a random polygon soup on a tiny lattice (vertices coincide, edges
  cross, touch, overlap), a mesh with forced degeneracies (a collapsed edge, an edge ending inside another, collinear edges), a mesh with an inconsistent physical edge (the oracle's `ValueError`), and
  a mesh with a vertex the positions lack (the oracle's `KeyError`, which the port declines);
* number styles: small `int`s, `Fraction`s with small denominators (not powers of two), binary64 and binary32 values read exactly (the `before` the host makes), values wide enough to leave the
  `i128` road of the port (magnitude beyond `2^40`, beyond `2^83`), and the widths at the border;
* snaps: `after is before` (UNSNAPPED), an equal copy, the grid snap of `source_grid.snap_positions` at a scale, and that snap with forced edits (a vertex moved onto another, an edge collapsed, two
  vertices swapped, one perturbed by a grid step), so the four violation counters and the corner counters are non-zero;
* ids: `v1 < v10 < v2` string order, accented, Cyrillic and astral characters (Python `str` order is the order of UTF-8 bytes).
"""

from __future__ import annotations

import math
import random
import struct
from fractions import Fraction

import native_embedding_corpus as corpus
from cftuv_envelope.contracts.metric import GridSnappingLawV1
from cftuv_envelope.ids import PatchId, PhysicalEdgeId, SourceFaceId, SourceVertexId

STYLES = ("ints", "fractions", "binary64", "binary32", "wide", "border", "huge")
SNAPS = ("same", "copy", "snap", "snap", "snap", "snap-collapse", "snap-merge", "snap-swap", "snap-nudge")
MESHES = ("grid", "grid", "tilted", "terrain", "fan", "soup", "soup", "degenerate", "inconsistent", "missing", "twin", "tie", "fold")
SCALES = (1, 2, 3, 4, 5, 7, 8, 10, 16, 32, 100, 128, 1000, 1024, 7919, 10**6, 1 << 20, 1 << 30, (1 << 40) + 1)
NAMINGS = ("v{}", "vertex-{}", "é{}", "я{}", "\U0001d518{}", "z{}")
PREFIXES = ("", "é", "я", "\U0001d518", "~")


def _floor_half(value: Fraction) -> int:
    return math.floor(value + Fraction(1, 2))


def snap(point, scale: int) -> tuple:
    return tuple(Fraction(_floor_half(Fraction(item) * scale), scale) for item in point)


def _binary32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


class Style:
    """How a lattice integer becomes a coordinate."""

    def __init__(self, name: str, rng: random.Random):
        self.name, self.rng = name, rng
        self.unit = {"ints": 1, "fractions": 1, "binary64": rng.choice((0.1, 0.25, 0.3, 1.7, 0.01)), "binary32": rng.choice((0.1, 0.3, 0.7, 0.05)),
                     "wide": rng.choice((1e-18, 1e9, 3.3e22, 1e-9)), "border": 0, "huge": 1}[name]
        self.shift = {"border": rng.choice((36, 37, 38, 39, 40, 41, 80, 81, 82, 83, 84, 85)), "huge": rng.choice((130, 200, 330))}.get(name, 0)
        self.denominators = (1, 2, 3, 4, 6, 7, 10, 12)

    def number(self, lattice: int):
        name, rng = self.name, self.rng
        if name == "ints":
            return lattice
        if name == "fractions":
            return Fraction(lattice, rng.choice(self.denominators))
        if name == "binary64":
            return Fraction(lattice * self.unit)
        if name == "binary32":
            return Fraction(_binary32(lattice * self.unit))
        if name == "wide":
            return Fraction(lattice * self.unit * (1 + rng.random() * 1e-3))
        if name == "border":
            return Fraction(lattice * (1 << self.shift) + rng.randrange(3) - 1, 1 << rng.choice((0, 3)))
        return Fraction(lattice * (1 << self.shift) + rng.randrange(7), 1 << rng.choice((0, 5, 60)))

    def point(self, x: int, y: int, z: int) -> tuple:
        return (self.number(x), self.number(y), self.number(z))


class Mesh:
    """Faces as `(face value, [vertex indices], [edge values])`, positions by vertex index, corner triples by index."""

    def __init__(self, naming: str):
        self.naming = naming
        self.positions: dict = {}
        self.faces: list = []
        self.intended: list = []
        self.unclassifiable: list = []

    def vertex(self, index: int) -> SourceVertexId:
        return SourceVertexId(self.naming.format(index))

    def add_face(self, key: str, cycle, edge_names=None) -> None:
        size = len(cycle)
        names = edge_names or [f"e{min(cycle[i], cycle[(i + 1) % size])}_{max(cycle[i], cycle[(i + 1) % size])}" for i in range(size)]
        self.faces.append((key, list(cycle), list(names)))

    def face_objects(self) -> tuple:
        out = []
        for key, cycle, names in self.faces:
            vertices, edges = tuple(self.vertex(i) for i in cycle), tuple(PhysicalEdgeId(name) for name in names)
            if len(vertices) >= 3 and len(vertices) == len(edges):
                from cftuv_envelope.contracts.surface import SourceFaceV1
                from cftuv_envelope.numeric import LocalVector3V1

                out.append(SourceFaceV1(SourceFaceId(key), PatchId("patch"), vertices, edges, LocalVector3V1(0.0, 0.0, 1.0), ()))
            else:
                out.append(corpus.FaceLike(SourceFaceId(key), vertices, edges))
        return tuple(out)


# --------------------------------------------------------------------------
# meshes
# --------------------------------------------------------------------------


def _grid(rng: random.Random, style: Style, naming: str, *, tilted: bool, terrain: bool) -> Mesh:
    mesh = Mesh(naming)
    width, height = rng.randint(2, 6), rng.randint(2, 5)
    index = lambda x, y: y * (width + 1) + x  # noqa: E731
    plane = (rng.choice((0, 1, -1, 2)), rng.choice((0, 1, -1))) if tilted else (0, 0)
    for y in range(height + 1):
        for x in range(width + 1):
            z = rng.randint(-2, 2) if terrain else plane[0] * x + plane[1] * y
            mesh.positions[mesh.vertex(index(x, y))] = style.point(x, y, z)
    keep = [(x, y) for y in range(height) for x in range(width) if rng.random() < 0.8] or [(0, 0)]
    for number, (x, y) in enumerate(keep):
        mesh.add_face(f"f{number}", [index(x, y), index(x + 1, y), index(x + 1, y + 1), index(x, y + 1)])
    for x, y in keep[: rng.randint(0, 4)]:
        mesh.intended.append((index(x, y), index(x + 1, y), index(x + 1, y + 1)))
    for _ in range(rng.randint(0, 3)):
        mesh.unclassifiable.append(tuple(rng.randrange((width + 1) * (height + 1)) for _ in range(3)))
    return mesh


def _fan(rng: random.Random, style: Style, naming: str) -> Mesh:
    mesh = Mesh(naming)
    ring = rng.randint(3, 9)
    mesh.positions[mesh.vertex(0)] = style.point(0, 0, rng.randint(0, 1))
    for i in range(ring):
        angle = 2 * math.pi * i / ring
        mesh.positions[mesh.vertex(i + 1)] = style.point(round(4 * math.cos(angle)), round(4 * math.sin(angle)), rng.randint(0, 1))
    for i in range(ring):
        mesh.add_face(f"f{i}", [0, i + 1, (i + 1) % ring + 1])
    mesh.intended = [(i + 1, 0, (i + 1) % ring + 1) for i in range(rng.randint(0, ring))]
    mesh.unclassifiable = [(1, 0, 2)] if rng.random() < 0.5 else []
    return mesh


def _soup(rng: random.Random, style: Style, naming: str, *, degenerate: bool = False) -> Mesh:
    mesh = Mesh(naming)
    count, lattice = rng.randint(4, 12), rng.choice((2, 3, 4))
    spatial = rng.random() < 0.5
    for i in range(count):
        mesh.positions[mesh.vertex(i)] = style.point(rng.randint(-lattice, lattice), rng.randint(-lattice, lattice), rng.randint(-1, 1) if spatial else 0)
    if rng.random() < 0.4:  # two vertices at the same place before the snap
        a, b = rng.sample(range(count), 2)
        mesh.positions[mesh.vertex(a)] = mesh.positions[mesh.vertex(b)]
    for number in range(rng.randint(1, 6)):
        size = rng.randint(3, 5)
        cycle = rng.sample(range(count), min(size, count))
        mesh.add_face(f"f{number}", cycle)
    if degenerate and count >= 6:
        a, b, c = 0, 1, 2
        mesh.positions[mesh.vertex(b)] = mesh.positions[mesh.vertex(a)]  # a zero-length edge
        mid = style.point(0, 0, 0)
        mesh.positions[mesh.vertex(c)] = mid
        mesh.add_face("fd", [a, b, 3])
        mesh.add_face("fe", [3, 4, 5])
    mesh.intended = [tuple(rng.randrange(count) for _ in range(3)) for _ in range(rng.randint(0, 4))]
    mesh.unclassifiable = [tuple(rng.randrange(count) for _ in range(3)) for _ in range(rng.randint(0, 4))]
    return mesh


def _inconsistent(rng: random.Random, style: Style, naming: str) -> Mesh:
    mesh = _grid(rng, style, naming, tilted=False, terrain=False)
    victim = rng.randrange(len(mesh.faces))
    key, cycle, names = mesh.faces[victim]
    other = rng.randrange(len(mesh.faces))
    if other != victim:
        names[rng.randrange(len(names))] = mesh.faces[other][2][rng.randrange(len(mesh.faces[other][2]))]
    else:
        mesh.faces.append(("fx", list(cycle[:3]), [names[0], "ex", "ey"]))
    return mesh


def _twin(rng: random.Random, style: Style, naming: str, *, tie: bool) -> Mesh:
    """A grid whose faces are repeated: reversed copies under new keys (`tie` False), or copies with the SAME key (the oracle's `min` meets equal keys)."""

    mesh = _grid(rng, style, naming, tilted=rng.random() < 0.5, terrain=False)
    for number, (key, cycle, names) in enumerate(list(mesh.faces)):
        if rng.random() < 0.5:
            continue
        if tie:
            turn = rng.randrange(len(cycle)) if rng.random() < 0.5 else 0
            mesh.faces.append((key, cycle[turn:] + cycle[:turn], names[turn:] + names[:turn]))
        else:
            mesh.faces.append((f"t{number}", [cycle[0], *reversed(cycle[1:])], list(reversed(names))))
    return mesh


def _fold(rng: random.Random, style: Style, naming: str) -> Mesh:
    """A grid with triangles hung on existing edges: three faces on one physical edge (non-manifold), spatial positions."""

    mesh = _grid(rng, style, naming, tilted=False, terrain=False)
    base = len(mesh.positions)
    for extra in range(rng.randint(1, 3)):
        key, cycle, names = rng.choice(mesh.faces)
        index = rng.randrange(len(cycle))
        a, b = cycle[index], cycle[(index + 1) % len(cycle)]
        new = base + extra
        mesh.positions[mesh.vertex(new)] = style.point(rng.randint(0, 3), rng.randint(0, 3), rng.randint(1, 3))
        mesh.faces.append((f"h{extra}", [a, b, new], [names[index], f"eh{a}_{new}", f"eh{b}_{new}"]))
    return mesh


def _mesh(rng: random.Random, kind: str, style: Style, naming: str) -> Mesh:
    if kind == "grid":
        return _grid(rng, style, naming, tilted=False, terrain=False)
    if kind == "tilted":
        return _grid(rng, style, naming, tilted=True, terrain=False)
    if kind == "terrain":
        return _grid(rng, style, naming, tilted=False, terrain=True)
    if kind == "fan":
        return _fan(rng, style, naming)
    if kind in ("soup", "degenerate"):
        return _soup(rng, style, naming, degenerate=kind == "degenerate")
    if kind == "inconsistent":
        return _inconsistent(rng, style, naming)
    if kind in ("twin", "tie"):
        return _twin(rng, style, naming, tie=kind == "tie")
    if kind == "fold":
        return _fold(rng, style, naming)
    mesh = _grid(rng, style, naming, tilted=False, terrain=False)  # missing: a face names a vertex the positions lack
    key, cycle, names = mesh.faces[0]
    cycle[rng.randrange(len(cycle))] = 10_000
    return mesh


# --------------------------------------------------------------------------
# snaps
# --------------------------------------------------------------------------


def _after(rng: random.Random, kind: str, before: dict, mesh: Mesh):
    if kind == "same":
        return before
    if kind == "copy":
        return dict(before)
    scale = rng.choice(SCALES)
    after = {vertex: snap(point, scale) for vertex, point in before.items()}
    keys = list(after)
    if kind == "snap-collapse" and mesh.faces:
        cycle = rng.choice(mesh.faces)[1]
        a, b = mesh.vertex(cycle[0]), mesh.vertex(cycle[1])
        if a in after and b in after:
            after[b] = after[a]
    elif kind == "snap-merge" and len(keys) >= 2:
        a, b = rng.sample(keys, 2)
        after[b] = after[a]
    elif kind == "snap-swap" and len(keys) >= 2:
        a, b = rng.sample(keys, 2)
        after[a], after[b] = after[b], after[a]
    elif kind == "snap-nudge" and keys:
        victim = rng.choice(keys)
        after[victim] = tuple(item + Fraction(rng.randint(-1, 1), scale) for item in after[victim])
    return after


def _corners(mesh: Mesh, rng: random.Random, triples: list) -> tuple:
    out = []
    for triple in triples:
        out.append(tuple(mesh.vertex(i) if rng.random() > 0.03 else SourceVertexId("nowhere") for i in triple))
    return tuple(out)


def build(index: int, seed: int) -> tuple:
    """`(label, arguments)` of case `index`."""

    rng = random.Random(f"embedding-synthetic-{seed}-{index}")
    kind, snap_kind = MESHES[index % len(MESHES)], SNAPS[(index // len(MESHES)) % len(SNAPS)]
    style = Style(rng.choice(STYLES) if rng.random() < 0.5 else STYLES[(index // 3) % len(STYLES)], rng)
    mesh = _mesh(rng, kind, style, rng.choice(NAMINGS))
    face_prefix, edge_prefix = rng.choice(PREFIXES), rng.choice(PREFIXES)  # the order of face keys and edge ids is the order of their UTF-8 bytes
    mesh.faces = [(face_prefix + key, cycle, [edge_prefix + name for name in names]) for key, cycle, names in mesh.faces]
    before = dict(mesh.positions)
    if kind == "missing" and rng.random() < 0.5:
        before.pop(next(iter(before)))
    after = _after(rng, snap_kind, before, mesh)
    law = GridSnappingLawV1.UNSNAPPED_EXACT_V1 if after is before else rng.choice((GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1, GridSnappingLawV1.INTEGER_GRID_SNAP_V1))
    arguments = (before, after, mesh.face_objects(), _corners(mesh, rng, mesh.intended), _corners(mesh, rng, mesh.unclassifiable), law)
    return f"{kind}/{style.name}/{snap_kind}/{index}", arguments


def cases(count: int, seed: int = 20261008):
    for index in range(count):
        yield build(index, seed)
