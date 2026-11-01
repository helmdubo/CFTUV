"""SEMANTIC_DECAL_EQUIVALENCE_V1: are two decal results the same PRODUCT, whatever their bytes?

    python tools/semantic_decal_equivalence.py A.semantic.json B.semantic.json [--json report.json]

The executable form of NATIVE_PATH_PRODUCT_CONTRACT_V1 (DECISIONS.md): the closed native path is bound to the product's invariants, not to the bytes of
the Python reference. A press record (`press_record`: every domain's named outcome and view, and the whole welded mesh) of one side is compared with the
other's, and every difference is classified:

  GEOMETRY_ERROR       the product differs: coverage / silhouette (the boundary of every region, exactly), seams, inter-patch joins, open edges (cracks),
                       the owner of a region, UV along every region boundary (seams included) and inside it, provenance (the position and UV of every
                       shared semantic vertex), a named outcome, the set of domains.
  RESOURCE_POLICY      a named outcome differs and one of the two is a refusal by a resource budget (`RESOURCE_OUTCOME`): the native path has a budget of its own.
  REPRESENTATION_ONLY  the product is the same: vertex / face order, corner rotation, ids (owner labels by a consistent bijection), the spelling of a number
                       (int, Fraction, binary64), diagnostic text, service counters, the triangulation inside a region.

HOW. Numbers are compared exactly (`Fraction`). A face is its corners `(x, y, z, u, v)`; a REGION is a UV-continuous set of faces of one domain and one
owner (faces sharing an edge by vertex index with the same UV on both loops). Its boundary is the signed 1-chain of its directed corner edges in R^5,
cancelled and merged along each 5-D line, so a collinear subdivision of a boundary edge (with UV linear along it) is the same boundary, and a UV kink or
a moved vertex is not. Two regions match when their domain and boundary chain are equal; the owners of matched regions must be a bijection. A matched
region whose faces differ (another triangulation) must be fold-free in UV and its 3-D surface must be the same piecewise-linear function of UV: every
corner, edge midpoint and face centroid of each side is located in the other side's faces (exact barycentrics) and must map to the same point.
Seams, inter-patch joins (edges whose faces belong to two domains, by vertex index: an unwelded join is missing) and open edges are compared as
unsigned 1-chains in R^3.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from fractions import Fraction

SCHEMA = "cftuv.semantic-decal.v1"
CONTRACT = "SEMANTIC_DECAL_EQUIVALENCE_V1"
GEOMETRY_ERROR = "GEOMETRY_ERROR"
REPRESENTATION_ONLY = "REPRESENTATION_ONLY"
RESOURCE_POLICY = "RESOURCE_POLICY"
CLASSES = (GEOMETRY_ERROR, RESOURCE_POLICY, REPRESENTATION_ONLY)
#: A refusal by a WORK budget (the exact arithmetic, a generation, a refinement), not by a product policy (stretch, distortion, residual budgets are the request's answer).
RESOURCE_OUTCOME = re.compile(r"WORK_BUDGET|BUDGET_EXHAUSTED|AUTHORITY_EXHAUSTED|REFINEMENT_EXHAUSTED|GENERATION_BUDGET")


# --------------------------------------------------------------------------
# Numbers and records
# --------------------------------------------------------------------------


def exact(value) -> Fraction:
    """`int`, `Fraction`, finite `float`, `"p/q"`, a decimal or a hex float string -> the exact rational."""

    if isinstance(value, bool):
        raise TypeError("a boolean is not a coordinate")
    if isinstance(value, (int, Fraction)):
        return Fraction(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"non-finite coordinate {value!r}")
        return Fraction(value)
    if isinstance(value, str):
        text = value.strip()
        if "0x" in text.lower():
            return Fraction(float.fromhex(text))
        return Fraction(text)
    raise TypeError(f"not a number: {value!r}")


def wire(value):
    """A number as it travels in a record: a binary64 as its hex spelling (exact), a Fraction as `p/q`, an int as it is."""

    if isinstance(value, float):
        return value.hex()
    if isinstance(value, Fraction):
        return int(value) if value.denominator == 1 else f"{value.numerator}/{value.denominator}"
    return value


def view_record(view) -> dict | None:
    """The domain view the mesh writer reads (`DomainViewV1`): positions, provenance refs, faces, loop UVs, owners, seams; a writer refusal as `failure`."""

    if view is None:
        return None
    if view.failure is not None or view.vertices is None:
        outcome, detail = view.failure or ("ADAPTER_VIEW_EMPTY", "")
        return {"failure": [str(outcome), str(detail)]}
    vertices = view.vertices
    return {
        "positions": [[wire(c) for c in point] for point in vertices.positions], "refs": [None if ref is None else str(ref) for ref in vertices.refs],
        "faces": [list(face) for face in view.faces], "uvs": [[wire(c) for c in uv] for uv in view.uvs], "owners": list(view.owners), "seams": [list(edge) for edge in view.seams],
    }


def mesh_record(arrays) -> dict:
    """The welded mesh (`MeshArraysV1`): positions after the offset, faces, loop UVs, the domain and owner of every face, seam edges, the skipped domains."""

    return {
        "positions": [[wire(c) for c in point] for point in arrays.positions], "faces": [list(face) for face in arrays.faces], "uvs": [[wire(c) for c in uv] for uv in arrays.uvs],
        "face_domain": list(arrays.face_domain), "face_owner": list(arrays.face_owner), "seams": [list(edge) for edge in arrays.seam_edges],
        "skipped": sorted([str(item[0]), str(item[2])] for item in arrays.skipped),
    }


def press_record(results, arrays, view_of) -> dict:
    """One press (cold build, tick, COMMIT): every domain's named outcome, detail, diagnostics and view (`view_of(result)`, materialized ones), and the mesh."""

    domains = {}
    for item in results:
        view = view_of(item) if item.is_materialized else None
        domains[str(int(item.patch_id))] = {
            "outcome": str(item.outcome), "detail": str(item.detail), "diagnostics": [str(entry) for entry in item.diagnostics], "view": view_record(view),
        }
    return {"schema": SCHEMA, "domains": domains, "mesh": None if arrays is None else mesh_record(arrays)}


# --------------------------------------------------------------------------
# Findings
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class Finding:
    kind: str
    where: str
    what: str

    def line(self) -> str:
        return f"{self.kind} {self.where}: {self.what}"


@dataclass
class Report:
    findings: list = field(default_factory=list)
    compared: Counter = field(default_factory=Counter)

    def add(self, kind: str, where: str, what: str) -> None:
        self.findings.append(Finding(kind, where, what))

    def of(self, kind: str) -> list:
        return [item for item in self.findings if item.kind == kind]

    @property
    def counts(self) -> dict:
        return {kind: len(self.of(kind)) for kind in CLASSES}

    @property
    def verdict(self) -> str:
        """`EQUAL` (nothing differs), `EQUIVALENT` (representation only), `RESOURCE_POLICY`, `GEOMETRY_ERROR` (the worst class found)."""

        if self.of(GEOMETRY_ERROR):
            return GEOMETRY_ERROR
        if self.of(RESOURCE_POLICY):
            return RESOURCE_POLICY
        return "EQUIVALENT" if self.findings else "EQUAL"

    def as_record(self) -> dict:
        return {"contract": CONTRACT, "verdict": self.verdict, "counts": self.counts, "compared": dict(self.compared), "findings": [item.line() for item in self.findings]}


# --------------------------------------------------------------------------
# Surfaces
# --------------------------------------------------------------------------


@dataclass
class Surface:
    points: list
    faces: list
    corner_uvs: list
    owner: list
    domain: list
    seams: list
    refs: list | None = None
    #: Coordinates are integers in units of 1 / `scale` once `on_common_scale` made them so (exact rationals before).
    scale: int = 1

    def corners(self, face: int) -> tuple:
        return tuple((*self.points[vertex], *uv) for vertex, uv in zip(self.faces[face], self.corner_uvs[face]))


class MalformedRecord(ValueError):
    pass


def _surface(record: dict, face_domain, face_owner, refs) -> Surface:
    points = [tuple(exact(c) for c in point) for point in record["positions"]]
    faces = [tuple(int(v) for v in face) for face in record["faces"]]
    uvs = [tuple(exact(c) for c in uv) for uv in record["uvs"]]
    if sum(len(face) for face in faces) != len(uvs):
        raise MalformedRecord(f"{len(uvs)} loop UVs for {sum(len(face) for face in faces)} face corners")
    if any(len(face) < 3 or min(face) < 0 or max(face) >= len(points) for face in faces):
        raise MalformedRecord("a face with fewer than three corners or a vertex out of range")
    corner_uvs, start = [], 0
    for face in faces:
        corner_uvs.append(tuple(uvs[start : start + len(face)]))
        start += len(face)
    owner = list(face_owner) if face_owner is not None else [0] * len(faces)
    domain = list(face_domain) if face_domain is not None else [""] * len(faces)
    if len(owner) != len(faces) or len(domain) != len(faces):
        raise MalformedRecord("the owners or domains do not cover the faces")
    seams = [tuple(int(v) for v in edge) for edge in record.get("seams", ())]
    return Surface(points, faces, corner_uvs, owner, domain, seams, refs)


def surface_of_view(record: dict) -> Surface:
    return _surface(record, None, record["owners"], record.get("refs"))


def surface_of_mesh(record: dict) -> Surface:
    return _surface(record, record["face_domain"], record["face_owner"], None)


def _denominators(surface: Surface):
    for point in surface.points:
        yield from (value.denominator for value in point)
    for uvs in surface.corner_uvs:
        for uv in uvs:
            yield from (value.denominator for value in uv)


def on_common_scale(first: Surface, second: Surface) -> tuple:
    """Both surfaces with integer coordinates in units of 1 / (the least common denominator of both): every exact test after is integer arithmetic
    (binary64 coordinates are dyadic, so the scale is a power of two)."""

    scale = 1
    for surface in (first, second):
        for denominator in set(_denominators(surface)):
            scale = math.lcm(scale, denominator)

    def scaled(surface):
        number = lambda value: value.numerator * (scale // value.denominator)  # noqa: E731
        points = [tuple(number(value) for value in point) for point in surface.points]
        uvs = [tuple(tuple(number(value) for value in uv) for uv in face) for face in surface.corner_uvs]
        return Surface(points, surface.faces, uvs, surface.owner, surface.domain, surface.seams, surface.refs, scale)

    return scaled(first), scaled(second)


def _rotated(corners: tuple) -> tuple:
    start = min(range(len(corners)), key=corners.__getitem__)
    return corners[start:] + corners[:start]


def _face_multiset(surface: Surface, with_owner: bool = True) -> Counter:
    return Counter((surface.domain[f], surface.owner[f] if with_owner else None, _rotated(surface.corners(f))) for f in range(len(surface.faces)))


# --------------------------------------------------------------------------
# 1-chains along lines (exact)
# --------------------------------------------------------------------------


def _line(p: tuple, q: tuple):
    """The canonical line through integer points `p`, `q` - its primitive direction `d` (positive on its first nonzero coordinate `k`) and the
    invariants `p_i d_k - p_k d_i` (the same for every point of the line, and only of it) - and the parameters of `p` and `q` along it (coordinate `k`)."""

    direction = [b - a for a, b in zip(p, q)]
    k = next((i for i, x in enumerate(direction) if x), None)
    if k is None:
        return None
    divisor = math.gcd(*direction) * (1 if direction[k] > 0 else -1)
    unit = tuple(x // divisor for x in direction)
    return (k, unit, tuple(a * unit[k] - p[k] * x for a, x in zip(p, unit))), p[k], q[k]


def chain(segments, signed: bool = True) -> frozenset:
    """The 1-chain of `segments` (`(p, q)`, points of any dimension): per line, the merged intervals of nonzero net coverage (signed by direction, or unsigned: covered at all)."""

    lines: dict = defaultdict(lambda: defaultdict(int))
    for p, q in segments:
        found = _line(p, q)
        if found is None:
            continue
        key, a, b = found
        sign = 1 if (b > a or not signed) else -1
        events = lines[key]
        events[min(a, b)] += sign
        events[max(a, b)] -= sign
    out = []
    for key, events in lines.items():
        coverage, pieces, stops = 0, [], sorted(events)
        for t0, t1 in zip(stops, stops[1:]):
            coverage += events[t0]
            value = coverage if signed else int(coverage > 0)
            if not value:
                continue
            if pieces and pieces[-1][1] == t0 and pieces[-1][2] == value:
                pieces[-1] = (pieces[-1][0], t1, value)
            else:
                pieces.append((t0, t1, value))
        if pieces:
            out.append((key, tuple(pieces)))
    return frozenset(out)


def _point_of(piece, scale: int) -> str:
    """A readable point of a chain piece (its first parameter): where a difference is."""

    (k, unit, invariants), pieces = piece
    t = pieces[0][0]
    return "(" + ", ".join(f"{(q + t * u) / (unit[k] * scale):.6g}" for q, u in zip(invariants, unit)) + ")"


def _chain_difference(first: frozenset, second: frozenset, scale: int) -> str:
    only_a, only_b = first - second, second - first
    sample = next(iter(only_a or only_b))
    return f"{len(only_a)} line pieces only in A, {len(only_b)} only in B, e.g. at {_point_of(sample, scale)}"


# --------------------------------------------------------------------------
# Regions
# --------------------------------------------------------------------------


@dataclass
class Region:
    domain: object
    owner: object
    faces: tuple
    boundary: frozenset
    silhouette: frozenset


def _find(parent: list, item: int) -> int:
    while parent[item] != item:
        parent[item] = parent[parent[item]]
        item = parent[item]
    return item


def regions(surface: Surface) -> list:
    """UV-continuous regions of one domain and one owner: faces sharing an edge (by vertex index) with equal UV at both of its loops on both faces."""

    parent = list(range(len(surface.faces)))
    edges: dict = {}
    for f, face in enumerate(surface.faces):
        uvs = surface.corner_uvs[f]
        for i, vertex in enumerate(face):
            j = (i + 1) % len(face)
            a, b, ua, ub = vertex, face[j], uvs[i], uvs[j]
            if a > b:
                a, b, ua, ub = b, a, ub, ua
            key = (surface.domain[f], surface.owner[f], a, b, ua, ub)
            if key in edges:
                parent[_find(parent, f)] = _find(parent, edges[key])
            else:
                edges[key] = f
    groups: dict = defaultdict(list)
    for f in range(len(surface.faces)):
        groups[_find(parent, f)].append(f)
    out = []
    for members in groups.values():
        directed = [(c[i], c[(i + 1) % len(c)]) for f in members for c in (surface.corners(f),) for i in range(len(c))]
        out.append(Region(surface.domain[members[0]], surface.owner[members[0]], tuple(members), chain(directed), chain([(p[:3], q[:3]) for p, q in directed])))
    return out


def _area2(a, b, c):
    return (b[3] - a[3]) * (c[4] - a[4]) - (b[4] - a[4]) * (c[3] - a[3])


def _fan(corners: tuple) -> list:
    """The fan triangles of a face from its canonical first corner (a planar face with an affine UV map is the same under any fan)."""

    return [(corners[0], corners[i], corners[i + 1]) for i in range(1, len(corners) - 1)]


def _sixfold(polygons: list) -> list:
    """Integer corners times 6: every edge midpoint and centroid is then an integer point too, and the exact tests below need no division."""

    return [tuple(tuple(6 * value for value in corner) for corner in polygon) for polygon in polygons]


class _Locator:
    """Exact point location in the UV plane over a float grid of triangle boxes (a hint only: a miss widens to the neighbours, then to every triangle)."""

    def __init__(self, triangles: list):
        self.triangles = [(t, _area2(*t)) for t in triangles]
        us = [float(c[3]) for t in triangles for c in t]
        vs = [float(c[4]) for t in triangles for c in t]
        self.u0, self.v0 = min(us), min(vs)
        side = max(1, int(math.sqrt(len(triangles))))
        self.du = (max(us) - self.u0) / side or 1.0
        self.dv = (max(vs) - self.v0) / side or 1.0
        self.cells: dict = defaultdict(list)
        for index, (t, _area) in enumerate(self.triangles):
            for i in range(self._cell(min(float(c[3]) for c in t), self.u0, self.du), self._cell(max(float(c[3]) for c in t), self.u0, self.du) + 1):
                for j in range(self._cell(min(float(c[4]) for c in t), self.v0, self.dv), self._cell(max(float(c[4]) for c in t), self.v0, self.dv) + 1):
                    self.cells[(i, j)].append(index)

    @staticmethod
    def _cell(value: float, origin: float, step: float) -> int:
        return int(math.floor((value - origin) / step))

    def _candidates(self, point: tuple):
        i, j = self._cell(float(point[3]), self.u0, self.du), self._cell(float(point[4]), self.v0, self.dv)
        yield self.cells.get((i, j), ())
        yield [index for di in (-1, 0, 1) for dj in (-1, 0, 1) if di or dj for index in self.cells.get((i + di, j + dj), ())]
        yield range(len(self.triangles))

    def consistent(self, point: tuple):
        """`True` when the piecewise-linear surface takes the 3-D point of `point` at its UV, `False` when it takes another, `None` outside it."""

        for candidates in self._candidates(point):
            for index in candidates:
                (a, b, c), area = self.triangles[index]
                wa, wb, wc = _area2(point, b, c), _area2(a, point, c), _area2(a, b, point)
                if area < 0:
                    wa, wb, wc, area = -wa, -wb, -wc, -area
                if wa >= 0 and wb >= 0 and wc >= 0:
                    return all(point[i] * area == wa * a[i] + wb * b[i] + wc * c[i] for i in range(3))
        return None


def _samples(triangles: list):
    """Edge midpoints and centroids of integer triangles whose coordinates are multiples of 6 (so the samples stay integers)."""

    for a, b, c in triangles:
        for p, q in ((a, b), (b, c), (c, a)):
            yield tuple((x + y) // 2 for x, y in zip(p, q))
        yield tuple((x + y + z) // 3 for x, y, z in zip(a, b, c))


def interior_difference(polygons_a: list, polygons_b: list, scale: int = 1) -> str | None:
    """`None` when two face sets with one boundary (the faces one side has and the other has not, in one region) are the same surface: both are
    fold-free and UV-nondegenerate, and each side's corners, edge midpoints and face centroids lie on the other side's piecewise-linear surface,
    exactly. Otherwise what differs. The corners are integers in units of 1 / `scale` (`on_common_scale`)."""

    sides = []
    scale *= 6
    for name, polygons in zip("AB", (_sixfold(polygons_a), _sixfold(polygons_b))):
        fans = [_fan(polygon) for polygon in polygons]
        if any(not any(_area2(*t) for t in fan) for fan in fans):
            return f"side {name} has UV-degenerate faces in the region: a change of triangulation cannot be proved"
        # a zero-area fan triangle (a collinear corner) covers nothing: its points lie on the other triangles of its face
        triangles = [t for fan in fans for t in fan if _area2(*t)]
        if len({_area2(*t) > 0 for t in triangles}) != 1:
            return f"side {name} folds in UV inside the region"
        sides.append((name, triangles, [corner for polygon in polygons for corner in polygon]))
    for (name, own, corners), (_other, triangles, _corners) in ((sides[0], sides[1]), (sides[1], sides[0])):
        locator = _Locator(triangles)
        for point in (*corners, *_samples(own)):
            found = locator.consistent(point)
            if not found:
                where = f"UV ({point[3] / scale:.6g}, {point[4] / scale:.6g})"
                return f"side {name} has a point the other side does not cover, {where}" if found is None else f"side {name} leaves the other side's surface at {where}"
    return None


# --------------------------------------------------------------------------
# Surface comparison
# --------------------------------------------------------------------------


def _edge_uses(surface: Surface) -> dict:
    uses: dict = defaultdict(list)
    for f, face in enumerate(surface.faces):
        for i, vertex in enumerate(face):
            uses[tuple(sorted((vertex, face[(i + 1) % len(face)])))].append(f)
    return uses


def _segment(surface: Surface, edge) -> tuple:
    return surface.points[edge[0]], surface.points[edge[1]]


def _open_edges(surface: Surface, uses: dict) -> frozenset:
    return chain([_segment(surface, edge) for edge, faces in uses.items() if len(faces) == 1], signed=False)


def _joins(surface: Surface, uses: dict) -> dict:
    by_pair: dict = defaultdict(list)
    for edge, faces in uses.items():
        domains = sorted({surface.domain[f] for f in faces}, key=str)
        for i, first in enumerate(domains):
            for second in domains[i + 1 :]:
                by_pair[(first, second)].append(_segment(surface, edge))
    return {pair: chain(segments, signed=False) for pair, segments in by_pair.items()}


def _provenance(surface: Surface) -> dict:
    """Every shared semantic vertex (a provenance ref): its positions and the UVs of its loops."""

    out: dict = defaultdict(lambda: (set(), set()))
    loops: dict = defaultdict(set)
    for f, face in enumerate(surface.faces):
        for vertex, uv in zip(face, surface.corner_uvs[f]):
            loops[vertex].add(uv)
    for vertex, ref in enumerate(surface.refs or ()):
        if ref is not None:
            out[ref][0].add(surface.points[vertex])
            out[ref][1].update(loops[vertex])
    return {ref: (frozenset(points), frozenset(uvs)) for ref, (points, uvs) in out.items()}


def _compare_regions(first: Surface, second: Surface, where: str, report: Report) -> None:
    table_a, table_b = defaultdict(list), defaultdict(list)
    for region in regions(first):
        table_a[(region.domain, region.boundary)].append(region)
    for region in regions(second):
        table_b[(region.domain, region.boundary)].append(region)
    report.compared["regions"] += sum(len(items) for items in table_a.values())
    owners: dict = {}
    retriangulated = 0
    for key in sorted(set(table_a) | set(table_b), key=lambda item: str(item[0])):
        items_a, items_b = table_a.get(key, []), table_b.get(key, [])
        if len(items_a) != len(items_b):
            continue
        for region_a, region_b in zip(items_a, items_b):
            mapped = owners.setdefault((region_a.domain, region_a.owner), region_b.owner)
            if mapped != region_b.owner:
                report.add(GEOMETRY_ERROR, where, f"domain {region_a.domain}: the owner of a region is {region_b.owner} in B, the owner {region_a.owner} of A is {mapped} elsewhere")
            faces_a = Counter(_rotated(first.corners(f)) for f in region_a.faces)
            faces_b = Counter(_rotated(second.corners(f)) for f in region_b.faces)
            if faces_a != faces_b:
                # the faces both sides have are the same surface already: only the rest (one boundary, by the equal chains of the regions) is compared
                problem = interior_difference(list((faces_a - faces_b).elements()), list((faces_b - faces_a).elements()), first.scale)
                if problem is None:
                    retriangulated += 1
                else:
                    report.add(GEOMETRY_ERROR, where, f"domain {region_a.domain} owner {region_a.owner}: inside a region with the same boundary, {problem}")
    unmatched_a = [region for key, items in table_a.items() if len(items) != len(table_b.get(key, [])) for region in items]
    unmatched_b = [region for key, items in table_b.items() if len(items) != len(table_a.get(key, [])) for region in items]
    for region in unmatched_a + unmatched_b:
        side, others = ("A", unmatched_b) if region in unmatched_a else ("B", unmatched_a)
        twin = next((other for other in others if other.domain == region.domain and other.silhouette == region.silhouette), None)
        if twin is not None and side == "B":
            continue
        what = "UV along the boundary of the region differs (the 3-D outline is the same)" if twin is not None else "the outline of the region (silhouette / owner boundary) differs"
        report.add(GEOMETRY_ERROR, where, f"domain {region.domain} owner {region.owner}, region of {len(region.faces)} faces only in {side}: {what}")
    if retriangulated:
        report.add(REPRESENTATION_ONLY, where, f"{retriangulated} regions triangulated otherwise inside the same boundary (the same surface and UV)")
    images: dict = defaultdict(set)
    for (domain, owner), value in owners.items():
        images[(domain, value)].add(owner)
    merged = {key: sorted(found, key=str) for key, found in images.items() if len(found) > 1}
    if merged:
        report.add(GEOMETRY_ERROR, where, f"owners of A merged into one owner of B: {dict(list(merged.items())[:6])}")
    relabelled = {key: value for key, value in owners.items() if key[1] != value}
    if relabelled and not merged:
        report.add(REPRESENTATION_ONLY, where, f"owner labels relabelled by a bijection: {dict(list(relabelled.items())[:6])}")


def compare_surfaces(first: Surface, second: Surface, where: str, report: Report) -> None:
    """Every product difference between two surfaces of one domain (a view) or of the whole mesh, classified into `report`."""

    report.compared["surfaces"] += 1
    first, second = on_common_scale(first, second)
    scale = first.scale
    if _face_multiset(first) != _face_multiset(second):
        _compare_regions(first, second, where, report)
    uses_a, uses_b = _edge_uses(first), _edge_uses(second)
    open_a, open_b = _open_edges(first, uses_a), _open_edges(second, uses_b)
    if open_a != open_b:
        report.add(GEOMETRY_ERROR, where, f"open edges (the silhouette and every crack) differ: {_chain_difference(open_a, open_b, scale)}")
    seams_a = chain([_segment(first, edge) for edge in first.seams], signed=False)
    seams_b = chain([_segment(second, edge) for edge in second.seams], signed=False)
    if seams_a != seams_b:
        report.add(GEOMETRY_ERROR, where, f"seams differ: {_chain_difference(seams_a, seams_b, scale)}")
    joins_a, joins_b = _joins(first, uses_a), _joins(second, uses_b)
    for pair in sorted(set(joins_a) | set(joins_b), key=str):
        if joins_a.get(pair, frozenset()) != joins_b.get(pair, frozenset()):
            report.add(GEOMETRY_ERROR, where, f"the join of domains {pair[0]} and {pair[1]} differs: {_chain_difference(joins_a.get(pair, frozenset()), joins_b.get(pair, frozenset()), scale)}")
    if first.refs is not None or second.refs is not None:
        refs_a, refs_b = _provenance(first), _provenance(second)
        report.compared["semantic vertices"] += len(refs_a)
        for ref in sorted(set(refs_a) | set(refs_b)):
            if refs_a.get(ref) != refs_b.get(ref):
                missing = "A" if ref not in refs_a else ("B" if ref not in refs_b else "")
                what = f"only in {'B' if missing == 'A' else 'A'}" if missing else ("position differs" if refs_a[ref][0] != refs_b[ref][0] else "UV differs")
                report.add(GEOMETRY_ERROR, where, f"shared semantic vertex {ref}: {what}")


def _representation_reason(first: dict, second: dict) -> str:
    numbers = all(
        [[exact(c) for c in row] for row in first[name]] == [[exact(c) for c in row] for row in second[name]] for name in ("positions", "uvs")
    ) and all(first.get(name) == second.get(name) for name in ("faces", "owners", "face_owner", "face_domain", "seams", "refs"))
    return "number representation only (int / Fraction / binary64 spelling)" if numbers else "order, ids or triangulation only"


def _compare_surface_records(first: dict, second: dict, build, where: str, report: Report) -> None:
    if first == second:
        report.compared["identical surfaces"] += 1
        return
    try:
        surface_a, surface_b = build(first), build(second)
    except (MalformedRecord, KeyError, TypeError, ValueError) as error:
        report.add(GEOMETRY_ERROR, where, f"the record cannot be read as a surface: {error}")
        return
    before = len(report.of(GEOMETRY_ERROR))
    compare_surfaces(surface_a, surface_b, where, report)
    if len(report.of(GEOMETRY_ERROR)) == before:
        report.add(REPRESENTATION_ONLY, where, _representation_reason(first, second))


# --------------------------------------------------------------------------
# Presses
# --------------------------------------------------------------------------


def is_resource_refusal(outcome: str, detail: str = "") -> bool:
    return bool(RESOURCE_OUTCOME.search(outcome or "") or RESOURCE_OUTCOME.search(detail or ""))


def _compare_domain(patch: str, a: dict, b: dict, report: Report) -> None:
    where = f"domain {patch}"
    report.compared["domains"] += 1
    if a["outcome"] != b["outcome"]:
        kind = RESOURCE_POLICY if (is_resource_refusal(a["outcome"], a.get("detail", "")) or is_resource_refusal(b["outcome"], b.get("detail", ""))) else GEOMETRY_ERROR
        report.add(kind, where, f"named outcome A {a['outcome']} != B {b['outcome']}")
        return
    if a.get("detail") != b.get("detail"):
        report.add(REPRESENTATION_ONLY, where, "detail text differs")
    if a.get("diagnostics") != b.get("diagnostics"):
        report.add(REPRESENTATION_ONLY, where, "diagnostic text differs")
    view_a, view_b = a.get("view"), b.get("view")
    if view_a is None or view_b is None:
        if (view_a is None) != (view_b is None):
            report.add(REPRESENTATION_ONLY, where, f"the view is carried by {'B' if view_a is None else 'A'} only (the mesh is compared whole)")
        return
    if "failure" in view_a or "failure" in view_b:
        failure_a, failure_b = view_a.get("failure"), view_b.get("failure")
        if (failure_a or [None])[0] != (failure_b or [None])[0]:
            report.add(GEOMETRY_ERROR, where, f"the writer's named outcome A {failure_a} != B {failure_b}")
        elif failure_a != failure_b:
            report.add(REPRESENTATION_ONLY, where, "the writer's detail text differs")
        return
    _compare_surface_records(view_a, view_b, surface_of_view, where, report)


def compare_press_records(first: dict, second: dict) -> Report:
    """Two press records (`press_record`) of one case: every difference, classified."""

    report = Report()
    for name, record in (("A", first), ("B", second)):
        if record.get("schema") != SCHEMA:
            report.add(GEOMETRY_ERROR, "record", f"{name} is not a {SCHEMA} record: {record.get('schema')!r}")
            return report
    domains_a, domains_b = first["domains"], second["domains"]
    for patch in sorted(set(domains_a) ^ set(domains_b), key=lambda item: (len(item), item)):
        report.add(GEOMETRY_ERROR, f"domain {patch}", f"present only in {'A' if patch in domains_a else 'B'}")
    for patch in sorted(set(domains_a) & set(domains_b), key=lambda item: (len(item), item)):
        _compare_domain(patch, domains_a[patch], domains_b[patch], report)
    mesh_a, mesh_b = first.get("mesh"), second.get("mesh")
    if (mesh_a is None) != (mesh_b is None):
        report.add(GEOMETRY_ERROR, "mesh", f"the mesh is present only in {'A' if mesh_b is None else 'B'}")
    elif mesh_a is not None:
        if mesh_a.get("skipped") != mesh_b.get("skipped"):
            report.add(GEOMETRY_ERROR, "mesh", f"the skipped domains differ: A {mesh_a.get('skipped')[:6]} B {mesh_b.get('skipped')[:6]}")
        _compare_surface_records(mesh_a, mesh_b, surface_of_mesh, "mesh", report)
    return report


def compare_determinism(records: dict) -> Report:
    """Records of one case under different threads / task orders / histories (`{label: record}`): the answer must not depend on them, so every
    difference that is not REPRESENTATION_ONLY - a budget refusal included - is a GEOMETRY_ERROR of determinism."""

    report = Report()
    labels = list(records)
    for label in labels[1:]:
        found = compare_press_records(records[labels[0]], records[label])
        for item in found.findings:
            kind = REPRESENTATION_ONLY if item.kind == REPRESENTATION_ONLY else GEOMETRY_ERROR
            report.add(kind, f"determinism {labels[0]} vs {label}, {item.where}", item.what)
        report.compared.update(found.compared)
    return report


def main(argv: list | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("first")
    parser.add_argument("second")
    parser.add_argument("--json", default="", help="write the report here")
    parser.add_argument("--fail-on-resource", action="store_true", help="exit 1 on RESOURCE_POLICY too")
    options = parser.parse_args(argv)
    try:
        records = [json.loads(open(path, encoding="utf-8").read()) for path in (options.first, options.second)]
    except (OSError, ValueError) as error:
        print(f"SEMANTIC_UNREADABLE {error}")
        return 2
    report = compare_press_records(*records)
    for item in report.findings[:40]:
        print(item.line())
    print(f"SEMANTIC_{report.verdict} {report.counts} compared={dict(report.compared)}")
    if options.json:
        with open(options.json, "w", encoding="utf-8") as handle:
            json.dump(report.as_record(), handle, indent=1)
    failing = report.of(GEOMETRY_ERROR) or (options.fail_on_resource and report.of(RESOURCE_POLICY))
    return 1 if failing else 0


if __name__ == "__main__":
    sys.exit(main())
