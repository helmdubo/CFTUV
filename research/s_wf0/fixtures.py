"""Детерминированные mesh fixtures для research-only S-WF0."""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, pi, sin, sqrt

import numpy as np


@dataclass(frozen=True)
class SourceBranch:
    owner: str
    vertex_ids: tuple[int, ...]
    closed: bool = False


@dataclass(frozen=True)
class Fixture:
    name: str
    variant: str
    vertices: np.ndarray
    faces: np.ndarray
    branches: tuple[SourceBranch, ...]
    alpha_reference: float
    planar_polygon: tuple[tuple[float, float], ...] = ()
    common_keys: tuple[str, ...] = ()

    @property
    def key(self) -> str:
        return self.name if not self.variant else f"{self.name}/{self.variant}"


def _triangulate_cells(cells, diagonal="a"):
    faces = []
    for cell_index, (a, b, c, d) in enumerate(cells):
        use_a = diagonal == "a" or (
            diagonal == "checker" and cell_index % 2 == 0
        )
        if use_a:
            faces.extend(((a, b, c), (a, c, d)))
        else:
            faces.extend(((a, b, d), (b, c, d)))
    return np.asarray(faces, dtype=np.int32)


def _grid_polygon(xs, ys, inside, *, diagonal="checker"):
    raw_vertices = [
        (float(x), float(y), 0.0) for y in ys for x in xs
    ]
    row = len(xs)
    raw_cells = []
    for j in range(len(ys) - 1):
        for i in range(len(xs) - 1):
            cx = 0.5 * (xs[i] + xs[i + 1])
            cy = 0.5 * (ys[j] + ys[j + 1])
            if inside(float(cx), float(cy)):
                raw_cells.append(
                    (
                        j * row + i,
                        j * row + i + 1,
                        (j + 1) * row + i + 1,
                        (j + 1) * row + i,
                    )
                )
    used = sorted({vertex for cell in raw_cells for vertex in cell})
    remap = {old: new for new, old in enumerate(used)}
    vertices = np.asarray([raw_vertices[index] for index in used], dtype=float)
    cells = [tuple(remap[value] for value in cell) for cell in raw_cells]
    coordinate_ids = {
        (round(raw_vertices[old][0], 9), round(raw_vertices[old][1], 9)): new
        for new, old in enumerate(used)
    }
    return vertices, _triangulate_cells(cells, diagonal), coordinate_ids


def _axis_path(coordinate_ids, first, second, step=0.5):
    x0, y0 = first
    x1, y1 = second
    length = max(abs(x1 - x0), abs(y1 - y0))
    count = int(round(length / step))
    result = []
    for index in range(count + 1):
        factor = index / max(count, 1)
        key = (round(x0 + (x1 - x0) * factor, 9), round(y0 + (y1 - y0) * factor, 9))
        result.append(coordinate_ids[key])
    return tuple(result)


def _polygon_branches(coordinate_ids, polygon, *, step=0.5):
    return tuple(
        SourceBranch(
            owner=f"boundary_{index}",
            vertex_ids=_axis_path(
                coordinate_ids,
                polygon[index],
                polygon[(index + 1) % len(polygon)],
                step,
            ),
        )
        for index in range(len(polygon))
    )


def planar_l():
    polygon = (
        (-3.0, -3.0),
        (3.0, -3.0),
        (3.0, 0.0),
        (0.0, 0.0),
        (0.0, 3.0),
        (-3.0, 3.0),
    )
    xs = np.linspace(-3.0, 3.0, 13)
    ys = np.linspace(-3.0, 3.0, 13)
    vertices, faces, ids = _grid_polygon(
        xs,
        ys,
        lambda x, y: not (x > 0.0 and y > 0.0),
    )
    return Fixture(
        "planar_l",
        "",
        vertices,
        faces,
        _polygon_branches(ids, polygon),
        1.6,
        polygon,
    )


def planar_tx():
    xs = np.linspace(-3.0, 3.0, 13)
    ys = np.linspace(-3.0, 3.0, 13)
    vertices, faces, ids = _grid_polygon(xs, ys, lambda _x, _y: True)
    branches = tuple(
        SourceBranch(owner, _axis_path(ids, first, second))
        for owner, first, second in (
            ("west", (0.0, 0.0), (-2.5, 0.0)),
            ("east", (0.0, 0.0), (2.5, 0.0)),
            ("south", (0.0, 0.0), (0.0, -2.5)),
            ("north", (0.0, 0.0), (0.0, 2.5)),
        )
    )
    return Fixture("planar_tx", "", vertices, faces, branches, 1.0)


def concave_owner():
    polygon = (
        (-3.0, -3.0),
        (3.0, -3.0),
        (3.0, 3.0),
        (1.5, 3.0),
        (1.5, 0.0),
        (-1.5, 0.0),
        (-1.5, 3.0),
        (-3.0, 3.0),
    )
    xs = np.linspace(-3.0, 3.0, 13)
    ys = np.linspace(-3.0, 3.0, 13)
    vertices, faces, ids = _grid_polygon(
        xs,
        ys,
        lambda x, y: not (-1.5 < x < 1.5 and y > 0.0),
    )
    return Fixture(
        "concave_owner",
        "",
        vertices,
        faces,
        _polygon_branches(ids, polygon),
        1.6,
        polygon,
    )


def cylinder_one_seam(*, azimuth_segments=24, height_segments=12):
    vertices = []
    at = {}
    for j in range(height_segments + 1):
        z = -2.0 + 4.0 * j / height_segments
        for i in range(azimuth_segments):
            theta = 2.0 * pi * i / azimuth_segments
            at[(i, j)] = len(vertices)
            vertices.append((cos(theta), sin(theta), z))
    cells = []
    for j in range(height_segments):
        for i in range(azimuth_segments):
            following = (i + 1) % azimuth_segments
            cells.append(
                (at[(i, j)], at[(following, j)], at[(following, j + 1)], at[(i, j + 1)])
            )
    branch = SourceBranch(
        "seam",
        tuple(at[(0, j)] for j in range(height_segments + 1)),
    )
    return Fixture(
        "cylinder_one_seam",
        "",
        np.asarray(vertices, dtype=float),
        _triangulate_cells(cells, "checker"),
        (branch,),
        3.2,
    )


def half_sphere(*, azimuth_segments=24, latitude_segments=10):
    vertices = [(0.0, 0.0, 1.0)]
    rings = []
    for j in range(1, latitude_segments + 1):
        theta = 0.5 * pi * j / latitude_segments
        ring = []
        for i in range(azimuth_segments):
            phi = 2.0 * pi * i / azimuth_segments
            ring.append(len(vertices))
            vertices.append(
                (sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta))
            )
        rings.append(ring)
    faces = []
    for i in range(azimuth_segments):
        faces.append((0, rings[0][i], rings[0][(i + 1) % azimuth_segments]))
    for j in range(len(rings) - 1):
        for i in range(azimuth_segments):
            following = (i + 1) % azimuth_segments
            a, b = rings[j][i], rings[j][following]
            c, d = rings[j + 1][following], rings[j + 1][i]
            if (i + j) % 2:
                faces.extend(((d, b, a), (d, c, b)))
            else:
                faces.extend(((c, b, a), (d, c, a)))
    branch = SourceBranch(
        "meridian",
        (0,) + tuple(ring[0] for ring in rings),
    )
    return Fixture(
        "half_sphere",
        "",
        np.asarray(vertices, dtype=float),
        np.asarray(faces, dtype=np.int32),
        (branch,),
        1.7,
    )


def _open_sheet(z, id_offset, *, cells_per_axis=8):
    vertices = []
    at = {}
    for j in range(cells_per_axis + 1):
        y = -2.0 + 4.0 * j / cells_per_axis
        for i in range(cells_per_axis + 1):
            x = -2.0 + 4.0 * i / cells_per_axis
            at[(i, j)] = id_offset + len(vertices)
            vertices.append((x, y, z))
    cells = []
    for j in range(cells_per_axis):
        for i in range(cells_per_axis):
            cells.append(
                (at[(i, j)], at[(i + 1, j)], at[(i + 1, j + 1)], at[(i, j + 1)])
            )
    return vertices, cells, at


def close_parallel_sheets():
    first_vertices, first_cells, first_at = _open_sheet(0.0, 0)
    second_vertices, second_cells, _second_at = _open_sheet(
        0.04, len(first_vertices)
    )
    vertices = np.asarray(first_vertices + second_vertices, dtype=float)
    cells = first_cells + second_cells
    branch = SourceBranch(
        "sheet_a",
        tuple(first_at[(4, j)] for j in range(9)),
    )
    return Fixture(
        "close_parallel_sheets",
        "",
        vertices,
        _triangulate_cells(cells, "checker"),
        (branch,),
        2.1,
    )


def retriangulated_surface(variant):
    azimuth_segments = 16
    height_segments = 12
    angles = np.linspace(-1.2, 1.2, azimuth_segments + 1)
    vertices = []
    at = {}
    keys = []
    for j in range(height_segments + 1):
        z = -2.0 + 4.0 * j / height_segments
        for i, theta in enumerate(angles):
            at[(i, j)] = len(vertices)
            vertices.append((1.7 * cos(theta), 1.7 * sin(theta), z))
            keys.append(f"u{i}:v{j}")
    cells = []
    for j in range(height_segments):
        for i in range(azimuth_segments):
            cells.append(
                (at[(i, j)], at[(i + 1, j)], at[(i + 1, j + 1)], at[(i, j + 1)])
            )
    center = azimuth_segments // 2
    branch = SourceBranch(
        "center_seam",
        tuple(at[(center, j)] for j in range(height_segments + 1)),
    )
    return Fixture(
        "retriangulated_surface",
        variant,
        np.asarray(vertices, dtype=float),
        _triangulate_cells(cells, variant),
        (branch,),
        2.1,
        common_keys=tuple(keys),
    )


def fold_near_smooth():
    cells_per_axis = 16
    vertices = []
    at = {}
    for j in range(cells_per_axis + 1):
        y = -2.0 + 4.0 * j / cells_per_axis
        for i in range(cells_per_axis + 1):
            x = -2.0 + 4.0 * i / cells_per_axis
            z = -0.7 * x if x < 0.0 else 0.08 * x * x
            at[(i, j)] = len(vertices)
            vertices.append((x, y, z))
    cells = []
    for j in range(cells_per_axis):
        for i in range(cells_per_axis):
            cells.append(
                (at[(i, j)], at[(i + 1, j)], at[(i + 1, j + 1)], at[(i, j + 1)])
            )
    middle = cells_per_axis // 2
    branch = SourceBranch(
        "cross_fold",
        tuple(at[(i, middle)] for i in range(cells_per_axis + 1)),
    )
    return Fixture(
        "fold_near_smooth",
        "",
        np.asarray(vertices, dtype=float),
        _triangulate_cells(cells, "checker"),
        (branch,),
        2.1,
    )


def all_fixtures():
    return (
        planar_l(),
        planar_tx(),
        concave_owner(),
        cylinder_one_seam(),
        half_sphere(),
        close_parallel_sheets(),
        retriangulated_surface("a"),
        retriangulated_surface("b"),
        fold_near_smooth(),
    )


def mesh_edges(fixture):
    edges = {
        tuple(sorted((int(face[index]), int(face[(index + 1) % 3]))))
        for face in fixture.faces
        for index in range(3)
    }
    return tuple(sorted(edges))


def boundary_edges(fixture):
    counts = {}
    for face in fixture.faces:
        for index in range(3):
            edge = tuple(sorted((int(face[index]), int(face[(index + 1) % 3]))))
            counts[edge] = counts.get(edge, 0) + 1
    return tuple(sorted(edge for edge, count in counts.items() if count == 1))


def connected_components(fixture):
    graph = {index: set() for index in range(len(fixture.vertices))}
    for first, second in mesh_edges(fixture):
        graph[first].add(second)
        graph[second].add(first)
    remaining = set(graph)
    result = []
    while remaining:
        seed = min(remaining)
        remaining.remove(seed)
        component = {seed}
        stack = [seed]
        while stack:
            current = stack.pop()
            for neighbor in graph[current]:
                if neighbor not in remaining:
                    continue
                remaining.remove(neighbor)
                component.add(neighbor)
                stack.append(neighbor)
        result.append(tuple(sorted(component)))
    return tuple(result)


def source_records(fixture):
    records = []
    station_offset = 0.0
    for branch in fixture.branches:
        station = 0.0
        ids = branch.vertex_ids
        for index, vertex_id in enumerate(ids):
            if index:
                station += float(
                    np.linalg.norm(
                        fixture.vertices[vertex_id]
                        - fixture.vertices[ids[index - 1]]
                    )
                )
            records.append(
                {
                    "vertex_id": int(vertex_id),
                    "owner": branch.owner,
                    "source_s": station_offset + station,
                }
            )
        station_offset += station
        if branch.closed and len(ids) > 1:
            station_offset += float(
                np.linalg.norm(fixture.vertices[ids[0]] - fixture.vertices[ids[-1]])
            )
    return tuple(records), max(station_offset, 1e-12)


__all__ = (
    "Fixture",
    "SourceBranch",
    "all_fixtures",
    "boundary_edges",
    "connected_components",
    "mesh_edges",
    "source_records",
)
