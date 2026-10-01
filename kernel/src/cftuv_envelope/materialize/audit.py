"""Геометрический аудит материализованного батча: то, чего валидатор не знает.

`validate_geometry_batch` проверяет СВЯЗНОСТЬ записей (ссылки, факты станций,
дайджест). Он не знает, что меш — это сетка, и не сможет заметить трещину,
наложение или перевёрнутую грань. Эти свойства сетки считаются здесь, один раз,
и входят в исход материализации:

ГРАНИ — ЛЮБОЙ ДЛИНЫ (закон топологии `QUAD_STRIPS_V1` кладёт в батч четырёхгранья).
Обход, вектор площади и UV-площадь считаются веером из первой вершины; у
треугольника это ПРЕЖНИЕ формулы по трём точкам, побитово. Имена счётчиков
(`..._TRIANGLES_...`) остались прежними, считают они ГРАНИ: под `TRIANGLES_V1`
это треугольники, под `QUAD_STRIPS_V1` — и четырёхгранья.

ЖЁСТКИЕ (нарушение — отказ `BATCH_DID_NOT_VALIDATE`, деталь `AUDIT:<имя>`):

* `EDGE_SHARED_BY_MORE_THAN_TWO` — ребро в трёх и более гранях;
* `HALF_EDGE_DUPLICATED` — одно направленное ребро в двух гранях
  (наложение либо противоположные обходы соседей);
* `BOUNDARY_DOES_NOT_MATCH_CHAINS` — рёбра в ОДНОЙ грани — это ровно
  рёбра граничных цепей, не больше и не меньше (иначе трещина либо цепь
  описывает не то);
* `V_OUT_OF_UNIT_RANGE` — закон `UV_DIRECT_STRIP_V1` обещает `v` в `[0, 1]`.

МЯГКИЕ (числа в счётчиках, не отказ — это свойства закона, а не дефекты):

* `flipped_vs_source` — грани, смотрящие против нормали исходной грани;
* `uv_degenerate` — грани с НУЛЕВОЙ UV-площадью: грани веера (станция
  константна, `u` не меняется — ограничение закона V1, названное, а не
  скрытое);
* `uv_reversed` — грани с обратным обходом в UV относительно
  большинства невырожденных.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BatchAuditV1:
    faces: int
    boundary_edges: int
    overshared_edges: int
    duplicated_half_edges: int
    boundary_chain_mismatch: int
    v_min: float
    v_max: float
    flipped_vs_source: int
    uv_degenerate: int
    uv_reversed: int

    def problems(self) -> tuple[str, ...]:
        found = []
        if self.overshared_edges:
            found.append("EDGE_SHARED_BY_MORE_THAN_TWO")
        if self.duplicated_half_edges:
            found.append("HALF_EDGE_DUPLICATED")
        if self.boundary_chain_mismatch:
            found.append("BOUNDARY_DOES_NOT_MATCH_CHAINS")
        if self.faces and (self.v_min < 0.0 or self.v_max > 1.0):
            found.append("V_OUT_OF_UNIT_RANGE")
        return tuple(found)

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            ("MATERIALIZE_BOUNDARY_EDGES", self.boundary_edges),
            ("MATERIALIZE_TRIANGLES_FLIPPED_VS_SOURCE", self.flipped_vs_source),
            ("MATERIALIZE_TRIANGLES_UV_DEGENERATE", self.uv_degenerate),
            ("MATERIALIZE_TRIANGLES_UV_REVERSED", self.uv_reversed),
        )


def _cross(a, b, c):
    ux, uy, uz = b.x - a.x, b.y - a.y, b.z - a.z
    vx, vy, vz = c.x - a.x, c.y - a.y, c.z - a.z
    return (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)


def _area_vector(points):
    """Вектор удвоенной площади грани: у треугольника — `_cross`, дальше веер из первой вершины."""

    total = _cross(points[0], points[1], points[2])
    for index in range(2, len(points) - 1):
        part = _cross(points[0], points[index], points[index + 1])
        total = (total[0] + part[0], total[1] + part[1], total[2] + part[2])
    return total


def _uv_area(uv):
    """Удвоенная ориентированная UV-площадь: у треугольника прежняя формула, дальше веер."""

    def term(first, second):
        return (second.u - uv[0].u) * (first.v - uv[0].v) - (second.v - uv[0].v) * (
            first.u - uv[0].u
        )

    area = term(uv[2], uv[1])
    for index in range(2, len(uv) - 1):
        area += term(uv[index + 1], uv[index])
    return area


def audit_batch(batch, source_normal, vertex_normals=None) -> BatchAuditV1:
    """Свойства сетки батча. `source_normal` — нормаль исходной грани (`x, y, z`).

    `vertex_normals` — `{vert_key.value: нормаль смещения}` у домена развёртки: на сгибе
    нет ОДНОЙ нормали исходной грани, и «смотрит против источника» меряется против
    среднего нормалей вершин самой грани сетки. У плоского домена не передаётся.
    """

    position = {item.vert_key: item.position for item in batch.vertices}
    directed: Counter = Counter()
    flipped = degenerate = 0
    uv_signs: list[int] = []
    v_min, v_max = 1.0, 0.0
    for face in batch.faces:
        keys = face.ordered_vert_keys
        for index in range(len(keys)):
            directed[(keys[index], keys[(index + 1) % len(keys)])] += 1
        normal = _area_vector(tuple(position[key] for key in keys))
        reference = (
            source_normal
            if not vertex_normals
            else tuple(
                sum(vertex_normals[key.value][axis] for key in keys) for axis in range(3)
            )
        )
        facing = (
            normal[0] * reference[0]
            + normal[1] * reference[1]
            + normal[2] * reference[2]
        )
        flipped += int(facing <= 0.0)
        uv = [fact.uv for fact in face.uv_facts]
        area = _uv_area(uv)
        if area == 0.0:
            degenerate += 1
        else:
            uv_signs.append(1 if area > 0.0 else -1)
        for item in uv:
            v_min = min(v_min, item.v)
            v_max = max(v_max, item.v)
    undirected: Counter = Counter()
    for (first, second), count in directed.items():
        undirected[frozenset((first, second))] += count
    boundary = {key for key, count in undirected.items() if count == 1}
    chain_edges = {
        frozenset((chain.ordered_vert_keys[index], chain.ordered_vert_keys[index + 1]))
        for chain in batch.boundary_chains
        for index in range(len(chain.ordered_vert_keys) - 1)
    }
    majority = 1 if sum(uv_signs) >= 0 else -1
    return BatchAuditV1(
        faces=len(batch.faces),
        boundary_edges=len(boundary),
        overshared_edges=sum(1 for count in undirected.values() if count > 2),
        duplicated_half_edges=sum(1 for count in directed.values() if count > 1),
        boundary_chain_mismatch=len(boundary ^ chain_edges),
        v_min=v_min if batch.faces else 0.0,
        v_max=v_max if batch.faces else 0.0,
        flipped_vs_source=flipped,
        uv_degenerate=degenerate,
        uv_reversed=sum(1 for sign in uv_signs if sign != majority),
    )
