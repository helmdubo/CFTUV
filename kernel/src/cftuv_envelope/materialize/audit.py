"""Геометрический аудит материализованного батча: то, чего валидатор не знает.

`validate_geometry_batch` проверяет СВЯЗНОСТЬ записей (ссылки, факты станций,
дайджест). Он не знает, что меш — это сетка, и не сможет заметить трещину,
наложение или перевёрнутую грань. Эти свойства сетки считаются здесь, один раз,
и входят в исход материализации:

ЖЁСТКИЕ (нарушение — отказ `BATCH_DID_NOT_VALIDATE`, деталь `AUDIT:<имя>`):

* `EDGE_SHARED_BY_MORE_THAN_TWO` — ребро в трёх и более треугольниках;
* `HALF_EDGE_DUPLICATED` — одно направленное ребро в двух треугольниках
  (наложение либо противоположные обходы соседей);
* `BOUNDARY_DOES_NOT_MATCH_CHAINS` — рёбра в ОДНОМ треугольнике — это ровно
  рёбра граничных цепей, не больше и не меньше (иначе трещина либо цепь
  описывает не то);
* `V_OUT_OF_UNIT_RANGE` — закон `UV_DIRECT_STRIP_V1` обещает `v` в `[0, 1]`.

МЯГКИЕ (числа в счётчиках, не отказ — это свойства закона, а не дефекты):

* `flipped_vs_source` — треугольники, смотрящие против нормали исходной грани;
* `uv_degenerate` — треугольники с НУЛЕВОЙ UV-площадью: грани веера (станция
  константна, `u` не меняется — ограничение закона V1, названное, а не
  скрытое);
* `uv_reversed` — треугольники с обратным обходом в UV относительно
  большинства невырожденных.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BatchAuditV1:
    triangles: int
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
        if self.triangles and (self.v_min < 0.0 or self.v_max > 1.0):
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


def audit_batch(batch, source_normal) -> BatchAuditV1:
    """Свойства сетки батча. `source_normal` — нормаль исходной грани (`x, y, z`)."""

    position = {item.vert_key: item.position for item in batch.vertices}
    directed: Counter = Counter()
    flipped = degenerate = 0
    uv_signs: list[int] = []
    v_min, v_max = 1.0, 0.0
    for face in batch.faces:
        keys = face.ordered_vert_keys
        for index in range(3):
            directed[(keys[index], keys[(index + 1) % 3])] += 1
        normal = _cross(*(position[key] for key in keys))
        facing = (
            normal[0] * source_normal[0]
            + normal[1] * source_normal[1]
            + normal[2] * source_normal[2]
        )
        flipped += int(facing <= 0.0)
        uv = [fact.uv for fact in face.uv_facts]
        area = (uv[1].u - uv[0].u) * (uv[2].v - uv[0].v) - (
            uv[1].v - uv[0].v
        ) * (uv[2].u - uv[0].u)
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
        triangles=len(batch.faces),
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
