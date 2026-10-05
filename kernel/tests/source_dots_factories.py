"""Полоса вдоль цепи источника как батч домена (`GeometryBatchV1`, собран руками): общая фабрика тестов `SILHOUETTE_SOURCE_DOTS_V1`.

Батч проходит `validate_geometry_batch` и `audit_batch`; вершины `src:s<i>` лежат на цепи источника (`y = 0`), так что полоса
`side = 1` и полоса `side = -1` с теми же `xs` делят цепь: одни и те же ключи, ссылки места и позиции, как у соседних доменов.
"""

from __future__ import annotations

import dataclasses
import math
from decimal import Decimal

from cftuv_envelope.canonical import geometry_batch_semantic_digest
from cftuv_envelope.contracts.envelopes import StationModelId
from cftuv_envelope.contracts.geometry_batch import (
    GEOMETRY_BATCH_SCHEMA_V1,
    GeometryBatchV1,
    GeometryBoundaryChainV1,
    GeometryFaceV1,
    GeometryProvenanceV1,
    GeometrySemanticRegionV1,
    GeometryStationFactV1,
    GeometryUvFactV1,
    GeometryVertexV1,
)
from cftuv_envelope.ids import (
    DecalRequestId,
    GeometryFaceId,
    GeometryStationFactId,
    MaterialId,
    OwnershipClaimId,
    PatchDomainId,
    SemanticBoundaryId,
    SemanticDigestValue,
    SemanticLocationId,
    SemanticRegionId,
    SourceRevision,
    VertexKey,
)
from cftuv_envelope.materialize.assemble import paths_of
from cftuv_envelope.numeric import LocalCoordinateV1, LocalPoint3V1, UvPoint2V1

NORMAL = (0.0, 0.0, 1.0)
PROVENANCE = GeometryProvenanceV1(frozenset(), frozenset(), frozenset(), frozenset())
HEIGHT = 0.5  # метры: ширина полосы
XS = [0.0, 0.25, 0.5, 0.75, 1.0]


def _vertex(key, x, y, z=0.0, ref=None):
    return GeometryVertexV1(VertexKey(key), LocalPoint3V1(x, y, z), SemanticLocationId(ref or f"location:{key}"), PROVENANCE)


def strip(name, xs, *, side=1, bent=(), skew=(), tilt=0.0, rung=None, sag=(), local=(), height=HEIGHT, refs=()):
    """Полоса `xs` метров вдоль цепи источника: вершины `src:s<i>` на `y = 0`, два угла на фронте; `side = -1` — полоса по другую сторону цепи.

    `bent` — номера вершин, выведенных из прямой на 1 см (излом много больше 0.1 градуса), `skew` — `{номер: сдвиг u}` (UV вершины
    сдвинута), `sag` — `{номер: метры}`: отклонение вершины от прямой (малое: излом остаётся в допуске), `tilt` — наклон полосы вокруг
    цепи (радианы: компланарность с соседом), `rung` — номер вершины, из которой идёт внутреннее ребро на фронт (грань делится надвое),
    `local` — `{номер: ключ}`: вершина домена (не `src:`) на цепи между вершинами `i` и `i + 1` (середина ребра), `height` — ширина полосы, `refs` — `{ключ: ссылка места}` (подмена `location:<ключ>`: двойник-копия места).
    """

    count = len(xs)
    sag, skew, local, refs = dict(sag), dict(skew), dict(local), dict(refs)
    bottom = {
        f"src:s{i}": (xs[i], (0.01 if i in bent else sag.get(i, 0.0)), 0.0) for i in range(count)
    }
    low = []
    for i in range(count):
        low.append(f"src:s{i}")
        if i in local:
            low.append(local[i])
            bottom[local[i]] = ((xs[i] + xs[i + 1]) / 2, 0.0, 0.0)
    top_z = side * height * math.sin(tilt)
    top = {
        f"node:{name}0": (xs[0], side * height * math.cos(tilt), top_z),
        f"node:{name}1": (xs[-1], side * height * math.cos(tilt), top_z),
    }
    if rung is not None:
        top[f"node:{name}r"] = (xs[rung], side * height * math.cos(tilt), top_z)
    points = {**bottom, **top}
    if rung is None:
        rings = [low + [f"node:{name}1", f"node:{name}0"]] if side > 0 else [[low[0], f"node:{name}0", f"node:{name}1"] + low[::-1][:-1]]
    elif side > 0:
        left = low[: rung + 1] + [f"node:{name}r", f"node:{name}0"]
        right = low[rung:] + [f"node:{name}1", f"node:{name}r"]
        rings = [left, right]
    else:
        left = [low[0], f"node:{name}0", f"node:{name}r"] + low[: rung + 1][::-1][:-1]
        right = [low[rung], f"node:{name}r", f"node:{name}1"] + low[rung:][::-1][:-1]
        rings = [left, right]
    uv = {}
    for key, (x, y, _z) in points.items():
        uv[key] = (x / 1.0 + skew.get(int(key[5:]), 0.0) if key.startswith("src:s") else x / 1.0, min(1.0, abs(y) / height))
    def kind_of(first, second):
        if first in bottom and second in bottom:
            return "SOURCE"
        return "WALL" if first in bottom or second in bottom else "RIM"

    return _assemble(name, points, rings, uv, refs, kind_of)


def square(name, per_side=3):
    """Квадрат, чья ВЕСЬ граница — одна ЗАМКНУТАЯ цепь источника: углы `src:z<k>` (излом 90 градусов) и `per_side` точек `src:a<k><i>` на каждой стороне.

    Имена точек меньше имён углов, поэтому замкнутый путь цепи начинается и кончается в точке, а не в углу.
    """

    corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    points, ring = {}, []
    for side, (start, end) in enumerate(zip(corners, corners[1:] + corners[:1])):
        points[f"src:z{side}"] = (*start, 0.0)
        ring.append(f"src:z{side}")
        for step in range(1, per_side + 1):
            share = step / (per_side + 1)
            points[f"src:a{side}{step}"] = (start[0] + (end[0] - start[0]) * share, start[1] + (end[1] - start[1]) * share, 0.0)
            ring.append(f"src:a{side}{step}")
    uv = {key: (x, y) for key, (x, y, _z) in points.items()}
    return _assemble(name, points, [ring], uv, {}, lambda _first, _second: "SOURCE")


def _assemble(name, points, rings, uv, refs, kind_of):
    """Батч из колец одного региона: UV и факты станций по ключам, цепи — по полурёбрам без пары (вид ребра называет `kind_of`)."""

    faces = []
    for number, ring in enumerate(rings):
        faces.append(
            GeometryFaceV1(
                GeometryFaceId(f"face:{number}"),
                tuple(VertexKey(key) for key in ring),
                tuple(GeometryUvFactV1(VertexKey(key), UvPoint2V1(*uv[key])) for key in ring),
                SemanticRegionId("region:0"),
                OwnershipClaimId("claim:0"),
                PROVENANCE,
                MaterialId("material"),
            )
        )
    kinds = {}
    for ring in rings:
        for first, second in zip(ring, ring[1:] + ring[:1]):
            twin = any((second, first) in zip(other, other[1:] + other[:1]) for other in rings)
            if twin:
                continue
            kinds.setdefault(kind_of(first, second), []).append((first, second))
    chains = []
    for kind, edges in sorted(kinds.items()):
        for number, path in enumerate(paths_of(edges)):
            chains.append(GeometryBoundaryChainV1(SemanticBoundaryId(f"boundary:{kind}:0:{number}"), tuple(VertexKey(key) for key in path)))
    batch = GeometryBatchV1(
        schema_version=GEOMETRY_BATCH_SCHEMA_V1,
        source_revision=SourceRevision("revision"),
        decal_request_id=DecalRequestId("request"),
        patch_domain_id=PatchDomainId(f"domain:{name}"),
        vertices=frozenset(_vertex(key, *position, ref=refs.get(key)) for key, position in points.items()),
        faces=tuple(faces),
        station_facts=frozenset(
            GeometryStationFactV1(
                GeometryStationFactId(f"station:{key}"),
                VertexKey(key),
                SemanticRegionId("region:0"),
                OwnershipClaimId("claim:0"),
                LocalCoordinateV1(Decimal(repr(uv[key][0]))),
                LocalCoordinateV1(Decimal(repr(uv[key][1]))),
                StationModelId.SEMANTIC_CHAIN_USE_S,
                frozenset(),
            )
            for key in points
        ),
        semantic_regions=frozenset(
            {GeometrySemanticRegionV1(SemanticRegionId("region:0"), OwnershipClaimId("claim:0"), MaterialId("material"), PROVENANCE)}
        ),
        boundary_chains=frozenset(chains),
        interface_chains=frozenset(),
        diagnostics=frozenset(),
        contract_versions=frozenset(),
        semantic_digest=SemanticDigestValue("pending"),
    )
    return dataclasses.replace(batch, semantic_digest=SemanticDigestValue(geometry_batch_semantic_digest(batch).sha256_hex))
