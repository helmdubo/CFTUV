"""Кадр станции каждой слитой грани: чья система `(s, r)` у неё и под чьим именем.

Что называется. У грани ровно два имени, и оба обязаны быть выведены, а не
придуманы:

* ОГИБАЮЩАЯ (`claim_key`) — кто владеет гранью: имя экземпляра юбки на этой
  alpha, а если ядро его не назвало (у скрытых опор веера экземпляра нет),
  имя спеки (`spec:...`), а если и спеки нет (коллинеарные рёбра одной цепи:
  мост не различает их законы, `AMBIGUOUS_OWNER_EDGES`), имя ЦЕПИ источника
  (`chain:...`). Ни одного имени — именованный отказ, а не общий мешок;
* КАДР (`frame_key`) — в чьей системе станций считается `(s, r)`: пробег цепи
  (`stations.StationRunV1`) для грани ребра-источника, либо (вершина, пробег
  входящего ребра) для веера. Грани одного кадра делят вершины с ОДНИМ
  набором `(s, r)`; на стыке кадров вершина законно имеет два набора, и они
  живут в разных семантических регионах (`GeometryBatch` требует по одному
  `(s, r)` на вершину в регионе).

Веер вогнутой вершины считается в кадре ВХОДЯЩЕГО в вершину ребра-источника
(если оно источник, иначе исходящего): станция константна и равна станции
вершины на цепи (`CONSTANT_PHYSICAL_ENDPOINT_S`), `r` — время прихода скрытой
опоры. Тогда веер продолжает полосу входящего ребра по `u` без разрыва, а шов
остаётся только там, где цепи действительно разные.

Мягкий излом одной цепи (`CORNER_JOIN_SOFT_BEND_V1`, веера нет) — ПОТОК: кадр
и огибающая у полос по обе стороны угла одни (`stations.flow_of_run`), `s`
копится сквозь угол, а перекладина на биссектрисе получает станцию вершины
цепи (`assemble.station_values`).
"""

from __future__ import annotations

from dataclasses import dataclass

from ..contracts.envelopes import StationModelId
from ..exact_sqrt_sum import SqrtSumV1
from .admit import MaterializationOutcome
from .coalesce import CoveredFaceV1
from .stations import ChainStationTableV1, StationRunV1, station_of


class MaterializationRefusal(Exception):
    """Именованный отказ материализации домена; исход лежит в `outcome`.

    `counters` — числа, которые стадия успела посчитать до отказа: отказ без
    чисел не отличить от «не дошли», а потеря граней именно такой отказ.
    """

    def __init__(
        self,
        outcome: MaterializationOutcome,
        detail: str,
        counters: tuple[tuple[str, int], ...] = (),
    ):
        super().__init__(f"{outcome.value}: {detail}")
        self.outcome = outcome
        self.detail = detail
        self.counters = tuple(counters)

    def augmented(
        self, extra: str = "", counters: tuple[tuple[str, int], ...] = ()
    ) -> "MaterializationRefusal":
        """Тот же исход с пояснением в хвосте детали и добавленными числами."""

        return MaterializationRefusal(
            self.outcome, f"{self.detail}{extra}", (*self.counters, *counters)
        )


@dataclass(frozen=True, slots=True)
class FrameFaceV1:
    """Слитая грань вместе со своим кадром станции и провенансом."""

    face: CoveredFaceV1
    line: object
    claim_key: str
    frame_key: str
    station_model: StationModelId
    run: StationRunV1
    #: Константная станция веера (`SqrtSumV1`, единицы решётки); у полосы `None`.
    fan_station: SqrtSumV1 | None
    physical_edge_ids: frozenset[str]
    chain_use_ids: frozenset[str]
    chain_ids: frozenset[str]
    #: Ключ ПОТОКА (`stations.flow_of_run`) у полосы, чей пробег лежит в потоке из двух
    #: и более `ChainUse` (`CORNER_JOIN_SOFT_BEND_V1`); иначе `None`. Только у такой
    #: полосы четырёхгранье может нести билинейную UV (`QUAD_UV_BILINEAR_V1`).
    flow_key: str | None = None

    @property
    def is_fan(self) -> bool:
        return self.fan_station is not None


def claim_key_of(face: CoveredFaceV1) -> str | None:
    """Имя огибающей грани: экземпляр, иначе спека, иначе цепь источника."""

    if face.envelope_instance_id:
        return face.envelope_instance_id
    if face.envelope_spec_id:
        return f"spec:{face.envelope_spec_id}"
    if face.source_chain_id:
        return f"chain:{face.source_chain_id}"
    return None


def _is_source(source_keys, key) -> bool:
    return key in source_keys or (key[2], key[3], key[0], key[1]) in source_keys


def _strip_frame(table: ChainStationTableV1, region_id: str, face):
    owners = (face.owner, *face.merged_owners)
    edges = [table.edge_of_owner(region_id, owner) for owner in owners]
    if any(edge is None for edge in edges):
        raise MaterializationRefusal(
            MaterializationOutcome.STATION_CHAIN_UNNAMED,
            f"owner {face.owner}: no chain-use edge in the domain loops",
        )
    runs = {edge.run_id for edge in edges}
    if len(runs) != 1:
        raise MaterializationRefusal(
            MaterializationOutcome.STATION_FRAME_IS_AMBIGUOUS,
            f"owner {face.owner}: merged owners span stations {sorted(runs)}",
        )
    return table.runs[edges[0].run_id], edges


def _fan_frame(table: ChainStationTableV1, region_id: str, face, source_keys):
    node = (int(face.owner[0]), int(face.owner[1]))
    for corner in table.corners.get((region_id, node), ()):
        for key in (corner.incoming, corner.outgoing):
            edge = table.edges.get((region_id, key))
            if edge is not None and _is_source(source_keys, key):
                if corner.vertex_id is None:
                    break
                return table.runs[edge.run_id], edge, corner
    raise MaterializationRefusal(
        MaterializationOutcome.STATION_CHAIN_UNNAMED,
        f"fan owner {face.owner}: its corner has no named source edge",
    )


def resolve_frame(
    table: ChainStationTableV1,
    region_id: str,
    face: CoveredFaceV1,
    line,
    source_keys,
) -> FrameFaceV1:
    """Кадр и имя одной слитой грани, либо `MaterializationRefusal`."""

    claim = claim_key_of(face)
    if claim is None:
        raise MaterializationRefusal(
            MaterializationOutcome.STATION_CHAIN_UNNAMED,
            f"owner {face.owner}: no instance, spec or chain names its claim",
        )
    if len(face.owner) == 4:
        run, edges = _strip_frame(table, region_id, face)
        # ПОТОК (`CORNER_JOIN_SOFT_BEND_V1`): пробеги вхождений, связанных углом
        # JOIN, делят один кадр И одно имя огибающей — иначе грани двух полос
        # легли бы в разные регионы, а граница регионов есть шов.
        flow = table.flow_of_run.get(run.run_id)
        return FrameFaceV1(
            face=face,
            line=line,
            claim_key=claim if flow is None else f"flow:{flow}",
            frame_key=run.run_id if flow is None else flow,
            station_model=StationModelId.SEMANTIC_CHAIN_USE_S,
            run=run,
            fan_station=None,
            physical_edge_ids=frozenset(edge.physical_edge_id for edge in edges),
            chain_use_ids=frozenset(edge.chain_use_id for edge in edges),
            chain_ids=frozenset(edge.chain_id for edge in edges),
            flow_key=flow,
        )
    run, edge, corner = _fan_frame(table, region_id, face, source_keys)
    incident = [
        table.edges.get((region_id, corner.incoming)),
        table.edges.get((region_id, corner.outgoing)),
    ]
    incident = [item for item in incident if item is not None]
    node = corner.node
    station = station_of(
        run, (SqrtSumV1.rational(node[0]), SqrtSumV1.rational(node[1]))
    )
    return FrameFaceV1(
        face=face,
        line=line,
        claim_key=claim,
        frame_key=f"fan:{corner.vertex_id}:{run.run_id}",
        station_model=StationModelId.CONSTANT_PHYSICAL_ENDPOINT_S,
        run=run,
        fan_station=station,
        physical_edge_ids=frozenset(item.physical_edge_id for item in incident),
        chain_use_ids=frozenset(item.chain_use_id for item in incident),
        chain_ids=frozenset(item.chain_id for item in incident),
    )
