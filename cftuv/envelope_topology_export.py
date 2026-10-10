"""Immutable host-topology export for Envelope debug.

The export owns PhysicalChain common refinement and directed ChainUse
collection for one SourceRevision.  Request selection is a cheap view over
this immutable result; it never rebuilds host analysis.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from fractions import Fraction
from types import MappingProxyType
from typing import TYPE_CHECKING, Mapping

from .envelope_debug_profile import EnvelopeDebugProfileBuilderV1
from .envelope_request_policy import DEFAULT_ENVELOPE_STRETCH_BUDGET
from .surface_index import graph_index_of, surface_index_of

if TYPE_CHECKING:
    from .surface_ir import AnalysisBundle


#: Ключ канонической PhysicalChain: замкнутость плюс рёбра и вершины.
HostChainKey = tuple[bool, tuple[int, ...], tuple[int, ...]]

#: Закон выбора масштаба привязки источника, который заказывает ТОЛЬКО повторная попытка после названного отказа лотереи привязки
#: (`envelope_snap_retry`, `SOURCE_SNAP_PLANE_PRESERVED_RETRY_V1`). Строка равна `GridScaleLawV1.PLANE_PRESERVING_V1.value` ядра (сверка тестом:
#: хост не импортирует ядро на импорте модуля). `None` в экспорте — умолчание ядра («первый масштаб, восстановивший углы»), и тогда
#: ни один ключ кэша не меняется ни в одном байте.
GRID_SCALE_LAW_PLANE_PRESERVING = "PLANE_PRESERVING_V1"

#: Счётчики дополнения выделения. Объявлены перечнем и пишутся ВСЕГДА, даже
#: нулями: иначе «выделение не дополнялось» неотличимо от «дополнение не
#: измерялось» — тот же дефект, который уже лечили счётчиками стадии углов.
SELECTION_COMPLETION_COUNTERS = (
    "SELECTION_COMPLETED_CHAINS",
    "SELECTION_COMPLETED_EDGES",
)

#: Код диагностики сцены топологии о том, что выделение было дополнено.
SELECTION_COMPLETED_DIAGNOSTIC_CODE = "PARTIAL_CHAIN_SELECTION_COMPLETED"


@dataclass(frozen=True, slots=True)
class ChartBandPolicyV1:
    """Политика полосовой карты ЗАПРОСА: досягаемость и выделение (физические рёбра хоста).

    Полоса зависит от выбора цепей, а метрика патча - нет, поэтому выделение едет отдельно от факта топологии: ядро
    строит полосу вокруг цепей домена, у которых есть выбранное ребро, и только если целый патч не развёртывается.

    `alpha` (метры, точная дробь, `None` - неизвестна) нужна ОДНОМУ решению сессии: суженной досягаемости
    (`envelope_chart_band.tightened_export`). Ключ полосы (`band_key_of`) её не содержит: полоса под досягаемостью запроса от
    alpha не зависит, и кэш метрик не пересобирается ползунком. `tightened_reach_cap` и `tightened_after` называют карту,
    которую сессия пересобрала ОДИН раз после отказа (`CHART_REACH_TIGHTENED_FOR_SEAM`); они у экспорта запроса пусты и
    заполняются только копией для этой пересборки (`with_band_tightened`).
    """

    reach_cap: Fraction
    selected_physical_edge_ids: frozenset[int]
    alpha: Fraction | None = None
    tightened_reach_cap: Fraction | None = None
    tightened_after: str = ""


@dataclass(frozen=True, slots=True)
class EnvelopeTopologyExportV1:
    """SourceRevision-scoped host topology prepared exactly once.

    `developable_stretch_budget` — допуск растяжения развёртки ЗАПРОСА (`None`: умолчание ядра). Это политика
    запроса, не факт топологии: она едет здесь только потому, что метрика домена (сертификат развёртки в снапшоте)
    строится из этого экспорта и обязана быть записана под ТЕМ ЖЕ допуском, что и запрос (ядро не компилирует
    иначе). Ключи кэшей метрики и геометрии сессии содержат этот допуск.

    `silhouette_uv_slide` — допуск UV закона силуэта запроса (`None`: умолчание ядра). Метрике он не нужен: едет здесь, чтобы запрос,
    собранный из этого экспорта (родителем и воркером), нёс то же число, что видит материализатор.
    """

    source_revision_value: str
    analysis_bundle: AnalysisBundle
    host_chains: tuple[object, ...]
    patch_domain_id_by_patch: Mapping[int, str]
    developable_stretch_budget: Fraction | None = None
    chart_band: ChartBandPolicyV1 | None = None
    silhouette_uv_slide: Fraction | None = None
    #: Закон выбора масштаба привязки источника метрики (`GRID_SCALE_LAW_PLANE_PRESERVING` либо `None` - умолчание ядра). Метрика домена
    #: (сертификат решётки в снапшоте) строится из этого экспорта, поэтому закон входит в ключи кэшей метрики, геометрии, подготовки и результата
    #: (`metric_law_key`): подготовка, построенная повторной попыткой, никогда не ложится под ключ обычной.
    grid_scale_law: str | None = None
    #: Производные от `host_chains`: цепочки и рёбра по патчу, строятся при первом обращении ОДИН раз на объект. Не часть значения
    #: экспорта (не сравнивается, не печатается); копия через `replace` получает свою пустую запись, и ключ не может пережить свои цепочки.
    _derived: dict = field(default_factory=dict, init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "patch_domain_id_by_patch",
            MappingProxyType(dict(self.patch_domain_id_by_patch)),
        )
        # Явное умолчание (1/5) и «не назван» — один и тот же запрос: форма одна, иначе ключи кэшей метрики
        # и геометрии сессии расщепились бы на два одинаковых.
        if self.developable_stretch_budget == DEFAULT_ENVELOPE_STRETCH_BUDGET:
            object.__setattr__(self, "developable_stretch_budget", None)

    def patch_chains(self, patch_id: int) -> tuple[object, ...]:
        """Цепочки хоста ОДНОГО патча в порядке `host_chains` (пусто, если у патча их нет): индекс вместо прохода по всем цепочкам на патч."""

        groups = self._derived.get("chains")
        if groups is None:
            built: dict = {}
            for record in self.host_chains:
                built.setdefault(record.patch_id, []).append(record)
            groups = self._derived["chains"] = {key: tuple(value) for key, value in built.items()}
        return groups.get(int(patch_id), ())

    def patch_edge_ids(self, patch_id: int) -> frozenset[int]:
        """Физические рёбра цепочек ОДНОГО патча: то, что `band_key_of` пересекает с выделением."""

        found = self._derived.setdefault("edges", {})
        key = int(patch_id)
        own = found.get(key)
        if own is None:
            own = found[key] = frozenset(
                int(edge) for record in self.patch_chains(key) for edge in record.canonical_edge_ids
            )
        return own

    def with_developable_stretch_budget(self, budget: Fraction | None):
        """Тот же экспорт под допуском запроса `budget`; тяжёлые части общие, копии нет."""

        if budget == DEFAULT_ENVELOPE_STRETCH_BUDGET:
            budget = None
        if budget == self.developable_stretch_budget:
            return self
        return replace(self, developable_stretch_budget=budget)

    def with_silhouette_uv_slide(self, slide: Fraction | None):
        """Тот же экспорт под допуском UV запроса `slide` (`None` и умолчание - одно и то же); тяжёлые части общие."""

        from .envelope_request_policy import envelope_silhouette_uv_slide

        slide = envelope_silhouette_uv_slide(slide)
        return self if slide == self.silhouette_uv_slide else replace(self, silhouette_uv_slide=slide)

    def with_grid_scale_retry(self):
        """Тот же экспорт под законом `PLANE_PRESERVING_V1` (повторная попытка после отказа лотереи привязки); тяжёлые части общие."""

        return self if self.grid_scale_law == GRID_SCALE_LAW_PLANE_PRESERVING else replace(self, grid_scale_law=GRID_SCALE_LAW_PLANE_PRESERVING)

    def with_chart_band(self, reach_cap: Fraction | None, selected_physical_edge_ids, alpha: Fraction | None = None):
        """Тот же экспорт с политикой полосы запроса: `reach_cap=None` - умолчание ядра (полметра); `alpha` - метры или `None`."""

        from .envelope_request_policy import DEFAULT_ENVELOPE_CHART_REACH_CAP

        policy = ChartBandPolicyV1(
            DEFAULT_ENVELOPE_CHART_REACH_CAP if reach_cap is None else Fraction(reach_cap),
            frozenset(int(item) for item in selected_physical_edge_ids),
            None if alpha is None else Fraction(alpha),
        )
        return self if policy == self.chart_band else replace(self, chart_band=policy)

    def with_band_tightened(self, tightened_reach_cap: Fraction, refused_outcome: str):
        """Копия для ОДНОЙ пересборки полосы под суженной досягаемостью: запрос остаётся при своей, карта - под `tightened`."""

        if self.chart_band is None:
            raise ValueError("a band is tightened only under a band policy")
        return replace(
            self,
            chart_band=replace(
                self.chart_band, tightened_reach_cap=Fraction(tightened_reach_cap), tightened_after=str(refused_outcome)
            ),
        )

    def without_chart_band(self):
        """Тот же экспорт без полосы: метрика ЦЕЛОГО патча, как она кэшируется независимо от выделения."""

        return self if self.chart_band is None else replace(self, chart_band=None)


def metric_law_key(source) -> tuple:
    """Хвост ключа кэша метрики, геометрии, подготовки и результата: закон выбора масштаба решётки, если он заказан; иначе пусто.

    `source` — экспорт топологии либо запись метрики с полем `grid_scale_law`. Пустой хвост у умолчания — ключи прежних прогонов
    побитово те же; непустой делает метрику и подготовку повторной попытки отдельными записями.
    """

    law = getattr(source, "grid_scale_law", None)
    return () if law is None else (("grid_scale_law", law),)


@dataclass(frozen=True, slots=True)
class _PatchGraphIdView:
    source_revision: object
    nodes: Mapping[int, object]
    edges: Mapping[object, object]

    def __post_init__(self) -> None:
        object.__setattr__(self, "nodes", MappingProxyType(dict(self.nodes)))
        object.__setattr__(self, "edges", MappingProxyType(dict(self.edges)))


@dataclass(frozen=True, slots=True)
class NeighbourFaceV1:
    """Грань патча ВНЕ запроса: номера хоста, обход вершин и их положения (числа и кортежи: воркер пула получает их, как остальной срез).

    Поле `patch_id` называется так нарочно: ключ содержимого домена кодирует номер патча рангом среди соседей по швам, и сдвиг номеров
    не меняет ключа.
    """

    face_id: int
    patch_id: int
    vertex_cycle: tuple[int, ...]
    positions: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True, slots=True)
class _PatchSurfaceIdView:
    source_revision: object
    vertices: tuple[object, ...]
    edges: tuple[object, ...]
    faces: tuple[object, ...]
    triangles: tuple[object, ...]
    #: Грани патчей ВНЕ запроса, касающиеся вершин граней запроса (`NeighbourFaceV1`). Вторая сторона шва: решение по внутренней
    #: вершине цепи (`CHAIN_STATION_PLAN_V1`) обязано видеть поверхность обеих сторон, а срез запроса — только свою.
    neighbour_faces: tuple[NeighbourFaceV1, ...] = ()

    @property
    def vertex_by_id(self) -> dict[int, object]:
        return {int(item.vertex_id): item for item in self.vertices}

    def patch_faces(self, patch_id: int) -> tuple[object, ...]:
        return tuple(
            item for item in self.faces if int(item.patch_id) == int(patch_id)
        )


def _neighbour_face(index, position: int) -> NeighbourFaceV1:
    """Грань кольца по позиции в поверхности; запись неизменяема и общая для всех срезов этой поверхности (память индекса)."""

    found = index.memo.get(position)
    if found is None:
        item = index.faces[position]
        position_of = index.position_of()
        found = NeighbourFaceV1(
            int(item.face_id),
            int(item.patch_id),
            tuple(int(vertex_id) for vertex_id in item.vertex_cycle),
            tuple(tuple(float(axis) for axis in position_of[int(vertex_id)]) for vertex_id in item.vertex_cycle),
        )
        index.memo[position] = found
    return found


@dataclass(frozen=True, slots=True)
class AnalysisBundleIdView:
    """Immutable patch-ID view over a full AnalysisBundle.

    The view stores no copied PatchGraph or PatchSurfaceIR records.  Exporters
    filter the referenced full bundle by ``included_patch_ids``.
    """

    analysis_bundle: AnalysisBundle
    included_patch_ids: frozenset[int]
    _patch_graph: _PatchGraphIdView = field(init=False, repr=False)
    _patch_surface: _PatchSurfaceIdView = field(init=False, repr=False)

    def __post_init__(self) -> None:
        graph = self.analysis_bundle.patch_graph
        surface = self.analysis_bundle.patch_surface
        graph_index = graph_index_of(graph)
        unknown = self.included_patch_ids - graph_index.node_ids
        if not self.included_patch_ids or unknown:
            from .envelope_request_export import (
                EnvelopeDebugHostOutcome,
                EnvelopeHostAdapterError,
            )

            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_ANALYSIS_SNAPSHOT_INVALID,
                "request-scoped PatchDomain set is empty or unknown: "
                f"{sorted(self.included_patch_ids)}",
            )
        included = self.included_patch_ids
        # Срез по ИНДЕКСУ поверхности и графа (`surface_index`), а не фильтром по всей поверхности на каждый домен: содержимое и
        # порядок те же, что давал фильтр (`tests/test_surface_index.py`), цена - домена, а не меша.
        index = surface_index_of(surface)
        nodes = graph_index.nodes_of(graph, included)
        edges = graph_index.edges_of(graph, included)
        faces = tuple(index.faces[position] for position in index.face_positions(included))
        face_ids = frozenset(int(item.face_id) for item in faces)
        edge_ids = frozenset(
            int(edge_id) for face in faces for edge_id in face.edge_cycle
        )
        vertex_ids = frozenset(
            int(vertex_id) for face in faces for vertex_id in face.vertex_cycle
        )
        # Вид вида (лёгкий вход воркера) уже несёт кольцо: его граней среди `surface.faces` нет, и оно переходит как есть.
        neighbours = tuple(index.ring) + tuple(
            _neighbour_face(index, position) for position in index.ring_positions(vertex_ids, included)
        )
        object.__setattr__(
            self,
            "_patch_graph",
            _PatchGraphIdView(self.source_revision, nodes, edges),
        )
        object.__setattr__(
            self,
            "_patch_surface",
            _PatchSurfaceIdView(
                self.source_revision,
                tuple(index.vertices[position] for position in index.vertex_item_positions(vertex_ids)),
                tuple(index.edges[position] for position in index.edge_item_positions(edge_ids)),
                faces,
                tuple(index.triangles[position] for position in index.triangle_item_positions(face_ids)),
                neighbours,
            ),
        )

    @property
    def source_revision(self):
        return self.analysis_bundle.source_revision

    @property
    def capabilities(self):
        return self.analysis_bundle.capabilities

    @property
    def patch_graph(self):
        return self._patch_graph

    @property
    def patch_surface(self):
        return self._patch_surface


def build_analysis_bundle_id_view(
    analysis_bundle: AnalysisBundle,
    included_patch_ids: frozenset[int],
) -> AnalysisBundleIdView:
    return AnalysisBundleIdView(
        analysis_bundle,
        frozenset(int(item) for item in included_patch_ids),
    )


@dataclass(frozen=True, slots=True)
class EnvelopeSelectionScopeV1:
    """Выделение владельца, дополненное до полных PhysicalChain.

    `requested_edge_ids` — то, что стояло выделенным во вьюпорте; `edge_ids` —
    то, чем считает движок. Два поля, а не одно, намеренно: восстановление
    выделения владельца обязано вернуть ИСХОДНОЕ, и одно поле сделало бы
    возврат исходного неотличимым от возврата дополненного.
    """

    requested_edge_ids: frozenset[int]
    edge_ids: frozenset[int]
    chain_keys: frozenset[HostChainKey]
    patch_ids: frozenset[int]
    #: Цепочки, которые были выделены ЧАСТИЧНО и достроены до полных.
    completed_chain_keys: tuple[HostChainKey, ...]

    @property
    def added_edge_ids(self) -> frozenset[int]:
        return self.edge_ids - self.requested_edge_ids

    @property
    def completed(self) -> bool:
        return bool(self.completed_chain_keys)


def _complete_selection_to_whole_chains(
    selected_edges: frozenset[int],
    chain_groups: Mapping[HostChainKey, list],
) -> tuple[frozenset[int], tuple[HostChainKey, ...]]:
    """Дополнить выделение до объединения ПОЛНЫХ задетых цепочек.

    Владелец выделяет рёбра мышью, а не цепочки: на меше, где шов между двумя
    патчами разбит вершинами на четыре ребра, попадание в одно из них — норма,
    а не ошибка. Прежний жёсткий отказ гасил при этом ВЕСЬ билд, и владелец
    видел warning вместо результата.
    """

    completed = set(selected_edges)
    completed_keys = []
    for key in sorted(chain_groups):
        chain_edges = frozenset(key[1])
        overlap = selected_edges & chain_edges
        if not overlap:
            continue
        if overlap != chain_edges:
            completed_keys.append(key)
        completed.update(chain_edges)
    return frozenset(completed), tuple(completed_keys)


def resolve_selection_scope(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    host_chains: tuple[object, ...],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
) -> EnvelopeSelectionScopeV1:
    """Разрешить выделение владельца в полные цепочки и домены."""

    from .envelope_request_export import (
        EnvelopeDebugHostOutcome,
        EnvelopeHostAdapterError,
        _group_host_chains,
        _validate_chain_group_topology,
    )

    if not selected_physical_edge_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "Envelope debug requires at least one selected physical edge",
        )
    requested = frozenset(int(item) for item in selected_physical_edge_ids)
    known_edges = {
        int(item.edge_id) for item in analysis_bundle.patch_surface.edges
    }
    unknown = requested - known_edges
    if unknown:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_UNKNOWN,
            "selected physical edges are absent from AnalysisBundle: "
            f"{sorted(unknown)}",
        )

    chain_groups = _group_host_chains(host_chains)
    selected_edges, completed_keys = _complete_selection_to_whole_chains(
        requested,
        chain_groups,
    )
    if profile is not None:
        profile.set_counter(
            "SELECTION_COMPLETED_CHAINS",
            len(completed_keys),
        )
        profile.set_counter(
            "SELECTION_COMPLETED_EDGES",
            len(selected_edges - requested),
        )
    selected_keys = {
        key
        for key in chain_groups
        if selected_edges & frozenset(key[1])
    }
    covered = {edge_id for key in selected_keys for edge_id in key[1]}
    off_chain = sorted(requested - covered)
    if off_chain:
        # Дополнение здесь НЕВОЗМОЖНО: ребро принадлежит поверхности, но не
        # входит ни в одну граничную цепочку ни одного патча — то есть лежит
        # внутри патча. Достраивать его не до чего.
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_SELECTED_EDGE_OFF_PHYSICAL_CHAIN,
            "selected physical edges belong to no PhysicalChain and cannot be "
            f"completed: {off_chain}",
        )

    selected_patch_ids = frozenset(
        record.patch_id
        for key in selected_keys
        for record in chain_groups[key]
    )
    if not selected_patch_ids:
        raise EnvelopeHostAdapterError(
            EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EMPTY_SELECTION,
            "whole-chain selection resolved to no PatchDomain",
        )

    # Exact-frame admission is request-scoped, but topology is not guessed.
    # Every seam touching a selected domain must still have its real counterpart
    # in the original AnalysisBundle before the opposite unselected domain is
    # omitted from the evaluation slice.
    for records in chain_groups.values():
        if any(record.patch_id in selected_patch_ids for record in records):
            _validate_chain_group_topology(records)
    return EnvelopeSelectionScopeV1(
        requested,
        selected_edges,
        frozenset(selected_keys),
        selected_patch_ids,
        completed_keys,
    )


def build_envelope_topology_export(
    analysis_bundle: AnalysisBundle,
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
) -> EnvelopeTopologyExportV1:
    """Collect and normalize host chains once for one SourceRevision."""

    from .envelope_request_export import (
        _collect_host_chains,
        _revision_value,
        _typed_value,
    )

    revision = _revision_value(analysis_bundle.source_revision)
    host_chains = _collect_host_chains(
        analysis_bundle,
        profile=profile,
    )
    return EnvelopeTopologyExportV1(
        revision,
        analysis_bundle,
        host_chains,
        {
            int(patch_id): _typed_value(
                "patch-domain",
                revision,
                int(patch_id),
            )
            for patch_id in analysis_bundle.patch_graph.nodes
        },
    )


def build_envelope_topology_debug_scene(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export: EnvelopeTopologyExportV1 | None = None,
):
    """Build a request-scoped display scene from cached host topology."""

    from .envelope_request_export import (
        build_envelope_topology_debug_scene as _build_scene,
    )

    export = topology_export or build_envelope_topology_export(
        analysis_bundle,
        profile=profile,
    )
    if export.analysis_bundle.source_revision != analysis_bundle.source_revision:
        raise ValueError("topology export SourceRevision does not match bundle")
    return _build_scene(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=export,
    )


#: Сколько прологов (выделений) держит память. Ширина меняется ползунком при ОДНОМ выделении, и к двум-трём недавним
#: возвращает отмена, поэтому запас мал: запись держит пакет анализа.
STAGE_INPUTS_MEMO_LIMIT = 4
STAGE_INPUTS_HIT = "HIT"
STAGE_INPUTS_MISS = "MISS"


class StageInputsMemoV1:
    """Пролог постадийного прогона (`stage_domain_inputs`) по `(ревизия, выделение)`: от alpha, плотности и допусков не зависит.

    Сцена топологии, перечень доменов, `DecalRequestId` и выделенные рёбра доменов — функция пакета анализа, цепочек хоста
    экспорта и выделения; полоса (`chart_band`), допуски и alpha экспорта в неё не входят (`stage_domain_inputs` читает у
    экспорта только ревизию и `host_chains`). Запись держит сам пакет и цепочки хоста и принимается лишь на ТЕХ ЖЕ
    объектах (`is`): занятое тождество не уходит другому, а пересобранный пакет той же ревизии промахивается и пишет заново.
    Записи профиля пролога (счётчики и квитанции доменов) идут при попадании в профиль ЭТОГО прогона теми же вызовами, что
    у счёта; секунды стадий при попадании не пишутся (их не было). Ключ несёт отпечаток кода ядра и хоста
    (`envelope_content_key.code_identity`). Память живёт и сбрасывается с кэшами ревизии сессии.
    """

    def __init__(self) -> None:
        self._entries: dict[tuple, tuple] = {}
        self.hits = 0
        self.misses = 0
        #: `HIT`, `MISS` либо `OFF` последнего вызова (для счётчика профиля прогона).
        self.last = ""
        #: Выключенная память считает пролог каждый раз (сверка «с памятью и без», замер «до»).
        self.enabled = True

    def __len__(self) -> int:
        return len(self._entries)

    def clear(self) -> None:
        self._entries.clear()

    def stage(self, analysis_bundle, selected, *, profile, topology_export):
        from .envelope_content_key import ContentKeyUnsupported, code_identity

        try:
            code = code_identity() if self.enabled else None
        except ContentKeyUnsupported:
            code = None
        if code is None:
            self.last = "OFF"
            return _stage_domain_inputs(analysis_bundle, selected, profile=profile, topology_export=topology_export)
        key = (code, str(topology_export.source_revision_value), frozenset(int(item) for item in selected))
        entry = self._entries.get(key)
        if entry is not None and entry[0] is analysis_bundle and entry[1] is topology_export.host_chains:
            self._entries[key] = self._entries.pop(key)  # давность
            self.hits += 1
            self.last = STAGE_INPUTS_HIT
            return self._replayed(entry[2], entry[3], profile)
        private = None if profile is None else EnvelopeDebugProfileBuilderV1(profile.source_name, profile.build_kind)
        built = _stage_domain_inputs(analysis_bundle, selected, profile=private, topology_export=topology_export)
        recorded = None if private is None else private.snapshot()
        self._entries.pop(key, None)
        self._entries[key] = (analysis_bundle, topology_export.host_chains, built, recorded)
        while len(self._entries) > STAGE_INPUTS_MEMO_LIMIT:
            del self._entries[next(iter(self._entries))]
        self.misses += 1
        self.last = STAGE_INPUTS_MISS
        return self._replayed(built, recorded, profile, timings=True)

    @staticmethod
    def _replayed(built, recorded, profile, *, timings: bool = False):
        """Пролог с записями профиля, повторёнными в профиль прогона; словарь выделенных рёбер отдаётся копией."""

        if profile is not None and recorded is not None:
            if timings:
                for item in recorded.timings:
                    profile.add_timing(item.stage, item.elapsed_seconds, item.patch_domain_id)
            for item in recorded.counters:
                profile.set_counter(item.name, item.value, item.patch_domain_id)
            for receipt in recorded.receipts:
                profile.set_receipt(receipt)
        scene, revision, patch_ids, request_id, by_domain = built
        return scene, revision, patch_ids, request_id, {name: set(edges) for name, edges in by_domain.items()}


def stage_domain_inputs(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export: EnvelopeTopologyExportV1 | None = None,
    memo: StageInputsMemoV1 | None = None,
):
    """Сцена топологии плюс адресация доменов одного постадийного прогона.

    `memo` (`StageInputsMemoV1`, у сессии) — пролог по `(ревизия, выделение)`: ответ тот же, записи профиля те же.
    Без экспорта (`topology_export=None`) память не используется: у такого вызова нет цепочек хоста, которыми она ключуется.
    """

    if memo is None or topology_export is None:
        return _stage_domain_inputs(
            analysis_bundle, selected_physical_edge_ids, profile=profile, topology_export=topology_export
        )
    return memo.stage(analysis_bundle, selected_physical_edge_ids, profile=profile, topology_export=topology_export)


def _stage_domain_inputs(
    analysis_bundle: AnalysisBundle,
    selected_physical_edge_ids: frozenset[int],
    *,
    profile: EnvelopeDebugProfileBuilderV1 | None = None,
    topology_export: EnvelopeTopologyExportV1 | None = None,
):
    """Сцена топологии плюс адресация доменов одного постадийного прогона.

    Возвращает `(topology_scene, revision, patch_ids, decal_request_id,
    selected_edges_by_domain)`.

    Общая точка ОБОИХ движков (LEGACY и QUEUE) намеренно: одинаковая нумерация
    доменов и один `DecalRequestId` — это и есть то, чем их результаты
    сравнимы. Две разошедшиеся копии этого пролога сделали бы сравнение
    свойством копии, а не свойством движков.
    """

    from .envelope_request_export import (
        _revision_value,
        _typed_value,
        build_envelope_topology_debug_scene as _build_scene,
    )
    from .envelope_topology_debug import EnvelopeTopologyPathKind

    topology_scene = _build_scene(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=topology_export,
    )
    revision = _revision_value(analysis_bundle.source_revision)
    patch_ids = tuple(
        sorted(
            int(patch_id)
            for patch_id in analysis_bundle.patch_graph.nodes
            if _typed_value("patch-domain", revision, int(patch_id))
            in topology_scene.patch_domain_ids
        )
    )
    decal_request_id = _typed_value(
        "decal-request",
        revision,
        tuple(
            sorted(
                path.physical_chain_id
                for path in topology_scene.paths
                if path.kind is EnvelopeTopologyPathKind.SELECTED_SOURCE
                and path.physical_chain_id is not None
            )
        ),
    )
    selected_edges_by_domain: dict[str, set[int]] = {
        domain_id: set()
        for domain_id in topology_scene.patch_domain_ids
    }
    for path in topology_scene.paths:
        if (
            path.kind is EnvelopeTopologyPathKind.DIRECTED_CHAIN_USE
            and path.selected
            and path.patch_domain_id is not None
        ):
            selected_edges_by_domain[path.patch_domain_id].update(
                path.host_edge_ids
            )
    return (
        topology_scene,
        revision,
        patch_ids,
        decal_request_id,
        selected_edges_by_domain,
    )


__all__ = (
    "ChartBandPolicyV1",
    "SELECTION_COMPLETED_DIAGNOSTIC_CODE",
    "SELECTION_COMPLETION_COUNTERS",
    "AnalysisBundleIdView",
    "EnvelopeSelectionScopeV1",
    "EnvelopeTopologyExportV1",
    "HostChainKey",
    "STAGE_INPUTS_HIT",
    "STAGE_INPUTS_MISS",
    "STAGE_INPUTS_MEMO_LIMIT",
    "StageInputsMemoV1",
    "build_analysis_bundle_id_view",
    "build_envelope_topology_debug_scene",
    "build_envelope_topology_export",
    "resolve_selection_scope",
    "stage_domain_inputs",
)
