"""`materialize_domain`: покрытие домена очереди -> `GeometryBatchV1`.

Ключ исполнения — ВЕСЬ домен: все источники и все регионы считаются вместе
(AGENTS.md, п.2 envelope-ядра). Материализации по одной цепи с последующей
сшивкой здесь нет: слияние, станции, вершины и цепи берут грани всего домена.

Стадии (каждая под именем в `timings`, под ОДНИМ бюджетом точной работы
`stage="MATERIALIZE"`):

1. допуск (`admit`): покрытие `EXACT`, плоскость, UV-закон — три отказа до работы;
2. таблица станций (`stations`): `s0` по цепям, пробеги, углы петель;
3. контуры и слияние (`coalesce`): ТОТ ЖЕ код, что у отладочной картинки, но
   с другим ответом на потерю: каждая грань покрытия учтена (`FaceMatchV1`), и
   любая потеря — именованный отказ `COVERAGE_FACE_LOST` (картинке пропавшая
   грань — нерисованный кусок, мешу — дыра, которую не видит ни валидатор, ни
   аудит сетки); площадь слитых граней региона сверяется с площадью его
   покрытия точно;
4. кадры (`frames`): огибающая и система `(s, r)` каждой слитой грани;
5. вершины, факты `(s, r)`, UV (`assemble`, `uv_law`): точно, одно округление;
6. тесселяция (`tessellate`): отсечение ушей, сумма площадей — точное равенство;
   под `QUAD_STRIPS_V1` строго выпуклый четырёхугольник ленты остаётся одной
   гранью (веера, невыпуклые и слитые пробеги — треугольники, и каждый такой
   случай назван счётчиком); под `PLANAR_POLYGONS_V1` слитый пробег режется по
   перекладинам обратно в грани рёбер-источников, а лента на точной плоскости
   остаётся одним многоугольником любой длины, выпуклым или нет, если контур
   прост и UV в нём аффинен по положению на карте (`PLANAR_AFFINE_UV_POLYGON_V1`,
   точно; иначе отсечение ушей под своим счётчиком), веера — треугольники;
7. подъём вершин (по одному разу) и закон `QUAD_IN_ONE_SOURCE_TRIANGLE_V1`
   (`settle_topology`): четырёхгранья, лёгшие на разные треугольники источника
   либо смещаемые по разным нормалям (развёртка), режутся каноническим `fan_out`,
   чтобы не выдать непланарную грань;
8. сборка, валидация `validate_geometry_batch`, дайджесты.

Исход всегда назван (`MaterializationOutcome`). Бюджет кончился — именованный
`EXACT_WORK_BUDGET_EXHAUSTED`, а не зависание; батч не прошёл валидатор —
`BATCH_DID_NOT_VALIDATE` с пересчётом замечаний в `detail`. Отказ несёт числа,
которые стадии успели посчитать (`counters`): отказ без чисел не отличить от
«не дошли».

Каждая материализация считается с ХОЛОДНОЙ памятью канонизации, поэтому статьи
`EXACT_WORK_*` — свойство входа, а не истории процесса.

ВСЁ ПИКЛИТСЯ: результат — замороженные записи из чисел, строк и множеств, а
вход (`prepared`, `coverage`) воркер пула держит у себя, поэтому вызов можно
делать прямо в воркере после покрытия.
"""

from __future__ import annotations

import time
from collections import Counter
from dataclasses import dataclass, replace
from hashlib import sha256
from typing import NamedTuple

from ..canonical import geometry_batch_semantic_digest
from ..codec import canonical_json_bytes
from ..contracts.geometry_batch import (
    GEOMETRY_BATCH_SCHEMA_V1,
    DecalTopologyLawV1,
    GeometryDiagnosticSeverity,
    GeometryDiagnosticV1,
)
from ..contracts.metric import AffineChartOrientationV1, DevelopableBandChartCertificateV1, NearPlanarLiftLawV1
from ..exact_sqrt_sum import (
    ExactCanonicalizationWorkBudgetExhausted,
    SqrtSumV1,
    exact_work_budget,
    reset_factorization_memory,
)
from ..ids import GeometryDiagnosticId, LineageId, SemanticDigestValue
from ..outcomes import NamedOutcome
from ..validation import validate_geometry_batch
from .admit import MaterializationOutcome, PlanarityKind, admit_domain
from .audit import audit_batch
from .assemble import (
    RUNG_STATIONS_FROM_CHAIN_VERTEX,
    Layout,
    assemble_batch,
    canonical_triangles,
    intern_vertices,
    lift_vertices,
    settle_topology,
    station_values,
    tessellate_faces,
)
from .chord_station import station_chord_vertices
from .clip import cut_domain, piece_triangles
from .coalesce import FaceMatchV1, MergeStatsV1
from .coalesce import match_region_faces, merge_same_chain_faces, region_contours
from .frames import MaterializationRefusal, resolve_frame
from .lift import plane_lift_of
from .offset_normal import OFFSET_NORMAL_LAW, offset_normals_digest
from .lift_surface import surface_lift_of
from .source_lift import (
    host_positions_of,
    lift_source_vertices,
    rebind_offset_normals,
    settle_emitted_faces,
    source_step_of,
)
from .stations import chain_station_table, source_chain_by_span
from .uv_law import UV_DIRECT_STRIP_V1

MATERIALIZER_CONTRACT = "cftuv.envelope.materializer.v1"


@dataclass(frozen=True, slots=True)
class MaterializationV1:
    """Исход материализации домена: батч либо именованный отказ, с числами."""

    outcome: MaterializationOutcome
    batch: object | None
    detail: str
    counters: tuple[tuple[str, int], ...]
    timings: tuple[tuple[str, float], ...]
    #: Читаемые строки диагностик батча (сами диагностики лежат в нём).
    diagnostics: tuple[str, ...]
    #: sha256 канонических байтов ПОЛНОГО батча (с триангуляцией). Пусто у отказа.
    content_digest: str
    #: Нормаль смещения КАЖДОЙ вершины батча (`vert_key`, единичная нормаль) у домена
    #: развёртки, по закону `offset_normal_law`; у плоского и near-planar домена пусто:
    #: там одна нормаль плоскости на домен. В дайджест батча не входит, поэтому у них свой
    #: `offset_normals_digest` (побитовый sha256 нормалей): писатель хоста сдвигает вершины
    #: меша ими, и без собственного дайджеста сверка прогонов их не видела бы.
    vertex_normals: tuple = ()
    offset_normal_law: str = ""
    offset_normals_digest: str = ""
    #: Закон топологии, КОТОРЫЙ ПРОСИЛИ (и у отказа тоже): поле результата, а не
    #: диагностики и не `contract_versions` батча — те входят в семантический
    #: дайджест, а он тесселяции не видит.
    decal_topology_law: DecalTopologyLawV1 = DecalTopologyLawV1.TRIANGLES_V1

    @property
    def is_materialized(self) -> bool:
        return self.outcome is MaterializationOutcome.MATERIALIZED


class _Clock:
    def __init__(self) -> None:
        self.marks: list[tuple[str, float]] = []
        self._started = time.perf_counter()

    def lap(self, stage: str) -> None:
        now = time.perf_counter()
        self.marks.append((stage, now - self._started))
        self._started = now


def _refused(outcome, detail, clock, counters=()) -> MaterializationV1:
    return MaterializationV1(
        outcome=outcome,
        batch=None,
        detail=detail,
        counters=tuple(counters),
        timings=tuple(clock.marks),
        diagnostics=(),
        content_digest="",
    )


def _edge_faces(context) -> dict[str, tuple[str, ...]]:
    """`физическое ребро -> исходные грани, его несущие` (по циклам рёбер)."""

    found: dict[str, list[str]] = {}
    for face_id, face in context.source_faces_by_id.items():
        for edge in face.edge_cycle:
            found.setdefault(edge.value, []).append(face_id.value)
    return {edge: tuple(sorted(names)) for edge, names in found.items()}


def _diagnostics(
    prepared,
    table,
    planarity,
    lines,
    lift_law: NearPlanarLiftLawV1 = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1,
    lift_note: str = "",
    gap_note: str = "",
    sourced=None,
    clip_note: str = "",
    chords=None,
    opposition_note: str = "",
):
    """Диагностики батча: near-planar, рестарт `u`, деградировавшие митры, положение вершин `src:`."""

    result = []

    def add(severity, outcome, token, lineage, text):
        result.append(
            GeometryDiagnosticV1(
                GeometryDiagnosticId(f"diagnostic:{outcome.value}:{token}"),
                severity,
                outcome,
                frozenset(LineageId(item) for item in lineage),
            )
        )
        lines.append(f"{outcome.value}: {text}")

    if planarity is PlanarityKind.NEAR_PLANAR:
        certificate = prepared.context.frame.planarity_certificate
        onto_surface = lift_law.onto_surface
        add(
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.NEAR_PLANAR_LIFT_ONTO_SOURCE_TRIANGLES
            if onto_surface
            else NamedOutcome.NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE,
            "domain",
            (),
            _near_planar_numbers(certificate, onto_surface, lift_note),
        )
    if planarity is PlanarityKind.DEVELOPABLE_UNFOLDED:
        _developable_diagnostics(
            prepared.context.frame.planarity_certificate, lift_note, gap_note, opposition_note, add
        )
    if clip_note:
        add(
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.SOURCE_EDGES_LIFTED_ONTO_SURFACE,
            "domain",
            (),
            clip_note,
        )
    if chords is not None and chords.placed:
        add(
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.SOURCE_VERTEX_STATIONED_ON_CHORD_V1,
            "domain",
            (),
            chords.placed_note(),
        )
    if chords is not None and (chords.skipped or chords.not_in_coverage):
        add(
            GeometryDiagnosticSeverity.WARNING,
            NamedOutcome.SOURCE_VERTEX_CHORD_STATION_SKIPPED,
            "domain",
            (),
            chords.skipped_note(),
        )
    if sourced is not None:
        _lift_diagnostics(sourced, add)
    _table_diagnostics(table, add)
    for region in prepared.regions:
        for corner in region.degraded_miter_corners:
            add(
                GeometryDiagnosticSeverity.WARNING,
                NamedOutcome.DEGRADED_MITER_CORNER_IN_GEOMETRY,
                corner.corner_relation_id,
                (corner.corner_relation_id,),
                f"{corner.corner_relation_id}: {corner.reason}",
            )
    return result


def _developable_diagnostics(certificate, lift_note, gap_note, opposition_note, add) -> None:
    """Диагностики домена-развёртки: числа растяжения, зазор смещения, допущенное противостояние нормалей."""

    add(
        GeometryDiagnosticSeverity.INFO,
        NamedOutcome.DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES,
        "domain",
        (),
        _developable_numbers(certificate, lift_note),
    )
    for note, outcome in (
        (gap_note, NamedOutcome.DEVELOPABLE_OFFSET_MIN_GAP_COSINE),
        (opposition_note, NamedOutcome.SURFACE_OFFSET_OPPOSITION_TOLERATED),
    ):
        if note:
            add(GeometryDiagnosticSeverity.INFO, outcome, "domain", (), note)
    if type(certificate) is DevelopableBandChartCertificateV1:
        # Грани патча вне носителя полосы не вошли в карту: названо, а не молча отброшено (числа - в сертификате).
        number = certificate.chart_reach_margin_squared
        add(
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.FACE_BEYOND_CHART_REACH,
            "domain",
            (),
            f"excluded_triangles={certificate.excluded_triangle_count} "
            f"first={certificate.first_excluded_triangle_id.value} "
            f"support_triangles={len(certificate.support_triangle_ids)} "
            f"reach_cap_m={certificate.reach_cap.numerator / certificate.reach_cap.denominator:.6g} "
            f"reach_margin_m={(number.numerator / number.denominator) ** 0.5:.6g}",
        )


def _lift_diagnostics(sourced, add) -> None:
    """Диагностики закона положения вершин `src:`: подвинутые, оставленные, возвращённые, узлы-спутники."""

    steps = (
        (
            sourced.moved,
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1,
            sourced.lifted_note,
        ),
        (
            sourced.displaced,
            GeometryDiagnosticSeverity.WARNING,
            NamedOutcome.SOURCE_VERTEX_DISPLACED_BY_LATTICE,
            sourced.displaced_note,
        ),
        (
            sourced.kept_for_orientation,
            GeometryDiagnosticSeverity.WARNING,
            NamedOutcome.SOURCE_VERTEX_LIFT_REFUSED_BY_FACE_ORIENTATION,
            sourced.orientation_note,
        ),
        (
            sourced.followed,
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.SOURCE_VERTEX_LIFT_NODES_FOLLOWED,
            sourced.followed_note,
        ),
    )
    for count, severity, outcome, note in steps:
        if count:
            add(severity, outcome, "domain", (), note())


def _table_diagnostics(table, add) -> None:
    """Диагностики таблицы станций: рестарт `u` у границы домена и разрез замкнутого потока."""

    for chain_id in sorted(table.restart_chain_ids):
        add(
            GeometryDiagnosticSeverity.WARNING,
            NamedOutcome.U_RESTARTS_AT_DOMAIN_BORDER,
            chain_id,
            (chain_id,),
            chain_id,
        )
    for flow_key, closing, opening, chain_id in table.cuts:
        add(
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.U_RESTARTS_AT_CLOSED_FLOW_OPENING,
            flow_key,
            (chain_id,),
            f"{flow_key}: the closed flow is opened at {opening}; the corner {closing} -> {opening} is the cut",
        )


def _near_planar_numbers(certificate, onto_surface: bool, lift_note: str) -> str:
    """Числа диагностики near-planar.

    На сертифицированной плоскости — прежний текст, побитово. На поверхности
    источника — искажение ширины (то, чем судят) и невязка плоскости (запись,
    а не суд), в читаемых числах; точные величины лежат в сертификате.
    """

    if not onto_surface:
        return (
            f"residual_budget={certificate.residual_budget} "
            f"max_residual_squared={certificate.max_residual_squared}"
        )

    def number(value) -> float:
        return value.numerator / value.denominator

    sigma = certificate.width_distortion
    return (
        f"min_cos_squared={number(sigma.min_cos_squared):.9g} "
        f"width_budget={number(sigma.width_budget):.6g} "
        f"triangles_measured={sigma.triangles_measured} "
        f"max_residual_squared={number(certificate.max_residual_squared):.6g} "
        f"(recorded, not judging; residual_budget="
        f"{number(certificate.residual_budget):.6g}) {lift_note}"
    )


def _developable_numbers(certificate, lift_note: str) -> str:
    """Числа диагностики развёртки: растяжение (то, чем судят), ярлыки, привязка карты."""

    def number(value) -> float:
        return value.numerator / value.denominator

    stretch = certificate.stretch
    classes = {}
    for item in certificate.vertex_classes:
        classes[item.developability_class.value] = (
            classes.get(item.developability_class.value, 0) + 1
        )
    labels = " ".join(f"{name}={classes[name]}" for name in sorted(classes)) or "none"
    hinge = certificate.hinge_chart_worst_band_squared_upper
    rival = certificate.arap_chart_worst_band_squared_upper
    chart = "ARAP" if certificate.proposal_law.value.startswith("ARAP") else "HINGE"
    return (
        f"worst_band_squared<={number(stretch.worst_band_squared_upper):.9g} "
        f"stretch_budget={number(stretch.stretch_budget):.6g} "
        f"triangles_measured={stretch.triangles_measured} "
        f"proposal={chart} proposal_selection={certificate.proposal_selection_law.value} "
        f"hinge_band_squared<={'none' if hinge is None else format(number(hinge), '.9g')} "
        f"arap_band_squared<={'none' if rival is None else format(number(rival), '.9g')} "
        f"arap_refusal={certificate.arap_refusal or 'none'} "
        f"chart_scale={certificate.chart_scale} "
        f"chart_scale_trials={certificate.chart_scale_trials} "
        f"proposal_snapped_vertices={certificate.snapped_vertex_count} "
        f"proposal_snap_residual_cells={number(certificate.snap_residual):.6g} "
        f"interior_vertices[{labels}] "
        f"previous_refusals={list(certificate.previous_refusals)} "
        f"offset_normal_law={OFFSET_NORMAL_LAW} {lift_note}"
    )


def _merged_area_gap(merged, covered) -> bool:
    """Слитые грани региона НЕ дают площади покрытия: ТОЧНОЕ сравнение сумм.

    Независимая от счёта граней проверка той же потери: потерянный кусок с
    площадью сдвинул бы сумму, как бы ни сошёлся счёт, а слияние, потерявшее
    грань, сдвинуло бы её тоже. Ни одного порога: `SqrtSumV1` канонична.
    """

    total = SqrtSumV1.zero()
    for face in merged:
        total = total + face.doubled_area
    return not (total - covered.doubled_area).is_zero


def _covered_regions(prepared, coverage, table, spans, budget, clock):
    """Слитые грани ВСЕХ регионов домена: `(items, stats, match)`.

    НИ ОДНА грань покрытия не пропадает без счёта. Для картинки отладки
    пропавшая грань — нерисованный кусок; для меша — ДЫРА, которую не видит ни
    `validate_geometry_batch`, ни аудит сетки (её полурёбра станут «стеной»).
    Поэтому любая потеря здесь — именованный отказ `COVERAGE_FACE_LOST` с
    причиной в `detail` и числами в `counters`; отказ собирается по ВСЕМ
    регионам разом, чтобы одна потеря не прятала другую.

    Потерей считаются: регион подготовки без записи покрытия или без
    разбиения, запись покрытия, которой в подготовке нет, грань покрытия без
    контура (`FaceMatchV1`) и слияние/сопоставление, после которых площадь
    региона уже не равна площади его покрытия.
    """

    covered_by_region = {item.region_id: item for item in coverage.regions}
    prepared_ids = {region.region_id for region in prepared.regions}
    lattice_alpha = coverage.lattice_alpha
    items = []
    stats = MergeStatsV1()
    match = FaceMatchV1()
    problems = [
        f"COVERAGE_REGION_NOT_PREPARED:{name}"
        for name in sorted(set(covered_by_region) - prepared_ids)
    ]
    for region in prepared.regions:
        covered = covered_by_region.get(region.region_id)
        if covered is None:
            problems.append(f"REGION_WITHOUT_COVERAGE:{region.region_id}")
            continue
        if region.partition is None:
            problems.append(f"REGION_WITHOUT_PARTITION:{region.region_id}")
            continue
        contours = region_contours(region, lattice_alpha, budget)
        clock.lap("CONTOURS")
        plain, region_match = match_region_faces(
            covered, contours, spans.get(region.region_id, {})
        )
        match = match + region_match
        if region_match.lost_total:
            problems.append(f"{region.region_id}:{region_match.describe_losses()}")
            continue
        merged, region_stats = merge_same_chain_faces(plain, budget)
        stats = stats + region_stats
        if _merged_area_gap(merged, covered):
            problems.append(f"{region.region_id}:AREA_DOES_NOT_CLOSE")
            continue
        lines = {face.owner: face.line for face in region.partition.faces}
        source_keys = frozenset(key for key, _name in region.owner_by_edge) | frozenset(
            region.ambiguous_owner_spans
        )
        for face in merged:
            items.append((region.region_id, face, lines[face.owner], source_keys))
    clock.lap("COALESCE")
    if problems:
        shown = "; ".join(problems[:4])
        more = len(problems) - 4
        raise MaterializationRefusal(
            MaterializationOutcome.COVERAGE_FACE_LOST,
            shown + (f"; +{more} more" if more > 0 else ""),
            (*match.counters(), ("MATERIALIZE_DOMAIN_REGIONS", len(prepared.regions))),
        )
    return items, stats, match


class _Built(NamedTuple):
    """Всё, что стадия сборки отдаёт материализатору вместе с батчем."""

    batch: object
    frame_faces: list
    stats: MergeStatsV1
    match: FaceMatchV1
    table: object
    lines: tuple
    region_count: int
    dropped_names: int
    lift_counters: tuple = ()
    lift: object = None
    topology_counters: tuple = ()


def _counters(built: _Built, budget):
    batch, frame_faces = built.batch, built.frame_faces
    return (
        *built.match.counters(),
        ("MATERIALIZE_DOMAIN_REGIONS", built.region_count),
        ("MATERIALIZE_FACES_MERGED", len(frame_faces)),
        ("MATERIALIZE_SEPARATORS_MERGED", built.stats.merged_separators),
        ("MATERIALIZE_MERGE_UNRESOLVED", built.stats.unresolved_groups),
        ("MATERIALIZE_FAN_FACES", sum(1 for item in frame_faces if item.is_fan)),
        ("MATERIALIZE_VERTEX_SOURCE_NAMES_DROPPED", built.dropped_names),
        # Треугольники — СУММА `n - 2` по граням: от выбора диагонали она не
        # зависит, поэтому под любым законом топологии это одно и то же число.
        (
            "MATERIALIZE_TRIANGLES",
            sum(len(face.ordered_vert_keys) - 2 for face in batch.faces),
        ),
        ("MATERIALIZE_FACES_EMITTED", len(batch.faces)),
        (
            "MATERIALIZE_QUADS",
            sum(1 for face in batch.faces if len(face.ordered_vert_keys) == 4),
        ),
        ("MATERIALIZE_VERTICES", len(batch.vertices)),
        ("MATERIALIZE_REGIONS", len(batch.semantic_regions)),
        ("MATERIALIZE_STATION_FACTS", len(batch.station_facts)),
        (
            "MATERIALIZE_STATION_CONSTANT_S",
            sum(
                1
                for fact in batch.station_facts
                if fact.station_model_id.value == "CONSTANT_PHYSICAL_ENDPOINT_S"
            ),
        ),
        ("MATERIALIZE_BOUNDARY_CHAINS", len(batch.boundary_chains)),
        ("MATERIALIZE_INTERFACE_CHAINS", len(batch.interface_chains)),
        *built.table.counters,
        *built.lift_counters,
        *built.topology_counters,
        *budget.counters(),
    )


def _source_normal(prepared) -> tuple[float, float, float]:
    """Нормаль первой исходной грани владельца (по имени грани): меркой для аудита."""

    snapshot = prepared.context.snapshot
    owner = prepared.compilation.owner_patch_id
    faces = sorted(
        (item for item in snapshot.surface_ir.source_faces if item.patch_id == owner),
        key=lambda item: item.face_id.value,
    )
    normal = faces[0].polygon_normal if faces else None
    return (0.0, 0.0, 0.0) if normal is None else (normal.x, normal.y, normal.z)


def _on_exact_plane(admission) -> bool:
    """Укладка домена — ТОЧНАЯ плоскость (аффинный подъём), а не треугольники источника."""

    return not admission.lift_law.onto_surface


def _lift_of(prepared, admission, scale, budget):
    """Подъём домена по ДЕЙСТВУЮЩЕМУ закону укладки (`admission.lift_law`)."""

    context = prepared.context
    if not _on_exact_plane(admission):
        return surface_lift_of(
            context.frame,
            context.snapshot,
            prepared.compilation.owner_patch_id,
            scale,
        ).bind(budget)
    return plane_lift_of(context.frame, scale)


def _tessellation_law(clipped: bool, law):
    """Закон тесселяции: под резкой `PLANAR_POLYGONS_V1` при ЛЮБОМ запрошенном законе.

    На вход резки идёт целый простой многоугольник с аффинной UV (как на плоскости), а плоским в 3D его
    делает резка: каждый кусок лежит в одном треугольнике источника. Запрошенный закон решает потом
    только форму кусков, поэтому вершины `clip:`, цепи и семантический дайджест у законов одни и те же.
    Следствия, которые нельзя вывести из кода: (1) числа закона топологии (`MATERIALIZE_POLYGON_FACES_*`,
    `FAN_*`, `QUADS_REFUSED_NOT_CONVEX`) описывают ТЕССЕЛЯЦИЮ ДО резки и идут по `PLANAR_POLYGONS_V1`
    при любом запрошенном законе; (2) срезанный соседом веер на кривом домене — один многоугольник
    (`exact_plane`), его режет резка, а не разрез от вершины веера (`FAN_FACE_TRIANGULATED_FROM_APEX_V1`
    его не видит); (3) имена подъёма под резкой обнулены (`names`), поэтому счёт `QUADS_SPLIT_*`
    нулевой по построению и доказательством не служит.
    """

    return DecalTopologyLawV1.PLANAR_POLYGONS_V1 if clipped else law


def _is_clipped(admission) -> bool:
    """Укладка домена — треугольники источника С РЕЗКОЙ граней (`SOURCE_TRIANGLES_CLIPPED_V1`, `SOURCE_FACES_CLIPPED_V1`)."""

    return admission.lift_law.clips


def _lifted(plane, points, cut):
    """`(позиции, записи об источнике)` вершин: узлы подъёмом, новые вершины резки — как они подняты.

    Вершины `src:`, привязанные к углу карты перед резкой (`cut.snapped`, `clip_snap`), поднимаются в угле: в домене
    у вершины ОДНА точка карты, и уши выпущенных кусков, положение вершин и сварка с соседом читают её же.
    """

    positions, names = lift_vertices(points if cut is None else {**points, **cut.snapped}, plane)
    if cut is not None:
        positions.update((key, lifted[0]) for key, lifted in cut.lifted.items())
        names.update((key, lifted[1]) for key, lifted in cut.lifted.items())
    return positions, names


def _at_host_positions(prepared, plane, faces, lifted, law, budget, chart_cw):
    """Закон `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1` и проверка выпущенных граней: `(грани, итог, числа граней)`.

    Позиции не вправе зависеть от закона: ориентация сверяется по каноническим треугольникам
    `TRIANGLES_V1` слитых граней (`source_lift`), а не по выпущенным. Под резкой к ним добавлены уши
    ВЫПУЩЕННЫХ кусков: обрезок у вершины на ячейку, которого нет среди канонических, тоже обязан
    остаться гранью, когда вершина `src:` встаёт в позицию хоста.
    """

    frame_faces, cycles, points = faces
    positions, polygons, cut = lifted
    canonical = (
        polygons
        if law is DecalTopologyLawV1.TRIANGLES_V1 and cut is None
        else canonical_triangles(frame_faces, cycles, budget, chart_cw)
    )
    sourced = lift_source_vertices(
        positions,
        [triangle for face in canonical for triangle in face]
        + ([] if cut is None else piece_triangles(cut.polygons, points, budget)),
        host_positions_of(prepared.context.snapshot),
        source_step_of(prepared.context.frame),
    )
    rebind_offset_normals(plane, positions, sourced)
    final, faces_after = settle_emitted_faces(
        polygons if cut is None else cut.polygons, positions, sourced, points, budget, chart_cw
    )
    return final, sourced, faces_after


def _assemble(prepared, coverage, request, admission, budget, clock, parts, law):
    """Кадры, вершины, станции, тесселяция, батч — по слитым граням домена."""

    items, table, lines, notes, chords = parts
    frame_faces = [
        resolve_frame(table, region_id, face, line, source_keys)
        for region_id, face, line, source_keys in items
    ]
    clock.lap("FRAMES")
    layout = Layout(frame_faces)
    cycles, points = intern_vertices(
        [(item[0], frame) for item, frame in zip(items, frame_faces)], table, notes, chords.names
    )
    lattice_alpha = coverage.lattice_alpha
    tally, rungs = Counter(), set()
    facts = station_values(frame_faces, cycles, layout, table, lattice_alpha, budget, tally, rungs)
    clock.lap("STATIONS_UV")
    chart_cw = (
        prepared.context.frame.chart_orientation
        is AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH
    )
    clipped = _is_clipped(admission)
    tessellation_law = _tessellation_law(clipped, law)
    polygons = tessellate_faces(
        frame_faces,
        cycles,
        budget,
        reverse=chart_cw,
        law=tessellation_law,
        exact_plane=_on_exact_plane(admission) or clipped,
        tally=tally,
        uv_values=lambda frame_face, key: facts[(layout.region_of(frame_face), key)],
        lattice_alpha=lattice_alpha,
        is_rung=lambda frame_face, key: (layout.region_of(frame_face), key) in rungs,
    )
    clock.lap("TESSELLATE")
    plane = _lift_of(prepared, admission, table.scale, budget)
    cut = None
    if clipped:
        cut = cut_domain(
            plane,
            budget,
            frame_faces=frame_faces,
            cycles=cycles,
            points=points,
            polygons=polygons,
            facts=facts,
            layout=layout,
            table=table,
            lattice_alpha=lattice_alpha,
            law=law,
            by_faces=admission.lift_law.clips_by_faces,
            tally=tally,
        )
        clock.lap("CLIP")
    positions, names = _lifted(plane, points, cut)
    if cut is not None:
        points = {**points, **cut.snapped, **cut.points}
        # Счёт закона топологии идёт по ТЕССЕЛЯЦИИ (входу резки); режет и называет куски резка, поэтому
        # здесь ни один многоугольник не делится по записи об источнике.
        names = {key: (None, None) for key in names}
    polygons, topology = settle_topology(
        frame_faces, cycles, polygons, names, tessellation_law, tally
    )
    polygons, sourced, faces_after = _at_host_positions(
        prepared, plane, (frame_faces, cycles, points), (positions, polygons, cut), law, budget, chart_cw
    )
    positions = sourced.positions
    batch = assemble_batch(
        frame_faces=frame_faces,
        cycles=cycles if cut is None else cut.cycles,
        vertex_cycles=None if cut is None else cut.vertex_lists,
        positions=positions,
        polygons=polygons,
        facts=facts,
        layout=layout,
        scale=table.scale,
        lattice_alpha=lattice_alpha,
        edge_faces=_edge_faces(prepared.context),
        request=request,
        source_revision=prepared.context.snapshot.source_revision,
        patch_domain_id=prepared.compilation.plan_key.patch_domain_id,
        contract_versions=(
            GEOMETRY_BATCH_SCHEMA_V1,
            MATERIALIZER_CONTRACT,
            f"cftuv.envelope.uv_policy.{UV_DIRECT_STRIP_V1.value}",
        ),
        # Лениво: счётчики продолжений копятся при подъёме вершин, запись — после него.
        diagnostics=lambda: _diagnostics(
            prepared,
            table,
            admission.planarity,
            lines,
            admission.lift_law,
            plane.note(),
            plane.gap_note(),
            sourced,
            "" if cut is None else cut.note,
            chords, plane.opposition_note(),
        ),
    )
    batch = replace(
        batch,
        semantic_digest=SemanticDigestValue(
            geometry_batch_semantic_digest(batch).sha256_hex
        ),
    )
    clock.lap("ASSEMBLE")
    return (
        batch,
        frame_faces,
        (
            *plane.counters(),
            *sourced.counters(),
            *faces_after.counters(),
            *(() if cut is None else cut.counters),
            (RUNG_STATIONS_FROM_CHAIN_VERTEX, tally[RUNG_STATIONS_FROM_CHAIN_VERTEX]),
        ),
        plane,
        topology,
    )


def _build(prepared, coverage, request, admission, budget, clock, law) -> _Built:
    table = chain_station_table(prepared, budget)
    spans = source_chain_by_span(prepared)
    clock.lap("STATIONS")
    items, stats, match = _covered_regions(
        prepared, coverage, table, spans, budget, clock
    )
    items, chords = station_chord_vertices(prepared, items, table)
    clock.lap("CHORD_STATIONS")
    lines: list[str] = []
    notes: list[str] = []
    try:
        batch, frame_faces, lift_counters, lift, topology = _assemble(
            prepared, coverage, request, admission, budget, clock,
            (items, table, lines, notes, chords), law,
        )
    except MaterializationRefusal as refusal:
        # Отказ поздней стадии несёт числа ранних: сколько граней пришло и куда
        # они ушли, что станция пропустила и почему. Для `STATION_CHAIN_UNNAMED`
        # причину пропуска пишем в деталь: сама грань её не знает.
        extra = (
            table.skip_text()
            if refusal.outcome is MaterializationOutcome.STATION_CHAIN_UNNAMED
            else ""
        )
        raise refusal.augmented(
            extra, (*match.counters(), *table.counters, *chords.counters())
        ) from None
    lines.extend(notes)
    return _Built(
        batch,
        frame_faces,
        stats,
        match,
        table,
        tuple(lines),
        len(prepared.regions),
        len(notes),
        (*chords.counters(), *lift_counters),
        lift,
        topology,
    )


def materialize_domain(
    prepared,
    coverage,
    *,
    request=None,
    work_budget=None,
    near_planar_lift_law: NearPlanarLiftLawV1 = (
        NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
    ),
    decal_topology_law: DecalTopologyLawV1 = DecalTopologyLawV1.TRIANGLES_V1,
) -> MaterializationV1:
    """Материализует ОДИН домен очереди. Исход назван, отказ не бросает исключение.

    `request` по умолчанию — запрос самой подготовки; материализатору нужны из
    него только идентичность запроса и два политических идентификатора
    (материал, UV-закон). `work_budget` по умолчанию — свежий бюджет
    транзакции `MATERIALIZE` домена. `near_planar_lift_law` — на что кладётся
    near-planar домен: по умолчанию на сертифицированную плоскость (поведение
    не менялось), `SOURCE_TRIANGLES_V1` — на треугольники источника,
    `SOURCE_TRIANGLES_CLIPPED_V1` — на них же с РЕЗКОЙ граней рёбрами источника (`clip`):
    каждый кусок в одном треугольнике; домен развёртки при этом тоже режется.
    `decal_topology_law` — из каких граней собирается сетка: по умолчанию
    только треугольники (`TRIANGLES_V1`), и закон записан в поле результата.
    """

    result = _materialize_domain(
        prepared, coverage, request, work_budget, near_planar_lift_law, decal_topology_law
    )
    return replace(result, decal_topology_law=decal_topology_law)


def _materialize_domain(
    prepared, coverage, request, work_budget, near_planar_lift_law, law
) -> MaterializationV1:
    clock = _Clock()
    request = request if request is not None else prepared.compilation.decal_request
    admission = admit_domain(prepared, coverage, request, near_planar_lift_law)
    clock.lap("ADMIT")
    if admission.outcome is not None:
        return _refused(admission.outcome, admission.detail, clock)
    domain_id = prepared.compilation.plan_key.patch_domain_id.value
    # КАЖДАЯ материализация считается с холодной памятью канонизации, как и
    # подготовка с покрытием (`run_queue_domain`): попадание в процессную память
    # разложений возвращается до любой оплаты, и статьи `EXACT_WORK_*` зависели
    # бы от того, какие домены процесс видел раньше (в пуле — от раскладки задач
    # по воркерам). Сброс меняет цену, а не ответ; потолок бюджета — авторитет
    # отказа, и он не может быть свойством истории процесса.
    reset_factorization_memory()
    budget = (
        work_budget
        if work_budget is not None
        else exact_work_budget(stage="MATERIALIZE", domain_id=domain_id)
    )
    try:
        built = _build(prepared, coverage, request, admission, budget, clock, law)
    except MaterializationRefusal as refusal:
        return _refused(
            refusal.outcome,
            refusal.detail,
            clock,
            (*refusal.counters, *budget.counters()),
        )
    except ExactCanonicalizationWorkBudgetExhausted as exhausted:
        return _refused(
            MaterializationOutcome.EXACT_WORK_BUDGET_EXHAUSTED,
            str(exhausted),
            clock,
            budget.counters(),
        )
    batch = built.batch
    issues = validate_geometry_batch(batch)
    clock.lap("VALIDATE")
    offset_normals = (
        built.lift.offset_normals(batch.vertices)
        if getattr(built.lift, "has_offset_normals", False)
        else ()
    )
    audit = (
        audit_batch(batch, _source_normal(prepared), dict(offset_normals))
        if offset_normals
        else audit_batch(batch, _source_normal(prepared))
    )
    clock.lap("AUDIT")
    counters = _counters(built, budget) + audit.counters()
    if issues:
        shown = "; ".join(
            f"{item.code.value}@{'/'.join(map(str, item.path))}" for item in issues[:8]
        )
        return _refused(
            MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
            f"{len(issues)} issues: {shown}",
            clock,
            counters,
        )
    if audit.problems():
        return _refused(
            MaterializationOutcome.BATCH_DID_NOT_VALIDATE,
            "AUDIT:" + ",".join(audit.problems()),
            clock,
            counters,
        )
    digest = sha256(canonical_json_bytes(batch)).hexdigest()
    clock.lap("DIGEST")
    return MaterializationV1(
        outcome=MaterializationOutcome.MATERIALIZED,
        batch=batch,
        detail="",
        counters=counters,
        timings=tuple(clock.marks),
        diagnostics=built.lines,
        content_digest=digest,
        vertex_normals=offset_normals,
        offset_normal_law=OFFSET_NORMAL_LAW if offset_normals else "",
        offset_normals_digest=offset_normals_digest(offset_normals),
    )
