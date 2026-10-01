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
7. сборка, валидация `validate_geometry_batch`, дайджесты.

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
from dataclasses import dataclass, replace
from hashlib import sha256
from typing import NamedTuple

from ..canonical import geometry_batch_semantic_digest
from ..codec import canonical_json_bytes
from ..contracts.geometry_batch import (
    GEOMETRY_BATCH_SCHEMA_V1,
    GeometryDiagnosticSeverity,
    GeometryDiagnosticV1,
)
from ..contracts.metric import AffineChartOrientationV1, NearPlanarLiftLawV1
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
    Layout,
    assemble_batch,
    intern_vertices,
    station_values,
    tessellate_faces,
)
from .coalesce import FaceMatchV1, MergeStatsV1
from .coalesce import match_region_faces, merge_same_chain_faces, region_contours
from .frames import MaterializationRefusal, resolve_frame
from .lift import plane_lift_of
from .offset_normal import OFFSET_NORMAL_LAW
from .lift_surface import surface_lift_of
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
    #: там одна нормаль плоскости на домен. В дайджест батча не входит.
    vertex_normals: tuple = ()
    offset_normal_law: str = ""

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
):
    """Диагностики батча: near-planar, рестарт `u`, деградировавшие митры."""

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
        onto_surface = lift_law is NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
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
        add(
            GeometryDiagnosticSeverity.INFO,
            NamedOutcome.DEVELOPABLE_LIFT_ONTO_UNFOLDED_SOURCE_TRIANGLES,
            "domain",
            (),
            _developable_numbers(prepared.context.frame.planarity_certificate, lift_note),
        )
    for chain_id in sorted(table.restart_chain_ids):
        add(
            GeometryDiagnosticSeverity.WARNING,
            NamedOutcome.U_RESTARTS_AT_DOMAIN_BORDER,
            chain_id,
            (chain_id,),
            chain_id,
        )
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
    return (
        f"worst_band_squared<={number(stretch.worst_band_squared_upper):.9g} "
        f"stretch_budget={number(stretch.stretch_budget):.6g} "
        f"triangles_measured={stretch.triangles_measured} "
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
        ("MATERIALIZE_TRIANGLES", len(batch.faces)),
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


def _lift_of(prepared, admission, scale, budget):
    """Подъём домена по ДЕЙСТВУЮЩЕМУ закону укладки (`admission.lift_law`)."""

    context = prepared.context
    if admission.lift_law is NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1:
        return surface_lift_of(
            context.frame,
            context.snapshot,
            prepared.compilation.owner_patch_id,
            scale,
        ).bind(budget)
    return plane_lift_of(context.frame, scale)


def _assemble(prepared, coverage, request, admission, budget, clock, parts):
    """Кадры, вершины, станции, тесселяция, батч — по слитым граням домена."""

    items, table, lines, notes = parts
    frame_faces = [
        resolve_frame(table, region_id, face, line, source_keys)
        for region_id, face, line, source_keys in items
    ]
    clock.lap("FRAMES")
    layout = Layout(frame_faces)
    cycles, points = intern_vertices(
        [(item[0], frame) for item, frame in zip(items, frame_faces)], table, notes
    )
    lattice_alpha = coverage.lattice_alpha
    facts = station_values(frame_faces, cycles, layout, table, lattice_alpha, budget)
    clock.lap("STATIONS_UV")
    chart_cw = (
        prepared.context.frame.chart_orientation
        is AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH
    )
    triangles = tessellate_faces(frame_faces, cycles, budget, reverse=chart_cw)
    clock.lap("TESSELLATE")
    plane = _lift_of(prepared, admission, table.scale, budget)
    batch = assemble_batch(
        frame_faces=frame_faces,
        cycles=cycles,
        points=points,
        triangles=triangles,
        facts=facts,
        layout=layout,
        plane=plane,
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
        ),
    )
    batch = replace(
        batch,
        semantic_digest=SemanticDigestValue(
            geometry_batch_semantic_digest(batch).sha256_hex
        ),
    )
    clock.lap("ASSEMBLE")
    return batch, frame_faces, plane.counters(), plane


def _build(prepared, coverage, request, admission, budget, clock) -> _Built:
    table = chain_station_table(prepared, budget)
    spans = source_chain_by_span(prepared)
    clock.lap("STATIONS")
    items, stats, match = _covered_regions(
        prepared, coverage, table, spans, budget, clock
    )
    lines: list[str] = []
    notes: list[str] = []
    try:
        batch, frame_faces, lift_counters, lift = _assemble(
            prepared, coverage, request, admission, budget, clock,
            (items, table, lines, notes),
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
        raise refusal.augmented(extra, (*match.counters(), *table.counters)) from None
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
        lift_counters,
        lift,
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
) -> MaterializationV1:
    """Материализует ОДИН домен очереди. Исход назван, отказ не бросает исключение.

    `request` по умолчанию — запрос самой подготовки; материализатору нужны из
    него только идентичность запроса и два политических идентификатора
    (материал, UV-закон). `work_budget` по умолчанию — свежий бюджет
    транзакции `MATERIALIZE` домена. `near_planar_lift_law` — на что кладётся
    near-planar домен: по умолчанию на сертифицированную плоскость (поведение
    не менялось), `SOURCE_TRIANGLES_V1` — на треугольники источника.
    """

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
        built = _build(prepared, coverage, request, admission, budget, clock)
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
    )
