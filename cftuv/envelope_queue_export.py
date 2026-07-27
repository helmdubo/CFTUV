"""Движок QUEUE в Envelope-debug: подготовка и покрытие очереди для хоста.

Зачем отдельный модуль. Эталонный путь (`evaluate_reference_raw_coverage` +
`resolve_coverage_interactions`) остаётся нетронутым: движок LEGACY обязан
вести себя ровно так же, как вёл, пока владелец сам не переключит движок.
Поэтому вход очереди живёт своим файлом, а не веткой внутри эталонного
вычислителя — иначе «не изменилось» пришлось бы доказывать чтением диффа, а не
отсутствием диффа.

Что здесь есть:

1. отображение контрактов ядра (`ConveyorPreparationV1`, `ConveyorCoverageV1`)
   в неизменяемые записи хоста, пригодные для GP-проекции и sidecar;
2. палитра владельцев: `envelope_spec_id` -> слот 0..7, детерминированно по
   сортировке имён; переполнение палитры считается, а не замалчивается;
3. постадийный вычислитель домена, зовущий `prepare_conveyor` и
   `conveyor_coverage` под именованными стадиями профиля.

Чего здесь нет: геометрии. Модуль отображает контракты и не чинит их.

ПОЧЕМУ КОНТУР БЕРЁТСЯ УСЕЧЁННЫЙ. `ConveyorFaceCoverageV1` несёт площадь куска
и имена владельца, но не его контур. Полная грань разбиения
(`partition.faces[*].points`) лежит в тех же координатах решётки, и нарисовать
можно было бы её — но тогда картинка показывала бы БОЛЬШЕ, чем измеренное
число рядом с ней, и расхождение было бы тихим. Поэтому контур берётся у
`coverage_at` — того же вызова, которым покрытие и считалось, — и нарисованная
область равна измеренной по построению. Цена названа стадией `QUEUE_CONTOUR`.
"""

from __future__ import annotations

import time
from contextlib import nullcontext
from dataclasses import dataclass, replace
from fractions import Fraction


def _measure(profile, stage: str, patch_domain_id: str | None = None):
    """Стадия профиля, если профиль есть. Свой, а не заимствованный.

    Тот же трёхстрочный помощник живёт в `envelope_request_export`, но импорт
    оттуда тянет `model.py` и через него `mathutils` — то есть Blender — в
    лёгкий путь ползунка, которому ни то, ни другое не нужно.
    """

    if profile is None:
        return nullcontext()
    return profile.measure(stage, patch_domain_id)


# Слои очереди. Номера 90..92, а НЕ 50..52: числа 50, 51, 52 уже заняты
# эталонным путём (`ENV_50_INTERACTION_COMPONENTS`, `ENV_51_FRONT_READINGS`,
# `ENV_52_EQUALITY_LOCI`), и одинаковый префикс у слоёв разных движков читался
# бы как одна группа. Порядок номера задаёт только высоту подъёма над
# поверхностью, поэтому свободный диапазон ничего не стоит.
QUEUE_SKELETON_LAYER = "ENV_90_QUEUE_SKELETON"
QUEUE_WALL_LAYER = "ENV_91_QUEUE_WALLS"
QUEUE_OWNER_LAYER_PREFIX = "ENV_92_QUEUE_OWNER_"
QUEUE_OWNER_SLOT_COUNT = 8
QUEUE_OWNER_LAYERS = tuple(
    f"{QUEUE_OWNER_LAYER_PREFIX}{slot:02d}"
    for slot in range(QUEUE_OWNER_SLOT_COUNT)
)

# Восемь цветов, различимых на тёмном фоне вьюпорта и между собой. Слот
# владельца — индекс в этом кортеже, поэтому порядок здесь и есть палитра.
QUEUE_OWNER_COLORS = (
    (1.00, 0.25, 0.25, 1.0),
    (1.00, 0.60, 0.10, 1.0),
    (1.00, 0.95, 0.15, 1.0),
    (0.30, 1.00, 0.30, 1.0),
    (0.10, 0.90, 0.95, 1.0),
    (0.30, 0.50, 1.00, 1.0),
    (0.85, 0.35, 1.00, 1.0),
    (1.00, 1.00, 1.00, 1.0),
)

QUEUE_LAYER_STYLES = {
    QUEUE_SKELETON_LAYER: ((0.55, 0.55, 0.60, 1.0), 3),
    QUEUE_WALL_LAYER: ((0.45, 0.45, 0.45, 1.0), 5),
    **{
        name: (QUEUE_OWNER_COLORS[slot], 7)
        for slot, name in enumerate(QUEUE_OWNER_LAYERS)
    },
}

QUEUE_VISIBILITY_PROPERTY = "envelope_debug_show_queue"

ENVELOPE_DEBUG_ENGINE_LEGACY = "LEGACY"
ENVELOPE_DEBUG_ENGINE_QUEUE = "QUEUE"

# Счётчик переполнения палитры. Девятый владелец получает слот первого, и это
# должно быть видно числом: молчание здесь означало бы две разные огибающие
# одного цвета без единого следа о том, что так вышло.
QUEUE_OWNER_PALETTE_WRAPPED = "QUEUE_OWNER_PALETTE_WRAPPED"

#: Разрядность целочисленной оболочки при переводе `SqrtSumV1` в число.
#: Читать величину по частям правило проекта запрещает; оболочка — объявленный
#: способ, и её середина отличается от истинного значения не больше чем на
#: половину ширины, то есть на 2^-64 в единицах решётки.
QUEUE_ENCLOSURE_BITS = 64


def sqrt_sum_float(value, *, bits: int = QUEUE_ENCLOSURE_BITS) -> float:
    """Число из `SqrtSumV1` — серединой строгой оболочки, а не по членам."""

    low, high = value.enclosure(bits)
    return float((low + high) / 2)


@dataclass(frozen=True, slots=True)
class EnvelopeQueueFaceV1:
    """Кусок покрытия: владелец обоими именами и его контур в метрах карты."""

    region_id: str
    owner: tuple[int, int, int, int]
    envelope_spec_id: str
    #: Имя экземпляра юбки при ЭТОЙ alpha. `None` — вывести не удалось, и
    #: причина лежит в исходе домена, а не подменена пустой строкой.
    envelope_instance_id: str | None
    points: tuple[tuple[float, float], ...]
    doubled_area: float
    doubled_area_text: str


@dataclass(frozen=True, slots=True)
class EnvelopeQueueSegmentV1:
    """Отрезок служебного слоя очереди: дуга скелета либо стена."""

    layer: str
    region_id: str
    label: str
    points: tuple[tuple[float, float], ...]


@dataclass(frozen=True, slots=True)
class EnvelopeQueueRegionV1:
    """Исходы одного региона домена, каждый своим типом и своим именем."""

    region_id: str
    bridge_outcome: str
    skeleton_outcome: str | None
    face_outcome: str | None
    coverage_outcome: str | None
    findings: tuple[str, ...]
    wall_edge_count: int
    ambiguous_owner_span_count: int


@dataclass(frozen=True, slots=True)
class EnvelopeQueueDomainV1:
    """Домен, пройденный очередью: исходы, числа, геометрия для проекции."""

    patch_id: int
    patch_domain_id: str
    preparation_outcome: str
    coverage_outcome: str
    detail: str
    lattice_scale: int
    alpha: str
    lattice_alpha: str
    law_names: tuple[str, ...]
    regions: tuple[EnvelopeQueueRegionV1, ...]
    faces: tuple[EnvelopeQueueFaceV1, ...]
    segments: tuple[EnvelopeQueueSegmentV1, ...]
    counters: tuple[tuple[str, float], ...]
    timings: tuple[tuple[str, float], ...]
    prepare_seconds: float
    coverage_seconds: float
    contour_seconds: float
    #: Сама `ConveyorPreparationV1`. Лежит рядом с записью, а не только в кэше
    #: сессии: лёгкий путь ползунка считает покрытие именно по ней, и искать
    #: её по ключу кэша означало бы второй способ её найти.
    preparation: object | None = None

    @property
    def is_exact(self) -> bool:
        return (
            self.preparation_outcome == "EXACT"
            and self.coverage_outcome == "EXACT"
        )


@dataclass(frozen=True, slots=True)
class EnvelopeQueueSceneV1:
    """Все домены очереди одного прогона плюс общая палитра владельцев."""

    domains: tuple[EnvelopeQueueDomainV1, ...]
    #: `envelope_spec_id` -> слот палитры. Сортировка имён, индекс по модулю
    #: восьми: цвет владельца не зависит ни от alpha, ни от порядка доменов.
    palette: tuple[tuple[str, int], ...]
    palette_wrapped: int

    def slot_of(self, envelope_spec_id: str) -> int:
        return dict(self.palette).get(str(envelope_spec_id), 0)

    def layer_of(self, envelope_spec_id: str) -> str:
        return QUEUE_OWNER_LAYERS[self.slot_of(envelope_spec_id)]

    @property
    def prepare_seconds(self) -> float:
        return sum(item.prepare_seconds for item in self.domains)

    @property
    def coverage_seconds(self) -> float:
        return sum(
            item.coverage_seconds + item.contour_seconds
            for item in self.domains
        )


def build_queue_palette(
    envelope_spec_ids,
) -> tuple[tuple[tuple[str, int], ...], int]:
    """Слот владельца по имени спеки: сортировка, индекс по модулю восьми.

    Ключ — `envelope_spec_id`, а не `envelope_instance_id`: имя экземпляра
    стоит на ЭФФЕКТИВНОЙ alpha и меняется при перетаскивании ползунка, то есть
    цвет владельца прыгал бы вместе с ним. Спека же — сама идентичность
    источника, и от alpha она не двигается.
    """

    unique = tuple(sorted({str(item) for item in envelope_spec_ids if item}))
    palette = tuple(
        (name, index % QUEUE_OWNER_SLOT_COUNT)
        for index, name in enumerate(unique)
    )
    wrapped = max(0, len(unique) - QUEUE_OWNER_SLOT_COUNT)
    return palette, wrapped


def build_queue_scene(domains) -> EnvelopeQueueSceneV1:
    domains = tuple(domains)
    palette, wrapped = build_queue_palette(
        face.envelope_spec_id
        for domain in domains
        for face in domain.faces
    )
    return EnvelopeQueueSceneV1(domains, palette, wrapped)


def _lattice_scale(prepared) -> int:
    for region in prepared.regions:
        scale = region.bridge.lattice_scale
        if scale:
            return int(scale)
    lattice = prepared.lattice
    return int(lattice.scale) if lattice is not None else 1


def _region_findings(region) -> tuple[str, ...]:
    """Все именованные наблюдения моста региона, а не только его исход.

    `findings` перечисляет ВСЕ применимые исходы, `outcome` — первый по
    объявленному порядку. Печатать один исход означало бы потерять остальные.
    """

    report = region.bridge
    findings = [item.value for item in report.findings]
    if report.unmatched_laws:
        findings.append(
            f"UNMATCHED_LAWS={len(report.unmatched_laws)}"
        )
    if report.surplus_laws:
        findings.append(f"SURPLUS_LAWS={len(report.surplus_laws)}")
    if report.non_unit_speed_laws:
        findings.append(
            f"NON_UNIT_SPEED_LAWS={len(report.non_unit_speed_laws)}"
        )
    if report.off_lattice_points:
        findings.append(
            f"OFF_LATTICE_POINTS={len(report.off_lattice_points)}"
        )
    if region.ambiguous_owner_spans:
        findings.append(
            f"AMBIGUOUS_OWNER_SPANS={len(region.ambiguous_owner_spans)}"
        )
    if report.undetermined_source_count:
        findings.append(
            f"UNDETERMINED_SOURCE_EDGES={report.undetermined_source_count}"
        )
    if report.snap_residual is not None and report.snap_residual != 0:
        findings.append(f"SNAP_RESIDUAL={report.snap_residual}")
    return tuple(findings)


def _skeleton_segments(region, scale: int) -> list[EnvelopeQueueSegmentV1]:
    """Дуги скелета региона: контур каждой грани без её опорного ребра.

    Порядок точек грани объявлен ядром: `(начало ребра, конец ребра, узлы...)`.
    Значит отрезок `points[0] -> points[1]` — само ребро домена, а вся
    остальная цепочка — дуги скелета. Обход `points[1] -> ... -> points[0]`
    берёт ровно их и ничего сверх.

    Дуга принадлежит двум соседним граням, поэтому отрезки дедуплицируются по
    ТОЧНЫМ координатам (`SqrtSumV1` каноничен и хешируем): иначе каждая дуга
    рисовалась бы дважды и число штрихов перестало бы быть измерением.
    """

    partition = region.partition
    if partition is None:
        return []
    seen: set[tuple] = set()
    segments: list[EnvelopeQueueSegmentV1] = []
    for face in partition.faces:
        chain = tuple(face.points[1:]) + (face.points[0],)
        for index in range(len(chain) - 1):
            start, end = chain[index], chain[index + 1]
            key = tuple(
                sorted(
                    (
                        (start[0].terms, start[1].terms),
                        (end[0].terms, end[1].terms),
                    )
                )
            )
            if key in seen:
                continue
            seen.add(key)
            segments.append(
                EnvelopeQueueSegmentV1(
                    QUEUE_SKELETON_LAYER,
                    region.region_id,
                    "SKELETON_ARC",
                    (
                        (
                            sqrt_sum_float(start[0]) / scale,
                            sqrt_sum_float(start[1]) / scale,
                        ),
                        (
                            sqrt_sum_float(end[0]) / scale,
                            sqrt_sum_float(end[1]) / scale,
                        ),
                    ),
                )
            )
    return segments


def _wall_segments(region, scale: int) -> list[EnvelopeQueueSegmentV1]:
    """Стены региона: рёбра домена без закона прихода, нейтральным цветом."""

    return [
        EnvelopeQueueSegmentV1(
            QUEUE_WALL_LAYER,
            region.region_id,
            "WALL_SPAN",
            (
                (span[0] / scale, span[1] / scale),
                (span[2] / scale, span[3] / scale),
            ),
        )
        for span in region.wall_spans
    ]


def _face_area_text(value) -> str:
    rational = value.as_rational()
    if rational is not None:
        return str(rational)
    return " + ".join(
        f"{coefficient}*sqrt({radicand})"
        for radicand, coefficient in value.terms
    )


def _covered_contours(region, lattice_alpha: Fraction):
    """Усечённые по времени контуры граней региона, в порядке разбиения.

    Тот же вызов, которым `conveyor_coverage` считал площадь, поэтому порядок
    и владельцы совпадают с `ConveyorRegionCoverageV1.faces` по построению, а
    не по совпадению чисел.
    """

    from cftuv_envelope.wavefront.coverage import coverage_at

    if region.partition is None:
        return ()
    return coverage_at(region.partition, lattice_alpha).faces


def build_queue_domain(
    patch_id: int,
    patch_domain_id: str,
    prepared,
    coverage,
    *,
    prepare_seconds: float,
    coverage_seconds: float,
) -> EnvelopeQueueDomainV1:
    """Домен очереди -> неизменяемая запись хоста. Ни одного тихого исхода."""

    scale = _lattice_scale(prepared)
    coverage_by_region = {
        item.region_id: item for item in coverage.regions
    }
    regions: list[EnvelopeQueueRegionV1] = []
    faces: list[EnvelopeQueueFaceV1] = []
    segments: list[EnvelopeQueueSegmentV1] = []
    contour_started = time.perf_counter()
    for region in prepared.regions:
        covered = coverage_by_region.get(region.region_id)
        regions.append(
            EnvelopeQueueRegionV1(
                region_id=region.region_id,
                bridge_outcome=region.bridge_outcome.value,
                skeleton_outcome=(
                    None
                    if region.skeleton_outcome is None
                    else region.skeleton_outcome.value
                ),
                face_outcome=(
                    None
                    if region.face_outcome is None
                    else region.face_outcome.value
                ),
                coverage_outcome=(
                    None if covered is None else covered.outcome.value
                ),
                findings=_region_findings(region),
                wall_edge_count=int(region.wall_edge_count),
                ambiguous_owner_span_count=len(region.ambiguous_owner_spans),
            )
        )
        segments.extend(_skeleton_segments(region, scale))
        segments.extend(_wall_segments(region, scale))
        if covered is None:
            continue
        contours = _covered_contours(region, coverage.lattice_alpha)
        for index, named in enumerate(covered.faces):
            if index >= len(contours):
                break
            contour = contours[index]
            if contour.owner != named.owner or len(contour.points) < 3:
                continue
            faces.append(
                EnvelopeQueueFaceV1(
                    region_id=named.region_id,
                    owner=tuple(int(item) for item in named.owner),
                    envelope_spec_id=str(named.envelope_spec_id),
                    envelope_instance_id=named.envelope_instance_id,
                    points=tuple(
                        (
                            sqrt_sum_float(point[0]) / scale,
                            sqrt_sum_float(point[1]) / scale,
                        )
                        for point in contour.points
                    ),
                    doubled_area=sqrt_sum_float(named.doubled_area),
                    doubled_area_text=_face_area_text(named.doubled_area),
                )
            )
    contour_seconds = time.perf_counter() - contour_started
    return EnvelopeQueueDomainV1(
        patch_id=int(patch_id),
        patch_domain_id=str(patch_domain_id),
        preparation_outcome=prepared.outcome.value,
        coverage_outcome=coverage.outcome.value,
        detail=coverage.detail or prepared.detail,
        lattice_scale=scale,
        alpha=str(coverage.alpha),
        lattice_alpha=str(coverage.lattice_alpha),
        law_names=tuple(prepared.law_names),
        regions=tuple(regions),
        faces=tuple(faces),
        segments=tuple(segments),
        counters=tuple(prepared.counters) + tuple(coverage.counters),
        timings=tuple(prepared.timings) + tuple(coverage.timings),
        prepare_seconds=float(prepare_seconds),
        coverage_seconds=float(coverage_seconds),
        contour_seconds=contour_seconds,
    )


def refused_queue_domain(
    patch_id: int,
    patch_domain_id: str,
    prepared,
    coverage,
    *,
    prepare_seconds: float,
    coverage_seconds: float,
) -> EnvelopeQueueDomainV1:
    """Отказ — тоже результат: числа подготовки сохраняются, геометрии нет."""

    return EnvelopeQueueDomainV1(
        patch_id=int(patch_id),
        patch_domain_id=str(patch_domain_id),
        preparation_outcome=prepared.outcome.value,
        coverage_outcome=(
            "" if coverage is None else coverage.outcome.value
        ),
        # Отказавшая ПОДГОТОВКА объясняет себя подробнее покрытия: покрытие в
        # этом случае отвечает `PREPARATION_IS_NOT_EXACT` и пересказывает лишь
        # ИМЯ её исхода, а имя спеки и опоры, на которых закон оказался
        # нерациональным, лежит только в детали подготовки. Взять деталь
        # покрытия значило бы потерять её молча.
        detail=(
            prepared.detail
            if coverage is None or prepared.outcome.value != "EXACT"
            else (coverage.detail or prepared.detail)
        ),
        lattice_scale=_lattice_scale(prepared),
        alpha="" if coverage is None else str(coverage.alpha),
        lattice_alpha=(
            "" if coverage is None else str(coverage.lattice_alpha)
        ),
        law_names=tuple(prepared.law_names),
        regions=tuple(
            EnvelopeQueueRegionV1(
                region_id=region.region_id,
                bridge_outcome=region.bridge_outcome.value,
                skeleton_outcome=(
                    None
                    if region.skeleton_outcome is None
                    else region.skeleton_outcome.value
                ),
                face_outcome=(
                    None
                    if region.face_outcome is None
                    else region.face_outcome.value
                ),
                coverage_outcome=None,
                findings=_region_findings(region),
                wall_edge_count=int(region.wall_edge_count),
                ambiguous_owner_span_count=len(region.ambiguous_owner_spans),
            )
            for region in prepared.regions
        ),
        faces=(),
        segments=(),
        counters=tuple(prepared.counters)
        + (() if coverage is None else tuple(coverage.counters)),
        timings=tuple(prepared.timings)
        + (() if coverage is None else tuple(coverage.timings)),
        prepare_seconds=float(prepare_seconds),
        coverage_seconds=float(coverage_seconds),
        contour_seconds=0.0,
    )


def queue_domain_payload(domain: EnvelopeQueueDomainV1) -> dict:
    """Sidecar-представление домена очереди. Исход каждой ступени — по имени."""

    return {
        "patch_id": domain.patch_id,
        "patch_domain_id": domain.patch_domain_id,
        "preparation_outcome": domain.preparation_outcome,
        "coverage_outcome": domain.coverage_outcome,
        "detail": domain.detail,
        "lattice_scale": domain.lattice_scale,
        "alpha": domain.alpha,
        "lattice_alpha": domain.lattice_alpha,
        "law_names": list(domain.law_names),
        "prepare_seconds": domain.prepare_seconds,
        "coverage_seconds": domain.coverage_seconds,
        "contour_seconds": domain.contour_seconds,
        "counters": [
            {"name": name, "value": value}
            for name, value in domain.counters
        ],
        "timings": [
            {"stage": stage, "elapsed_seconds": value}
            for stage, value in domain.timings
        ],
        "regions": [
            {
                "region_id": region.region_id,
                "bridge_outcome": region.bridge_outcome,
                "skeleton_outcome": region.skeleton_outcome,
                "face_outcome": region.face_outcome,
                "coverage_outcome": region.coverage_outcome,
                "findings": list(region.findings),
                "wall_edge_count": region.wall_edge_count,
                "ambiguous_owner_span_count": (
                    region.ambiguous_owner_span_count
                ),
            }
            for region in domain.regions
        ],
    }


def queue_scene_payload(scene: EnvelopeQueueSceneV1) -> dict:
    """Соответствие слот -> цвет -> spec_id -> instance_id и все домены."""

    instance_by_spec: dict[str, str] = {}
    for domain in scene.domains:
        for face in domain.faces:
            if face.envelope_instance_id:
                instance_by_spec.setdefault(
                    face.envelope_spec_id,
                    face.envelope_instance_id,
                )
    return {
        "engine": ENVELOPE_DEBUG_ENGINE_QUEUE,
        "palette_wrapped": scene.palette_wrapped,
        "owners": [
            {
                "envelope_spec_id": spec_id,
                "envelope_instance_id": instance_by_spec.get(spec_id),
                "palette_slot": slot,
                "layer": QUEUE_OWNER_LAYERS[slot],
                "color": list(QUEUE_OWNER_COLORS[slot]),
            }
            for spec_id, slot in scene.palette
        ],
        "domains": [
            queue_domain_payload(domain) for domain in scene.domains
        ],
    }


def _queue_diagnostics(domain: EnvelopeQueueDomainV1):
    """Каждый неудавшийся исход и каждое наблюдение моста — своей строкой.

    Тихих исходов нет: `SKELETON_DID_NOT_CLOSE` без сообщения был бы виден
    только в JSON, а исход, который надо искать в JSON, почти неотличим от
    несостоявшегося.
    """

    from .envelope_request_export import (
        EnvelopeDebugHostDiagnosticV1,
        EnvelopeDebugHostSeverity,
    )

    rows = []
    if domain.preparation_outcome != "EXACT":
        rows.append(
            EnvelopeDebugHostDiagnosticV1(
                domain.preparation_outcome,
                EnvelopeDebugHostSeverity.ERROR,
                f"QUEUE prepare: {domain.detail}",
                domain.patch_domain_id,
            )
        )
    elif domain.coverage_outcome not in ("EXACT", ""):
        rows.append(
            EnvelopeDebugHostDiagnosticV1(
                domain.coverage_outcome,
                EnvelopeDebugHostSeverity.ERROR,
                f"QUEUE coverage: {domain.detail}",
                domain.patch_domain_id,
            )
        )
    for region in domain.regions:
        for finding in region.findings:
            if finding == "EXACT":
                continue
            rows.append(
                EnvelopeDebugHostDiagnosticV1(
                    finding,
                    EnvelopeDebugHostSeverity.UNSUPPORTED,
                    f"QUEUE bridge finding on {region.region_id}",
                    domain.patch_domain_id,
                )
            )
    return tuple(rows)


def _queue_stage(domain: EnvelopeQueueDomainV1):
    from .envelope_debug_profile import EnvelopeDomainStage

    if domain.preparation_outcome != "EXACT":
        return EnvelopeDomainStage.QUEUE_PREPARE_REJECTED
    if domain.coverage_outcome != "EXACT":
        return EnvelopeDomainStage.QUEUE_COVERAGE_REJECTED
    return EnvelopeDomainStage.QUEUE_RESOLVED


def _queue_receipt(domain: EnvelopeQueueDomainV1):
    from .envelope_debug_profile import (
        EnvelopeDomainStage,
        EnvelopeDomainStageReceiptV1,
    )

    stage = _queue_stage(domain)
    if stage is EnvelopeDomainStage.QUEUE_RESOLVED:
        outcome = "EXACT"
        message = (
            f"QUEUE coverage: {len(domain.faces)} owned faces, "
            f"prepare {domain.prepare_seconds * 1000.0:.1f} ms, "
            f"coverage {domain.coverage_seconds * 1000.0:.1f} ms"
        )
    elif stage is EnvelopeDomainStage.QUEUE_PREPARE_REJECTED:
        outcome = domain.preparation_outcome
        message = domain.detail or domain.preparation_outcome
    else:
        outcome = domain.coverage_outcome
        message = domain.detail or domain.coverage_outcome
    return EnvelopeDomainStageReceiptV1(
        domain.patch_id,
        domain.patch_domain_id,
        stage,
        outcome,
        message,
    )


def run_queue_domain(
    patch_id: int,
    patch_domain_id: str,
    snapshot,
    request,
    alpha_text: str,
    *,
    selected_edges: frozenset[int] = frozenset(),
    profile=None,
    preparation_provider=None,
):
    """Две ступени очереди на одном домене, каждая под своей стадией профиля.

    Возвращает `(ConveyorPreparationV1, EnvelopeQueueDomainV1)`: подготовка
    нужна вызывающему и дальше — её кэширует сессия и по ней же считается
    покрытие при смене alpha.
    """

    from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor

    started = time.perf_counter()
    with _measure(profile, "QUEUE_PREPARE", patch_domain_id):
        if preparation_provider is None:
            prepared = prepare_conveyor(snapshot, request)
        else:
            prepared = preparation_provider(
                patch_id,
                patch_domain_id,
                frozenset(selected_edges),
                snapshot,
                request,
            )
    prepare_seconds = time.perf_counter() - started

    started = time.perf_counter()
    with _measure(profile, "QUEUE_COVERAGE", patch_domain_id):
        coverage = conveyor_coverage(prepared, alpha_text)
    coverage_seconds = time.perf_counter() - started

    builder = (
        build_queue_domain
        if coverage.outcome.value == "EXACT"
        else refused_queue_domain
    )
    return prepared, replace(
        builder(
            patch_id,
            patch_domain_id,
            prepared,
            coverage,
            prepare_seconds=prepare_seconds,
            coverage_seconds=coverage_seconds,
        ),
        preparation=prepared,
    )


def recompute_queue_coverage(
    entries,
    alpha_text: str,
    *,
    profile=None,
) -> EnvelopeQueueSceneV1:
    """Лёгкий путь ползунка: только покрытие на ГОТОВЫХ подготовках.

    `entries` — `(patch_id, patch_domain_id, prepared)`. Ни `prepare_conveyor`,
    ни компиляция плана здесь не вызываются: подготовка alpha-независима, и это
    измерено ядром побитовым совпадением скелета при 0.25 и 0.5.
    """

    from cftuv_envelope.wavefront import conveyor_coverage

    domains = []
    for patch_id, patch_domain_id, prepared in entries:
        started = time.perf_counter()
        with _measure(profile, "QUEUE_COVERAGE", patch_domain_id):
            coverage = conveyor_coverage(prepared, alpha_text)
        coverage_seconds = time.perf_counter() - started
        builder = (
            build_queue_domain
            if coverage.outcome.value == "EXACT"
            else refused_queue_domain
        )
        domains.append(
            replace(
                builder(
                    patch_id,
                    patch_domain_id,
                    prepared,
                    coverage,
                    prepare_seconds=0.0,
                    coverage_seconds=coverage_seconds,
                ),
                preparation=prepared,
            )
        )
    return build_queue_scene(domains)


def _queue_snapshot_and_request(
    analysis_bundle,
    patch_id: int,
    patch_domain_id: str,
    selected_edges: frozenset[int],
    alpha,
    request_id: str,
    *,
    profile=None,
    topology_export=None,
    domain_snapshot_provider=None,
):
    from .envelope_request_export import (
        build_envelope_analysis_snapshot,
        build_envelope_decal_request,
    )

    with _measure(profile, "SNAPSHOT_EXPORT", patch_domain_id):
        if domain_snapshot_provider is None:
            snapshot = build_envelope_analysis_snapshot(
                analysis_bundle,
                included_patch_ids=frozenset({patch_id}),
                profile=profile,
                topology_export=topology_export,
            )
        else:
            snapshot = domain_snapshot_provider(patch_id, patch_domain_id)
        request = build_envelope_decal_request(
            snapshot,
            selected_edges,
            alpha,
            decal_request_id_value=request_id,
        )
    return snapshot, request


def _queue_debug_scene(
    kernel,
    profile,
    domain_id,
    snapshot,
    request,
    prepared_compilation,
    diagnostics,
):
    """Сцена ядра из ОДНОГО плана: очередь не даёт ни союза, ни разрешения.

    Нужна ради кадра патча (`patch_frames`): проекция слоёв очереди поднимает
    точки карты в 3D тем же кадром, что и эталонный путь, а не своим.
    """

    from .envelope_request_export import (
        EnvelopeDebugHostDiagnosticV1,
        EnvelopeDebugHostOutcome,
        EnvelopeDebugHostSeverity,
        _scene_diagnostics,
    )

    with _measure(profile, "DEBUG_SCENE", domain_id):
        try:
            return kernel.build_envelope_debug_scene(
                snapshot,
                request,
                (prepared_compilation,) if prepared_compilation else (),
                (),
                (),
                _scene_diagnostics(kernel, diagnostics),
            ), None
        except (KeyError, TypeError, ValueError) as exc:
            return None, EnvelopeDebugHostDiagnosticV1(
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_PIPELINE_STAGE_FAILED,
                EnvelopeDebugHostSeverity.ERROR,
                f"DebugScene projection failed without fallback: {exc}",
                domain_id,
            )


def evaluate_envelope_queue_staged(
    analysis_bundle,
    selected_physical_edge_ids: frozenset[int],
    alpha: float,
    *,
    profile=None,
    topology_export=None,
    domain_snapshot_provider=None,
    preparation_provider=None,
):
    """Движок QUEUE: подготовка плюс покрытие с владельцами, без союза.

    Тот же пролог, что и у эталонного движка (`stage_domain_inputs`), поэтому
    домены нумеруются одинаково и колонки двух движков сравнимы. После пролога
    общего кода нет вовсе: `evaluate_reference_raw_coverage` и
    `resolve_coverage_interactions` здесь не вызываются ни разу.
    """

    from .envelope_debug_profile import (
        EnvelopeDebugProfileBuilderV1,
        EnvelopeDomainStage,
    )
    from .envelope_request_export import (
        EnvelopeDebugDomainEvaluationV1,
        EnvelopeDebugStagedEvaluationV1,
        EnvelopeHostAdapterError,
        _receipt_for_failure,
        _typed_value,
        _load_kernel,
    )
    from .envelope_topology_export import stage_domain_inputs

    if profile is None:
        profile = EnvelopeDebugProfileBuilderV1(
            analysis_bundle.source_revision.source_name,
            ENVELOPE_DEBUG_ENGINE_QUEUE,
        )
    (
        topology_scene,
        revision,
        patch_ids,
        request_id,
        selected_edges_by_domain,
    ) = stage_domain_inputs(
        analysis_bundle,
        selected_physical_edge_ids,
        profile=profile,
        topology_export=topology_export,
    )
    alpha_text = str(float(alpha))
    domain_evaluations = []
    kernel = None
    kernel_failure = None
    try:
        kernel, _ = _load_kernel()
    except EnvelopeHostAdapterError as exc:
        kernel_failure = exc.diagnostic()

    for patch_id in patch_ids:
        domain_id = _typed_value("patch-domain", revision, patch_id)
        if kernel_failure is not None:
            receipt = _receipt_for_failure(
                patch_id,
                domain_id,
                EnvelopeDomainStage.QUEUE_PREPARE_REJECTED,
                kernel_failure,
            )
            profile.set_receipt(receipt)
            domain_evaluations.append(
                EnvelopeDebugDomainEvaluationV1(
                    patch_id,
                    domain_id,
                    None,
                    None,
                    None,
                    None,
                    None,
                    None,
                    receipt,
                    (kernel_failure,),
                )
            )
            continue
        domain_evaluations.append(
            _queue_domain_evaluation(
                kernel,
                analysis_bundle,
                patch_id,
                domain_id,
                frozenset(selected_edges_by_domain[domain_id]),
                alpha,
                alpha_text,
                request_id,
                profile=profile,
                topology_export=topology_export,
                domain_snapshot_provider=domain_snapshot_provider,
                preparation_provider=preparation_provider,
            )
        )
    return EnvelopeDebugStagedEvaluationV1(
        topology_scene,
        tuple(domain_evaluations),
    )


def _queue_domain_evaluation(
    kernel,
    analysis_bundle,
    patch_id: int,
    domain_id: str,
    selected_edges: frozenset[int],
    alpha,
    alpha_text: str,
    request_id: str,
    *,
    profile=None,
    topology_export=None,
    domain_snapshot_provider=None,
    preparation_provider=None,
):
    from .envelope_debug_profile import EnvelopeDomainStage
    from .envelope_request_export import (
        EnvelopeDebugDomainEvaluationV1,
        EnvelopeDebugHostOutcome,
        EnvelopeHostAdapterError,
        _receipt_for_failure,
    )

    try:
        snapshot, request = _queue_snapshot_and_request(
            analysis_bundle,
            patch_id,
            domain_id,
            selected_edges,
            alpha,
            request_id,
            profile=profile,
            topology_export=topology_export,
            domain_snapshot_provider=domain_snapshot_provider,
        )
    except EnvelopeHostAdapterError as exc:
        diagnostic = exc.diagnostic()
        stage = (
            EnvelopeDomainStage.METRIC_REJECTED
            if exc.outcome
            in {
                EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE,
                EnvelopeDebugHostOutcome.RUNTIME_NEAR_PLANAR_PROJECTION_POLICY_REQUIRED,
            }
            else EnvelopeDomainStage.QUEUE_PREPARE_REJECTED
        )
        receipt = _receipt_for_failure(patch_id, domain_id, stage, diagnostic)
        profile.set_receipt(receipt)
        return EnvelopeDebugDomainEvaluationV1(
            patch_id,
            domain_id,
            None,
            None,
            None,
            None,
            None,
            None,
            receipt,
            (diagnostic,),
        )

    prepared, queue_domain = run_queue_domain(
        patch_id,
        domain_id,
        snapshot,
        request,
        alpha_text,
        selected_edges=selected_edges,
        profile=profile,
        preparation_provider=preparation_provider,
    )
    diagnostics = list(_queue_diagnostics(queue_domain))
    receipt = _queue_receipt(queue_domain)
    profile.set_receipt(receipt)
    profile.add_timing("QUEUE_CONTOUR", queue_domain.contour_seconds, domain_id)
    for name, value in queue_domain.counters:
        profile.set_counter(name, value, domain_id)
    compilation = prepared.compilation
    debug_scene, scene_failure = _queue_debug_scene(
        kernel,
        profile,
        domain_id,
        snapshot,
        request,
        compilation,
        diagnostics,
    )
    if scene_failure is not None:
        diagnostics.append(scene_failure)
    return EnvelopeDebugDomainEvaluationV1(
        patch_id,
        domain_id,
        snapshot,
        request,
        compilation,
        None,
        None,
        debug_scene,
        receipt,
        tuple(diagnostics),
        queue_domain,
    )


def queue_timing_text(scene: EnvelopeQueueSceneV1) -> str:
    """Строка владельцу: цена очереди суммарно и у самого дорогого домена.

    Суммы мало: на полевом меше один домен стоит впятеро больше остальных, и по
    одной сумме этого не видно. Полные постадийные числа по каждому домену
    печатает консольный профиль (`QUEUE_PREPARE` / `QUEUE_COVERAGE`).
    """

    text = (
        f"QUEUE {len(scene.domains)} domains: "
        f"prepare {scene.prepare_seconds * 1000.0:.0f} ms, "
        f"coverage {scene.coverage_seconds * 1000.0:.0f} ms"
    )
    if not scene.domains:
        return text
    slowest = max(
        scene.domains,
        key=lambda item: item.prepare_seconds
        + item.coverage_seconds
        + item.contour_seconds,
    )
    total = (
        slowest.prepare_seconds
        + slowest.coverage_seconds
        + slowest.contour_seconds
    )
    return (
        f"{text} | slowest {slowest.patch_domain_id[-3:]} "
        f"{total * 1000.0:.0f} ms"
    )


__all__ = (
    "ENVELOPE_DEBUG_ENGINE_LEGACY",
    "ENVELOPE_DEBUG_ENGINE_QUEUE",
    "EnvelopeQueueDomainV1",
    "EnvelopeQueueFaceV1",
    "EnvelopeQueueRegionV1",
    "EnvelopeQueueSceneV1",
    "EnvelopeQueueSegmentV1",
    "QUEUE_ENCLOSURE_BITS",
    "QUEUE_LAYER_STYLES",
    "QUEUE_OWNER_COLORS",
    "QUEUE_OWNER_LAYERS",
    "QUEUE_OWNER_LAYER_PREFIX",
    "QUEUE_OWNER_PALETTE_WRAPPED",
    "QUEUE_OWNER_SLOT_COUNT",
    "QUEUE_SKELETON_LAYER",
    "QUEUE_VISIBILITY_PROPERTY",
    "QUEUE_WALL_LAYER",
    "build_queue_domain",
    "build_queue_palette",
    "build_queue_scene",
    "evaluate_envelope_queue_staged",
    "queue_domain_payload",
    "queue_scene_payload",
    "queue_timing_text",
    "recompute_queue_coverage",
    "refused_queue_domain",
    "run_queue_domain",
    "sqrt_sum_float",
)
