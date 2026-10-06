"""Domain extraction and boundary-limited FrontComponent saturation."""

from __future__ import annotations

from dataclasses import dataclass, replace

import sympy as sp

from .._cpython311 import sorted_as_cpython311
from ..contracts.envelopes import StripEnvelopeSpec
from ..numeric import LocalLengthV1
from . import symbolic_backend as _backend
from .boundary_native import SourceContactFrame, compare_contacts, contact_candidates_native
from .native_exact import NativeExactError, NativeSignUndecided
from .symbolic_backend import SymbolicBackendV1
from .common import (
    GeometryContext,
    SourceSupportSegment,
    stable_id,
)
from .alpha_bounds import bounds_of_contacts, sign_against
from .contracts import (
    ComponentEffectiveAlphaV1,
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)
from .planar_types import (
    ConstructionCertificate,
    ConstructionKind,
    ExactPlanarPoint,
    ExactScalar,
    exact_normalize,
    exact_sign,
    point_add,
    point_key,
    point_sub,
    points_equal,
    vector_scale,
)
from .domain_geometry import (
    BlockingBoundarySegment,
    BoundaryRole,
    SparsePatchDomainGeometryV1,
    build_sparse_patch_domain_geometry,
)
from .provenance import (
    merge_provenance,
)

@dataclass(frozen=True, slots=True)
class ComponentResolution:
    front_component_id: str
    requested_alpha: LocalLengthV1
    effective_alpha: ExactScalar
    capacity_outcome: ReferenceOutcome | None
    event_keys: tuple[str, ...]
    diagnostics: tuple[ReferenceEvaluationDiagnosticV1, ...]

    def public(self) -> ComponentEffectiveAlphaV1:
        return ComponentEffectiveAlphaV1(
            front_component_id=self.front_component_id,
            requested_alpha=self.requested_alpha,
            effective_alpha=self.effective_alpha,
            capacity_outcome=self.capacity_outcome,
        )


class ContactCandidatesMemoV1:
    """Контакты `(источник, граница)` ОДНОЙ подготовки очереди: значения, а не кэш процесса.

    `_contact_candidates` — функция только геометрии: опорного отрезка источника и
    отрезка границы домена в метрике контекста. Запрошенная alpha в неё не входит, она
    стоит лишь в сравнениях ПОСЛЕ неё (`resolve_component_alphas`). Поэтому покрытие при
    каждой новой alpha пересчитывало одни и те же точные контакты: на центральном патче
    `building` 1332 вызова стоили 2.0 с из 2.2 с покрытия (клип самой юбки — 45 мс).

    Здесь контакты считаются один раз на подготовку, на первом покрытии, и ездят вместе с
    ней: память НЕ обнуляется при пересылке (в отличие от `_DensityExactMemo`, чьи
    интервалы `mpmath` не пиклятся), потому что воркер, получивший пикл подготовки, иначе
    начинал бы с пустой памяти на каждое нажатие. Значения — те же выражения `sympy` и
    точки, что вернула бы функция, поэтому ответ побитово прежний.

    Ключ несёт геометрию обоих отрезков, а не только имена: отрезок границы и его
    `reversed()` носят одно `segment_id`.
    """

    __slots__ = ("entries", "computed", "reused")

    def __init__(self, entries: dict | None = None) -> None:
        self.entries: dict = {} if entries is None else entries
        # Сколько раз память посчитала и сколько отдала в ЭТОМ процессе: счётчики не
        # едут с пиклом, иначе копия в воркере выдавала бы чужие числа за свои.
        self.computed = 0
        self.reused = 0

    def __reduce__(self):
        return (type(self), (self.entries,))


def _remembered(memo: ContactCandidatesMemoV1 | None, key: tuple, compute):
    if memo is None:
        return compute()
    found = memo.entries.get(key)
    if found is None:
        found = memo.entries[key] = compute()
        memo.computed += 1
    else:
        memo.reused += 1
    return found


def _contact_key(tag, source, boundary):
    segment = boundary.segment
    return (
        tag,
        source.support_id,
        point_key(source.start),
        point_key(source.end),
        segment.segment_id,
        point_key(segment.start),
        point_key(segment.end),
    )


def _contacts_of(context, source, boundary, memo, frame=None):
    """`_contact_candidates` через память подготовки (`None` — как раньше, без неё); `frame` — данные источника (`SourceContactFrame`)."""

    return _remembered(
        memo,
        _contact_key("contacts", source, boundary),
        lambda: _contact_candidates(context, source, boundary, frame),
    )


def _contacts_bounded(context, source, boundary, memo, frame=None):
    """`((alpha, station, point), оболочка alpha)` по контактам пары: оболочки (`alpha_bounds`) едут с подготовкой."""

    contacts = _contacts_of(context, source, boundary, memo, frame)
    bounds = _remembered(
        memo, _contact_key("bounds", source, boundary), lambda: bounds_of_contacts(contacts)
    )
    return zip(contacts, bounds)


def _blocking_of(domain_geometry, memo):
    """`domain_geometry.blocking_segments` через память подготовки: свойство строит кортеж и точные вогнутые углы заново на каждый вызов.

    Граница домена от alpha не зависит, а вызывается она на КАЖДЫЙ опорный интервал каждого покрытия.
    """

    return _remembered(memo, ("blocking", domain_geometry.patch_domain_id), lambda: domain_geometry.blocking_segments)


def _source_length(context, source, memo):
    """Метрическая длина опорного отрезка источника; считается один раз на отрезок."""

    return _remembered(
        memo,
        ("length", source.support_id, point_key(source.start), point_key(source.end)),
        lambda: _length_of(context, point_sub(source.end, source.start)),
    )


def _length_of(context, direction):
    """Длина отрезка в метрике контекста; в `NATIVE_EXACT` — родная, с названной уступкой sympy."""

    if _backend.backend_mode() is SymbolicBackendV1.NATIVE_EXACT:
        try:
            return context.metric.length_g_native(direction)
        except NativeExactError as refusal:
            _backend.count(
                "source_length",
                "sign_undecided" if isinstance(refusal, NativeSignUndecided) else "outside_field",
            )
    return context.metric.length_g(direction)


def build_domain_geometry(
    context: GeometryContext,
) -> SparsePatchDomainGeometryV1:
    """Compatibility facade for the sparse runtime contract."""

    return build_sparse_patch_domain_geometry(context)


def _native_contacts(context, source, boundary, frame) -> tuple:
    """Родные контакты пары: сначала предфильтр «контактов нет» (`SourceContactFrame.excludes`), иначе точный путь.

    Предфильтр может лишь доказать пустоту (строгая сторона прямой у обоих концов отрезка в binary64 с границей ошибки),
    поэтому ответ не меняется; исход назван и посчитан: `prefilter_rejected` (пуст доказанно) и `prefilter_passed`
    (не доказано, считано точно). Без `frame` (прямой вызов) предфильтра нет.
    """

    if frame is None:
        return contact_candidates_native(context, source, boundary)
    if frame.excludes(boundary):
        _backend.count("contact_candidates", "prefilter_rejected")
        return ()
    _backend.count("contact_candidates", "prefilter_passed")
    return contact_candidates_native(context, source, boundary, frame)


def _contact_candidates(
    context: GeometryContext,
    source: SourceSupportSegment,
    boundary: BlockingBoundarySegment,
    frame: SourceContactFrame | None = None,
) -> tuple[tuple[sp.Expr, sp.Expr, ExactPlanarPoint], ...]:
    """Контакты пары под выбранным символьным бэкендом (`symbolic_backend`).

    `SYMPY` — прежний путь. `NATIVE_EXACT` — родной двойник (`boundary_native`), alpha и station в
    нём `RadicalSumV1`; уступка sympy названа и посчитана. `SHADOW` — оба пути, ответ sympy.
    """

    mode = _backend.backend_mode()
    if mode is SymbolicBackendV1.SYMPY:
        return _contact_candidates_sympy(context, source, boundary)
    try:
        native = _native_contacts(context, source, boundary, frame)
    except NativeExactError as refusal:
        _backend.count(
            "contact_candidates",
            "sign_undecided" if isinstance(refusal, NativeSignUndecided) else "outside_field",
        )
        return _contact_candidates_sympy(context, source, boundary)
    if mode is SymbolicBackendV1.NATIVE_EXACT:
        _backend.count("contact_candidates", "native")
        return native
    legacy = _contact_candidates_sympy(context, source, boundary)
    compare_contacts(legacy, native)
    return legacy


def _contact_candidates_sympy(
    context: GeometryContext,
    source: SourceSupportSegment,
    boundary: BlockingBoundarySegment,
) -> tuple[tuple[sp.Expr, sp.Expr, ExactPlanarPoint], ...]:
    barrier = boundary.segment
    barrier_direction = point_sub(barrier.end, barrier.start)
    source_direction = point_sub(source.end, source.start)
    length = context.metric.length_g(source_direction)
    s0 = context.metric.dot_g(
        point_sub(barrier.start, source.start), source.tangent
    )
    ds = context.metric.dot_g(barrier_direction, source.tangent)
    a0 = context.metric.dot_g(
        point_sub(barrier.start, source.start), source.owner_normal
    )
    da = context.metric.dot_g(barrier_direction, source.owner_normal)
    candidates = {sp.Integer(0), sp.Integer(1)}
    if exact_sign(ds) != 0:
        candidates.add(exact_normalize(-s0 / ds))
        candidates.add(exact_normalize((length - s0) / ds))
    if exact_sign(da) != 0:
        candidates.add(exact_normalize(-a0 / da))
    result = []
    for parameter in candidates:
        if exact_sign(parameter) < 0 or exact_sign(parameter - 1) > 0:
            continue
        station = exact_normalize(s0 + parameter * ds)
        alpha = exact_normalize(a0 + parameter * da)
        if exact_sign(station) < 0 or exact_sign(station - length) > 0:
            continue
        if exact_sign(alpha) < 0:
            continue
        point = point_add(barrier.start, vector_scale(barrier_direction, parameter))
        result.append((alpha, station, point))
    result[:] = sorted_as_cpython311(
        result, lambda left, right: exact_sign(left[0] - right[0])
    )
    return tuple(result)


def _continuous_support_intervals(
    context: GeometryContext,
    segments: tuple[SourceSupportSegment, ...]
) -> tuple[SourceSupportSegment, ...]:
    if not segments:
        return ()
    groups = [[segments[0]]]
    for segment in segments[1:]:
        previous = groups[-1][-1]
        continuous = points_equal(previous.end, segment.start)
        collinear = (
            exact_sign(
                context.metric.oriented_cross(
                    previous.tangent, segment.tangent
                )
            )
            == 0
        )
        same_direction = (
            exact_sign(
                context.metric.dot_g(previous.tangent, segment.tangent)
            )
            > 0
        )
        same_normal = (
            exact_sign(
                context.metric.oriented_cross(
                    previous.owner_normal, segment.owner_normal
                )
            )
            == 0
            and exact_sign(
                context.metric.dot_g(
                    previous.owner_normal, segment.owner_normal
                )
            )
            > 0
        )
        if continuous and collinear and same_direction and same_normal:
            groups[-1].append(segment)
        else:
            groups.append([segment])
    result = []
    for group in groups:
        first = group[0]
        last = group[-1]
        result.append(
            replace(
                first,
                source_vertex_end_id=last.source_vertex_end_id,
                end=last.end,
                support_id=stable_id(
                    "continuous-front-support",
                    first.front_component_id,
                    *(item.support_id for item in group),
                ),
                provenance=merge_provenance(*(item.provenance for item in group)),
            )
        )
    return tuple(result)


def _sign_traced(alpha, bounds, requested, trace) -> int:
    """`sign_against`, записывающий оболочку контакта в `trace` (если он есть): знак против запрошенной alpha - единственное, чем ход резолвера зависит от alpha."""

    if trace is not None:
        trace.append(bounds)
    return sign_against(alpha, bounds, requested)


# `trace` (список) - запись для сертификата шага ширины (`materialize.step`): оболочка alpha КАЖДОГО контакта, чей знак против запрошенной
# alpha решал ход (`None` - оболочка не посчитана). От запрошенной alpha ход зависит ТОЛЬКО этими знаками, поэтому, пока ни один из них не
# сменился, исход (эффективные alpha и имена экземпляров) тот же. Ответа запись не меняет.
def resolve_component_alphas(
    context: GeometryContext,
    requested_alpha: LocalLengthV1,
    domain_geometry: SparsePatchDomainGeometryV1,
    contact_memo: ContactCandidatesMemoV1 | None = None, trace: list | None = None,
) -> tuple[dict[str, ComponentResolution], tuple[ReferenceEvaluationDiagnosticV1, ...]]:
    requested = sp.Rational(str(requested_alpha.value))
    resolutions = {
        item.front_component_id.value: ComponentResolution(
            item.front_component_id.value,
            requested_alpha,
            ExactScalar.from_value(requested),
            None,
            (),
            (),
        )
        for item in context.compilation.front_components
    }
    all_diagnostics = []
    for spec in sorted(
        (
            item
            for item in context.compilation.envelope_specs
            if isinstance(item, StripEnvelopeSpec)
        ),
        key=lambda item: item.envelope_spec_id.value,
    ):
        seed = next(
            item
            for item in context.compilation.seeds
            if getattr(item, "seed_id", None) == spec.source_seed_id
        )
        for source in _continuous_support_intervals(
            context,
            context.support_segments_for_use(
                seed.chain_use_id, spec.envelope_spec_id.value
            )
        ):
            best_split = best_bypass = None
            endpoint_events = []
            frame = SourceContactFrame(context, source)
            for boundary in _blocking_of(domain_geometry, contact_memo):
                if source.physical_edge_id.value in boundary.segment.provenance.physical_edge_ids:
                    continue
                for (alpha, station, point), bounds in _contacts_bounded(
                    context, source, boundary, contact_memo, frame
                ):
                    if sign_against(alpha, bounds, sp.Integer(0)) == 0:
                        continue
                    if _sign_traced(alpha, bounds, requested, trace) > 0:
                        continue
                    source_length = _source_length(context, source, contact_memo)
                    interior = exact_sign(station) > 0 and exact_sign(station - source_length) < 0
                    split = interior and (
                        boundary.role in (BoundaryRole.HOLE, BoundaryRole.EXPLICIT_BARRIER)
                        or point_key(point) in boundary.concave_vertex_keys
                    )
                    event_key = stable_id(
                        "boundary-contact",
                        source.front_component_id,
                        source.support_id,
                        boundary.segment.segment_id,
                        ExactScalar.from_value(alpha).expression,
                    )
                    construction = ConstructionCertificate(
                        kind=ConstructionKind.EVENT_ANCHOR,
                        support_ids=frozenset({source.support_id}),
                        boundary_constraint_ids=boundary.segment.boundary_constraint_ids,
                        physical_edge_ids=frozenset(
                            boundary.segment.provenance.physical_edge_ids
                        ),
                        event_key=event_key,
                    )
                    if split:
                        candidate = (alpha, event_key, construction)
                        if best_split is None or exact_sign(alpha - best_split[0]) < 0:
                            best_split = candidate
                    elif (
                        boundary.role is BoundaryRole.EXPLICIT_BARRIER
                        and not interior
                        and sign_against(alpha, bounds, requested) < 0
                    ):
                        candidate = (alpha, event_key, construction)
                        if (
                            best_bypass is None
                            or exact_sign(alpha - best_bypass[0]) < 0
                        ):
                            best_bypass = candidate
                    else:
                        endpoint_events.append((alpha, event_key, construction, interior))
            diagnostics = []
            if best_bypass is not None and (
                best_split is None
                or exact_sign(best_bypass[0] - best_split[0]) < 0
            ):
                alpha, event_key, construction = best_bypass
                diagnostic = ReferenceEvaluationDiagnosticV1(
                    outcome=ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED,
                    severity=ReferenceDiagnosticSeverity.CAPACITY,
                    message=(
                        "continuation beyond an explicit barrier endpoint "
                        "would require unsupported obstacle bypass"
                    ),
                    envelope_spec_id=spec.envelope_spec_id.value,
                    front_component_id=source.front_component_id,
                    requested_alpha=requested_alpha,
                    effective_alpha=ExactScalar.from_value(alpha),
                    construction=construction,
                )
                diagnostics.append(diagnostic)
                current = resolutions[source.front_component_id]
                if exact_sign(alpha - current.effective_alpha.as_expr()) <= 0:
                    resolutions[source.front_component_id] = ComponentResolution(
                        source.front_component_id,
                        requested_alpha,
                        ExactScalar.from_value(alpha),
                        ReferenceOutcome.BARRIER_BYPASS_UNSUPPORTED,
                        tuple(sorted(set((*current.event_keys, event_key)))),
                        tuple((*current.diagnostics, diagnostic)),
                    )
            elif best_split is not None:
                alpha, event_key, construction = best_split
                diagnostic = ReferenceEvaluationDiagnosticV1(
                    outcome=ReferenceOutcome.BARRIER_SPLIT_REQUIRED,
                    severity=ReferenceDiagnosticSeverity.CAPACITY,
                    message="interior boundary contact would increase FrontComponent branch count",
                    envelope_spec_id=spec.envelope_spec_id.value,
                    front_component_id=source.front_component_id,
                    requested_alpha=requested_alpha,
                    effective_alpha=ExactScalar.from_value(alpha),
                    construction=construction,
                )
                diagnostics.append(diagnostic)
                current = resolutions[source.front_component_id]
                if exact_sign(alpha - current.effective_alpha.as_expr()) <= 0:
                    resolutions[source.front_component_id] = ComponentResolution(
                        source.front_component_id,
                        requested_alpha,
                        ExactScalar.from_value(alpha),
                        ReferenceOutcome.BARRIER_SPLIT_REQUIRED,
                        tuple(sorted(set((*current.event_keys, event_key)))),
                        tuple((*current.diagnostics, diagnostic)),
                    )
            elif endpoint_events:
                ordered_events = sorted_as_cpython311(
                    endpoint_events,
                    lambda left, right: exact_sign(left[0] - right[0]),
                )
                minimum_alpha = ordered_events[0][0]
                simultaneous = [
                    item
                    for item in ordered_events
                    if exact_sign(item[0] - minimum_alpha) == 0
                ]
                for alpha, event_key, construction, interior in simultaneous:
                    diagnostics.append(
                        ReferenceEvaluationDiagnosticV1(
                            outcome=ReferenceOutcome.EXACT,
                            severity=ReferenceDiagnosticSeverity.INFO,
                            message=(
                                "boundary contact clips or shrinks one active interval"
                                if interior
                                else "endpoint contact may slide on the same boundary component"
                            ),
                            envelope_spec_id=spec.envelope_spec_id.value,
                            front_component_id=source.front_component_id,
                            requested_alpha=requested_alpha,
                            effective_alpha=ExactScalar.from_value(requested),
                            construction=construction,
                        )
                    )
                current = resolutions[source.front_component_id]
                resolutions[source.front_component_id] = ComponentResolution(
                    current.front_component_id,
                    current.requested_alpha,
                    current.effective_alpha,
                    current.capacity_outcome,
                    tuple(
                        sorted(
                            set(
                                (
                                    *current.event_keys,
                                    *(item[1] for item in simultaneous),
                                )
                            )
                        )
                    ),
                    tuple((*current.diagnostics, *diagnostics)),
                )
            all_diagnostics.extend(diagnostics)
    return resolutions, tuple(all_diagnostics)
