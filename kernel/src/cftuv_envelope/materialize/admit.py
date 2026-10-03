"""Допуск домена к материализации: именованные отказы ДО любой работы.

Материализатор не чинит и не угадывает вход. Он принимает домен, у которого

1. покрытие и подготовка доказаны (`EXACT`) — иначе `COVERAGE_IS_NOT_EXACT`;
2. запрос и покрытие принадлежат ЭТОЙ подготовке: ключ исполнения батча
   `(DecalRequestId, PatchDomainId)` берётся у подготовки, а переданный запрос
   обязан с ней совпасть везде, кроме законов выхода (`OUTPUT_POLICY_FIELDS`) —
   иначе `REQUEST_DOES_NOT_MATCH_PREPARATION`;
3. метрика — точный аффинный дескриптор плоскости (`RationalAffinePlanarMetricV2`
   с сертификатом точной плоскости, near-planar проекции либо РАЗВЁРТКИ). Кривой
   домен без годного сертификата сюда не доходит (его отвергает метрика при
   подготовке), а вход с иным дескриптором — `DOMAIN_IS_NOT_PLANAR_ADMITTED`;
4. UV-закон запроса — из `SUPPORTED_UV_POLICIES`, иначе `UV_POLICY_UNSUPPORTED`.

NEAR_PLANAR допущен (решение 2026-10-01): меш строится на СЕРТИФИЦИРОВАННОЙ
плоскости, а не на исходных вершинах, и этот факт именован диагностикой
`NEAR_PLANAR_LIFT_ON_CERTIFIED_PLANE`; смещение над поверхностью — политика
хоста.

DEVELOPABLE допущен (S1): карта домена — развёртка, у неё нет плоскости источника,
поэтому укладка ЕЩЁ ОДНА и единственная — на треугольники источника (запрошенный
закон роли не играет, как у точной плоскости), а сертификат растяжения судится
здесь заново, до единицы работы. Направление смещения над поверхностью у такого
домена — нормаль вершины (`offset_normal`), не нормаль плоскости.
"""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from enum import Enum
from fractions import Fraction

from ..contracts.metric import (
    DevelopableUnfoldCertificateV1,
    ExactSourcePlaneCertificateV1,
    NearPlanarLiftLawV1,
    NearPlanarProjectionCertificateV1,
    RationalAffinePlanarMetricV2,
)
from .._stretch import stretch_refusal_text, stretch_violations
from .._width_distortion import (
    width_distortion_refusal_text,
    width_distortion_violations,
)
from ..ids import PolicyId
from .uv_law import SUPPORTED_UV_POLICIES


class MaterializationOutcome(str, Enum):
    """Чем кончилась материализация. Тихого пустого результата нет."""

    MATERIALIZED = "MATERIALIZED"
    COVERAGE_IS_NOT_EXACT = "COVERAGE_IS_NOT_EXACT"
    REQUEST_DOES_NOT_MATCH_PREPARATION = "REQUEST_DOES_NOT_MATCH_PREPARATION"
    DOMAIN_IS_NOT_PLANAR_ADMITTED = "DOMAIN_IS_NOT_PLANAR_ADMITTED"
    UV_POLICY_UNSUPPORTED = "UV_POLICY_UNSUPPORTED"
    STATION_CHAIN_UNNAMED = "STATION_CHAIN_UNNAMED"
    STATION_FRAME_IS_AMBIGUOUS = "STATION_FRAME_IS_AMBIGUOUS"
    TESSELLATION_DID_NOT_CLOSE = "TESSELLATION_DID_NOT_CLOSE"
    COVERAGE_FACE_LOST = "COVERAGE_FACE_LOST"
    BATCH_DID_NOT_VALIDATE = "BATCH_DID_NOT_VALIDATE"
    EXACT_WORK_BUDGET_EXHAUSTED = "EXACT_WORK_BUDGET_EXHAUSTED"
    # Укладка на треугольники источника запрошена, а у near-planar домена нет
    # сертификата искажения ширины: класть не на что, и считать ширину не из чего.
    SURFACE_LIFT_UNAVAILABLE = "SURFACE_LIFT_UNAVAILABLE"
    # Имена отказов допуска укладки совпадают с именами исходов ядра и хоста:
    # одна причина называется одним словом на всех уровнях.
    NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED = (
        "NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED"
    )
    NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE = "NEAR_PLANAR_OWNER_TRIANGLE_DEGENERATE"
    NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED = "NEAR_PLANAR_SOURCE_TRIANGLE_FOLDED"
    NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED = "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED"
    # Точка меша лежит вне проекции ВСЕЙ триангуляции источника: ни один
    # замкнутый треугольник её не накрывает. Не «ближайший треугольник» и не
    # допуск — именованный отказ с числами.
    SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION = (
        "SURFACE_LIFT_POINT_OUTSIDE_PROJECTED_TRIANGULATION"
    )
    # Привязка карты к решётке, на которой посчитано покрытие, ПЕРЕВЕРНУЛА
    # проекцию треугольника источника (знак площади сменился): триангуляция
    # решётки перестала быть вложением, и укладывать на неё нельзя.
    SURFACE_LIFT_CHART_SNAP_FLIPPED_TRIANGLE = (
        "SURFACE_LIFT_CHART_SNAP_FLIPPED_TRIANGLE"
    )
    # Привязка к решётке создала пересечение, касание или схлопывание рёбер
    # границы: граница перестала быть простой.
    SURFACE_LIFT_CHART_SNAP_BOUNDARY_NOT_SIMPLE = (
        "SURFACE_LIFT_CHART_SNAP_BOUNDARY_NOT_SIMPLE"
    )
    # Сертификат развёртки не проходит судью растяжения: домен принят под другим
    # судьёй либо запись подделана. Имена совпадают с исходами метрики.
    DEVELOPABLE_STRETCH_BUDGET_EXCEEDED = "DEVELOPABLE_STRETCH_BUDGET_EXCEEDED"
    DEVELOPABLE_CHART_TRIANGLE_FLIPPED = "DEVELOPABLE_CHART_TRIANGLE_FLIPPED"
    DEVELOPABLE_CHART_SELF_OVERLAP = "DEVELOPABLE_CHART_SELF_OVERLAP"
    # Нормаль смещения вершины (сумма нормалей инцидентных треугольников с весами
    # углов) нулевая либо смотрит ПРОТИВ нормали одного из своих треугольников:
    # смещение втолкнуло бы декаль в поверхность. Отказ, а не молчаливый выбор.
    SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE = "SURFACE_OFFSET_NORMAL_OPPOSES_TRIANGLE"
    # Закон `SOURCE_TRIANGLES_CLIPPED_V1`: вершина куска грани вышла за ЗАМКНУТЫЙ треугольник
    # источника, в котором кусок обязан лежать (доказательство каждого выпущенного куска точным
    # знаком трёх рёбер не сошлось). Это не молчаливый разрез, а отказ с числами.
    CLIP_PIECE_LEFT_ITS_TRIANGLE = "CLIP_PIECE_LEFT_ITS_TRIANGLE"


class PlanarityKind(str, Enum):
    PLANAR_EXACT = "PLANAR_EXACT"
    NEAR_PLANAR = "NEAR_PLANAR"
    DEVELOPABLE_UNFOLDED = "DEVELOPABLE_UNFOLDED"


@dataclass(frozen=True, slots=True)
class AdmissionV1:
    """Исход допуска: отказ с деталью либо вид плоскости."""

    outcome: MaterializationOutcome | None
    detail: str = ""
    planarity: PlanarityKind | None = None
    #: ДЕЙСТВУЮЩИЙ закон укладки: запрошенный, если он применим к домену. Точно
    #: планарный домен лежит на своей плоскости при любом запросе.
    lift_law: NearPlanarLiftLawV1 = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1


#: Поля запроса, которые материализатор берёт у ПЕРЕДАННОГО запроса, а не у
#: подготовки: alpha (покрытие несёт собственную, и в батч она входит через
#: него), закон UV и политика материала (законы ВЫХОДА: ни план, ни
#: подготовка от них не зависят — ядро доказало это тем, что ключ кэша
#: подготовки хоста их не содержит). Всё остальное обязано совпасть с запросом,
#: по которому подготовка скомпилирована: ключ исполнения батча —
#: `(DecalRequestId, PatchDomainId)`, и чужой запрос дал бы батч с чужим ключом.
OUTPUT_POLICY_FIELDS = frozenset(
    {"requested_alpha", "uv_policy_id", "material_policy_id"}
)


def materialization_request(prepared, *, uv_policy_id):
    """Скомпилированный запрос подготовки, в котором заменён ТОЛЬКО закон UV.

    Закон UV — явный параметр материализации: личность запроса (`DecalRequestId`
    и всё, что влияет на план) остаётся ровно той, с которой подготовка
    скомпилирована, а применённый закон лежит в `contract_versions` батча.
    Запрос, собранный хостом заново и подставленный `dataclasses.replace`, давал
    бы то же при удаче и чужой ключ при расхождении — этот путь расхождению
    места не оставляет.
    """

    compiled = getattr(getattr(prepared, "compilation", None), "decal_request", None)
    if compiled is None:
        raise ValueError("the preparation carries no compiled DecalRequestV1")
    return replace(
        compiled, uv_policy_id=PolicyId(getattr(uv_policy_id, "value", uv_policy_id))
    )


def _shown(value) -> str:
    text = repr(value)
    return text if len(text) <= 90 else text[:87] + "..."


def request_mismatch(prepared, coverage, request) -> str:
    """Первое расхождение запроса с подготовкой и покрытия с подготовкой, либо `""`.

    Сверка по ПОЛЯМ, не по `==` целого запроса: `OUTPUT_POLICY_FIELDS`
    законно отличаются. Покрытие, посчитанное на другой подготовке (другой ключ
    плана), — то же нарушение ключа исполнения, только с другой стороны.
    """

    compilation = getattr(prepared, "compilation", None)
    compiled = getattr(compilation, "decal_request", None)
    if compiled is None:
        return "the preparation carries no compiled DecalRequestV1"
    for field in fields(compiled):
        if field.name in OUTPUT_POLICY_FIELDS:
            continue
        left = getattr(compiled, field.name)
        right = getattr(request, field.name, _MISSING)
        if left != right:
            return f"{field.name}: compiled {_shown(left)}, passed {_shown(right)}"
    key = compilation.plan_key
    if key.decal_request_id != compiled.decal_request_id:
        return (
            f"plan_key.decal_request_id: {_shown(key.decal_request_id)}, "
            f"compiled request {_shown(compiled.decal_request_id)}"
        )
    covered = getattr(getattr(coverage, "preparation", None), "compilation", None)
    if covered is not None and covered.plan_key != key:
        return (
            f"coverage plan_key {_shown(covered.plan_key)} "
            f"differs from the preparation's {_shown(key)}"
        )
    return ""


_MISSING = object()


def admit_domain(
    prepared,
    coverage,
    request,
    lift_law: NearPlanarLiftLawV1 = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1,
) -> AdmissionV1:
    """Допуск по порядку: точность, ключ исполнения, плоскость, закон UV, укладка.

    Первая не прошедшая проверка называется; до любой работы материализатора.
    """

    if prepared.outcome.value != "EXACT":
        inner = getattr(prepared, "detail", "")  # причина отказа подготовки, а не только её имя
        return AdmissionV1(
            MaterializationOutcome.COVERAGE_IS_NOT_EXACT,
            f"preparation:{prepared.outcome.value}" + (f": {inner}" if inner else ""),
        )
    if coverage.outcome.value != "EXACT":
        return AdmissionV1(
            MaterializationOutcome.COVERAGE_IS_NOT_EXACT,
            f"coverage:{coverage.outcome.value}",
        )
    mismatch = request_mismatch(prepared, coverage, request)
    if mismatch:
        return AdmissionV1(
            MaterializationOutcome.REQUEST_DOES_NOT_MATCH_PREPARATION, mismatch
        )
    context = getattr(prepared, "context", None)
    frame = getattr(context, "frame", None)
    if not isinstance(frame, RationalAffinePlanarMetricV2):
        return AdmissionV1(
            MaterializationOutcome.DOMAIN_IS_NOT_PLANAR_ADMITTED,
            type(frame).__name__,
        )
    certificate = frame.planarity_certificate
    if isinstance(certificate, ExactSourcePlaneCertificateV1):
        planarity = PlanarityKind.PLANAR_EXACT
    elif isinstance(certificate, NearPlanarProjectionCertificateV1):
        planarity = PlanarityKind.NEAR_PLANAR
    elif isinstance(certificate, DevelopableUnfoldCertificateV1):
        planarity = PlanarityKind.DEVELOPABLE_UNFOLDED
    else:
        return AdmissionV1(
            MaterializationOutcome.DOMAIN_IS_NOT_PLANAR_ADMITTED,
            type(certificate).__name__,
        )
    if request.uv_policy_id not in SUPPORTED_UV_POLICIES:
        return AdmissionV1(
            MaterializationOutcome.UV_POLICY_UNSUPPORTED,
            str(request.uv_policy_id.value),
        )
    effective = NearPlanarLiftLawV1.CERTIFIED_PLANE_V1
    if planarity is PlanarityKind.DEVELOPABLE_UNFOLDED:
        refusal = _developable_refusal(certificate)
        if refusal is not None:
            return refusal
        effective = (
            lift_law if lift_law.clips else NearPlanarLiftLawV1.SOURCE_TRIANGLES_V1
        )
    if planarity is PlanarityKind.NEAR_PLANAR:
        refusal = _lift_refusal(certificate, lift_law)
        if refusal is not None:
            return refusal
        if lift_law.onto_surface:
            effective = lift_law
    return AdmissionV1(None, "", planarity, effective)


def _developable_refusal(certificate) -> AdmissionV1 | None:
    """Сертификат развёртки против судьи растяжения: принятый домен обязан его проходить."""

    failures = stretch_violations(certificate.stretch)
    if failures:
        return AdmissionV1(
            MaterializationOutcome(failures[0].value),
            stretch_refusal_text(certificate.stretch),
        )
    if certificate.chart_boundary_overlap_count:
        return AdmissionV1(
            MaterializationOutcome.DEVELOPABLE_CHART_SELF_OVERLAP,
            f"{certificate.chart_boundary_overlap_count} boundary edge pairs of "
            "the unfolded chart meet or overlap",
        )
    return None


def _lift_refusal(certificate, requested) -> AdmissionV1 | None:
    """Закон укладки против сертификата: чем домен ПРИНЯТ, на то и ложится.

    * Запрошена поверхность: нужен годный σ. Судит его и построитель (под
      `SOURCE_TRIANGLES_V1`), но сертификат мог быть принят под плоскостью, где σ
      только записан, — поэтому судья здесь свой, до единицы работы.
    * Запрошена плоскость, а домен принят под поверхностью: невязка плоскости
      там не судилась и может быть больше бюджета юбки. Класть меш на плоскость
      с неоценённой невязкой — молча нарушить бюджет, поэтому именованный отказ.
    """

    sigma = certificate.width_distortion
    if requested.onto_surface:
        if sigma is None:
            return AdmissionV1(
                MaterializationOutcome.SURFACE_LIFT_UNAVAILABLE,
                "the near-planar certificate carries no width-distortion "
                "record to lift onto source triangles with",
            )
        failures = width_distortion_violations(sigma)
        if failures:
            return AdmissionV1(
                MaterializationOutcome(failures[0].value),
                width_distortion_refusal_text(sigma),
            )
        return None
    if certificate.lift_law.onto_surface:
        residual = _fraction(certificate.max_residual_squared)
        budget = _fraction(certificate.residual_budget)
        if residual > budget * budget:
            return AdmissionV1(
                MaterializationOutcome.NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED,
                "the domain was admitted for the source-triangle lift, where "
                "the plane residual does not judge; a lift onto the certified "
                f"plane needs it within budget: max_residual_squared="
                f"{float(residual):.6e} > residual_budget_squared="
                f"{float(budget * budget):.6e}",
            )
    return None


def _fraction(value):
    return Fraction(value.numerator, value.denominator)
