"""Полосовая карта на стороне хоста: вход ядра из выделения, исходы-триггеры и отказ по alpha.

Полоса вокруг выбранных цепей - запасной путь после именованного отказа развёртки ЦЕЛОГО патча
(`DevelopableBandChartCertificateV1`). Хост решает три вещи, и они живут здесь, а не в `envelope_request_export`
(тот помечен открытым блокером `HOST_REQUEST_EXPORT_COMPLEXITY`): какие цепи домена выбраны (`chart_band_request`),
после каких отказов целого патча полосу пробуют (`band_trigger_host_outcomes`) и что отвечает запрос с alpha выше
досягаемости (`refuse_alpha_beyond_reach`). Саму полосу строит ядро.

СУЖЕНИЕ (`CHART_REACH_TIGHTENED_FOR_SEAM`). Карта под досягаемостью запроса может отказать швом разреза кольца либо растяжением
полосы, хотя декали нужна лишь собственная досягаемость `alpha * (1 + b)`: носитель растёт с досягаемостью, а не с alpha. Тогда
сессия пересобирает полосу ОДИН раз под `alpha * (1 + b)` (`tightened_export`; арифметика - `tightened_reach_cap` ядра), и
запись о сужении едет в сертификате. Суженная карта зависит от alpha, поэтому её кэши ключуются точной досягаемостью, а
подготовка и хранилище по содержимому сверяются с ней (`tightening_is_current`): устаревшая карта не отдаётся, а пересобирается.

КОЛЬЦО (носитель вокруг замкнутой цепи, `BandCutV1`): вершины пути разреза на карте раздвоены (`<вершина>|cut:R` - правая
копия). Хост читает карту по именам ВЕРШИН ИСТОЧНИКА, поэтому у цикла грани и у ребра цепи, лежащих у разреза, имена карты
берутся отсюда (`chart_face_points`, `chart_edge_ends`), а угол, чья вершина на пути разреза, на карте разрезан на два
угла со стеной и угловым отношением не описывается (`cut_path_vertices`, счётчик `ANGULAR_CORNERS_AT_RING_CUT`).
"""

from __future__ import annotations

import importlib
from fractions import Fraction
from typing import NamedTuple

_METRIC_CONTRACTS = "cftuv_envelope.contracts.metric"


def band_certificate_type():
    """Тип сертификата полосы; корневой фасад ядра заморожен и его не экспортирует."""

    return importlib.import_module(_METRIC_CONTRACTS).DevelopableBandChartCertificateV1


class BeyondChartReach(KeyError):
    """У вершины нет координат на карте-полосе: она дальше досягаемости запроса от выбранных цепей."""


class ChartPoints(dict):
    """Координаты карты-полосы: вершина вне носителя называется `BeyondChartReach`, а не безымянным `KeyError`."""

    def __missing__(self, key):
        raise BeyondChartReach(key)


def chart_points(points: dict, frame) -> dict:
    """Координаты вершин кадра: у карты-полосы - `ChartPoints`, у остальных - тот же словарь (отсутствие - дефект)."""

    return ChartPoints(points) if type(frame.planarity_certificate) is band_certificate_type() else points


def band_trigger_host_outcomes() -> frozenset:
    """Исходы ХОСТА отказов целого патча, после которых пробуется полоса: имена `BAND_TRIGGER_OUTCOMES` ядра."""

    from .envelope_request_export import EnvelopeDebugHostOutcome

    triggers = importlib.import_module("cftuv_envelope.planar_metric").BAND_TRIGGER_OUTCOMES
    return frozenset(EnvelopeDebugHostOutcome(item.value) for item in triggers)


def _cut_of(frame):
    return getattr(frame.planarity_certificate, "cut", None)


def cut_path_vertices(frame) -> frozenset:
    """Вершины пути разреза кольца (имена ядра) либо пустое множество: углы в них на карте разрезаны."""

    cut = _cut_of(frame)
    return frozenset() if cut is None else frozenset(cut.path_vertex_ids)


def chart_face_points(coordinates, frame, kernel_vertex, triangle_ids, face) -> tuple:
    """Точки цикла грани на карте: у кольца с разрезом вершина пути на правой стороне разреза - правая копия."""

    names = tuple(kernel_vertex[int(item)] for item in face.vertex_cycle)
    cut = _cut_of(frame)
    if cut is not None:
        strip = importlib.import_module("cftuv_envelope._annulus_cut")
        corners = frozenset((item.triangle_id, item.vertex_id) for item in cut.right_corners)
        names = strip.chart_cycle(names, tuple(triangle_ids[int(item)] for item in face.triangle_ids), corners)
    return tuple(coordinates[item] for item in names)


def chart_edge_ends(frame, start, end) -> tuple:
    """Имена карты концов ребра границы `start -> end` (имена ядра): у ребра при разрезе кольца копии вершины разные."""

    if _cut_of(frame) is None:
        return start, end
    side_ends = importlib.import_module("cftuv_envelope._annulus_cut").chart_side_ends
    return side_ends(frame.planarity_certificate.strip_boundary, start, end)


def chart_band_request(policy, edge_ids, physical_chains, chain_uses, domain_id):
    """Вход полосы домена: цепи с выбранным ребром (целиком) и досягаемость запроса, либо `None`.

    Выделение хоста - физические рёбра; цепь выбрана, если выбрано любое её ребро (запрос принимает только целые цепи,
    а выделение дополняется до них так же). `None` - полосы нет: политика не названа либо в домене ничего не выбрано.
    Политика с суженной досягаемостью строит полосу под ней и несёт досягаемость запроса записью (`requested_reach_cap`).
    """

    if policy is None:
        return None
    request = importlib.import_module("cftuv_envelope.chart_band").chart_band_request
    chosen = {edge_ids[item] for item in policy.selected_physical_edge_ids if item in edge_ids}
    chain_ids = {item.physical_chain_id for item in physical_chains if chosen.intersection(item.ordered_physical_edge_ids)}
    tight = policy.tightened_reach_cap
    return request(
        physical_chains,
        chain_uses,
        frozenset(item.chain_use_id for item in chain_uses if item.physical_chain_id in chain_ids),
        domain_id,
        policy.reach_cap if tight is None else tight,
        None if tight is None else policy.reach_cap,
        None if tight is None else policy.tightened_after,
    )


def policy_alpha(alpha) -> Fraction | None:
    """alpha запроса в метрах точной дробью (ТА величина, что несёт запрос) либо `None`, если она не принимается."""

    from .envelope_request_policy import request_alpha_decimal

    try:
        return Fraction(request_alpha_decimal(alpha))
    except (ArithmeticError, TypeError, ValueError):
        return None


def tightened_export(topology_export, refused_outcome):
    """Экспорт для ОДНОЙ пересборки полосы под `alpha * (1 + b)` после отказа `refused_outcome`, либо `None`.

    `None`: политики полосы или alpha нет, экспорт уже суженный, отказ не из тех, что лечит узкий носитель
    (`BAND_TIGHTEN_OUTCOMES`), либо досягаемость запроса не шире собственной досягаемости декали - тогда названный отказ
    остаётся как есть. Решение - арифметика ядра (`tightened_reach_cap`), хост её только зовёт.
    """

    policy = topology_export.chart_band
    if policy is None or policy.tightened_reach_cap is not None:
        return None
    from .envelope_request_policy import DEFAULT_ENVELOPE_STRETCH_BUDGET

    chart_band = importlib.import_module("cftuv_envelope.chart_band")
    value = getattr(refused_outcome, "value", refused_outcome)
    if value not in {item.value for item in chart_band.BAND_TIGHTEN_OUTCOMES}:
        return None
    budget = topology_export.developable_stretch_budget
    cap = chart_band.tightened_reach_cap(
        policy.reach_cap, policy.alpha, DEFAULT_ENVELOPE_STRETCH_BUDGET if budget is None else budget
    )
    return None if cap is None else topology_export.with_band_tightened(cap, value)


class BandFactV1(NamedTuple):
    """Что хост читает у одной карты-полосы снапшота: домен метрики, досягаемость, суженность и были ли отброшены треугольники."""

    domain_id: str
    reach_cap: Fraction
    tightened: bool
    excluded: bool


def band_facts_of(snapshot) -> tuple:
    """Факты карт-полос снапшота в порядке его метрик (пусто: полос нет). Воркер шлёт их вместе с пиклом подготовки (`LazyPreparationV1`)."""

    band = band_certificate_type()
    facts = []
    for metric in snapshot.surface_metric_descriptors:
        certificate = getattr(metric, "planarity_certificate", None)
        if type(certificate) is band:
            facts.append(
                BandFactV1(
                    metric.patch_domain_id.value,
                    Fraction(certificate.reach_cap.numerator, certificate.reach_cap.denominator),
                    certificate.tightened is not None,
                    bool(certificate.excluded_triangle_count),
                )
            )
    return tuple(facts)


def tightened_cap_of_facts(facts) -> Fraction | None:
    """Суженная досягаемость первой суженной карты среди `facts` (метры) либо `None`."""

    for fact in facts:
        if fact.tightened:
            return fact.reach_cap
    return None


def tightened_cap_of(snapshot) -> Fraction | None:
    """Суженная досягаемость карты-полосы снапшота (метры) либо `None`: карта под досягаемостью запроса либо не полоса."""

    for metric in snapshot.surface_metric_descriptors:
        certificate = getattr(metric, "planarity_certificate", None)
        if type(certificate) is band_certificate_type() and certificate.tightened is not None:
            return Fraction(certificate.reach_cap.numerator, certificate.reach_cap.denominator)
    return None


def tightening_is_current(topology_export, snapshot) -> bool:
    """Карта снапшота годится запросу: она не суженная либо суженная ровно под собственной досягаемостью alpha запроса (`_tightening_is_current`)."""

    return _tightening_is_current(topology_export, tightened_cap_of(snapshot))


def tightening_is_current_facts(topology_export, facts) -> bool:
    """То же по фактам карт-полос (`band_facts_of`): подготовка, которую родитель не разворачивал, получает тот же ответ."""

    return _tightening_is_current(topology_export, tightened_cap_of_facts(facts))


def _tightening_is_current(topology_export, stored) -> bool:
    """Суженная карта записывает отказ под досягаемостью запроса (он от alpha не зависит), а её досягаемость - функция alpha:
    `tightened_export` называет ту, что нужна ЭТОМУ запросу. Другая (alpha выросла и карта короче неё, либо упала и карта
    шире нужной) - устаревшая карта: подготовку на ней не отдают, а пересобирают. Карта под досягаемостью запроса годится
    всегда: её отказа нет, и от alpha она не зависит.
    """

    if stored is None:
        return True
    policy = topology_export.chart_band
    if policy is None:
        return False
    from .envelope_request_policy import DEFAULT_ENVELOPE_STRETCH_BUDGET

    budget = topology_export.developable_stretch_budget
    wanted = importlib.import_module("cftuv_envelope.chart_band").tightened_reach_cap(
        policy.reach_cap, policy.alpha, DEFAULT_ENVELOPE_STRETCH_BUDGET if budget is None else budget
    )
    return wanted == stored


def _beyond_reach(domain_id, cap, alpha_decimal):
    """Отказ `REQUEST_ALPHA_EXCEEDS_CHART_REACH` домена `domain_id`, чья карта-полоса короче alpha запроса (один текст на оба пути)."""

    from .envelope_request_export import EnvelopeDebugHostOutcome, EnvelopeHostAdapterError

    return EnvelopeHostAdapterError(
        EnvelopeDebugHostOutcome.REQUEST_ALPHA_EXCEEDS_CHART_REACH,
        f"alpha={float(alpha_decimal):.6g} m is beyond the chart reach cap {float(cap):.6g} m of the band chart: "
        "the whole Patch does not unfold, and the band around the selected chains is valid up to the cap",
        patch_domain_id=domain_id,
    )


def refuse_alpha_beyond_reach(snapshot, alpha_decimal) -> None:
    """`REQUEST_ALPHA_EXCEEDS_CHART_REACH`, если карта-полоса домена короче alpha запроса: усечённого покрытия нет."""

    for metric in snapshot.surface_metric_descriptors:
        certificate = getattr(metric, "planarity_certificate", None)
        if type(certificate) is not band_certificate_type():
            continue
        cap = Fraction(certificate.reach_cap.numerator, certificate.reach_cap.denominator)
        if certificate.excluded_triangle_count and Fraction(alpha_decimal) > cap:
            raise _beyond_reach(metric.patch_domain_id.value, cap, alpha_decimal)


def refuse_alpha_beyond_reach_facts(facts, alpha_decimal) -> None:
    """То же по фактам карт-полос (`band_facts_of`): подготовка, которую родитель не разворачивал, отвечает тем же отказом."""

    for fact in facts:
        if fact.excluded and Fraction(alpha_decimal) > fact.reach_cap:
            raise _beyond_reach(fact.domain_id, fact.reach_cap, alpha_decimal)


__all__ = (
    "BandFactV1",
    "BeyondChartReach",
    "ChartPoints",
    "band_certificate_type",
    "band_facts_of",
    "band_trigger_host_outcomes",
    "chart_band_request",
    "chart_edge_ends",
    "chart_face_points",
    "chart_points",
    "cut_path_vertices",
    "policy_alpha",
    "refuse_alpha_beyond_reach",
    "refuse_alpha_beyond_reach_facts",
    "tightened_cap_of",
    "tightened_cap_of_facts",
    "tightened_export",
    "tightening_is_current",
    "tightening_is_current_facts",
)
