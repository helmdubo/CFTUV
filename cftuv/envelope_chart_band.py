"""Полосовая карта на стороне хоста: вход ядра из выделения, исходы-триггеры и отказ по alpha.

Полоса вокруг выбранных цепей - запасной путь после именованного отказа развёртки ЦЕЛОГО патча
(`DevelopableBandChartCertificateV1`). Хост решает три вещи, и они живут здесь, а не в `envelope_request_export`
(тот помечен открытым блокером `HOST_REQUEST_EXPORT_COMPLEXITY`): какие цепи домена выбраны (`chart_band_request`),
после каких отказов целого патча полосу пробуют (`band_trigger_host_outcomes`) и что отвечает запрос с alpha выше
досягаемости (`refuse_alpha_beyond_reach`). Саму полосу строит ядро.
"""

from __future__ import annotations

import importlib
from fractions import Fraction

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


def chart_band_request(policy, edge_ids, physical_chains, chain_uses, domain_id):
    """Вход полосы домена: цепи с выбранным ребром (целиком) и досягаемость запроса, либо `None`.

    Выделение хоста - физические рёбра; цепь выбрана, если выбрано любое её ребро (запрос принимает только целые цепи,
    а выделение дополняется до них так же). `None` - полосы нет: политика не названа либо в домене ничего не выбрано.
    """

    if policy is None:
        return None
    request = importlib.import_module("cftuv_envelope.chart_band").chart_band_request
    chosen = {edge_ids[item] for item in policy.selected_physical_edge_ids if item in edge_ids}
    chain_ids = {item.physical_chain_id for item in physical_chains if chosen.intersection(item.ordered_physical_edge_ids)}
    return request(
        physical_chains,
        chain_uses,
        frozenset(item.chain_use_id for item in chain_uses if item.physical_chain_id in chain_ids),
        domain_id,
        policy.reach_cap,
    )


def refuse_alpha_beyond_reach(snapshot, alpha_decimal) -> None:
    """`REQUEST_ALPHA_EXCEEDS_CHART_REACH`, если карта-полоса домена короче alpha запроса: усечённого покрытия нет."""

    from .envelope_request_export import EnvelopeDebugHostOutcome, EnvelopeHostAdapterError

    for metric in snapshot.surface_metric_descriptors:
        certificate = getattr(metric, "planarity_certificate", None)
        if type(certificate) is not band_certificate_type():
            continue
        cap = Fraction(certificate.reach_cap.numerator, certificate.reach_cap.denominator)
        if Fraction(alpha_decimal) > cap:
            raise EnvelopeHostAdapterError(
                EnvelopeDebugHostOutcome.REQUEST_ALPHA_EXCEEDS_CHART_REACH,
                f"alpha={float(alpha_decimal):.6g} m is beyond the chart reach cap {float(cap):.6g} m of the band chart: "
                "the whole Patch does not unfold, and the band around the selected chains is valid up to the cap",
                patch_domain_id=metric.patch_domain_id.value,
            )


__all__ = (
    "BeyondChartReach",
    "ChartPoints",
    "band_certificate_type",
    "band_trigger_host_outcomes",
    "chart_band_request",
    "chart_points",
    "refuse_alpha_beyond_reach",
)
