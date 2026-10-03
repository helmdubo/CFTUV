"""Вход полосовой карты: что метрика домена знает о выбранных цепях и досягаемости запроса.

Единая выборка для хоста (вход построителя метрики), валидатора (пересчёт) и тестов, как `declared_chains` для прямых
цепей: пересчёт расходился бы с записью на самом определении «обода». Обод — рёбра `ChainUse` ДОМЕНА, выбранных запросом;
граница патча — рёбра всех его `ChainUse` с направлениями, в которых они идут по петлям (внутренность владельца
слева). Этот модуль только выбирает и упорядочивает факты снапшота, геометрии в нём нет.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from .contracts.analysis import ChainUseOrientation


@dataclass(frozen=True, slots=True)
class ChartBandRequestV1:
    """Выбранные цепи домена, досягаемость и опознание сторон границы патча.

    `rim_edges` — пары вершин рёбер выбранных `ChainUse` (по их направлению), `boundary_uses` — для КАЖДОЙ стороны
    границы патча (направленная пара вершин) её `ChainUse` и физическое ребро. Оба упорядочены по именам: запрос — часть
    ключа памяти построителя и должен быть хэшируемым и воспроизводимым.
    """

    selected_chain_use_ids: frozenset
    reach_cap: Fraction
    rim_edges: tuple
    boundary_uses: tuple


@dataclass(frozen=True, slots=True)
class RequestChartPolicyV1:
    """Что запрос говорит полосе: досягаемость, выбранные цепи, alpha. Валидатор сверяет с ним запись снапшота."""

    reach_cap: Fraction
    selected_chain_use_ids: frozenset
    #: `None` - alpha символическая (запрос oracle без числа): сравнивать с досягаемостью нечем, и это не отказ.
    requested_alpha: Fraction | None


def request_chart_policy(request) -> RequestChartPolicyV1:
    value = getattr(request.requested_alpha, "value", None)
    return RequestChartPolicyV1(
        Fraction(request.chart_reach_cap.numerator, request.chart_reach_cap.denominator),
        frozenset(request.selected_chain_use_ids),
        None if value is None else Fraction(value),
    )


def directed_use_edges(chain, chain_use) -> tuple:
    """Направленные рёбра `ChainUse`: `((начало, конец, физическое ребро), ...)` в порядке обхода."""

    vertices = tuple(chain.ordered_source_vertex_ids)
    edges = tuple(chain.ordered_physical_edge_ids)
    count = len(vertices)
    pairs = [(vertices[index], vertices[(index + 1) % count], edges[index]) for index in range(len(edges))]
    if chain_use.orientation is ChainUseOrientation.B_START_TO_END:
        pairs = [(end, start, edge) for start, end, edge in reversed(pairs)]
    return tuple(pairs)


def chart_band_request(physical_chains, chain_uses, selected_chain_use_ids, patch_domain_id, reach_cap):
    """`ChartBandRequestV1` домена либо `None`, если запрос не выбрал в нём ни одной цепи."""

    chains = {item.physical_chain_id: item for item in physical_chains}
    domain_uses = sorted(
        (item for item in chain_uses if item.patch_domain_id == patch_domain_id),
        key=lambda item: item.chain_use_id.value,
    )
    selected = frozenset(
        item.chain_use_id for item in domain_uses if item.chain_use_id in selected_chain_use_ids
    )
    if not selected:
        return None
    rim: list = []
    boundary: list = []
    for use in domain_uses:
        for start, end, edge in directed_use_edges(chains[use.physical_chain_id], use):
            boundary.append(((start, end), use.chain_use_id, edge))
            if use.chain_use_id in selected:
                rim.append((start, end))
    return ChartBandRequestV1(
        selected_chain_use_ids=selected,
        reach_cap=Fraction(reach_cap),
        rim_edges=tuple(rim),
        boundary_uses=tuple(boundary),
    )


__all__ = (
    "ChartBandRequestV1",
    "RequestChartPolicyV1",
    "chart_band_request",
    "directed_use_edges",
    "request_chart_policy",
)
