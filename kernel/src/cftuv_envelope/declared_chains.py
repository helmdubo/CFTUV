"""Цепи, объявленные ПРЯМЫМИ: единая выборка для хоста (вход метрики) и валидатора (пересчёт).

Объявленная прямой цепь — открытая `PhysicalChain` из трёх и более вершин, чьи `ChainUse`
принадлежат домену. Углов у неё нет: хост режет цепь в каждой вершине излома и в каждом
объявленном угле, поэтому внутренняя вершина цепи — не угол (якорь `BoundaryCorner` всегда
конец цепи). Очередь требует от такой цепи точной коллинеарности в карте
(`evaluation_geometry`), и карта развёртки обязана её обеспечить — для этого построитель
метрики должен знать, какие это цепи. Выборка живёт в ядре и ОДНА: хост и валидатор берут её
отсюда, иначе пересчёт расходился бы с записью на самом определении «объявленной».
"""

from __future__ import annotations


def declared_straight_chain_vertices(physical_chains, chain_uses, patch_domain_id):
    """Упорядоченные вершины объявленных прямыми цепей домена; порядок — по именам вершин."""

    in_domain = {
        item.physical_chain_id
        for item in chain_uses
        if item.patch_domain_id == patch_domain_id
    }
    chains = {
        tuple(chain.ordered_source_vertex_ids)
        for chain in physical_chains
        if chain.physical_chain_id in in_domain
        and not chain.is_closed
        and len(chain.ordered_source_vertex_ids) >= 3
    }
    return tuple(sorted(chains, key=lambda chain: tuple(vertex.value for vertex in chain)))
