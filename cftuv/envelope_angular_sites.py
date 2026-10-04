"""Места поворота петель домена для угловых отношений хоста: объявленные углы и вершины разреза цепи.

Вынесено из `envelope_request_export` (тот на своём потолке длины и помечен блокером `HOST_REQUEST_EXPORT_COMPLEXITY`)
вместе с одним новым правилом, которого раньше не было.

ОБЪЯВЛЕННЫЙ УГОЛ БЕЗ СТЫКА ЦЕПЕЙ. Петля из ОДНОЙ цепи (гладкое кольцо: обод купола, колонна) получает от анализа хоста
ГЕОМЕТРИЧЕСКИЕ углы (`_build_geometric_loop_corners`: две вершины, `prev_chain_index == next_chain_index`), которые
не стыки цепей: вершина угла не конец ни одной из двух записей. Раньше такой угол кончался отказом
`ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE: anchor is not a physical ChainUse endpoint` у ЛЮБОГО домена с гладкой
замкнутой петлёй (плоский диск тоже). Настоящий излом в такой вершине есть всегда на стыке кусков разреза цепи (вершины
разреза перечисляются ниже), поэтому виртуальный угол даёт место с пустыми ссылками и счётчик `ANGULAR_CORNERS_OFF_JUNCTION`,
а не отказ и не молчание.
"""

from __future__ import annotations


def _is_endpoint(record, anchor: int) -> bool:
    vertices = tuple(int(item) for item in record.chain.vert_indices)
    return anchor in (vertices[0], vertices[-1])


def angular_sites(patch, patch_id: int, normalized_refs_by_source, record_by_ref: dict, domain_id) -> tuple:
    """Объявленные углы петли И вершины разреза - один перечень мест поворота.

    Место - `(петля, ключ места, вершина разреза?, вершина-якорь, ссылка входящей цепи, ссылка выходящей)`; у виртуального
    угла (якорь не конец записи) обе ссылки `None`.

    Разрез изломанной физической цепочки создаёт стык, которого нет в `loop.corners`: у хоста нет записи об углу внутри
    `BoundaryChain` - именно это и есть чинимый дефект. Рефлексный стык без углового комплекта превратился бы в ДВА
    `CapEnvelopeSpec` вместо веера, то есть в тихую потерю; поэтому вершина разреза проходит РОВНО тот же вывод меры, что и
    объявленный угол, и при недоступности сертифицированной меры даёт именованный отказ, а не колпачки.

    Ключ места остаётся тем же целым `corner_index` для объявленных углов: идентичности углов на доменах без изломов
    обязаны не сдвинуться.
    """

    from .envelope_request_export import EnvelopeDebugHostOutcome, EnvelopeHostAdapterError

    sites = []
    for loop_index, loop in enumerate(patch.boundary_loops):
        for corner_index, corner in enumerate(loop.corners):
            refs = tuple(
                normalized_refs_by_source.get((patch_id, loop_index, int(index)), ())
                for index in (corner.prev_chain_index, corner.next_chain_index)
            )
            if not all(refs):
                raise EnvelopeHostAdapterError(
                    EnvelopeDebugHostOutcome.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                    "BoundaryCorner incident ChainUse references are incomplete",
                    patch_domain_id=domain_id.value,
                )
            anchor = int(corner.vert_index)
            previous, following = refs[0][-1], refs[1][0]
            junction = _is_endpoint(record_by_ref[previous], anchor) and _is_endpoint(record_by_ref[following], anchor)
            sites.append((loop_index, corner_index, False, anchor, previous if junction else None, following if junction else None))
        for source_chain_index in range(len(loop.chains)):
            refs = normalized_refs_by_source.get((patch_id, loop_index, source_chain_index), ())
            if len(refs) < 2:
                continue
            adjacent = list(zip(refs, refs[1:], strict=False))
            if record_by_ref[refs[0]].source_is_closed:
                adjacent.append((refs[-1], refs[0]))
            for incoming_ref, outgoing_ref in adjacent:
                sites.append(
                    (
                        loop_index,
                        f"cut:{incoming_ref[2]}",
                        True,
                        int(record_by_ref[incoming_ref].chain.vert_indices[-1]),
                        incoming_ref,
                        outgoing_ref,
                    )
                )
    return tuple(sites)


__all__ = ("angular_sites",)
