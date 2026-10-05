"""Общее решение по точкам на прямых цепях источника и стены: итоги доменов прогона -> итоги с растворёнными точками.

АДАПТЕР ОТОБРАЖАЕТ КОНТРАКТ (AGENTS.md): правило, расчёт, растворение и проверка — закон ядра `SILHOUETTE_SOURCE_DOTS_V1`
(`cftuv_envelope.materialize.source_dots`); здесь только упаковка. Результаты доменов (чистая функция входа, лежат в кэшах по
содержимому) остаются как есть; общий закон — чистая функция НАБОРА результатов прогона, и считается над ними в самом конце
`run_production`, поэтому кнопка, живая ширина и свип делят один ответ, а кэш не хранит ничего, что зависит от соседей.

Результат домена, где точки растворены, получает батч без них и числа батча заново (`MATERIALIZE_VERTICES`, ...): ядро отдаёт их
готовыми, хост подставляет по имени и дописывает числа закона. Закон работает только когда ВСЕ материализованные домены собраны
законом `SILHOUETTE_TOPOLOGY_V1` (иначе набор не меняется: три прежних закона побитово прежние).
"""

from __future__ import annotations

import traceback
from dataclasses import replace
from fractions import Fraction

#: Закон топологии, под которым решаются точки (значение `DecalTopologyLawV1.SILHOUETTE_TOPOLOGY_V1`).
SILHOUETTE_LAW = "SILHOUETTE_TOPOLOGY_V1"
#: Исключение внутри общего решения: результаты остаются как были, исход назван (счётчик профиля, строка консоли).
OUTCOME_RAISED = "SILHOUETTE_SOURCE_DOTS_V1_RAISED"


def _counters(item, domain) -> tuple:
    """Числа домена: числа батча подставлены по имени (порядок прежний), числа закона дописаны."""

    overrides = dict(domain.overrides)
    kept = tuple((name, overrides.get(name, value)) for name, value in item.counters)
    return kept + tuple(domain.counters)


def _updated(item, domain):
    changed = {"counters": _counters(item, domain)}
    if domain.removed:
        changed.update(
            batch=domain.batch,
            content_digest=domain.content_digest,
            vertex_normals=domain.vertex_normals,
            offset_normals_digest=domain.offset_normals_digest,
            diagnostics=(*item.diagnostics, domain.note),
        )
    return replace(item, **changed)


def dissolve_source_dots(results, slide=None):
    """`(результаты, итог закона | None)`: точки на прямых цепях источника и стены растворены по всем доменам прогона сразу.

    `slide` — допуск UV запроса (доля alpha; `None` — умолчание ядра). Прогон не под законом силуэта или без материализованных
    доменов возвращается как есть. Порядок результатов сохраняется; домены вне итога (отказы) не тронуты.
    """

    from cftuv_envelope.contracts.metric import DEFAULT_SILHOUETTE_UV_SLIDE
    from cftuv_envelope.materialize.source_dots import SourceDotInputV1, SourceDotsV1, reconcile_source_dots

    results = tuple(results)
    live = [item for item in results if item.is_materialized]
    if not live or {item.decal_topology_law for item in live} != {SILHOUETTE_LAW}:
        return results, None
    try:
        found = reconcile_source_dots(
            [SourceDotInputV1(item.patch_id, item.batch, item.source_normal, tuple(item.vertex_normals)) for item in live],
            DEFAULT_SILHOUETTE_UV_SLIDE if slide is None else Fraction(slide),
        )
    except Exception:  # noqa: BLE001 - исход называется, а не теряется: косметика не роняет кнопку
        tail = traceback.format_exc().strip().splitlines()[-1]
        print(f"[CFTUV][Production] {OUTCOME_RAISED}: {tail}; the results are left as they were", flush=True)
        return results, SourceDotsV1((), False, (OUTCOME_RAISED,))
    by_patch = {domain.key: domain for domain in found.domains}
    return tuple(_updated(item, by_patch[item.patch_id]) if item.is_materialized else item for item in results), found
