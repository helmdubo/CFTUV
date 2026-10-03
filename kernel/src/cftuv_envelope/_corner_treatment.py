"""Решение об обработке вогнутого угла ДО закона счёта: JOIN мягкого излома одной цепи.

Чистый закон, без `reference`: им пользуются и компиляция (`reference/corner_treatment.py`),
и проверяющий плана (`validation_corner_treatment.py`), а проверяющий верхнего уровня
не вправе тянуть за собой `reference` (там sympy). Одно место решения — одна
реализация на обе двери: расхождение компиляции и проверки было бы расхождением
двух копий, а не обнаруженной подделкой.

Решение владельца (2026-10-03): излом меньше порога внутри ОДНОЙ цепи источника —
не веер, а продолжение полосы: `k = 0` (митра прямого скелета), `u`
непрерывна, шва нет. Порог сначала был 30° (`CORNER_ANGLE_THRESHOLD_DEG` главного
UV-солвера: хост режет свои цепи на куски в точных изломах), затем поднят до 45°
по жалобе владельца на веера там, где они не нужны: контур плоской стены ломался
на 31–36°, шум верха — до 45° (DECISIONS.md, 2026-10-03, JOIN 45°). На углах от
порога кончается сама цепь.

ДВЕ ДВЕРИ ФАКТА, и каждая одна.

* Угол: порог сверяется с интервалом δ/π, который видит закон счёта, то есть с
  выходом `selector_reflex_excess_interval` (единственная дверь сырого углового
  факта; восстановленный канонический угол и сырой идут тем же путём, что и в
  счёте). Сравнение — по нижней и верхней границам порознь.
* Цепь: «одна цепь» — факт хоста, и только ТОГО патча, чей это угол. Хост пишет в
  `PhysicalChainV1.data_record_lineage` запись `chain-source:<PatchId>:<токен>`
  каждому куску цепи патча до разреза по изломам. Шовная цепь двух патчей
  несёт записи ОБОИХ, и общая запись соседа не делает два куска одной цепью
  владельца: берутся только записи с префиксом владельца угла
  (`contracts.lineage.owner_chain_source_prefix`, формат един для хоста и ядра).

Без общей записи владельца ядро знает лишь «одной цепью не доказано»
(`SOURCE_CHAIN_UNPROVEN`), а не «цепи разные»: старые снапшоты записи не несут, и
закон на них инертен — все прежние снапшоты и фикстуры дают прежний ответ побитово.

Известное следствие, которое нельзя вывести из кода: хост режет цепь на куски не
только в изломах, но и в вершинах разбиения шва (`cut_vertices_by_pair`), где
у владельца излом может быть ≈ 0°; такой стык двух кусков одной цепи владельца
JOIN получает по тому же закону (δ < порога, одна цепь), и это верно: цепь одна.
"""

from __future__ import annotations

from fractions import Fraction

from ._canonical_angle import selector_reflex_excess_interval
from .contracts.envelopes import (
    CornerTreatmentReasonV1,
    CornerTreatmentRecordV1,
    CornerTreatmentV1,
)
from .contracts.lineage import owner_chain_source_prefix
from .numeric import ExactRatioV1, IntervalEndpointKind

CORNER_TREATMENT_LAW = "CORNER_TREATMENT_V1"
#: 45° = π/4 рефлексного избытка (было 30° = π/6, порог главного UV-солвера): владелец увидел веера на изломах плоской
#: стены 31–36° и 30–45° шума верха (`sagging_wall`, `rounded_wall_noise_top`) и сдвинул порог. Запись реестра допусков.
JOIN_THRESHOLD_OVER_PI = Fraction(1, 4)
#: Подшаг, который JOIN держит в геометрии вычисления: `q = 2` (изгиб <= pi/2, слабейший потолок Density A). У JOIN веера нет, и его
#: изгиб ограничивает ЗАКОН УГЛА (порог выше, на сырых опорах), а не потолок `pi/q` плотности: иначе излом 30-45 градусов строился
#: на d1/d2 и отказывал целым доменом на d3/d4 (`BINDING_INSIDE_OWN_ORDINAL_WINDOW`, DECISIONS.md 2026-10-03, JOIN-BEND-DENSITY).
JOIN_EVALUATION_SUBTURN_Q = 2


def shared_source_lineage(chain_a, chain_b, owner_patch_id) -> frozenset:
    """Общие записи `chain-source` ВЛАДЕЛЬЦА угла двух цепей: непусто — куски одной его цепи."""

    prefix = owner_chain_source_prefix(owner_patch_id)
    shared = frozenset(chain_a.data_record_lineage) & frozenset(chain_b.data_record_lineage)
    return frozenset(item for item in shared if item.value.startswith(prefix))


def softness(interval) -> CornerTreatmentReasonV1 | None:
    """`None` — δ < порога (`JOIN_THRESHOLD_OVER_PI`) доказано; иначе причина, по которой JOIN не положен."""

    lower, upper = Fraction(interval.lower), Fraction(interval.upper)
    if upper < JOIN_THRESHOLD_OVER_PI or (
        upper == JOIN_THRESHOLD_OVER_PI
        and interval.upper_kind is IntervalEndpointKind.OPEN
    ):
        return None
    if lower >= JOIN_THRESHOLD_OVER_PI:
        return CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT
    return CornerTreatmentReasonV1.REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD


def decide(sector, measure, uses_by_id, chains_by_id):
    """`(обработка, причина, общая линия)` одного угла по сырым фактам."""

    incoming, outgoing = (
        sector.ordered_incident_chain_use_ids[0],
        sector.ordered_incident_chain_use_ids[-1],
    )
    first, second = uses_by_id[incoming], uses_by_id[outgoing]
    shared = shared_source_lineage(
        chains_by_id[first.physical_chain_id],
        chains_by_id[second.physical_chain_id],
        first.owner_patch_id,
    )
    interval, _restoration = selector_reflex_excess_interval(measure.reflex_excess_over_pi)
    reason = softness(interval)
    if reason is not None:
        return CornerTreatmentV1.ANGULAR_PROFILE, reason, shared
    if not shared:
        return (
            CornerTreatmentV1.ANGULAR_PROFILE,
            CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN,
            shared,
        )
    return (
        CornerTreatmentV1.JOIN_CONTINUATION,
        CornerTreatmentReasonV1.SOFT_BEND_IN_ONE_SOURCE_CHAIN,
        shared,
    )


def treatment_record(relation, sector, selection_id, measure, decision) -> CornerTreatmentRecordV1:
    """Запись одного угла по решению `decide`."""

    treatment, reason, shared = decision
    return CornerTreatmentRecordV1(
        treatment_law=CORNER_TREATMENT_LAW,
        corner_relation_id=relation.corner_relation_id,
        selection_certificate_id=selection_id,
        incoming_chain_use_id=sector.ordered_incident_chain_use_ids[0],
        outgoing_chain_use_id=sector.ordered_incident_chain_use_ids[-1],
        treatment=treatment,
        reason=reason,
        threshold_over_pi=ExactRatioV1(
            JOIN_THRESHOLD_OVER_PI.numerator, JOIN_THRESHOLD_OVER_PI.denominator
        ),
        reflex_excess_over_pi=measure.reflex_excess_over_pi,
        shared_source_lineage_ids=shared,
    )


def recompute_record(relation, sector, selection_id, measure, uses_by_id, chains_by_id):
    """Запись угла, пересчитанная ЗАНОВО по сырым фактам снапшота (то, чем сверяются компиляция и план)."""

    return treatment_record(
        relation, sector, selection_id, measure,
        decide(sector, measure, uses_by_id, chains_by_id),
    )
