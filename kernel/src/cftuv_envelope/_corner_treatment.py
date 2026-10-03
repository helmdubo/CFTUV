"""Решение об обработке вогнутого угла ДО закона счёта: JOIN излома ОДНОЙ цепи хоста (`CORNER_JOIN_SAME_PCHAIN_V1`).

Чистый закон, без `reference`: им пользуются и компиляция (`reference/corner_treatment.py`),
и проверяющий плана (`validation_corner_treatment.py`), а проверяющий верхнего уровня
не вправе тянуть за собой `reference` (там sympy). Одно место решения — одна
реализация на обе двери: расхождение компиляции и проверки было бы расхождением
двух копий, а не обнаруженной подделкой.

РЕШЕНИЕ ВЛАДЕЛЬЦА (2026-10-04): JOIN решает ТОЖДЕСТВО ЦЕПИ, а не порог угла. Любой излом внутри ОДНОЙ цепи
хоста (цепь главного солвера, «pChain») — продолжение полосы: `k = 0` (митра прямого скелета), `u` непрерывна,
шва нет. Швы остаются там, где сходятся фронты РАЗНЫХ pChain (стык разных цепей, локус равного времени). Порог
угла прежних решений (30° 2026-10-03, затем 45° по жалобе на веера шумного верха) этим законом заменён:
на шумном верхе `rounded_wall_noise_top` каждая из двух длинных цепей давала десятки швов на изломах 30–78° внутри
одной цепи, а владелец хотел видеть шов лишь там, где встречаются фронты разных цепей. Единственный предел
оставшийся — ПРЕДЕЛ ИЗГИБА, он не порог выбора: JOIN только для изгиба СТРОГО меньше четверти оборота
(`δ/π < 1/2`, `JOIN_BEND_BOUND_OVER_PI`; решение 2026-10-04 после замера: у ТОЧНОГО прямого угла излом билинейной UV
доходил до 1.0–1.15 alpha, то есть видимого сдвига текстуры на рамах двери). Точный прямой угол и острее остаются
ИМЕНОВАННЫМ углом (веер/шов) под прежним счётом, как до закона. Проверка опор (`JOIN_EVALUATION_SUBTURN_Q = 2`,
`dot >= 0`) слабее и никогда не срабатывает раньше.

ДВЕ ДВЕРИ ФАКТА, и каждая одна.

* Цепь: «одна цепь» — факт хоста, и только ТОГО патча, чей это угол. Хост пишет в
  `PhysicalChainV1.data_record_lineage` запись `chain-source:<PatchId>:<токен>`
  каждому куску цепи патча до разреза по изломам. Шовная цепь двух патчей
  несёт записи ОБОИХ, и общая запись соседа не делает два куска одной цепью
  владельца: берутся только записи с префиксом владельца угла
  (`contracts.lineage.owner_chain_source_prefix`, формат един для хоста и ядра).
  Без общей записи владельца ядро знает лишь «одной цепью не доказано»
  (`SOURCE_CHAIN_UNPROVEN`), а не «цепи разные»: старые снапшоты записи не несут, и
  закон на них инертен — все прежние снапшоты и фикстуры дают прежний ответ побитово.
* Угол: предел изгиба сверяется с интервалом δ/π, который видит закон счёта, то есть с
  выходом `selector_reflex_excess_interval` (единственная дверь сырого углового
  факта; восстановленный канонический угол и сырой идут тем же путём, что и в
  счёте). Сравнение — по нижней и верхней границам порознь; точный прямой угол (канонический
  `δ/π = 1/2`, замкнутый конец) — вне предела (предел исключительный), и угол в допуске 0.1° от прямого, который
  восстанавливается на канонический, решается так же: прямой угол и его шум ведут себя одинаково (угол).

Известное следствие, которое нельзя вывести из кода: хост режет цепь на куски не
только в изломах, но и в вершинах разбиения шва (`cut_vertices_by_pair`), где
у владельца излом может быть ≈ 0°; такой стык двух кусков одной цепи владельца
JOIN получает по тому же закону, и это верно: цепь одна. Выпуклые и вырожденные в карте стыки одной цепи записи
угла не имеют (хост пишет её только вогнутым): их продолжает материализатор по тому же тождеству
(`materialize/stations._same_chain_successors`), с тем же пределом изгиба.
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

CORNER_TREATMENT_LAW = "CORNER_JOIN_SAME_PCHAIN_V1"
#: ПРЕДЕЛ ИЗГИБА JOIN, доля π рефлексного избытка: 1/2 = четверть оборота = 90°, ИСКЛЮЧИТЕЛЬНЫЙ (JOIN при `δ/π < 1/2`; прежде
#: здесь стоял порог выбора 1/4 = 45°, до него 1/6 = 30°: решал угол; теперь решает тождество цепи, а предел — строго
#: меньше четверти оборота). Запись реестра допусков.
JOIN_BEND_BOUND_OVER_PI = Fraction(1, 2)
#: Подшаг, который JOIN держит в геометрии вычисления: `q = 2` (изгиб <= pi/2, слабейший потолок Density A). У JOIN веера нет, и его
#: изгиб ограничивает ЗАКОН УГЛА (предел выше, на сырых опорах), а не потолок `pi/q` плотности: иначе излом 30-45 градусов строился
#: на d1/d2 и отказывал целым доменом на d3/d4 (`BINDING_INSIDE_OWN_ORDINAL_WINDOW`, DECISIONS.md 2026-10-03, JOIN-BEND-DENSITY).
JOIN_EVALUATION_SUBTURN_Q = 2


def shared_source_lineage(chain_a, chain_b, owner_patch_id) -> frozenset:
    """Общие записи `chain-source` ВЛАДЕЛЬЦА угла двух цепей: непусто — куски одной его цепи."""

    prefix = owner_chain_source_prefix(owner_patch_id)
    shared = frozenset(chain_a.data_record_lineage) & frozenset(chain_b.data_record_lineage)
    return frozenset(item for item in shared if item.value.startswith(prefix))


def bend_reason(interval) -> CornerTreatmentReasonV1 | None:
    """`None` — `δ/π < JOIN_BEND_BOUND_OVER_PI` доказано (строго); иначе причина, по которой JOIN не положен."""

    lower, upper = Fraction(interval.lower), Fraction(interval.upper)
    if upper < JOIN_BEND_BOUND_OVER_PI or (
        upper == JOIN_BEND_BOUND_OVER_PI
        and interval.upper_kind is IntervalEndpointKind.OPEN
    ):
        return None
    if lower >= JOIN_BEND_BOUND_OVER_PI:
        return CornerTreatmentReasonV1.REFLEX_EXCESS_NOT_SOFT
    return CornerTreatmentReasonV1.REFLEX_EXCESS_INTERVAL_CONTAINS_THRESHOLD


def decide(sector, measure, uses_by_id, chains_by_id):
    """`(обработка, причина, общая линия)` одного угла по сырым фактам.

    Порядок решения — порядок закона: сначала тождество цепи (две опоры ОДНОЙ цепи владельца?), затем предел
    изгиба. Куски разных цепей остаются углом под своим счётом с причиной `SOURCE_CHAIN_UNPROVEN`.
    """

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
    if not shared:
        return (
            CornerTreatmentV1.ANGULAR_PROFILE,
            CornerTreatmentReasonV1.SOURCE_CHAIN_UNPROVEN,
            shared,
        )
    interval, _restoration = selector_reflex_excess_interval(measure.reflex_excess_over_pi)
    reason = bend_reason(interval)
    if reason is not None:
        return CornerTreatmentV1.ANGULAR_PROFILE, reason, shared
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
            JOIN_BEND_BOUND_OVER_PI.numerator, JOIN_BEND_BOUND_OVER_PI.denominator
        ),
        reflex_excess_over_pi=measure.reflex_excess_over_pi,
        shared_source_lineage_ids=shared,
    )


def recompute_record(relation, sector, selection_id, measure, uses_by_id, chains_by_id) -> CornerTreatmentRecordV1:
    """Запись угла, пересчитанная ЗАНОВО по сырым фактам снапшота (то, чем сверяются компиляция и план)."""

    return treatment_record(
        relation, sector, selection_id, measure,
        decide(sector, measure, uses_by_id, chains_by_id),
    )
