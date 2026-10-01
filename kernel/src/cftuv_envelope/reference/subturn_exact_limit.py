"""Веер ровно на пределе подшага: точный признак неосуществимости счёта.

Ideal-веер Huber равношаговый. Если его подшаг РОВНО `pi/q` (остаток подшага
`4*dot^2 - 3*|l|^2*|r|^2` для `q = 6` и его аналоги для других `q` — точный
нуль), то все `H + 1` шагов равны `pi/q`, а их сумма — полный поворот угла.
Значит допустимая область скрытых лучей — ТОЧКА: каждый скрытый луч закреплён
поворотом, и любое отклонение хоть одного луча даёт шаг больше `pi/q`.

В точке с иррациональным лучом рационального направления нет, поэтому веер при
этом счёте неосуществим. Это утверждение точное и доказывается ДО поиска окон:
прежде такой веер доходил до `_termination_boxes`, где ящик вокруг точки
сжимался 96 раз и отказывал, не называя причины.

Признак — НЕ допуск. Две половины условия (остаток ровно нуль, луч
ДОКАЗАННО иррационален) решаются точной арифметикой, и ни одной из них нельзя
заменить сравнением «почти». Иррациональность — доказанная в точном
квадратичном поле (`density_support_direction_rationality`), а не «не похоже
на дробь»: `(1 + sqrt 3, 2 + 2 sqrt 3)` — направление `(1, 2)`. Луч вне поля
(`None`) иррациональным не назван, и признак на нём не срабатывает. Рациональный
веер на пределе — осуществим: его точка рациональна, и этот случай признак НЕ
срабатывает (`needs_binding` ему не нужен, и ответ прежний).

Следствие для лифта (`_evaluation_density_spec`): счёт на пределе с
иррациональным лучом считается неосуществимым, и существующий лифт доходит до
`H + 1`. Запись лифта в этом случае несёт собственный закон
`EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1`; независимый
верификатор (`verify_exact_limit_lift`) перепроверяет обе половины по
геометрии вычисления, а не по записи.
"""

from __future__ import annotations

from fractions import Fraction

from .._density_policy import EVALUATION_SUBTURN_LIFT_PREDICATES
from ..contracts.envelopes import (
    EvaluationGeometrySubturnCountLiftLawV1,
    EvaluationGeometrySubturnCountLiftV1,
    ExactTurnSignV1,
)
from .direction_binding import (
    BINDING_SUBTURN_LE_DELTA_MAX,
    DirectionBindingCertificateUnproven,
)


def ideal_is_exact_limit_with_irrational_direction(
    metric,
    ideal,
    q: int,
) -> bool:
    """Подшаг ideal-веера ровно `pi/q` и хотя бы один скрытый луч иррационален.

    Достаточно первой пары: конструкция Huber равношаговая (тот же довод, что у
    `_density_ideal_is_subturn_feasible`). Порядок проверки — от дешёвого к
    дорогому: знак остатка уже посчитан и запомнен проверкой осуществимости,
    рациональность лучей считается только на самом пределе.
    """

    from .adaptive_density_fan import _covectors, _subturn_boundary
    from .direction_binding import density_support_direction_rationality

    if len(ideal) < 3:
        return False
    covectors = _covectors(metric, ideal)
    if not _subturn_boundary(metric, covectors[0], covectors[1], q):
        return False
    return any(
        density_support_direction_rationality(metric, ideal[index]) is False
        for index in range(1, len(ideal) - 1)
    )


def density_count_is_feasible(metric, ideal, q: int) -> bool:
    """Осуществим ли счёт веера: подшаг `<= pi/q`, и не «ровно на пределе».

    Веер ровно на пределе с иррациональным скрытым лучом удовлетворяет
    `подшаг <= pi/q`, но рационального представителя у его единственной точки
    нет: счёт неосуществим, и это называется здесь, а не в отказе поиска окон.
    """

    from .compile import _density_ideal_is_subturn_feasible

    return _density_ideal_is_subturn_feasible(
        metric, ideal, q
    ) and not ideal_is_exact_limit_with_irrational_direction(metric, ideal, q)


def build_evaluation_subturn_count_lift(
    metric,
    selection,
    source_count: int,
    effective_count: int,
    q: int,
    effective_ideal,
    predecessor_ideal,
    canonical_predecessor_ideal=None,
) -> EvaluationGeometrySubturnCountLiftV1:
    """Запись лифта под тем законом, чьи факты проверяемы в геометрии вычисления.

    Строго неосуществимый предшественник — прежний закон. Иначе — закон
    «на пределе»: предшественник стоит ровно на `pi/q` и его скрытый луч
    иррационален. Третий — предел на КАНОНИЧЕСКОМ веере
    (`canonical_predecessor_ideal`): шум привязки сдвинул вычислительный угол,
    но канонический предшественник стоит ровно на `pi/q` с иррациональным
    лучом. Ни один из трёх не подтверждён — запись не выдаётся вовсе:
    лифт без проверяемого основания есть ровно та подмена, которую запрещает п. 4.

    Для восстановленного угла предшественник может быть неосуществим на
    КАНОНИЧЕСКОМ веере (предел с иррациональным лучом), тогда как сырая
    геометрия строго его превышает: основание записи — второе, оно проверяемо
    независимо, поэтому закон прежний.
    """

    from .angular import _lift_count_is_feasible
    from .compile import _exact_turn_witness

    sign, cosine_squared = _exact_turn_witness(metric, effective_ideal)
    laws = EvaluationGeometrySubturnCountLiftLawV1

    def record(law):
        return EvaluationGeometrySubturnCountLiftV1(
            lift_law=law,
            source_selection_certificate_id=selection.certificate_id,
            source_hidden_edge_count=source_count,
            effective_hidden_edge_count=effective_count,
            max_subturn_q=q,
            evaluation_turn_sign=sign,
            evaluation_turn_cosine_squared=cosine_squared,
            minimality_predecessor_hidden_edge_count=effective_count - 1,
            proven_predicates=EVALUATION_SUBTURN_LIFT_PREDICATES[law],
        )

    strict = record(laws.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_V1)
    if not _lift_count_is_feasible(
        strict, source_count
    ) and not _lift_count_is_feasible(strict, effective_count - 1):
        return strict
    if turn_is_exactly_at_count_limit(
        sign,
        Fraction(cosine_squared.numerator, cosine_squared.denominator),
        effective_count - 1,
        q,
    ) and ideal_is_exact_limit_with_irrational_direction(
        metric, predecessor_ideal, q
    ):
        return record(
            laws.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_EXACT_LIMIT_V1
        )
    if (
        canonical_predecessor_ideal is not None
        and ideal_is_exact_limit_with_irrational_direction(
            metric, canonical_predecessor_ideal, q
        )
    ):
        return record(
            laws.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1
        )
    raise DirectionBindingCertificateUnproven(BINDING_SUBTURN_LE_DELTA_MAX)


def turn_is_exactly_at_count_limit(
    sign: ExactTurnSignV1,
    cosine_squared: Fraction,
    count: int,
    q: int,
) -> bool:
    """Поворот угла равен РОВНО `(count + 1) * pi / q` по знаку и `cos^2`.

    `cos^2` одинаков у `theta` и `pi - theta`, поэтому равенства `cos^2` мало:
    знак `cos` отделяет сам предел от его зеркала. Порог `>= 1` — не предел, а
    потолок (поворот не превосходит `pi`), там осуществимо всегда.
    """

    from .angular import _compare_turn_cos_squared

    steps = count + 1
    if steps >= q:
        return False
    if _compare_turn_cos_squared(cosine_squared, Fraction(steps, q)) != 0:
        return False
    # Знак `cos(steps * pi / q)`: положителен до четверти оборота, нуль на ней,
    # отрицателен после — сравнением целых, без числовых порогов.
    quarter = 2 * steps - q
    if quarter < 0:
        return sign is ExactTurnSignV1.POSITIVE
    if quarter == 0:
        return sign is ExactTurnSignV1.ZERO
    return sign is ExactTurnSignV1.NEGATIVE


def verify_exact_limit_lift(
    context,
    spec,
    lift: EvaluationGeometrySubturnCountLiftV1,
) -> None:
    """Независимо проверить обе половины предела на предшествующем счёте.

    ОБЕ половины пересчитываются на веере предшествующего счёта, а не берутся
    из записи: остаток подшага ровно нуль (`_subturn_boundary`, тот же точный
    предикат, что у компиляции; память подшагов делает повтор дешёвым) и
    иррациональность луча, доказанная в точном поле. Знак и `cos^2` записи
    сверяются отдельно (общая часть `_verify_evaluation_subturn_count_lift`),
    здесь они читаются лишь как дешёвое предусловие. Веер строится СЫРЫМ
    рецептом без канонической ветви и без записи наблюдений контекста: проба не
    должна менять то, что видит настоящая спека.

    Монотонность закрывает остальное: исходный счёт не выше предшествующего, а
    у веера с `H' < H_предел` подшаг строго больше `pi/q`.
    """

    from .adaptive_density_fan import _covectors, _subturn_boundary
    from .angular import _incident_normal, _interpolated_normals
    from .direction_binding import density_support_direction_rationality

    predecessor = lift.minimality_predecessor_hidden_edge_count
    if lift.source_hidden_edge_count > predecessor:
        raise ValueError("exact-limit lift source count exceeds its predecessor")
    witness = Fraction(
        lift.evaluation_turn_cosine_squared.numerator,
        lift.evaluation_turn_cosine_squared.denominator,
    )
    if not turn_is_exactly_at_count_limit(
        lift.evaluation_turn_sign,
        witness,
        predecessor,
        lift.max_subturn_q,
    ):
        raise ValueError(
            "predecessor count is not exactly at the subturn limit"
        )
    relation = next(
        item
        for item in context.snapshot.corner_relations
        if item.corner_relation_id == spec.source_relation_id
    )
    sector = next(
        item
        for item in context.snapshot.angular_owner_sectors
        if item.owner_sector_id == spec.owner_sector_id
    )
    incoming, _ = _incident_normal(
        context,
        sector.ordered_incident_chain_use_ids[0],
        relation.source_vertex_id,
    )
    outgoing, _ = _incident_normal(
        context,
        sector.ordered_incident_chain_use_ids[-1],
        relation.source_vertex_id,
    )
    ideal = _interpolated_normals(
        context.metric,
        incoming,
        outgoing,
        predecessor,
        sector.turn_orientation,
        huber_density=True,
    )
    covectors = _covectors(context.metric, ideal)
    if not _subturn_boundary(
        context.metric, covectors[0], covectors[1], lift.max_subturn_q
    ):
        raise ValueError(
            "predecessor fan subturn residual is not exactly zero"
        )
    if not any(
        density_support_direction_rationality(context.metric, ideal[index])
        is False
        for index in range(1, predecessor + 1)
    ):
        raise ValueError(
            "predecessor fan at the subturn limit has no provably irrational "
            "hidden direction"
        )
