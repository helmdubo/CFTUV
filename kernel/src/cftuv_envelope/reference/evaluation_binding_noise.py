"""Шум привязки к решётке на каноническом угле: счёт решает канон, а не знак шума.

ПОЛЕВОЙ ДЕФЕКТ (`artifacts/fan_consistency`, замер 2026-10-03). Прямой
вогнутый угол с точной долей `1/2` селектор видит как точный канон. На чётном
`q` канонический подшаг `pi/(2(H+1))` стоит РОВНО на `pi/q` (d2: H=1, d4: H=2),
и гарантия «шаг <= pi/q» проверялась на вычислительной геометрии — вершинах,
привязанных к решётке в косой карте. Шум привязки там ±(1e-5..3e-4) рад с
произвольным знаком, а точный прямой угол сидит на самой границе: знак шума
решал счёт. Одинаковые углы одной стены получали разные веера (d2: H=1 либо 2,
шаги 45/45 либо 26/34/30; d4: H=2 либо 3).

ЗАКОН `EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1` (решение владельца:
визуальная согласованность канонических углов важнее строгой гарантии шага на
шуме привязки). Когда селектор видит РОВНО канонический интервал — сырой точный
либо восстановленный, — счёт на тугом пороге (`u*q == H+1`) решается на
КАНОНИЧЕСКОМ веере, а не на вычислительном:

* канонический веер осуществим (лучи рациональны, d2): счёт прежний; если веер
  вычислительной геометрии при этом держал бы подшаг больше `pi/q` на шум, лучи
  ставятся точными поворотами на канонический подшаг (исход
  `CANONICAL_ROTATION_FAN`), и шум остаётся в последнем секторе;
* канонический веер на пределе с иррациональным лучом (d4): счёт поднят на
  единицу независимо от знака шума (исход `CANONICAL_COUNT_LIFT`, закон лифта
  `..._LIFTED_AT_CANONICAL_EXACT_LIMIT_V1`).

Это ЭВРИСТИКА, МЕНЯЮЩАЯ ОТВЕТ, поэтому она не молчит (п. 4 `AGENTS.md`): запись
`EvaluationBindingNoiseOnCanonicalAngleV1` несёт точные знак и `cos^2` шума
(поворот между опорами в вычислительной геометрии) и точную границу смещений
привязки по вершинам угла. Нет допуска и нет float.

ГРАНИЦА ПРИМЕНИМОСТИ — две, обе точные. (1) Структурная: БОКОВОЕ смещение
привязки каждого из двух рёбер угла укладывается в ОДНУ ячейку решётки (квадрат
боковой Грам-нормы не больше квадрата Грам-нормы диагонали ячейки базовой
решётки). Боковое, а не полное: продольный сдвиг вершины вдоль цепи (внутренние
вершины прямой цепи V2 садятся на уточнённую решётку и скользят вдоль неё)
направления ребра не меняет. (2) УГЛОВАЯ: сдвиг в одну ячейку угол не
ограничивает (короткое ребро при малом сдвиге даёт большой поворот), поэтому
sin^2 поворота направления каждого ребра и cos^2 поворота между опорами (для
прямого угла это sin^2 отклонения от 90 градусов, то есть превышение
последнего сектора над `pi/q`) не превосходят квадрата объявленного синуса
`NOISE_DIRECTION_SINE_BOUND` (он же в реестре допусков). Шум, который границы не
объясняют, законом не покрывается: ответ решает прежний закон на
вычислительной геометрии, байты прежние, но МОЛЧА он не решает — отказ закона
называет причину (`EvaluationBindingNoiseRefusalV1`) в диагностике ядра и в
счётчике `CONVEYOR_BINDING_NOISE_LAW_REFUSED`.

НЕЗАВИСИМЫЙ ПРОВЕРЯЮЩИЙ пересчитывает всё по сырой геометрии (интервал угла
снапшота, исходные и привязанные координаты, опоры): запись — предмет проверки,
а не источник истины. Обратное тоже проверяется: счёт, который канонический
веер обязан был поднять, не может остаться прежним.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from .._canonical_angle import (
    canonical_count_is_tight,
    canonical_rotation_denominator,
    canonical_subturn_is_within_max_subturn,
    exact_canonical_selector_fact,
)
from .._density_policy import huber_density_value_contract
from ..contracts.envelopes import (
    AngularEnvelopeSpec,
    EvaluationGeometrySubturnCountLiftLawV1,
)
from ..contracts.metric import ExactRationalV1
from ..contracts.plan import ChainStraightEvaluationGeometryBindingV2
from ..numeric import ExactRatioV1
from .contracts import (
    EvaluationBindingNoiseEffectV1,
    EvaluationBindingNoiseLawV1,
    EvaluationBindingNoiseOnCanonicalAngleV1,
    EvaluationBindingNoiseRefusalV1,
    ReferenceDiagnosticSeverity,
    ReferenceEvaluationDiagnosticV1,
    ReferenceOutcome,
)

NOISE_LAW = EvaluationBindingNoiseLawV1.EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1
CANONICAL_LIFT_LAW = (
    EvaluationGeometrySubturnCountLiftLawV1.EVALUATION_GEOMETRY_SUBTURN_COUNT_LIFTED_AT_CANONICAL_EXACT_LIMIT_V1
)

#: Объявленная граница синуса углового шума: поворот направления ребра при
#: привязке и отклонение поворота между опорами от канона не больше `1/400`
#: (около 0.143 градуса). Граница накрывает ДОПУСК ВОССТАНОВЛЕНИЯ (синус 0.1
#: градуса — 1.745e-3: восстановленный угол отклоняется от канона на столько же в
#: самой геометрии меша) плюс шум привязки к решётке (поле: до 2.26e-4 по ребру
#: и 2.89e-4 по повороту на 227 канонических углах d2). До решения владельца
#: 2026-10-03 (RIGHT-ANGLE-STABLE) допуск восстановления был 7e-6 рад, граница —
#: `1/1000`. Одно число и одно место; в реестре допусков —
#: `EVALUATION_BINDING_NOISE_ON_CANONICAL_ANGLE_V1`.
NOISE_DIRECTION_SINE_BOUND = Fraction(1, 400)

_COMMON_PREDICATES = frozenset(
    {
        "SELECTOR_INTERVAL_IS_EXACTLY_CANONICAL",
        "EVALUATION_TURN_SIGN_AND_COSINE_SQUARED_ARE_EXACT",
        "BINDING_LATERAL_OFFSET_BOUND_IS_EXACT_AND_WITHIN_ONE_LATTICE_CELL",
        "EDGE_DIRECTION_NOISE_IS_EXACT_AND_WITHIN_THE_DECLARED_SINE_BOUND",
        "TURN_NOISE_IS_WITHIN_THE_DECLARED_SINE_BOUND",
    }
)

NOISE_PREDICATES = {
    EvaluationBindingNoiseEffectV1.CANONICAL_COUNT_LIFT: _COMMON_PREDICATES
    | {
        "CANONICAL_PREDECESSOR_COUNT_IS_EXACTLY_AT_SUBTURN_LIMIT",
        "CANONICAL_PREDECESSOR_FAN_HAS_IRRATIONAL_HIDDEN_DIRECTION",
    },
    EvaluationBindingNoiseEffectV1.CANONICAL_ROTATION_FAN: _COMMON_PREDICATES
    | {
        "EVALUATION_EQUAL_SUBTURN_FAN_VIOLATES_GUARANTEE_ON_EVALUATION_SUPPORTS",
        "CANONICAL_SUBTURN_IS_EXACTLY_AT_MAX_SUBTURN_WITH_RATIONAL_RAYS",
        "CANONICAL_RAYS_ARE_EXACT_ORDINAL_ROTATIONS_OF_THE_INCOMING_SUPPORT",
    },
}


@dataclass(frozen=True, slots=True)
class CanonicalNoiseFact:
    """Что закон видит в углу: канон и ТОЧНЫЕ границы шума привязки."""

    relation: object
    canonical: Fraction
    lateral_offset_gram_squared_bound: Fraction
    edge_direction_sine_squared_bound: Fraction
    turn_cosine_squared: Fraction


@dataclass(frozen=True, slots=True)
class NoiseApplicability:
    """Исход проверки применимости: факт закона либо ИМЕНОВАННАЯ причина молчания.

    `canonical is None` — селектор видит сырое число, закон не про этот угол.
    `canonical` есть, а `fact` и `refusal` пусты — привязки нет (вычислительная
    геометрия равна исходной), шума нет, молчать закону нечем. `refusal` — закон
    не применён по названной причине, и счёт на тугом пороге решит знак шума.
    """

    canonical: tuple | None
    fact: CanonicalNoiseFact | None
    refusal: EvaluationBindingNoiseRefusalV1 | None


@dataclass(frozen=True, slots=True)
class _BindingNoise:
    lateral_squared: Fraction
    cell_bound: Fraction
    edge_sine_squared: Fraction
    turn_cosine_squared: Fraction


def _fraction(value) -> Fraction:
    return Fraction(value.numerator, value.denominator)


def _rational(value: Fraction) -> ExactRationalV1:
    return ExactRationalV1(value.numerator, value.denominator)


def _corner_ids(context, spec):
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
    return relation, sector


def _coordinates(records) -> dict:
    return {
        item.source_vertex_id: (
            _fraction(item.domain_coordinate.x),
            _fraction(item.domain_coordinate.y),
        )
        for item in records
    }


def _gram(frame):
    matrix = frame.exact_gram_matrix
    return (
        _fraction(matrix.m00),
        _fraction(matrix.m01),
        _fraction(matrix.m11),
    )


def _gram_squared(gram, vector) -> Fraction:
    return _gram_dot(gram, vector, vector)


def _gram_dot(gram, left, right) -> Fraction:
    g00, g01, g11 = gram
    return (
        g00 * left[0] * right[0]
        + g01 * (left[0] * right[1] + left[1] * right[0])
        + g11 * left[1] * right[1]
    )


def _lattice_cell_bound(binding, gram) -> Fraction:
    """Квадрат Грам-нормы диагонали ячейки БАЗОВОЙ решётки привязки.

    Базовая, а не уточнённая: внутренние вершины прямой цепи V2 садятся на
    уточнённую решётку, но вершины угла привязаны к базовой.
    """

    scale = (
        binding.base_lattice_scale
        if isinstance(binding, ChainStraightEvaluationGeometryBindingV2)
        else binding.lattice_scale
    )
    g00, g01, g11 = gram
    return (g00 + 2 * abs(g01) + g11) / (scale * scale)


def _corner_vertices(context, relation, sector):
    """`(предыдущая, вершина, следующая)` по ходу владельца или `None`."""

    incoming = context.directed_chain_vertices(
        context.uses_by_id[sector.ordered_incident_chain_use_ids[0]]
    )
    outgoing = context.directed_chain_vertices(
        context.uses_by_id[sector.ordered_incident_chain_use_ids[-1]]
    )
    vertex = relation.source_vertex_id
    if len(incoming) < 2 or len(outgoing) < 2:
        return None
    if incoming[-1] != vertex or outgoing[0] != vertex:
        return None
    return incoming[-2], vertex, outgoing[1]


def _edge_noise(gram, source, evaluation, start, end):
    """`(боковой сдвиг^2, sin^2 поворота направления, вычислительное ребро)` либо `None`.

    Боковая компонента сдвига конца ребра относительно начала —
    `|cross(edge, shift)| / |edge|` в метрике; квадрат площади в метрике — это
    `det(G) * cross^2`, поэтому всё считается в рациональных числах. sin^2
    поворота направления — `det(G) * cross^2 / (|edge|^2 * |edge'|^2)` с
    вычислительным ребром `edge'`: ТОЧНОЕ значение, а не оценка `lateral/|edge|`.
    Продольный сдвиг направления не меняет и в обе величины не входит.
    """

    g00, g01, g11 = gram
    edge = (
        source[end][0] - source[start][0],
        source[end][1] - source[start][1],
    )
    moved = (
        evaluation[end][0] - evaluation[start][0],
        evaluation[end][1] - evaluation[start][1],
    )
    length_squared = _gram_squared(gram, edge)
    moved_squared = _gram_squared(gram, moved)
    if length_squared == 0 or moved_squared == 0:
        return None
    cross = edge[0] * moved[1] - edge[1] * moved[0]
    area_squared = (g00 * g11 - g01 * g01) * cross * cross
    return (
        area_squared / length_squared,
        area_squared / (length_squared * moved_squared),
        moved,
    )


def _binding_noise(context, spec):
    """`_BindingNoise`, `None` (привязки нет — шума нет) либо причина молчания."""

    refusals = EvaluationBindingNoiseRefusalV1
    binding = context.compilation.evaluation_geometry_binding
    if binding is None:
        return None
    frame = context.frame
    source_records = getattr(frame, "exact_source_vertex_coordinates", None)
    if source_records is None:
        return refusals.SOURCE_COORDINATES_UNAVAILABLE
    relation, sector = _corner_ids(context, spec)
    ids = _corner_vertices(context, relation, sector)
    if ids is None:
        return refusals.CORNER_VERTICES_UNAVAILABLE
    cache = context.evaluation_noise_cache
    if "coordinates" not in cache:
        cache["coordinates"] = (
            _coordinates(source_records),
            _coordinates(binding.source_vertex_coordinates),
        )
    source, evaluation = cache["coordinates"]
    if any(vertex not in source or vertex not in evaluation for vertex in ids):
        return refusals.CORNER_VERTICES_UNAVAILABLE
    gram = _gram(frame)
    edges = tuple(
        _edge_noise(gram, source, evaluation, start, end)
        for start, end in zip(ids, ids[1:])
    )
    if None in edges:
        return refusals.DEGENERATE_CORNER_EDGE
    first, second = (item[2] for item in edges)
    dot = _gram_dot(gram, first, second)
    return _BindingNoise(
        max(item[0] for item in edges),
        _lattice_cell_bound(binding, gram),
        max(item[1] for item in edges),
        dot * dot / (_gram_squared(gram, first) * _gram_squared(gram, second)),
    )


def _selector_interval(context, selection):
    angle = next(
        (
            item
            for item in context.snapshot.reflex_angle_certificates
            if item.certificate_id == selection.reflex_angle_certificate_id
        ),
        None,
    )
    return getattr(
        getattr(angle, "measure_payload", None), "reflex_excess_over_pi", None
    )


def _noise_refusal(noise, canonical):
    """Первая нарушенная граница применимости либо `None`.

    Угловые условия ТОЧНЫЕ и объявлены одним числом `NOISE_DIRECTION_SINE_BOUND`
    (реестр допусков): боковой сдвиг в одну ячейку угол НЕ ограничивает —
    короткое ребро при малом сдвиге даёт большой поворот, — поэтому закон
    требует отдельно, чтобы поворот направления каждого из двух рёбер и
    отклонение поворота между опорами от канонического прямого угла не
    превосходили объявленного синуса. Для прямого угла `cos^2` поворота и есть
    `sin^2` отклонения, а отклонение равно превышению последнего сектора
    канонического веера над `pi/q` (первые `H` секторов — точные повороты).
    """

    refusals = EvaluationBindingNoiseRefusalV1
    # `u = 1/2`: у отношения без углового предела закон не определён, а не «примерно».
    if canonical[1] * 2 != 1:
        return refusals.CANONICAL_RELATION_HAS_NO_ANGULAR_BOUND
    bound_squared = NOISE_DIRECTION_SINE_BOUND**2
    if noise.lateral_squared > noise.cell_bound:
        return refusals.LATERAL_SHIFT_EXCEEDS_ONE_LATTICE_CELL
    if noise.edge_sine_squared > bound_squared:
        return refusals.EDGE_DIRECTION_NOISE_EXCEEDS_DECLARED_BOUND
    if noise.turn_cosine_squared > bound_squared:
        return refusals.TURN_NOISE_EXCEEDS_DECLARED_BOUND
    return None


def canonical_noise_applicability(context, spec, selection) -> NoiseApplicability:
    """Применим ли закон к углу, и если нет — по какой названной причине.

    Исход кэшируется по спеке: ни интервал, ни геометрия контекста не меняются.
    Отказ закона — НЕ ошибка: ответ тогда решает прежний закон на
    вычислительной геометрии, байт в байт, но молча он не решает (диагностика и
    счётчик `CONVEYOR_BINDING_NOISE_LAW_REFUSED`).
    """

    key = ("applicability", spec.envelope_spec_id.value)
    cache = context.evaluation_noise_cache
    if key in cache:
        return cache[key]
    interval = _selector_interval(context, selection)
    canonical = (
        None if interval is None else exact_canonical_selector_fact(interval)
    )
    result = NoiseApplicability(canonical, None, None)
    if canonical is not None:
        noise = _binding_noise(context, spec)
        if isinstance(noise, EvaluationBindingNoiseRefusalV1):
            result = NoiseApplicability(canonical, None, noise)
        elif noise is not None:
            refusal = _noise_refusal(noise, canonical)
            result = NoiseApplicability(
                canonical,
                None
                if refusal is not None
                else CanonicalNoiseFact(
                    canonical[0],
                    canonical[1],
                    noise.lateral_squared,
                    noise.edge_sine_squared,
                    noise.turn_cosine_squared,
                ),
                refusal,
            )
    cache[key] = result
    return result


def canonical_noise_fact(context, spec, selection) -> CanonicalNoiseFact | None:
    """Закон применим к углу: канон точен, шум привязки внутри объявленных границ.

    `None` — закон молчит, и ответ решает прежний закон на вычислительной
    геометрии, байт в байт; причину называет `canonical_noise_applicability`.
    """

    return canonical_noise_applicability(context, spec, selection).fact


def canonical_ideal(context, spec, count: int, canonical: Fraction):
    """Канонический веер на опорах вычислительной геометрии: точные повороты."""

    key = ("ideal", spec.envelope_spec_id.value, count, canonical)
    cache = context.evaluation_noise_cache
    if key not in cache:
        from .angular import _incident_normal, _interpolated_normals

        relation, sector = _corner_ids(context, spec)
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
        cache[key] = _interpolated_normals(
            context.metric,
            incoming,
            outgoing,
            count,
            sector.turn_orientation,
            huber_density=True,
            canonical_excess_over_pi=canonical,
        )
    return cache[key]


def canonical_predecessor_ideal(context, spec, selection, count: int, q: int):
    """Канонический веер счёта `count`, если закон применим и счёт тугой."""

    fact = canonical_noise_fact(context, spec, selection)
    if fact is None or not canonical_count_is_tight(fact.canonical, count, q):
        return None
    return canonical_ideal(context, spec, count, fact.canonical)


def canonical_sectors_hold(context, ideal, q: int) -> bool:
    """Каждый сектор канонического веера, кроме последнего, не шире `pi/q` — точно.

    Канонический веер НЕ равношаговый: `ideal[-1]` — опора вычислительной
    геометрии, а не канонический конец, поэтому проверка первой пары (как у
    `_density_ideal_is_subturn_feasible`) последнего сектора не видит. Первые `H`
    секторов — точные повороты и проверяются здесь все по очереди; последний
    сектор равен `pi/q` плюс отклонение поворота от канона, и его верхнюю границу
    даёт точное угловое условие закона (`cos^2` поворота не больше квадрата
    объявленного синуса): условие закона, а не допущение.
    """

    from .adaptive_density_fan import _covectors, _subturn
    from .angular import canonical_sector_over_pi

    # Все `H` секторов — один и тот же точный поворот `u * pi / (H + 1)`: условие каждого —
    # `u / (H + 1) <= 1 / q`, и повторять его для каждой пары незачем.
    sector = canonical_sector_over_pi(context.metric, ideal)
    if sector is not None:
        return sector <= Fraction(1, q)
    covectors = _covectors(context.metric, ideal)
    return all(
        _subturn(context.metric, covectors[index], covectors[index + 1], q)
        for index in range(len(covectors) - 2)
    )


def canonical_count_is_feasible(context, ideal, q: int) -> bool:
    """Осуществим ли канонический веер: все его секторы и не иррациональный предел."""

    from .subturn_exact_limit import density_count_is_feasible

    return canonical_sectors_hold(context, ideal, q) and density_count_is_feasible(
        context.metric, ideal, q
    )


def evaluation_count_is_feasible(context, spec, selection, count, ideal, q) -> bool:
    """Осуществим ли счёт: на тугом пороге решает канонический веер, иначе прежнее."""

    from .subturn_exact_limit import density_count_is_feasible

    canonical = canonical_predecessor_ideal(context, spec, selection, count, q)
    if canonical is not None:
        return canonical_count_is_feasible(context, canonical, q)
    return density_count_is_feasible(context.metric, ideal, q)


def canonical_fan_rays_are_rational(context, spec, count, canonical) -> bool:
    from .direction_binding import has_rational_density_support_direction

    ideal = canonical_ideal(context, spec, count, canonical)
    return all(
        has_rational_density_support_direction(context.metric, ray)
        for ray in ideal[1:-1]
    )


def noise_record_for(context, spec):
    return next(
        (
            item
            for item in context.compilation.evaluation_binding_noise_records
            if item.envelope_spec_id == spec.envelope_spec_id
        ),
        None,
    )


def _turn_witness(context, ideal):
    from .compile import _exact_turn_witness

    return _exact_turn_witness(context.metric, ideal)


def build_noise_record(context, spec, selection, effect, ideal):
    """Запись закона по ФАКТАМ геометрии контекста, а не по решению компилятора."""

    fact = canonical_noise_fact(context, spec, selection)
    sign, cosine_squared = _turn_witness(context, ideal)
    contract = huber_density_value_contract(selection.max_subturn_value_id)
    return EvaluationBindingNoiseOnCanonicalAngleV1(
        noise_law=NOISE_LAW,
        effect=effect,
        envelope_spec_id=spec.envelope_spec_id,
        selection_certificate_id=selection.certificate_id,
        canonical_relation=fact.relation,
        canonical_reflex_excess_over_pi=ExactRatioV1(
            fact.canonical.numerator, fact.canonical.denominator
        ),
        source_hidden_edge_count=selection.resolved_hidden_edge_count,
        effective_hidden_edge_count=spec.resolved_hidden_edge_count,
        max_subturn_q=contract[0],
        evaluation_turn_sign=sign,
        evaluation_turn_cosine_squared=cosine_squared,
        binding_lateral_offset_gram_squared_bound=_rational(
            fact.lateral_offset_gram_squared_bound
        ),
        edge_direction_sine_squared_bound=_rational(
            fact.edge_direction_sine_squared_bound
        ),
        direction_sine_bound=_rational(NOISE_DIRECTION_SINE_BOUND),
        proven_predicates=NOISE_PREDICATES[effect],
    )


def evaluation_binding_noise_records(compilation, context, specs) -> frozenset:
    """Записать закон шума привязки там, где он изменил ответ.

    Исход снимается с итоговой спеки, а не с решения компилятора: лифт под
    названным законом — исход `CANONICAL_COUNT_LIFT`, канонический веер сырого
    точного угла (власти восстановления у него нет) — `CANONICAL_ROTATION_FAN`.
    """

    from .angular import _ideal_angular_support_data

    effects = EvaluationBindingNoiseEffectV1
    restorations = {
        item.selection_certificate_id
        for item in compilation.canonical_angle_restorations
    }
    records = set()
    for spec in specs:
        if not isinstance(spec, AngularEnvelopeSpec):
            continue
        lift = getattr(spec, "evaluation_subturn_count_lift", None)
        if lift is not None and lift.lift_law is CANONICAL_LIFT_LAW:
            effect = effects.CANONICAL_COUNT_LIFT
        elif (
            context.canonical_subturn_fan.get(spec.envelope_spec_id.value)
            and spec.selection_certificate_id not in restorations
        ):
            effect = effects.CANONICAL_ROTATION_FAN
        else:
            continue
        selection = next(
            item
            for item in compilation.profile_selection_certificates
            if item.certificate_id == spec.selection_certificate_id
        )
        *_, ideal = _ideal_angular_support_data(context, spec)
        records.add(build_noise_record(context, spec, selection, effect, ideal))
    return frozenset(records)


def evaluation_binding_noise_diagnostics(compilation, context, specs) -> tuple:
    """Именованный отказ закона там, где счёт на тугом пороге решил знак шума.

    Только тугой счёт селекции (`u*q == H+1`): вне его закон ничего не менял бы,
    и отказ молчанием не является. Привязки нет — шума нет, и отказа тоже нет.
    """

    diagnostics = []
    for spec in sorted(
        (item for item in specs if isinstance(item, AngularEnvelopeSpec)),
        key=lambda item: item.envelope_spec_id.value,
    ):
        selection = next(
            item
            for item in compilation.profile_selection_certificates
            if item.certificate_id == spec.selection_certificate_id
        )
        contract = huber_density_value_contract(selection.max_subturn_value_id)
        if contract is None:
            continue
        applicability = canonical_noise_applicability(context, spec, selection)
        if applicability.refusal is None or applicability.canonical is None:
            continue
        if not canonical_count_is_tight(
            applicability.canonical[1],
            selection.resolved_hidden_edge_count,
            contract[0],
        ):
            continue
        diagnostics.append(
            ReferenceEvaluationDiagnosticV1(
                outcome=ReferenceOutcome.EVALUATION_BINDING_NOISE_LAW_NOT_APPLIED,
                severity=ReferenceDiagnosticSeverity.INFO,
                message=(
                    f"{applicability.refusal.value}: the count of a canonical "
                    "angle at the exact subturn limit is decided by the sign "
                    "of the evaluation binding noise"
                ),
                envelope_spec_id=spec.envelope_spec_id.value,
            )
        )
    return tuple(diagnostics)


def noise_record_error(record, context, spec, selection, ideal, effect) -> str | None:
    """Пересчитать запись по сырой геометрии и назвать первое расхождение."""

    if type(record) is not EvaluationBindingNoiseOnCanonicalAngleV1:
        return "evaluation binding noise record has a foreign type"
    if record.noise_law is not NOISE_LAW:
        return "evaluation binding noise record names another law"
    if record.effect is not effect:
        return "evaluation binding noise record names another effect"
    if record.proven_predicates != NOISE_PREDICATES[effect]:
        return "evaluation binding noise predicate set is not the declared one"
    fact = canonical_noise_fact(context, spec, selection)
    if fact is None:
        return (
            "evaluation binding noise law does not apply: the selector "
            "interval is not exactly canonical or the binding offsets leave "
            "one lattice cell"
        )
    contract = huber_density_value_contract(selection.max_subturn_value_id)
    sign, cosine_squared = _turn_witness(context, ideal)
    expected = EvaluationBindingNoiseOnCanonicalAngleV1(
        noise_law=NOISE_LAW,
        effect=effect,
        envelope_spec_id=spec.envelope_spec_id,
        selection_certificate_id=selection.certificate_id,
        canonical_relation=fact.relation,
        canonical_reflex_excess_over_pi=ExactRatioV1(
            fact.canonical.numerator, fact.canonical.denominator
        ),
        source_hidden_edge_count=selection.resolved_hidden_edge_count,
        effective_hidden_edge_count=spec.resolved_hidden_edge_count,
        max_subturn_q=contract[0],
        evaluation_turn_sign=sign,
        evaluation_turn_cosine_squared=cosine_squared,
        binding_lateral_offset_gram_squared_bound=_rational(
            fact.lateral_offset_gram_squared_bound
        ),
        edge_direction_sine_squared_bound=_rational(
            fact.edge_direction_sine_squared_bound
        ),
        direction_sine_bound=_rational(NOISE_DIRECTION_SINE_BOUND),
        proven_predicates=NOISE_PREDICATES[effect],
    )
    if record != expected:
        return "evaluation binding noise record does not follow from the geometry"
    return None


def canonical_fan_noise_error(context, spec, selection, ideal) -> str | None:
    """Канонический веер при шуме привязки: запись есть и честна, веер осуществим."""

    record = noise_record_for(context, spec)
    if record is None:
        return (
            "canonical subturn fan is in force without its recorded evaluation "
            "binding noise"
        )
    error = noise_record_error(
        record,
        context,
        spec,
        selection,
        ideal,
        EvaluationBindingNoiseEffectV1.CANONICAL_ROTATION_FAN,
    )
    if error is not None:
        return error
    if record.effective_hidden_edge_count != record.source_hidden_edge_count:
        return "canonical rotation fan changes the selector count"
    canonical = _fraction(record.canonical_reflex_excess_over_pi)
    count = spec.resolved_hidden_edge_count
    if not canonical_count_is_tight(canonical, count, record.max_subturn_q):
        return "canonical rotation fan stands on a count that is not at the limit"
    denominator = canonical_rotation_denominator(canonical, count)
    if denominator is None or denominator < record.max_subturn_q:
        return "canonical subturn is not an exact rotation within pi/q"
    if not canonical_subturn_is_within_max_subturn(
        canonical, count, record.max_subturn_q
    ):
        return "canonical subturn exceeds the declared maximum subturn"
    if not canonical_fan_rays_are_rational(context, spec, count, canonical):
        return "canonical rotation fan has an irrational ray"
    if not canonical_sectors_hold(
        context, canonical_ideal(context, spec, count, canonical), record.max_subturn_q
    ):
        return "canonical rotation fan has a sector wider than pi/q"
    return None


def verify_canonical_exact_limit_lift(context, spec, lift) -> None:
    """Независимо проверить лифт на пределе канонического веера.

    Пересчитывается по геометрии, не по записи: канон из интервала угла,
    тугость предшествующего счёта, иррациональность луча канонического веера,
    точные знак и `cos^2` шума и граница смещений привязки.
    """

    from .subturn_exact_limit import ideal_is_exact_limit_with_irrational_direction

    selection = next(
        item
        for item in context.compilation.profile_selection_certificates
        if item.certificate_id == spec.selection_certificate_id
    )
    predecessor = lift.minimality_predecessor_hidden_edge_count
    if lift.source_hidden_edge_count > predecessor:
        raise ValueError("canonical-limit lift source count exceeds its predecessor")
    fact = canonical_noise_fact(context, spec, selection)
    if fact is None:
        raise ValueError(
            "selector interval is not exactly canonical or the binding "
            "offsets leave one lattice cell"
        )
    if not canonical_count_is_tight(fact.canonical, predecessor, lift.max_subturn_q):
        raise ValueError(
            "predecessor count is not exactly at the canonical subturn limit"
        )
    ideal = canonical_ideal(context, spec, predecessor, fact.canonical)
    if not ideal_is_exact_limit_with_irrational_direction(
        context.metric, ideal, lift.max_subturn_q
    ):
        raise ValueError(
            "predecessor canonical fan has no provably irrational hidden direction"
        )
    record = noise_record_for(context, spec)
    if record is None:
        raise ValueError("canonical-limit lift has no recorded binding noise")
    from .angular import _ideal_angular_support_data

    *_, effective_ideal = _ideal_angular_support_data(context, spec)
    error = noise_record_error(
        record,
        context,
        spec,
        selection,
        effective_ideal,
        EvaluationBindingNoiseEffectV1.CANONICAL_COUNT_LIFT,
    )
    if error is not None:
        raise ValueError(error)
    if (
        record.evaluation_turn_sign is not lift.evaluation_turn_sign
        or record.evaluation_turn_cosine_squared != lift.evaluation_turn_cosine_squared
    ):
        raise ValueError("lift and noise record cite different turn witnesses")


def canonical_count_law_error(context, spec, selection) -> str | None:
    """Обратная сторона закона: счёт, который канон обязан был поднять.

    Спека без лифта (или лифтованная не выше счёта селекции) на тугом пороге
    с иррациональным лучом канонического веера — это счёт, который решил знак
    шума привязки, а не канон. Проверка не зависит от того, какая запись
    приложена: она читает веер.
    """

    fact = canonical_noise_fact(context, spec, selection)
    if fact is None:
        return None
    contract = huber_density_value_contract(selection.max_subturn_value_id)
    source = selection.resolved_hidden_edge_count
    if contract is None or spec.resolved_hidden_edge_count > source:
        return None
    if not canonical_count_is_tight(fact.canonical, source, contract[0]):
        return None
    ideal = canonical_ideal(context, spec, source, fact.canonical)
    if canonical_count_is_feasible(context, ideal, contract[0]):
        return None
    return (
        "canonical exact-limit count is not lifted: the sign of the "
        "evaluation binding noise decided it"
    )


def noise_records_error(context) -> str | None:
    """Записи закона и спеки — взаимно однозначны, и каждая запись честна по исходу."""

    records = context.compilation.evaluation_binding_noise_records
    specs = {
        item.envelope_spec_id: item
        for item in context.compilation.envelope_specs
        if isinstance(item, AngularEnvelopeSpec)
    }
    seen = set()
    for record in records:
        if record.envelope_spec_id in seen:
            return "two evaluation binding noise records for one spec"
        seen.add(record.envelope_spec_id)
        spec = specs.get(record.envelope_spec_id)
        if spec is None:
            return "evaluation binding noise record cites an unknown spec"
        lift = getattr(spec, "evaluation_subturn_count_lift", None)
        lifted = lift is not None and lift.lift_law is CANONICAL_LIFT_LAW
        in_force = bool(
            context.canonical_subturn_fan.get(spec.envelope_spec_id.value)
        )
        if record.effect is EvaluationBindingNoiseEffectV1.CANONICAL_COUNT_LIFT:
            if not lifted:
                return "canonical-limit noise record has no canonical-limit lift"
        elif not in_force or lifted:
            return "canonical-fan noise record stands on a fan that is not in force"
    for spec in specs.values():
        lift = getattr(spec, "evaluation_subturn_count_lift", None)
        if (
            lift is not None
            and lift.lift_law is CANONICAL_LIFT_LAW
            and spec.envelope_spec_id not in seen
        ):
            return "canonical-limit lift has no recorded binding noise"
    return None


__all__ = (
    "CANONICAL_LIFT_LAW",
    "NOISE_DIRECTION_SINE_BOUND",
    "NOISE_LAW",
    "NOISE_PREDICATES",
    "CanonicalNoiseFact",
    "build_noise_record",
    "canonical_count_law_error",
    "canonical_fan_noise_error",
    "canonical_fan_rays_are_rational",
    "canonical_count_is_feasible",
    "canonical_ideal",
    "canonical_noise_applicability",
    "canonical_noise_fact",
    "canonical_sectors_hold",
    "canonical_predecessor_ideal",
    "evaluation_binding_noise_diagnostics",
    "evaluation_binding_noise_records",
    "evaluation_count_is_feasible",
    "noise_record_error",
    "noise_record_for",
    "noise_records_error",
    "verify_canonical_exact_limit_lift",
)
