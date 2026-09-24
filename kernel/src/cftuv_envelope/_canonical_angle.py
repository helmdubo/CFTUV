"""Восстановление канонического авторского угла ДО селектора плотности.

Задача, оплаченная полем. Граница счётной ячейки плотности стоит РОВНО на
каноническом отношении: при чётном `q` (d0, d2, d4) точка `u = δ/π = 1/2` —
прямой угол, интерьер 270° — является верхней CLOSED-границей ячейки. Меш
владельца несёт по этому месту шум моделирования ±(6e-5..2e-4)°, то есть
1.1e-6..2.9e-6 рад (слепок `artifacts/field_snapshots/wall_2_001_corner_angles.json`,
вершины 6 и 28 чуть больше 90°, вершина 27 чуть меньше, остальные 25 точны).
Угол чуть больше 90° уходит в следующую ячейку, и на «одинаковых» углах одной
стены счёт скрытых рёбер расходится.

ЧТО ИМЕННО ЗДЕСЬ ПРОИСХОДИТ, и чего здесь НЕ происходит.

* Происходит: восстановление ЗАДУМАННОГО отношения. Если авторский угол
  отклоняется от канонического в пределах уже объявленного допуска намерения
  `_authoring_intent.AUTHOR_ANGULAR_ERROR` (7e-6 рад), селектор получает
  СИМВОЛИЧЕСКИЙ канонический факт (`CANONICAL_REFLEX_EXCESS_PI_OVER_2`), а не
  сырое число. Это тот же допуск и та же категория (AUTHORING_INTENT), что и
  у привязки источника к решётке: там восстанавливается задуманное ПОЛОЖЕНИЕ
  вершины, здесь — задуманное ОТНОШЕНИЕ сторон.

* НЕ происходит: закон счёта не меняется ни на йоту. Жёсткая гарантия
  максимума (`MIN_K/ceil`, подшаг ≤ π/q) остаётся прежней властью и
  применяется к канонизированному факту так же, как раньше к сырому.
  Границы ячеек не двигаются (это отвергнутый путь `exec/fan-cell-midpoint`).

* НЕ происходит: молчаливого округления. Всякое восстановление несёт
  сертификат `CanonicalAngleRestorationCertificateV1` — какой угол, какое
  отклонение, какой канон применён. Точный угол сертификата не получает
  (восстанавливать нечего), угол вне допуска — тоже (восстанавливать не
  дозволено).

ПОЧЕМУ ПОРОГА В ГЕОМЕТРИИ НЕТ. Эпсилон применяется РОВНО ОДИН РАЗ, у двери
селектора, к классификации намерения. Ниже по конвейеру решения принимаются на
точной рациональной доле π, а не на интервале с допуском: лифт в метрике карты
берёт исходный счёт из сертификата селекции, то есть из канонического факта, и
дайджесты идут оттуда же.

ГДЕ КАНОНИЗАЦИЯ ЗАКАНЧИВАЕТСЯ — решением владельца ОНА НЕ ЗАКАНЧИВАЕТСЯ НА
СЧЁТЕ. Гарантия подшага держится на ВОССТАНОВЛЕННОМ угле
(`SubturnGuaranteeLawV1.SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1`), потому
что иначе обещание «канонические 90° всегда дают один счёт» невыполнимо при
чётном q: у угла 90.0000015° равношаговый веер при H=1 требует подшага
pi/4 + 7.5e-7, ординальное окно бинарной привязки при этом пусто
(`positive full-fan termination width is not proven`), и патч отвергается
целиком — измерено. Новый закон применяется ТОЛЬКО там, где старый отказал;
где старый проходит, не меняется ни байта. Полное обязательство нового
закона — в докстринге `SubturnGuaranteeLawV1`.

АРИФМЕТИКА ТОЧНАЯ. Допуск объявлен в радианах, а мера приходит в долях π,
поэтому сравнение требует π. Вместо оценки берётся ДОКАЗАННАЯ рациональная
верхняя граница `PI_RATIONAL_UPPER_BOUND`: она делает принимаемое множество
строгим подмножеством объявленного допуска (ошибаемся в сторону отказа, не в
сторону восстановления). Доказательство границы исполняемое, а не
декларативное — интервальной оболочкой π в тесте.
"""

from __future__ import annotations

from dataclasses import dataclass
from fractions import Fraction

from .contracts.envelopes import (
    AngleTolerancePolicyIdV1,
    CanonicalAngleRestorationCertificateV1,
    CanonicalAngleRestorationLawV1,
    CanonicalReflexAngleRelationV1,
    CanonicalSubturnFanAuthorityV1,
    SubturnGuaranteeLawV1,
)
from .contracts.metric import ExactRationalV1
from .numeric import ExactRatioV1, IntervalEndpointKind
from ._authoring_intent import AUTHOR_ANGULAR_ERROR


# π = 3.14159265358979323846264338327950288... — последняя записанная цифра
# поднята, поэтому значение строго больше π. Используется только сверху:
# `|u - u_c| * PI_UPPER >= |u - u_c| * π`, то есть доказанное отклонение не
# может оказаться заниженным. Проверяется тестом через интервальную оболочку.
PI_RATIONAL_UPPER_BOUND = Fraction(3141592653589793239, 10**18)


# МНОЖЕСТВО КАНОНИЧЕСКИХ ОТНОШЕНИЙ — МИНИМАЛЬНОЕ, ОПЛАЧЕННОЕ ПОЛЕМ.
#
# Здесь ровно один элемент: прямой угол. Сравнение долей π рационально и
# потому одинаково дёшево для любого `p/q`, но дешевизна СРАВНЕНИЯ не есть
# основание объявлять отношение задуманным. Оплачено полем и уже названо
# кодом именно прямое: `source_grid.intended_right_corner_facts` классифицирует
# «задуманно прямые» углы тем же допуском и никаких других не знает, а слепок
# стены 2.001 показывает шум ровно на 90°.
#
# ИМЕНОВАННАЯ ГРАНИЦА КАРТОЧКИ: семейства 45°/60° (`u` = 1/4, 1/3, 2/3, 3/4)
# сюда НЕ входят. Машинерия сравнения их пропустила бы бесплатно, но полевого
# свидетельства, что 224.9997° — это задуманные 225°, а не задуманные 225.0°
# с другой стороны допуска, нет; и классификатора намерения для них тоже нет.
# Расширение — отдельное решение владельца с отдельным полевым слепком.
CANONICAL_REFLEX_EXCESS_RELATIONS = (
    (
        CanonicalReflexAngleRelationV1.CANONICAL_REFLEX_EXCESS_PI_OVER_2,
        Fraction(1, 2),
    ),
)

CANONICAL_ANGLE_RESTORATION_PREDICATES = frozenset(
    {
        "CANONICAL_RELATION_IS_DECLARED",
        "SOURCE_MEASURE_IS_NOT_ALREADY_CANONICAL",
        "DEVIATION_PROVEN_WITHIN_AUTHORING_INTENT_TOLERANCE",
        "CANONICAL_FACT_REPLACES_SOURCE_BEFORE_SELECTION",
    }
)

CANONICAL_ANGLE_RESTORATION_LAW = (
    CanonicalAngleRestorationLawV1.AUTHORING_INTENT_CANONICAL_ANGLE_RESTORED_V1
)

CANONICAL_ANGLE_TOLERANCE_POLICY_ID = (
    AngleTolerancePolicyIdV1.AUTHOR_ANGULAR_ERROR_AUTHORING_INTENT_V1
)

CANONICAL_SUBTURN_FAN_LAW = (
    SubturnGuaranteeLawV1.SUBTURN_GUARANTEE_ON_CANONICAL_SUPPORTS_V1
)

CANONICAL_SUBTURN_FAN_PREDICATES = frozenset(
    {
        "SOURCE_EQUAL_SUBTURN_FAN_VIOLATES_GUARANTEE_ON_SOURCE_SUPPORTS",
        "CANONICAL_SUBTURN_IS_EXACTLY_WITHIN_MAX_SUBTURN",
        "CANONICAL_RAYS_ARE_EXACT_ORDINAL_ROTATIONS_OF_THE_INCOMING_SUPPORT",
        "RESIDUAL_IS_BOUNDED_BY_THE_AUTHORING_INTENT_TOLERANCE",
    }
)

# Знаменатели, для которых точный поворот на `pi/n` вообще существует в
# принятой машинерии (`_exact_q_trig`). Канонический подшаг прямого угла —
# `pi/(2(H+1))`, а закон плотности даёт `H` в {1, 2}, поэтому попадаем в 4 и 6
# и ни разу не выходим за набор. Выход за него — именованный отказ, а не
# приближение: тогда точного поворота нет и веер строить не из чего.
CANONICAL_ROTATION_DENOMINATORS = frozenset({2, 3, 4, 5, 6})


@dataclass(frozen=True, slots=True)
class CanonicalExcessIntervalV1:
    """Точная рациональная доля π как вырожденный закрытый интервал.

    Селектор читает интервал, а не число, поэтому канонический факт приходит
    к нему в том же виде — но без единой десятичной цифры: обе границы равны
    одной и той же `Fraction`.
    """

    lower: Fraction
    upper: Fraction
    lower_kind: IntervalEndpointKind
    upper_kind: IntervalEndpointKind


@dataclass(frozen=True, slots=True)
class CanonicalAngleRestorationV1:
    """Факт восстановления без идентификаторов конвейера."""

    relation: CanonicalReflexAngleRelationV1
    canonical_excess_over_pi: Fraction
    deviation_upper_bound_radians: Fraction

    @property
    def canonical_interval(self) -> CanonicalExcessIntervalV1:
        return CanonicalExcessIntervalV1(
            self.canonical_excess_over_pi,
            self.canonical_excess_over_pi,
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
        )


def _rational(value: Fraction) -> ExactRationalV1:
    item = Fraction(value)
    return ExactRationalV1(item.numerator, item.denominator)


def _ratio(value: Fraction) -> ExactRatioV1:
    item = Fraction(value)
    return ExactRatioV1(item.numerator, item.denominator)


# Тот же допуск, выраженный в ДОЛЯХ π. Величина производная и считается один
# раз: сравнивать в долях дешевле, чем умножать каждое отклонение на π, а
# результат тот же — обе части неравенства положительны. Направление ошибки
# сохранено: делитель — верхняя граница π, значит порог в долях занижен.
_TOLERANCE_OVER_PI = AUTHOR_ANGULAR_ERROR / PI_RATIONAL_UPPER_BOUND


def _deviation_over_pi(interval, canonical: Fraction) -> Fraction:
    """`|u − u_канон|` по ВСЕМУ интервалу, в долях π.

    Берётся дальняя граница интервала, а не его середина: восстановление
    обязано покрывать КАЖДОЕ значение, которое оболочка допускает, иначе оно
    доказывает не то, что утверждает. Ноль означает «уже точно канонично».
    """

    return max(
        abs(Fraction(interval.lower) - canonical),
        abs(Fraction(interval.upper) - canonical),
    )


def canonical_reflex_excess_restoration(
    interval,
) -> CanonicalAngleRestorationV1 | None:
    """Восстановить задуманное отношение, либо уступить сырому числу.

    `None` означает ровно одно: сырой факт идёт к селектору без изменения.
    Причин ровно три, и все три — «восстанавливать нечего или нельзя»:
    доля уже точно канонична; отклонение больше допуска намерения; ни одно
    объявленное каноническое отношение не подходит.
    """

    matches = []
    for relation, canonical in CANONICAL_REFLEX_EXCESS_RELATIONS:
        deviation = _deviation_over_pi(interval, canonical)
        if deviation == 0:
            return None
        if deviation <= _TOLERANCE_OVER_PI:
            matches.append((relation, canonical, deviation))
    if len(matches) != 1:
        # Ноль — вне всех допусков. Больше одного — объявленные отношения
        # сошлись ближе допуска друг к другу, то есть таблица противоречива;
        # выбирать «ближайшее» значило бы вводить не объявленный закон.
        return None
    relation, canonical, deviation = matches[0]
    # Радианы считаются ОДИН раз и только на состоявшемся восстановлении:
    # в сертификат идёт доказанная сверху величина, а не порог.
    return CanonicalAngleRestorationV1(
        relation,
        canonical,
        deviation * PI_RATIONAL_UPPER_BOUND,
    )


def selector_reflex_excess_interval(interval):
    """Единственная дверь: что именно увидит закон счёта.

    Возвращает `(интервал_для_селектора, восстановление|None)`. Сырой интервал
    возвращается тем же объектом, поэтому путь без восстановления побитово
    прежний.
    """

    restoration = canonical_reflex_excess_restoration(interval)
    if restoration is None:
        return interval, None
    return restoration.canonical_interval, restoration


def build_canonical_angle_restoration_certificate(
    restoration: CanonicalAngleRestorationV1,
    *,
    selection_certificate_id,
    corner_relation_id,
    reflex_angle_certificate_id,
    source_interval,
) -> CanonicalAngleRestorationCertificateV1:
    return CanonicalAngleRestorationCertificateV1(
        restoration_law=CANONICAL_ANGLE_RESTORATION_LAW,
        selection_certificate_id=selection_certificate_id,
        corner_relation_id=corner_relation_id,
        reflex_angle_certificate_id=reflex_angle_certificate_id,
        canonical_relation=restoration.relation,
        canonical_reflex_excess_over_pi=_ratio(
            restoration.canonical_excess_over_pi
        ),
        source_reflex_excess_over_pi=source_interval,
        deviation_upper_bound_radians=_rational(
            restoration.deviation_upper_bound_radians
        ),
        tolerance_radians=_rational(AUTHOR_ANGULAR_ERROR),
        tolerance_policy_id=CANONICAL_ANGLE_TOLERANCE_POLICY_ID,
        proven_predicates=CANONICAL_ANGLE_RESTORATION_PREDICATES,
    )


def canonical_angle_restoration_error(
    certificate: CanonicalAngleRestorationCertificateV1,
    source_interval,
) -> str | None:
    """Пересчитать восстановление заново и сравнить с записью.

    Проверяющий не верит ни одному полю сертификата: он берёт СЫРОЙ угол
    снапшота и повторяет закон. Совпало — запись честна; не совпало — назван
    исход, а не выбрана сторона.
    """

    if type(certificate) is not CanonicalAngleRestorationCertificateV1:
        return "canonical angle restoration record has a foreign type"
    if certificate.restoration_law is not CANONICAL_ANGLE_RESTORATION_LAW:
        return "canonical angle restoration law is not the declared one"
    if certificate.tolerance_policy_id is not CANONICAL_ANGLE_TOLERANCE_POLICY_ID:
        return "canonical angle restoration names another tolerance policy"
    if certificate.proven_predicates != CANONICAL_ANGLE_RESTORATION_PREDICATES:
        return "canonical angle restoration predicate set is not the declared one"
    if Fraction(
        certificate.tolerance_radians.numerator,
        certificate.tolerance_radians.denominator,
    ) != AUTHOR_ANGULAR_ERROR:
        return "canonical angle restoration tolerance differs from AUTHOR_ANGULAR_ERROR"
    if certificate.source_reflex_excess_over_pi != source_interval:
        return "canonical angle restoration cites another source angle"
    restoration = canonical_reflex_excess_restoration(source_interval)
    if restoration is None:
        return (
            "canonical angle restoration is recorded for an angle that is "
            "exact or outside the authoring-intent tolerance"
        )
    if certificate.canonical_relation is not restoration.relation:
        return "canonical angle restoration names another canonical relation"
    if Fraction(
        certificate.canonical_reflex_excess_over_pi.numerator,
        certificate.canonical_reflex_excess_over_pi.denominator,
    ) != restoration.canonical_excess_over_pi:
        return "canonical angle restoration value differs from its relation"
    if Fraction(
        certificate.deviation_upper_bound_radians.numerator,
        certificate.deviation_upper_bound_radians.denominator,
    ) != restoration.deviation_upper_bound_radians:
        return "canonical angle restoration deviation bound is not the proven one"
    return None


def canonical_subturn_over_pi(
    canonical_excess_over_pi: Fraction,
    hidden_count: int,
) -> Fraction:
    """Каноническая доля π в ОДНОМ подшаге равношагового веера."""

    return Fraction(canonical_excess_over_pi, hidden_count + 1)


def canonical_subturn_is_within_max_subturn(
    canonical_excess_over_pi: Fraction,
    hidden_count: int,
    q: int,
) -> bool:
    """`u*pi/(H+1) <= pi/q` — целочисленно и точно, без единого порога."""

    return canonical_excess_over_pi * q <= hidden_count + 1


def canonical_rotation_denominator(
    canonical_excess_over_pi: Fraction,
    hidden_count: int,
) -> int | None:
    """Знаменатель точного поворота или `None` — «повернуть нечем».

    Луч ординала `j` ставится поворотом входящей опоры на `j` шагов по
    `pi/n`. Это возможно ровно тогда, когда канонический подшаг равен `pi/n` с
    целым `n` из принятого набора: числитель доли обязан быть единицей, иначе
    шаг не является одним поворотом.
    """

    subturn = canonical_subturn_over_pi(canonical_excess_over_pi, hidden_count)
    if subturn.numerator != 1:
        return None
    denominator = subturn.denominator
    if denominator not in CANONICAL_ROTATION_DENOMINATORS:
        return None
    return denominator


def build_canonical_subturn_fan_authority(
    restoration_certificate: CanonicalAngleRestorationCertificateV1,
    *,
    envelope_spec_id,
    hidden_count: int,
    q: int,
) -> CanonicalSubturnFanAuthorityV1:
    canonical = Fraction(
        restoration_certificate.canonical_reflex_excess_over_pi.numerator,
        restoration_certificate.canonical_reflex_excess_over_pi.denominator,
    )
    subturn = canonical_subturn_over_pi(canonical, hidden_count)
    return CanonicalSubturnFanAuthorityV1(
        guarantee_law=CANONICAL_SUBTURN_FAN_LAW,
        envelope_spec_id=envelope_spec_id,
        selection_certificate_id=(
            restoration_certificate.selection_certificate_id
        ),
        canonical_relation=restoration_certificate.canonical_relation,
        canonical_reflex_excess_over_pi=(
            restoration_certificate.canonical_reflex_excess_over_pi
        ),
        hidden_edge_count=hidden_count,
        max_subturn_q=q,
        canonical_subturn_over_pi=_ratio(subturn),
        raw_residual_upper_bound_radians=(
            restoration_certificate.deviation_upper_bound_radians
        ),
        proven_predicates=CANONICAL_SUBTURN_FAN_PREDICATES,
    )


def canonical_subturn_fan_authority_error(
    authority: CanonicalSubturnFanAuthorityV1,
    restoration_certificate,
) -> str | None:
    """Пересчитать власть канонического веера и сравнить с записью."""

    if type(authority) is not CanonicalSubturnFanAuthorityV1:
        return "canonical subturn fan authority has a foreign type"
    if authority.guarantee_law is not CANONICAL_SUBTURN_FAN_LAW:
        return "canonical subturn fan authority names another guarantee law"
    if authority.proven_predicates != CANONICAL_SUBTURN_FAN_PREDICATES:
        return "canonical subturn fan predicate set is not the declared one"
    if restoration_certificate is None:
        return (
            "canonical subturn fan authority stands on an angle without a "
            "proven canonical restoration"
        )
    expected = build_canonical_subturn_fan_authority(
        restoration_certificate,
        envelope_spec_id=authority.envelope_spec_id,
        hidden_count=authority.hidden_edge_count,
        q=authority.max_subturn_q,
    )
    if authority != expected:
        return "canonical subturn fan authority does not follow from its restoration"
    canonical = Fraction(
        authority.canonical_reflex_excess_over_pi.numerator,
        authority.canonical_reflex_excess_over_pi.denominator,
    )
    if not canonical_subturn_is_within_max_subturn(
        canonical,
        authority.hidden_edge_count,
        authority.max_subturn_q,
    ):
        return "canonical subturn exceeds the declared maximum subturn"
    return None


def canonical_selection_interval(certificate, source_interval):
    """`(интервал_для_доказательства, ошибка|None)` для проверяющего план.

    Записи нет — доказывается сырой угол, тем же объектом. Запись есть и
    честна — доказывается канонический факт. Запись есть и лжёт — сырой угол
    и названная ошибка: подмена не происходит, а факт подделки не теряется.
    """

    if certificate is None:
        return source_interval, None
    error = canonical_angle_restoration_error(certificate, source_interval)
    if error is not None:
        return source_interval, error
    value = Fraction(
        certificate.canonical_reflex_excess_over_pi.numerator,
        certificate.canonical_reflex_excess_over_pi.denominator,
    )
    return (
        CanonicalExcessIntervalV1(
            value,
            value,
            IntervalEndpointKind.CLOSED,
            IntervalEndpointKind.CLOSED,
        ),
        None,
    )


def canonical_restoration_reference_errors(restorations, certificate_by_id):
    """Связь записи восстановления со своим сертификатом селекции.

    Структурная половина: запись обязана указывать на ТОТ ЖЕ угол и ту же
    угловую связь, что и сертификат, счёт которого она объясняет.
    """

    errors = []
    for restoration in restorations:
        selection = certificate_by_id.get(restoration.selection_certificate_id)
        if selection is None:
            continue
        if (
            restoration.corner_relation_id != selection.corner_relation_id
            or restoration.reflex_angle_certificate_id
            != selection.reflex_angle_certificate_id
        ):
            errors.append(
                (
                    restoration.selection_certificate_id,
                    "canonical angle restoration differs from its selection "
                    "certificate",
                )
            )
    return tuple(errors)


__all__ = (
    "CANONICAL_ANGLE_RESTORATION_LAW",
    "CANONICAL_ROTATION_DENOMINATORS",
    "CANONICAL_SUBTURN_FAN_LAW",
    "CANONICAL_SUBTURN_FAN_PREDICATES",
    "build_canonical_subturn_fan_authority",
    "canonical_rotation_denominator",
    "canonical_subturn_fan_authority_error",
    "canonical_subturn_is_within_max_subturn",
    "canonical_subturn_over_pi",
    "CANONICAL_ANGLE_RESTORATION_PREDICATES",
    "CANONICAL_ANGLE_TOLERANCE_POLICY_ID",
    "CANONICAL_REFLEX_EXCESS_RELATIONS",
    "PI_RATIONAL_UPPER_BOUND",
    "CanonicalAngleRestorationV1",
    "CanonicalExcessIntervalV1",
    "build_canonical_angle_restoration_certificate",
    "canonical_angle_restoration_error",
    "canonical_reflex_excess_restoration",
    "canonical_restoration_reference_errors",
    "canonical_selection_interval",
    "selector_reflex_excess_interval",
)
