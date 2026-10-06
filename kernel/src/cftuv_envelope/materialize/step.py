"""Шаг ширины ДОМЕНА: покрытие и имена экземпляров из сертификата внутри заверенного интервала, остальное - тем же кодом, что при полном счёте.

ЗАЧЕМ. Живая ширина (`cftuv/envelope_width_live`) считает каждый домен заново на каждом шаге ползунка. Полный счёт - покрытие
(`wavefront.conveyor_coverage`, затем `materialize_domain` на нём), и в покрытии две дорогие части, обе подчинены ширине очень слабо.
(1) Отсечение граней скелета фронтом: точная арифметика, а точки отсечения аффинны по ширине (`wavefront.coverage_template`). Пока ширина
внутри интервала событий покрытия (`interval.coverage_interval`: фронт не проходит ни одной вершины грани скелета), образец знаков у
каждой грани тот же, а точка отсечения - `A + alpha * B` с ТОЧНЫМИ коэффициентами: покрытие воспроизводится без отсечения и делений, и
значение РАВНО полному (каноническая форма `SqrtSumV1` единственна). (2) Имена экземпляров юбок (`conveyor._instance_ids_by_spec`) выводит
резолвер границы (`reference.boundary.resolve_component_alphas`: sympy, около двух миллисекунд на домен), и от запрошенной ширины его ход
зависит ТОЛЬКО знаками «alpha контакта против запрошенной» (`trace`): пока ни один знак не сменился, у каждой спеки та же эффективная alpha
(равная запрошенной либо константа укорочения), а имя - тот же `strip_envelope_instance_id` на новой ширине. Это окно имён, оно отдельно
от окна покрытия: за ним резолвер считает сам, ответ тот же.

ЧТО ЭТО НЕ ДЕЛАЕТ. Решающие стадии (тесселяция, разбиение, закон положения вершин `src:`, силуэт, станции, резка источника) НЕ
заменяются: они идут своим кодом на тех же точных значениях, что и при полном счёте, поэтому их решения, счётчики и числа - те же
побитово, а не угаданные по соседней ширине. Подпись структуры батча (`structure.batch_structure`) считается каждый шаг и называет
переключение структуры относительно записанной ширины (`INTERVAL_STRUCTURE_SWITCH`), но на ответ не влияет: ответ - полный счёт.
Номера `region:N` и `claim:N` выводит `Layout` из отсортированных имён (имена - выше), поэтому перенумерация идёт сама; станции, граничные и
интерфейсные цепи называются от ключей вершин и регионов, а не от alpha.

ЧТО ЗАПИСЫВАЕТСЯ. Каждый полный счёт домена (первый, промах) кладёт СЕРТИФИКАТ в память подготовки воркера (`memo`): ширину alpha0, границы
интервала покрытия, шаблоны покрытия разбиений и окно имён. Пикл подготовки несёт память пустой, поэтому сертификат живёт в воркере,
который держит подготовку, и по трубе ничего не едет. Шаг внутри ЛЮБОГО из последних `CERTIFICATE_LIMIT` сертификатов - попадание.

ИСХОДЫ. Попадание (`FAST_HIT`) либо полный счёт под ИМЕНЕМ причины (`NO_CERTIFICATE`, `OUTSIDE_BELOW`, `OUTSIDE_ABOVE`, `OUTSIDE_BETWEEN`,
`TEMPLATE_UNAVAILABLE`, `PARTIAL_TEMPLATE`, `COVERAGE_REFUSED`, `DISABLED`): счёт попаданий и промахов по причинам ведёт `STEP_COUNTERS`, а
хост - счётчики профиля прогона. Сверка (`CFTUV_INTERVAL_VERIFY=1`) после каждого попадания считает полный путь заново (память недавних
покрытий сброшена) и сравнивает ответы; расхождение называется (`VERIFY_MISMATCH:<части>`), а возвращается ПОЛНЫЙ ответ.
`CFTUV_INTERVAL_STEP=0` выключает быстрый путь целиком (замер «до», сверка «с ним и без него»).
"""

from __future__ import annotations

import os
from collections import Counter
from dataclasses import dataclass
from fractions import Fraction
from functools import lru_cache

import sympy as sp

from ..codec import canonical_json_bytes
from ..contracts.envelopes import StripEnvelopeSpec
from ..reference.strip import strip_envelope_instance_id
from .. import wavefront as wavefront_package
from ..wavefront.conveyor import ConveyorOutcome, requested_alpha_fraction
from ..wavefront.coverage import CoverageOutcome, _coverage_at, clear_recent_coverage, coverage_source
from ..wavefront.coverage_template import build_template, instantiate
from . import domain as domain_module
from .domain import MaterializationV1
from .interval import CERTIFIED
from .memo import memo_enabled, memo_of

ENVIRONMENT_SWITCH = "CFTUV_INTERVAL_STEP"
ENVIRONMENT_VERIFY = "CFTUV_INTERVAL_VERIFY"
#: Сертификатов на подготовку и закон: ширина ходит туда-обратно, и новый интервал не должен вытеснять только что полезный.
CERTIFICATE_LIMIT = 3
#: Попыток записать шаблон на подготовку, после которых запись прекращается (шаблон отказал: домен идёт полным путём под своим именем).
TEMPLATE_ATTEMPT_LIMIT = 3

PATH_FAST = "FAST_HIT"
FALLBACK_NO_CERTIFICATE = "NO_CERTIFICATE"
FALLBACK_BELOW = "OUTSIDE_BELOW"
FALLBACK_ABOVE = "OUTSIDE_ABOVE"
FALLBACK_BETWEEN = "OUTSIDE_BETWEEN"
FALLBACK_REFUSED = "COVERAGE_REFUSED"
FALLBACK_TEMPLATE = "TEMPLATE_UNAVAILABLE"
FALLBACK_DISABLED = "DISABLED"
FALLBACK_PARTIAL = "PARTIAL_TEMPLATE"
VERIFY_MISMATCH = "VERIFY_MISMATCH"
STRUCTURE_SWITCH = "INTERVAL_STRUCTURE_SWITCH"
#: `INTERVAL_FAST_HITS` и `INTERVAL_FALLBACK_<причина>`: счёт процесса (воркера), а не ответ.
STEP_COUNTERS: Counter = Counter()


def step_enabled() -> bool:
    return os.environ.get(ENVIRONMENT_SWITCH, "1").strip().lower() not in ("0", "off", "false", "no")


def verify_enabled() -> bool:
    return os.environ.get(ENVIRONMENT_VERIFY, "0").strip().lower() in ("1", "on", "true", "yes")


@dataclass(slots=True, eq=False)
class StepCertificateV1:
    """Запись полного счёта домена при alpha0: интервал покрытия и шаблоны покрытия разбиений (по `id` разбиения)."""

    alpha: object
    low: float
    high: float | None
    templates: dict
    #: Разбиения, на которых записаны шаблоны: пока сертификат держит их, `id` не переиспользуется.
    partitions: tuple
    cuts: int
    signature: object | None = None
    #: Имена экземпляров юбок: `{спека: None}` - эффективная alpha равна запрошенной (имя считается на новой ширине), `{спека: имя}` -
    #: спека укорочена резолвером до константы. Верны, пока ни один знак контакта против запрошенной alpha не сменился: открытый
    #: интервал `(names_low, names_high)` (метры, точные дроби; `names_high` `None` - сверху нет). `names` `None` - не заверены, и
    #: резолвер границы идёт каждый шаг.
    names: dict | None = None
    names_low: Fraction | None = None
    names_high: Fraction | None = None

    def covers(self, alpha) -> bool:
        """`alpha` (точная дробь, метры) строго внутри интервала покрытия: сравнение точное, границы - их `binary64` как дроби."""

        return Fraction(self.low) < alpha and (self.high is None or alpha < Fraction(self.high))

    def names_hold(self, alpha) -> bool:
        return self.names is not None and self.names_low < alpha and (self.names_high is None or alpha < self.names_high)


class _Recorder:
    """Источник покрытия записывающего прохода: полный счёт и шаблон по каждому разбиению."""

    bypass_recent = True

    def __init__(self) -> None:
        self.templates: dict = {}
        self.partitions: list = []
        self.refused = False
        self.names: dict | None = None
        self.names_window: tuple | None = None

    def record_names(self, prepared, alpha_value, kinds, trace) -> None:
        """Исход резолвера границы: имена и окно ширины, в котором ни один знак контакта против запрошенной alpha не меняется."""

        self.names = None
        self.names_window = _contact_window(trace, Fraction(str(alpha_value.value)))
        if self.names_window is not None:
            self.names = dict(kinds)

    def coverage(self, partition, alpha, work_budget, store):
        traces: list = []
        result = _coverage_at(partition, alpha, work_budget, store, traces)
        if result.outcome is not CoverageOutcome.EXACT:
            return result
        template = build_template(partition, alpha, work_budget, store, result, traces)
        if template is None:
            self.refused = True
        else:
            self.templates[id(partition)] = template
            self.partitions.append(partition)
        return result


class _Instantiator:
    """Источник покрытия попадания: покрытие из шаблона; разбиение без шаблона считает `coverage_at` сам (и это названо)."""

    bypass_recent = False

    def __init__(self, certificate: StepCertificateV1) -> None:
        self.certificate = certificate
        self.misses = 0

    def instance_ids(self, prepared, alpha_value):
        """Имена экземпляров юбок из сертификата, пока знаки контактов те же (иначе `None`: резолвер границы считает сам)."""

        certificate = self.certificate
        if certificate.names is None or not certificate.names_hold(Fraction(str(alpha_value.value))):
            return None
        requested = _requested(str(alpha_value.value))
        names = {}
        for spec in prepared.compilation.envelope_specs:
            if not isinstance(spec, StripEnvelopeSpec):
                continue
            kind = certificate.names.get(spec.envelope_spec_id.value, "")
            if kind == "":
                return None
            names[spec.envelope_spec_id.value] = strip_envelope_instance_id(spec, requested) if kind is None else kind
        return names, ""

    def coverage(self, partition, alpha, work_budget, store):
        template = self.certificate.templates.get(id(partition))
        produced = None if template is None else instantiate(template, partition, alpha, work_budget)
        if produced is None:
            self.misses += 1
        return produced


def _miss_side(certificates, alpha) -> str:
    """Где ширина относительно ВСЕХ сертификатов: ниже всех, выше всех либо между непересекающимися интервалами."""

    if all(alpha <= Fraction(item.low) for item in certificates):
        return FALLBACK_BELOW
    if all(item.high is not None and alpha >= Fraction(item.high) for item in certificates):
        return FALLBACK_ABOVE
    return FALLBACK_BETWEEN


@lru_cache(maxsize=64)
def _requested(text: str):
    """Запрошенная alpha как рациональное `sympy` (то же, что берёт резолвер): одна на шаг на процесс, а не на домен."""

    return sp.Rational(text)


def _contact_window(trace, alpha: Fraction):
    """`(низ, верх)` открытого окна ширины вокруг `alpha`, в котором знаки всех контактов трассы против запрошенной alpha те же, либо `None`.

    Оболочка контакта `[lo, hi]` - строгая (точные дроби): контакт строго ниже `alpha`, если `hi < alpha`, строго выше, если `lo > alpha`;
    оболочка, накрывшая `alpha` (равенство), либо контакт без оболочки - окна нет, и имена не заверены.
    """

    low, high = Fraction(0), None
    for bounds in trace:
        if bounds is None:
            return None
        first, last = bounds
        if first <= alpha <= last:
            return None
        if last < alpha:
            low = max(low, last)
        else:
            high = first if high is None else min(high, first)
    return low, high


@dataclass(frozen=True, slots=True)
class StepV1:
    """Итог шага: материализация, путь (`FAST_HIT` либо `FALLBACK:<причина>`) и числа сверки."""

    result: MaterializationV1
    path: str
    #: Подпись структуры отличается от подписи записанной ширины (только у попадания): запись, а не ответ.
    structure_switched: bool = False

    @property
    def is_hit(self) -> bool:
        return self.path == PATH_FAST


def _materialize(prepared, coverage, arguments) -> MaterializationV1:
    request, lift_law, topology_law, digests = arguments
    # Через модуль, а не по имени: подмена стадии (отказ ядра в тестах хоста, сверка) должна действовать и здесь.
    return domain_module.materialize_domain(
        prepared,
        coverage,
        request=request,
        near_planar_lift_law=lift_law,
        decal_topology_law=topology_law,
        digests=digests,
        certify=True,
    )


def answer_differences(first: MaterializationV1, second: MaterializationV1) -> tuple[str, ...]:
    """Названия частей ответа, в которых два результата расходятся (пусто - ответы равны); секунды, метки и записанные факты не в счёте."""

    found = []
    for name in ("outcome", "detail", "counters", "diagnostics", "vertex_normals", "offset_normal_law", "offset_normals_digest",
                 "decal_topology_law", "content_digest", "digests_deferred"):
        if getattr(first, name) != getattr(second, name):
            found.append(name)
    if (first.batch is None) != (second.batch is None):
        found.append("batch")
    elif first.batch is not None and canonical_json_bytes(first.batch) != canonical_json_bytes(second.batch):
        found.append("batch")
    return tuple(found)


def _coverage(prepared, alpha_text):
    """Покрытие домена; через пакет `wavefront`, а не по имени: подмена стадии (проба, сверка) действует и здесь."""

    return wavefront_package.conveyor_coverage(prepared, alpha_text)


def _full(prepared, alpha_text, arguments) -> MaterializationV1:
    return _materialize(prepared, _coverage(prepared, alpha_text), arguments)


def _recorded(prepared, alpha_text, alpha, arguments, memo, key, certificates):
    """Полный счёт с записью шаблона: `(результат, сертификат либо None)`."""

    recorder = _Recorder()
    with coverage_source(recorder):
        coverage = _coverage(prepared, alpha_text)
    result = _materialize(prepared, coverage, arguments)
    bounds = result.coverage_bounds
    if (
        not result.is_materialized
        or bounds is None
        or bounds.status != CERTIFIED
        or recorder.refused
        or len(recorder.templates) != len(prepared.regions)
    ):
        return result, None
    certificate = StepCertificateV1(
        alpha,
        bounds.low,
        bounds.high,
        recorder.templates,
        tuple(recorder.partitions),
        sum(item.cuts for item in recorder.templates.values()),
        None if result.structure is None else result.structure.digest,
        recorder.names,
        None if recorder.names_window is None else recorder.names_window[0],
        None if recorder.names_window is None else recorder.names_window[1],
    )
    memo.put(key, (certificate, *certificates)[:CERTIFICATE_LIMIT])
    return result, certificate


def step_domain(prepared, alpha_text, *, request, near_planar_lift_law, decal_topology_law, digests=True) -> StepV1:
    """Покрытие и материализация домена при `alpha_text`: из шаблона внутри заверенного интервала, иначе полным путём с записью шаблона.

    Ответ (`StepV1.result`) равен ответу `materialize_domain(prepared, conveyor_coverage(prepared, alpha_text), ...)` с `certify=True`.
    """

    arguments = (request, near_planar_lift_law, decal_topology_law, digests)
    memo = memo_of(prepared)
    if memo is None or not memo_enabled() or not step_enabled():
        STEP_COUNTERS["INTERVAL_FALLBACK_" + FALLBACK_DISABLED] += 1
        return StepV1(_full(prepared, alpha_text, arguments), "FALLBACK:" + FALLBACK_DISABLED)
    alpha = requested_alpha_fraction(alpha_text)
    key = ("step", near_planar_lift_law.value, decal_topology_law.value)
    certificates = memo.fetch(key) or ()
    chosen = next((item for item in certificates if item.covers(alpha)), None)
    if chosen is None:
        reason = FALLBACK_NO_CERTIFICATE if not certificates else _miss_side(certificates, alpha)
        attempts = memo.fetch(key + ("attempts",)) or 0
        if attempts >= TEMPLATE_ATTEMPT_LIMIT and not certificates:
            STEP_COUNTERS["INTERVAL_FALLBACK_" + FALLBACK_TEMPLATE] += 1
            return StepV1(_full(prepared, alpha_text, arguments), "FALLBACK:" + FALLBACK_TEMPLATE)
        result, certificate = _recorded(prepared, alpha_text, alpha, arguments, memo, key, certificates)
        if certificate is None and result.is_materialized:
            memo.put(key + ("attempts",), attempts + 1)
        STEP_COUNTERS["INTERVAL_FALLBACK_" + reason] += 1
        return StepV1(result, "FALLBACK:" + reason)
    source = _Instantiator(chosen)
    with coverage_source(source):
        coverage = _coverage(prepared, alpha_text)
    result = _materialize(prepared, coverage, arguments)
    if coverage.outcome is not ConveyorOutcome.EXACT:
        STEP_COUNTERS["INTERVAL_FALLBACK_" + FALLBACK_REFUSED] += 1
        return StepV1(result, "FALLBACK:" + FALLBACK_REFUSED)
    if source.misses:
        STEP_COUNTERS["INTERVAL_FALLBACK_" + FALLBACK_PARTIAL] += 1
        return StepV1(result, "FALLBACK:" + FALLBACK_PARTIAL)
    switched = result.structure is not None and chosen.signature is not None and result.structure.digest != chosen.signature
    if verify_enabled():
        clear_recent_coverage()
        full = _full(prepared, alpha_text, arguments)
        clear_recent_coverage()
        differing = answer_differences(result, full)
        if differing:
            STEP_COUNTERS[VERIFY_MISMATCH] += 1
            return StepV1(full, VERIFY_MISMATCH + ":" + ",".join(differing), switched)
    STEP_COUNTERS["INTERVAL_FAST_HITS"] += 1
    if switched:
        STEP_COUNTERS[STRUCTURE_SWITCH] += 1
    return StepV1(result, PATH_FAST, switched)
