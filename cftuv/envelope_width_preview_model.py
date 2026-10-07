"""Модель превью меша ширины: настоящий меш декали двигается между точными пересчётами, и каждое такое движение названо.

ЗАЧЕМ. Пока владелец тянет ширину, точный пересчёт стоит секунду и больше, а линии превью (`envelope_width_preview`) — не меш. Модель
позволяет за миллисекунды писать в СУЩЕСТВУЮЩИЙ меш декали позиции и UV на промежуточной ширине, пока точный счёт идёт в фоне, и точный
результат потом заменяет превью. Исход назван `PREVIEW_MESH_FROM_INTERVAL_V1`: это ПРИБЛИЗИТЕЛЬНОЕ превью, а не сертифицированная
геометрия, и каждая доля, в которой оно не сертифицировано, называется (правило 4 `AGENTS.md`).

ЧТО ЗАВЕРЕНО, А ЧТО НЕТ (аудит ad6074f, F4; разведено в именах: «сертифицировано» — только первое).
  ЗАВЕРЕНО (R1, ядро): ИНТЕРВАЛ ширины домена, внутри которого нет событий покрытия и резки и структура домена та же
  (`AlphaIntervalV1`, статус `CERTIFIED`). Модель его читает и не доказывает заново.
  НЕ ЗАВЕРЕНО (эвристика этого модуля): ФОРМА зависимости позиций и UV от ширины ВНУТРИ интервала — многочлен через два либо три точных
  образца (`PreviewModelV1`); доверительная область модели (`reach`, доля ширины базы: 10 % у кривого домена, 3 % у хорды); самопроверка
  (`deviation`) — выборка в одной точке, а не граница ошибки. Интервал ядра не заверяет тесселяцию, силуэт, позиции источника, станции,
  UV и диагонали граней; три образца не доказывают аффинность целой функции (дробно-линейный закон `f(a) = 0.01 / (1.11 - a)` ломает
  любой фиксированный процент досягаемости: аудит приводит ошибку в 50 раз выше порога на 10 % ширины). Доказанных границ ошибки у
  модели нет, и ни один текст не называет её «сертификатом».

ИЗ ЧЕГО СТРОИТСЯ. Из ТОЧНЫХ прогонов продуктового пути, которые уже посчитаны (`PreviewSampleV1`): прогон на экране (база, `alpha0`) и один либо
два других прогона того же ключа (`alpha1`, `alpha2`). Модель ничего не считает сама и ядра не знает: она читает массивы меша
(`envelope_production_mesh.MeshArraysV1`) и записанные факты домена (`alpha_interval`, `structure_digest`).

ЧТО В МОДЕЛИ, ПО ДОМЕНУ. Домен смоделирован, когда (1) у него заверенный интервал ширины (`CERTIFIED`), а `alpha1` лежит строго внутри
интервала БАЗЫ (между ними нет событий покрытия и резки: `AlphaIntervalV1.contains_exact`); (2) его структура в обоих прогонах та же
(подпись структуры ядра и токен локальных массивов: грани, швы, ссылки вершин). Иначе он назван причиной (`DOMAIN_*`) и остаётся на последней
точной геометрии. Внутри интервала позиции вершин и произведения `u * alpha`, `v * alpha` у ядра АФФИННЫ по ширине (`kernel/tests/
test_width_affine_premise.py`), кроме вершин на рёбрах источника под резкой: там положение — дробно-линейная функция ширины (пересечение
поворачивающейся прямой с неподвижной), и две точки дают лишь хорду. Поэтому при ДВУХ других прогонах модель берёт квадрат через три точки
(`a1 * dt + a2 * dt^2`, ошибка третьего порядка), при одном — прямую; степень каждого домена записана (`degree`). Позиции меша — позиции
батча плюс смещение декали, и смещение вдоль нормали вершины в ширину не аффинно (нормаль развёртки меняется): оно входит в ту же подгонку и
даёт ошибку второго порядка, измеренную, а не заявленную.

ДОВЕРИТЕЛЬНАЯ ОБЛАСТЬ (`reach`). Интервал ядра заверяет структуру, а не форму зависимости, поэтому у модели своя область: квадрат без значимой
кривизны (добавка на опорных ширинах не выше шума округления, `CURVATURE_NOISE_ULPS`) — аффинный домен, границы нет, кроме интервала; квадрат
со значимой кривизной — `CURVED_REACH_RATIO` (десять процентов); прямая через две точки кривизны проверить не может — `CHORD_REACH_RATIO`
(три процента). Это НАЧАЛЬНОЕ ДОВЕРИЕ, а не гарантия: за областью домен придержан под именем `PREVIEW_DOMAIN_BEYOND_MODEL_REACH`, а опровержение
точным прогоном её СЖИМАЕТ (ниже).

ЖУРНАЛ ДОВЕРИЯ И ПОВТОРНЫЙ ДОПУСК (аудит F5; `TrustLedgerV1`, `advance_ledger`). Домен, предсказание которого точный прогон опроверг
(`deviation.refuted`), входит в КАРАНТИН: во всех моделях того же ключа образца он придержан под именем `PREVIEW_DOMAIN_QUARANTINED_AFTER_
REFUTATION` и не двигается. Повторный допуск — по правилу: (1) каждое следующее точное обновление СЧИТАЕТ предсказание карантинного
домена «в тени» (оно не пишется в меш) на ширине, не равной ни одной опорной ширине модели и лежащей в НОМИНАЛЬНОЙ области домена
(интервал ядра и десять процентов, а не сжатая область); (2) предсказание в пределах `READMIT_ERROR_FRACTION` предела ошибки на расстоянии от
базы не меньше `READMIT_MIN_DISTANCE_FRACTION` досягаемости, с которой домен вернётся, — ЧИСТАЯ сверка (у самой базы любая модель чиста);
`READMIT_CLEAN_CHECKS` таких сверок возвращают домен с СЖАТОЙ областью `min(номинал * 0.5^k, 0.5 * расстояние опровержения)` (`k` — число
опровержений; аффинный домен теряет освобождение от досягаемости и получает 10 %); (3) повторное опровержение (в том числе в тени) сжимает
область ещё вдвое, а третье — снимает модель этого домена до следующего нажатия кнопки (`PREVIEW_DOMAIN_ABANDONED_AFTER_REFUTATIONS`).
Журнал живёт, пока жив ключ образца, и забывается кнопкой «Build Decal Mesh».

ФОРМУЛЫ (ровно как в коде; `dt = alpha - alpha0`, на `dt = 0` ответ — база БИТОВО):
    позиция = P0 + a1 * dt + a2 * dt^2
    uv      = uv0 + (e1 + e2 * dt) * dt / alpha        (произведение `uv * alpha` аффинно, поэтому делитель стоит ВНЕ многочлена)

ГДЕ ПРЕВЬЮ ЖИВО. Вершина двигается, если ВСЕ домены, которым она принадлежит, смоделированы и не в карантине, а `alpha` строго внутри
пересечения их областей; петля UV — если жив её домен. Остальные остаются на геометрии базы (последняя точная), и кадр называет их числом
и номерами патчей. Если не жив ни один домен, кадр отказан именем (`PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL` либо
`PREVIEW_ALL_DOMAINS_QUARANTINED`) и равен базе.

САМОПРОВЕРКА (апостериорная). Каждый новый точный прогон (`deviation`) сверяет то, что модель предсказала бы на его ширине, с самим прогоном,
домен за доменом; домен, отклонение которого выше `DEVIATION_LIMIT_RATIO * alpha`, опровергнут и идёт в журнал доверия. Прогон на ширине,
которая была опорной для модели, ничего не проверяет (`PREVIEW_CHECK_SAMPLE_NOT_INDEPENDENT`): он воспроизвёл бы точку подгонки.

Модуль чистый: ни `bpy`, ни настроек, ни контроллера; его зовут и поток точного счёта (построить), и главный поток (кадр).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from fractions import Fraction

import numpy as np

PREVIEW_MESH_FROM_INTERVAL_V1 = "PREVIEW_MESH_FROM_INTERVAL_V1"

#: Статус заверенного интервала у ядра (`AlphaIntervalV1.status`): единственное, что здесь СЕРТИФИЦИРОВАНО.
INTERVAL_CERTIFIED = "CERTIFIED"

#: Отказы целой модели и кадра.
REFUSED_KEY = "PREVIEW_KEY_MISMATCH"
REFUSED_NO_OTHER_SAMPLE = "PREVIEW_NO_OTHER_EXACT_SAMPLE"
REFUSED_NO_MODELLED_DOMAIN = "PREVIEW_NO_MODELLED_DOMAIN"
REFUSED_NO_MODEL = "PREVIEW_NO_MODEL"
REFUSED_ALPHA = "PREVIEW_ALPHA_NOT_POSITIVE_FINITE"
REFUSED_OUTSIDE_EVERY_INTERVAL = "PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL"
REFUSED_ALL_QUARANTINED = "PREVIEW_ALL_DOMAINS_QUARANTINED"
REFUSED_NOT_INDEPENDENT = "PREVIEW_CHECK_SAMPLE_NOT_INDEPENDENT"

#: Исходы домена при построении (`MODELLED` либо причина, по которой домен остаётся на последней точной геометрии).
DOMAIN_MODELLED = "MODELLED"
DOMAIN_NOT_MATERIALIZED = "PREVIEW_DOMAIN_NOT_MATERIALIZED"
DOMAIN_NO_INTERVAL = "PREVIEW_DOMAIN_INTERVAL_NOT_CERTIFIED"
DOMAIN_MISSING = "PREVIEW_DOMAIN_MISSING_IN_SAMPLE"
DOMAIN_OUTSIDE = "PREVIEW_DOMAIN_SAMPLE_OUTSIDE_INTERVAL"
DOMAIN_STRUCTURE_SWITCH = "PREVIEW_DOMAIN_STRUCTURE_SWITCH"
#: Исходы домена в кадре.
DOMAIN_HELD_OUTSIDE = "PREVIEW_DOMAIN_ALPHA_OUTSIDE_INTERVAL"
DOMAIN_HELD_REACH = "PREVIEW_DOMAIN_BEYOND_MODEL_REACH"
#: Исходы журнала доверия: карантин после опровержения точным прогоном, возврат со сжатой областью, снятие модели домена.
DOMAIN_QUARANTINED = "PREVIEW_DOMAIN_QUARANTINED_AFTER_REFUTATION"
DOMAIN_READMITTED = "PREVIEW_DOMAIN_READMITTED_WITH_REDUCED_REACH"
DOMAIN_ABANDONED = "PREVIEW_DOMAIN_ABANDONED_AFTER_REFUTATIONS"

#: Состояния домена в журнале доверия.
TRUST_QUARANTINED = "QUARANTINED"
TRUST_REDUCED = "REDUCED"
TRUST_ABANDONED = "ABANDONED"

#: Кривизна, добавка которой на опорных ширинах не выше стольких единиц последнего разряда величины, - шум округления, а не кривизна.
CURVATURE_NOISE_ULPS = 64
#: Начальная доверительная область модели, доля ширины базы: домен со значимой кривизной (квадрат) и домен, у которого кривизну проверить нечем
#: (прямая). Это решение хоста о начальном доверии, а не доказанная граница: опровержение сжимает область (`advance_ledger`).
CURVED_REACH_RATIO = 0.10
CHORD_REACH_RATIO = 0.03
_EPSILON = float(np.finfo(np.float64).eps)

#: Отклонение превью от точного прогона, при котором домен опровергнут: доля ширины (метры у позиций, единицы тайла у UV). Это решение
#: хоста о том, что глазу ещё неважно (полпроцента ширины полосы), а не допуск ядра; число пишется в запись самопроверки.
DEVIATION_LIMIT_RATIO = 0.005

#: Правило повторного допуска (см. «ЖУРНАЛ ДОВЕРИЯ»): во сколько раз сжимается область за каждое опровержение, какую долю расстояния
#: опровержения она не переходит, сколько чистых независимых сверок в тени возвращают домен, какая доля предела ошибки считается чистой,
#: на каком расстоянии от базы (доля будущей досягаемости) сверка чиста и сколько опровержений снимают модель домена.
TRUST_SCALE_PER_REFUTATION = 0.5
TRUST_CAP_FRACTION = 0.5
READMIT_CLEAN_CHECKS = 1
READMIT_ERROR_FRACTION = 0.5
READMIT_MIN_DISTANCE_FRACTION = 0.5
ABANDON_AFTER_REFUTATIONS = 3

_ALIVE_NEVER_LOW = math.inf
_ALIVE_NEVER_HIGH = -math.inf


@dataclass(frozen=True, slots=True, eq=False)
class DomainSampleV1:
    """Домен точного прогона на меше: что нужно модели (вершины меша, петли, интервал, токены структуры)."""

    patch_id: int
    domain_id: str
    token: str
    structure: str
    #: Статус заверенного интервала (`CERTIFIED`, `AT_EVENT`, `NOT_CERTIFIED`) либо `""`, если записи нет.
    status: str
    low: float
    high: float | None
    #: Номера вершин МЕША по локальным вершинам домена (порядок ключей батча).
    vertices: object
    loop_start: int
    loop_count: int
    #: Домен-значение для точной проверки вхождения (`AlphaIntervalV1`) либо `None`.
    interval: object | None = None


@dataclass(frozen=True, slots=True, eq=False)
class PreviewSampleV1:
    """Точный прогон в виде, в котором модель его читает: массивы меша и домены. Неизменяем; ключ — чья это геометрия."""

    key: tuple
    alpha_text: str
    alpha: float
    positions: object
    uvs: object
    domains: tuple
    arrays_digest: str
    source_revision: str

    @property
    def vertex_count(self) -> int:
        return int(self.positions.shape[0])

    @property
    def loop_count(self) -> int:
        return int(self.uvs.shape[0])

    @property
    def alpha_fraction(self) -> Fraction:
        return Fraction(self.alpha_text)

    @property
    def nbytes(self) -> int:
        return int(self.positions.nbytes + self.uvs.nbytes + sum(item.vertices.nbytes for item in self.domains))


@dataclass(frozen=True, slots=True)
class PreviewRefusalV1:
    """Отказ построения: имя и деталь (не молчание)."""

    outcome: str
    detail: str = ""


@dataclass(frozen=True, slots=True)
class DomainTrustV1:
    """Что журнал знает о доверии к модели одного домена: состояние, число опровержений, сжатие области и чистые сверки."""

    state: str
    refutations: int
    #: Множитель номинальной досягаемости: `TRUST_SCALE_PER_REFUTATION ** refutations`.
    scale: float
    #: Верхняя граница досягаемости (доля ширины базы), найденная опровержением: `TRUST_CAP_FRACTION` расстояния, на котором оно случилось.
    reach_cap: float
    #: Независимых чистых сверок в тени с последнего опровержения.
    clean_checks: int = 0


@dataclass(frozen=True, slots=True, eq=False)
class TrustLedgerV1:
    """Журнал доверия по ключу образца: `(патч, домен) -> DomainTrustV1`. Неизменяем по соглашению: `advance_ledger` даёт новый."""

    key: tuple
    entries: dict

    def get(self, patch_id: int, domain_id: str) -> DomainTrustV1 | None:
        return self.entries.get((int(patch_id), str(domain_id)))

    def counts(self) -> dict:
        found: dict = {}
        for entry in self.entries.values():
            found[entry.state] = found.get(entry.state, 0) + 1
        return found


def empty_ledger(key: tuple) -> TrustLedgerV1:
    return TrustLedgerV1(tuple(key), {})


@dataclass(frozen=True, slots=True, eq=False)
class PreviewModelV1:
    """Многочлены позиций и UV по вершинам и петлям базы, окна жизни доменов, исходы доменов и карантин. ПРИБЛИЗИТЕЛЬНАЯ модель."""

    base: PreviewSampleV1
    #: Ширины других точных прогонов, на которых построено (одна — прямая, две — квадрат).
    others: tuple
    a1: object
    a2: object | None
    e1: object
    e2: object | None
    vlow: object
    vhigh: object
    #: Число петель UV у каждого домена базы (петли домена лежат в `uvs` подряд, в порядке доменов): по нему живость домена
    #: раскладывается на петли без индекса на каждую петлю.
    loop_counts: object
    dlow: object
    dhigh: object
    #: `((патч, домен, исход, степень, досягаемость), ...)` по доменам базы: `MODELLED` со степенью 1 (прямая) либо 2 (квадрат) и
    #: досягаемостью (доля ширины базы либо `None` — только интервал), `QUARANTINED`/`ABANDONED` с той же подгонкой (по ней идёт сверка в
    #: тени), либо причина со степенью 0.
    rows: tuple
    #: Карантин: булев массив по доменам и по вершинам (`None` — карантина нет). Окна `dlow`/`dhigh` и `vlow`/`vhigh` его не знают: сверка в
    #: тени считает предсказание по ним, а кадр двигает только то, что не в карантине.
    quarantined: object | None = None
    vertex_quarantined: object | None = None
    #: Окно ТЕНИ по доменам (интервал ядра и НОМИНАЛЬНАЯ область, не сжатая журналом): где предсказание карантинного домена судится в тени.
    shadow_low: object | None = None
    shadow_high: object | None = None

    @property
    def key(self) -> tuple:
        return self.base.key

    @property
    def base_alpha(self) -> float:
        return self.base.alpha

    @property
    def modelled_domains(self) -> int:
        return sum(1 for row in self.rows if row[2] == DOMAIN_MODELLED)

    @property
    def domain_count(self) -> int:
        return len(self.rows)

    @property
    def quadratic_domains(self) -> int:
        return sum(1 for row in self.rows if row[2] == DOMAIN_MODELLED and row[3] == 2)

    def reason_counts(self) -> dict:
        found: dict = {}
        for row in self.rows:
            if row[2] != DOMAIN_MODELLED:
                found[row[2]] = found.get(row[2], 0) + 1
        return found

    @property
    def own_bytes(self) -> int:
        """Байты самой модели (многочлены, окна, индекс петель, карантин); база считается отдельно: она нужна и без неё."""

        arrays = (
            self.a1, self.a2, self.e1, self.e2, self.vlow, self.vhigh, self.loop_counts, self.dlow, self.dhigh,
            self.quarantined, self.vertex_quarantined, self.shadow_low, self.shadow_high,
        )
        return int(sum(item.nbytes for item in arrays if item is not None))

    @property
    def nbytes(self) -> int:
        return self.own_bytes + self.base.nbytes


@dataclass(frozen=True, slots=True)
class PreviewFrameV1:
    """Кадр превью: готовые к записи плоские массивы float32, живые и придержанные домены, отказ (пусто — кадр есть)."""

    alpha: float
    positions: object
    uvs: object
    live_domains: int
    held_domains: int
    #: Номера патчей придержанных доменов в порядке базы и их исход.
    held: tuple
    refusal: str = ""
    seconds: float = 0.0

    @property
    def outcome(self) -> str:
        return self.refusal or PREVIEW_MESH_FROM_INTERVAL_V1


@dataclass(frozen=True, slots=True)
class DeviationV1:
    """Апостериорная сверка предсказания модели с точным прогоном: наибольшие отклонения, судимые домены и опровергнутые."""

    alpha: float
    max_position: float
    max_uv: float
    domains_checked: int
    domains_skipped: int
    worst_patch: int
    #: Номера доменов (в `rows`), отклонение которых выше предела.
    refuted: tuple
    refusal: str = ""
    #: Карантинные домены: номера (в `rows`), чьё предсказание в тени чисто (в пределах доли предела) и чьё выше предела.
    shadow_clean: tuple = ()
    shadow_refuted: tuple = ()
    #: Расстояние ширины прогона от базы модели, доля ширины базы; наибольшее отношение отклонения к пределу среди судимых доменов.
    relative_distance: float = 0.0
    worst_ratio: float = 0.0


# --------------------------------------------------------------------------
# Прогон -> образец
# --------------------------------------------------------------------------


def sample_of(results, arrays, *, key: tuple, alpha_text: str) -> PreviewSampleV1:
    """Образец точного прогона: `arrays` — массивы меша тех же `results` (`build_mesh_arrays`), `key` — чья это геометрия.

    Домены берутся в порядке меша (`arrays.domain_keys`); запись интервала и подпись структуры читаются из результата домена.
    """

    by_key = {(item.patch_id, item.domain_id): item for item in results}
    domains = []
    start = 0
    for number, (patch_id, domain_id) in enumerate(arrays.domain_keys):
        result = by_key[(patch_id, domain_id)]
        interval = getattr(result, "alpha_interval", None)
        count = int(arrays.domain_loop_counts[number])
        domains.append(
            DomainSampleV1(
                patch_id=int(patch_id),
                domain_id=str(domain_id),
                token=str(arrays.domain_tokens[number]),
                structure=str(getattr(result, "structure_digest", "")),
                status="" if interval is None else str(interval.status),
                low=float("nan") if interval is None else float(interval.low),
                high=None if interval is None or interval.high is None else float(interval.high),
                vertices=np.asarray(arrays.domain_vertex_index[number], dtype=np.int32),
                loop_start=start,
                loop_count=count,
                interval=interval,
            )
        )
        start += count
    positions = np.asarray(arrays.positions, dtype=np.float64).reshape(-1, 3)
    uvs = np.asarray(arrays.uvs, dtype=np.float64).reshape(-1, 2)
    return PreviewSampleV1(
        key=tuple(key),
        alpha_text=str(alpha_text),
        alpha=float(alpha_text),
        positions=positions,
        uvs=uvs,
        domains=tuple(domains),
        arrays_digest=str(arrays.digest),
        source_revision=str(arrays.source_revision),
    )


# --------------------------------------------------------------------------
# Журнал доверия
# --------------------------------------------------------------------------


def advance_ledger(ledger: TrustLedgerV1 | None, model: PreviewModelV1 | None, check: DeviationV1 | None):
    """`(новый журнал, события)` после апостериорной сверки `check` модели `model` с точным прогоном.

    События — `(патч, домен, имя)`: `DOMAIN_QUARANTINED`, `DOMAIN_ABANDONED`, `DOMAIN_READMITTED` (правило — в шапке модуля). Журнал другого
    ключа забыт: он говорил про другую геометрию. Сверка без вердикта (`refusal`) журнал не меняет.
    """

    if model is None:
        return (ledger if ledger is not None else empty_ledger(())), ()
    if ledger is None or ledger.key != model.key:
        ledger = empty_ledger(model.key)
    if check is None or check.refusal:
        return ledger, ()
    entries = dict(ledger.entries)
    events = []

    def refute(number: int) -> None:
        patch_id, domain_id = model.rows[number][0], model.rows[number][1]
        prior = entries.get((patch_id, domain_id))
        refutations = 1 if prior is None else prior.refutations + 1
        cap = min(math.inf if prior is None else prior.reach_cap, TRUST_CAP_FRACTION * check.relative_distance)
        abandoned = refutations >= ABANDON_AFTER_REFUTATIONS
        entries[(patch_id, domain_id)] = DomainTrustV1(
            TRUST_ABANDONED if abandoned else TRUST_QUARANTINED, refutations, TRUST_SCALE_PER_REFUTATION**refutations, cap
        )
        events.append((patch_id, domain_id, DOMAIN_ABANDONED if abandoned else DOMAIN_QUARANTINED))

    for number in (*check.refuted, *check.shadow_refuted):
        refute(int(number))
    for number in check.shadow_clean:
        patch_id, domain_id = model.rows[number][0], model.rows[number][1]
        entry = entries.get((patch_id, domain_id))
        if entry is None or entry.state != TRUST_QUARANTINED:
            continue
        clean = entry.clean_checks + 1
        if clean >= READMIT_CLEAN_CHECKS:
            entries[(patch_id, domain_id)] = replace(entry, state=TRUST_REDUCED, clean_checks=clean)
            events.append((patch_id, domain_id, DOMAIN_READMITTED))
        else:
            entries[(patch_id, domain_id)] = replace(entry, clean_checks=clean)
    if not events and entries == ledger.entries:
        return ledger, ()
    return TrustLedgerV1(ledger.key, entries), tuple(events)


# --------------------------------------------------------------------------
# Построение
# --------------------------------------------------------------------------


def _domain_verdict(base: PreviewSampleV1, domain: DomainSampleV1, other: PreviewSampleV1, lookup: dict) -> tuple[str, object | None]:
    """`(MODELLED, домен другого прогона)` либо `(причина, None)`: условия модели одного домена по одному другому прогону."""

    if not domain.token:
        return DOMAIN_NOT_MATERIALIZED, None
    if domain.status != INTERVAL_CERTIFIED or domain.interval is None:
        return DOMAIN_NO_INTERVAL, None
    twin = lookup.get((domain.patch_id, domain.domain_id))
    if twin is None:
        return DOMAIN_MISSING, None
    if not domain.interval.contains_exact(other.alpha_fraction):
        return DOMAIN_OUTSIDE, None
    if twin.token != domain.token or twin.structure != domain.structure or twin.vertices.shape != domain.vertices.shape:
        return DOMAIN_STRUCTURE_SWITCH, None
    if twin.loop_count != domain.loop_count:
        return DOMAIN_STRUCTURE_SWITCH, None
    return DOMAIN_MODELLED, twin


def _newton(first, second, third, alpha0, alpha1, alpha2):
    """`(a1, a2, кривизна значима)` многочлена `v0 + a1 * dt + a2 * dt^2` через три точки: `third` `None` — прямая (`a2` `None`).

    Вторая разделённая разность аффинной величины — шум округления (`~eps * |v| / шаг^2`), и квадрат на нём дал бы кривизну там, где её
    нет: добавка `|a2| * шаг1 * шаг2` не выше `CURVATURE_NOISE_ULPS` единиц последнего разряда — шум, и `a2` в нём нулевой.
    """

    c1 = (second - first) / (alpha1 - alpha0)
    if third is None:
        return c1, None, False
    c2 = (((third - first) / (alpha2 - alpha0)) - c1) / (alpha2 - alpha1)
    noise = CURVATURE_NOISE_ULPS * _EPSILON * np.maximum(1.0, np.abs(first))
    significant = np.abs(c2) * abs((alpha1 - alpha0) * (alpha2 - alpha0)) > noise
    c2 = np.where(significant, c2, 0.0)
    return c1 + c2 * (alpha0 - alpha1), c2, bool(significant.any())


def _trusted_reach(reach, entry: DomainTrustV1 | None):
    """Начальная область, сжатая записью журнала: `min(номинал * масштаб, предел опровержения)`; аффинный домен получает 10 %."""

    if entry is None:
        return reach
    nominal = CURVED_REACH_RATIO if reach is None else reach
    return min(nominal * entry.scale, entry.reach_cap)


def build_model(base: PreviewSampleV1, others, ledger: TrustLedgerV1 | None = None) -> PreviewModelV1 | PreviewRefusalV1:
    """Модель базы по другим точным прогонам того же ключа: два ближайших по ширине, если они есть; иначе отказ именем.

    `ledger` — журнал доверия сессии: домены в карантине (или снятые) построены, но придержаны; вернувшиеся двигаются в сжатой области.
    """

    usable = [item for item in others if item.key == base.key and item.alpha != base.alpha]
    if not usable:
        mismatched = any(item.key != base.key for item in others)
        return PreviewRefusalV1(REFUSED_KEY if mismatched and not any(item.key == base.key for item in others) else REFUSED_NO_OTHER_SAMPLE)
    usable.sort(key=lambda item: abs(item.alpha - base.alpha))
    chosen = usable[:2]
    lookups = [{(item.patch_id, item.domain_id): item for item in sample.domains} for sample in chosen]
    count_v, count_l, count_d = base.vertex_count, base.loop_count, len(base.domains)
    trust_of = ledger.get if ledger is not None and ledger.key == base.key else (lambda _patch, _domain: None)
    a1 = np.zeros((count_v, 3))
    a2 = np.zeros((count_v, 3))
    e1 = np.zeros((count_l, 2))
    e2 = np.zeros((count_l, 2))
    dlow = np.full(count_d, _ALIVE_NEVER_LOW)
    dhigh = np.full(count_d, _ALIVE_NEVER_HIGH)
    quarantined = np.zeros(count_d, dtype=bool)
    shadow_low = np.full(count_d, _ALIVE_NEVER_LOW)
    shadow_high = np.full(count_d, _ALIVE_NEVER_HIGH)
    rows = []
    modelled = []
    alpha0 = base.alpha
    quadratic = False
    for number, domain in enumerate(base.domains):
        verdicts = [_domain_verdict(base, domain, other, lookup) for other, lookup in zip(chosen, lookups)]
        good = [(other, twin) for other, (reason, twin) in zip(chosen, verdicts) if reason == DOMAIN_MODELLED]
        if not good:
            reason = verdicts[0][0] if verdicts[0][0] != DOMAIN_MODELLED else verdicts[-1][0]
            rows.append((domain.patch_id, domain.domain_id, reason, 0, None))
            continue
        (first, first_twin), second = good[0], (good[1] if len(good) == 2 else None)
        alpha1 = first.alpha
        alpha2 = None if second is None else second[0].alpha
        sl = slice(domain.loop_start, domain.loop_start + domain.loop_count)
        p0 = base.positions[domain.vertices]
        step = _newton(
            p0,
            first.positions[first_twin.vertices],
            None if second is None else second[0].positions[second[1].vertices],
            alpha0,
            alpha1,
            alpha2,
        )
        a1[domain.vertices] = step[0]
        if step[1] is not None:
            a2[domain.vertices] = step[1]
        u0 = base.uvs[sl]
        s_first = first.uvs[first_twin.loop_start : first_twin.loop_start + first_twin.loop_count] * alpha1
        s_second = (
            None
            if second is None
            else second[0].uvs[second[1].loop_start : second[1].loop_start + second[1].loop_count] * alpha2
        )
        slope, curve, uv_curved = _newton(u0 * alpha0, s_first, s_second, alpha0, alpha1, alpha2)
        e1[sl] = slope - u0
        if curve is not None:
            e2[sl] = curve
        degree = 2 if second is not None else 1
        reach = CHORD_REACH_RATIO if second is None else (CURVED_REACH_RATIO if step[2] or uv_curved else None)
        entry = trust_of(domain.patch_id, domain.domain_id)
        nominal_reach = reach
        reach = _trusted_reach(reach, entry)
        outcome = DOMAIN_MODELLED
        low = domain.low
        high = math.inf if domain.high is None else domain.high
        if entry is not None and entry.state in (TRUST_QUARANTINED, TRUST_ABANDONED):
            outcome = DOMAIN_QUARANTINED if entry.state == TRUST_QUARANTINED else DOMAIN_ABANDONED
            quarantined[number] = True
            shadow_reach = CURVED_REACH_RATIO if nominal_reach is None else nominal_reach
            shadow_low[number], shadow_high[number] = max(low, alpha0 * (1.0 - shadow_reach)), min(high, alpha0 * (1.0 + shadow_reach))
        if reach is not None:
            low, high = max(low, alpha0 * (1.0 - reach)), min(high, alpha0 * (1.0 + reach))
        dlow[number], dhigh[number] = low, high
        quadratic = quadratic or degree == 2
        rows.append((domain.patch_id, domain.domain_id, outcome, degree, reach))
        modelled.append(number)
    if not modelled:
        return PreviewRefusalV1(REFUSED_NO_MODELLED_DOMAIN, ", ".join(f"{name}={value}" for name, value in sorted(_reasons(rows).items())))
    vlow, vhigh, vertex_quarantined, frozen = _vertex_windows(base, modelled, dlow, dhigh, quarantined)
    a1[frozen] = 0.0
    a2[frozen] = 0.0
    held_back = bool(quarantined.any())
    return PreviewModelV1(
        base=base,
        others=tuple(item.alpha for item in chosen),
        a1=a1,
        # Кривизна нужна на малой доле смещения (`dt^2`, `dt` в сотые доли метра): float32 хранит её с ошибкой ниже
        # float32-хранилища самого меша, и память модели делится на два.
        a2=a2.astype(np.float32) if quadratic else None,
        e1=e1,
        e2=e2.astype(np.float32) if quadratic else None,
        vlow=vlow,
        vhigh=vhigh,
        loop_counts=np.asarray([item.loop_count for item in base.domains], dtype=np.int32),
        dlow=dlow,
        dhigh=dhigh,
        rows=tuple(rows),
        quarantined=quarantined if held_back else None,
        vertex_quarantined=vertex_quarantined if held_back else None,
        shadow_low=shadow_low if held_back else None,
        shadow_high=shadow_high if held_back else None,
    )


def _vertex_windows(base: PreviewSampleV1, modelled, dlow, dhigh, quarantined):
    """`(vlow, vhigh, вершины в карантине, замороженные)`: окно вершины — пересечение окон её доменов, и все её домены должны быть смоделированы.

    Вершина, принадлежащая хотя бы одному несмоделированному домену, заморожена (никогда не жива); вершина карантинного домена помечена отдельно: окно
    у неё есть (по нему считается тень), а кадр её не двигает.
    """

    count_v = base.vertex_count
    total = np.zeros(count_v, dtype=np.int32)
    alive = np.zeros(count_v, dtype=np.int32)
    vlow = np.full(count_v, -math.inf)
    vhigh = np.full(count_v, math.inf)
    vertex_quarantined = np.zeros(count_v, dtype=bool)
    live_set = set(modelled)
    for number, domain in enumerate(base.domains):
        np.add.at(total, domain.vertices, 1)
        if number in live_set:
            np.add.at(alive, domain.vertices, 1)
            np.maximum.at(vlow, domain.vertices, dlow[number])
            np.minimum.at(vhigh, domain.vertices, dhigh[number])
            if quarantined[number]:
                vertex_quarantined[domain.vertices] = True
    frozen = alive != total
    vlow[frozen], vhigh[frozen] = _ALIVE_NEVER_LOW, _ALIVE_NEVER_HIGH
    return vlow, vhigh, vertex_quarantined, frozen


def _reasons(rows) -> dict:
    found: dict = {}
    for row in rows:
        if row[2] != DOMAIN_MODELLED:
            found[row[2]] = found.get(row[2], 0) + 1
    return found


# --------------------------------------------------------------------------
# Кадр
# --------------------------------------------------------------------------


def _live_masks(model: PreviewModelV1, alpha: float):
    live_vertex = (model.vlow < alpha) & (alpha < model.vhigh)
    live_domain = (model.dlow < alpha) & (alpha < model.dhigh)
    if model.quarantined is not None:
        live_vertex &= ~model.vertex_quarantined
        live_domain &= ~model.quarantined
    return live_vertex, live_domain, np.repeat(live_domain, model.loop_counts)


def shadow_prediction(model: PreviewModelV1, number: int, alpha: float):
    """`(позиции вершин домена, UV его петель)` по многочленам модели на ширине `alpha`, как двигался бы домен, будь он допущен.

    Предсказание «в тени» карантинного домена: в кадр оно не идёт, им судится возврат домена (`deviation`). Формулы те же, что у кадра.
    """

    base = model.base
    domain = base.domains[number]
    dt = alpha - base.alpha
    vertices = domain.vertices
    positions = base.positions[vertices] + model.a1[vertices] * dt
    if model.a2 is not None:
        positions = positions + model.a2[vertices] * (dt * dt)
    sl = slice(domain.loop_start, domain.loop_start + domain.loop_count)
    polynomial = model.e1[sl] if model.e2 is None else model.e1[sl] + model.e2[sl] * dt
    return positions, base.uvs[sl] + polynomial * (dt / alpha)


def positions_and_uvs(model: PreviewModelV1, alpha: float):
    """`(позиции (N, 3), UV (L, 2), маска живых доменов)` float64 на ширине `alpha`; на ширине базы — сама база."""

    base = model.base
    live_vertex, live_domain, live_loop = _live_masks(model, alpha)
    dt = alpha - base.alpha
    if dt == 0.0:
        return base.positions, base.uvs, live_domain
    moved = np.where(live_vertex, dt, 0.0)
    positions = base.positions + model.a1 * moved[:, None]
    if model.a2 is not None:
        positions = positions + model.a2 * (moved * dt)[:, None]
    scale = np.where(live_loop, dt / alpha, 0.0)
    polynomial = model.e1 if model.e2 is None else model.e1 + model.e2 * dt
    uvs = base.uvs + polynomial * scale[:, None]
    return positions, uvs, live_domain


def evaluate(model: PreviewModelV1 | None, alpha: float) -> PreviewFrameV1:
    """Кадр превью на ширине `alpha`: плоские float32 для `foreach_set`, живые и придержанные домены, отказ именем."""

    import time

    started = time.perf_counter()
    if model is None:
        return PreviewFrameV1(float(alpha), None, None, 0, 0, (), REFUSED_NO_MODEL)
    base = model.base
    alpha = float(alpha)
    if not (math.isfinite(alpha) and alpha > 0.0):
        return PreviewFrameV1(alpha, _flat32(base.positions), _flat32(base.uvs), 0, len(model.rows), (), REFUSED_ALPHA)
    positions, uvs, live = positions_and_uvs(model, alpha)
    held = tuple(_held_reason(model, number, alpha) for number in np.flatnonzero(~live))
    live_count = int(live.sum())
    refusal = _empty_frame_refusal(held) if live_count == 0 else ""
    return PreviewFrameV1(
        alpha,
        _flat32(positions),
        _flat32(uvs),
        live_count,
        len(held),
        held,
        refusal,
        time.perf_counter() - started,
    )


def _empty_frame_refusal(held: tuple) -> str:
    """Имя отказа кадра без живых доменов: все придержаны карантином — `PREVIEW_ALL_DOMAINS_QUARANTINED`, иначе ширина за областью."""

    if held and all(reason in (DOMAIN_QUARANTINED, DOMAIN_ABANDONED) for _patch, reason in held):
        return REFUSED_ALL_QUARANTINED
    return REFUSED_OUTSIDE_EVERY_INTERVAL


def _held_reason(model: PreviewModelV1, number: int, alpha: float) -> tuple:
    """`(патч, исход)` придержанного домена: причина из построения, карантин, либо за интервалом ядра, либо за досягаемостью модели."""

    row = model.rows[number]
    if row[2] != DOMAIN_MODELLED:
        return row[0], row[2]
    domain = model.base.domains[number]
    inside_interval = domain.low < alpha and (domain.high is None or alpha < domain.high)
    return row[0], DOMAIN_HELD_REACH if inside_interval else DOMAIN_HELD_OUTSIDE


def base_frame(sample: PreviewSampleV1) -> PreviewFrameV1:
    """Кадр самой базы: геометрия, как её записал точный путь (возврат при отмене); все домены живы, придержанных нет."""

    return PreviewFrameV1(sample.alpha, _flat32(sample.positions), _flat32(sample.uvs), len(sample.domains), 0, ())


def _flat32(values):
    return np.ascontiguousarray(values, dtype=np.float32).reshape(-1)


# --------------------------------------------------------------------------
# Самопроверка
# --------------------------------------------------------------------------


def deviation(model: PreviewModelV1 | None, sample: PreviewSampleV1) -> DeviationV1:
    """Что модель предсказала бы на ширине точного прогона `sample` и насколько это отличается от него, домен за доменом.

    Судятся домены, живые на этой ширине, чей токен и размеры те же. Предел — `DEVIATION_LIMIT_RATIO * alpha`; домены выше предела
    названы в `refuted` (номера в `rows`), и `advance_ledger` вводит их в карантин. Карантинные домены, чья область покрывает ширину,
    судятся «в тени» (`shadow_clean`, `shadow_refuted`): так домен возвращается либо снимается. Прогон другого ключа не судится
    (`PREVIEW_KEY_MISMATCH`), прогон на опорной ширине модели — тоже (`PREVIEW_CHECK_SAMPLE_NOT_INDEPENDENT`). Это ВЫБОРКА в одной
    точке, а не граница ошибки.
    """

    if model is None:
        return DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), REFUSED_NO_MODEL)
    if sample.key != model.key:
        return DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), REFUSED_KEY)
    alpha = sample.alpha
    if alpha == model.base.alpha or alpha in model.others:
        return DeviationV1(alpha, 0.0, 0.0, 0, 0, -1, (), REFUSED_NOT_INDEPENDENT)
    distance = abs(alpha - model.base.alpha) / model.base.alpha
    positions, uvs, live = positions_and_uvs(model, alpha)
    twins = {(item.patch_id, item.domain_id): item for item in sample.domains}
    limit = DEVIATION_LIMIT_RATIO * alpha
    worst_position = worst_uv = worst_ratio = 0.0
    worst_patch = -1
    checked = skipped = 0
    refuted, shadow_clean, shadow_refuted = [], [], []

    def errors(domain, twin, source_positions, source_uvs):
        position_error = float(np.max(np.abs(source_positions - sample.positions[twin.vertices]), initial=0.0))
        uv_error = float(
            np.max(
                np.abs(source_uvs - sample.uvs[twin.loop_start : twin.loop_start + twin.loop_count]),
                initial=0.0,
            )
        )
        return position_error, uv_error

    for number, domain in enumerate(model.base.domains):
        in_shadow = (
            not live[number]
            and model.shadow_low is not None
            and model.rows[number][2] == DOMAIN_QUARANTINED
            and model.shadow_low[number] < alpha < model.shadow_high[number]
        )
        if not live[number] and not in_shadow:
            continue
        twin = twins.get((domain.patch_id, domain.domain_id))
        if twin is None or twin.token != domain.token or twin.vertices.shape != domain.vertices.shape or twin.loop_count != domain.loop_count:
            skipped += 1
            continue
        if in_shadow:
            position_error, uv_error = errors(domain, twin, *shadow_prediction(model, number, alpha))
            if position_error > limit or uv_error > DEVIATION_LIMIT_RATIO:
                shadow_refuted.append(number)
            elif (
                position_error <= READMIT_ERROR_FRACTION * limit
                and uv_error <= READMIT_ERROR_FRACTION * DEVIATION_LIMIT_RATIO
                and distance >= READMIT_MIN_DISTANCE_FRACTION * model.rows[number][4]
            ):
                shadow_clean.append(number)
            continue
        checked += 1
        position_error, uv_error = errors(
            domain,
            twin,
            positions[domain.vertices],
            uvs[domain.loop_start : domain.loop_start + domain.loop_count],
        )
        if position_error > worst_position:
            worst_position, worst_patch = position_error, domain.patch_id
        worst_uv = max(worst_uv, uv_error)
        worst_ratio = max(worst_ratio, position_error / limit, uv_error / DEVIATION_LIMIT_RATIO)
        if position_error > limit or uv_error > DEVIATION_LIMIT_RATIO:
            refuted.append(number)
    return DeviationV1(
        alpha, worst_position, worst_uv, checked, skipped, worst_patch, tuple(refuted), "",
        tuple(shadow_clean), tuple(shadow_refuted), distance, worst_ratio,
    )


__all__ = (
    "ABANDON_AFTER_REFUTATIONS",
    "CHORD_REACH_RATIO",
    "CURVATURE_NOISE_ULPS",
    "CURVED_REACH_RATIO",
    "DEVIATION_LIMIT_RATIO",
    "DOMAIN_ABANDONED",
    "DOMAIN_HELD_OUTSIDE",
    "DOMAIN_HELD_REACH",
    "DOMAIN_MISSING",
    "DOMAIN_MODELLED",
    "DOMAIN_NOT_MATERIALIZED",
    "DOMAIN_NO_INTERVAL",
    "DOMAIN_OUTSIDE",
    "DOMAIN_QUARANTINED",
    "DOMAIN_READMITTED",
    "DOMAIN_STRUCTURE_SWITCH",
    "DeviationV1",
    "DomainSampleV1",
    "DomainTrustV1",
    "INTERVAL_CERTIFIED",
    "PREVIEW_MESH_FROM_INTERVAL_V1",
    "PreviewFrameV1",
    "PreviewModelV1",
    "PreviewRefusalV1",
    "PreviewSampleV1",
    "READMIT_CLEAN_CHECKS",
    "READMIT_ERROR_FRACTION",
    "READMIT_MIN_DISTANCE_FRACTION",
    "REFUSED_ALL_QUARANTINED",
    "REFUSED_ALPHA",
    "REFUSED_KEY",
    "REFUSED_NOT_INDEPENDENT",
    "REFUSED_NO_MODEL",
    "REFUSED_NO_MODELLED_DOMAIN",
    "REFUSED_NO_OTHER_SAMPLE",
    "REFUSED_OUTSIDE_EVERY_INTERVAL",
    "TRUST_ABANDONED",
    "TRUST_CAP_FRACTION",
    "TRUST_QUARANTINED",
    "TRUST_REDUCED",
    "TRUST_SCALE_PER_REFUTATION",
    "TrustLedgerV1",
    "advance_ledger",
    "base_frame",
    "build_model",
    "deviation",
    "empty_ledger",
    "evaluate",
    "positions_and_uvs",
    "sample_of",
    "shadow_prediction",
)
