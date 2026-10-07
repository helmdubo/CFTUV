"""Сертификат превью меша ширины: настоящий меш декали двигается между точными пересчётами, и каждое такое движение названо.

ЗАЧЕМ. Пока владелец тянет ширину, точный пересчёт стоит секунду и больше, а линии превью (`envelope_width_preview`) — не меш. Сертификат
позволяет за миллисекунды писать в СУЩЕСТВУЮЩИЙ меш декали позиции и UV на промежуточной ширине, пока точный счёт идёт в фоне, и точный
результат потом заменяет превью. Исход назван `PREVIEW_MESH_FROM_INTERVAL_V1`: это превью, а не сертифицированная геометрия, и каждая
доля, в которой оно не сертифицировано, называется (правило 4 `AGENTS.md`).

ИЗ ЧЕГО СТРОИТСЯ. Из ТОЧНЫХ прогонов продуктового пути, которые уже посчитаны (`PreviewSampleV1`): прогон на экране (база, `alpha0`) и один либо
два других прогона того же ключа (`alpha1`, `alpha2`). Сертификат ничего не считает сам и ядра не знает: он читает массивы меша
(`envelope_production_mesh.MeshArraysV1`) и записанные факты домена (`alpha_interval`, `structure_digest`).

ЧТО В СЕРТИФИКАТЕ, ПО ДОМЕНУ. Домен сертифицирован, когда (1) у него заверенный интервал ширины (`CERTIFIED`), а `alpha1` лежит строго внутри
интервала БАЗЫ (между ними нет событий покрытия и резки: `AlphaIntervalV1.contains_exact`); (2) его структура в обоих прогонах та же
(подпись структуры ядра и токен локальных массивов: грани, швы, ссылки вершин). Иначе он назван причиной (`DOMAIN_*`) и остаётся на последней
точной геометрии. Внутри интервала позиции вершин и произведения `u * alpha`, `v * alpha` у ядра АФФИННЫ по ширине (`kernel/tests/
test_width_affine_premise.py`), кроме вершин на рёбрах источника под резкой: там положение — дробно-линейная функция ширины (пересечение
поворачивающейся прямой с неподвижной), и две точки дают лишь хорду. Поэтому при ДВУХ других прогонах сертификат берёт квадрат через три точки
(`a1 * dt + a2 * dt^2`, ошибка третьего порядка), при одном — прямую; степень каждого домена записана (`degree`). Позиции меша — позиции
батча плюс смещение декали, и смещение вдоль нормали вершины в ширину не аффинно (нормаль развёртки меняется): оно входит в ту же подгонку и
даёт ошибку второго порядка, измеренную, а не заявленную.

ДОСЯГАЕМОСТЬ МОДЕЛИ. Интервал ядра заверяет структуру, а не форму зависимости, поэтому у модели своя граница (`reach`, доля ширины базы):
квадрат без значимой кривизны (добавка на опорных ширинах не выше шума округления, `CURVATURE_NOISE_ULPS`) — аффинный домен, границы нет,
кроме интервала; квадрат со значимой кривизной — `CURVED_REACH_RATIO` (десять процентов: там ошибка третьего порядка укладывается в
`DEVIATION_LIMIT_RATIO`); прямая через две точки кривизны проверить не может — `CHORD_REACH_RATIO` (три процента). За досягаемостью домен
придержан под именем `PREVIEW_DOMAIN_BEYOND_MODEL_REACH`, а не двигается наугад.

ФОРМУЛЫ (ровно как в коде; `dt = alpha - alpha0`, на `dt = 0` ответ — база БИТОВО):
    позиция = P0 + a1 * dt + a2 * dt^2
    uv      = uv0 + (e1 + e2 * dt) * dt / alpha        (произведение `uv * alpha` аффинно, поэтому делитель стоит ВНЕ многочлена)

ГДЕ ПРЕВЬЮ ЖИВО. Вершина двигается, если ВСЕ домены, которым она принадлежит, сертифицированы и `alpha` строго внутри пересечения их
интервалов; петля UV — если жив её домен. Остальные остаются на геометрии базы (последняя точная), и кадр называет их числом и номерами
патчей. Если не жив ни один домен, кадр отказан именем `PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL` и равен базе.

САМОПРОВЕРКА. Каждый новый точный прогон (`deviation`) сверяет то, что сертификат предсказал бы на его ширине, с самим прогоном, домен за доменом;
домен, отклонение которого выше `DEVIATION_LIMIT_RATIO * alpha`, сертификат снимает (`without_domains`, `PREVIEW_DOMAIN_REFUTED_BY_EXACT`).

Модуль чистый: ни `bpy`, ни настроек, ни контроллера; его зовут и поток точного счёта (построить), и главный поток (кадр).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

import numpy as np

PREVIEW_MESH_FROM_INTERVAL_V1 = "PREVIEW_MESH_FROM_INTERVAL_V1"

#: Отказы целого сертификата и кадра.
REFUSED_KEY = "PREVIEW_KEY_MISMATCH"
REFUSED_NO_OTHER_SAMPLE = "PREVIEW_NO_OTHER_EXACT_SAMPLE"
REFUSED_NO_CERTIFIED_DOMAIN = "PREVIEW_NO_CERTIFIED_DOMAIN"
REFUSED_NO_CERTIFICATE = "PREVIEW_NO_CERTIFICATE"
REFUSED_ALPHA = "PREVIEW_ALPHA_NOT_POSITIVE_FINITE"
REFUSED_OUTSIDE_EVERY_INTERVAL = "PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL"

#: Исходы домена при построении (`CERTIFIED` либо причина, по которой домен остаётся на последней точной геометрии).
DOMAIN_CERTIFIED = "CERTIFIED"
DOMAIN_NOT_MATERIALIZED = "PREVIEW_DOMAIN_NOT_MATERIALIZED"
DOMAIN_NO_INTERVAL = "PREVIEW_DOMAIN_INTERVAL_NOT_CERTIFIED"
DOMAIN_MISSING = "PREVIEW_DOMAIN_MISSING_IN_SAMPLE"
DOMAIN_OUTSIDE = "PREVIEW_DOMAIN_SAMPLE_OUTSIDE_INTERVAL"
DOMAIN_STRUCTURE_SWITCH = "PREVIEW_DOMAIN_STRUCTURE_SWITCH"
#: Исходы домена в кадре и после самопроверки.
DOMAIN_HELD_OUTSIDE = "PREVIEW_DOMAIN_ALPHA_OUTSIDE_INTERVAL"
DOMAIN_HELD_REACH = "PREVIEW_DOMAIN_BEYOND_MODEL_REACH"
DOMAIN_REFUTED = "PREVIEW_DOMAIN_REFUTED_BY_EXACT"

#: Кривизна, добавка которой на опорных ширинах не выше стольких единиц последнего разряда величины, - шум округления, а не кривизна.
CURVATURE_NOISE_ULPS = 64
#: Досягаемость модели, доля ширины базы: домен со значимой кривизной (квадрат) и домен, у которого кривизну проверить нечем (прямая).
CURVED_REACH_RATIO = 0.10
CHORD_REACH_RATIO = 0.03
_EPSILON = float(np.finfo(np.float64).eps)

#: Отклонение превью от точного прогона, при котором домен снимается: доля ширины (метры у позиций, единицы тайла у UV). Это решение
#: хоста о том, что глазу ещё неважно (полпроцента ширины полосы), а не допуск ядра; число пишется в запись самопроверки.
DEVIATION_LIMIT_RATIO = 0.005

_ALIVE_NEVER_LOW = math.inf
_ALIVE_NEVER_HIGH = -math.inf


@dataclass(frozen=True, slots=True, eq=False)
class DomainSampleV1:
    """Домен точного прогона на меше: что нужно сертификату (вершины меша, петли, интервал, токены структуры)."""

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
    """Точный прогон в виде, в котором сертификат его читает: массивы меша и домены. Неизменяем; ключ — чья это геометрия."""

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


@dataclass(frozen=True, slots=True, eq=False)
class PreviewCertificateV1:
    """Многочлены позиций и UV по вершинам и петлям базы, живые интервалы и исходы доменов."""

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
    #: `((патч, домен, исход, степень, досягаемость), ...)` по доменам базы: `CERTIFIED` со степенью 1 (прямая) либо 2 (квадрат) и
    #: досягаемостью (доля ширины базы либо `None` — только интервал), либо причина со степенью 0.
    rows: tuple

    @property
    def key(self) -> tuple:
        return self.base.key

    @property
    def base_alpha(self) -> float:
        return self.base.alpha

    @property
    def certified_domains(self) -> int:
        return sum(1 for row in self.rows if row[2] == DOMAIN_CERTIFIED)

    @property
    def domain_count(self) -> int:
        return len(self.rows)

    @property
    def quadratic_domains(self) -> int:
        return sum(1 for row in self.rows if row[2] == DOMAIN_CERTIFIED and row[3] == 2)

    def reason_counts(self) -> dict:
        found: dict = {}
        for row in self.rows:
            if row[2] != DOMAIN_CERTIFIED:
                found[row[2]] = found.get(row[2], 0) + 1
        return found

    @property
    def own_bytes(self) -> int:
        """Байты самого сертификата (многочлены, интервалы, индекс петель); база считается отдельно: она нужна и без него."""

        arrays = (self.a1, self.a2, self.e1, self.e2, self.vlow, self.vhigh, self.loop_counts, self.dlow, self.dhigh)
        return int(sum(item.nbytes for item in arrays if item is not None))

    @property
    def nbytes(self) -> int:
        return self.own_bytes + self.base.nbytes

    def without_domains(self, indices) -> "PreviewCertificateV1":
        """Тот же сертификат, где перечисленные домены (номера в `rows`) сняты исходом `PREVIEW_DOMAIN_REFUTED_BY_EXACT`."""

        wanted = {int(item) for item in indices}
        if not wanted:
            return self
        rows = tuple(
            (row[0], row[1], DOMAIN_REFUTED, 0, None) if number in wanted else row for number, row in enumerate(self.rows)
        )
        dlow, dhigh = self.dlow.copy(), self.dhigh.copy()
        vlow, vhigh = self.vlow.copy(), self.vhigh.copy()
        for number in wanted:
            dlow[number], dhigh[number] = _ALIVE_NEVER_LOW, _ALIVE_NEVER_HIGH
            members = self.base.domains[number].vertices
            vlow[members], vhigh[members] = _ALIVE_NEVER_LOW, _ALIVE_NEVER_HIGH
        return PreviewCertificateV1(
            self.base, self.others, self.a1, self.a2, self.e1, self.e2, vlow, vhigh, self.loop_counts, dlow, dhigh, rows
        )


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
    """Сверка предсказания сертификата с точным прогоном: наибольшие отклонения, судимые домены и снятые."""

    alpha: float
    max_position: float
    max_uv: float
    domains_checked: int
    domains_skipped: int
    worst_patch: int
    #: Номера доменов (в `rows`), отклонение которых выше предела.
    refuted: tuple
    refusal: str = ""


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
# Построение
# --------------------------------------------------------------------------


def _domain_verdict(base: PreviewSampleV1, domain: DomainSampleV1, other: PreviewSampleV1, lookup: dict) -> tuple[str, object | None]:
    """`(CERTIFIED, домен другого прогона)` либо `(причина, None)`: условия сертификации одного домена по одному другому прогону."""

    if not domain.token:
        return DOMAIN_NOT_MATERIALIZED, None
    if domain.status != "CERTIFIED" or domain.interval is None:
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
    return DOMAIN_CERTIFIED, twin


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


def build_certificate(base: PreviewSampleV1, others) -> PreviewCertificateV1 | PreviewRefusalV1:
    """Сертификат базы по другим точным прогонам того же ключа: два ближайших по ширине, если они есть; иначе отказ именем."""

    usable = [item for item in others if item.key == base.key and item.alpha != base.alpha]
    if not usable:
        mismatched = any(item.key != base.key for item in others)
        return PreviewRefusalV1(REFUSED_KEY if mismatched and not any(item.key == base.key for item in others) else REFUSED_NO_OTHER_SAMPLE)
    usable.sort(key=lambda item: abs(item.alpha - base.alpha))
    chosen = usable[:2]
    lookups = [{(item.patch_id, item.domain_id): item for item in sample.domains} for sample in chosen]
    count_v, count_l, count_d = base.vertex_count, base.loop_count, len(base.domains)
    a1 = np.zeros((count_v, 3))
    a2 = np.zeros((count_v, 3))
    e1 = np.zeros((count_l, 2))
    e2 = np.zeros((count_l, 2))
    dlow = np.full(count_d, _ALIVE_NEVER_LOW)
    dhigh = np.full(count_d, _ALIVE_NEVER_HIGH)
    rows = []
    certified = []
    alpha0 = base.alpha
    quadratic = False
    for number, domain in enumerate(base.domains):
        verdicts = [_domain_verdict(base, domain, other, lookup) for other, lookup in zip(chosen, lookups)]
        good = [(other, twin) for other, (reason, twin) in zip(chosen, verdicts) if reason == DOMAIN_CERTIFIED]
        if not good:
            reason = verdicts[0][0] if verdicts[0][0] != DOMAIN_CERTIFIED else verdicts[-1][0]
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
        low = domain.low
        high = math.inf if domain.high is None else domain.high
        if reach is not None:
            low, high = max(low, alpha0 * (1.0 - reach)), min(high, alpha0 * (1.0 + reach))
        dlow[number], dhigh[number] = low, high
        quadratic = quadratic or degree == 2
        rows.append((domain.patch_id, domain.domain_id, DOMAIN_CERTIFIED, degree, reach))
        certified.append(number)
    if not certified:
        return PreviewRefusalV1(REFUSED_NO_CERTIFIED_DOMAIN, ", ".join(f"{name}={value}" for name, value in sorted(_reasons(rows).items())))
    total = np.zeros(count_v, dtype=np.int32)
    alive = np.zeros(count_v, dtype=np.int32)
    vlow = np.full(count_v, -math.inf)
    vhigh = np.full(count_v, math.inf)
    live_set = set(certified)
    for number, domain in enumerate(base.domains):
        np.add.at(total, domain.vertices, 1)
        if number in live_set:
            np.add.at(alive, domain.vertices, 1)
            np.maximum.at(vlow, domain.vertices, dlow[number])
            np.minimum.at(vhigh, domain.vertices, dhigh[number])
    frozen = alive != total
    vlow[frozen], vhigh[frozen] = _ALIVE_NEVER_LOW, _ALIVE_NEVER_HIGH
    a1[frozen] = 0.0
    a2[frozen] = 0.0
    return PreviewCertificateV1(
        base=base,
        others=tuple(item.alpha for item in chosen),
        a1=a1,
        # Кривизна нужна на малой доле смещения (`dt^2`, `dt` в сотые доли метра): float32 хранит её с ошибкой ниже
        # float32-хранилища самого меша, и память сертификата делится на два.
        a2=a2.astype(np.float32) if quadratic else None,
        e1=e1,
        e2=e2.astype(np.float32) if quadratic else None,
        vlow=vlow,
        vhigh=vhigh,
        loop_counts=np.asarray([item.loop_count for item in base.domains], dtype=np.int32),
        dlow=dlow,
        dhigh=dhigh,
        rows=tuple(rows),
    )


def _reasons(rows) -> dict:
    found: dict = {}
    for row in rows:
        if row[2] != DOMAIN_CERTIFIED:
            found[row[2]] = found.get(row[2], 0) + 1
    return found


# --------------------------------------------------------------------------
# Кадр
# --------------------------------------------------------------------------


def _live_masks(certificate: PreviewCertificateV1, alpha: float):
    live_vertex = (certificate.vlow < alpha) & (alpha < certificate.vhigh)
    live_domain = (certificate.dlow < alpha) & (alpha < certificate.dhigh)
    return live_vertex, live_domain, np.repeat(live_domain, certificate.loop_counts)


def positions_and_uvs(certificate: PreviewCertificateV1, alpha: float):
    """`(позиции (N, 3), UV (L, 2), маска живых доменов)` float64 на ширине `alpha`; на ширине базы — сама база."""

    base = certificate.base
    live_vertex, live_domain, live_loop = _live_masks(certificate, alpha)
    dt = alpha - base.alpha
    if dt == 0.0:
        return base.positions, base.uvs, live_domain
    moved = np.where(live_vertex, dt, 0.0)
    positions = base.positions + certificate.a1 * moved[:, None]
    if certificate.a2 is not None:
        positions = positions + certificate.a2 * (moved * dt)[:, None]
    scale = np.where(live_loop, dt / alpha, 0.0)
    polynomial = certificate.e1 if certificate.e2 is None else certificate.e1 + certificate.e2 * dt
    uvs = base.uvs + polynomial * scale[:, None]
    return positions, uvs, live_domain


def evaluate(certificate: PreviewCertificateV1 | None, alpha: float) -> PreviewFrameV1:
    """Кадр превью на ширине `alpha`: плоские float32 для `foreach_set`, живые и придержанные домены, отказ именем."""

    import time

    started = time.perf_counter()
    if certificate is None:
        return PreviewFrameV1(float(alpha), None, None, 0, 0, (), REFUSED_NO_CERTIFICATE)
    base = certificate.base
    alpha = float(alpha)
    if not (math.isfinite(alpha) and alpha > 0.0):
        return PreviewFrameV1(alpha, _flat32(base.positions), _flat32(base.uvs), 0, len(certificate.rows), (), REFUSED_ALPHA)
    positions, uvs, live = positions_and_uvs(certificate, alpha)
    held = tuple(_held_reason(certificate, number, alpha) for number in np.flatnonzero(~live))
    live_count = int(live.sum())
    refusal = REFUSED_OUTSIDE_EVERY_INTERVAL if live_count == 0 else ""
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


def _held_reason(certificate: PreviewCertificateV1, number: int, alpha: float) -> tuple:
    """`(патч, исход)` придержанного домена: причина из построения, либо за интервалом ядра, либо за досягаемостью модели."""

    row = certificate.rows[number]
    if row[2] != DOMAIN_CERTIFIED:
        return row[0], row[2]
    domain = certificate.base.domains[number]
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


def deviation(certificate: PreviewCertificateV1 | None, sample: PreviewSampleV1) -> DeviationV1:
    """Что сертификат предсказал бы на ширине точного прогона `sample` и насколько это отличается от него, домен за доменом.

    Судятся домены, живые на этой ширине, чей токен и размеры те же. Предел — `DEVIATION_LIMIT_RATIO * alpha`; домены выше предела
    названы в `refuted` (номера в `rows`), и `without_domains` их снимает. Прогон другого ключа не судится (`PREVIEW_KEY_MISMATCH`).
    """

    if certificate is None:
        return DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), REFUSED_NO_CERTIFICATE)
    if sample.key != certificate.key:
        return DeviationV1(sample.alpha, 0.0, 0.0, 0, 0, -1, (), REFUSED_KEY)
    alpha = sample.alpha
    positions, uvs, live = positions_and_uvs(certificate, alpha)
    twins = {(item.patch_id, item.domain_id): item for item in sample.domains}
    limit = DEVIATION_LIMIT_RATIO * alpha
    worst_position = worst_uv = 0.0
    worst_patch = -1
    checked = skipped = 0
    refuted = []
    for number, domain in enumerate(certificate.base.domains):
        if not live[number]:
            continue
        twin = twins.get((domain.patch_id, domain.domain_id))
        if twin is None or twin.token != domain.token or twin.vertices.shape != domain.vertices.shape or twin.loop_count != domain.loop_count:
            skipped += 1
            continue
        checked += 1
        position_error = float(np.max(np.abs(positions[domain.vertices] - sample.positions[twin.vertices]), initial=0.0))
        uv_error = float(
            np.max(
                np.abs(
                    uvs[domain.loop_start : domain.loop_start + domain.loop_count]
                    - sample.uvs[twin.loop_start : twin.loop_start + twin.loop_count]
                ),
                initial=0.0,
            )
        )
        if position_error > worst_position:
            worst_position, worst_patch = position_error, domain.patch_id
        worst_uv = max(worst_uv, uv_error)
        if position_error > limit or uv_error > DEVIATION_LIMIT_RATIO:
            refuted.append(number)
    return DeviationV1(alpha, worst_position, worst_uv, checked, skipped, worst_patch, tuple(refuted))


__all__ = (
    "CHORD_REACH_RATIO",
    "CURVATURE_NOISE_ULPS",
    "CURVED_REACH_RATIO",
    "DEVIATION_LIMIT_RATIO",
    "DOMAIN_CERTIFIED",
    "DOMAIN_HELD_OUTSIDE",
    "DOMAIN_HELD_REACH",
    "DOMAIN_MISSING",
    "DOMAIN_NOT_MATERIALIZED",
    "DOMAIN_NO_INTERVAL",
    "DOMAIN_OUTSIDE",
    "DOMAIN_REFUTED",
    "DOMAIN_STRUCTURE_SWITCH",
    "DeviationV1",
    "DomainSampleV1",
    "PREVIEW_MESH_FROM_INTERVAL_V1",
    "PreviewCertificateV1",
    "PreviewFrameV1",
    "PreviewRefusalV1",
    "PreviewSampleV1",
    "REFUSED_ALPHA",
    "REFUSED_KEY",
    "REFUSED_NO_CERTIFICATE",
    "REFUSED_NO_CERTIFIED_DOMAIN",
    "REFUSED_NO_OTHER_SAMPLE",
    "REFUSED_OUTSIDE_EVERY_INTERVAL",
    "base_frame",
    "build_certificate",
    "deviation",
    "evaluate",
    "positions_and_uvs",
    "sample_of",
)
