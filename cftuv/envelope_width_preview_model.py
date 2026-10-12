"""Модель превью меша ширины из аффинного сертификата ядра: настоящий меш декали двигается между точными пересчётами, и каждое такое движение названо.

ЗАЧЕМ. Пока владелец тянет ширину, точный пересчёт стоит секунду и больше, а линии превью (`envelope_width_preview`) — не меш. Модель
позволяет за миллисекунды писать в СУЩЕСТВУЮЩИЙ меш декали позиции и UV на промежуточной ширине, пока точный счёт идёт в фоне, и точный
результат потом заменяет превью. Исход назван `PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1`.

ОТКУДА ФОРМА. Раньше форму зависимости от ширины подбирали по двум-трём точным прогонам (многочлен через образцы, «досягаемость» модели, затравочные прогоны
при входе в инструмент). Теперь ядро выпускает её из ОДНОГО прогона: аффинный сертификат (`cftuv_envelope.materialize.parametric`, `ParametricCertificateV1`) несёт
ТОЧНЫЙ наклон по ширине позиции каждой вершины и произведений `u * alpha`, `v * alpha` каждой петли UV (внутри заверенного интервала с той же структурой они точно аффинны). Модель
не считает ничего и ничего не подбирает: она читает наклоны из образца точного прогона и складывает их с базой. Затравок нет, вход в инструмент не ждёт
второго прогона.

ЧТО ЗАВЕРЕНО, А ЧТО НЕТ (разведено в именах: «сертифицировано» — только первое).
  ЗАВЕРЕНО (ядро): ИНТЕРВАЛ ширины домена (`AlphaIntervalV1`, статус `CERTIFIED`: нет событий покрытия и резки) и АФФИННОСТЬ внутри него при неизменной структуре: значение при
  alpha0 плюс наклон на сдвиг равны точному прогону на сдвинутой ширине ТОЧНО (`SqrtSumV1`, `kernel/tests/parametric_theorem.py`); хост различает только округление binary64
  (позиция и наклон — по одному округлению точного).
  НЕ ЗАВЕРЕНО: решения тесселяции, силуэта, положения вершин `src:` у хоста, диагоналей ячейки и закон митры сварки (`interval.UNCERTIFIED`). Структуру домена на другой ширине
  проверяет СРАВНЕНИЕ апостериори, как раньше (подпись структуры ядра и токен локальных массивов): новый точный прогон судит предсказание модели (`deviation`), опровергнутый домен
  идёт в карантин (`advance_ledger`), а точный результат в любом случае заменяет превью.

ЧТО НЕ ОБСЛУЖИВАЕТСЯ И ПОЧЕМУ (домен остаётся на последней точной геометрии под ИМЕНЕМ; линии превью показывают контур, а пересчёт ведёт точный путь).
  * названные ядром причины сертификата (`PARAM_*`): развёртка (смещение вдоль нормалей вершин не рационально), узел резки не аффинен, интервал не заверен, ... — имя берётся у сертификата;
  * у результата нет сертификата (`PARAM_NO_CERTIFICATE`: результат перенесён на другую ревизию либо шаг ширины выключен);
  * две петли одной сваренной вершины дали разные наклоны (`PARAM_WELD_SLOPE_DISAGREES`): сварка по равным позициям при alpha0 не держится на окне;
  * смещение узла стены зависит от нормалей соседа-развёртки (`PARAM_NON_AFFINE_WELD_NEIGHBOUR_NORMAL`, закон `WALL_MITER_OFFSET_V1`);
  * ширина вышла за интервал домена (`PARAM_INTERVAL_END`, в кадре).

ФОРМУЛЫ (ровно как в коде; `dt = alpha - alpha0`, на `dt = 0` ответ — база БИТОВО):
    позиция = P0 + s * dt                            (s — наклон сертификата, метры на метр; смещение декали над поверхностью — константа у плоского домена)
    uv      = uv0 + (S - uv0) * dt / alpha            (S — наклон произведения `uv * alpha`: оно аффинно, поэтому делитель стоит ВНЕ многочлена)

ГДЕ ПРЕВЬЮ ЖИВО. Вершина двигается, если ВСЕ домены, которым она принадлежит, смоделированы и не в карантине, а `alpha` строго внутри пересечения их интервалов; петля UV — если жив её
домен. Остальные остаются на геометрии базы (последняя точная), и кадр называет их числом и номерами патчей. Если не жив ни один домен, кадр отказан именем
(`PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL` либо `PREVIEW_ALL_DOMAINS_QUARANTINED`) и равен базе.

ЖУРНАЛ ДОВЕРИЯ И ПОВТОРНЫЙ ДОПУСК (`TrustLedgerV1`, `advance_ledger`). Домен, предсказание которого точный прогон опроверг (`deviation.refuted`), входит в КАРАНТИН: во всех моделях того же
ключа образца он придержан под именем `PREVIEW_DOMAIN_QUARANTINED_AFTER_REFUTATION` и не двигается. Повторный допуск — по правилу: (1) каждое следующее точное обновление СЧИТАЕТ
предсказание карантинного домена «в тени» (оно не пишется в меш) на ширине, не равной базе модели и лежащей в НОМИНАЛЬНОЙ области домена (интервал ядра и `READMIT_NOMINAL_REACH_RATIO`
доли ширины базы); (2) предсказание в пределах `READMIT_ERROR_FRACTION` предела ошибки на расстоянии от базы не меньше `READMIT_MIN_DISTANCE_FRACTION` номинальной области — ЧИСТАЯ сверка;
`READMIT_CLEAN_CHECKS` таких сверок возвращают домен с СЖАТОЙ областью `min(номинал * 0.5^k, 0.5 * расстояние опровержения)` (`k` — число опровержений); (3) повторное опровержение
(в том числе в тени) сжимает область ещё вдвое, а третье — снимает модель этого домена до следующего нажатия кнопки (`PREVIEW_DOMAIN_ABANDONED_AFTER_REFUTATIONS`). Журнал живёт, пока
жив ключ образца, и забывается кнопкой «Build Decal Mesh». У аффинного сертификата опровержение редкость (структура сменилась внутри интервала, а токен не поймал); журнал — страховка.

САМОПРОВЕРКА (апостериорная). Каждый новый точный прогон (`deviation`) сверяет то, что модель предсказала бы на его ширине, с самим прогоном, домен за доменом; домен, отклонение которого
выше `DEVIATION_LIMIT_RATIO * alpha`, опровергнут и идёт в журнал доверия. Прогон на ширине базы ничего не проверяет (`PREVIEW_CHECK_SAMPLE_NOT_INDEPENDENT`): он воспроизвёл бы базу.

Модуль чистый: ни `bpy`, ни настроек, ни контроллера; его зовут и поток точного счёта (построить), и главный поток (кадр).
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, replace
from itertools import chain
from fractions import Fraction

import numpy as np

PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1 = "PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1"

#: Статус заверенного интервала у ядра (`AlphaIntervalV1.status`): единственное, что здесь СЕРТИФИЦИРОВАНО вместе с аффинностью.
INTERVAL_CERTIFIED = "CERTIFIED"

#: Отказы целой модели и кадра.
REFUSED_KEY = "PREVIEW_KEY_MISMATCH"
REFUSED_NO_MODELLED_DOMAIN = "PREVIEW_NO_MODELLED_DOMAIN"
REFUSED_NO_MODEL = "PREVIEW_NO_MODEL"
REFUSED_ALPHA = "PREVIEW_ALPHA_NOT_POSITIVE_FINITE"
REFUSED_OUTSIDE_EVERY_INTERVAL = "PREVIEW_ALPHA_OUTSIDE_EVERY_INTERVAL"
REFUSED_ALL_QUARANTINED = "PREVIEW_ALL_DOMAINS_QUARANTINED"
REFUSED_NOT_INDEPENDENT = "PREVIEW_CHECK_SAMPLE_NOT_INDEPENDENT"

#: Исходы домена при построении (`MODELLED` либо причина, по которой домен остаётся на последней точной геометрии). Причины сертификата (`PARAM_*`) приходят от ядра как есть.
DOMAIN_MODELLED = "MODELLED"
DOMAIN_NOT_MATERIALIZED = "PREVIEW_DOMAIN_NOT_MATERIALIZED"
DOMAIN_NO_INTERVAL = "PREVIEW_DOMAIN_INTERVAL_NOT_CERTIFIED"
DOMAIN_NO_CERTIFICATE = "PARAM_NO_CERTIFICATE"
DOMAIN_SHAPE_MISMATCH = "PARAM_CERTIFICATE_SHAPE_MISMATCH"
DOMAIN_WELD_DISAGREES = "PARAM_WELD_SLOPE_DISAGREES"
DOMAIN_WELD_PARTNER = "PARAM_NON_AFFINE_WELD_NEIGHBOUR_NORMAL"
#: Исход домена в кадре: ширина вышла за заверенный интервал домена (концы интервала — события ядра).
DOMAIN_HELD_OUTSIDE = "PARAM_INTERVAL_END"
#: Исходы журнала доверия: карантин после опровержения точным прогоном, возврат со сжатой областью, снятие модели домена.
DOMAIN_QUARANTINED = "PREVIEW_DOMAIN_QUARANTINED_AFTER_REFUTATION"
DOMAIN_READMITTED = "PREVIEW_DOMAIN_READMITTED_WITH_REDUCED_REACH"
DOMAIN_ABANDONED = "PREVIEW_DOMAIN_ABANDONED_AFTER_REFUTATIONS"
#: Имя причины сертификата ядра, при которой смещение по нормалям вершин делает позиции неаффинными (соседу с ней узел стены доверять нельзя).
DEVELOPABLE_MARKER = "PARAM_NON_AFFINE_DEVELOPABLE_OFFSET"

#: Состояния домена в журнале доверия.
TRUST_QUARANTINED = "QUARANTINED"
TRUST_REDUCED = "REDUCED"
TRUST_ABANDONED = "ABANDONED"

#: Номинальная область домена для ПОВТОРНОГО допуска (доля ширины базы): до опровержения домен живёт во всём интервале ядра, а после него возвращается не дальше этой доли (и сжатой журналом).
READMIT_NOMINAL_REACH_RATIO = 0.10
#: Отличие наклонов одной сваренной вершины у двух доменов, выше которого сварка не держится на окне (метры на метр): наклоны одной точки — один и тот же точный наклон, округлённый независимо.
WELD_SLOPE_TOLERANCE = 1e-9

#: Отклонение превью от точного прогона, при котором домен опровергнут: доля ширины (метры у позиций, единицы тайла у UV). Это решение
#: хоста о том, что глазу ещё неважно (полпроцента ширины полосы), а не допуск ядра; число пишется в запись самопроверки.
DEVIATION_LIMIT_RATIO = 0.005

#: Правило повторного допуска (см. «ЖУРНАЛ ДОВЕРИЯ»): во сколько раз сжимается область за каждое опровержение, какую долю расстояния
#: опровержения она не переходит, сколько чистых независимых сверок в тени возвращают домен, какая доля предела ошибки считается чистой,
#: на каком расстоянии от базы (доля номинальной области) сверка чиста и сколько опровержений снимают модель домена.
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
    """Домен точного прогона на меше: что нужно модели (вершины меша, петли, интервал, токены структуры, наклоны сертификата)."""

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
    #: Наклоны аффинного сертификата ядра: позиции `(вершин, 3)` метры на метр и произведения `uv * alpha` `(петель, 2)`; `None` - сертификата нет либо он назвал причину (`markers`).
    slopes: object | None = None
    uv_slopes: object | None = None
    #: Названные причины, по которым домен не обслуживается (`PARAM_*`); пусто - сертификат с числами.
    markers: tuple = ()
    #: Номера доменов (в порядке образца) соседей по стене, чьи нормали входят в смещение узлов этого домена (`MeshArraysV1.domain_partners`).
    partners: tuple = ()


@dataclass(frozen=True, slots=True, eq=False)
class MeshImageV1:
    """Меш точного прогона, кроме позиций и UV, в numpy (int32): всё, что `_fill_mesh` кладёт в меш, и ничего лишнего.

    Грани: начало каждой в плоском списке вершин, сам плоский список (индексы вершин меша по обходу граней), домен и номер владельца каждой грани; швы - пары вершин.
    Из образа и позиций с UV образца меш собирается на месте без питоновских кортежей (`envelope_production_mesh.rebuild_mesh_from_image`).
    """

    face_starts: object
    face_vertices: object
    face_domain: object
    face_owner: object
    seams: object

    @property
    def face_count(self) -> int:
        return int(self.face_starts.shape[0])


def mesh_image_of(arrays) -> MeshImageV1:
    """Образ меша по массивам меша (`build_mesh_arrays`): те же числа, что кладёт в меш `_fill_mesh`, в numpy."""

    faces = arrays.faces
    lengths = np.fromiter(map(len, faces), dtype=np.int32, count=len(faces))
    starts = np.zeros(len(faces), dtype=np.int32)
    if len(faces) > 1:
        np.cumsum(lengths[:-1], out=starts[1:])
    return MeshImageV1(
        face_starts=starts,
        face_vertices=np.fromiter(chain.from_iterable(faces), dtype=np.int32, count=int(lengths.sum())),
        face_domain=np.asarray(arrays.face_domain, dtype=np.int32),
        face_owner=np.asarray(arrays.face_owner, dtype=np.int32),
        seams=np.asarray(arrays.seam_edges, dtype=np.int32).reshape(-1, 2),
    )


def layout_digest_of(image: MeshImageV1, vertices: int, loops: int) -> str:
    """Отпечаток раскладки (без номеров владельцев): число вершин и петель, грани, домен каждой грани, швы."""

    digest = hashlib.blake2b(digest_size=16)
    digest.update(f"{vertices}:{loops}:{image.face_count}".encode("ascii"))
    for array in (image.face_starts, image.face_vertices, image.face_domain, image.seams):
        digest.update(array.tobytes())
    return digest.hexdigest()


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
    #: Отпечаток раскладки меша, которую кладёт `_fill_mesh` по этим массивам (число вершин и петель, грани, домен каждой грани, швы): два образца с равным отпечатком кладут меш
    #: одинаково во всём, кроме чисел позиций и UV и номеров владельцев граней, поэтому точный кадр одного вправе лечь в меш другого одной записью `co`, `uv` и `cftuv_owner`
    #: (WIDTH_REFRESH_LEAN_V1). Номера владельцев (`claim:N`) в отпечаток НЕ входят: ядро нумерует огибающие по отсортированным именам, имена несут ширину, и порядок меняется
    #: от ширины к ширине при той же структуре (на `building` у 480-500 граней из 907 уже при +0.1 %); это целочисленный атрибут граней, а не раскладка.
    layout_digest: str = ""
    #: Меш образца в numpy (`MeshImageV1`): из него меш пересобирается на месте, когда раскладка сменилась (точный lean-кадр, отмена после него).
    image: object = None

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
    """Наклоны позиций и UV по вершинам и петлям базы, окна жизни доменов, исходы доменов и карантин: модель ИЗ АФФИННОГО СЕРТИФИКАТА одного точного прогона."""

    base: PreviewSampleV1
    #: Наклон позиции каждой вершины меша (метры на метр) и `d(uv * alpha) / d alpha - uv0` каждой петли: `uv = uv0 + e1 * dt / alpha`.
    a1: object
    e1: object
    vlow: object
    vhigh: object
    #: Число петель UV у каждого домена базы (петли домена лежат в `uvs` подряд, в порядке доменов): по нему живость домена
    #: раскладывается на петли без индекса на каждую петлю.
    loop_counts: object
    dlow: object
    dhigh: object
    #: `((патч, домен, исход, степень, досягаемость), ...)` по доменам базы: `MODELLED` со степенью 1 (аффинный сертификат) и досягаемостью (`None` — только интервал ядра; после
    #: возврата из карантина — доля ширины базы), `QUARANTINED`/`ABANDONED` с тем же наклоном (по нему идёт сверка в тени), либо причина со степенью 0.
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

    def reason_counts(self) -> dict:
        found: dict = {}
        for row in self.rows:
            if row[2] != DOMAIN_MODELLED:
                found[row[2]] = found.get(row[2], 0) + 1
        return found

    @property
    def own_bytes(self) -> int:
        """Байты самой модели (наклоны, окна, индекс петель, карантин); база считается отдельно: она нужна и без неё."""

        arrays = (
            self.a1, self.e1, self.vlow, self.vhigh, self.loop_counts, self.dlow, self.dhigh,
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
        return self.refusal or PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1


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

    Домены берутся в порядке меша (`arrays.domain_keys`); запись интервала, подпись структуры и аффинный сертификат читаются из результата домена. Наклоны сертификата лежат в порядке вида
    домена (вершины по ключу батча, петли по граням), как и номера вершин меша и петли `uvs`, поэтому выравнивать их не нужно.
    """

    by_key = {(item.patch_id, item.domain_id): item for item in results}
    domains = []
    start = 0
    partners = tuple(getattr(arrays, "domain_partners", ()) or ())
    for number, (patch_id, domain_id) in enumerate(arrays.domain_keys):
        result = by_key[(patch_id, domain_id)]
        interval = getattr(result, "alpha_interval", None)
        count = int(arrays.domain_loop_counts[number])
        vertices = np.asarray(arrays.domain_vertex_index[number], dtype=np.int32)
        slopes, uv_slopes, markers = _certificate_slopes(getattr(result, "parametric", None), int(vertices.shape[0]), count)
        domains.append(
            DomainSampleV1(
                patch_id=int(patch_id),
                domain_id=str(domain_id),
                token=str(arrays.domain_tokens[number]),
                structure=str(getattr(result, "structure_digest", "")),
                status="" if interval is None else str(interval.status),
                low=float("nan") if interval is None else float(interval.low),
                high=None if interval is None or interval.high is None else float(interval.high),
                vertices=vertices,
                loop_start=start,
                loop_count=count,
                interval=interval,
                slopes=slopes,
                uv_slopes=uv_slopes,
                markers=markers,
                partners=tuple(int(item) for item in partners[number]) if number < len(partners) else (),
            )
        )
        start += count
    positions = np.asarray(arrays.positions, dtype=np.float64).reshape(-1, 3)
    uvs = np.asarray(arrays.uvs, dtype=np.float64).reshape(-1, 2)
    image = mesh_image_of(arrays)
    return PreviewSampleV1(
        key=tuple(key),
        alpha_text=str(alpha_text),
        alpha=float(alpha_text),
        positions=positions,
        uvs=uvs,
        domains=tuple(domains),
        arrays_digest=str(arrays.digest),
        source_revision=str(arrays.source_revision),
        layout_digest=layout_digest_of(image, len(arrays.positions), len(arrays.uvs)),
        image=image,
    )


def _certificate_slopes(certificate, vertices: int, loops: int) -> tuple:
    """`(наклоны позиций, наклоны произведений uv * alpha, причины)` сертификата домена; чисел нет, когда сертификата нет, он назвал причину либо не сошёлся по размерам с видом домена."""

    if certificate is None:
        return None, None, (DOMAIN_NO_CERTIFICATE,)
    if certificate.markers:
        return None, None, tuple(certificate.markers)
    if certificate.vertex_count != vertices or certificate.loop_count != loops:
        return None, None, (DOMAIN_SHAPE_MISMATCH,)
    slopes = np.frombuffer(certificate.position_slopes, dtype=np.float64).reshape(-1, 3)
    uv_slopes = np.frombuffer(certificate.loop_slopes, dtype=np.float64).reshape(-1, 2)
    return slopes, uv_slopes, ()


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


def _domain_reason(domain: DomainSampleV1) -> str:
    """`""` — домен обслуживается; иначе имя причины (без сравнения с другими прогонами: наклоны дал сертификат)."""

    if not domain.token:
        return DOMAIN_NOT_MATERIALIZED
    if domain.status != INTERVAL_CERTIFIED or domain.interval is None:
        return DOMAIN_NO_INTERVAL
    if domain.markers:
        return domain.markers[0]
    return ""


def _trusted_reach(reach, entry: DomainTrustV1 | None):
    """Область домена после возврата из карантина: `min(номинал * масштаб, предел опровержения)`; `None` без записи журнала (весь интервал ядра)."""

    if entry is None:
        return reach
    nominal = READMIT_NOMINAL_REACH_RATIO if reach is None else reach
    return min(nominal * entry.scale, entry.reach_cap)


def _weld_conflicts(base: PreviewSampleV1, modelled, slopes_of) -> set:
    """Номера доменов, у которых хоть одна сваренная вершина получила от другого домена иной наклон (больше `WELD_SLOPE_TOLERANCE`)."""

    count = base.vertex_count
    low = np.full((count, 3), np.inf)
    high = np.full((count, 3), -np.inf)
    for number in modelled:
        vertices = base.domains[number].vertices
        np.minimum.at(low, vertices, slopes_of[number])
        np.maximum.at(high, vertices, slopes_of[number])
    broken = (high - low).max(axis=1) > WELD_SLOPE_TOLERANCE
    if not broken.any():
        return set()
    return {number for number in modelled if broken[base.domains[number].vertices].any()}


def _partner_blocked(base: PreviewSampleV1, domain: DomainSampleV1) -> bool:
    """Сосед по стене с нормалями вершин (развёртка) либо без сертификата: смещение узлов этого домена от ширины не аффинно (закон `WALL_MITER_OFFSET_V1`)."""

    for other in domain.partners:
        neighbour = base.domains[other]
        if DEVELOPABLE_MARKER in neighbour.markers or DOMAIN_NO_CERTIFICATE in neighbour.markers:
            return True
    return False


def build_model(base: PreviewSampleV1, ledger: TrustLedgerV1 | None = None) -> PreviewModelV1 | PreviewRefusalV1:
    """Модель базы из аффинных сертификатов её доменов; отказ — именем. Ничего не подбирается и других прогонов не нужно.

    `ledger` — журнал доверия сессии: домены в карантине (или снятые) построены, но придержаны; вернувшиеся двигаются в сжатой области.
    """

    count_v, count_l, count_d = base.vertex_count, base.loop_count, len(base.domains)
    trust_of = ledger.get if ledger is not None and ledger.key == base.key else (lambda _patch, _domain: None)
    reasons = [_domain_reason(domain) or ("" if not _partner_blocked(base, domain) else DOMAIN_WELD_PARTNER) for domain in base.domains]
    live = [number for number, reason in enumerate(reasons) if not reason]
    slopes_of = {number: base.domains[number].slopes for number in live}
    conflicts = _weld_conflicts(base, live, slopes_of) if live else set()
    for number in conflicts:
        reasons[number] = DOMAIN_WELD_DISAGREES
    a1 = np.zeros((count_v, 3))
    e1 = np.zeros((count_l, 2))
    dlow = np.full(count_d, _ALIVE_NEVER_LOW)
    dhigh = np.full(count_d, _ALIVE_NEVER_HIGH)
    quarantined = np.zeros(count_d, dtype=bool)
    shadow_low = np.full(count_d, _ALIVE_NEVER_LOW)
    shadow_high = np.full(count_d, _ALIVE_NEVER_HIGH)
    rows = []
    modelled = []
    alpha0 = base.alpha
    for number, domain in enumerate(base.domains):
        if reasons[number]:
            rows.append((domain.patch_id, domain.domain_id, reasons[number], 0, None))
            continue
        sl = slice(domain.loop_start, domain.loop_start + domain.loop_count)
        a1[domain.vertices] = domain.slopes
        e1[sl] = domain.uv_slopes - base.uvs[sl]
        entry = trust_of(domain.patch_id, domain.domain_id)
        reach = _trusted_reach(None, entry)
        outcome = DOMAIN_MODELLED
        low = domain.low
        high = math.inf if domain.high is None else domain.high
        if entry is not None and entry.state in (TRUST_QUARANTINED, TRUST_ABANDONED):
            outcome = DOMAIN_QUARANTINED if entry.state == TRUST_QUARANTINED else DOMAIN_ABANDONED
            quarantined[number] = True
            shadow_low[number], shadow_high[number] = max(low, alpha0 * (1.0 - READMIT_NOMINAL_REACH_RATIO)), min(high, alpha0 * (1.0 + READMIT_NOMINAL_REACH_RATIO))
        if reach is not None:
            low, high = max(low, alpha0 * (1.0 - reach)), min(high, alpha0 * (1.0 + reach))
        dlow[number], dhigh[number] = low, high
        rows.append((domain.patch_id, domain.domain_id, outcome, 1, reach))
        modelled.append(number)
    if not modelled:
        return PreviewRefusalV1(REFUSED_NO_MODELLED_DOMAIN, ", ".join(f"{name}={value}" for name, value in sorted(_reasons(rows).items())))
    vlow, vhigh, vertex_quarantined, frozen = _vertex_windows(base, modelled, dlow, dhigh, quarantined)
    a1[frozen] = 0.0
    held_back = bool(quarantined.any())
    return PreviewModelV1(
        base=base,
        a1=a1,
        e1=e1,
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


def _live_domains(model: PreviewModelV1, alpha: float):
    live_domain = (model.dlow < alpha) & (alpha < model.dhigh)
    if model.quarantined is not None:
        live_domain &= ~model.quarantined
    return live_domain


def _live_masks(model: PreviewModelV1, alpha: float):
    live_vertex = (model.vlow < alpha) & (alpha < model.vhigh)
    live_domain = _live_domains(model, alpha)
    if model.quarantined is not None:
        live_vertex &= ~model.vertex_quarantined
    return live_vertex, live_domain, np.repeat(live_domain, model.loop_counts)


def moving_patches(model: PreviewModelV1, held) -> frozenset:
    """Номера патчей, которые кадр модели двигает целиком: патчи базы без единого придержанного домена (`held` — пары `(патч, исход)` кадра).

    Остальное — придержанные домены (последняя точная геометрия) и патчи, которых в базе нет (домен отказан до меша): их меш не двигает, и
    линии превью (`PREVIEW_CONTOUR_ONLY_WHERE_MODEL_HELD_V1`) остаются именно там.
    """

    return frozenset(row[0] for row in model.rows) - frozenset(patch for patch, _reason in held)


def moving_patches_at(model: PreviewModelV1 | None, alpha: float) -> frozenset:
    """То же без кадра и без записи в меш: что двигала бы модель на ширине `alpha`. Пусто — модели нет, ширина негодна либо ни один домен не жив."""

    alpha = float(alpha)
    if model is None or not (math.isfinite(alpha) and alpha > 0.0):
        return frozenset()
    live = _live_domains(model, alpha)
    if not live.any():
        return frozenset()
    return moving_patches(model, [(model.rows[number][0], None) for number in np.flatnonzero(~live)])


def shadow_prediction(model: PreviewModelV1, number: int, alpha: float):
    """`(позиции вершин домена, UV его петель)` по наклонам модели на ширине `alpha`, как двигался бы домен, будь он допущен.

    Предсказание «в тени» карантинного домена: в кадр оно не идёт, им судится возврат домена (`deviation`). Формулы те же, что у кадра.
    """

    base = model.base
    domain = base.domains[number]
    dt = alpha - base.alpha
    vertices = domain.vertices
    positions = base.positions[vertices] + model.a1[vertices] * dt
    sl = slice(domain.loop_start, domain.loop_start + domain.loop_count)
    return positions, base.uvs[sl] + model.e1[sl] * (dt / alpha)


def positions_and_uvs(model: PreviewModelV1, alpha: float):
    """`(позиции (N, 3), UV (L, 2), маска живых доменов)` float64 на ширине `alpha`; на ширине базы — сама база."""

    base = model.base
    live_vertex, live_domain, live_loop = _live_masks(model, alpha)
    dt = alpha - base.alpha
    if dt == 0.0:
        return base.positions, base.uvs, live_domain
    moved = np.where(live_vertex, dt, 0.0)
    positions = base.positions + model.a1 * moved[:, None]
    scale = np.where(live_loop, dt / alpha, 0.0)
    uvs = base.uvs + model.e1 * scale[:, None]
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
    """`(патч, исход)` придержанного домена: причина из построения, карантин либо ширина за интервалом ядра (концы интервала — события)."""

    row = model.rows[number]
    if row[2] != DOMAIN_MODELLED:
        return row[0], row[2]
    return row[0], DOMAIN_HELD_OUTSIDE


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
    if alpha == model.base.alpha:
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
    "DEVELOPABLE_MARKER",
    "DEVIATION_LIMIT_RATIO",
    "DOMAIN_ABANDONED",
    "DOMAIN_HELD_OUTSIDE",
    "DOMAIN_MODELLED",
    "DOMAIN_NOT_MATERIALIZED",
    "DOMAIN_NO_CERTIFICATE",
    "DOMAIN_NO_INTERVAL",
    "DOMAIN_QUARANTINED",
    "DOMAIN_READMITTED",
    "DOMAIN_SHAPE_MISMATCH",
    "DOMAIN_WELD_DISAGREES",
    "DOMAIN_WELD_PARTNER",
    "DeviationV1",
    "DomainSampleV1",
    "DomainTrustV1",
    "INTERVAL_CERTIFIED",
    "PREVIEW_MESH_FROM_AFFINE_CERTIFICATE_V1",
    "PreviewFrameV1",
    "PreviewModelV1",
    "PreviewRefusalV1",
    "PreviewSampleV1",
    "READMIT_CLEAN_CHECKS",
    "READMIT_ERROR_FRACTION",
    "READMIT_MIN_DISTANCE_FRACTION",
    "READMIT_NOMINAL_REACH_RATIO",
    "REFUSED_ALL_QUARANTINED",
    "REFUSED_ALPHA",
    "REFUSED_KEY",
    "REFUSED_NOT_INDEPENDENT",
    "REFUSED_NO_MODEL",
    "REFUSED_NO_MODELLED_DOMAIN",
    "REFUSED_OUTSIDE_EVERY_INTERVAL",
    "TRUST_ABANDONED",
    "TRUST_CAP_FRACTION",
    "TRUST_QUARANTINED",
    "TRUST_REDUCED",
    "TRUST_SCALE_PER_REFUTATION",
    "TrustLedgerV1",
    "WELD_SLOPE_TOLERANCE",
    "advance_ledger",
    "base_frame",
    "build_model",
    "deviation",
    "empty_ledger",
    "evaluate",
    "moving_patches",
    "moving_patches_at",
    "positions_and_uvs",
    "sample_of",
    "shadow_prediction",
)
