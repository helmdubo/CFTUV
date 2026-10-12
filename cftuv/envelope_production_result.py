"""Результат продуктового пути по ОДНОМУ домену: тип, отложенная форма, законы продукта и имена исходов, размещений и счётчиков.

Вынесено из `envelope_production_export.py` ПЕРЕНОСОМ без изменения поведения (PRODUCTION_EXPORT_SPLIT_V1). Тип результата, его
отложенная форма и имена нужны воркеру (`envelope_production_task`), родителю (`envelope_production_scan`,
`envelope_production_export`) и писателю меша, и ни от чьей логики не зависят. Все имена по-прежнему доступны из
`envelope_production_export` (переэкспорт): вызывающие и тесты ничего не заметили.
"""

from __future__ import annotations

import pickle
import traceback
import zlib
from dataclasses import dataclass, field, fields, replace

from .envelope_request_policy import ENVELOPE_UV_POLICY_DIRECT_STRIP
from .surface_ir import HOST_DECAL_TOPOLOGY_POLICY, HostDecalTopologyPolicy

#: Закон UV продукта. Реестр законов — `envelope_request_policy`.
PRODUCTION_UV_POLICY = ENVELOPE_UV_POLICY_DIRECT_STRIP
#: Закон топологии продукта и допустимые имена (политика хоста, не выбор ядра).
PRODUCTION_TOPOLOGY_LAW = HOST_DECAL_TOPOLOGY_POLICY.value
PRODUCTION_TOPOLOGY_LAWS = frozenset(item.value for item in HostDecalTopologyPolicy)

MATERIALIZED = "MATERIALIZED"
OUTCOME_PREPARATION_UNAVAILABLE = "PREPARATION_UNAVAILABLE"
OUTCOME_PRODUCTION_CANCELLED = "PRODUCTION_CANCELLED"
OUTCOME_DOMAIN_RAISED = "PRODUCTION_DOMAIN_RAISED"
#: Зеркало исхода ядра `EXACT_SCALAR_TEXT_CANON_UNSUPPORTED`; исключение, которое ядро не успело назвать само (подготовка), хост называет тем же словом.
OUTCOME_CANON_UNSUPPORTED = "EXACT_SCALAR_TEXT_CANON_UNSUPPORTED"

#: Стадия профиля продукта и числа, которые он называет.
PRODUCTION_BUILD_KIND = "PRODUCTION"
#: 1, если хоть один домен был холодным (без подготовки в кэше сессии): считались подготовки.
PRODUCTION_COLD_FILL = "PRODUCTION_COLD_FILL"
PRODUCTION_PREPARATION_REUSED = "PRODUCTION_PREPARATION_REUSED"
PRODUCTION_PREPARATION_BUILDS = "PRODUCTION_PREPARATION_BUILDS"
PRODUCTION_PATCH_METRIC_BUILDS = "PRODUCTION_PATCH_METRIC_BUILDS"
PRODUCTION_DOMAIN_GEOMETRY_BUILDS = "PRODUCTION_DOMAIN_GEOMETRY_BUILDS"
PRODUCTION_DOMAINS = "PRODUCTION_DOMAINS"
PRODUCTION_MATERIALIZED = "PRODUCTION_MATERIALIZED"
PRODUCTION_REFUSED = "PRODUCTION_REFUSED"
#: Домены, чей результат лежал в кэше сессии, и домены, которые пришлось считать.
PRODUCTION_RESULT_CACHE_HIT = "PRODUCTION_RESULT_CACHE_HIT"
PRODUCTION_RESULT_CACHE_MISS = "PRODUCTION_RESULT_CACHE_MISS"
#: Из посчитанных в этом прогоне доменов: чья резка взята из памяти стадии (`cftuv_envelope.materialize.clip_memo`, у
#: воркера своя) и чья посчитана и записана в неё. Остальные — без резки либо память выключена.
PRODUCTION_CLIP_MEMO_HITS = "PRODUCTION_CLIP_MEMO_HITS"
PRODUCTION_CLIP_MEMO_MISSES = "PRODUCTION_CLIP_MEMO_MISSES"
#: Хранилище по содержимому домена (`envelope_content_store`): домены с ключом содержимого, домены, чей
#: результат либо подготовка взяты оттуда при другой ревизии, результаты, перенесённые на ревизию прогона,
#: и переносы, которых не вышло (домен тогда считается заново, причина названа строкой консоли).
PRODUCTION_CONTENT_KEYED = "PRODUCTION_CONTENT_KEYED"
#: Из ключённых: сколько доменов этого прогона легло в хранилище. Меньше `KEYED` — домен, чей снапшот строили не
#: под записью токенов (он взят из кэша ревизии), и результат с исключением: переносить их потом нечем.
PRODUCTION_CONTENT_REGISTERED = "PRODUCTION_CONTENT_REGISTERED"
PRODUCTION_CONTENT_RESULT_REUSED = "PRODUCTION_CONTENT_RESULT_REUSED"
PRODUCTION_CONTENT_PREPARATION_REUSED = "PRODUCTION_CONTENT_PREPARATION_REUSED"
PRODUCTION_CONTENT_RELABELED = "PRODUCTION_CONTENT_RELABELED"
PRODUCTION_CONTENT_RELABEL_FAILED = "PRODUCTION_CONTENT_RELABEL_FAILED"
PRODUCTION_CONTENT_UNKEYED = "PRODUCTION_CONTENT_UNKEYED"
#: Пролог прогона (`StageInputsMemoV1`) взят из памяти (1) либо посчитан (0); сколько доменов взяли вход из записей сборки
#: (`envelope_scan_memo`, ширина подставлена в запрос) вместо сборки заново. Это память, а не ответ: тот же ответ без неё.
PRODUCTION_STAGE_INPUTS_MEMO_HIT = "PRODUCTION_STAGE_INPUTS_MEMO_HIT"
PRODUCTION_SCAN_RECORDS_REUSED = "PRODUCTION_SCAN_RECORDS_REUSED"
#: Записанные факты шага ширины (`ProductionDomainResultV1.alpha_interval`): сколько материализованных доменов получили заверенный
#: интервал, сколько стоят на самом событии (окрестности нет) и у скольких событий посчитать нельзя (причина - в записи домена).
PRODUCTION_ALPHA_INTERVALS_CERTIFIED = "PRODUCTION_ALPHA_INTERVALS_CERTIFIED"
PRODUCTION_ALPHA_INTERVALS_AT_EVENT = "PRODUCTION_ALPHA_INTERVALS_AT_EVENT"
PRODUCTION_ALPHA_INTERVALS_NOT_CERTIFIED = "PRODUCTION_ALPHA_INTERVALS_NOT_CERTIFIED"
#: Шаг ширины домена (`cftuv_envelope.materialize.step`): домены прогона, покрытие которых воспроизведено из шаблона внутри заверенного
#: интервала (`FAST_HITS`), и домены, считанные полным путём под именем причины (`FALLBACK_<ПРИЧИНА>`: `NO_CERTIFICATE`, `OUTSIDE_BELOW`,
#: `OUTSIDE_ABOVE`, `TEMPLATE_UNAVAILABLE`, `DISABLED`, `PARTIAL_TEMPLATE`). `VERIFY_MISMATCH` - расхождения сверки быстрого пути с полным
#: (`CFTUV_INTERVAL_VERIFY=1`): ноль всегда, иначе ответ - полный, а число - тревога. Метки запуска, не ответ.
PRODUCTION_INTERVAL_FAST_HITS = "PRODUCTION_INTERVAL_FAST_HITS"
PRODUCTION_INTERVAL_FALLBACK_PREFIX = "PRODUCTION_INTERVAL_FALLBACK_"
PRODUCTION_INTERVAL_VERIFY_MISMATCH = "PRODUCTION_INTERVAL_VERIFY_MISMATCH"

#: Что стоил пулу прогон по трубам (`PoolStatsV1`): байты туда и обратно, подготовки ключом и пиклом, промахи памяти воркера и
#: разбор ответов в родителе (микросекунды: CPU потоков-читателей и стена с ожиданием GIL). Пишутся, когда пул работал.
PRODUCTION_POOL_BYTES_SENT = "PRODUCTION_POOL_BYTES_SENT"
PRODUCTION_POOL_BYTES_RECEIVED = "PRODUCTION_POOL_BYTES_RECEIVED"
PRODUCTION_POOL_BLOB_HITS = "PRODUCTION_POOL_BLOB_HITS"
PRODUCTION_POOL_BLOBS_SHIPPED = "PRODUCTION_POOL_BLOBS_SHIPPED"
PRODUCTION_POOL_BLOB_BYTES_SHIPPED = "PRODUCTION_POOL_BLOB_BYTES_SHIPPED"
PRODUCTION_POOL_BLOB_MISSES = "PRODUCTION_POOL_BLOB_MISSES"
PRODUCTION_POOL_UNPICKLE_CPU_US = "PRODUCTION_POOL_UNPICKLE_CPU_US"
PRODUCTION_POOL_UNPICKLE_WALL_US = "PRODUCTION_POOL_UNPICKLE_WALL_US"

PLACEMENT_WORKER = "worker"
PLACEMENT_PARENT = "parent"
#: Домен не считался: результат взят из кэша сессии.
PLACEMENT_CACHED = "cache"
#: Домен считал родитель, и причина названа: те же имена, что у отладки.
PLACEMENT_UNAVAILABLE = "parent:ENVELOPE_DOMAIN_POOL_UNAVAILABLE"
PLACEMENT_FALLBACK = "parent:ENVELOPE_DOMAIN_POOL_TASK_FALLBACK"

#: WIDTH_REFRESH_LEAN_V1. Обновление точного пути ширины идёт одним из двух классов, и класс назван (профиль прогона, строка статуса, результат
#: домена): `COMMIT` - полный путь (проверки, дайджесты, отчёты, кэши сессии и хранилище по содержимому, квитанция записи), `WIDTH_REFRESH_LEAN_V1` -
#: обновление БЕЗ работы, нужной только фиксации: воркер не пакует батч (пикл и zlib), родитель не считает дайджест массивов, отчёты о плоскости,
#: шве и обходе рёбер, не кладёт результат ни в какой кэш. Геометрия, исходы доменов и счётчики материализатора у них побитово одни: эти стадии
#: только проверяют и сообщают (доказательство - `tests/test_envelope_refresh_lean.py`). Проверки ядра, влияющие на исход домена (`VALIDATE`,
#: `AUDIT`), и повтор лотереи привязки остаются: они могут отказать домен, то есть меняют ответ.
REFRESH_COMMIT = "COMMIT"
REFRESH_LEAN = "WIDTH_REFRESH_LEAN_V1"
#: WIDTH_REFRESH_INCREMENTAL_V1. Третий класс: lean-обновление, которое считает ТОЛЬКО домены, чей аффинный сертификат ширину не обслуживает (интервал ядра кончился, домен в карантине журнала доверия, сертификата нет),
#: а остальные домены (обслуженные) берёт из прежних результатов, сдвинутых сертификатом на ширину тика (`envelope_width_rebase`). Структура обслуженных доменов (тесселяция, силуэт, митра) не заверена, поэтому полную сверку делает
#: периодический полный lean-такт и подтверждение (политика - `envelope_width_incremental`).
REFRESH_INCREMENTAL = "WIDTH_REFRESH_INCREMENTAL_V1"
#: Размещение обслуженного домена: его не считал никто в этом прогоне, результат прежнего прогона сдвинут сертификатом на ширину тика.
PLACEMENT_REBASED = "certificate"
#: Домены incremental-прогона: обслуженные сертификатом и посчитанные заново (в сумме - все домены прогона).
PRODUCTION_REFRESH_INCREMENTAL_SERVED = "PRODUCTION_REFRESH_INCREMENTAL_SERVED"
PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED = "PRODUCTION_REFRESH_INCREMENTAL_RECOMPUTED"
#: Домены lean-прогона, посчитанные без фиксирующей работы, и домены того же прогона, что пошли полным путём (холодные либо с переносом из хранилища).
PRODUCTION_REFRESH_LEAN_DOMAINS = "PRODUCTION_REFRESH_LEAN_DOMAINS"
PRODUCTION_REFRESH_LEAN_FULL_DOMAINS = "PRODUCTION_REFRESH_LEAN_FULL_DOMAINS"
#: Исход, которым называется неверное обращение с lean-результатом (чтение батча, дайджеста; запись в кэш).
OUTCOME_LEAN_RESULT_MISUSED = "LEAN_RESULT_MISUSED"


class ProductionCancelled(RuntimeError):
    """Прогон остановлен по заказу (`cancel`): результата нет и не будет, кэши сессии целы.

    Это НЕ отказ домена: ни один домен не назван отказавшим и ничего не посчитано родителем
    взамен недоделанного пулом. Бросается только когда вызывающий передал `cancel`.
    """


class LeanResultError(RuntimeError):
    """Lean-результат (`WIDTH_REFRESH_LEAN_V1`) просят о том, чего у него нет по построению (батч, нормали вершин, дайджест), либо кладут туда, где живут
    только зафиксированные результаты (кэш результатов сессии, хранилище по содержимому). Громко, а не `None`: молчаливый `None` читался бы как отказ домена.
    """

    def __init__(self, what: str):
        super().__init__(f"{OUTCOME_LEAN_RESULT_MISUSED}: {what}")


@dataclass(frozen=True, slots=True)
class ProductionInputV1:
    """Вход задачи пула: пикл готовой подготовки, закон UV и закон топологии.

    Остальное воркер берёт из самой задачи (`DomainTaskV1.alpha_text`,
    `patch_id`, `domain_id`). Снапшот и запрос не пересылаются: запрос подготовки
    лежит в ней самой, а законы — по одному параметру. Закон топологии — поле со
    значением по умолчанию, поэтому задача без него (прежняя форма) читается.
    """

    blob: bytes | None
    uv_policy_id: str = PRODUCTION_UV_POLICY
    topology_law: str = PRODUCTION_TOPOLOGY_LAW
    #: Перенос результата на ревизию и запрос прогона (`RelabelV1`) делает воркер, параллельно; `None` — без
    #: переноса (подготовка этой ревизии). `carried` — `blob` это не подготовка, а готовый результат из
    #: хранилища по содержимому: воркер только переносит его.
    relabel: object | None = None
    carried: bool = False
    #: Ключ пикла в памяти подготовок воркера (`envelope_worker_store.blob_key`); пусто - подготовка не запоминается. Пул шлёт
    #: задачу без `blob` (одним ключом) воркеру, который подготовку держит, и с `blob` - остальным.
    key: str = ""
    #: `WIDTH_REFRESH_LEAN_V1`: воркер не пакует батч, а присылает вид домена (`lean_result`). Только для домена без переноса (`relabel` пуст, `carried` нет).
    lean: bool = False


@dataclass(frozen=True, slots=True)
class ColdProductionInputV1:
    """Вход задачи «холодный домен»: два закона; сами входы идут в самой задаче.

    Снапшот и запрос домена воркер берёт у задачи (`DomainTaskV1.snapshot`, `request`) либо
    выгружает сам (`DomainTaskV1.export`) — как у задачи отладочного вычислителя.
    """

    uv_policy_id: str = PRODUCTION_UV_POLICY
    topology_law: str = PRODUCTION_TOPOLOGY_LAW
    snapshot_byte_check: object | None = None  # `SnapshotByteCheckPlanV1` нажатия (`None` - проверка байтов на каждом домене); воркер кладёт его в экспорт топологии задачи


#: Поля результата, которых у ОТЛОЖЕННОГО результата нет в `__dict__` до первого чтения: они лежат в `heavy` (`batch` и нормали
#: вершин) либо считаются из батча (`content_digest`), и `__getattr__` отдаёт их по требованию. У них нет значения по
#: умолчанию на уровне класса (`default_factory`), иначе чтение не дошло бы до `__getattr__`.
_LAZY_FIELDS = frozenset({"batch", "vertex_normals", "content_digest"})
#: Поля, от которых зависит вид домена (`view`): их смена делает присланный вид чужим.
_VIEW_INPUT_FIELDS = frozenset({"patch_id", "normal", "source_normal"})
PICKLE_PROTOCOL = 5
#: Пикл батча сжимается zlib (уровень 1): он повторяет одни и те же строки ключей, и на `building` 2.35 МБ идут как 0.53 МБ за 17 мс на все
#: домены (разжатие - 5 мс). Это байты трубы и память кэшей результатов, а не секунды критического пути.
HEAVY_COMPRESSION_LEVEL = 1


@dataclass(frozen=True)
class ProductionDomainResultV1:
    """Исход продуктового пути по ОДНОМУ домену: батч либо названный отказ.

    `outcome` — `MATERIALIZED` либо имя отказа (исход ядра, исход хоста либо
    одно из двух имён этого модуля). `normal` — единичная нормаль плоскости
    домена, лицевая сторона сетки (направление смещения над поверхностью —
    политика писателя меша). `counters` — числа материализатора, всё это ответ.

    ОТЛОЖЕННЫЙ РЕЗУЛЬТАТ (`deferred_result`, так отвечает воркер пула). Родитель читает из батча домена немногое (`view`: вершины,
    грани, UV, владельцы, швы, цепи шва - плоские кортежи, их разбор стоит копейки), а разворот графа записей батча - около
    0.4 с под GIL на `building` на каждом шаге ширины. Поэтому воркер присылает вид и всё тяжёлое одним куском байтов
    (`heavy`: батч и нормали вершин, пикл под zlib), а `batch`, `vertex_normals` и `content_digest` читаются как прежде, но разворачиваются и
    считаются при ПЕРВОМ чтении. Что прочитано, то равно eager-результату: батч запечатывается настоящим семантическим
    дайджестом, `content_digest` — sha256 канонических байтов того же батча (оба - чистые функции батча). Тихо не пропадает
    ничего: полей нет в `__dict__`, но чтение их достаёт, пикл несёт `heavy`, сравнение сравнивает прочитанное.
    `dataclasses.replace` на отложенном результате честно разворачивает его (и сбрасывает `view` и `heavy`: у нового результата
    они не наследуются); сохраняет отложенность только `with_changes`.

    Не входят в сравнение: секунды и `placement` — где посчитан домен. Размещение
    — свойство запуска, а не ответа (так же, как счётчики пула отладки).
    """

    patch_id: int
    domain_id: str
    outcome: str
    batch: object | None
    detail: str = ""
    counters: tuple[tuple[str, int], ...] = ()
    normal: tuple[float, float, float] | None = None
    content_digest: str = field(default_factory=str)
    diagnostics: tuple[str, ...] = ()
    #: Нормаль первой исходной грани владельца (то, что видит источник) —
    #: свидетельство для именованной проверки хоста `normal . source_normal > 0`.
    source_normal: tuple[float, float, float] | None = None
    #: Ориентация карты домена (`AffineChartOrientationV1.value`): от неё зависят
    #: обход треугольников и знак нормали, поэтому она идёт в сводку зонда.
    chart_orientation: str = ""
    #: Нормаль смещения КАЖДОЙ вершины батча (`vert_key`, единичная) у домена-развёртки
    #: и закон, который её дал (`SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1`): у развёртки нет
    #: плоскости источника, смещение декали идёт по нормали вершины, а `normal` выше —
    #: лишь сводка (нормированное среднее). У плоского и near-planar домена пусто.
    vertex_normals: tuple = field(default_factory=tuple)
    offset_normal_law: str = ""
    #: Побитовый sha256 этих нормалей (`offset_normals_digest` ядра): они сдвигают вершины
    #: меша, но в дайджест батча не входят, и только этот дайджест виден воротам и свипу.
    offset_normals_digest: str = ""
    #: Закон топологии, по которому собрана сетка (`DecalTopologyLawV1.value`): пусто у отказа.
    decal_topology_law: str = ""
    seconds: float = field(default=0.0, compare=False)
    placement: str = field(default=PLACEMENT_PARENT, compare=False)
    #: Как получена резка домена (`clip_memo.HIT/MISS/OFF/BYPASS`, пусто — резки нет): метка запуска, не ответ.
    clip_memo: str = field(default="", compare=False)
    #: ЗАПИСАННЫЕ факты шага ширины (`MaterializationV1.interval` и `.structure`), не ответ: заверенный интервал ширины, внутри
    #: которого структура покрытия и резки домена та же (`cftuv_envelope.materialize.interval.AlphaIntervalV1`), и подпись структуры
    #: батча по содержанию (`structure.StructureSignatureV1.digest`; ключи вершин несут идентичности ревизии, поэтому перенос на
    #: другую ревизию пересчитывает её). Пусто у отказа.
    alpha_interval: object | None = field(default=None, compare=False)
    structure_digest: str = field(default="", compare=False)
    #: Как получен домен на шаге ширины (`cftuv_envelope.materialize.step.StepV1.path`): `FAST_HIT` (покрытие из шаблона внутри
    #: заверенного интервала), `FALLBACK:<причина>` (полный счёт, причина названа) либо `VERIFY_MISMATCH:<части>`. Метка запуска, не ответ.
    step_path: str = field(default="", compare=False)
    #: Аффинный сертификат перетаскивания ширины (`cftuv_envelope.materialize.parametric.ParametricCertificateV1`) из ЭТОГО точного прогона: наклон по ширине позиции каждой вершины вида домена и
    #: петель UV, либо названные причины (`PARAM_*`), по которым домен моделью не обслуживается. Записанный факт, не ответ; `None` - сертификата нет (шаг ширины выключен, результат перенесён
    #: на другую ревизию: порядок ключей вершин там другой, и наклоны к нему не приложить).
    parametric: object | None = field(default=None, compare=False, repr=False)
    #: Запись идентичностей хоста, при которых посчитан результат (`DomainLabelingV1`), либо `None`: по ней
    #: результат переносится на другую ревизию источника (`envelope_content_store.relabel_result`). Это
    #: происхождение идентичностей, а не ответ, поэтому в сравнение не входит.
    labels: object | None = field(default=None, compare=False, repr=False)
    #: Что нативный бэкенд сделал с доменом (`BackendRecordV1`: кто посчитал, названный откат на Python) либо `None` (`PYTHON`).
    backend_record: object | None = field(default=None, compare=False, repr=False)
    #: `DomainViewV1` (`envelope_production_view`), посчитанный воркером, и пикл `(батч, нормали вершин)` отложенного результата.
    #: Не аргументы конструктора: у результата, собранного заново, их нет (чужой вид к новому батчу не прикладывается).
    view: object | None = field(default=None, init=False, compare=False, repr=False)
    heavy: bytes | None = field(default=None, init=False, compare=False, repr=False)
    #: `WIDTH_REFRESH_LEAN_V1` (`lean_result`): вид посчитан, а батча, нормалей вершин и дайджеста содержимого НЕТ ПО ПОСТРОЕНИЮ (чтение - `LeanResultError`).
    #: Такой результат не ложится ни в какой кэш и не переносится на другую ревизию: он про одно обновление ширины, а не про зафиксированный ответ.
    lean: bool = field(default=False, compare=False)

    @property
    def is_materialized(self) -> bool:
        state = self.__dict__
        return self.outcome == MATERIALIZED and (
            state.get("heavy") is not None or state.get("batch") is not None or bool(state.get("lean"))
        )

    def __getattr__(self, name):
        if name in _LAZY_FIELDS and self.__dict__.get("lean"):
            raise LeanResultError(
                f"the result of patch {self.__dict__.get('patch_id')} is a WIDTH_REFRESH_LEAN_V1 result: it carries the view, not the {name}"
            )
        if name in _LAZY_FIELDS and self.__dict__.get("heavy") is not None:
            if name == "content_digest":
                object.__setattr__(self, name, self._content_digest_of(self.batch))
            else:
                self._thaw()
            return self.__dict__[name]
        raise AttributeError(f"{type(self).__name__!r} object has no attribute {name!r}")

    def _thaw(self) -> None:
        """Разворачивает `heavy` в `batch` и `vertex_normals`; батч без дайджеста запечатывается настоящим."""

        batch, vertex_normals = pickle.loads(zlib.decompress(self.__dict__["heavy"]))
        if batch is not None:
            from cftuv_envelope.canonical import PENDING_SEMANTIC_DIGEST, sealed_geometry_batch

            if batch.semantic_digest.value == PENDING_SEMANTIC_DIGEST:
                batch = sealed_geometry_batch(batch)
        object.__setattr__(self, "vertex_normals", vertex_normals)
        object.__setattr__(self, "batch", batch)

    @staticmethod
    def _content_digest_of(batch) -> str:
        from hashlib import sha256

        from cftuv_envelope.codec import canonical_json_bytes

        return "" if batch is None else sha256(canonical_json_bytes(batch)).hexdigest()

    def __getstate__(self) -> dict:
        state = dict(self.__dict__)
        if state.get("heavy") is not None:
            # Развёрнутое лежит и в `heavy`: дважды его не пикляют (отложенность переживает пересылку).
            state.pop("batch", None)
            state.pop("vertex_normals", None)
        return state

    def materialized(self, with_digest: bool = True) -> "ProductionDomainResultV1":
        """Тот же результат без отложенности: поля развёрнуты, `view` и `heavy` нет (новый батч не наследует чужой вид)."""

        if self.__dict__.get("lean"):
            raise LeanResultError(f"the result of patch {self.__dict__.get('patch_id')} is a WIDTH_REFRESH_LEAN_V1 result: there is no batch to materialize")
        if self.__dict__.get("heavy") is None:
            return self
        state = dict(self.__dict__)
        state["batch"], state["vertex_normals"] = self.batch, self.vertex_normals
        if "content_digest" not in state:
            state["content_digest"] = self._content_digest_of(state["batch"]) if with_digest else ""
        state.pop("heavy")
        state.pop("view", None)
        copy = object.__new__(type(self))
        copy.__dict__.update(state)
        return copy

    def with_changes(self, **changes) -> "ProductionDomainResultV1":
        """`dataclasses.replace`, не разворачивающий отложенный результат (метки запуска, запись идентичностей, секунды).

        Смена поля, от которого зависит вид (`patch_id`, нормали), сбрасывает присланный вид; смена `batch`, нормалей вершин
        и дайджеста разворачивает результат целиком (новый батч - новый результат, без чужого `heavy`).
        """

        names = {item.name for item in fields(self) if item.init}
        unknown = set(changes) - names
        if unknown:
            raise TypeError(f"unexpected fields {sorted(unknown)}")
        if _LAZY_FIELDS & changes.keys():
            return replace(self.materialized(), **changes)
        state = dict(self.__dict__)
        state.update(changes)
        if _VIEW_INPUT_FIELDS & changes.keys():
            state.pop("view", None)
        copy = object.__new__(type(self))
        copy.__dict__.update(state)
        return copy


def deferred_result(result: ProductionDomainResultV1) -> ProductionDomainResultV1:
    """Отложенная копия материализованного результата: вид посчитан, батч и нормали вершин упакованы в `heavy`.

    Отказ (без батча) и уже отложенный результат возвращаются как есть. Батч может быть без дайджеста
    (`materialize_domain(digests=False)`): его запечатывает первое чтение. Известный `content_digest` остаётся в поле.
    """

    if result.__dict__.get("lean") or result.__dict__.get("heavy") is not None or result.batch is None:
        return result
    from .envelope_production_view import build_domain_view

    state = dict(result.__dict__)
    state["view"] = build_domain_view(result)
    packed = pickle.dumps((state.pop("batch"), state.pop("vertex_normals")), protocol=PICKLE_PROTOCOL)
    state["heavy"] = zlib.compress(packed, HEAVY_COMPRESSION_LEVEL)
    if not state.get("content_digest"):
        state.pop("content_digest", None)
    copy = object.__new__(type(result))
    copy.__dict__.update(state)
    return copy


def lean_result(result: ProductionDomainResultV1) -> ProductionDomainResultV1:
    """Lean-копия материализованного результата (`WIDTH_REFRESH_LEAN_V1`): вид посчитан ТЕМ ЖЕ кодом, что у `deferred_result`, а батч и нормали вершин не пакуются.

    Не делается то, что `deferred_result` делает ради фиксации: пикл батча и zlib (на `building` 11 % ЦП воркеров и ~85 % байтов ответа). Всё, что читает писатель меша и
    образец превью, лежит в виде и в полях результата. Отказ, отложенный и уже lean-результат возвращаются как есть.
    """

    if result.__dict__.get("lean") or result.__dict__.get("heavy") is not None or result.batch is None:
        return result
    from .envelope_production_view import build_domain_view

    state = dict(result.__dict__)
    state["view"] = build_domain_view(result)
    for name in _LAZY_FIELDS:
        state.pop(name, None)
    state["lean"] = True
    copy = object.__new__(type(result))
    copy.__dict__.update(state)
    return copy


def rebased_result(result: ProductionDomainResultV1, *, view, certificate) -> ProductionDomainResultV1:
    """Lean-копия материализованного результата, СДВИНУТАЯ на другую ширину (`envelope_width_rebase`): `view` и `certificate` уже на ширине тика, остальное - поля прежнего прогона.

    Это не новый ответ ядра, а его аффинное продолжение внутри заверенного интервала домена: размещение называет это (`PLACEMENT_REBASED`), батча, нормалей вершин и дайджеста у копии нет (как у lean-результата,
    чтение - `LeanResultError`), и она не ложится ни в какой кэш. Счётчики, исход, подпись структуры и интервал - прежние: структура обслуженного домена не пересчитывалась.
    """

    state = dict(result.__dict__)
    for name in _LAZY_FIELDS:
        state.pop(name, None)
    state.pop("heavy", None)
    state.update(view=view, parametric=certificate, lean=True, placement=PLACEMENT_REBASED, seconds=0.0, clip_memo="", step_path="REBASED")
    copy = object.__new__(type(result))
    copy.__dict__.update(state)
    return copy


def _refusal(patch_id, domain_id, outcome, detail, seconds=0.0, placement=PLACEMENT_PARENT):
    return ProductionDomainResultV1(
        patch_id=int(patch_id),
        domain_id=str(domain_id),
        outcome=str(outcome),
        batch=None,
        detail=str(detail),
        seconds=seconds,
        placement=placement,
    )


def _raised(exc, patch_id, domain_id, seconds, placement=PLACEMENT_PARENT):
    """Отказ домена по исключению: «нет канонической строки V2» называется своим словом, прочее - `PRODUCTION_DOMAIN_RAISED` с хвостом трассы."""

    from cftuv_envelope.reference.native_exact import ExactScalarTextCanonUnsupported

    if isinstance(exc, ExactScalarTextCanonUnsupported):
        return _refusal(patch_id, domain_id, OUTCOME_CANON_UNSUPPORTED, str(exc), seconds, placement)
    tail = " | ".join(item.strip() for item in traceback.format_exc().strip().splitlines()[-3:])
    return _refusal(patch_id, domain_id, OUTCOME_DOMAIN_RAISED, tail, seconds, placement)


def _placed(result: ProductionDomainResultV1, placement: str):
    return result.with_changes(placement=placement)
