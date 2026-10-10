"""Ключ СОДЕРЖИМОГО домена: что именно определяет его подготовку и результат, без ревизии источника.

Кэши сессии держат подготовки и результаты под ревизией источника, а ревизия — хэш ВСЕГО меша:
сдвинутая вершина одного патча меняла её у всех, и все домены становились холодными, хотя
содержимое девяноста девяти из ста не изменилось. Здесь домен получает ключ по тому, от чего он
зависит на самом деле.

ПОЛНОТА — ГЛАВНОЕ СВОЙСТВО. Недостающий вход — это устаревший результат, поэтому ключ строится не
перечнем полей, которые автор вспомнил, а ИЗ ВХОДА ВОРКЕРА: `HostExportInputV1` — всё, что воркер
получает, чтобы выгрузить снапшот и запрос домена (`envelope_export_input`: воркер не видит ни
`AnalysisBundle`, ни соседей, и его снапшот побитово равен снапшоту родителя). Снапшот и запрос —
функция этого входа, подготовка — функция снапшота и запроса, поэтому ключ, кодирующий этот вход
целиком, полон по построению. Кодировщик обходит ВСЕ поля датаклассов, а не названные; поле, которое
ключу не нужно, обязано быть названо в `EXCLUDED_FIELDS` (тест держит равенство перечня полей входа и
этого списка: новое поле входа без решения «входит или нет» — красный тест).

ЧТО НЕ ВХОДИТ И ПОЧЕМУ:

* ревизия источника — это и есть то, что вычитается: строки, несущие её, кодируются с меткой
  вместо неё, `SourceRevision` — меткой целиком;
* НОМЕРА ПАТЧЕЙ: шов, поставленный или снятый на одном конце меша, сдвигает номера всех следующих
  патчей при том же их содержимом (нумерация идёт по порядку обхода граней). Номер патча домена
  кодируется как 0, номера соседей по швам — рангом среди номеров соседей этого домена (1, 2, ...),
  отсутствие соседа (-1) — как есть: ключ не зависит от сдвига, но зависит от того, СКОЛЬКО соседей
  у домена и в каком порядке их номера. Нумерация вершин, рёбер, граней и треугольников источника
  (глобальные индексы меша) в ключе как есть: правка топологии меша, сдвигающая индексы, домены
  пересчитывает;
* `alpha` — подготовка alpha-независима (ядро доказало побитовым совпадением скелета), а alpha
  результата — отдельная часть ключа результата (`result_slot`);
* `request_id` — метка запроса: он уже не входит в ключ подготовки сессии, а результат несёт id
  запроса, при котором посчитан (`envelope_host_labels.REQUEST_SCOPED_KINDS`).

ПОЛОСОВАЯ КАРТА (`envelope_chart_band`). Досягаемость запроса (`chart_reach_cap`) едет в самом входе воркера и
потому кодируется как любое его поле. Саму полосу воркер НЕ строит: после именованного отказа развёртки целого
патча её строит РОДИТЕЛЬ по выбору цепей домена, поэтому выбор — отдельная часть ключа: `band_key`
(`envelope_metric_export.band_key_of`: досягаемость и выбранные рёбра ЭТОГО патча), ровно тот ключ, под которым
сессия кэширует метрику-полосу. Результат, перенесённый на новую ревизию, поэтому не обходит решение о полосе:
другая досягаемость или другой выбор цепей домена — другой ключ, и домен считается заново.

ЧТО ВХОДИТ СВЕРХ ВХОДА ВОРКЕРА: выделенные рёбра домена (они у задачи отдельно), подпись политики запроса
(`envelope_angular_policy` от плотности и допуска растяжения вместе с умолчанием допуска: правка таблицы
веера или умолчания меняет ключ), политики хоста, которые читает выгрузка снапшота (планарность, решётка,
репер, закон подъёма, лестница кривизны), схемы контрактов ядра и ОТПЕЧАТОК КОДА.

ОТПЕЧАТОК КОДА (`code_identity`) — не версия: `cftuv_envelope.__version__` не менялась за десяток слияний ядра.
Это sha256 по содержимому всех .py пакета ядра и пакета хоста тем же законом, что у установщика
(`tools/blender_check_install.py --fingerprint`; тест держит равенство), посчитанный один раз за процесс.
Хранилище по содержимому ПРОЦЕССНО-ЛОКАЛЬНО: оно никогда не пишется на диск (`ContentStoreV1` не
сериализуется, тест архитектуры это держит), поэтому запись не переживает процесс, а код процесса не меняется
под ней; отпечаток в ключе — второй замок на случай, если ключ когда-нибудь окажется вне процесса.

БЭКЕНД ЯДРА (`execution_identity`). Нативный бэкенд (`cftuv_envelope.backend`) отвечает побитово так же, как Python-эталон, и
политикой запроса не является, но входит в идентичность исполнения: ключ несёт `NATIVE:<версия колеса>` либо `PYTHON` рядом с
отпечатком кода, чтобы результат одного бэкенда не подменял результат другого и сверка двух бэкендов не читала память друг друга.
`code_identity` сам остаётся отпечатком файлов (тест держит его равным отпечатку установщика).

Тип, которого кодировщик не знает, — `ContentKeyUnsupported`: домен тогда просто не адресуется по
содержимому и считается как раньше, а ключ с молча пропущенным значением не получается никогда.

Модуль не знает ни Blender, ни `mathutils`: вход воркера — числа и кортежи.
"""

from __future__ import annotations

import dataclasses
import functools
import hashlib
import os
from enum import Enum
from fractions import Fraction

from .envelope_kernel_backend import DEFAULT_KERNEL_BACKEND

CONTENT_KEY_SCHEMA = "cftuv.content-key.v1"
#: Поля `HostExportInputV1`, которые ключ содержимого НЕ кодирует (см. модуль). Остальные входят.
EXCLUDED_FIELDS = frozenset({"source_revision_value", "alpha", "request_id"})

_REVISION_MARK = "\x00R"
_FIELDS_OF: dict[type, tuple[str, ...]] = {}
#: Поля входа, в которых целое — номер патча (кодируется рангом, см. модуль), и поле со словарём патчей.
_PATCH_FIELDS = frozenset({"patch_id", "neighbor_patch_id"})
_PATCH_MAP_FIELD = "nodes"
_SOURCE_REVISION = "SourceRevision"
_REVISION_RECORD = "\x00SRC;"
_PLAIN, _PATCH, _PATCH_MAP = 0, 1, 2


class ContentKeyUnsupported(TypeError):
    """Вход содержит значение, которое кодировщик ключа не умеет кодировать ТОЧНО."""


def _fields(cls: type) -> tuple[str, ...]:
    names = _FIELDS_OF.get(cls)
    if names is None:
        names = tuple(item.name for item in dataclasses.fields(cls))
        _FIELDS_OF[cls] = names
    return names


class _Encoder:
    """Детерминированная запись значения: тип и значение каждого узла, множества — по порядку записи.

    Запись побитово та же, что у прямого рекурсивного обхода с цепочкой проверок типа (его хранит тест `tests/content_key_legacy.py` как
    оракул): запись выбирается по ТОЧНОМУ типу значения (`type(value)`) таблицей `_HANDLERS`, однородные кортежи целых и вещественных пишутся
    одним `join` без вызова на элемент, а запись датакласса - обработчиком, собранным на класс из его полей (по ИМЕНИ поля: номер патча либо словарь
    патчей, остальное как есть). Тип вне таблицы идёт прежней цепочкой (`_other`): перечисление, датакласс (его обработчик ложится в таблицу), отказ.
    """

    __slots__ = ("revision", "patches", "memo")

    def __init__(self, revision: str, patches: dict[int, int], memo: dict | None = None) -> None:
        self.revision = revision
        self.patches = patches
        #: Память текстов общих записей одного прогона (`_shared_handler`); `None` - без памяти.
        self.memo = memo

    def patch(self, number) -> str:
        """Номер патча как ранг: свой — 0, соседи — по порядку номеров, отрицательный — как есть."""

        if type(number) is not int:
            raise ContentKeyUnsupported(f"a patch number must be an int, got {type(number).__name__}")
        if number < 0:
            return f"p{number};"
        rank = self.patches.get(number)
        if rank is None:
            raise ContentKeyUnsupported(f"patch {number} is not the domain or one of its neighbours")
        return f"p{rank};"

    def encode(self, value) -> str:
        handler = _HANDLERS.get(type(value))
        if handler is not None:
            return handler(self, value)
        return self._other(value)

    def _other(self, value) -> str:
        """Тип вне таблицы: перечисление, датакласс (обработчик класса ложится в таблицу) либо отказ - в том же порядке проверок, что у прежнего обхода."""

        kind = type(value)
        if isinstance(value, Enum):
            _HANDLERS[kind] = _enum_text
            return _enum_text(self, value)
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            handler = _HANDLERS[kind] = _record_handler(kind)
            return handler(self, value)
        raise ContentKeyUnsupported(f"{kind.__qualname__} is not encodable in a content key")

    def _string(self, value: str) -> str:
        if self.revision in value:
            value = value.replace(self.revision, _REVISION_MARK)
        return f"s{len(value)}:{value};"

    def _sequence(self, value) -> str:
        if not value:
            return "()"
        kinds = set(map(type, value))
        get = _HANDLERS.get
        if len(kinds) == 1:
            kind = kinds.pop()
            if kind is int:
                return "(i" + ";i".join(map(str, value)) + ";)"
            if kind is float:
                return "(f" + ";f".join(map(float.hex, value)) + ";)"
            handler = get(kind)
            if handler is not None:
                return "(" + "".join([handler(self, item) for item in value]) + ")"
        other = self._other
        parts = []
        for item in value:
            handler = get(type(item))
            parts.append(handler(self, item) if handler is not None else other(item))
        return "(" + "".join(parts) + ")"

    def _unordered(self, value) -> str:
        encode = self.encode
        return "{" + "".join(sorted([encode(item) for item in value])) + "}"

    def _mapping(self, value) -> str:
        encode = self.encode
        pairs = sorted([(encode(key), encode(item)) for key, item in value.items()])
        return "<" + "".join([key + item for key, item in pairs]) + ">"


def _enum_text(self, value) -> str:
    return f"e{type(value).__qualname__}.{value.name};"


def _record_handler(kind: type):
    """Запись датакласса, собранная на класс: голова `D<имя>[`, поля в порядке `dataclasses.fields`, режим поля - по его ИМЕНИ."""

    if kind.__qualname__ == _SOURCE_REVISION:
        return lambda self, value: _REVISION_RECORD
    names = _fields(kind)
    head = f"D{kind.__qualname__}["
    modes = tuple(_PATCH if name in _PATCH_FIELDS else _PATCH_MAP if name == _PATCH_MAP_FIELD else _PLAIN for name in names)
    fields = tuple((f"{name}=", name, mode) for name, mode in zip(names, modes))

    def write(self, value) -> str:
        get = _HANDLERS.get
        other = self._other
        parts = [head]
        for prefix, name, mode in fields:
            item = getattr(value, name)
            if mode == _PLAIN:
                handler = get(type(item))
                parts.append(prefix + (handler(self, item) if handler is not None else other(item)))
            elif mode == _PATCH:
                parts.append(prefix + self.patch(item))
            elif type(item) is dict:
                pairs = sorted([(self.patch(key), self.encode(entry)) for key, entry in item.items()])
                parts.append(prefix + "<" + "".join([key + entry for key, entry in pairs]) + ">")
            else:
                parts.append(prefix + self.encode(item))
        parts.append("]")
        return "".join(parts)

    ranked = _shared_kinds().get(kind)
    return write if ranked is None else _shared_handler(write, ranked)


def _shared_kinds() -> dict[type, bool]:
    """Общие неизменяемые записи, которые входят во входы МНОГИХ доменов одного прогона: `{класс: пишется ли номер патча рангом}`.

    Вершина и ребро поверхности - в срезах всех патчей, которые они касаются; грань кольца соседей (`NeighbourFaceV1`) - в срезах всех доменов вокруг
    неё. Записаны числами и кортежами чисел, поэтому текст записи - функция ТОЛЬКО самой записи, ревизии кодировщика и (у грани кольца, единственного
    поля-номера патча `patch_id`) ранга её патча среди номеров этого домена. Тест держит перечень полей этих классов.
    """

    from .envelope_topology_export import NeighbourFaceV1
    from .surface_ir import SourceEdge, SourceVertex

    return {SourceVertex: False, SourceEdge: False, NeighbourFaceV1: True}


def _shared_handler(write, ranked: bool):
    """Запись общей записи с памятью прогона: текст, написанный для одного домена, берёт следующий домен, у которого та же запись.

    Ключ памяти - `(ревизия, id записи, ранг её патча или None)`; значение держит САМУ запись, поэтому `id` не может достаться другому объекту,
    пока память жива (память живёт один прогон). Запись неизменяема, текст - функция записи, ревизии и ранга (см. `_shared_kinds`), поэтому взятый
    из памяти текст побитово равен тому, что написал бы `write`. Номер патча не `int` либо отказ записи (патч не домен и не сосед) - не из памяти:
    `write` бросит тот же отказ, и отказ в память не кладётся.
    """

    def shared(self, value):
        memo = self.memo
        if memo is None:
            return write(self, value)
        rank = None
        if ranked:
            number = value.patch_id
            if type(number) is not int:
                return write(self, value)
            rank = self.patches.get(number) if number >= 0 else None
        key = (self.revision, id(value), rank)
        found = memo.get(key)
        if found is None:
            text = write(self, value)
            memo[key] = (value, text)
            return text
        return found[1]

    return shared


#: Кодировщик по ТОЧНОМУ типу значения: подкласс `int`/`float`/`str`/`tuple` (в том числе `bool`, `IntEnum`, `numpy.float64`, именованный кортеж) таблицы не находит.
#: Классы перечислений и датаклассов ложатся сюда при первой встрече (`_Encoder._other`).
_HANDLERS = {
    int: lambda self, value: f"i{value};",
    float: lambda self, value: f"f{value.hex()};",
    str: _Encoder._string,
    bool: lambda self, value: "T;" if value else "F;",
    type(None): lambda self, value: "N;",
    tuple: _Encoder._sequence,
    list: _Encoder._sequence,
    frozenset: _Encoder._unordered,
    set: _Encoder._unordered,
    dict: _Encoder._mapping,
    Fraction: lambda self, value: f"q{value.numerator}/{value.denominator};",
}


def _patch_ranks(export) -> dict[int, int]:
    """`{номер патча: ранг}`: патч домена — 0, соседи по швам — 1, 2, ... по порядку их номеров."""

    nodes = export.bundle.patch_graph.nodes
    if len(nodes) != 1:
        raise ContentKeyUnsupported(f"a domain input holds {len(nodes)} patches, not one")
    (own,) = nodes
    try:
        found = {int(record.chain.neighbor_patch_id) for record in export.host_chains}
    except AttributeError as exc:
        raise ContentKeyUnsupported(f"a host chain record is not encodable: {exc}") from exc
    neighbours = sorted({number for number in found if number >= 0} - {int(own)})
    ranks = {int(own): 0}
    ranks.update({number: index + 1 for index, number in enumerate(neighbours)})
    return ranks


def package_fingerprint(root: str) -> str:
    """sha256 по содержимому всех .py каталога: ТОТ ЖЕ закон, что у `tools/blender_check_install.py`.

    Путь внутри пакета и содержимое с переводами строк, приведёнными к LF: копия того же кода в CRLF и в LF
    даёт один отпечаток.
    """

    if not root or not os.path.isdir(root):
        return "<нет каталога>"
    digest = hashlib.sha256()
    for current, directories, files in os.walk(root):
        directories[:] = sorted(item for item in directories if item != "__pycache__")
        for name in sorted(files):
            if not name.endswith(".py"):
                continue
            path = os.path.join(current, name)
            digest.update(os.path.relpath(path, root).replace("\\", "/").encode())
            with open(path, "rb") as handle:
                digest.update(handle.read().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


@functools.lru_cache(maxsize=None)
def _fingerprint_once(root: str) -> str:
    return package_fingerprint(root)


def code_identity() -> tuple[str, str]:
    """`(отпечаток ядра, отпечаток хоста)` кода ЭТОГО процесса: считается один раз (около 40 мс)."""

    try:
        import cftuv_envelope as kernel
    except ImportError as exc:
        raise ContentKeyUnsupported(f"the kernel is not importable: {exc}") from exc
    return (
        _fingerprint_once(os.path.dirname(kernel.__file__)),
        _fingerprint_once(os.path.dirname(os.path.abspath(__file__))),
    )


def execution_identity(backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None) -> tuple[str, str, str]:
    """`(отпечаток ядра, отпечаток хоста, идентичность бэкенда)`: отпечаток кода процесса и бэкенды всех стадий, которыми считают (`None` у стадии - как главный переключатель `backend`)."""

    from .envelope_kernel_backend import backend_identity_of

    return (*code_identity(), backend_identity_of(backend, skeleton_backend, embedding_backend))


def _policy_constants(backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None) -> tuple:
    """Политики хоста, которые читает выгрузка снапшота, схемы контрактов ядра, отпечаток кода и бэкенд ядра."""

    from . import envelope_request_export as export

    try:
        import cftuv_envelope as kernel
        from cftuv_envelope.contracts.geometry_batch import GEOMETRY_BATCH_SCHEMA_V1
    except ImportError as exc:
        raise ContentKeyUnsupported(f"the kernel is not importable: {exc}") from exc
    return (
        export.HOST_PLANARITY_POLICY.value,
        export.HOST_GRID_POLICY.value,
        export.HOST_NEAR_PLANAR_FRAME_POLICY.value,
        export.HOST_NEAR_PLANAR_LIFT_POLICY.value,
        export.HOST_CURVATURE_LADDER_POLICY.value,
        execution_identity(backend, skeleton_backend, embedding_backend),
        kernel.ANALYSIS_SNAPSHOT_SCHEMA_V1,
        GEOMETRY_BATCH_SCHEMA_V1,
    )


def _request_policy_signature(density, budget) -> tuple:
    """Подпись политики запроса: ровно то, что `envelope_angular_policy` строит из плотности и допуска.

    Плотность в ключе — число 0..4, а что за веер оно значит, решает таблица политики; подпись берёт её
    значение, поэтому правка таблицы или умолчания допуска растяжения (в том числе подменой константы
    модуля) даёт другой ключ, а не устаревший результат.
    """

    from . import envelope_request_policy as policy

    try:
        import cftuv_envelope as kernel

        angular = policy.envelope_angular_policy(kernel, density, budget)
    except (ImportError, TypeError, ValueError) as exc:
        raise ContentKeyUnsupported(f"the request policy is not signable: {exc}") from exc
    default = policy.DEFAULT_ENVELOPE_STRETCH_BUDGET
    spent = angular.developable_stretch_budget
    return (
        angular.signature,
        angular.density,
        None if spent is None else (spent.numerator, spent.denominator),
        (default.numerator, default.denominator),
    )


def _normalized_budget(budget):
    from . import envelope_request_policy as policy

    return None if budget == policy.DEFAULT_ENVELOPE_STRETCH_BUDGET else budget


def domain_content_key(export, selected_edge_ids, band_key=None, backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=None, embedding_backend=None, memo=None) -> str:
    """Ключ содержимого домена: sha256 от входа воркера без ревизии, выделения домена и политик.

    `export` — `HostExportInputV1` ЭТОГО домена, `selected_edge_ids` — выделенные рёбра домена, `band_key` —
    ключ полосы домена (`band_key_of`: досягаемость и выбранные рёбра патча, `None` — политики полосы нет).
    Ключ не зависит от ревизии источника, `alpha` и id запроса; от всего остального — зависит. `backend` — имя бэкенда ядра, которым
    считают (`PYTHON` либо `NATIVE`: главный переключатель), `skeleton_backend` и `embedding_backend` — постадийный порядок (`None` - как `backend`; подготовка, лежащая в хранилище под ключом,
    построена ими): идентичность стадий (`execution_identity`) входит в ключ, и подготовка Python не читается как подготовка Rust.

    `memo` - словарь ОДНОГО прогона для текстов общих записей (`_shared_kinds`: вершины, рёбра и грани кольца соседей входят во входы многих
    доменов); ключ от него не зависит ни в одном байте (тест сверяет его с кодировщиком без памяти и с прежним), `None` - без памяти.
    """

    from .envelope_request_policy import normalize_envelope_fan_density

    encoder = _Encoder(export.source_revision_value, _patch_ranks(export), memo)
    parts = [CONTENT_KEY_SCHEMA, encoder.encode(_policy_constants(backend, skeleton_backend, embedding_backend))]
    for name in _fields(type(export)):
        if name in EXCLUDED_FIELDS:
            continue
        value = getattr(export, name)
        if name == "grid_scale_law" and value is None:
            continue  # умолчание ядра: ключ прежних прогонов побитово тот же; закон повторной попытки делает ключ другим
        if name == "density":
            try:
                value = normalize_envelope_fan_density(value)
            except (TypeError, ValueError) as exc:
                raise ContentKeyUnsupported(f"density is not canonical: {exc}") from exc
        elif name == "developable_stretch_budget":
            value = _normalized_budget(value)
        parts.append(f"{name}={encoder.encode(value)}")
    parts.append(
        "policy="
        + encoder.encode(_request_policy_signature(export.density, export.developable_stretch_budget))
    )
    parts.append("selected=" + encoder.encode(tuple(sorted(int(item) for item in selected_edge_ids))))
    parts.append("band=" + encoder.encode(band_key))
    return hashlib.sha256("\x1e".join(parts).encode("utf-8")).hexdigest()


def result_slot(alpha_text: str, uv_policy_id: str, topology_law: str, lift_law: str, backend_id: str | None = None) -> tuple:
    """Часть ключа РЕЗУЛЬТАТА поверх ключа содержимого: alpha, законы материализации и идентичность бэкенда ядра (без неё — идентичность умолчания продукта)."""

    if backend_id is None:
        from .envelope_kernel_backend import backend_identity_of

        backend_id = backend_identity_of(DEFAULT_KERNEL_BACKEND)
    return (str(alpha_text), str(uv_policy_id), str(topology_law), str(lift_law), str(backend_id))


__all__ = (
    "CONTENT_KEY_SCHEMA",
    "ContentKeyUnsupported",
    "EXCLUDED_FIELDS",
    "code_identity",
    "domain_content_key",
    "execution_identity",
    "package_fingerprint",
    "result_slot",
)
