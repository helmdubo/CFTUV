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

from .envelope_kernel_backend import DEFAULT_KERNEL_BACKEND, DEFAULT_SKELETON_BACKEND

CONTENT_KEY_SCHEMA = "cftuv.content-key.v1"
#: Поля `HostExportInputV1`, которые ключ содержимого НЕ кодирует (см. модуль). Остальные входят.
EXCLUDED_FIELDS = frozenset({"source_revision_value", "alpha", "request_id"})

_REVISION_MARK = "\x00R"
_FIELDS_OF: dict[type, tuple[str, ...]] = {}
#: Поля входа, в которых целое — номер патча (кодируется рангом, см. модуль), и поле со словарём патчей.
_PATCH_FIELDS = frozenset({"patch_id", "neighbor_patch_id"})
_PATCH_MAP_FIELD = "nodes"


class ContentKeyUnsupported(TypeError):
    """Вход содержит значение, которое кодировщик ключа не умеет кодировать ТОЧНО."""


def _fields(cls: type) -> tuple[str, ...]:
    names = _FIELDS_OF.get(cls)
    if names is None:
        names = tuple(item.name for item in dataclasses.fields(cls))
        _FIELDS_OF[cls] = names
    return names


class _Encoder:
    """Детерминированная запись значения: тип и значение каждого узла, множества — по порядку записи."""

    __slots__ = ("revision", "patches")

    def __init__(self, revision: str, patches: dict[int, int]) -> None:
        self.revision = revision
        self.patches = patches

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
        kind = type(value)
        if kind is int:
            return f"i{value};"
        if kind is float:
            return f"f{value.hex()};"
        if kind is str:
            if self.revision in value:
                value = value.replace(self.revision, _REVISION_MARK)
            return f"s{len(value)}:{value};"
        if kind is bool:
            return "T;" if value else "F;"
        if value is None:
            return "N;"
        if kind is tuple or kind is list:
            return "(" + "".join(self.encode(item) for item in value) + ")"
        if kind is frozenset or kind is set:
            return "{" + "".join(sorted(self.encode(item) for item in value)) + "}"
        if kind is dict:
            pairs = sorted((self.encode(key), self.encode(item)) for key, item in value.items())
            return "<" + "".join(key + item for key, item in pairs) + ">"
        if kind is Fraction:
            return f"q{value.numerator}/{value.denominator};"
        if isinstance(value, Enum):
            return f"e{kind.__qualname__}.{value.name};"
        if dataclasses.is_dataclass(value) and not isinstance(value, type):
            return self._record(value, kind)
        raise ContentKeyUnsupported(f"{kind.__qualname__} is not encodable in a content key")

    def _record(self, value, kind: type) -> str:
        if kind.__qualname__ == "SourceRevision":
            return "\x00SRC;"
        return f"D{kind.__qualname__}[{''.join(self._field(value, name) for name in _fields(kind))}]"

    def _field(self, owner, name: str) -> str:
        item = getattr(owner, name)
        if name in _PATCH_FIELDS:
            return f"{name}={self.patch(item)}"
        if name == _PATCH_MAP_FIELD and type(item) is dict:
            pairs = sorted((self.patch(key), self.encode(entry)) for key, entry in item.items())
            return f"{name}=<" + "".join(key + entry for key, entry in pairs) + ">"
        return f"{name}={self.encode(item)}"


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


def execution_identity(backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> tuple[str, str, str]:
    """`(отпечаток ядра, отпечаток хоста, идентичность бэкенда)`: отпечаток кода процесса и бэкенды обеих стадий (покрытие с резкой и скелет), которыми считают."""

    from .envelope_kernel_backend import backend_identity_of

    return (*code_identity(), backend_identity_of(backend, skeleton_backend))


def _policy_constants(backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> tuple:
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
        execution_identity(backend, skeleton_backend),
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


def domain_content_key(export, selected_edge_ids, band_key=None, backend=DEFAULT_KERNEL_BACKEND, skeleton_backend=DEFAULT_SKELETON_BACKEND) -> str:
    """Ключ содержимого домена: sha256 от входа воркера без ревизии, выделения домена и политик.

    `export` — `HostExportInputV1` ЭТОГО домена, `selected_edge_ids` — выделенные рёбра домена, `band_key` —
    ключ полосы домена (`band_key_of`: досягаемость и выбранные рёбра патча, `None` — политики полосы нет).
    Ключ не зависит от ревизии источника, `alpha` и id запроса; от всего остального — зависит. `backend` — имя бэкенда ядра, которым
    считают (`PYTHON` либо `NATIVE`), `skeleton_backend` — то же для стадии скелета (подготовка, лежащая в хранилище под ключом, построена им): их идентичность
    (`execution_identity`) входит в ключ, и подготовка Python не читается как подготовка Rust.
    """

    from .envelope_request_policy import normalize_envelope_fan_density

    encoder = _Encoder(export.source_revision_value, _patch_ranks(export))
    parts = [CONTENT_KEY_SCHEMA, encoder.encode(_policy_constants(backend, skeleton_backend))]
    for name in _fields(type(export)):
        if name in EXCLUDED_FIELDS:
            continue
        value = getattr(export, name)
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
