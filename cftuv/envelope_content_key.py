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

ЧТО ВХОДИТ СВЕРХ ВХОДА ВОРКЕРА: выделенные рёбра домена (они у задачи отдельно), нормализованная
плотность веера и допуск растяжения запроса, политики хоста, которые читает выгрузка снапшота
(планарность, решётка, репер, закон подъёма, лестница кривизны), и версии контрактов ядра.

Тип, которого кодировщик не знает, — `ContentKeyUnsupported`: домен тогда просто не адресуется по
содержимому и считается как раньше, а ключ с молча пропущенным значением не получается никогда.

Модуль не знает ни Blender, ни `mathutils`: вход воркера — числа и кортежи.
"""

from __future__ import annotations

import dataclasses
import hashlib
from enum import Enum
from fractions import Fraction

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


def _policy_constants() -> tuple:
    """Политики хоста, которые читает выгрузка снапшота, и версии контрактов ядра."""

    from . import envelope_request_export as export

    try:
        import cftuv_envelope as kernel
        from cftuv_envelope.contracts.geometry_batch import GEOMETRY_BATCH_SCHEMA_V1
        from cftuv_envelope.version import __version__
    except ImportError as exc:
        raise ContentKeyUnsupported(f"the kernel is not importable: {exc}") from exc
    return (
        export.HOST_PLANARITY_POLICY.value,
        export.HOST_GRID_POLICY.value,
        export.HOST_NEAR_PLANAR_FRAME_POLICY.value,
        export.HOST_NEAR_PLANAR_LIFT_POLICY.value,
        export.HOST_CURVATURE_LADDER_POLICY.value,
        __version__,
        kernel.ANALYSIS_SNAPSHOT_SCHEMA_V1,
        GEOMETRY_BATCH_SCHEMA_V1,
    )


def _normalized_budget(budget):
    from .envelope_request_policy import DEFAULT_ENVELOPE_STRETCH_BUDGET

    return None if budget == DEFAULT_ENVELOPE_STRETCH_BUDGET else budget


def domain_content_key(export, selected_edge_ids) -> str:
    """Ключ содержимого домена: sha256 от входа воркера без ревизии, выделения домена и политик.

    `export` — `HostExportInputV1` ЭТОГО домена, `selected_edge_ids` — выделенные рёбра домена.
    Ключ не зависит от ревизии источника, `alpha` и id запроса; от всего остального — зависит.
    """

    from .envelope_request_policy import normalize_envelope_fan_density

    encoder = _Encoder(export.source_revision_value, _patch_ranks(export))
    parts = [CONTENT_KEY_SCHEMA, encoder.encode(_policy_constants())]
    for name in _fields(type(export)):
        if name in EXCLUDED_FIELDS:
            continue
        value = getattr(export, name)
        if name == "density":
            value = normalize_envelope_fan_density(value)
        elif name == "developable_stretch_budget":
            value = _normalized_budget(value)
        parts.append(f"{name}={encoder.encode(value)}")
    parts.append("selected=" + encoder.encode(tuple(sorted(int(item) for item in selected_edge_ids))))
    return hashlib.sha256("\x1e".join(parts).encode("utf-8")).hexdigest()


def result_slot(alpha_text: str, uv_policy_id: str, topology_law: str, lift_law: str) -> tuple:
    """Часть ключа РЕЗУЛЬТАТА поверх ключа содержимого: alpha и законы материализации."""

    return (str(alpha_text), str(uv_policy_id), str(topology_law), str(lift_law))


__all__ = (
    "CONTENT_KEY_SCHEMA",
    "ContentKeyUnsupported",
    "EXCLUDED_FIELDS",
    "domain_content_key",
    "result_slot",
)
