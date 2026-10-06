"""Подпись СТРУКТУРЫ батча: то, что не меняется, пока ширина движется внутри заверенного интервала (`interval`).

ЗАЧЕМ. Заверенный интервал (`interval.alpha_interval`) утверждает, что структура покрытия и резки внутри него та же. Проверить это
(и то, чего интервал не заверяет: тесселяцию, силуэт, положение вершин, план станций) можно только сравнением самих батчей.
Подпись - краткий способ сравнить два батча одного домена по СОДЕРЖАНИЮ структуры, без чисел.

ЧТО В ПОДПИСИ. Ключи вершин; кольца граней (обход, с наименьшего ключа) вместе с меткой региона; факты станции `(вершина, метка
региона)`; неориентированные рёбра цепей источника/стены и цепей интерфейса (каждая цепь - отсортированный набор пар ключей); метки
регионов; идентификаторы диагностик. Чисел нет: позиции, UV и станции `(s, r)` аффинны по ширине и сравниваются отдельно.

РЕГИОНЫ ПЕРЕНУМЕРОВАНЫ ПО СОДЕРЖАНИЮ. Номера `region:N` и `claim:N` выводятся из отсортированных имён экземпляров огибающей, а те
зависят от эффективной ширины (`strip_envelope_instance_id(spec, alpha)`): тот же регион при другой ширине получает другой номер. Поэтому
метка региона - наименьшее кольцо его граней: оно определяет регион однозначно (грань принадлежит одному региону, кольца граней
домена различны), не зависит от имён и меняется лишь вместе с составом граней.

Подпись - sha256 по каноническому тексту частей; части (`parts`) - те же, по отдельности, чтобы расхождение называло, где оно.
"""

from __future__ import annotations

from dataclasses import dataclass
from hashlib import sha256

STRUCTURE_LAW = "BATCH_STRUCTURE_BY_CONTENT_V1"
_SEPARATOR = "\x1f"
_RECORD = "\x1e"


@dataclass(frozen=True, slots=True)
class StructureSignatureV1:
    """Подпись структуры батча: общий дайджест и дайджесты частей (по названию части)."""

    digest: str
    parts: tuple[tuple[str, str], ...]

    def differing(self, other: "StructureSignatureV1") -> tuple[str, ...]:
        """Названия частей, где подписи расходятся (пусто - структуры равны)."""

        mine, theirs = dict(self.parts), dict(other.parts)
        return tuple(name for name in sorted(set(mine) | set(theirs)) if mine.get(name) != theirs.get(name))


def _ring(keys) -> tuple:
    """Кольцо ключей, начатое с наименьшего: один и тот же контур даёт одну запись при любом начале обхода."""

    keys = tuple(keys)
    start = keys.index(min(keys))
    return keys[start:] + keys[:start]


def _chain_edges(chain) -> tuple:
    keys = [item.value for item in chain.ordered_vert_keys]
    return tuple(sorted((a, b) if a <= b else (b, a) for a, b in zip(keys, keys[1:])))


def _part_digest(records) -> str:
    text = _RECORD.join(records)
    return sha256(text.encode("utf-8")).hexdigest()[:16]


def batch_structure(batch) -> StructureSignatureV1:
    """Подпись структуры `GeometryBatchV1`: чисел и номеров регионов в ней нет."""

    rings = [(_ring(key.value for key in face.ordered_vert_keys), face.semantic_region_id.value) for face in batch.faces]
    label_of: dict[str, tuple] = {}
    for ring, region in rings:
        known = label_of.get(region)
        if known is None or ring < known:
            label_of[region] = ring
    faces = sorted((ring, label_of[region]) for ring, region in rings)
    parts = {
        "vertices": sorted(vertex.vert_key.value for vertex in batch.vertices),
        "faces": [_SEPARATOR.join(ring) + "@" + _SEPARATOR.join(label) for ring, label in faces],
        "facts": sorted(
            fact.vert_key.value + "@" + _SEPARATOR.join(label_of.get(fact.semantic_region_id.value, ("?",)))
            for fact in batch.station_facts
        ),
        "boundary_chains": sorted(
            _RECORD.join(_SEPARATOR.join(edge) for edge in _chain_edges(chain)) for chain in batch.boundary_chains
        ),
        "interface_chains": sorted(
            _RECORD.join(_SEPARATOR.join(edge) for edge in _chain_edges(chain)) for chain in batch.interface_chains
        ),
        "regions": sorted(_SEPARATOR.join(label) for label in set(label_of.values())),
        "diagnostics": sorted(item.diagnostic_id.value for item in batch.diagnostics),
    }
    digests = tuple((name, _part_digest(records)) for name, records in parts.items())
    return StructureSignatureV1(_part_digest([f"{name}={value}" for name, value in digests]), digests)
