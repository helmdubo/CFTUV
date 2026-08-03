"""Once-only леджер зародышей контакта одного superlevel (AUTH Q-10-ADD).

Зародыш — **локус контакта**, а не событие. Его ключ:

* канонический `t` (через `EventTimeV1.canonical`),
* каноническая точка,
* канонический **НАБОР КОНЦОВ**, где конец — пара
  `(ключ вхождения, примитивный целочисленный луч ОТ точки контакта вдоль
  этого вхождения)`.

Плоская тройка `(t, point, participants)` тождеством быть не может: близнецы
разреза делят ключ вхождения, и их различают ТОЛЬКО лучи. Вид события
(`EventKind`) в ключ не входит — иначе дубль другого вида пролезает как новый
зародыш; именно это и наблюдалось на `ell_12_source_edges_0_1`, где один локус
описывали два EDGE-инцидента и два endpoint-SPLIT.

Луч берётся из frozen prestate (`_VertexSnapshot.incoming_ray/outgoing_ray`,
`SuperlevelIncidentV1.target_ray`), то есть из `_direction(line)/gcd`, а не из
ключа вхождения: у скрытой опоры веера ключ вырожден (`x, y, x, y, ordinal`) и
направления не несёт, а прямая — несёт.

Скоуп леджера — ВЕСЬ superlevel-пакет, а не processing unit: 35 из 63 фигур
корпуса несут больше одной компоненты на уровень, и юнитовый леджер вернул бы
ровно ту двойную материализацию, ради которой леджер и заведён.
"""

from __future__ import annotations

from dataclasses import dataclass


def port_ends(vertex) -> frozenset:
    """Концы существующего порта LAV: назад по prev и вперёд по next."""

    ends = set()
    if vertex.prev_occurrence is not None:
        ends.add((vertex.prev_occurrence[0], vertex.incoming_ray))
    if vertex.next_occurrence is not None:
        ends.add((vertex.next_occurrence[0], vertex.outgoing_ray))
    return frozenset(ends)


def target_ends(incident) -> frozenset:
    """Концы, которые рассекаемое вхождение приносит в точку контакта.

    Контакт в конце пролёта даёт ОДИН луч — внутрь пролёта. Контакт внутри
    (настоящий разрез) даёт ДВА: локус лежит между двумя будущими близнецами.
    Вырожденный пролёт, у которого оба конца стоят в точке контакта, приносит
    оба луча: он весь лежит в локусе.
    """

    occurrence, ray = incident.target_occurrence, incident.target_ray
    if occurrence is None or ray is None:
        return frozenset()
    backward = (-ray[0], -ray[1])
    ends = set()
    if incident.point_key == occurrence[1]:
        ends.add((occurrence[0], ray))
    if incident.point_key == occurrence[2]:
        ends.add((occurrence[0], backward))
    if not ends:
        ends.update(((occurrence[0], ray), (occurrence[0], backward)))
    return frozenset(ends)


def incident_ends(incident, vertices, *, edge_kind) -> frozenset:
    """Полный набор концов, который инцидент приносит в свой локус."""

    ends = port_ends(vertices[incident.event.vertex])
    if incident.event.kind is edge_kind:
        return ends | port_ends(vertices[incident.event.peer])
    return ends | target_ends(incident)


def locus_ends(idents, vertices) -> frozenset:
    """Набор концов локуса, который уже занят планом смерти этих портов."""

    return frozenset().union(
        frozenset(), *(port_ends(vertices[ident]) for ident in idents)
    )


def junction_ends(
    prev_occurrence,
    next_occurrence,
    rays,
) -> tuple | None:
    """Канонический набор концов одного ContactJunction.

    Луч prev-конца смотрит НАЗАД по входящему плечу, луч next-конца — ВПЕРЁД
    по исходящему. Знак и есть то, что различает близнецов одного разреза:
    ключ вхождения у них общий.
    """

    prev_ray = rays.get(prev_occurrence[0])
    next_ray = rays.get(next_occurrence[0])
    if prev_ray is None or next_ray is None:
        return None
    return tuple(
        sorted(
            (
                (prev_occurrence[0], (-prev_ray[0], -prev_ray[1])),
                (next_occurrence[0], next_ray),
            ),
            key=repr,
        )
    )


@dataclass(frozen=True, slots=True)
class GermKeyV1:
    """Тождество локуса контакта. Вид события сюда не входит намеренно."""

    time_key: tuple
    point_key: tuple
    ends: tuple


class SuperlevelGermLedgerV1:
    """Зародыш материализуется единожды; повтор ключа — тот же объект.

    Леджер не «фильтрует дубликаты» — он ОТВЕЧАЕТ, каким объектом является
    предъявленный локус. Конфликт (один ключ, два разных представителя)
    остаётся именованным отказом вызывающего, а не молчаливым выбором.
    """

    __slots__ = ("_by_key", "_rays")

    def __init__(self, rays: dict | None = None):
        self._by_key: dict[GermKeyV1, object] = {}
        self._rays: dict = dict(rays or {})

    @property
    def rays(self) -> dict:
        return self._rays

    def learn_rays(self, occurrence, ray) -> None:
        if occurrence is not None and ray is not None:
            self._rays.setdefault(occurrence[0], ray)

    def key(self, time_key, point_key, prev_occurrence, next_occurrence):
        ends = junction_ends(prev_occurrence, next_occurrence, self._rays)
        if ends is None:
            return None
        return GermKeyV1(time_key, point_key, ends)

    def materialize(self, key, germ, *, fold):
        """Вернуть уже материализованный зародыш либо запомнить новый.

        `fold` сворачивает нагрузку двух предъявлений одного ключа. Порядок
        предъявления на результат не влияет: представитель — канонический
        минимум по `repr` ключа объекта, нагрузка — объединение.
        """

        previous = self._by_key.get(key)
        if previous is None:
            self._by_key[key] = germ
            return germ
        merged = fold(previous, germ)
        self._by_key[key] = merged
        return merged

    def __len__(self) -> int:
        return len(self._by_key)
