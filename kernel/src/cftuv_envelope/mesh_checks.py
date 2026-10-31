"""Проверки собранного меша декали (MESH_ASSEMBLY_NATIVE_V1, срез B1): эталон, с которым побитово сверен нативный порт (`cftuv_native.mesh_*`).

Чистые функции чисел над массивами меша после сварки: плоскость граней после смещения, складки между доменами с разрывом UV и
полурёбра двух граней. Хост (`cftuv/envelope_mesh_assembly.py`) зовёт их через `backend.mesh_assembly_compute`, когда заказан
нативный путь, и сюда, когда заказан эталон или порт отказал по имени. Семантика — CPython: вычитание и сумма слева направо
(без FMA), `max` берёт первое из равных, `round` — к чётному (его делает хост над `worst`), `!=` над UV — равенство чисел
(`-0.0 == 0.0`).
"""

from __future__ import annotations

import hashlib
import json
import math


def _cross(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _dot(left, right) -> float:
    return left[0] * right[0] + left[1] * right[1] + left[2] * right[2]


def off_plane_measure(positions, faces) -> tuple:
    """`(наибольшее отклонение, метры; число граней с отклонением от нанометра)` граней от четырёх вершин.

    Плоскость грани — через её первую вершину с нормалью вектора площади (веер из первой вершины); мера — наибольшее расстояние
    вершины грани от неё. Грань нулевой площади ничего не меряет. Разрешение записи — нанометр: шум округления binary64
    (1e-16 м) гранью «вне плоскости» не числится (`round(отклонение * 1e9) > 0`).
    """

    worst, off = 0.0, 0
    for loop in faces:
        if len(loop) < 4:
            continue
        points = [positions[index] for index in loop]
        origin = points[0]
        normal = (0.0, 0.0, 0.0)
        for index in range(1, len(points) - 1):
            part = _cross(
                tuple(b - a for a, b in zip(origin, points[index])),
                tuple(b - a for a, b in zip(origin, points[index + 1])),
            )
            normal = tuple(a + b for a, b in zip(normal, part))
        length = math.sqrt(_dot(normal, normal))
        if not length:
            continue
        deviation = max(
            abs(_dot(tuple(b - a for a, b in zip(origin, point)), normal)) / length
            for point in points[1:]
        )
        worst = max(worst, deviation)
        off += int(round(deviation * 10**9) > 0)
    return worst, off


def _edge_uvs(faces, uvs, face_domain):
    """`{ребро: [(грань, домен, uv у меньшей вершины, uv у большей)]}` по петлям меша."""

    edges: dict = {}
    cursor = 0
    for number, loop in enumerate(faces):
        size = len(loop)
        for position in range(size):
            first, second = loop[position], loop[(position + 1) % size]
            uv_first = uvs[cursor + position]
            uv_second = uvs[cursor + (position + 1) % size]
            if first > second:
                first, second, uv_first, uv_second = second, first, uv_second, uv_first
            edges.setdefault((first, second), []).append(
                (number, face_domain[number], uv_first, uv_second)
            )
        cursor += size
    return edges


def cross_domain_seams(faces, uvs, face_domain) -> tuple:
    """Рёбра меша между гранями РАЗНЫХ доменов, где UV разрывен: шов на сваренной складке (по возрастанию)."""

    return tuple(
        sorted(
            edge
            for edge, sides in _edge_uvs(faces, uvs, face_domain).items()
            if len(sides) == 2
            and sides[0][1] != sides[1][1]
            and (sides[0][2] != sides[1][2] or sides[0][3] != sides[1][3])
        )
    )


def half_edge_conflicts(faces) -> int:
    """Направленные рёбра, лежащие в двух гранях: так сваренные соседи с разным обходом."""

    seen: dict = {}
    for loop in faces:
        size = len(loop)
        for position in range(size):
            half = (loop[position], loop[(position + 1) % size])
            seen[half] = seen.get(half, 0) + 1
    return sum(1 for count in seen.values() if count > 1)


def _run_between(anchors, pair, own) -> bool:
    """Идёт ли по цепи (`anchors`) от одного конца `pair` к другому путь через `src:`, которых нет у домена пары (`own`)."""

    for at, key in enumerate(anchors):
        if key not in pair:
            continue
        for stop in range(at + 1, len(anchors)):
            if anchors[stop] == key:
                break
            if anchors[stop] in pair:
                between = anchors[at + 1 : stop]
                if between and own.isdisjoint(between):
                    return True
                break
    return False


def _anchors_between(anchored, held) -> int:
    """Пары соседних `src:` одного домена, между которыми у другого домена на цепи стоят ещё `src:`, каких у первого домена нет (вершина растворена лишь с одной стороны).

    Счёт по числу вершин `node:`/`clip:` между соседними `src:` такого шва не видит: пара, соседняя у одного домена, у другого
    не пара вовсе. Каждая такая пара — T-стык: вершина другого домена лежит на ребре этого.

    Пара сверяется с соседними `src:` ВСЕХ цепей другого домена, а не одной (вершина стоит на стыке двух цепей, и «первая» цепь
    зависела от `PYTHONHASHSEED`), и пара, соседняя хоть на одной цепи, шов не рвёт. Путь, замкнутый на ребро пары, — обход:
    цепь `14 -> 21 -> 20 -> 19 -> 18 -> 13` соседа идёт вокруг ребра `(13, 14)` домена, у которого есть и ребро, и вершины обхода
    (шестиугольный патч внутри кольца); это другой путь по источнику, а не то же ребро с лишней вершиной. Поэтому лишняя вершина
    считается, только если её нет у домена пары вовсе (`held`: `src:` его цепей источника и стены). Ответ — множество, от
    порядка цепей не зависит.
    """

    # Индекс по вершине: `{ключ: {(домен, номер цепи домена)}}`. Путь между концами пары (`_run_between`) бывает только на цепи, где есть ОБА
    # конца, поэтому кандидаты пары — пересечение двух множеств, а не все домены: счёт линеен по числу пар (без квадрата по числу доменов).
    runs: dict = {}
    consecutive: dict = {}
    where: dict = {}
    for number, _chain_id, anchors in anchored:
        chains = runs.setdefault(number, [])
        for key in anchors:
            where.setdefault(key, set()).add((number, len(chains)))
        chains.append(anchors)
        consecutive.setdefault(number, set()).update(frozenset(pair) for pair in zip(anchors, anchors[1:]))
    open_pairs = set()
    for number, pairs in consecutive.items():
        for pair in pairs:
            if len(pair) != 2 or pair in open_pairs:
                continue
            first, second = tuple(pair)
            for other, at in where[first] & where[second]:
                if other != number and pair not in consecutive[other] and _run_between(runs[other][at], pair, held[number]):
                    open_pairs.add(pair)
                    break
    return len(open_pairs)


def seam_report_numbers(domains) -> tuple:
    """`(T-стыков шва между доменами, вершин clip: на шовных цепях)` по цепям доменов (`((имя цепи, (ключ вершины, ...)), ...)` на домен).

    Шовные цепи — `boundary:SOURCE:*` и `boundary:WALL:*`. Точно, без допусков: ключи вершин, а не координаты; ответ от порядка множеств не зависит.
    """

    by_pair: dict = {}
    clip_vertices = 0
    anchored: list = []
    held: dict = {}
    for number, chains in enumerate(domains):
        for name, chain_keys in chains:
            kind = name.split(":")[1]
            if kind not in ("SOURCE", "WALL"):
                continue
            keys = list(chain_keys)
            anchors = [index for index, key in enumerate(keys) if key.startswith("src:")]
            clip_vertices += sum(1 for key in keys if key.startswith("clip:"))
            held.setdefault(number, set()).update(keys[index] for index in anchors)
            if kind != "SOURCE":
                continue
            anchored.append((number, name, [keys[index] for index in anchors]))
            for first, second in zip(anchors, anchors[1:]):
                by_pair.setdefault(frozenset((keys[first], keys[second])), []).append(second - first - 1)
    junctions = sum(1 for counts in by_pair.values() if len(counts) > 1 and len(set(counts)) > 1)
    junctions += _anchors_between(anchored, held)
    return junctions, clip_vertices


def arrays_digest(positions, faces, uvs, face_domain, face_owner, seams) -> str:
    """sha256 канонического JSON массивов меша (ключи по алфавиту, без пробелов, числа `float.__repr__`)."""

    arrays = {"positions": positions, "faces": faces, "uvs": uvs, "face_domain": face_domain, "face_owner": face_owner, "seams": seams}
    text = json.dumps(arrays, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("ascii")).hexdigest()


__all__ = ("arrays_digest", "cross_domain_seams", "half_edge_conflicts", "off_plane_measure", "seam_report_numbers")
