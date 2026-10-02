"""Приёмка граней веера: одинаковые углы дают одинаковые грани, вне Blender.

Вход — выгрузки `*_patchNNNN.geometry_batch.json` (кнопка `Build Decal Mesh` и
`tools/blender_field_sweep.py` пишут их тем же кодеком). Для каждого региона станции
`CONSTANT_PHYSICAL_ENDPOINT_S` (один угол: вершина-вершина веера и её лучи) берётся

* ОБВОДКА веера: граничные полурёбра его граней, одним циклом от вершины веера
  (`r = 0`), а в 3D-координатах — относительно неё;
* РАЗБИЕНИЕ обводки на грани: множество множеств её вершин (порядок обхода и
  выбор диагонали различимы, обход нет).

Две обводки КОНГРУЭНТНЫ, если матрицы Грама векторов от вершины веера совпадают
при тождественном либо зеркальном порядке обхода (перенос, поворот и отражение
плоскости не различаются). Конгруэнтные обводки образуют ГРУППУ, и приёмка такая:
внутри группы ОДИН класс разбиения (`--require-single-class` делает это кодом
возврата). Конгруэнтные углы с разной нарезкой на грани — то, что владелец видит
глазами как «одинаковые углы окон выглядят по-разному».

Обводки разных углов (разные лучи, разный срез соседом) в одну метрическую группу не
попадают: это различие решателя, а не материализатора. Но глазами владельца угол окна —
это число вершин обводки и угол веера, а не точное место среза, поэтому есть и ГРУБЫЕ
группы: (число вершин обводки, угол веера в целых градусах), классы разбиения в них — с
точностью до зеркала. Одиночный класс в грубой группе — то, что закон `FAN_FACE_*` обязан
дать на точной плоскости: срезанный веер — одна грань либо один и тот же разрез от вершины.
Плоскость 3D-проверки — точная плоскость домена; на кривой укладке конгруэнтность не
обещана и инструмент о ней ничего не говорит.

    PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python tools/fan_congruence_check.py <dir> [<dir> ...]
        [--tolerance 3e-3] [--require-single-class] [--all-groups]
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from cftuv_envelope import GeometryBatchCodecV1

FAN_MODEL = "CONSTANT_PHYSICAL_ENDPOINT_S"
SLIVER_DEGREES = 2.0


@dataclass(frozen=True)
class Fan:
    """Один веер угла: обводка (вершина веера первой), векторы от неё и грани по индексам обводки."""

    label: str
    vectors: tuple
    faces: tuple
    interior: int
    #: Время прихода `r` каждой вершины обводки (у вершины веера — нуль).
    radii: tuple = ()

    @property
    def partition(self):
        return frozenset(frozenset(face) for face in self.faces)

    @property
    def cut(self) -> bool:
        """Веер срезан соседом: у какой-то вершины фронта `r` меньше, чем у остальных (целый веер — все равны)."""

        rim = self.radii[1:]
        return bool(rim) and min(rim) < max(rim) * (1.0 - 1e-6)


def _sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _outline(faces, apex):
    """Цикл граничных вершин от `apex` либо `None`: полурёбра без встречной пары, один цикл."""

    half = {(face[i], face[(i + 1) % len(face)]) for face in faces for i in range(len(face))}
    successor = {a: b for (a, b) in half if (b, a) not in half}
    if apex not in successor or len(set(successor.values())) != len(successor):
        return None
    cycle = [apex]
    while successor[cycle[-1]] != apex:
        following = successor[cycle[-1]]
        if following in cycle or following not in successor:
            return None
        cycle.append(following)
    return cycle if len(cycle) == len(successor) else None


def fans_of(batch, name):
    """Веера батча: `[Fan, ...]`; веер без одной вершины `r = 0` либо без цикла обводки пропущен (счёт — в 'skipped')."""

    position = {v.vert_key.value: (v.position.x, v.position.y, v.position.z) for v in batch.vertices}
    regions = {
        f.semantic_region_id.value
        for f in batch.station_facts
        if f.station_model_id.value == FAN_MODEL
    }
    apexes: dict = {}
    radius: dict = {}
    for fact in batch.station_facts:
        region = fact.semantic_region_id.value
        if region in regions:
            radius[(region, fact.vert_key.value)] = float(fact.source_r.value)
            if radius[(region, fact.vert_key.value)] == 0.0:
                apexes.setdefault(region, set()).add(fact.vert_key.value)
    found, skipped = [], 0
    for region in sorted(regions):
        faces = [
            tuple(key.value for key in face.ordered_vert_keys)
            for face in batch.faces
            if face.semantic_region_id.value == region
        ]
        apex = apexes.get(region, set())
        cycle = _outline(faces, next(iter(apex))) if len(apex) == 1 else None
        if cycle is None:
            skipped += 1
            continue
        index = {key: number for number, key in enumerate(cycle)}
        found.append(
            Fan(
                label=f"{name}:{region}",
                vectors=tuple(_sub(position[key], position[cycle[0]]) for key in cycle),
                faces=tuple(tuple(index[key] for key in face if key in index) for face in faces),
                interior=len({key for face in faces for key in face} - set(cycle)),
                radii=tuple(radius.get((region, key), 0.0) for key in cycle),
            )
        )
    return found, skipped


def _gram(vectors):
    return [[_dot(a, b) for b in vectors] for a in vectors]


def _alignments(base, other, tolerance):
    """Перестановки индексов `other` -> `base`, при которых матрицы Грама совпадают, и наибольшее отклонение."""

    if len(base.vectors) != len(other.vectors):
        return [], 0.0
    size = len(base.vectors)
    scale = max(_dot(v, v) for v in base.vectors) or 1.0
    reference = _gram(base.vectors)
    valid, worst = [], 0.0
    for order in (list(range(size)), [0, *range(size - 1, 0, -1)]):
        mapped = _gram([other.vectors[i] for i in order])
        gap = max(abs(mapped[i][j] - reference[i][j]) for i in range(size) for j in range(size))
        if gap <= tolerance * scale:
            valid.append(order)
            worst = max(worst, gap / scale)
    return valid, worst


def _group(fans, tolerance):
    """`[(представитель, [(веер, перестановки)], наибольшее отклонение)]`: конгруэнтные обводки вместе."""

    groups: list = []
    for fan in fans:
        for entry in groups:
            orders, worst = _alignments(entry[0], fan, tolerance)
            if orders:
                entry[1].append((fan, orders))
                entry[2] = max(entry[2], worst)
                break
        else:
            groups.append([fan, [(fan, [list(range(len(fan.vectors)))])], 0.0])
    return groups


def _classes(members):
    """Классы разбиения группы: `[(множество разбиений в кадре представителя, число вееров, образец)]`."""

    classes: list = []
    for fan, orders in members:
        # `order[i]` — индекс обводки веера, ставший `i`-м индексом представителя.
        views = set()
        for order in orders:
            back = {old: new for new, old in enumerate(order)}
            views.add(frozenset(frozenset(back[i] for i in face) for face in fan.faces))
        for entry in classes:
            if entry[0] & views:
                entry[0] |= views
                entry[1] += 1
                break
        else:
            classes.append([set(views), 1, fan])
    return classes


def _canonical(partition, size) -> tuple:
    """Разбиение без зеркала: меньшее из прямого и обращённого порядка обводки."""

    mirror = lambda i: 0 if i == 0 else size - i  # noqa: E731
    forms = [
        tuple(sorted(tuple(sorted(mapped(i) for i in face)) for face in partition))
        for mapped in (lambda i: i, mirror)
    ]
    return min(forms)


def coarse_classes(fans):
    """`{(вершин обводки, угол веера в целых градусах, срезан ли): {разбиение: [число вееров, образец]}}`."""

    table: dict = {}
    for fan in fans:
        key = (len(fan.vectors), round(_angle(fan)), fan.cut)
        form = _canonical(fan.partition, len(fan.vectors))
        slot = table.setdefault(key, {}).setdefault(form, [0, fan])
        slot[0] += 1
    return table


def shape_of(fan) -> str:
    """`triangle` (одна грань-треугольник), `sectors` (несколько треугольников целого веера, по сектору на опору),
    `polygon` (есть грань от четырёх вершин), `from-apex` либо `ear-fallback` (срезанный веер из треугольников:
    все ли они выходят из вершины веера)."""

    if any(len(face) > 3 for face in fan.faces):
        return "polygon"
    if len(fan.faces) == 1:
        return "triangle"
    if not fan.cut:
        return "sectors"
    return "from-apex" if all(0 in face for face in fan.faces) else "ear-fallback"


def _min_angle(points):
    best = 180.0
    for i, corner in enumerate(points):
        u, w = _sub(points[(i + 1) % len(points)], corner), _sub(points[i - 1], corner)
        lu, lw = math.sqrt(_dot(u, u)), math.sqrt(_dot(w, w))
        if lu and lw:
            best = min(best, math.degrees(math.acos(max(-1.0, min(1.0, _dot(u, w) / (lu * lw))))))
        else:
            best = 0.0
    return best


def slivers(fans) -> int:
    """Грани веера с углом меньше `SLIVER_DEGREES`."""

    return sum(
        1
        for fan in fans
        for face in fan.faces
        if _min_angle([fan.vectors[i] for i in face]) < SLIVER_DEGREES
    )


def load(paths):
    fans, skipped = [], 0
    for root in paths:
        for path in sorted(Path(root).glob("*_patch*.geometry_batch.json")):
            batch = GeometryBatchCodecV1.loads(path.read_bytes())
            more, none = fans_of(batch, path.name.split(".")[0])
            fans.extend(more)
            skipped += none
    return fans, skipped


def _angle(fan) -> float:
    a, b = fan.vectors[1], fan.vectors[-1]
    la, lb = math.sqrt(_dot(a, a)), math.sqrt(_dot(b, b))
    return math.degrees(math.acos(max(-1.0, min(1.0, _dot(a, b) / (la * lb))))) if la and lb else 0.0


def report(fans, skipped, tolerance, out=print, every_group=False):
    """Печатает группы и возвращает `(групп, групп с несколькими классами, лишних классов)`.

    Строка группы — для групп из нескольких вееров и для любой группы с несколькими классами
    (`every_group` — для всех, и одиночных тоже).
    """

    groups = _group(fans, tolerance)
    extra = multi = 0
    shapes = Counter(shape_of(fan) for fan in fans)
    out(
        f"fans {len(fans)} (skipped {skipped}, with interior vertices "
        f"{sum(1 for fan in fans if fan.interior)}) in {len(groups)} congruent groups "
        f"(tolerance {tolerance:g} of the squared fan radius)"
    )
    for number, (base, members, worst) in enumerate(groups, 1):
        classes = _classes(members)
        multi += len(classes) > 1
        extra += len(classes) - 1
        if len(members) < 2 and len(classes) < 2 and not every_group:
            continue
        parts = "; ".join(
            f"{count}x {shape_of(sample)} {sorted(len(f) for f in sample.faces)}"
            for _views, count, sample in classes
        )
        out(
            f"group {number}: angle {_angle(base):.2f} deg, outline {len(base.vectors)} vertices, "
            f"{len(members)} fans, {len(classes)} class(es) [{parts}], max gram gap {worst:.1e}"
        )
    coarse = coarse_classes(fans)
    coarse_multi = 0
    for (count, angle, cut), forms in sorted(coarse.items()):
        coarse_multi += len(forms) > 1
        parts = "; ".join(
            f"{number}x {shape_of(sample)} {sorted(len(f) for f in sample.faces)}"
            for number, sample in forms.values()
        )
        out(
            f"coarse group outline {count} vertices, angle {angle} deg, {'cut' if cut else 'intact'}: "
            f"{sum(n for n, _s in forms.values())} fans, {len(forms)} class(es) [{parts}]"
        )
    out(f"shapes {dict(sorted(shapes.items()))}; faces with a corner under {SLIVER_DEGREES} deg: {slivers(fans)}")
    out(
        f"metric groups {len(groups)}, with several classes {multi} (extra classes {extra}); "
        f"coarse groups {len(coarse)}, with several classes {coarse_multi}"
    )
    return len(groups), multi + coarse_multi, extra


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("directories", nargs="+")
    parser.add_argument("--tolerance", type=float, default=3e-3)
    parser.add_argument("--require-single-class", action="store_true")
    parser.add_argument("--all-groups", action="store_true")
    args = parser.parse_args(argv)
    fans, skipped = load(args.directories)
    _groups, multi, _extra = report(fans, skipped, args.tolerance, every_group=args.all_groups)
    return 1 if args.require_single_class and (multi or not fans) else 0


if __name__ == "__main__":
    sys.exit(main())
