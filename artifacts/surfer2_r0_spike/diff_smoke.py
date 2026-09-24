"""R0 дифференциальный smoke: exact-Surfer2 против ядра CFTUV.

МЕТОДИКА (объявляется здесь, а не подразумевается).

1. Модели входа РАЗНЫЕ. Наш вход — многоугольник с фронтом ВНУТРИ. Вход
   Surfer2 — PSLG, и фронт у него идёт с ОБЕИХ сторон каждого ребра. Поэтому
   выход Surfer2 фильтруется: берутся только узлы, лежащие СТРОГО ВНУТРИ
   контура. Фильтр геометрический (point-in-polygon по ray casting на точных
   целых), а не «по номеру компоненты»: номер компоненты у Surfer2 зависит от
   порядка рёбер во входе (BasicTriangulation::tag_components сеет компоненту 0
   слева от ПЕРВОГО ребра списка), и опираться на него значило бы опираться на
   недокументированную деталь.
   Тот же прогон делается и через --component=0/--component=1; согласие двух
   независимых способов выделения внутренности печатается как отдельная строка
   отчёта. Расходятся — smoke считается несостоявшимся, а не «почти прошедшим».

2. Узлы t=0 отбрасываются с обеих сторон. Surfer2 кладёт в DCEL и вершины
   ВХОДА (z=0); у нас `sk.nodes` — только события. Сравниваются события.

3. Surfer2 печатает координаты через CGAL::to_double, то есть выход ВСЕГДА
   double, даже при точном ядре внутри. Поэтому сравнение — с допуском ЧТЕНИЯ.
   Это оракул-смок, не доказательство: допуск описывает потерю на печати, а не
   заявленную точность ядра. Наша сторона переводится в float через строгую
   оболочку `enclosure(160)` (середина интервала), то есть ошибка перевода
   лежит ниже 2^-100 и в бюджет допуска не входит.

4. Сопоставление — паросочетание по минимуму расстояния (жадное на полном
   переборе; на смоук-размерах это точно). Кратности учитываются: каждый узел
   используется один раз. Невязка отчёта — max по паре из |dx|,|dy|,|dt|.
"""

from __future__ import annotations

import json
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

sys.path[:0] = ["/home/user/CFTUV/kernel/src", "/home/user/CFTUV/kernel/tests"]

HERE = Path(__file__).resolve().parent
SURFER = HERE.parent / "src" / "build" / "cc" / "surfer"
OUT = HERE / "out"

# Бюджет потери на ПЕЧАТИ. SkeletonDCEL::write_obj пишет
# `os << CGAL::to_double(p.x())` БЕЗ setprecision, то есть на дефолтной точности
# ostream — 6 ЗНАЧАЩИХ цифр. Значит абсолютного допуска не существует: невязка
# печати пропорциональна модулю координаты. Допуск объявляется относительным к
# масштабу фигуры, и это свойство ЧУЖОГО вывода, а не наша уступка.
READ_RELATIVE = 1e-5      # > 0.5 ulp шестизначной печати (5e-6)
READ_FLOOR = 1e-9         # пол для фигур с координатами O(1)
ENCLOSURE_BITS = 160


# ---------------------------------------------------------------- наша сторона

def exact_to_float(value) -> float:
    low, high = value.enclosure(ENCLOSURE_BITS)
    return float((low + high) / 2)


def time_to_float(t) -> float:
    return float(Fraction(t.dividend)) / exact_to_float(t.divisor)


def kernel_nodes(polygon):
    from cftuv_envelope.wavefront import build_skeleton

    sk = build_skeleton(polygon)
    rows = []
    for n in sk.nodes:
        rows.append(
            {
                "kind": str(n.kind),
                "x": exact_to_float(n.point.x),
                "y": exact_to_float(n.point.y),
                "t": time_to_float(n.time),
                "converging": n.converging_vertices,
            }
        )
    return sk, rows


# -------------------------------------------------------------- сторона Surfer2

def parse_obj(path: Path):
    verts = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if parts and parts[0] == "v":
            verts.append(tuple(float(v) for v in parts[1:4]))
    return verts


def point_in_polygon(px: float, py: float, points) -> bool:
    """Строго внутри. Контур на целых -> сравнение через Fraction, без порога."""
    x = Fraction(px).limit_denominator(10**12)
    y = Fraction(py).limit_denominator(10**12)
    n = len(points)
    inside = False
    for i in range(n):
        ax, ay = (Fraction(c) for c in points[i])
        bx, by = (Fraction(c) for c in points[(i + 1) % n])
        # на границе -> не «строго внутри»
        cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax)
        if cross == 0 and min(ax, bx) <= x <= max(ax, bx) and min(ay, by) <= y <= max(ay, by):
            return False
        if (ay > y) != (by > y):
            xint = ax + (y - ay) * (bx - ax) / (by - ay)
            if x < xint:
                inside = not inside
    return inside


def run_surfer_simple(graphml: Path, component: str, tag: str):
    OUT.mkdir(exist_ok=True)
    obj = OUT / f"{tag}.obj"
    err = OUT / f"{tag}.err"
    with err.open("wb") as fe:
        rc = subprocess.call([str(SURFER), f"--component={component}",
                              str(graphml), str(obj)], stderr=fe)
    return rc, obj


# -------------------------------------------------------------------- сравнение

def match(ours, theirs, tol):
    """Паросочетание по минимуму расстояния; возвращает пары и невязки."""
    unused = list(range(len(theirs)))
    pairs, unmatched_ours = [], []
    for i, a in enumerate(ours):
        best, best_d = None, None
        for j in unused:
            b = theirs[j]
            d = max(abs(a["x"] - b[0]), abs(a["y"] - b[1]), abs(a["t"] - b[2]))
            if best_d is None or d < best_d:
                best, best_d = j, d
        if best is None or best_d > tol:
            unmatched_ours.append(i)
        else:
            unused.remove(best)
            pairs.append((i, best, best_d))
    return pairs, unmatched_ours, unused


def main():
    from make_graphml import cases as corpus, ccw

    cases = corpus()
    report = {
        "tolerance_model": {
            "relative": READ_RELATIVE,
            "floor": READ_FLOOR,
            "reason": "SkeletonDCEL::write_obj печатает to_double без "
                      "setprecision -> 6 значащих цифр",
        },
        "cases": {},
    }

    for name, polygon in cases.items():
        graphml = HERE / f"{name}.graphml"
        pts = ccw(polygon.outer.points)
        scale = max(max(abs(x), abs(y)) for x, y in pts)
        tol = max(READ_FLOOR, READ_RELATIVE * scale)

        rc_all, obj_all = run_surfer_simple(graphml, "-1", f"{name}.all")
        rc_c0, obj_c0 = run_surfer_simple(graphml, "0", f"{name}.c0")
        rc_c1, obj_c1 = run_surfer_simple(graphml, "1", f"{name}.c1")

        v_all = parse_obj(obj_all)
        v_c0 = parse_obj(obj_c0)
        v_c1 = parse_obj(obj_c1)

        # способ A: геометрический фильтр по всему прогону
        inner_geom = sorted(v for v in v_all if v[2] > 0 and point_in_polygon(v[0], v[1], pts))
        # способ B: компонента 0 (по порядку рёбер = внутренность CCW-контура)
        inner_comp = sorted(v for v in v_c0 if v[2] > 0)
        outer_comp = sorted(v for v in v_c1 if v[2] > 0)

        # Согласие двух способов выделения внутренности — контроль методики.
        # Оно ОСМЫСЛЕННО только если прогон по компоненте вообще состоялся:
        # на `--component=0` Surfer2 падает по SIGSEGV для 3-вершинных входов
        # (воспроизводится и на их собственном test-data/convex03.graphml).
        # Записываем это как отдельный исход, а не как «методики разошлись».
        if rc_c0 != 0:
            agree = "n/a: surfer --component=0 rc=%d" % rc_c0
        else:
            agree = inner_geom == inner_comp
        # Опорный прогон обязан быть успешным: на нём и строится сравнение.
        assert rc_all == 0, f"{name}: surfer --component=-1 rc={rc_all}"

        sk, ours = kernel_nodes(polygon)
        pairs, un_ours, un_theirs = match(ours, inner_geom, tol)
        max_res = max((d for _, _, d in pairs), default=0.0)

        report["cases"][name] = {
            "scale": scale,
            "tolerance_used": tol,
            "max_residual_relative": max_res / scale if scale else 0.0,
            "input_points": [list(p) for p in pts],
            "surfer_rc": {"all": rc_all, "c0": rc_c0, "c1": rc_c1},
            "surfer_vertices_total_all_components": len(v_all),
            "surfer_interior_nodes_geometric": [list(v) for v in inner_geom],
            "surfer_interior_nodes_component0": [list(v) for v in inner_comp],
            "surfer_exterior_nodes_component1": [list(v) for v in outer_comp],
            "interior_selection_methods_agree": agree,
            "kernel_outcome": str(sk.outcome),
            "kernel_levels": sk.levels,
            "kernel_nodes": ours,
            "matched": len(pairs),
            "kernel_only": [ours[i] for i in un_ours],
            "surfer_only": [list(inner_geom[j]) for j in un_theirs],
            "max_residual": max_res,
        }

    print(json.dumps(report, indent=2, ensure_ascii=False))
    (HERE / "diff_smoke_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False)
    )


if __name__ == "__main__":
    main()
