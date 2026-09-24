"""GraphML-вход Surfer2 из полигонов нашего корпуса (kernel/tests/wavefront_cases.py).

Формат — по README Surfer2 (GraphML + GRAPH-ATTRIBUTES): узлы несут
vertex-coordinate-x/y, рёбра — edge-weight=1.0 (единичная скорость).
Порядок рёбер значим: BasicTriangulation::tag_components() сеет компоненту 0
СЛЕВА от первого ребра списка (u->v), поэтому для CCW-контура компонента 0 —
внутренность. Это проверяется эмпирически, а не принимается на веру.
"""
import sys

sys.path[:0] = ["/home/user/CFTUV/kernel/src", "/home/user/CFTUV/kernel/tests"]

HEADER = (
 '<graphml xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" '
 'xmlns="http://graphml.graphdrawing.org/xmlns" '
 'xsi:schemaLocation="http://graphml.graphdrawing.org/xmlns '
 'http://graphml.graphdrawing.org/xmlns/1.0/graphml.xsd">\n'
 '  <key attr.name="vertex-coordinate-x" attr.type="string" for="node" id="x"/>\n'
 '  <key attr.name="vertex-coordinate-y" attr.type="string" for="node" id="y"/>\n'
 '  <key attr.name="edge-weight" attr.type="string" for="edge" id="w"/>\n'
 '  <key attr.name="edge-weight-additive" attr.type="string" for="edge" id="wa"/>\n'
 '  <graph edgedefault="undirected">\n')


def shoelace(points):
    n = len(points)
    return sum(points[i][0] * points[(i + 1) % n][1]
               - points[(i + 1) % n][0] * points[i][1] for i in range(n))


def ccw(points):
    """Surfer2 сеет компоненту 0 СЛЕВА от первого ребра списка; для CCW-контура
    это внутренность. Ориентация нормируется здесь, чтобы это было так по
    построению, а не по удаче конкретной фигуры корпуса."""
    return tuple(points) if shoelace(points) > 0 else tuple(reversed(points))


def emit(points, path):
    points = ccw(points)
    out = [HEADER]
    for i, (x, y) in enumerate(points):
        out.append(f'    <node id="{i}">\n      <data key="x">{x}</data>\n'
                   f'      <data key="y">{y}</data>\n    </node>\n')
    n = len(points)
    for i in range(n):
        out.append(f'    <edge source="{i}" target="{(i + 1) % n}">\n'
                   f'      <data key="w">1.0</data>\n    </edge>\n')
    out.append('  </graph>\n</graphml>\n')
    with open(path, "w") as fh:
        fh.write("".join(out))


CASES = {}


def cases():
    """Односвязные фигуры корпуса с положительными единичными весами.

    Пересечение моделей, объявленное карточкой R0: плоский PSLG, положительные
    веса, без стен и вееров. Фигуры с дырами (holes_*) и весами сюда НЕ входят —
    им нужен многокольцевой генератор, это отдельная карточка.
    """
    import wavefront_cases as wc
    out = {
        "axis_square_8": wc.axis_square(8),
        "ell_12": wc.ell(12),
        # ниже — расширение сверх плана карточки: узлы с иррациональными
        # координатами, ради которых допуск чтения вообще имеет смысл.
        "right_triangle_12": wc.right_triangle(12),
        "diamond_6": wc.diamond(6),
        "staircase": wc.staircase(),
        "u_shape": wc.u_shape(),
        "double_notch": wc.double_notch(),
        "comb_3": wc.comb(3),
        # крест — вырожденный контроль: центральный блок схлопывается РАЗОМ,
        # четыре события в одном времени. Ровно тут модели могут разойтись
        # представлением узла (у нас один MULTIWAY-узел, у Surfer2 — DCEL).
        "cross_4x4": wc.cross(),
        "cross_6x4": wc.cross(wide=6),
    }
    star = wc.star(9, 7)
    if star is not None:
        out["star_9_7"] = star
    return out


if __name__ == "__main__":
    outdir = sys.argv[1] if len(sys.argv) > 1 else "."
    for name, poly in cases().items():
        pts = poly.outer.points
        emit(pts, f"{outdir}/{name}.graphml")
        print(name, pts)
