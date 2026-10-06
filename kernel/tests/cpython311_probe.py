"""Зонд для `test_cpython311.py`: печатает одну JSON-строку с ответами и ЦЕНОЙ представительных случаев резки и смещения.

Запускается как скрипт под любым интерпретатором (без pytest): `PYTHONPATH=kernel/src python cpython311_probe.py`. Под CPython 3.11 и
3.13 строка обязана совпасть целиком, кроме ключей `info_*`: ответы, `SIGN_COUNTS` и `EXACT_WORK_*` не зависят от версии.
Случаи: резка длинных граней по решётке из 2 * 72^2 треугольников (в `_ordered` попадают наборы из сотни узлов: слияния Powersort и
галоп), сортировка точных значений с корнями под бюджетом (цена `EXACT_WORK_*` ненулевая), смешение нормалей смещения и float-суммы
аудита, силуэта и разреза кольца.
"""

from __future__ import annotations

import hashlib
import json
import sys
from fractions import Fraction

CELLS = 72


def _stream(seed: int):
    state = seed
    while True:
        state = (state * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFFFFFFFFFF
        yield (state >> 11) / float(1 << 53)


def _lattice_lift():
    from cftuv_envelope.materialize.lift_surface import SurfaceLiftV1

    def corner(i, j):
        return (Fraction(i), Fraction(j), Fraction(((i * 7 + j * 13) % 11) / 10.0))

    items = []
    for i in range(CELLS):
        for j in range(CELLS):
            a, b, c, d = (i, j), (i + 1, j), (i + 1, j + 1), (i, j + 1)
            items.append((f"t{i:03d}_{j:03d}a", (a, b, c), tuple(corner(*p) for p in (a, b, c))))
            items.append((f"t{i:03d}_{j:03d}b", (a, c, d), tuple(corner(*p) for p in (a, c, d))))
    return SurfaceLiftV1.from_triangles(items, scale=CELLS)


def _rect(x0, y0, x1, y1):
    return [(Fraction(x0), Fraction(y0)), (Fraction(x1), Fraction(y0)), (Fraction(x1), Fraction(y1)), (Fraction(x0), Fraction(y1))]


CLIP_CASES = {
    "long_rect": [_rect(Fraction(1, 2), Fraction(1, 4), CELLS - Fraction(1, 2), Fraction(3, 4))],
    "tall_rect": [_rect(Fraction(1, 4), Fraction(1, 2), Fraction(3, 4), CELLS - Fraction(1, 2))],
    "slanted": [
        [
            (Fraction(1, 3), Fraction(1, 3)),
            (CELLS - Fraction(1, 3), Fraction(5, 3)),
            (CELLS - Fraction(1, 3), Fraction(8, 3)),
            (Fraction(1, 3), Fraction(4, 3)),
        ]
    ],
    "neighbours": [
        _rect(Fraction(1, 2), Fraction(5, 4), CELLS - Fraction(1, 2), Fraction(9, 4)),
        _rect(Fraction(1, 2), Fraction(9, 4), CELLS - Fraction(1, 2), Fraction(13, 4)),
        _rect(Fraction(1, 2), Fraction(13, 4), Fraction(30), Fraction(17, 4)),
    ],
}


def _clip_record(lift, faces):
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.exact_sqrt_sum import SIGN_COUNTS, SqrtSumV1, exact_work_budget, reset_factorization_memory, reset_sign_counts
    from cftuv_envelope.materialize import clip
    from cftuv_envelope.materialize.clip import ClipStageV1

    reset_factorization_memory()
    reset_sign_counts()
    sizes: list[int] = []
    original = clip.sorted_as_cpython311

    def recording(items, compare):
        sizes.append(len(items))
        return original(items, compare)

    clip.sorted_as_cpython311 = recording
    try:
        budget = exact_work_budget(stage="MATERIALIZE_PROBE", domain_id="clip")
        keys, points, cycles, polygons = {}, {}, [], []
        for face in faces:
            cycle = []
            for xy in face:
                if xy not in keys:
                    keys[xy] = f"p{len(keys)}"
                    points[keys[xy]] = (SqrtSumV1.rational(xy[0]), SqrtSumV1.rational(xy[1]))
                cycle.append((keys[xy], points[keys[xy]]))
            cycles.append(cycle)
            polygons.append((tuple(key for key, _point in cycle),))
        stage = ClipStageV1(lift.bind(budget), budget, points)
        result = stage.run(cycles, polygons, DecalTopologyLawV1.PLANAR_POLYGONS_V1)
    finally:
        clip.sorted_as_cpython311 = original
    return {
        "points": {key: [str(axis.as_rational()) for axis in value] for key, value in sorted(result.points.items())},
        "polygons": [[list(piece) for piece in face] for face in result.polygons],
        "cycles": [[key for key, _point in cycle] for cycle in result.cycles],
        "counters": [list(item) for item in result.counters],
        "note": result.note,
        "sign_counts": dict(SIGN_COUNTS),
        "exact_work": [list(item) for item in budget.counters()],
        "ordered_sizes": sorted(sizes),
    }


def _sqrt_sorted():
    from cftuv_envelope._cpython311 import sorted_as_cpython311
    from cftuv_envelope.exact_sqrt_sum import SIGN_COUNTS, SqrtSumV1, exact_work_budget, reset_factorization_memory, reset_sign_counts

    reset_factorization_memory()
    reset_sign_counts()
    budget = exact_work_budget(stage="MATERIALIZE_PROBE", domain_id="sqrt")
    base = 10**18 + 9
    values = [
        SqrtSumV1.radical(1, base + k, budget) + SqrtSumV1.radical(1, base - k, budget) for k in range(0, 90)
    ]
    draw = _stream(311)
    order = sorted(range(len(values)), key=lambda _index: next(draw))
    ordered = sorted_as_cpython311(order, lambda a, b: (values[a] - values[b]).sign(budget=budget))
    return {
        "order": ordered,
        "sign_counts": dict(SIGN_COUNTS),
        "exact_work": [list(item) for item in budget.counters()],
    }


def _digest(items) -> str:
    return hashlib.sha256("\n".join(items).encode("ascii")).hexdigest()


def _blend():
    from cftuv_envelope._cpython311 import left_fold_sum
    from cftuv_envelope.materialize.offset_normal import blend

    draw = _stream(2026)
    lines, differs = [], 0
    for _ in range(4000):
        raw = [next(draw) + 1e-3 for _ in range(3)]
        weights = tuple(item / (raw[0] + raw[1] + raw[2]) for item in raw)
        normals = tuple(tuple(next(draw) * 2.0 - 1.0 for _ in range(3)) for _ in range(3))
        try:
            mixed = blend(weights, normals)
        except Exception as error:  # именованный отказ нулевой нормали тоже ответ
            lines.append(type(error).__name__)
            continue
        lines.append(",".join(float(axis).hex() for axis in mixed))
        for axis in range(3):
            terms = [weight * normal[axis] for weight, normal in zip(weights, normals)]
            differs += sum(terms) != left_fold_sum(terms)
    return {"digest": _digest(lines), "cases": len(lines)}, differs


def _float_sums():
    from cftuv_envelope._annulus_cut import _bend_cost, _bisector_cost
    from cftuv_envelope._cpython311 import left_fold_sum
    from cftuv_envelope.materialize.audit import _normal_spread
    from cftuv_envelope.materialize.silhouette import _plane_of, uv_fit_residual

    draw = _stream(7)
    spread, plane, fit, costs = [], [], [], []
    for _ in range(600):
        normals = [tuple(next(draw) - 0.5 for _ in range(3)) for _ in range(4)]
        normals = [tuple(axis / left_fold_sum(part * part for part in item) ** 0.5 for axis in item) for item in normals]
        spread.append(_normal_spread(normals).hex())
        points = [tuple(next(draw) * 3.0 for _ in range(3)) for _ in range(5)]
        found = _plane_of(points)
        plane.append("none" if found is None else ",".join(float(axis).hex() for part in found[:2] for axis in part))
        chart = [(next(draw), next(draw)) for _ in range(5)]
        uvs = [(next(draw), next(draw)) for _ in range(5)]
        residual = uv_fit_residual(chart, uvs)
        fit.append("none" if residual is None else residual.hex())
        positions = {index: tuple(next(draw) * 4.0 for _ in range(3)) for index in range(4)}
        costs.append(_bisector_cost(positions, 0, 1, 2, 3).hex())
        costs.append(_bend_cost(positions, 0, 1, 2).hex())
    return {
        "audit_spread": _digest(spread),
        "silhouette_plane": _digest(plane),
        "silhouette_fit": _digest(fit),
        "annulus_cost": _digest(costs),
    }


def main() -> None:
    lift = _lattice_lift()
    blended, differs = _blend()
    out = {
        "python": list(sys.version_info[:3]),
        "info_builtin_sum_differs": differs,
        "blend": blended,
        "sqrt_sorted": _sqrt_sorted(),
        **_float_sums(),
    }
    for name, faces in CLIP_CASES.items():
        out[f"clip_{name}"] = _clip_record(lift, faces)
    print(json.dumps(out, sort_keys=True))


if __name__ == "__main__":
    main()
