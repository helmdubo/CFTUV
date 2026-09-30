"""Какая доля sympy-арифметики планарного пути — ЧИСТО РАЦИОНАЛЬНАЯ (кандидат в Fraction).

    python rational_share.py <patch> <density> <out.json>

Обёртки на `ExactPlanarMetric.dot_g`, `oriented_cross`, `point_sub`/`point_add`/`vector_scale`
и `boundary._contact_candidates` считают вызовы, у которых ВСЕ входные выражения — `sp.Rational`
(их результат — тоже `Rational`, то есть его `srepr` — `Integer(n)`/`Rational(p, q)`, и замена на
`Fraction` + одну конструкцию `sp.Rational` не меняет ни одной строки идентичности), против вызовов
с радикалами/углами. Файлы ядра не правятся.
"""
from __future__ import annotations

import collections
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gate  # noqa: E402
import pool_sweep  # noqa: E402


def main():
    patch, density, out = int(sys.argv[1]), int(sys.argv[2]), sys.argv[3]
    pool_sweep.init_worker(quiet=True)
    gate.compute_row(100, density)
    import sympy as sp
    from cftuv_envelope.reference import metric as metric_mod
    from cftuv_envelope.reference import planar_types as pt
    from cftuv_envelope.reference import boundary as boundary_mod

    stats = collections.defaultdict(lambda: collections.Counter())

    def classify(exprs):
        if all(e.is_Rational for e in exprs):
            return "all-Rational"
        if any(e.has(sp.pi, sp.sin, sp.cos, sp.atan) for e in exprs):
            return "has-angle"
        return "has-surd"

    def wrap_vec2(name, cls, method):
        original = getattr(cls, method)

        def wrapper(self, left, right, *a, **k):
            try:
                exprs = list(left.expressions()) + list(right.expressions())
                stats[name][classify(exprs)] += 1
            except Exception:
                stats[name]["unclassifiable"] += 1
            return original(self, left, right, *a, **k)

        setattr(cls, method, wrapper)

    wrap_vec2("ExactPlanarMetric.dot_g", metric_mod.ExactPlanarMetric, "dot_g")
    wrap_vec2("ExactPlanarMetric.oriented_cross", metric_mod.ExactPlanarMetric, "oriented_cross")

    def wrap_fn(name, module, fname, getter):
        original = getattr(module, fname)

        def wrapper(*args, **kwargs):
            try:
                stats[name][classify(getter(*args, **kwargs))] += 1
            except Exception:
                stats[name]["unclassifiable"] += 1
            return original(*args, **kwargs)

        for mname, mod in list(sys.modules.items()):
            if mname.startswith("cftuv_envelope") and mod is not None:
                if getattr(mod, fname, None) is original:
                    setattr(mod, fname, wrapper)

    wrap_fn("point_sub", pt, "point_sub", lambda l, r: list(l.expressions()) + list(r.expressions()))
    wrap_fn("point_add", pt, "point_add", lambda p, v: list(p.expressions()) + list(v.expressions()))
    wrap_fn(
        "boundary._contact_candidates",
        boundary_mod,
        "_contact_candidates",
        lambda ctx, source, boundary: (
            list(source.start.expressions()) + list(source.end.expressions())
            + list(source.tangent.expressions()) + list(source.owner_normal.expressions())
            + list(boundary.segment.start.expressions()) + list(boundary.segment.end.expressions())
        ),
    )
    started = time.perf_counter()
    row = gate.compute_row(patch, density)
    wall = time.perf_counter() - started
    record = {
        "patch": patch, "density": density, "wall_seconds": round(wall, 1),
        "outcome": row["answer"]["outcome"],
        "by_function": {k: dict(v) for k, v in stats.items()},
    }
    Path(out).write_text(json.dumps(record, ensure_ascii=False, indent=1), encoding="utf-8")
    print(json.dumps(record["by_function"]))


if __name__ == "__main__":
    main()
