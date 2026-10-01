"""Реальное (не cProfile) ИСКЛЮЧИТЕЛЬНОЕ время по функциям ядра на одном домене.

    PYTHONSAFEPATH=1 PYTHONPATH=kernel/src python artifacts/numeric_repr/exclusive_time.py <patch> <density> [модуль:функция ...] [--json out.json]

Функции из списка оборачиваются (подмена имени во всех модулях `cftuv_envelope`), стек вычитает
время вложенных обёрток из внешней; домен прогревается малым (patch 100). Накладные расходы
обёрток — доли процента от секунд домена (в отличие от cProfile, который раздувает вызовы).
"""
import sys, time, importlib
from pathlib import Path
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gate, pool_sweep

patch, density = int(sys.argv[1]), int(sys.argv[2])
TARGETS = """
wavefront.candidate_law:evaluate_split_candidate
wavefront.exact_candidate_view:span_containment
wavefront.exact_candidate_view:position
wavefront.exact_candidate_view:_hydrate_position
wavefront.exact_candidate_view:span_end
wavefront.exact_candidate_view:_span_bound
wavefront.event_time:_event_point
wavefront.event_time:concurrency_time
wavefront.event_time:sliding_time
wavefront.event_time:sliding_point
wavefront.event_time:compare_times
wavefront.event_time:times_are_equal
wavefront.event_time:event_point
exact_sqrt_sum:_divide_with_prime_universe
exact_sqrt_sum:squarefree_split
wavefront.symbolic_component:overlay_signature
wavefront.symbolic_component:clone_overlay
wavefront.symbolic_superlevel_coordinator:discover_interior_split_contacts
wavefront.symbolic_superlevel_coordinator:_initial_interior_closure
wavefront.symbolic_mixed_generation:plan_mixed_generations
wavefront.symbolic_junction_contacts:discover_junction_contacts
wavefront.symbolic_split_endpoint:discover_endpoint_contacts
wavefront.superlevel_snapshot:collect_superlevel_snapshot
wavefront.symbolic_runtime_commit:materialize_symbolic_runtime_commit
wavefront.symbolic_runtime_commit:plan_symbolic_runtime_commit
wavefront.symbolic_f0_overlay:build_f0_overlay
wavefront.symbolic_overlay:build_symbolic_overlay
wavefront.superlevel_closure:plan_split_materialization
wavefront.skeleton:build_skeleton
reference.boundary:_contact_candidates
reference.boundary:resolve_component_alphas
reference.compile:compile_reference_envelopes
wavefront.conveyor:conveyor_coverage
wavefront.conveyor:prepare_conveyor
""".split()
out_json = None
args = sys.argv[3:]
if "--json" in args:
    i = args.index("--json"); out_json = args[i+1]; args = args[:i] + args[i+2:]
extra = args
TARGETS += extra

pool_sweep.init_worker(quiet=True)
gate.compute_row(100, density)
incl = {}
excl = {}
cnt = {}
stack = []


def wrap(name, fn):
    def w(*a, **k):
        t0 = time.perf_counter()
        stack.append(0.0)
        try:
            return fn(*a, **k)
        finally:
            dt = time.perf_counter() - t0
            child = stack.pop()
            cnt[name] = cnt.get(name, 0) + 1
            excl[name] = excl.get(name, 0.0) + dt - child
            if stack:
                stack[-1] += dt
            # inclusive only when not recursive handled loosely
            incl[name] = incl.get(name, 0.0) + dt
    w.__wrapped__ = fn
    return w


for tg in TARGETS:
    mod, fname = tg.split(":")
    try:
        m = importlib.import_module("cftuv_envelope." + mod)
        orig = getattr(m, fname)
    except Exception as e:
        print("skip", tg, e, file=sys.stderr)
        continue
    neww = wrap(tg, orig)
    for mm in list(sys.modules.values()):
        if getattr(mm, "__name__", "").startswith("cftuv_envelope"):
            d = getattr(mm, "__dict__", {})
            for k, v in list(d.items()):
                if v is orig:
                    setattr(mm, k, neww)
t = time.perf_counter()
row = gate.compute_row(patch, density)
wall = time.perf_counter() - t
print(f"patch{patch} d{density} wall={wall:.2f}")
accounted = 0.0
for k, s in sorted(excl.items(), key=lambda kv: -kv[1]):
    print(f"  excl {s:7.2f}s {100*s/wall:5.1f}%  incl {incl[k]:7.2f}s  n={cnt[k]:>8}  {k}")
if out_json:
    import json
    json.dump({"patch": patch, "density": density, "wall_seconds": round(wall, 3),
               "rows": [{"function": k, "exclusive_seconds": round(excl[k], 3), "inclusive_seconds": round(incl[k], 3),
                         "calls": cnt[k], "exclusive_share": round(excl[k] / wall, 4)}
                        for k in sorted(excl, key=lambda k: -excl[k])]},
              open(out_json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
