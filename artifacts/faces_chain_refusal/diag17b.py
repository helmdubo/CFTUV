from __future__ import annotations
import sys, collections
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon


def stage(patch_id):
    import big_scene
    from cftuv.envelope_request_export import (
        _typed_value, build_envelope_analysis_snapshot, build_envelope_decal_request)
    from cftuv.envelope_topology_export import stage_domain_inputs
    _, bundle, selected, _ = big_scene.survey()
    _, revision, patch_ids, request_id, by_domain = stage_domain_inputs(bundle, selected)
    domain_id = _typed_value("patch-domain", revision, patch_id)
    snap = build_envelope_analysis_snapshot(bundle, included_patch_ids=frozenset({patch_id}))
    return snap, (lambda dd: build_envelope_decal_request(
        snap, frozenset(by_domain[domain_id]), 0.45, decal_request_id_value=request_id, density=dd))


def f(x):
    try:
        return round(float(x.as_float() if hasattr(x, "as_float") else x), 3)
    except Exception:
        return str(x)[:30]


def short(k):
    return k[:2] + ("->",) + k[2:4] + ((("#%d" % k[4]),) if len(k) > 4 else ())


def main():
    patch_id = int(sys.argv[1]); d = int(sys.argv[2]); ekey = tuple(int(x) for x in sys.argv[3].split(","))
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront import faces as F
    snap, mk = stage(patch_id)
    canon.reset_factorization_memory()
    prepared = prepare_conveyor(snap, mk(d))
    r = prepared.regions[0]
    poly = r.bridge.polygon
    print("LOOPS")
    for i, loop in enumerate(poly.loops):
        print(" loop", i, list(loop.points), "src", list(loop.source_flags))
    print("FANS")
    for fan in poly.vertex_fans:
        print(" fan", fan.point, "supports", len(fan.supports), [getattr(s, "__dict__", s) if False else s for s in fan.supports])
    print("FRONTS")
    for key, s, e, line in F.polygon_fronts(poly):
        print(" ", key, "stationary" if line.is_stationary else "", "q=", line.q)
    nb = F.edge_neighbours(poly)
    print("NEIGH of e", nb.get(ekey))
    for n in r.skeleton.nodes:
        inc = n.incidences if n.kind.value == "MULTIWAY" else (n.participants,)
        hit = any(ekey in i for i in inc)
        if hit:
            print("NODE", n.kind.value, "pt=(%s, %s)" % (f(n.point.x), f(n.point.y)), "t=", f(n.time) if hasattr(n.time, "as_float") else "", "conv", n.converging_vertices)
            print("   participants", [short(p) for p in n.participants])
            if n.kind.value == "MULTIWAY":
                for i in n.incidences:
                    print("   incidence", [short(p) for p in i])
                print("   kinds", [k.value for k in n.kinds])


if __name__ == "__main__":
    main()
