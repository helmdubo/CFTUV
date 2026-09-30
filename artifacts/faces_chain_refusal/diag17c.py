"""Места грани упавшего ребра: точка, время, вторые участники, граф смежности."""
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


def fl(v):
    lo, hi = v.enclosure(64)
    return float(lo + hi) / 2


def main():
    patch_id = int(sys.argv[1]); d = int(sys.argv[2])
    ekey = tuple(int(x) for x in sys.argv[3].split(","))
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront import faces as F
    snap, mk = stage(patch_id)
    canon.reset_factorization_memory()
    prepared = prepare_conveyor(snap, mk(d))
    r = prepared.regions[0]
    poly = r.bridge.polygon
    label = {}
    for li, loop in enumerate(poly.loops):
        pts = loop.points
        for i in range(len(pts)):
            label[F.edge_key(pts[i], pts[(i + 1) % len(pts)])] = f"L{li}e{i}"
    for li, loop in enumerate(poly.loops):
        for i, p in enumerate(loop.points):
            fan = poly.fan_at(p)
            if fan:
                for o in range(1, len(fan.supports) + 1):
                    label[F.fan_edge_key(p, o)] = f"L{li}v{i}F{o}"
    print("label of e:", label[ekey], "neigh:", [label[k] for k in F.edge_neighbours(poly)[ekey]])
    sc = 262144.0
    nodes = []
    for n in r.skeleton.nodes:
        inc = n.incidences if n.kind.value == "MULTIWAY" else (n.participants,)
        if any(ekey in i for i in inc):
            nodes.append(n)
    rows = []
    for n in nodes:
        t = float((n.time.dividend)) / fl(n.time.divisor)
        rows.append((t, n))
    rows.sort(key=lambda z: z[0])
    places = collections.OrderedDict()
    for t, n in rows:
        key = (n.point.x.terms, n.point.y.terms)
        places.setdefault(key, []).append((t, n))
    print("places", len(places))
    for idx, (key, lst) in enumerate(places.items()):
        n0 = lst[0][1]
        partners = sorted({label.get(p, str(p)) for _, n in lst for p in n.participants if p != ekey})
        print(f" P{idx}: pt=({fl(n0.point.x)/sc:.5f},{fl(n0.point.y)/sc:.5f}) t={lst[0][0]/sc:.5f}  kinds={[n.kind.value+('/'+'+'.join(k.value for k in n.kinds) if n.kinds else '') for _,n in lst]} partners={partners}")
    print("ALL skeleton nodes in time order:")
    allrows = sorted(((float(n.time.dividend)/fl(n.time.divisor), n) for n in r.skeleton.nodes), key=lambda z: z[0])
    for t, n in allrows:
        print(f"  t={t/sc:.5f} {n.kind.value:8s} pt=({fl(n.point.x)/sc:.5f},{fl(n.point.y)/sc:.5f}) {[label.get(p,str(p)) for p in n.participants]} conv={n.converging_vertices}")


if __name__ == "__main__":
    main()
