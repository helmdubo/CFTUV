from __future__ import annotations
import sys
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon
from diag17c import stage, fl
from allfaces import places_of, hamiltonian_paths


def main():
    patch_id = int(sys.argv[1]); d = int(sys.argv[2])
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront import faces as F
    from cftuv_envelope.wavefront.events import EventKind
    from cftuv_envelope.wavefront.superlevel import validate_multiway_node
    snap, mk = stage(patch_id)
    prepared = prepare_conveyor(snap, mk(d))
    r = prepared.regions[0]; poly = r.bridge.polygon; sk = r.skeleton
    label = {}
    for li, loop in enumerate(poly.loops):
        pts = loop.points
        for i in range(len(pts)):
            label[F.edge_key(pts[i], pts[(i + 1) % len(pts)])] = f"L{li}e{i}"
        for i, p in enumerate(pts):
            fan = poly.fan_at(p)
            if fan:
                for o in range(1, len(fan.supports) + 1):
                    label[F.fan_edge_key(p, o)] = f"L{li}v{i}F{o}"
    nbk = {}
    for node in sk.nodes:
        incs = validate_multiway_node(node)[1] if node.kind is EventKind.MULTIWAY else (node.participants,)
        for inc in incs:
            for key in inc:
                nbk.setdefault(key, []).append(node)
    nb = F.edge_neighbours(poly)
    sc = 262144.0
    for key in [(673740, 137852, 0, 262144), (0, 262144, 262144, 0)]:
        prev, foll = nb[key]
        places, order, partners = places_of(tuple(nbk[key]), key)
        paths = hamiltonian_paths(partners, prev, foll)
        print("FACE", label[key], "prev", label[prev], "next", label[foll])
        for i, pl in enumerate(order):
            n0 = places[pl][0]
            print(f"  P{i} ({fl(n0.point.x)/sc:.3f},{fl(n0.point.y)/sc:.3f}) partners={sorted(label[p] for p in partners[i])}")
        for p in paths:
            print("  PATH", " ".join(f"P{i}" for i in p))


if __name__ == "__main__":
    main()
