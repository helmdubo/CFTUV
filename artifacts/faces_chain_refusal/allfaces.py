"""Все рёбра региона: где face_chain отказывает (не только первое) и что даёт
прототип «Гамильтонов путь по графу смежности» (ТОЛЬКО скрипт, продукт не тронут)."""
from __future__ import annotations
import sys, collections, itertools
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon
from diag17c import stage  # noqa


def places_of(nodes, owner):
    places = {}
    for node in nodes:
        places.setdefault((node.point.x.terms, node.point.y.terms), []).append(node)
    order = list(places)
    partners = [{p for node in places[pl] for p in node.participants if p != owner} for pl in order]
    return places, order, partners


def hamiltonian_paths(partners, previous, following, cap=50):
    n = len(partners)
    shared = collections.defaultdict(list)
    for i, g in enumerate(partners):
        for p in g:
            shared[p].append(i)
    links = {i: set() for i in range(n)}
    for seats in shared.values():
        for a, b in itertools.combinations(seats, 2):
            links[a].add(b); links[b].add(a)
    heads = [i for i, g in enumerate(partners) if following in g]
    tails = {i for i, g in enumerate(partners) if previous in g}
    found = []
    def dfs(path, seen):
        if len(found) >= cap: return
        if len(path) == n:
            if path[-1] in tails: found.append(tuple(path))
            return
        for nx in sorted(links[path[-1]] - seen):
            path.append(nx); seen.add(nx)
            dfs(path, seen)
            path.pop(); seen.discard(nx)
    for h in heads:
        dfs([h], {h})
    return found


def main():
    patch_id = int(sys.argv[1]); dens = [int(x) for x in sys.argv[2].split(",")]
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront import faces as F
    from cftuv_envelope.wavefront.events import EventKind
    from cftuv_envelope.wavefront.superlevel import validate_multiway_node
    snap, mk = stage(patch_id)
    for d in dens:
        canon.reset_factorization_memory()
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
        nodes_by_key = {}
        for node in sk.nodes:
            if node.kind is EventKind.MULTIWAY:
                _, incidences = validate_multiway_node(node)
            else:
                incidences = (node.participants,)
            for inc in incidences:
                for key in inc:
                    nodes_by_key.setdefault(key, []).append(node)
        nb = F.edge_neighbours(poly)
        print(f"=== d{d}: skeleton nodes {len(sk.nodes)}")
        bad = 0
        for key, s, e, line in F.polygon_fronts(poly):
            if line.is_stationary: continue
            cands = nodes_by_key.get(key, ())
            prev, foll = nb[key]
            chain, why = F.face_chain(key, tuple(cands), prev, foll)
            if chain is None:
                bad += 1
                places, order, partners = places_of(tuple(cands), key)
                paths = hamiltonian_paths(partners, prev, foll)
                shared = collections.defaultdict(list)
                for i, g in enumerate(partners):
                    for p in g: shared[p].append(i)
                crowd = {label.get(p): len(v) for p, v in shared.items() if len(v) > 2}
                print(f" FAIL {label[key]} why={why[:60]!r} places={len(places)} crowded={crowd} hamiltonian_paths={len(paths)}")
        print(" failing faces:", bad)


if __name__ == "__main__":
    main()
