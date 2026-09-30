"""ПРОТОТИП (только скрипт): перебор Гамильтоновых путей на упавших гранях, отбор
по объявленным границам (простой контур, положительная площадь), затем сумма по
полигону. Продукт не тронут."""
from __future__ import annotations
import sys, itertools
import env  # noqa: F401
from cftuv_envelope import exact_sqrt_sum as canon
from diag17c import stage
from allfaces import places_of, hamiltonian_paths


def main():
    patch_id = int(sys.argv[1]); dens = [int(x) for x in sys.argv[2].split(",")]
    from cftuv_envelope.wavefront import prepare_conveyor
    from cftuv_envelope.wavefront import faces as F
    from cftuv_envelope.wavefront.events import EventKind
    from cftuv_envelope.wavefront.superlevel import validate_multiway_node
    from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1
    snap, mk = stage(patch_id)
    for d in dens:
        canon.reset_factorization_memory()
        prepared = prepare_conveyor(snap, mk(d))
        r = prepared.regions[0]; poly = r.bridge.polygon; sk = r.skeleton
        nodes_by_key = {}
        for node in sk.nodes:
            if node.kind is EventKind.MULTIWAY:
                _, incs = validate_multiway_node(node)
            else:
                incs = (node.participants,)
            for inc in incs:
                for key in inc:
                    nodes_by_key.setdefault(key, []).append(node)
        nb = F.edge_neighbours(poly)
        fronts = F.polygon_fronts(poly)
        # варианты на каждую грань: список (points, doubled_area)
        per_face = []
        for key, s, e, line in fronts:
            if line.is_stationary: continue
            cands = tuple(nodes_by_key[key]); prev, foll = nb[key]
            chain, why = F.face_chain(key, cands, prev, foll)
            variants = []
            if chain is not None:
                paths = [None]; chains = [chain]
            else:
                places, order, partners = places_of(cands, key)
                paths = hamiltonian_paths(partners, prev, foll)
                chains = [tuple(n for i in p for n in places[order[i]]) for p in paths]
            for ch in chains:
                pts = ((SqrtSumV1.rational(s[0]), SqrtSumV1.rational(s[1])),
                       (SqrtSumV1.rational(e[0]), SqrtSumV1.rational(e[1]))) + tuple((n.point.x, n.point.y) for n in ch)
                cross = F.contour_crossings(pts)
                area = F.doubled_shoelace(pts)
                variants.append((F.FaceV1(key, s, e, pts, area, line), len(cross), area.sign()))
            per_face.append((key, chain is None, variants))
        print(f"=== d{d}")
        pick = []
        for key, ambiguous, variants in per_face:
            if ambiguous:
                print(" ambiguous face", key[:4], "variants (crossings, sign):", [(c, sgn) for _, c, sgn in variants])
            good = [v for v in variants if v[1] == 0 and v[2] > 0]
            pick.append((key, ambiguous, good))
        combos = 1
        for key, amb, good in pick: combos *= max(1, len(good))
        print(" simple+positive variants per ambiguous face:", [len(g) for k, a, g in pick if a], "combos", combos)
        # собрать разбиение: по первой годной на грань, проверить три границы
        faces = []; total = SqrtSumV1.zero()
        ok = True
        for key, amb, good in pick:
            if not good: ok = False; print(" no valid variant for", key[:4]); continue
            f = good[0][0]; faces.append(f); total = total + f.doubled_area
        if ok:
            poly_area = sum(F.signed_double_area(l.points) for l in poly.loops)
            part = F.FacePartitionV1(F.FaceOutcome.EXACT, tuple(faces), total, poly_area)
            res = F.check_declared_boundaries(part)
            print(" partition outcome with first-good variants:", res.outcome, res.detail[:200])
            # все комбинации среди неоднозначных
            amb_idx = [i for i, (k, a, g) in enumerate(pick) if a]
            results = []
            for combo in itertools.product(*[range(len(pick[i][2])) for i in amb_idx]):
                fs = [p[2][0][0] for p in pick]
                tot = SqrtSumV1.zero()
                for j, (k, a, g) in enumerate(pick):
                    f = g[combo[amb_idx.index(j)]][0] if a else g[0][0]
                    fs[j] = f; tot = tot + f.doubled_area
                rr = F.check_declared_boundaries(F.FacePartitionV1(F.FaceOutcome.EXACT, tuple(fs), tot, poly_area))
                results.append((combo, rr.outcome.value))
            print(" all combos:", results)


if __name__ == "__main__":
    main()
