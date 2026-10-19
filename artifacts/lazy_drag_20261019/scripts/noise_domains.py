import json
import sys

for path in sys.argv[1:]:
    r = json.load(open(path))
    tied = disjoint_ok = 0
    total = 0
    for d in r["domains"]:
        if "skip" in d:
            continue
        total += 1
        keys = d["log0"].get("edge_order") or []
        groups, current = [], [keys[0]] if keys else []
        for a, b in zip(keys, keys[1:]):
            if b[0] - a[0] < 1e-12:
                current.append(b)
            else:
                groups.append(current)
                current = [b]
        if current:
            groups.append(current)
        noisy = [g for g in groups if len(g) > 1]
        if not noisy:
            continue
        tied += 1
        # disjointness witness proxy: the tied edges share no endpoint vertex (faces of distinct edges may still share; proxy only)
        ok = all(len({v for e in g for v in e[1:]}) == 2 * len(g) for g in noisy)
        disjoint_ok += ok
    print(path, "domains", total, "with_noise_tied_edge_keys", tied, "of_which_tied_edges_vertex_disjoint", disjoint_ok)
