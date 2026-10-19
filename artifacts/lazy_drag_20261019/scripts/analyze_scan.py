"""Reduce a silhouette scan: flips between the two widths, linear crossing estimates of every tolerance decision, order-key gaps."""
import json
import sys
from collections import Counter

CHORD = 0.005
r = json.load(open(sys.argv[1]))
counts = Counter()
near = {0.02: set(), 0.05: set(), 0.10: set(), 0.25: set()}
nearest_share = []
flips = []
order_gaps = []
noise_ties = Counter()
for d in r["domains"]:
    if "skip" in d:
        counts["SKIP_" + d["skip"]] += 1
        continue
    counts["DOMAINS"] += 1
    if not d["structure_same"]:
        counts["STRUCTURE_CHANGED_AT_DELTA"] += 1
    w0, w1 = d["w"]
    tau = d.get("tau")
    l0, l1 = d["log0"], d["log1"]
    best = None

    def judge(kind, q0, q1, limit):
        global best
        if q0 is None or q1 is None or limit is None:
            return
        counts["DECISIONS_" + kind] += 1
        m0, m1 = limit - q0, limit - q1
        if (m0 > 0) != (m1 > 0):
            flips.append((d["patch"], kind, q0, q1, limit))
        if m1 == m0:
            return
        wc = w0 + (w1 - w0) * m0 / (m0 - m1)
        share = (wc - w0) / w0
        lo, hi = d["low"], d["high"]
        if (hi is not None and wc >= hi) or wc <= (lo or 0):
            return
        for k in near:
            if abs(share) <= k:
                near[k].add(d["patch"])
        return abs(share)

    shares = []
    for key, e0 in (l0.get("merges") or {}).items():
        e1 = (l1.get("merges") or {}).get(key)
        if not e1:
            counts["MERGE_MISSING_AT_W1"] += 1
            continue
        shares.append(judge("MERGE_CHORD", e0.get("depth"), e1.get("depth"), CHORD))
        shares.append(judge("MERGE_UV_FIT", e0.get("residual"), e1.get("residual"), tau))
    for key, e0 in (l0.get("vertices") or {}).items():
        e1 = (l1.get("vertices") or {}).get(key)
        if not e1:
            counts["VERTEX_MISSING_AT_W1"] += 1
            continue
        shares.append(judge("VERTEX_CHORD", e0.get("chord"), e1.get("chord"), CHORD))
        if not e0.get("planned"):
            shares.append(judge("VERTEX_UV_SLIDE", e0.get("slide"), e1.get("slide"), tau))
    shares = [s for s in shares if s is not None]
    if shares:
        nearest_share.append(min(shares))
    # greedy edge order: relative gap of adjacent float keys; ties at noise level
    keys = [k[0] for k in l0.get("edge_order") or []]
    for a, b in zip(keys, keys[1:]):
        if a == b:
            noise_ties["EDGE_KEY_EXACT_TIE"] += 1
        elif b - a < 1e-12:
            noise_ties["EDGE_KEY_GAP_BELOW_1e-12_RAD"] += 1
    order0 = [tuple(k[1:]) for k in l0.get("edge_order") or []]
    order1 = [tuple(k[1:]) for k in l1.get("edge_order") or []]
    if order0 != order1:
        counts["EDGE_ORDER_CHANGED_AT_DELTA"] += 1
nearest_share.sort()
print(json.dumps({"counts": dict(counts), "noise": dict(noise_ties), "flips_between_the_two_widths": flips[:20], "n_flips": len(flips),
                  "domains_with_silhouette_crossing_estimate_within": {f"{int(k*100)}%": len(v) for k, v in near.items()},
                  "nearest_crossing_share_quantiles": [nearest_share[int(q * (len(nearest_share) - 1))] for q in (0, 0.01, 0.05, 0.1, 0.5)] if nearest_share else None,
                  "domains_with_any_tolerance_decision": len(nearest_share)}, indent=1, default=str))
