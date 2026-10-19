import json
import sys

r = json.load(open(sys.argv[1]))
shown = 0
for d in r["domains"]:
    if "skip" in d:
        continue
    o0 = d["log0"].get("edge_order") or []
    o1 = d["log1"].get("edge_order") or []
    if [k[1:] for k in o0] != [k[1:] for k in o1]:
        print(d["patch"], "tau", d["tau"], "same_structure", d["structure_same"], "w", d["w"])
        k1 = {tuple(k[1:]): k[0] for k in o1}
        for k in o0[:8]:
            merged0 = (d["log0"].get("merges") or {}).get(f"{k[1]}|{k[2]}", {}).get("merged")
            print("   %.3e -> %.3e merged=%s" % (k[0], k1.get(tuple(k[1:]), float("nan")), merged0))
        shown += 1
        if shown >= 4:
            break
