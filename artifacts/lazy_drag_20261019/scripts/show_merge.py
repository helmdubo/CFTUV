import json
import sys

r = json.load(open(sys.argv[1]))
for w, row in r["widths"].items():
    for e in row.get("log", []):
        if e["kind"] == "MERGE":
            print(w, e["edge"][0], "depth", e.get("depth"), "tolerated", e.get("tolerated"), "delta", e.get("tally_delta"))
