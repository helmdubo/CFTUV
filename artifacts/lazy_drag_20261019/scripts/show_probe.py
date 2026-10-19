import json
import sys

r = json.load(open(sys.argv[1]))
for w, row in r["widths"].items():
    print("==", w, row.get("faces"), row.get("vertices"), "in_faces", row.get("input_faces"), "in_verts", row.get("input_vertices"), "out", row.get("output_faces"))
    print(" counters", row.get("counters"))
    print(" rings", row.get("rings"))
    print(" interval", row.get("interval"))
    for e in row.get("log", []):
        print("  ", json.dumps(e)[:600])
