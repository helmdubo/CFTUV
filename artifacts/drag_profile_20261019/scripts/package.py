"""Copies the evidence of the drag profile from the scratch (D:/dragprof-scratch/out) into this folder, reduced where the raw file is large.

    python package.py D:/dragprof-scratch/out <artifact dir>
"""
import json
import shutil
import sys
from pathlib import Path

src, dst = Path(sys.argv[1]), Path(sys.argv[2])
for name in ("building_r1.json", "cover008_r1.json", "building_r1_summary.json", "cover008_r1_summary.json", "building_tickprof.json", "cover008_tickprof.json", "treeab_presses_summary.json"):
    shutil.copyfile(src / name, dst / name)
for name, out in (("building_lin.json", "building_linearity.json"), ("cover008_lin2.json", "cover008_linearity.json"), ("cover008_focus.json", "cover008_focus_change_inside_interval.json")):
    data = json.loads((src / name).read_text(encoding="utf-8"))
    reduced = []
    for row in data.pop("per_domain", []):
        samples = row.pop("samples", None) or []
        flagged = [s for s in samples if s["structure_parts_changed"] or s["differences"] or s["float_slopes_equal"] is False]
        row["samples_judged"] = len(samples)
        if flagged:
            row["samples_flagged"] = flagged
        row.pop("interval", None) if not flagged else None
        reduced.append(row)
    data["per_domain_reduced"] = reduced if name != "cover008_focus.json" else []
    (dst / out).write_text(json.dumps(data, indent=1, default=str), encoding="utf-8")
print("packaged")
