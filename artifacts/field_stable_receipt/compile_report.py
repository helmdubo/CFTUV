import json, os

OUT = "/tmp/claude-0/-home-user-CFTUV/4255c1b9-7873-5749-a1f5-eb79eb899ce6/scratchpad/field_stable"

def load(name):
    return json.load(open(os.path.join(OUT, name), encoding="utf-8"))

def wall_row(vertex, snapshot_label, path, alpha_label):
    d = load(path)
    r = d["domains"][0]
    c = r["counters"]
    return {
        "vertex": vertex,
        "snapshot": snapshot_label,
        "alpha": d["alpha"],
        "alpha_label": alpha_label,
        "density": d["density_raw"],
        "preparation_outcome": r["preparation_outcome"],
        "coverage_outcome": r["coverage_outcome"],
        "detail": r["detail"],
        "CONVEYOR_SKELETON_NODES": c.get("CONVEYOR_SKELETON_NODES"),
        "CONVEYOR_FACES": c.get("CONVEYOR_FACES"),
        "domain_faces_len": r["faces"],
        "prepare_ms": r["prepare_ms"],
        "coverage_ms": r["coverage_ms"],
        "contour_ms": r["contour_ms"],
        "total_ms": r["total_ms"],
        "lattice_scale": r["lattice_scale"],
        "CONVEYOR_ARRIVAL_LAWS": c.get("CONVEYOR_ARRIVAL_LAWS"),
        "CONVEYOR_RATIONAL_VERTEX_FANS": c.get("CONVEYOR_RATIONAL_VERTEX_FANS"),
        "CONVEYOR_FAN_SUPPORTS": c.get("CONVEYOR_FAN_SUPPORTS"),
        "CONVEYOR_AMBIGUOUS_OWNER_EDGES": c.get("CONVEYOR_AMBIGUOUS_OWNER_EDGES"),
        "bundle_seconds": d["bundle_seconds"],
    }

wall_2_001_rows = [
    wall_row("6ce0227", "wall_2_001", "6ce0227_wall2001_a0254_d0.json", "field(0.254)"),
    wall_row("6ce0227", "wall_2_001", "6ce0227_wall2001_a045_d0.json", "harness_default(0.45)"),
    wall_row("1dbf712", "wall_2_001", "1dbf712_wall2001_a0254_d0.json", "field(0.254)"),
    wall_row("1dbf712", "wall_2_001", "1dbf712_wall2001_a045_d0.json", "harness_default(0.45)"),
]

walls_012_rows = [
    wall_row("6ce0227", "walls_012", "6ce0227_walls012_a0254_d0.json", "field(0.254)"),
    wall_row("1dbf712", "walls_012", "1dbf712_walls012_a0254_d0.json", "field(0.254)"),
]

def native_row(vertex, path):
    d = load(path)
    return {
        "vertex": vertex,
        "snapshot": "walls_012",
        "alpha": 0.45,
        "alpha_label": "harness_default(0.45)_native_density_sweep.py",
        "density": d["density"],
        "preparation_outcome": d["preparation_outcome"],
        "coverage_outcome": d["coverage_outcome"],
        "detail": d["detail"],
        "CONVEYOR_SKELETON_NODES": d["counters"].get("CONVEYOR_SKELETON_NODES"),
        "CONVEYOR_FACES": d["counters"].get("CONVEYOR_FACES"),
        "domain_faces_len": d["faces"],
        "prepare_ms": d["prepare_ms"],
        "coverage_ms": d["coverage_ms"],
        "contour_ms": d["contour_ms"],
        "total_ms": d["total_ms"],
        "lattice_scale": d["lattice_scale"],
        "CONVEYOR_ARRIVAL_LAWS": d["counters"].get("CONVEYOR_ARRIVAL_LAWS"),
        "CONVEYOR_RATIONAL_VERTEX_FANS": d["counters"].get("CONVEYOR_RATIONAL_VERTEX_FANS"),
        "CONVEYOR_FAN_SUPPORTS": d["counters"].get("CONVEYOR_FAN_SUPPORTS"),
    }

walls_012_rows.append(native_row("6ce0227", "6ce0227_walls012_native_density_sweep_a045_d0.json"))
walls_012_rows.append(native_row("1dbf712", "1dbf712_walls012_native_density_sweep_a045_d0.json"))

def parse_building_log(vertex, patch, timeout_s=300):
    fname = f"{vertex}_building_patch{patch}_d0.log"
    path = os.path.join(OUT, fname)
    text = open(path, encoding="utf-8").read()
    lines = [l for l in text.splitlines() if l.strip().startswith("{")]
    record = None
    for l in lines:
        try:
            obj = json.loads(l)
            if obj.get("patch_id") == patch:
                record = obj
        except json.JSONDecodeError:
            continue
    timed_out = record is None
    row = {
        "vertex": vertex,
        "patch_id": patch,
        "timed_out": timed_out,
        "timeout_budget_s": timeout_s,
        "raw_log_file": fname,
    }
    if record is not None:
        row.update({
            "faces_boundary": record.get("faces"),
            "boundary_edges": record.get("boundary_edges"),
            "out_of_plane_m": record.get("out_of_plane_m"),
            "host_export_s": record.get("host_export_s"),
            "queue_s": record.get("queue_s"),
            "prepare_s": record.get("prepare_s"),
            "coverage_s": record.get("coverage_s"),
            "contour_s": record.get("contour_s"),
            "outcome": record.get("outcome"),
            "coverage_outcome": record.get("coverage_outcome"),
            "detail": record.get("detail"),
            "counters": record.get("counters"),
        })
    else:
        row["note"] = "did not complete within timeout budget (process killed by `timeout`)"
    return row

building_rows = []
for vertex in ("6ce0227", "1dbf712"):
    for patch in (89, 109, 121):
        building_rows.append(parse_building_log(vertex, patch))

report = {
    "task": "FIELD-STABLE-CANDIDATE",
    "commits": {
        "candidate": "6ce0227302f28b992a4c558d48227a5e511cea6d",
        "candidate_short": "6ce0227",
        "control_rc": "1dbf7128ce67b7cdfd74cf1dff09e9d874a9ade8",
        "control_rc_short": "1dbf712",
    },
    "alpha_note": (
        "Owner-stated alpha=0.254 for the 2.001 field failure could NOT be "
        "corroborated inside the repo's own artifacts (walls_001_door_queue_profile.txt "
        "contains no alpha field; no other file in the tree ties 0.254 to object 2.001). "
        "Per instructions, ran wall_2_001 and walls_012 at alpha=0.254 as the primary "
        "measurement, AND cross-checked both at the harness's own hardcoded default "
        "alpha=0.45 (run_domain.py ALPHA constant / density_sweep.py). Outcome "
        "(EXACT vs FACES_DID_NOT_ASSEMBLE / SKELETON_DID_NOT_CLOSE) is IDENTICAL at "
        "both alpha values on both commits -- alpha is not the variable that flips the "
        "verdict; the commit is."
    ),
    "density_note": "Fan Density = 0 (density=0 kwarg), as requested; supported natively by the harness.",
    "wall_2_001": wall_2_001_rows,
    "walls_012": walls_012_rows,
    "building_full_three_patches": building_rows,
    "building_full_note": (
        "big_scene.py hardcodes alpha=0.45 for its envelope decal request "
        "(not exposed as a CLI parameter) -- used as-is per reuse instruction. "
        "Each of the 3 patches (89, 109, 121 -- the same three the perf diagnostician "
        "measured, per REPORT.txt) was run in its OWN process via "
        "`timeout 300 python3 big_scene.py 1 0 300 <patch>` so a hang on one patch "
        "cannot block measurement of the others."
    ),
}

with open(os.path.join(OUT, "comparison_report.json"), "w", encoding="utf-8") as f:
    json.dump(report, f, ensure_ascii=False, indent=2)

print("wrote comparison_report.json")
print(json.dumps(report, ensure_ascii=False, indent=2)[:2000])
