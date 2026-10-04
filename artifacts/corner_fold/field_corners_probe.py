"""Кнопка «Build Decal Mesh» на одном меше в фоновом Blender при СОХРАНЁННОЙ политике сцены (alpha, Fan Density, Max stretch
не трогаются), с по-угловой записью решений обработки вогнутых углов (`--capture`: старое решение — `_corner_treatment.decide`,
новое — записанная обработка). Сцена не сохраняется. Исполняется только из дерева, где есть `CornerFoldFacts`.

    blender -b E:/testscene.blend --python-exit-code 1 --python artifacts/corner_fold/field_corners_probe.py --         --root <дерево> --mesh <имя> --out <json> [--capture] [--workers 0]

Результат — JSON: политика, число граней и рёбер меша, дайджест, по доменам исход, секунды, счётчики вееров и потоков, а с `--capture` —
по каждому углу: патч, вершина, одна ли цепь, изгиб δ (градусы), складка кольца-1 (`sin^2` и градусы), обработка до и после.
"""

import argparse
import json
import math
import sys
import time
from fractions import Fraction
from pathlib import Path

import bpy

PROBE = str(Path(__file__).resolve().parents[1] / "production_sweep" / "production_probe.py")
tail = sys.argv[sys.argv.index("--") + 1 :]
parser = argparse.ArgumentParser()
parser.add_argument("--root", required=True)
parser.add_argument("--mesh", required=True)
parser.add_argument("--out", required=True)
parser.add_argument("--capture", action="store_true")
parser.add_argument("--workers", type=int, default=0)
args = parser.parse_args(tail)

source_text = open(PROBE, encoding="utf-8").read().rsplit("\nmain()", 1)[0]
ns = {"__name__": "probe_namespace", "__file__": PROBE}
exec(compile(source_text, PROBE, "exec"), ns)
root = Path(args.root).resolve()
ns["_load_tree"](root)
ns["_install_capture"]()

CORNERS = []
if args.capture:
    import cftuv_envelope._canonical_angle as canon
    import cftuv_envelope._corner_treatment as law
    import cftuv_envelope.reference.compile as compiled

    original = compiled.resolve_corner_selection

    def wrapper(request, relation, sector, angle_certificate, selection_id, uses_by_id, chains_by_id, resolve_profile, fold):
        result = original(request, relation, sector, angle_certificate, selection_id, uses_by_id, chains_by_id, resolve_profile, fold)
        resolved, record, failure = result
        if record is not None:
            measure = angle_certificate.measure_payload
            old = law.decide(sector, measure, uses_by_id, chains_by_id)
            interval, restoration = canon.selector_reflex_excess_interval(measure.reflex_excess_over_pi)
            sin2 = fold.sin2_at(relation.source_vertex_id, sector.owner_patch_id)
            CORNERS.append(
                {
                    "patch": sector.owner_patch_id.value.rsplit(":", 1)[-1],
                    "vertex": relation.source_vertex_id.value.rsplit(":", 1)[-1],
                    "same_chain": bool(old[2]),
                    "delta_lower_deg": float(Fraction(interval.lower)) * 180.0,
                    "delta_upper_deg": float(Fraction(interval.upper)) * 180.0,
                    "restored": restoration is not None,
                    "sin2": None if sin2 is None else float(sin2),
                    "dihedral_deg": None if sin2 is None else math.degrees(math.asin(math.sqrt(float(sin2)))),
                    "old": [old[0].value, old[1].value],
                    "new": [record.treatment.value, record.reason.value],
                    "k_old_selection": None if resolved is None else resolved.hidden_count,
                }
            )
        return result

    compiled.resolve_corner_selection = wrapper

source = bpy.data.objects[args.mesh]
settings = bpy.context.scene.hotspotuv_settings
settings.envelope_debug_engine = "QUEUE"
settings.envelope_debug_workers = args.workers
policy = {
    "alpha": settings.envelope_debug_alpha,
    "density": settings.envelope_debug_fan_density,
    "max_stretch": settings.envelope_debug_max_stretch,
    "workers": settings.envelope_debug_workers,
}
ns["_reset_session"]()
started = time.perf_counter()
step = ns["_press_production"](source)
wall = time.perf_counter() - started
run = ns["CAPTURED"]["run"]
decal = bpy.data.objects.get("%s.CFTUV_Decal" % args.mesh) or next(
    (item for item in bpy.data.objects if item.name.startswith(args.mesh + ".CFTUV_Decal")), None
)
edges = faces = vertices = seams = None
if decal is not None:
    mesh = decal.data
    edges, faces, vertices = len(mesh.edges), len(mesh.polygons), len(mesh.vertices)
    seams = sum(1 for edge in mesh.edges if edge.use_seam)
result = {
    "mesh": args.mesh,
    "root": str(root),
    "policy": policy,
    "wall_seconds": round(wall, 3),
    "faces": faces,
    "edges": edges,
    "vertices": vertices,
    "seam_edges": seams,
    "mesh_digest": step["object"].get("mesh_digest"),
    "outcomes": step["run"].get("outcomes"),
    "domains": {
        str(item.patch_id): {
            "outcome": item.outcome,
            "seconds": round(item.seconds, 4),
            "content_digest": item.content_digest,
            "faces": None if item.batch is None else len(item.batch.faces),
            "diagnostics": [line.split(":", 1)[0] for line in item.diagnostics],
            "counters": {
                name: value
                for name, value in item.counters
                if any(token in name for token in ("FAN", "JOIN", "MITER", "FLOW", "QUADS", "POLYGON_FACES", "REGIONS", "FACES_EMITTED"))
            },
        }
        for item in run.results
    },
    "corners": CORNERS,
    "stage_totals": {k: round(v, 3) for k, v in run.profile.stage_totals.items()},
    "domain_stage_seconds": {
        str(domain): {stage: round(sec, 3) for stage, sec in stages.items()}
        for domain, stages in [
            (d, {t.stage: t.elapsed_seconds for t in run.profile.timings if t.patch_domain_id == d})
            for d in sorted({t.patch_domain_id for t in run.profile.timings if t.patch_domain_id})
        ]
    },
    "conveyor_counters": {
        str(domain): {c.name: c.value for c in run.profile.counters if c.patch_domain_id == domain and ("FAN" in c.name or "MITER" in c.name)}
        for domain in sorted({c.patch_domain_id for c in run.profile.counters if c.patch_domain_id})
    },
}
Path(args.out).parent.mkdir(parents=True, exist_ok=True)
Path(args.out).write_text(json.dumps(result, ensure_ascii=False, indent=1, sort_keys=True) + "\n", encoding="utf-8")
print("PROBE_DONE", args.mesh, policy, "faces", faces, "edges", edges, "wall", round(wall, 2))
