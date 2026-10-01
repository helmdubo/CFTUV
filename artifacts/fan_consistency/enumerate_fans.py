"""Перечень ВСЕХ веерных углов очереди: что решили селектор, восстановление, лифт и решётка.

Вход — экспорт сцены (`export_domains.py`, Blender headless), выход — JSON по мешу и плотности
в `RESULTS` (scratchpad) и сводка в stdout. Ядро и хост берутся из ЭТОГО дерева; продукт не правится.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/enumerate_fans.py <mesh> <density> [workers] [patch,patch]

На каждую AngularEnvelopeSpec домена пишется запись:
  сырой угол хоста (интервал доли пи), восстановление (есть/нет), точный угол в СЫРОЙ карте (после
  привязки источника) и в EVALUATION-геометрии (решётка карты), решение селектора (C, H), итоговый H
  спеки, закон лифта, число граней веера в разбиении и в покрытии при alpha 0.45.
"""
from __future__ import annotations

import json
import math
import sys
import time
from fractions import Fraction
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

import fanlib  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.wavefront import conveyor_coverage, prepare_conveyor  # noqa: E402

ALPHA_TEXT = "0.45"


def _sqrt_float(value) -> float:
    return sum(float(c) * math.sqrt(m) for m, c in value.terms)


def _fan_nodes(prep) -> dict[str, tuple[int, int]]:
    """Имя спеки -> узел решётки веера (из ключей владения скрытых опор)."""

    nodes: dict[str, tuple[int, int]] = {}
    for region in prep.regions:
        for key, name in region.owner_by_edge:
            if len(key) == 5:
                nodes[name] = (key[0], key[1])
    return nodes


def _fan_faces(prep, coverage):
    partition: dict[tuple[int, int], list] = {}
    for region in prep.regions:
        if region.partition is None:
            continue
        for face in region.partition.faces:
            if len(face.owner) == 5:
                partition.setdefault((face.owner[0], face.owner[1]), []).append(
                    (face.owner[4], _sqrt_float(face.doubled_area))
                )
    covered: dict[str, list] = {}
    for face in coverage.faces:
        if len(face.owner) == 5:
            covered.setdefault(face.envelope_spec_id, []).append(
                (face.owner[4], _sqrt_float(face.doubled_area))
            )
    return partition, covered


def domain_records(mesh: str, density: int, patch: str) -> dict:
    base = _paths.mesh_dir(mesh) / f"patch_{patch}"
    snap = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
    req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
    started = time.perf_counter()
    prep = prepare_conveyor(snap, req)
    row: dict = {
        "mesh": mesh,
        "patch": int(patch),
        "density": density,
        "outcome": prep.outcome.value,
        "detail": (prep.detail or "")[:200],
        "seconds": round(time.perf_counter() - started, 2),
        "counters": {
            k: v for k, v in prep.counters if "FAN" in k or "MITER" in k or "LIFT" in k
        },
        "corners": [],
    }
    if prep.compilation is None or prep.outcome.value != "EXACT":
        return row
    coverage = conveyor_coverage(prep, ALPHA_TEXT)
    row["coverage_outcome"] = coverage.outcome.value
    view = fanlib.DomainView(snap, prep)
    nodes = _fan_nodes(prep)
    part_faces, cov_faces = _fan_faces(prep, coverage)
    scale = prep.lattice.scale if prep.lattice is not None else 1
    degraded = {c.envelope_spec_id: c.reason for r in prep.regions for c in r.degraded_miter_corners}
    for spec in view.specs:
        name = spec.envelope_spec_id.value
        relation = view.relations[spec.source_relation_id]
        iv = view.certs[relation.reflex_angle_certificate_id].measure_payload.reflex_excess_over_pi
        sel = view.selections[spec.selection_certificate_id]
        restoration = view.restorations.get(spec.selection_certificate_id)
        vertex, prev, nxt = view.corner_neighbours(spec)
        lift = getattr(spec, "evaluation_subturn_count_lift", None)
        authority = any(
            a.envelope_spec_id == spec.envelope_spec_id
            for a in view.comp.canonical_subturn_fan_authorities
        )
        s_dot, _s_c2, _, _, s_ang = view.turn(view.source_xy, vertex, prev, nxt)
        e_dot, _e_c2, _, _, e_ang = view.turn(view.eval_xy, vertex, prev, nxt)
        lo, hi = Fraction(iv.lower), Fraction(iv.upper)
        mid = (lo + hi) / 2
        node = nodes.get(name)
        ex, ey = view.eval_xy[vertex]
        node_matches_eval = (
            node is not None and Fraction(node[0]) == ex * scale and Fraction(node[1]) == ey * scale
        )
        pf = sorted(part_faces.get(node, [])) if node is not None else []
        cf = sorted(cov_faces.get(name, []))
        row["corners"].append(
            {
                "vertex": fanlib.vertex_index(vertex),
                "prev": fanlib.vertex_index(prev),
                "next": fanlib.vertex_index(nxt),
                "raw_lo": str(iv.lower),
                "raw_hi": str(iv.upper),
                "raw_exact_half": lo == hi == Fraction(1, 2),
                "raw_dev_deg": float(mid - Fraction(1, 2)) * 180.0,
                "restored": restoration is not None,
                "restoration_dev_rad": (
                    None
                    if restoration is None
                    else restoration.deviation_upper_bound_radians.numerator
                    / restoration.deviation_upper_bound_radians.denominator
                ),
                "src_dot0": s_dot == 0,
                "src_turn_deg": s_ang,
                "eval_dot0": e_dot == 0,
                "eval_turn_deg": e_ang,
                "sel_C": getattr(sel.selection_interval_certificate, "bucket_c", None),
                "sel_H": sel.resolved_hidden_edge_count,
                "spec_H": spec.resolved_hidden_edge_count,
                "lift_law": None if lift is None else lift.lift_law.name,
                "lift_sign": None if lift is None else lift.evaluation_turn_sign.name,
                "canonical_authority": authority,
                "degraded": degraded.get(name),
                "node_matches_eval": node_matches_eval,
                "partition_fan_faces": len(pf),
                "coverage_fan_faces": len(cf),
                "coverage_fan_ordinals": [o for o, _ in cf],
                "coverage_fan_areas": [round(a / (scale * scale), 9) for _, a in cf],
            }
        )
    return row


def _task(args):
    try:
        return domain_records(*args)
    except Exception as exc:  # noqa: BLE001 - диагностика: отказ домена тоже факт
        return {
            "mesh": args[0],
            "patch": int(args[2]),
            "density": args[1],
            "outcome": f"EXCEPTION:{type(exc).__name__}",
            "detail": str(exc)[:200],
            "corners": [],
        }


def main() -> None:
    mesh, density = sys.argv[1], int(sys.argv[2])
    workers = int(sys.argv[3]) if len(sys.argv) > 3 else 6
    manifest = json.loads((_paths.mesh_dir(mesh) / "manifest.json").read_text(encoding="utf-8"))
    patches = sorted(manifest["domains"], key=int)
    if len(sys.argv) > 4:
        wanted = sys.argv[4].split(",")
        patches = [p for p in patches if p in wanted]
    jobs = [(mesh, density, p) for p in patches]
    started = time.perf_counter()
    if workers <= 1:
        rows = [_task(job) for job in jobs]
    else:
        with Pool(workers) as pool:
            rows = pool.map(_task, jobs, chunksize=1)
    _paths.RESULTS.mkdir(parents=True, exist_ok=True)
    out = _paths.RESULTS / f"fans_{mesh.replace('.', '_')}_d{density}.json"
    out.write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
    corners = sum(len(r["corners"]) for r in rows)
    outcomes: dict[str, int] = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    print(f"{mesh} d{density}: domains={len(rows)} corners={corners} outcomes={outcomes} "
          f"seconds={time.perf_counter() - started:.1f} -> {out}")


if __name__ == "__main__":
    main()
