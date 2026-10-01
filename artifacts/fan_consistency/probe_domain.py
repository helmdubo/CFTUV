"""Зонд одного домена: что селектор / лифт / решётка решили по каждому веерному углу.

  PYTHONSAFEPATH=1 python artifacts/fan_consistency/probe_domain.py <mesh> <density> <patch>
"""
from __future__ import annotations

import sys
from fractions import Fraction
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import _paths  # noqa: E402

_paths.add_kernel_paths()

import fanlib  # noqa: E402
from cftuv_envelope import AnalysisSnapshotCodecV1, DecalRequestCodecV1  # noqa: E402
from cftuv_envelope.wavefront import prepare_conveyor  # noqa: E402


def main() -> None:
    mesh, density, patch = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    base = _paths.mesh_dir(mesh) / f"patch_{patch}"
    snap = AnalysisSnapshotCodecV1.loads((base / "analysis_snapshot.json").read_bytes())
    req = DecalRequestCodecV1.loads((base / f"decal_request_d{density}.json").read_bytes())
    prep = prepare_conveyor(snap, req)
    view = fanlib.DomainView(snap, prep)
    print(prep.outcome, prep.lattice, "gram", view.gram)
    print("binding", type(view.binding).__name__, view.binding.binding_law.name)
    g = view.frame.grid_certificate
    print("grid_cert", g.snapping_law.name, "source_scale", g.source_scale, "step", g.window_step,
          "intended", g.intended_right_corners, "restored", g.restored_right_corners)
    for spec in view.specs:
        relation = view.relations[spec.source_relation_id]
        iv = view.certs[relation.reflex_angle_certificate_id].measure_payload.reflex_excess_over_pi
        sel = view.selections[spec.selection_certificate_id]
        vertex, prev, nxt = view.corner_neighbours(spec)
        lift = getattr(spec, "evaluation_subturn_count_lift", None)
        s_dot, s_c2, _, _, s_ang = view.turn(view.source_xy, vertex, prev, nxt)
        e_dot, e_c2, _, _, e_ang = view.turn(view.eval_xy, vertex, prev, nxt)
        print(
            f"v{fanlib.vertex_index(vertex):5d} prev={fanlib.vertex_index(prev)} next={fanlib.vertex_index(nxt)} "
            f"raw_iv=[{iv.lower},{iv.upper}] selH={sel.resolved_hidden_edge_count} specH={spec.resolved_hidden_edge_count} "
            f"lift={lift.lift_law.name[-20:] if lift else None} "
            f"SRC dot0={s_dot == 0} ang={s_ang:+.6f} | EVAL dot0={e_dot == 0} ang={e_ang:+.6f}"
        )
        if "-v" in sys.argv:
            for name, coords in (("src", view.source_xy), ("eval", view.eval_xy)):
                print("     ", name, [(fanlib.vertex_index(x), tuple(map(str, coords[x]))) for x in (prev, vertex, nxt)])


main()
