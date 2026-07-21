"""R2 + S-CM.b Blender gate без сохранения исходного ``.blend``.

Один headless job проверяет канонические шесть объектов, RF7/RF17 receipts,
публичные MITER/BEVEL preview/confirm, post-competition ResolvedCornerView и
S-DF2-lite oracle над финальными arrangement faces. В полном режиме тот же
процесс строит два обязательных wireframe-ракурса RF28.
"""

from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

import bpy


REPO_ROOT = Path(__file__).resolve().parents[1]
ARTIFACT_ROOT = REPO_ROOT / "artifacts"
TESTS_ROOT = REPO_ROOT / "tests"
for path in (REPO_ROOT, ARTIFACT_ROOT, TESTS_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

from cftuv import decal_voronoi  # noqa: E402
from cftuv.decal_rails import compile_decal_rail_plan  # noqa: E402
from cftuv.decal_distance_witness import (  # noqa: E402
    CornerPolygonField2D,
    MaterializedCornerPolygon2D,
)
from cftuv.decal_transform import (  # noqa: E402
    local_decal_settings_for_source,
)
from cftuv.decal_voronoi import serialize_network_faces  # noqa: E402
from cftuv.decals import (  # noqa: E402
    compile_manual_seam_decal_plan,
    evaluate_manual_seam_faces,
)
from cftuv.model import CornerJoinMode, DecalSettings  # noqa: E402

import render_decal_r18_rc5 as field_render  # noqa: E402
import verify_decal_r13_rr9a as field  # noqa: E402
from decal_rail_fixtures import rr10_rf7_opposing_cylinder  # noqa: E402


OUTPUT_JSON = ARTIFACT_ROOT / "decal_r2_s_cm_b_field_acceptance.json"
RF28_CASES = {
    "walls.006": tuple(range(15)),
    "sagging_wall": (6, 7, 8),
}
RF28_WIDTHS = (0.8, 3.2)
FIELD_WIDTHS = (0.4, 0.8)


def _arguments():
    raw = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Только два RF28-кейса, без канонических шести и render.",
    )
    parser.add_argument("--output", default=str(OUTPUT_JSON))
    return parser.parse_args(raw)


def _digest(faces):
    snapshot = serialize_network_faces(faces)
    payload = json.dumps(
        snapshot,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return {
        "sha256": sha256(payload).hexdigest(),
        "topology_signature": snapshot["topology_signature"],
    }


def _component_counts(faces):
    return dict(
        sorted(Counter(str(face.component_kind) for face in faces).items())
    )


def _is_strip_segment(face):
    """Отделяет ленту от corner-local fallback с kind=SEGMENT."""

    if str(face.component_kind) != "SEGMENT":
        return False
    owner_id = getattr(face.provenance, "semantic_owner_id", None)
    return not (
        isinstance(owner_id, tuple)
        and owner_id[:1] in {("corner",), ("corner-model",)}
    )


def _segment_digest(faces):
    return _digest(
        tuple(face for face in faces if _is_strip_segment(face))
    )["sha256"]


def _segment_edge_digests(faces):
    surface_edges = sorted(
        {
            (int(face.surface_id), int(edge_id))
            for face in faces
            if _is_strip_segment(face)
            for edge_id in getattr(face.provenance, "source_edge_ids", ())
        }
    )
    return {
        f"{surface_id}:{edge_id}": _digest(
            tuple(
                face
                for face in faces
                if _is_strip_segment(face)
                and int(face.surface_id) == surface_id
                and edge_id
                in getattr(face.provenance, "source_edge_ids", ())
            )
        )["sha256"]
        for surface_id, edge_id in surface_edges
    }


def _segment_edge_counts(faces):
    counts = Counter()
    for face in faces:
        if not _is_strip_segment(face):
            continue
        for edge_id in getattr(face.provenance, "source_edge_ids", ()):
            counts[f"{int(face.surface_id)}:{int(edge_id)}"] += 1
    return dict(sorted(counts.items()))


def _corner_local_segment_count(faces):
    return sum(
        1
        for face in faces
        if str(face.component_kind) == "SEGMENT"
        and not _is_strip_segment(face)
    )


def _freeze_record(locus):
    station_key = (
        ("VERTEX", int(locus.source_vertex_id))
        if locus.source_vertex_id is not None
        else (
            "EDGE",
            int(locus.source_edge_id),
            float(locus.edge_parameter),
        )
    )
    return {
        "kind": locus.competition_kind.value,
        "route_ids": list(locus.route_ids),
        "arrival_distances": [float(value) for value in locus.arrival_distances],
        "chain_refs": [list(value) for value in locus.chain_refs],
        "owner_chain_ref": repr(locus.owner_chain_ref),
        "station_key": repr(station_key),
        "canonical_distance": float(locus.canonical_distance),
        "source_vertex_id": locus.source_vertex_id,
        "source_edge_id": locus.source_edge_id,
        "edge_parameter": locus.edge_parameter,
    }


def _rf7_rf17_receipt():
    """Live-Blender receipt: одна нить, два чтения, immutable freeze."""

    graph, _edge_ids, selected, _vertex_at = rr10_rf7_opposing_cylinder()
    narrow = compile_decal_rail_plan(
        graph,
        selected,
        alpha_budget=3.0,
    )
    wide = compile_decal_rail_plan(
        graph,
        tuple(reversed(selected)),
        alpha_budget=30.0,
    )
    return {
        "route_count": len(narrow.routes),
        "route_reading_count": len(narrow.route_readings),
        "freeze_locus_count": len(narrow.freeze_loci),
        "event_counts": [list(value) for value in narrow.event_counts],
        "all_routes_pchain": all(
            route.termination.value == "PCHAIN" for route in narrow.routes
        ),
        "two_readings_per_route": all(
            sum(
                reading.route_id == route.route_id
                for reading in narrow.route_readings
            )
            == 2
            for route in narrow.routes
        ),
        "compile_static_across_drag_and_enumeration": (
            wide.routes == narrow.routes
            and wide.route_readings == narrow.route_readings
            and wide.freeze_loci == narrow.freeze_loci
        ),
        "freeze_loci": [
            _freeze_record(locus) for locus in narrow.freeze_loci
        ],
    }


def _rail_receipt(plan):
    rail_plan = plan.rail_plan
    if rail_plan is None:
        return {
            "route_reading_count": 0,
            "freeze_locus_count": 0,
            "freeze_loci": [],
        }
    return {
        "route_reading_count": len(rail_plan.route_readings),
        "freeze_locus_count": len(rail_plan.freeze_loci),
        "freeze_loci": [
            _freeze_record(locus) for locus in rail_plan.freeze_loci
        ],
    }


def _capacity_receipt(plan):
    backends = []
    for partition in plan.backend_partitions:
        policy = getattr(partition.compiled_plan, "capacity_policy", None)
        backends.append(
            {
                "backend": partition.backend.value,
                "edges": list(partition.edge_indices),
                "capacity_policy": getattr(policy, "value", None),
            }
        )
    return {
        "backends": backends,
        "terminals": [
            {
                "vertex": terminal.spine_vertex_id,
                "edge": terminal.spine_edge_id,
                "choice": terminal.choice,
                "route_id": terminal.route_id,
                "capacity_policy": terminal.capacity_policy.value,
            }
            for terminal in plan.terminal_routing
        ],
    }


def _captured_evaluation(obj, settings, plan, *, preview):
    """Acceptance-only hook: читает arrangement, не меняя production route."""

    captured = []
    original = decal_voronoi._build_decal_arrangement

    def capture(*args, **kwargs):
        arrangement = original(*args, **kwargs)
        captured.append(
            (
                arrangement,
                tuple(kwargs.get("corner_resolution_sources", ())),
            )
        )
        return arrangement

    decal_voronoi._build_decal_arrangement = capture
    try:
        result = evaluate_manual_seam_faces(
            obj,
            settings,
            plan,
            preview=preview,
        )
    finally:
        decal_voronoi._build_decal_arrangement = original
    return result, tuple(captured)


def _oracle_receipt(captured, requested_half_width):
    records = []
    own_segment_keys = set()
    for arrangement, raw_sources in captured:
        sources = {
            (int(patch_id), owner_id, int(chart_id)): (model, derived)
            for patch_id, owner_id, model, derived, chart_id in raw_sources
        }
        source_keys = tuple(sorted(sources, key=repr))
        own_segment_keys.update(
            (int(source_key[0]), int(anchor.station_ref.source_edge_id))
            for source_key, (model, derived) in sources.items()
            if model.seed.join is CornerJoinMode.BEVEL
            and derived.releases_competitor
            for anchor in (model.p1, model.p2)
        )
        if len(source_keys) != len(arrangement.resolved_corner_views):
            raise RuntimeError(
                "S_CM_B_RESOLVED_VIEW_SOURCE_COUNT_MISMATCH: "
                f"sources={len(source_keys)} "
                f"views={len(arrangement.resolved_corner_views)}"
            )
        for source_key, view in zip(
            source_keys, arrangement.resolved_corner_views
        ):
            patch_id, owner_id, chart_id = source_key
            polygons = tuple(
                MaterializedCornerPolygon2D(
                    points=tuple(face.points),
                    owner_id=owner_id,
                )
                for face in arrangement.faces
                if int(face.surface.patch_id) == patch_id
                and face.crop.semantic_owner_id == owner_id
                and int(face.surface.domain.chart_id) == chart_id
            )
            if not polygons:
                continue
            model, derived = sources[source_key]
            oracle = CornerPolygonField2D.from_model(
                model,
                derived,
                requested_half_width=requested_half_width,
            )
            comparison = oracle.compare_materialized(polygons)
            leak_count = sum(
                sample.materialized_material
                and not sample.intended_material
                for sample in comparison.samples
            )
            contour_ids = tuple(
                vertex.semantic_id for vertex in derived.emission_boundary
            )
            p1_witness = oracle.query_source(
                view.p1.station_ref.site_id,
                view.p1.chart_point,
            )
            p2_witness = oracle.query_source(
                view.p2.station_ref.site_id,
                view.p2.chart_point,
            )
            records.append(
                {
                    "owner_id": repr(owner_id),
                    "patch_id": patch_id,
                    "chart_id": chart_id,
                    "join": model.seed.join.value,
                    "anchor_identity": (
                        view.p1 is model.p1 and view.p2 is model.p2
                    ),
                    "derived_identity": view.derived is derived,
                    "boundary_vertex_count": len(view.materialized_vertices),
                    "competition_vertex_count": len(
                        view.competition_vertices
                    ),
                    "polygon_count": len(polygons),
                    "sample_count": len(comparison.samples),
                    "matches": comparison.matches,
                    "safe_competition_subset": (
                        leak_count == 0
                        and not comparison.owner_mismatches
                        and (
                            model.seed.join is not CornerJoinMode.BEVEL
                            or len(contour_ids) == 3
                            and set(contour_ids) == {"V", "P1", "P2"}
                        )
                    ),
                    "contour_ids": list(contour_ids),
                    "leak_count": leak_count,
                    "coverage_mismatch_count": len(
                        comparison.coverage_mismatches
                    ),
                    "owner_mismatch_count": len(
                        comparison.owner_mismatches
                    ),
                    "width_error_count": len(comparison.width_errors),
                    "missing_anchor_ids": list(
                        comparison.missing_anchor_ids
                    ),
                    "p1_distance": float(p1_witness.distance),
                    "p2_distance": float(p2_witness.distance),
                    "p1_source_s": p1_witness.source_s,
                    "p2_source_s": p2_witness.source_s,
                    "side": oracle.side.value,
                }
            )
    return {
        "resolved_view_count": len(records),
        "all_anchor_identity": all(
            record["anchor_identity"] and record["derived_identity"]
            for record in records
        ),
        "all_matches": all(record["matches"] for record in records),
        "all_safe_competition_subsets": all(
            record["safe_competition_subset"] for record in records
        ),
        "competition_vertex_count": sum(
            record["competition_vertex_count"] for record in records
        ),
        "own_segment_keys": [list(key) for key in sorted(own_segment_keys)],
        "records": records,
    }


def _compile_case(obj, selected, join_mode, widths):
    graph, selected, _all_seams = field._source_graph(obj, selected)
    base = DecalSettings(
        width_seam=float(widths[0]),
        offset=0.01,
        corner_join_mode=CornerJoinMode(join_mode),
    )
    plan = compile_manual_seam_decal_plan(
        graph,
        base,
        selected,
        alpha_budget=max(float(value) for value in widths),
    )
    record = {
        "join": join_mode,
        "backend_summary": plan.backend_summary,
        "accounting_is_exact": plan.accounting_is_exact,
        "rail": _rail_receipt(plan),
        "capacity": _capacity_receipt(plan),
        "widths": [],
    }
    first_digest = None
    for width in widths:
        settings = replace(base, width_seam=float(width))
        preview, _preview_arrangements = _captured_evaluation(
            obj, settings, plan, preview=True
        )
        confirm, confirm_arrangements = _captured_evaluation(
            obj, settings, plan, preview=False
        )
        preview_digest = _digest(preview.faces)
        confirm_digest = _digest(confirm.faces)
        local = local_decal_settings_for_source(settings, obj)
        oracle = _oracle_receipt(
            confirm_arrangements,
            requested_half_width=float(local.width_seam) * 0.5,
        )
        width_record = {
            "width": float(width),
            "face_count": len(confirm.faces),
            "component_counts": _component_counts(confirm.faces),
            "corner_local_segment_count": _corner_local_segment_count(
                confirm.faces
            ),
            "segment_digest": _segment_digest(confirm.faces),
            "segment_edge_digests": _segment_edge_digests(confirm.faces),
            "segment_edge_counts": _segment_edge_counts(confirm.faces),
            "preview": preview_digest,
            "confirm": confirm_digest,
            "preview_confirm_equal": preview_digest == confirm_digest,
            "policy_counts": [list(item) for item in confirm.policy_counts],
            "oracle": oracle,
        }
        if first_digest is None:
            first_digest = preview_digest
        record["widths"].append(width_record)

    roundtrip_settings = replace(base, width_seam=float(widths[0]))
    roundtrip = evaluate_manual_seam_faces(
        obj, roundtrip_settings, plan, preview=True
    )
    record["roundtrip_first_width_bit_identical"] = (
        first_digest == _digest(roundtrip.faces)
    )
    return record


def _case_record(object_name, selected, widths, joins):
    obj = bpy.data.objects.get(object_name)
    if obj is None or obj.type != "MESH":
        return {"object": object_name, "status": "MISSING"}
    result = {
        "object": object_name,
        "selected": list(selected) if selected != "ALL_SEAMS" else "ALL_SEAMS",
        "joins": {},
    }
    for join_mode in joins:
        try:
            result["joins"][join_mode] = _compile_case(
                obj, selected, join_mode, widths
            )
        except Exception as exc:
            result["joins"][join_mode] = {
                "status": "ERROR",
                "error": repr(exc),
            }
    if {"MITER", "BEVEL"} <= set(result["joins"]):
        miter = result["joins"]["MITER"]
        bevel = result["joins"]["BEVEL"]
        if "widths" in miter and "widths" in bevel:
            result["rf28"] = []
            for miter_width, bevel_width in zip(
                miter["widths"], bevel["widths"]
            ):
                width = miter_width["width"]
                own_segment_keys = bevel_width["oracle"][
                    "own_segment_keys"
                ]
                result["rf28"].append(
                    {
                        "width": width,
                        "own_segment_keys": own_segment_keys,
                        "own_segments_equal": all(
                            miter_width["segment_edge_digests"].get(
                                f"{patch_id}:{edge_id}"
                            )
                            == bevel_width["segment_edge_digests"].get(
                                f"{patch_id}:{edge_id}"
                            )
                            for patch_id, edge_id in own_segment_keys
                        ),
                        "bevel_face_count": bevel_width[
                            "component_counts"
                        ].get("BEVEL", 0),
                        "bevel_oracle_safe": bevel_width["oracle"][
                            "all_safe_competition_subsets"
                        ],
                        "foreign_frontier_station_count": bevel_width[
                            "oracle"
                        ]["competition_vertex_count"],
                    }
                )
    return result


def _render_rf28_cases():
    outputs = []
    for object_name, selected in RF28_CASES.items():
        output = ARTIFACT_ROOT / (
            "decal_r2_s_cm_b_field_"
            + object_name.replace(".", "")
            + "_bevel_w32.png"
        )
        field_render.OBJECT_NAME = object_name
        field_render.WIDTH = 3.2
        field_render.JOIN_MODE = "BEVEL"
        field_render.EDGE_OVERRIDE = selected
        field_render.REPORT_LABEL = "R2 + S-CM.b / RF28"
        field_render.LEGEND = (
            "orange=spine | cyan=owned strips | magenta=CornerModel chord"
        )
        field_render.OUTPUT_PNG = output
        field_render.main()
        outputs.append(str(output.relative_to(REPO_ROOT)))
    return outputs


def _is_green(report):
    rf7 = report["rf7_rf17"]
    if not (
        rf7["route_count"] == 8
        and rf7["route_reading_count"] == 16
        and rf7["freeze_locus_count"] == 8
        and rf7["all_routes_pchain"]
        and rf7["two_readings_per_route"]
        and rf7["compile_static_across_drag_and_enumeration"]
    ):
        return False
    records = report["rf28_cases"] + report["field_objects"]
    saw_foreign_frontier_at_chord = False
    for case in records:
        if case.get("status") == "MISSING":
            return False
        for joined in case.get("joins", {}).values():
            if joined.get("status") == "ERROR":
                return False
            if not joined.get("accounting_is_exact", False):
                return False
            if not joined.get("roundtrip_first_width_bit_identical", False):
                return False
            for width in joined.get("widths", ()):
                if not width["preview_confirm_equal"]:
                    return False
                if not width["oracle"]["all_anchor_identity"]:
                    return False
                if not width["oracle"]["all_safe_competition_subsets"]:
                    return False
        for rf28 in case.get("rf28", ()):
            if not rf28["own_segments_equal"]:
                return False
            if rf28["width"] == 3.2 and rf28["bevel_face_count"] <= 0:
                return False
            if not rf28["bevel_oracle_safe"]:
                return False
            if (
                rf28["width"] == 3.2
                and rf28["foreign_frontier_station_count"] > 0
            ):
                saw_foreign_frontier_at_chord = True
    return saw_foreign_frontier_at_chord


def main():
    args = _arguments()
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    blend_path = Path(bpy.data.filepath)
    report = {
        "schema": "cftuv.decal_r2_s_cm_b.field.v1",
        "source_commit": os.environ.get("CFTUV_SOURCE_COMMIT", "UNKNOWN"),
        "source_root": str(REPO_ROOT),
        "blender_version": bpy.app.version_string,
        "python_version": sys.version.split()[0],
        "blend_file": str(blend_path),
        "blend_sha256": sha256(blend_path.read_bytes()).hexdigest(),
        "blend_saved": False,
        "decision": "A",
        "quick": bool(args.quick),
        "rf7_rf17": _rf7_rf17_receipt(),
        "rf28_cases": [],
        "field_objects": [],
        "renders": [],
    }
    for object_name, selected in RF28_CASES.items():
        report["rf28_cases"].append(
            _case_record(
                object_name,
                selected,
                RF28_WIDTHS,
                ("MITER", "BEVEL"),
            )
        )
    if not args.quick:
        for object_name, selected in field.FIELD_OBJECTS.items():
            report["field_objects"].append(
                _case_record(
                    object_name,
                    selected,
                    FIELD_WIDTHS,
                    ("MITER",),
                )
            )
        report["renders"] = _render_rf28_cases()
    report["green"] = _is_green(report)
    output_path = Path(args.output)
    output_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_R2_S_CM_B=" + json.dumps(report, ensure_ascii=False))
    if not report["green"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
