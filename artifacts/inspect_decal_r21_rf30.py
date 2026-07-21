"""R2.1/RF30: labeled mutual-arrival probe on ``sagging_wall``.

Research harness only.  It never saves the source blend and reports regions
where a segment crop reaches another compile-static Voronoi cell before that
cell's owner material reaches the same region.
"""

from __future__ import annotations

from dataclasses import replace
from hashlib import sha256
import json
import os
from pathlib import Path
import sys

import bmesh
import bpy


SCRIPT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(os.environ.get("CFTUV_SOURCE_ROOT", SCRIPT_ROOT)).resolve()
OUTPUT_JSON = Path(
    os.environ.get(
        "CFTUV_RF30_INSPECT",
        SCRIPT_ROOT / "artifacts" / "decal_r21_rf30_inspect.json",
    )
)
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

loaded_cftuv = sys.modules.get("cftuv")
if loaded_cftuv is not None:
    loaded_path = Path(getattr(loaded_cftuv, "__file__", "")).resolve()
    if REPO_ROOT not in loaded_path.parents:
        for module_name in tuple(sys.modules):
            if module_name == "cftuv" or module_name.startswith("cftuv."):
                del sys.modules[module_name]

from cftuv.analysis import build_analysis_bundle  # noqa: E402
from cftuv.decals import compile_manual_seam_decal_plan  # noqa: E402
from cftuv.decal_voronoi import (  # noqa: E402
    _clip_to_convex,
    _polygon_area2,
    _segment_crop_polygon,
    _subtract_convex_polygon,
)
from cftuv.model import CornerJoinMode, DecalSettings  # noqa: E402


def _source_graph(obj, selected_edges):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bm.verts.ensure_lookup_table()
    bm.edges.ensure_lookup_table()
    bm.faces.ensure_lookup_table()
    try:
        return build_analysis_bundle(
            bm, [face.index for face in bm.faces], obj
        ), tuple(sorted(int(value) for value in selected_edges))
    finally:
        bm.free()


def _positive_area(polygons):
    return sum(abs(_polygon_area2(polygon)) * 0.5 for polygon in polygons)


def _one_sided_records(surface, alpha):
    records = []
    for atom_index, atom in enumerate(surface.atoms):
        primary = surface.sites[atom.site_index]
        primary_crop = _segment_crop_polygon(primary, alpha)
        for competitor_index, competitor in enumerate(surface.sites):
            if competitor_index == atom.site_index:
                continue
            competitor_crop = _segment_crop_polygon(competitor, alpha)
            area = 0.0
            witnesses = []
            for fragment in atom.fragments:
                reached = _clip_to_convex(fragment, competitor_crop)
                if len(reached) < 3:
                    continue
                outside_primary = _subtract_convex_polygon(
                    reached, primary_crop
                )
                fragment_area = _positive_area(outside_primary)
                if fragment_area <= 1e-9:
                    continue
                area += fragment_area
                witnesses.extend(
                    tuple(tuple(float(value) for value in point) for point in piece)
                    for piece in outside_primary
                )
            if area <= 1e-9:
                continue
            records.append(
                {
                    "surface": int(surface.patch_id),
                    "domain": str(surface.domain.kind),
                    "atom": int(atom_index),
                    "cell_kind": str(atom.cell_kind),
                    "primary_site": int(atom.site_index),
                    "primary_edge": int(primary.edge_index),
                    "competitor_site": int(competitor_index),
                    "competitor_edge": int(competitor.edge_index),
                    "area": float(area),
                    "witnesses": witnesses[:4],
                }
            )
    return sorted(
        records,
        key=lambda record: (
            -record["area"],
            record["surface"],
            record["primary_edge"],
            record["competitor_edge"],
        ),
    )


def main():
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    obj = bpy.data.objects["sagging_wall"]
    graph, selected = _source_graph(obj, (3, 4, 5, 6, 7, 8, 9, 11))
    settings = DecalSettings(
        width_seam=0.8,
        offset=0.01,
        corner_join_mode=CornerJoinMode.BEVEL,
    )
    plan = compile_manual_seam_decal_plan(
        graph, settings, selected, alpha_budget=7.8
    )
    surfaces = tuple(
        surface
        for partition in plan.backend_partitions
        for surface in getattr(partition.compiled_plan, "surfaces", ())
    )
    report = {
        "schema": "cftuv.decal_rf30_inspect.v2",
        "source_commit": os.environ.get("CFTUV_SOURCE_COMMIT", "WORKTREE"),
        "source_root": str(REPO_ROOT),
        "blend_file": str(Path(bpy.data.filepath)),
        "blend_sha256": sha256(Path(bpy.data.filepath).read_bytes()).hexdigest(),
        "blend_saved": False,
        "object": obj.name,
        "selected": list(selected),
        "join": settings.corner_join_mode.value,
        "surfaces": [
            {
                "patch": int(surface.patch_id),
                "domain": str(surface.domain.kind),
                "sites": [
                    {
                        "site": index,
                        "edge": int(site.edge_index),
                        "vertices": [int(site.vert_a), int(site.vert_b)],
                    }
                    for index, site in enumerate(surface.sites)
                ],
                "corners": [
                    {
                        "corner": index,
                        "vertex": int(corner.vert_index),
                        "incident_sites": list(corner.incident_sites),
                    }
                    for index, corner in enumerate(surface.corners)
                ],
                "atoms": [
                    {
                        "atom": index,
                        "site": int(atom.site_index),
                        "kind": str(atom.cell_kind),
                        "corner": int(atom.corner_index),
                    }
                    for index, atom in enumerate(surface.atoms)
                ],
                "corner_releases": [
                    {
                        "corner": int(corner_index),
                        "point_site": int(release.point_site_index),
                        "owner_site": int(release.owner_site_index),
                        "owner_edge": int(
                            surface.sites[release.owner_site_index].edge_index
                        ),
                        "fragment_count": len(release.fragments),
                    }
                    for corner_index, releases in sorted(
                        surface.corner_release_atoms.items()
                    )
                    for release in releases
                ],
                "mutual_arrivals": [
                    {
                        "corner": int(corner_index),
                        "point_site": int(atom.point_site_index),
                        "point_edge": int(
                            surface.sites[atom.point_site_index].edge_index
                        ),
                        "blocking_site": int(atom.blocking_site_index),
                        "blocking_edge": int(
                            surface.sites[atom.blocking_site_index].edge_index
                        ),
                        "fragment_count": len(atom.fragments),
                    }
                    for corner_index, atoms in sorted(
                        getattr(surface, "mutual_arrival_atoms", {}).items()
                    )
                    for atom in atoms
                ],
            }
            for surface in surfaces
        ],
        "widths": [],
    }
    for alpha in (0.2, 0.4, 0.8, 1.6, 3.2, 7.8):
        records = [
            record
            for surface in surfaces
            for record in _one_sided_records(surface, alpha)
        ]
        report["widths"].append(
            {
                "alpha": alpha,
                "record_count": len(records),
                "total_area": sum(record["area"] for record in records),
                "records": records[:40],
            }
        )
    OUTPUT_JSON.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("CFTUV_RF30_INSPECT=" + json.dumps(report, ensure_ascii=False))


if __name__ == "__main__":
    main()
