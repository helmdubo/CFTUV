from __future__ import annotations

from types import MappingProxyType

try:
    from .constants import CORNER_ANGLE_THRESHOLD_DEG
    from .model import ChainNeighborKind, FrameRole, LoopKind, PatchType, WorldFacing
    from .analysis_records import (
        _LoopFrameRunBuildResult,
        _LoopDerivedTopologySummary,
        _PatchDerivedTopologySummary,
        _PatchGraphAggregateCounts,
        _PatchGraphDerivedTopology,
    )
    from .analysis_frame_runs import _build_patch_graph_loop_frame_results
    from .analysis_junctions import _build_junction_run_refs_by_corner, _build_patch_graph_junctions
except ImportError:
    from constants import CORNER_ANGLE_THRESHOLD_DEG
    from model import ChainNeighborKind, FrameRole, LoopKind, PatchType, WorldFacing
    from analysis_records import (
        _LoopFrameRunBuildResult,
        _LoopDerivedTopologySummary,
        _PatchDerivedTopologySummary,
        _PatchGraphAggregateCounts,
        _PatchGraphDerivedTopology,
    )
    from analysis_frame_runs import _build_patch_graph_loop_frame_results
    from analysis_junctions import _build_junction_run_refs_by_corner, _build_patch_graph_junctions


def _build_patch_topology_summaries(graph, loop_frame_results):
    """Build canonical per-patch and aggregate topology summaries."""

    patch_summaries: list[_PatchDerivedTopologySummary] = []

    walls = 0
    floors = 0
    slopes = 0
    singles = 0
    total_loops = 0
    total_chains = 0
    total_corners = 0
    total_sharp_corners = 0
    total_holes = 0
    total_h = 0
    total_v = 0
    total_free = 0
    total_up = 0
    total_down = 0
    total_side = 0
    total_patch_links = 0
    total_self_seams = 0
    total_mesh_borders = 0
    total_run_h = 0
    total_run_v = 0
    total_run_free = 0

    for patch_id in sorted(graph.nodes.keys()):
        node = graph.nodes[patch_id]
        patch_type = node.patch_type
        world_facing = node.world_facing

        if world_facing == WorldFacing.UP:
            total_up += 1
        elif world_facing == WorldFacing.DOWN:
            total_down += 1
        else:
            total_side += 1

        if patch_type == PatchType.WALL:
            walls += 1
        elif patch_type == PatchType.FLOOR:
            floors += 1
        elif patch_type == PatchType.SLOPE:
            slopes += 1

        if len(node.face_indices) == 1:
            singles += 1

        patch_chain_count = 0
        patch_corner_count = 0
        patch_hole_count = 0
        patch_run_count = 0
        patch_h = 0
        patch_v = 0
        patch_free = 0
        role_sequence: list[FrameRole] = []
        loop_kinds: list[LoopKind] = []
        loop_summaries: list[_LoopDerivedTopologySummary] = []

        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            loop_kind = boundary_loop.kind
            frame_runs = loop_frame_results.get(
                (patch_id, loop_index),
                _LoopFrameRunBuildResult(effective_roles=(), runs=()),
            ).runs

            total_loops += 1
            patch_chain_count += len(boundary_loop.chains)
            patch_corner_count += len(boundary_loop.corners)
            total_chains += len(boundary_loop.chains)
            total_corners += len(boundary_loop.corners)
            patch_run_count += len(frame_runs)
            loop_kinds.append(loop_kind)

            if loop_kind == LoopKind.HOLE:
                total_holes += 1
                patch_hole_count += 1

            total_sharp_corners += sum(
                1
                for corner in boundary_loop.corners
                if corner.turn_angle_deg >= CORNER_ANGLE_THRESHOLD_DEG
            )

            for chain in boundary_loop.chains:
                role_sequence.append(chain.frame_role)
                if chain.frame_role == FrameRole.H_FRAME:
                    total_h += 1
                    patch_h += 1
                elif chain.frame_role == FrameRole.V_FRAME:
                    total_v += 1
                    patch_v += 1
                else:
                    total_free += 1
                    patch_free += 1

                if chain.neighbor_kind == ChainNeighborKind.PATCH:
                    total_patch_links += 1
                elif chain.neighbor_kind == ChainNeighborKind.SEAM_SELF:
                    total_self_seams += 1
                else:
                    total_mesh_borders += 1

            for frame_run in frame_runs:
                if frame_run.dominant_role == FrameRole.H_FRAME:
                    total_run_h += 1
                elif frame_run.dominant_role == FrameRole.V_FRAME:
                    total_run_v += 1
                else:
                    total_run_free += 1

            loop_summaries.append(
                _LoopDerivedTopologySummary(
                    patch_id=patch_id,
                    loop_index=loop_index,
                    kind=loop_kind,
                    chain_count=len(boundary_loop.chains),
                    corner_count=len(boundary_loop.corners),
                    run_count=len(frame_runs),
                )
            )

        patch_summaries.append(
            _PatchDerivedTopologySummary(
                patch_id=patch_id,
                semantic_key=graph.get_patch_semantic_key(patch_id),
                patch_type=patch_type,
                world_facing=world_facing,
                face_count=len(node.face_indices),
                loop_kinds=tuple(loop_kinds),
                chain_count=patch_chain_count,
                corner_count=patch_corner_count,
                hole_count=patch_hole_count,
                run_count=patch_run_count,
                role_sequence=tuple(role_sequence),
                h_count=patch_h,
                v_count=patch_v,
                free_count=patch_free,
                loop_summaries=tuple(loop_summaries),
            )
        )

    aggregate_counts = _PatchGraphAggregateCounts(
        total_patches=len(graph.nodes),
        walls=walls,
        floors=floors,
        slopes=slopes,
        singles=singles,
        total_loops=total_loops,
        total_chains=total_chains,
        total_corners=total_corners,
        total_sharp_corners=total_sharp_corners,
        total_holes=total_holes,
        total_h=total_h,
        total_v=total_v,
        total_free=total_free,
        total_up=total_up,
        total_down=total_down,
        total_side=total_side,
        total_patch_links=total_patch_links,
        total_self_seams=total_self_seams,
        total_mesh_borders=total_mesh_borders,
        total_run_h=total_run_h,
        total_run_v=total_run_v,
        total_run_free=total_run_free,
    )
    return tuple(patch_summaries), aggregate_counts


def _build_patch_graph_derived_topology(graph, measure_chain_axis_metrics):
    """Build the canonical derived topology bundle over the final PatchGraph."""

    loop_frame_results = _build_patch_graph_loop_frame_results(graph, measure_chain_axis_metrics)
    patch_summaries, aggregate_counts = _build_patch_topology_summaries(graph, loop_frame_results)
    run_refs_by_corner = _build_junction_run_refs_by_corner(loop_frame_results)
    junctions = tuple(_build_patch_graph_junctions(graph, run_refs_by_corner=run_refs_by_corner))

    patch_summaries_by_id = {
        patch_summary.patch_id: patch_summary
        for patch_summary in patch_summaries
    }
    loop_summaries_by_key = {
        (loop_summary.patch_id, loop_summary.loop_index): loop_summary
        for patch_summary in patch_summaries
        for loop_summary in patch_summary.loop_summaries
    }
    frame_runs_by_loop = {
        loop_key: build_result.runs
        for loop_key, build_result in loop_frame_results.items()
    }
    junctions_by_vert_index = {
        junction.vert_index: junction
        for junction in junctions
    }

    return _PatchGraphDerivedTopology(
        patch_summaries=patch_summaries,
        patch_summaries_by_id=MappingProxyType(dict(patch_summaries_by_id)),
        loop_summaries_by_key=MappingProxyType(dict(loop_summaries_by_key)),
        aggregate_counts=aggregate_counts,
        loop_frame_results=MappingProxyType(dict(loop_frame_results)),
        frame_runs_by_loop=MappingProxyType(dict(frame_runs_by_loop)),
        run_refs_by_corner=MappingProxyType(dict(run_refs_by_corner)),
        junctions=junctions,
        junctions_by_vert_index=MappingProxyType(dict(junctions_by_vert_index)),
    )
