from __future__ import annotations

from typing import Optional

from mathutils import Vector

try:
    from .model import FrameAxisKind, FrameRole, FormattedReport, PatchGraph, ScaffoldMap
    from .solve_records import AttachmentCandidate, ClosureCutHeuristic, SolverGraph, SolvePlan
    from .solve_planning import _analyze_quilt_closure_cuts, _build_quilt_tree_edges
    from .solve_transfer import (
        _build_patch_transfer_targets,
        _collect_phase1_unsupported_patch_ids,
        _format_scaffold_uv_points,
        _ordered_quilt_patch_ids,
    )
    from .solve_instrumentation import format_quilt_telemetry_summary, format_quilt_telemetry_detail
except ImportError:
    from model import FrameAxisKind, FrameRole, FormattedReport, PatchGraph, ScaffoldMap
    from solve_records import AttachmentCandidate, ClosureCutHeuristic, SolverGraph, SolvePlan
    from solve_planning import _analyze_quilt_closure_cuts, _build_quilt_tree_edges
    from solve_transfer import (
        _build_patch_transfer_targets,
        _collect_phase1_unsupported_patch_ids,
        _format_scaffold_uv_points,
        _ordered_quilt_patch_ids,
    )
    from solve_instrumentation import format_quilt_telemetry_summary, format_quilt_telemetry_detail


def format_root_scaffold_report(
    graph: PatchGraph,
    scaffold_map: ScaffoldMap,
    mesh_name: Optional[str] = None,
) -> FormattedReport:
    lines = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    total_patches = 0
    unsupported = 0
    invalid_closure = 0
    closure_seam_count = 0
    max_closure_span_mismatch = 0.0
    max_closure_axis_phase = 0.0
    frame_group_count = 0
    max_row_scatter = 0.0
    max_column_scatter = 0.0
    for quilt in scaffold_map.quilts:
        placed_patch_ids = []
        for step_patch_id in [quilt.root_patch_id] + [patch_id for patch_id in quilt.patches.keys() if patch_id != quilt.root_patch_id]:
            if step_patch_id not in quilt.patches:
                continue
            placed_patch_ids.append(step_patch_id)
        if not placed_patch_ids:
            placed_patch_ids = [quilt.root_patch_id]
        lines.append(f"Quilt {quilt.quilt_index}: root=Patch {quilt.root_patch_id} ({graph.get_patch_semantic_key(quilt.root_patch_id)}) | patches:{placed_patch_ids}")
        closure_reports = getattr(quilt, 'closure_seam_reports', ())
        frame_reports = getattr(quilt, 'frame_alignment_reports', ())
        if closure_reports:
            closure_seam_count += len(closure_reports)
            max_closure_span_mismatch = max(
                max_closure_span_mismatch,
                max((report.span_mismatch for report in closure_reports), default=0.0),
            )
            max_closure_axis_phase = max(
                max_closure_axis_phase,
                max((report.axis_phase_offset_max for report in closure_reports), default=0.0),
            )
            lines.append(
                f"  ClosureSeams: {len(closure_reports)} | "
                f"max_span_mismatch:{max((report.span_mismatch for report in closure_reports), default=0.0):.6f} | "
                f"max_axis_phase:{max((report.axis_phase_offset_max for report in closure_reports), default=0.0):.6f}"
            )
            for report in closure_reports:
                lines.append(
                    "    "
                    + f"P{report.owner_patch_id} L{report.owner_loop_index}C{report.owner_chain_index}"
                    + f"<->P{report.target_patch_id} L{report.target_loop_index}C{report.target_chain_index} "
                    + f"{report.frame_role.value} mode:{report.anchor_mode.value} "
                    + f"a:{report.owner_anchor_count}/{report.target_anchor_count} "
                    + f"span3d:{report.canonical_3d_span:.6f} "
                    + f"uv:{report.owner_uv_span:.6f}/{report.target_uv_span:.6f} "
                    + f"mismatch:{report.span_mismatch:.6f} "
                    + f"axis:{report.owner_axis_error:.6f}/{report.target_axis_error:.6f} "
                    + f"phase:{report.axis_phase_offset_max:.6f}/{report.axis_phase_offset_mean:.6f} "
                    + f"cross:{report.cross_axis_offset_max:.6f}/{report.cross_axis_offset_mean:.6f} "
                    + f"uvd:{report.shared_uv_delta_max:.6f}/{report.shared_uv_delta_mean:.6f} "
                    + f"path:{report.tree_patch_distance} free:{report.free_bridge_count}"
                )
        if frame_reports:
            frame_group_count += len(frame_reports)
            max_row_scatter = max(
                max_row_scatter,
                max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.ROW), default=0.0),
            )
            max_column_scatter = max(
                max_column_scatter,
                max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.COLUMN), default=0.0),
            )
            lines.append(
                f"  FrameGroups: {len(frame_reports)} | "
                f"max_row_scatter:{max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.ROW), default=0.0):.6f} | "
                f"max_column_scatter:{max((report.scatter_max for report in frame_reports if report.axis_kind == FrameAxisKind.COLUMN), default=0.0):.6f}"
            )
            for report in frame_reports:
                coord_label = (
                    f"z:{report.class_coord_a:.6f}"
                    if report.axis_kind == FrameAxisKind.ROW
                    else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
                )
                refs_label = ','.join(
                    f"P{patch_id}L{loop_index}C{chain_index}"
                    for patch_id, loop_index, chain_index in report.member_refs
                )
                closure_tag = " closure:1" if report.closure_sensitive else " closure:0"
                lines.append(
                    "    "
                    + f"{report.axis_kind.value} {report.frame_role.value} {coord_label} "
                    + f"chains:{report.chain_count} target:{report.target_cross_uv:.6f} "
                    + f"scatter:{report.scatter_max:.6f}/{report.scatter_mean:.6f} "
                    + f"weight:{report.total_weight:.6f}{closure_tag} refs:[{refs_label}]"
                )
        telemetry = getattr(quilt, 'frontier_telemetry', None)
        if telemetry is not None:
            for tline in format_quilt_telemetry_detail(telemetry):
                lines.append(tline)

        for patch_id in placed_patch_ids:
            patch_placement = quilt.patches.get(patch_id)
            total_patches += 1
            signature = graph.get_patch_semantic_key(patch_id)
            if patch_placement is None:
                unsupported += 1
                lines.append(f"  Patch {patch_id} ({signature}) | scaffold:missing")
                continue
            if patch_placement.notes:
                unsupported += 1
                lines.append(f"  Patch {patch_id} ({signature}) | scaffold:unsupported | notes:{', '.join(patch_placement.notes)}")
                continue

            scaffold_status = 'ok' if patch_placement.closure_valid else 'invalid_closure'
            if not patch_placement.closure_valid:
                invalid_closure += 1
            lines.append(
                f"  Patch {patch_id} ({signature}) | loop:{patch_placement.loop_index} | start_chain:{patch_placement.root_chain_index} | "
                f"bbox:({patch_placement.bbox_min.x:.4f}, {patch_placement.bbox_min.y:.4f}) -> ({patch_placement.bbox_max.x:.4f}, {patch_placement.bbox_max.y:.4f}) | "
                f"closure:{patch_placement.closure_error:.6f} | max_gap:{patch_placement.max_chain_gap:.6f} | status:{scaffold_status}"
            )
            if patch_placement.gap_reports:
                for gap_report in patch_placement.gap_reports:
                    lines.append(
                        f"    Gap {gap_report.chain_index}->{gap_report.next_chain_index}: {gap_report.gap:.6f}"
                    )
            node = graph.nodes.get(patch_id)
            if node is None or patch_placement.loop_index < 0 or patch_placement.loop_index >= len(node.boundary_loops):
                continue
            boundary_loop = node.boundary_loops[patch_placement.loop_index]
            for corner_index in sorted(patch_placement.corner_positions.keys()):
                point = patch_placement.corner_positions[corner_index]
                turn_angle = 0.0
                prev_chain = '-'
                next_chain = '-'
                if 0 <= corner_index < len(boundary_loop.corners):
                    corner = boundary_loop.corners[corner_index]
                    turn_angle = corner.turn_angle_deg
                    prev_chain = corner.prev_chain_index
                    next_chain = corner.next_chain_index
                lines.append(
                    f"    Corner {corner_index}: ({point.x:.4f}, {point.y:.4f}) | chains:{prev_chain}->{next_chain} | turn:{turn_angle:.1f}"
                )
            sc_set = patch_placement.scaffold_connected_chains
            for chain_placement in patch_placement.chain_placements:
                if not chain_placement.points:
                    continue
                start_point = chain_placement.points[0][1]
                end_point = chain_placement.points[-1][1]
                isolated_tag = ""
                if chain_placement.frame_role in {FrameRole.H_FRAME, FrameRole.V_FRAME} and chain_placement.chain_index not in sc_set:
                    isolated_tag = " [ISOLATED]"
                lines.append(
                    f"    {chain_placement.source_kind.value.title()} {chain_placement.chain_index}: {chain_placement.frame_role.value} | "
                    f"points:{len(chain_placement.points)} | start:({start_point.x:.4f}, {start_point.y:.4f}) | "
                    f"end:({end_point.x:.4f}, {end_point.y:.4f}) | uv:{_format_scaffold_uv_points(chain_placement.points)}{isolated_tag}"
                )

    summary = (
        f"Scaffold quilts: {len(scaffold_map.quilts)} | Patches: {total_patches} | Unsupported: {unsupported} | "
        f"Invalid closure: {invalid_closure} | Closure seams: {closure_seam_count} | "
        f"Max seam mismatch: {max_closure_span_mismatch:.6f} | "
        f"Max seam phase: {max_closure_axis_phase:.6f} | "
        f"Frame groups: {frame_group_count} | "
        f"Max row scatter: {max_row_scatter:.6f} | "
        f"Max column scatter: {max_column_scatter:.6f}"
    )
    return FormattedReport(lines=lines, summary=summary)


def format_regression_snapshot_report(
    bm,
    graph: PatchGraph,
    solve_plan: Optional[SolvePlan],
    scaffold_map: ScaffoldMap,
    mesh_name: Optional[str] = None,
) -> FormattedReport:
    lines = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")

    solve_plan = solve_plan or SolvePlan()
    quilt_plan_by_index = {quilt.quilt_index: quilt for quilt in solve_plan.quilts}
    unsupported_patch_ids = _collect_phase1_unsupported_patch_ids(scaffold_map)
    invalid_closure_patch_ids = sorted({
        patch_id
        for quilt_scaffold in scaffold_map.quilts
        for patch_id, patch_placement in quilt_scaffold.patches.items()
        if patch_placement is not None and not patch_placement.notes and not patch_placement.closure_valid
    })
    closure_seam_count = sum(
        len(getattr(quilt_scaffold, 'closure_seam_reports', ()))
        for quilt_scaffold in scaffold_map.quilts
    )
    max_closure_span_mismatch = max(
        (
            report.span_mismatch
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'closure_seam_reports', ())
        ),
        default=0.0,
    )
    max_closure_axis_phase = max(
        (
            report.axis_phase_offset_max
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'closure_seam_reports', ())
        ),
        default=0.0,
    )
    frame_group_count = sum(
        len(getattr(quilt_scaffold, 'frame_alignment_reports', ()))
        for quilt_scaffold in scaffold_map.quilts
    )
    max_row_scatter = max(
        (
            report.scatter_max
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'frame_alignment_reports', ())
            if report.axis_kind == FrameAxisKind.ROW
        ),
        default=0.0,
    )
    max_column_scatter = max(
        (
            report.scatter_max
            for quilt_scaffold in scaffold_map.quilts
            for report in getattr(quilt_scaffold, 'frame_alignment_reports', ())
            if report.axis_kind == FrameAxisKind.COLUMN
        ),
        default=0.0,
    )

    lines.append(f"Patches: {len(graph.nodes)}")
    lines.append(f"Quilts: {len(scaffold_map.quilts)}")
    lines.append(f"Skipped patches: {sorted(solve_plan.skipped_patch_ids)}")
    lines.append(f"Unsupported patches: {unsupported_patch_ids}")
    lines.append(f"Invalid closure patches: {invalid_closure_patch_ids}")
    lines.append(f"Conformal fallback patches: {len(unsupported_patch_ids)}")
    lines.append(
        "Closure seams: "
        f"{closure_seam_count} | max_span_mismatch:{max_closure_span_mismatch:.6f} | "
        f"max_axis_phase:{max_closure_axis_phase:.6f}"
    )
    lines.append(
        "Frame groups: "
        f"{frame_group_count} | max_row_scatter:{max_row_scatter:.6f} | "
        f"max_column_scatter:{max_column_scatter:.6f}"
    )

    for quilt_scaffold in scaffold_map.quilts:
        quilt_plan = quilt_plan_by_index.get(quilt_scaffold.quilt_index)
        ordered_patch_ids = _ordered_quilt_patch_ids(quilt_scaffold, quilt_plan)
        if not ordered_patch_ids:
            ordered_patch_ids = sorted(quilt_scaffold.patches.keys())

        lines.append("")
        lines.append(f"## Quilt {quilt_scaffold.quilt_index}")
        lines.append(f"root_patch_id: {quilt_scaffold.root_patch_id}")
        lines.append(f"patch_ids: {ordered_patch_ids}")
        lines.append(f"stop_reason: {quilt_plan.stop_reason.value if quilt_plan is not None else ''}")
        lines.append(
            "build_order: "
            + "[" + ", ".join(
                f"P{patch_id}L{loop_index}C{chain_index}"
                for patch_id, loop_index, chain_index in quilt_scaffold.build_order
            ) + "]"
        )

        quilt_unsupported_patch_ids = sorted(
            patch_id
            for patch_id in ordered_patch_ids
            if quilt_scaffold.patches.get(patch_id) is not None
            and quilt_scaffold.patches[patch_id].notes
        )
        quilt_invalid_closure_patch_ids = sorted(
            patch_id
            for patch_id in ordered_patch_ids
            if quilt_scaffold.patches.get(patch_id) is not None
            and not quilt_scaffold.patches[patch_id].notes
            and not quilt_scaffold.patches[patch_id].closure_valid
        )
        lines.append(f"unsupported_patch_ids: {quilt_unsupported_patch_ids}")
        lines.append(f"invalid_closure_patch_ids: {quilt_invalid_closure_patch_ids}")

        closure_reports = getattr(quilt_scaffold, 'closure_seam_reports', ())
        if closure_reports:
            lines.append("closure_seams:")
            for report in closure_reports:
                lines.append(
                    "  - "
                    + f"P{report.owner_patch_id}L{report.owner_loop_index}C{report.owner_chain_index}"
                    + f"<->P{report.target_patch_id}L{report.target_loop_index}C{report.target_chain_index} "
                    + f"{report.frame_role.value} mode:{report.anchor_mode.value} "
                    + f"mismatch:{report.span_mismatch:.6f} phase:{report.axis_phase_offset_max:.6f} "
                    + f"free:{report.free_bridge_count}"
                )
        else:
            lines.append("closure_seams: []")

        frame_reports = getattr(quilt_scaffold, 'frame_alignment_reports', ())
        if frame_reports:
            lines.append("frame_groups:")
            for report in frame_reports:
                coord_label = (
                    f"z:{report.class_coord_a:.6f}"
                    if report.axis_kind == FrameAxisKind.ROW
                    else f"xy:({report.class_coord_a:.6f},{report.class_coord_b:.6f})"
                )
                lines.append(
                    "  - "
                    + f"{report.axis_kind.value} {report.frame_role.value} {coord_label} "
                    + f"chains:{report.chain_count} target:{report.target_cross_uv:.6f} "
                    + f"scatter:{report.scatter_max:.6f}/{report.scatter_mean:.6f}"
                )
        else:
            lines.append("frame_groups: []")

        telemetry = getattr(quilt_scaffold, 'frontier_telemetry', None)
        if telemetry is not None:
            for tline in format_quilt_telemetry_summary(telemetry):
                lines.append(tline)

        lines.append("patches:")
        for patch_id in ordered_patch_ids:
            patch_placement = quilt_scaffold.patches.get(patch_id)
            if patch_placement is None:
                lines.append(f"  - P{patch_id} status:missing_patch")
                continue

            transfer_state = _build_patch_transfer_targets(
                bm,
                graph,
                patch_placement,
                Vector((0.0, 0.0)),
            )
            node = graph.nodes.get(patch_id)
            signature = graph.get_patch_semantic_key(patch_id)
            note_label = ",".join(patch_placement.notes) if patch_placement.notes else "-"
            dep_label = list(patch_placement.dependency_patches) if patch_placement.dependency_patches else []
            lines.append(
                "  - "
                + f"P{patch_id} {signature} "
                + f"status:{patch_placement.status.value} transfer:{transfer_state.status.value} "
                + f"loop:{patch_placement.loop_index} root_chain:{patch_placement.root_chain_index} "
                + f"closure_valid:{1 if patch_placement.closure_valid else 0} "
                + f"closure_error:{patch_placement.closure_error:.6f} "
                + f"placed_chains:{len(patch_placement.chain_placements)} "
                + f"unplaced:{list(patch_placement.unplaced_chain_indices)} "
                + f"deps:{dep_label} "
                + f"notes:{note_label} "
                + f"resolved:{transfer_state.resolved_scaffold_points}/{transfer_state.scaffold_points} "
                + f"uv_targets:{transfer_state.uv_targets_resolved} "
                + f"pinned:{transfer_state.pinned_uv_targets} "
                + f"unpinned:{transfer_state.unpinned_uv_targets} "
                + f"conflicts:{transfer_state.conflicting_uv_targets}"
            )
            if node is not None and 0 <= patch_placement.loop_index < len(node.boundary_loops):
                sc_count = len(patch_placement.scaffold_connected_chains)
                lines.append(
                    f"    scaffold_connected_chains:{sc_count}/{len(node.boundary_loops[patch_placement.loop_index].chains)}"
                )
            if transfer_state.pin_map is not None:
                pin_parts = [
                    f"C{dec.chain_index}:{dec.reason}"
                    for dec in transfer_state.pin_map.chain_decisions
                ]
                lines.append(f"    pin_reasons:[{', '.join(pin_parts)}]")

    summary = (
        f"Regression snapshot | quilts:{len(scaffold_map.quilts)} | patches:{len(graph.nodes)} | "
        f"unsupported:{len(unsupported_patch_ids)} | invalid_closure:{len(invalid_closure_patch_ids)} | "
        f"conformal_fallback_patches:{len(unsupported_patch_ids)}"
    )
    return FormattedReport(lines=lines, summary=summary)


def _format_closure_cut_heuristic(item: ClosureCutHeuristic) -> str:
    candidate = item.candidate
    return (
        f"{item.edge_key[0]}-{item.edge_key[1]} cut:{item.score:.2f} {item.support_label} "
        f"| roles:{candidate.owner_role.value}<->{candidate.target_role.value} "
        f"| refs:L{candidate.owner_loop_index}C{candidate.owner_chain_index}->"
        f"L{candidate.target_loop_index}C{candidate.target_chain_index} "
        f"| {' '.join(item.reasons)}"
    )


def _format_patch_signature(graph: PatchGraph, patch_id: int) -> str:
    return graph.get_patch_semantic_key(patch_id)


def _format_candidate_line(candidate: AttachmentCandidate) -> str:
    return (
        f"{candidate.owner_patch_id} -> {candidate.target_patch_id} | score:{candidate.score:.2f} "
        f"| seam:{candidate.seam_length:.4f} | roles:{candidate.owner_role.value}<->{candidate.target_role.value} "
        f"| loops:{candidate.owner_loop_kind.value}<->{candidate.target_loop_kind.value} "
        f"| refs:L{candidate.owner_loop_index}C{candidate.owner_chain_index}->L{candidate.target_loop_index}C{candidate.target_chain_index}"
    )


def format_solve_plan_report(
    graph: PatchGraph,
    solver_graph: SolverGraph,
    solve_plan: SolvePlan,
    mesh_name: Optional[str] = None,
) -> FormattedReport:
    lines: list[str] = []
    if mesh_name:
        lines.append(f"Mesh: {mesh_name}")
    lines.append(
        f"Thresholds: propagate>={solve_plan.propagate_threshold:.2f} weak>={solve_plan.weak_threshold:.2f}"
    )

    components = solver_graph.solve_components or graph.connected_components()
    for component_index, component in enumerate(components):
        patch_ids = sorted(component)
        lines.append(f"Component {component_index}: patches={patch_ids}")
        lines.append("  Patch Certainty:")
        component_scores = sorted(
            (solver_graph.patch_scores[patch_id] for patch_id in patch_ids),
            key=lambda item: item.root_score,
            reverse=True,
        )
        for certainty in component_scores:
            lines.append(
                "    "
                + f"Patch {certainty.patch_id}: {_format_patch_signature(graph, certainty.patch_id)} "
                + f"| local:{'Y' if certainty.local_solvable else 'N'} | root:{certainty.root_score:.2f} "
                + f"| loops:outer={certainty.outer_count} hole={certainty.hole_count} "
                + f"| chains:{certainty.chain_count} H:{certainty.h_count} V:{certainty.v_count} Free:{certainty.free_count}"
            )
            if certainty.reasons:
                lines.append(f"      reasons: {' | '.join(certainty.reasons)}")

        component_candidates = [
            candidate
            for candidate in solver_graph.candidates
            if candidate.owner_patch_id in component and candidate.target_patch_id in component
        ]
        if component_candidates:
            lines.append("  Conductivity:")
            for candidate in sorted(component_candidates, key=lambda item: (-item.score, item.owner_patch_id, item.target_patch_id)):
                lines.append("    " + _format_candidate_line(candidate))
                lines.append(
                    "      "
                    + f"transitions:{candidate.owner_transition} | {candidate.target_transition} "
                    + f"| seam:{candidate.seam_norm:.2f} pair:{candidate.best_pair_strength:.2f} "
                    + f"cont:{candidate.frame_continuation:.2f} bridge:{candidate.endpoint_bridge:.2f} "
                    + f"corner:{candidate.corner_strength:.2f} sem:{candidate.semantic_strength:.2f} "
                    + f"ep:{candidate.endpoint_strength:.2f} amb:{candidate.ambiguity_penalty:.2f}"
                )

    closure_cut_analyses_by_quilt = {
        quilt.quilt_index: _analyze_quilt_closure_cuts(graph, solver_graph, quilt)
        for quilt in solve_plan.quilts
    }

    for quilt in solve_plan.quilts:
        tree_edges = sorted(_build_quilt_tree_edges(quilt))
        closure_cut_analyses = closure_cut_analyses_by_quilt.get(quilt.quilt_index, ())
        lines.append(
            f"Quilt {quilt.quilt_index}: component={quilt.component_index} root=Patch {quilt.root_patch_id} "
            f"({_format_patch_signature(graph, quilt.root_patch_id)}) root_score={quilt.root_score:.2f}"
        )
        if tree_edges:
            lines.append(
                "  Tree edges: " + ", ".join(f"{edge_key[0]}-{edge_key[1]}" for edge_key in tree_edges)
            )
        for step in quilt.steps:
            if step.is_root or step.incoming_candidate is None:
                lines.append(
                    f"  Step {step.step_index}: ROOT Patch {step.patch_id} "
                    f"({_format_patch_signature(graph, step.patch_id)})"
                )
                continue
            candidate = step.incoming_candidate
            lines.append(
                f"  Step {step.step_index}: Patch {candidate.owner_patch_id} -> Patch {step.patch_id} "
                f"| score:{candidate.score:.2f} | roles:{candidate.owner_role.value}<->{candidate.target_role.value}"
            )
            lines.append(
                "    "
                + f"refs:L{candidate.owner_loop_index}C{candidate.owner_chain_index}->"
                + f"L{candidate.target_loop_index}C{candidate.target_chain_index} "
                + f"| transitions:{candidate.owner_transition} | {candidate.target_transition}"
            )
        if closure_cut_analyses:
            lines.append("  Closure cuts:")
            for analysis_index, analysis in enumerate(closure_cut_analyses):
                current_edge = f"{analysis.current_cut.edge_key[0]}-{analysis.current_cut.edge_key[1]}"
                recommended_edge = f"{analysis.recommended_cut.edge_key[0]}-{analysis.recommended_cut.edge_key[1]}"
                decision = (
                    f"keep {current_edge}"
                    if analysis.current_cut.edge_key == analysis.recommended_cut.edge_key
                    else f"swap {current_edge} -> {recommended_edge}"
                )
                lines.append(
                    f"    Cycle {analysis_index}: path={list(analysis.path_patch_ids)} | decision:{decision}"
                )
                lines.append("      current: " + _format_closure_cut_heuristic(analysis.current_cut))
                lines.append("      best:    " + _format_closure_cut_heuristic(analysis.recommended_cut))
                for cycle_edge in analysis.cycle_edges:
                    marker = ""
                    if cycle_edge.edge_key == analysis.recommended_cut.edge_key:
                        marker += " [BEST]"
                    if cycle_edge.edge_key == analysis.current_cut.edge_key:
                        marker += " [CURRENT]"
                    lines.append("      edge: " + _format_closure_cut_heuristic(cycle_edge) + marker)
        lines.append(f"  Stop: {quilt.stop_reason.value}")
        if quilt.deferred_candidates:
            lines.append("  Deferred frontier:")
            for candidate in quilt.deferred_candidates:
                lines.append("    " + _format_candidate_line(candidate))
        if quilt.rejected_candidates:
            lines.append("  Rejected frontier:")
            for candidate in quilt.rejected_candidates:
                lines.append("    " + _format_candidate_line(candidate))

    if solve_plan.skipped_patch_ids:
        lines.append(f"Skipped patches: {sorted(solve_plan.skipped_patch_ids)}")

    total_steps = sum(len(quilt.steps) for quilt in solve_plan.quilts)
    total_deferred = sum(len(quilt.deferred_candidates) for quilt in solve_plan.quilts)
    total_rejected = sum(len(quilt.rejected_candidates) for quilt in solve_plan.quilts)
    total_closure_cycles = sum(len(items) for items in closure_cut_analyses_by_quilt.values())
    total_cut_swaps = sum(
        1
        for analyses in closure_cut_analyses_by_quilt.values()
        for analysis in analyses
        if analysis.current_cut.edge_key != analysis.recommended_cut.edge_key
    )
    summary = (
        f"Quilts: {len(solve_plan.quilts)} | Steps: {total_steps} | "
        f"Deferred: {total_deferred} | Rejected: {total_rejected} | "
        f"ClosureCycles: {total_closure_cycles} | CutSwaps: {total_cut_swaps} | "
        f"Skipped: {len(solve_plan.skipped_patch_ids)}"
    )
    return FormattedReport(lines=lines, summary=summary)





