from __future__ import annotations

from dataclasses import replace
from decimal import Decimal
from functools import lru_cache

from cftuv_envelope import *
from reference_factories import angular_snapshot, straight_snapshot


@lru_cache(maxsize=None)
def opposing_strip_state(
    alpha: str,
    *,
    width: float = 10.0,
    height: float = 10.0,
    scale: float = 1.0,
    translation: tuple[float, float] = (0.0, 0.0),
    reverse_sources: bool = False,
):
    tx, ty = translation

    def point(x: float, y: float) -> tuple[float, float]:
        return tx + scale * x, ty + scale * y

    face = (
        point(0.0, 0.0),
        point(width, 0.0),
        point(width, height),
        point(0.0, height),
    )
    routes = (
        {
            "name": "left",
            "points": (point(0.0, height), point(0.0, 0.0)),
        },
        {
            "name": "right",
            "points": (point(width, 0.0), point(width, height)),
        },
    )
    if reverse_sources:
        routes = tuple(reversed(routes))
    snapshot, request = straight_snapshot(
        faces=(face,),
        source_routes=routes,
        alpha=alpha,
        revision_name=f"session-d-opposed-{alpha}-{scale}-{translation}",
    )
    compile_result = compile_reference_envelopes(snapshot, request)
    assert compile_result.compilation is not None, compile_result.diagnostics
    raw_result = evaluate_reference_raw_coverage(
        compile_result.compilation, alpha
    )
    assert raw_result.raw_coverage is not None, raw_result.diagnostics
    resolved = resolve_coverage_interactions(
        compile_result.compilation,
        raw_result.raw_coverage.boundary_resolved_envelopes,
        raw_result.raw_coverage,
    )
    return compile_result.compilation, raw_result.raw_coverage, resolved


def _with_third_strip(snapshot, request, alpha: str):
    d1 = SourceVertexId("d1")
    d2 = SourceVertexId("d2")
    edge = next(
        item
        for item in snapshot.surface_ir.source_edges
        if {item.vertex_a_id, item.vertex_b_id} == {d1, d2}
    )
    patch_id = PatchId("patch")
    domain_id = PatchDomainId("domain")
    chain_id = PhysicalChainId("third-chain")
    use_id = ChainUseId("third-use")
    launch_id = BoundaryConstraintId("third-launch")
    sector_id = OwnerSectorId("third-sector")
    lineage = frozenset({LineageId("third-lineage")})
    chain = PhysicalChainV1(
        chain_id,
        PhysicalChainKind.PHYSICAL_DECAL_SOURCE,
        False,
        (d1, d2),
        (edge.edge_id,),
        lineage,
        frozenset(),
    )
    use = ChainUseV1(
        use_id,
        chain_id,
        patch_id,
        domain_id,
        BoundaryLoopId("outer-loop"),
        ChainUseOrientation.A_START_TO_END,
        frozenset({ChainUseRole.DECAL_SOURCE}),
        LaunchLocusV1(
            launch_id,
            OwnerInteriorDirection.OWNER_INTERIOR,
            True,
        ),
    )
    launch = BoundaryConstraintV1(
        launch_id,
        domain_id,
        BoundaryConstraintKind.SOURCE_LAUNCH_BOUNDARY,
        ChainUseConstraintTargetV1(use_id),
        use_id,
        False,
        True,
    )
    sector = PatchSectorV1(
        sector_id,
        use_id,
        True,
        LawId("owner-sector"),
    )
    domain = next(iter(snapshot.patch_domains))
    loop = next(iter(snapshot.boundary_loops))
    terminals = frozenset(
        TerminalRelationV1(
            TerminalRelationId(f"third-{role.value.lower()}"),
            vertex_id,
            use_id,
            patch_id,
            domain_id,
            TerminalRelationKind.PHYSICAL_CHAIN_ENDPOINT,
            role,
            sector_id,
            lineage,
        )
        for role, vertex_id in (
            (TerminalEndpointRole.START, d1),
            (TerminalEndpointRole.END, d2),
        )
    )
    snapshot = replace(
        snapshot,
        physical_chains=snapshot.physical_chains | frozenset({chain}),
        chain_uses=snapshot.chain_uses | frozenset({use}),
        boundary_constraints=(
            snapshot.boundary_constraints | frozenset({launch})
        ),
        patch_domains=frozenset(
            {
                replace(
                    domain,
                    boundary_constraint_ids=(
                        domain.boundary_constraint_ids
                        | frozenset({launch_id})
                    ),
                    sectors=domain.sectors | frozenset({sector}),
                )
            }
        ),
        boundary_loops=frozenset(
            {
                replace(
                    loop,
                    ordered_chain_use_ids=(
                        *loop.ordered_chain_use_ids,
                        use_id,
                    ),
                )
            }
        ),
        terminal_relations=snapshot.terminal_relations | terminals,
    )
    request = replace(
        request,
        selected_chain_use_ids=(
            request.selected_chain_use_ids | frozenset({use_id})
        ),
        requested_alpha=LocalLengthV1(Decimal(alpha)),
    )
    return snapshot, request


@lru_cache(maxsize=None)
def angular_third_strip_state(hidden_count: int, alpha: str):
    snapshot, request = angular_snapshot(hidden_count)
    snapshot, request = _with_third_strip(snapshot, request, alpha)
    compile_result = compile_reference_envelopes(snapshot, request)
    assert compile_result.compilation is not None, compile_result.diagnostics
    raw_result = evaluate_reference_raw_coverage(
        compile_result.compilation, alpha
    )
    assert raw_result.raw_coverage is not None, raw_result.diagnostics
    resolved = resolve_coverage_interactions(
        compile_result.compilation,
        raw_result.raw_coverage.boundary_resolved_envelopes,
        raw_result.raw_coverage,
    )
    return compile_result.compilation, raw_result.raw_coverage, resolved
