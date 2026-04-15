from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Mapping, Optional

try:
    from .analysis_records import BandSpineData, _PatchDerivedTopologySummary
    from .band_spine import build_band_spine_from_groups
    from .model import ChainRef, LoopKind, PatchGraph
    from .shape_classify import PatchShapeSemantics, classify_patch_shape_semantics
    from .shape_types import LoopShapeInterpretation, PatchShapeClass
    from .structural_tokens import LoopSignature, build_loop_signature
except ImportError:
    from analysis_records import BandSpineData, _PatchDerivedTopologySummary
    from band_spine import build_band_spine_from_groups
    from model import ChainRef, LoopKind, PatchGraph
    from shape_classify import PatchShapeSemantics, classify_patch_shape_semantics
    from shape_types import LoopShapeInterpretation, PatchShapeClass
    from structural_tokens import LoopSignature, build_loop_signature


ShapeSupportHandler = Callable[
    [PatchGraph, _PatchDerivedTopologySummary, "PatchShapeFingerprint"],
    "PatchShapeSupportArtifact",
]


@dataclass(frozen=True)
class PatchShapeFingerprint:
    patch_id: int
    loop_signatures: tuple[LoopSignature, ...]
    loop_interpretations: tuple[LoopShapeInterpretation, ...]
    shape_class: PatchShapeClass


@dataclass(frozen=True)
class PatchShapeSupportResult:
    patch_shape_classes: Mapping[int, PatchShapeClass]
    loop_signatures: Mapping[int, list[LoopSignature]]
    loop_shape_interpretations: Mapping[int, list[LoopShapeInterpretation]]
    straighten_chain_refs: frozenset[ChainRef]
    band_spine_data: Mapping[int, BandSpineData]


@dataclass(frozen=True)
class PatchShapeSupportArtifact:
    straighten_chain_refs: frozenset[ChainRef] = frozenset()
    band_spine_data: Optional[BandSpineData] = None


def detect_patch_shape_fingerprint(
    graph: PatchGraph,
    patch_id: int,
) -> PatchShapeFingerprint:
    node = graph.nodes[patch_id]
    loop_signatures = tuple(
        build_loop_signature(patch_id, loop_index, boundary_loop, node)
        for loop_index, boundary_loop in enumerate(node.boundary_loops)
    )
    semantics: PatchShapeSemantics = classify_patch_shape_semantics(
        loop_signatures,
        _debug_patch_id=patch_id,
    )
    return PatchShapeFingerprint(
        patch_id=patch_id,
        loop_signatures=loop_signatures,
        loop_interpretations=semantics.loop_interpretations,
        shape_class=semantics.shape_class,
    )


def classify_patch_shape_fingerprint(fingerprint: PatchShapeFingerprint) -> PatchShapeClass:
    return fingerprint.shape_class


def _resolve_outer_loop_shape(
    graph: PatchGraph,
    patch_id: int,
    fingerprint: PatchShapeFingerprint,
) -> tuple[Optional[LoopSignature], Optional[LoopShapeInterpretation]]:
    node = graph.nodes.get(patch_id)
    if node is None:
        return None, None

    outer_signature = next(
        (
            signature
            for signature in fingerprint.loop_signatures
            if 0 <= signature.loop_index < len(node.boundary_loops)
            and node.boundary_loops[signature.loop_index].kind == LoopKind.OUTER
        ),
        None,
    )
    if outer_signature is None:
        return None, None

    outer_interpretation = next(
        (
            interpretation
            for interpretation in fingerprint.loop_interpretations
            if interpretation.loop_index == outer_signature.loop_index
        ),
        None,
    )
    return outer_signature, outer_interpretation


def _default_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> PatchShapeSupportArtifact:
    _ = graph, patch_summary, fingerprint
    return PatchShapeSupportArtifact()


def _summary_has_split_band_support(patch_summary: _PatchDerivedTopologySummary) -> bool:
    """Structural summary can express BAND CAP paths that generic 4-chain shape cannot."""

    cap_path_groups = tuple(patch_summary.band_cap_path_groups)
    return (
        len(tuple(patch_summary.band_side_indices)) == 2
        and len(cap_path_groups) == 2
        and any(len(group) > 1 for group in cap_path_groups)
    )


def _summary_implies_band_shape(patch_summary: _PatchDerivedTopologySummary) -> bool:
    """Structural summary is authoritative for split-cap BAND shape classification."""

    return _summary_has_split_band_support(patch_summary)


def _band_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> PatchShapeSupportArtifact:
    node = graph.nodes.get(patch_summary.patch_id)
    if node is None or any(boundary_loop.kind == LoopKind.HOLE for boundary_loop in node.boundary_loops):
        return PatchShapeSupportArtifact()

    outer_signature, outer_interpretation = _resolve_outer_loop_shape(
        graph,
        patch_summary.patch_id,
        fingerprint,
    )
    if outer_signature is None:
        return PatchShapeSupportArtifact()

    shape_side_indices = (
        tuple(outer_interpretation.side_chain_indices)
        if outer_interpretation is not None
        else ()
    )
    shape_cap_path_groups = (
        tuple((chain_index,) for chain_index in outer_interpretation.cap_chain_indices)
        if outer_interpretation is not None and len(outer_interpretation.cap_chain_indices) == 2
        else ()
    )
    if shape_side_indices and shape_cap_path_groups:
        side_chain_indices = shape_side_indices
        cap_path_groups = shape_cap_path_groups
    else:
        side_chain_indices = tuple(patch_summary.band_side_indices)
        cap_path_groups = tuple(patch_summary.band_cap_path_groups)
    if len(side_chain_indices) != 2 or len(cap_path_groups) != 2:
        return PatchShapeSupportArtifact()

    straighten_chain_refs = frozenset(
        (patch_summary.patch_id, outer_signature.loop_index, chain_index)
        for chain_index in side_chain_indices
        if 0 <= chain_index < outer_signature.chain_count
    )
    spine_data = build_band_spine_from_groups(
        graph,
        patch_summary.patch_id,
        outer_signature.loop_index,
        side_chain_indices,
        cap_path_groups,
    )
    return PatchShapeSupportArtifact(
        straighten_chain_refs=straighten_chain_refs,
        band_spine_data=spine_data,
    )


def _cylinder_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> PatchShapeSupportArtifact:
    _ = graph, patch_summary, fingerprint
    return PatchShapeSupportArtifact()


def _cable_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> PatchShapeSupportArtifact:
    _ = graph, patch_summary, fingerprint
    return PatchShapeSupportArtifact()


_ACTIVE_SHAPE_SUPPORT_HANDLERS: dict[PatchShapeClass, ShapeSupportHandler] = {
    PatchShapeClass.BAND: _band_shape_support,
    PatchShapeClass.MIX: _default_shape_support,
}


def build_patch_shape_support_artifact(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> PatchShapeSupportArtifact:
    """Dispatch classified shape semantics to runtime-support builders.

    Reserved no-op hooks for future shapes stay local to this layer:
    `CYLINDER` -> `_cylinder_shape_support`
    `CABLE` -> `_cable_shape_support`
    """

    if _summary_implies_band_shape(patch_summary):
        return _band_shape_support(graph, patch_summary, fingerprint)

    handler = _ACTIVE_SHAPE_SUPPORT_HANDLERS.get(
        fingerprint.shape_class,
        _default_shape_support,
    )
    return handler(graph, patch_summary, fingerprint)


def build_patch_shape_support(
    graph: PatchGraph,
    patch_summaries_by_id: Mapping[int, _PatchDerivedTopologySummary],
) -> PatchShapeSupportResult:
    """Shape support phases: fingerprint -> classify -> support -> finalize."""

    loop_signatures: dict[int, list[LoopSignature]] = {}
    loop_shape_interpretations: dict[int, list[LoopShapeInterpretation]] = {}
    patch_shape_classes: dict[int, PatchShapeClass] = {}
    straighten_chain_refs: set[ChainRef] = set()
    band_spine_data: dict[int, BandSpineData] = {}

    for patch_id in sorted(graph.nodes.keys()):
        fingerprint = detect_patch_shape_fingerprint(graph, patch_id)
        loop_signatures[patch_id] = list(fingerprint.loop_signatures)
        loop_shape_interpretations[patch_id] = list(fingerprint.loop_interpretations)
        patch_shape_class = classify_patch_shape_fingerprint(fingerprint)

        patch_summary = patch_summaries_by_id.get(patch_id)
        if patch_summary is None:
            patch_shape_classes[patch_id] = patch_shape_class
            continue

        if _summary_implies_band_shape(patch_summary):
            patch_shape_class = PatchShapeClass.BAND

        artifact = build_patch_shape_support_artifact(
            graph,
            patch_summary,
            fingerprint,
        )
        patch_shape_classes[patch_id] = patch_shape_class
        straighten_chain_refs.update(artifact.straighten_chain_refs)
        if artifact.band_spine_data is not None:
            band_spine_data[patch_id] = artifact.band_spine_data

    return PatchShapeSupportResult(
        patch_shape_classes=MappingProxyType(dict(patch_shape_classes)),
        loop_signatures=MappingProxyType(dict(loop_signatures)),
        loop_shape_interpretations=MappingProxyType(dict(loop_shape_interpretations)),
        straighten_chain_refs=frozenset(straighten_chain_refs),
        band_spine_data=MappingProxyType(dict(band_spine_data)),
    )


__all__ = [
    "PatchShapeFingerprint",
    "PatchShapeSupportArtifact",
    "PatchShapeSupportResult",
    "detect_patch_shape_fingerprint",
    "classify_patch_shape_fingerprint",
    "build_patch_shape_support_artifact",
    "build_patch_shape_support",
]
