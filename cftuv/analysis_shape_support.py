from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Callable, Mapping, Optional

try:
    from .analysis_records import BandSpineData, _PatchDerivedTopologySummary
    from .model import BandMode, ChainRef, LoopKind, PatchGraph
    from .structural_tokens import LoopSignature, PatchShapeClass, build_loop_signature, classify_patch_shape
    from .band_spine import build_band_spine_from_groups
except ImportError:
    from analysis_records import BandSpineData, _PatchDerivedTopologySummary
    from model import BandMode, ChainRef, LoopKind, PatchGraph
    from structural_tokens import LoopSignature, PatchShapeClass, build_loop_signature, classify_patch_shape
    from band_spine import build_band_spine_from_groups


ShapeSupportHandler = Callable[
    [PatchGraph, _PatchDerivedTopologySummary, "PatchShapeFingerprint"],
    tuple[frozenset[ChainRef], Optional[BandSpineData]],
]


@dataclass(frozen=True)
class PatchShapeFingerprint:
    patch_id: int
    loop_signatures: tuple[LoopSignature, ...]
    shape_class: PatchShapeClass


@dataclass(frozen=True)
class PatchShapeSupportResult:
    patch_shape_classes: Mapping[int, PatchShapeClass]
    loop_signatures: Mapping[int, list[LoopSignature]]
    straighten_chain_refs: frozenset[ChainRef]
    band_spine_data: Mapping[int, BandSpineData]


def detect_patch_shape_fingerprint(
    graph: PatchGraph,
    patch_id: int,
) -> PatchShapeFingerprint:
    node = graph.nodes[patch_id]
    loop_signatures = tuple(
        build_loop_signature(patch_id, loop_index, boundary_loop, node)
        for loop_index, boundary_loop in enumerate(node.boundary_loops)
    )
    return PatchShapeFingerprint(
        patch_id=patch_id,
        loop_signatures=loop_signatures,
        shape_class=classify_patch_shape(list(loop_signatures), _debug_patch_id=patch_id),
    )


def classify_patch_shape_fingerprint(fingerprint: PatchShapeFingerprint) -> PatchShapeClass:
    return fingerprint.shape_class


def _default_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> tuple[frozenset[ChainRef], Optional[BandSpineData]]:
    _ = graph, patch_summary, fingerprint
    return frozenset(), None


def _band_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> tuple[frozenset[ChainRef], Optional[BandSpineData]]:
    if (
        patch_summary.band_mode == BandMode.NOT_BAND
        or not patch_summary.band_requires_intervention
        or len(patch_summary.band_side_indices) != 2
        or len(patch_summary.band_cap_path_groups) != 2
    ):
        return frozenset(), None

    node = graph.nodes.get(patch_summary.patch_id)
    if node is None or any(boundary_loop.kind == LoopKind.HOLE for boundary_loop in node.boundary_loops):
        return frozenset(), None

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
        return frozenset(), None

    straighten_chain_refs = frozenset(
        (patch_summary.patch_id, outer_signature.loop_index, chain_index)
        for chain_index in patch_summary.band_side_indices
        if 0 <= chain_index < outer_signature.chain_count
    )
    spine_data = build_band_spine_from_groups(
        graph,
        patch_summary.patch_id,
        outer_signature.loop_index,
        patch_summary.band_side_indices,
        patch_summary.band_cap_path_groups,
    )
    return straighten_chain_refs, spine_data


def _cylinder_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> tuple[frozenset[ChainRef], Optional[BandSpineData]]:
    _ = graph, patch_summary, fingerprint
    return frozenset(), None


def _cable_shape_support(
    graph: PatchGraph,
    patch_summary: _PatchDerivedTopologySummary,
    fingerprint: PatchShapeFingerprint,
) -> tuple[frozenset[ChainRef], Optional[BandSpineData]]:
    _ = graph, patch_summary, fingerprint
    return frozenset(), None


_SHAPE_SUPPORT_HANDLERS: dict[str, ShapeSupportHandler] = {
    PatchShapeClass.BAND.value: _band_shape_support,
    "CYLINDER": _cylinder_shape_support,
    "CABLE": _cable_shape_support,
    PatchShapeClass.MIX.value: _default_shape_support,
}


def build_patch_shape_support(
    graph: PatchGraph,
    patch_summaries_by_id: Mapping[int, _PatchDerivedTopologySummary],
) -> PatchShapeSupportResult:
    """Shape support phases: fingerprint -> classify -> support -> finalize."""

    loop_signatures: dict[int, list[LoopSignature]] = {}
    patch_shape_classes: dict[int, PatchShapeClass] = {}
    straighten_chain_refs: set[ChainRef] = set()
    band_spine_data: dict[int, BandSpineData] = {}

    for patch_id in sorted(graph.nodes.keys()):
        fingerprint = detect_patch_shape_fingerprint(graph, patch_id)
        loop_signatures[patch_id] = list(fingerprint.loop_signatures)
        patch_shape_class = classify_patch_shape_fingerprint(fingerprint)
        patch_shape_classes[patch_id] = patch_shape_class

        patch_summary = patch_summaries_by_id.get(patch_id)
        if patch_summary is None:
            continue

        handler = _SHAPE_SUPPORT_HANDLERS.get(patch_shape_class.value, _default_shape_support)
        patch_chain_refs, spine_data = handler(graph, patch_summary, fingerprint)
        straighten_chain_refs.update(patch_chain_refs)
        if spine_data is not None:
            band_spine_data[patch_id] = spine_data

    return PatchShapeSupportResult(
        patch_shape_classes=MappingProxyType(dict(patch_shape_classes)),
        loop_signatures=MappingProxyType(dict(loop_signatures)),
        straighten_chain_refs=frozenset(straighten_chain_refs),
        band_spine_data=MappingProxyType(dict(band_spine_data)),
    )


__all__ = [
    "PatchShapeFingerprint",
    "PatchShapeSupportResult",
    "detect_patch_shape_fingerprint",
    "classify_patch_shape_fingerprint",
    "build_patch_shape_support",
]
