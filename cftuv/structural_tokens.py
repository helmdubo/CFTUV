from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Optional

try:
    from .model import (
        BoundaryChain,
        BoundaryCorner,
        BoundaryLoop,
        ChainRef,
        FrameRole,
        PatchNode,
    )
except ImportError:
    from model import (
        BoundaryChain,
        BoundaryCorner,
        BoundaryLoop,
        ChainRef,
        FrameRole,
        PatchNode,
    )


# ============================================================
# Enums
# ============================================================


class ChainRoleClass(str, Enum):
    """Structural role of a chain in its loop. Independent of FrameRole."""

    SIDE = "SIDE"
    CAP = "CAP"
    BORDER = "BORDER"
    FREE = "FREE"


class PatchShapeClass(str, Enum):
    """Shape classification of a patch. Determines operator choice."""

    MIX = "MIX"
    BAND = "BAND"


# ============================================================
# Token dataclasses
# ============================================================


@dataclass(frozen=True)
class ChainToken:
    """Thin structural view of one boundary chain."""

    chain_ref: ChainRef
    role_class: ChainRoleClass
    effective_frame_role: FrameRole     # SUPPORTING SIGNAL ONLY
    length: float                       # 3D arc length
    neighbor_patch_id: Optional[int]    # None if mesh border
    is_border: bool
    opposite_ref: Optional[ChainRef]    # paired opposite chain if detected


@dataclass(frozen=True)
class CornerToken:
    """Thin structural view of one boundary corner."""

    vert_index: int
    turn_angle_deg: float
    prev_chain_ref: ChainRef
    next_chain_ref: ChainRef


@dataclass(frozen=True)
class LoopSignature:
    """Ordered structural view of one boundary loop."""

    patch_id: int
    loop_index: int
    chain_tokens: tuple[ChainToken, ...]
    corner_tokens: tuple[CornerToken, ...]
    chain_count: int
    has_opposite_pairs: bool
    side_count: int
    cap_count: int


# ============================================================
# Internal helpers
# ============================================================


def _compute_chain_arc_length(chain: BoundaryChain) -> float:
    """Compute sum of 3D edge lengths from chain.vert_cos."""
    verts = chain.vert_cos
    if len(verts) < 2:
        return 0.0
    total = 0.0
    for i in range(len(verts) - 1):
        total += (verts[i + 1] - verts[i]).length
    return total


def _chains_share_corner(chain_a: BoundaryChain, chain_b: BoundaryChain) -> bool:
    """Return True if two chains share at least one corner index."""
    corner_indices_a = {
        corner_index
        for corner_index in (chain_a.start_corner_index, chain_a.end_corner_index)
        if corner_index >= 0
    }
    corner_indices_b = {
        corner_index
        for corner_index in (chain_b.start_corner_index, chain_b.end_corner_index)
        if corner_index >= 0
    }
    return bool(corner_indices_a & corner_indices_b)


def _find_opposite_ref(
    chain_index: int,
    chain: BoundaryChain,
    loop: BoundaryLoop,
    patch_id: int,
    loop_index: int,
) -> Optional[ChainRef]:
    """Find the best opposite chain for the given chain in the same loop.

    Looks for the longest chain in the same loop with the same frame_role
    that does not share a corner with the given chain.
    Returns its ChainRef, or None if no candidate is found.
    """
    role = chain.frame_role
    if role not in {FrameRole.H_FRAME, FrameRole.V_FRAME}:
        return None

    best_ref: Optional[ChainRef] = None
    best_length = -1.0

    for other_index, other_chain in enumerate(loop.chains):
        if other_index == chain_index:
            continue
        if other_chain.frame_role != role:
            continue
        if _chains_share_corner(chain, other_chain):
            continue
        other_length = _compute_chain_arc_length(other_chain)
        if other_length > best_length:
            best_length = other_length
            best_ref = (patch_id, loop_index, other_index)

    return best_ref


# ============================================================
# Builder
# ============================================================


def build_loop_signature(
    patch_id: int,
    loop_index: int,
    loop: BoundaryLoop,
    node: PatchNode,
) -> LoopSignature:
    """Construct a frozen LoopSignature from a BoundaryLoop and its PatchNode."""

    # ------------------------------------------------------------------
    # Pass 1: compute per-chain data needed for opposite detection.
    # ------------------------------------------------------------------
    chain_count = len(loop.chains)

    chain_lengths: list[float] = []
    neighbor_patch_ids: list[Optional[int]] = []
    is_borders: list[bool] = []

    for chain in loop.chains:
        chain_lengths.append(_compute_chain_arc_length(chain))
        is_border = not chain.has_patch_neighbor and chain.neighbor_patch_id == -1
        is_borders.append(is_border)
        neighbor_patch_ids.append(
            chain.neighbor_patch_id if chain.has_patch_neighbor else None
        )

    # ------------------------------------------------------------------
    # Pass 2: detect opposite pairs.
    # ------------------------------------------------------------------
    opposite_refs: list[Optional[ChainRef]] = []
    for chain_index, chain in enumerate(loop.chains):
        opposite_refs.append(
            _find_opposite_ref(chain_index, chain, loop, patch_id, loop_index)
        )

    # ------------------------------------------------------------------
    # Pass 3: assign role_class.
    # A chain is SIDE if it has an opposite_ref and is the longer of the
    # mutual pair (or equal length).  If both sides of a pair are found
    # to reference each other, both are classified SIDE.
    # A chain is CAP if it has no opposite but another chain in the loop
    # is SIDE and this chain is shorter than the mean SIDE length.
    # Otherwise FREE.
    # BORDER overrides everything.
    # ------------------------------------------------------------------
    side_set: set[int] = set()
    for chain_index in range(chain_count):
        opp_ref = opposite_refs[chain_index]
        if opp_ref is None:
            continue
        _, _, opp_index = opp_ref
        # Both members of a detected pair are SIDEs.
        side_set.add(chain_index)
        side_set.add(opp_index)

    role_classes: list[ChainRoleClass] = []
    for chain_index in range(chain_count):
        if is_borders[chain_index]:
            role_classes.append(ChainRoleClass.BORDER)
        elif chain_index in side_set:
            role_classes.append(ChainRoleClass.SIDE)
        elif side_set:
            # Non-border, non-side chain in a loop that has sides → CAP
            role_classes.append(ChainRoleClass.CAP)
        else:
            role_classes.append(ChainRoleClass.FREE)

    # ------------------------------------------------------------------
    # Build ChainTokens (frozen).
    # ------------------------------------------------------------------
    chain_tokens: list[ChainToken] = []
    for chain_index, chain in enumerate(loop.chains):
        chain_ref: ChainRef = (patch_id, loop_index, chain_index)
        chain_tokens.append(
            ChainToken(
                chain_ref=chain_ref,
                role_class=role_classes[chain_index],
                effective_frame_role=chain.frame_role,
                length=chain_lengths[chain_index],
                neighbor_patch_id=neighbor_patch_ids[chain_index],
                is_border=is_borders[chain_index],
                opposite_ref=opposite_refs[chain_index],
            )
        )

    # ------------------------------------------------------------------
    # Build CornerTokens (frozen).
    # ------------------------------------------------------------------
    corner_tokens: list[CornerToken] = []
    for corner in loop.corners:
        prev_chain_index = corner.prev_chain_index
        next_chain_index = corner.next_chain_index
        prev_ref: ChainRef = (patch_id, loop_index, prev_chain_index)
        next_ref: ChainRef = (patch_id, loop_index, next_chain_index)
        corner_tokens.append(
            CornerToken(
                vert_index=corner.vert_index,
                turn_angle_deg=corner.turn_angle_deg,
                prev_chain_ref=prev_ref,
                next_chain_ref=next_ref,
            )
        )

    # ------------------------------------------------------------------
    # Derived counts.
    # ------------------------------------------------------------------
    has_opposite_pairs = bool(side_set)
    side_count = sum(1 for rc in role_classes if rc == ChainRoleClass.SIDE)
    cap_count = sum(1 for rc in role_classes if rc == ChainRoleClass.CAP)

    return LoopSignature(
        patch_id=patch_id,
        loop_index=loop_index,
        chain_tokens=tuple(chain_tokens),
        corner_tokens=tuple(corner_tokens),
        chain_count=chain_count,
        has_opposite_pairs=has_opposite_pairs,
        side_count=side_count,
        cap_count=cap_count,
    )


# ============================================================
# Shape classifier
# ============================================================

_BAND_SIDE_CAP_RATIO_THRESHOLD = 1.5


def classify_patch_shape(signatures: list[LoopSignature]) -> PatchShapeClass:
    """Classify patch shape from its loop signatures.

    Uses only the primary (outer) loop — signatures[0].
    Classification is strict: under-classify (MIX) rather than false-positive BAND.
    """
    if not signatures:
        return PatchShapeClass.MIX

    sig = signatures[0]

    if not sig.chain_tokens:
        return PatchShapeClass.MIX

    # Rule 1: exactly 4 boundary chains
    if sig.chain_count != 4:
        return PatchShapeClass.MIX

    # Rule 2: exactly 2 SIDE chains
    if sig.side_count != 2:
        return PatchShapeClass.MIX

    # Rule 3: exactly 2 CAP chains
    if sig.cap_count != 2:
        return PatchShapeClass.MIX

    sides = [t for t in sig.chain_tokens if t.role_class == ChainRoleClass.SIDE]
    caps = [t for t in sig.chain_tokens if t.role_class == ChainRoleClass.CAP]

    # Rule 4: both SIDE chains share the same effective_frame_role
    if sides[0].effective_frame_role != sides[1].effective_frame_role:
        return PatchShapeClass.MIX

    # Rule 5: mutual pairing — each SIDE's opposite_ref points to the other SIDE
    if not (
        sides[0].opposite_ref == sides[1].chain_ref
        and sides[1].opposite_ref == sides[0].chain_ref
    ):
        return PatchShapeClass.MIX

    # Rule 6: side/cap length ratio > threshold
    avg_side = (sides[0].length + sides[1].length) / 2.0
    avg_cap = (caps[0].length + caps[1].length) / 2.0
    if avg_side / max(avg_cap, 1e-9) <= _BAND_SIDE_CAP_RATIO_THRESHOLD:
        return PatchShapeClass.MIX

    return PatchShapeClass.BAND
