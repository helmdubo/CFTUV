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
    # Pass 2 + 3: detect opposite pairs and assign role_class.
    #
    # Strategy: purely topological/geometric, independent of FrameRole.
    # In a 4-chain loop there are exactly 2 non-adjacent pairs: (0,2)
    # and (1,3).  The pair with the greater combined arc length is SIDE,
    # the other is CAP.  This works for both H/V and FREE chains.
    #
    # For loops with != 4 chains, fall back to role-based detection.
    # ------------------------------------------------------------------
    side_set: set[int] = set()
    cap_set: set[int] = set()
    opposite_refs: list[Optional[ChainRef]] = [None] * chain_count

    if chain_count == 4:
        # Two non-adjacent pairs in a 4-chain loop.
        pair_a = (0, 2)
        pair_b = (1, 3)
        len_a = chain_lengths[0] + chain_lengths[2]
        len_b = chain_lengths[1] + chain_lengths[3]

        # Also verify non-adjacency via corners (should always hold for
        # a proper 4-chain loop, but check defensively).
        a_share = _chains_share_corner(loop.chains[0], loop.chains[2])
        b_share = _chains_share_corner(loop.chains[1], loop.chains[3])

        if not a_share and not b_share:
            # Both pairs are valid non-adjacent pairs.
            if len_a >= len_b:
                side_pair, cap_pair = pair_a, pair_b
            else:
                side_pair, cap_pair = pair_b, pair_a
            side_set.update(side_pair)
            cap_set.update(cap_pair)
            # Set opposite refs for both pairs.
            opposite_refs[side_pair[0]] = (patch_id, loop_index, side_pair[1])
            opposite_refs[side_pair[1]] = (patch_id, loop_index, side_pair[0])
            opposite_refs[cap_pair[0]] = (patch_id, loop_index, cap_pair[1])
            opposite_refs[cap_pair[1]] = (patch_id, loop_index, cap_pair[0])
        elif not a_share:
            # Only pair_a is non-adjacent.
            side_set.update(pair_a)
            opposite_refs[pair_a[0]] = (patch_id, loop_index, pair_a[1])
            opposite_refs[pair_a[1]] = (patch_id, loop_index, pair_a[0])
        elif not b_share:
            # Only pair_b is non-adjacent.
            side_set.update(pair_b)
            opposite_refs[pair_b[0]] = (patch_id, loop_index, pair_b[1])
            opposite_refs[pair_b[1]] = (patch_id, loop_index, pair_b[0])
    else:
        # For non-4-chain loops, use role-based detection (original logic).
        for chain_index, chain in enumerate(loop.chains):
            opp = _find_opposite_ref(chain_index, chain, loop, patch_id, loop_index)
            opposite_refs[chain_index] = opp

        # Collect mutual pairs, pick longest as SIDE.
        seen: set[int] = set()
        opposite_pairs: list[tuple[int, int]] = []
        for chain_index in range(chain_count):
            if chain_index in seen:
                continue
            opp_ref = opposite_refs[chain_index]
            if opp_ref is None:
                continue
            _, _, opp_index = opp_ref
            other_opp = opposite_refs[opp_index]
            if other_opp is not None and other_opp[2] == chain_index:
                opposite_pairs.append((chain_index, opp_index))
                seen.add(chain_index)
                seen.add(opp_index)
        if opposite_pairs:
            best_pair = max(
                opposite_pairs,
                key=lambda p: chain_lengths[p[0]] + chain_lengths[p[1]],
            )
            side_set.add(best_pair[0])
            side_set.add(best_pair[1])

    role_classes: list[ChainRoleClass] = []
    for chain_index in range(chain_count):
        if is_borders[chain_index]:
            role_classes.append(ChainRoleClass.BORDER)
        elif chain_index in side_set:
            role_classes.append(ChainRoleClass.SIDE)
        elif chain_index in cap_set or side_set:
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


def classify_patch_shape(signatures: list[LoopSignature], _debug_patch_id: int = -1) -> PatchShapeClass:
    """Classify patch shape from its loop signatures.

    Uses only the primary (outer) loop — signatures[0].
    Classification is strict: under-classify (MIX) rather than false-positive BAND.
    """
    if not signatures:
        return PatchShapeClass.MIX

    sig = signatures[0]

    if not sig.chain_tokens:
        return PatchShapeClass.MIX

    # Diagnostic: show what the classifier sees for 4-chain loops
    if sig.chain_count == 4:
        roles = [(t.role_class.value, t.effective_frame_role.value, f"len={t.length:.3f}", f"opp={t.opposite_ref}") for t in sig.chain_tokens]
        print(f"[CFTUV][Classify] P{_debug_patch_id} 4-chain loop: side={sig.side_count} cap={sig.cap_count} tokens={roles}")

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

    # Rule 4: both SIDE chains share the same effective_frame_role,
    # OR both are FREE (structurally band-shaped but no H/V assignment yet).
    side_roles = {sides[0].effective_frame_role, sides[1].effective_frame_role}
    if len(side_roles) > 1 and FrameRole.FREE not in side_roles:
        # Different H/V roles on SIDEs and neither is FREE — not a band.
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL rule4: roles differ {sides[0].effective_frame_role} vs {sides[1].effective_frame_role}")
        return PatchShapeClass.MIX

    # Rule 5: mutual pairing — each SIDE's opposite_ref points to the other SIDE
    if not (
        sides[0].opposite_ref == sides[1].chain_ref
        and sides[1].opposite_ref == sides[0].chain_ref
    ):
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL rule5: not mutual opp={sides[0].opposite_ref} vs {sides[1].chain_ref}")
        return PatchShapeClass.MIX

    # Rule 6: side/cap length ratio > threshold
    avg_side = (sides[0].length + sides[1].length) / 2.0
    avg_cap = (caps[0].length + caps[1].length) / 2.0
    ratio = avg_side / max(avg_cap, 1e-9)
    if ratio <= _BAND_SIDE_CAP_RATIO_THRESHOLD:
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL rule6: ratio={ratio:.2f} <= {_BAND_SIDE_CAP_RATIO_THRESHOLD}")
        return PatchShapeClass.MIX

    # Rule 7: skip patches where ALL chains are already H/V.
    # The generic frontier already builds a perfect rectified scaffold for
    # these — band_operator adds no value and breaks pinning integration.
    all_hv = all(
        t.effective_frame_role in (FrameRole.H_FRAME, FrameRole.V_FRAME)
        for t in sig.chain_tokens
    )
    if all_hv:
        print(f"[CFTUV][Classify] P{_debug_patch_id} SKIP: all chains already H/V, generic frontier sufficient")
        return PatchShapeClass.MIX

    print(f"[CFTUV][Classify] P{_debug_patch_id} → BAND (ratio={ratio:.2f})")
    return PatchShapeClass.BAND
