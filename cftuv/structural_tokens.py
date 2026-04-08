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
    effective_frame_role: FrameRole     # H/V/STRAIGHTEN/FREE
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


def _pair_length_similarity(len_a: float, len_b: float) -> float:
    """Return similarity in [0, 1].  1.0 = identical lengths, 0.0 = maximally different."""
    max_len = max(len_a, len_b)
    if max_len <= 1e-9:
        return 1.0
    return 1.0 - abs(len_a - len_b) / max_len


# ============================================================
# Builder
# ============================================================

# Threshold: CAP pair similarity must be >= this to qualify as BAND.
# 0.5 means CAP lengths may differ by at most 50 %.
_CAP_SIMILARITY_THRESHOLD = 0.5


def build_loop_signature(
    patch_id: int,
    loop_index: int,
    loop: BoundaryLoop,
    node: PatchNode,
) -> LoopSignature:
    """Construct a frozen LoopSignature from a BoundaryLoop and its PatchNode."""

    # ------------------------------------------------------------------
    # Pass 1: compute per-chain data.
    # ------------------------------------------------------------------
    chain_count = len(loop.chains)

    chain_lengths: list[float] = []
    chain_frame_roles: list[FrameRole] = []
    neighbor_patch_ids: list[Optional[int]] = []
    is_borders: list[bool] = []

    for chain in loop.chains:
        chain_lengths.append(_compute_chain_arc_length(chain))
        chain_frame_roles.append(chain.frame_role)
        is_border = not chain.has_patch_neighbor and chain.neighbor_patch_id == -1
        is_borders.append(is_border)
        neighbor_patch_ids.append(
            chain.neighbor_patch_id if chain.has_patch_neighbor else None
        )

    # ------------------------------------------------------------------
    # Pass 2 + 3: detect opposite pairs and assign role_class.
    #
    # For 4-chain loops: two non-adjacent pairs (0,2) and (1,3).
    # SIDE = the pair where BOTH chains are FREE.
    # CAP  = the other pair.
    # If both pairs are FREE-FREE → pick the pair with higher internal
    # similarity as CAP (since CAPs must have similar lengths).
    # If neither pair is FREE-FREE → no BAND candidate, all get FREE.
    #
    # For non-4-chain loops: fall back to role-based detection.
    # ------------------------------------------------------------------
    side_set: set[int] = set()
    cap_set: set[int] = set()
    opposite_refs: list[Optional[ChainRef]] = [None] * chain_count

    if chain_count == 4:
        pair_a = (0, 2)
        pair_b = (1, 3)

        a_share = _chains_share_corner(loop.chains[0], loop.chains[2])
        b_share = _chains_share_corner(loop.chains[1], loop.chains[3])

        a_both_free = (
            chain_frame_roles[0] == FrameRole.FREE
            and chain_frame_roles[2] == FrameRole.FREE
        )
        b_both_free = (
            chain_frame_roles[1] == FrameRole.FREE
            and chain_frame_roles[3] == FrameRole.FREE
        )

        side_pair: Optional[tuple[int, int]] = None
        cap_pair: Optional[tuple[int, int]] = None

        if not a_share and not b_share:
            # Both pairs are valid non-adjacent pairs.
            if a_both_free and not b_both_free:
                # Only pair_a is both-FREE → SIDE.
                side_pair, cap_pair = pair_a, pair_b
            elif b_both_free and not a_both_free:
                # Only pair_b is both-FREE → SIDE.
                side_pair, cap_pair = pair_b, pair_a
            elif a_both_free and b_both_free:
                # Both pairs are FREE-FREE. Pick the pair with HIGHER
                # internal similarity as CAP (CAPs should be similar).
                sim_a = _pair_length_similarity(chain_lengths[0], chain_lengths[2])
                sim_b = _pair_length_similarity(chain_lengths[1], chain_lengths[3])
                if sim_a >= sim_b:
                    # pair_a is more similar → CAP.
                    side_pair, cap_pair = pair_b, pair_a
                else:
                    side_pair, cap_pair = pair_a, pair_b
            # else: neither pair is both-FREE → no SIDE/CAP assignment.

        elif not a_share and a_both_free:
            side_pair = pair_a
        elif not b_share and b_both_free:
            side_pair = pair_b

        if side_pair is not None:
            side_set.update(side_pair)
            opposite_refs[side_pair[0]] = (patch_id, loop_index, side_pair[1])
            opposite_refs[side_pair[1]] = (patch_id, loop_index, side_pair[0])
        if cap_pair is not None:
            cap_set.update(cap_pair)
            opposite_refs[cap_pair[0]] = (patch_id, loop_index, cap_pair[1])
            opposite_refs[cap_pair[1]] = (patch_id, loop_index, cap_pair[0])

    else:
        # Non-4-chain loops: no BAND candidate, simple opposite detection.
        for chain_index, chain in enumerate(loop.chains):
            opp = _find_opposite_ref(chain_index, chain, loop, patch_id, loop_index)
            opposite_refs[chain_index] = opp

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

    # ------------------------------------------------------------------
    # Assign role_class and effective_frame_role per chain.
    # SIDE chains in a 4-chain loop get STRAIGHTEN as effective_frame_role.
    # ------------------------------------------------------------------
    role_classes: list[ChainRoleClass] = []
    effective_roles: list[FrameRole] = []

    for chain_index in range(chain_count):
        if is_borders[chain_index]:
            role_classes.append(ChainRoleClass.BORDER)
            effective_roles.append(chain_frame_roles[chain_index])
        elif chain_index in side_set:
            role_classes.append(ChainRoleClass.SIDE)
            # SIDE chains of a 4-chain loop are STRAIGHTEN candidates.
            if chain_count == 4 and chain_frame_roles[chain_index] == FrameRole.FREE:
                effective_roles.append(FrameRole.STRAIGHTEN)
            else:
                effective_roles.append(chain_frame_roles[chain_index])
        elif chain_index in cap_set or side_set:
            role_classes.append(ChainRoleClass.CAP)
            effective_roles.append(chain_frame_roles[chain_index])
        else:
            role_classes.append(ChainRoleClass.FREE)
            effective_roles.append(chain_frame_roles[chain_index])

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
                effective_frame_role=effective_roles[chain_index],
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


def _find_opposite_ref(
    chain_index: int,
    chain: BoundaryChain,
    loop: BoundaryLoop,
    patch_id: int,
    loop_index: int,
) -> Optional[ChainRef]:
    """Find the best opposite chain for non-4-chain loops (role-based)."""
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
# Shape classifier
# ============================================================


def classify_patch_shape(signatures: list[LoopSignature], _debug_patch_id: int = -1) -> PatchShapeClass:
    """Classify patch shape from its loop signatures.

    Uses only the primary (outer) loop — signatures[0].

    BAND rules:
      1. Exactly 4 boundary chains.
      2. Exactly 2 SIDE chains (both FREE → STRAIGHTEN).
      3. Exactly 2 CAP chains.
      4. Both SIDE chains must be FREE (now STRAIGHTEN). H/V can never be SIDE.
      5. Mutual opposite pairing between SIDEs.
      6. CAP length similarity >= threshold (band doesn't diverge).
    """
    if not signatures:
        return PatchShapeClass.MIX

    sig = signatures[0]
    if not sig.chain_tokens:
        return PatchShapeClass.MIX

    # Diagnostic
    if sig.chain_count == 4:
        roles = [(t.role_class.value, t.effective_frame_role.value, f"len={t.length:.3f}") for t in sig.chain_tokens]
        print(f"[CFTUV][Classify] P{_debug_patch_id} 4-chain: side={sig.side_count} cap={sig.cap_count} {roles}")

    # Rule 1: exactly 4 chains
    if sig.chain_count != 4:
        return PatchShapeClass.MIX

    # Rule 2: exactly 2 SIDE chains
    if sig.side_count != 2:
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL: side_count={sig.side_count} != 2")
        return PatchShapeClass.MIX

    # Rule 3: exactly 2 CAP chains
    if sig.cap_count != 2:
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL: cap_count={sig.cap_count} != 2")
        return PatchShapeClass.MIX

    sides = [t for t in sig.chain_tokens if t.role_class == ChainRoleClass.SIDE]
    caps = [t for t in sig.chain_tokens if t.role_class == ChainRoleClass.CAP]

    # Rule 4: both SIDE chains must have STRAIGHTEN effective_frame_role
    # (set during build when both are FREE). H/V chains can never be SIDE.
    for s in sides:
        if s.effective_frame_role != FrameRole.STRAIGHTEN:
            print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL rule4: SIDE {s.chain_ref} role={s.effective_frame_role.value} (must be STRAIGHTEN)")
            return PatchShapeClass.MIX

    # Rule 5: mutual opposite pairing
    if not (
        sides[0].opposite_ref == sides[1].chain_ref
        and sides[1].opposite_ref == sides[0].chain_ref
    ):
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL rule5: not mutual opposite")
        return PatchShapeClass.MIX

    # Rule 6: CAP length similarity — band must not diverge
    cap_similarity = _pair_length_similarity(caps[0].length, caps[1].length)
    if cap_similarity < _CAP_SIMILARITY_THRESHOLD:
        print(f"[CFTUV][Classify] P{_debug_patch_id} FAIL rule6: cap_similarity={cap_similarity:.2f} < {_CAP_SIMILARITY_THRESHOLD}")
        return PatchShapeClass.MIX

    print(f"[CFTUV][Classify] P{_debug_patch_id} → BAND (cap_sim={cap_similarity:.2f})")
    return PatchShapeClass.BAND
