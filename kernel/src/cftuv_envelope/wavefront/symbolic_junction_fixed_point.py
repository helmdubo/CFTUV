"""Order-free mixed EDGE/endpoint junction generations on one overlay."""

from __future__ import annotations

from dataclasses import dataclass

from .symbolic_component import clone_overlay, overlay_signature
from .symbolic_junction_contacts import (
    SymbolicJunctionContactV1,
    contact_identity as _identity,
    discover_junction_contacts,
    edge_contact as _edge_contact,
    endpoint_contact as _endpoint_contact,
    initial_junction_seeds,
)
from .symbolic_junction_normalize import (
    apply_junction_generation,
    normalize_junction_generation as _normalize,
)
from .symbolic_overlay import SymbolicOverlayV1


@dataclass(frozen=True, slots=True)
class SymbolicJunctionGenerationV1:
    contacts: tuple[SymbolicJunctionContactV1, ...]
    deltas: tuple


@dataclass(frozen=True, slots=True)
class SymbolicJunctionFixedPointV1:
    generations: tuple[SymbolicJunctionGenerationV1, ...]
    overlay: SymbolicOverlayV1 | None
    signatures: tuple[tuple, ...]
    unresolved_reason: str | None = None


def normalize_junction_generation(builder, overlay, contacts):
    return _normalize(
        builder, overlay, contacts, SymbolicJunctionGenerationV1
    )


def _rebuild(builder, initial, causal):
    overlay, generations, signatures = clone_overlay(initial), [], []
    for contacts in causal:
        generation, reason = normalize_junction_generation(
            builder, overlay, contacts
        )
        if reason is not None:
            return None, tuple(generations), tuple(signatures), reason
        overlay, reason = apply_junction_generation(overlay, generation)
        if reason is not None:
            return None, tuple(generations), tuple(signatures), reason
        generations.append(generation)
        signatures.append(overlay_signature(overlay))
    return overlay, tuple(generations), tuple(signatures), None


def plan_symbolic_junction_generations(builder, initial, *, seeds=(), budget):
    causal = []
    for iteration in range(budget + 1):
        overlay, generations, signatures, reason = _rebuild(
            builder, initial, causal
        )
        if reason is not None:
            return SymbolicJunctionFixedPointV1(generations, None, signatures, reason)
        discovered, reason = discover_junction_contacts(builder, overlay)
        if reason is not None:
            return SymbolicJunctionFixedPointV1(generations, overlay, signatures, reason)
        candidates = (*((seeds) if iteration == 0 else ()), *discovered)
        unique = {}
        for item in candidates:
            identity = _identity(item)
            previous = unique.get(identity)
            if previous is not None and previous != item:
                return SymbolicJunctionFixedPointV1(
                    generations,
                    overlay,
                    signatures,
                    "SYMBOLIC_JUNCTION_CONTACT_METADATA_CONFLICT",
                )
            unique[identity] = item
        contacts = tuple(sorted(
            unique.values(), key=lambda item: repr(_identity(item))
        ))
        if not contacts:
            return SymbolicJunctionFixedPointV1(generations, overlay, signatures)
        generation, reason = normalize_junction_generation(
            builder, overlay, contacts
        )
        if reason is not None:
            return SymbolicJunctionFixedPointV1(generations, overlay, signatures, reason)
        if iteration == budget:
            return SymbolicJunctionFixedPointV1(
                generations, overlay, signatures,
                "SYMBOLIC_JUNCTION_GENERATION_BUDGET_EXHAUSTED",
            )
        causal.append(generation.contacts)
    raise AssertionError("unreachable symbolic junction generation loop")
