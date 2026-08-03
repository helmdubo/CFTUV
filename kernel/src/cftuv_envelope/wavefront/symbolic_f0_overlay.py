"""Sparse symbolic overlay for the frozen pre-contact state F0."""

from __future__ import annotations

from .superlevel_closure import SymbolicMaterializationPlanV1
from .symbolic_overlay import (
    SymbolicOverlayV1,
    SymbolicSpanBindingV1,
    _initial_vertices,
    _leaf_bindings,
    refreshed_span_bindings,
)
from .symbolic_sparse_ports import with_line_ports


def build_f0_overlay(builder, snapshot, time):
    """Admit only queried exact occurrences plus their line-only ports."""

    snapshot = with_line_ports(builder, snapshot, time)
    if snapshot is None:
        return None
    materialization = SymbolicMaterializationPlanV1((), (), ())
    leaves, physical = _leaf_bindings(snapshot, materialization)
    if leaves is None:
        return None
    vertices, _ = _initial_vertices(
        builder, snapshot, materialization, leaves
    )
    if vertices is None:
        return None
    spans = {
        leaf: SymbolicSpanBindingV1(
            leaf, physical[occurrence], None, None
        )
        for occurrence, leaf in leaves.items()
    }
    overlay = SymbolicOverlayV1(vertices, spans, set(), time)
    bindings, reason = refreshed_span_bindings(overlay)
    if reason is not None:
        return None
    overlay.spans = bindings
    return overlay
