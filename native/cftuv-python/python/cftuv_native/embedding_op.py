"""The source-snap embedding certificate as a WHOLE operation: `_embedding._compute_source_snap_embedding_certificate(before, after, faces, intended_corners, unclassifiable_corners, snapping_law)`, native.

The unit the product replaces is the pure leaf the memo wrapper `_embedding.build_source_snap_embedding_certificate` calls at its three places (the wrapper itself, its memo, its
statistics and its key stay Python): exact `Fraction` positions before and after the snap of the source vertices to the grid, the faces of the patch, the corners the host intended to be right
and those it could not classify, the snapping law -> `SourceSnapEmbeddingCertificateV1`. The leaf has no state and no cost accounting (no budget, no canonicalization memory, no counters,
no float), so there is nothing to mirror or to restore: equality is the twelve fields and the one exception the oracle raises, `ValueError("physical edge '<id>' has inconsistent endpoints")`,
which is raised here with the oracle's text.

The extension reads the arguments of the oracle from the objects themselves (exact types only: `dict` of `SourceVertexId -> (x, y, z)`, coordinates `int` or `Fraction`, ids of the exact classes, faces
with `face_id.value`, `vertex_cycle`, `edge_cycle`) and answers the counts; the certificate is built HERE, from the oracle's own class, with its own `__post_init__`. What the port does not carry
(`NativePortUnsupported`, the call changed nothing and has no effect on any state: the caller runs the oracle) is named by the extension: another type, a vertex of `after` that `before` lacks, a vertex
of a face the positions lack (the oracle's `KeyError`).

The classes the answer is made of are resolved from the live kernel on every call (a reloaded kernel is a new set of classes) and their shape is checked when they change (`NativePortStale` naming the file).
"""

from __future__ import annotations

import dataclasses
from fractions import Fraction

from . import pin

OPERATION = "snap_embedding"

#: The fields of `SourceSnapEmbeddingCertificateV1`, in the order the oracle fills them.
CERTIFICATE_FIELDS = (
    "snapping_law",
    "source_vertex_ids",
    "source_vertex_count",
    "source_edge_count",
    "intended_right_corner_count",
    "newly_coincident_vertex_pair_count",
    "collapsed_nonzero_source_edge_count",
    "new_nonadjacent_edge_intersection_count",
    "unclassifiable_source_corner_count",
    "unchanged_unclassifiable_source_corner_count",
    "degenerated_intended_right_corner_count",
    "exact_pair_test_count",
)

#: The status codes of the extension's answer (`embedding.rs`).
STATUS_COUNTS = 0
STATUS_INCONSISTENT = 1
STATUS_DECLINED = 2

_KERNEL: list = []


def _names(cls) -> tuple:
    return tuple(item.name for item in dataclasses.fields(cls))


def _bind(ids, metric, surface) -> tuple:
    """`(vertex id class, edge id class, certificate class)` after checking the classes have the shape the Rust side reads (`NativePortStale` otherwise)."""

    pin.check_shapes(
        (
            ("contracts/metric.py", "SourceSnapEmbeddingCertificateV1 fields", _names(metric.SourceSnapEmbeddingCertificateV1) == CERTIFICATE_FIELDS),
            ("contracts/surface.py", "SourceFaceV1 carries face_id, vertex_cycle and edge_cycle", {"face_id", "vertex_cycle", "edge_cycle"} <= set(_names(surface.SourceFaceV1))),
            ("ids.py", "SourceVertexId is a dataclass of one field `value`", _names(ids.SourceVertexId) == ("value",)),
            ("ids.py", "PhysicalEdgeId is a dataclass of one field `value`", _names(ids.PhysicalEdgeId) == ("value",)),
            ("ids.py", "an id is equal to another id of its own class and value only", ids.SourceVertexId.__dataclass_params__.eq and ids.SourceVertexId.__dataclass_params__.frozen),
            ("fractions.py", "Fraction keeps its parts in _numerator and _denominator", hasattr(Fraction(1, 2), "_numerator") and hasattr(Fraction(1, 2), "_denominator")),
        )
    )
    return ids.SourceVertexId, ids.PhysicalEdgeId, metric.SourceSnapEmbeddingCertificateV1


def _classes() -> tuple:
    from cftuv_envelope import ids
    from cftuv_envelope.contracts import metric, surface

    if _KERNEL and _KERNEL[0][0] is ids and _KERNEL[0][1] is metric:
        return _KERNEL[0][2]
    bound = _bind(ids, metric, surface)
    _KERNEL[:] = [(ids, metric, bound)]
    return bound


def snap_embedding_certificate(core, before, after, faces, intended_corners, unclassifiable_corners, snapping_law):
    """`_embedding._compute_source_snap_embedding_certificate(...)`, whole, answered by the extension function `core` (the shim hands `_core.snap_embedding`).

    Raises `NativePortStale` when the pinned oracle file moved, `NativePortUnsupported` when the port does not carry the call, and `ValueError` with the oracle's text for an inconsistent physical edge.
    """

    pin.require(OPERATION)
    vertex_class, edge_class, certificate_class = _classes()
    answer = core(vertex_class, edge_class, Fraction, before, after, faces, intended_corners, unclassifiable_corners)
    status = answer[0]
    if status == STATUS_COUNTS:
        ids = answer[1]
        return certificate_class(
            snapping_law=snapping_law,
            source_vertex_ids=ids,
            source_vertex_count=len(ids),
            source_edge_count=answer[2],
            intended_right_corner_count=len(intended_corners),
            newly_coincident_vertex_pair_count=answer[3],
            collapsed_nonzero_source_edge_count=answer[4],
            new_nonadjacent_edge_intersection_count=answer[5],
            unclassifiable_source_corner_count=len(unclassifiable_corners),
            unchanged_unclassifiable_source_corner_count=answer[6],
            degenerated_intended_right_corner_count=answer[7],
            exact_pair_test_count=answer[8],
        )
    if status == STATUS_INCONSISTENT:
        raise ValueError(f"physical edge {answer[1]!r} has inconsistent endpoints")
    raise pin.NativePortUnsupported(f"the native `{OPERATION}` port does not carry this call: {answer[1]}")
