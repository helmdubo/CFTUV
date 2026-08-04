"""Exact affine classification of spans born by a superlevel transaction."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from fractions import Fraction

from .exact_candidate_view import collapsing_span
from .sqrt_sum import SqrtSumV1


class PoststateSpanDisposition(str, Enum):
    """Знак ориентированной длины сразу после рождения смежности."""

    OPENING = "OPENING"
    CLOSING_WITH_FUTURE_EVENT = "CLOSING_WITH_FUTURE_EVENT"
    INVERTED = "INVERTED"
    NONPOINT_JUNCTION_OUTSIDE_LAW = "NONPOINT_JUNCTION_OUTSIDE_LAW"
    AFFINE_CLASSIFICATION_UNPROVEN = "AFFINE_CLASSIFICATION_UNPROVEN"


@dataclass(frozen=True, slots=True)
class PoststateSpanClassificationV1:
    disposition: PoststateSpanDisposition
    birth_length: SqrtSumV1 | None
    slope: SqrtSumV1 | None
    orientation_sign: int | None


def _vertex_velocity(view, vertex_ref):
    """Exact ``dp/dt`` from support lines, including a sliding vertex."""

    vertex = view.vertex_state(vertex_ref)
    first = view.span_state(vertex.prev_span).line
    second = view.span_state(vertex.next_span).line
    determinant = first.a * second.b - second.a * first.b
    if determinant:
        scale = Fraction(1, determinant)
        return (
            (
                SqrtSumV1.radical(second.b, first.q)
                - SqrtSumV1.radical(first.b, second.q)
            ).scaled(scale),
            (
                SqrtSumV1.radical(first.a, second.q)
                - SqrtSumV1.radical(second.a, first.q)
            ).scaled(scale),
        )
    if vertex.sliding is None:
        return None
    scale = Fraction(1, first.normal_squared)
    return (
        SqrtSumV1.radical(first.a, first.q).scaled(scale),
        SqrtSumV1.radical(first.b, first.q).scaled(scale),
    )


def _span_orientation(view, span_ref):
    """Canonical segment order, independent of runtime vertex identities."""

    span = view.span_state(span_ref)
    occurrence = getattr(span_ref, "occurrence", None)
    if occurrence is not None and len(occurrence) == 3:
        start_x, start_y = (SqrtSumV1(item) for item in occurrence[1])
        end_x, end_y = (SqrtSumV1(item) for item in occurrence[2])
        direction = (
            end_x.scaled(span.line.b)
            - end_y.scaled(span.line.a)
            - start_x.scaled(span.line.b)
            + start_y.scaled(span.line.a)
        )
        sign = direction.sign()
        if sign:
            return sign
    x0, y0, x1, y1 = span.source_span
    source_direction = (
        (x1 - x0) * span.line.b - (y1 - y0) * span.line.a
    )
    return (source_direction > 0) - (source_direction < 0)


def classify_poststate_span(view, vertex_ref, peer_ref, birth_time):
    """Classify one newborn adjacency from ``length(birth)`` and its slope.

    A vertex position is affine in time because it solves two affine support
    lines.  Projection onto the shared support is therefore affine too, and
    so is the oriented span length ``high - low``.  Its derivative is obtained
    directly from support velocities: no event time, historical or invented,
    participates in the predicate.

    Positive length with a negative slope has one exact zero in the future.
    Zero length with a positive slope opens from the birth point.  Negative
    length, or zero followed by a negative slope, is inverted.  Identically
    zero length is ambiguous and must remain fail-closed.
    """

    vertex = view.vertex_state(vertex_ref)
    peer = view.vertex_state(peer_ref)
    shared = vertex.next_span
    if peer.prev_span != shared:
        return PoststateSpanClassificationV1(
            PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN,
            None,
            None,
            None,
        )
    birth_length = collapsing_span(
        view, vertex_ref, peer_ref, birth_time
    )
    low_velocity = _vertex_velocity(view, vertex_ref)
    high_velocity = _vertex_velocity(view, peer_ref)
    if (
        birth_length is None
        or low_velocity is None
        or high_velocity is None
    ):
        return PoststateSpanClassificationV1(
            PoststateSpanDisposition.NONPOINT_JUNCTION_OUTSIDE_LAW,
            birth_length,
            None,
            None,
        )
    line = view.span_state(shared).line
    slope = (
        high_velocity[0].scaled(line.b)
        - high_velocity[1].scaled(line.a)
        - low_velocity[0].scaled(line.b)
        + low_velocity[1].scaled(line.a)
    )
    orientation_sign = _span_orientation(view, shared)
    if orientation_sign == 0:
        return PoststateSpanClassificationV1(
            PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN,
            birth_length,
            slope,
            None,
        )
    birth_length = birth_length.scaled(orientation_sign)
    slope = slope.scaled(orientation_sign)
    length_sign = birth_length.sign()
    slope_sign = slope.sign()
    if length_sign < 0 or (length_sign == 0 and slope_sign < 0):
        disposition = PoststateSpanDisposition.INVERTED
    elif length_sign == 0 and slope_sign == 0:
        disposition = (
            PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN
        )
    elif length_sign > 0 and slope_sign < 0:
        disposition = (
            PoststateSpanDisposition.CLOSING_WITH_FUTURE_EVENT
        )
    else:
        disposition = PoststateSpanDisposition.OPENING
    return PoststateSpanClassificationV1(
        disposition,
        birth_length,
        slope,
        orientation_sign,
    )
