//! The exact affine classification of a span born by a superlevel transaction (`wavefront/poststate_span.py`).
//!
//! A vertex position is affine in time (it solves two affine support lines), so the projection of the two ends on the shared support is too, and so is the
//! oriented length `high - low`. Its derivative comes straight from the support velocities: no event time, historical or invented, takes part in the predicate.
//! A positive length with a negative slope has one exact zero in the future; zero length with a positive slope opens from the birth point; a negative length, or
//! zero followed by a negative slope, is inverted; an identically zero length is ambiguous and stays fail-closed.
//!
//! The order of the exact questions is the oracle's: the shared length first (both ends), then the velocity of the low vertex (four radicals, or two for a sliding
//! vertex on one line), then the high one's, then the orientation of the span (a sign of the exact end points when the reference carries them), then the two
//! signs, the length first.

use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SqrtSum, SIGN_FILTER_BITS};

use crate::error::{SkelError, SkelResult};
use crate::time::EventTime;
use crate::view::{collapsing_span, unpack_source_span, CandidateView, PositionMemo, SpanOccurrence, SpanRef, VertexRef};

/// `PoststateSpanDisposition`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PoststateDisposition {
    Opening,
    ClosingWithFutureEvent,
    Inverted,
    NonpointJunctionOutsideLaw,
    AffineClassificationUnproven,
}

impl PoststateDisposition {
    pub fn value(self) -> &'static str {
        match self {
            PoststateDisposition::Opening => "OPENING",
            PoststateDisposition::ClosingWithFutureEvent => "CLOSING_WITH_FUTURE_EVENT",
            PoststateDisposition::Inverted => "INVERTED",
            PoststateDisposition::NonpointJunctionOutsideLaw => "NONPOINT_JUNCTION_OUTSIDE_LAW",
            PoststateDisposition::AffineClassificationUnproven => "AFFINE_CLASSIFICATION_UNPROVEN",
        }
    }

    pub const ALL: [PoststateDisposition; 5] = [
        PoststateDisposition::Opening,
        PoststateDisposition::ClosingWithFutureEvent,
        PoststateDisposition::Inverted,
        PoststateDisposition::NonpointJunctionOutsideLaw,
        PoststateDisposition::AffineClassificationUnproven,
    ];
}

/// `PoststateSpanClassificationV1`.
#[derive(Debug, Clone)]
pub struct PoststateClassification {
    pub disposition: PoststateDisposition,
    pub birth_length: Option<SqrtSum>,
    pub slope: Option<SqrtSum>,
    pub orientation_sign: Option<i8>,
}

fn classified(disposition: PoststateDisposition, birth_length: Option<SqrtSum>, slope: Option<SqrtSum>, orientation_sign: Option<i8>) -> PoststateClassification {
    PoststateClassification { disposition, birth_length, slope, orientation_sign }
}

/// `_vertex_velocity(view, vertex)`: the exact `dp/dt` from the two support lines, a sliding vertex on one line included; `None` for a vertex that neither crosses
/// nor slides.
pub fn vertex_velocity<V: CandidateView>(ctx: &mut ExactCtx<'_>, view: &V, vertex_ref: VertexRef) -> SkelResult<Option<(SqrtSum, SqrtSum)>> {
    let vertex = view.vertex_state(vertex_ref)?;
    let first = view.span_state(vertex.prev_span)?.line;
    let second = view.span_state(vertex.next_span)?.line;
    let determinant = first.determinant(second);
    let coefficient = |value: i64| Rat::from_i64(value);
    if determinant != 0 {
        let scale = Rat::new(IBig::ONE, IBig::from(determinant)).map_err(|_| SkelError::Unsupported("a zero determinant".to_string()))?;
        let a = exact::radical(ctx, &coefficient(second.b), &first.q)?;
        let b = exact::radical(ctx, &coefficient(first.b), &second.q)?;
        let x = a.sub(&b).scaled(&scale);
        let c = exact::radical(ctx, &coefficient(first.a), &second.q)?;
        let d = exact::radical(ctx, &coefficient(second.a), &first.q)?;
        let y = c.sub(&d).scaled(&scale);
        return Ok(Some((x, y)));
    }
    if vertex.sliding.is_none() {
        return Ok(None);
    }
    let scale = Rat::new(IBig::ONE, IBig::from(first.normal_squared())).map_err(|_| SkelError::Unsupported("a zero normal".to_string()))?;
    let x = exact::radical(ctx, &coefficient(first.a), &first.q)?.scaled(&scale);
    let y = exact::radical(ctx, &coefficient(first.b), &first.q)?.scaled(&scale);
    Ok(Some((x, y)))
}

/// `_span_orientation(view, span)`: the canonical order of the segment, independent of runtime vertex identities: the sign of the exact end points along the line when
/// the reference carries them (and they are not equal), else the sign of the source nodes.
fn span_orientation<V: CandidateView>(ctx: &mut ExactCtx<'_>, view: &V, span_ref: SpanRef) -> SkelResult<i8> {
    let span = view.span_state(span_ref)?;
    let (a, b) = (Rat::from_i64(span.line.a), Rat::from_i64(span.line.b));
    match view.span_occurrence(span_ref)? {
        SpanOccurrence::Absent => {}
        // oracle commit 3da8cdd: a start or an end without a place orients as zero (the law's own UNPROVEN reason), not as the sign of the source nodes
        SpanOccurrence::WithoutEnd => return Ok(0),
        SpanOccurrence::Points([start_x, start_y, end_x, end_y]) => {
            let direction = end_x.scaled(&b).sub(&end_y.scaled(&a)).sub(&start_x.scaled(&b)).add(&start_y.scaled(&a));
            let sign = exact::sign(ctx, &direction, SIGN_FILTER_BITS)?;
            if sign != 0 {
                return Ok(sign);
            }
        }
    }
    let [x0, y0, x1, y1] = unpack_source_span(span.source_span)?;
    let direction = (i128::from(x1) - i128::from(x0)) * i128::from(span.line.b) - (i128::from(y1) - i128::from(y0)) * i128::from(span.line.a);
    Ok((direction > 0) as i8 - (direction < 0) as i8)
}

/// `classify_poststate_span(view, vertex, peer, birth_time)`: one newborn adjacency, from the length at birth and its slope.
pub fn classify_poststate_span<V: CandidateView>(
    ctx: &mut ExactCtx<'_>,
    view: &V,
    memo: &mut PositionMemo,
    vertex_ref: VertexRef,
    peer_ref: VertexRef,
    birth_time: &EventTime,
) -> SkelResult<PoststateClassification> {
    let vertex = view.vertex_state(vertex_ref)?;
    let peer = view.vertex_state(peer_ref)?;
    let shared = vertex.next_span;
    if peer.prev_span != shared {
        return Ok(classified(PoststateDisposition::AffineClassificationUnproven, None, None, None));
    }
    let birth_length = collapsing_span(ctx, view, memo, vertex_ref, peer_ref, birth_time)?;
    let low_velocity = vertex_velocity(ctx, view, vertex_ref)?;
    let high_velocity = vertex_velocity(ctx, view, peer_ref)?;
    let (Some(birth_length), Some(low_velocity), Some(high_velocity)) = (birth_length.clone(), low_velocity, high_velocity) else {
        return Ok(classified(PoststateDisposition::NonpointJunctionOutsideLaw, birth_length, None, None));
    };
    let line = view.span_state(shared)?.line;
    let (a, b) = (Rat::from_i64(line.a), Rat::from_i64(line.b));
    let slope = high_velocity.0.scaled(&b).sub(&high_velocity.1.scaled(&a)).sub(&low_velocity.0.scaled(&b)).add(&low_velocity.1.scaled(&a));
    let orientation_sign = span_orientation(ctx, view, shared)?;
    if orientation_sign == 0 {
        return Ok(classified(PoststateDisposition::AffineClassificationUnproven, Some(birth_length), Some(slope), None));
    }
    let orientation = Rat::from_i64(i64::from(orientation_sign));
    let birth_length = birth_length.scaled(&orientation);
    let slope = slope.scaled(&orientation);
    let length_sign = exact::sign(ctx, &birth_length, SIGN_FILTER_BITS)?;
    let slope_sign = exact::sign(ctx, &slope, SIGN_FILTER_BITS)?;
    let disposition = if length_sign < 0 || (length_sign == 0 && slope_sign < 0) {
        PoststateDisposition::Inverted
    } else if length_sign == 0 && slope_sign == 0 {
        PoststateDisposition::AffineClassificationUnproven
    } else if length_sign > 0 && slope_sign < 0 {
        PoststateDisposition::ClosingWithFutureEvent
    } else {
        PoststateDisposition::Opening
    };
    Ok(classified(disposition, Some(birth_length), Some(slope), Some(orientation_sign)))
}
