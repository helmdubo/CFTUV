from fractions import Fraction

import pytest

from cftuv_envelope.wavefront.event_time import SupportLineV1, ZERO_TIME
from cftuv_envelope.wavefront.exact_candidate_view import (
    CandidateSpanStateV1,
    CandidateVertexStateV1,
    ExactCandidateViewV1,
)
from cftuv_envelope.wavefront.poststate_span import (
    PoststateSpanDisposition,
    classify_poststate_span,
)
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton
from wavefront_cases import partial_source_corpus


def _line(start, end, speed=1):
    return SupportLineV1.with_speed(start, end, Fraction(speed))


def _view(low_side, high_side, *, low_point, high_point):
    spans = {
        "low": CandidateSpanStateV1(
            low_side, (*low_point, low_point[0], low_point[1] + 1), None, None
        ),
        "shared": CandidateSpanStateV1(
            _line((0, 0), (10, 0), 0), (0, 0, 10, 0), "low", "high"
        ),
        "high": CandidateSpanStateV1(
            high_side,
            (*high_point, high_point[0], high_point[1] + 1),
            None,
            None,
        ),
    }
    vertices = {
        "low": CandidateVertexStateV1(
            "low", "shared", ZERO_TIME, None
        ),
        "high": CandidateVertexStateV1(
            "shared", "high", ZERO_TIME, None
        ),
    }
    return ExactCandidateViewV1(
        (), vertices.__getitem__, spans.__getitem__, lambda *_: None
    )


@pytest.mark.parametrize(
    "low_side,high_side,low_point,high_point,expected",
    (
        (
            _line((0, 0), (0, 1)),
            _line((10, 1), (10, 0)),
            (0, 0),
            (10, 0),
            PoststateSpanDisposition.OPENING,
        ),
        (
            _line((0, 1), (0, 0)),
            _line((10, 0), (10, 1)),
            (0, 0),
            (10, 0),
            PoststateSpanDisposition.CLOSING_WITH_FUTURE_EVENT,
        ),
        (
            _line((10, 1), (10, 0)),
            _line((0, 0), (0, 1)),
            (10, 0),
            (0, 0),
            PoststateSpanDisposition.INVERTED,
        ),
        (
            _line((5, 0), (5, 1)),
            _line((5, 1), (5, 0)),
            (5, 0),
            (5, 0),
            PoststateSpanDisposition.OPENING,
        ),
    ),
)
def test_affine_span_law_uses_birth_sign_and_slope(
    low_side, high_side, low_point, high_point, expected
):
    result = classify_poststate_span(
        _view(
            low_side,
            high_side,
            low_point=low_point,
            high_point=high_point,
        ),
        "low",
        "high",
        ZERO_TIME,
    )
    assert result.disposition is expected
    assert result.birth_length is not None
    assert result.slope is not None


def test_identically_zero_newborn_span_is_fail_closed_ambiguity():
    result = classify_poststate_span(
        _view(
            _line((5, 1), (5, 0), 0),
            _line((5, 0), (5, 1), 0),
            low_point=(5, 0),
            high_point=(5, 0),
        ),
        "low",
        "high",
        ZERO_TIME,
    )
    assert result.disposition is (
        PoststateSpanDisposition.AFFINE_CLASSIFICATION_UNPROVEN
    )
    assert result.birth_length.is_zero
    assert result.slope.is_zero


def test_nonpoint_codirectional_joint_stays_outside_the_affine_law():
    shared = _line((0, 0), (10, 0), 0)
    view = _view(
        shared,
        _line((10, 0), (10, 1), 0),
        low_point=(0, 0),
        high_point=(10, 0),
    )
    result = classify_poststate_span(
        view, "low", "high", ZERO_TIME
    )
    assert result.disposition is (
        PoststateSpanDisposition.NONPOINT_JUNCTION_OUTSIDE_LAW
    )


def test_ell_reverse_normal_account_is_named_outside_the_law(monkeypatch):
    from cftuv_envelope.wavefront import symbolic_runtime_commit as runtime

    original = runtime.plan_symbolic_runtime_commit
    observed = []

    def capture(builder, snapshot, closure):
        observed.extend(
            witness
            for witness in runtime.poststate_span_classifications(
                builder, snapshot, closure.overlay
            )
            if witness.classification.disposition
            is PoststateSpanDisposition.NONPOINT_JUNCTION_OUTSIDE_LAW
        )
        return original(builder, snapshot, closure)

    monkeypatch.setattr(
        runtime, "plan_symbolic_runtime_commit", capture
    )
    polygon = dict(partial_source_corpus())["ell_12_source_edge_4"]
    skeleton = build_skeleton(polygon)
    assert skeleton.outcome is SkeletonOutcome.WAVEFRONT_LEFT_UNRESOLVED
    assert {
        (
            (12, 6, 6, 6),
            (6, 12, 0, 12),
            (0, 12, 0, 0),
        ),
        (
            (12, 0, 12, 6),
            (12, 6, 6, 6),
            (6, 12, 0, 12),
        ),
    }.issubset({
        tuple(witness.participant_edge_keys) for witness in observed
    })
    assert not any(
        "SYMBOLIC_POSTSTATE_SPAN" in key and value
        for key, value in dict(skeleton.counters).items()
    )
