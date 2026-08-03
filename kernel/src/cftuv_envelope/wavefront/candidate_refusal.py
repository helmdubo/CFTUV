"""Named candidate refusals shared without importing the runtime builder."""

from __future__ import annotations

from enum import Enum

from .event_time import SupportLineV1


class CandidateRefusal(str, Enum):
    """FILTER means proven absence; NO_RULE means a named open seam."""

    FILTER_SOLO_VERTEX = "FILTER_SOLO_VERTEX"
    FILTER_TRIPLE_NEVER_CONCURRENT = "FILTER_TRIPLE_NEVER_CONCURRENT"
    FILTER_EVENT_IN_THE_PAST = "FILTER_EVENT_IN_THE_PAST"
    FILTER_POINT_OUTSIDE_FRONT = "FILTER_POINT_OUTSIDE_FRONT"
    FILTER_BEYOND_TRACE = "FILTER_BEYOND_TRACE"
    FILTER_EDGE_IS_OWN = "FILTER_EDGE_IS_OWN"
    FILTER_SPAN_DOES_NOT_COLLAPSE = "FILTER_SPAN_DOES_NOT_COLLAPSE"
    FILTER_SPAN_IS_BORN_ZERO = "FILTER_SPAN_IS_BORN_ZERO"
    NO_RULE_TRIPLE_ALWAYS_CONCURRENT = "NO_RULE_TRIPLE_ALWAYS_CONCURRENT"
    NO_RULE_JOINT_IS_ANTIPARALLEL = "NO_RULE_JOINT_IS_ANTIPARALLEL"
    NO_RULE_JOINT_IS_CODIRECTIONAL = "NO_RULE_JOINT_IS_CODIRECTIONAL"
    NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS = (
        "NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS"
    )
    NO_RULE_SPAN_VANISHED = "NO_RULE_SPAN_VANISHED"
    NO_RULE_MEETING_NOT_RECONNECTABLE = (
        "NO_RULE_MEETING_NOT_RECONNECTABLE"
    )


def refusal_counter(reason: CandidateRefusal) -> str:
    return f"refused_{reason.value.lower()}"


REFUSAL_COUNTERS = tuple(refusal_counter(item) for item in CandidateRefusal)


def joint_refusal(
    first: SupportLineV1,
    second: SupportLineV1,
) -> CandidateRefusal:
    dot = first.b * second.b + first.a * second.a
    if dot <= 0:
        return CandidateRefusal.NO_RULE_JOINT_IS_ANTIPARALLEL
    if first.q * second.normal_squared != second.q * first.normal_squared:
        return (
            CandidateRefusal
            .NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS
        )
    return CandidateRefusal.NO_RULE_JOINT_IS_CODIRECTIONAL
