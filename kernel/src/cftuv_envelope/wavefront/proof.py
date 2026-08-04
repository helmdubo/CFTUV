"""Независимая proof-ось результата волнового фронта.

Геометрия создаёт только наблюдения с уже снятым identity. Этот модуль хранит
их, погашает доказанной смертью названных вершин и вычисляет итоговый статус.
Он не решает ни одного геометрического предиката и не меняет SkeletonOutcome.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import Enum
from typing import Iterable, TYPE_CHECKING

from .event_time import EventTimeV1
from .events import EventKind

if TYPE_CHECKING:
    from .candidate_refusal import CandidateRefusal


class ProofObligationDisposition(str, Enum):
    """Что стало с явно названной, но ещё не доказанной ветвью."""

    OBSERVED = "OBSERVED"
    DISCHARGED_BY_PROVEN_SAME_TIME_EVENT = (
        "DISCHARGED_BY_PROVEN_SAME_TIME_EVENT"
    )
    UNPROVEN_FALLBACK_APPLIED = "UNPROVEN_FALLBACK_APPLIED"
    EVENT_ACCEPTED_WITH_UNPROVEN_SPAN = "EVENT_ACCEPTED_WITH_UNPROVEN_SPAN"
    SURVIVED_PAST_EVENT_TIME = "SURVIVED_PAST_EVENT_TIME"
    UNSUPPORTED_EVENT_KIND_DROPPED = "UNSUPPORTED_EVENT_KIND_DROPPED"
    SUPERLEVEL_COMPONENT_UNRESOLVABLE = (
        "SUPERLEVEL_COMPONENT_UNRESOLVABLE"
    )


class ProofStatus(str, Enum):
    COMPLETE = "COMPLETE"
    INCOMPLETE = "INCOMPLETE"


class ProofObligationBranch(str, Enum):
    EDGE_COLLAPSE_SPAN_UNPROVEN = "EDGE_COLLAPSE_SPAN_UNPROVEN"
    UNSUPPORTED_EVENT_KIND = "UNSUPPORTED_EVENT_KIND"
    SUPERLEVEL_COMPONENT_UNRESOLVABLE = (
        "SUPERLEVEL_COMPONENT_UNRESOLVABLE"
    )


@dataclass(frozen=True, slots=True)
class ProofObligationV1:
    """Стабильное тождество одного непогашенного доказательного долга."""

    cause: CandidateRefusal | ProofObligationBranch
    disposition: ProofObligationDisposition
    vertex_ids: tuple[int, ...]
    participant_edge_keys: tuple[tuple[int, ...], ...]
    target_edge_keys: tuple[tuple[int, ...], ...]
    level: EventTimeV1
    event_kind: EventKind | None


_NO_RULE_DISPOSITIONS = {
    "NO_RULE_TRIPLE_ALWAYS_CONCURRENT": ProofObligationDisposition.OBSERVED,
    "NO_RULE_JOINT_IS_ANTIPARALLEL": ProofObligationDisposition.OBSERVED,
    "NO_RULE_JOINT_IS_CODIRECTIONAL": ProofObligationDisposition.OBSERVED,
    "NO_RULE_JOINT_IS_CODIRECTIONAL_AT_DIFFERENT_SPEEDS": (
        ProofObligationDisposition.OBSERVED
    ),
    "NO_RULE_SPAN_VANISHED": (
        ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED
    ),
    "NO_RULE_MEETING_NOT_RECONNECTABLE": (
        ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED
    ),
}

_INCOMPLETE_DISPOSITIONS = frozenset(
    {
        ProofObligationDisposition.UNPROVEN_FALLBACK_APPLIED,
        ProofObligationDisposition.EVENT_ACCEPTED_WITH_UNPROVEN_SPAN,
        ProofObligationDisposition.SURVIVED_PAST_EVENT_TIME,
        ProofObligationDisposition.UNSUPPORTED_EVENT_KIND_DROPPED,
        ProofObligationDisposition.SUPERLEVEL_COMPONENT_UNRESOLVABLE,
    }
)


def _obligation_sort_key(obligation: ProofObligationV1) -> tuple:
    level = obligation.level.canonical()
    dividend = level.dividend
    divisor = tuple(
        (radicand, coefficient.numerator, coefficient.denominator)
        for radicand, coefficient in level.divisor.terms
    )
    return (
        dividend.numerator,
        dividend.denominator,
        divisor,
        obligation.cause.value,
        obligation.disposition.value,
        obligation.vertex_ids,
        obligation.participant_edge_keys,
        obligation.target_edge_keys,
        obligation.event_kind.value if obligation.event_kind is not None else "",
    )


class ProofLedger:
    """Владелец obligations и их жизненного цикла."""

    def __init__(self) -> None:
        self._obligations: list[ProofObligationV1] = []

    @property
    def obligations(self) -> tuple[ProofObligationV1, ...]:
        return tuple(self._obligations)

    def record(
        self,
        *,
        cause: CandidateRefusal | ProofObligationBranch,
        disposition: ProofObligationDisposition,
        vertex_ids: tuple[int, ...] = (),
        participant_edge_keys: tuple[tuple[int, ...], ...] = (),
        target_edge_keys: tuple[tuple[int, ...], ...] = (),
        level: EventTimeV1,
        event_kind: EventKind | None = None,
    ) -> None:
        """Нормализовать identity, сохранив кратность самих записей."""

        self._obligations.append(
            ProofObligationV1(
                cause=cause,
                disposition=disposition,
                vertex_ids=tuple(sorted(set(vertex_ids))),
                participant_edge_keys=tuple(
                    sorted(set(participant_edge_keys))
                ),
                target_edge_keys=tuple(sorted(set(target_edge_keys))),
                level=level.canonical(),
                event_kind=event_kind,
            )
        )

    def record_refusal(
        self,
        reason: CandidateRefusal,
        *,
        vertex_ids: tuple[int, ...],
        participant_edge_keys: tuple[tuple[int, ...], ...],
        target_edge_keys: tuple[tuple[int, ...], ...],
        level: EventTimeV1,
    ) -> None:
        disposition = _NO_RULE_DISPOSITIONS.get(reason.value)
        if disposition is None:
            return
        self.record(
            cause=reason,
            disposition=disposition,
            vertex_ids=vertex_ids,
            participant_edge_keys=participant_edge_keys,
            target_edge_keys=target_edge_keys,
            level=level,
        )

    def discharge(self, dead_vertex_ids: Iterable[int]) -> None:
        """Погасить OBSERVED только непустой доказанной смертью всех IDs."""

        dead = frozenset(dead_vertex_ids)
        self._obligations = [
            replace(
                obligation,
                disposition=(
                    ProofObligationDisposition.DISCHARGED_BY_PROVEN_SAME_TIME_EVENT
                ),
            )
            if (
                obligation.disposition is ProofObligationDisposition.OBSERVED
                and obligation.vertex_ids
                and set(obligation.vertex_ids).issubset(dead)
            )
            else obligation
            for obligation in self._obligations
        ]

    def finalize(
        self, dead_vertex_ids: Iterable[int]
    ) -> tuple[ProofStatus, tuple[ProofObligationV1, ...]]:
        self.discharge(dead_vertex_ids)
        self._obligations = [
            replace(
                obligation,
                disposition=ProofObligationDisposition.SURVIVED_PAST_EVENT_TIME,
            )
            if obligation.disposition is ProofObligationDisposition.OBSERVED
            else obligation
            for obligation in self._obligations
        ]
        obligations = tuple(sorted(self._obligations, key=_obligation_sort_key))
        status = (
            ProofStatus.INCOMPLETE
            if any(
                obligation.disposition in _INCOMPLETE_DISPOSITIONS
                for obligation in obligations
            )
            else ProofStatus.COMPLETE
        )
        return status, obligations
