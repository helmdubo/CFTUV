//! The proof axis of a skeleton result (`wavefront/proof.py`): the obligations a run could not prove, their life cycle and the final status.
//!
//! Geometry only makes observations with the identity already taken; this module keeps them, discharges them by the proven death of the named vertices
//! and computes the status. It decides no geometric predicate and changes no outcome. Nothing here costs: the level of an obligation is made canonical (a
//! scaling, no sign), the order is a key of values.

use std::collections::BTreeSet;

use cftuv_core::num::{IBig, UBig};

use crate::candidate::CandidateRefusal;
use crate::error::SkelResult;
use crate::queue::EventKind;
use crate::time::EventTime;

/// `ProofObligationDisposition`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProofDisposition {
    Observed,
    DischargedByProvenSameTimeEvent,
    UnprovenFallbackApplied,
    EventAcceptedWithUnprovenSpan,
    SurvivedPastEventTime,
    UnsupportedEventKindDropped,
    SuperlevelComponentUnresolvable,
}

impl ProofDisposition {
    pub const ALL: [ProofDisposition; 7] = [
        ProofDisposition::Observed,
        ProofDisposition::DischargedByProvenSameTimeEvent,
        ProofDisposition::UnprovenFallbackApplied,
        ProofDisposition::EventAcceptedWithUnprovenSpan,
        ProofDisposition::SurvivedPastEventTime,
        ProofDisposition::UnsupportedEventKindDropped,
        ProofDisposition::SuperlevelComponentUnresolvable,
    ];

    pub fn value(self) -> &'static str {
        match self {
            ProofDisposition::Observed => "OBSERVED",
            ProofDisposition::DischargedByProvenSameTimeEvent => "DISCHARGED_BY_PROVEN_SAME_TIME_EVENT",
            ProofDisposition::UnprovenFallbackApplied => "UNPROVEN_FALLBACK_APPLIED",
            ProofDisposition::EventAcceptedWithUnprovenSpan => "EVENT_ACCEPTED_WITH_UNPROVEN_SPAN",
            ProofDisposition::SurvivedPastEventTime => "SURVIVED_PAST_EVENT_TIME",
            ProofDisposition::UnsupportedEventKindDropped => "UNSUPPORTED_EVENT_KIND_DROPPED",
            ProofDisposition::SuperlevelComponentUnresolvable => "SUPERLEVEL_COMPONENT_UNRESOLVABLE",
        }
    }

    pub fn from_value(value: &str) -> Option<ProofDisposition> {
        ProofDisposition::ALL.into_iter().find(|found| found.value() == value)
    }

    /// `_INCOMPLETE_DISPOSITIONS`.
    fn makes_incomplete(self) -> bool {
        matches!(
            self,
            ProofDisposition::UnprovenFallbackApplied
                | ProofDisposition::EventAcceptedWithUnprovenSpan
                | ProofDisposition::SurvivedPastEventTime
                | ProofDisposition::UnsupportedEventKindDropped
                | ProofDisposition::SuperlevelComponentUnresolvable
        )
    }
}

/// `ProofStatus`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProofStatus {
    Complete,
    Incomplete,
}

impl ProofStatus {
    pub fn value(self) -> &'static str {
        match self {
            ProofStatus::Complete => "COMPLETE",
            ProofStatus::Incomplete => "INCOMPLETE",
        }
    }
}

/// `ProofObligationBranch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProofBranch {
    EdgeCollapseSpanUnproven,
    UnsupportedEventKind,
    SuperlevelComponentUnresolvable,
}

impl ProofBranch {
    pub const ALL: [ProofBranch; 3] = [ProofBranch::EdgeCollapseSpanUnproven, ProofBranch::UnsupportedEventKind, ProofBranch::SuperlevelComponentUnresolvable];

    pub fn value(self) -> &'static str {
        match self {
            ProofBranch::EdgeCollapseSpanUnproven => "EDGE_COLLAPSE_SPAN_UNPROVEN",
            ProofBranch::UnsupportedEventKind => "UNSUPPORTED_EVENT_KIND",
            ProofBranch::SuperlevelComponentUnresolvable => "SUPERLEVEL_COMPONENT_UNRESOLVABLE",
        }
    }
}

/// `cause: CandidateRefusal | ProofObligationBranch`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ProofCause {
    Refusal(CandidateRefusal),
    Branch(ProofBranch),
}

impl ProofCause {
    pub fn value(self) -> &'static str {
        match self {
            ProofCause::Refusal(reason) => reason.value(),
            ProofCause::Branch(branch) => branch.value(),
        }
    }
}

/// An edge key of an obligation: `(x0, y0, x1, y1)` of a source edge or `(x, y, x, y, ordinal)` of a fan edge.
pub type EdgeKey = Vec<i64>;

/// `ProofObligationV1`: the identity of one unproven branch (the level is canonical).
#[derive(Debug, Clone)]
pub struct ProofObligation {
    pub cause: ProofCause,
    pub disposition: ProofDisposition,
    pub vertex_ids: Vec<i64>,
    pub participant_edge_keys: Vec<EdgeKey>,
    pub target_edge_keys: Vec<EdgeKey>,
    pub level: EventTime,
    pub event_kind: Option<EventKind>,
}

/// `_NO_RULE_DISPOSITIONS.get(reason.value)`: what a refusal is recorded as (the FILTER reasons are not recorded at all).
fn refusal_disposition(reason: CandidateRefusal) -> Option<ProofDisposition> {
    match reason {
        CandidateRefusal::NoRuleTripleAlwaysConcurrent
        | CandidateRefusal::NoRuleJointIsAntiparallel
        | CandidateRefusal::NoRuleJointIsCodirectional
        | CandidateRefusal::NoRuleJointIsCodirectionalAtDifferentSpeeds => Some(ProofDisposition::Observed),
        CandidateRefusal::NoRuleSpanVanished | CandidateRefusal::NoRuleMeetingNotReconnectable => Some(ProofDisposition::UnprovenFallbackApplied),
        _ => None,
    }
}

/// The sort key of `_obligation_sort_key`: the canonical level (dividend, divisor terms), then the names, the vertices and the edge keys. Python's tuple order is
/// the lexicographic order of these values (strings by code point, which is the byte order of their UTF-8).
type SortKey = (IBig, UBig, Vec<(UBig, IBig, UBig)>, &'static str, &'static str, Vec<i64>, Vec<EdgeKey>, Vec<EdgeKey>, &'static str);

fn sort_key(obligation: &ProofObligation) -> SortKey {
    let divisor = obligation.level.divisor.terms().iter().map(|term| (term.radicand.clone(), term.coef.value().numerator().clone(), term.coef.value().denominator().clone())).collect();
    (
        obligation.level.dividend.numerator().clone(),
        obligation.level.dividend.denominator().clone(),
        divisor,
        obligation.cause.value(),
        obligation.disposition.value(),
        obligation.vertex_ids.clone(),
        obligation.participant_edge_keys.clone(),
        obligation.target_edge_keys.clone(),
        obligation.event_kind.map_or("", EventKind::value),
    )
}

fn sorted_unique<T: Ord + Clone>(values: &[T]) -> Vec<T> {
    values.iter().cloned().collect::<BTreeSet<T>>().into_iter().collect()
}

/// `ProofLedger`: the owner of the obligations and of their life cycle.
#[derive(Debug, Clone, Default)]
pub struct ProofLedger {
    obligations: Vec<ProofObligation>,
}

impl ProofLedger {
    pub fn new() -> ProofLedger {
        ProofLedger::default()
    }

    pub fn obligations(&self) -> &[ProofObligation] {
        &self.obligations
    }

    /// A ledger holding these obligations as they are (the seams restore the oracle's with it).
    pub fn from_obligations(obligations: Vec<ProofObligation>) -> ProofLedger {
        ProofLedger { obligations }
    }

    /// `record(...)`: the identity is normalised (sorted, repeats dropped), the multiplicity of the records themselves is kept.
    #[allow(clippy::too_many_arguments)]
    pub fn record(
        &mut self,
        cause: ProofCause,
        disposition: ProofDisposition,
        vertex_ids: &[i64],
        participant_edge_keys: &[EdgeKey],
        target_edge_keys: &[EdgeKey],
        level: &EventTime,
        event_kind: Option<EventKind>,
    ) -> SkelResult<()> {
        self.obligations.push(ProofObligation {
            cause,
            disposition,
            vertex_ids: sorted_unique(vertex_ids),
            participant_edge_keys: sorted_unique(participant_edge_keys),
            target_edge_keys: sorted_unique(target_edge_keys),
            level: level.canonical()?,
            event_kind,
        });
        Ok(())
    }

    /// `record_refusal(reason, ...)`: a NO_RULE refusal becomes an obligation, a FILTER one is not recorded.
    pub fn record_refusal(&mut self, reason: CandidateRefusal, vertex_ids: &[i64], participant_edge_keys: &[EdgeKey], target_edge_keys: &[EdgeKey], level: &EventTime) -> SkelResult<()> {
        match refusal_disposition(reason) {
            None => Ok(()),
            Some(disposition) => self.record(ProofCause::Refusal(reason), disposition, vertex_ids, participant_edge_keys, target_edge_keys, level, None),
        }
    }

    /// `discharge(dead)`: an OBSERVED obligation whose (non-empty) vertices all died is discharged.
    pub fn discharge(&mut self, dead_vertex_ids: &[i64]) {
        let dead: BTreeSet<i64> = dead_vertex_ids.iter().copied().collect();
        for obligation in &mut self.obligations {
            if obligation.disposition == ProofDisposition::Observed && !obligation.vertex_ids.is_empty() && obligation.vertex_ids.iter().all(|vertex| dead.contains(vertex)) {
                obligation.disposition = ProofDisposition::DischargedByProvenSameTimeEvent;
            }
        }
    }

    /// `finalize(dead)`: discharge, what is still OBSERVED survived the event time, then the order of the result and the status.
    pub fn finalize(&mut self, dead_vertex_ids: &[i64]) -> (ProofStatus, Vec<ProofObligation>) {
        self.discharge(dead_vertex_ids);
        for obligation in &mut self.obligations {
            if obligation.disposition == ProofDisposition::Observed {
                obligation.disposition = ProofDisposition::SurvivedPastEventTime;
            }
        }
        let mut obligations = self.obligations.clone();
        obligations.sort_by_cached_key(sort_key);
        let status = if obligations.iter().any(|obligation| obligation.disposition.makes_incomplete()) { ProofStatus::Incomplete } else { ProofStatus::Complete };
        (status, obligations)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::rat::Rat;
    use cftuv_core::sqrt_sum::SqrtSum;

    fn level(dividend: i64, divisor: i64) -> EventTime {
        EventTime::new(Rat::from_i64(dividend), SqrtSum::rational(&Rat::from_i64(divisor)))
    }

    #[test]
    fn a_filter_refusal_is_not_recorded_and_a_no_rule_one_is() {
        let mut ledger = ProofLedger::new();
        ledger.record_refusal(CandidateRefusal::FilterEventInThePast, &[1], &[], &[], &level(1, 1)).unwrap();
        assert!(ledger.obligations().is_empty());
        ledger.record_refusal(CandidateRefusal::NoRuleSpanVanished, &[3, 1, 3], &[vec![2, 2, 1, 1], vec![0, 0, 1, 1], vec![2, 2, 1, 1]], &[], &level(2, 4)).unwrap();
        let found = &ledger.obligations()[0];
        assert_eq!(found.vertex_ids, vec![1, 3]);
        assert_eq!(found.participant_edge_keys, vec![vec![0, 0, 1, 1], vec![2, 2, 1, 1]]);
        assert_eq!(found.disposition, ProofDisposition::UnprovenFallbackApplied);
        // the level is the canonical one: the divisor's first coefficient is one
        assert_eq!(found.level.divisor, SqrtSum::rational(&Rat::one()));
    }

    #[test]
    fn finalize_discharges_the_dead_turns_the_rest_into_survivors_and_sorts() {
        let mut ledger = ProofLedger::new();
        ledger.record_refusal(CandidateRefusal::NoRuleJointIsAntiparallel, &[7], &[], &[], &level(5, 1)).unwrap();
        ledger.record_refusal(CandidateRefusal::NoRuleTripleAlwaysConcurrent, &[8], &[], &[], &level(3, 1)).unwrap();
        ledger.record_refusal(CandidateRefusal::NoRuleJointIsCodirectional, &[], &[], &[], &level(4, 1)).unwrap();
        let (status, found) = ledger.finalize(&[7]);
        assert_eq!(status, ProofStatus::Incomplete);
        let by_vertex: Vec<(Vec<i64>, ProofDisposition)> = found.iter().map(|each| (each.vertex_ids.clone(), each.disposition)).collect();
        assert_eq!(
            by_vertex,
            vec![
                (vec![8], ProofDisposition::SurvivedPastEventTime),
                (Vec::new(), ProofDisposition::SurvivedPastEventTime),
                (vec![7], ProofDisposition::DischargedByProvenSameTimeEvent)
            ]
        );
        let mut calm = ProofLedger::new();
        calm.record_refusal(CandidateRefusal::NoRuleJointIsAntiparallel, &[7], &[], &[], &level(5, 1)).unwrap();
        assert_eq!(calm.finalize(&[7]).0, ProofStatus::Complete);
    }
}
