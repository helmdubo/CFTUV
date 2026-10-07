//! The result of a skeleton (`SkeletonOutcome`, `SkeletonNodeV1`, `SkeletonV1` of `wavefront/superlevel.py`) and the exact-time utilities that shape it: the check of a
//! multiway node (`validate_multiway_node`), the accumulation of nodes into one node per incidence component of one exact `(time, point)` (`accumulate_nodes`), the
//! duplicate counters of the canonical records (`digest.duplicate_node_counts`) and the test whether a packet of the current level is still queued
//! (`has_same_time_residual`).
//!
//! Nothing here asks the exact layer a costly question except `has_same_time_residual` (one `compare_times`); the rest is canonical arithmetic and set algebra over
//! the participants, in the oracle's order: dictionaries keep their insertion order, a set that is iterated is sorted first (the oracle sorts them too).

use std::collections::{BTreeSet, HashMap};

use cftuv_canon::fxhash::FxBuild;
use cftuv_core::exact::ExactCtx;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::IntForm;

use crate::error::{SkelError, SkelResult};
use crate::proof::{EdgeKey, ProofObligation, ProofStatus};
use crate::queue::{EventKind, EventQueue};
use crate::time::{compare_times, EventTime, PointRef, TimeRef};

/// `SkeletonOutcome`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum SkeletonOutcome {
    Exact,
    LevelBudgetExhausted,
    MultiwaySplitUnproven,
    WavefrontLeftUnresolved,
    SuperlevelComponentUnresolvable,
}

impl SkeletonOutcome {
    pub const ALL: [SkeletonOutcome; 5] = [
        SkeletonOutcome::Exact,
        SkeletonOutcome::LevelBudgetExhausted,
        SkeletonOutcome::MultiwaySplitUnproven,
        SkeletonOutcome::WavefrontLeftUnresolved,
        SkeletonOutcome::SuperlevelComponentUnresolvable,
    ];

    pub fn value(self) -> &'static str {
        match self {
            SkeletonOutcome::Exact => "EXACT",
            SkeletonOutcome::LevelBudgetExhausted => "LEVEL_BUDGET_EXHAUSTED",
            SkeletonOutcome::MultiwaySplitUnproven => "MULTIWAY_SPLIT_UNPROVEN",
            SkeletonOutcome::WavefrontLeftUnresolved => "WAVEFRONT_LEFT_UNRESOLVED",
            SkeletonOutcome::SuperlevelComponentUnresolvable => "SUPERLEVEL_COMPONENT_UNRESOLVABLE",
        }
    }

    pub fn from_value(value: &str) -> Option<SkeletonOutcome> {
        SkeletonOutcome::ALL.into_iter().find(|outcome| outcome.value() == value)
    }
}

/// `SkeletonNodeV1`: a node of the skeleton, in the exact `(time, point)`; `kinds` and `incidences` are those of a `MULTIWAY` node.
#[derive(Debug, Clone)]
pub struct SkeletonNode {
    pub kind: EventKind,
    pub time: TimeRef,
    pub point: PointRef,
    pub participants: Vec<EdgeKey>,
    pub converging_vertices: i64,
    pub kinds: Vec<EventKind>,
    pub incidences: Vec<Vec<EdgeKey>>,
}

/// `SkeletonV1`.
#[derive(Debug, Clone)]
pub struct Skeleton {
    pub outcome: SkeletonOutcome,
    pub nodes: Vec<SkeletonNode>,
    pub levels: i64,
    /// The counters sorted by name (the oracle's `tuple(sorted(counters.items()))`).
    pub counters: Vec<(String, i64)>,
    pub proof_status: ProofStatus,
    pub proof_obligations: Vec<ProofObligation>,
}

fn value_error(text: &str) -> SkelError {
    SkelError::Value(text.to_string())
}

fn sorted_kinds(kinds: impl IntoIterator<Item = EventKind>) -> Vec<EventKind> {
    let mut found: Vec<EventKind> = kinds.into_iter().collect();
    found.sort_by_key(|kind| kind.value());
    found.dedup();
    found
}

fn sorted_unique_keys(keys: impl IntoIterator<Item = EdgeKey>) -> Vec<EdgeKey> {
    keys.into_iter().collect::<BTreeSet<_>>().into_iter().collect()
}

/// `validate_multiway_node(node)`: the canonical original kinds and incidences of a multiway node, or the named schema error (`ValueError` with the
/// oracle's text).
pub fn validate_multiway_node(node: &SkeletonNode) -> SkelResult<(Vec<EventKind>, Vec<Vec<EdgeKey>>)> {
    let canonical = sorted_kinds(node.kinds.iter().copied());
    if canonical.is_empty() {
        return Err(value_error("MULTIWAY_NODE_KINDS_UNAVAILABLE"));
    }
    if canonical.contains(&EventKind::Multiway) || node.kinds != canonical {
        return Err(value_error("MULTIWAY_NODE_KINDS_NOT_CANONICAL"));
    }
    if node.incidences.is_empty() {
        return Err(value_error("MULTIWAY_NODE_INCIDENCE_UNAVAILABLE"));
    }
    if node.incidences.iter().any(Vec::is_empty) {
        return Err(value_error("MULTIWAY_NODE_INCIDENCE_EMPTY"));
    }
    let mut incidences: Vec<Vec<EdgeKey>> = node.incidences.iter().map(|incidence| sorted_unique_keys(incidence.iter().cloned())).collect();
    incidences.sort();
    if node.incidences != incidences {
        return Err(value_error("MULTIWAY_NODE_INCIDENCES_NOT_CANONICAL"));
    }
    let union = sorted_unique_keys(incidences.iter().flatten().cloned());
    if union != node.participants {
        return Err(value_error("MULTIWAY_NODE_INCIDENCE_UNION_MISMATCH"));
    }
    Ok((canonical, incidences))
}

fn original_kinds(node: &SkeletonNode) -> SkelResult<Vec<EventKind>> {
    if node.kind != EventKind::Multiway {
        return Ok(vec![node.kind]);
    }
    Ok(validate_multiway_node(node)?.0)
}

fn original_incidences(node: &SkeletonNode) -> SkelResult<Vec<Vec<EdgeKey>>> {
    if node.kind != EventKind::Multiway {
        return Ok(vec![node.participants.clone()]);
    }
    Ok(validate_multiway_node(node)?.1)
}

/// The exact value of the `(time, point)` of a node: the canonical time (its dividend and the canonical form of its divisor) and the canonical forms of the two
/// coordinates. Two nodes are at one place exactly when these are equal.
type Place = ((Rat, IntForm), IntForm, IntForm);

fn place_of(node: &SkeletonNode) -> SkelResult<Place> {
    let time = node.time.canonical()?;
    Ok(((time.dividend.clone(), time.divisor.canonical_form().clone()), node.point.x.canonical_form().clone(), node.point.y.canonical_form().clone()))
}

/// `_incidence_components(nodes)`: the components of the nodes that share a participant, each as ascending indices, in order of their smallest member.
fn incidence_components(nodes: &[&SkeletonNode]) -> Vec<Vec<usize>> {
    let mut pending: BTreeSet<usize> = (0..nodes.len()).collect();
    let mut components = Vec::new();
    while let Some(&seed) = pending.iter().next() {
        let mut component: BTreeSet<usize> = BTreeSet::from([seed]);
        let mut participants: BTreeSet<&EdgeKey> = nodes[seed].participants.iter().collect();
        loop {
            let joined: Vec<usize> = pending.iter().copied().filter(|index| !component.contains(index) && nodes[*index].participants.iter().any(|participant| participants.contains(participant))).collect();
            if joined.is_empty() {
                break;
            }
            for index in joined {
                component.insert(index);
                participants.extend(nodes[index].participants.iter());
            }
        }
        for index in &component {
            pending.remove(index);
        }
        components.push(component.into_iter().collect());
    }
    components
}

/// `accumulate_nodes(nodes, converged_vertex_ids)`: one node per incidence component of one exact `(time, point)`; a component of several nodes becomes one `MULTIWAY`
/// node (the first node's time and point, the union of participants, the union of original kinds, every original incidence), the result in order of the first member.
pub fn accumulate_nodes(nodes: &[SkeletonNode], converged_vertex_ids: &[Vec<i64>]) -> SkelResult<Vec<SkeletonNode>> {
    if nodes.len() != converged_vertex_ids.len() {
        return Err(value_error("NODE_ACCUMULATOR_IDENTITY_LENGTH_MISMATCH"));
    }
    let mut slots: HashMap<Place, usize, FxBuild> = HashMap::default();
    let mut by_place: Vec<Vec<usize>> = Vec::new();
    for (index, node) in nodes.iter().enumerate() {
        let place = place_of(node)?;
        match slots.get(&place) {
            Some(slot) => by_place[*slot].push(index),
            None => {
                slots.insert(place, by_place.len());
                by_place.push(vec![index]);
            }
        }
    }
    let mut accumulated: Vec<(usize, SkeletonNode)> = Vec::new();
    for indices in &by_place {
        let placed: Vec<&SkeletonNode> = indices.iter().map(|index| &nodes[*index]).collect();
        for local in incidence_components(&placed) {
            let component: Vec<usize> = local.iter().map(|index| indices[*index]).collect();
            let first = component[0];
            if component.len() == 1 {
                accumulated.push((first, nodes[first].clone()));
                continue;
            }
            let mut kinds = Vec::new();
            for index in &component {
                kinds.extend(original_kinds(&nodes[*index])?);
            }
            let kinds = sorted_kinds(kinds);
            let participants = sorted_unique_keys(component.iter().flat_map(|index| nodes[*index].participants.iter().cloned()));
            let vertices: BTreeSet<i64> = component.iter().flat_map(|index| converged_vertex_ids[*index].iter().copied()).collect();
            let mut incidences = Vec::new();
            for index in &component {
                incidences.extend(original_incidences(&nodes[*index])?);
            }
            incidences.sort();
            let mut merged = nodes[first].clone();
            merged.kind = EventKind::Multiway;
            merged.participants = participants;
            merged.converging_vertices = vertices.len() as i64;
            merged.kinds = kinds;
            merged.incidences = incidences;
            accumulated.push((first, merged));
        }
    }
    accumulated.sort_by_key(|(first, _)| *first);
    Ok(accumulated.into_iter().map(|(_, node)| node).collect())
}

/// `digest.duplicate_node_counts(nodes)`: `(duplicate canonical (time, point) records, places with more than one kind)`. The oracle groups the JSON text of the
/// canonical record; the text is equal exactly when the exact values are, so the groups here are those of the exact `(time, point)`.
pub fn duplicate_node_counts(nodes: &[SkeletonNode]) -> SkelResult<(i64, i64)> {
    let mut slots: HashMap<Place, usize, FxBuild> = HashMap::default();
    let mut groups: Vec<Vec<EventKind>> = Vec::new();
    for node in nodes {
        if node.kind == EventKind::Multiway {
            validate_multiway_node(node)?;
        }
        let place = place_of(node)?;
        match slots.get(&place) {
            Some(slot) => groups[*slot].push(node.kind),
            None => {
                slots.insert(place, groups.len());
                groups.push(vec![node.kind]);
            }
        }
    }
    let duplicates = groups.iter().map(|kinds| kinds.len().saturating_sub(1) as i64).sum();
    let mixed = groups.iter().filter(|kinds| kinds.iter().any(|kind| *kind != kinds[0])).count() as i64;
    Ok((duplicates, mixed))
}

/// `has_same_time_residual(queue, now)`: the head of the queue is at exactly `now` (one `compare_times`, asked only when the queue is not empty).
pub fn has_same_time_residual(ctx: &mut ExactCtx<'_>, queue: &EventQueue, now: &EventTime) -> SkelResult<bool> {
    match queue.peek_time() {
        None => Ok(false),
        Some(upcoming) => Ok(compare_times(ctx, upcoming, now)? == 0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cftuv_core::sqrt_sum::SqrtSum;
    use std::rc::Rc;

    fn node(kind: EventKind, participants: &[&[i64]], kinds: &[EventKind], incidences: &[&[&[i64]]], x: i64) -> SkeletonNode {
        let time = Rc::new(EventTime::new(Rat::from_i64(1), SqrtSum::rational(&Rat::from_i64(1))));
        let point = Rc::new(crate::time::EventPoint { x: SqrtSum::rational(&Rat::from_i64(x)), y: SqrtSum::rational(&Rat::from_i64(0)) });
        SkeletonNode {
            kind,
            time,
            point,
            participants: participants.iter().map(|key| key.to_vec()).collect(),
            converging_vertices: 2,
            kinds: kinds.to_vec(),
            incidences: incidences.iter().map(|incidence| incidence.iter().map(|key| key.to_vec()).collect()).collect(),
        }
    }

    #[test]
    fn nodes_sharing_a_participant_at_one_place_become_one_multiway_node() {
        let a = node(EventKind::Edge, &[&[1], &[2]], &[], &[], 0);
        let b = node(EventKind::Split, &[&[2], &[3]], &[], &[], 0);
        let far = node(EventKind::Edge, &[&[1], &[2]], &[], &[], 5);
        let lone = node(EventKind::Edge, &[&[7], &[8]], &[], &[], 0);
        let merged = accumulate_nodes(&[a, b, far, lone], &[vec![1, 2], vec![2, 3], vec![9], vec![4]]).unwrap();
        assert_eq!(merged.len(), 3);
        assert_eq!(merged[0].kind, EventKind::Multiway);
        assert_eq!(merged[0].participants, vec![vec![1], vec![2], vec![3]]);
        assert_eq!(merged[0].converging_vertices, 3);
        assert_eq!(merged[0].kinds, vec![EventKind::Edge, EventKind::Split]);
        assert_eq!(merged[0].incidences, vec![vec![vec![1], vec![2]], vec![vec![2], vec![3]]]);
        assert_eq!(merged[1].kind, EventKind::Edge);
        assert_eq!(merged[2].participants, vec![vec![7], vec![8]]);
    }

    #[test]
    fn the_schema_errors_are_named_with_the_oracles_text() {
        let mut broken = node(EventKind::Multiway, &[&[1]], &[EventKind::Split, EventKind::Edge], &[&[&[1]]], 0);
        assert_eq!(validate_multiway_node(&broken).unwrap_err(), SkelError::Value("MULTIWAY_NODE_KINDS_NOT_CANONICAL".to_string()));
        broken.kinds = vec![];
        assert_eq!(validate_multiway_node(&broken).unwrap_err(), SkelError::Value("MULTIWAY_NODE_KINDS_UNAVAILABLE".to_string()));
        broken.kinds = vec![EventKind::Edge];
        broken.incidences = vec![];
        assert_eq!(validate_multiway_node(&broken).unwrap_err(), SkelError::Value("MULTIWAY_NODE_INCIDENCE_UNAVAILABLE".to_string()));
        broken.incidences = vec![vec![]];
        assert_eq!(validate_multiway_node(&broken).unwrap_err(), SkelError::Value("MULTIWAY_NODE_INCIDENCE_EMPTY".to_string()));
        broken.incidences = vec![vec![vec![1]]];
        broken.participants = vec![vec![2]];
        assert_eq!(validate_multiway_node(&broken).unwrap_err(), SkelError::Value("MULTIWAY_NODE_INCIDENCE_UNION_MISMATCH".to_string()));
        assert_eq!(accumulate_nodes(&[broken], &[]).unwrap_err(), SkelError::Value("NODE_ACCUMULATOR_IDENTITY_LENGTH_MISMATCH".to_string()));
    }

    #[test]
    fn duplicate_places_are_counted_once_per_extra_node_and_mixed_ones_once() {
        let nodes = [
            node(EventKind::Edge, &[&[1]], &[], &[], 0),
            node(EventKind::Edge, &[&[2]], &[], &[], 0),
            node(EventKind::Split, &[&[3]], &[], &[], 0),
            node(EventKind::Edge, &[&[4]], &[], &[], 9),
        ];
        assert_eq!(duplicate_node_counts(&nodes).unwrap(), (2, 1));
    }
}
