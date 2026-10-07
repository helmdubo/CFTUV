//! The fail-closed composition of the frozen initial mixed plans (`wavefront/symbolic_initial_composition.py`): the one proven bijection of an EDGE contact with an
//! interior SPLIT whose emitter dies in both.
//!
//! When a vertex dies in an edge contact AND emits an interior cut, the two plans overlap on that vertex. The overlap is accepted only in the one shape the proof covers
//! (one contact with one chain and one birth, one cut with one event, two segments, two births, two final ports, the emitter inside the chain, the births at the ends
//! of the chain); then the two births of the cut are replaced by two COMPOSED births (`head.prev` to the first piece, the second piece to `tail.next`) and the contact
//! gives up its own. Any other overlap is an ambiguity, answered `false`.

use std::collections::{BTreeSet, HashMap, HashSet};

use cftuv_canon::fxhash::FxBuild;

use crate::error::SkelResult;
use crate::germ::GermLedger;
use crate::plans::{birth, BoundaryBirth, EdgeContactPlan, SplitCutPlan};
use crate::pyval::{sorted_by_val_or_refuse, Val};
use crate::queue::EventKind;
use crate::snapshot::VertexSnapshot;

/// What the composition answers: the contacts, the cuts, the vertices the two paths share and the composition accounts for, and whether the shape was provable.
pub type Composed = (Vec<EdgeContactPlan>, Vec<SplitCutPlan>, BTreeSet<i64>, bool);

/// `compose_edge_split_overlap(contacts, split_cuts, vertices, birth_factory)`.
pub fn compose_edge_split_overlap(contacts: Vec<EdgeContactPlan>, split_cuts: Vec<SplitCutPlan>, vertices: &[VertexSnapshot], ledger: &mut GermLedger) -> SkelResult<Composed> {
    let contact_dead: BTreeSet<i64> = contacts.iter().flat_map(|contact| contact.dead_vertex_ids.iter().copied()).collect();
    let cut_dead: BTreeSet<i64> = split_cuts.iter().flat_map(|cut| cut.events.iter().map(|event| event.vertex)).collect();
    let overlap: BTreeSet<i64> = contact_dead.intersection(&cut_dead).copied().collect();
    if overlap.is_empty() {
        return Ok((contacts, split_cuts, BTreeSet::new(), true));
    }
    let refused = |contacts: Vec<EdgeContactPlan>, split_cuts: Vec<SplitCutPlan>| Ok((contacts, split_cuts, BTreeSet::new(), false));
    if contacts.len() != 1 || split_cuts.len() != 1 || overlap.len() != 1 {
        return refused(contacts, split_cuts);
    }
    let (contact, cut) = (&contacts[0], &split_cuts[0]);
    if !contact.kinds.contains(&EventKind::Edge)
        || contact.chains.len() != 1
        || contact.births.len() != 1
        || cut.events.len() != 1
        || cut.segment_occurrences.len() != 2
        || cut.births.len() != 2
        || cut.final_birth_ports.len() != 2
    {
        return refused(contacts, split_cuts);
    }
    let emitter_id = *overlap.iter().next().unwrap_or(&-1);
    let chain = &contact.chains[0];
    if !chain.contains(&emitter_id) || cut.events[0].vertex != emitter_id {
        return refused(contacts, split_cuts);
    }
    let (head, tail) = (&vertices[chain[0] as usize], &vertices[chain[chain.len() - 1] as usize]);
    let emitter = &vertices[emitter_id as usize];
    let contact_birth = &contact.births[0];
    if Some(&contact_birth.prev_occurrence) != head.prev_occurrence.as_ref() || Some(&contact_birth.next_occurrence) != tail.next_occurrence.as_ref() || contact_birth.point_key != contact.point_key {
        return refused(contacts, split_cuts);
    }
    let replaced_by_emitter = |item: &BoundaryBirth| item.replaces == [emitter_id];
    let prev_side: Vec<usize> = (0..cut.births.len()).filter(|index| Some(&cut.births[*index].prev_occurrence) == emitter.prev_occurrence.as_ref() && replaced_by_emitter(&cut.births[*index])).collect();
    let next_side: Vec<usize> = (0..cut.births.len()).filter(|index| Some(&cut.births[*index].next_occurrence) == emitter.next_occurrence.as_ref() && replaced_by_emitter(&cut.births[*index])).collect();
    if prev_side.len() != 1
        || next_side.len() != 1
        || prev_side[0] == next_side[0]
        || cut.births[prev_side[0]].next_occurrence != cut.segment_occurrences[1]
        || cut.births[next_side[0]].prev_occurrence != cut.segment_occurrences[0]
        || cut.births.iter().any(|item| item.point_key != contact.point_key)
    {
        return refused(contacts, split_cuts);
    }
    let (prev_birth, next_birth) = (&cut.births[prev_side[0]], &cut.births[next_side[0]]);
    let composed_prev = birth(&contact.time, &contact.point_key, head.prev_occurrence.as_ref(), Some(&prev_birth.next_occurrence), vec![emitter_id], ledger)?;
    let composed_next = birth(&contact.time, &contact.point_key, Some(&next_birth.prev_occurrence), tail.next_occurrence.as_ref(), vec![emitter_id], ledger)?;
    let (Some(composed_prev), Some(composed_next)) = (composed_prev, composed_next) else {
        return refused(contacts, split_cuts);
    };
    let composed = [composed_prev, composed_next];
    let distinct = |select: &dyn Fn(&BoundaryBirth) -> Val| composed.iter().map(select).collect::<HashSet<Val>>().len();
    if distinct(&|item| item.key.clone()) != 2 || distinct(&|item| item.prev_occurrence.clone()) != 2 || distinct(&|item| item.next_occurrence.clone()) != 2 {
        return refused(contacts, split_cuts);
    }
    let flags: HashMap<Val, (bool, bool), FxBuild> = cut.final_birth_ports.iter().map(|(key, keep_prev, keep_next)| (key.clone(), (*keep_prev, *keep_next))).collect();
    let birth_keys: HashSet<Val> = cut.births.iter().map(|item| item.key.clone()).collect();
    if flags.len() != birth_keys.len() || !flags.keys().all(|key| birth_keys.contains(key)) {
        return refused(contacts, split_cuts);
    }
    let remapped: HashMap<Val, Val, FxBuild> = [(prev_birth.key.clone(), composed[0].key.clone()), (next_birth.key.clone(), composed[1].key.clone())].into_iter().collect();
    let mut ports: Vec<(Val, bool, bool)> = Vec::new();
    for (key, _, _) in &cut.final_birth_ports {
        let Some(new_key) = remapped.get(key) else {
            return refused(contacts, split_cuts);
        };
        ports.push((new_key.clone(), flags[key].0, flags[key].1));
    }
    let final_birth_ports = sorted_by_val_or_refuse(&ports, |(key, prev, next)| Val::tuple(vec![key.clone(), Val::boolean(*prev), Val::boolean(*next)]), "symbolic_initial_composition.compose_edge_split_overlap")?;
    let births = sorted_by_val_or_refuse(&composed, |item| item.key.clone(), "symbolic_initial_composition.compose_edge_split_overlap")?;
    let (mut contact, mut cut) = (contacts[0].clone(), split_cuts[0].clone());
    contact.births = Vec::new();
    cut.births = births;
    cut.final_birth_ports = final_birth_ports;
    Ok((vec![contact], vec![cut], overlap, true))
}
