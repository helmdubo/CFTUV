//! What the transaction of a packet does to the builder besides the symbolic closure (`wavefront/superlevel.py`, the live helpers): the head of
//! `apply_superlevel_transaction`, the records of the refusals of a packet, and the emission of the nodes of a component.
//!
//! The head ([`transaction_prefix`]) takes the frozen snapshot, counts the stale candidates, records the events of a kind the transaction does not carry, and returns
//! early where the oracle does: a packet with two live owners of one edge is a named refusal of the whole packet; a packet without a live candidate changes nothing.
//! Everything else (the symbolic closure, the runtime commit) is behind the boundary [`Prefix::Continue`], and a transaction implements the trait
//! `builder::Transaction` with it.
//!
//! The records and the emissions mutate the builder only through its own primitives (`record_obligation`, `emit`, the counters), in the order of the oracle.

use cftuv_core::exact::ExactCtx;

use crate::builder::{Builder, Counter, ProofBranchOrRefusal};
use crate::error::SkelResult;
use crate::plans::{ComponentPlan, EdgeContactPlan, SplitCutPlan, VertexMeetingPlan};
use crate::proof::{EdgeKey, ProofBranch, ProofDisposition};
use crate::queue::{CandidateEvent, EventKind};
use crate::skeleton::{SkeletonNode, SkeletonOutcome};
use crate::snapshot::{collect_superlevel_snapshot, Snapshot};
use crate::time::EventTime;

/// What the head of the transaction decided.
pub enum Prefix {
    /// The oracle's `apply_superlevel_transaction` returned here.
    Done,
    /// The symbolic closure is next: the frozen snapshot, and the budget of its outer and its junction generations (`max(8, 2 * vertices + incidents)`).
    Continue { snapshot: Box<Snapshot>, budget: i64 },
}

/// The head of `apply_superlevel_transaction(builder, level)`.
pub fn transaction_prefix(ctx: &mut ExactCtx<'_>, builder: &mut Builder, level: &[CandidateEvent]) -> SkelResult<Prefix> {
    let snapshot = collect_superlevel_snapshot(ctx, builder, level)?;
    builder.counters.bump(Counter::DiscardedStaleCandidates, snapshot.stale_candidates);
    for event in &snapshot.unsupported {
        record_unsupported(builder, event)?;
    }
    if !snapshot.duplicate_live_owner_edge_ids.is_empty() {
        record_duplicate_live_owner(builder, &snapshot, level)?;
        return Ok(Prefix::Done);
    }
    if snapshot.incidents.is_empty() {
        return Ok(Prefix::Done);
    }
    let budget = (2 * snapshot.vertices.len() as i64 + snapshot.incidents.len() as i64).max(8);
    Ok(Prefix::Continue { snapshot: Box::new(snapshot), budget })
}

fn unresolvable(builder: &mut Builder) {
    builder.counters.bump(Counter::SuperlevelUnresolvableComponents, 1);
    builder.refusal = Some(SkeletonOutcome::SuperlevelComponentUnresolvable);
}

/// `_record_unsupported(builder, event)`: an event of a kind the transaction does not carry is dropped, by name.
pub fn record_unsupported(builder: &mut Builder, event: &CandidateEvent) -> SkelResult<()> {
    builder.counters.bump(Counter::UnsupportedEventKindDropped, 1);
    let valid: Vec<i64> = [event.vertex, event.peer].into_iter().filter(|ident| 0 <= *ident && (*ident as usize) < builder.vertices.len()).collect();
    let carried = builder.edge_keys(&[event.edge]);
    builder.record_obligation(
        ProofBranchOrRefusal::Branch(ProofBranch::UnsupportedEventKind),
        ProofDisposition::UnsupportedEventKindDropped,
        &(valid, carried.clone(), carried),
        &event.time,
        Some(event.kind),
    )
}

/// `_record_duplicate_live_owner(builder, snapshot, level)`: two live vertices start spans of one edge: the whole packet is refused, with the vertices that own the edge.
pub fn record_duplicate_live_owner(builder: &mut Builder, snapshot: &Snapshot, level: &[CandidateEvent]) -> SkelResult<()> {
    let edge_ids = &snapshot.duplicate_live_owner_edge_ids;
    let mut vertex_ids: Vec<i64> = snapshot.vertices.iter().filter(|vertex| vertex.alive && edge_ids.contains(&vertex.next_edge)).map(|vertex| vertex.ident).collect();
    vertex_ids.sort_unstable();
    let participants = builder.edge_keys(edge_ids);
    unresolvable(builder);
    let time = level.first().map_or_else(|| std::rc::Rc::new(EventTime::zero()), |event| std::rc::Rc::clone(&event.time));
    builder.record_obligation(
        ProofBranchOrRefusal::Branch(ProofBranch::SuperlevelComponentUnresolvable),
        ProofDisposition::SuperlevelComponentUnresolvable,
        &(vertex_ids, participants.clone(), participants),
        &time,
        Some(EventKind::Multiway),
    )
}

/// `_record_symbolic_unresolvable(builder, snapshot, reason)`: a refusal of the closure or the commit, with the complete identity of the frozen packet.
pub fn record_symbolic_unresolvable(builder: &mut Builder, snapshot: &Snapshot, reason: &str) -> SkelResult<()> {
    let mut vertex_ids: Vec<i64> = snapshot.vertices.iter().filter(|vertex| vertex.alive).map(|vertex| vertex.ident).collect();
    vertex_ids.sort_unstable();
    let unique = |select: &dyn Fn(&crate::snapshot::Incident) -> &Vec<EdgeKey>| -> Vec<EdgeKey> {
        snapshot.incidents.iter().flat_map(|incident| select(incident).iter().cloned()).collect::<std::collections::BTreeSet<EdgeKey>>().into_iter().collect()
    };
    let (participants, targets) = (unique(&|incident| &incident.participants), unique(&|incident| &incident.target_participants));
    unresolvable(builder);
    builder.counters.bump_reason(reason);
    let time = snapshot.incidents.first().map_or_else(|| std::rc::Rc::new(EventTime::zero()), |incident| std::rc::Rc::clone(&incident.event.time));
    builder.record_obligation(
        ProofBranchOrRefusal::Branch(ProofBranch::SuperlevelComponentUnresolvable),
        ProofDisposition::SuperlevelComponentUnresolvable,
        &(vertex_ids, participants, targets),
        &time,
        Some(EventKind::Multiway),
    )
}

/// `_record_edge_span_debts(builder, events)`: an edge event accepted although the span that must collapse had no proof is a debt of the proof axis.
pub fn record_edge_span_debts(builder: &mut Builder, events: &[CandidateEvent]) -> SkelResult<()> {
    for event in events {
        if !event.span_unproven {
            continue;
        }
        let (vertex, peer) = (builder.vertex_at(event.vertex)?.clone(), builder.vertex_at(event.peer)?.clone());
        let identity = builder.edge_obligation_identity(&vertex, &peer);
        builder.counters.bump(Counter::EdgeCollapseSpanUnprovenButAccepted, 1);
        builder.record_obligation(ProofBranchOrRefusal::Branch(ProofBranch::EdgeCollapseSpanUnproven), ProofDisposition::EventAcceptedWithUnprovenSpan, &identity, &event.time, None)?;
    }
    Ok(())
}

/// `_emit_edge_contact(builder, contact)`: the node of a contact of edges (one `EDGE` node, or one `MULTIWAY` node when splits were absorbed into it).
pub fn emit_edge_contact(builder: &mut Builder, contact: &EdgeContactPlan) -> SkelResult<()> {
    let edge_events: Vec<CandidateEvent> = contact.events.iter().filter(|event| event.kind == EventKind::Edge).cloned().collect();
    record_edge_span_debts(builder, &edge_events)?;
    if contact.kinds == [EventKind::Edge] {
        let first = edge_events.first().ok_or_else(|| crate::error::SkelError::Unsupported("IndexError: a contact of edges without an edge event".to_string()))?;
        builder.emit(EventKind::Edge, first, contact.participants.clone(), &contact.dead_vertex_ids);
    } else {
        builder.nodes.push(SkeletonNode {
            kind: EventKind::Multiway,
            time: std::rc::Rc::clone(&contact.time),
            point: std::rc::Rc::clone(&contact.point),
            participants: contact.participants.clone(),
            converging_vertices: contact.dead_vertex_ids.len() as i64,
            kinds: contact.kinds.clone(),
            incidences: vec![contact.participants.clone()],
        });
        builder.node_vertex_ids.push(contact.dead_vertex_ids.clone());
        builder.counters.bump(Counter::SplitEvents, 1);
    }
    builder.counters.bump(Counter::EdgeEvents, 1);
    if contact.participants.len() > 3 {
        builder.counters.bump(Counter::MultiParticipantNodes, 1);
    }
    Ok(())
}

/// `_emit_meeting(builder, meeting)`: the node of a reconnection of vertices that met.
pub fn emit_meeting(builder: &mut Builder, meeting: &VertexMeetingPlan) -> SkelResult<()> {
    let first = meeting.events.first().ok_or_else(|| crate::error::SkelError::Unsupported("IndexError: a meeting without events".to_string()))?;
    builder.emit(EventKind::Split, first, meeting.participants.clone(), &meeting.meeting_vertex_ids);
    builder.counters.bump(Counter::VertexMeetingEvents, 1);
    if meeting.participants.len() > 3 {
        builder.counters.bump(Counter::MultiParticipantNodes, 1);
    }
    Ok(())
}

/// `_emit_component_nodes(builder, plan)`: the nodes of a component, contacts first, then meetings, then the cuts (one split node per event).
pub fn emit_component_nodes(builder: &mut Builder, plan: &ComponentPlan) -> SkelResult<()> {
    emit_nodes(builder, &plan.edge_contacts, &plan.vertex_meetings, &plan.split_cuts)
}

/// The same from the three lists a component is made of (the seams carry no more of a plan than the emission reads).
pub fn emit_nodes(builder: &mut Builder, contacts: &[EdgeContactPlan], meetings: &[VertexMeetingPlan], cuts: &[SplitCutPlan]) -> SkelResult<()> {
    for contact in contacts {
        emit_edge_contact(builder, contact)?;
    }
    for meeting in meetings {
        emit_meeting(builder, meeting)?;
    }
    for cut in cuts {
        let edge = cut.edge_id;
        for event in &cut.events {
            builder.emit_split_node(event.vertex, edge, event)?;
        }
    }
    Ok(())
}
