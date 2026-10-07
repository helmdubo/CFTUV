//! The seam of the primitives of the builder (test-only, WP-S3): the methods the transaction of a packet calls on the builder and the helpers of `superlevel`, each on the state
//! the oracle has at the call, answering the result and the whole state after it. Opcode 300.
//!
//! `[state, options, op, future now | none, arguments...]` (the time is the `now` of the oracle's `_FutureQueueV1` while it wraps the queue); the answer `[result, state]` (the state as `seam_builder::state_value` writes it). The operations:
//!
//! ```text
//!  0 twin [edge]                          -> the new edge             10 front_vertex_met_by [event]       -> [vertex | none, adjacent]
//!  1 new_vertex [prev edge, next edge,    -> the new vertex           11 edge_event_is_live [event]        -> bool
//!      prev, next, birth, point]                                      12 split_is_live [event]             -> bool
//!  2 emit [kind, event, participants,     -> none                     13 position [vertex, time]           -> point | none
//!      converged ids]                                                 14 record_unsupported [event]
//!  3 emit_split_node [vertex, edge, event]                            15 record_duplicate_live_owner [snapshot, level]
//!  4 refuse [reason, ids, participants, targets]                      16 record_symbolic_unresolvable [snapshot, reason]
//!  5 enqueue_for [vertex]                                             17 record_edge_span_debts [events]
//!  6 enqueue_edge_event [vertex]                                      18 emit_edge_contact [events, time, point, participants, dead, kinds]
//!  7 enqueue_splits_against [edge, excluded]                          19 emit_meeting [events, participants, vertices]
//!  8 register [vertex]                                                20 emit_component_nodes [contacts, meetings, cuts]
//!  9 record_obligation [cause kind, cause, disposition, ids,          21 transaction_prefix [level] -> [0 done | 1 continue, budget]
//!      participants, targets, level, event kind | none]
//! ```
//! The cost of the call is the cost answer of the seam (`seam.rs`).

use std::collections::BTreeSet;
use std::rc::Rc;
use std::time::Instant;

use cftuv_core::codec::Value;
use cftuv_core::exact::ExactCtx;

use crate::builder::{Builder, NewVertex, ProofBranchOrRefusal};
use crate::candidate::CandidateRefusal;
use crate::error::SkelResult;
use crate::plans::{EdgeContactPlan, SplitCutPlan, VertexMeetingPlan};
use crate::proof::ProofDisposition;
use crate::pyval::Val;
use crate::seam_builder::{builder_of, event_of, events_of, kind_of, options_of, snapshot_of, state_value};
use crate::seam_graph::{cause_of, ints_of, keys_of, nanoseconds};
use crate::superlevel::{emit_edge_contact, emit_meeting, emit_nodes, record_duplicate_live_owner, record_edge_span_debts, record_symbolic_unresolvable, record_unsupported, transaction_prefix, Prefix};
use crate::time::EventTime;
use crate::wire::{bad, fixed, i64_of, int, list, optional, point_of, point_value, str_of, time_of, Wire};

pub(crate) const SEAMS: &[(u16, &str)] = &[(300, "BUILDER_PRIMITIVE")];

fn identity_of(ids: &Value, participants: &Value, targets: &Value) -> Wire<(Vec<i64>, Vec<Vec<i64>>, Vec<Vec<i64>>)> {
    Ok((ints_of(ids, "proof ids")?, keys_of(participants, "proof keys")?, keys_of(targets, "proof keys")?))
}

fn contact_of(value: &Value) -> Wire<EdgeContactPlan> {
    let [events, time, point, participants, dead, kinds] = fixed::<6>(value, "a contact")?;
    Ok(EdgeContactPlan {
        events: events_of(events, "contact events")?,
        time: Rc::new(time_of(time)?),
        point: Rc::new(point_of(point)?),
        point_key: Val::none(),
        participants: keys_of(participants, "contact participants")?,
        dead_vertex_ids: ints_of(dead, "contact dead ids")?,
        chains: Vec::new(),
        births: Vec::new(),
        kinds: list(kinds, "contact kinds")?.iter().map(kind_of).collect::<Wire<_>>()?,
    })
}

fn meeting_of(value: &Value) -> Wire<VertexMeetingPlan> {
    let [events, participants, vertices] = fixed::<3>(value, "a meeting")?;
    let events = events_of(events, "meeting events")?;
    let first = events.first().ok_or_else(|| bad("a meeting without events"))?;
    Ok(VertexMeetingPlan {
        time: Rc::clone(&first.time),
        point: Rc::clone(&first.point),
        events,
        meeting_vertex_ids: ints_of(vertices, "meeting vertices")?,
        pairs: Vec::new(),
        participants: keys_of(participants, "meeting participants")?,
        births: Vec::new(),
    })
}

fn cut_of(value: &Value) -> Wire<SplitCutPlan> {
    let [edge, events] = fixed::<2>(value, "a cut")?;
    Ok(SplitCutPlan {
        edge_id: i64_of(edge, "a cut edge")?,
        target_occurrence: Val::none(),
        events: events_of(events, "cut events")?,
        segment_occurrences: Vec::new(),
        births: Vec::new(),
        final_birth_ports: Vec::new(),
    })
}

fn run(builder: &mut Builder, ctx: &mut ExactCtx<'_>, op: i64, args: &[Value]) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    Ok(match op {
        0 => builder.twin(i64_of(at(0)?, "an edge")?).map(int),
        1 => {
            let fields = NewVertex {
                prev_edge: i64_of(at(0)?, "prev edge")?,
                next_edge: i64_of(at(1)?, "next edge")?,
                prev: i64_of(at(2)?, "prev")?,
                next: i64_of(at(3)?, "next")?,
                birth: Rc::new(time_of(at(4)?)?),
                point: Rc::new(point_of(at(5)?)?),
            };
            builder.new_vertex(ctx, fields).map(int)
        }
        2 => {
            let (kind, event) = (kind_of(at(0)?)?, event_of(at(1)?)?);
            builder.emit(kind, &event, keys_of(at(2)?, "participants")?, &ints_of(at(3)?, "converged ids")?);
            Ok(Value::None)
        }
        3 => builder.emit_split_node(i64_of(at(0)?, "a vertex")?, i64_of(at(1)?, "an edge")?, &event_of(at(2)?)?).map(|()| Value::None),
        4 => {
            let reason = CandidateRefusal::from_value(&str_of(at(0)?, "a reason")?).ok_or_else(|| bad("a refusal name"))?;
            builder.refuse(reason, &identity_of(at(1)?, at(2)?, at(3)?)?).map(|()| Value::None)
        }
        5 => builder.enqueue_for(ctx, i64_of(at(0)?, "a vertex")?).map(|()| Value::None),
        6 => builder.enqueue_edge_event(ctx, i64_of(at(0)?, "a vertex")?).map(|()| Value::None),
        7 => {
            let excluded: BTreeSet<i64> = ints_of(at(1)?, "excluded vertices")?.into_iter().collect();
            builder.enqueue_splits_against(ctx, i64_of(at(0)?, "an edge")?, &excluded).map(|()| Value::None)
        }
        8 => {
            builder.register(i64_of(at(0)?, "a vertex")?);
            Ok(Value::None)
        }
        9 => {
            let cause = match cause_of(at(0)?, at(1)?)? {
                crate::proof::ProofCause::Refusal(reason) => ProofBranchOrRefusal::Refusal(reason),
                crate::proof::ProofCause::Branch(branch) => ProofBranchOrRefusal::Branch(branch),
            };
            let disposition = ProofDisposition::from_value(&str_of(at(2)?, "a disposition")?).ok_or_else(|| bad("a disposition"))?;
            let level: EventTime = time_of(at(6)?)?;
            let kind = optional(at(7)?, kind_of)?;
            builder.record_obligation(cause, disposition, &identity_of(at(3)?, at(4)?, at(5)?)?, &level, kind).map(|()| Value::None)
        }
        10 => builder
            .front_vertex_met_by(ctx, &event_of(at(0)?)?)
            .map(|(met, adjacent)| Value::List(vec![met.map_or(Value::None, int), Value::Bool(adjacent)])),
        11 => builder.edge_event_is_live(&event_of(at(0)?)?).map(Value::Bool),
        12 => builder.split_is_live(ctx, &event_of(at(0)?)?).map(Value::Bool),
        13 => builder.position(ctx, i64_of(at(0)?, "a vertex")?, &time_of(at(1)?)?).map(|point| point.map_or(Value::None, |found| point_value(&found))),
        14 => record_unsupported(builder, &event_of(at(0)?)?).map(|()| Value::None),
        15 => {
            let (snapshot, level) = (snapshot_of(at(0)?)?, events_of(at(1)?, "the level")?);
            record_duplicate_live_owner(builder, &snapshot, &level).map(|()| Value::None)
        }
        16 => {
            let (snapshot, reason) = (snapshot_of(at(0)?)?, str_of(at(1)?, "a reason")?);
            record_symbolic_unresolvable(builder, &snapshot, &reason).map(|()| Value::None)
        }
        17 => record_edge_span_debts(builder, &events_of(at(0)?, "events")?).map(|()| Value::None),
        18 => emit_edge_contact(builder, &contact_of(at(0)?)?).map(|()| Value::None),
        19 => emit_meeting(builder, &meeting_of(at(0)?)?).map(|()| Value::None),
        20 => {
            let contacts = list(at(0)?, "contacts")?.iter().map(contact_of).collect::<Wire<Vec<_>>>()?;
            let meetings = list(at(1)?, "meetings")?.iter().map(meeting_of).collect::<Wire<Vec<_>>>()?;
            let cuts = list(at(2)?, "cuts")?.iter().map(cut_of).collect::<Wire<Vec<_>>>()?;
            emit_nodes(builder, &contacts, &meetings, &cuts).map(|()| Value::None)
        }
        21 => transaction_prefix(ctx, builder, &events_of(at(0)?, "the level")?).map(|prefix| match prefix {
            Prefix::Done => Value::List(vec![int(0u8), int(0u8)]),
            Prefix::Continue { budget, .. } => Value::List(vec![int(1u8), int(budget)]),
        }),
        _ => return Err(bad("the primitive operation")),
    })
}

pub(crate) fn dispatch(code: u16, args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<SkelResult<Value>> {
    if code != 300 {
        return Err(bad(&format!("unknown skeleton seam opcode {code}")));
    }
    let [state, options, op, future, rest @ ..] = args else {
        return Err(bad("too few arguments"));
    };
    let mut builder = builder_of(state, options_of(options)?)?;
    builder.future_only = optional(future, |found| Ok(Rc::new(time_of(found)?)))?;
    let started = Instant::now();
    let answered = run(&mut builder, ctx, i64_of(op, "the operation")?, rest)?;
    extras.push(nanoseconds(started));
    Ok(answered.map(|result| Value::List(vec![result, state_value(&builder)])))
}
