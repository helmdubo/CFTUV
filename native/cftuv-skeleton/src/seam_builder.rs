//! The differential seams of the builder and of the planning layer (test-only, WP-S3 and WP-S4): the state of the builder after `__init__`, the loop up to a transaction
//! (from the start or from a recorded state), the frozen snapshot of a packet, the plans of a snapshot and the symbolic materialisation. Same wire as `seam.rs`; the opcodes
//! run from 251 (the opcode is a sixteen-bit number: 251..255 and 300..).
//!
//! ```text
//! options  [dense hydration, march steps | none, budgeted]
//! state    [universe, edges, vertices, edge_start, edge_end, queue, nodes, node vertex ids, proof, refusal, graph, index, traces,
//!           line_id, edges_by_line, sets, now, memo, counters, next identity]
//!          edge    [ident, [a, b, c, q, line identity], span]
//!          vertex  [ident, prev edge, next edge, prev, next, birth, point, reflex, alive, [sliding sum, identity] | none]
//!          queue   [pushed, popped, counter, [[sequence, event], ...]]   event [kind, time, point, vertex, peer, edge, span_unproven]
//!          memo    [active, entries] (a state read by the seam) / [active, places, times] (a state answered by it)
//! ```
//! A part a seam does not need may be `None` in a state it reads (the snapshot seam is given the front and the memory only).
//!
//! The REPR NODES (`[tag, payload]` of `enc_repr` in the shim) carry the keys of a snapshot into [`Val`]s; the answers of the plan seams are `repr` TEXTS of the oracle's
//! dataclasses, compared byte for byte with `repr()` of what the oracle returned (an `int` against a `Fraction`, a tuple against a list: all differ in a `repr`).

use std::collections::BTreeSet;
use std::rc::Rc;
use std::time::Instant;

use cftuv_core::codec::Value;
use cftuv_core::exact::ExactCtx;

use crate::builder::{Builder, BuilderOptions, Counters, Edge, ResumeAt, RunEnd, SlidingValue, Step, Transaction, Vertex};
use crate::closure::plan_split_materialization;
use crate::error::SkelResult;
use crate::grid::Cell;
use crate::line::SupportLine;
use crate::motorcycle::TraceCandidateIndex;
use crate::plans::{plan_superlevel_components, PlanVal};
use crate::proof::{ProofLedger, ProofObligation};
use crate::pyval::Val;
use crate::queue::{CandidateEvent, EventKind, EventQueue};
use crate::seam::memo_of;
use crate::seam_graph::{cause_of, cells_of, cells_value, graph_of, graph_value, grid_of, grid_value, ints_of, ints_value, keys_of, keys_value, nanoseconds, obligation_value, polygon_of, trace_of, trace_value};
use crate::skeleton::{Skeleton, SkeletonNode};
use crate::snapshot::{collect_superlevel_snapshot, Incident, Snapshot, VertexSnapshot};
use crate::wire::{
    bad, fixed, flag_of, i64_of, int, int_of, list, optional, point_of, point_value, rat_of, speed_value, str_of, str_value, sum_of, time_of, time_value, u32_of, u64_of, Wire,
};

pub(crate) const SEAMS: &[(u16, &str)] = &[(251, "BUILDER_INIT"), (252, "BUILDER_RUN"), (253, "COLLECT_SNAPSHOT"), (254, "PLAN_COMPONENTS"), (255, "PLAN_SPLIT_MATERIALIZATION")];

// --------------------------------------------------------------------------
// repr nodes
// --------------------------------------------------------------------------

/// The value of a repr node `[tag, payload]` (the tags of `enc_repr`; a sum, a time and a point are not keys and are refused).
pub(crate) fn val_of(value: &Value) -> Wire<Val> {
    let [tag, payload] = fixed::<2>(value, "a repr node")?;
    let nodes = |items: &Value| -> Wire<Vec<Val>> { list(items, "repr items")?.iter().map(val_of).collect() };
    Ok(match u32_of(tag, "a repr tag")? {
        0 => Val::none(),
        1 => Val::boolean(flag_of(payload, "a repr bool")?),
        2 => Val::big(int_of(payload, "a repr int")?),
        3 => {
            let text = str_of(payload, "a repr string")?;
            if !text.is_ascii() {
                return Err(bad("a repr string must be ASCII"));
            }
            Val::str(&text)
        }
        4 => Val::frac(rat_of(payload, "a repr fraction")?),
        5 => Val::tuple(nodes(payload)?),
        6 => Val::list(nodes(payload)?),
        7 => {
            let [name, fields] = fixed::<2>(payload, "a repr dataclass")?;
            let fields = list(fields, "repr fields")?
                .iter()
                .map(|field| {
                    let [field_name, field_value] = fixed::<2>(field, "a repr field")?;
                    Ok((str_of(field_name, "a repr field name")?, val_of(field_value)?))
                })
                .collect::<Wire<Vec<(String, Val)>>>()?;
            Val::data_named(str_of(name, "a repr class name")?, fields)
        }
        8 => {
            let [class, member, inner] = fixed::<3>(payload, "a repr member")?;
            Val::member(str_of(class, "a repr enum class")?, str_of(member, "a repr enum member")?, val_of(inner)?)
        }
        _ => return Err(bad("a repr tag a key cannot carry")),
    })
}

// --------------------------------------------------------------------------
// events, nodes, obligations, options
// --------------------------------------------------------------------------

pub(crate) fn kind_of(value: &Value) -> Wire<EventKind> {
    EventKind::from_value(&str_of(value, "an event kind")?).ok_or_else(|| bad("an event kind"))
}

pub(crate) fn event_of(value: &Value) -> Wire<CandidateEvent> {
    let [kind, time, point, vertex, peer, edge, unproven] = fixed::<7>(value, "an event")?;
    Ok(CandidateEvent {
        kind: kind_of(kind)?,
        time: Rc::new(time_of(time)?),
        point: Rc::new(point_of(point)?),
        vertex: i64_of(vertex, "an event vertex")?,
        peer: i64_of(peer, "an event peer")?,
        edge: i64_of(edge, "an event edge")?,
        span_unproven: flag_of(unproven, "span_unproven")?,
    })
}

pub(crate) fn event_value(event: &CandidateEvent) -> Value {
    Value::List(vec![
        str_value(event.kind.value()),
        time_value(&event.time),
        point_value(&event.point),
        int(event.vertex),
        int(event.peer),
        int(event.edge),
        Value::Bool(event.span_unproven),
    ])
}

pub(crate) fn events_of(value: &Value, what: &str) -> Wire<Vec<CandidateEvent>> {
    list(value, what)?.iter().map(event_of).collect()
}

fn node_value(node: &SkeletonNode) -> Value {
    Value::List(vec![
        str_value(node.kind.value()),
        time_value(&node.time),
        point_value(&node.point),
        keys_value(&node.participants),
        int(node.converging_vertices),
        Value::List(node.kinds.iter().map(|kind| str_value(kind.value())).collect()),
        Value::List(node.incidences.iter().map(|incidence| keys_value(incidence)).collect()),
    ])
}

fn node_of(value: &Value) -> Wire<SkeletonNode> {
    let [kind, time, point, participants, converging, kinds, incidences] = fixed::<7>(value, "a node")?;
    Ok(SkeletonNode {
        kind: kind_of(kind)?,
        time: Rc::new(time_of(time)?),
        point: Rc::new(point_of(point)?),
        participants: keys_of(participants, "node participants")?,
        converging_vertices: i64_of(converging, "converging vertices")?,
        kinds: list(kinds, "node kinds")?.iter().map(kind_of).collect::<Wire<_>>()?,
        incidences: list(incidences, "node incidences")?.iter().map(|incidence| keys_of(incidence, "an incidence")).collect::<Wire<_>>()?,
    })
}

fn obligation_of(value: &Value) -> Wire<ProofObligation> {
    let [kind, cause, disposition, vertices, participants, targets, level, event_kind] = fixed::<8>(value, "an obligation")?;
    Ok(ProofObligation {
        cause: cause_of(kind, cause)?,
        disposition: crate::proof::ProofDisposition::from_value(&str_of(disposition, "a disposition")?).ok_or_else(|| bad("a disposition"))?,
        vertex_ids: ints_of(vertices, "obligation vertices")?,
        participant_edge_keys: keys_of(participants, "obligation keys")?,
        target_edge_keys: keys_of(targets, "obligation keys")?,
        level: time_of(level)?,
        event_kind: optional(event_kind, kind_of)?,
    })
}

/// `[dense hydration, march steps | none, budgeted, replay check]`.
pub(crate) fn options_of(value: &Value) -> Wire<BuilderOptions> {
    let [dense, march, budgeted, replay] = fixed::<4>(value, "the options")?;
    Ok(BuilderOptions {
        dense_hydration: flag_of(dense, "dense hydration")?,
        march_steps: optional(march, |found| i64_of(found, "the march steps"))?,
        budgeted: flag_of(budgeted, "budgeted")?,
        replay_check: flag_of(replay, "replay check")?,
        ..BuilderOptions::default()
    })
}

fn counters_value(counters: &Counters) -> Value {
    Value::List(counters.sorted().into_iter().map(|(name, count)| Value::List(vec![str_value(&name), int(count)])).collect())
}

fn counters_of(value: &Value) -> Wire<Counters> {
    let mut counters = Counters::default();
    let items = list(value, "counters")?
        .iter()
        .map(|item| {
            let [name, count] = fixed::<2>(item, "a counter")?;
            Ok((str_of(name, "a counter name")?, i64_of(count, "a counter value")?))
        })
        .collect::<Wire<Vec<(String, i64)>>>()?;
    counters.restore(&items).map_err(|error| bad(&format!("{error:?}")))?;
    Ok(counters)
}

// --------------------------------------------------------------------------
// the state
// --------------------------------------------------------------------------

fn line_full(line: &SupportLine) -> Value {
    Value::List(vec![int(line.a), int(line.b), int(line.c), speed_value(&line.q), int(line.ident)])
}

fn line_of(value: &Value) -> Wire<SupportLine> {
    let [a, b, c, q, ident] = fixed::<5>(value, "a support line")?;
    SupportLine::new(i64_of(a, "line a")?, i64_of(b, "line b")?, i128::try_from(&int_of(c, "line c")?).map_err(|_| bad("line c"))?, rat_of(q, "a line speed")?, u64_of(ident, "a line identity")?)
        .map_err(|error| bad(&format!("unsupported: {error:?}")))
}

fn pairs_value(pairs: impl Iterator<Item = (i64, i64)>) -> Value {
    Value::List(pairs.map(|(first, second)| Value::List(vec![int(first), int(second)])).collect())
}

fn pairs_of(value: &Value, what: &str) -> Wire<Vec<(i64, i64)>> {
    list(value, what)?
        .iter()
        .map(|pair| {
            let [first, second] = fixed::<2>(pair, what)?;
            Ok((i64_of(first, what)?, i64_of(second, what)?))
        })
        .collect()
}

fn sorted_pairs(table: &std::collections::HashMap<i64, i64, cftuv_canon::fxhash::FxBuild>) -> Vec<(i64, i64)> {
    let mut pairs: Vec<(i64, i64)> = table.iter().map(|(first, second)| (*first, *second)).collect();
    pairs.sort_unstable();
    pairs
}

fn cell_table_value(table: &[(i64, Vec<Cell>)]) -> Value {
    Value::List(table.iter().map(|(key, cells)| Value::List(vec![int(*key), cells_value(cells)])).collect())
}

fn cell_table_of(value: &Value, what: &str) -> Wire<Vec<(i64, Vec<Cell>)>> {
    list(value, what)?
        .iter()
        .map(|entry| {
            let [key, cells] = fixed::<2>(entry, what)?;
            Ok((i64_of(key, what)?, cells_of(cells)?))
        })
        .collect()
}

fn buckets_value(buckets: &[(Cell, Vec<i64>)]) -> Value {
    Value::List(buckets.iter().map(|(cell, idents)| Value::List(vec![int(cell.0), int(cell.1), ints_value(idents)])).collect())
}

fn index_value(index: &TraceCandidateIndex) -> Value {
    let (lines, vertices) = index.buckets();
    Value::List(vec![
        grid_value(&index.grid),
        int(index.speed_bound),
        cell_table_value(index.line_cell_table()),
        cell_table_value(index.vertex_cell_table()),
        buckets_value(lines),
        buckets_value(vertices),
    ])
}

/// The state of a builder as the seams answer it.
pub(crate) fn state_value(builder: &Builder) -> Value {
    let edges = builder.edges.iter().map(|edge| Value::List(vec![int(edge.ident), line_full(&edge.line), ints_value(&edge.span)])).collect();
    let vertices = builder
        .vertices
        .iter()
        .map(|vertex| {
            Value::List(vec![
                int(vertex.ident),
                int(vertex.prev_edge),
                int(vertex.next_edge),
                int(vertex.prev),
                int(vertex.next),
                time_value(&vertex.birth),
                point_value(&vertex.point),
                Value::Bool(vertex.reflex),
                Value::Bool(vertex.alive),
                vertex.sliding.as_ref().map_or(Value::None, |sliding| Value::List(vec![Value::Sum(sliding.value.clone()), int(sliding.ident)])),
            ])
        })
        .collect();
    let queue = Value::List(vec![
        int(builder.queue.pushed),
        int(builder.queue.popped),
        int(builder.queue.counter()),
        Value::List(builder.queue.entries().map(|(event, sequence)| Value::List(vec![int(sequence), event_value(event)])).collect()),
    ]);
    let (places, times) = builder.memo.len();
    Value::List(vec![
        Value::List(builder.prime_universe.iter().map(|prime| Value::Int(cftuv_core::num::IBig::from(prime.clone()))).collect()),
        Value::List(edges),
        Value::List(vertices),
        pairs_value(sorted_pairs(&builder.edge_start).into_iter()),
        pairs_value(sorted_pairs(&builder.edge_end).into_iter()),
        queue,
        Value::List(builder.nodes.iter().map(node_value).collect()),
        Value::List(builder.node_vertex_ids.iter().map(|ids| ints_value(ids)).collect()),
        Value::List(builder.proof.obligations().iter().map(obligation_value).collect()),
        builder.refusal.map_or(Value::None, |outcome| str_value(outcome.value())),
        builder.graph.as_ref().map_or(Value::None, graph_value),
        builder.index.as_ref().map_or(Value::None, index_value),
        Value::List(builder.traces.iter().map(|(vertex, trace)| Value::List(vec![int(*vertex), trace_value(trace)])).collect()),
        Value::List(builder.line_order.iter().map(|key| Value::List(vec![int(key.0), int(key.1), int(key.2), speed_value(&key.3)])).collect()),
        Value::List(builder.edges_by_line.iter().map(|edges| ints_value(edges)).collect()),
        Value::List(vec![
            ints_value(&builder.unindexed_reflex.iter().copied().collect::<Vec<_>>()),
            ints_value(&builder.sliding_vertices.iter().copied().collect::<Vec<_>>()),
            ints_value(&builder.fan_vertices.iter().copied().collect::<Vec<_>>()),
            pairs_value(builder.origin_vertex.iter().map(|(origin, vertex)| (*origin, *vertex))),
            int(builder.origin_count),
        ]),
        time_value(&builder.now),
        Value::List(vec![Value::Bool(builder.memo.active), int(places as u64), int(times as u64)]),
        counters_value(&builder.counters),
        int(builder.next_ident()),
    ])
}

fn optional_list<'a>(value: &'a Value, what: &str) -> Wire<&'a [Value]> {
    match value {
        Value::None => Ok(&[]),
        other => list(other, what),
    }
}

/// The builder of a state (see the module note): the front and the memory are always read, the rest when it is given.
pub(crate) fn builder_of(value: &Value, options: BuilderOptions) -> Wire<Builder> {
    let [universe, edges, vertices, edge_start, edge_end, queue, nodes, node_vertex_ids, proof, refusal, graph, index, traces, line_id, edges_by_line, sets, now, memo, counters, next_ident] =
        fixed::<20>(value, "a builder state")?;
    let universe = list(universe, "a prime universe")?.iter().map(|prime| crate::wire::ubig_of(prime, "a prime")).collect::<Wire<Vec<_>>>()?;
    let polygon = crate::polygon::Polygon { loops: Vec::new(), fans: Vec::new() };
    let mut builder = Builder::empty(polygon, options, universe);
    for entry in list(edges, "edges")? {
        let [ident, line, span] = fixed::<3>(entry, "an edge")?;
        builder.edges.push(Edge { ident: i64_of(ident, "an edge identity")?, line: Rc::new(line_of(line)?), span: Rc::new(ints_of(span, "an edge span")?) });
    }
    for entry in list(vertices, "vertices")? {
        let [ident, prev_edge, next_edge, prev, next, birth, point, reflex, alive, sliding] = fixed::<10>(entry, "a vertex")?;
        let sliding = optional(sliding, |found| {
            let [sum, identity] = fixed::<2>(found, "a sliding projection")?;
            Ok(SlidingValue { value: sum_of(sum, "a sliding value")?.clone(), ident: u64_of(identity, "a sliding identity")? })
        })?;
        builder.vertices.push(Vertex {
            ident: i64_of(ident, "a vertex identity")?,
            prev_edge: i64_of(prev_edge, "a vertex edge")?,
            next_edge: i64_of(next_edge, "a vertex edge")?,
            prev: i64_of(prev, "a vertex link")?,
            next: i64_of(next, "a vertex link")?,
            birth: Rc::new(time_of(birth)?),
            point: Rc::new(point_of(point)?),
            reflex: flag_of(reflex, "a reflex flag")?,
            alive: flag_of(alive, "an alive flag")?,
            sliding,
        });
    }
    for (first, second) in pairs_of(edge_start, "edge_start")? {
        builder.edge_start.insert(first, second);
    }
    for (first, second) in pairs_of(edge_end, "edge_end")? {
        builder.edge_end.insert(first, second);
    }
    if !matches!(queue, Value::None) {
        let [pushed, popped, counter, entries] = fixed::<4>(queue, "a queue")?;
        let heap = list(entries, "queue entries")?
            .iter()
            .map(|entry| {
                let [sequence, event] = fixed::<2>(entry, "a queue entry")?;
                Ok((event_of(event)?, u64_of(sequence, "a sequence")?))
            })
            .collect::<Wire<Vec<_>>>()?;
        builder.queue = EventQueue::from_parts(heap, u64_of(counter, "the counter")?, u64_of(pushed, "pushed")?, u64_of(popped, "popped")?);
    }
    builder.nodes = optional_list(nodes, "nodes")?.iter().map(node_of).collect::<Wire<_>>()?;
    builder.node_vertex_ids = optional_list(node_vertex_ids, "node vertex ids")?.iter().map(|ids| ints_of(ids, "node vertex ids")).collect::<Wire<_>>()?;
    builder.proof = ProofLedger::from_obligations(optional_list(proof, "the proof ledger")?.iter().map(obligation_of).collect::<Wire<_>>()?);
    builder.refusal = optional(refusal, |found| crate::skeleton::SkeletonOutcome::from_value(&str_of(found, "a refusal")?).ok_or_else(|| bad("a refusal outcome")))?;
    builder.graph = optional(graph, graph_of)?;
    builder.index = optional(index, |found| {
        let [grid, speed_bound, line_cells, vertex_cells, _lines, _vertices] = fixed::<6>(found, "an index")?;
        Ok(TraceCandidateIndex::restore(grid_of(grid)?, i64_of(speed_bound, "the speed bound")?, cell_table_of(line_cells, "line cells")?, cell_table_of(vertex_cells, "vertex cells")?))
    })?;
    for entry in optional_list(traces, "traces")? {
        let [vertex, trace] = fixed::<2>(entry, "a trace entry")?;
        builder.traces.insert(i64_of(vertex, "a trace vertex")?, trace_of(trace)?);
    }
    if matches!(line_id, Value::None) {
        for index in 0..builder.edges.len() {
            let edge = builder.edges[index].clone();
            builder.register_line(&edge);
        }
    } else {
        for key in list(line_id, "line_id")? {
            let [a, b, c, q] = fixed::<4>(key, "a line key")?;
            let key = (i64_of(a, "line a")?, i64_of(b, "line b")?, i128::try_from(&int_of(c, "line c")?).map_err(|_| bad("line c"))?, rat_of(q, "a line speed")?);
            builder.line_id.insert(key.clone(), builder.line_order.len());
            builder.line_order.push(key);
        }
        builder.edges_by_line = list(edges_by_line, "edges_by_line")?.iter().map(|edges| ints_of(edges, "edges of a line")).collect::<Wire<_>>()?;
    }
    if !matches!(sets, Value::None) {
        let [unindexed, sliding, fans, origins, origin_count] = fixed::<5>(sets, "the sets")?;
        builder.unindexed_reflex = ints_of(unindexed, "unindexed")?.into_iter().collect::<BTreeSet<_>>();
        builder.sliding_vertices = ints_of(sliding, "sliding vertices")?.into_iter().collect::<BTreeSet<_>>();
        builder.fan_vertices = ints_of(fans, "fan vertices")?.into_iter().collect::<BTreeSet<_>>();
        builder.origin_vertex = pairs_of(origins, "origin vertices")?.into_iter().collect();
        builder.origin_count = i64_of(origin_count, "the origin count")?;
    }
    builder.now = Rc::new(time_of(now)?);
    builder.memo = memo_of(memo)?;
    if !matches!(counters, Value::None) {
        builder.counters = counters_of(counters)?;
    }
    builder.set_next_ident(u64_of(next_ident, "the next identity")?);
    Ok(builder)
}

fn skeleton_value(skeleton: &Skeleton) -> Value {
    Value::List(vec![
        str_value(skeleton.outcome.value()),
        Value::List(skeleton.nodes.iter().map(node_value).collect()),
        int(skeleton.levels),
        Value::List(skeleton.counters.iter().map(|(name, count)| Value::List(vec![str_value(name), int(*count)])).collect()),
        str_value(skeleton.proof_status.value()),
        Value::List(skeleton.proof_obligations.iter().map(obligation_value).collect()),
    ])
}

// --------------------------------------------------------------------------
// the snapshot
// --------------------------------------------------------------------------

fn opt_val(value: &Value, what: &str) -> Wire<Option<Val>> {
    optional(value, |found| val_of(found)).map_err(|error| bad(&format!("{what}: {error}")))
}

fn ray_of(value: &Value, what: &str) -> Wire<(i64, i64)> {
    let [first, second] = fixed::<2>(value, what)?;
    Ok((i64_of(first, what)?, i64_of(second, what)?))
}

fn incident_of(value: &Value) -> Wire<Incident> {
    let [event, vertex_ids, edge_occurrences, participants, target_participants, point_key, met_vertex_id, met_adjacent, projection, start, end, emitter, peer, occurrence, ray] =
        fixed::<15>(value, "an incident")?;
    Ok(Incident {
        event: event_of(event)?,
        vertex_ids: ints_of(vertex_ids, "vertex ids")?,
        edge_occurrences: ints_of(edge_occurrences, "edge occurrences")?,
        participants: keys_of(participants, "participants")?,
        target_participants: keys_of(target_participants, "target participants")?,
        point_key: val_of(point_key)?,
        met_vertex_id: optional(met_vertex_id, |found| i64_of(found, "a met vertex"))?,
        met_adjacent: flag_of(met_adjacent, "met_adjacent")?,
        target_projection: optional(projection, |found| Ok(sum_of(found, "a projection")?.clone()))?,
        target_start_id: optional(start, |found| i64_of(found, "a target start"))?,
        target_end_id: optional(end, |found| i64_of(found, "a target end"))?,
        emitter_key: val_of(emitter)?,
        peer_key: val_of(peer)?,
        target_occurrence: opt_val(occurrence, "the target occurrence")?,
        target_ray: optional(ray, |found| ray_of(found, "a target ray"))?,
        sort_cache: std::cell::OnceCell::new(),
        identity_cache: std::cell::OnceCell::new(),
    })
}

pub(crate) fn vertex_snapshot_of(value: &Value) -> Wire<VertexSnapshot> {
    let [ident, prev, next, prev_edge, next_edge, alive, incoming, outgoing, point_key, prev_occurrence, next_occurrence] = fixed::<11>(value, "a vertex snapshot")?;
    Ok(VertexSnapshot {
        ident: i64_of(ident, "an identity")?,
        prev: i64_of(prev, "prev")?,
        next: i64_of(next, "next")?,
        prev_edge: i64_of(prev_edge, "prev edge")?,
        next_edge: i64_of(next_edge, "next edge")?,
        alive: flag_of(alive, "alive")?,
        incoming_ray: ray_of(incoming, "the incoming ray")?,
        outgoing_ray: ray_of(outgoing, "the outgoing ray")?,
        point_key: opt_val(point_key, "a point key")?,
        prev_occurrence: opt_val(prev_occurrence, "an occurrence")?,
        next_occurrence: opt_val(next_occurrence, "an occurrence")?,
    })
}

pub(crate) fn snapshot_of(value: &Value) -> Wire<Snapshot> {
    let [incidents, vertices, unsupported, stale, duplicates] = fixed::<5>(value, "a snapshot")?;
    Ok(Snapshot {
        incidents: list(incidents, "incidents")?.iter().map(incident_of).collect::<Wire<_>>()?,
        vertices: list(vertices, "snapshot vertices")?.iter().map(vertex_snapshot_of).collect::<Wire<_>>()?,
        unsupported: events_of(unsupported, "unsupported events")?,
        stale_candidates: i64_of(stale, "the stale count")?,
        duplicate_live_owner_edge_ids: ints_of(duplicates, "duplicate owners")?,
    })
}

fn opt(value: Option<Val>) -> Val {
    value.unwrap_or_else(Val::none)
}

fn int_pair(pair: (i64, i64)) -> Val {
    Val::ints(&[pair.0, pair.1])
}

impl PlanVal for VertexSnapshot {
    fn to_val(&self) -> Val {
        Val::data(
            "_VertexSnapshot",
            vec![
                ("ident", Val::int(self.ident)),
                ("prev", Val::int(self.prev)),
                ("next", Val::int(self.next)),
                ("prev_edge", Val::int(self.prev_edge)),
                ("next_edge", Val::int(self.next_edge)),
                ("alive", Val::boolean(self.alive)),
                ("incoming_ray", int_pair(self.incoming_ray)),
                ("outgoing_ray", int_pair(self.outgoing_ray)),
                ("point_key", opt(self.point_key.clone())),
                ("prev_occurrence", opt(self.prev_occurrence.clone())),
                ("next_occurrence", opt(self.next_occurrence.clone())),
            ],
        )
    }
}

fn keys_val(keys: &[Vec<i64>]) -> Val {
    Val::tuple(keys.iter().map(|key| Val::ints(key)).collect())
}

impl PlanVal for Incident {
    fn to_val(&self) -> Val {
        let sum = |sum: &cftuv_core::sqrt_sum::SqrtSum| Val::data("SqrtSumV1", vec![("terms", Val::terms_of(sum))]);
        Val::data(
            "SuperlevelIncidentV1",
            vec![
                ("event", self.event.to_val()),
                ("vertex_ids", Val::ints(&self.vertex_ids)),
                ("edge_occurrences", Val::ints(&self.edge_occurrences)),
                ("participants", keys_val(&self.participants)),
                ("target_participants", keys_val(&self.target_participants)),
                ("point_key", self.point_key.clone()),
                ("met_vertex_id", self.met_vertex_id.map_or_else(Val::none, Val::int)),
                ("met_adjacent", Val::boolean(self.met_adjacent)),
                ("target_projection", self.target_projection.as_ref().map_or_else(Val::none, sum)),
                ("target_start_id", self.target_start_id.map_or_else(Val::none, Val::int)),
                ("target_end_id", self.target_end_id.map_or_else(Val::none, Val::int)),
                ("emitter_key", self.emitter_key.clone()),
                ("peer_key", self.peer_key.clone()),
                ("target_occurrence", opt(self.target_occurrence.clone())),
                ("target_ray", self.target_ray.map_or_else(Val::none, int_pair)),
            ],
        )
    }
}

impl PlanVal for Snapshot {
    fn to_val(&self) -> Val {
        Val::data(
            "SuperlevelSnapshotV1",
            vec![
                ("incidents", Val::tuple(self.incidents.iter().map(PlanVal::to_val).collect())),
                ("vertices", Val::tuple(self.vertices.iter().map(PlanVal::to_val).collect())),
                ("unsupported", Val::tuple(self.unsupported.iter().map(PlanVal::to_val).collect())),
                ("stale_candidates", Val::int(self.stale_candidates)),
                ("duplicate_live_owner_edge_ids", Val::ints(&self.duplicate_live_owner_edge_ids)),
            ],
        )
    }
}

// --------------------------------------------------------------------------
// the seams
// --------------------------------------------------------------------------

/// The transaction that stops the loop at its first call (the seams of the loop compare the state there).
struct StopAtTransaction;

impl Transaction for StopAtTransaction {
    fn apply(&mut self, _builder: &mut Builder, _ctx: &mut ExactCtx<'_>, _level: &[CandidateEvent]) -> SkelResult<Step> {
        Ok(Step::Stop)
    }
}

fn text_value(val: &Val) -> Value {
    str_value(&val.repr())
}

/// `[0, levels, level, state]` when the loop stopped at a transaction, `[1, skeleton]` when it finished.
fn run_end_value(end: RunEnd, builder: &Builder) -> Value {
    match end {
        RunEnd::Stopped { levels, level } => Value::List(vec![int(0u8), int(levels), Value::List(level.iter().map(event_value).collect()), state_value(builder)]),
        RunEnd::Finished(skeleton) => Value::List(vec![int(1u8), skeleton_value(&skeleton)]),
    }
}

pub(crate) fn dispatch(code: u16, args: &[Value], ctx: &mut ExactCtx<'_>, extras: &mut Vec<Value>) -> Wire<SkelResult<Value>> {
    let at = |index: usize| args.get(index).ok_or_else(|| bad("too few arguments"));
    Ok(match code {
        251 => {
            let (polygon, options) = (polygon_of(at(0)?)?, options_of(at(1)?)?);
            let started = Instant::now();
            let built = Builder::new(ctx, polygon, options);
            extras.push(nanoseconds(started));
            extras.push(str_value(&ctx.budget.superlevel));
            built.map(|builder| state_value(&builder))
        }
        252 => {
            let options = options_of(at(1)?)?;
            let limit = i64_of(at(2)?, "the level limit")?;
            let started = Instant::now();
            let mut builder = match i64_of(at(0)?, "the mode")? {
                0 => match Builder::new(ctx, polygon_of(at(3)?)?, options) {
                    Ok(builder) => builder,
                    Err(error) => {
                        extras.push(nanoseconds(started));
                        extras.push(int(0u8));
                        extras.push(str_value(&ctx.budget.superlevel));
                        return Ok(Err(error));
                    }
                },
                1 => builder_of(at(3)?, options)?,
                _ => return Err(bad("the run mode")),
            };
            let seeded = started.elapsed().as_nanos() as u64;
            let resume = match (i64_of(at(4)?, "the resume point")?, i64_of(at(5)?, "the levels")?) {
                (0, levels) => ResumeAt::Top { levels },
                (1, levels) => ResumeAt::AfterTransaction { levels },
                _ => return Err(bad("the resume point")),
            };
            let looped = Instant::now();
            let end = builder.run_from(ctx, limit, resume, &mut StopAtTransaction);
            extras.push(int(looped.elapsed().as_nanos() as u64));
            extras.push(int(seeded));
            extras.push(str_value(&ctx.budget.superlevel));
            end.map(|end| run_end_value(end, &builder))
        }
        253 => {
            let mut builder = builder_of(at(0)?, options_of(at(2)?)?)?;
            let level = events_of(at(1)?, "the level")?;
            let before = builder.memo.len();
            let started = Instant::now();
            let snapshot = collect_superlevel_snapshot(ctx, &mut builder, &level);
            extras.push(nanoseconds(started));
            let after = builder.memo.len();
            snapshot.map(|snapshot| Value::List(vec![text_value(&snapshot.to_val()), crate::seam::growth_value(before, after)]))
        }
        254 => {
            let snapshot = snapshot_of(at(0)?)?;
            let started = Instant::now();
            let plans = plan_superlevel_components(ctx, &snapshot);
            extras.push(nanoseconds(started));
            plans.map(|plans| text_value(&Val::tuple(plans.iter().map(PlanVal::to_val).collect())))
        }
        255 => {
            let snapshot = snapshot_of(at(0)?)?;
            let started = Instant::now();
            let materialization = plan_split_materialization(ctx, &snapshot);
            extras.push(nanoseconds(started));
            materialization.map(|found| text_value(&found.to_val()))
        }
        other => return crate::seam_primitive::dispatch(other, args, ctx, extras),
    })
}
