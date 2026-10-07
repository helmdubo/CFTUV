//! The outer fixed point of the symbolic closure of a packet (`wavefront/symbolic_superlevel_coordinator.py`): root splits first, then the unified junction closure.
//!
//! [`plan_symbolic_superlevel_closure`] rebuilds the packet from F0 (the frozen state before the contact) whenever the stable set of interior contacts grows: every round the
//! contacts are compiled back into incidents of the frozen packet, the packet is planned again (`plan_split_materialization`), the plan becomes an overlay, and the overlay is
//! asked for new interior contacts. When the set is stable the mixed generations of the overlay are planned (`plan_mixed_generations`) AND PLANNED AGAIN: the second run is the
//! replay that must reach the same contacts and the same overlay (a named refusal otherwise), and its cost is part of the transaction. Every refusal is a named reason, never an
//! empty answer.
//!
//! STRUCTURE FOR A CHANGE OF THE ORACLE. The units a simplification of the oracle would remove or merge are separate functions, each called from one place: the replay
//! ([`replay_unit`], called once by [`plan_symbolic_superlevel_closure`]; its refusal is a reason of that call), the last discovery pass of the initial closure
//! (`initial_interior_closure`), and the three passes of every round of the generations (`contacts`: interior, endpoint, edge; `generations::Discovery`). The gate of the law of a
//! split candidate on this path is [`crate::contacts::SYMBOLIC_GATE`]. Mirroring such a change is a deletion of a unit (or the change of that constant), not a rewrite.

use std::rc::Rc;

use cftuv_core::exact::ExactCtx;

use crate::builder::Builder;
use crate::closure::{plan_split_materialization, Materialization};
use crate::component::{overlay_signature, SignatureMemo};
use crate::contacts::{compile_contacts, contacts_equal, discover_interior_split_contacts, initial_interior_contacts, merge_symbolic_split_contacts, SymSplitContact};
use crate::error::{SkelError, SkelResult};
use crate::generations::{fixed_point_val, plan_mixed_generations, JunctionFixedPoint};
use crate::omap::OrderedMap;
use crate::overlay::{build_f0_overlay, build_symbolic_overlay, overlay_val, with_line_ports, Overlay};
use crate::plans::{PlanVal, Resolution};
use crate::pyval::Val;
use crate::profile::{timed, Phase};
use crate::queue::EventKind;
use crate::snapshot::{Incident, Snapshot};
use crate::superlevel::record_symbolic_unresolvable;
use crate::time::TimeRef;

/// `SymbolicSuperlevelClosureV1`.
#[derive(Clone)]
pub struct SymbolicSuperlevelClosure {
    pub materialization: Option<Materialization>,
    pub overlay: Option<Overlay>,
    pub split_contacts: Vec<SymSplitContact>,
    pub junction: Option<JunctionFixedPoint>,
    pub outer_iterations: i64,
    pub signatures: Vec<Val>,
    pub canonical_batch_count: i64,
    pub unresolved_reason: Option<&'static str>,
}

/// `_refusal(materialization, contacts, junction, iterations, signatures, reason)`.
fn refusal(materialization: Option<Materialization>, contacts: Vec<SymSplitContact>, junction: Option<JunctionFixedPoint>, iterations: i64, signatures: Vec<Val>, reason: &'static str) -> SymbolicSuperlevelClosure {
    SymbolicSuperlevelClosure { materialization, overlay: None, split_contacts: contacts, junction, outer_iterations: iterations, signatures, canonical_batch_count: 0, unresolved_reason: Some(reason) }
}

/// An incident that is a cut of the INTERIOR of a span (a split whose point is not an end of its target occurrence): the ones the contacts replace.
fn is_interior_split(incident: &Incident) -> bool {
    let Some(occurrence) = &incident.target_occurrence else {
        return false;
    };
    incident.event.kind == EventKind::Split && occurrence.get(1) != Some(&incident.point_key) && occurrence.get(2) != Some(&incident.point_key)
}

/// What `_rebuild_splits` answers: the materialization, the overlay, the reason.
type Rebuilt = (Option<Materialization>, Option<Overlay>, Option<&'static str>);

/// `_rebuild_splits(builder, snapshot, contacts, time)`: one mixed frozen-prestate plan rebuilt from the F0 contact authority.
fn rebuild_splits(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, contacts: &[SymSplitContact], time: &TimeRef) -> SkelResult<Rebuilt> {
    let Some(ported) = timed(Phase::ClosureOverlay, || with_line_ports(ctx, builder, &snapshot.vertices, time))? else {
        return Ok((None, None, Some("SYMBOLIC_SPLIT_LINE_PORT_HYDRATION_UNRESOLVABLE")));
    };
    let (compiled, reason) = timed(Phase::ClosureCompile, || compile_contacts(ctx, builder, &ported, contacts))?;
    if reason.is_some() {
        return Ok((None, None, reason));
    }
    let mut incidents: Vec<Incident> = snapshot.incidents.iter().filter(|incident| !is_interior_split(incident)).cloned().collect();
    incidents.extend(compiled);
    incidents.sort_by(|left, right| left.sort_key().cmp(right.sort_key()));
    let working = Snapshot {
        incidents,
        vertices: ported,
        unsupported: snapshot.unsupported.clone(),
        stale_candidates: snapshot.stale_candidates,
        duplicate_live_owner_edge_ids: snapshot.duplicate_live_owner_edge_ids.clone(),
    };
    let materialization = timed(Phase::ClosurePlans, || plan_split_materialization(ctx, &working))?;
    if let Some(reason) = materialization.unresolved_reason {
        return Ok((Some(materialization), None, Some(reason)));
    }
    if materialization.plans.iter().any(|plan| plan.resolution == Resolution::Unresolvable) {
        return Ok((Some(materialization), None, Some("SYMBOLIC_UNIFIED_CONTACT_COMPONENT_UNRESOLVABLE")));
    }
    let overlay = timed(Phase::ClosureOverlay, || build_symbolic_overlay(ctx, builder, &working.vertices, &materialization, true))?;
    let reason = if overlay.is_some() { None } else { Some("SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE") };
    Ok((Some(materialization), overlay, reason))
}

/// What `_initial_interior_closure` answers.
struct InitialClosure {
    contacts: Vec<SymSplitContact>,
    batches: i64,
    materialization: Option<Materialization>,
    overlay: Option<Overlay>,
    reason: Option<&'static str>,
}

/// `_initial_interior_closure(builder, snapshot, f0, contacts, budget=...)`: grow the stable interior contacts until a rebuild finds none new.
fn initial_interior_closure(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, time: &TimeRef, contacts: &[SymSplitContact], budget: i64) -> SkelResult<InitialClosure> {
    let (mut ordered, reason) = merge_symbolic_split_contacts(&[contacts]);
    if reason.is_some() {
        return Ok(InitialClosure { contacts: Vec::new(), batches: 0, materialization: None, overlay: None, reason });
    }
    for iteration in 0..=budget {
        let (materialization, mixed, reason) = rebuild_splits(ctx, builder, snapshot, &ordered, time)?;
        if reason.is_some() {
            return Ok(InitialClosure { contacts: Vec::new(), batches: iteration, materialization, overlay: None, reason });
        }
        let Some(mixed) = mixed else {
            return Err(SkelError::Unsupported("a rebuild of the splits without an overlay and without a reason".to_string()));
        };
        let (latent, reason) = timed(Phase::ClosureDiscover, || discover_interior_split_contacts(ctx, builder, &mixed))?;
        if reason.is_some() {
            return Ok(InitialClosure { contacts: Vec::new(), batches: iteration, materialization, overlay: Some(mixed), reason });
        }
        let (merged, reason) = merge_symbolic_split_contacts(&[&ordered, &latent]);
        if reason.is_some() {
            return Ok(InitialClosure { contacts: Vec::new(), batches: iteration, materialization, overlay: Some(mixed), reason });
        }
        if merged.len() == ordered.len() {
            return Ok(InitialClosure { contacts: ordered, batches: iteration, materialization, overlay: Some(mixed), reason: None });
        }
        if iteration == budget {
            return Ok(InitialClosure { contacts: Vec::new(), batches: iteration, materialization, overlay: Some(mixed), reason: Some("SYMBOLIC_SUPERLEVEL_OUTER_BUDGET_EXHAUSTED") });
        }
        ordered = merged;
    }
    Err(SkelError::Unsupported("AssertionError in the oracle: unreachable initial interior closure".to_string()))
}

/// The replay of the closure (`symbolic_superlevel_coordinator.py` after the first `plan_mixed_generations`): the generations of the split overlay planned a SECOND time, and the
/// second answer compared with the first: the same interior contacts (every field) and the same overlay by its signature. The cost of the second plan is real (signs, hydrations,
/// memory) and is part of the transaction. `None`: the replay agreed; `Some(fixed point)`: it did not (or did not resolve), and the fixed point of the replay is what the refusal
/// carries as its `junction`.
fn replay_unit(
    ctx: &mut ExactCtx<'_>,
    builder: &mut Builder,
    split_overlay: &Overlay,
    budget: i64,
    memo: &mut SignatureMemo,
    first_overlay: &Overlay,
    first_later: &[SymSplitContact],
) -> SkelResult<Option<JunctionFixedPoint>> {
    let (replay, replay_later) = plan_mixed_generations(ctx, builder, split_overlay, budget, memo)?;
    let repeated = match (&replay.overlay, replay.unresolved_reason) {
        (Some(overlay), None) => {
            replay_later.len() == first_later.len()
                && replay_later.iter().zip(first_later).all(|(left, right)| contacts_equal(left, right))
                && overlay_signature(overlay, memo)? == overlay_signature(first_overlay, memo)?
        }
        _ => false,
    };
    Ok(if repeated { None } else { Some(replay) })
}

/// `plan_symbolic_superlevel_closure(builder, snapshot, outer_budget=..., junction_budget=...)`: rebuild from F0 whenever the stable interior contact set grows, then close the
/// junctions. The signatures of the overlays of one transaction share the memory of their immutable parts (`signature_memo`).
pub fn plan_symbolic_superlevel_closure(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, outer_budget: i64, junction_budget: i64) -> SkelResult<SymbolicSuperlevelClosure> {
    let mut memo = SignatureMemo::new();
    let Some(first) = snapshot.incidents.first() else {
        return Ok(refusal(None, Vec::new(), None, 0, Vec::new(), "SYMBOLIC_SUPERLEVEL_EMPTY_PACKET"));
    };
    let time: TimeRef = Rc::clone(&first.event.time);
    if timed(Phase::ClosureOverlay, || build_f0_overlay(ctx, builder, &snapshot.vertices, &time))?.is_none() {
        return Ok(refusal(None, Vec::new(), None, 0, Vec::new(), "SYMBOLIC_F0_OVERLAY_UNRESOLVABLE"));
    }
    let (initial, reason) = initial_interior_contacts(snapshot)?;
    if let Some(reason) = reason {
        return Ok(refusal(None, Vec::new(), None, 0, Vec::new(), reason));
    }
    let InitialClosure { contacts, batches, materialization, overlay: split_overlay, reason } = initial_interior_closure(ctx, builder, snapshot, &time, &initial, outer_budget)?;
    if let Some(reason) = reason {
        return Ok(refusal(materialization, contacts, None, batches, Vec::new(), reason));
    }
    let (Some(materialization), Some(split_overlay)) = (materialization, split_overlay) else {
        return Ok(refusal(None, contacts, None, 0, Vec::new(), "SYMBOLIC_SPLIT_OVERLAY_UNRESOLVABLE"));
    };
    let budget = outer_budget + junction_budget;
    let (junction, later) = plan_mixed_generations(ctx, builder, &split_overlay, budget, &mut memo)?;
    let Some(junction_overlay) = junction.overlay.clone().filter(|_| junction.unresolved_reason.is_none()) else {
        let reason = junction.unresolved_reason.unwrap_or("SYMBOLIC_JUNCTION_OVERLAY_UNRESOLVABLE");
        let signatures = junction.signatures.clone();
        return Ok(refusal(Some(materialization), contacts, Some(junction), later.len() as i64, signatures, reason));
    };
    // THE REPLAY (a removable unit: the call and `replay_unit` go together, and with them the refusal `REPEATED_CONTACT_SET_CHANGED_SIGNATURE`)
    if let Some(replayed) = replay_unit(ctx, builder, &split_overlay, budget, &mut memo, &junction_overlay, &later)? {
        let mut every = contacts;
        every.extend(later.iter().cloned());
        let signatures = junction.signatures.clone();
        return Ok(refusal(Some(materialization), every, Some(replayed), later.len() as i64, signatures, "SYMBOLIC_SUPERLEVEL_REPEATED_CONTACT_SET_CHANGED_SIGNATURE"));
    }
    let mut all_contacts: OrderedMap<Val, SymSplitContact> = OrderedMap::new();
    for item in contacts.iter().chain(later.iter()) {
        all_contacts.insert(item.key.val.clone(), item.clone());
    }
    let mut ordered: Vec<SymSplitContact> = all_contacts.values().cloned().collect();
    ordered.sort_by(|left, right| left.key.val.repr().as_bytes().cmp(right.key.val.repr().as_bytes()));
    let mut signatures = vec![overlay_signature(&split_overlay, &mut memo)?];
    signatures.extend(junction.signatures.iter().cloned());
    signatures.push(overlay_signature(&junction_overlay, &mut memo)?);
    let generations = junction.generations.len() as i64;
    Ok(SymbolicSuperlevelClosure {
        materialization: Some(materialization),
        overlay: Some(junction_overlay),
        split_contacts: ordered,
        junction: Some(junction),
        outer_iterations: later.len() as i64,
        signatures,
        canonical_batch_count: 1 + batches + generations,
        unresolved_reason: None,
    })
}

/// The closure stage of `apply_superlevel_transaction`: the closure of the packet with the budget the head of the transaction gave it for both of its fixed points; a closure that
/// is refused, or that holds no plan, is recorded as the named refusal of the packet (`_record_symbolic_unresolvable`) and the answer is none. The commit follows only a `Some`.
pub fn closure_stage(ctx: &mut ExactCtx<'_>, builder: &mut Builder, snapshot: &Snapshot, budget: i64) -> SkelResult<Option<SymbolicSuperlevelClosure>> {
    let closure = plan_symbolic_superlevel_closure(ctx, builder, snapshot, budget, budget)?;
    let has_plans = closure.materialization.as_ref().is_some_and(|found| !found.plans.is_empty());
    if closure.unresolved_reason.is_some() || !has_plans {
        record_symbolic_unresolvable(builder, snapshot, closure.unresolved_reason.unwrap_or("SYMBOLIC_SUPERLEVEL_MATERIALIZATION_UNAVAILABLE"))?;
        return Ok(None);
    }
    Ok(Some(closure))
}

/// `repr` of a closure as the seams compare it.
pub fn closure_val(closure: &SymbolicSuperlevelClosure) -> Val {
    Val::data(
        "SymbolicSuperlevelClosureV1",
        vec![
            ("materialization", closure.materialization.as_ref().map_or_else(Val::none, PlanVal::to_val)),
            ("overlay", closure.overlay.as_ref().map_or_else(Val::none, overlay_val)),
            ("split_contacts", Val::tuple(closure.split_contacts.iter().map(PlanVal::to_val).collect())),
            ("junction", closure.junction.as_ref().map_or_else(Val::none, fixed_point_val)),
            ("outer_iterations", Val::int(closure.outer_iterations)),
            ("signatures", Val::tuple(closure.signatures.clone())),
            ("canonical_batch_count", Val::int(closure.canonical_batch_count)),
            ("unresolved_reason", closure.unresolved_reason.map_or_else(Val::none, Val::str)),
        ],
    )
}
