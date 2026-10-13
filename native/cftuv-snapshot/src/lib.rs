//! `snapshot_check`: the native proof that an `AnalysisSnapshotV1` passes the Python validators of the EXPORT stage - CLEAN or DECLINED, never a refusal.
//!
//! The Python validators (`validation_snapshot.validate_analysis_snapshot`, `reference.validation.validate_reference_geometry_payload`) stay the oracle and stay the independent
//! check of the background byte check and of the field tools. This crate answers one question about a snapshot - "would they report nothing?" - and answers it only when it can prove
//! "yes":
//!
//! * [`check`] returns `Ok(Attestation)`: the snapshot has no issue and no diagnostic, and the attestation carries the process counters the Python validators would have incremented;
//! * or `Err(Decline)`: something is wrong, or not carried by the port. The caller then runs the Python validators exactly as before, and THEY produce the refusal and its text. Native never
//!   refuses on its own, and never attests a snapshot Python would refuse (a silent acceptance is what AGENTS.md rule 4 forbids). Every statement of the oracle is mirrored, and where the
//!   port does not know what the oracle does (another certificate, a float direction, an angle on which the exact path would run) it declines by name.
//!
//! The port reads a [`model::Snapshot`] built by the PyO3 layer from the Python graph (the reading is the other half of the proof: a value that is not the canonical shape of its field
//! is a decline of the reader). What it carries is the product's snapshot: planar domains with an exact-source-plane metric on the canonical basis, rational affine support directions,
//! certified reflex measures. Near-planar, developable and band domains are declined by name and run Python.

pub mod angle;
pub mod decimal;
pub mod enclosure;
pub mod interval;
pub mod metric;
pub mod model;
pub mod reference;
pub mod structural;
pub mod verdict;

pub use verdict::{Attestation, Check, Context, Decline, Reason, Scope};

/// The verdict on one snapshot: `Ok` is the attestation, `Err` the named decline.
pub fn check(snapshot: &model::Snapshot<'_>, context: &Context) -> Check<Attestation> {
    structural::analysis(snapshot, context)?;
    let mut attestation = Attestation::default();
    if context.scope == Scope::AnalysisAndReference {
        reference::payload(snapshot, context, &mut attestation)?;
    }
    Ok(attestation)
}
