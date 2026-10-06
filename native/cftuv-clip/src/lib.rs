//! Native port of `cftuv_envelope.materialize.clip.clip_geometry` (Python is the oracle), first third:
//! the numeric facade, the cells and the corner snap, the tessellation predicates, and the version-pinned
//! CPython emulation the rest of the port stands on.
//!
//! Every function that can pay cost takes an [`ExactCtx`](cftuv_core::exact::ExactCtx) (budget, canonicalization
//! memory, sign counters, product cache) and asks its exact questions in the oracle's order, so the cost is identical.
//! A refusal is a named [`error::ClipError`]; nothing here panics on valid input and nothing falls back silently.

pub mod cells;
pub mod edge;
pub mod error;
pub mod faces;
pub mod lift;
pub mod numeric;
pub mod order;
pub mod plane;
pub mod point;
pub mod pyemu;
pub mod seam;
pub mod snap;
pub mod tessellate;
