//! Native port of the hot part of the materializer's SILHOUETTE stage (Python is the oracle, bit for bit, counters included):
//!
//! * [`plane_lift`]: `lift.PlaneLiftV1.lift` of every vertex (`assemble.lift_vertices` on an exact plane);
//! * [`source_lift`]: `source_lift.lift_source_vertices` (the law `SOURCE_VERTEX_LIFTED_AT_HOST_POSITION_V1`) and `source_lift.settle_emitted_faces`;
//! * [`silhouette`]: `silhouette.dissolve_silhouette` and `silhouette.verify_silhouette` (the law `SILHOUETTE_TOPOLOGY_V1`) with the chains of the
//!   final faces (`assemble.chains_of_faces`, `chains_of`, `paths_of`, `edge_kind`).
//!
//! Every exact question (an orientation the binary64 filter cannot prove, a sign of a sum of products) is asked in the oracle's order with its
//! short circuits, on an [`ExactCtx`](cftuv_core::exact::ExactCtx), so the budget, the canonicalization memory and the sign counters end where
//! Python leaves them. The floats are CPython 3.11's ([`pymath`]): the same operations in the same order, `math.hypot`, `math.atan2`, `** 2`,
//! the left fold of `sum`, the builtin `min`/`max`, and exact comparisons of floats with fractions. Python's set and dict iteration orders that
//! decide the order of exact questions are reproduced ([`pyset`]).
//!
//! What the port does not carry is [`Unsupported`](silhouette::SilError::Unsupported) (a key missing from a table, a non-finite UV, ...): the
//! host then runs the oracle on the same state.

pub mod assemble_law;
pub mod plane_lift;
pub mod pymath;
pub mod pyset;
pub mod silhouette;
pub mod source_lift;
