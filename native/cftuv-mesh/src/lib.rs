//! Native port of the exact mesh of the materializer (`kernel/src/cftuv_envelope/materialize/mesh.py`, Python is the oracle): W1 of the
//! MATERIALIZE chain.
//!
//! * [`intern`]: `intern_vertices`, the key of every point of the merged faces (`src:` / `node:`) and the contour of every face;
//! * [`stations`]: `station_values`, the exact `(s, r)` of every vertex of every region (the rung stations, the unification across frames,
//!   the plain faces);
//! * [`tessellate`]: `tessellate_faces` and `canonical_triangles`, the faces of every merged face under the three laws (the polygon law
//!   with its fans, rung pieces, bilinear and partitioned faces);
//! * [`partition`]: `convex_partition.py` (the exact planner up to eight vertices, the greedy merge above, the proof);
//! * [`affine`] and [`predicates`]: the exact UV predicates of `tessellate.py` and the contour predicates of `wavefront/faces.py` the clip
//!   port does not carry.
//!
//! Every function that can pay cost takes an [`ExactCtx`](cftuv_core::exact::ExactCtx) and asks its exact questions in the oracle's order and with
//! the oracle's short circuits, so the budget, the canonicalization memory and the sign counters end exactly where Python leaves them. The
//! operation is a WHOLE one (no state of its own survives the call): the PyO3 layer (`cftuv-python/src/materialize_w1.rs`) reads the Python
//! objects, runs it on the cost mirror and builds the Python result.
//!
//! A refusal is a named [`error::MeshError`]; nothing here panics on valid input and nothing falls back silently. What the port does not carry
//! is [`error::MeshError::Unsupported`]: the host decides (the shim raises the pre-state refusal and the oracle answers).

pub mod affine;
pub mod data;
pub mod error;
pub mod intern;
pub mod mesh;
pub mod partition;
pub mod predicates;
pub mod stations;
pub mod tessellate;
pub mod tally;
