//! Native port of `cftuv_envelope.materialize.clip.clip_geometry` (Python is the oracle): the whole operation.
//!
//! * the numeric facade, cells, corner snap, tessellation predicates and the CPython 3.11 semantics the kernel names explicitly
//!   (`plane`, `numeric`, `edge`, `faces`, `lift`, `cells`, `snap`, `tessellate`, `order`, `cpython311`);
//! * the stage (`stage`: node arena, signs, crossings, edge subdivision; `cut`: phase 1; `emit`: phases 2 and 3, the
//!   result, counters and note) and the two entry points (`geometry`: `clip_geometry` with its one or two stages);
//! * the test-only differential seams of the parts (`seam`) and of the whole operation (`geometry_seam`, opcode 124);
//! * `profile`: phase timers behind the cargo feature `profile` (`examples/clip_profile.rs`).
//!
//! Every function that can pay cost takes an [`ExactCtx`](cftuv_core::exact::ExactCtx) (budget, canonicalization
//! memory, sign counters, product cache) and asks its exact questions in the oracle's order, so the cost is identical.
//! A refusal is a named [`error::ClipError`]; nothing here panics on valid input and nothing falls back silently.
//! Nothing depends on the interpreter that runs the host: the sort of `_ordered` and the float fold of `offset_normal.blend` are the
//! explicit CPython 3.11 semantics of `kernel/src/cftuv_envelope/_cpython311.py` (`cpython311`), whatever the version of CPython.

pub mod cells;
pub mod cpython311;
pub mod cut;
pub mod edge;
pub mod emit;
pub mod error;
pub mod faces;
pub mod fxhash;
pub mod geometry;
pub mod geometry_seam;
pub mod lift;
pub mod numeric;
pub mod order;
pub mod plane;
pub mod profile;
pub mod point;
pub mod regions;
pub mod seam;
pub mod snap;
pub mod stage;
pub mod tessellate;
pub mod warm;
