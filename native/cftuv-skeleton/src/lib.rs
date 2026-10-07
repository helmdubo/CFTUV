//! Native port of the exact event layer of the straight skeleton (`kernel/src/cftuv_envelope/wavefront`; Python is the oracle).
//!
//! First work packages of the port (`build_skeleton` whole is the target, see the port map): the event-time machinery with its queue and the
//! candidate law of a split event, in the oracle's exact order of cost-bearing operations (budget articles, memory tables, sign counters).
//!
//! * `line`, `time`: `SupportLineV1`, `EventTimeV1`, `compare_times`, `concurrency_time`, `sliding_time`, `sliding_point`, `event_point`;
//! * `heap`, `queue`: CPython's `heapq` over a fallible comparison (its comparison sequence is cost) and `EventQueueV1`;
//! * `repr`: the byte-exact Python `repr` of the values the kernel orders by text (~65 sites sort by `repr`);
//! * `view`, `candidate`: the exact candidate view with the superlevel's memory (value-keyed places, identity-keyed times) and
//!   `evaluate_split_candidate`;
//! * `profile`: phase timers behind the cargo feature `profile` (what the time of an evaluate call is spent on);
//! * `seam`, `wire`: the test-only differential seams (opcodes from 200) over the boundary buffers of `cftuv_core::codec`.
//!
//! Every function that pays cost takes an `ExactCtx` and asks its exact questions in the order the oracle asks them. A refusal is a named
//! [`error::SkelError`]; nothing here panics on valid input and nothing falls back silently.

pub mod candidate;
pub mod error;
pub mod grid;
pub mod heap;
pub mod line;
pub mod motorcycle;
pub mod polygon;
pub mod poststate;
pub mod profile;
pub mod proof;
pub mod queue;
pub mod repr;
pub mod seam;
mod seam_graph;
pub mod time;
pub mod view;
mod wire;
