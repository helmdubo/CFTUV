//! Native port of the exact event layer of the straight skeleton (`kernel/src/cftuv_envelope/wavefront`; Python is the oracle).
//!
//! The work packages of the port so far (`build_skeleton` whole is the target, see the port map): the event-time machinery with its queue (WP-S0), the exact candidate view
//! with both candidate laws (WP-S2, WP-S2b), the cell grid and the motorcycle graph (WP-S1), the poststate classification and the proof ledger, all in the oracle's exact
//! order of cost-bearing operations (budget articles, memory tables, sign counters).
//!
//! * `line`, `time`: `SupportLineV1`, `EventTimeV1`, `compare_times`, `concurrency_time`, `sliding_time`, `sliding_point`, `event_point`;
//! * `heap`, `queue`: CPython's `heapq` over a fallible comparison (its comparison sequence is cost) and `EventQueueV1`;
//! * `repr`: the byte-exact Python `repr` of the values the kernel orders by text (~65 sites sort by `repr`);
//! * `view`, `candidate`: the exact candidate view with the superlevel's memory (value-keyed places, identity-keyed times), `evaluate_split_candidate` and
//!   `evaluate_edge_candidate`, with `edge_event_time`, `is_future`, `collapsing_span`, `span_end`, `sliding_projection`;
//! * `polygon`: the input polygon as the builder reads it (loops, speeds, fans);
//! * `grid`: the cell grid and its index (floor and ceiling of exact rationals toward minus infinity);
//! * `motorcycle`: the motorcycle graph (marches, the crash queue, `trace_for`) and the two-sided `TraceCandidateIndex`;
//! * `poststate`, `proof`: the affine classification of a newborn span and the ledger of proof obligations;
//! * `profile`: phase timers behind the cargo feature `profile` (what the time of an evaluate call or a graph build is spent on);
//! * `builder`: the front (`_Builder`) and the event loop, with the primitives the transaction of a packet uses and the BOUNDARY of that transaction (`Transaction`):
//!   the seed, the candidates of every vertex, the loop (levels, the memory of places on a new exact time, the residual of one time, the short LAVs, the finish) (WP-S3);
//! * `skeleton`, `superlevel`: the result (`Skeleton`, the accumulation of nodes, the duplicate counters), and the head of the transaction with the records of its refusals
//!   and the emission of the nodes of a component;
//! * `pyset`: the iteration order of a CPython set of small ints (the members of a component are iterated as one);
//! * `pyval`: the Python values the planning layer keys, groups and orders by (identity by value, `repr`, the tuple order of CPython with its `TypeError`);
//! * `snapshot`, `plans`, `germ`, `closure`, `composition`: the frozen prestate of a packet, the plan of each component (death of ports, reconnection of meetings, cut of a
//!   span, composition, wiring of the births), the ledger of germs, the stable symbolic normal form of the cuts (WP-S4);
//! * `omap`, `overlay`, `component`, `contacts`, `generations`, `coordinator`: the symbolic closure of a packet (WP-S5): the overlay of junctions and leaves with its exact view
//!   (`overlay`, the dictionaries of the oracle in insertion order in `omap`), the delta of a component and the signature of an overlay (`component`), the contacts of an exact time
//!   (`contacts`, with the memo of the law's decisions: one per pair), the generations of mixed junction and interior contacts replayed to a fixed point (`generations`), and the outer
//!   fixed point itself (`coordinator`; its second run of the generations, the replay, is the oracle's self-check and runs only when asked);
//! * `commit`, `transaction`: the runtime commit of a closed overlay (the plan that validates every reference, the materialization that applies it) and the transaction of a packet that
//!   joins the stages; `transaction::build_skeleton` is the whole operation (WP-S6);
//! * `seam`, `seam_graph`, `seam_builder`, `seam_primitive`, `seam_closure`, `wire`: the test-only differential seams (opcodes from 200) over the boundary buffers of `cftuv_core::codec`.
//!
//! Every function that pays cost takes an `ExactCtx` and asks its exact questions in the order the oracle asks them. A refusal is a named
//! [`error::SkelError`]; nothing here panics on valid input and nothing falls back silently.

pub mod builder;
pub mod candidate;
pub mod closure;
pub mod commit;
pub mod component;
pub mod composition;
pub mod contacts;
pub mod coordinator;
pub mod error;
pub mod generations;
pub mod germ;
pub mod grid;
pub mod heap;
pub mod line;
pub mod motorcycle;
pub mod omap;
pub mod overlay;
pub mod plans;
pub mod polygon;
pub mod poststate;
pub mod profile;
pub mod proof;
pub mod pyset;
pub mod pyval;
pub mod queue;
pub mod repr;
pub mod seam;
mod seam_builder;
mod seam_closure;
mod seam_graph;
mod seam_primitive;
pub mod skeleton;
pub mod snapshot;
pub mod superlevel;
pub mod time;
pub mod transaction;
pub mod view;
mod wire;
