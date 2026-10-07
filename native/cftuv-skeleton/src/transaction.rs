//! The transaction of a packet and the whole operation (`wavefront/superlevel.py::apply_superlevel_transaction`, `wavefront/skeleton.py::build_skeleton`).
//!
//! [`SuperlevelTransaction`] is the implementation of `builder::Transaction` the product uses: the head of the oracle's function (the frozen snapshot, the stale and unsupported
//! records, the early returns: `superlevel::transaction_prefix`), the symbolic closure (`coordinator::closure_stage`, which records its own refusal), the plan of the runtime
//! commit and the commit (`commit`). A refusal of the closure, the plan or the commit is recorded as the named refusal of the whole packet
//! (`superlevel::record_symbolic_unresolvable`) and the loop ends with it (`Builder::refusal`); nothing is repaired and nothing falls back.
//!
//! [`build_skeleton`] is the oracle's function of the same name: the builder (seed included) and the event loop over this transaction, in the oracle's exact sequence of
//! cost-bearing operations. `level_limit` is the oracle's LIVE `level_budget(polygon)` and `BuilderOptions::march_steps` the live `march_budget` (see `builder`).

use cftuv_core::exact::ExactCtx;

use crate::builder::{Builder, BuilderOptions, RunEnd, Step, Transaction};
use crate::commit::{materialize_symbolic_runtime_commit, plan_symbolic_runtime_commit};
use crate::coordinator::closure_stage;
use crate::error::{SkelError, SkelResult};
use crate::polygon::Polygon;
use crate::queue::CandidateEvent;
use crate::skeleton::Skeleton;
use crate::superlevel::{record_symbolic_unresolvable, transaction_prefix, Prefix};

/// `apply_superlevel_transaction(builder, level)`: plan and validate everything on the frozen prestate, then commit the deltas at once.
#[derive(Debug, Default, Clone, Copy)]
pub struct SuperlevelTransaction;

impl Transaction for SuperlevelTransaction {
    fn apply(&mut self, builder: &mut Builder, ctx: &mut ExactCtx<'_>, level: &[CandidateEvent]) -> SkelResult<Step> {
        let (snapshot, budget) = match transaction_prefix(ctx, builder, level)? {
            Prefix::Done => return Ok(Step::Continue),
            Prefix::Continue { snapshot, budget } => (*snapshot, budget),
        };
        let Some(closure) = closure_stage(ctx, builder, &snapshot, budget)? else {
            return Ok(Step::Continue);
        };
        let (plan, reason) = plan_symbolic_runtime_commit(ctx, builder, &snapshot, &closure)?;
        let Some(plan) = plan else {
            record_symbolic_unresolvable(builder, &snapshot, reason.unwrap_or("SYMBOLIC_RUNTIME_COMMIT_PLAN_UNAVAILABLE"))?;
            return Ok(Step::Continue);
        };
        materialize_symbolic_runtime_commit(ctx, builder, &snapshot, &plan)?;
        Ok(Step::Continue)
    }
}

/// `build_skeleton(polygon, split_search=MOTORCYCLE, work_budget=..., dense_hydration=...)`: the skeleton of the polygon as a sequence of events in order of time.
pub fn build_skeleton(ctx: &mut ExactCtx<'_>, polygon: Polygon, options: BuilderOptions, level_limit: i64) -> SkelResult<Skeleton> {
    let mut builder = Builder::new(ctx, polygon, options)?;
    match builder.run(ctx, level_limit, &mut SuperlevelTransaction)? {
        RunEnd::Finished(skeleton) => Ok(*skeleton),
        RunEnd::Stopped { .. } => Err(SkelError::Unsupported("the transaction of the product stopped the loop".to_string())),
    }
}
