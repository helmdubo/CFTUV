//! `ExactWorkBudgetV1` (`exact_sqrt_sum.py` lines 97-343): six physical-operation articles and one cap.
//!
//! Python raises `ExactCanonicalizationWorkBudgetExhausted` from inside `spend_*`; here every spend returns
//! `Err(Exhausted)` instead. The detail string is never formatted in Rust: the host shim owns the Python
//! budget object and calls `budget.exhaustion_detail(operation, radicand)` itself. The articles stay incremented
//! on the failing spend, exactly as in Python (the increment precedes `_enforce`).

use dashu_int::UBig;

/// `ExactWorkOperationV1`: the operation that ran out of budget. `as_str` is the enum's `.value`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Operation {
    PrimeUniverse,
    CoprimeBasis,
    Primality,
    PollardRhoBrent,
    SquarefreeSplit,
    PrimeSupport,
    ExactPosition,
}

impl Operation {
    pub const ALL: [Operation; 7] = [
        Operation::PrimeUniverse,
        Operation::CoprimeBasis,
        Operation::Primality,
        Operation::PollardRhoBrent,
        Operation::SquarefreeSplit,
        Operation::PrimeSupport,
        Operation::ExactPosition,
    ];

    pub fn as_str(self) -> &'static str {
        match self {
            Operation::PrimeUniverse => "PRIME_UNIVERSE",
            Operation::CoprimeBasis => "COPRIME_BASIS",
            Operation::Primality => "PRIMALITY",
            Operation::PollardRhoBrent => "POLLARD_RHO_BRENT",
            Operation::SquarefreeSplit => "SQUAREFREE_SPLIT",
            Operation::PrimeSupport => "PRIME_SUPPORT",
            Operation::ExactPosition => "EXACT_POSITION",
        }
    }

    pub fn from_value(value: &str) -> Option<Operation> {
        Operation::ALL.into_iter().find(|operation| operation.as_str() == value)
    }
}

/// `ExactWorkBudgetModeV1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BudgetMode {
    Bounded,
    UnlimitedReference,
}

/// Where and on what a bounded budget ran out. Everything the host needs to rebuild the Python exception.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Exhausted {
    pub operation: Operation,
    pub radicand: UBig,
}

/// Constructor refusals mirroring the two `ValueError`s of `ExactWorkBudgetV1.__init__`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BudgetConfigError {
    /// `BOUNDED` without a cap.
    BoundedWithoutCap,
    /// `UNLIMITED_REFERENCE` with a cap.
    UnlimitedWithCap,
}

/// Article indices in `spent_by_article` order.
pub const ARTICLES: usize = 6;

/// The counter. `stage`, `domain_id` and `superlevel` are opaque host strings carried only so that the shim can
/// rebuild an identical Python object; Rust never reads them.
#[derive(Clone, Debug)]
pub struct WorkBudget {
    pub mode: BudgetMode,
    pub cap: Option<u64>,
    pub stage: String,
    pub domain_id: String,
    pub superlevel: String,
    pub modular_squarings: u64,
    pub gcd_operations: u64,
    pub miller_rabin_rounds: u64,
    pub pollard_attempts: u64,
    pub radical_materializations: u64,
    pub exact_position_hydrations: u64,
}

impl WorkBudget {
    pub fn new(
        mode: BudgetMode,
        cap: Option<u64>,
        stage: &str,
        domain_id: &str,
        superlevel: &str,
    ) -> Result<WorkBudget, BudgetConfigError> {
        match (mode, cap) {
            (BudgetMode::Bounded, None) => return Err(BudgetConfigError::BoundedWithoutCap),
            (BudgetMode::UnlimitedReference, Some(_)) => return Err(BudgetConfigError::UnlimitedWithCap),
            _ => {}
        }
        Ok(WorkBudget::raw(mode, cap, stage, domain_id, superlevel))
    }

    fn raw(mode: BudgetMode, cap: Option<u64>, stage: &str, domain_id: &str, superlevel: &str) -> WorkBudget {
        WorkBudget {
            mode,
            cap,
            stage: stage.to_owned(),
            domain_id: domain_id.to_owned(),
            superlevel: superlevel.to_owned(),
            modular_squarings: 0,
            gcd_operations: 0,
            miller_rabin_rounds: 0,
            pollard_attempts: 0,
            radical_materializations: 0,
            exact_position_hydrations: 0,
        }
    }

    pub fn bounded(cap: u64) -> WorkBudget {
        WorkBudget::raw(BudgetMode::Bounded, Some(cap), "", "", "")
    }

    pub fn unlimited() -> WorkBudget {
        WorkBudget::raw(BudgetMode::UnlimitedReference, None, "", "", "")
    }

    /// `spent`: sum of the six articles.
    pub fn spent(&self) -> u64 {
        self.articles().iter().fold(0u64, |total, article| total.saturating_add(*article))
    }

    /// `spent_by_article`.
    pub fn articles(&self) -> [u64; ARTICLES] {
        [
            self.modular_squarings,
            self.gcd_operations,
            self.miller_rabin_rounds,
            self.pollard_attempts,
            self.radical_materializations,
            self.exact_position_hydrations,
        ]
    }

    pub fn set_articles(&mut self, articles: [u64; ARTICLES]) {
        self.modular_squarings = articles[0];
        self.gcd_operations = articles[1];
        self.miller_rabin_rounds = articles[2];
        self.pollard_attempts = articles[3];
        self.radical_materializations = articles[4];
        self.exact_position_hydrations = articles[5];
    }

    /// `remaining`: `None` for the reference mode.
    pub fn remaining(&self) -> Option<i128> {
        self.cap.map(|cap| i128::from(cap) - i128::from(self.spent()))
    }

    /// `is_exhausted`.
    pub fn is_exhausted(&self) -> bool {
        matches!(self.cap, Some(cap) if self.spent() > cap)
    }

    /// `replay`: add a recorded stage price if it fits under the cap, otherwise change nothing and return `false`.
    pub fn replay(&mut self, delta: [u64; ARTICLES]) -> bool {
        if let Some(cap) = self.cap {
            let total = delta.iter().fold(self.spent(), |total, article| total.saturating_add(*article));
            if total > cap {
                return false;
            }
        }
        self.modular_squarings = self.modular_squarings.saturating_add(delta[0]);
        self.gcd_operations = self.gcd_operations.saturating_add(delta[1]);
        self.miller_rabin_rounds = self.miller_rabin_rounds.saturating_add(delta[2]);
        self.pollard_attempts = self.pollard_attempts.saturating_add(delta[3]);
        self.radical_materializations = self.radical_materializations.saturating_add(delta[4]);
        self.exact_position_hydrations = self.exact_position_hydrations.saturating_add(delta[5]);
        true
    }

    /// `_enforce`, shared tail of every spend.
    #[inline]
    fn enforce(&self, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        match self.cap {
            Some(cap) if self.spent() > cap => Err(Exhausted { operation, radicand: radicand.clone() }),
            _ => Ok(()),
        }
    }

    #[inline]
    pub fn spend_modular_squarings(&mut self, count: u64, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        self.modular_squarings = self.modular_squarings.saturating_add(count);
        self.enforce(operation, radicand)
    }

    #[inline]
    pub fn spend_gcd_operations(&mut self, count: u64, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        self.gcd_operations = self.gcd_operations.saturating_add(count);
        self.enforce(operation, radicand)
    }

    #[inline]
    pub fn spend_miller_rabin_rounds(&mut self, count: u64, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        self.miller_rabin_rounds = self.miller_rabin_rounds.saturating_add(count);
        self.enforce(operation, radicand)
    }

    #[inline]
    pub fn spend_pollard_attempts(&mut self, count: u64, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        self.pollard_attempts = self.pollard_attempts.saturating_add(count);
        self.enforce(operation, radicand)
    }

    #[inline]
    pub fn spend_radical_materializations(&mut self, count: u64, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        self.radical_materializations = self.radical_materializations.saturating_add(count);
        self.enforce(operation, radicand)
    }

    #[inline]
    pub fn spend_exact_position_hydrations(&mut self, count: u64, operation: Operation, radicand: &UBig) -> Result<(), Exhausted> {
        self.exact_position_hydrations = self.exact_position_hydrations.saturating_add(count);
        self.enforce(operation, radicand)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn constructor_mirrors_the_two_value_errors() {
        assert_eq!(
            WorkBudget::new(BudgetMode::Bounded, None, "s", "d", "l").unwrap_err(),
            BudgetConfigError::BoundedWithoutCap
        );
        assert_eq!(
            WorkBudget::new(BudgetMode::UnlimitedReference, Some(1), "s", "d", "l").unwrap_err(),
            BudgetConfigError::UnlimitedWithCap
        );
    }

    #[test]
    fn spend_keeps_the_failing_increment_and_reports_the_operation() {
        let mut budget = WorkBudget::bounded(5);
        let radicand = UBig::from(77u8);
        assert!(budget.spend_gcd_operations(5, Operation::CoprimeBasis, &radicand).is_ok());
        let error = budget.spend_modular_squarings(1, Operation::Primality, &radicand).unwrap_err();
        assert_eq!(error, Exhausted { operation: Operation::Primality, radicand });
        assert_eq!(budget.articles(), [1, 5, 0, 0, 0, 0]);
        assert!(budget.is_exhausted());
        assert_eq!(budget.remaining(), Some(-1));
    }

    #[test]
    fn replay_is_all_or_nothing() {
        let mut budget = WorkBudget::bounded(10);
        assert!(budget.replay([3, 2, 1, 0, 0, 0]));
        assert!(!budget.replay([4, 0, 0, 0, 0, 1]));
        assert_eq!(budget.articles(), [3, 2, 1, 0, 0, 0]);
        assert!(budget.replay([4, 0, 0, 0, 0, 0]));
        assert_eq!(budget.spent(), 10);
        assert!(!budget.is_exhausted());
    }

    #[test]
    fn operation_names_round_trip() {
        for operation in Operation::ALL {
            assert_eq!(Operation::from_value(operation.as_str()), Some(operation));
        }
        assert_eq!(Operation::from_value("nope"), None);
    }
}
