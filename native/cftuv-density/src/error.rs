use cftuv_radical::{Failure, Priced, Work};

/// Ошибки входного контракта этого среза, без продуктовых исходов oracle.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InvalidInput {
    ZeroDenominator,
    InvalidOwnerOrientation(i8),
    InvalidTurnOrientation(i8),
    AsymmetricGram,
    NonPositiveDefiniteGram,
    UnsupportedDensityQ(u8),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DensityFailureKind {
    InvalidInput(InvalidInput),
    /// Внутреннее ограничение/дефект exact-арифметики, не UNDECIDABLE oracle.
    InternalAlgebra(Failure),
}

/// Неуспех сохраняет причину и всю работу, выполненную до него в операции.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DensityFailure {
    pub kind: DensityFailureKind,
    pub work: Work,
}

impl DensityFailure {
    pub fn from_algebra(failure: Failure, work_before: Work) -> Self {
        let failed_work = match &failure {
            Failure::Refused(refusal) => refusal.work,
            _ => Work::NONE,
        };
        Self {
            kind: DensityFailureKind::InternalAlgebra(failure),
            work: work_before + failed_work,
        }
    }

    pub(crate) fn invalid(reason: InvalidInput) -> Self {
        Self {
            kind: DensityFailureKind::InvalidInput(reason),
            work: Work::NONE,
        }
    }

    pub(crate) fn after(mut self, work_before: Work) -> Self {
        self.work = work_before + self.work;
        self
    }
}

pub type DensityResult<T> = Result<Priced<T>, DensityFailure>;
