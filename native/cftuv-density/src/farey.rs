//! Экспериментальный single-chart механизм поиска Density, отдельно от алгебры.
//!
//! Порядок CPython set и fallible CPython 3.11 sort ОБЯЗАН предоставить caller.
//! Здесь нет default Rust-сортировки: последовательность сравнений/отказов
//! наблюдаема. Atlas не перенесён и отказывает явно до расхода работы.

use std::collections::HashSet;
use std::fmt;

use cftuv_radical::arith::{gcd_signed, signed};

use crate::{IBig, Rational, UBig};

pub const SEARCH_WORK_CAP: u32 = 1 << 17;
pub const PREPARATION_WORK_CAP: u32 = 1 << 16;

/// Снимок счётчиков остаётся наблюдаемым после превышения капа.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkBudget {
    pub cap: UBig,
    pub shell_probes: UBig,
    pub order_steps: UBig,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkBudgetExhausted(pub WorkBudget);

impl fmt::Display for WorkBudgetExhausted {
    fn fmt(&self, out: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            out,
            "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED cap={} shell_probes={} order_steps={}",
            self.0.cap, self.0.shell_probes, self.0.order_steps
        )
    }
}

impl std::error::Error for WorkBudgetExhausted {}

impl WorkBudget {
    pub fn new(cap: UBig) -> Self {
        Self {
            cap,
            shell_probes: UBig::ZERO,
            order_steps: UBig::ZERO,
        }
    }

    pub fn search() -> Self {
        Self::new(UBig::from(SEARCH_WORK_CAP))
    }
    pub fn preparation() -> Self {
        Self::new(UBig::from(PREPARATION_WORK_CAP))
    }
    pub fn spent(&self) -> UBig {
        &self.shell_probes + &self.order_steps
    }

    pub fn counters(&self) -> [(&'static str, UBig); 2] {
        [
            ("DENSITY_FAN_SHELL_PROBES", self.shell_probes.clone()),
            ("DENSITY_FAN_ORDER_STEPS", self.order_steps.clone()),
        ]
    }

    pub fn spend_shell_probes(&mut self, count: UBig) -> Result<(), WorkBudgetExhausted> {
        self.shell_probes += count;
        self.enforce()
    }

    pub fn spend_order_steps(&mut self, count: UBig) -> Result<(), WorkBudgetExhausted> {
        self.order_steps += count;
        self.enforce()
    }

    fn enforce(&self) -> Result<(), WorkBudgetExhausted> {
        if self.spent() > self.cap {
            Err(WorkBudgetExhausted(self.clone()))
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct PrimitiveVector(pub IBig, pub IBig);

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SingleChartWindow {
    pub use_x: bool,
    pub denominator_sign: i8,
    /// outer_lower, inner_lower, inner_upper, outer_upper, как в Python.
    pub sealed: [Rational; 4],
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SearchMode {
    SingleChart,
    Atlas,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FareyFailure<E> {
    Work(WorkBudgetExhausted),
    Predicate(E),
    InvalidOrdering,
    AtlasNotPorted,
}

#[derive(Clone, Copy, Debug)]
pub enum Endpoint<'a> {
    Incoming,
    Primitive(&'a PrimitiveVector),
    Outgoing,
}

/// Ordinal — 1-based. edge_valid сохраняет cross-before-subturn short circuit.
pub trait FareyPredicates {
    type Error;
    fn local_candidate(
        &mut self,
        ordinal: usize,
        vector: &PrimitiveVector,
    ) -> Result<bool, Self::Error>;
    fn candidate_compare(
        &mut self,
        ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, Self::Error>;
    fn edge_valid(
        &mut self,
        ordinal: usize,
        left: Endpoint<'_>,
        right: Endpoint<'_>,
    ) -> Result<bool, Self::Error>;
}

/// Обе операции обязательны, возвращают перестановку индексов входа.
/// sort вызывает predicates.candidate_compare в каноническом порядке Python.
pub trait FareyOrdering<P: FareyPredicates> {
    fn iteration_order(
        &mut self,
        ordinal: usize,
        insertion_order: &[PrimitiveVector],
    ) -> Result<Vec<usize>, P::Error>;
    fn sort(
        &mut self,
        ordinal: usize,
        items: &[PrimitiveVector],
        predicates: &mut P,
    ) -> Result<Vec<usize>, P::Error>;
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchResult {
    pub height: Option<UBig>,
    pub fan: Option<Vec<PrimitiveVector>>,
    pub previous_counts: Vec<usize>,
    pub counts: Vec<usize>,
}

/// Floor Python `//`: отрицательный неполный quotient округляется вниз.
fn floor(numerator: &IBig, denominator: &UBig) -> IBig {
    let quotient = numerator / denominator;
    if numerator < &IBig::ZERO && !(numerator % denominator).is_zero() {
        quotient - IBig::ONE
    } else {
        quotient
    }
}

pub fn strict_numerator_bounds(lower: &Rational, upper: &Rational, height: &UBig) -> (IBig, IBig) {
    let (ln, ld) = lower.parts();
    let (un, ud) = upper.parts();
    let scaled_lower = ln * height;
    let negated_upper = -(un * height);
    (
        floor(&scaled_lower, ld) + IBig::ONE,
        -floor(&negated_upper, ud) - IBig::ONE,
    )
}

fn compare_slope(numerator: &IBig, height: &UBig, value: &Rational) -> std::cmp::Ordering {
    let (n, d) = value.parts();
    (numerator * d).cmp(&(n * height))
}

pub(crate) fn vector_from_slope(
    numerator: &IBig,
    height: &UBig,
    window: &SingleChartWindow,
) -> PrimitiveVector {
    let sign = IBig::from(window.denominator_sign);
    let n = numerator * &sign;
    let d = signed(height) * sign;
    if window.use_x {
        PrimitiveVector(d, n)
    } else {
        PrimitiveVector(n, d)
    }
}

/// Цена ВСЕГО integer input span оплачена до gcd/предикатов; empty shell = 1.
pub fn farey_shell_candidates<P: FareyPredicates>(
    window: &SingleChartWindow,
    ordinal: usize,
    height: &UBig,
    budget: &mut WorkBudget,
    predicates: &mut P,
) -> Result<Vec<PrimitiveVector>, FareyFailure<P::Error>> {
    let [outer_lower, inner_lower, inner_upper, outer_upper] = &window.sealed;
    let (first, last) = strict_numerator_bounds(outer_lower, outer_upper, height);
    let probes = if first > last {
        UBig::ONE
    } else {
        (&last - &first + IBig::ONE)
            .try_into()
            .expect("positive span")
    };
    budget
        .spend_shell_probes(probes)
        .map_err(FareyFailure::Work)?;
    let mut result = Vec::new();
    let mut numerator = first;
    while numerator <= last {
        if gcd_signed(&numerator, height).is_one() {
            let lower = compare_slope(&numerator, height, inner_lower);
            let upper = compare_slope(&numerator, height, inner_upper);
            let vector = vector_from_slope(&numerator, height, window);
            let valid = if lower.is_gt() && upper.is_lt() {
                true
            } else if (compare_slope(&numerator, height, outer_lower).is_gt() && !lower.is_gt())
                || (!upper.is_lt() && compare_slope(&numerator, height, outer_upper).is_lt())
            {
                predicates
                    .local_candidate(ordinal, &vector)
                    .map_err(FareyFailure::Predicate)?
            } else {
                false
            };
            if valid {
                result.push(vector);
            }
        }
        numerator += IBig::ONE;
    }
    Ok(result)
}

fn permute<E>(
    items: &[PrimitiveVector],
    indices: Vec<usize>,
) -> Result<Vec<PrimitiveVector>, FareyFailure<E>> {
    let mut seen = vec![false; items.len()];
    if indices.len() != items.len() {
        return Err(FareyFailure::InvalidOrdering);
    }
    let mut result = Vec::with_capacity(items.len());
    for index in indices {
        if index >= items.len() || seen[index] {
            return Err(FareyFailure::InvalidOrdering);
        }
        seen[index] = true;
        result.push(items[index].clone());
    }
    Ok(result)
}

/// Весь вход сортировок оплачен прежде первого comparator; каждое visit edge +1.
pub fn best_fan<P: FareyPredicates, O: FareyOrdering<P>>(
    candidates: &[Vec<PrimitiveVector>],
    budget: &mut WorkBudget,
    predicates: &mut P,
    ordering: &mut O,
) -> Result<Option<Vec<PrimitiveVector>>, FareyFailure<P::Error>> {
    let input_count = candidates
        .iter()
        .fold(UBig::ZERO, |sum, items| sum + UBig::from(items.len()));
    budget
        .spend_order_steps(input_count)
        .map_err(FareyFailure::Work)?;
    let mut ordered = Vec::with_capacity(candidates.len());
    for (index, items) in candidates.iter().enumerate() {
        let indices = ordering
            .iteration_order(index + 1, items)
            .map_err(FareyFailure::Predicate)?;
        let set_order = permute(items, indices)?;
        let indices = ordering
            .sort(index + 1, &set_order, predicates)
            .map_err(FareyFailure::Predicate)?;
        ordered.push(permute(&set_order, indices)?);
    }
    visit(&ordered, 0, &mut Vec::new(), budget, predicates)
}

fn visit<P: FareyPredicates>(
    ordered: &[Vec<PrimitiveVector>],
    ordinal: usize,
    prefix: &mut Vec<PrimitiveVector>,
    budget: &mut WorkBudget,
    predicates: &mut P,
) -> Result<Option<Vec<PrimitiveVector>>, FareyFailure<P::Error>> {
    if ordinal == ordered.len() {
        let previous = prefix
            .last()
            .map_or(Endpoint::Incoming, Endpoint::Primitive);
        return predicates
            .edge_valid(ordinal + 1, previous, Endpoint::Outgoing)
            .map(|valid| if valid { Some(prefix.clone()) } else { None })
            .map_err(FareyFailure::Predicate);
    }
    for vector in &ordered[ordinal] {
        budget
            .spend_order_steps(UBig::ONE)
            .map_err(FareyFailure::Work)?;
        let previous = prefix
            .last()
            .map_or(Endpoint::Incoming, Endpoint::Primitive);
        if !predicates
            .edge_valid(ordinal + 1, previous, Endpoint::Primitive(vector))
            .map_err(FareyFailure::Predicate)?
        {
            continue;
        }
        prefix.push(vector.clone());
        let result = visit(ordered, ordinal + 1, prefix, budget, predicates)?;
        prefix.pop();
        if result.is_some() {
            return Ok(result);
        }
    }
    Ok(None)
}

/// Высоты 1..=stop_height, local-ordinal порядок и previous_counts как у oracle.
pub fn search<P: FareyPredicates, O: FareyOrdering<P>>(
    mode: SearchMode,
    windows: &[SingleChartWindow],
    stop_height: &UBig,
    budget: &mut WorkBudget,
    predicates: &mut P,
    ordering: &mut O,
) -> Result<SearchResult, FareyFailure<P::Error>> {
    if mode == SearchMode::Atlas {
        return Err(FareyFailure::AtlasNotPorted);
    }
    let mut candidates = vec![Vec::new(); windows.len()];
    // HashSet используется ТОЛЬКО для membership; порядок caller читает из Vec.
    let mut membership = vec![HashSet::new(); windows.len()];
    let mut previous_counts = vec![0; windows.len()];
    let mut height = UBig::ONE;
    while &height <= stop_height {
        for (index, window) in windows.iter().enumerate() {
            for vector in farey_shell_candidates(window, index + 1, &height, budget, predicates)? {
                if membership[index].insert(vector.clone()) {
                    candidates[index].push(vector);
                }
            }
        }
        if candidates.iter().all(|items| !items.is_empty()) {
            if let Some(fan) = best_fan(&candidates, budget, predicates, ordering)? {
                return Ok(SearchResult {
                    height: Some(height),
                    fan: Some(fan),
                    previous_counts,
                    counts: candidates.iter().map(Vec::len).collect(),
                });
            }
        }
        previous_counts = candidates.iter().map(Vec::len).collect();
        height += UBig::ONE;
    }
    Ok(SearchResult {
        height: None,
        fan: None,
        previous_counts,
        counts: candidates.iter().map(Vec::len).collect(),
    })
}
