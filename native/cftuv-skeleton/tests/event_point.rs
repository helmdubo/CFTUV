//! `event_point` with a prime universe divides the coordinates of the crossing without making their numerators (it multiplies the radicals of the two lines by the conjugate of the
//! time and adds the rational term over the same denominator). The oracle makes the numerators and divides each by `divisor * determinant` with `_divide_with_prime_universe`:
//! this holds the two equal, as values (terms, types of the coefficients), in the work they spend (the six articles), in the questions they ask the memory (the log, in order)
//! and in the signs they count.

use cftuv_canon::{CanonMemory, MemOp, Operation, WorkBudget};
use cftuv_core::exact::{self, ExactCtx};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::Rat;
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_skeleton::line::SupportLine;
use cftuv_skeleton::time::{concurrency_time, event_point, TimeOutcome};

struct World {
    memory: CanonMemory,
    budget: WorkBudget,
    counts: SignCounts,
    products: ProductMemo,
}

impl World {
    fn new() -> World {
        let mut memory = CanonMemory::new();
        memory.start_log();
        World { memory, budget: WorkBudget::unlimited(), counts: SignCounts::default(), products: ProductMemo::new() }
    }

    fn ctx(&mut self) -> ExactCtx<'_> {
        ExactCtx { memory: &mut self.memory, budget: &mut self.budget, counts: &mut self.counts, products: &mut self.products }
    }

    fn account(&mut self) -> ([u64; 6], Vec<MemOp>, [u64; 5]) {
        (self.budget.articles(), self.memory.take_log(), self.counts.as_array())
    }
}

fn rat(numerator: i64, denominator: i64) -> Rat {
    Rat::new(IBig::from(numerator), IBig::from(denominator)).unwrap()
}

fn rat_of(value: i128) -> Rat {
    Rat::from_int(IBig::from(value))
}

/// The coordinates the oracle makes: both numerators, then one `_divide_with_prime_universe` after the other.
fn the_oracles_point(world: &mut World, first: &SupportLine, second: &SupportLine, time: &cftuv_skeleton::time::EventTime, universe: &[UBig]) -> (SqrtSum, SqrtSum) {
    let determinant = first.determinant(second);
    let mut ctx = world.ctx();
    ctx.budget.spend_exact_position_hydrations(1, Operation::ExactPosition, &first.q_int).unwrap();
    let first_radical = exact::radical(&mut ctx, &time.dividend, &first.q).unwrap();
    let second_radical = exact::radical(&mut ctx, &time.dividend, &second.q).unwrap();
    let right_first = time.divisor.scaled(&rat_of(first.c)).add(&first_radical);
    let right_second = time.divisor.scaled(&rat_of(second.c)).add(&second_radical);
    let x_numerator = right_first.scaled_difference(&rat_of(i128::from(second.b)), &right_second, &rat_of(i128::from(first.b)));
    let y_numerator = right_second.scaled_difference(&rat_of(i128::from(first.a)), &right_first, &rat_of(i128::from(second.a)));
    let scale = time.divisor.scaled(&rat_of(determinant));
    let x = exact::divide_with_prime_universe(&mut ctx, &x_numerator, &scale, universe).unwrap();
    let y = exact::divide_with_prime_universe(&mut ctx, &y_numerator, &scale, universe).unwrap();
    (x, y)
}

#[test]
fn a_place_made_without_its_numerators_is_the_place_the_oracle_makes() {
    // lines of walls and of slanted walls at several speeds (the squared speed is a rational)
    let lines = [
        SupportLine::through((0, 0), (4, 0), 1).unwrap(),
        SupportLine::through((4, 0), (4, 4), 2).unwrap(),
        SupportLine::through((4, 4), (0, 0), 3).unwrap(),
        SupportLine::with_speed((0, 4), (0, 0), rat(2, 1), 4).unwrap(),
        SupportLine::with_speed((0, 0), (3, 5), rat(9, 4), 5).unwrap(),
        SupportLine::with_speed((7, -2), (1, 6), rat(3, 1), 6).unwrap(),
        SupportLine::with_speed((-3, 8), (9, 1), rat(5, 7), 7).unwrap(),
        SupportLine::through((10, 3), (-4, 12), 8).unwrap(),
    ];
    let universe: Vec<UBig> = [2u32, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113, 127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191, 193, 197, 199].iter().map(|prime| UBig::from(*prime)).collect();
    let mut compared = 0;
    for first in 0..lines.len() {
        for second in 0..lines.len() {
            for third in 0..lines.len() {
                if first == second || second == third || first == third {
                    continue;
                }
                // the time at which the three lines meet is the time the places are asked for
                let mut maker = World::new();
                let (time, outcome) = concurrency_time(&mut maker.ctx(), &lines[first], &lines[second], &lines[third]).unwrap();
                let Some(time) = time.filter(|_| outcome == TimeOutcome::Exact) else { continue };
                for (left, right) in [(first, second), (second, third), (first, third)] {
                    if lines[left].determinant(&lines[right]) == 0 {
                        continue;
                    }
                    let (mut ours, mut theirs) = (World::new(), World::new());
                    let point = event_point(&mut ours.ctx(), &lines[left], &lines[right], &time, Some(&universe)).unwrap();
                    let (x, y) = the_oracles_point(&mut theirs, &lines[left], &lines[right], &time, &universe);
                    assert_eq!(point.x, x, "x of lines {left} and {right} at the time of {first}, {second}, {third}");
                    assert_eq!(point.y, y, "y of lines {left} and {right} at the time of {first}, {second}, {third}");
                    assert_eq!(point.x.canonical_form(), x.canonical_form());
                    assert_eq!(point.y.canonical_form(), y.canonical_form());
                    assert_eq!(ours.account(), theirs.account(), "work, memory and signs of lines {left} and {right} at the time of {first}, {second}, {third}");
                    compared += 1;
                }
            }
        }
    }
    assert!(compared > 100, "{compared} places compared");
}
