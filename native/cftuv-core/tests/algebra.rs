//! Algebraic laws of the sqrt-sum arithmetic on random structured operands (a property layer over the
//! per-function tests; the differential tests against the Python oracle live in `tests/test_native_numbers.py`).
//!
//! Operands: radicands are products of distinct primes from a small random universe (so squarefree), coefficients
//! are integers or fractions of 1 to ~300 bits, negative and positive, `int`-typed and `Fraction`-typed.

use cftuv_canon::{CanonMemory, WorkBudget};
use cftuv_core::exact::{divided_by, divided_by_form, ExactCtx, Quotient};
use cftuv_core::fused::{oriented_sum, product_added, product_added_form, sum_of_products};
use cftuv_core::num::{IBig, UBig};
use cftuv_core::products::ProductMemo;
use cftuv_core::rat::{Coef, Rat};
use cftuv_core::sqrt_sum::{integer_certified_sign, scaled_difference_parts, SignCounts, SqrtSum, Term};

struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    fn below(&mut self, bound: u64) -> u64 {
        self.next() % bound
    }

    fn integer(&mut self, bits: usize) -> IBig {
        let bytes: Vec<u8> = (0..bits.div_ceil(8)).map(|_| self.next() as u8).collect();
        let magnitude = (UBig::from_le_bytes(&bytes) >> (bytes.len() * 8 - bits)) | (UBig::ONE << (bits - 1));
        if self.below(2) == 0 {
            IBig::from(magnitude)
        } else {
            -IBig::from(magnitude)
        }
    }

    fn bits(&mut self) -> usize {
        const SIZES: [usize; 8] = [1, 3, 8, 31, 64, 65, 130, 300];
        SIZES[self.below(8) as usize]
    }

    fn coefficient(&mut self) -> Coef {
        let bits = self.bits();
        let numerator = self.integer(bits);
        if self.below(3) == 0 {
            return Coef::int(numerator);
        }
        let denominator = if self.below(2) == 0 {
            UBig::from(1 + self.below(12))
        } else {
            let bits = self.bits();
            cftuv_core::num::magnitude(&self.integer(bits))
        };
        Coef::fraction(Rat::reduced(numerator, denominator))
    }

    fn sum(&mut self, universe: &[u64], max_terms: u64) -> SqrtSum {
        let mut radicands: Vec<UBig> = vec![UBig::ONE];
        for _ in 0..12 {
            let mut product = UBig::ONE;
            for prime in universe {
                if self.below(3) == 0 {
                    product *= UBig::from(*prime);
                }
            }
            if !radicands.contains(&product) {
                radicands.push(product);
            }
        }
        radicands.sort();
        let wanted = self.below(max_terms + 1) as usize;
        let mut taken: Vec<UBig> = Vec::new();
        for _ in 0..wanted {
            let pick = radicands[self.below(radicands.len() as u64) as usize].clone();
            if !taken.contains(&pick) {
                taken.push(pick);
            }
        }
        taken.sort();
        let terms = taken
            .into_iter()
            .map(|radicand| {
                let mut coef = self.coefficient();
                while coef.is_zero() {
                    coef = self.coefficient();
                }
                Term { radicand, coef }
            })
            .collect();
        SqrtSum::from_terms(terms).unwrap()
    }
}

const PRIMES: [u64; 9] = [2, 3, 5, 7, 11, 13, 17, 19, 23];

/// Equality of values: the Python `int`/`Fraction` type of a coefficient is not part of the value.
fn same_value(left: &SqrtSum, right: &SqrtSum) -> bool {
    left.terms().len() == right.terms().len()
        && left.terms().iter().zip(right.terms()).all(|(a, b)| a.radicand == b.radicand && a.coef.value() == b.coef.value())
}

fn cases(count: usize, seed: u64, mut body: impl FnMut(&mut Rng, &[u64])) {
    let mut rng = Rng(seed);
    for _ in 0..count {
        let universe: Vec<u64> = PRIMES.iter().copied().filter(|_| rng.below(5) != 0).collect();
        let universe = if universe.len() < 3 { PRIMES[..4].to_vec() } else { universe };
        body(&mut rng, &universe);
    }
}

#[test]
fn addition_is_a_commutative_group_on_values() {
    cases(400, 1, |rng, universe| {
        let (a, b, c) = (rng.sum(universe, 6), rng.sum(universe, 6), rng.sum(universe, 6));
        assert!(same_value(&a.add(&b), &b.add(&a)));
        assert!(same_value(&a.add(&b).add(&c), &a.add(&b.add(&c))));
        assert!(a.add(&a.neg()).is_zero());
        assert!(a.sub(&a).is_zero());
        assert!(same_value(&a.sub(&b), &a.add(&b.neg())));
        assert_eq!(a.neg().neg(), a, "negation is an involution, types included");
        assert_eq!(a.add(&SqrtSum::zero()), a, "adding zero keeps the types of the left operand");
    });
}

#[test]
fn multiplication_is_commutative_associative_and_distributes() {
    let mut memo = ProductMemo::new();
    cases(200, 2, |rng, universe| {
        let (a, b, c) = (rng.sum(universe, 4), rng.sum(universe, 4), rng.sum(universe, 4));
        assert!(same_value(&a.mul(&b, &mut memo), &b.mul(&a, &mut memo)));
        let left = a.mul(&b, &mut memo).mul(&c, &mut memo);
        let right = a.mul(&b.mul(&c, &mut memo), &mut memo);
        assert!(same_value(&left, &right));
        let distributed = a.mul(&b, &mut memo).add(&a.mul(&c, &mut memo));
        assert!(same_value(&a.mul(&b.add(&c), &mut memo), &distributed));
        assert!(a.mul(&SqrtSum::zero(), &mut memo).is_zero());
        assert!(same_value(&a.mul(&SqrtSum::rational(&Rat::one()), &mut memo), &a));
    });
}

#[test]
fn scaling_composes_and_scaled_difference_is_the_chain() {
    cases(300, 3, |rng, universe| {
        let (a, b) = (rng.sum(universe, 5), rng.sum(universe, 5));
        let (f, g) = (rng.coefficient().into_value(), rng.coefficient().into_value());
        assert!(same_value(&a.scaled(&f).scaled(&g), &a.scaled(&f.mul(&g))));
        let fused = a.scaled_difference(&f, &b, &g);
        assert_eq!(fused, a.scaled(&f).sub(&b.scaled(&g)), "the one-pass kernel equals the chain, types included");
        assert_eq!(a.difference_is_zero(&b), a.sub(&b).is_zero());
    });
}

#[test]
fn the_fused_kernels_equal_their_chains_including_coefficient_types() {
    let mut memo = ProductMemo::new();
    cases(300, 4, |rng, universe| {
        let (base, left, right) = (rng.sum(universe, 5), rng.sum(universe, 4), rng.sum(universe, 4));
        assert_eq!(product_added(&base, &left, &right, &mut memo), base.add(&left.mul(&right, &mut memo)));
        let (x, y) = (rng.sum(universe, 5), rng.sum(universe, 5));
        let mut step = |limit: u64| {
            let bits = 1 + rng.below(limit) as usize;
            rng.integer(bits)
        };
        let steps = [step(70), step(70), step(40)];
        let chain = y
            .scaled(&Rat::from_int(steps[0].clone()))
            .sub(&x.scaled(&Rat::from_int(steps[1].clone())))
            .add(&SqrtSum::rational(&Rat::from_int(steps[2].clone())));
        assert_eq!(oriented_sum(&x, &y, &steps[0], &steps[1], &steps[2]), chain);
        let (p, q) = (rng.sum(universe, 3), rng.sum(universe, 3));
        let products = [(&left, &right, IBig::ONE), (&p, &q, IBig::NEG_ONE), (&base, &p, IBig::ONE)];
        let expected = left.mul(&right, &mut memo).sub(&p.mul(&q, &mut memo)).add(&base.mul(&p, &mut memo));
        assert!(same_value(&sum_of_products(&products, &mut memo), &expected));
    });
}

/// One division on a fresh canonicalization memory under a cap: the answer, the articles after, the whole ordered memory.
fn division_run(numerator: &SqrtSum, denominator: &SqrtSum, cap: Option<u64>, as_form: bool) -> (Result<SqrtSum, String>, [u64; 6], cftuv_canon::MemoryState) {
    let (mut memory, mut counts, mut products) = (CanonMemory::new(), SignCounts::default(), ProductMemo::new());
    let mut budget = cap.map_or_else(WorkBudget::unlimited, WorkBudget::bounded);
    let result = {
        let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
        if as_form {
            divided_by_form(&mut ctx, numerator, denominator).map(|quotient| match quotient {
                Quotient::Form(form) => form.into_sqrt_sum(),
                Quotient::Sum(sum) => sum,
            })
        } else {
            divided_by(&mut ctx, numerator, denominator)
        }
    };
    (result.map_err(|error| format!("{error:?}")), budget.articles(), memory.export_state())
}

#[test]
fn the_division_as_a_form_is_the_division_with_the_same_cost_the_same_memory_and_the_same_refusals() {
    let mut memo = ProductMemo::new();
    let (mut done, mut refused) = (0, 0);
    cases(250, 31, |rng, universe| {
        let (numerator, denominator) = (rng.sum(universe, 5), rng.sum(universe, 4));
        for cap in [None, Some(rng.below(4)), Some(rng.below(30))] {
            let (plain, plain_articles, plain_memory) = division_run(&numerator, &denominator, cap, false);
            let (form, form_articles, form_memory) = division_run(&numerator, &denominator, cap, true);
            assert_eq!(plain, form, "the quotient, canonical term by term, types included (or the same refusal)");
            assert_eq!(plain_articles, form_articles, "the same budget paid");
            assert_eq!(plain_memory, form_memory, "the same memory, in the same order");
            match plain {
                Ok(_) => done += 1,
                Err(_) => refused += 1,
            }
        }
        // multiplied on, the form and the canonical quotient give one value of one type
        let (Ok(sum), _, _) = division_run(&numerator, &denominator, None, false) else {
            return;
        };
        let (mut memory, mut counts, mut products) = (CanonMemory::new(), SignCounts::default(), ProductMemo::new());
        let mut budget = WorkBudget::unlimited();
        let mut ctx = ExactCtx { memory: &mut memory, budget: &mut budget, counts: &mut counts, products: &mut products };
        let Ok(Quotient::Form(form)) = divided_by_form(&mut ctx, &numerator, &denominator) else {
            return;
        };
        let (base, left) = (rng.sum(universe, 5), rng.sum(universe, 4));
        assert_eq!(product_added_form(&base, &left, &form, &mut memo), product_added(&base, &left, &sum, &mut memo));
    });
    assert!(done > 300 && refused > 30, "the cases must reach both the answers and the exhaustions: {done} / {refused}");
}

#[test]
fn the_product_memory_never_changes_an_answer() {
    let (mut warm, mut cold) = (ProductMemo::new(), ProductMemo::disabled());
    cases(200, 5, |rng, universe| {
        let (a, b) = (rng.sum(universe, 6), rng.sum(universe, 6));
        assert_eq!(a.mul(&b, &mut warm), a.mul(&b, &mut cold));
        assert_eq!(a.mul(&b, &mut warm), a.mul(&b, &mut warm), "a second call is served from memory");
    });
}

#[test]
fn the_enclosure_brackets_the_value_and_certified_signs_agree_with_it() {
    cases(300, 6, |rng, universe| {
        let a = rng.sum(universe, 4);
        let b = rng.sum(universe, 4);
        let difference = a.sub(&b);
        let (low, high) = difference.enclosure(64);
        assert!(low <= high);
        if difference.is_rational() {
            assert_eq!(low, high, "a rational has a point enclosure");
        }
        match difference.certified_sign(64) {
            Some(1) => assert!(low.signum() > 0),
            Some(-1) => assert!(high.signum() < 0),
            Some(other) => panic!("sign {other}"),
            None => assert!(low.signum() <= 0 && high.signum() >= 0),
        }
        let parts = scaled_difference_parts(&a, &Rat::one(), &b, &Rat::one());
        assert_eq!(integer_certified_sign(&parts.items, 64), difference.certified_sign(64), "the difference filter decides like the difference itself");
    });
}
