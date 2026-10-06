//! Fused sqrt-sum kernels (`exact_sqrt_sum_fused.py`): several actions on `SqrtSumV1` with ONE fraction
//! normalisation per result term. The values, terms and order are those of the plain chain; so are the
//! coefficient types (`product_added` hands the base terms the product does not touch through as the same
//! coefficient objects, which keeps an `int` an `int`).

use std::cmp::Ordering;

use std::sync::OnceLock;

use crate::fx::{self, FxItems};
use crate::num::{self, IBig, UBig};
use crate::products::{accumulate_products, Accumulator, ProductMemo};
use crate::rat::{Coef, Rat};
use crate::sqrt_sum::{IntForm, SqrtSum, Term};
use crate::wide::Wide;

/// The right factor of [`product_added_form`]: an integer form with any common scale and no canonical terms behind it, that the conjugation loop
/// hands over on the stack when it ended there (the form of `dashu-int` integers is made only if somebody asks for it).
pub struct Share {
    form: OnceLock<IntForm>,
    stack: Option<Box<(Wide, FxItems)>>,
}

impl Share {
    pub fn from_form(form: IntForm) -> Share {
        Share { form: OnceLock::from(form), stack: None }
    }

    pub(crate) fn from_stack(common: Wide, items: FxItems) -> Share {
        Share { form: OnceLock::new(), stack: Some(Box::new((common, items))) }
    }

    pub fn form(&self) -> &IntForm {
        self.form.get_or_init(|| {
            let (common, items) = &**self.stack.as_ref().expect("a share is a form or a stack twin");
            IntForm { common: common.to_ubig(), items: items.to_items() }
        })
    }

    /// The value the form stands for.
    pub fn into_sqrt_sum(self) -> SqrtSum {
        self.form().clone().into_sqrt_sum()
    }

    pub fn is_empty(&self) -> bool {
        match &self.stack {
            Some(twin) => twin.1.is_empty(),
            None => self.form().items.is_empty(),
        }
    }
}

/// A right factor, borrowed.
enum Right<'a> {
    Form(&'a IntForm),
    Share(&'a Share),
}

/// `step_x * y - step_y * x + offset` for integer steps and offset, with one normalisation per result term.
pub fn oriented_sum(x: &SqrtSum, y: &SqrtSum, step_x: &IBig, step_y: &IBig, offset: &IBig) -> SqrtSum {
    let (x_form, y_form) = (x.int_form(), y.int_form());
    let scale = if x_form.common != y_form.common { num::lcm(&x_form.common, &y_form.common) } else { x_form.common.clone() };
    let mut merged = Accumulator::with_capacity(x_form.items.len() + y_form.items.len() + 1);
    if !step_x.is_zero() {
        let factor = step_x * (&scale / &y_form.common);
        for (radicand, numerator) in &y_form.items {
            merged.add(radicand, numerator * &factor);
        }
    }
    if !step_y.is_zero() {
        let factor = step_y * (&scale / &x_form.common);
        for (radicand, numerator) in &x_form.items {
            merged.add(radicand, -(numerator * &factor));
        }
    }
    if !offset.is_zero() {
        merged.add(&UBig::ONE, offset * &scale);
    }
    from_scaled(merged, &scale)
}

/// `base + left * right` with one normalisation per result term.
pub fn product_added(base: &SqrtSum, left: &SqrtSum, right: &SqrtSum, memo: &mut ProductMemo) -> SqrtSum {
    if left.is_zero() || right.is_zero() {
        return base.clone();
    }
    product_added_with(base, left, Right::Form(right.int_form()), memo)
}

/// [`product_added`] with the right factor given as an integer form (any common scale, no canonical terms behind
/// it): the result is the same canonical value, type for type, because every result term is normalised at the end.
pub fn product_added_form(base: &SqrtSum, left: &SqrtSum, right: &Share, memo: &mut ProductMemo) -> SqrtSum {
    product_added_with(base, left, Right::Share(right), memo)
}

fn product_added_with(base: &SqrtSum, left: &SqrtSum, right: Right<'_>, memo: &mut ProductMemo) -> SqrtSum {
    let empty = match &right {
        Right::Form(form) => form.items.is_empty(),
        Right::Share(share) => share.is_empty(),
    };
    if left.is_zero() || empty {
        return base.clone();
    }
    let right_form = || match &right {
        Right::Form(form) => *form,
        Right::Share(share) => share.form(),
    };
    if base.has_py_int() {
        return product_added_typed(base, left, right_form(), memo);
    }
    let stacked = match &right {
        Right::Share(Share { stack: Some(twin), .. }) => fx::product_added(base, left, &twin.0, &twin.1, memo),
        _ => {
            let form = right_form();
            match (Wide::from_ubig(&form.common), FxItems::from_items(&form.items)) {
                (Some(common), Some(items)) => fx::product_added(base, left, &common, &items, memo),
                _ => None,
            }
        }
    };
    if let Some(sum) = stacked {
        return sum;
    }
    let right_form = right_form();
    // no coefficient of the base is a Python `int`, so the sum is one value over a common denominator and nothing else: the base terms the
    // product does not touch are the same coefficients in that form (as `Fraction`s they were, as `Fraction`s they stay)
    let (base_form, left_form) = (base.int_form(), left.int_form());
    let common = &left_form.common * &right_form.common;
    let scale = if base_form.common != common { num::lcm(&base_form.common, &common) } else { common.clone() };
    let mut merged = Accumulator::with_capacity(base_form.items.len() + left_form.items.len() + right_form.items.len());
    accumulate_products(&mut merged, &left_form.items, &right_form.items, &IBig::from(&scale / &common), memo);
    let factor = IBig::from(&scale / &base_form.common);
    for (radicand, numerator) in &base_form.items {
        merged.add(radicand, numerator * &factor);
    }
    from_scaled(merged, &scale)
}

/// [`product_added_form`] for a base that carries Python `int` coefficients: the base terms the product does not touch are handed through as the
/// same coefficient objects (an `int` stays an `int`), so the result is built term by term.
fn product_added_typed(base: &SqrtSum, left: &SqrtSum, right_form: &IntForm, memo: &mut ProductMemo) -> SqrtSum {
    let (base_form, left_form) = (base.int_form(), left.int_form());
    let common = &left_form.common * &right_form.common;
    let scale = if base_form.common != common { num::lcm(&base_form.common, &common) } else { common.clone() };
    let mut product = Accumulator::new();
    accumulate_products(&mut product, &left_form.items, &right_form.items, &IBig::from(&scale / &common), memo);
    let factor = IBig::from(&scale / &base_form.common);
    let mut out: Vec<Term> = Vec::with_capacity(product.len() + base.terms().len());
    let mut base_index = 0;
    let base_terms = base.terms();
    let push_total = |out: &mut Vec<Term>, radicand: &UBig, total: IBig| {
        if !total.is_zero() {
            out.push(Term { radicand: radicand.clone(), coef: Coef::fraction(Rat::reduced(total, scale.clone())) });
        }
    };
    let pass_through = |out: &mut Vec<Term>, term: &Term| {
        if !term.coef.is_zero() {
            out.push(term.clone());
        }
    };
    for (radicand, value) in product.iter() {
        // base terms below this radicand are untouched by the product: they pass through as the same objects
        while base_index < base_terms.len() && base_terms[base_index].radicand < *radicand {
            pass_through(&mut out, &base_terms[base_index]);
            base_index += 1;
        }
        let matching = base_terms.get(base_index).filter(|term| term.radicand.cmp(radicand) == Ordering::Equal);
        match matching {
            Some(term) => {
                if value.is_zero() {
                    pass_through(&mut out, term);
                } else {
                    let numerator = &base_form.items[base_index].1;
                    push_total(&mut out, radicand, value + numerator * &factor);
                }
                base_index += 1;
            }
            None => {
                if !value.is_zero() {
                    push_total(&mut out, radicand, value.clone());
                }
            }
        }
    }
    for term in &base_terms[base_index..] {
        pass_through(&mut out, term);
    }
    SqrtSum::from_terms_unchecked(out)
}

/// `sum sign_i * left_i * right_i` (the sign is `+1` or `-1`, any integer works) with ONE normalisation at the
/// end. A zero factor skips its summand, as `__mul__` would.
pub fn sum_of_products(products: &[(&SqrtSum, &SqrtSum, IBig)], memo: &mut ProductMemo) -> SqrtSum {
    let mut scale = UBig::ONE;
    let mut forms = Vec::with_capacity(products.len());
    for (left, right, sign) in products {
        if left.is_zero() || right.is_zero() {
            continue;
        }
        let (left_form, right_form) = (left.int_form(), right.int_form());
        let common = &left_form.common * &right_form.common;
        if !common.is_one() {
            scale = num::lcm(&scale, &common);
        }
        forms.push((left_form, right_form, sign, common));
    }
    let mut merged = Accumulator::new();
    for (left_form, right_form, sign, common) in forms {
        let weight = sign * IBig::from(&scale / &common);
        accumulate_products(&mut merged, &left_form.items, &right_form.items, &weight, memo);
    }
    from_scaled(merged, &scale)
}

/// `(radicand, Fraction(value, scale))` for the non-zero values, by radicand.
fn from_scaled(merged: Accumulator, scale: &UBig) -> SqrtSum {
    IntForm { common: scale.clone(), items: merged.into_nonzero_items() }.into_sqrt_sum()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rat(n: i64, d: i64) -> Rat {
        Rat::new(IBig::from(n), IBig::from(d)).unwrap()
    }

    fn frac(radicand: u64, n: i64, d: i64) -> Term {
        Term { radicand: UBig::from(radicand), coef: Coef::fraction(rat(n, d)) }
    }

    fn int(radicand: u64, n: i64) -> Term {
        Term { radicand: UBig::from(radicand), coef: Coef::int(IBig::from(n)) }
    }

    fn sum(terms: Vec<Term>) -> SqrtSum {
        SqrtSum::from_terms(terms).unwrap()
    }

    #[test]
    fn oriented_sum_equals_the_chain() {
        let x = sum(vec![frac(1, 1, 2), frac(2, 1, 3)]);
        let y = sum(vec![frac(2, 5, 4), frac(3, -1, 1)]);
        let fused = oriented_sum(&x, &y, &IBig::from(3), &IBig::from(-2), &IBig::from(7));
        let chain = y.scaled(&rat(3, 1)).sub(&x.scaled(&rat(-2, 1))).add(&SqrtSum::rational(&rat(7, 1)));
        assert_eq!(fused, chain);
        // zero steps and offset skip their parts
        assert_eq!(oriented_sum(&x, &y, &IBig::ZERO, &IBig::ZERO, &IBig::ZERO), SqrtSum::zero());
        assert_eq!(oriented_sum(&x, &y, &IBig::ZERO, &IBig::ZERO, &IBig::from(5)), SqrtSum::rational(&rat(5, 1)));
    }

    #[test]
    fn product_added_equals_the_chain_and_keeps_untouched_base_types() {
        let mut memo = ProductMemo::new();
        let base = sum(vec![int(1, 2), int(5, 3), frac(7, 1, 3)]);
        let left = sum(vec![frac(1, 1, 2), frac(5, 1, 1)]);
        let right = sum(vec![frac(1, 3, 1), frac(2, 1, 2)]);
        let fused = product_added(&base, &left, &right, &mut memo);
        let chain = base.add(&left.mul(&right, &mut memo));
        assert_eq!(fused, chain);
        // radicand 7 is only in the base: the Fraction stays a Fraction; with an int base it stays an int
        let int_base = sum(vec![int(7, 4)]);
        let fused = product_added(&int_base, &left, &right, &mut memo);
        let kept = fused.terms().iter().find(|term| term.radicand == UBig::from(7u8)).unwrap();
        assert_eq!(kept.coef, Coef::int(IBig::from(4)));
        let kept = product_added(&base, &left, &right, &mut memo);
        assert_eq!(kept.terms().iter().find(|term| term.radicand == UBig::from(7u8)).unwrap().coef, Coef::fraction(rat(1, 3)));
        // a zero factor returns the base untouched
        assert_eq!(product_added(&int_base, &SqrtSum::zero(), &right, &mut memo), int_base);
    }

    #[test]
    fn product_added_drops_terms_that_cancel() {
        let mut memo = ProductMemo::new();
        // base = 1; left*right = -1  ->  0
        let base = sum(vec![int(1, 1)]);
        let left = sum(vec![frac(1, 1, 1), frac(2, 1, 1)]);
        let right = sum(vec![frac(1, 1, 1), frac(2, -1, 1)]);
        assert!(product_added(&base, &left, &right, &mut memo).is_zero());
        // a cancelled product term leaves the base term as the same object (here an int)
        let left = sum(vec![frac(2, 1, 1)]);
        let right = sum(vec![frac(3, 1, 1)]);
        let base = sum(vec![int(6, 5)]);
        // sqrt(2)*sqrt(3) = sqrt(6): total 1 + 5 = 6, a Fraction
        assert_eq!(product_added(&base, &left, &right, &mut memo), sum(vec![frac(6, 6, 1)]));
    }

    #[test]
    fn sum_of_products_equals_the_chain() {
        let mut memo = ProductMemo::new();
        let a = sum(vec![frac(1, 1, 2), frac(2, 1, 3)]);
        let b = sum(vec![frac(2, 5, 4), frac(3, -1, 1)]);
        let c = sum(vec![frac(1, 7, 5)]);
        let products = [(&a, &b, IBig::ONE), (&b, &c, IBig::from(-1)), (&a, &SqrtSum::zero(), IBig::ONE), (&c, &a, IBig::ONE)];
        let fused = sum_of_products(&products, &mut memo);
        let chain = a.mul(&b, &mut memo).sub(&b.mul(&c, &mut memo)).add(&c.mul(&a, &mut memo));
        assert_eq!(fused, chain);
        assert!(sum_of_products(&[], &mut memo).is_zero());
    }
}
