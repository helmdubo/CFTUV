use std::collections::HashMap;

use cftuv_radical::{Failure, IBig, Priced, Radical, UBig, Work};

use crate::{DensityFailure, DensityResult, InvalidInput, Metric2, Rational, Vector2};

type VectorPair = (Vector2, Vector2);

/// Значения и memo принадлежат только одной prepare/compile-транзакции.
/// Повторное чтение не повторяет exact-работу; ошибки никогда не запоминаются.
pub struct DensityTransaction {
    metric: Metric2,
    gram_dots: HashMap<VectorPair, Radical>,
    dual_dots: HashMap<VectorPair, Radical>,
    crosses: HashMap<VectorPair, Radical>,
    signs: HashMap<Radical, i8>,
    quarter_turns: HashMap<(Vector2, i8), Vector2>,
    subturns: HashMap<(Vector2, Vector2, u8), bool>,
    boundaries: HashMap<(Vector2, Vector2, u8), bool>,
    determinant_root: Option<Radical>,
    q5_coefficient: Option<Radical>,
}

impl DensityTransaction {
    pub fn new(metric: Metric2) -> Self {
        Self {
            metric,
            gram_dots: HashMap::new(),
            dual_dots: HashMap::new(),
            crosses: HashMap::new(),
            signs: HashMap::new(),
            quarter_turns: HashMap::new(),
            subturns: HashMap::new(),
            boundaries: HashMap::new(),
            determinant_root: None,
            q5_coefficient: None,
        }
    }

    pub fn metric(&self) -> &Metric2 {
        &self.metric
    }

    /// `left · G · right` для chart vectors.
    pub fn gram_dot(&mut self, left: &Vector2, right: &Vector2) -> Priced<Radical> {
        dot(&mut self.gram_dots, self.metric.gram(), left, right)
    }

    /// `_dual_dot`: `left · G^-1 · right` для covectors Density.
    pub fn dual_dot(&mut self, left: &Vector2, right: &Vector2) -> Priced<Radical> {
        dot(&mut self.dual_dots, self.metric.inverse_gram(), left, right)
    }

    /// `_oriented_cross`: owner sign умножает chart determinant, без metric scale.
    pub fn oriented_cross(&mut self, left: &Vector2, right: &Vector2) -> Priced<Radical> {
        let key = (left.clone(), right.clone());
        if let Some(cached) = self.crosses.get(&key) {
            return (cached.clone(), Work::NONE);
        }
        let (first, work_a) = left.x.mul(&right.y);
        let (second, work_b) = left.y.mul(&right.x);
        let (difference, work_c) = first.sub(&second);
        let result = difference.scaled(
            &IBig::from(self.metric.owner_orientation_sign()),
            &UBig::ONE,
        );
        self.crosses.insert(key, result.clone());
        self.crosses
            .insert((right.clone(), left.clone()), result.neg());
        (result, work_a + work_b + work_c)
    }

    /// Знак всегда exact: cap Python-поиска сюда не подставляется.
    pub fn sign(&mut self, value: &Radical) -> DensityResult<i8> {
        if let Some(cached) = self.signs.get(value) {
            return Ok((*cached, Work::NONE));
        }
        let (sign, work) = value
            .signum(None)
            .map_err(|failure| DensityFailure::from_algebra(failure, Work::NONE))?;
        self.signs.insert(value.clone(), sign);
        Ok((sign, work))
    }

    /// `_quarter_turn`: `orientation * owner_sign * sqrt(det G) * J * G^-1`.
    /// Корень только рациональный; результат имеет ту же dual-норму.
    pub fn quarter_turn(&mut self, vector: &Vector2, orientation: i8) -> DensityResult<Vector2> {
        if !matches!(orientation, -1 | 1) {
            return Err(DensityFailure::invalid(
                InvalidInput::InvalidTurnOrientation(orientation),
            ));
        }
        let key = (vector.clone(), orientation);
        if let Some(cached) = self.quarter_turns.get(&key) {
            return Ok((cached.clone(), Work::NONE));
        }
        let (dual, mut work) = matrix_vector(self.metric.inverse_gram(), vector);
        let root = match &self.determinant_root {
            Some(root) => root.clone(),
            None => {
                let (n, d) = self.metric.determinant().parts();
                let (root, root_work) =
                    Radical::sqrt_of_rational(n.clone(), d.clone()).map_err(|refusal| {
                        DensityFailure::from_algebra(Failure::Refused(refusal), work)
                    })?;
                work = work + root_work;
                self.determinant_root = Some(root.clone());
                root
            }
        };
        let scale = root.scaled(
            &IBig::from(orientation * self.metric.owner_orientation_sign()),
            &UBig::ONE,
        );
        let (x, work_x) = scale.mul(&dual.y);
        let (y, work_y) = scale.mul(&dual.x);
        let result = Vector2::new(x.neg(), y);
        self.quarter_turns.insert(key, result.clone());
        Ok((result, work + work_x + work_y))
    }

    /// `_subturn` oracle: q=2 принимает dot>=0, q=3..6 требуют dot>0.
    /// Нулевые vectors специально сохраняют этот порядок проверок.
    pub fn subturn(&mut self, left: &Vector2, right: &Vector2, q: u8) -> DensityResult<bool> {
        validate_q(q)?;
        let key = (left.clone(), right.clone(), q);
        if let Some(cached) = self.subturns.get(&key) {
            return Ok((*cached, Work::NONE));
        }
        let (dot_value, mut work) = self.dual_dot(left, right);
        let (dot_sign, sign_work) = self
            .sign(&dot_value)
            .map_err(|failure| failure.after(work))?;
        work = work + sign_work;
        let result = if q == 2 {
            dot_sign >= 0
        } else if dot_sign <= 0 {
            false
        } else {
            let (residual, residual_work) = self
                .subturn_residual(left, right, &dot_value, q)
                .map_err(|failure| failure.after(work))?;
            work = work + residual_work;
            let (sign, sign_work) = self
                .sign(&residual)
                .map_err(|failure| failure.after(work))?;
            work = work + sign_work;
            sign >= 0
        };
        self.subturns.insert(key, result);
        self.subturns
            .insert((right.clone(), left.clone(), q), result);
        Ok((result, work))
    }

    /// `_subturn_boundary` oracle: отрицательный dot отсекается прежде residual.
    pub fn subturn_boundary(
        &mut self,
        left: &Vector2,
        right: &Vector2,
        q: u8,
    ) -> DensityResult<bool> {
        validate_q(q)?;
        let key = (left.clone(), right.clone(), q);
        if let Some(cached) = self.boundaries.get(&key) {
            return Ok((*cached, Work::NONE));
        }
        let (dot_value, mut work) = self.dual_dot(left, right);
        let (dot_sign, sign_work) = self
            .sign(&dot_value)
            .map_err(|failure| failure.after(work))?;
        work = work + sign_work;
        let result = if dot_sign < 0 {
            false
        } else if q == 2 {
            dot_sign == 0
        } else {
            let (residual, residual_work) = self
                .subturn_residual(left, right, &dot_value, q)
                .map_err(|failure| failure.after(work))?;
            work = work + residual_work;
            let (sign, sign_work) = self
                .sign(&residual)
                .map_err(|failure| failure.after(work))?;
            work = work + sign_work;
            sign == 0
        };
        self.boundaries.insert(key, result);
        self.boundaries
            .insert((right.clone(), left.clone(), q), result);
        Ok((result, work))
    }

    fn subturn_residual(
        &mut self,
        left: &Vector2,
        right: &Vector2,
        dot_value: &Radical,
        q: u8,
    ) -> DensityResult<Radical> {
        let (norm_left, first) = self.dual_dot(left, left);
        let (norm_right, second) = self.dual_dot(right, right);
        let (norm_product, third) = norm_left.mul(&norm_right);
        let (dot_squared, fourth) = dot_value.mul(dot_value);
        let mut work = first + second + third + fourth;
        let (a, second_term) = match q {
            3 => (4, norm_product),
            4 => (2, norm_product),
            6 => (4, norm_product.scaled(&IBig::from(3u8), &UBig::ONE)),
            5 => {
                let coefficient = match &self.q5_coefficient {
                    Some(cached) => cached.clone(),
                    None => {
                        // cos²(pi/5)=(3+sqrt(5))/8; вложенный sin(pi/5) не нужен.
                        let (root, root_work) =
                            Radical::sqrt_of_rational(IBig::from(5u8), UBig::ONE).map_err(
                                |refusal| {
                                    DensityFailure::from_algebra(Failure::Refused(refusal), work)
                                },
                            )?;
                        let (coefficient, add_work) =
                            Radical::rational(IBig::from(3u8), UBig::ONE).add(&root);
                        work = work + root_work + add_work;
                        self.q5_coefficient = Some(coefficient.clone());
                        coefficient
                    }
                };
                let (second_term, product_work) = coefficient.mul(&norm_product);
                work = work + product_work;
                (8, second_term)
            }
            _ => {
                return Err(
                    DensityFailure::invalid(InvalidInput::UnsupportedDensityQ(q)).after(work),
                )
            }
        };
        let first_term = dot_squared.scaled(&IBig::from(a), &UBig::ONE);
        let (residual, final_work) = first_term.sub(&second_term);
        Ok((residual, work + final_work))
    }
}

fn validate_q(q: u8) -> Result<(), DensityFailure> {
    if matches!(q, 2..=6) {
        Ok(())
    } else {
        Err(DensityFailure::invalid(InvalidInput::UnsupportedDensityQ(
            q,
        )))
    }
}

fn matrix_vector(matrix: &[[Rational; 2]; 2], vector: &Vector2) -> Priced<Vector2> {
    let (x, first) = matrix[0][0]
        .scale(&vector.x)
        .add(&matrix[0][1].scale(&vector.y));
    let (y, second) = matrix[1][0]
        .scale(&vector.x)
        .add(&matrix[1][1].scale(&vector.y));
    (Vector2::new(x, y), first + second)
}

fn dot(
    memo: &mut HashMap<VectorPair, Radical>,
    matrix: &[[Rational; 2]; 2],
    left: &Vector2,
    right: &Vector2,
) -> Priced<Radical> {
    let key = (left.clone(), right.clone());
    if let Some(cached) = memo.get(&key) {
        return (cached.clone(), Work::NONE);
    }
    let (covector, first) = matrix_vector(matrix, right);
    let (x, second) = left.x.mul(&covector.x);
    let (y, third) = left.y.mul(&covector.y);
    let (value, fourth) = x.add(&y);
    memo.insert(key, value.clone());
    memo.insert((right.clone(), left.clone()), value.clone());
    (value, first + second + third + fourth)
}
