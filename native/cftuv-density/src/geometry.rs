//! Exact V2 геометрия Farey-предикатов; без бюджета поиска и product dispatcher.
//! Source: density_projective._inside_ordinal, adaptive_density_fan local/
//! objective/compare/edge, adaptive_density_band real-neighbour/phantom laws.
//! Ошибка V2 остаётся внутренней: не называется продуктовым UNDECIDABLE.

use std::cmp::Ordering;
use std::collections::HashMap;

use cftuv_radical::arith::{fraction, magnitude, signed, signum};

use crate::farey::{Endpoint, FareyPredicates, PrimitiveVector};
use crate::{
    DensityFailure, DensityFailureKind, DensityResult, DensityTransaction, IBig, Priced, Radical,
    UBig, Vector2, Work,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindowLaw {
    Voronoi,
    NarrowBand,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum GeometryFailureKind {
    InvalidQ(u8),
    InvalidOrientation(i8),
    MissingIncidentSupports,
    InvalidOrdinal(usize),
    InternalDensity(DensityFailureKind),
    NonRationalCandidateNorm,
    NonPositiveCandidateNorm,
    ZeroSlopeDenominator,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct GeometryFailure {
    pub kind: GeometryFailureKind,
    pub work: Work,
}

pub struct GeometryAdapter {
    transaction: DensityTransaction,
    ideal: Vec<Vector2>,
    q: u8,
    expected_orientation: i8,
    window_law: WindowLaw,
    band_cache: HashMap<usize, (Vector2, Vector2)>,
    work: Work,
}

impl GeometryAdapter {
    pub fn new(
        transaction: DensityTransaction,
        ideal: Vec<Vector2>,
        q: u8,
        expected_orientation: i8,
        window_law: WindowLaw,
    ) -> Result<Self, GeometryFailure> {
        let invalid = if !matches!(q, 2..=6) {
            Some(GeometryFailureKind::InvalidQ(q))
        } else if !matches!(expected_orientation, -1 | 1) {
            Some(GeometryFailureKind::InvalidOrientation(
                expected_orientation,
            ))
        } else if ideal.len() < 2 {
            Some(GeometryFailureKind::MissingIncidentSupports)
        } else {
            None
        };
        if let Some(kind) = invalid {
            return Err(GeometryFailure {
                kind,
                work: Work::NONE,
            });
        }
        Ok(Self {
            transaction,
            ideal,
            q,
            expected_orientation,
            window_law,
            band_cache: HashMap::new(),
            work: Work::NONE,
        })
    }

    pub fn work(&self) -> Work {
        self.work
    }
    pub fn ideal_covectors(&self) -> &[Vector2] {
        &self.ideal
    }
    pub fn window_law(&self) -> WindowLaw {
        self.window_law
    }

    fn error(&self, kind: GeometryFailureKind) -> GeometryFailure {
        GeometryFailure {
            kind,
            work: self.work,
        }
    }

    fn charged<T>(&mut self, priced: Priced<T>) -> T {
        self.work = self.work + priced.1;
        priced.0
    }

    fn checked<T>(&mut self, result: DensityResult<T>) -> Result<T, GeometryFailure> {
        match result {
            Ok(priced) => Ok(self.charged(priced)),
            Err(DensityFailure { kind, work }) => {
                self.work = self.work + work;
                Err(self.error(GeometryFailureKind::InternalDensity(kind)))
            }
        }
    }

    fn ordinal(&self, ordinal: usize) -> Result<(), GeometryFailure> {
        if ordinal == 0 || ordinal >= self.ideal.len() - 1 {
            Err(self.error(GeometryFailureKind::InvalidOrdinal(ordinal)))
        } else {
            Ok(())
        }
    }

    fn dual_dot(&mut self, left: &Vector2, right: &Vector2) -> Radical {
        let result = self.transaction.dual_dot(left, right);
        self.charged(result)
    }

    fn sign(&mut self, expression: &Radical) -> Result<i8, GeometryFailure> {
        let result = self.transaction.sign(expression);
        self.checked(result)
    }

    fn subturn(&mut self, left: &Vector2, right: &Vector2) -> Result<bool, GeometryFailure> {
        let result = self.transaction.subturn(left, right, self.q);
        self.checked(result)
    }

    fn difference(&mut self, left: &Vector2, right: &Vector2) -> Vector2 {
        let x = self.charged(left.x.sub(&right.x));
        let y = self.charged(left.y.sub(&right.y));
        Vector2::new(x, y)
    }

    /// Обе разности строятся до первого знака; второй dot читается только если первый >0.
    pub fn inside_window(
        &mut self,
        candidate: &Vector2,
        previous: &Vector2,
        center: &Vector2,
        following: &Vector2,
    ) -> Result<bool, GeometryFailure> {
        let from_previous = self.difference(center, previous);
        let from_following = self.difference(center, following);
        let left = self.dual_dot(candidate, &from_previous);
        if self.sign(&left)? <= 0 {
            return Ok(false);
        }
        let right = self.dual_dot(candidate, &from_following);
        Ok(self.sign(&right)? > 0)
    }

    pub fn inside_ordinal(
        &mut self,
        ordinal: usize,
        candidate: &PrimitiveVector,
    ) -> Result<bool, GeometryFailure> {
        let (previous, following) = self.window_neighbours(ordinal)?;
        let center = self.ideal[ordinal].clone();
        self.inside_window(&integer_vector(candidate), &previous, &center, &following)
    }

    /// Exact dot>0, затем dot²-cos²(2omega)*|center|²*|real|²>=0.
    pub fn neighbour_is_inside_double_band(
        &mut self,
        center: &Vector2,
        real: &Vector2,
    ) -> Result<bool, GeometryFailure> {
        let dot = self.dual_dot(center, real);
        if self.sign(&dot)? <= 0 {
            return Ok(false);
        }
        let norm_center = self.dual_dot(center, center);
        let norm_real = self.dual_dot(real, real);
        let norms = self.charged(norm_center.mul(&norm_real));
        let dot_squared = self.charged(dot.mul(&dot));
        let second = norms.scaled(&IBig::from(3248u32 * 3248), &UBig::from(3250u32 * 3250));
        let residual = self.charged(dot_squared.sub(&second));
        Ok(self.sign(&residual)? >= 0)
    }

    pub fn rotated_by_double_band(
        &mut self,
        center: &Vector2,
        side: i8,
    ) -> Result<Vector2, GeometryFailure> {
        if !matches!(side, -1 | 1) {
            return Err(self.error(GeometryFailureKind::InvalidOrientation(side)));
        }
        let result = self
            .transaction
            .quarter_turn(center, self.expected_orientation);
        let perpendicular = self.checked(result)?;
        let cosine_x = center.x.scaled(&IBig::from(3248u32), &UBig::from(3250u32));
        let cosine_y = center.y.scaled(&IBig::from(3248u32), &UBig::from(3250u32));
        let sine = IBig::from(side) * 114u8;
        let sine_x = perpendicular.x.scaled(&sine, &UBig::from(3250u32));
        let sine_y = perpendicular.y.scaled(&sine, &UBig::from(3250u32));
        let x = self.charged(cosine_x.add(&sine_x));
        let y = self.charged(cosine_y.add(&sine_y));
        Ok(Vector2::new(x, y))
    }

    pub fn window_neighbours(
        &mut self,
        ordinal: usize,
    ) -> Result<(Vector2, Vector2), GeometryFailure> {
        self.ordinal(ordinal)?;
        if self.window_law == WindowLaw::Voronoi {
            return Ok((
                self.ideal[ordinal - 1].clone(),
                self.ideal[ordinal + 1].clone(),
            ));
        }
        if let Some(cached) = self.band_cache.get(&ordinal) {
            return Ok(cached.clone());
        }
        let center = self.ideal[ordinal].clone();
        let previous = self.ideal[ordinal - 1].clone();
        let following = self.ideal[ordinal + 1].clone();
        let previous = if self.neighbour_is_inside_double_band(&center, &previous)? {
            previous
        } else {
            self.rotated_by_double_band(&center, -1)?
        };
        let following = if self.neighbour_is_inside_double_band(&center, &following)? {
            following
        } else {
            self.rotated_by_double_band(&center, 1)?
        };
        self.band_cache
            .insert(ordinal, (previous.clone(), following.clone()));
        Ok((previous, following))
    }

    /// Integer candidate -> rational positive norm; general radical inverse не нужен.
    pub fn objective(
        &mut self,
        ordinal: usize,
        candidate: &PrimitiveVector,
    ) -> Result<Radical, GeometryFailure> {
        self.ordinal(ordinal)?;
        let center = self.ideal[ordinal].clone();
        let candidate = integer_vector(candidate);
        let dot = self.dual_dot(&candidate, &center);
        let norm = self.dual_dot(&candidate, &candidate);
        let (numerator, denominator) = norm
            .as_rational()
            .ok_or_else(|| self.error(GeometryFailureKind::NonRationalCandidateNorm))?;
        if signum(&numerator) <= 0 {
            return Err(self.error(GeometryFailureKind::NonPositiveCandidateNorm));
        }
        let squared = self.charged(dot.mul(&dot));
        let reciprocal_denominator = magnitude(&numerator);
        Ok(squared.scaled(&signed(&denominator), &reciprocal_denominator))
    }

    fn endpoint(&self, endpoint: Endpoint<'_>) -> Vector2 {
        match endpoint {
            Endpoint::Incoming => self.ideal[0].clone(),
            Endpoint::Outgoing => self.ideal.last().expect("incident supports").clone(),
            Endpoint::Primitive(vector) => integer_vector(vector),
        }
    }
}

fn integer_vector(vector: &PrimitiveVector) -> Vector2 {
    Vector2::new(
        Radical::rational(vector.0.clone(), UBig::ONE),
        Radical::rational(vector.1.clone(), UBig::ONE),
    )
}

impl FareyPredicates for GeometryAdapter {
    type Error = GeometryFailure;

    fn local_candidate(
        &mut self,
        ordinal: usize,
        candidate: &PrimitiveVector,
    ) -> Result<bool, GeometryFailure> {
        if !self.inside_ordinal(ordinal, candidate)? {
            return Ok(false);
        }
        let previous = self.ideal[ordinal - 1].clone();
        let following = self.ideal[ordinal + 1].clone();
        let candidate = integer_vector(candidate);
        if !self.subturn(&previous, &candidate)? {
            return Ok(false);
        }
        self.subturn(&candidate, &following)
    }

    fn candidate_compare(
        &mut self,
        ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, GeometryFailure> {
        let left_score = self.objective(ordinal, left)?;
        let right_score = self.objective(ordinal, right)?;
        let difference = self.charged(right_score.sub(&left_score));
        let sign = self.sign(&difference)?;
        if sign != 0 {
            return Ok(sign);
        }
        // Ветвь и обе Fraction берутся по LEFT dominant axis, включая right/0.
        let use_x = magnitude(&left.0) >= magnitude(&left.1);
        let (ln, ld) = if use_x {
            (&left.1, &left.0)
        } else {
            (&left.0, &left.1)
        };
        let (rn, rd) = if use_x {
            (&right.1, &right.0)
        } else {
            (&right.0, &right.1)
        };
        let (ln, ld) = fraction(ln.clone(), ld.clone())
            .ok_or_else(|| self.error(GeometryFailureKind::ZeroSlopeDenominator))?;
        let (rn, rd) = fraction(rn.clone(), rd.clone())
            .ok_or_else(|| self.error(GeometryFailureKind::ZeroSlopeDenominator))?;
        Ok(match (ln * &rd).cmp(&(rn * &ld)) {
            Ordering::Greater => -1,
            Ordering::Less => 1,
            Ordering::Equal => 0,
        })
    }

    fn edge_valid(
        &mut self,
        _ordinal: usize,
        left: Endpoint<'_>,
        right: Endpoint<'_>,
    ) -> Result<bool, GeometryFailure> {
        let left = self.endpoint(left);
        let right = self.endpoint(right);
        let result = self.transaction.oriented_cross(&left, &right);
        let cross = self.charged(result);
        if self.sign(&cross)? != self.expected_orientation {
            return Ok(false);
        }
        self.subturn(&left, &right)
    }
}
