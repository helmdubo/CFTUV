//! Общие owned законы Density: candidate values отделены от прежнего sign admission.
//! Контроллеры окна/Atlas/Farey остаются существующими; здесь только их arithmetic adapter.
use crate::atlas_plan::{DecimalEnvelope, EndpointKind};
use crate::density_angle_value_v1::{AngleValueErrorV1, AngleValueRegistryV1};
use crate::density_construct_v1::{
    self, ConstructErrorV1, ConstructorV1, OwnedExprV1, VectorExprV1,
};
use crate::density_expand_v1::{self, ExpandLimits, LazySignErrorV1};
use crate::density_trig_v1::{self, TrigErrorV1};
use crate::farey::{Endpoint, PrimitiveVector};
use crate::geometry::WindowLaw;
use crate::interval::{rational_cmp, rational_mul, rational_sub};
use crate::{IBig, Rational, UBig};
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LawsErrorV1 {
    Construction(ConstructErrorV1),
    Sign(LazySignErrorV1),
    Value(AngleValueErrorV1),
    Trig(TrigErrorV1),
    InvalidInput(&'static str),
    InvalidOrdinal(usize),
    ZeroDenominator,
    RationalCandidateRequired,
    ChartUnrepresentable,
    LegacyEnvelopeUnproven,
    Projection(crate::decimal_projection::ProjectionError),
}
impl From<ConstructErrorV1> for LawsErrorV1 {
    fn from(value: ConstructErrorV1) -> Self {
        Self::Construction(value)
    }
}
impl From<LazySignErrorV1> for LawsErrorV1 {
    fn from(value: LazySignErrorV1) -> Self {
        Self::Sign(value)
    }
}
impl From<AngleValueErrorV1> for LawsErrorV1 {
    fn from(value: AngleValueErrorV1) -> Self {
        Self::Value(value)
    }
}
impl From<TrigErrorV1> for LawsErrorV1 {
    fn from(value: TrigErrorV1) -> Self {
        Self::Trig(value)
    }
}
pub type LawsResultV1<T> = Result<T, LawsErrorV1>;

/// Доказательство только от прежнего lazy sign; внешний caller не может создать token.
pub struct NonzeroWitnessV1 {
    denominator: OwnedExprV1,
}
impl NonzeroWitnessV1 {
    pub(crate) fn denominator(&self) -> &OwnedExprV1 {
        &self.denominator
    }
}
pub fn prove_nonzero(
    c: &mut ConstructorV1<'_>,
    value: &OwnedExprV1,
    limits: ExpandLimits,
) -> LawsResultV1<NonzeroWitnessV1> {
    if density_expand_v1::lazy_sign(c, value, limits)?.result.sign == 0 {
        return Err(LawsErrorV1::ZeroDenominator);
    }
    Ok(NonzeroWitnessV1 {
        denominator: value.clone(),
    })
}

/// Ideal уже freshly built в этой arena; raw/incident producer не импортирует старые AST.
pub struct OwnedFanGeometryV1 {
    pub inverse_gram: [[Rational; 2]; 2],
    pub owner_orientation: i8,
    pub turn_orientation: i8,
    pub ideal: Vec<VectorExprV1>,
    pub q: usize,
    pub window_law: WindowLaw,
}
type VectorKey = [usize; 2];
type PairKey = [usize; 4];
pub struct OwnedDensityLawsV1<'c, 'g> {
    c: &'c mut ConstructorV1<'g>,
    values: AngleValueRegistryV1,
    input: OwnedFanGeometryV1,
    inverse: [[OwnedExprV1; 2]; 2],
    limits: ExpandLimits,
    dots: HashMap<PairKey, OwnedExprV1>,
    crosses: HashMap<PairKey, OwnedExprV1>,
    subturns: HashMap<(PairKey, usize), bool>,
    neighbours: HashMap<usize, (VectorExprV1, VectorExprV1)>,
    scores: HashMap<(usize, PrimitiveVector), OwnedExprV1>,
    envelopes: HashMap<usize, DecimalEnvelope>,
    shell_outcomes: Vec<(Vec<IBig>, crate::density_shell_v2::OutcomeBundle)>,
}
impl<'c, 'g> OwnedDensityLawsV1<'c, 'g> {
    pub fn new(
        c: &'c mut ConstructorV1<'g>,
        input: OwnedFanGeometryV1,
        values: AngleValueRegistryV1,
        limits: ExpandLimits,
    ) -> LawsResultV1<Self> {
        if !matches!(input.owner_orientation, -1 | 1)
            || !matches!(input.turn_orientation, -1 | 1)
            || !(2..=6).contains(&input.q)
            || input.ideal.len() < 2
        {
            return Err(LawsErrorV1::InvalidInput(
                "fan orientation/q/incident supports",
            ));
        }
        let g = &input.inverse_gram;
        let det = rational_sub(
            &rational_mul(&g[0][0], &g[1][1]),
            &rational_mul(&g[0][1], &g[1][0]),
        );
        if g[0][1] != g[1][0] || g[0][0].parts().0 <= &IBig::ZERO || det.parts().0 <= &IBig::ZERO {
            return Err(LawsErrorV1::InvalidInput(
                "dual Gram must be symmetric positive definite",
            ));
        }
        for vector in &input.ideal {
            c.raw_id(&vector[0])?;
            c.raw_id(&vector[1])?;
        }
        let inverse = [
            [c.rational(&g[0][0])?, c.rational(&g[0][1])?],
            [c.rational(&g[1][0])?, c.rational(&g[1][1])?],
        ];
        Ok(Self {
            c,
            input,
            inverse,
            values,
            limits,
            dots: HashMap::new(),
            crosses: HashMap::new(),
            subturns: HashMap::new(),
            neighbours: HashMap::new(),
            scores: HashMap::new(),
            envelopes: HashMap::new(),
            shell_outcomes: Vec::new(),
        })
    }
    pub fn constructor(&self) -> &ConstructorV1<'g> {
        self.c
    }
    pub fn ideal(&self) -> &[VectorExprV1] {
        &self.input.ideal
    }
    /// Геометрия идеала прежняя; только соседи ordinal окна зависят от закона.
    pub fn set_window_law(&mut self, law: WindowLaw) {
        if self.input.window_law != law {
            self.input.window_law = law;
            self.neighbours.clear();
        }
    }
    fn vector_key(&self, v: &VectorExprV1) -> LawsResultV1<VectorKey> {
        Ok([self.c.raw_id(&v[0])?, self.c.raw_id(&v[1])?])
    }
    fn pair_key(&self, l: &VectorExprV1, r: &VectorExprV1) -> LawsResultV1<PairKey> {
        let l = self.vector_key(l)?;
        let r = self.vector_key(r)?;
        Ok([l[0], l[1], r[0], r[1]])
    }
    fn reverse(k: PairKey) -> PairKey {
        [k[2], k[3], k[0], k[1]]
    }
    fn triple(&self, ordinal: usize) -> LawsResultV1<(VectorExprV1, VectorExprV1, VectorExprV1)> {
        if ordinal == 0 || ordinal + 1 >= self.input.ideal.len() {
            return Err(LawsErrorV1::InvalidOrdinal(ordinal));
        }
        Ok((
            self.input.ideal[ordinal - 1].clone(),
            self.input.ideal[ordinal].clone(),
            self.input.ideal[ordinal + 1].clone(),
        ))
    }
    pub fn source_sign(&mut self, value: &OwnedExprV1) -> LawsResultV1<i8> {
        Ok(density_expand_v1::lazy_sign(self.c, value, self.limits)?
            .result
            .sign)
    }
    pub fn dual_dot(&mut self, l: &VectorExprV1, r: &VectorExprV1) -> LawsResultV1<OwnedExprV1> {
        let key = self.pair_key(l, r)?;
        if let Some(value) = self.dots.get(&key) {
            return Ok(value.clone());
        }
        let value = density_construct_v1::dot(self.c, &self.inverse, l, r)?;
        self.dots.insert(key, value.clone());
        self.dots.insert(Self::reverse(key), value.clone());
        Ok(value)
    }
    pub fn cross(&mut self, l: &VectorExprV1, r: &VectorExprV1) -> LawsResultV1<OwnedExprV1> {
        let key = self.pair_key(l, r)?;
        if let Some(value) = self.crosses.get(&key) {
            return Ok(value.clone());
        }
        let value =
            density_construct_v1::oriented_cross(self.c, l, r, self.input.owner_orientation)?;
        let negative = self.c.neg(&value)?;
        self.crosses.insert(key, value.clone());
        self.crosses.insert(Self::reverse(key), negative);
        Ok(value)
    }
    pub fn quarter_turn(
        &mut self,
        v: &VectorExprV1,
        orientation: i8,
    ) -> LawsResultV1<VectorExprV1> {
        self.vector_key(v)?;
        if !matches!(orientation, -1 | 1) {
            return Err(LawsErrorV1::InvalidInput("quarter orientation"));
        }
        let mut components = Vec::new();
        for row in &self.inverse {
            let a = self.c.mul(&[row[0].clone(), v[0].clone()])?;
            let b = self.c.mul(&[row[1].clone(), v[1].clone()])?;
            components.push(self.c.add(&[a, b])?);
        }
        let g = &self.input.inverse_gram;
        let determinant = rational_sub(
            &rational_mul(&g[0][0], &g[1][1]),
            &rational_mul(&g[0][1], &g[1][0]),
        );
        let d = self.c.rational(&determinant)?;
        let root = self.c.sqrt(&d)?;
        let factor = self.c.rational(&Rational::integer(IBig::from(
            orientation * self.input.owner_orientation,
        )))?;
        let scale = self.c.div(&factor, &root)?;
        let x = self.c.mul(&[scale.clone(), components[1].clone()])?;
        let x = self.c.neg(&x)?;
        let y = self.c.mul(&[scale, components[0].clone()])?;
        Ok([x, y])
    }
    fn rotate_components(
        &mut self,
        v: &VectorExprV1,
        orientation: i8,
        trig: [OwnedExprV1; 2],
    ) -> LawsResultV1<VectorExprV1> {
        let p = self.quarter_turn(v, orientation)?;
        let mut out = Vec::new();
        for axis in 0..2 {
            let a = self.c.mul(&[trig[0].clone(), v[axis].clone()])?;
            let b = self.c.mul(&[trig[1].clone(), p[axis].clone()])?;
            out.push(self.c.add(&[a, b])?);
        }
        Ok([out.remove(0), out.remove(0)])
    }
    pub fn rotate_q(
        &mut self,
        v: &VectorExprV1,
        orientation: i8,
        q: usize,
    ) -> LawsResultV1<VectorExprV1> {
        if !(2..=6).contains(&q) {
            return Err(LawsErrorV1::InvalidInput("unsupported Density q"));
        }
        let trig = density_trig_v1::owned_exact_q_trig(self.c, q as u8)?;
        self.rotate_components(v, orientation, trig)
    }
    fn residual_sign(
        &mut self,
        dot: &OwnedExprV1,
        l: &OwnedExprV1,
        r: &OwnedExprV1,
        q: usize,
    ) -> LawsResultV1<i8> {
        if let Some(sign) = self.c.filtered_residual_sign(dot, l, r, q)? {
            return Ok(sign);
        }
        let norms = self.c.mul(&[l.clone(), r.clone()])?;
        let dot2 = self.c.mul(&[dot.clone(), dot.clone()])?;
        let first = self.c.scaled(
            &dot2,
            &Rational::integer(IBig::from(if q == 5 {
                8
            } else if q == 4 {
                2
            } else {
                4
            })),
        )?;
        let second = if q == 5 {
            let five = self.c.rational(&Rational::integer(IBig::from(5)))?;
            let root = self.c.sqrt(&five)?;
            let three = self.c.rational(&Rational::integer(IBig::from(3)))?;
            let scale = self.c.add(&[three, root])?;
            self.c.mul(&[scale, norms])?
        } else {
            self.c.scaled(
                &norms,
                &Rational::integer(IBig::from(if q == 6 { 3 } else { 1 })),
            )?
        };
        let residual = self.c.sub(&first, &second)?;
        self.source_sign(&residual)
    }
    pub fn subturn(&mut self, l: &VectorExprV1, r: &VectorExprV1, q: usize) -> LawsResultV1<bool> {
        if !(2..=6).contains(&q) {
            return Err(LawsErrorV1::InvalidInput("unsupported Density q"));
        }
        let key = self.pair_key(l, r)?;
        if let Some(value) = self.subturns.get(&(key, q)) {
            return Ok(*value);
        }
        let dot = self.dual_dot(l, r)?;
        let sign = self.source_sign(&dot)?;
        let result = if q == 2 {
            sign >= 0
        } else if sign <= 0 {
            false
        } else {
            let nl = self.dual_dot(l, l)?;
            let nr = self.dual_dot(r, r)?;
            self.residual_sign(&dot, &nl, &nr, q)? >= 0
        };
        self.subturns.insert((key, q), result);
        self.subturns.insert((Self::reverse(key), q), result);
        Ok(result)
    }
    pub fn subturn_boundary(
        &mut self,
        l: &VectorExprV1,
        r: &VectorExprV1,
        q: usize,
    ) -> LawsResultV1<bool> {
        if !(2..=6).contains(&q) {
            return Err(LawsErrorV1::InvalidInput("unsupported Density q"));
        }
        let dot = self.dual_dot(l, r)?;
        if self.source_sign(&dot)? < 0 {
            return Ok(false);
        }
        // Эти norm calls стоят до q2 ветки в исходнике; shortcut не переносится выше.
        let nl = self.dual_dot(l, l)?;
        let nr = self.dual_dot(r, r)?;
        if q == 2 {
            Ok(self.source_sign(&dot)? == 0)
        } else {
            Ok(self.residual_sign(&dot, &nl, &nr, q)? == 0)
        }
    }
    pub fn inside_ordinal(
        &mut self,
        v: &VectorExprV1,
        p: &VectorExprV1,
        center: &VectorExprV1,
        f: &VectorExprV1,
    ) -> LawsResultV1<bool> {
        let previous = [
            self.c.sub(&center[0], &p[0])?,
            self.c.sub(&center[1], &p[1])?,
        ];
        let following = [
            self.c.sub(&center[0], &f[0])?,
            self.c.sub(&center[1], &f[1])?,
        ];
        let dot = self.dual_dot(v, &previous)?;
        if self.source_sign(&dot)? <= 0 {
            return Ok(false);
        }
        let dot = self.dual_dot(v, &following)?;
        Ok(self.source_sign(&dot)? > 0)
    }
    fn neighbour_in_band(
        &mut self,
        center: &VectorExprV1,
        neighbour: &VectorExprV1,
    ) -> LawsResultV1<bool> {
        let dot = self.dual_dot(center, neighbour)?;
        if self.source_sign(&dot)? <= 0 {
            return Ok(false);
        }
        let l = self.dual_dot(center, center)?;
        let r = self.dual_dot(neighbour, neighbour)?;
        let norms = self.c.mul(&[l, r])?;
        let cosine = Rational::new(IBig::from(1624), UBig::from(1625u32)).unwrap();
        let second = self.c.scaled(&norms, &rational_mul(&cosine, &cosine))?;
        let first = self.c.mul(&[dot.clone(), dot])?;
        let residual = self.c.sub(&first, &second)?;
        Ok(self.source_sign(&residual)? >= 0)
    }
    pub fn window_neighbours(
        &mut self,
        ordinal: usize,
    ) -> LawsResultV1<(VectorExprV1, VectorExprV1)> {
        let (p, center, f) = self.triple(ordinal)?;
        if self.input.window_law == WindowLaw::Voronoi {
            return Ok((p, f));
        }
        if let Some(pair) = self.neighbours.get(&ordinal) {
            return Ok(pair.clone());
        }
        let mut result = Vec::new();
        for (real, sign) in [(p, -1), (f, 1)] {
            result.push(if self.neighbour_in_band(&center, &real)? {
                real
            } else {
                let cosine = self
                    .c
                    .rational(&Rational::new(IBig::from(1624), UBig::from(1625u32)).unwrap())?;
                let sine = self.c.rational(
                    &Rational::new(IBig::from(57 * sign), UBig::from(1625u32)).unwrap(),
                )?;
                self.rotate_components(&center, self.input.turn_orientation, [cosine, sine])?
            });
        }
        let pair = (result.remove(0), result.remove(0));
        self.neighbours.insert(ordinal, pair.clone());
        Ok(pair)
    }
    pub fn local_candidate(
        &mut self,
        ordinal: usize,
        v: &VectorExprV1,
        q: usize,
    ) -> LawsResultV1<bool> {
        let (realp, center, realf) = self.triple(ordinal)?;
        let (p, f) = self.window_neighbours(ordinal)?;
        Ok(self.inside_ordinal(v, &p, &center, &f)?
            && self.subturn(&realp, v, q)?
            && self.subturn(v, &realf, q)?)
    }
    pub fn boundary_vectors(
        &mut self,
        ordinal: usize,
        orientation: i8,
        q: usize,
    ) -> LawsResultV1<Vec<VectorExprV1>> {
        let (p, _, f) = self.triple(ordinal)?;
        Ok(vec![
            self.rotate_q(&p, orientation, q)?,
            self.rotate_q(&f, -orientation, q)?,
        ])
    }
    pub fn primitive_vector(&mut self, v: &PrimitiveVector) -> LawsResultV1<VectorExprV1> {
        Ok([
            self.c.rational(&Rational::integer(v.0.clone()))?,
            self.c.rational(&Rational::integer(v.1.clone()))?,
        ])
    }
    pub fn slope(&mut self, v: &VectorExprV1, use_x: bool) -> LawsResultV1<OwnedExprV1> {
        self.vector_key(v)?;
        let denominator = &v[usize::from(!use_x)];
        let numerator = &v[usize::from(use_x)];
        let proof = prove_nonzero(self.c, denominator, self.limits)?;
        Ok(self.c.divide_with_nonzero_witness(numerator, &proof)?)
    }
    pub fn decimal_envelope(&mut self, v: &OwnedExprV1) -> LawsResultV1<DecimalEnvelope> {
        let id = self.c.raw_id(v)?;
        if let Some(envelope) = self.envelopes.get(&id) {
            let envelope = envelope.clone();
            self.record_shell_outcome(v)?;
            return Ok(envelope);
        }
        // A1/A2 меняют value shell только после прежнего literal interval admission.
        match self.c.interval(
            v,
            crate::legacy_mpf::Precision::new(160).expect("source precision"),
        ) {
            Ok(_) => {}
            Err(ConstructErrorV1::Arena(
                crate::density_expression::ExpressionError::UnsupportedExpression(_)
                | crate::density_expression::ExpressionError::Legacy(
                    crate::legacy_mpf::LegacyError::ZeroDivision
                    | crate::legacy_mpf::LegacyError::ComplexResult,
                ),
            )) => return Err(LawsErrorV1::LegacyEnvelopeUnproven),
            Err(error) => return Err(error.into()),
        }
        let enclosure = self.values.enclosure(self.c, v)?;
        let shell = crate::density_shell_v2::integer_shell(&enclosure);
        let scale: UBig = "1000000000000000000000000000".parse().unwrap();
        let envelope = DecimalEnvelope {
            lower: Rational::new(shell.lower_units, scale.clone()).unwrap(),
            upper: Rational::new(shell.upper_units, scale.clone()).unwrap(),
            lower_kind: EndpointKind::Closed,
            upper_kind: EndpointKind::Closed,
            absolute_error_bound: Rational::new(shell.width_units, scale).unwrap(),
            quantum: Some(crate::atlas_plan::DecimalQuantum::uniform(-27)),
        };
        self.envelopes.insert(id, envelope.clone());
        self.record_shell_outcome(v)?;
        Ok(envelope)
    }
    fn record_shell_outcome(&mut self, value: &OwnedExprV1) -> LawsResultV1<()> {
        if let Some(value) = self.c.numeric_value(value)? {
            self.shell_outcomes.push((
                value.value().wire(),
                crate::density_shell_v2::outcome_bundle(value.value()),
            ));
        }
        Ok(())
    }
    pub fn take_shell_outcomes(
        &mut self,
    ) -> Vec<(Vec<IBig>, crate::density_shell_v2::OutcomeBundle)> {
        std::mem::take(&mut self.shell_outcomes)
    }
    pub fn take_values(&mut self) -> crate::density_angle_value_v1::AngleValueRegistryV1 {
        std::mem::take(&mut self.values)
    }
    pub fn objective(&mut self, ordinal: usize, v: &PrimitiveVector) -> LawsResultV1<OwnedExprV1> {
        self.triple(ordinal)?;
        let key = (ordinal, v.clone());
        if let Some(score) = self.scores.get(&key) {
            return Ok(score.clone());
        }
        let candidate = self.primitive_vector(v)?;
        let ideal = self.input.ideal[ordinal].clone();
        let dot = self.dual_dot(&candidate, &ideal)?;
        let norm = self.dual_dot(&candidate, &candidate)?;
        let square = self.c.mul(&[dot.clone(), dot])?;
        let score = self.c.div(&square, &norm)?;
        self.scores.insert(key, score.clone());
        Ok(score)
    }
    fn rational(&self, v: &OwnedExprV1) -> LawsResultV1<Rational> {
        let (n, d) = self
            .c
            .numeric_value(v)?
            .and_then(|v| v.value().as_rational())
            .ok_or(LawsErrorV1::RationalCandidateRequired)?;
        Ok(Rational::new(n, d).unwrap())
    }
    pub fn candidate_compare(
        &mut self,
        ordinal: usize,
        l: &PrimitiveVector,
        r: &PrimitiveVector,
    ) -> LawsResultV1<i8> {
        let ls = self.objective(ordinal, l)?;
        let le = self.decimal_envelope(&ls)?;
        let rs = self.objective(ordinal, r)?;
        let re = self.decimal_envelope(&rs)?;
        if rational_cmp(&le.lower, &re.upper).is_gt() {
            return Ok(-1);
        }
        if rational_cmp(&re.lower, &le.upper).is_gt() {
            return Ok(1);
        }
        let difference = self.c.sub(&rs, &ls)?;
        let sign = self.source_sign(&difference)?;
        if sign != 0 {
            return Ok(sign);
        }
        let use_x = cftuv_radical::arith::magnitude(&l.0) >= cftuv_radical::arith::magnitude(&l.1);
        let slope = |v: &PrimitiveVector| {
            let (n, d) = if use_x { (&v.1, &v.0) } else { (&v.0, &v.1) };
            if d.is_zero() {
                None
            } else if d < &IBig::ZERO {
                Some(Rational::new(-n.clone(), (-d.clone()).try_into().unwrap()).unwrap())
            } else {
                Some(Rational::new(n.clone(), d.clone().try_into().unwrap()).unwrap())
            }
        };
        let l = slope(l).ok_or(LawsErrorV1::ZeroDenominator)?;
        let r = slope(r).ok_or(LawsErrorV1::ZeroDenominator)?;
        Ok(if rational_cmp(&l, &r).is_gt() {
            -1
        } else if rational_cmp(&l, &r).is_lt() {
            1
        } else {
            0
        })
    }
    fn endpoint_vector(&mut self, endpoint: Endpoint<'_>) -> LawsResultV1<VectorExprV1> {
        match endpoint {
            Endpoint::Incoming => Ok(self.input.ideal[0].clone()),
            Endpoint::Outgoing => Ok(self.input.ideal.last().unwrap().clone()),
            Endpoint::Primitive(v) => self.primitive_vector(v),
        }
    }
    fn slope_vector(&mut self, q: &Rational, use_x: bool, sign: i8) -> LawsResultV1<VectorExprV1> {
        if !matches!(sign, -1 | 1) {
            return Err(LawsErrorV1::InvalidInput("chart denominator sign"));
        }
        let (n, d) = q.parts();
        let numerator = n * IBig::from(sign);
        let denominator = IBig::from(d.clone()) * IBig::from(sign);
        self.primitive_vector(&if use_x {
            PrimitiveVector(denominator, numerator)
        } else {
            PrimitiveVector(numerator, denominator)
        })
    }
    fn decimal_width(
        &mut self,
        lower: &DecimalEnvelope,
        upper: &DecimalEnvelope,
    ) -> LawsResultV1<Rational> {
        let scale: IBig = "1000000000000000000000000000".parse().unwrap();
        let units = |q: &Rational| -> LawsResultV1<IBig> {
            let (n, d) = q.parts();
            let scaled = n * &scale;
            let d = IBig::from(d.clone());
            if &scaled % &d != IBig::ZERO {
                return Err(LawsErrorV1::InvalidInput(
                    "Decimal endpoint outside quantum -27",
                ));
            }
            Ok(scaled / d)
        };
        let l = units(&lower.upper)?;
        let u = units(&upper.lower)?;
        let (width, _) = crate::decimal_projection::width_from_units(&l, &u, 28)
            .map_err(LawsErrorV1::Projection)?;
        let exponent = width.exponent();
        let magnitude = usize::try_from(exponent.unsigned_abs())
            .map_err(|_| LawsErrorV1::InvalidInput("Decimal width exponent"))?;
        let power: UBig = format!("1{}", "0".repeat(magnitude)).parse().unwrap();
        Ok(if exponent < 0 {
            Rational::new(width.coefficient().clone(), power).unwrap()
        } else {
            Rational::integer(width.coefficient() * IBig::from(power))
        })
    }
    pub fn edge_valid(
        &mut self,
        _ordinal: usize,
        l: Endpoint<'_>,
        r: Endpoint<'_>,
    ) -> LawsResultV1<bool> {
        let l = self.endpoint_vector(l)?;
        let r = self.endpoint_vector(r)?;
        let cross = self.cross(&l, &r)?;
        Ok(self.source_sign(&cross)? == self.input.turn_orientation
            && self.subturn(&l, &r, self.input.q)?)
    }
}

// Совпадающие scalar/vector законы одного source adapter, без второго controller.
macro_rules! fan_scalar_methods {
    () => {
        fn denominator(&mut self, v: &VectorExprV1, use_x: bool) -> LawsResultV1<OwnedExprV1> {
            self.vector_key(v)?;
            Ok(v[usize::from(!use_x)].clone())
        }
        fn sign(&mut self, v: &OwnedExprV1) -> LawsResultV1<i8> {
            self.source_sign(v)
        }
        fn slope(&mut self, v: &VectorExprV1, use_x: bool) -> LawsResultV1<OwnedExprV1> {
            OwnedDensityLawsV1::slope(self, v, use_x)
        }
        fn decimal_envelope(&mut self, v: &OwnedExprV1) -> LawsResultV1<DecimalEnvelope> {
            OwnedDensityLawsV1::decimal_envelope(self, v)
        }
        fn local_candidate(&mut self, o: usize, v: &VectorExprV1, q: usize) -> LawsResultV1<bool> {
            OwnedDensityLawsV1::local_candidate(self, o, v, q)
        }
        fn vector_from_slope(
            &mut self,
            v: &Rational,
            use_x: bool,
            sign: i8,
        ) -> LawsResultV1<VectorExprV1> {
            self.slope_vector(v, use_x, sign)
        }
        fn subturn_boundary_vectors(
            &mut self,
            o: usize,
            orientation: i8,
            q: usize,
        ) -> LawsResultV1<Vec<VectorExprV1>> {
            self.boundary_vectors(o, orientation, q)
        }
    };
}
impl crate::fan_prepare::FanPrepareLaws for OwnedDensityLawsV1<'_, '_> {
    type Vector = VectorExprV1;
    type Scalar = OwnedExprV1;
    type Error = LawsErrorV1;
    fan_scalar_methods!();
    fn candidate_vector(
        &mut self,
        v: &Rational,
        use_x: bool,
        sign: i8,
    ) -> LawsResultV1<VectorExprV1> {
        self.slope_vector(v, use_x, sign)
    }
    fn window_neighbours(&mut self, o: usize) -> LawsResultV1<(VectorExprV1, VectorExprV1)> {
        OwnedDensityLawsV1::window_neighbours(self, o)
    }
    fn inside_ordinal(
        &mut self,
        v: &VectorExprV1,
        p: &VectorExprV1,
        c: &VectorExprV1,
        f: &VectorExprV1,
    ) -> LawsResultV1<bool> {
        OwnedDensityLawsV1::inside_ordinal(self, v, p, c, f)
    }
    fn oriented_cross_sign(&mut self, l: &VectorExprV1, r: &VectorExprV1) -> LawsResultV1<i8> {
        let cross = self.cross(l, r)?;
        self.source_sign(&cross)
    }
    fn subturn(&mut self, l: &VectorExprV1, r: &VectorExprV1, q: usize) -> LawsResultV1<bool> {
        OwnedDensityLawsV1::subturn(self, l, r, q)
    }
    fn subturn_boundary(
        &mut self,
        l: &VectorExprV1,
        r: &VectorExprV1,
        q: usize,
    ) -> LawsResultV1<bool> {
        OwnedDensityLawsV1::subturn_boundary(self, l, r, q)
    }
    fn is_reference_geometry_error(e: &LawsErrorV1) -> bool {
        matches!(
            e,
            LawsErrorV1::Sign(LazySignErrorV1::Sign(
                crate::density_construct_v1::OwnedSignErrorV1::Sign(
                    crate::density_sign::SignFailure::Undecidable
                )
            ))
        )
    }
}
impl crate::atlas_plan::SegmentPlanLaws for OwnedDensityLawsV1<'_, '_> {
    type Vector = VectorExprV1;
    type Scalar = OwnedExprV1;
    type Error = LawsErrorV1;
    fan_scalar_methods!();
    fn local_transition(&mut self, o: usize, v: &PrimitiveVector, q: usize) -> LawsResultV1<bool> {
        let v = self.primitive_vector(v)?;
        OwnedDensityLawsV1::local_candidate(self, o, &v, q)
    }
}
impl crate::farey::FareyPredicates for OwnedDensityLawsV1<'_, '_> {
    type Error = LawsErrorV1;
    fn local_candidate(&mut self, o: usize, v: &PrimitiveVector) -> LawsResultV1<bool> {
        let v = self.primitive_vector(v)?;
        OwnedDensityLawsV1::local_candidate(self, o, &v, self.input.q)
    }
    fn candidate_compare(
        &mut self,
        o: usize,
        l: &PrimitiveVector,
        r: &PrimitiveVector,
    ) -> LawsResultV1<i8> {
        OwnedDensityLawsV1::candidate_compare(self, o, l, r)
    }
    fn edge_valid(&mut self, o: usize, l: Endpoint<'_>, r: Endpoint<'_>) -> LawsResultV1<bool> {
        OwnedDensityLawsV1::edge_valid(self, o, l, r)
    }
}
impl crate::window_build::WindowBuildLaws for OwnedDensityLawsV1<'_, '_> {
    type Vector = VectorExprV1;
    type Scalar = OwnedExprV1;
    type Error = LawsErrorV1;
    fn window_neighbours(&mut self, o: usize) -> LawsResultV1<(VectorExprV1, VectorExprV1)> {
        OwnedDensityLawsV1::window_neighbours(self, o)
    }
    fn coordinates(&mut self, v: &VectorExprV1) -> LawsResultV1<(OwnedExprV1, OwnedExprV1)> {
        self.vector_key(v)?;
        Ok((v[0].clone(), v[1].clone()))
    }
    fn add(&mut self, l: &OwnedExprV1, r: &OwnedExprV1) -> LawsResultV1<OwnedExprV1> {
        Ok(self.c.add(&[l.clone(), r.clone()])?)
    }
    fn sub(&mut self, l: &OwnedExprV1, r: &OwnedExprV1) -> LawsResultV1<OwnedExprV1> {
        Ok(self.c.sub(l, r)?)
    }
    fn mul(&mut self, l: &OwnedExprV1, r: &OwnedExprV1) -> LawsResultV1<OwnedExprV1> {
        Ok(self.c.mul(&[l.clone(), r.clone()])?)
    }
    fn sign(&mut self, v: &OwnedExprV1) -> LawsResultV1<i8> {
        self.source_sign(v)
    }
    fn vector(&mut self, x: OwnedExprV1, y: OwnedExprV1) -> LawsResultV1<VectorExprV1> {
        self.c.raw_id(&x)?;
        self.c.raw_id(&y)?;
        Ok([x, y])
    }
    fn integer_vector(&mut self, x: &IBig, y: &IBig) -> LawsResultV1<VectorExprV1> {
        self.primitive_vector(&PrimitiveVector(x.clone(), y.clone()))
    }
    fn slope(&mut self, v: &VectorExprV1, use_x: bool) -> LawsResultV1<OwnedExprV1> {
        OwnedDensityLawsV1::slope(self, v, use_x)
    }
    fn decimal_envelope(&mut self, v: &OwnedExprV1) -> LawsResultV1<DecimalEnvelope> {
        OwnedDensityLawsV1::decimal_envelope(self, v)
    }
    fn equals_integer(&mut self, v: &OwnedExprV1, integer: i8) -> LawsResultV1<bool> {
        Ok(self
            .c
            .numeric_value(v)?
            .and_then(|v| v.value().as_rational())
            == Some((IBig::from(integer), UBig::ONE)))
    }
    fn integer_value(&mut self, v: &OwnedExprV1) -> LawsResultV1<IBig> {
        let q = self.rational(v)?;
        let (n, d) = q.parts();
        if d != &UBig::ONE {
            Err(LawsErrorV1::RationalCandidateRequired)
        } else {
            Ok(n.clone())
        }
    }
    fn rational_lt_scalar(&mut self, q: &Rational, v: &OwnedExprV1) -> LawsResultV1<bool> {
        let q = self.c.rational(q)?;
        let d = self.c.sub(v, &q)?;
        Ok(self.source_sign(&d)? > 0)
    }
    fn scalar_lt_rational(&mut self, v: &OwnedExprV1, q: &Rational) -> LawsResultV1<bool> {
        let q = self.c.rational(q)?;
        let d = self.c.sub(&q, v)?;
        Ok(self.source_sign(&d)? > 0)
    }
    fn is_chart_unrepresentable(e: &LawsErrorV1) -> bool {
        matches!(e, LawsErrorV1::ChartUnrepresentable)
    }
}
impl crate::legacy_binding::LegacyBindingLaws for OwnedDensityLawsV1<'_, '_> {
    type Vector = VectorExprV1;
    type Score = OwnedExprV1;
    type Error = LawsErrorV1;
    fn vector(&mut self, v: &PrimitiveVector) -> LawsResultV1<VectorExprV1> {
        self.primitive_vector(v)
    }
    fn subturn(&mut self, l: &VectorExprV1, r: &VectorExprV1, q: usize) -> LawsResultV1<bool> {
        OwnedDensityLawsV1::subturn(self, l, r, q)
    }
    fn objective(&mut self, o: usize, v: &PrimitiveVector) -> LawsResultV1<OwnedExprV1> {
        OwnedDensityLawsV1::objective(self, o, v)
    }
    fn score_difference_sign(&mut self, l: &OwnedExprV1, r: &OwnedExprV1) -> LawsResultV1<i8> {
        let d = self.c.sub(l, r)?;
        self.source_sign(&d)
    }
    fn oriented_cross_sign(&mut self, l: &VectorExprV1, r: &VectorExprV1) -> LawsResultV1<i8> {
        let cross = self.cross(l, r)?;
        self.source_sign(&cross)
    }
    fn window_triple(
        &mut self,
        o: usize,
    ) -> LawsResultV1<(VectorExprV1, VectorExprV1, VectorExprV1)> {
        let (_, c, _) = self.triple(o)?;
        let (p, f) = self.window_neighbours(o)?;
        Ok((p, c, f))
    }
    fn inside_ordinal(
        &mut self,
        v: &VectorExprV1,
        t: &(VectorExprV1, VectorExprV1, VectorExprV1),
    ) -> LawsResultV1<bool> {
        OwnedDensityLawsV1::inside_ordinal(self, v, &t.0, &t.1, &t.2)
    }
    fn slope(&mut self, v: &VectorExprV1, use_x: bool) -> LawsResultV1<Rational> {
        let slope = OwnedDensityLawsV1::slope(self, v, use_x)?;
        self.rational(&slope)
    }
    fn certified_width(
        &mut self,
        l: &DecimalEnvelope,
        u: &DecimalEnvelope,
    ) -> LawsResultV1<Rational> {
        self.decimal_width(l, u)
    }
}
