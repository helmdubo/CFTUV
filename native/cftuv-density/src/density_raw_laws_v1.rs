//! Стабильный raw geometry/turn/count leaf. Одна arena и invocation-local metric cache.
//! Oracle: density_owned_laws_v1.py; не читает prepared AST/готовые нормали/решения.
use crate::density_construct_v1::{self, ConstructorV1, MatrixExprV1, OwnedExprV1, VectorExprV1};
use crate::density_expand_v1::{self, ExpandLimits};
use crate::density_laws_v1::LawsErrorV1;
use crate::density_raw_types_v1::*;
use crate::interval::{rational_add, rational_mul, rational_sub};
use crate::{IBig, Rational, UBig};
use std::collections::HashMap;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RawLawsFailureV1 {
    Contract(RawContractFailureV1),
    Geometry(RawGeometryRefusalV1),
    Arithmetic(LawsErrorV1),
    NonPositiveCovectorNorm,
    CanonicalRestorationInvalid { message: String },
}
impl From<crate::density_construct_v1::ConstructErrorV1> for RawLawsFailureV1 {
    fn from(error: crate::density_construct_v1::ConstructErrorV1) -> Self {
        Self::Arithmetic(LawsErrorV1::Construction(error))
    }
}
pub type RawLawsResultV1<T> = Result<T, RawLawsFailureV1>;
#[derive(Clone, Debug)]
pub struct ProducedRawMetricV1 {
    pub declaration: RawDensityMetricV1,
    pub gram: MatrixExprV1,
    pub inverse_gram: MatrixExprV1,
}
#[derive(Clone, Debug)]
pub struct ProducedRawGeometryV1 {
    pub metric: ProducedRawMetricV1,
    pub incoming: VectorExprV1,
    pub outgoing: VectorExprV1,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TurnWitnessV1 {
    pub dot_sign: i8,
    pub cosine_squared: Rational,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum RationalDirectionV1 {
    Rational(crate::farey::PrimitiveVector),
    Irrational,
    Undetermined,
}
pub struct ProducedRawFanV1 {
    pub ideal: Vec<VectorExprV1>,
    pub values: crate::density_angle_value_v1::AngleValueRegistryV1,
}

pub struct RawDensityLawsV1<'c, 'g> {
    c: &'c mut ConstructorV1<'g>,
    limits: ExpandLimits,
    metrics: HashMap<(RawDensityMetricV1, bool), ProducedRawMetricV1>,
    values: crate::density_angle_value_v1::AngleValueRegistryV1,
}
impl<'c, 'g> RawDensityLawsV1<'c, 'g> {
    pub fn new(c: &'c mut ConstructorV1<'g>, limits: ExpandLimits) -> Self {
        Self {
            c,
            limits,
            metrics: HashMap::new(),
            values: crate::density_angle_value_v1::AngleValueRegistryV1::new(),
        }
    }
    pub fn constructor(&mut self) -> &mut ConstructorV1<'g> {
        self.c
    }
    pub fn metric_cache_len(&self) -> usize {
        self.metrics.len()
    }
    pub fn registry(&self) -> crate::density_angle_value_v1::AngleValueRegistryV1 {
        self.values.clone()
    }
    pub fn replace_registry(
        &mut self,
        values: crate::density_angle_value_v1::AngleValueRegistryV1,
    ) {
        self.values = values;
    }
    pub fn sign(&mut self, value: &OwnedExprV1) -> RawLawsResultV1<i8> {
        density_expand_v1::lazy_sign(self.c, value, self.limits)
            .map(|v| v.result.sign)
            .map_err(|e| RawLawsFailureV1::Arithmetic(LawsErrorV1::Sign(e)))
    }
    pub fn fraction(&self, value: &OwnedExprV1) -> RawLawsResultV1<Rational> {
        let (n, d) = self
            .c
            .numeric_value(value)?
            .and_then(|v| v.value().as_rational())
            .ok_or(RawLawsFailureV1::Arithmetic(
                LawsErrorV1::RationalCandidateRequired,
            ))?;
        Ok(Rational::new(n, d).unwrap())
    }
    pub fn vector(&mut self, x: &Rational, y: &Rational) -> RawLawsResultV1<VectorExprV1> {
        Ok([self.c.rational(x)?, self.c.rational(y)?])
    }
    fn matrix(&mut self, values: &[Rational; 3]) -> RawLawsResultV1<MatrixExprV1> {
        let a = self.c.rational(&values[0])?;
        let b = self.c.rational(&values[1])?;
        let d = self.c.rational(&values[2])?;
        Ok([[a, b.clone()], [b, d]])
    }
    fn validate_metric(raw: &RawDensityMetricV1) -> RawLawsResultV1<()> {
        if !matches!(raw.owner_orientation_sign, -1 | 1) {
            return Err(RawLawsFailureV1::Contract(
                RawContractFailureV1::InvalidOwnerOrientation,
            ));
        }
        let [g00, g01, g11] = &raw.gram;
        let [i00, i01, i11] = &raw.inverse_gram;
        let det = rational_sub(&rational_mul(g00, g11), &rational_mul(g01, g01));
        let dot = |a, b, c, d| rational_add(&rational_mul(a, b), &rational_mul(c, d));
        let one = Rational::integer(IBig::ONE);
        let zero = Rational::integer(IBig::ZERO);
        if g00.parts().0 <= &IBig::ZERO
            || det.parts().0 <= &IBig::ZERO
            || dot(g00, i00, g01, i01) != one
            || dot(g00, i01, g01, i11) != zero
            || dot(g01, i00, g11, i01) != zero
            || dot(g01, i01, g11, i11) != one
        {
            return Err(RawLawsFailureV1::Contract(
                RawContractFailureV1::InvalidGramFacts,
            ));
        }
        Ok(())
    }
    fn validate_incident(raw: &RawDensityIncidentV1) -> RawLawsResultV1<()> {
        let cycles = &raw.owner_face_cycles;
        let error = if cycles.iter().any(|c| c.len() < 3) {
            Some(RawContractFailureV1::FaceCycleCardinality)
        } else if !raw.owner_face_ids.is_empty() && raw.owner_face_ids.len() != cycles.len() {
            Some(RawContractFailureV1::FaceIdCardinality)
        } else if !raw.owner_face_vertex_id_cycles.is_empty()
            && (raw.owner_face_vertex_id_cycles.len() != cycles.len()
                || raw
                    .owner_face_vertex_id_cycles
                    .iter()
                    .zip(cycles)
                    .any(|(i, c)| i.len() != c.len()))
        {
            Some(RawContractFailureV1::FaceVertexIdCardinality)
        } else if !raw.owner_face_edge_ordinals.is_empty()
            && (raw.owner_face_edge_ordinals.len() != cycles.len()
                || raw
                    .owner_face_edge_ordinals
                    .iter()
                    .zip(cycles)
                    .any(|(i, c)| *i >= c.len()))
        {
            Some(RawContractFailureV1::FaceEdgeOrdinal)
        } else {
            None
        };
        error.map_or(Ok(()), |e| Err(RawLawsFailureV1::Contract(e)))
    }
    pub fn raw_geometry(
        &mut self,
        raw: &RawDensityGeometryV1,
        use_direction_declarations: bool,
    ) -> RawLawsResultV1<ProducedRawGeometryV1> {
        Self::validate_metric(&raw.metric)?;
        Self::validate_incident(&raw.incoming)?;
        Self::validate_incident(&raw.outgoing)?;
        let key = (raw.metric.clone(), use_direction_declarations);
        let metric = if let Some(metric) = self.metrics.get(&key) {
            metric.clone()
        } else {
            let metric = ProducedRawMetricV1 {
                declaration: raw.metric.clone(),
                gram: self.matrix(&raw.metric.gram)?,
                inverse_gram: self.matrix(&raw.metric.inverse_gram)?,
            };
            self.metrics.insert(key, metric.clone());
            metric
        };
        let incoming =
            self.raw_incident_normal(&metric, &raw.incoming, use_direction_declarations)?;
        let outgoing =
            self.raw_incident_normal(&metric, &raw.outgoing, use_direction_declarations)?;
        Ok(ProducedRawGeometryV1 {
            metric,
            incoming,
            outgoing,
        })
    }
    pub fn dot_g(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
    ) -> RawLawsResultV1<OwnedExprV1> {
        Ok(density_construct_v1::dot(self.c, &m.gram, left, right)?)
    }
    pub fn dual_dot(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
    ) -> RawLawsResultV1<OwnedExprV1> {
        Ok(density_construct_v1::dot(
            self.c,
            &m.inverse_gram,
            left,
            right,
        )?)
    }
    pub fn oriented_cross(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
    ) -> RawLawsResultV1<OwnedExprV1> {
        Ok(density_construct_v1::oriented_cross(
            self.c,
            left,
            right,
            m.declaration.owner_orientation_sign,
        )?)
    }
    pub fn unit_g(
        &mut self,
        m: &ProducedRawMetricV1,
        vector: &VectorExprV1,
    ) -> RawLawsResultV1<VectorExprV1> {
        let norm = self.dot_g(m, vector, vector)?;
        self.fraction(&norm)?;
        let root = self.c.sqrt(&norm)?;
        Ok([
            self.c.div(&vector[0], &root)?,
            self.c.div(&vector[1], &root)?,
        ])
    }
    pub fn unit_covector(
        &mut self,
        m: &ProducedRawMetricV1,
        vector: &VectorExprV1,
    ) -> RawLawsResultV1<VectorExprV1> {
        let norm = self.dual_dot(m, vector, vector)?;
        if self.fraction(&norm)?.parts().0 <= &IBig::ZERO {
            return Err(RawLawsFailureV1::NonPositiveCovectorNorm);
        }
        let root = self.c.sqrt(&norm)?;
        Ok([
            self.c.div(&vector[0], &root)?,
            self.c.div(&vector[1], &root)?,
        ])
    }
    fn apply_matrix(
        &mut self,
        matrix: &MatrixExprV1,
        vector: &VectorExprV1,
    ) -> RawLawsResultV1<VectorExprV1> {
        let mut rows = Vec::new();
        for row in matrix {
            let x = self.c.mul(&[row[0].clone(), vector[0].clone()])?;
            let y = self.c.mul(&[row[1].clone(), vector[1].clone()])?;
            rows.push(self.c.add(&[x, y])?);
        }
        Ok([rows.remove(0), rows.remove(0)])
    }
    pub fn covector(
        &mut self,
        m: &ProducedRawMetricV1,
        normal: &VectorExprV1,
    ) -> RawLawsResultV1<VectorExprV1> {
        self.apply_matrix(&m.gram, normal)
    }
    pub fn primal(
        &mut self,
        m: &ProducedRawMetricV1,
        covector: &VectorExprV1,
    ) -> RawLawsResultV1<VectorExprV1> {
        self.apply_matrix(&m.inverse_gram, covector)
    }
    fn owner_normal_g(
        &mut self,
        m: &ProducedRawMetricV1,
        tangent: &VectorExprV1,
        owner_left: bool,
    ) -> RawLawsResultV1<VectorExprV1> {
        let covector = self.covector(m, tangent)?;
        let side = m.declaration.owner_orientation_sign * if owner_left { 1 } else { -1 };
        Ok([
            self.c
                .scaled(&covector[1], &Rational::integer(IBig::from(-side)))?,
            self.c
                .scaled(&covector[0], &Rational::integer(IBig::from(side)))?,
        ])
    }
    pub fn raw_incident_normal(
        &mut self,
        m: &ProducedRawMetricV1,
        raw: &RawDensityIncidentV1,
        use_declarations: bool,
    ) -> RawLawsResultV1<VectorExprV1> {
        Self::validate_incident(raw)?;
        let direction = self.vector(
            &rational_sub(&raw.end.x, &raw.start.x),
            &rational_sub(&raw.end.y, &raw.start.y),
        )?;
        let tangent = self.unit_g(m, &direction)?;
        let mut candidates = Vec::new();
        if use_declarations {
            for declaration in &raw.source_direction_declarations {
                let pair = match declaration {
                    RawDirectionDeclarationV1::CertifiedAffine {
                        x,
                        y,
                        reference_metric_id,
                    } => {
                        if reference_metric_id != &m.declaration.metric_id {
                            return Err(RawLawsFailureV1::Geometry(
                                RawGeometryRefusalV1::AffineMetricMismatch,
                            ));
                        }
                        Some((x, y))
                    }
                    RawDirectionDeclarationV1::CertifiedPlanar { x, y } => Some((x, y)),
                    RawDirectionDeclarationV1::Other { .. } => None,
                };
                if let Some((x, y)) = pair {
                    candidates.push(self.vector(x, y)?);
                }
            }
        }
        let analysis = if let Some(first) = candidates.first() {
            Some(self.unit_g(m, first)?)
        } else {
            None
        };
        if let Some(analysis) = &analysis {
            for candidate in candidates.iter().skip(1) {
                let normal = self.unit_g(m, candidate)?;
                let cross = self.oriented_cross(m, analysis, &normal)?;
                if self.sign(&cross)? != 0 {
                    return Err(RawLawsFailureV1::Geometry(
                        RawGeometryRefusalV1::ConflictingSupportDirections {
                            chain_use_id: raw.chain_use_id.clone(),
                        },
                    ));
                }
                let dot = self.dot_g(m, analysis, &normal)?;
                if self.sign(&dot)? <= 0 {
                    return Err(RawLawsFailureV1::Geometry(
                        RawGeometryRefusalV1::ConflictingSupportDirections {
                            chain_use_id: raw.chain_use_id.clone(),
                        },
                    ));
                }
            }
        }
        let mut cycle_normals = Vec::new();
        for (cycle, ordinal) in raw
            .owner_face_cycles
            .iter()
            .zip(&raw.owner_face_edge_ordinals)
        {
            let mut area = Rational::integer(IBig::ZERO);
            for i in 0..cycle.len() {
                let next = &cycle[(i + 1) % cycle.len()];
                area = rational_add(
                    &area,
                    &rational_sub(
                        &rational_mul(&cycle[i].x, &next.y),
                        &rational_mul(&cycle[i].y, &next.x),
                    ),
                );
            }
            let area_sign = area.parts().0.cmp(&IBig::ZERO);
            let start = &cycle[*ordinal];
            let end = &cycle[(*ordinal + 1) % cycle.len()];
            let edge = self.vector(
                &rational_sub(&end.x, &start.x),
                &rational_sub(&end.y, &start.y),
            )?;
            let dot = self.dot_g(m, &tangent, &edge)?;
            let same_direction = self.sign(&dot)? > 0;
            let coordinate_left = area_sign.is_gt() == same_direction;
            let owner_left = coordinate_left == (m.declaration.owner_orientation_sign > 0);
            cycle_normals.push(self.owner_normal_g(m, &tangent, owner_left)?);
        }
        let mut face = None;
        if let Some(first) = cycle_normals.first() {
            let mut consistent = true;
            for item in cycle_normals.iter().skip(1) {
                let cross = self.oriented_cross(m, first, item)?;
                if self.sign(&cross)? != 0 {
                    consistent = false;
                    break;
                }
                let dot = self.dot_g(m, first, item)?;
                if self.sign(&dot)? <= 0 {
                    consistent = false;
                    break;
                }
            }
            if consistent {
                face = Some(first.clone());
            }
        }
        if face.is_none() {
            let left = raw.owner_left.or_else(|| match raw.chain_use_orientation {
                Some(ChainUseOrientationV1::AStartToEnd | ChainUseOrientationV1::BStartToEnd) => {
                    Some(true)
                }
                _ => None,
            });
            if let Some(left) = left {
                face = Some(self.owner_normal_g(m, &tangent, left)?);
            }
        }
        let mut normal = None;
        if let Some(analysis) = analysis {
            let dot = self.dot_g(m, &analysis, &tangent)?;
            if self.sign(&dot)? == 0 {
                normal = Some(analysis);
            } else {
                let cross = self.oriented_cross(m, &analysis, &tangent)?;
                if self.sign(&cross)? == 0 {
                    normal = face.clone();
                } else {
                    return Err(RawLawsFailureV1::Geometry(
                        RawGeometryRefusalV1::SupportNeitherTangentNorNormal {
                            physical_edge_id: raw.physical_edge_id.clone(),
                        },
                    ));
                }
            }
        }
        let normal = normal.or(face).ok_or_else(|| {
            RawLawsFailureV1::Geometry(RawGeometryRefusalV1::AmbiguousOwnerNormal {
                physical_edge_id: raw.physical_edge_id.clone(),
            })
        })?;
        self.unit_g(m, &normal)
    }
    pub fn turn_witness(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
        turn: i8,
    ) -> RawLawsResultV1<TurnWitnessV1> {
        if !matches!(turn, -1 | 1) {
            return Err(RawLawsFailureV1::Contract(
                RawContractFailureV1::InvalidTurnOrientation,
            ));
        }
        let cross = self.oriented_cross(m, left, right)?;
        if self.sign(&cross)? != turn {
            return Err(RawLawsFailureV1::Geometry(
                RawGeometryRefusalV1::OrderedSupportsDoNotRealizeTurn,
            ));
        }
        let dot = self.dual_dot(m, left, right)?;
        let nl = self.dual_dot(m, left, left)?;
        let nr = self.dual_dot(m, right, right)?;
        let norms = self.c.mul(&[nl, nr])?;
        let square = self.c.mul(&[dot.clone(), dot.clone()])?;
        let ratio = self.c.div(&square, &norms)?;
        Ok(TurnWitnessV1 {
            dot_sign: self.sign(&dot)?,
            cosine_squared: self.fraction(&ratio)?,
        })
    }
    fn count_steps(count: usize, q: usize) -> RawLawsResultV1<(usize, usize)> {
        if !(2..=6).contains(&q) || count >= q {
            Err(RawLawsFailureV1::Contract(
                RawContractFailureV1::InvalidCountProfile,
            ))
        } else {
            Ok((count + 1, q))
        }
    }
    pub fn turn_count_comparison(
        &mut self,
        cosine_squared: &Rational,
        count: usize,
        q: usize,
    ) -> RawLawsResultV1<Option<i8>> {
        let (steps, q) = Self::count_steps(count, q)?;
        if steps >= q {
            return Ok(None);
        }
        let g = cftuv_radical::arith::gcd_u64(steps as u64, q as u64) as usize;
        let n = steps / g;
        let d = q / g;
        let threshold = if d == 5 {
            let five = self.c.rational(&Rational::integer(IBig::from(5)))?;
            let root = self.c.sqrt(&five)?;
            let root = self.c.scaled(
                &root,
                &Rational::integer(IBig::from(if n == 1 || n == 4 { 1 } else { -1 })),
            )?;
            let three = self.c.rational(&Rational::integer(IBig::from(3)))?;
            let sum = self.c.add(&[three, root])?;
            self.c
                .scaled(&sum, &Rational::new(IBig::ONE, UBig::from(8u8)).unwrap())?
        } else {
            let (a, b) = match (n, d) {
                (1, 6) | (5, 6) => (3, 4),
                (1, 4) | (3, 4) => (1, 2),
                (1, 3) | (2, 3) => (1, 4),
                (1, 2) => (0, 1),
                _ => {
                    return Err(RawLawsFailureV1::Contract(
                        RawContractFailureV1::InvalidCountProfile,
                    ))
                }
            };
            self.c
                .rational(&Rational::new(IBig::from(a), UBig::from(b as u32)).unwrap())?
        };
        let cosine = self.c.rational(cosine_squared)?;
        let residual = self.c.sub(&cosine, &threshold)?;
        Ok(Some(self.sign(&residual)?))
    }
    pub fn turn_count_feasible(
        &mut self,
        sign: i8,
        cosine_squared: &Rational,
        count: usize,
        q: usize,
    ) -> RawLawsResultV1<bool> {
        let (steps, q) = Self::count_steps(count, q)?;
        if steps >= q {
            return Ok(true);
        }
        if 2 * steps == q {
            return Ok(sign >= 0);
        }
        let comparison = self
            .turn_count_comparison(cosine_squared, count, q)?
            .unwrap();
        Ok(if 2 * steps < q {
            sign > 0 && comparison >= 0
        } else {
            sign >= 0 || comparison <= 0
        })
    }
    pub fn turn_count_exact_limit(
        &mut self,
        sign: i8,
        cosine_squared: &Rational,
        count: usize,
        q: usize,
    ) -> RawLawsResultV1<bool> {
        let (steps, q) = Self::count_steps(count, q)?;
        if steps >= q {
            return Ok(false);
        }
        Ok(
            self.turn_count_comparison(cosine_squared, count, q)? == Some(0)
                && sign
                    == if 2 * steps < q {
                        1
                    } else if 2 * steps > q {
                        -1
                    } else {
                        0
                    },
        )
    }
    pub fn rational_direction(
        &mut self,
        vector: &VectorExprV1,
    ) -> RawLawsResultV1<RationalDirectionV1> {
        let x = self.c.numeric_value(&vector[0])?;
        let y = self.c.numeric_value(&vector[1])?;
        let (Some(x), Some(y)) = (x, y) else {
            return Ok(RationalDirectionV1::Undetermined);
        };
        if x.value().terms().is_empty() {
            if y.value().terms().is_empty() {
                return Ok(RationalDirectionV1::Irrational);
            }
            return Ok(RationalDirectionV1::Rational(
                crate::farey::PrimitiveVector(IBig::ZERO, IBig::from(self.sign(&vector[1])?)),
            ));
        }
        if y.value().terms().is_empty() {
            return Ok(RationalDirectionV1::Rational(
                crate::farey::PrimitiveVector(IBig::from(self.sign(&vector[0])?), IBig::ZERO),
            ));
        }
        let first = &x.value().terms()[0];
        let coefficient =
            Rational::new(first.numerator.clone(), first.denominator.clone()).unwrap();
        let mut ratio = None;
        for term in y.value().terms() {
            if let Some(root) =
                cftuv_radical::arith::exact_root(&(&first.radicand * &term.radicand))
            {
                let other =
                    Rational::new(term.numerator.clone(), term.denominator.clone()).unwrap();
                let candidate = crate::interval::rational_div(&other, &coefficient)
                    .expect("nonzero canonical first term");
                let factor = Rational::new(IBig::from(root), first.radicand.clone()).unwrap();
                let candidate = rational_mul(&candidate, &factor);
                if y.sub(&x.scaled(&candidate)).value().terms().is_empty() {
                    ratio = Some(candidate);
                }
                break;
            }
        }
        let Some(ratio) = ratio else {
            return Ok(RationalDirectionV1::Irrational);
        };
        let sign = IBig::from(self.sign(&vector[0])?);
        let (n, d) = ratio.parts();
        Ok(RationalDirectionV1::Rational(
            crate::farey::PrimitiveVector(&sign * IBig::from(d.clone()), sign * n),
        ))
    }
    pub fn verify_raw_selection(
        &mut self,
        raw: &RawDensitySpecInputV1,
        canonical: Option<&Rational>,
    ) -> RawLawsResultV1<()> {
        let selection = &raw.selection;
        if selection.selection_policy_id != "HUBER_EMANATED_COUNT_DENSITY_A_V1"
            || selection.max_subturn_value_id
                != format!("LINEAR_REFLEX_DENSITY_{}_V1", selection.q - 2)
        {
            return Err(RawLawsFailureV1::Contract(
                RawContractFailureV1::SelectionPolicyMismatch,
            ));
        }
        let mut lower = raw.canonical.source_reflex_interval[0].clone();
        let mut upper = raw.canonical.source_reflex_interval[1].clone();
        let mut lower_open = false;
        if let Some(value) = declaration_value(
            &raw.canonical.source_reflex_declaration,
            "reflex_excess_over_pi",
        ) {
            let rows = value.rows().ok_or(RawLawsFailureV1::Contract(
                RawContractFailureV1::DeclarationRowsInvalid,
            ))?;
            lower_open = matches!(declaration_value(&rows,"lower_kind"),Some(PrimitiveFactV1::Text(kind)) if kind=="OPEN");
        }
        if let Some(canonical) = canonical {
            lower = canonical.clone();
            upper = canonical.clone();
            lower_open = false;
        }
        let mut bucket = None;
        for candidate in 1..=selection.q {
            let boundary =
                Rational::new(IBig::from(candidate - 1), UBig::from(selection.q)).unwrap();
            let residual = self.c.rational(&rational_sub(&lower, &boundary))?;
            let comparison = self.sign(&residual)?;
            if comparison > 0 || comparison == 0 && lower_open {
                let boundary =
                    Rational::new(IBig::from(candidate), UBig::from(selection.q)).unwrap();
                let residual = self.c.rational(&rational_sub(&upper, &boundary))?;
                if self.sign(&residual)? <= 0 {
                    bucket = Some(candidate);
                    break;
                }
            }
        }
        if bucket != Some(selection.bucket_c) {
            return Err(RawLawsFailureV1::Contract(
                RawContractFailureV1::SelectionBucketUnproven,
            ));
        }
        Ok(())
    }
    fn fan_math<T>(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
        turn: i8,
        q: usize,
        operation: impl FnOnce(
            &mut crate::density_laws_v1::OwnedDensityLawsV1<'_, '_>,
        ) -> crate::density_laws_v1::LawsResultV1<T>,
    ) -> RawLawsResultV1<T> {
        let inverse = &m.declaration.inverse_gram;
        let geometry = crate::density_laws_v1::OwnedFanGeometryV1 {
            inverse_gram: [
                [inverse[0].clone(), inverse[1].clone()],
                [inverse[1].clone(), inverse[2].clone()],
            ],
            owner_orientation: m.declaration.owner_orientation_sign,
            turn_orientation: turn,
            ideal: vec![left.clone(), right.clone()],
            q,
            window_law: crate::geometry::WindowLaw::Voronoi,
        };
        let mut laws = crate::density_laws_v1::OwnedDensityLawsV1::new(
            self.c,
            geometry,
            crate::density_angle_value_v1::AngleValueRegistryV1::new(),
            self.limits,
        )
        .map_err(RawLawsFailureV1::Arithmetic)?;
        operation(&mut laws).map_err(RawLawsFailureV1::Arithmetic)
    }
    pub fn quarter_turn(
        &mut self,
        m: &ProducedRawMetricV1,
        vector: &VectorExprV1,
        turn: i8,
    ) -> RawLawsResultV1<VectorExprV1> {
        self.fan_math(m, vector, vector, turn, 2, |laws| {
            laws.quarter_turn(vector, turn)
        })
    }
    pub fn rotate_q(
        &mut self,
        m: &ProducedRawMetricV1,
        vector: &VectorExprV1,
        turn: i8,
        q: usize,
    ) -> RawLawsResultV1<VectorExprV1> {
        self.fan_math(m, vector, vector, turn, q, |laws| {
            laws.rotate_q(vector, turn, q)
        })
    }
    pub fn subturn(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
        q: usize,
    ) -> RawLawsResultV1<bool> {
        self.fan_math(m, left, right, 1, q, |laws| laws.subturn(left, right, q))
    }
    pub fn subturn_boundary(
        &mut self,
        m: &ProducedRawMetricV1,
        left: &VectorExprV1,
        right: &VectorExprV1,
        q: usize,
    ) -> RawLawsResultV1<bool> {
        self.fan_math(m, left, right, 1, q, |laws| {
            laws.subturn_boundary(left, right, q)
        })
    }
    pub fn canonical_fan(
        &mut self,
        m: &ProducedRawMetricV1,
        incoming: &VectorExprV1,
        outgoing: &VectorExprV1,
        turn: i8,
        count: usize,
        canonical: &Rational,
    ) -> RawLawsResultV1<Vec<VectorExprV1>> {
        let step =
            crate::interval::rational_div(canonical, &Rational::integer(IBig::from(count + 1)))
                .unwrap();
        let (n, d) = step.parts();
        let q = usize::try_from(d.clone())
            .map_err(|_| RawLawsFailureV1::Contract(RawContractFailureV1::InvalidCountProfile))?;
        if n != &IBig::ONE || !(2..=6).contains(&q) {
            let text = if d == &UBig::ONE {
                n.to_string()
            } else {
                format!("{n}/{d}")
            };
            return Err(RawLawsFailureV1::CanonicalRestorationInvalid {
                message: format!("canonical subturn is not a single exact rotation: {text} of pi"),
            });
        }
        let initial = self.unit_covector(m, incoming)?;
        let right = self.unit_covector(m, outgoing)?;
        self.turn_witness(m, &initial, &right, turn)?;
        let mut left = initial.clone();
        let mut fan = vec![initial];
        for _ in 0..count {
            left = self.rotate_q(m, &left, turn, q)?;
            fan.push(left.clone());
        }
        fan.push(right);
        Ok(fan)
    }
    pub fn rotation_fan(
        &mut self,
        m: &ProducedRawMetricV1,
        incoming: &VectorExprV1,
        outgoing: &VectorExprV1,
        turn: i8,
        rotations: &[[i32; 2]],
    ) -> RawLawsResultV1<Vec<VectorExprV1>> {
        let left = self.unit_covector(m, incoming)?;
        let right = self.unit_covector(m, outgoing)?;
        self.turn_witness(m, &left, &right, turn)?;
        let perpendicular = self.quarter_turn(m, &left, turn)?;
        let mut fan = vec![left.clone()];
        for &[a, b] in rotations {
            let norm = self
                .c
                .rational(&Rational::integer(IBig::from(a * a + b * b)))?;
            let norm = self.c.sqrt(&norm)?;
            let mut coordinates = vec![];
            for axis in 0..2 {
                let x = self
                    .c
                    .scaled(&left[axis], &Rational::integer(IBig::from(a)))?;
                let y = self
                    .c
                    .scaled(&perpendicular[axis], &Rational::integer(IBig::from(b)))?;
                let sum = self.c.add(&[x, y])?;
                coordinates.push(self.c.div(&sum, &norm)?);
            }
            fan.push([coordinates.remove(0), coordinates.remove(0)]);
        }
        fan.push(right);
        Ok(fan)
    }
    pub fn principal_fan(
        &mut self,
        m: &ProducedRawMetricV1,
        incoming: &VectorExprV1,
        outgoing: &VectorExprV1,
        turn: i8,
        count: usize,
    ) -> RawLawsResultV1<ProducedRawFanV1> {
        let left = self.unit_covector(m, incoming)?;
        let right = self.unit_covector(m, outgoing)?;
        let witness = self.turn_witness(m, &left, &right, turn)?;
        let mut ideal = vec![left.clone()];
        if count > 0 {
            let ratio = self.c.rational(&witness.cosine_squared)?;
            let one = self.c.rational(&Rational::integer(IBig::ONE))?;
            let cosine = self.c.sqrt(&ratio)?;
            let cosine = self
                .c
                .scaled(&cosine, &Rational::integer(IBig::from(witness.dot_sign)))?;
            let sine_squared = self.c.sub(&one, &ratio)?;
            let sine = self.c.sqrt(&sine_squared)?;
            let principal = self.c.raw_atan2(&sine, &cosine)?;
            let perpendicular = self.quarter_turn(m, &left, turn)?;
            let n = u8::try_from(count + 1).map_err(|_| {
                RawLawsFailureV1::Contract(RawContractFailureV1::InvalidCountProfile)
            })?;
            let atom =
                crate::angle::AngleAtom::new(n, witness.cosine_squared.clone(), witness.dot_sign)
                    .map_err(|_| {
                    RawLawsFailureV1::Contract(RawContractFailureV1::InvalidCountProfile)
                })?;
            for ordinal in 1..n {
                let pair = if let Some(pair) = crate::density_trig_v1::owned_principal_ray(
                    self.c,
                    &witness.cosine_squared,
                    witness.dot_sign,
                    n,
                    ordinal,
                )
                .map_err(|e| RawLawsFailureV1::Arithmetic(LawsErrorV1::Trig(e)))?
                {
                    pair
                } else {
                    let angle = self.c.scaled(
                        &principal,
                        &Rational::new(IBig::from(ordinal), UBig::from(n)).unwrap(),
                    )?;
                    [self.c.raw_cos(&angle)?, self.c.raw_sin(&angle)?]
                };
                if matches!(
                    self.c.form(&pair[0])?,
                    crate::density_construct_v1::OwnedFormV1::Cos(_)
                ) || matches!(
                    self.c.form(&pair[1])?,
                    crate::density_construct_v1::OwnedFormV1::Sin(_)
                ) {
                    self.values
                        .register_ray(self.c, &pair[0], &pair[1], &atom, ordinal)
                        .map_err(|e| RawLawsFailureV1::Arithmetic(LawsErrorV1::Value(e)))?;
                }
                let mut coordinates = vec![];
                for axis in 0..2 {
                    let x = self.c.mul(&[pair[0].clone(), left[axis].clone()])?;
                    let y = self
                        .c
                        .mul(&[pair[1].clone(), perpendicular[axis].clone()])?;
                    coordinates.push(self.c.add(&[x, y])?);
                }
                ideal.push([coordinates.remove(0), coordinates.remove(0)]);
            }
        }
        ideal.push(right);
        Ok(ProducedRawFanV1 {
            ideal,
            values: self.values.clone(),
        })
    }
}
