//! Value-level Stage A prototype: raw Gram turn facts and normalized incident vectors.
//! Это не legacy DAG/admission/provenance ABI и не replay состояния Python memo.
//! Семантически рациональная V2-норма не заменяет будущий `expand(...).is_Rational` gate.
//! Бюджет Density-поиска не расходуется; V2 Work возвращается отдельно как диагностика.

use std::collections::HashMap;

use crate::{
    DensityFailure, DensityFailureKind, DensityResult, DensityTransaction, IBig, Metric2, Priced,
    Radical, UBig, Vector2, Work,
};

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TurnProductReason {
    OwnerTurnMismatch,
    SourceNormsNotRational,
    SupportNormNonPositive,
    BasisNotOrthogonal,
}

impl TurnProductReason {
    pub fn outcome(&self) -> &'static str {
        match self {
            Self::OwnerTurnMismatch | Self::SupportNormNonPositive => {
                "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED"
            }
            Self::SourceNormsNotRational | Self::BasisNotOrthogonal => {
                "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE"
            }
        }
    }

    pub fn message(&self) -> &'static str {
        match self {
            Self::OwnerTurnMismatch => {
                "ordered support normals do not realize the certified owner-sector turn"
            }
            Self::SourceNormsNotRational => "Density A source Gram norms are not rational",
            Self::SupportNormNonPositive => {
                "Density A support direction has non-positive Gram norm"
            }
            Self::BasisNotOrthogonal => "Density A Gram-orthogonal basis construction is not exact",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TurnUnsupported {
    /// V2 допускает только sqrt рационального; вложенный корень здесь не создаётся.
    NonRationalNormalizationRoot,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TurnFailureKind {
    Product(TurnProductReason),
    InternalUnsupported(TurnUnsupported),
    InternalDensity(DensityFailureKind),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TurnFailure {
    pub kind: TurnFailureKind,
    pub work: Work,
}

pub type TurnResult<T> = Result<Priced<T>, TurnFailure>;

impl TurnFailure {
    fn product(reason: TurnProductReason, work: Work) -> Self {
        Self {
            kind: TurnFailureKind::Product(reason),
            work,
        }
    }

    fn after(mut self, work: Work) -> Self {
        self.work = work + self.work;
        self
    }
}

/// Только первые пять полей `_TurnAtom`; H0 не вычисляет principal/cosine/basis.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TurnAtom {
    pub incoming_squared: Radical,
    pub outgoing_squared: Radical,
    pub raw_dot: Radical,
    pub incoming: Vector2,
    pub outgoing: Vector2,
}

/// Внешний A4 memo сюда не переносится. Все таблицы живут ровно одну транзакцию.
pub struct TurnBasisTransaction {
    transaction: DensityTransaction,
    atoms: HashMap<(Vector2, Vector2, i8), TurnAtom>,
    units: HashMap<(Vector2, Radical), Vector2>,
    left_normals: HashMap<Vector2, Vector2>,
}

impl TurnBasisTransaction {
    pub fn new(transaction: DensityTransaction) -> Self {
        Self {
            transaction,
            atoms: HashMap::new(),
            units: HashMap::new(),
            left_normals: HashMap::new(),
        }
    }

    pub fn metric(&self) -> &Metric2 {
        self.transaction.metric()
    }
    pub fn atom_cache_len(&self) -> usize {
        self.atoms.len()
    }
    pub fn unit_cache_len(&self) -> usize {
        self.units.len()
    }
    pub fn basis_cache_len(&self) -> usize {
        self.left_normals.len()
    }

    /// Source `_turn_atom`: cross gate прежде rational-norm gate, затем обе нормализации.
    /// Orientation=0 не запрещён: Python принимает коллинеарный H0 с ненулевыми нормами.
    pub fn prepare_turn(
        &mut self,
        incoming: &Vector2,
        outgoing: &Vector2,
        orientation: i8,
    ) -> TurnResult<TurnAtom> {
        let key = (incoming.clone(), outgoing.clone(), orientation);
        if let Some(atom) = self.atoms.get(&key) {
            return Ok((atom.clone(), Work::NONE));
        }
        let (incoming_squared, first) = self.transaction.gram_dot(incoming, incoming);
        let (outgoing_squared, second) = self.transaction.gram_dot(outgoing, outgoing);
        let (raw_dot, third) = self.transaction.gram_dot(incoming, outgoing);
        let (cross, fourth) = self.transaction.oriented_cross(incoming, outgoing);
        let mut work = first + second + third + fourth;
        let (sign, priced) = density(self.transaction.sign(&cross), work)?;
        work = work + priced;
        if sign != orientation {
            return Err(TurnFailure::product(
                TurnProductReason::OwnerTurnMismatch,
                work,
            ));
        }
        if incoming_squared.as_rational().is_none() || outgoing_squared.as_rational().is_none() {
            return Err(TurnFailure::product(
                TurnProductReason::SourceNormsNotRational,
                work,
            ));
        }
        let (incoming_unit, priced) = self
            .unit_from_squared(incoming, &incoming_squared)
            .map_err(|failure| failure.after(work))?;
        work = work + priced;
        let (outgoing_unit, priced) = self
            .unit_from_squared(outgoing, &outgoing_squared)
            .map_err(|failure| failure.after(work))?;
        let atom = TurnAtom {
            incoming_squared,
            outgoing_squared,
            raw_dot,
            incoming: incoming_unit,
            outgoing: outgoing_unit,
        };
        self.atoms.insert(key, atom.clone());
        Ok((atom, work + priced))
    }

    /// Source `_density_unit_from_squared`: неположительный знак проверяется прежде допуска sqrt.
    pub fn unit_from_squared(
        &mut self,
        vector: &Vector2,
        squared: &Radical,
    ) -> TurnResult<Vector2> {
        let (sign, mut work) = density(self.transaction.sign(squared), Work::NONE)?;
        if sign <= 0 {
            return Err(TurnFailure::product(
                TurnProductReason::SupportNormNonPositive,
                work,
            ));
        }
        let key = (vector.clone(), squared.clone());
        if let Some(unit) = self.units.get(&key) {
            return Ok((unit.clone(), work));
        }
        let Some((n, d)) = squared.as_rational() else {
            return Err(TurnFailure {
                kind: TurnFailureKind::InternalUnsupported(
                    TurnUnsupported::NonRationalNormalizationRoot,
                ),
                work,
            });
        };
        let (root, priced) = Radical::sqrt_of_rational(n, d)
            .map_err(|refusal| internal(DensityFailure::from_algebra(refusal.into(), work)))?;
        work = work + priced;
        let (x, priced) = vector
            .x
            .div(&root)
            .map_err(|failure| internal(DensityFailure::from_algebra(failure, work)))?;
        work = work + priced;
        let (y, priced) = vector
            .y
            .div(&root)
            .map_err(|failure| internal(DensityFailure::from_algebra(failure, work)))?;
        let unit = Vector2::new(x, y);
        self.units.insert(key, unit.clone());
        Ok((unit, work + priced))
    }

    /// Source `_density_left_unit_normal`: J*(G*t), не covector quarter_turn с G^-1.
    pub fn left_unit_normal(&mut self, tangent: &Vector2) -> TurnResult<Vector2> {
        if let Some(normal) = self.left_normals.get(tangent) {
            return Ok((normal.clone(), Work::NONE));
        }
        let gram = self.transaction.metric().gram();
        let (cx, first) = gram[0][0]
            .scale(&tangent.x)
            .add(&gram[0][1].scale(&tangent.y));
        let (cy, second) = gram[1][0]
            .scale(&tangent.x)
            .add(&gram[1][1].scale(&tangent.y));
        let owner = IBig::from(self.transaction.metric().owner_orientation_sign());
        let raw = Vector2::new(
            cy.scaled(&owner, &UBig::ONE).neg(),
            cx.scaled(&owner, &UBig::ONE),
        );
        let (orthogonality, third) = self.transaction.gram_dot(tangent, &raw);
        let work = first + second + third;
        if !orthogonality.is_zero() {
            return Err(TurnFailure::product(
                TurnProductReason::BasisNotOrthogonal,
                work,
            ));
        }
        let (squared, fourth) = self.transaction.gram_dot(&raw, &raw);
        let (unit, priced) = self
            .unit_from_squared(&raw, &squared)
            .map_err(|failure| failure.after(work + fourth))?;
        self.left_normals.insert(tangent.clone(), unit.clone());
        Ok((unit, work + fourth + priced))
    }
}

fn internal(failure: DensityFailure) -> TurnFailure {
    TurnFailure {
        kind: TurnFailureKind::InternalDensity(failure.kind),
        work: failure.work,
    }
}

fn density<T>(result: DensityResult<T>, work_before: Work) -> TurnResult<T> {
    result.map_err(|failure| internal(failure).after(work_before))
}
