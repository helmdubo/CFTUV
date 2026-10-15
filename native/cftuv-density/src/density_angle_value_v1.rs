//! Локальная semantic A2 metadata, отдельная от owned literal admission.
//! Caller удостоверяет связь source angle с exact principal facts до registration.
//! Вся композиция интервальна; component160 не доказывает whole-shell точность.
use crate::angle::AngleAtom;
use crate::angle_profile::{self, FixedAngleError};
use crate::density_construct_v1::{ConstructErrorV1, ConstructorV1, OwnedExprV1, OwnedFormV1};
use crate::interval::{rational_sub, IntervalError, RationalInterval};
use crate::{IBig, Rational, UBig};
use std::collections::HashMap;

pub const CANON: &str = "DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1";
pub const VALUE_BITS: u32 = 160;
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AngleValueErrorV1 {
    Construction(ConstructErrorV1),
    InvalidRegistration,
    ConflictingRegistration,
    UnregisteredOpaque,
    NoCanonicalRecipe,
    UnsupportedPower,
    Interval(IntervalError),
    Ray(FixedAngleError),
}
impl From<ConstructErrorV1> for AngleValueErrorV1 {
    fn from(value: ConstructErrorV1) -> Self {
        Self::Construction(value)
    }
}
impl From<IntervalError> for AngleValueErrorV1 {
    fn from(value: IntervalError) -> Self {
        Self::Interval(value)
    }
}
impl From<FixedAngleError> for AngleValueErrorV1 {
    fn from(value: FixedAngleError) -> Self {
        Self::Ray(value)
    }
}
pub type ValueResultV1<T> = Result<T, AngleValueErrorV1>;

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
struct ComponentKey {
    subdivision: u8,
    ratio: Rational,
    sign: i8,
    ordinal: u8,
    sine: bool,
    quarter_turns: i8,
}
#[derive(Clone, Default)]
pub struct AngleValueRegistryV1 {
    // Owned anchor удерживает arena token и запрещает перенос memo в новую arena.
    anchor: Option<OwnedExprV1>,
    components: HashMap<usize, (OwnedExprV1, ComponentKey)>,
    rays: HashMap<(u8, Rational, i8, u8), crate::angle::RayEnclosure>,
    memo: HashMap<usize, RationalInterval>,
}
impl AngleValueRegistryV1 {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn register_canonical_ray(
        &mut self,
        c: &ConstructorV1<'_>,
        cosine: &OwnedExprV1,
        sine: &OwnedExprV1,
        multiple: &Rational,
    ) -> ValueResultV1<()> {
        self.bind(c, cosine)?;
        self.bind(c, sine)?;
        let (atom, ordinal) =
            canonical_pi_recipe(multiple).ok_or(AngleValueErrorV1::NoCanonicalRecipe)?;
        self.register_ray(c, cosine, sine, &atom, ordinal)
    }
    fn bind(&mut self, c: &ConstructorV1<'_>, value: &OwnedExprV1) -> ValueResultV1<usize> {
        if let Some(anchor) = &self.anchor {
            c.raw_id(anchor)?;
        }
        let id = c.raw_id(value)?;
        if self.anchor.is_none() {
            self.anchor = Some(value.clone());
        }
        Ok(id)
    }
    pub fn register_ray(
        &mut self,
        c: &ConstructorV1<'_>,
        cosine: &OwnedExprV1,
        sine: &OwnedExprV1,
        atom: &AngleAtom,
        ordinal: u8,
    ) -> ValueResultV1<()> {
        self.register_ray_with_quarter_turns(c, cosine, sine, atom, ordinal, 0)
    }
    /// Только direct source ray и наблюдённая literal-компонента phi-pi/2.
    pub fn register_ray_with_quarter_turns(
        &mut self,
        c: &ConstructorV1<'_>,
        cosine: &OwnedExprV1,
        sine: &OwnedExprV1,
        atom: &AngleAtom,
        ordinal: u8,
        quarter_turns: i8,
    ) -> ValueResultV1<()> {
        let cosine_id = self.bind(c, cosine)?;
        let sine_id = self.bind(c, sine)?;
        if ordinal == 0 || ordinal >= atom.subdivision() || !matches!(quarter_turns, 0 | -1) {
            return Err(AngleValueErrorV1::InvalidRegistration);
        }
        let cosine_form = c.form(cosine)?;
        let sine_form = c.form(sine)?;
        if !matches!(cosine_form, OwnedFormV1::Numeric(_) | OwnedFormV1::Cos(_))
            || !matches!(sine_form, OwnedFormV1::Numeric(_) | OwnedFormV1::Sin(_))
        {
            return Err(AngleValueErrorV1::InvalidRegistration);
        }
        if let (OwnedFormV1::Cos(a), OwnedFormV1::Sin(b)) = (&cosine_form, &sine_form) {
            if !c.same_literal(a, b)? {
                return Err(AngleValueErrorV1::InvalidRegistration);
            }
        }
        let angle = match (&cosine_form, &sine_form) {
            (OwnedFormV1::Cos(a), _) | (_, OwnedFormV1::Sin(a)) => Some(a),
            _ => None,
        };
        let (atom, ordinal) = if let Some(multiple) = angle
            .map(|a| canonical_multiple(c, a))
            .transpose()?
            .flatten()
        {
            let Some(source_angle) =
                crate::density_trig_v1::principal_pi_multiple(atom.ratio(), atom.dot_sign())
                    .expect("validated atom")
            else {
                return Err(AngleValueErrorV1::InvalidRegistration);
            };
            let (a, d) = source_angle.parts();
            let expected =
                Rational::new(a * IBig::from(ordinal), d * UBig::from(atom.subdivision()))
                    .expect("atom subdivision");
            let shift = Rational::new(IBig::from(quarter_turns), UBig::from(2u8))
                .expect("closed quarter shift");
            let unshifted = rational_sub(&multiple, &shift);
            if expected != unshifted {
                return Err(AngleValueErrorV1::InvalidRegistration);
            }
            canonical_pi_recipe(&unshifted).ok_or(AngleValueErrorV1::NoCanonicalRecipe)?
        } else {
            (atom.clone(), ordinal)
        };
        let mut additions = Vec::new();
        for (id, value, form, is_sine) in [
            (cosine_id, cosine, cosine_form, false),
            (sine_id, sine, sine_form, true),
        ] {
            if matches!(form, OwnedFormV1::Numeric(_)) {
                continue;
            }
            let key = ComponentKey {
                subdivision: atom.subdivision(),
                ratio: atom.ratio().clone(),
                sign: atom.dot_sign(),
                ordinal,
                sine: is_sine,
                quarter_turns,
            };
            if self
                .components
                .get(&id)
                .is_some_and(|(_, previous)| previous != &key)
            {
                return Err(AngleValueErrorV1::ConflictingRegistration);
            }
            additions.push((id, (value.clone(), key)));
        }
        self.components.extend(additions);
        self.memo.clear();
        Ok(())
    }
    pub fn enclosure(
        &mut self,
        c: &ConstructorV1<'_>,
        expression: &OwnedExprV1,
    ) -> ValueResultV1<RationalInterval> {
        let id = self.bind(c, expression)?;
        if let Some(value) = self.memo.get(&id) {
            return Ok(value.clone());
        }
        let form = c.form(expression)?;
        let value = if let OwnedFormV1::Numeric(scalar) = form {
            let ((lower, upper), _) = scalar.value().enclosure(VALUE_BITS as usize);
            RationalInterval::new(
                Rational::new(lower.0, lower.1).expect("V2 lower denominator"),
                Rational::new(upper.0, upper.1).expect("V2 upper denominator"),
            )?
        } else if let Some((handle, key)) = self.components.get(&id).cloned() {
            c.raw_id(&handle)?;
            let ray_key = (key.subdivision, key.ratio.clone(), key.sign, key.ordinal);
            if !self.rays.contains_key(&ray_key) {
                let atom = AngleAtom::new(key.subdivision, key.ratio, key.sign)
                    .expect("validated immutable AngleAtom fields");
                self.rays
                    .insert(ray_key.clone(), angle_profile::ray(&atom, key.ordinal)?);
            }
            let ray = &self.rays[&ray_key];
            if key.quarter_turns == -1 {
                if key.sine {
                    RationalInterval::point(Rational::integer(IBig::ZERO)).sub(&ray.cosine)
                } else {
                    ray.sine.clone()
                }
            } else if key.sine {
                ray.sine.clone()
            } else {
                ray.cosine.clone()
            }
        } else {
            self.compose(c, form)?
        };
        self.memo.insert(id, value.clone());
        Ok(value)
    }
    fn compose(
        &mut self,
        c: &ConstructorV1<'_>,
        form: OwnedFormV1,
    ) -> ValueResultV1<RationalInterval> {
        match form {
            OwnedFormV1::Add(children) => self.fold(c, children, true),
            OwnedFormV1::Mul(children) => self.fold(c, children, false),
            OwnedFormV1::IntegerPow { base, exponent } => {
                interval_integer_power(&self.enclosure(c, &base)?, exponent)
            }
            OwnedFormV1::OpaquePow { base, exponent } => {
                let (n, d) = exponent.parts();
                if d != &UBig::from(2u8) || n < &IBig::ZERO {
                    return Err(AngleValueErrorV1::UnsupportedPower);
                }
                let power =
                    i64::try_from(n.clone()).map_err(|_| AngleValueErrorV1::UnsupportedPower)?;
                let root = self.enclosure(c, &base)?.positive_sqrt(VALUE_BITS)?;
                interval_integer_power(&root, power)
            }
            _ => Err(AngleValueErrorV1::UnregisteredOpaque),
        }
    }
    fn fold(
        &mut self,
        c: &ConstructorV1<'_>,
        children: Vec<OwnedExprV1>,
        additive: bool,
    ) -> ValueResultV1<RationalInterval> {
        let mut value = RationalInterval::point(Rational::integer(if additive {
            IBig::ZERO
        } else {
            IBig::ONE
        }));
        for child in children {
            let next = self.enclosure(c, &child)?;
            value = if additive {
                value.add(&next)
            } else {
                value.mul(&next)
            };
        }
        Ok(value)
    }
}
fn canonical_multiple(
    c: &ConstructorV1<'_>,
    angle: &OwnedExprV1,
) -> ValueResultV1<Option<Rational>> {
    // Только closed owned Numeric(Q)*Pi template, без generic AST parser.
    Ok(match c.form(angle)? {
        OwnedFormV1::Pi => Some(Rational::integer(IBig::ONE)),
        OwnedFormV1::Numeric(v) if v.value().is_zero() => Some(Rational::integer(IBig::ZERO)),
        OwnedFormV1::Mul(args) if args.len() == 2 => match (c.form(&args[0])?, c.form(&args[1])?) {
            (OwnedFormV1::Numeric(v), OwnedFormV1::Pi) => v
                .value()
                .as_rational()
                .map(|(n, d)| Rational::new(n, d).expect("numeric denominator")),
            _ => None,
        },
        _ => None,
    })
}
/// Детерминированный tuple order (n,r,s,j); только объявленные principal facts.
pub fn canonical_pi_recipe(multiple: &Rational) -> Option<(AngleAtom, u8)> {
    let q =
        |n: i32, d: u32| Rational::new(IBig::from(n), UBig::from(d)).expect("table denominator");
    for n in 2..=6 {
        for ratio in [q(0, 1), q(1, 4), q(1, 2), q(3, 4)] {
            let signs: &[i8] = if ratio.parts().0.is_zero() {
                &[0]
            } else {
                &[-1, 1]
            };
            for &sign in signs {
                let angle = crate::density_trig_v1::principal_pi_multiple(&ratio, sign)
                    .expect("valid finite principal facts")
                    .expect("known finite angle");
                for j in 1..n {
                    let (a, d) = angle.parts();
                    let value = Rational::new(a * IBig::from(j), d * UBig::from(n))
                        .expect("positive subdivision");
                    if &value == multiple {
                        return Some((AngleAtom::new(n, ratio, sign).expect("finite atom"), j));
                    }
                }
            }
        }
    }
    None
}
pub fn interval_integer_power(
    value: &RationalInterval,
    exponent: i64,
) -> ValueResultV1<RationalInterval> {
    let one = RationalInterval::point(Rational::integer(IBig::ONE));
    let mut base = if exponent < 0 {
        one.div(value)?
    } else {
        value.clone()
    };
    let mut remaining = exponent.unsigned_abs();
    let mut result = one;
    while remaining != 0 {
        if remaining & 1 != 0 {
            result = result.mul(&base);
        }
        remaining >>= 1;
        if remaining != 0 {
            base = base.square();
        }
    }
    Ok(result)
}
