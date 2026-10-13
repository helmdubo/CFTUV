//! Pairing gate: mode, operation, n, r=n/d, sign, j (TAB); exact bound rows only.
use cftuv_density::angle::AngleAtom;
use cftuv_density::density_angle_value_v1::*;
use cftuv_density::density_construct_v1::{ConstructorV1, OwnedExprV1};
use cftuv_density::density_expression::Expressions;
use cftuv_density::density_trig_v1::{owned_exact_q_trig, principal_pi_multiple};
use cftuv_density::interval::RationalInterval;
use cftuv_density::{IBig, Rational, UBig};
use std::io::{self, BufRead, Write};
fn q(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn fraction(text: &str) -> Result<Rational, String> {
    let (n, d) = text.split_once('/').ok_or("rational n/d required")?;
    Rational::new(
        n.parse::<IBig>().map_err(|e| format!("{e:?}"))?,
        d.parse::<UBig>().map_err(|e| format!("{e:?}"))?,
    )
    .map_err(|e| format!("{e:?}"))
}
fn bound(value: &RationalInterval) -> String {
    let (ln, ld) = value.lower().parts();
    let (un, ud) = value.upper().parts();
    format!("{ln}/{ld}\t{un}/{ud}")
}
fn build(
    c: &mut ConstructorV1<'_>,
    atom: &AngleAtom,
    j: u8,
    canonical: bool,
) -> Result<[OwnedExprV1; 2], String> {
    let angle = if canonical {
        let multiple = principal_pi_multiple(atom.ratio(), atom.dot_sign())
            .map_err(|e| format!("{e:?}"))?
            .ok_or("NoCanonicalRecipe")?;
        let (n, d) = multiple.parts();
        let multiple =
            Rational::new(n * IBig::from(j), d * UBig::from(atom.subdivision())).unwrap();
        let pi = c.pi().unwrap();
        c.scaled(&pi, &multiple).unwrap()
    } else {
        let r = c.rational(atom.ratio()).unwrap();
        let root = c.sqrt(&r).unwrap();
        let cosine = c.scaled(&root, &q(i32::from(atom.dot_sign()), 1)).unwrap();
        let one = c.rational(&q(1, 1)).unwrap();
        let rest = c.sub(&one, &r).unwrap();
        let sine = c.sqrt(&rest).unwrap();
        let angle = c.raw_atan2(&sine, &cosine).unwrap();
        c.scaled(&angle, &q(i32::from(j), u32::from(atom.subdivision())))
            .unwrap()
    };
    Ok([c.raw_cos(&angle).unwrap(), c.raw_sin(&angle).unwrap()])
}
fn run(line: &str) -> Result<String, String> {
    let fields: Vec<_> = line.split('\t').collect();
    if fields.len() != 6 {
        return Err("six fields required".into());
    }
    let n = fields[2].parse().map_err(|e| format!("{e:?}"))?;
    let r = fraction(fields[3])?;
    let sign = fields[4].parse().map_err(|e| format!("{e:?}"))?;
    let j = fields[5].parse().map_err(|e| format!("{e:?}"))?;
    let atom = AngleAtom::new(n, r, sign).map_err(|e| format!("{e:?}"))?;
    let mut graph = Expressions::new();
    let mut c = ConstructorV1::new(&mut graph);
    let mut values = AngleValueRegistryV1::new();
    let [cosine, sine] = if fields[0] == "q" {
        owned_exact_q_trig(&mut c, n).map_err(|e| format!("{e:?}"))?
    } else {
        build(&mut c, &atom, j, fields[0] == "canonical")?
    };
    if fields[0] != "q" {
        values
            .register_ray(&c, &cosine, &sine, &atom, j)
            .map_err(|e| format!("{e:?}"))?;
    }
    let operation = match fields[1] {
        "cos" => cosine.clone(),
        "sin" => sine.clone(),
        "sum" => c.add(&[cosine.clone(), sine.clone()]).unwrap(),
        "mul" => c.mul(&[cosine.clone(), sine.clone()]).unwrap(),
        "ratio" => c.partial_div(&sine, &cosine).unwrap(),
        "reciprocal" => c.formal_reciprocal(&sine).unwrap(),
        "sqrt" => c.sqrt(&sine).unwrap(),
        "unit" => {
            let a = c.integer_power(&cosine, 2).unwrap();
            let b = c.integer_power(&sine, 2).unwrap();
            let sum = c.add(&[a, b]).unwrap();
            let one = c.rational(&q(1, 1)).unwrap();
            c.sub(&sum, &one).unwrap()
        }
        _ => return Err("unknown operation".into()),
    };
    match values.enclosure(&c, &operation) {
        Err(AngleValueErrorV1::Interval(
            cftuv_density::interval::IntervalError::DenominatorContainsZero,
        )) => Ok("GAP\tDENOMINATOR_CONTAINS_ZERO".into()),
        Err(error) => Ok(format!("GAP\t{error:?}")),
        Ok(result) => Ok(format!(
            "VALUE\t{}\t{}\t{}",
            bound(&values.enclosure(&c, &cosine).unwrap()),
            bound(&values.enclosure(&c, &sine).unwrap()),
            bound(&result)
        )),
    }
}
fn main() -> io::Result<()> {
    let mut out = io::BufWriter::new(io::stdout().lock());
    for line in io::stdin().lock().lines() {
        let line = line?;
        match run(&line) {
            Ok(value) => writeln!(out, "{value}")?,
            Err(error) => writeln!(out, "READ_ERROR\t{error}")?,
        }
    }
    Ok(())
}
