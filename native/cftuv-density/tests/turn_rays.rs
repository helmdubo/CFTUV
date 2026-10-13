use cftuv_density::turn_basis::TurnBasisTransaction;
use cftuv_density::turn_rays::{
    cosine_squared, principal_turn, table_ray, table_rays, CertifiedGramSquares, PiMultiple,
    RayFailureKind, RayInvalid, RayProductReason,
};
use cftuv_density::{DensityTransaction, IBig, Metric2, Radical, Rational, UBig, Vector2, Work};

fn rat(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn integer(n: i32) -> Radical {
    rat(n, 1).radical()
}
fn vector(x: i32, y: i32) -> Vector2 {
    Vector2::new(integer(x), integer(y))
}
fn facts(left: Rational, right: Rational, square: Rational) -> CertifiedGramSquares {
    CertifiedGramSquares {
        incoming_squared: left,
        outgoing_squared: right,
        dot_squared: square,
    }
}
fn pair(a: i32, b: i32) -> (IBig, IBig) {
    (IBig::from(a), IBig::from(b))
}
fn product() -> RayFailureKind {
    RayFailureKind::Product(RayProductReason::StrictPrincipalTurnRequired)
}

#[test]
fn certified_ratio_is_exact_and_keeps_nonpositive_norms_internal() {
    assert_eq!(
        cosine_squared(&facts(rat(2, 1), rat(8, 1), rat(9, 1))).unwrap(),
        (rat(9, 16), Work::NONE)
    );
    assert_eq!(
        cosine_squared(&facts(rat(3, 2), rat(2, 5), rat(1, 5)))
            .unwrap()
            .0,
        rat(1, 3)
    );
    for norm in [rat(0, 1), rat(-1, 1)] {
        let error = cosine_squared(&facts(norm, rat(1, 1), rat(0, 1))).unwrap_err();
        assert_eq!(
            error.kind,
            RayFailureKind::InternalInvalid(RayInvalid::NonPositiveCertifiedNorm)
        );
        assert_eq!(error.work, Work::NONE);
    }
}

#[test]
fn cosine_bounds_keep_source_product_outcome_and_zero_is_admitted() {
    for square in [rat(-1, 1), rat(1, 1), rat(2, 1)] {
        let error = cosine_squared(&facts(rat(1, 1), rat(1, 1), square)).unwrap_err();
        assert_eq!(error.kind, product());
        assert_eq!(error.work, Work::NONE);
    }
    assert_eq!(
        cosine_squared(&facts(rat(1, 1), rat(1, 1), rat(0, 1)))
            .unwrap()
            .0,
        rat(0, 1)
    );
    assert_eq!(
        RayProductReason::StrictPrincipalTurnRequired.outcome(),
        "PLANAR_OWNER_INTERIOR_DIRECTION_REQUIRED"
    );
    assert_eq!(
        RayProductReason::StrictPrincipalTurnRequired.message(),
        "Density A requires a strict principal turn in (0, pi)"
    );
}

#[test]
fn principal_descriptor_checks_ratio_before_dot_consistency_and_handles_exact_zero() {
    let zero = principal_turn(&rat(0, 1), &integer(0)).unwrap().0;
    assert_eq!(zero.source().cosine_squared(), &rat(0, 1));
    assert_eq!(zero.source().dot_sign(), 0);
    for sign in [-1, 1] {
        let turn = principal_turn(&rat(9, 16), &integer(3 * sign)).unwrap().0;
        assert_eq!(turn.source().dot_sign(), sign as i8);
    }
    for (ratio, dot, expected) in [(rat(1, 4), integer(0), 0), (rat(0, 1), integer(1), 1)] {
        assert_eq!(
            principal_turn(&ratio, &dot).unwrap_err().kind,
            RayFailureKind::InternalInvalid(RayInvalid::InconsistentDotSign(expected))
        );
    }
    for ratio in [rat(-1, 1), rat(1, 1)] {
        assert_eq!(
            principal_turn(&ratio, &integer(0)).unwrap_err().kind,
            product()
        );
    }
}

#[test]
fn canonical_pi_multiple_is_rational_arithmetic_for_every_hidden_ordinal() {
    for subdivision in 2u8..=6 {
        for ordinal in 1..subdivision {
            let angle = PiMultiple::ray_angle(&rat(3, 4), ordinal, subdivision).unwrap();
            assert_eq!(
                angle.0.coefficient(),
                &rat(3 * ordinal as i32, 4 * subdivision as u32)
            );
            assert_eq!(angle.1, Work::NONE);
        }
    }
    assert_eq!(
        PiMultiple::ray_angle(&rat(-1, 2), 1, 2)
            .unwrap()
            .0
            .coefficient(),
        &rat(-1, 4)
    );
    assert_eq!(
        PiMultiple::ray_angle(&rat(1, 2), 0, 2).unwrap_err().kind,
        RayFailureKind::InternalInvalid(RayInvalid::InvalidOrdinal {
            ordinal: 0,
            subdivision: 2
        })
    );
    assert_eq!(
        PiMultiple::ray_angle(&rat(1, 2), 1, 1).unwrap_err().kind,
        RayFailureKind::InternalInvalid(RayInvalid::InvalidSubdivision(1))
    );
}

#[test]
fn table_formula_normalizes_integer_pairs_and_keeps_numeric_orientation() {
    let ray = table_ray(&vector(1, 0), &vector(0, 1), 1, &pair(12, 5)).unwrap();
    assert_eq!(
        ray.0,
        Vector2::new(rat(12, 13).radical(), rat(5, 13).radical())
    );
    assert_eq!(
        table_ray(&vector(1, 0), &vector(0, 1), -1, &pair(12, 5))
            .unwrap()
            .0,
        Vector2::new(rat(12, 13).radical(), rat(-5, 13).radical())
    );
    assert_eq!(
        table_ray(&vector(1, 0), &vector(0, 1), 0, &pair(12, 5))
            .unwrap()
            .0,
        Vector2::new(rat(12, 13).radical(), integer(0))
    );
    assert_eq!(
        table_ray(&vector(1, 0), &vector(0, 1), 1, &pair(-24, -10))
            .unwrap()
            .0,
        Vector2::new(rat(-12, 13).radical(), rat(-5, 13).radical())
    );
    assert_eq!(
        table_ray(&vector(1, 0), &vector(0, 1), 1, &pair(12, 5)).unwrap(),
        ray
    );
}

#[test]
fn all_table_ordinals_keep_value_and_failed_work_without_mutable_cache() {
    let rows = vec![pair(12, 5), pair(1, 1), pair(5, 12)];
    let (rays, work) = table_rays(&vector(1, 0), &vector(0, 1), 1, 3, &rows).unwrap();
    let mut expected_work = Work::NONE;
    for (ordinal, row) in rows.iter().enumerate() {
        let priced = table_ray(&vector(1, 0), &vector(0, 1), 1, row).unwrap();
        assert_eq!(rays[ordinal], priced.0);
        expected_work = expected_work + priced.1;
    }
    assert_eq!(work, expected_work);
    let first_work = table_ray(&vector(1, 0), &vector(0, 1), 1, &pair(12, 5))
        .unwrap()
        .1;
    let invalid = vec![pair(12, 5), pair(0, 0)];
    let failure = table_rays(&vector(1, 0), &vector(0, 1), 1, 2, &invalid).unwrap_err();
    assert_eq!(
        failure.kind,
        RayFailureKind::InternalInvalid(RayInvalid::ZeroRotationRow { ordinal: 2 })
    );
    assert_eq!(failure.work, first_work);
    assert_eq!(
        table_rays(&vector(1, 0), &vector(0, 1), 1, 2, &invalid).unwrap_err(),
        failure
    );
    let short = table_rays(&vector(1, 0), &vector(0, 1), 1, 3, &rows[..1]).unwrap_err();
    assert_eq!(
        short.kind,
        RayFailureKind::InternalInvalid(RayInvalid::WrongTableLength {
            expected: 3,
            actual: 1
        })
    );
    assert_eq!(short.work, Work::NONE);
}

#[test]
fn table_rays_use_the_existing_gram_stage_a_basis_on_both_owner_and_turn_signs() {
    for owner in [-1, 1] {
        let (metric, _) =
            Metric2::from_gram([[rat(2, 1), rat(1, 3)], [rat(1, 3), rat(3, 1)]], owner).unwrap();
        for orientation in [-1, 1] {
            let mut stage_a = TurnBasisTransaction::new(DensityTransaction::new(metric.clone()));
            let outgoing = vector(
                -2 * owner as i32 * orientation as i32,
                owner as i32 * orientation as i32,
            );
            let atom = stage_a
                .prepare_turn(&vector(1, 2), &outgoing, orientation)
                .unwrap()
                .0;
            let left = stage_a.left_unit_normal(&atom.incoming).unwrap().0;
            let rays = table_rays(
                &atom.incoming,
                &left,
                orientation,
                3,
                &[pair(12, 5), pair(1, 1), pair(5, 12)],
            )
            .unwrap()
            .0;
            let mut facts = DensityTransaction::new(metric.clone());
            for ray in rays {
                assert_eq!(facts.gram_dot(&ray, &ray).0, integer(1));
            }
        }
    }
}

#[test]
#[ignore = "Needs CFTUV_TURN_RAYS_ORACLE_PYTHON and CFTUV_TURN_RAYS_REPLAY"]
fn live_python_stage_b_descriptors_and_table_rays_match_certified_inputs() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle =
        Command::new(std::env::var("CFTUV_TURN_RAYS_ORACLE_PYTHON").expect("oracle Python"))
            .args(["-c", ORACLE])
            .env(
                "PYTHONPATH",
                std::env::join_paths([repo.join("kernel/src"), repo.join("tools")]).unwrap(),
            )
            .output()
            .unwrap();
    assert!(
        oracle.status.success(),
        "{}",
        String::from_utf8_lossy(&oracle.stderr)
    );
    let text = String::from_utf8(oracle.stdout).unwrap();
    let mut input = String::new();
    let mut expected = Vec::new();
    let mut observations = String::new();
    for line in text.lines() {
        if let Some(row) = line.strip_prefix("IN\t") {
            input.push_str(row);
            input.push('\n');
        } else if let Some(row) = line.strip_prefix("OUT\t") {
            expected.push(row.to_owned());
        } else if let Some(row) = line.strip_prefix("OBS\t") {
            observations.push_str(row);
            observations.push('\n');
        } else {
            panic!("unexpected oracle row:{line}");
        }
    }
    if let Ok(path) = std::env::var("CFTUV_TURN_RAYS_ARTIFACTS") {
        let path = std::path::Path::new(&path);
        std::fs::create_dir_all(path).unwrap();
        std::fs::write(path.join("input.tsv"), &input).unwrap();
        std::fs::write(path.join("expected.tsv"), expected.join("\n") + "\n").unwrap();
        std::fs::write(path.join("python-observations.jsonl"), observations).unwrap();
    }
    let mut native = Command::new(std::env::var("CFTUV_TURN_RAYS_REPLAY").expect("built replay"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    let mut stdin = native.stdin.take().unwrap();
    let writer = std::thread::spawn(move || stdin.write_all(input.as_bytes()));
    let output = native.wait_with_output().unwrap();
    writer.join().unwrap().unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
    let actual = String::from_utf8(output.stdout).unwrap();
    if let Ok(path) = std::env::var("CFTUV_TURN_RAYS_ARTIFACTS") {
        std::fs::write(std::path::Path::new(&path).join("actual.tsv"), &actual).unwrap();
    }
    let actual: Vec<_> = actual
        .lines()
        .map(|line| line.split("\tV2_WORK:").next().unwrap())
        .collect();
    assert_eq!(actual.len(), expected.len());
    assert!(expected.len() > 500);
    for (index, (actual, expected)) in actual.iter().zip(&expected).enumerate() {
        assert_eq!(*actual, expected.as_str(), "case{index}");
    }
    println!("{} actual Python certified StageB/descriptor/table cases:0 mismatch; raw/state/budget observations saved separately; no legacy admission or fan-memo parity claim", expected.len());
}

const ORACLE: &str = r#"
import json
import sympy as sp
from fractions import Fraction as F
from cftuv_envelope.reference import density_turn as turn
from cftuv_envelope.reference.density_vectors import _density_runtime_vector as V, _density_left_unit_normal
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.planar_types import exact_quadratic_value
from cftuv_envelope.reference.radical_v2 import RadicalSumV2, canonical_text
from native_density_corpus import Observation

def value(expr): return canonical_text(RadicalSumV2.from_terms(exact_quadratic_value(sp.expand(expr)).terms)[0])
def vector(v): return '~'.join(value(e) for e in v.expressions())
def ratio(v): return str(v.p)+'/'+str(v.q)
def state(atom): return {k:None if getattr(atom,k) is None else sp.srepr(getattr(atom,k)) if k in ('cosine_squared','principal_turn') else getattr(atom,k) if k=='turn_sign' else True for k in ('cosine_squared','turn_sign','principal_turn','basis')}
def metric(gram,owner):
 mat=sp.Matrix([[gram[0],gram[1]],[gram[1],gram[2]]]); inv=mat.inv()
 return ExactPlanarMetric(tuple(tuple(sp.Rational(mat[i,j]) for j in range(2)) for i in range(2)),tuple(tuple(inv[i,j] for j in range(2)) for i in range(2)),owner)
sequence=0
def emit(op,gram,owner,a,b,orientation,count,canonical=None,rows=None,ordinal=1):
 global sequence
 rows=None if rows is None else tuple(tuple(row) for row in rows)
 m=metric(gram,owner); incoming,outgoing=V(*a),V(*b)
 base=turn._turn_atom(m,incoming,outgoing,orientation)
 squared=sp.expand(base.raw_dot*base.raw_dot)
 assert base.incoming_squared.is_Rational and base.outgoing_squared.is_Rational and squared.is_Rational
 left=_density_left_unit_normal(m,base.incoming)
 ident='case'+str(sequence); sequence+=1
 u=F(1,2) if canonical is None else canonical
 rows_text='-' if rows is None else '|'.join(str(x)+','+str(y) for x,y in rows)
 context=json.dumps({'gram':list(map(str,gram)),'owner':owner,'incoming':list(map(str,a)),'outgoing':list(map(str,b))},separators=(',',':'))
 wire=[op,ident,ratio(base.incoming_squared),ratio(base.outgoing_squared),ratio(squared),value(base.raw_dot),str(u.numerator)+'/'+str(u.denominator),str(ordinal),str(count+1),vector(base.incoming),vector(left),str(orientation),rows_text,str(count),context]
 before=state(base); error=None
 with Observation() as observed:
  try:
   fan=turn._huber_density_interpolated_normals(m,incoming,outgoing,count,orientation,canonical,rows)
   atom=next(iter(m._density_exact_memo.turn_atoms.values()))
   if op=='ratio': answer='RATIO:'+ratio(atom.cosine_squared)
   elif op=='principal': answer='PRINCIPAL:'+ratio(atom.cosine_squared)+'~'+str(atom.turn_sign)
   elif op=='canonical':
    assert atom.principal_turn is not None
    angle=u*F(ordinal,count+1); answer='PI_MULTIPLE:'+str(angle.numerator)+'/'+str(angle.denominator)
   else:
    assert atom.principal_turn is None and atom.turn_sign is None
    answer='RAYS:'+'|'.join(vector(v) for v in fan[1:-1])
   after=state(atom)
  except ReferenceGeometryError as e:
   error=e; answer='PRODUCT:'+e.outcome.value+':'+str(e)
   atom=next(iter(m._density_exact_memo.turn_atoms.values())); after=state(atom)
  cost=observed.outcome(None,error,[])
 print('IN\t'+'\t'.join(wire)); print('OUT\t'+op+'\t'+ident+'\t'+answer)
 print('OBS\t'+json.dumps({'id':ident,'raw':json.loads(context),'stage_before':before,'stage_after':after,'cost':{k:cost[k] for k in ('budgets','counts','symbolic','disagreements')},'count':count,'canonical':str(canonical),'rotations':rows},separators=(',',':')))
grams=[(1,0,1),(2,sp.Rational(1,3),3),(sp.Rational(9,7),sp.Rational(-2,5),sp.Rational(5,3))]
row_sets=[[(1,1)],[(2,1),(1,2)],[(12,5),(1,1),(5,12)],[(4,1),(3,2),(2,3),(1,4)],[(5,1),(4,2),(1,1),(2,4),(1,5)]]
for gram in grams:
 for owner in [-1,1]:
  for orientation in [-1,1]:
   s=owner*orientation
   for a in [(1,0),(1,2),(sp.sqrt(2)/2,sp.sqrt(2)/2)]:
    for shift in [-2,0,2]:
     b=(shift*a[0]-s*a[1],shift*a[1]+s*a[0])
     emit('ratio',gram,owner,a,b,orientation,1,rows=[(1,1)])
     emit('principal',gram,owner,a,b,orientation,1)
     for count,rows in enumerate(row_sets,1):
      emit('table',gram,owner,a,b,orientation,count,rows=rows)
      for ordinal in range(1,count+1): emit('canonical',gram,owner,a,b,orientation,count,F(1,2),ordinal=ordinal)
# Реальный source c²=1 отказ после коллинеарного orientation=0 StageA.
for op in ['ratio','principal']:
 emit(op,(1,0,1),1,(1,0),(2,0),0,1,rows=[(1,1)] if op=='ratio' else None)
"#;
