use cftuv_density::turn_basis::{
    TurnBasisTransaction, TurnFailureKind, TurnProductReason, TurnUnsupported,
};
use cftuv_density::{DensityTransaction, IBig, Metric2, Radical, Rational, UBig, Vector2, Work};

fn rat(n: i32, d: u32) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn integer(n: i32) -> Radical {
    Radical::rational(IBig::from(n), UBig::ONE)
}
fn scalar(text: &str) -> Radical {
    Radical::from_text(text).unwrap().0
}
fn vector(x: i32, y: i32) -> Vector2 {
    Vector2::new(integer(x), integer(y))
}
fn transaction(off_diagonal: bool, owner: i8) -> TurnBasisTransaction {
    let (a, b, c) = if off_diagonal {
        (rat(2, 1), rat(1, 3), rat(3, 1))
    } else {
        (rat(1, 1), rat(0, 1), rat(1, 1))
    };
    let metric = Metric2::from_gram([[a, b.clone()], [b, c]], owner)
        .unwrap()
        .0;
    TurnBasisTransaction::new(DensityTransaction::new(metric))
}

#[test]
fn stage_a_prepares_h0_supports_without_requesting_a_basis_and_repeats_warm() {
    let mut tx = transaction(false, 1);
    let (atom, cold) = tx.prepare_turn(&vector(3, 0), &vector(0, 4), 1).unwrap();
    assert_eq!(atom.incoming_squared, integer(9));
    assert_eq!(atom.outgoing_squared, integer(16));
    assert_eq!(atom.raw_dot, integer(0));
    assert_eq!(atom.incoming, vector(1, 0));
    assert_eq!(atom.outgoing, vector(0, 1));
    assert_eq!(tx.atom_cache_len(), 1);
    assert_eq!(tx.unit_cache_len(), 2);
    assert_eq!(tx.basis_cache_len(), 0);
    assert_ne!(cold, Work::NONE);
    assert_eq!(
        tx.prepare_turn(&vector(3, 0), &vector(0, 4), 1).unwrap(),
        (atom, Work::NONE)
    );
}

#[test]
fn left_basis_uses_gram_and_has_unit_gram_norm_on_off_diagonal_spd() {
    for owner in [-1, 1] {
        let mut tx = transaction(true, owner);
        let tangent = vector(1, 2);
        let normal = tx.left_unit_normal(&tangent).unwrap().0;
        let mut facts = DensityTransaction::new(tx.metric().clone());
        assert!(facts.gram_dot(&tangent, &normal).0.is_zero());
        assert_eq!(facts.gram_dot(&normal, &normal).0, integer(1));
        let cross = facts.oriented_cross(&tangent, &normal).0;
        assert_eq!(facts.sign(&cross).unwrap().0, 1);
        let covector_quarter_turn = facts.quarter_turn(&tangent, 1).unwrap().0;
        assert_ne!(normal, covector_quarter_turn);
        assert_eq!(tx.left_unit_normal(&tangent).unwrap(), (normal, Work::NONE));
    }
}

#[test]
fn normalized_irrational_supported_vectors_keep_exact_unit_norms() {
    let half_root = scalar("Sqrt(Rational(1, 2))");
    let incoming = Vector2::new(half_root.clone(), half_root.clone());
    let outgoing = Vector2::new(half_root.neg(), half_root);
    let mut tx = transaction(false, 1);
    let atom = tx.prepare_turn(&incoming, &outgoing, 1).unwrap().0;
    assert_eq!(atom.incoming_squared, integer(1));
    assert_eq!(atom.outgoing_squared, integer(1));
    assert_eq!(atom.raw_dot, integer(0));
    assert_eq!(atom.incoming, incoming);
    assert_eq!(atom.outgoing, outgoing);
    assert_eq!(
        tx.left_unit_normal(&atom.incoming).unwrap().0,
        atom.outgoing
    );
}

#[test]
fn cross_gate_precedes_rational_norm_and_normalization_refusals() {
    let irrational = scalar("Sum(Integer(1), Sqrt(Rational(2, 1)))");
    let incoming = Vector2::new(irrational, integer(0));
    let mut tx = transaction(false, 1);
    let error = tx.prepare_turn(&incoming, &vector(0, -1), 1).unwrap_err();
    assert_eq!(
        error.kind,
        TurnFailureKind::Product(TurnProductReason::OwnerTurnMismatch)
    );
    // Накопление failed V2 Work равно сумме реально выполненных ступеней до cross gate.
    let root_two = scalar("Sqrt(Rational(2, 1))");
    let root_three = scalar("Sqrt(Rational(3, 1))");
    let left = Vector2::new(root_two, integer(0));
    let right = Vector2::new(integer(0), root_three);
    let mut facts = DensityTransaction::new(tx.metric().clone());
    let a = facts.gram_dot(&left, &left).1;
    let b = facts.gram_dot(&right, &right).1;
    let c = facts.gram_dot(&left, &right).1;
    let (cross, d) = facts.oriented_cross(&left, &right);
    let e = facts.sign(&cross).unwrap().1;
    let mut priced = transaction(false, 1);
    let failure = priced.prepare_turn(&left, &right, -1).unwrap_err();
    assert_eq!(failure.work, a + b + c + d + e);
    let error = tx.prepare_turn(&incoming, &vector(0, 1), 1).unwrap_err();
    assert_eq!(
        error.kind,
        TurnFailureKind::Product(TurnProductReason::SourceNormsNotRational)
    );
    assert_eq!(tx.atom_cache_len(), 0);
    assert_eq!(tx.unit_cache_len(), 0);
    let error = tx
        .prepare_turn(&vector(0, 0), &vector(0, 1), 1)
        .unwrap_err();
    assert_eq!(
        error.kind,
        TurnFailureKind::Product(TurnProductReason::OwnerTurnMismatch)
    );
}

#[test]
fn collinear_nonzero_orientation_zero_is_accepted_as_python_h0() {
    let mut tx = transaction(false, 1);
    let atom = tx.prepare_turn(&vector(1, 2), &vector(2, 4), 0).unwrap().0;
    assert_eq!(atom.incoming, atom.outgoing);
    assert_eq!(atom.incoming_squared, integer(5));
    assert_eq!(atom.outgoing_squared, integer(20));
    assert_eq!(atom.raw_dot, integer(10));
    assert_eq!(tx.basis_cache_len(), 0);
    assert_eq!(
        tx.prepare_turn(&vector(1, 2), &vector(2, 4), 1)
            .unwrap_err()
            .kind,
        TurnFailureKind::Product(TurnProductReason::OwnerTurnMismatch)
    );
}

#[test]
fn outgoing_normalization_failure_preserves_only_the_successful_incoming_unit() {
    let mut tx = transaction(false, 1);
    let incoming = vector(3, 0);
    let outgoing = vector(0, 0);
    let first = tx.prepare_turn(&incoming, &outgoing, 0).unwrap_err();
    assert_eq!(
        first.kind,
        TurnFailureKind::Product(TurnProductReason::SupportNormNonPositive)
    );
    assert_ne!(first.work, Work::NONE);
    assert_eq!(tx.atom_cache_len(), 0);
    assert_eq!(tx.unit_cache_len(), 1);
    assert_eq!(tx.basis_cache_len(), 0);
    let repeated = tx.prepare_turn(&incoming, &outgoing, 0).unwrap_err();
    assert_eq!(repeated.kind, first.kind);
    assert_eq!(repeated.work, Work::NONE);
    assert_eq!(tx.atom_cache_len(), 0);
    assert_eq!(
        tx.unit_from_squared(&incoming, &integer(9)).unwrap(),
        (vector(1, 0), Work::NONE)
    );
    // Холодный failed Work включает подготовку и нормализацию incoming до отказа outgoing.
    let mut reference = transaction(false, 1);
    let first_work = reference
        .unit_from_squared(&incoming, &integer(9))
        .unwrap()
        .1;
    assert_eq!(first.work, first_work);
}

#[test]
fn normalization_distinguishes_product_nonpositive_from_internal_unsupported_root() {
    let mut tx = transaction(false, 1);
    for n in [-1, 0] {
        assert_eq!(
            tx.unit_from_squared(&vector(1, 0), &integer(n))
                .unwrap_err()
                .kind,
            TurnFailureKind::Product(TurnProductReason::SupportNormNonPositive)
        );
    }
    let squared = scalar("Sum(Integer(1), Sqrt(Rational(2, 1)))");
    let error = tx.unit_from_squared(&vector(1, 0), &squared).unwrap_err();
    assert_eq!(
        error.kind,
        TurnFailureKind::InternalUnsupported(TurnUnsupported::NonRationalNormalizationRoot)
    );
    assert_eq!(tx.unit_cache_len(), 0);
    let tangent = Vector2::new(squared, integer(0));
    assert_eq!(
        tx.left_unit_normal(&tangent).unwrap_err().kind,
        TurnFailureKind::InternalUnsupported(TurnUnsupported::NonRationalNormalizationRoot)
    );
    assert_eq!(tx.basis_cache_len(), 0);
}

#[test]
#[ignore = "Needs CFTUV_TURN_ORACLE_PYTHON and CFTUV_TURN_REPLAY"]
fn live_python_stage_a_unit_left_and_h0_match_supported_value_boundary() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle = Command::new(std::env::var("CFTUV_TURN_ORACLE_PYTHON").expect("oracle Python"))
        .args(["-c", ORACLE])
        .env("PYTHONPATH", repo.join("kernel/src"))
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
    for line in text.lines() {
        if let Some(line) = line.strip_prefix("IN\t") {
            input.push_str(line);
            input.push('\n');
        } else if let Some(line) = line.strip_prefix("OUT\t") {
            expected.push(line.to_owned());
        } else {
            panic!("unknown oracle line: {line}");
        }
    }
    let mut native = Command::new(std::env::var("CFTUV_TURN_REPLAY").expect("built replay"))
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
    let actual: Vec<_> = actual
        .lines()
        .map(|line| line.split("\tV2_WORK:").next().unwrap())
        .collect();
    assert_eq!(actual.len(), expected.len());
    assert!(expected.len() > 1800);
    for (index, (actual, expected)) in actual.iter().zip(&expected).enumerate() {
        assert_eq!(*actual, expected.as_str(), "case{index}");
    }
    println!("{} actual Python StageA/unit/left/H0 value cases:0 mismatch; legacy admission/DAG/memo-state parity outside this prototype", expected.len());
}

const ORACLE: &str = r#"
import sympy as sp
from cftuv_envelope.reference.density_turn import _turn_atom, _huber_density_interpolated_normals
from cftuv_envelope.reference.density_vectors import _density_runtime_vector as V, _density_unit_from_squared, _density_left_unit_normal
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.reference.common import ReferenceGeometryError
from cftuv_envelope.reference.planar_types import exact_quadratic_value
from cftuv_envelope.reference.radical_v2 import RadicalSumV2, canonical_text

def value(expr): return canonical_text(RadicalSumV2.from_terms(exact_quadratic_value(sp.expand(expr)).terms)[0])
def vector(v): return '~'.join(value(e) for e in v.expressions())
def atom(a): return '~'.join(map(value,(a.incoming_squared,a.outgoing_squared,a.raw_dot)))+'|'+vector(a.incoming)+'|'+vector(a.outgoing)
def emit(op,m,left,right,arg,compute):
 global sequence
 ident='case'+str(sequence); sequence+=1
 g=m.gram
 wire=[op,ident,*[str(v.p)+'/'+str(v.q) for v in (g[0][0],g[0][1],g[1][1])],str(m.owner_orientation_sign),*[value(x) for x in (*left.expressions(),*right.expressions())],str(arg)]
 try: answer=compute()
 except ReferenceGeometryError as e: answer='PRODUCT:'+e.outcome.value+':'+str(e)
 print('IN\t'+'\t'.join(wire)); print('OUT\t'+op+'\t'+ident+'\t'+answer)
sequence=0
points=[(0,0),(1,0),(0,1),(2,1),(1,2),(-1,1),(1,-1),(-1,-1),(1+sp.sqrt(2),0),(sp.sqrt(2)/2,sp.sqrt(2)/2)]
for gram in [(1,0,1),(2,sp.Rational(1,3),3),(sp.Rational(9,7),sp.Rational(-2,5),sp.Rational(5,3))]:
 for owner in [-1,1]:
  mat=sp.Matrix([[gram[0],gram[1]],[gram[1],gram[2]]]); inv=mat.inv()
  m=ExactPlanarMetric(tuple(tuple(sp.Rational(mat[i,j]) for j in range(2)) for i in range(2)),tuple(tuple(inv[i,j] for j in range(2)) for i in range(2)),owner)
  for a in points:
   for b in points:
    left,right=V(*a),V(*b)
    for orientation in [-1,0,1]:
     emit('turn',m,left,right,orientation,lambda left=left,right=right,orientation=orientation:'ATOM:'+atom(_turn_atom(m,left,right,orientation)))
  for a in points[:8]:
   left=V(*a)
   emit('left',m,left,V(0,0),'-',lambda left=left:'VECTOR:'+vector(_density_left_unit_normal(m,left)))
  for squared in [sp.Integer(-1),sp.Integer(0),sp.Integer(1),sp.Integer(5),sp.Rational(1,2)]:
   emit('unit',m,V(3,2),V(squared,0),'-',lambda squared=squared:'VECTOR:'+vector(_density_unit_from_squared(V(3,2),squared,m)))
  for orientation,outgoing in [(0,(2,0)),(owner,(0,1)),(-owner,(0,1))]:
   for repeat in range(2):
    emit('h0',m,V(1,0),V(*outgoing),orientation,lambda orientation=orientation,outgoing=outgoing:'VECTORS:'+'|'.join(map(vector,_huber_density_interpolated_normals(m,V(1,0),V(*outgoing),0,orientation))))
"#;
