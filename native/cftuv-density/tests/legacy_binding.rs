use cftuv_density::UBig;
use cftuv_density::legacy_binding::LegacyBindingFailure;

#[allow(dead_code)]
#[path = "../examples/legacy_binding_replay.rs"]
mod replay;
use replay::ReplayResult;

const SINGLE: &str = "S!1,1,-5/1~-5/1~C~C~0/1,5/1~5/1~C~C~0/1";

fn fixture(
    op: &str,
    record: &str,
    certs: &str,
    cap: usize,
    throw: usize,
    inside: &str,
    score: &str,
) -> String {
    [
        op.into(),
        "unit".into(),
        cap.to_string(),
        "1".into(),
        "1".into(),
        "5".into(),
        throw.to_string(),
        "1".into(),
        "1".into(),
        inside.into(),
        score.into(),
        "actual".into(),
        "1/1,-2/1;1/1,0/1;1/1,0/1;1/1,2/1".into(),
        record.into(),
        certs.into(),
        "1/1".into(),
        "10/1".into(),
    ]
    .join("\t")
}

#[test]
fn atlas_and_invalid_width_return_before_budget_but_cap_refusal_keeps_upfront_spend() {
    let (result, budget, trace) =
        replay::evaluate_line(&fixture("CERT", "A", "_", 0, 1, "1", "tie"));
    assert!(matches!(result, Ok(ReplayResult::Certificate(None))));
    assert!(budget.is_none());
    assert!(trace.is_empty());
    let invalid = "S!1,1,0/1~0/1~C~C~0/1,0/1~0/1~C~C~0/1";
    let (result, budget, trace) =
        replay::evaluate_line(&fixture("CERT", invalid, "_", 0, 1, "1", "tie"));
    assert!(matches!(result, Err(LegacyBindingFailure::EmptyWindow)));
    assert!(budget.is_none());
    assert!(trace.is_empty());
    let (result, budget, trace) =
        replay::evaluate_line(&fixture("CERT", SINGLE, "_", 0, 1, "1", "tie"));
    assert!(matches!(result, Err(LegacyBindingFailure::Work(_))));
    let budget = budget.unwrap();
    assert_eq!(budget.shell_probes, UBig::ONE);
    assert_eq!(budget.order_steps, UBig::ZERO);
    assert!(trace.is_empty());
}

#[test]
fn tie_has_two_sign_calls_and_larger_slope_wins_without_sorting() {
    let (result, budget, trace) =
        replay::evaluate_line(&fixture("CERT", SINGLE, "_", 65536, 0, "1", "tie"));
    let Ok(ReplayResult::Certificate(Some(c))) = result else {
        panic!("certificate");
    };
    assert_eq!(c.bound_primitive_integer_vector.0.to_string(), "1");
    assert_eq!(c.bound_primitive_integer_vector.1.to_string(), "4");
    assert_eq!(trace.iter().filter(|e| e.starts_with("S:")).count(), 16);
    assert_eq!(budget.unwrap().shell_probes, UBig::ONE);
    let (_, _, asc) = replay::evaluate_line(&fixture("CERT", SINGLE, "_", 65536, 0, "1", "asc"));
    assert_eq!(asc.iter().filter(|e| e.starts_with("S:")).count(), 8);
    let second_sign = trace.iter().position(|e| e.starts_with("S:")).unwrap() + 2;
    let (result, budget, failed) = replay::evaluate_line(&fixture(
        "CERT",
        SINGLE,
        "_",
        65536,
        second_sign,
        "1",
        "tie",
    ));
    assert!(matches!(result, Err(LegacyBindingFailure::Callback(_))));
    assert_eq!(failed.len(), second_sign);
    assert_eq!(budget.unwrap().spent(), UBig::ONE);
}

#[test]
fn all_vectors_resolve_before_proven_predicates_and_callback_refusal_can_precede_false() {
    let recs = format!("{SINGLE}#{SINGLE}");
    let (result, _, trace) =
        replay::evaluate_line(&fixture("FULL", &recs, "1,0,0#1,0,1", 0, 0, "1", "tie"));
    assert!(matches!(result, Ok(ReplayResult::Valid(false))));
    assert_eq!(trace.len(), 2);
    assert!(trace.iter().all(|e| e.starts_with("V:")));
    let (result, _, trace) =
        replay::evaluate_line(&fixture("FULL", &recs, "1,0,0#1,0,1", 0, 2, "1", "tie"));
    assert!(matches!(result, Err(LegacyBindingFailure::Callback(_))));
    assert_eq!(trace.len(), 2);
}

#[test]
fn width_callback_is_late_and_zero_ordinal_is_an_internal_input_error() {
    let input = fixture("CERT", SINGLE, "_", 65536, 0, "1", "tie");
    let (_, _, trace) = replay::evaluate_line(&input);
    assert!(trace.last().unwrap().starts_with("W:"));
    let (result, budget, _) = replay::evaluate_line(&fixture(
        "CERT",
        SINGLE,
        "_",
        65536,
        trace.len(),
        "1",
        "tie",
    ));
    assert!(matches!(result, Err(LegacyBindingFailure::Callback(_))));
    assert_eq!(budget.unwrap().spent(), UBig::ONE);
    let mut fields: Vec<_> = input.split('\t').collect();
    fields[3] = "0";
    let (result, budget, trace) = replay::evaluate_line(&fields.join("\t"));
    assert!(matches!(result, Err(LegacyBindingFailure::InvalidOrdinal)));
    assert_eq!(budget.unwrap().spent(), UBig::ONE);
    assert_eq!(trace.len(), 1);
    assert!(trace[0].starts_with("V:"));
}

#[test]
fn strict_zip_errors_are_late_and_earlier_false_suppresses_them() {
    let (result, _, trace) =
        replay::evaluate_line(&fixture("FULL", SINGLE, "1,0,1#1,0,1", 0, 0, "1", "tie"));
    assert!(matches!(result, Err(LegacyBindingFailure::RecordsShorter)));
    assert!(trace.iter().any(|e| e.starts_with("T:")));
    let (result, _, trace) =
        replay::evaluate_line(&fixture("FULL", SINGLE, "1,0,1#1,0,1", 0, 0, "0", "tie"));
    assert!(matches!(result, Ok(ReplayResult::Valid(false))));
    assert!(!trace.iter().any(|e| e.starts_with("T:")));
    let recs = format!("{SINGLE}#{SINGLE}");
    let (result, _, _) = replay::evaluate_line(&fixture("FULL", &recs, "1,0,1", 0, 0, "1", "tie"));
    assert!(matches!(result, Err(LegacyBindingFailure::RecordsLonger)));
    let (result, _, _) = replay::evaluate_line(&fixture("FULL", &recs, "1,0,1", 0, 0, "0", "tie"));
    assert!(matches!(result, Ok(ReplayResult::Valid(false))));
}

#[test]
#[ignore = "Needs CFTUV_LEGACY_BINDING_ORACLE_PYTHON and CFTUV_LEGACY_BINDING_REPLAY"]
fn actual_python_certificate_and_full_fan_match_all_callbacks_prices_and_late_errors() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let child = Command::new(std::env::var("CFTUV_LEGACY_BINDING_ORACLE_PYTHON").unwrap())
        .arg(repo.join("tools/density_legacy_binding_diff.py"))
        .arg("--emit-fixtures")
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    println!("oracle_pid={}", child.id());
    let oracle = child.wait_with_output().unwrap();
    assert!(
        oracle.status.success(),
        "{}",
        String::from_utf8_lossy(&oracle.stderr)
    );
    let oracle = String::from_utf8(oracle.stdout).unwrap();
    let mut input = String::new();
    let mut expected = Vec::new();
    let mut names = Vec::new();
    for line in oracle.lines() {
        if let Some(line) = line.strip_prefix("IN\t") {
            names.push(line.split('\t').nth(1).unwrap().to_owned());
            input.push_str(line);
            input.push('\n');
        } else if let Some(line) = line.strip_prefix("OUT\t") {
            expected.push(line.to_owned());
        } else {
            panic!("oracle output");
        }
    }
    assert!(expected.len() > 1000);
    let mut native = Command::new(std::env::var("CFTUV_LEGACY_BINDING_REPLAY").unwrap())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    println!("replay_pid={}", native.id());
    let mut stdin = native.stdin.take().unwrap();
    let writer = std::thread::spawn(move || stdin.write_all(input.as_bytes()));
    let result = native.wait_with_output().unwrap();
    writer.join().unwrap().unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let actual = String::from_utf8(result.stdout).unwrap();
    let rows: Vec<_> = actual.lines().collect();
    assert_eq!(rows.len(), expected.len());
    for (row, (actual, expected)) in rows.iter().zip(&expected).enumerate() {
        assert_eq!(*actual, expected, "case {row} {}", names[row]);
    }
    println!(
        "{} actual Python legacy binding cases: 0 mismatch in prepared certificate values, budgets, callback order and late strictzip errors",
        expected.len()
    );
}
