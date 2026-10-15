use cftuv_density::UBig;
use cftuv_density::fan_compile::FanCompileFailure;
use cftuv_density::fan_prepare::{FanPrepareFailure, PrepareInvalid};

#[allow(dead_code)]
#[path = "../examples/fan_compile_replay.rs"]
mod replay;

fn line(
    name: &str,
    record: &str,
    throw: usize,
    box_cap: usize,
    search_cap: usize,
    height: usize,
) -> String {
    [
        name.into(),
        "65536".into(),
        search_cap.to_string(),
        box_cap.to_string(),
        height.to_string(),
        "5".into(),
        "1".into(),
        throw.to_string(),
        "1".into(),
        "1".into(),
        "1".into(),
        "1".into(),
        "false".into(),
        "1/1,-3/4;1/1,3/4".into(),
        "0/1".into(),
        "1/1,-2/1;1/1,0/1;1/1,2/1".into(),
        record.into(),
    ]
    .join("\t")
}
const LOCAL: &str = "1,1,-1/1~-1/1~C~C~0/1,1/1~1/1~C~C~0/1";
fn single() -> String {
    format!("S!{LOCAL}")
}
fn atlas() -> String {
    format!("A!0!0,1,1,-1/1~-1/1~C~C~0/1,1/1~1/1~C~C~0/1,0,0,1,Y!{LOCAL}")
}

#[test]
fn observer_hook_is_late_for_center_error_invalid_gap_and_zero_cap() {
    for throw in [1, 2] {
        let (report, trace) =
            replay::evaluate_line(&line("center-error", &single(), throw, 96, 131072, 10));
        assert!(matches!(
            report.result,
            Err(FanCompileFailure::Preparation(FanPrepareFailure::Callback(
                _
            )))
        ));
        assert!(report.budgets.preparation.is_none());
        assert!(report.budgets.search.is_none());
        assert_eq!(trace.len(), throw);
    }
    let (report, _) = replay::evaluate_line(&line(
        "invalid-gap",
        "S!1,1,0/1~0/1~C~C~0/1,1/1~1/1~C~C~0/1",
        0,
        96,
        131072,
        10,
    ));
    assert!(matches!(
        report.result,
        Err(FanCompileFailure::Preparation(FanPrepareFailure::Invalid(
            PrepareInvalid::IdealOutsideWindow
        )))
    ));
    assert!(report.budgets.preparation.is_none());
    let (report, trace) = replay::evaluate_line(&line("cap-zero", &single(), 0, 0, 131072, 10));
    assert!(matches!(
        report.result,
        Err(FanCompileFailure::Preparation(
            FanPrepareFailure::TerminationBoxesExhausted { halvings: 0, .. }
        ))
    ));
    assert_eq!(report.budgets.preparation.unwrap().spent(), UBig::ZERO);
    assert!(report.budgets.search.is_none());
    assert_eq!(trace.len(), 3);
    assert!(trace[2].starts_with("F:"));
}

#[test]
fn segment_failure_precedes_search_budget_and_success_keeps_prepared_authority_fields() {
    let (report, trace) =
        replay::evaluate_line(&line("atlas-success", &atlas(), 0, 96, 131072, 10));
    let a = report.result.unwrap();
    assert_eq!(a.minimal_common_height, UBig::ONE);
    assert_eq!(a.previous_counts, [0]);
    assert_eq!(a.counts, [1]);
    assert_eq!(a.ordinal_windows.len(), 1);
    let last_local = trace
        .iter()
        .rposition(|entry| entry.starts_with("L:"))
        .unwrap();
    let (report, _) = replay::evaluate_line(&line(
        "segments-refused",
        &atlas(),
        last_local + 1,
        96,
        131072,
        10,
    ));
    assert!(matches!(report.result, Err(FanCompileFailure::Segments(_))));
    assert!(report.budgets.preparation.is_some());
    assert!(report.budgets.search.is_none());
}

#[test]
fn zero_height_and_search_refusal_have_search_budget_after_successful_preparation() {
    let (report, _) = replay::evaluate_line(&line("height-zero", &single(), 0, 96, 131072, 0));
    assert!(matches!(
        report.result,
        Err(FanCompileFailure::RationalAuthorityExhausted)
    ));
    assert_eq!(
        report.budgets.preparation.unwrap().order_steps,
        UBig::from(2u8)
    );
    assert_eq!(report.budgets.search.unwrap().spent(), UBig::ZERO);
    let (report, _) = replay::evaluate_line(&line("search-zero", &single(), 0, 96, 0, 10));
    assert!(matches!(report.result, Err(FanCompileFailure::Search(_))));
    assert_eq!(report.budgets.search.unwrap().shell_probes, UBig::ONE);
}

#[test]
#[ignore = "Needs CFTUV_FAN_COMPILE_ORACLE_PYTHON and CFTUV_FAN_COMPILE_REPLAY"]
fn actual_python_controller_matches_authority_values_budgets_and_full_callback_order() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let child = Command::new(std::env::var("CFTUV_FAN_COMPILE_ORACLE_PYTHON").unwrap())
        .arg(repo.join("tools/density_fan_compile_diff.py"))
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
            names.push(line.split('\t').next().unwrap().to_owned());
            input.push_str(line);
            input.push('\n');
        } else if let Some(line) = line.strip_prefix("OUT\t") {
            expected.push(line.to_owned());
        } else {
            panic!("oracle output")
        }
    }
    assert!(expected.len() > 500);
    let mut native = Command::new(std::env::var("CFTUV_FAN_COMPILE_REPLAY").unwrap())
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
        "{} actual Python prepared controller cases: 0 mismatch in authority values, both observed budgets, counts and full callback/failure order",
        expected.len()
    );
}
