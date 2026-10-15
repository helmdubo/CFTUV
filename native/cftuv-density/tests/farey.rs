use std::cmp::Ordering;

use cftuv_density::farey::{
    Endpoint, FareyFailure, FareyOrdering, FareyPredicates, PREPARATION_WORK_CAP, PrimitiveVector,
    SEARCH_WORK_CAP, SearchMode, SingleChartWindow, WorkBudget, best_fan, farey_shell_candidates,
    search, strict_numerator_bounds,
};
use cftuv_density::{IBig, Rational, UBig};

fn rational(n: i64, d: u64) -> Rational {
    Rational::new(IBig::from(n), UBig::from(d)).unwrap()
}
fn vector(x: i64, y: i64) -> PrimitiveVector {
    PrimitiveVector(IBig::from(x), IBig::from(y))
}
fn window(values: [(i64, u64); 4]) -> SingleChartWindow {
    SingleChartWindow {
        use_x: true,
        denominator_sign: 1,
        sealed: values.map(|(n, d)| rational(n, d)),
    }
}

#[derive(Default)]
struct Predicates {
    log: Vec<String>,
    fail_compare: bool,
    valid_local: bool,
}

impl FareyPredicates for Predicates {
    type Error = &'static str;
    fn local_candidate(
        &mut self,
        ordinal: usize,
        candidate: &PrimitiveVector,
    ) -> Result<bool, Self::Error> {
        self.log
            .push(format!("local:{ordinal}:{},{}", candidate.0, candidate.1));
        Ok(self.valid_local)
    }
    fn candidate_compare(
        &mut self,
        ordinal: usize,
        left: &PrimitiveVector,
        right: &PrimitiveVector,
    ) -> Result<i8, Self::Error> {
        self.log.push(format!("compare:{ordinal}"));
        if self.fail_compare {
            return Err("comparison refused");
        }
        Ok(match left.1.cmp(&right.1) {
            Ordering::Less => -1,
            Ordering::Equal => 0,
            Ordering::Greater => 1,
        })
    }
    fn edge_valid(
        &mut self,
        ordinal: usize,
        _: Endpoint<'_>,
        _: Endpoint<'_>,
    ) -> Result<bool, Self::Error> {
        self.log.push(format!("edge:{ordinal}"));
        Ok(true)
    }
}

// Только тестовый внешний порядок; библиотека не выбирает ни эту модель,
// ни иной сортировщик вместо обязательных caller callbacks.
struct TestOrdering;
impl FareyOrdering<Predicates> for TestOrdering {
    fn iteration_order(
        &mut self,
        _: usize,
        items: &[PrimitiveVector],
    ) -> Result<Vec<usize>, &'static str> {
        Ok((0..items.len()).collect())
    }
    fn sort(
        &mut self,
        ordinal: usize,
        items: &[PrimitiveVector],
        predicates: &mut Predicates,
    ) -> Result<Vec<usize>, &'static str> {
        let mut indices: Vec<_> = (0..items.len()).collect();
        for i in 1..indices.len() {
            let mut j = i;
            while j > 0
                && predicates.candidate_compare(
                    ordinal,
                    &items[indices[j]],
                    &items[indices[j - 1]],
                )? < 0
            {
                indices.swap(j, j - 1);
                j -= 1;
            }
        }
        Ok(indices)
    }
}

#[test]
fn strict_bounds_use_floor_and_ceil_for_negative_and_arbitrarily_large_values() {
    assert_eq!(
        strict_numerator_bounds(&rational(-1, 3), &rational(2, 3), &UBig::from(6u8)),
        (IBig::from(-1), IBig::from(3))
    );
    assert_eq!(
        strict_numerator_bounds(&rational(-1, 3), &rational(-1, 7), &UBig::from(8u8)),
        (IBig::from(-2), IBig::from(-2))
    );
    let n: IBig = "10000000000000000000000000000000000000000000000000"
        .parse()
        .unwrap();
    let lower = Rational::integer(-n.clone());
    let upper = Rational::integer(n.clone());
    assert_eq!(
        strict_numerator_bounds(&lower, &upper, &UBig::from(2u8)),
        (-(&n * 2u8) + IBig::ONE, &n * 2u8 - IBig::ONE)
    );
}

#[test]
fn budget_factories_and_increment_before_throw_match_the_exact_search_contract() {
    assert_eq!(WorkBudget::search().cap, UBig::from(SEARCH_WORK_CAP));
    assert_eq!(
        WorkBudget::preparation().cap,
        UBig::from(PREPARATION_WORK_CAP)
    );
    let mut budget = WorkBudget::new(UBig::ONE);
    budget.spend_shell_probes(UBig::ONE).unwrap();
    let error = budget.spend_order_steps(UBig::ONE).unwrap_err();
    assert_eq!(error.0, budget);
    assert_eq!(budget.spent(), UBig::from(2u8));
    assert_eq!(
        error.to_string(),
        "DENSITY_RATIONAL_AUTHORITY_EXHAUSTED cap=1 shell_probes=1 order_steps=1"
    );
    let huge: UBig = "100000000000000000000000000000000000000000"
        .parse()
        .unwrap();
    budget.spend_shell_probes(huge.clone()).unwrap_err();
    assert_eq!(budget.shell_probes, huge + UBig::ONE);
}

#[test]
fn shell_charges_input_span_before_gcd_and_asks_only_outer_candidates() {
    let mut predicates = Predicates {
        valid_local: true,
        ..Default::default()
    };
    let mut budget = WorkBudget::search();
    let shell = window([(-1, 1), (-1, 2), (1, 2), (1, 1)]);
    let result =
        farey_shell_candidates(&shell, 1, &UBig::from(4u8), &mut budget, &mut predicates).unwrap();
    assert_eq!(
        result,
        vec![vector(4, -3), vector(4, -1), vector(4, 1), vector(4, 3)]
    );
    assert_eq!(budget.shell_probes, UBig::from(7u8));
    assert_eq!(predicates.log, ["local:1:4,-3", "local:1:4,3"]);
    let mut limited = WorkBudget::new(UBig::from(6u8));
    predicates.log.clear();
    assert!(matches!(
        farey_shell_candidates(&shell, 1, &UBig::from(4u8), &mut limited, &mut predicates),
        Err(FareyFailure::Work(_))
    ));
    assert_eq!(limited.shell_probes, UBig::from(7u8));
    assert!(predicates.log.is_empty());
}

#[test]
fn empty_shell_is_one_probe_and_chart_sign_changes_only_coordinates() {
    let mut predicates = Predicates::default();
    let mut budget = WorkBudget::search();
    let empty = window([(0, 1), (1, 20), (1, 10), (1, 8)]);
    assert!(
        farey_shell_candidates(&empty, 1, &UBig::ONE, &mut budget, &mut predicates)
            .unwrap()
            .is_empty()
    );
    assert_eq!(budget.shell_probes, UBig::ONE);
    assert!(
        farey_shell_candidates(&empty, 1, &UBig::ZERO, &mut budget, &mut predicates)
            .unwrap()
            .is_empty()
    );
    assert_eq!(budget.shell_probes, UBig::from(2u8));
    let mut flipped = window([(-1, 1), (-1, 1), (1, 1), (1, 1)]);
    flipped.use_x = false;
    flipped.denominator_sign = -1;
    assert_eq!(
        farey_shell_candidates(&flipped, 1, &UBig::from(2u8), &mut budget, &mut predicates)
            .unwrap(),
        vec![vector(1, -2), vector(-1, -2)]
    );
    assert!(predicates.log.is_empty());
}

#[test]
fn sort_input_is_fully_charged_before_any_comparison_and_refusal_precedes_visits() {
    let candidates = vec![vec![vector(2, 1), vector(3, 2)], vec![vector(1, 2)]];
    let mut predicates = Predicates {
        fail_compare: true,
        ..Default::default()
    };
    let mut ordering = TestOrdering;
    let mut budget = WorkBudget::search();
    assert_eq!(
        best_fan(&candidates, &mut budget, &mut predicates, &mut ordering),
        Err(FareyFailure::Predicate("comparison refused"))
    );
    assert_eq!(budget.order_steps, UBig::from(3u8));
    assert_eq!(predicates.log, ["compare:1"]);
    let mut budget = WorkBudget::new(UBig::from(2u8));
    predicates.log.clear();
    assert!(matches!(
        best_fan(&candidates, &mut budget, &mut predicates, &mut ordering),
        Err(FareyFailure::Work(_))
    ));
    assert!(predicates.log.is_empty());
    assert_eq!(budget.order_steps, UBig::from(3u8));
}

#[test]
fn previous_counts_and_every_backtracking_edge_have_the_oracle_prices() {
    let mut predicates = Predicates::default();
    let mut ordering = TestOrdering;
    let mut budget = WorkBudget::search();
    let result = search(
        SearchMode::SingleChart,
        &[window([(0, 1), (0, 1), (1, 1), (1, 1)])],
        &UBig::from(5u8),
        &mut budget,
        &mut predicates,
        &mut ordering,
    )
    .unwrap();
    assert_eq!(result.height, Some(UBig::from(2u8)));
    assert_eq!(result.fan, Some(vec![vector(2, 1)]));
    assert_eq!(result.previous_counts, [0]);
    assert_eq!(result.counts, [1]);
    assert_eq!(budget.shell_probes, UBig::from(2u8));
    assert_eq!(budget.order_steps, UBig::from(2u8));
    assert_eq!(predicates.log, ["edge:1", "edge:2"]);
}

#[test]
fn atlas_is_explicitly_not_ported_before_budget_or_callbacks() {
    let mut predicates = Predicates::default();
    let mut budget = WorkBudget::search();
    assert_eq!(
        search(
            SearchMode::Atlas,
            &[],
            &UBig::ONE,
            &mut budget,
            &mut predicates,
            &mut TestOrdering
        ),
        Err(FareyFailure::AtlasNotPorted)
    );
    assert_eq!(budget.spent(), UBig::ZERO);
    assert!(predicates.log.is_empty());
}

/// Внешний дифференциал: НАСТОЯЩИЙ Python `_search/_best_fan`, его set order,
/// CPython311 comparator-журнал и geometry callbacks. Никакой копии алгоритма.
#[test]
#[ignore = "Needs CFTUV_DENSITY_ORACLE_PYTHON and the built CFTUV_FAREY_REPLAY example"]
fn live_python_reference_search_replays_every_callback_budget_and_refusal() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let python = std::env::var("CFTUV_DENSITY_ORACLE_PYTHON").expect("oracle Python path");
    let replay = std::env::var("CFTUV_FAREY_REPLAY").expect("farey_replay example path");
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let oracle = Command::new(python)
        .args(["-c", ORACLE])
        .env("PYTHONPATH", repo.join("kernel/src"))
        .current_dir(repo)
        .output()
        .expect("run Python oracle");
    assert!(
        oracle.status.success(),
        "{}",
        String::from_utf8_lossy(&oracle.stderr)
    );
    let mut native = Command::new(replay)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .expect("run replay");
    native
        .stdin
        .take()
        .unwrap()
        .write_all(&oracle.stdout)
        .unwrap();
    let result = native.wait_with_output().unwrap();
    assert!(
        result.status.success(),
        "{}",
        String::from_utf8_lossy(&result.stderr)
    );
    let text = String::from_utf8(result.stdout).unwrap();
    assert_eq!(text.lines().count(), 76);
    assert!(text.contains("callback_compare_refused"));
    assert!(text.contains("callback_local_refused"));
    assert!(text.contains("callback_edge_refused"));
    assert!(text.contains("DENSITY_RATIONAL_AUTHORITY_EXHAUSTED"));
    assert!(text.contains("multi-backtrack"));
    println!(
        "76 live Python search cases: all outputs, budgets and callback/refusal sequences identical"
    );
}

const ORACLE: &str = r#"
import sys
from fractions import Fraction as F
import sympy as sp
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference.metric import ExactPlanarMetric
from cftuv_envelope.contracts.analysis import TurnOrientation

IDENTITY=((sp.Integer(1),sp.Integer(0)),(sp.Integer(0),sp.Integer(1)))
old_local, old_compare = fan._local_candidate, fan._candidate_compare
old_sort, old_cross, old_subturn = fan.sorted_as_cpython311, fan._oriented_cross, fan._subturn
expected_orientation=1

def case(name, cap, stop, windows, q=3, failure=None):
    events=[]
    counts={'L':0,'C':0,'E':0}
    metric=ExactPlanarMetric(IDENTITY,IDENTITY,1)
    normals=[fan._vector(1,0,metric)]
    if len(windows)==1:
        normals.append(fan._vector(sp.Rational(4,5),sp.Rational(3,5),metric))
    elif len(windows)==2:
        normals.extend([fan._vector(sp.sqrt(3)/2,sp.Rational(1,2),metric),fan._vector(sp.Rational(1,2),sp.sqrt(3)/2,metric)])
    normals.append(fan._vector(0,1,metric))
    ideal=fan._IdealTuple(normals)
    ideal.metric=metric
    ideal.window_law='VORONOI_OF_NEIGHBOUR_MIDPOINTS'
    ideal.band_cache={}
    ideal.band_orientation=None
    def vtext(vector):
        if vector is ideal[0]: return 'IN'
        if vector is ideal[-1]: return 'OUT'
        if hasattr(vector,'expressions'): vector=vector.expressions()
        return f'{int(vector[0])},{int(vector[1])}'
    def vlist(items): return ';'.join(map(vtext,items)) or '-'
    def emit(kind, inputs, compute):
        counts[kind]+=1
        if failure and failure==(kind,counts[kind]):
            message={'L':'callback_local_refused','C':'callback_compare_refused','E':'callback_edge_refused'}[kind]
            events.append('\t'.join([kind,*map(str,inputs),'!'+message]))
            raise ValueError(message)
        try: value=compute()
        except Exception as error:
            events.append('\t'.join([kind,*map(str,inputs),'!'+str(error)]))
            raise
        events.append('\t'.join([kind,*map(str,inputs),str(int(value))]))
        return value
    def local(metric,ideal,ordinal,vector,q):
        return emit('L',[ordinal,vtext(vector)],lambda:old_local(metric,ideal,ordinal,vector,q))
    def compare_factory(metric,ideal,ordinal):
        original=old_compare(metric,ideal,ordinal)
        def compare(left,right):
            return emit('C',[ordinal,vtext(left),vtext(right)],lambda:original(left,right))
        compare.ordinal=ordinal
        return compare
    def sorter(items,compare):
        items=list(items)
        events.append('\t'.join(['I',str(compare.ordinal),vlist(items)]))
        answer=old_sort(items,compare)
        events.append('\t'.join(['S',str(compare.ordinal),vlist(answer)]))
        return answer
    def cross(metric,left,right):
        value=old_cross(metric,left,right)
        caller=sys._getframe(1)
        if caller.f_code.co_name=='visit' and fan._sign(value,metric)!=1:
            emit('E',[caller.f_locals['ordinal']+1,vtext(left),vtext(right)],lambda:False)
        return value
    def subturn(metric,left,right,q):
        caller=sys._getframe(1)
        if caller.f_code.co_name=='visit':
            return emit('E',[caller.f_locals['ordinal']+1,vtext(left),vtext(right)],lambda:old_subturn(metric,left,right,q))
        return old_subturn(metric,left,right,q)
    fan._local_candidate=local
    fan._candidate_compare=compare_factory
    fan.sorted_as_cpython311=sorter
    fan._oriented_cross=cross
    fan._subturn=subturn
    records=[(use_x,sign,None,None) for use_x,sign,interval in windows]
    intervals=[interval for _,_,interval in windows]
    budget=fan._work_budget(cap)
    try:
        height,answer,previous,total=fan._search(metric,ideal,TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,q,records,intervals,stop,budget)
        fmt_counts=lambda values: ','.join(map(str,values)) or '-'
        result='\t'.join(['OK',str(height) if height is not None else '-',vlist(answer) if answer is not None else '-',fmt_counts(previous),fmt_counts(total),str(budget.shell_probes),str(budget.order_steps)])
    except Exception as error:
        result='\t'.join(['ERR',str(error),str(budget.shell_probes),str(budget.order_steps)])
    print('\t'.join(['CASE',name,str(cap),str(stop),'single',str(len(windows))]))
    for use_x,sign,interval in windows:
        print('\t'.join(['W',str(int(use_x)),str(sign),*[f'{v.numerator}/{v.denominator}' for v in interval]]))
    print('TRACE')
    print('\n'.join(events))
    print('EXPECT\t'+result)
    print('END')

single=[(True,1,[F(1,4),F(2,5),F(4,5),F(1)])]
empty=[(True,1,[F(0),F(1,20),F(1,10),F(1,8)])]
multi=[(True,1,[F(1,3),F(1,2),F(2,3),F(3,4)]),(False,1,[F(1,3),F(1,2),F(2,3),F(3,4)])]
case('empty-heights',131072,5,empty)
case('empty-refusal',3,20,empty)
case('success-after-previous-height',131072,12,single)
case('multi-backtrack',131072,10,multi,6)
case('local-refusal',131072,12,single,failure=('L',1))
case('compare-refusal',131072,12,single,failure=('C',1))
case('edge-refusal',131072,12,single,failure=('E',1))
case('empty-records-final-edge',131072,2,[],2)
case('zero-stop',0,0,single)
case('span-before-loop',0,3,[(True,1,[F(-10**50),F(-1),F(1),F(10**50)])])
case('negative-slope-and-sign',131072,8,[(False,-1,[F(-3,4),F(-2,3),F(-1,3),F(-1,4)])],2)
case('equal-inner-boundaries',131072,8,[(True,1,[F(1,4),F(1,2),F(1,2),F(3,4)])],3)
for cap in range(32):
    case(f'cap-single-{cap}',cap,12,single)
for stop in range(16):
    case(f'multi-stop-{stop}',131072,stop,multi,6)
for at in range(1,9):
    case(f'compare-refusal-{at}',131072,10,multi,6,('C',at))
for at in range(1,9):
    case(f'edge-refusal-{at}',131072,10,multi,6,('E',at))
"#;
