use std::cmp::Ordering;

use cftuv_density::farey::PrimitiveVector;
use cftuv_density::order::sorted_as_cpython311;
use cftuv_density::order_set::{TupleSet, integer_hash, tuple_hash};
use cftuv_density::IBig;

fn pair(x: i64, y: i64) -> PrimitiveVector {
    PrimitiveVector(IBig::from(x), IBig::from(y))
}
fn sign(value: Ordering) -> i8 {
    match value {
        Ordering::Less => -1,
        Ordering::Equal => 0,
        Ordering::Greater => 1,
    }
}

#[test]
fn integer_and_tuple_hashes_keep_signed_modulus_remapping_and_wrapping() {
    let modulus = (1u64 << 61) - 1;
    assert_eq!(integer_hash(&IBig::from(-1)), -2);
    assert_eq!(integer_hash(&IBig::from(-2)), -2);
    assert_eq!(integer_hash(&IBig::from(modulus)), 0);
    assert_eq!(integer_hash(&-(IBig::from(modulus) + 1u8)), -2);
    assert_eq!(integer_hash(&(IBig::ONE << 200)), 131_072);
    assert_eq!(integer_hash(&-(IBig::ONE << 200)), -131_072);
    assert_eq!(tuple_hash(&pair(0, 0)), -8_458_139_203_682_520_985);
    assert_eq!(tuple_hash(&pair(1, 2)), -3_550_055_125_485_641_917);
    assert_eq!(tuple_hash(&pair(-1, -1)), 960_947_337_673_586_213);
    // Эта пара попадает точно в tuple sentinel UINT64_MAX.
    assert_eq!(
        tuple_hash(&pair(17, -1_555_522_700_513_432_331)),
        1_546_275_796
    );
}

#[test]
fn equal_hashes_are_not_equal_pairs_and_duplicate_add_never_resizes() {
    let modulus = IBig::from((1u64 << 61) - 1);
    let values: Vec<_> = (0..512u32)
        .map(|i| PrimitiveVector(-(&modulus * i) - IBig::ONE, IBig::ONE))
        .collect();
    let hash = tuple_hash(&values[0]);
    assert!(values.iter().all(|value| tuple_hash(value) == hash));
    let mut set = TupleSet::new();
    set.update(values.iter());
    assert_eq!(set.len(), values.len());
    let capacity = set.capacity();
    let order = set.iter().cloned().collect::<Vec<_>>();
    for value in values.iter().rev() {
        assert!(!set.add(value.clone()));
    }
    assert_eq!(set.capacity(), capacity);
    assert_eq!(set.iter().cloned().collect::<Vec<_>>(), order);
}

#[test]
fn tuple_iterable_updates_have_the_same_history_across_chunk_boundaries() {
    let values: Vec<_> = (0..800u32)
        .map(|i| PrimitiveVector(IBig::from(i), -IBig::from(i)))
        .collect();
    let mut whole = TupleSet::new();
    whole.update(values.iter());
    let mut chunked = TupleSet::new();
    for chunk in values.chunks(11) {
        chunked.update(chunk.iter());
        chunked.update(chunk.iter());
    }
    assert_eq!(whole.capacity(), chunked.capacity());
    assert_eq!(
        whole.insertion_indices().collect::<Vec<_>>(),
        chunked.insertion_indices().collect::<Vec<_>>()
    );
}

#[test]
fn large_set_resize_switches_multiplier_only_when_used_exceeds_50000() {
    let mut set = TupleSet::new();
    for i in 0..50_000u32 {
        set.add(PrimitiveVector(IBig::from(i), IBig::ZERO));
    }
    assert_eq!(set.capacity(), 131_072);
    for i in 50_000..78_643u32 {
        set.add(PrimitiveVector(IBig::from(i), IBig::ZERO));
    }
    assert_eq!(set.len(), 78_643);
    assert_eq!(set.capacity(), 262_144);
}

#[test]
fn generic_sort_accepts_noncopy_values_and_stably_keeps_distinct_equal_keys() {
    let items = vec![
        Box::new((2, "a")),
        Box::new((2, "b")),
        Box::new((1, "c")),
        Box::new((1, "d")),
    ];
    let result =
        sorted_as_cpython311(items, |left, right| Ok::<_, ()>(sign(left.0.cmp(&right.0)))).unwrap();
    assert_eq!(
        result.iter().map(|item| item.1).collect::<Vec<_>>(),
        ["c", "d", "a", "b"]
    );
}

#[test]
fn the_failing_comparator_call_is_last_and_preserves_its_generic_error() {
    struct Refusal(u8);
    let mut calls = 0;
    let result = sorted_as_cpython311((0..300u32).rev().collect(), |left, right| {
        calls += 1;
        if calls == 5 {
            Err(Refusal(7))
        } else {
            Ok(sign(left.cmp(right)))
        }
    });
    assert_eq!(calls, 5);
    assert!(matches!(result, Err(Refusal(7))));
    for size in 0..2 {
        let result = sorted_as_cpython311((0..size).collect::<Vec<_>>(), |_, _| {
            Err::<i8, _>(Refusal(0))
        });
        assert!(result.is_ok());
    }
}

/// Oracle stdlib-only: Blender Python запускается как EXE, без Blender GUI.
/// Сравниваются ВСЕ вопросы компаратора, а не только итог и digest.
#[test]
#[ignore = "Needs CFTUV_ORDER_REPLAY, CFTUV_ORDER_ORACLE_311 and CFTUV_ORDER_ORACLE_313"]
fn live_python_311_and_313_match_every_hash_set_slot_sort_question_and_failure() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let replay = std::env::var("CFTUV_ORDER_REPLAY").expect("built order_replay path");
    let repo = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .parent()
        .unwrap();
    let mut first_reference = None;
    for runtime in ["CFTUV_ORDER_ORACLE_311", "CFTUV_ORDER_ORACLE_313"] {
        let python = std::env::var(runtime).expect("Python interpreter path");
        let oracle = Command::new(python)
            .args(["-c", ORACLE])
            .env("CFTUV_ORDER_REPO", repo)
            .output()
            .expect("Python oracle");
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
                expected.push(line.to_string());
            } else {
                panic!("invalid oracle line");
            }
        }
        assert!(expected.len() > 1_000);
        if let Some(previous) = &first_reference {
            assert_eq!(previous, &expected, "Python runtime drift");
        } else {
            first_reference = Some(expected.clone());
        }
        let mut native = Command::new(&replay)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .unwrap();
        let mut stdin = native.stdin.take().unwrap();
        // stdin и stdout могут превышать pipe capacity: читаем и пишем параллельно.
        let writer = std::thread::spawn(move || stdin.write_all(input.as_bytes()));
        let output = native.wait_with_output().unwrap();
        writer.join().unwrap().unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let actual = String::from_utf8(output.stdout).unwrap();
        let actual: Vec<_> = actual.lines().collect();
        assert_eq!(actual.len(), expected.len());
        for (index, (actual, expected)) in actual.iter().zip(&expected).enumerate() {
            assert!(
                actual == expected,
                "{runtime} case {index}: actual {} / expected {}",
                &actual[..actual.len().min(400)],
                &expected[..expected.len().min(400)]
            );
        }
        println!(
            "{runtime}: {} cases, all hashes/slots/capacities/complete comparator traces identical",
            expected.len()
        );
    }
}

const ORACLE: &str = r#"
import importlib.util, os, random, sys
from functools import cmp_to_key
from pathlib import Path
path=Path(os.environ['CFTUV_ORDER_REPO'])/'kernel/src/cftuv_envelope/_cpython311.py'
spec=importlib.util.spec_from_file_location('order_oracle',path)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
random=random.Random(20261012)
p=2**61-1

def listing(items): return ';'.join(map(str,items)) or '-'
def pairs(items): return ';'.join(f'{x},{y}' for x,y in items) or '-'
def emit(command,expected):
    print('IN\t'+command)
    print('OUT\t'+expected)
def hashes(name,values):
    emit('HASH\t'+name+'\t'+listing(values),'HASH\t'+name+'\t'+listing(map(hash,values)))
def tuple_hashes(name,values):
    emit('TUPLE\t'+name+'\t'+pairs(values),'TUPLE\t'+name+'\t'+listing(map(hash,values)))
def set_case(name,phases):
    current=set()
    base=current.__sizeof__()
    results=[]
    for phase in phases:
        current.update(tuple(phase))
        capacity=8 if current.__sizeof__()==base else (current.__sizeof__()-base)//16
        results.append(pairs(current)+'\t'+str(capacity))
    emit('SET\t'+name+'\t'+'|'.join(map(pairs,phases)), 'SET\t'+name+'\t'+'|'.join(results))
def outcome(values,fail_at,sorter=module.sorted_as_cpython311,modulo=None):
    trace=[]
    def compare(left,right):
        trace.append((left[1],right[1]))
        if fail_at and len(trace)==fail_at: raise ValueError('ORDER_COMPARE_REFUSED')
        a,b=left[0],right[0]
        if modulo: a,b=a[0]%modulo,b[0]%modulo
        return (a>b)-(a<b)
    try:
        result=sorter([(value,i) for i,value in enumerate(values)],compare)
        result=('OK',listing(item[1] for item in result))
    except ValueError as error: result=(str(error),'-')
    return '\t'.join([*result,pairs(trace)]),len(trace)
def sort_case(name,values,fail_at=0):
    result,compares=outcome(values,fail_at)
    if sys.version_info[:2]==(3,11):
        direct=lambda values,compare: sorted(values,key=cmp_to_key(compare))
        assert outcome(values,fail_at,direct)==(result,compares)
    emit('SORT\t'+name+'\t'+str(fail_at)+'\t'+listing(values),'SORT\t'+name+'\t'+result)
    return compares
def canonical(name,values,fail_at=0):
    current=set()
    current.update(tuple(values))
    set_order=list(current)
    indices=[values.index(value) for value in set_order]
    result,_=outcome(set_order,fail_at,modulo=7)
    status,order,trace=result.split('\t')
    if order!='-': order=listing(indices[int(at)] for at in order.split(';'))
    if trace!='-': trace=pairs((indices[int(a)],indices[int(b)]) for a,b in (part.split(',') for part in trace.split(';')))
    emit('CANON\t'+name+'\t'+str(fail_at)+'\t'+pairs(values),'CANON\t'+name+'\t'+'\t'.join([listing(indices),status,order,trace]))

values=[0,1,-1,2,-2,p,-p,p+1,-p-1,2**200,-2**250]
values.extend(random.getrandbits(bits)*sign for bits in [61,62,64,100,127,256,521,2048,8192] for sign in [-1,1] for _ in range(8))
hashes('signed-bigint-boundaries',values)
tuple_hashes('pairs',[(-1,-1),(17,-1555522700513432331),*(tuple(random.sample(values,2)) for _ in range(256))])
set_case('empty-duplicate',[[],[(1,2),(1,2)],[(1,2)],[]])
collisions=[(p*i-1,1) for i in range(-128,129)]
set_case('full-hash-collisions',[collisions[:80],collisions[80:],list(reversed(collisions))])
points=[(i*(-1 if i%3==0 else 1),(i*131)^12345) for i in range(80000)]
set_case('large-resize-50000',[points[:50000],points[50000:78642],points[78642:],points[-20:]])
for n in [4,5,18,19,76,77,306,307,1228,1229,4914,4915]:
    points=[(i,-i) for i in range(n)]
    random.shuffle(points)
    set_case(f'resize-boundary-{n}',[points[:n//2],points[n//2:],points[:n//3]])
families={
 'ascending':lambda n:list(range(n)),
 'descending':lambda n:list(range(n-1,-1,-1)),
 'ties':lambda n:[0]*n,
 'random':lambda n:[random.randrange(101) for _ in range(n)],
 'clusters':lambda n:[(i*7//19)%13 for i in range(n)],
 'runs':lambda n:[(i//23)*37+(i%23 if (i//23)%2==0 else 23-i%23) for i in range(n)],
 'saw':lambda n:[(i*17)%31 for i in range(n)],
 'gallop':lambda n:[i%2*10000+i//2 for i in range(n)],
}
for family,make in families.items():
    for n in [0,1,2,3,4,7,15,31,32,33,63,64,65,66,95,127,128,129,257,512,1001,4097]:
        sort_case(f'{family}-{n}',make(n))
for round in range(100):
    n=random.randrange(700)
    values=[random.randrange(1+round%17) for _ in range(n)]
    sort_case(f'random-round-{round}',values)
    for fail_at in [1,2,7,30,71,200,1000]: sort_case(f'random-throw-{round}-{fail_at}',values,fail_at)
values=[random.randrange(13) for _ in range(96)]
count=sort_case('all-failure-positions-base',values)
for fail_at in range(1,count+2): sort_case(f'all-failure-positions-{fail_at}',values,fail_at)
for n in [0,1,2,4,5,19,31,63,64,65,127,129,257]:
    values=[(i*(-1 if i%3==0 else 1),i) for i in range(n)]
    random.shuffle(values)
    for fail_at in [0,1,2,7,30,1000]: canonical(f'canonical-{n}-{fail_at}',values,fail_at)
"#;
