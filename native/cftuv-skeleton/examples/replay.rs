//! Replay of real `build_skeleton` calls, Python-free: `cargo run --release [--features profile] --example replay -p cftuv-skeleton -- <polygons.txt> [repeat] [--out digests.txt] [--only name,name] [--top N]`.
//!
//! The file holds one call per block: `polygon <name> <mesh> level_budget <n> cap <c> dense <0|1> patch <p>`, one `loop x,y;... | n/d;...` per loop (points, then the squared speed of every
//! edge), one `fan x,y | nx,ny,n/d;...` per vertex fan, `end`. A call is made the way the PyO3 layer makes it for a cold preparation: a fresh session (empty canonicalization memory, empty product
//! memo), a BOUNDED budget of the cap with six zero articles, the options of the block. Every call runs `repeat` times (the best time is kept) and answers a DIGEST of everything observable:
//! the result (nodes with their canonical terms, counters, proof obligations), the sign counters, the six articles, the superlevel string, the memory log and a refusal. Two builds that
//! print the same digests answer alike, bit for bit, for these calls. `--out` writes `name digest` lines to compare with `diff`.

use std::fmt::Write as _;
use std::time::Instant;

#[global_allocator]
static GLOBAL: mimalloc::MiMalloc = mimalloc::MiMalloc;

use cftuv_core::exact::ExactCtx;
use cftuv_core::num::IBig;
use cftuv_core::rat::Rat;
use cftuv_core::session::{CostRun, Session};
use cftuv_core::sqrt_sum::{SignCounts, SqrtSum};
use cftuv_skeleton::builder::BuilderOptions;
use cftuv_skeleton::polygon::{FanSupport, Loop, Polygon, VertexFan};
use cftuv_skeleton::profile::{reset, reset_stats, take, take_stats, PHASES, STATS};
use cftuv_skeleton::skeleton::Skeleton;
use cftuv_skeleton::time::EventTime;
use cftuv_skeleton::transaction::build_skeleton;

fn rat(text: &str) -> Rat {
    let (numerator, denominator) = text.split_once('/').expect("a speed is n/d");
    Rat::new(numerator.parse::<IBig>().expect("a numerator"), denominator.parse::<IBig>().expect("a denominator")).expect("a denominator that is not zero")
}

fn point(text: &str) -> (i64, i64) {
    let (x, y) = text.split_once(',').expect("a point is x,y");
    (x.parse().expect("an integer x"), y.parse().expect("an integer y"))
}

struct Fnv(u64);

impl std::fmt::Write for Fnv {
    fn write_str(&mut self, text: &str) -> std::fmt::Result {
        for byte in text.bytes() {
            self.0 ^= u64::from(byte);
            self.0 = self.0.wrapping_mul(0x0000_0100_0000_01b3);
        }
        Ok(())
    }
}

fn sum(out: &mut Fnv, value: &SqrtSum) {
    for term in value.terms() {
        let _ = write!(out, "[{:?}*{:?}]", term.radicand, term.coef);
    }
    out.write_str("|").ok();
}

fn time(out: &mut Fnv, value: &EventTime) {
    let _ = write!(out, "T{:?}/", value.dividend);
    sum(out, &value.divisor);
}

fn digest_of(result: &Result<Skeleton, cftuv_skeleton::error::SkelError>, counts: &SignCounts, articles: [u64; 6], log: &[cftuv_canon::MemOp], superlevel: &str) -> u64 {
    let mut out = Fnv(0xcbf2_9ce4_8422_2325);
    match result {
        Ok(skeleton) => {
            let _ = write!(out, "OK {:?} levels={} status={:?} counters={:?}", skeleton.outcome, skeleton.levels, skeleton.proof_status, skeleton.counters);
            for node in &skeleton.nodes {
                let _ = write!(out, "N{:?}", node.kind);
                time(&mut out, &node.time);
                sum(&mut out, &node.point.x);
                sum(&mut out, &node.point.y);
                let _ = write!(out, "{:?}{}{:?}{:?}", node.participants, node.converging_vertices, node.kinds, node.incidences);
            }
            for obligation in &skeleton.proof_obligations {
                let _ = write!(out, "O{:?}{:?}{:?}{:?}{:?}{:?}", obligation.cause, obligation.disposition, obligation.vertex_ids, obligation.participant_edge_keys, obligation.target_edge_keys, obligation.event_kind);
                time(&mut out, &obligation.level);
            }
        }
        Err(error) => {
            let _ = write!(out, "ERR {error:?}");
        }
    }
    let _ = write!(out, "C{:?}A{:?}S{superlevel}L{:?}", counts.as_array(), articles, log);
    out.0
}

struct Call {
    name: String,
    mesh: String,
    patch: i64,
    limit: i64,
    cap: u64,
    dense: bool,
    polygon: Polygon,
}

fn parse(text: &str) -> Vec<Call> {
    let mut calls = Vec::new();
    let (mut loops, mut fans): (Vec<Loop>, Vec<VertexFan>) = (Vec::new(), Vec::new());
    let mut header: Vec<String> = Vec::new();
    for line in text.lines() {
        let mut words = line.splitn(2, ' ');
        match (words.next(), words.next()) {
            (Some("polygon"), Some(rest)) => header = rest.split(' ').map(str::to_string).collect(),
            (Some("loop"), Some(rest)) => {
                let (points, speeds) = rest.split_once(" | ").expect("points | speeds");
                loops.push(Loop { points: points.split(';').map(point).collect(), speeds: speeds.split(';').map(rat).collect() });
            }
            (Some("fan"), Some(rest)) => {
                let (at, supports) = rest.split_once(" | ").expect("point | supports");
                let supports = supports
                    .split(';')
                    .map(|support| {
                        let parts: Vec<&str> = support.splitn(3, ',').collect();
                        FanSupport { normal: (parts[0].parse().unwrap(), parts[1].parse().unwrap()), speed: rat(parts[2]) }
                    })
                    .collect();
                fans.push(VertexFan { point: point(at), supports });
            }
            (Some("end"), _) => {
                let polygon = Polygon::new(std::mem::take(&mut loops), std::mem::take(&mut fans)).expect("a polygon of the corpus");
                calls.push(Call {
                    name: header[0].clone(),
                    mesh: header[1].clone(),
                    limit: header[3].parse().expect("a level budget"),
                    cap: header[5].parse().expect("a cap"),
                    dense: header[7] == "1",
                    patch: header[9].parse().expect("a patch"),
                    polygon,
                });
            }
            _ => {}
        }
    }
    calls
}

/// A sampling profiler of this process on Windows, from the OS alone (`kernel32`, `dbghelp`): a thread suspends the main one every millisecond, walks its stack with the unwind tables
/// and counts the functions (self and inclusive) and the lines of the top frame. `--sample` prints the tables after the run; nothing is installed and nothing leaves the process.
#[cfg(windows)]
mod sampler {
    use std::collections::HashMap;
    use std::ffi::c_void;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::sync::Arc;

    type Handle = *mut c_void;

    #[repr(C, align(16))]
    struct Context([u8; 1232]);

    const FLAGS: usize = 0x30;
    const RSP: usize = 0x98;
    const RIP: usize = 0xF8;

    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentProcess() -> Handle;
        fn GetCurrentThread() -> Handle;
        fn DuplicateHandle(source_process: Handle, source: Handle, target_process: Handle, target: *mut Handle, access: u32, inherit: i32, options: u32) -> i32;
        fn SuspendThread(thread: Handle) -> u32;
        fn ResumeThread(thread: Handle) -> u32;
        fn GetThreadContext(thread: Handle, context: *mut Context) -> i32;
        fn RtlLookupFunctionEntry(pc: u64, base: *mut u64, history: *mut c_void) -> *mut c_void;
        fn RtlVirtualUnwind(kind: u32, base: u64, pc: u64, entry: *mut c_void, context: *mut Context, data: *mut *mut c_void, establisher: *mut u64, pointers: *mut c_void) -> *mut c_void;
    }

    #[link(name = "dbghelp")]
    extern "system" {
        fn SymInitialize(process: Handle, path: *const u8, invade: i32) -> i32;
        fn SymSetOptions(options: u32) -> u32;
        fn SymFromAddr(process: Handle, address: u64, displacement: *mut u64, symbol: *mut u8) -> i32;
        fn SymGetLineFromAddr64(process: Handle, address: u64, displacement: *mut u32, line: *mut Line) -> i32;
    }

    #[repr(C)]
    struct Line {
        size: u32,
        key: *mut c_void,
        number: u32,
        file: *const u8,
        address: u64,
    }

    fn read(context: &Context, offset: usize) -> u64 {
        u64::from_le_bytes(context.0[offset..offset + 8].try_into().unwrap())
    }

    fn write(context: &mut Context, offset: usize, value: u64) {
        context.0[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }

    pub struct Sampler {
        stop: Arc<AtomicBool>,
        worker: Option<std::thread::JoinHandle<Vec<Vec<u64>>>>,
    }

    impl Sampler {
        pub fn start(period_micros: u64) -> Sampler {
            let mut target: Handle = std::ptr::null_mut();
            unsafe {
                let process = GetCurrentProcess();
                assert!(DuplicateHandle(process, GetCurrentThread(), process, &mut target, 0x1FFFFF, 0, 0) != 0, "DuplicateHandle");
            }
            let target = target as usize;
            let stop = Arc::new(AtomicBool::new(false));
            let flag = Arc::clone(&stop);
            let worker = std::thread::spawn(move || {
                let thread = target as Handle;
                let mut samples: Vec<Vec<u64>> = Vec::with_capacity(1 << 20);
                let mut context = Box::new(Context([0; 1232]));
                let mut stack: Vec<u64> = Vec::with_capacity(256);
                while !flag.load(Ordering::Relaxed) {
                    std::thread::sleep(std::time::Duration::from_micros(period_micros));
                    stack.clear();
                    unsafe {
                        if SuspendThread(thread) == u32::MAX {
                            continue;
                        }
                        write(&mut context, FLAGS, 0x0010_0003);
                        if GetThreadContext(thread, &mut *context) != 0 {
                            for _ in 0..200 {
                                let rip = read(&context, RIP);
                                if rip == 0 {
                                    break;
                                }
                                stack.push(rip);
                                let mut base = 0u64;
                                let entry = RtlLookupFunctionEntry(rip, &mut base, std::ptr::null_mut());
                                if entry.is_null() {
                                    let rsp = read(&context, RSP);
                                    if rsp == 0 {
                                        break;
                                    }
                                    let ret = *(rsp as *const u64);
                                    write(&mut context, RIP, ret);
                                    write(&mut context, RSP, rsp + 8);
                                } else {
                                    let (mut data, mut establisher) = (std::ptr::null_mut(), 0u64);
                                    RtlVirtualUnwind(0, base, rip, entry, &mut *context, &mut data, &mut establisher, std::ptr::null_mut());
                                }
                            }
                        }
                        ResumeThread(thread);
                    }
                    // the samples are kept outside the suspended window: the main thread may hold the allocator
                    samples.push(stack.clone());
                }
                samples
            });
            Sampler { stop, worker: Some(worker) }
        }

        pub fn finish(mut self, top: usize) {
            self.stop.store(true, Ordering::Relaxed);
            let samples = self.worker.take().unwrap().join().unwrap();
            report(&samples, top);
        }
    }

    struct Resolved {
        function: String,
        line: String,
    }

    fn resolve(address: u64, cache: &mut HashMap<u64, Resolved>) -> &Resolved {
        cache.entry(address).or_insert_with(|| unsafe {
            let process = GetCurrentProcess();
            let mut buffer = vec![0u8; 88 + 1024];
            buffer[0..4].copy_from_slice(&88u32.to_le_bytes());
            buffer[80..84].copy_from_slice(&1023u32.to_le_bytes());
            let mut displacement = 0u64;
            // the address of a CALLER frame is a return address: step back into the call instruction
            let function = if SymFromAddr(process, address, &mut displacement, buffer.as_mut_ptr()) != 0 {
                let length = u32::from_le_bytes(buffer[76..80].try_into().unwrap()) as usize;
                String::from_utf8_lossy(&buffer[84..84 + length.min(1023)]).to_string()
            } else {
                format!("?{address:x}")
            };
            let mut line = Line { size: std::mem::size_of::<Line>() as u32, key: std::ptr::null_mut(), number: 0, file: std::ptr::null(), address: 0 };
            let mut line_displacement = 0u32;
            let line = if SymGetLineFromAddr64(process, address, &mut line_displacement, &mut line) != 0 && !line.file.is_null() {
                let file = std::ffi::CStr::from_ptr(line.file as *const _).to_string_lossy().to_string();
                let short = file.rsplit(['\\', '/']).next().unwrap_or("?").to_string();
                format!("{short}:{}", line.number)
            } else {
                "?".to_string()
            };
            Resolved { function, line }
        })
    }

    fn report(samples: &[Vec<u64>], top: usize) {
        unsafe {
            SymSetOptions(0x2 | 0x10); // UNDNAME, LOAD_LINES (no deferred loads: the table is read once)
            let path = std::env::current_exe().ok().and_then(|exe| exe.parent().map(|dir| format!("{}\0", dir.display())));
            let ok = SymInitialize(GetCurrentProcess(), path.as_ref().map_or(std::ptr::null(), |text| text.as_ptr()), 1);
            assert!(ok != 0, "SymInitialize");
        }
        let mut cache: HashMap<u64, Resolved> = HashMap::new();
        let mut own: HashMap<String, u64> = HashMap::new();
        let mut inclusive: HashMap<String, u64> = HashMap::new();
        let mut lines: HashMap<String, u64> = HashMap::new();
        let total = samples.iter().filter(|stack| !stack.is_empty()).count() as u64;
        for stack in samples.iter().filter(|stack| !stack.is_empty()) {
            let top_frame = resolve(stack[0], &mut cache);
            *own.entry(top_frame.function.clone()).or_default() += 1;
            *lines.entry(format!("{} {}", top_frame.line, top_frame.function.chars().take(60).collect::<String>())).or_default() += 1;
            let mut seen: std::collections::HashSet<String> = Default::default();
            for (depth, address) in stack.iter().enumerate() {
                let name = resolve(if depth == 0 { *address } else { *address - 1 }, &mut cache).function.clone();
                if seen.insert(name.clone()) {
                    *inclusive.entry(name).or_default() += 1;
                }
            }
        }
        let print = |title: &str, table: &HashMap<String, u64>| {
            let mut rows: Vec<(&String, &u64)> = table.iter().collect();
            rows.sort_by_key(|row| std::cmp::Reverse(*row.1));
            println!("{title} ({total} samples)");
            for (name, count) in rows.into_iter().take(top) {
                println!("  {:6.2} %  {}", 100.0 * *count as f64 / total as f64, name.chars().take(150).collect::<String>());
            }
        };
        print("SELF by function", &own);
        print("INCLUSIVE by function", &inclusive);
        print("SELF by line", &lines);
        // SAMPLER_FOCUS=a,b: for each fragment, what the samples whose stack holds it spend their time in, which callees it calls and who calls it
        for focus in std::env::var("SAMPLER_FOCUS").unwrap_or_default().split(',').filter(|text| !text.is_empty()) {
            let (mut inside, mut callees, mut callers): (HashMap<String, u64>, HashMap<String, u64>, HashMap<String, u64>) = Default::default();
            let mut count = 0u64;
            for stack in samples.iter().filter(|stack| !stack.is_empty()) {
                let names: Vec<String> = stack.iter().enumerate().map(|(depth, address)| resolve(if depth == 0 { *address } else { *address - 1 }, &mut cache).function.clone()).collect();
                if let Some(position) = names.iter().position(|name| name.contains(focus)) {
                    count += 1;
                    *inside.entry(names[0].clone()).or_default() += 1;
                    let callee = if position > 0 { names[position - 1].clone() } else { "(itself)".to_string() };
                    *callees.entry(callee).or_default() += 1;
                    let short = |text: Option<&String>| text.map_or(String::new(), |name| name.chars().take(60).collect::<String>());
                    let outer = names.iter().rposition(|name| name.contains(focus)).unwrap_or(position);
                    let caller = format!("{}  <-  {}  <-  {}", short(names.get(outer + 1)), short(names.get(outer + 2)), short(names.get(outer + 3)));
                    *callers.entry(caller).or_default() += 1;
                }
            }
            println!("== focus {focus}: {count} samples = {:.2} %", 100.0 * count as f64 / total as f64);
            for (title, table) in [("  spends its time in (top frame)", &inside), ("  calls", &callees), ("  called from", &callers)] {
                let mut rows: Vec<(&String, &u64)> = table.iter().collect();
                rows.sort_by_key(|row| std::cmp::Reverse(*row.1));
                println!("{title}");
                for (name, number) in rows.into_iter().take(14) {
                    println!("    {:6.2} %  {}", 100.0 * *number as f64 / total as f64, name.chars().take(200).collect::<String>());
                }
            }
        }
    }
}

/// The CPU cycles this thread has run (Windows `QueryThreadCycleTime`): unlike the wall clock it does not count the time the thread was preempted, so two builds can be compared
/// on a machine that other work shares. Zero where the call does not exist.
#[cfg(windows)]
fn thread_cycles() -> u64 {
    #[link(name = "kernel32")]
    extern "system" {
        fn GetCurrentThread() -> *mut std::ffi::c_void;
        fn QueryThreadCycleTime(thread: *mut std::ffi::c_void, cycles: *mut u64) -> i32;
    }
    let mut cycles = 0u64;
    unsafe {
        QueryThreadCycleTime(GetCurrentThread(), &mut cycles);
    }
    cycles
}

#[cfg(not(windows))]
fn thread_cycles() -> u64 {
    0
}

fn main() {
    let arguments: Vec<String> = std::env::args().collect();
    let text = std::fs::read_to_string(&arguments[1]).expect("the polygons file");
    let mut repeat = 3usize;
    let (mut out, mut only, mut top): (Option<String>, Option<Vec<String>>, usize) = (None, None, 8);
    let mut sample = false;
    let mut index = 2;
    while index < arguments.len() {
        match arguments[index].as_str() {
            "--sample" => {
                sample = true;
                index += 1;
            }
            "--out" => {
                out = Some(arguments[index + 1].clone());
                index += 2;
            }
            "--only" => {
                only = Some(arguments[index + 1].split(',').map(str::to_string).collect());
                index += 2;
            }
            "--top" => {
                top = arguments[index + 1].parse().expect("top");
                index += 2;
            }
            other => {
                repeat = other.parse().expect("repeat");
                index += 1;
            }
        }
    }
    let calls = parse(&text);
    let mut lines = Vec::new();
    let mut rows: Vec<(String, String, i64, f64, usize)> = Vec::new();
    let mut by_mesh: std::collections::BTreeMap<String, (f64, usize)> = Default::default();
    let mut cycles_total = 0u64;
    let mut profile_total: Vec<u64> = vec![0; PHASES.len()];
    let mut stats_total: Vec<u64> = vec![0; STATS.len()];
    #[cfg(windows)]
    let sampling = sample.then(|| sampler::Sampler::start(1000));
    for call in &calls {
        if let Some(names) = &only {
            if !names.iter().any(|name| name == &call.name) {
                continue;
            }
        }
        let mut best = f64::MAX;
        let mut best_cycles = u64::MAX;
        let mut digest = 0u64;
        let mut nodes = 0usize;
        let mut profile: Vec<u64> = Vec::new();
        let mut stats: Vec<u64> = Vec::new();
        for _ in 0..repeat {
            let mut session = Session::new();
            let mut run = CostRun::begin_parts(&mut session, None, Some(call.cap), Some([0; 6])).expect("a cold run");
            let options = BuilderOptions { dense_hydration: call.dense, march_steps: None, budgeted: true, exhaustive: false, replay_check: false };
            let polygon = call.polygon.clone();
            let mut counts = SignCounts::default();
            reset();
            reset_stats();
            let cycles_before = thread_cycles();
            let started = Instant::now();
            let result = {
                let mut ctx = ExactCtx { memory: &mut session.memory, budget: run.budget_mut(), counts: &mut counts, products: &mut session.products };
                build_skeleton(&mut ctx, polygon, options, call.limit)
            };
            let seconds = started.elapsed().as_secs_f64();
            best_cycles = best_cycles.min(thread_cycles() - cycles_before);
            let articles = run.budget_mut().articles();
            let superlevel = run.budget_mut().superlevel.clone();
            let log = session.memory.take_log();
            let found = if sample { 1 } else { digest_of(&result, &counts, articles, &log, &superlevel) };
            if digest != 0 && found != digest {
                panic!("{}: two runs of one call answer differently", call.name);
            }
            digest = found;
            nodes = result.as_ref().map_or(0, |skeleton| skeleton.nodes.len());
            if seconds < best {
                best = seconds;
                profile = take();
                stats = take_stats();
            }
        }
        for (slot, value) in stats_total.iter_mut().zip(&stats) {
            *slot += value;
        }
        for (slot, value) in profile_total.iter_mut().zip(&profile) {
            *slot += value;
        }
        cycles_total += best_cycles;
        let entry = by_mesh.entry(call.mesh.clone()).or_insert((0.0, 0));
        entry.0 += best;
        entry.1 += 1;
        lines.push(format!("{} {:016x}", call.name, digest));
        rows.push((call.name.clone(), call.mesh.clone(), call.patch, best, nodes));
    }
    #[cfg(windows)]
    if let Some(running) = sampling {
        running.finish(top.max(40));
    }
    rows.sort_by(|left, right| right.3.total_cmp(&left.3));
    let total: f64 = rows.iter().map(|row| row.3).sum();
    println!("calls: {}  total best: {:.1} ms  thread cycles (best per call): {:.1} M", rows.len(), total * 1e3, cycles_total as f64 * 1e-6);
    for (mesh, (seconds, count)) in &by_mesh {
        println!("  {mesh:28} {count:5} calls {:9.1} ms", seconds * 1e3);
    }
    println!("heaviest:");
    for (name, mesh, patch, seconds, nodes) in rows.iter().take(top) {
        println!("  {name:32} {mesh:14} patch {patch:5} {nodes:5} nodes {:9.2} ms", seconds * 1e3);
    }
    if stats_total.iter().any(|value| *value > 0) {
        println!("counters (summed over the calls):");
        for ((label, _), value) in STATS.iter().zip(&stats_total) {
            println!("    {label:48} {value:12}");
        }
    }
    let phases: u64 = profile_total.iter().sum();
    if phases > 0 {
        let mut table: Vec<(&str, u64)> = PHASES.iter().zip(&profile_total).map(|((label, _), nanoseconds)| (*label, *nanoseconds)).collect();
        table.sort_by_key(|row| std::cmp::Reverse(row.1));
        println!("phases (sum of the best runs, exclusive times):");
        for (label, nanoseconds) in table.iter().filter(|row| row.1 > 0) {
            println!("    {label:34} {:10.2} ms {:5.1} %", *nanoseconds as f64 * 1e-6, 100.0 * *nanoseconds as f64 / phases as f64);
        }
    }
    if let Some(path) = out {
        std::fs::write(path, lines.join("\n") + "\n").expect("write digests");
    }
}
