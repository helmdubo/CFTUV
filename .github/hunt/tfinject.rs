// tfinject <pid> <thread-index|all> : sets the CPU trap flag in the saved context of a thread of ONE process (our own test child), as a stray TF would.
// Used to see how Git bash (msys2 runtime) ends when a single-step exception arrives that its signal machinery did not ask for.
use std::ffi::c_void;
use std::mem::{size_of, zeroed};

type Handle = *mut c_void;

#[repr(C)]
struct ThreadEntry32 {
    size: u32,
    usage: u32,
    thread_id: u32,
    owner_process_id: u32,
    base_pri: i32,
    delta_pri: i32,
    flags: u32,
}

#[repr(C, align(16))]
struct Context {
    p1_home: [u64; 6],
    context_flags: u32,
    mx_csr: u32,
    seg_cs: u16,
    seg_ds: u16,
    seg_es: u16,
    seg_fs: u16,
    seg_gs: u16,
    seg_ss: u16,
    eflags: u32,
    rest: [u8; 1232 - 6 * 8 - 4 - 4 - 12 - 4],
}

extern "system" {
    fn CreateToolhelp32Snapshot(flags: u32, pid: u32) -> Handle;
    fn Thread32First(snapshot: Handle, entry: *mut ThreadEntry32) -> i32;
    fn Thread32Next(snapshot: Handle, entry: *mut ThreadEntry32) -> i32;
    fn OpenThread(access: u32, inherit: i32, tid: u32) -> Handle;
    fn SuspendThread(thread: Handle) -> u32;
    fn ResumeThread(thread: Handle) -> u32;
    fn GetThreadContext(thread: Handle, context: *mut Context) -> i32;
    fn SetThreadContext(thread: Handle, context: *const Context) -> i32;
    fn CloseHandle(h: Handle) -> i32;
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let pid: u32 = args[1].parse().unwrap();
    let which = args.get(2).cloned().unwrap_or_else(|| "all".into());
    assert_eq!(size_of::<Context>(), 1232);
    unsafe {
        let snapshot = CreateToolhelp32Snapshot(4, 0);
        let mut entry: ThreadEntry32 = zeroed();
        entry.size = size_of::<ThreadEntry32>() as u32;
        let mut index = 0usize;
        let mut ok = Thread32First(snapshot, &mut entry);
        while ok != 0 {
            if entry.owner_process_id == pid {
                let hit = which == "all" || which.parse::<usize>().ok() == Some(index);
                if hit {
                    // THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT | THREAD_SET_CONTEXT
                    let thread = OpenThread(0x0002 | 0x0008 | 0x0010, 0, entry.thread_id);
                    if !thread.is_null() {
                        SuspendThread(thread);
                        let mut context: Context = zeroed();
                        context.context_flags = 0x0010_0001; // CONTEXT_CONTROL
                        if GetThreadContext(thread, &mut context) != 0 {
                            context.eflags |= 0x100;
                            let set = SetThreadContext(thread, &context);
                            println!("thread #{index} tid {} : TF set ({set})", entry.thread_id);
                        } else {
                            println!("thread #{index} tid {} : GetThreadContext failed", entry.thread_id);
                        }
                        ResumeThread(thread);
                        CloseHandle(thread);
                    }
                }
                index += 1;
            }
            ok = Thread32Next(snapshot, &mut entry);
        }
        CloseHandle(snapshot);
        println!("{index} threads seen");
    }
}
