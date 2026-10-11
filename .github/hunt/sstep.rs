// Investigation probe (branch native/investigate-raw-crash only): ends the process the way the failed CI job ended, in each way a Windows process can end with 0x80000004.
//
//   exit  -> ExitProcess(0x80000004)                      (what `os._exit(0x80000004)` does)
//   raise -> RaiseException(0x80000004), unhandled         (an exception, no CPU involved)
//   tf    -> the CPU trap flag: a genuine #DB single step  (what a real STATUS_SINGLE_STEP is)
use std::arch::asm;

extern "system" {
    fn RaiseException(code: u32, flags: u32, n: u32, args: *const usize);
    fn ExitProcess(code: u32) -> !;
}

fn main() {
    let mode = std::env::args().nth(1).unwrap_or_default();
    match mode.as_str() {
        "exit" => unsafe { ExitProcess(0x8000_0004) },
        "raise" => unsafe { RaiseException(0x8000_0004, 0, 0, std::ptr::null()) },
        "tf" => unsafe {
            asm!("pushfq", "or qword ptr [rsp], 0x100", "popfq", "nop", "nop");
        },
        _ => println!("modes: exit raise tf"),
    }
    println!("survived {mode}");
}
