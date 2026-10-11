"""Investigation probe: a genuine CPU single step (trap flag) on a native thread of a Python process, with faulthandler on, as pytest runs it.

Shows what a real STATUS_SINGLE_STEP of a PYTHON process looks like in the CI log (faulthandler prints `Windows fatal exception: code 0x80000004`) and which exit code the
step reports for it: the failed job showed NEITHER a faulthandler line NOR the low-byte exit code a child death gets under Git bash.
"""

import ctypes
import faulthandler
import time

faulthandler.enable()
kernel32 = ctypes.windll.kernel32
kernel32.VirtualAlloc.restype = ctypes.c_void_p
kernel32.VirtualAlloc.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_ulong, ctypes.c_ulong]
kernel32.CreateThread.restype = ctypes.c_void_p
kernel32.CreateThread.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_ulong, ctypes.c_void_p]

# pushfq; or qword ptr [rsp], 0x100; popfq; nop; nop; xor eax,eax; ret
code = bytes([0x9C, 0x48, 0x81, 0x0C, 0x24, 0x00, 0x01, 0x00, 0x00, 0x9D, 0x90, 0x90, 0x31, 0xC0, 0xC3])
memory = kernel32.VirtualAlloc(None, 4096, 0x3000, 0x40)
ctypes.memmove(memory, code, len(code))
print("starting a native thread that sets the trap flag", flush=True)
kernel32.CreateThread(None, 0, memory, None, 0, None)
time.sleep(5)
print("survived (the thread did not take the single step)", flush=True)
