"""Own Windows Job for external strict launcher; HANDLE lives until OS exit."""
import ctypes,json,os,subprocess,time,hashlib
from ctypes import wintypes as w
from pathlib import Path
K=ctypes.WinDLL('kernel32',use_last_error=True)
class BASIC(ctypes.Structure):
 _fields_=[('PerProcessUserTimeLimit',ctypes.c_longlong),('PerJobUserTimeLimit',ctypes.c_longlong),('LimitFlags',w.DWORD),('MinimumWorkingSetSize',ctypes.c_size_t),('MaximumWorkingSetSize',ctypes.c_size_t),('ActiveProcessLimit',w.DWORD),('Affinity',ctypes.c_size_t),('PriorityClass',w.DWORD),('SchedulingClass',w.DWORD)]
class IO(ctypes.Structure):_fields_=[(n,ctypes.c_ulonglong) for n in ('ReadOperationCount','WriteOperationCount','OtherOperationCount','ReadTransferCount','WriteTransferCount','OtherTransferCount')]
class EXT(ctypes.Structure):_fields_=[('BasicLimitInformation',BASIC),('IoInfo',IO),('ProcessMemoryLimit',ctypes.c_size_t),('JobMemoryLimit',ctypes.c_size_t),('PeakProcessMemoryUsed',ctypes.c_size_t),('PeakJobMemoryUsed',ctypes.c_size_t)]
K.CreateJobObjectW.argtypes=[ctypes.c_void_p,w.LPCWSTR];K.CreateJobObjectW.restype=w.HANDLE
K.GetCurrentProcess.restype=w.HANDLE
K.SetInformationJobObject.argtypes=[w.HANDLE,ctypes.c_int,ctypes.c_void_p,w.DWORD]
K.AssignProcessToJobObject.argtypes=[w.HANDLE,w.HANDLE]
K.IsProcessInJob.argtypes=[w.HANDLE,w.HANDLE,ctypes.POINTER(w.BOOL)]
K.CloseHandle.argtypes=[w.HANDLE]
K.DuplicateHandle.argtypes=[w.HANDLE,w.HANDLE,w.HANDLE,ctypes.POINTER(w.HANDLE),w.DWORD,w.BOOL,w.DWORD]
K.GetExitCodeProcess.argtypes=[w.HANDLE,ctypes.POINTER(w.DWORD)]
K.WaitForSingleObject.argtypes=[w.HANDLE,w.DWORD]
_JOB=None

def check(ok):
 if not ok:raise ctypes.WinError(ctypes.get_last_error())
def setup(path,*,force_failure=False):
 global _JOB
 from owned_job_guard import creation_filetime,save
 path=Path(path);assert not path.exists(),'JOB_RECEIPT_EXISTS'
 state=dict(status='SETUP',pid=os.getpid(),creation_filetime=creation_filetime(K.GetCurrentProcess()),helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),children=[])
 job=None;assigned=False
 try:
  prior=w.BOOL();check(K.IsProcessInJob(K.GetCurrentProcess(),None,ctypes.byref(prior)));state['preexisting_job']=bool(prior.value)
  if force_failure:raise RuntimeError('INJECTED_SETUP_FAILURE_BEFORE_TESTS')
  job=K.CreateJobObjectW(None,None);check(job)
  info=EXT();info.BasicLimitInformation.LimitFlags=0x2000
  check(K.SetInformationJobObject(job,9,ctypes.byref(info),ctypes.sizeof(info)))
  check(K.AssignProcessToJobObject(job,K.GetCurrentProcess()));assigned=True;_JOB=job
  present=w.BOOL();check(K.IsProcessInJob(K.GetCurrentProcess(),job,ctypes.byref(present)));assert present.value,'OWN_JOB_ASSIGNMENT_NOT_VERIFIED'
  state.update(status='ASSIGNED_VERIFIED',kill_on_job_close=True,handle_noninheritable=True)
  save(path,state)
 except BaseException as exc:
  state.update(status='SETUP_FAILED_NO_TESTS',error=repr(exc));save(path,state)
  if job and not assigned:K.CloseHandle(job)
  raise
 original=subprocess.Popen.__init__;tracked=[]
 def audited(proc,*args,**kwargs):
  original(proc,*args,**kwargs)
  # Keep original Popen object/return semantics. Any receipt failure fails the run.
  try:
   row=dict(pid=proc.pid,creation_filetime=creation_filetime(int(proc._handle)),argv=repr(proc.args),exit=None)
   duplicate=w.HANDLE();check(K.DuplicateHandle(K.GetCurrentProcess(),int(proc._handle),K.GetCurrentProcess(),ctypes.byref(duplicate),0x100000|0x1000,False,0))
   tracked.append((duplicate,row));state['children'].append(row);save(path,state)
  except BaseException as exc:
   state['audit_failure']=repr(exc)
   raise
 subprocess.Popen.__init__=audited
 def finish(code):
  subprocess.Popen.__init__=original
  for handle,row in tracked:
   wait=K.WaitForSingleObject(handle,0)
   if wait==0:
    code_value=w.DWORD();check(K.GetExitCodeProcess(handle,ctypes.byref(code_value)));row['exit']=code_value.value
   elif wait!=258:raise ctypes.WinError(ctypes.get_last_error())
   K.CloseHandle(handle)
  if state.get('audit_failure'):
   state.update(status='AUDIT_FAILED_NOT_ACCEPTED',exit=1);save(path,state);raise RuntimeError('STRICT_CHILD_AUDIT_FAILED')
  state.update(status='PYTEST_EXIT',exit=code);save(path,state)
  # NEVER CloseHandle(job): OS exit closes the sole handle and kills descendants.
 return finish
