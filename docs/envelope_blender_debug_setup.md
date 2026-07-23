# Blender Envelope Debug setup

## Current V0-A deliverable

V0-A provides the exact host adapter and the standalone kernel DebugScene. It
does not yet add a Blender button or Grease Pencil renderer. Install it now
only for adapter/API inspection or for preparation of the separately gated
V0-B runtime.

From this checkout, build the standalone wheel:

```powershell
$Repo = "C:\path\to\CFTUV"
py -m pip install build
py -m build --wheel "$Repo\kernel"
```

Prepare the pinned dependency as a local wheel set on a machine with package
access:

```powershell
$Deps = Join-Path $Repo "blender-envelope-deps"
New-Item -ItemType Directory -Force -Path $Deps
py -m pip download --only-binary=:all: --dest $Deps "sympy==1.14.0"
```

Choose the Python executable shipped with the installed Blender version. For
example, adjust both version components for Blender 4.5:

```powershell
$BlenderPython = "C:\Program Files\Blender Foundation\Blender 4.5\4.5\python\bin\python.exe"
& $BlenderPython -m ensurepip
& $BlenderPython -m pip install --no-index --find-links $Deps "sympy==1.14.0"
$KernelWheel = Get-ChildItem "$Repo\kernel\dist\cftuv_envelope_core-0.4.0-*.whl" | Select-Object -First 1
& $BlenderPython -m pip install --no-index --force-reinstall $KernelWheel.FullName
```

The Blender operator must never download packages. Missing or mismatched
dependencies are reported as `ENVELOPE_DEBUG_KERNEL_UNAVAILABLE` or
`ENVELOPE_DEBUG_SYMPY_VERSION_UNSUPPORTED`.

## File to copy into the add-on for V0-A

Find the installed `cftuv` add-on package, then copy exactly one host file:

```powershell
$AddonDir = Join-Path $env:APPDATA "Blender Foundation\Blender\4.5\scripts\addons\cftuv"
Copy-Item "$Repo\cftuv\envelope_host_adapter.py" "$AddonDir\envelope_host_adapter.py" -Force
```

Do not copy `kernel/src/cftuv_envelope` into the add-on directory; install the
wheel into Blender Python. V0-A does not change `cftuv/debug.py`,
`cftuv/operators.py`, or registration files, so copying the adapter alone
does not create a UI control.

After the V0-A gate is accepted, V0-B will provide the exact list of runtime
files to copy (renderer, centralized `debug.py` compatibility additions, and
thin operator/registration changes) plus a Blender background smoke command.

## Preflight

The following command checks only dependency visibility:

```powershell
& $BlenderPython -c "import sympy, cftuv_envelope; assert sympy.__version__ == '1.14.0'; print(cftuv_envelope.__version__)"
```

Expected V0-A kernel version: `0.4.0`.
