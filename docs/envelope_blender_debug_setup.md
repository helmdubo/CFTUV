# Blender Envelope Debug setup

## Installed Blender 4.3 layout

The tested add-on package location is:

```text
C:\Users\helmd\AppData\Roaming\Blender Foundation\Blender\4.3\scripts\addons\cftuv
```

Standalone Python dependencies are installed under:

```text
C:\Users\helmd\AppData\Roaming\Blender Foundation\Blender\4.3\scripts\addons\modules
```

V0-B requires `sympy==1.14.0` and `cftuv-envelope-core==0.5.0`.
The Blender operator never downloads packages.

## Files to copy

For a manual update, copy these four files from the branch checkout into the
installed `cftuv` package:

```text
cftuv\envelope_host_adapter.py
cftuv\envelope_debug_renderer.py
cftuv\debug.py
cftuv\operators.py
```

PowerShell example:

```powershell
$Repo = "C:\path\to\CFTUV"
$Addon = "$env:APPDATA\Blender Foundation\Blender\4.3\scripts\addons\cftuv"

Copy-Item "$Repo\cftuv\envelope_host_adapter.py" $Addon -Force
Copy-Item "$Repo\cftuv\envelope_debug_renderer.py" $Addon -Force
Copy-Item "$Repo\cftuv\debug.py" $Addon -Force
Copy-Item "$Repo\cftuv\operators.py" $Addon -Force
```

Do not copy `kernel/src/cftuv_envelope` into `cftuv`. Install the standalone
wheel into the `addons\modules` directory.

## Build and install the kernel

```powershell
$Repo = "C:\path\to\CFTUV"
$BlenderPython = "C:\Program Files\Blender Foundation\Blender 4.3\4.3\python\bin\python.exe"
$Modules = "$env:APPDATA\Blender Foundation\Blender\4.3\scripts\addons\modules"

py -m pip install build
py -m build --wheel "$Repo\kernel"
$KernelWheel = Get-ChildItem `
    "$Repo\kernel\dist\cftuv_envelope_core-0.5.0-*.whl" |
    Select-Object -First 1

& $BlenderPython -m pip install `
    --no-index --no-deps --upgrade `
    --target $Modules `
    $KernelWheel.FullName
```

Prepare the pinned SymPy wheel once on a machine with package access:

```powershell
$Deps = Join-Path $Repo "blender-envelope-deps"
New-Item -ItemType Directory -Force -Path $Deps
py -m pip download --only-binary=:all: --dest $Deps "sympy==1.14.0"
& $BlenderPython -m pip install `
    --no-index --find-links $Deps --upgrade `
    --target $Modules `
    "sympy==1.14.0"
```

## Runtime use

In Blender 4.3:

1. Open `View3D > Sidebar > Hotspot UV`.
2. Select a mesh and enter Edit Mode. The whole mesh may contain several
   planar or non-planar patches.
3. Enable Edge Select.
4. Select every edge of one or more complete PhysicalChains.
5. In `Envelope Debug (Exact Planar)`, set Alpha and press Build.

The result is a separate
`CFTUV_DEBUG_Envelope_<source object>` Grease Pencil object. Build/Clear do
not call the legacy Decal producer.

Exact-frame admission is applied only to PatchDomains reached by the selected
PhysicalChains. An unselected tilted or non-exact patch cannot block a
selected exact-planar patch. A selected seam reaches both of its patch-side
domains, so both frames must be certified.

An unproved plane reports
`ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE`. A partial chain reports
`ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED`. Neither case rounds the
frame or falls back to legacy geometry. Geometric planarity alone is not yet
sufficient: the selected PatchDomain's stored local U/V/N basis must also
satisfy the exact orthonormal certificate. Arbitrarily tilted local mesh
coordinates will commonly remain outside this V0 subset; an exact local mesh
with rotation carried by `matrix_world` is supported.

## Preflight and smoke

```powershell
$Blender = "C:\Program Files\Blender Foundation\Blender 4.3\blender.exe"

& $Blender --background --factory-startup --python-expr `
  "import addon_utils, sympy, cftuv_envelope; addon_utils.enable('cftuv'); assert sympy.__version__ == '1.14.0'; assert cftuv_envelope.__version__ == '0.5.0'"

& $Blender --background --factory-startup --python `
  "$Repo\tests\blender\test_envelope_debug_bridge.py"
```

Expected final line:

```text
ENVELOPE_DEBUG_BLENDER_SMOKE_OK
```
