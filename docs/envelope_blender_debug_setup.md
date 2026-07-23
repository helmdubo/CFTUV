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

For a manual update, copy these six files from the branch checkout into the
installed `cftuv` package:

```text
cftuv\envelope_host_adapter.py
cftuv\envelope_debug_renderer.py
cftuv\envelope_debug_profile.py
cftuv\envelope_topology_debug.py
cftuv\debug.py
cftuv\operators.py
```

PowerShell example:

```powershell
$Repo = "C:\path\to\CFTUV"
$Addon = "$env:APPDATA\Blender Foundation\Blender\4.3\scripts\addons\cftuv"

Copy-Item "$Repo\cftuv\envelope_host_adapter.py" $Addon -Force
Copy-Item "$Repo\cftuv\envelope_debug_renderer.py" $Addon -Force
Copy-Item "$Repo\cftuv\envelope_debug_profile.py" $Addon -Force
Copy-Item "$Repo\cftuv\envelope_topology_debug.py" $Addon -Force
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
5. In `Envelope Debug (Staged)`, use `Build Topology Debug` for the
   dependency-free host projection, or set Alpha and use
   `Build Exact Reference Envelope Debug`.

The result is a separate
`CFTUV_DEBUG_Envelope_<source object>` Grease Pencil object. Build/Clear do
not call the legacy Decal producer.

Exact-frame admission is applied only to PatchDomains reached by the selected
PhysicalChains. An unselected tilted or non-exact patch cannot block a
selected exact-planar patch. A selected seam reaches both of its patch-side
domains, so both frames must be certified.

PatchDomains are split by Seam topology, not merely by a visually planar face
selection. Sharp edges do not create PatchDomains. If one selected seam use
belongs to a connected non-planar patch, mark the required patch boundaries
as Seams and rebuild analysis; the bridge will not infer or insert those
domain cuts.

When the two patch sides declare different BoundaryChain segmentation over
the same exact physical-edge route, V0 uses their exact common endpoint
refinement. It never expands a partial selection: the selected edges must
still equal one complete refined PhysicalChain.

An unproved plane reports
`ENVELOPE_DEBUG_EXACT_PLANAR_FRAME_UNAVAILABLE`. A partial chain reports
`ENVELOPE_DEBUG_PARTIAL_CHAIN_SELECTION_UNSUPPORTED`. Neither case rounds the
frame or falls back to legacy geometry. The adapter proves the plane from the
PatchDomain source vertices and derives its exact frame; the approximate host
U/V/N values are orientation hints, not the certificate.

Exact coplanarity alone is still not sufficient under the V0 public
float-valued frame contract. A plane whose exact unit normal has irrational
components cannot be represented by `LocalVector3V1` without rounding, even
when every source vertex lies exactly on that plane. It fails named rather
than weakening the certificate. An exact local axis/rational frame with
rotation carried by `matrix_world` remains supported.

For signed-axis planes, the adapter derives a canonical tangent basis and uses
the exact canonical point on the certified plane as the display/kernel origin.
This avoids both normalized host-basis noise and a non-round-trippable
coordinate introduced by subtracting an arbitrary source vertex; source
coordinates themselves remain unchanged.

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
