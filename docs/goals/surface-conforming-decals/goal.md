# Surface-Conforming CFTUV Decals

## Objective

Replace the current patch-average/edge-first decal construction with one oriented, chain-aware, surface-following ribbon pipeline used by CORNERS, SEAMS, TOP, BOTTOM, and manual boundary decals.

## Original Request

Break the agreed surface-conforming decal work into substantial user-testable slices and execute it autonomously.

## Intake Summary

- Input shape: `existing_plan`
- Audience: CFTUV Blender user working on production hard-surface meshes
- Authority: `approved`
- Proof type: `test`
- Completion proof: automated geometry/UV regressions pass and the documented Blender walkthrough demonstrates arbitrary-angle corners, locally changing normals, wide curved-surface conformance, and stable closed TOP/BOTTOM rings
- Goal oracle: the staged Blender decal acceptance walkthrough plus automated ribbon invariants after every Worker package
- Likely misfire: producing mathematically cleaner helpers while decals still float, mirror, cross the surface, or cease to follow canonical PatchGraph chains in Blender
- Blind spots considered: patch-average normals are insufficient; normal-only tangent planes do not conform wide wings; point-contact vertices must not join unrelated runs; closed rings need orientation and UV seam validation; projection must remain restricted to the owner patch
- Existing plan facts: preserve PatchGraph as the source of topology; keep chains atomic and oriented; use a shared ribbon producer; add local surface normals in analysis; support one-sided and two-sided decals; project subdivided width samples to owner-patch geometry; keep UV in stable longitudinal/transverse coordinates

## Goal Oracle

The oracle for this goal is:

`All automated decal/ribbon tests pass, and docs/cftuv_decals.md contains a reproducible Blender walkthrough whose four user-visible slices verify: (1) non-90-degree planar corners and non-mirrored closed TOP/BOTTOM rings, (2) wings changing with local surface normals along a chain, (3) wide decals hugging curved/faceted owner-patch surfaces at the configured offset, and (4) all automatic/manual decal modes sharing stable orientation, UV, and closed-loop behavior.`

The PM must keep comparing task receipts to this oracle. Planning, discovery, a passing tiny slice, or a clean-looking board is not enough. The goal finishes only when a final Judge/PM audit maps receipts and verification back to this oracle and records `full_outcome_complete: true`.

## Goal Kind

`existing_plan`

## Current Tranche

Continuously implement and verify four substantial user-testable slices: oriented planar ribbon foundation; analysis-owned local surface frames; adaptive owner-surface conformance across ribbon width; full-mode integration and regression hardening. Continue until the full oracle is satisfied.

## Non-Negotiable Constraints

- Preserve CFTUV chain-first architecture: Patch and BoundaryChain remain the primary topology units.
- Do not use the legacy `Hotspot_UV_v2_5_26.py` monolith.
- `decals.py` remains a PatchGraph consumer and must not read the source BMesh.
- Store indices and immutable geometry facts in IR; never store BMFace/BMEdge references.
- Preserve Blender 4.1+ compatibility and existing automatic/manual decal UI semantics.
- UV orientation must remain semantic (`spine/base/tip`) even when geometry or polygon winding is reversed.
- Surface projection must stay within the owner patch and must fail safely rather than jumping to a nearby fold.
- Each Worker slice must produce a behavior the user can test in Blender and update the acceptance walkthrough.
- Do not commit, push, or open a PR unless explicitly requested.

## Stop Rule

Stop only when a final audit proves the full original outcome is complete.

Do not stop after planning, discovery, or Judge selection if a safe Worker task can be activated. Do not stop after one verified slice while later required slices remain.

## Slice Sizing

Each Worker package is a complete user-visible slice, not a helper-only task. Keep the package bounded, reversible, and independently verifiable.

## Canonical Board

Machine truth lives at:

`docs/goals/surface-conforming-decals/state.yaml`

## Run Command

```text
/goal Follow docs/goals/surface-conforming-decals/goal.md.
```

## PM Loop

1. Read this charter and `state.yaml`.
2. Work only on the active task.
3. Preserve the existing plan but validate it against current code and architecture.
4. Complete the whole active user-visible slice and record its receipt.
5. Run the strongest available automated checks plus compile/import checks.
6. Update the Blender acceptance walkthrough for the slice.
7. Immediately advance to the next safe slice unless a phase-risk decision is required.
8. Finish only after a final audit maps current evidence to every oracle item.
