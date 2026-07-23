# Session C-R2A — Sparse PatchDomain Boundary handoff

Status: `SESSION_C_R2A_SPARSE_PATCHDOMAIN_BOUNDARY_READY_FOR_REVIEW`

Branch: `codex/c-r2a-sparse-patchdomain`

Base: accepted V0-R1B `9132b1df8303847a4f75bb68cc28dcb79721a9cb`

## Closed gate

- Runtime domain geometry is one `SparsePatchDomainGeometryV1`, built from
  `BoundaryLoopV1`, ordered directed `ChainUseV1`, `PhysicalChainV1` and
  `BoundaryConstraintV1`.
- Runtime exact arrangement receives one `PlanarRegion` with one outer loop and
  its holes. Source-face cycles are not runtime regions or runtime boundaries.
- Explicit barriers stay separate from the closed domain silhouette and remain
  inputs to boundary-limited component resolution.
- Source face IDs exist only as the immutable domain contributor set.
- Per-boundary-segment provenance is emitted before arrangement and contains
  physical edge, physical chain, boundary loop, directed ChainUse, PatchDomain,
  boundary constraint and source lineage whenever those facts exist in the
  authoritative target.
- Disconnected PatchDomains fail with
  `REFERENCE_INPUT_CONTRACT_INVALID`; the kernel does not silently create two
  propagation fields.

## Independent face-union oracle

`cftuv_envelope.reference.domain_oracle` is deliberately absent from runtime
package imports. Tests invoke it directly and compare:

- exact area;
- outer component count;
- hole count;
- exact boundary geometry;
- boundary physical-edge lineage;
- source-face contributor set;
- absence of non-constraint internal source edges;
- rigid translation and coherent scale;
- retriangulation.

Cases cover convex, concave, one hole, several holes, disconnected invalid
domain, `SEAM_SELF`, explicit internal barrier, common-refined opposite seam
uses and retriangulation.

## Telemetry

`DOMAIN_BUILD` reports:

- `DOMAIN_FACE_BOUNDARY_SEGMENTS_ORACLE`;
- `DOMAIN_INPUT_SEGMENTS`;
- `DOMAIN_SPARSE_SEGMENTS`;
- `DOMAIN_BOUNDARY_SEGMENTS`;
- `ARRANGEMENT_DOMAIN_SEGMENTS`;
- `BOUNDARY_RESOLVER_BARRIER_SEGMENTS`;
- outer/hole/barrier counts and source-face contributor count.

`ARRANGEMENT_INPUT_SEGMENTS` remains the actual exact Boolean input count.
Open explicit barriers are counted in the resolver input instead of being
misrepresented as closed Boolean loops.

## building.002 receipt

Source: `E:\testscene.blend`, object `building.002`, selected physical edges
`2, 3, 7`.

| Patch | Source faces | Face-cycle segments | Sparse segments | Internal edges removed | Exact |
|---:|---:|---:|---:|---:|:---|
| 0 | 3 | 16 | 12 | 2 | equivalent |
| 7 | 1 | 4 | 4 | 0 | equivalent |
| 10 | 1 | 4 | 4 | 0 | equivalent |
| 12 | 1 | 4 | 4 | 0 | equivalent |

For every selected domain, exact area, component/hole count, boundary geometry
and physical-edge lineage match the face-union oracle. Source mesh fingerprint
before and after export is identical. Machine-readable evidence:
`artifacts/envelope_r2a/building_002_sparse_domain_report.json`.

## Verification

- Blender-free full kernel suite: `182 passed`.
- Host adapter staged telemetry suite: `23 passed`.
- Dedicated sparse/oracle plus public-contract checks: `16 passed`.

## Kernel handoff checklist

- Active phase: C-R2A reference domain geometry; gate closed locally.
- PatchDomain/PhysicalChain/ChainUse: full directed boundary identity is used;
  `SEAM_SELF` uses stay distinct; common-refined seam sides share physical
  identity.
- Unit of execution remains patch-level. Envelope, interaction, metric and
  ownership semantics were not changed.
- Existing boundary capacity outcomes are unchanged. The only new rejection is
  the named invalid disconnected-domain contract above.
- `CanonicalGeometryDigest` contract was not changed. Raw-coverage digest values
  can change where newly authoritative boundary loop/physical-chain provenance
  replaces source-face boundary provenance.
- Forbidden legacy geometry files were not opened or called.

## Residual risk

`PhysicalEdgeSequenceConstraintTargetV1` can name an explicit barrier without a
ChainUse or BoundaryLoop. Its provenance correctly retains physical edge,
PatchDomain and constraint, but cannot invent missing chain/loop lineage. This
is a host contract fact, not a kernel reconstruction opportunity.
