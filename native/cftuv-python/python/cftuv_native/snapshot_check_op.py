"""The snapshot validators of the EXPORT stage as a CLEAN-OR-DECLINE operation: `snapshot_check(snapshot, scope)`, native.

The unit the product replaces is the question the worker asks twice per domain of the snapshot it has just exported: does `validate_analysis_snapshot` (alone: under the request budget; and
then `validate_reference_geometry_payload` of every domain: on export) report nothing? The Python validators stay the ORACLE: this operation answers "nothing" only when it proves it, and in every
other case it DECLINES by name (`NativePortUnsupported`, raised before anything changed) and the caller runs the Python validators exactly as it ran them before the port existed. Native never
refuses a snapshot itself - the refusal, with its text, is the validators' - and never attests a snapshot they would refuse (a silent acceptance is what AGENTS.md rule 4 forbids).

What the extension takes from Python (exact types only; see `snapshot_check.rs`): the snapshot graph, read slot by slot at the offsets a probe of a real instance found. What it carries: planar
domains with an exact-source-plane metric on the canonical basis, exact affine support directions, certified reflex measures. What it declines by name: near-planar, developable and band
metrics, a float support direction, a coordinate-free fixture payload, an angle on which the cheap interval path and the exact right-angle rule do not decide, a process that is not in the state the
port mirrors (`symbolic_backend` not `NATIVE_EXACT`, a decimal context too narrow for `phi - 1`).

The ANSWER of a clean snapshot carries the side effects the Python validators leave on the process counters (`symbolic_backend.BACKEND_COUNTS`: `angle_certificate_sign.certified` and
`angle_certificate_sign.exact_fallback.straddles_zero`), so that the dispatcher (`cftuv_envelope.backend.snapshot_check_compute`) leaves the same ones and the tree A/B compares counters
bit for bit.

The mirrored code is pinned: whole files by `pin.SNAPSHOT_CHECK_FILES`, and the functions of files that change for other reasons by the digest of their source when the classes bind
(`SHAPE_FUNCTIONS`: a change in any of them is `NativePortStale` naming the function). `python -m cftuv_native.snapshot_check_op` prints the table of the tree it is run against.
"""

from __future__ import annotations

import dataclasses
import hashlib
import inspect
import sys
from decimal import getcontext

from . import pin

OPERATION = "snapshot_check"

#: `scope` of a call.
SCOPE_ANALYSIS = 0
SCOPE_ANALYSIS_AND_REFERENCE = 1

#: The status codes of the extension's answer (`snapshot_check.rs`).
STATUS_CLEAN = 0
STATUS_DECLINED = 2

#: `Reason::code` of the extension (`cftuv-snapshot/src/verdict.rs`).
REASONS = {1: "ISSUE", 2: "UNSUPPORTED", 3: "SHAPE", 4: "MODE", 5: "UNDECIDED"}

#: `(file, qualified name, sha256 of the source text with CRLF turned into LF)` of the functions the Rust code mirrors that live in files the pin does not hold whole.
#: Regenerate by `python -m cftuv_native.snapshot_check_op` after a deliberate catch-up.
SHAPE_FUNCTIONS = (
    ("validation_metric.py", "validate_rational_affine_planar_metric", "fe8f4e7173c6899bad1be48bce140aec343eacf110d7f69e208060c119e1ba5a"),
    ("validation_metric.py", "_check_gram", "3381150fe3025fb99f721f3bdc947fd4a630b073d91ca94a8c1e0e4204d407a6"),
    ("validation_metric.py", "_check_reduced_frame", "488bef96fd8958d2e2f4f27017bd19d3ce34c4028dcac98f7321318451cada43"),
    ("validation_metric.py", "position_under_grid_law", "5e4ed097888518e6f9fa78c06965ddec4b13214acdbae5be95ecbbd2c7c043ed"),
    ("validation_metric.py", "expected_source_position", "69a0b9572a3e1c8cdf942d200f8360b48e3b50a228309797169605ce9270df6e"),
    ("validation_metric.py", "validate_metric_against_source", "cc105bb9e531dac8605190e7d6dba70ec9e68df46e6ca875f563abcada1e8390"),
    ("validation_metric.py", "validate_width_distortion_recomputation", "abcecedb48bcc5358d7b566ae9f80fe324073957368031a802b6bf449c15fbb7"),
    ("validation_metric.py", "chart_source_vertices", "439c9d0b656b121d62f10e161604ba250ea5c89a6cdcf6dde3ea8fb373e6d1d3"),
    ("validation_metric.py", "metric_covers_patch", "4ce4ca00736303f7ec20a32fe11b510fc2cb546581912a44b3e06530840a6217"),
    ("validation_metric.py", "fraction_of", "ef3767b74e85df2da8bdafb4abdc63a35075add04dcd9915ea05b89a3547ce34"),
    ("validation_metric.py", "fraction_point3", "d27b0e8f20ba79973d99881caa9cc1e64b0c9909c411d5c71d4d6bc45dc41eab"),
    ("validation_metric.py", "fraction_dot3", "944a95b70ae4865c577b797d286864d43a24b8cb7ba3f95e4a233240dc38d7d5"),
    ("validation_metric.py", "fraction_cross3", "b9ad1413d1108cac7d38402e1e568dde45e00a26ef810fb49bc4d261af994f54"),
    ("validation_chain_station.py", "seam_neighbour_face_issues", "6ce3347d2cac167d334ede42ffa5655a0bc6b9fa0b707eb33f4ce6a1f86f1a3f"),
    ("_metric_wire.py", "classify_metric_normal_wire", "1916e96f864b2c979ec80e98e587d9c45dd48ac116bbb8e11da05c96ebb7fa2d"),
    ("robust/grid.py", "snap_value", "31b4edd1a522e2773c5e327b51cc86e7bfdc574441130c00e7eada6dae5d85f2"),
    ("robust/grid.py", "GridSpecV1.__post_init__", "061194e9b7573974d761967da213bca4d588679509f72e3e36006cc35c831399"),
    ("contracts/metric.py", "source_vertex_of_chart_vertex", "ca08ce310ade5acf1a2e9f0e9a9bf12e15ffcbe36a902338bb1a0b745822f833"),
    ("contracts/metric.py", "GridSnappingLawV1.snaps_source", "9843f6c06ef0aa263632311568d36d3da0259554bac27fbae2fb7a1374657f68"),
    ("contracts/metric.py", "GridSnappingLawV1.snaps_constructions", "f51123c22b105f9be52e8f72bddb6c622524deef268fc442bf0642fc007669a9"),
    ("source_grid.py", "chart_grid_for", "d5c81264519f303ab69d527042a07c8390f93ec80cc715f2b6b3d7965f680654"),
    ("reference/validation.py", "validate_reference_geometry_payload", "be7f66272ea328b7b5c4001468035f5be048951946a6298eda3d591dc2fd2d62"),
    ("reference/validation.py", "validate_reference_geometry_certificates", "bd447eecc2855a518d43c5fbda47b0a0a5b743610c9066cc43520c966bce6612"),
    ("reference/validation.py", "_validate_angle_certificates", "eef8afd97108dee32e0d047c9ff7370c2f5e50cf088505a7f7f0a197c36c23e9"),
    ("reference/validation.py", "_support_direction", "ae7789a752da4aa9e81bfe1b175721749b47c94817009e05088bac14dedb72fd"),
    ("reference/validation.py", "_interval_contains_oriented_support_delta", "e79b0e3590e4f32717ec8953edfb74a8547cc85ae5b4ae15f40e449bc342cf33"),
    ("reference/validation.py", "_interval_contains_oriented_support_delta_exact", "be2586d26ca83d88323e08394d02a9edf37459a951ff51ab70a9b8b2fe9211da"),
    ("reference/metric.py", "ExactPlanarMetric.from_descriptor", "e5e49076f34ee59d6d67516f0bff1baae04c6af3e8f27d70a9cc8a4b6985f51c"),
    ("reference/metric.py", "_chart_grid", "1dcd70bb3f92815bc7bc93ce4584b8789c3c187c81c6a482300930ef0ddd9c7f"),
    ("reference/metric.py", "_matrix", "601e8ffffc1fcd596abe3b24a2b95cc56038dc8103c92eb2a6c9c5943a138f18"),
    ("reference/metric.py", "_scalar", "c99e615cffd1e5a6186a6f7f5b11acede4941abc56f1f3ed152aa766e44a3adc"),
    ("reference/symbolic_backend.py", "count", "331df92a3415eddd77da270a818da2c4875ff2f90a203ee091bc19f314b7bc94"),
    ("reference/symbolic_backend.py", "backend_mode", "7a74eb9b3d718ff4656a7d49c293a48f51f51593b5945f380055f7d101a199c6"),
)

#: The classes the extension reads, by module: `{module: names}`; the dictionary handed to `SnapshotCheck` maps each name to the live class.
CLASS_NAMES = {
    "cftuv_envelope.ids": (
        "SourceRevision", "PatchId", "PatchDomainId", "PhysicalChainId", "ChainUseId", "SourceVertexId", "SourceFaceId", "SurfaceTriangleId", "PhysicalEdgeId", "BoundaryLoopId",
        "BoundaryConstraintId", "OwnerSectorId", "AngleCertificateId", "PlanarityCertificateId", "ReferenceMetricId", "CornerRelationId", "JunctionRelationId", "TerminalRelationId", "RoutePairingId",
    ),
    "cftuv_envelope.contracts.analysis": (
        "AnalysisSnapshotV1", "AnalysisCapability", "SurfaceRegime", "PatchDescriptorV1", "PatchDomainV1", "PatchSectorV1", "SourceVertexV1",
        "PhysicalChainV1", "ChainUseV1", "LaunchLocusV1", "BoundaryLoopV1", "BoundaryConstraintV1", "BoundaryLoopConstraintTargetV1", "ChainUseConstraintTargetV1",
        "PhysicalEdgeSequenceConstraintTargetV1", "UnavailableBoundaryConstraintTargetV1", "OrientedOwnerSectorV1", "SourceSupportRefV1", "CertifiedPlanarSupportDirectionV1",
        "UnavailableSourceSupportDirectionV1", "ReflexAngleCertificateV1", "CertifiedReflexAngleMeasureV1", "UnavailableReflexAngleMeasureV1", "TurnOrientation", "CornerRelationV1",
        "JunctionRelationV1", "JunctionRelationKind", "JunctionRoutePairV1", "TJunctionRouteTopologyV1", "XJunctionRouteTopologyV1", "YJunctionRouteTopologyV1",
        "DeclaredRoutePairsTopologyV1", "UnavailableJunctionRouteTopologyV1", "TerminalRelationV1", "TerminalEndpointRole", "SeamNeighbourFaceV1", "PlanarPatchFrameV1",
        "IntrinsicSurfaceMetricDescriptorV1", "UnavailableSurfaceMetricDescriptorV1",
    ),
    "cftuv_envelope.contracts.surface": ("PatchSurfaceIRV1", "SurfacePayloadMode", "SourceEdgeV1", "SourceFaceV1", "SurfaceTriangleV1"),
    "cftuv_envelope.contracts.metric": (
        "RationalAffinePlanarMetricV2", "ExactSourcePlaneCertificateV1", "NearPlanarProjectionCertificateV1", "DevelopableUnfoldCertificateV1", "DevelopableBandChartCertificateV1",
        "ExactRationalV1", "ExactPoint2V1", "ExactPoint3V1", "ExactVector2V1", "ExactVector3V1", "ExactMatrix2V1", "ExactSourceVertexCoordinateV2", "CertifiedAffineSupportDirectionV2",
        "IntegerGridCertificateV1", "GridSnappingLawV1", "AffineChartOrientationV1", "AffineFrameSelectionLawV1", "PlanarityAdmissionLawV1", "AffineReconstructionLawV1",
    ),
    "cftuv_envelope.numeric": ("IntervalEndpointKind", "CertifiedDecimalIntervalV1", "LocalPoint3V1", "LocalVector2V1", "UnavailableSourcePositionV1"),
}

_KERNEL: list = []


def _names(cls) -> tuple:
    return tuple(item.name for item in dataclasses.fields(cls))


def source_digest(function) -> str:
    """sha256 of the source text of `function` (a property is its getter; CRLF turned into LF): the shape of a function the pin does not hold whole."""

    function = function.fget if isinstance(function, property) else function
    return hashlib.sha256(inspect.getsource(function).replace("\r\n", "\n").encode("utf-8")).hexdigest()


def _resolve(module, qualified: str):
    found = module
    for part in qualified.split("."):
        found = getattr(found, part)
    return found


def _kernel_modules() -> dict:
    """`{file: module}` of the oracle files whose functions are digested."""

    import importlib

    names = {
        "validation_metric.py": "cftuv_envelope.validation_metric",
        "validation_chain_station.py": "cftuv_envelope.validation_chain_station",
        "_metric_wire.py": "cftuv_envelope._metric_wire",
        "robust/grid.py": "cftuv_envelope.robust.grid",
        "contracts/metric.py": "cftuv_envelope.contracts.metric",
        "source_grid.py": "cftuv_envelope.source_grid",
        "reference/validation.py": "cftuv_envelope.reference.validation",
        "reference/metric.py": "cftuv_envelope.reference.metric",
        "reference/symbolic_backend.py": "cftuv_envelope.reference.symbolic_backend",
    }
    return {file: importlib.import_module(module) for file, module in names.items()}


def shape_digests(modules: dict | None = None) -> dict:
    """`{(file, name): digest or None}` of the `SHAPE_FUNCTIONS`, read from the live kernel (`None`: the function or its source is gone)."""

    modules = _kernel_modules() if modules is None else modules
    found = {}
    for file, name, _wanted in SHAPE_FUNCTIONS:
        try:
            found[(file, name)] = source_digest(_resolve(modules[file], name))
        except (AttributeError, OSError, TypeError, KeyError):
            found[(file, name)] = None
    return found


def _class_table() -> dict:
    import importlib

    table: dict = {}
    for module_name, names in CLASS_NAMES.items():
        module = importlib.import_module(module_name)
        for name in names:
            table[name] = getattr(module, name)
    from decimal import Decimal

    table["Decimal"] = Decimal
    return table


def _bind(core) -> tuple:
    """`(handle, signature)`: the extension's reader built on the live classes, after checking the oracle functions it mirrors are the ones it was written against."""

    modules = _kernel_modules()
    live = shape_digests(modules)
    pin.check_shapes(tuple((file, f"{name} is the function the Rust code mirrors", live[(file, name)] == wanted) for file, name, wanted in SHAPE_FUNCTIONS))
    table = _class_table()
    try:
        handle = core.SnapshotCheck(table)
    except TypeError as error:
        if "raw slot access was forced" in str(error):
            raise  # the refusal of a forced slot mode (CFTUV_NATIVE_SLOTS=raw): not a change of the oracle's classes
        raise pin.NativePortStale(f"the oracle classes the native snapshot_check is built from changed: {error}") from None
    return handle, tuple(id(item) for item in table.values())


def _modules() -> tuple:
    """The kernel modules the classes come from, as they are in `sys.modules` now (a reloaded kernel is a new set of module objects, hence of classes)."""

    modules = sys.modules
    return tuple(modules.get(name) for name in CLASS_NAMES)


def _handle(core):
    """The reader bound to the live kernel classes: one dictionary lookup per module on the hot path, a rebind when the kernel was reloaded."""

    live = _modules()
    if _KERNEL and _KERNEL[0][0] is core and all(a is b for a, b in zip(_KERNEL[0][2], live)) and None not in live:
        return _KERNEL[0][1]
    handle, _signature = _bind(core)
    _KERNEL[:] = [(core, handle, _modules())]
    return handle


_LIVE: list = []


def _live() -> tuple:
    """`(schema text, backend_mode, NATIVE_EXACT)` of the live kernel, looked up once per kernel (the module objects are checked by `_handle`)."""

    if _LIVE and _LIVE[0][0] is _KERNEL[0][2]:
        return _LIVE[0][1]
    from cftuv_envelope.contracts.analysis import ANALYSIS_SNAPSHOT_SCHEMA_V1
    from cftuv_envelope.reference import symbolic_backend

    found = (ANALYSIS_SNAPSHOT_SCHEMA_V1, symbolic_backend.backend_mode, symbolic_backend.SymbolicBackendV1.NATIVE_EXACT)
    _LIVE[:] = [(_KERNEL[0][2], found)]
    return found


@dataclasses.dataclass(frozen=True, slots=True)
class SnapshotCheckV1:
    """The attestation: the snapshot has no issue and no diagnostic; `certified` and `straddled` are the counter increments the Python validators would have left (`angle_certificate_sign`)."""

    domains: int
    corners: int
    certified: int
    straddled: int


def snapshot_check(core, snapshot, scope: int = SCOPE_ANALYSIS_AND_REFERENCE) -> SnapshotCheckV1:
    """`SnapshotCheckV1` when the Python validators of `scope` would report nothing for `snapshot`; otherwise `NativePortUnsupported` naming why the port did not attest.

    `scope`: `SCOPE_ANALYSIS` (`validate_analysis_snapshot` under the request budget) or `SCOPE_ANALYSIS_AND_REFERENCE` (that, then `validate_reference_geometry_payload` of every domain).
    Raises `NativePortStale` when a pinned oracle moved; nothing is touched either way.
    """

    pin.require(OPERATION)
    handle = _handle(core)
    schema, mode_of, exact = _live()
    answer = handle.check(snapshot, scope, mode_of() is exact, getcontext().prec, schema)
    if answer[0] == STATUS_CLEAN:
        return SnapshotCheckV1(answer[1], answer[2], answer[3], answer[4])
    raise pin.NativePortUnsupported(f"snapshot_check declined: {REASONS.get(answer[1], answer[1])}: {answer[2]}")


def raw_layouts(core) -> dict:
    """`{class: raw slot layout confirmed}` of the reader bound to the live kernel (the CI step of the slot modes reads it)."""

    pin.require(OPERATION)
    return dict(_handle(core).raw_layouts())


def last_timings() -> tuple:
    """`(read ns, check ns)` of the last call: reading the Python graph into the model, and the check on the model (zeros before the first call)."""

    return _KERNEL[0][1].last_timings() if _KERNEL else (0, 0)


def main(argv=None) -> int:
    """Prints the `SHAPE_FUNCTIONS` table of the tree it runs against (the functions listed in `FUNCTIONS`)."""

    modules = _kernel_modules()
    print("SHAPE_FUNCTIONS = (")
    for file, name in FUNCTIONS:
        digest = source_digest(_resolve(modules[file], name))
        print(f'    ("{file}", "{name}", "{digest}"),')
    print(")")
    return 0


#: The functions of shared files the Rust code mirrors (their digests are `SHAPE_FUNCTIONS`).
FUNCTIONS = (
    ("validation_metric.py", "validate_rational_affine_planar_metric"),
    ("validation_metric.py", "_check_gram"),
    ("validation_metric.py", "_check_reduced_frame"),
    ("validation_metric.py", "position_under_grid_law"),
    ("validation_metric.py", "expected_source_position"),
    ("validation_metric.py", "validate_metric_against_source"),
    ("validation_metric.py", "validate_width_distortion_recomputation"),
    ("validation_metric.py", "chart_source_vertices"),
    ("validation_metric.py", "metric_covers_patch"),
    ("validation_metric.py", "fraction_of"),
    ("validation_metric.py", "fraction_point3"),
    ("validation_metric.py", "fraction_dot3"),
    ("validation_metric.py", "fraction_cross3"),
    ("validation_chain_station.py", "seam_neighbour_face_issues"),
    ("_metric_wire.py", "classify_metric_normal_wire"),
    ("robust/grid.py", "snap_value"),
    ("robust/grid.py", "GridSpecV1.__post_init__"),
    ("contracts/metric.py", "source_vertex_of_chart_vertex"),
    ("contracts/metric.py", "GridSnappingLawV1.snaps_source"),
    ("contracts/metric.py", "GridSnappingLawV1.snaps_constructions"),
    ("source_grid.py", "chart_grid_for"),
    ("reference/validation.py", "validate_reference_geometry_payload"),
    ("reference/validation.py", "validate_reference_geometry_certificates"),
    ("reference/validation.py", "_validate_angle_certificates"),
    ("reference/validation.py", "_support_direction"),
    ("reference/validation.py", "_interval_contains_oriented_support_delta"),
    ("reference/validation.py", "_interval_contains_oriented_support_delta_exact"),
    ("reference/metric.py", "ExactPlanarMetric.from_descriptor"),
    ("reference/metric.py", "_chart_grid"),
    ("reference/metric.py", "_matrix"),
    ("reference/metric.py", "_scalar"),
    ("reference/symbolic_backend.py", "count"),
    ("reference/symbolic_backend.py", "backend_mode"),
)


if __name__ == "__main__":
    sys.exit(main())
