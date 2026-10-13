"""The planar-grid admission of the EXPORT stage as an ACCEPT-OR-DECLINE operation: `frame_metric(...)`, native.

The unit the product replaces is the first rung of `planar_metric.build_rational_affine_planar_metric`, the one the worker climbs for every domain of the snapshot it exports: the owner patch snapped
to the grid of its window, on the exact plane of its faces, in the canonical frame of its first two independent source vertices. The Python builder stays the ORACLE: this operation returns the
`RationalAffinePlanarMetricV2` only when the patch is exactly planar after the snap and nothing else is asked of the ladder, and in every other case it DECLINES by name (`NativePortUnsupported`,
raised before anything changed) and the caller runs the Python builder exactly as it ran before the port existed: the near-planar projection, the unfolding of the whole patch or of a band, the
refusals with their texts and the exceptions are the oracle's, never the port's.

What the extension takes from Python (exact types only; see `frame.rs`): the source vertices and the faces of the owner patch, read slot by slot at the offsets a probe of a real instance found.
What it builds: the metric, from Rust, with the objects of the call (the revision, the domain, the lineage, the grid law and the id objects of the vertices) reused. What it declines by name: a grid
law that does not snap the source, the plane-preserving retry law, snapped vertices that are not in one plane, a window or a search that has no scale, a snap that breaks the embedding, a patch
without a canonical frame, a process in a state the port does not mirror (a decimal context narrower than the 28 digits of the angle certificates).

The mirrored code is pinned: whole files by `pin.FRAME_FILES`, and the functions of files that change for other reasons by the digest of their source when the classes bind (`SHAPE_FUNCTIONS`:
a change in any of them is `NativePortStale` naming the function). `python -m cftuv_native.frame_op` prints the table of the tree it is run against.
"""

from __future__ import annotations

import hashlib
import inspect
import sys
from decimal import getcontext

from . import pin

OPERATION = "frame"

#: The status codes of the extension's answer (`frame.rs`).
STATUS_FRAME = 0
STATUS_DECLINED = 2

#: `Reason::code` of the extension (`cftuv-frame/src/model.rs`), and the reasons the shim itself names.
REASONS = {1: "LAW", 2: "SHAPE", 3: "PLANE", 4: "GRID", 5: "FRAME", 6: "RANGE"}

#: The digits the angle certificates are written in (`reference.angle_measure.DECIMALS`): the count of restored right corners relies on the rational path of that file, which writes 28 digits.
DECIMAL_DIGITS = 28

#: `(file, qualified name, sha256 of the source text with CRLF turned into LF)` of the functions the Rust code mirrors that live in files the pin does not hold whole.
#: Regenerate by `python -m cftuv_native.frame_op` after a deliberate catch-up.
SHAPE_FUNCTIONS = (
    ("contracts/metric.py", "ExactRationalV1.__post_init__", "6fb1572623c251913ce7f23010a3fe7969fb013cc2259def89d378d1c48efc98"),
    ("contracts/metric.py", "GridScaleTrialV1.__post_init__", "9ed83f75e6cd4c26dacc1dc20b334810edfbb12e21b52e91e5a4448f5845b823"),
    ("contracts/metric.py", "IntegerGridCertificateV1.__post_init__", "f621df2b496ada888617a3865bb3f367d3b8abdf4d15909c94041d8be6df0650"),
    ("contracts/metric.py", "IntegerGridCertificateV1._check_trials", "7904a8cc7bd4499a2fc52b4f102f9691efef7bac1c3f9ccd88588a2b59c7dcd0"),
    ("contracts/metric.py", "IntegerGridCertificateV1.scale_law", "1ded09f006473b30fd2deefa56e392dde3a1efc531988fbf36ee54159b1fe51e"),
    ("contracts/metric.py", "IntegerGridCertificateV1.scales_skipped_for_plane", "c679b30ae59eff4c63a1f524d49792a50cdd8db64e711bf4ec071904cb3a5386"),
    ("contracts/metric.py", "ExactSourcePlaneCertificateV1.__post_init__", "042b8a92141f012be512cce01062a0e4b0bf680b6c6c94c2a6ee7dfb15dcc59f"),
    ("contracts/metric.py", "GridSnappingLawV1.snaps_source", "9843f6c06ef0aa263632311568d36d3da0259554bac27fbae2fb7a1374657f68"),
    ("ids.py", "OpaqueId.__post_init__", "e0f93f57af960b5533329678d2d2b0c2009e19689bd50ab4c013fe911c823bc3"),
    ("reference/angle_measure.py", "angular_fraction_of_pi", "f2bfc5c4ee10ff42534149c4a4ac9aa908d1d3f4be1cc5f9c634ccaaeb02f0f1"),
    ("reference/angle_measure.py", "rational_intervals_over_pi", "d4f7101a8b53c0cd386ba10988d9d10362d1cf11cd9501c66fcdc77f6e35e820"),
    ("reference/angle_measure.py", "_rational_interval", "19f484d675b821a4033b06c61e8e005527aa2b3953f002cf992a2ccfab64930f"),
    ("reference/angle_measure.py", "_scaled", "64abdf1924209639c2cca5d8492313d1c495c0aceb2a14d7d548510c1920e8ed"),
)

#: The classes the extension reads and builds, by module: `{module: names}`; the dictionary handed to `FrameOp` maps each name to the live class.
CLASS_NAMES = {
    "cftuv_envelope.ids": ("SourceVertexId", "SourceFaceId", "PatchId", "PhysicalEdgeId", "ReferenceMetricId", "PlanarityCertificateId"),
    "cftuv_envelope.contracts.analysis": ("SourceVertexV1",),
    "cftuv_envelope.contracts.surface": ("SourceFaceV1",),
    "cftuv_envelope.numeric": ("LocalPoint3V1",),
    "cftuv_envelope.contracts.metric": (
        "RationalAffinePlanarMetricV2", "ExactSourcePlaneCertificateV1", "IntegerGridCertificateV1", "GridScaleTrialV1", "ExactRationalV1", "ExactPoint2V1", "ExactPoint3V1", "ExactVector3V1",
        "ExactMatrix2V1", "ExactSourceVertexCoordinateV2",
    ),
}

_KERNEL: list = []


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
        "contracts/metric.py": "cftuv_envelope.contracts.metric",
        "ids.py": "cftuv_envelope.ids",
        "reference/angle_measure.py": "cftuv_envelope.reference.angle_measure",
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
    return table


def _members() -> dict:
    """The enum members the result carries, by role."""

    from cftuv_envelope.contracts.metric import (
        AffineChartOrientationV1,
        AffineFrameSelectionLawV1,
        AffineReconstructionLawV1,
        GridScaleSearchOrderV1,
        GridScaleTrialOutcomeV1,
        GridWindowOutcomeV1,
        PlanarityAdmissionLawV1,
    )

    return {
        "chart_ccw": AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH,
        "chart_cw": AffineChartOrientationV1.COORDINATE_CW_MATCHES_OWNER_PATCH,
        "frame_law": AffineFrameSelectionLawV1.CANONICAL_SOURCE_VERTEX_BASIS_V1,
        "admission_law": PlanarityAdmissionLawV1.EXACT_SOURCE_PLANE_V1,
        "reconstruction_law": AffineReconstructionLawV1.O_PLUS_U_A_PLUS_V_B_V1,
        "window_outcome": GridWindowOutcomeV1.WINDOW_AVAILABLE,
        "search_order": GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1,
        "trial_restored": GridScaleTrialOutcomeV1.RELATIONS_RESTORED,
        "trial_not_restored": GridScaleTrialOutcomeV1.RELATIONS_NOT_RESTORED,
    }


def _bind(core) -> tuple:
    """`(handle, signature)`: the extension's reader/builder built on the live classes, after checking the oracle functions it mirrors are the ones it was written against."""

    from cftuv_envelope.reference import angle_measure
    from cftuv_envelope.source_grid import GRID_SCALE_SEARCH_ORDER
    from cftuv_envelope.contracts.metric import GridScaleSearchOrderV1

    modules = _kernel_modules()
    live = shape_digests(modules)
    pin.check_shapes(
        (
            *((file, f"{name} is the function the Rust code mirrors", live[(file, name)] == wanted) for file, name, wanted in SHAPE_FUNCTIONS),
            ("reference/angle_measure.py", f"DECIMALS is {DECIMAL_DIGITS}", angle_measure.DECIMALS == DECIMAL_DIGITS),
            ("source_grid.py", "the search order is FINEST_ADMISSIBLE_FIRST_V1", GRID_SCALE_SEARCH_ORDER is GridScaleSearchOrderV1.FINEST_ADMISSIBLE_FIRST_V1),
        )
    )
    table = _class_table()
    try:
        handle = core.FrameOp(table, _members())
    except TypeError as error:
        if "raw slot access was forced" in str(error):
            raise  # the refusal of a forced slot mode (CFTUV_NATIVE_SLOTS=raw): not a change of the oracle's classes
        raise pin.NativePortStale(f"the oracle classes the native frame is built from changed: {error}") from None
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


def frame_metric(core, *, source_vertices, source_faces, owner_patch_id, source_revision, patch_domain_id, source_lineage, grid_policy, grid_scale_law=None):
    """The `RationalAffinePlanarMetricV2` of the owner patch (the value of `build_rational_affine_planar_metric(...)` for a patch exactly planar after the source snap), or `NativePortUnsupported` naming why the port did not build it.

    Raises `NativePortStale` when a pinned oracle moved; nothing is touched either way.
    """

    pin.require(OPERATION)
    if grid_scale_law is not None:
        raise pin.NativePortUnsupported("frame declined: LAW: a grid scale law is named (the retry law is the oracle's)")
    if getcontext().prec < DECIMAL_DIGITS:
        raise pin.NativePortUnsupported(f"frame declined: MODE: the decimal context has {getcontext().prec} digits, the angle certificates are written in {DECIMAL_DIGITS}")
    handle = _handle(core)
    answer = handle.compute(source_vertices, source_faces, owner_patch_id, source_revision, patch_domain_id, source_lineage, grid_policy, grid_policy.snaps_source)
    if answer[0] == STATUS_FRAME:
        return answer[1]
    raise pin.NativePortUnsupported(f"frame declined: {REASONS.get(answer[1], answer[1])}: {answer[2]}")


def warm(core) -> bool:
    """Binds the reader to the live kernel classes ahead of the first domain: the digests of the mirrored functions (`inspect.getsource`, on CPython 3.11 the pure-Python tokenizer) and the class
    table are the one-time cost of a process. A pool worker pays it in its start (`warm_worker_identities`), not in the seconds of its first domain.

    Raises what a call would raise (`NativePortStale` when a pinned oracle moved): the caller decides that the first domain then names it.
    """

    pin.require(OPERATION)
    _handle(core)
    return True


def raw_layouts(core) -> dict:
    """`{class: raw slot layout confirmed}` of the reader/builder bound to the live kernel (the CI step of the slot modes reads it)."""

    pin.require(OPERATION)
    return dict(_handle(core).raw_layouts())


def last_timings() -> tuple:
    """`(read ns, compute ns, build ns)` of the last call (zeros before the first call)."""

    return _KERNEL[0][1].last_timings() if _KERNEL else (0, 0, 0)


#: The functions of shared files the Rust code mirrors (their digests are `SHAPE_FUNCTIONS`).
FUNCTIONS = (
    ("contracts/metric.py", "ExactRationalV1.__post_init__"),
    ("contracts/metric.py", "GridScaleTrialV1.__post_init__"),
    ("contracts/metric.py", "IntegerGridCertificateV1.__post_init__"),
    ("contracts/metric.py", "IntegerGridCertificateV1._check_trials"),
    ("contracts/metric.py", "IntegerGridCertificateV1.scale_law"),
    ("contracts/metric.py", "IntegerGridCertificateV1.scales_skipped_for_plane"),
    ("contracts/metric.py", "ExactSourcePlaneCertificateV1.__post_init__"),
    ("contracts/metric.py", "GridSnappingLawV1.snaps_source"),
    ("ids.py", "OpaqueId.__post_init__"),
    ("reference/angle_measure.py", "angular_fraction_of_pi"),
    ("reference/angle_measure.py", "rational_intervals_over_pi"),
    ("reference/angle_measure.py", "_rational_interval"),
    ("reference/angle_measure.py", "_scaled"),
)


def main(argv=None) -> int:
    """Prints the `SHAPE_FUNCTIONS` table of the tree it runs against (the functions listed in `FUNCTIONS`)."""

    modules = _kernel_modules()
    print("SHAPE_FUNCTIONS = (")
    for file, name in FUNCTIONS:
        digest = source_digest(_resolve(modules[file], name))
        print(f'    ("{file}", "{name}", "{digest}"),')
    print(")")
    return 0


if __name__ == "__main__":
    sys.exit(main())
