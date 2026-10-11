"""The contact leaf of the effective-alpha resolver as a WHOLE operation: `reference.boundary._native_contacts(context, source, boundary, frame, counts)`, native.

The unit the product replaces is the pure pair step the resolver's memo calls (`boundary._contact_candidates` -> `_native_contacts`): the binary64 prefilter `SourceContactFrame.excludes` and
`boundary_native.contact_candidates_native` for ONE pair (a source segment against a barrier segment). The memo (`ContactCandidatesMemoV1`), the bounds of alpha, the counters other than the two
the pair moves, the sympy fallback and the resolver stay Python. The leaf has no state, no budget and no memory table, so there is nothing to mirror or to restore: equality is the contacts
(`RadicalSumV1` terms with their representatives, the two texts of the point) and the delta of the two prefilter counters.

What the extension takes from Python (exact types only): the frame, once per source (`SourceContactFrame` after `ready()`: the length and the two covectors as `terms_wire` tuples, the seven
`(centre, bound)` floats of `_float_parts()`, the texts of the source start; the converted frame is kept in the frame's `native` slot), and per pair the four texts of the barrier segment. What it
does not carry it DECLINES by name (`NativePortUnsupported`, raised before anything changed): a frame the oracle refuses (the refusal is raised by the oracle's leaf), a point that is not an
`Integer(n)` / `Rational(p, q)` text, a source of another frame, an operation of the algebra that refuses, a contact with an irrational parameter (its point is written through sympy). The oracle
then computes the pair with its own counts, its own fallback and its own side effects on `SYMBOLIC_FALLBACK_COUNTS`. The contacts come back as the oracle's own classes (`RadicalSumV1`, `Fraction`,
`ExactScalar`, `ExactPlanarPoint`), built by the extension slot by slot.

A refusal of the oracle's native path (`NativeExactError`) is never produced here: it is the oracle's, in Python, for a pair the extension declined.

The mirrored code is pinned by file (`pin.EFFECTIVE_ALPHA_FILES`); the three functions of files that are NOT pinned because they change for other reasons (the call site that applies the counts, the
products of `metric.py`) are checked by the digest of their SOURCE when the classes bind (`_bind`): a change in any of them is `NativePortStale` naming the function.
"""

from __future__ import annotations

import dataclasses
import hashlib
import inspect
from fractions import Fraction

from . import pin

OPERATION = "effective_alpha"

#: The status codes of the extension's answer (`effective_alpha.rs`).
STATUS_CONTACTS = 0
STATUS_DECLINED = 2

#: `(file, qualified name, sha256 of the source text with CRLF turned into LF)` of the functions the Rust code mirrors that live in files the pin does not hold whole.
#: Regenerate by `python -m cftuv_native.effective_alpha_op` after a deliberate catch-up.
SHAPE_FUNCTIONS = (
    ("reference/boundary.py", "_native_contacts", "87e7db7c95b0c63094ec7f5fbba6a521a34c7f29daa28b1123f981756fe372f5"),
    ("reference/metric.py", "_product", "a6814b08f326f34a265b03f31ae0bc797a2ffe4daa0856865235eb169842b5e2"),
    ("reference/metric.py", "ExactPlanarMetric.dot_covector_native", "cf2d753a9549f7297ca8667f1c65537e10b11e3d35e2a26de49126119da80132"),
)

_KERNEL: list = []


def _names(cls) -> tuple:
    return tuple(item.name for item in dataclasses.fields(cls))


def source_digest(function) -> str:
    """sha256 of the source text of `function` (CRLF turned into LF): the shape of a function the pin does not hold whole."""

    return hashlib.sha256(inspect.getsource(function).replace("\r\n", "\n").encode("utf-8")).hexdigest()


def _resolve(module, qualified: str):
    found = module
    for part in qualified.split("."):
        found = getattr(found, part)
    return found


def shape_digests(modules: dict) -> dict:
    """`{(file, name): digest or None}` of the `SHAPE_FUNCTIONS`, read from the live kernel modules (`None`: the function or its source is gone)."""

    found = {}
    for file, name, _wanted in SHAPE_FUNCTIONS:
        try:
            found[(file, name)] = source_digest(_resolve(modules[file], name))
        except (AttributeError, OSError, TypeError, KeyError):
            found[(file, name)] = None
    return found


def _bind(native_exact, planar_types, boundary_native, boundary, metric) -> tuple:
    """`(terms_wire, (Fraction, RadicalSumV1, ExactScalar, ExactPlanarPoint))` after checking the kernel has the shape the Rust side reads (`NativePortStale` otherwise)."""

    modules = {"reference/boundary.py": boundary, "reference/metric.py": metric}
    live = shape_digests(modules)
    frame = boundary_native.SourceContactFrame
    pin.check_shapes(
        (
            ("reference/native_exact.py", "RadicalSumV1 is a dataclass of one field `terms`", _names(native_exact.RadicalSumV1) == ("terms",)),
            ("reference/planar_types.py", "ExactScalar is a dataclass of one field `expression`", _names(planar_types.ExactScalar) == ("expression",)),
            ("reference/planar_types.py", "ExactPlanarPoint is a dataclass of two fields `x`, `y`", _names(planar_types.ExactPlanarPoint) == ("x", "y")),
            (
                "reference/planar_types.py",
                "the classes of a contact are built without their `__init__`: none of them has a `__post_init__`",
                not any(hasattr(cls, "__post_init__") for cls in (native_exact.RadicalSumV1, planar_types.ExactScalar, planar_types.ExactPlanarPoint)),
            ),
            ("fractions.py", "Fraction keeps its parts in _numerator and _denominator", hasattr(Fraction(1, 2), "_numerator") and hasattr(Fraction(1, 2), "_denominator")),
            (
                "reference/boundary_native.py",
                "SourceContactFrame holds `native`, `refusal`, `length`, `tangent_covector`, `normal_covector` and has `ready` and `_float_parts`",
                all(hasattr(frame, name) for name in ("native", "refusal", "length", "tangent_covector", "normal_covector", "ready", "_float_parts", "_source")),
            ),
            *((file, f"{name} is the function the Rust code mirrors", live[(file, name)] == wanted) for file, name, wanted in SHAPE_FUNCTIONS),
        )
    )
    return native_exact.terms_wire, (Fraction, native_exact.RadicalSumV1, planar_types.ExactScalar, planar_types.ExactPlanarPoint)


def _classes() -> tuple:
    """`(terms_wire, classes)` from the live kernel (a reloaded kernel is a new set of classes); the shape is checked when they change."""

    from cftuv_envelope.reference import boundary, boundary_native, metric, native_exact, planar_types

    if _KERNEL and _KERNEL[0][0] is native_exact and _KERNEL[0][1] is boundary_native and _KERNEL[0][2] is boundary and _KERNEL[0][3] is metric:
        return _KERNEL[0][4]
    bound = _bind(native_exact, planar_types, boundary_native, boundary, metric)
    _KERNEL[:] = [(native_exact, boundary_native, boundary, metric, bound)]
    return bound


def _bind_frame(core, source, frame):
    """The converted frame of `frame`, kept in its `native` slot; `NativePortUnsupported` when the port does not carry it."""

    terms_wire, classes = _classes()
    if frame._source is not source:
        raise pin.NativePortUnsupported(f"the native `{OPERATION}` port does not carry this pair: the frame belongs to another source")
    frame.ready()  # the zero vector of a source is the oracle's `ValueError`, raised here as the oracle's `excludes` raises it
    if frame.refusal is not None:
        raise pin.NativePortUnsupported(f"the native `{OPERATION}` port does not carry this pair: the source frame refused ({type(frame.refusal).__name__})")
    floats = frame._float_parts()
    start = source.start
    status, handle = core.alpha_frame(
        (start.x.expression, start.y.expression),
        terms_wire(frame.length),
        (terms_wire(frame.tangent_covector[0]), terms_wire(frame.tangent_covector[1])),
        (terms_wire(frame.normal_covector[0]), terms_wire(frame.normal_covector[1])),
        None if floats is False else list(floats),
        *classes,
    )
    if status != STATUS_CONTACTS:
        raise pin.NativePortUnsupported(f"the native `{OPERATION}` port does not carry this frame: {handle}")
    frame.native = handle
    return handle


def effective_alpha_contacts(core, source, boundary, frame):
    """`(prefilter rejected, prefilter passed, contacts)` of the pair: what the oracle's `_native_contacts` adds to `counts` and returns, answered by the extension `core` (the shim hands `_core`).

    `contacts` is the oracle's tuple of `(alpha, station, point)` with `RadicalSumV1` for alpha and station and an `ExactPlanarPoint` of two `ExactScalar` texts. Raises `NativePortStale` when the pinned
    oracle moved, `NativePortUnsupported` when the port does not carry the pair (nothing changed anywhere: the caller runs the oracle), and the oracle's `ValueError` for the zero vector of a source.
    """

    pin.require(OPERATION)
    handle = frame.native
    if handle is None or frame._source is not source:
        handle = _bind_frame(core, source, frame)
    segment = boundary.segment
    start, end = segment.start, segment.end
    answer = handle.pair(start.x.expression, start.y.expression, end.x.expression, end.y.expression)
    if answer[0] != STATUS_CONTACTS:
        raise pin.NativePortUnsupported(f"the native `{OPERATION}` port does not carry this pair: {answer[1]}")
    return answer[1], answer[2], answer[3]


def main() -> int:
    """`python -m cftuv_native.effective_alpha_op`: prints the `SHAPE_FUNCTIONS` literal of the importable kernel and the verdict against the current one."""

    from cftuv_envelope.reference import boundary, metric

    live = shape_digests({"reference/boundary.py": boundary, "reference/metric.py": metric})
    print("SHAPE_FUNCTIONS = (")
    for (file, name), digest in live.items():
        print(f'    ("{file}", "{name}", "{digest}"),')
    print(")")
    stale = [f"{file}:{name}" for file, name, wanted in SHAPE_FUNCTIONS if live[(file, name)] != wanted]
    print("# matches the current shape" if not stale else "# differs from the current shape: " + ", ".join(stale))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
