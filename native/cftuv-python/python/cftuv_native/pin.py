"""The staleness pin: which Python oracle each native operation was ported from, and whether the tree in front of it is that oracle.

A native operation is bit-for-bit equal to ONE version of the Python kernel. The kernel keeps moving (main gets new laws, new
counters), and a port that silently answers for a kernel it was never compared with is worse than no port. So each operation
carries a pin: the sha256 of exactly the oracle source files its Rust code mirrors (line endings normalised to `\\n`, so a
Windows checkout and a Linux one agree). The shim compares the live tree with the pin BEFORE it touches any state and
refuses by name when they differ (`NativePortStale`, the differing files in the message); `native_status()` reports the same
verdict per operation. There is no fallback to Python in the shim: the caller that switches backends decides what a refusal means.

Catching up with a moved oracle is a separate, deliberate step (a delta port, a corpus re-export, a new pin: `python -m
cftuv_native.pin` prints the digests of the tree it is run against). Nothing here repairs a pin.

What is pinned is the SOURCE FILES the port mirrors, not the whole kernel: an edit elsewhere (a new host law, a new
contract) does not stale a port that does not read it. A file the port mirrors only by NAME (the enum members and slot names
of the classes the result is built from) is checked structurally when the operation binds its classes (`check_shapes`).

The verdict is computed once per process per operation (the oracle in memory is the oracle on disk at that moment; a module
reload is a new process for this purpose) and kept; `refresh()` forgets it. This module imports no extension.
"""

from __future__ import annotations

import hashlib
import importlib.util
import sys
from pathlib import Path

#: The interpreters the differential tests run the ports on (`tools/native_catchup.py test`): 3.11 is the product runtime (Blender 4.5), 3.13 the
#: dev venv and the external pool Python. The kernel no longer depends on the interpreter (the sort of `ClipStageV1._ordered` and the float fold of the
#: offset normal are the explicit CPython 3.11 semantics of `_cpython311.py`, which the native ports mirror), so a version is no longer a semantic
#: reason to refuse; it is a TESTING statement.
TESTED_PYTHON = ((3, 11), (3, 13))

#: The oldest interpreter the ports answer on: the floor of the extension's stable ABI (`abi3-py311`) and of the kernel. An interpreter between or
#: above the tested ones (3.12, 3.14) is served like the tested ones; nothing in the answer or the cost depends on it.
MINIMUM_PYTHON = (3, 11)

#: Source files (relative to the `cftuv_envelope` package) the exact layer and the float filters of BOTH ports mirror.
FOUNDATION = (
    "_radicand_products.py",
    "exact_sqrt_sum.py",
    "exact_sqrt_sum_fused.py",
    "float_filter.py",
    "wavefront/faces.py",
)

#: `wavefront.coverage._coverage_at` and the support line it reads.
COVERAGE_FILES = ("wavefront/coverage.py", "wavefront/event_time.py", *FOUNDATION)

#: `materialize.clip.clip_geometry` with the cells, the snap, the tessellation predicates, the lift of a clip vertex
#: (`lift_surface.BoundSurfaceLiftV1`, `lift.sqrt_sum_binary64`, `offset_normal.blend`, `numeric.LocalPoint3V1`), the node identity
#: (`coalesce.point_key`), the refusal class (`frames.MaterializationRefusal`) and the explicit CPython 3.11 sort and float fold
#: (`_cpython311.sorted_as_cpython311` for `_ordered`, `left_fold_sum` for `blend`).
CLIP_FILES = (
    "_cpython311.py",
    "materialize/clip.py",
    "materialize/clip_cells.py",
    "materialize/clip_snap.py",
    "materialize/coalesce.py",
    "materialize/frames.py",
    "materialize/lift.py",
    "materialize/lift_surface.py",
    "materialize/offset_normal.py",
    "materialize/tessellate.py",
    "numeric.py",
    *FOUNDATION,
)

OPERATION_FILES = {"coverage": COVERAGE_FILES, "clip": CLIP_FILES}

#: `{file: sha256 of the file with CRLF turned into LF}` of the oracle the ports were compared with (the file lists of the
#: operations above overlap, a file has one digest). Regenerate by `python cftuv_native/pin.py <cftuv_envelope dir>` after a catch-up.
PINS: dict = {
    "_cpython311.py": "d0ce9f6eccb090f0ef1e0615830f0534ac5211615da0d04103934d017e97e034",
    "_radicand_products.py": "9d64abad54bd202442384fe79cc9c2f45582bcebf0781043c36968c4cf979f70",
    "exact_sqrt_sum.py": "80abe9927dd193609d32b53103aa6dd663a6bec963549fa90896c0c354fb1741",
    "exact_sqrt_sum_fused.py": "1f7f7d50a56159090eb5c33633221c7ff69cb5a02f4a0e373df949b887e9a9a4",
    "float_filter.py": "ab188717ec92dd5ed45425af9a9f98f7c4e663af82bf01b8ea7e3177775b6346",
    "materialize/clip.py": "01885fc8c425711e699e2a2e824ea61345993df5c8af394b978d8d31f4b6d606",
    "materialize/clip_cells.py": "26e94d1cea663dd21ee1468618fec57b43bb341b5e57bd8ad6e3d866320cd6bf",
    "materialize/clip_snap.py": "1f8997b3dfd6df0585475b6bb6cfcbf110888e6099da8b3120d18b6dcebe1374",
    "materialize/coalesce.py": "6a3f21b7ff98750686ad4fb6c6c14aa550068c8c8d44dd216efc4cfb21669c25",
    "materialize/frames.py": "99aff370cd9090bb700915d624da0ad2f17ac9bd36348d30b4c5f92c49bfd1f7",
    "materialize/lift.py": "ed91b42e5d6149d30469501922f8d7abfa65168d98043eccb3bdd2a87a78412b",
    "materialize/lift_surface.py": "5100062401a28741a5d501779fe6832e3c65a4f4401b393c7f4068e4b9b47da5",
    "materialize/offset_normal.py": "2461376ee7e37296b9caf4acd786375636f91bafbf3cfc066f6d858130930397",
    "materialize/tessellate.py": "f31338338e71822bcf5c0619f8cb4dabdbe7b636bdaecba03b00414cf9493c56",
    "numeric.py": "bbe162cbaab350b31928c5f2e2d8818e9c898195d1bc529c3062f4104c7de9b9",
    "wavefront/coverage.py": "68ef9f695cb3b1f9dbebdee9846a8ce12d65a7cca3c8e6717c08dd256614a238",
    "wavefront/event_time.py": "cf17d5da99d296bc951e0dbe8980967352ccc93e13fd04e98076f07e21467867",
    "wavefront/faces.py": "ab52d0a44599a902ec293c278b200300cc4140bc61cb33a2bcadf4c67bad700e",
}


class NativePortStale(RuntimeError):
    """The Python oracle moved past the one the native port was compared with: the operation refuses, it does not guess."""


class NativeUnsupportedPython(RuntimeError):
    """The interpreter is older than the floor of the ports (`MINIMUM_PYTHON`, the kernel's and the extension's own)."""


class NativePortUnsupported(RuntimeError):
    """The port declines this input by name (a plane without the table the offset normals are written into, a call that asks for the sign
    traces of the coverage template, ...): the oracle can do it, the port does not claim to."""


def kernel_root() -> Path:
    """The `cftuv_envelope` package directory of the oracle that is importable now."""

    spec = importlib.util.find_spec("cftuv_envelope")
    if spec is None or not spec.submodule_search_locations:
        raise NativePortStale("the Python oracle `cftuv_envelope` is not importable: the native ports cannot be checked against it")
    return Path(next(iter(spec.submodule_search_locations)))


def read_source(root: Path, name: str) -> bytes | None:
    """The bytes of an oracle file with `\\r\\n` turned into `\\n`; `None` when the file is gone."""

    try:
        return (root / name).read_bytes().replace(b"\r\n", b"\n")
    except FileNotFoundError:
        return None


def digest(root: Path, name: str) -> str | None:
    content = read_source(root, name)
    return None if content is None else hashlib.sha256(content).hexdigest()


def digests(operation: str, root: Path | None = None) -> dict:
    """`{file: sha256 or None}` of the files the operation mirrors, read from `root` (default: the importable oracle)."""

    root = kernel_root() if root is None else root
    return {name: digest(root, name) for name in OPERATION_FILES[operation]}


def fingerprint(operation: str, root: Path | None = None) -> str:
    """One sha256 over the operation's file digests, in the order of the file list (the pin of the operation as one number)."""

    lines = "".join(f"{name} {found}\n" for name, found in digests(operation, root).items())
    return hashlib.sha256(lines.encode("ascii")).hexdigest()


def stale_files(operation: str, root: Path | None = None) -> tuple:
    """The files of the operation whose digest differs from the pin (a gone file is named `<file> (missing)`)."""

    found = digests(operation, root)
    differing = [
        f"{name} (missing)" if found[name] is None else name
        for name in OPERATION_FILES[operation]
        if found[name] is None or found[name] != PINS.get(name)
    ]
    return tuple(differing)


def python_supported() -> bool:
    return tuple(sys.version_info[:2]) >= MINIMUM_PYTHON


_VERDICTS: dict = {}


def refresh() -> None:
    """Forgets the cached verdicts: the next call reads the oracle again."""

    _VERDICTS.clear()


def verdict(operation: str) -> tuple:
    """`("available",)`, `("unsupported_python",)` or `("stale", files)`, computed once per process per operation."""

    cached = _VERDICTS.get(operation)
    if cached is None:
        if not python_supported():
            cached = ("unsupported_python",)
        else:
            files = stale_files(operation)
            cached = ("stale", files) if files else ("available",)
        _VERDICTS[operation] = cached
    return cached


def status(operation: str) -> str:
    """`available`, `unsupported_python` or `stale(file, ...)`."""

    found = verdict(operation)
    return f"stale({', '.join(found[1])})" if found[0] == "stale" else found[0]


def native_status() -> dict:
    """`{operation: status}` for every native whole operation."""

    return {operation: status(operation) for operation in OPERATION_FILES}


def require(operation: str) -> None:
    """Returns when the operation may run; otherwise raises its named refusal. Touches no state: call it first."""

    found = verdict(operation)
    if found[0] == "unsupported_python":
        version = ".".join(str(part) for part in sys.version_info[:3])
        raise NativeUnsupportedPython(
            f"the native `{operation}` port needs CPython {'.'.join(str(part) for part in MINIMUM_PYTHON)} or newer (tested: {' and '.join('%d.%d' % item for item in TESTED_PYTHON)}), not {version}"
        )
    if found[0] == "stale":
        raise NativePortStale(
            f"the native `{operation}` port was compared with another version of the Python oracle; changed since the pin: "
            + ", ".join(found[1])
        )


def check_shapes(checks) -> None:
    """Raises `NativePortStale` naming the file when a class the result is built from no longer has the shape the Rust side builds.

    `checks` is `((file, description, ok), ...)`: the shim evaluates each `ok` against the live classes.
    """

    broken = [f"{file} ({description})" for file, description, ok in checks if not ok]
    if broken:
        raise NativePortStale("the oracle classes the native result is built from changed: " + ", ".join(broken))


def main(argv=None) -> int:
    """`python -m cftuv_native.pin [kernel/src/cftuv_envelope]`: prints the `PINS` literal of the tree and the verdict against the current pin."""

    arguments = list(sys.argv[1:] if argv is None else argv)
    root = Path(arguments[0]) if arguments else None
    found = {name: digest(kernel_root() if root is None else root, name) for name in sorted({name for names in OPERATION_FILES.values() for name in names})}
    print("PINS: dict = {")
    for name, value in found.items():
        print(f'    "{name}": "{value}",')
    print("}")
    for operation in OPERATION_FILES:
        files = stale_files(operation, root)
        print(f"# {operation}: {'matches the current pin' if not files else 'differs from the current pin: ' + ', '.join(files)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
