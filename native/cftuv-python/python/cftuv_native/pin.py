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

#: `wavefront.skeleton.build_skeleton` whole: the builder, the event loop, the superlevel transaction (snapshot, plans, the symbolic closure, the runtime commit), the motorcycle graph, the
#: event-time layer and the exact candidate view, the result classes, and the exact layer under all of them (`FOUNDATION`, and the explicit CPython 3.11 sort of the closure's comparator sorts).
SKELETON_FILES = (
    "_cpython311.py",
    "robust/predicates.py",
    "wavefront/candidate_law.py",
    "wavefront/candidate_refusal.py",
    "wavefront/cell_grid.py",
    "wavefront/digest.py",
    "wavefront/event_time.py",
    "wavefront/events.py",
    "wavefront/exact_candidate_view.py",
    "wavefront/exact_identity.py",
    "wavefront/motorcycle.py",
    "wavefront/polygon.py",
    "wavefront/poststate_span.py",
    "wavefront/proof.py",
    "wavefront/skeleton.py",
    "wavefront/superlevel.py",
    "wavefront/superlevel_closure.py",
    "wavefront/superlevel_fixed_point.py",
    "wavefront/superlevel_germ.py",
    "wavefront/superlevel_snapshot.py",
    "wavefront/symbolic_component.py",
    "wavefront/symbolic_edge_closure.py",
    "wavefront/symbolic_edge_fixed_point.py",
    "wavefront/symbolic_f0_overlay.py",
    "wavefront/symbolic_initial_composition.py",
    "wavefront/symbolic_junction_contacts.py",
    "wavefront/symbolic_junction_fixed_point.py",
    "wavefront/symbolic_junction_normalize.py",
    "wavefront/symbolic_mixed_generation.py",
    "wavefront/symbolic_overlay.py",
    "wavefront/symbolic_runtime_commit.py",
    "wavefront/symbolic_sparse_ports.py",
    "wavefront/symbolic_split_endpoint.py",
    "wavefront/symbolic_superlevel_coordinator.py",
    *FOUNDATION,
)

OPERATION_FILES = {"coverage": COVERAGE_FILES, "clip": CLIP_FILES, "skeleton": SKELETON_FILES}

#: `{file: sha256 of the file with CRLF turned into LF}` of the oracle the ports were compared with (the file lists of the
#: operations above overlap, a file has one digest). Regenerate by `python cftuv_native/pin.py <cftuv_envelope dir>` after a catch-up.
PINS: dict = {
    "_cpython311.py": "d0ce9f6eccb090f0ef1e0615830f0534ac5211615da0d04103934d017e97e034",
    "_radicand_products.py": "9d64abad54bd202442384fe79cc9c2f45582bcebf0781043c36968c4cf979f70",
    "exact_sqrt_sum.py": "80abe9927dd193609d32b53103aa6dd663a6bec963549fa90896c0c354fb1741",
    "exact_sqrt_sum_fused.py": "1f7f7d50a56159090eb5c33633221c7ff69cb5a02f4a0e373df949b887e9a9a4",
    "float_filter.py": "ab188717ec92dd5ed45425af9a9f98f7c4e663af82bf01b8ea7e3177775b6346",
    "materialize/clip.py": "60900b414a87a1b8eb9240cb91d6f58c2c9f679143003f51d572d568e883d773",
    "materialize/clip_cells.py": "26e94d1cea663dd21ee1468618fec57b43bb341b5e57bd8ad6e3d866320cd6bf",
    "materialize/clip_snap.py": "1f8997b3dfd6df0585475b6bb6cfcbf110888e6099da8b3120d18b6dcebe1374",
    "materialize/coalesce.py": "6a3f21b7ff98750686ad4fb6c6c14aa550068c8c8d44dd216efc4cfb21669c25",
    "materialize/frames.py": "99aff370cd9090bb700915d624da0ad2f17ac9bd36348d30b4c5f92c49bfd1f7",
    "materialize/lift.py": "ed91b42e5d6149d30469501922f8d7abfa65168d98043eccb3bdd2a87a78412b",
    "materialize/lift_surface.py": "5100062401a28741a5d501779fe6832e3c65a4f4401b393c7f4068e4b9b47da5",
    "materialize/offset_normal.py": "2461376ee7e37296b9caf4acd786375636f91bafbf3cfc066f6d858130930397",
    "materialize/tessellate.py": "f31338338e71822bcf5c0619f8cb4dabdbe7b636bdaecba03b00414cf9493c56",
    "numeric.py": "bbe162cbaab350b31928c5f2e2d8818e9c898195d1bc529c3062f4104c7de9b9",
    "robust/predicates.py": "913a134bb932fa6579072fa82549d086ad975c3b552e0a7d18953526ffe75ff8",
    "wavefront/candidate_law.py": "d8a84854151b170cc3543ad53d4b0db2eeafb7587cc68e45500e6e4df36fd4c0",
    "wavefront/candidate_refusal.py": "b1ca2217abbfe87286ef49ca1bb17c5121e1889b94700bcbeed9cf4e2aada787",
    "wavefront/cell_grid.py": "e2ef50ff61222345e338ca9836f4281350007039daf40292f383ecba8f60030a",
    "wavefront/coverage.py": "68ef9f695cb3b1f9dbebdee9846a8ce12d65a7cca3c8e6717c08dd256614a238",
    "wavefront/digest.py": "d0b56d73e0da58d196e72541208d7bfd3c91d5d46449573076184ec4b782432f",
    "wavefront/event_time.py": "cf17d5da99d296bc951e0dbe8980967352ccc93e13fd04e98076f07e21467867",
    "wavefront/events.py": "e69a1328c2aac9001f7d0830a6fe5d644d61195e1fb063058584717da288067f",
    "wavefront/exact_candidate_view.py": "64fdb7a4256dfe3e5d6858cb6a29bd18a3b61ab0499d84b1efedf6bf3001a579",
    "wavefront/exact_identity.py": "61479908d3973c37ff45067664b036c4d95ec0df469249074724ff6b13344fde",
    "wavefront/faces.py": "ab52d0a44599a902ec293c278b200300cc4140bc61cb33a2bcadf4c67bad700e",
    "wavefront/motorcycle.py": "4460f277c899186319b405187025b60692187a33d95b14d80d75d907e244866f",
    "wavefront/polygon.py": "d3fe537c884581844cbb62f626af79e6c085332ee53ba647100109e40c84c408",
    "wavefront/poststate_span.py": "37d4b30ccfbe41c09a7071b225148fd0f7a23593fc88637463503a8e5adb4a77",
    "wavefront/proof.py": "093e9b8de7b8e88687f0ce24d184cb434b608ebab2161b9c4e2a8721fa465de0",
    "wavefront/skeleton.py": "c3821dff47eb248adf3e09175b102a9d06c40f6cc317fcfcfd8fcdf927c984b2",
    "wavefront/superlevel.py": "10ff96083fb0b8f8f9868c395e1bec08b1a849125780681a8439002b7921960c",
    "wavefront/superlevel_closure.py": "8d4db91b5635cbb277e3cd3f31381756e395f830ed5bdfc80ae620f5c24f9352",
    "wavefront/superlevel_fixed_point.py": "cda43884788569ff63f4b44b857bd96b158b477ad9cea535486c9d3eb0b7d533",
    "wavefront/superlevel_germ.py": "7063de9c4497322fe1282b3c511a266fe9c20d361be9f35b5febd869778aee02",
    "wavefront/superlevel_snapshot.py": "8a9b948a0b91074b56b5bc780e38a22c33d9c234b21081df4b013f866ce2b38d",
    "wavefront/symbolic_component.py": "3aa84b099a7b676d992d3121410cf09be7715ff2995f6fe2acf7648243cfd84e",
    "wavefront/symbolic_edge_closure.py": "a65b1e50cf870a0f0dda4b10ed83f1b48074e5c3031a1ed36b0a00fb65db6240",
    "wavefront/symbolic_edge_fixed_point.py": "fb375a95860d997289ac57c8a8fa309a5fd45de21f20c4751f877ff17f64779a",
    "wavefront/symbolic_f0_overlay.py": "20abfc7826fa64b30a622095f1e3aa9eea9f668c89fc93c327a93cd0f36eaa87",
    "wavefront/symbolic_initial_composition.py": "725d5fe61e7bd8e1720ac3ae875f6c6966dca9fea6cd1fa0fa84522bdfd22c63",
    "wavefront/symbolic_junction_contacts.py": "94190d9ddc713a7b86db7946327e96c2b4c7790715f0fa21d495b292ec3a4380",
    "wavefront/symbolic_junction_fixed_point.py": "f71bcebe73579ef221244f9bd9b101a6785da5dacf28af7b2fc2f9f4b7059659",
    "wavefront/symbolic_junction_normalize.py": "613a044af6d235185634d4dc4c1c254495ee5905a8628d7e5e7bb185bd5a3b5a",
    "wavefront/symbolic_mixed_generation.py": "ae4e976a9c60e0cc7b39f2c1e7e60646039df05befde2a97c95a6181b3560cbe",
    "wavefront/symbolic_overlay.py": "3a61f6a67868595446504785248ea74ba43f4eb0fa044480dce1eaf2bcd8ed3f",
    "wavefront/symbolic_runtime_commit.py": "047ab115e585ef3952745ec6d710339140bf544068a54a54f804d842fdad4552",
    "wavefront/symbolic_sparse_ports.py": "c50aa17461fcf80880992c92ba8f3ba6e22a1eb32c5c2e343d05cab78874b011",
    "wavefront/symbolic_split_endpoint.py": "406d76af4cf802e5e93eb519c7f41f8fe3eeee0e4e33099f672feb20ba82643f",
    "wavefront/symbolic_superlevel_coordinator.py": "c5a724ec5c3272a6555d052341faede65332f1dc9dd5fadac0233f981ff52165",
}


class NativePortStale(RuntimeError):
    """The Python oracle moved past the one the native port was compared with: the operation refuses, it does not guess."""


class NativeUnsupportedPython(RuntimeError):
    """The interpreter is older than the floor of the ports (`MINIMUM_PYTHON`, the kernel's and the extension's own)."""


class NativePortUnsupported(RuntimeError):
    """The port declines this input by name (a plane without the table the offset normals are written into, ...): the oracle can do it, the port does not claim to."""


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
