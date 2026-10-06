"""The content identity of a build of the native accelerator: what `cftuv_native.native_build_id()` is made of.

Two builds of the same code are not the same bytes (the MSVC link stamps a timestamp and a PDB GUID into every `_core.pyd`), so the identity is never a hash of
the binary. It is a sha256 over the CONTENT of what makes the accelerator what it is, in two halves:

* `rust`: the sha256 of the Rust sources the extension was built from, computed at build time (`native/cftuv-python/build.rs`, rule in
  `native/cftuv-python/digest.rs`) and embedded in the extension: `Cargo.toml` and `Cargo.lock` of the workspace, and per crate its manifest, its `*.rs` next to
  the manifest (`build.rs`, `digest.rs`) and every `*.rs` under `src/`, CRLF turned into LF, in the order of the relative path. Not tests, examples or data:
  they do not link into the extension. `tools/native_build_id.py` computes the same digest of a source tree without building;
* `shim`: the sha256 of the shim's own `*.py` files (this directory, no subdirectories, no `.pyc`), computed when asked, from the files that are imported:
  for each file in the order of its name, `name 0x00 decimal length 0x00 content 0x0A`, CRLF turned into LF (a Windows checkout and a Linux one agree).
  `pin.py` is one of them, so a catch-up of the pins changes the identity.

`build_id(rust, shim_directory)` is `sha256("cftuv-native-build-id-v1\\nrust:<rust>\\nshim:<shim>\\n")`, lowercase hex: stable across relinks and rebuilds of the same
content, different whenever the native code, its manifests or the shim change. This module imports no extension: the shim passes the embedded `rust` digest in.
"""

from __future__ import annotations

import hashlib
from pathlib import Path

SCHEME = "cftuv-native-build-id-v1"

#: The directory of the shim's own files: what the `shim` half hashes.
SHIM_DIRECTORY = Path(__file__).resolve().parent


def shim_digest(directory: Path | str | None = None) -> str:
    """The sha256 of the `*.py` files of `directory` (default: the installed shim), by the rule in the module note."""

    root = SHIM_DIRECTORY if directory is None else Path(directory)
    hasher = hashlib.sha256()
    for name in sorted(path.name for path in root.glob("*.py") if path.is_file()):
        content = (root / name).read_bytes().replace(b"\r\n", b"\n")
        hasher.update(name.encode("utf-8") + b"\0" + str(len(content)).encode("ascii") + b"\0" + content + b"\n")
    return hasher.hexdigest()


def compose(rust: str, shim: str) -> str:
    """The identity of a build whose halves are `rust` and `shim`."""

    return hashlib.sha256(f"{SCHEME}\nrust:{rust}\nshim:{shim}\n".encode("ascii")).hexdigest()


def build_id(rust: str, shim_directory: Path | str | None = None) -> str:
    """The identity of the build whose embedded Rust digest is `rust`, with the shim read from `shim_directory` (default: the installed one)."""

    return compose(rust, shim_digest(shim_directory))
