"""Recipe and cache identity of the synthetic native corpora that `.github/workflows/native.yml` builds (jobs `corpus-clip`, `corpus-skeleton`).

The corpora are never committed: CI records them from the kernel tests, and only the cheap differential tests read them. Rebuilding them on every commit
cost 15-18 minutes of the critical path; this module makes a rebuild happen exactly when an INPUT of the recording changed.

    python tools/native_ci_corpus.py key   --kind clip|skeleton        # the cache key (+ its components) on stdout, `key=...` into $GITHUB_OUTPUT
    python tools/native_ci_corpus.py build --kind clip|skeleton --out DIR

THE KEY IS THE INPUTS, NOT A GUESS. A corpus is a function of: the kernel (its code identity `kernel_code_identity()`, the very value every corpus index
records as `kernel_identity`, and every other file under `kernel/`: the tests that make the calls, their fixtures and helpers), the recording tools (this very
file, which holds the recipe, and the import closure of the tools it runs, computed from their sources), the interpreter (the exact version: outcomes are
recorded "as the oracle gave them on THIS Python"), the pinned test dependencies, the platform and, for the clip corpus, the identity of the native TREE (its seam
calls are encoded by the shim, `cftuv_native.clip_seams`, which is the Rust extension). A key that omits an input would restore a corpus that does not describe the
code under test; so the key is computed from the files, and the tests of this module prove that every input moves it and that unrelated files do not.
No `restore-keys`: a prefix match is the same mistake with a longer name. A restored corpus is still checked by the strict differential tests: they refuse a corpus
whose `kernel_identity` is not the running kernel's.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
KINDS = ("clip", "skeleton")
#: Bump to invalidate every cached corpus by hand (a change of what the cache holds that no input file shows).
KEY_SCHEMA = "cftuv.native-corpus-cache.v1"
#: The scripts each recipe runs; their import closure inside `tools/` is part of the key.
ENTRY_SCRIPTS = {
    "clip": ("native_clip_synthetic.py",),
    "skeleton": ("native_skeleton_synthetic.py", "native_skeleton_generated.py", "native_skeleton_derive.py"),
}
#: Directories that hold files Python or the packaging tools write; never an input.
NOISE_DIRECTORIES = frozenset({"__pycache__", ".pytest_cache", "dist", "build", ".git"})
TEXT_SUFFIXES = frozenset({".py", ".json", ".toml", ".txt", ".yml", ".yaml", ".cfg", ".ini", ".md", ".rs", ".csv"})
SKELETON_SUBDIRECTORY = ("ci", "synthetic_skeleton")
#: The stages of the skeleton corpus (the kernel tests, the replay of the oracle over their calls, the derivation) run in this many processes at once. A windows-latest runner has 4 vCPUs, but
#: they do not deliver four cores of throughput: measured on CI, 3 processes took 105 s per part and 4 processes 143 s (the same work per second), so 3 it is.
CORPUS_PROCESSES = 3


def _content(path: Path) -> bytes:
    data = path.read_bytes()
    return data.replace(b"\r\n", b"\n") if path.suffix in TEXT_SUFFIXES else data


def kernel_identity(root: Path = ROOT) -> str:
    """The kernel's code identity: the SAME law as `cftuv_envelope.materialize.clip_memo.kernel_code_identity` (a test holds the equality)."""

    package = Path(root) / "kernel" / "src" / "cftuv_envelope"
    digest = hashlib.sha256()
    for current, directories, files in os.walk(package):
        directories[:] = sorted(item for item in directories if item != "__pycache__")
        for name in sorted(files):
            if name.endswith(".py"):
                path = Path(current, name)
                digest.update(path.relative_to(package).as_posix().encode())
                digest.update(path.read_bytes().replace(b"\r\n", b"\n"))
    return digest.hexdigest()[:16]


def files_digest(base: Path, files) -> str:
    """sha256 over `(relative path, content)` of `files` in path order (CRLF is no change in a text file)."""

    digest = hashlib.sha256()
    for path in sorted(files, key=lambda item: Path(item).relative_to(base).as_posix()):
        relative = Path(path).relative_to(base).as_posix().encode()
        content = _content(Path(path))
        digest.update(relative + b"\0" + str(len(content)).encode() + b"\0" + content + b"\n")
    return digest.hexdigest()


def tree_files(directory: Path) -> list[Path]:
    """Every file under `directory` that is not packaging or interpreter noise."""

    found = []
    for current, directories, names in os.walk(directory):
        directories[:] = sorted(item for item in directories if item not in NOISE_DIRECTORIES and not item.endswith(".egg-info"))
        found.extend(Path(current, name) for name in sorted(names))
    return found


def _imported_names(path: Path) -> set[str]:
    names = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
        if isinstance(node, ast.Import):
            names.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            names.add(node.module.split(".")[0])
    return names


def tool_closure(tools: Path, entries) -> list[Path]:
    """The entry scripts and every `tools/<name>.py` they import, transitively (an import anywhere in a file counts: the closure over-approximates)."""

    tools = Path(tools)
    seen: dict[str, Path] = {}
    pending = [name[:-3] if name.endswith(".py") else name for name in entries]
    while pending:
        name = pending.pop()
        path = tools / f"{name}.py"
        if name in seen or not path.is_file():
            continue
        seen[name] = path
        pending.extend(sorted(_imported_names(path)))
    return [seen[name] for name in sorted(seen)]


def requirements_digest(requirements: str) -> str:
    """The pinned dependencies as a set of tokens (order and spacing of the workflow line are no input)."""

    return hashlib.sha256("\n".join(sorted(requirements.split())).encode()).hexdigest()[:16]


def native_tree_id(root: Path = ROOT) -> str:
    """Identity of the native TREE (Rust sources and the shim): what `tools/native_build_id.py` calls `tree.id`, computed without a build."""

    spec = importlib.util.spec_from_file_location("cftuv_native_build_id_tool", Path(root) / "tools" / "native_build_id.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.tree_parts(Path(root) / "native")["id"]


def key_components(root: Path, kind: str, python_version: str, requirements: str, native_tree: str | None = None) -> dict:
    """Everything the corpus of `kind` is a function of, one value per input."""

    if kind not in KINDS:
        raise ValueError(f"kind is one of {KINDS}, not {kind!r}")
    root = Path(root)
    tools = root / "tools"
    closure = tool_closure(tools, ENTRY_SCRIPTS[kind])
    components = {
        "schema": KEY_SCHEMA,
        "kind": kind,
        "platform": sys.platform,
        "python": python_version,
        "requirements": requirements_digest(requirements),
        "kernel_identity": kernel_identity(root),
        "kernel_files": files_digest(root, tree_files(root / "kernel")),
        "tools": files_digest(root, [*closure, tools / Path(__file__).name]),
    }
    if kind == "clip":
        if native_tree is None:
            raise ValueError("the clip corpus is a function of the native tree (its seam calls are encoded by the shim): pass its identity")
        components["native_tree"] = native_tree
    return components


def cache_key(components: dict) -> str:
    """`cftuv-corpus-<kind>-<platform>-py<version>-<sha256 of all components>`: readable prefix, the digest is the identity."""

    digest = hashlib.sha256(json.dumps(components, sort_keys=True).encode()).hexdigest()[:40]
    return f"cftuv-corpus-{components['kind']}-{components['platform']}-py{components['python']}-{digest}"


def compute(root: Path, kind: str, python_version: str, requirements: str) -> tuple[str, dict]:
    components = key_components(root, kind, python_version, requirements, native_tree_id(root) if kind == "clip" else None)
    return cache_key(components), components


def build_commands(kind: str, out: Path) -> list[list[str]]:
    """The recorded recipe: exactly the commands the workflow ran inline before the cache existed."""

    tools = str(TOOLS)
    if kind == "clip":
        return [[sys.executable, str(Path(tools, "native_clip_synthetic.py")), "build", "--out", str(out), "--processes", str(CORPUS_PROCESSES)]]
    corpus = Path(out).joinpath(*SKELETON_SUBDIRECTORY)
    return [
        [sys.executable, str(Path(tools, "native_skeleton_synthetic.py")), "build", "--out", str(corpus), "--ci", "--processes", str(CORPUS_PROCESSES)],
        [sys.executable, str(Path(tools, "native_skeleton_generated.py")), "generate", "--corpus", str(corpus)],
        [sys.executable, str(Path(tools, "native_skeleton_derive.py")), "--corpus", str(corpus), "--processes", str(CORPUS_PROCESSES)],
    ]


def build(kind: str, out: Path) -> int:
    environment = dict(os.environ, PYTHONSAFEPATH="1")
    for command in build_commands(kind, out):
        print("+", " ".join(command), flush=True)
        code = subprocess.run(command, env=environment, cwd=str(ROOT)).returncode
        if code:
            return code
    return 0


def main(argv=None, environment: dict | None = None) -> int:
    environment = dict(os.environ if environment is None else environment)
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    commands = parser.add_subparsers(dest="command", required=True)
    key = commands.add_parser("key")
    key.add_argument("--kind", choices=KINDS, required=True)
    key.add_argument("--python-version", default=platform.python_version())
    key.add_argument("--requirements", default=environment.get("TEST_REQUIREMENTS", ""))
    run = commands.add_parser("build")
    run.add_argument("--kind", choices=KINDS, required=True)
    run.add_argument("--out", type=Path, required=True)
    arguments = parser.parse_args(argv)
    if arguments.command == "build":
        return build(arguments.kind, arguments.out)
    if not arguments.requirements.strip():
        print("NATIVE_CORPUS_KEY_FAILED the pinned test dependencies are an input of the key: pass --requirements or set TEST_REQUIREMENTS")
        return 1
    value, components = compute(ROOT, arguments.kind, arguments.python_version, arguments.requirements)
    print(json.dumps({"key": value, **components}, indent=1))
    if environment.get("GITHUB_OUTPUT"):
        with open(environment["GITHUB_OUTPUT"], "a", encoding="utf-8") as handle:
            handle.write(f"key={value}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
