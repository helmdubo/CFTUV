"""Cache identity of the synthetic native corpora (`tools/native_ci_corpus.py`): every input of the recording moves the key, nothing else does.

The corpora (`corpus-clip`, `corpus-skeleton` in `.github/workflows/native.yml`) are rebuilt only when the cache has no entry under the key. A key that is too NARROW restores a
corpus that does not describe the code under test (a correctness bug); one that is too wide only costs a rebuild. So each input is proved here on a copy of the tree: change it, the
key changes; and the files that are not inputs (the gate, the tests, the documents, bytecode, line endings) leave it where it was.
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "tools", ROOT / "kernel" / "src"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_ci_corpus as corpus  # noqa: E402

REQUIREMENTS = "pytest==9.1.1 sympy==1.14.0 mpmath==1.3.0 pyyaml==6.0.3"
PYTHON = "3.11.9"
TREE = "a" * 64


def _tree(tmp_path: Path) -> Path:
    """A small tree: a fake kernel, the REAL recording tools (the closure is computed from their sources) and files that are not inputs."""

    root = tmp_path / "tree"
    for relative, text in {
        "kernel/src/cftuv_envelope/__init__.py": "",
        "kernel/src/cftuv_envelope/numeric.py": "VALUE = 1\n",
        "kernel/tests/test_a.py": "def test_a():\n    pass\n",
        "kernel/tests/wavefront_cases.py": "CASES = ()\n",
        "kernel/fixtures/case.json": "{}\n",
        "kernel/pyproject.toml": "[project]\nname = 'k'\n",
        "tests/test_something.py": "def test_x():\n    pass\n",
        "docs/note.md": "text\n",
    }.items():
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(text.encode("utf-8"))  # LF whatever the platform: write_text would translate on Windows
    (root / "tools").mkdir()
    needed = {path.name for kind in corpus.KINDS for path in corpus.tool_closure(ROOT / "tools", corpus.ENTRY_SCRIPTS[kind])}
    for name in sorted(needed | {"native_ci_corpus.py", "native_ci_gate.py"}):
        shutil.copy(ROOT / "tools" / name, root / "tools" / name)
    return root


def _key(root: Path, kind: str = "clip", python: str = PYTHON, requirements: str = REQUIREMENTS, native: str = TREE) -> str:
    return corpus.cache_key(corpus.key_components(root, kind, python, requirements, native if kind == "clip" else None))


def _edit(root: Path, relative: str, suffix: str = "\n# changed\n") -> None:
    path = root / relative
    path.write_bytes(path.read_bytes() + suffix.encode())


@pytest.fixture
def tree(tmp_path):
    return _tree(tmp_path)


# --------------------------------------------------------------------------
# the identity IS the one the corpus records
# --------------------------------------------------------------------------


def test_the_kernel_identity_is_the_value_every_corpus_index_records():
    from cftuv_envelope.materialize.clip_memo import kernel_code_identity

    assert corpus.kernel_identity(ROOT) == kernel_code_identity(), "the key must use the identity the strict differential tests compare the corpus against"


def test_the_tool_closure_follows_the_imports_of_the_recording_scripts():
    clip = {path.name for path in corpus.tool_closure(ROOT / "tools", corpus.ENTRY_SCRIPTS["clip"])}
    skeleton = {path.name for path in corpus.tool_closure(ROOT / "tools", corpus.ENTRY_SCRIPTS["skeleton"])}
    assert {"native_clip_synthetic.py", "native_corpus.py", "native_clip_seams.py", "native_clip_generated.py"} <= clip
    assert {"native_skeleton_synthetic.py", "native_skeleton_generated.py", "native_skeleton_derive.py", "native_skeleton_corpus.py", "native_corpus.py"} <= skeleton
    for unrelated in ("native_ci_gate.py", "native_ci_changes.py", "native_build.py"):
        assert unrelated not in clip | skeleton, f"{unrelated} records nothing: it must not move the key"
    assert all(path.parent == ROOT / "tools" for path in corpus.tool_closure(ROOT / "tools", corpus.ENTRY_SCRIPTS["clip"]))


def test_a_transitive_import_is_part_of_the_closure(tmp_path):
    tools = tmp_path / "tools"
    tools.mkdir()
    (tools / "entry.py").write_text("import first\n", encoding="utf-8")
    (tools / "first.py").write_text("def f():\n    from second import g\n    return g\n", encoding="utf-8")
    (tools / "second.py").write_text("import json\nimport third_party\n", encoding="utf-8")
    (tools / "unrelated.py").write_text("", encoding="utf-8")
    assert [path.name for path in corpus.tool_closure(tools, ["entry.py"])] == ["entry.py", "first.py", "second.py"]


# --------------------------------------------------------------------------
# every input moves the key
# --------------------------------------------------------------------------


@pytest.mark.parametrize("kind", corpus.KINDS)
@pytest.mark.parametrize(
    "relative",
    [
        "kernel/src/cftuv_envelope/numeric.py",      # the oracle
        "kernel/tests/test_a.py",                    # the calls the recorder sees
        "kernel/tests/wavefront_cases.py",           # a helper of those tests
        "kernel/fixtures/case.json",                 # their data
        "kernel/pyproject.toml",                     # their configuration
        "tools/native_corpus.py",                    # the recorder (in the closure of both)
        "tools/native_ci_corpus.py",                 # the recipe itself
    ],
)
def test_every_input_of_the_recording_moves_the_key(tree, kind, relative):
    before = _key(tree, kind)
    _edit(tree, relative)
    assert _key(tree, kind) != before, f"{relative} is an input of the {kind} corpus but the key did not move"


def test_a_new_file_in_the_kernel_moves_the_key(tree):
    before = _key(tree, "skeleton")
    (tree / "kernel" / "tests" / "test_new.py").write_text("def test_n():\n    pass\n", encoding="utf-8")
    assert _key(tree, "skeleton") != before


def test_a_file_that_moves_does_not_keep_the_key(tree):
    before = _key(tree, "skeleton")
    (tree / "kernel" / "fixtures" / "case.json").rename(tree / "kernel" / "fixtures" / "renamed.json")
    assert _key(tree, "skeleton") != before, "the name of a file is part of what the key hashes"


@pytest.mark.parametrize(
    "script, kind",
    [("native_clip_seams.py", "clip"), ("native_clip_generated.py", "clip"), ("native_skeleton_corpus.py", "skeleton"), ("native_skeleton_generated.py", "skeleton"), ("native_skeleton_derive.py", "skeleton")],
)
def test_every_tool_of_the_closure_moves_its_key(tree, script, kind):
    before = _key(tree, kind)
    _edit(tree, f"tools/{script}")
    assert _key(tree, kind) != before


def test_the_interpreter_the_platform_the_pins_and_the_kind_move_the_key(tree, monkeypatch):
    base = _key(tree, "clip")
    assert _key(tree, "clip", python="3.11.10") != base, "the exact interpreter: outcomes are recorded as the oracle gave them on THIS Python"
    assert _key(tree, "clip", python="3.13.1") != base
    assert _key(tree, "clip", requirements=REQUIREMENTS.replace("sympy==1.14.0", "sympy==1.14.1")) != base
    assert _key(tree, "clip", requirements=REQUIREMENTS + " numpy==1.26.4") != base
    assert _key(tree, "clip", native="b" * 64) != base, "the clip corpus is encoded by the shim of the native tree"
    assert _key(tree, "skeleton") != _key(tree, "clip")
    monkeypatch.setattr(sys, "platform", "darwin" if sys.platform != "darwin" else "linux")  # whichever platform this test runs on, another one
    assert _key(tree, "clip") != base


def test_the_skeleton_corpus_is_not_a_function_of_the_native_tree(tree):
    components = corpus.key_components(tree, "skeleton", PYTHON, REQUIREMENTS)
    assert "native_tree" not in components, "the skeleton corpus is recorded by the oracle alone: a Rust-only commit must keep it"
    with pytest.raises(ValueError):
        corpus.key_components(tree, "clip", PYTHON, REQUIREMENTS)


# --------------------------------------------------------------------------
# what is not an input leaves the key alone
# --------------------------------------------------------------------------


@pytest.mark.parametrize("kind", corpus.KINDS)
def test_files_that_record_nothing_leave_the_key_alone(tree, kind):
    before = _key(tree, kind)
    _edit(tree, "tools/native_ci_gate.py")
    _edit(tree, "tests/test_something.py")
    _edit(tree, "docs/note.md", "more\n")
    (tree / "kernel" / "tests" / "__pycache__").mkdir()
    (tree / "kernel" / "tests" / "__pycache__" / "test_a.cpython-311.pyc").write_bytes(b"\x00bytecode")
    (tree / "kernel" / "dist").mkdir()
    (tree / "kernel" / "dist" / "k-0.7.0-py3-none-any.whl").write_bytes(b"wheel")
    (tree / "kernel" / "k.egg-info").mkdir()
    (tree / "kernel" / "k.egg-info" / "PKG-INFO").write_text("x", encoding="utf-8")
    assert _key(tree, kind) == before


def test_line_endings_of_text_files_are_no_input(tree):
    before = _key(tree, "skeleton")
    for path in (tree / "kernel").rglob("*.py"):
        path.write_bytes(path.read_bytes().replace(b"\n", b"\r\n"))
    assert _key(tree, "skeleton") == before, "a Windows checkout may convert line endings; the kernel identity law ignores them too"


def test_the_order_and_spacing_of_the_pins_are_no_input(tree):
    assert _key(tree, "skeleton", requirements="sympy==1.14.0   pytest==9.1.1\npyyaml==6.0.3 mpmath==1.3.0") == _key(tree, "skeleton", requirements=REQUIREMENTS)


# --------------------------------------------------------------------------
# the key as the workflow uses it
# --------------------------------------------------------------------------


def test_the_key_is_deterministic_and_fits_the_cache_key_limit(tree):
    first = _key(tree, "clip")
    assert first == _key(tree, "clip") and len(first) <= 512
    assert first.startswith("cftuv-corpus-clip-") and f"py{PYTHON}" in first


def test_the_cli_prints_the_components_and_writes_the_step_output(tmp_path, capsys):
    output = tmp_path / "github_output.txt"
    environment = {"TEST_REQUIREMENTS": REQUIREMENTS, "GITHUB_OUTPUT": str(output)}
    assert corpus.main(["key", "--kind", "skeleton", "--python-version", PYTHON], environment) == 0
    printed = json.loads(capsys.readouterr().out)
    assert printed["kind"] == "skeleton" and printed["python"] == PYTHON and printed["kernel_identity"] == corpus.kernel_identity(ROOT)
    assert output.read_text(encoding="utf-8") == f"key={printed['key']}\n"


def test_the_cli_refuses_a_key_without_the_pinned_dependencies(capsys):
    assert corpus.main(["key", "--kind", "skeleton"], {}) == 1
    assert "NATIVE_CORPUS_KEY_FAILED" in capsys.readouterr().out


def test_the_recipe_is_the_one_the_workflow_ran_inline_before_the_cache_existed(tmp_path):
    (clip,) = corpus.build_commands("clip", tmp_path)
    assert clip[1:] == [str(ROOT / "tools" / "native_clip_synthetic.py"), "build", "--out", str(tmp_path)]
    tests, generate, derive = corpus.build_commands("skeleton", tmp_path)
    where = str(tmp_path / "ci" / "synthetic_skeleton")
    assert tests[1:] == [str(ROOT / "tools" / "native_skeleton_synthetic.py"), "build", "--out", where, "--ci", "--processes", str(corpus.SKELETON_PROCESSES)]
    assert 2 <= corpus.SKELETON_PROCESSES <= 4, "a windows-latest runner has 4 vCPUs"
    assert generate[1:] == [str(ROOT / "tools" / "native_skeleton_generated.py"), "generate", "--corpus", where]
    assert derive[1:] == [str(ROOT / "tools" / "native_skeleton_derive.py"), "--corpus", where]
