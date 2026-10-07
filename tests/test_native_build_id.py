"""Личность сборки нативного ускорителя: `cftuv_native.native_build_id()` — содержательный отпечаток, а не байты `_core.pyd`.

MSVC пишет в каждую сборку метку времени и GUID PDB, поэтому две сборки одного кода различаются побайтно и хеш бинарника личностью не служит. Личность — sha256 от двух
половин: `rust` (отпечаток исходников Rust, вшитый при сборке `build.rs`: `Cargo.toml`, `Cargo.lock`, манифесты крейтов, `build.rs`, `digest.rs`, `src/**/*.rs`; CRLF -> LF) и
`shim` (отпечаток `*.py` шима, берётся при вызове; CRLF -> LF). Проверяется: устойчивость и вид, что смена файла шима меняет личность (на копии шима), что алгоритм отпечатка Rust один у
расширения и у `tools/native_build_id.py` (на дереве, которое строит сам тест: что хешируется и что нет, CRLF, порядок), и что вшитый отпечаток равен отпечатку дерева, из которого
расширение собрано (пропуск с названной причиной, если расширение собрано из другого дерева: `python tools/native_build_id.py`).

Модуль пропускается с названной причиной, пока расширение не собрано (`python tools/native_build.py`).
"""

from __future__ import annotations

import re
import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "tools",):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (личность сборки не проверена)",
        allow_module_level=True,
    )

import native_build_id as tool  # noqa: E402
from cftuv_native import buildid  # noqa: E402

SHIM_SOURCE = ROOT / "native" / "cftuv-python" / "python" / "cftuv_native"
HEX = re.compile(r"[0-9a-f]{64}")


def copy_shim(destination: Path) -> Path:
    destination.mkdir(parents=True, exist_ok=True)
    for path in Path(buildid.SHIM_DIRECTORY).glob("*.py"):
        shutil.copyfile(path, destination / path.name)
    return destination


# --------------------------------------------------------------------------
# Вид и устойчивость
# --------------------------------------------------------------------------


def test_the_build_id_is_a_stable_lowercase_sha256_and_is_made_of_its_two_halves():
    first, second = cftuv_native.native_build_id(), cftuv_native.native_build_id()
    parts = cftuv_native.native_build_parts()
    assert first == second == parts["id"] and HEX.fullmatch(first)
    assert HEX.fullmatch(parts["rust"]) and HEX.fullmatch(parts["shim"]) and parts["rust"] != parts["shim"]
    assert parts["id"] == buildid.compose(parts["rust"], parts["shim"])
    assert parts["version"] == cftuv_native.native_version()
    assert cftuv_native.native_build_parts() is not cftuv_native.native_build_parts(), "a caller's edit of the answer does not reach the cache"


def test_the_build_id_is_not_the_hash_of_the_binary():
    """Личность — содержимое: у `_core` нет её в своих байтах (они меняются при каждой линковке), а у шима и расширения она одна."""

    import cftuv_native as package

    binaries = list(Path(package.__file__).resolve().parent.glob("_core*"))
    assert binaries, "the extension binary sits next to the shim"
    import hashlib

    digests = {hashlib.sha256(path.read_bytes()).hexdigest() for path in binaries}
    assert cftuv_native.native_build_id() not in digests and cftuv_native.native_build_parts()["rust"] not in digests


# --------------------------------------------------------------------------
# Половина шима
# --------------------------------------------------------------------------


def test_the_shim_half_is_the_digest_of_the_shim_files_and_ignores_line_endings(tmp_path):
    copy = copy_shim(tmp_path / "shim")
    assert buildid.shim_digest(copy) == buildid.shim_digest() == cftuv_native.native_build_parts()["shim"]
    crlf = copy_shim(tmp_path / "crlf")
    for path in crlf.glob("*.py"):
        path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n"))
    assert buildid.shim_digest(crlf) == buildid.shim_digest(), "a Windows checkout and a Linux one agree"
    assert buildid.build_id("0" * 64, crlf) == buildid.build_id("0" * 64)


def test_a_changed_added_removed_or_renamed_shim_file_changes_the_id(tmp_path):
    rust = cftuv_native.native_build_parts()["rust"]
    base = buildid.build_id(rust)
    assert buildid.build_id(rust, copy_shim(tmp_path / "same")) == base
    changed = copy_shim(tmp_path / "changed")
    (changed / "cost.py").write_bytes((changed / "cost.py").read_bytes() + b"# one more line\n")
    assert buildid.build_id(rust, changed) != base, "an edit of a shim file"
    pin_edit = copy_shim(tmp_path / "pin")
    text = (pin_edit / "pin.py").read_bytes()
    assert b"PINS" in text
    (pin_edit / "pin.py").write_bytes(text.replace(b"PINS", b"PINZ", 1))
    assert buildid.build_id(rust, pin_edit) not in {base, buildid.build_id(rust, changed)}, "a catch-up of the pins is a change of the shim"
    added = copy_shim(tmp_path / "added")
    (added / "extra.py").write_text("x = 1\n")
    assert buildid.build_id(rust, added) != base, "a new shim file"
    removed = copy_shim(tmp_path / "removed")
    (removed / "numbers_oracle.py").unlink()
    assert buildid.build_id(rust, removed) != base, "a removed shim file"
    renamed = copy_shim(tmp_path / "renamed")
    (renamed / "codec.py").rename(renamed / "codec2.py")
    assert buildid.build_id(rust, renamed) != base, "the content is the same, the name is not"
    ignored = copy_shim(tmp_path / "ignored")
    (ignored / "notes.txt").write_text("not a shim file\n")
    (ignored / "__pycache__").mkdir()
    (ignored / "__pycache__" / "cost.cpython-313.pyc").write_bytes(b"\x00\x01")
    (ignored / "sub").mkdir()
    (ignored / "sub" / "deep.py").write_text("y = 2\n")
    assert buildid.build_id(rust, ignored) == base, "only the `*.py` files of the shim directory itself count"


def test_the_id_follows_the_rust_half():
    shim = buildid.shim_digest()
    first, second = buildid.compose("a" * 64, shim), buildid.compose("b" * 64, shim)
    assert first != second and HEX.fullmatch(first) and buildid.compose("a" * 64, shim) == first
    assert buildid.compose("a" * 64, "c" * 64) != first


# --------------------------------------------------------------------------
# Половина Rust: один алгоритм у расширения и у инструмента
# --------------------------------------------------------------------------


def build_tree(root: Path) -> dict:
    """Рабочий каталог `native/` в миниатюре: что хешируется и что нет. Возвращает `{метка: путь}`."""

    files = {
        "root manifest": root / "Cargo.toml",
        "root lock": root / "Cargo.lock",
        "crate a manifest": root / "crate-a" / "Cargo.toml",
        "crate a build": root / "crate-a" / "build.rs",
        "crate a lib": root / "crate-a" / "src" / "lib.rs",
        "crate a nested": root / "crate-a" / "src" / "deep" / "inner.rs",
        "crate b manifest": root / "crate-b" / "Cargo.toml",
        "crate b pyproject": root / "crate-b" / "pyproject.toml",
        "crate b digest": root / "crate-b" / "digest.rs",
        "crate b lib": root / "crate-b" / "src" / "lib.rs",
    }
    ignored = {
        "target": root / "target" / "debug" / "build.rs",
        "hidden crate": root / ".hidden" / "Cargo.toml",
        "tests": root / "crate-a" / "tests" / "case.rs",
        "examples": root / "crate-a" / "examples" / "demo.rs",
        "data": root / "crate-a" / "src" / "table.json",
        "python": root / "crate-b" / "python" / "pkg" / "__init__.py",
        "not a crate": root / "docs" / "note.rs",
        "stray file": root / "stray.rs",
    }
    for label, path in {**files, **ignored}.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(f"// {label}\r\nfn main() {{}}\r\n".encode())
    return {"hashed": files, "ignored": ignored}


def both_digests(root: Path) -> tuple:
    return cftuv_native.tree_digest(root), tool.rust_tree_digest(root)


def test_the_rust_digest_is_one_algorithm_for_the_extension_and_the_tool(tmp_path):
    tree = build_tree(tmp_path)
    found, wanted = both_digests(tmp_path)
    assert found == wanted and HEX.fullmatch(found)
    assert {relative for relative, _path in tool.rust_source_files(tmp_path)} == {
        "Cargo.toml", "Cargo.lock", "crate-a/Cargo.toml", "crate-a/build.rs", "crate-a/src/lib.rs", "crate-a/src/deep/inner.rs",
        "crate-b/Cargo.toml", "crate-b/pyproject.toml", "crate-b/digest.rs", "crate-b/src/lib.rs",
    }
    baseline = both_digests(tmp_path)[0]
    for label, path in tree["hashed"].items():
        before = path.read_bytes()
        path.write_bytes(before + b"// edited\n")
        found, wanted = both_digests(tmp_path)
        assert found == wanted != baseline, f"an edit of the {label} changes the digest"
        path.write_bytes(before)
    for label, path in tree["ignored"].items():
        before = path.read_bytes()
        path.write_bytes(before + b"// edited\n")
        found, wanted = both_digests(tmp_path)
        assert found == wanted == baseline, f"an edit of the {label} does not change the digest"
        path.write_bytes(before)
    assert both_digests(tmp_path)[0] == baseline


def test_the_rust_digest_ignores_line_endings_and_follows_names_added_files_and_content(tmp_path):
    build_tree(tmp_path)
    base = both_digests(tmp_path)[0]
    for path in list(tmp_path.rglob("*.rs")) + list(tmp_path.rglob("*.toml")) + [tmp_path / "Cargo.lock"]:
        path.write_bytes(path.read_bytes().replace(b"\r\n", b"\n"))
    assert both_digests(tmp_path) == (base, base), "CRLF and LF checkouts agree"
    (tmp_path / "crate-a" / "src" / "new.rs").write_text("fn new() {}\n")
    added = both_digests(tmp_path)
    assert added[0] == added[1] != base, "a new source file"
    (tmp_path / "crate-a" / "src" / "new.rs").rename(tmp_path / "crate-a" / "src" / "newer.rs")
    renamed = both_digests(tmp_path)
    assert renamed[0] == renamed[1] not in {base, added[0]}, "a renamed file with the same content"
    (tmp_path / "crate-c").mkdir()
    (tmp_path / "crate-c" / "Cargo.toml").write_text("[package]\n")
    assert both_digests(tmp_path)[0] == both_digests(tmp_path)[1] != renamed[0], "a new crate: its manifest is hashed"


def test_a_directory_that_is_no_workspace_is_a_named_refusal_not_a_digest(tmp_path):
    with pytest.raises(ValueError, match="not a native workspace root"):
        cftuv_native.tree_digest(tmp_path)
    with pytest.raises(FileNotFoundError, match="not a native workspace root"):
        tool.rust_tree_digest(tmp_path)


def test_the_require_switch_of_the_tool_fails_unless_the_installed_build_is_the_build_of_the_tree(monkeypatch, capsys):
    """`native_build_id.py --require` (the CI step): an absent extension, a moved Rust half, a moved shim half and a moved id are each a failure; all three equal is success."""

    tree = tool.tree_parts()

    def installed(**changed):
        monkeypatch.setattr(cftuv_native, "native_build_parts", lambda: {**tree, "version": "0", **changed})

    installed()
    assert tool.main(["--require"]) == 0
    for half in ("rust", "shim", "id"):
        installed(**{half: "0" * 64})
        assert tool.main(["--require"]) == 1, f"a moved {half} half must fail the CI step"
    assert "match: " in capsys.readouterr().out
    monkeypatch.setitem(sys.modules, "cftuv_native", None)
    assert tool.main(["--require"]) == 1, "no extension installed: the step fails, it does not pass for lack of a counterpart"
    assert tool.main([]) == 0, "without the switch the tool still only reports"


def test_the_embedded_rust_digest_is_the_digest_of_the_tree_the_extension_was_built_from():
    found, wanted = cftuv_native.native_build_parts()["rust"], tool.rust_tree_digest(ROOT / "native")
    if found != wanted:
        pytest.skip(
            "расширение собрано из другого дерева Rust, чем это (отпечатки исходников различаются): `python tools/native_build.py`; `python tools/native_build_id.py` показывает половины"
        )
    tree = tool.tree_parts()
    if buildid.shim_digest() == tree["shim"]:
        assert cftuv_native.native_build_id() == tree["id"], "the same sources and the same shim: one identity, built or computed from the tree"
