"""Живые Density pins проверяются без загрузки расширения и без вычисления геометрии."""
from __future__ import annotations

import dataclasses
import hashlib
import importlib
import importlib.machinery
from pathlib import Path
import sys
import types

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
PACKAGE = "_density_pin_test"
package = types.ModuleType(PACKAGE)
package.__path__ = [str(ROOT / "native/cftuv-python/python/cftuv_native")]
package.__spec__ = importlib.machinery.ModuleSpec(PACKAGE, loader=None, is_package=True)
sys.modules[PACKAGE] = package
pins = importlib.import_module(PACKAGE + ".density_pin")
pin = importlib.import_module(PACKAGE + ".pin")


def test_live_closure_matches_loaded_oracle():
    assert pins.runtime_stale() == ()


def test_warm_checks_do_not_read_source_again(monkeypatch):
    assert pins.runtime_stale() == ()
    def forbidden(*args):
        raise AssertionError("warm source read")
    monkeypatch.setattr(pins.inspect, "getsource", forbidden)
    assert pins.runtime_stale() == ()


def test_in_place_code_replacement_is_stale_and_restore_recovers():
    function = pins._resolve("reference/common.py", "stable_id")
    original = function.__code__
    assert pins.runtime_stale() == ()
    try:
        function.__code__ = (lambda *args: "wrong").__code__
        assert "reference/common.py:stable_id (live)" in pins.runtime_stale()
    finally:
        function.__code__ = original
    assert pins.runtime_stale() == ()


def test_mutable_constant_change_after_warm_is_stale():
    table = pins._resolve("_density_policy.py", "CANONICAL_ROTATION_TABLE")
    assert pins.runtime_stale() == ()
    original = dict(table)
    try:
        table.clear()
        assert "_density_policy.py:CANONICAL_ROTATION_TABLE (constant)" in pins.runtime_stale()
    finally:
        table.update(original)
    assert pins.runtime_stale() == ()


def test_dataclass_field_schema_change_after_warm_is_stale():
    file, name, _ = next(row for row in pins.CLASSES if dataclasses.is_dataclass(pins._resolve(row[0], row[1])))
    cls = pins._resolve(file, name)
    field = dataclasses.fields(cls)[0]
    original = field.compare
    assert pins.runtime_stale() == ()
    try:
        field.compare = not original
        assert file + ":" + name + " (shape)" in pins.runtime_stale()
    finally:
        field.compare = original
    assert pins.runtime_stale() == ()


def test_dataclass_method_code_change_is_stale():
    file, name, _ = next(row for row in pins.CLASSES if hasattr(pins._resolve(row[0], row[1]), "__post_init__"))
    method = pins._resolve(file, name).__post_init__
    original = method.__code__
    assert pins.runtime_stale() == ()
    try:
        method.__code__ = (lambda self: None).__code__
        assert file + ":" + name + " (shape)" in pins.runtime_stale()
    finally:
        method.__code__ = original
    assert pins.runtime_stale() == ()


def test_shared_source_helper_mutation_is_scoped(tmp_path):
    root = ROOT / "kernel/src/cftuv_envelope"
    files = {row[0] for rows in (pins.FUNCTIONS, pins.SOURCE_CONSTANTS, pins.SOURCE_ENUMS) for row in rows}
    for file in files:
        target = tmp_path / file
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((root / file).read_bytes())
    assert pins.stale_sources(tmp_path) == ()
    target = tmp_path / "reference/common.py"
    text = target.read_text(encoding="utf8")
    target.write_text(text + "\n# unrelated editable host detail\n", encoding="utf8")
    assert pins.stale_sources(tmp_path) == ()
    target.write_text(text.replace("def stable_id(", "def renamed_stable_id("), encoding="utf8")
    assert "reference/common.py:stable_id (missing/invalid)" in pins.stale_sources(tmp_path)


def test_live_staleness_refuses_before_work_even_cached_available(monkeypatch):
    pin.refresh()
    monkeypatch.setitem(pin._VERDICTS, "density", ("available",))
    module = pins._module("_authoring_intent.py")
    monkeypatch.setattr(module, "CANONICAL_RESTORATION_ARTIST_ERROR", 0)
    with pytest.raises(pin.NativePortStale, match="CANONICAL_RESTORATION_ARTIST_ERROR"):
        pin.require("density")
    assert pin.status("density").startswith("stale(")
    pin.refresh()


def test_dependency_version_refuses(monkeypatch):
    module = importlib.import_module("mpmath")
    monkeypatch.setattr(module, "__version__", "future")
    assert "mpmath:version" in pins.runtime_stale()


def test_no_unrelated_shared_modules_are_whole_pinned():
    assert pin.DENSITY_FILES == pins.FILES
    assert pin.DENSITY_CLOSURE_DIGEST == pins.expected_closure_digest()
    for file in ("reference/common.py", "reference/compile.py", "_density_policy.py", "reference/direction_binding.py",
                 "reference/density_raw_v1.py", "reference/density_attach_v1.py", "reference/density_compile_v1.py"):
        assert file not in pins.FILES
    assert "reference/density_result_v1.py" in pins.FILES


def test_pin_metadata_remains_loadable_without_a_package():
    import importlib.util
    spec = importlib.util.spec_from_file_location("_standalone_density_pin", ROOT / "native/cftuv-python/python/cftuv_native/pin.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.OPERATION_FILES["density"] == pins.FILES


def test_transitive_numeric_file_change_is_rejected(tmp_path):
    root = ROOT / "kernel/src/cftuv_envelope"
    for file in pins.FILES:
        target = tmp_path / file
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes((root / file).read_bytes())
    assert pin.stale_files("density", tmp_path) == ()
    target = tmp_path / "reference/density_construct_v1.py"
    target.write_bytes(target.read_bytes() + b"\n# changed numeric admission\n")
    assert pin.stale_files("density", tmp_path) == ("reference/density_construct_v1.py",)


def test_density_identity_includes_expected_closure_only(monkeypatch):
    monkeypatch.setattr(pin, "digests", lambda operation, root=None: {"file": "digest"})
    previous = hashlib.sha256(b"file digest\n").hexdigest()
    for operation in pin.OPERATION_FILES:
        if operation != "density":
            assert pin.fingerprint(operation) == previous
    first = pin.fingerprint("density")
    assert first != previous
    monkeypatch.setattr(pin, "DENSITY_CLOSURE_DIGEST", "changed expected contract")
    assert pin.fingerprint("density") != first
