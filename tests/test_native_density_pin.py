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

# Пин Density определён только там, где нативная операция вообще поддержана (`pin.MINIMUM_PYTHON`): ниже вердикт `unsupported_python`,
# и живой отпечаток формы там другой по самому интерпретатору (Python 3.10 повторяет унаследованный слот в `__slots__` датакласса с `slots=True`).
@pytest.fixture(autouse=True)
def _density_pins_belong_to_a_supported_interpreter():
    if not pin.python_supported():
        from native_gate import skip_for_interpreter
        skip_for_interpreter(f"Density pins belong to Python >= {pin.MINIMUM_PYTHON}")


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


def _first_dataclass_row():
    file, name, _ = next(row for row in pins.CLASSES if dataclasses.is_dataclass(pins._resolve(row[0], row[1])))
    return file, name, pins._resolve(file, name)


def test_dataclass_field_schema_change_after_bind_is_out_of_contract_and_named_at_the_next_bind():
    """DENSITY_SHAPES_BOUND_ONCE_V1: the schema is taken when the class is bound (worker start); an in-place change of a BOUND class is out of contract
    (the per-call check does not see it), and the next bind (`pin.refresh` / `unbind`) names it."""

    file, name, cls = _first_dataclass_row()
    field = dataclasses.fields(cls)[0]
    original = field.compare
    pins.unbind()
    assert pins.runtime_stale() == ()
    try:
        field.compare = not original
        assert pins.runtime_stale() == ()
        pins.unbind()
        assert file + ":" + name + " (shape)" in pins.runtime_stale()
    finally:
        field.compare = original
        pins.unbind()
    assert pins.runtime_stale() == ()


def test_dataclass_field_schema_change_before_bind_refuses_by_name(monkeypatch):
    file, name, cls = _first_dataclass_row()
    field = dataclasses.fields(cls)[0]
    original = field.compare
    pin.refresh()
    try:
        field.compare = not original
        assert file + ":" + name + " (shape)" in pins.runtime_stale()
        pin.refresh()
        monkeypatch.setitem(pin._VERDICTS, "density", ("available",))
        with pytest.raises(pin.NativePortStale, match=name.split(".")[-1]):
            pin.require("density")
    finally:
        field.compare = original
        pin.refresh()
    assert pins.runtime_stale() == ()


def test_a_bound_class_is_not_digested_again_per_call(monkeypatch):
    pins.unbind()
    assert pins.runtime_stale() == ()

    def forbidden(*args):
        raise AssertionError("schema taken again on a warm call")

    monkeypatch.setattr(pins, "_class_digest", forbidden)
    monkeypatch.setattr(pins, "_class_shape", forbidden)
    assert pins.runtime_stale() == ()


def test_a_module_attribute_naming_another_class_is_stale_on_the_next_call(monkeypatch):
    file, name, cls = _first_dataclass_row()
    assert pins.runtime_stale() == ()
    owner = pins._module(file)
    parts = name.split(".")
    for part in parts[:-1]:
        owner = getattr(owner, part)
    impostor = dataclasses.make_dataclass(cls.__name__, [("other", int)])
    monkeypatch.setattr(owner, parts[-1], impostor)
    assert file + ":" + name + " (shape)" in pins.runtime_stale()
    monkeypatch.undo()
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


# --------------------------------------------------------------------------
# DENSITY_SHAPES_BOUND_ONCE_V1 parity corpus: every pinned class, every perturbation the contract keeps, answered as the per-call reference answers
# --------------------------------------------------------------------------


def _reference_runtime_stale():
    """The per-call check before DENSITY_SHAPES_BOUND_ONCE_V1 (fc817d2d), verbatim but for the name: every class schema taken again on every call."""

    failures = []
    for package_name, wanted in pins.VERSIONS:
        try:
            if importlib.import_module(package_name).__version__ != wanted:
                failures.append(package_name + ":version")
        except (ImportError, AttributeError):
            failures.append(package_name + ":version (missing)")
    for file, qualified, wanted in pins.FUNCTIONS:
        try:
            if pins._source_digest(pins._resolve(file, qualified)) != wanted:
                failures.append(file + ":" + qualified + " (live)")
        except (ImportError, AttributeError, OSError, TypeError):
            failures.append(file + ":" + qualified + " (live missing/invalid)")
    for rows, reader, suffix in ((pins.CONSTANTS, pins._value_digest, "constant"), (pins.CLASSES, pins._class_digest, "shape")):
        for file, qualified, wanted in rows:
            try:
                value = pins._resolve(file, qualified)
                digest = pins._cached_value_digest((file, qualified), value) if suffix == "constant" else reader(value)
                if digest != wanted:
                    failures.append(file + ":" + qualified + " (" + suffix + ")")
            except (ImportError, AttributeError, OSError, TypeError, ValueError):
                failures.append(file + ":" + qualified + " (" + suffix + " missing/invalid)")
    for file, qualified, names, wanted in pins.ENUM_SUBSETS:
        try:
            if pins._cached_value_digest((file, qualified, names), pins._enum_subset(file, qualified, names)) != wanted:
                failures.append(file + ":" + qualified + " (enum subset)")
        except (ImportError, AttributeError, TypeError):
            failures.append(file + ":" + qualified + " (enum subset missing/invalid)")
    return tuple(failures)


def _owner(file, qualified):
    owner = pins._module(file)
    parts = qualified.split(".")
    for part in parts[:-1]:
        owner = getattr(owner, part)
    return owner, parts[-1]


def test_every_pinned_class_and_every_kept_perturbation_is_answered_as_the_per_call_reference(monkeypatch):
    checked = {"unchanged": 0, "rebound after bind": 0, "method code after bind": 0, "schema before bind": 0, "missing after bind": 0}

    def same(label):
        found = pins.runtime_stale()
        assert found == _reference_runtime_stale(), label
        return found

    pins.unbind()
    assert same("unchanged") == ()
    checked["unchanged"] += 1
    for file, qualified, _wanted in pins.CLASSES:
        cls = pins._resolve(file, qualified)
        owner, name = _owner(file, qualified)
        # the module attribute names another class: seen on the next call, without a rebind
        monkeypatch.setattr(owner, name, type(cls.__name__, (), {}))
        assert file + ":" + qualified + " (shape)" in same(f"{qualified} rebound")
        monkeypatch.undo()
        assert same(f"{qualified} restored") == ()
        checked["rebound after bind"] += 1
        # the attribute is gone: named missing/invalid on the next call
        monkeypatch.delattr(owner, name)
        assert file + ":" + qualified + " (shape missing/invalid)" in same(f"{qualified} deleted")
        monkeypatch.undo()
        assert same(f"{qualified} restored") == ()
        checked["missing after bind"] += 1
        # a checked method gets other code in place: seen on the next call
        for method_name in ("__post_init__", "validate_candidate"):  # literal: the checked methods of the contract, not the module's own list
            method = cls.__dict__.get(method_name)
            if method is None or not hasattr(method, "__code__"):
                continue
            original = method.__code__
            method.__code__ = (lambda self, *args, **kwargs: None).__code__
            try:
                assert file + ":" + qualified + " (shape)" in same(f"{qualified}.{method_name} code")
            finally:
                method.__code__ = original
            assert same(f"{qualified}.{method_name} restored") == ()
            checked["method code after bind"] += 1
        # the schema changed in place BEFORE the bind: named by both
        if dataclasses.is_dataclass(cls) and dataclasses.fields(cls):
            field = dataclasses.fields(cls)[0]
            original = field.compare
            pins.unbind()
            field.compare = not original
            try:
                assert file + ":" + qualified + " (shape)" in same(f"{qualified} schema before bind")
            finally:
                field.compare = original
                pins.unbind()
            assert same(f"{qualified} schema restored") == ()
            checked["schema before bind"] += 1
    assert checked["rebound after bind"] == checked["missing after bind"] == len(pins.CLASSES) and checked["method code after bind"] > 19 and checked["schema before bind"], checked
