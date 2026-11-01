"""Пин нативных операций (`cftuv_native.pin`): порт отвечает только за ту версию эталона, с которой его сверяли.

Нативная операция побитово равна ОДНОЙ версии ядра на Python; пока основная ветка двигает ядро, порт, сверенный со старым, не должен молча
отвечать за новое. Пин — sha256 файлов эталона, которые порт зеркалит (`\\r\\n` -> `\\n`); шим сверяет дерево с пином ДО того, как тронет
состояние, и отказывается названным `NativePortStale` с перечнем разошедшихся файлов; `native_status()` говорит то же по операциям.

Здесь проверено: пин узнаёт изменённый, удалённый и перекодированный файл (на копии дерева и подменой читателя файла), отказ не трогает ни
бюджет, ни память, ни счётчики, ни нормали, чужая версия интерпретатора — своё названное имя, отказ точен (правка файла одной операции не красит другую),
замер отказывается, а не меряет устаревший порт. Модуль НЕ пропускается при устаревшем порте — это сам механизм.

CONTRACT_PARITY_V1 / REFERENCE_DRIFT_NOT_A_REFUSAL_V1: у стадий, где продукт — Rust (`pin.RUST_PRODUCT_OPERATIONS`), то же обнаружение даёт не отказ, а дрейф
(`native_status()["reference_drift"]`), порт отвечает; отказ по имени остаётся там, где Python ещё в продукте (здесь — `radical_v2` и `frame`).
"""

from __future__ import annotations

import shutil
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (проверка пина нативных операций пропущена)",
        allow_module_level=True,
    )

import native_corpus as nc  # noqa: E402

import cftuv_envelope.exact_sqrt_sum as exact  # noqa: E402
from cftuv_native import pin  # noqa: E402

KERNEL = ROOT / "kernel" / "src" / "cftuv_envelope"
ALL_FILES = sorted({name for names in pin.OPERATION_FILES.values() for name in names})
CLIP_ONLY = "materialize/clip_cells.py"
COVERAGE_ONLY = "wavefront/coverage.py"
SHARED = "exact_sqrt_sum.py"
SKELETON_ONLY = "wavefront/symbolic_runtime_commit.py"


@pytest.fixture(autouse=True)
def _verdicts_are_read_again():
    pin.refresh()
    with exact.isolated_factorization_memory():
        yield
    pin.refresh()


def _tree_matches_the_pin() -> bool:
    return not any(pin.stale_files(operation) for operation in pin.OPERATION_FILES)


needs_matching_tree = pytest.mark.skipif(
    not _tree_matches_the_pin(),
    reason="дерево ядра ушло от пина (`native_status()` называет файлы): эти проверки сверяют пин с деревом, из которого он снят",
)


def _copy_tree(target: Path) -> Path:
    from cftuv_native import density_pin
    shared = {row[0] for rows in (density_pin.FUNCTIONS, density_pin.SOURCE_CONSTANTS, density_pin.SOURCE_ENUMS) for row in rows}
    for name in sorted(set(ALL_FILES) | shared):
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(KERNEL / name, destination)
    return target


# --------------------------------------------------------------------------
# Что такое пин
# --------------------------------------------------------------------------


def test_the_pin_holds_a_digest_for_exactly_the_files_the_operations_mirror():
    assert sorted(pin.PINS) == ALL_FILES
    assert all(len(digest) == 64 and set(digest) <= set("0123456789abcdef") for digest in pin.PINS.values())
    assert set(pin.COVERAGE_FILES) < set(ALL_FILES) and set(pin.CLIP_FILES) < set(ALL_FILES)
    assert set(pin.FOUNDATION) <= set(pin.COVERAGE_FILES) & set(pin.CLIP_FILES), "the exact layer and the filters are mirrored by both ports"
    assert CLIP_ONLY not in pin.COVERAGE_FILES and COVERAGE_ONLY not in pin.CLIP_FILES


@needs_matching_tree
def test_the_pin_matches_the_tree_it_was_made_from():
    assert pin.native_status() == {"coverage": "available", "clip": "available", "skeleton": "available", "snap_embedding": "available", "effective_alpha": "available", "materialize_w1": "available", "snapshot_check": "available", "frame": "available", "radical_v2": "available", "density": "available", "angular": "available", "mesh_assembly": "available"}
    assert cftuv_native.native_status() == {"coverage": "available", "clip": "available", "skeleton": "available", "snap_embedding": "available", "effective_alpha": "available", "materialize_w1": "available", "snapshot_check": "available", "frame": "available", "radical_v2": "available", "density": "available", "angular": "available", "mesh_assembly": "available", "chain_station": "available", "materialize_hot": "available"}
    assert pin.fingerprint("clip") != pin.fingerprint("coverage")
    assert pin.fingerprint("clip") == pin.fingerprint("clip", KERNEL)


@needs_matching_tree
def test_a_copy_of_the_pinned_files_matches_and_line_endings_do_not_matter(tmp_path):
    root = _copy_tree(tmp_path / "tree")
    assert pin.stale_files("clip", root) == () and pin.stale_files("coverage", root) == ()
    target = root / CLIP_ONLY
    target.write_bytes(target.read_bytes().replace(b"\r\n", b"\n").replace(b"\n", b"\r\n"))
    assert pin.stale_files("clip", root) == (), "a Windows checkout of the same source is the same oracle"


# --------------------------------------------------------------------------
# Что пин узнаёт
# --------------------------------------------------------------------------


@needs_matching_tree
@pytest.mark.parametrize("name", ALL_FILES)
def test_the_pin_detects_a_changed_file_in_every_pinned_file(tmp_path, name):
    root = _copy_tree(tmp_path / "tree")
    target = root / name
    target.write_bytes(target.read_bytes() + b"\n# a new law\n")
    for operation, files in pin.OPERATION_FILES.items():
        assert pin.stale_files(operation, root) == ((name,) if name in files else ())


@needs_matching_tree
def test_a_changed_byte_a_removed_file_and_a_renamed_file_are_all_named(tmp_path):
    root = _copy_tree(tmp_path / "tree")
    flipped = root / SHARED
    content = bytearray(flipped.read_bytes())
    content[len(content) // 2] ^= 0x01
    flipped.write_bytes(bytes(content))
    (root / CLIP_ONLY).unlink()
    (root / "float_filter.py").rename(root / "float_filter_renamed.py")
    def expected(files):
        return tuple(name if name == SHARED else f"{name} (missing)" for name in files if name in (CLIP_ONLY, "float_filter.py", SHARED))

    assert pin.stale_files("clip", root) == expected(pin.CLIP_FILES)
    assert pin.stale_files("coverage", root) == expected(pin.COVERAGE_FILES)


def test_a_file_missing_from_both_the_pin_and_the_tree_is_still_stale(tmp_path, monkeypatch):
    """Опечатка в списке файлов не должна обернуться «совпало»: нет файла — значит ушёл."""

    monkeypatch.setitem(pin.OPERATION_FILES, "ghost", ("never/was.py",))
    monkeypatch.setattr(pin, "PINS", {**pin.PINS})
    assert pin.stale_files("ghost", tmp_path) == ("never/was.py (missing)",)


# --------------------------------------------------------------------------
# Отказ операции: до всякого состояния
# --------------------------------------------------------------------------


def _a_call(path_index: int = 0):
    """A real answered call of the clip: from the field corpus when it is there, else from the synthetic one (any real call will do, the refusal is what is under test)."""

    from native_clip_geometry import field_paths, synthetic_paths

    paths = field_paths() or synthetic_paths()
    for path in paths[path_index:]:
        record = nc.read_record(path)
        if not record.expected().exception:
            return record, nc.prepare_call(nc.OP_CLIP, record.call_blob, record.before())
    pytest.skip("нет ни полевого, ни синтетического корпуса: нечем звать операцию")


def _tamper(monkeypatch, name: str):
    """Читатель файлов эталона видит другой `name`: так выглядит ядро, ушедшее от пина."""

    original = pin.read_source

    def reading(root, file):
        found = original(root, file)
        return found if file != name or found is None else found + b"\n# the oracle moved\n"

    monkeypatch.setattr(pin, "read_source", reading)
    pin.refresh()


PRODUCT = tuple(pin.RUST_PRODUCT_OPERATIONS)
#: CONTRACT_PARITY_V1: an operation where Python is still the product keeps the named refusal; `radical_v2` is the simplest to call.
PYTHON_PRODUCT_ONLY = "reference/radical_v2.py"


def _moved() -> dict:
    """`{operation: names}` of what the pin sees moved: the stale files of an operation where Python is in the product, the reference drift of a
    Rust-product operation (CONTRACT_PARITY_V1: the same detection, another consequence — the Rust-product port stays `available`)."""

    status = cftuv_native.native_status()
    drift = pin.reference_drift()
    found = {}
    for operation in pin.OPERATION_FILES:
        if operation in PRODUCT:
            assert status[operation] == "available", (operation, status[operation])
            if drift.get(operation):
                found[operation] = drift[operation]
        elif status[operation].startswith("stale("):
            found[operation] = tuple(status[operation][len("stale("):-1].split(", "))
    if drift:
        assert status[pin.REFERENCE_DRIFT_KEY] == "; ".join(f"{operation}: {', '.join(names)}" for operation, names in drift.items())
    else:
        assert pin.REFERENCE_DRIFT_KEY not in status
    return found


@needs_matching_tree
def test_a_moved_clip_reference_is_reported_and_the_rust_product_still_answers(monkeypatch):
    _record, call = _a_call()
    _tamper(monkeypatch, CLIP_ONLY)
    assert _moved() == {"clip": (CLIP_ONLY,)}
    assert cftuv_native.native_status()[pin.REFERENCE_DRIFT_KEY] == f"clip: {CLIP_ONLY}"
    assert cftuv_native.new_mirror().clip_geometry(call.args[0], call.budget, **call.kwargs).note, "the product does not refuse because the frozen reference moved"
    monkeypatch.undo()
    pin.refresh()
    assert _moved() == {}


@needs_matching_tree
def test_a_moved_reference_of_a_python_product_operation_still_refuses_by_name_before_it_touches_any_state(monkeypatch):
    _tamper(monkeypatch, PYTHON_PRODUCT_ONLY)
    assert _moved() == {"radical_v2": (PYTHON_PRODUCT_ONLY,), "density": (PYTHON_PRODUCT_ONLY,)}, "the Density mirror reads the same algebra: drift there, a refusal for the capability"
    with exact.isolated_factorization_memory():
        marker = (list(exact._KNOWN_PRIMES), dict(exact._FACTORIZATION_MEMO), dict(exact.SIGN_COUNTS))
        with pytest.raises(cftuv_native.NativePortStale, match="reference/radical_v2.py") as caught:
            cftuv_native.radical_v2_eval(None)
        assert isinstance(caught.value, RuntimeError) and "radical_v2" in str(caught.value)
        assert marker == (list(exact._KNOWN_PRIMES), dict(exact._FACTORIZATION_MEMO), dict(exact.SIGN_COUNTS))


@needs_matching_tree
def test_an_edit_to_one_operations_file_does_not_move_the_other(monkeypatch):
    _tamper(monkeypatch, COVERAGE_ONLY)
    assert _moved() == {"coverage": (COVERAGE_ONLY,)}
    _record, call = _a_call()
    assert cftuv_native.new_mirror().clip_geometry(call.args[0], call.budget, **call.kwargs).note
    _tamper(monkeypatch, CLIP_ONLY)
    assert _moved() == {"coverage": (COVERAGE_ONLY,), "clip": (CLIP_ONLY,)}


@needs_matching_tree
def test_a_moved_skeleton_reference_is_reported_and_the_skeleton_still_answers(monkeypatch):
    from cftuv_envelope.wavefront.polygon import PolygonV1

    polygon = PolygonV1.build([(0, 0), (4, 0), (4, 4), (0, 4)])
    _tamper(monkeypatch, SKELETON_ONLY)
    assert _moved() == {"skeleton": (SKELETON_ONLY,)}
    budget = exact.exact_work_budget(stage="PREPARE", domain_id="moved-skeleton", superlevel="", cap=1 << 20)
    assert cftuv_native.new_mirror().build_skeleton(polygon, work_budget=budget).outcome.value == "EXACT"


@needs_matching_tree
def test_an_embedding_oracle_edit_moves_embedding_and_stales_the_frame_that_python_still_owns(monkeypatch):
    _tamper(monkeypatch, "_embedding.py")
    assert _moved() == {"snap_embedding": ("_embedding.py",), "frame": ("_embedding.py",)}


@needs_matching_tree
def test_an_edit_to_the_pair_leaf_moves_only_effective_alpha_and_a_float_filter_edit_moves_it_with_the_stages_that_share_the_filters(monkeypatch):
    _tamper(monkeypatch, "reference/boundary_native.py")
    assert _moved() == {"effective_alpha": ("reference/boundary_native.py",)}
    monkeypatch.undo()
    _tamper(monkeypatch, "float_filter.py")
    moved = _moved()
    assert "snap_embedding" not in moved and "snapshot_check" not in moved, "the certificate and the snapshot validators do not use the float filters"
    assert all(moved[name] == ("float_filter.py",) for name in ("coverage", "clip", "skeleton", "effective_alpha", "materialize_w1")), moved


@needs_matching_tree
def test_an_edit_to_a_snapshot_validator_moves_only_snapshot_check_and_numeric_py_moves_it_with_the_clip(monkeypatch):
    _tamper(monkeypatch, "validation_snapshot.py")
    assert _moved() == {"snapshot_check": ("validation_snapshot.py",)}
    monkeypatch.undo()
    _tamper(monkeypatch, "numeric.py")
    moved = _moved()
    assert moved["snapshot_check"] == ("numeric.py",) and moved["clip"] == ("numeric.py",), moved


@needs_matching_tree
def test_an_edit_to_a_coverage_or_clip_file_does_not_move_the_skeleton_and_a_shared_file_moves_all_three(monkeypatch):
    _tamper(monkeypatch, COVERAGE_ONLY)
    assert "skeleton" not in _moved()
    _tamper(monkeypatch, CLIP_ONLY)
    assert "skeleton" not in _moved()
    monkeypatch.undo()
    _tamper(monkeypatch, SHARED)
    moved = _moved()
    assert moved["coverage"] == moved["clip"] == moved["skeleton"] == (SHARED,)


@needs_matching_tree
def test_an_interpreter_below_the_floor_is_a_named_refusal_not_a_fallback(monkeypatch):
    _record, call = _a_call()
    monkeypatch.setattr(pin, "MINIMUM_PYTHON", (99, 0))
    pin.refresh()
    assert cftuv_native.native_status() == {"coverage": "unsupported_python", "clip": "unsupported_python", "skeleton": "unsupported_python", "snap_embedding": "unsupported_python", "effective_alpha": "unsupported_python", "materialize_w1": "unsupported_python", "snapshot_check": "unsupported_python", "frame": "unsupported_python", "radical_v2": "unsupported_python", "density": "unsupported_python", "angular": "unsupported_python", "mesh_assembly": "unsupported_python", "chain_station": "available", "materialize_hot": "unsupported_python"}
    state = nc.capture_state(call.budget, None)
    with pytest.raises(cftuv_native.NativeUnsupportedPython, match="needs CPython 99.0 or newer"):
        cftuv_native.new_mirror().clip_geometry(call.args[0], call.budget, **call.kwargs)
    assert nc.compare_outcomes(nc.OP_CLIP, state, nc.Outcome(None, None, state, {}), nc.Outcome(None, None, nc.capture_state(call.budget, None), {})) == []


def test_the_verdict_is_computed_once_per_process_and_forgotten_on_refresh(monkeypatch):
    calls = []
    original = pin.stale_files

    def counting(operation, root=None):
        calls.append(operation)
        return original(operation, root)

    monkeypatch.setattr(pin, "stale_files", counting)
    pin.refresh()
    for _ in range(5):
        pin.native_status()
    assert sorted(calls) == ["angular", "clip", "coverage", "density", "effective_alpha", "frame", "materialize_w1", "mesh_assembly", "radical_v2", "skeleton", "snap_embedding", "snapshot_check"]
    pin.refresh()
    pin.native_status()
    assert len(calls) == 24


# --------------------------------------------------------------------------
# Замер и сверка
# --------------------------------------------------------------------------


@needs_matching_tree
def test_the_bench_measures_a_rust_product_with_a_moved_reference_and_refuses_a_stale_python_product(monkeypatch, capsys):
    import native_bench_native as bench

    _tamper(monkeypatch, CLIP_ONLY)
    assert bench.load_extension("clip") is cftuv_native, "the Rust product is what is measured; its reference moved, the product did not"
    _tamper(monkeypatch, PYTHON_PRODUCT_ONLY)
    with pytest.raises(SystemExit) as caught:
        bench.load_extension("radical_v2")
    assert caught.value.code == 2
    assert "NATIVE_BENCH_NATIVE_FAILED" in capsys.readouterr().out


def test_the_gate_of_the_diff_tests_runs_a_rust_product_against_a_moved_reference_and_skips_a_stale_python_product(monkeypatch):
    """CONTRACT_PARITY_V1: the differential of a Rust-product stage runs against the reference in front of it (its gate is the contract comparison, not the bytes of the pinned version)."""

    import native_gate

    _tamper(monkeypatch, CLIP_ONLY)
    native_gate.skip_unless_available(cftuv_native, "clip")
    _tamper(monkeypatch, PYTHON_PRODUCT_ONLY)
    with pytest.raises(pytest.skip.Exception) as caught:
        native_gate.skip_unless_available(cftuv_native, "radical_v2")
    assert f"stale({PYTHON_PRODUCT_ONLY})" in str(caught.value)
    native_gate.skip_unless_available(cftuv_native, "coverage")
