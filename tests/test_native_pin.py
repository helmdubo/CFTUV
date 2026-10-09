"""Пин нативных операций (`cftuv_native.pin`): порт отвечает только за ту версию эталона, с которой его сверяли.

Нативная операция побитово равна ОДНОЙ версии ядра на Python; пока основная ветка двигает ядро, порт, сверенный со старым, не должен молча
отвечать за новое. Пин — sha256 файлов эталона, которые порт зеркалит (`\\r\\n` -> `\\n`); шим сверяет дерево с пином ДО того, как тронет
состояние, и отказывается названным `NativePortStale` с перечнем разошедшихся файлов; `native_status()` говорит то же по операциям.

Здесь проверено: пин узнаёт изменённый, удалённый и перекодированный файл (на копии дерева и подменой читателя файла), отказ не трогает ни
бюджет, ни память, ни счётчики, ни нормали, чужая версия интерпретатора — своё названное имя, отказ точен (правка файла одной операции не красит другую),
замер отказывается, а не меряет устаревший порт. Модуль НЕ пропускается при устаревшем порте — это сам механизм.
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
    for name in ALL_FILES:
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
    assert pin.native_status() == {"coverage": "available", "clip": "available", "skeleton": "available", "snap_embedding": "available"}
    assert cftuv_native.native_status() == {"coverage": "available", "clip": "available", "skeleton": "available", "snap_embedding": "available"}
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


@needs_matching_tree
def test_a_stale_clip_port_refuses_by_name_before_it_touches_any_state(monkeypatch):
    _record, call = _a_call()
    _tamper(monkeypatch, CLIP_ONLY)
    assert cftuv_native.native_status() == {"coverage": "available", "clip": f"stale({CLIP_ONLY})", "skeleton": "available", "snap_embedding": "available"}
    before = nc.capture_state(call.budget, None), dict(call.args[0]._normal_by_position)
    mirror = cftuv_native.new_mirror()
    with pytest.raises(cftuv_native.NativePortStale) as caught:
        mirror.clip_geometry(call.args[0], call.budget, **call.kwargs)
    assert isinstance(caught.value, RuntimeError) and CLIP_ONLY in str(caught.value) and "clip" in str(caught.value)
    with pytest.raises(cftuv_native.NativePortStale):
        cftuv_native.clip_geometry(call.args[0], call.budget, **call.kwargs)
    after = nc.capture_state(call.budget, None), dict(call.args[0]._normal_by_position)
    assert after[1] == before[1] and nc.compare_outcomes(nc.OP_CLIP, before[0], nc.Outcome(None, None, before[0], {}), nc.Outcome(None, None, after[0], {})) == []
    assert mirror._session.clip_cache() == 0, "a refused call converted nothing"
    monkeypatch.undo()
    pin.refresh()
    assert cftuv_native.native_status() == {"coverage": "available", "clip": "available", "skeleton": "available", "snap_embedding": "available"}
    assert mirror.clip_geometry(call.args[0], call.budget, **call.kwargs).note, "the same session answers once the tree matches the pin again"


@needs_matching_tree
def test_an_edit_to_one_operations_file_does_not_stale_the_other(monkeypatch):
    _tamper(monkeypatch, COVERAGE_ONLY)
    assert cftuv_native.native_status() == {"coverage": f"stale({COVERAGE_ONLY})", "clip": "available", "skeleton": "available", "snap_embedding": "available"}
    _record, call = _a_call()
    assert cftuv_native.new_mirror().clip_geometry(call.args[0], call.budget, **call.kwargs).note
    _tamper(monkeypatch, CLIP_ONLY)
    assert cftuv_native.native_status() == {"coverage": f"stale({COVERAGE_ONLY})", "clip": f"stale({CLIP_ONLY})", "skeleton": "available", "snap_embedding": "available"}


@needs_matching_tree
def test_a_stale_skeleton_port_refuses_by_name_before_it_touches_any_state_and_the_other_operations_stay_available(monkeypatch):
    from cftuv_envelope.wavefront.polygon import PolygonV1

    polygon = PolygonV1.build([(0, 0), (4, 0), (4, 4), (0, 4)])
    _tamper(monkeypatch, SKELETON_ONLY)
    assert cftuv_native.native_status() == {"coverage": "available", "clip": "available", "skeleton": f"stale({SKELETON_ONLY})", "snap_embedding": "available"}
    budget = exact.exact_work_budget(stage="PREPARE", domain_id="stale-skeleton", superlevel="", cap=1 << 20)
    before = nc.capture_state(budget, None)
    with pytest.raises(cftuv_native.NativePortStale, match="wavefront/symbolic_runtime_commit.py") as caught:
        cftuv_native.new_mirror().build_skeleton(polygon, work_budget=budget)
    assert isinstance(caught.value, RuntimeError) and "skeleton" in str(caught.value)
    with pytest.raises(cftuv_native.NativePortStale):
        cftuv_native.build_skeleton(polygon, work_budget=budget)
    after = nc.capture_state(budget, None)
    assert nc.compare_outcomes(nc.OP_SKELETON, before, nc.Outcome(None, None, before, {}), nc.Outcome(None, None, after, {})) == []
    monkeypatch.undo()
    pin.refresh()
    assert cftuv_native.build_skeleton(polygon, work_budget=budget).outcome.value == "EXACT"


@needs_matching_tree
def test_an_embedding_oracle_edit_stales_only_embedding_before_argument_conversion(monkeypatch):
    _tamper(monkeypatch, "_embedding.py")
    assert cftuv_native.native_status() == {
        "coverage": "available", "clip": "available", "skeleton": "available", "snap_embedding": "stale(_embedding.py)",
    }
    # Негодные аргументы не должны маскировать именованный отказ по настоящему дайджесту файла.
    with pytest.raises(cftuv_native.NativePortStale, match="_embedding.py"):
        cftuv_native.snap_embedding_certificate(None, None, None, None, None, None)


@needs_matching_tree
def test_an_edit_to_a_coverage_or_clip_file_does_not_stale_the_skeleton_and_a_shared_file_stales_all_three(monkeypatch):
    _tamper(monkeypatch, COVERAGE_ONLY)
    assert cftuv_native.native_status()["skeleton"] == "available"
    _tamper(monkeypatch, CLIP_ONLY)
    assert cftuv_native.native_status()["skeleton"] == "available"
    monkeypatch.undo()
    _tamper(monkeypatch, SHARED)
    assert cftuv_native.native_status()["skeleton"] == f"stale({SHARED})"


@needs_matching_tree
def test_a_stale_shared_file_stales_both_operations_and_the_message_names_it(monkeypatch):
    _tamper(monkeypatch, SHARED)
    status = cftuv_native.native_status()
    assert status["coverage"] == f"stale({SHARED})" and status["clip"] == f"stale({SHARED})" and status["skeleton"] == f"stale({SHARED})"


@needs_matching_tree
def test_a_stale_coverage_port_refuses_before_it_touches_any_state(monkeypatch):
    from fractions import Fraction

    _tamper(monkeypatch, COVERAGE_ONLY)
    with exact.isolated_factorization_memory():
        marker = (list(exact._KNOWN_PRIMES), dict(exact._FACTORIZATION_MEMO), dict(exact.SIGN_COUNTS))
        with pytest.raises(cftuv_native.NativePortStale, match="wavefront/coverage.py"):
            cftuv_native.coverage_at(object(), Fraction(1), None, None)
        assert marker == (list(exact._KNOWN_PRIMES), dict(exact._FACTORIZATION_MEMO), dict(exact.SIGN_COUNTS))


@needs_matching_tree
def test_an_interpreter_below_the_floor_is_a_named_refusal_not_a_fallback(monkeypatch):
    _record, call = _a_call()
    monkeypatch.setattr(pin, "MINIMUM_PYTHON", (99, 0))
    pin.refresh()
    assert cftuv_native.native_status() == {"coverage": "unsupported_python", "clip": "unsupported_python", "skeleton": "unsupported_python", "snap_embedding": "unsupported_python"}
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
    assert sorted(calls) == ["clip", "coverage", "skeleton", "snap_embedding"]
    pin.refresh()
    pin.native_status()
    assert len(calls) == 8


# --------------------------------------------------------------------------
# Замер и сверка
# --------------------------------------------------------------------------


@needs_matching_tree
def test_the_bench_refuses_a_stale_port_instead_of_measuring_it(monkeypatch, capsys):
    import native_bench_native as bench

    _tamper(monkeypatch, CLIP_ONLY)
    with pytest.raises(SystemExit) as caught:
        bench.load_extension("clip")
    assert caught.value.code == 2
    assert "NATIVE_BENCH_NATIVE_FAILED" in capsys.readouterr().out
    assert bench.load_extension("coverage") is cftuv_native, "the coverage port is not stale"


def test_the_gate_of_the_diff_tests_skips_with_the_explicit_reason(monkeypatch):
    import native_gate

    _tamper(monkeypatch, CLIP_ONLY)
    with pytest.raises(pytest.skip.Exception) as caught:
        native_gate.skip_unless_available(cftuv_native, "clip")
    assert f"stale({CLIP_ONLY})" in str(caught.value)
    native_gate.skip_unless_available(cftuv_native, "coverage")
