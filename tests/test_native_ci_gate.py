"""Ворота CI нативного ускорителя (`.github/workflows/native.yml`): приёмка Rust не может пройти без самого Rust.

Две части.

1. САМ ПЛАГИН СТРОГОГО РЕЖИМА (`tests/native_gate.py`), без расширения: в подпроцессе `pytest` на временных тестах строгий режим превращает пропуск теста, пропуск модуля при сборе и пропуск параметра в провал; оставляет
   пропуск `skip_for_interpreter`; снимает уровень `native_field` по имени и называет его в отчёте; не даёт скрыть что-либо ещё через `-k`; не принимает опечатку в значении `CFTUV_NATIVE_STRICT`.
2. ТО, ЧТО УСТАНОВЛЕНО (нужно расширение; в строгом режиме его отсутствие — провал, не пропуск): обе нативные операции `available` против ЭТОГО дерева ядра; личность сборки равна личности дерева (`tools/native_build_id.py`);
   расширение — установленное колесо, не дерево исходников; интерпретатор и режим слотов — те, что ножка CI называет (`CFTUV_NATIVE_EXPECT_PYTHON`, `CFTUV_NATIVE_SLOTS`); синтетический корпус резки есть и записан под ЭТО ядро;
   настоящий малый домен ядра, посчитанный нативным бэкендом через диспетчер продукта (`backend.use_backend("NATIVE")`), даёт ответ и цену эталона, а журнал домена называет нативные вызовы без единого отката.
"""

from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
TESTS = ROOT / "tests"
for _path in (ROOT / "kernel" / "src", ROOT / "kernel" / "tests", ROOT / "tools", TESTS):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

import native_gate  # noqa: E402

# --------------------------------------------------------------------------
# 1. плагин строгого режима
# --------------------------------------------------------------------------

CASES = '''
import pytest
import native_gate


def test_passes():
    pass


def test_skips_because_something_is_missing():
    pytest.skip("the extension was not built")


@pytest.mark.parametrize("value", [1, pytest.param(2, marks=pytest.mark.skip(reason="a parameter that cannot run"))])
def test_a_skipped_parameter(value):
    pass


@pytest.mark.native_field
def test_needs_the_field_corpus():
    raise AssertionError("must be deselected, never run")


def test_belongs_to_another_interpreter():
    native_gate.skip_for_interpreter("only CPython 3.11 asks this question")
'''

QUIET_CASES = '''
import pytest
import native_gate


def test_passes():
    pass


def test_hidden_by_a_name_filter():
    pass


@native_gate.field_tier(False, "no field corpus")
def test_needs_the_field_corpus():
    raise AssertionError("must be deselected, never run")


def test_belongs_to_another_interpreter():
    native_gate.skip_for_interpreter("only CPython 3.11 asks this question")
'''

MODULE_SKIP = '''
import pytest

pytest.skip("cftuv_native is not built", allow_module_level=True)
'''


def _pytest(directory: Path, *arguments: str, strict: str | None, summary: Path | None = None) -> subprocess.CompletedProcess:
    """`pytest` in a child process on the temporary tests of `directory`, the gate loaded as a plugin (`-p native_gate`) and `--strict-markers` on as in `pytest.ini`; the developer's own variables are not inherited."""

    (directory / "pytest.ini").write_text("[pytest]\naddopts = --strict-markers\n", encoding="utf-8")
    environment = {key: value for key, value in os.environ.items() if key not in (native_gate.STRICT_ENVIRONMENT, native_gate.SUMMARY_ENVIRONMENT)}
    environment["PYTHONPATH"] = os.pathsep.join([str(TESTS), environment.get("PYTHONPATH", "")]).rstrip(os.pathsep)
    environment["PYTHONSAFEPATH"] = "1"
    if strict is not None:
        environment[native_gate.STRICT_ENVIRONMENT] = strict
    if summary is not None:
        environment[native_gate.SUMMARY_ENVIRONMENT] = str(summary)
    command = [sys.executable, "-m", "pytest", "-p", "native_gate", "-p", "no:cacheprovider", "--noconftest", "-q", "-rsfE", "--rootdir", str(directory), "-c", str(directory / "pytest.ini"), *arguments]
    return subprocess.run(command, cwd=str(directory), env=environment, capture_output=True, text=True, timeout=300, check=False)


def _summary(done: subprocess.CompletedProcess) -> str:
    return done.stdout + "\n" + done.stderr


def test_without_the_strict_switch_a_skip_stays_a_skip(tmp_path):
    (tmp_path / "test_cases.py").write_text(CASES, encoding="utf-8")
    done = _pytest(tmp_path, "test_cases.py", "-m", "not native_field", strict=None)
    assert done.returncode == 0, _summary(done)
    assert "2 passed" in done.stdout and "3 skipped" in done.stdout and "1 deselected" in done.stdout, done.stdout[-800:]
    assert "DESELECTED: 1 tests" in done.stdout, "the field tier is named in the report even when strict mode is off"


def test_strict_mode_turns_every_kind_of_skip_into_a_failure_except_the_interpreter_one(tmp_path):
    (tmp_path / "test_cases.py").write_text(CASES, encoding="utf-8")
    done = _pytest(tmp_path, "test_cases.py", "-m", "not native_field", strict="1")
    text = _summary(done)
    assert done.returncode == 1, text
    # a skipped test fails in its call phase, a skipped PARAMETER in its setup phase (pytest reports that one as an error): both are failures of the run
    assert "1 failed" in text and "1 error" in text and "2 passed" in text and "1 skipped" in text and "1 deselected" in text, text[-1200:]
    assert "test_skips_because_something_is_missing" in text and "the extension was not built" in text, "a skipped test is a failure that names its reason"
    assert "a parameter that cannot run" in text, "a skipped PARAMETER is a failure too"
    assert "in strict mode a skip is a failure" in text
    assert "SKIPPED" in text and "[interpreter] only CPython 3.11 asks this question" in text, "the interpreter skip stays visible as a skip"
    assert "field tier" in text and "DESELECTED: 1 tests" in text and "deselected: test_cases.py::test_needs_the_field_corpus" in text, "the deselected field tier is named"


def test_strict_mode_makes_a_module_that_skips_itself_at_collection_a_failure(tmp_path):
    (tmp_path / "test_module.py").write_text(MODULE_SKIP, encoding="utf-8")
    done = _pytest(tmp_path, "test_module.py", strict="1")
    text = _summary(done)
    assert done.returncode != 0, text
    assert "cftuv_native is not built" in text and "in strict mode a skip is a failure" in text, text[-1200:]
    assert _pytest(tmp_path, "test_module.py", strict=None).returncode == 5, "off: the module is skipped, nothing collected (pytest's `no tests ran`)"


def test_strict_mode_accepts_a_run_whose_only_gaps_are_the_field_tier_and_the_interpreter_skip(tmp_path):
    (tmp_path / "test_quiet.py").write_text(QUIET_CASES, encoding="utf-8")
    done = _pytest(tmp_path, "test_quiet.py", "-m", "not native_field", strict="1")
    text = _summary(done)
    assert done.returncode == 0, text
    assert "2 passed" in text and "1 skipped" in text and "1 deselected" in text, text[-800:]


def test_strict_mode_does_not_let_a_name_filter_hide_a_test(tmp_path):
    (tmp_path / "test_quiet.py").write_text(QUIET_CASES, encoding="utf-8")
    done = _pytest(tmp_path, "test_quiet.py", "-k", "not test_hidden_by_a_name_filter", "-m", "not native_field", strict="1")
    text = _summary(done)
    assert done.returncode != 0, text
    assert "NOT the field tier but deselected: 1 tests" in text and "hidden: test_quiet.py::test_hidden_by_a_name_filter" in text, text[-1200:]
    off = _pytest(tmp_path, "test_quiet.py", "-k", "not test_hidden_by_a_name_filter", "-m", "not native_field", strict=None)
    assert off.returncode == 0, "off: a developer may filter by name"


def test_the_step_summary_of_a_github_run_lists_the_deselected_field_tier(tmp_path):
    (tmp_path / "test_quiet.py").write_text(QUIET_CASES, encoding="utf-8")
    summary = tmp_path / "step-summary.md"
    done = _pytest(tmp_path, "test_quiet.py", "-m", "not native_field", strict="1", summary=summary)
    assert done.returncode == 0, _summary(done)
    text = summary.read_text(encoding="utf-8")
    assert "1 tests DESELECTED by name" in text and "test_quiet.py::test_needs_the_field_corpus" in text, text


def test_a_typo_in_the_strict_switch_is_a_usage_error_not_a_quiet_off(tmp_path):
    (tmp_path / "test_cases.py").write_text(CASES, encoding="utf-8")
    done = _pytest(tmp_path, "test_cases.py", strict="yes")
    assert done.returncode == 4, _summary(done)
    assert native_gate.STRICT_ENVIRONMENT in _summary(done)


# --------------------------------------------------------------------------
# 2. то, что установлено
# --------------------------------------------------------------------------


def _extension():
    try:
        import cftuv_native
    except ModuleNotFoundError as error:
        if error.name != "cftuv_native":
            raise
        pytest.skip("расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (что установлено — не проверено)")
    return cftuv_native


def test_both_native_operations_are_available_against_this_tree_of_the_kernel():
    status = _extension().native_status()
    assert status == {"coverage": "available", "clip": "available"}, f"a native operation is not compared with THIS kernel (the pin moved, or the interpreter is below the floor): {status}"


def test_the_installed_build_is_the_build_of_this_tree():
    extension = _extension()
    import native_build_id as tool

    tree, installed = tool.tree_parts(), extension.native_build_parts()
    if installed["rust"] != tree["rust"] or installed["shim"] != tree["shim"]:
        pytest.skip(
            f"the installed build is not the build of this tree: installed rust {installed['rust'][:16]} shim {installed['shim'][:16]}, tree rust {tree['rust'][:16]} shim {tree['shim'][:16]} "
            "(`python tools/native_build.py`; `python tools/native_build_id.py` shows the halves)"
        )
    assert installed["id"] == tree["id"] == extension.native_build_id()


def test_the_extension_is_an_installed_wheel_not_the_source_tree():
    extension = _extension()
    directory = Path(extension.__file__).resolve().parent
    assert ROOT / "native" not in directory.parents, f"cftuv_native is imported from the source tree {directory}: the wheel was not installed"
    binaries = sorted(path.name for path in directory.glob("_core*") if path.suffix in (".pyd", ".so", ".dylib"))
    assert binaries, f"no compiled `_core` next to the shim in {directory}"


def test_the_interpreter_is_the_one_this_leg_of_the_matrix_claims():
    _extension()
    expected = os.environ.get("CFTUV_NATIVE_EXPECT_PYTHON")
    if not expected:
        pytest.skip("CFTUV_NATIVE_EXPECT_PYTHON is not set: only a CI leg says which interpreter it is supposed to test")
    assert f"{sys.version_info.major}.{sys.version_info.minor}" == expected, f"the leg claims Python {expected}, this is {sys.version.split()[0]}"


def test_the_slot_mode_is_forced_by_every_leg_of_a_strict_run():
    extension = _extension()
    if native_gate.strict():
        assert os.environ.get(extension.SLOT_MODE_ENVIRONMENT, "").strip().lower() in extension.SLOT_MODES, "a strict run names its slot mode (raw, attr or auto); an unset one would be a leg that proves nothing about it"
    assert extension.slot_mode() == ((os.environ.get(extension.SLOT_MODE_ENVIRONMENT) or "auto").strip().lower() or "auto")


def test_the_synthetic_clip_corpus_is_there_and_was_recorded_under_this_kernel():
    _extension()
    import native_clip_geometry as geometry

    import cftuv_envelope.materialize.clip_memo as clip_memo

    if native_gate.strict():
        assert os.environ.get(geometry.SYNTHETIC_ENVIRONMENT), f"a strict run builds the synthetic corpus and says where ({geometry.SYNTHETIC_ENVIRONMENT})"
    base = geometry.synthetic_base()
    index = base / "index.json"
    if not index.is_file():
        pytest.skip(f"нет синтетического корпуса {base}: `python tools/native_clip_synthetic.py build`")
    import json

    document = json.loads(index.read_text(encoding="utf-8"))
    assert document.get("kernel_identity") == clip_memo.kernel_code_identity(), "the synthetic corpus was recorded under another kernel: it describes another oracle"
    assert len(geometry.synthetic_paths()) >= 400 and document["records_count"] >= 400, "the synthetic corpus is too small to be a corpus"
    assert sum(document["seam_calls"].values()) > 10_000, "the seam calls recorded from the kernel tests are missing"


def test_a_real_domain_computed_by_the_native_backend_through_the_products_dispatcher_equals_the_oracles_and_names_its_native_calls():
    extension = _extension()
    import developable_factories as df
    from developable_route import materialize_developable

    from cftuv_envelope import backend
    from cftuv_envelope.codec import canonical_json_bytes
    from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
    from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
    from cftuv_envelope.materialize.clip_memo import memo_disabled

    def answer(result):
        return (result.outcome.value, result.detail, tuple(result.counters), tuple(result.diagnostics), result.content_digest, result.offset_normals_digest, tuple(result.vertex_normals), canonical_json_bytes(result.batch))

    def build(law):
        result, _prepared = materialize_developable(df.fold_strip(), ("r0a", "r0b"), alpha="3.5", decal_topology_law=DecalTopologyLawV1.PLANAR_POLYGONS_V1, near_planar_lift_law=law)
        assert result.is_materialized, result.detail
        return result

    backend.refresh_native()
    status = backend.native_status()
    assert (status.coverage, status.clip) == ("available", "available") and status.build_id == extension.native_build_id(), status
    extension.reset_slot_counters()
    for law in (NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1, NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1):
        with memo_disabled():
            reference = answer(build(law))
        with memo_disabled(), backend.use_backend("NATIVE") as ledger:
            native = answer(build(law))
        record = ledger.record()
        assert native == reference, f"{law}: the native backend answered differently from the oracle"
        assert record.requested == "NATIVE" and record.ran == "native", record
        assert record.python_calls == 0 and record.fallbacks == () and record.native_calls > 0, f"{law}: the domain fell back to the oracle or never reached the extension: {record}"
    counters = extension.slot_counters()
    assert counters["raw_reads"] + counters["attr_reads"] > 0 and counters["raw_builds"] + counters["attr_builds"] > 0, f"the product's domain crossed no slot: {counters}"
    if extension.slot_mode() == "attr":
        assert counters["raw_reads"] == counters["raw_builds"] == 0, counters
    elif extension.slot_mode() == "raw":
        assert counters["attr_reads"] == counters["attr_builds"] == 0, counters
    backend.refresh_native()


def test_the_field_tier_marker_exists_and_the_gate_shows_it_to_pytest(pytestconfig):
    assert any(line.startswith(native_gate.FIELD_MARKER) for line in pytestconfig.getini("markers")), "the plugin registers the marker (`--strict-markers` would refuse its use otherwise)"
