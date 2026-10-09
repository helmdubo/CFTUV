"""Нативный `snap_embedding_certificate` как ВСТАВКА (`cftuv_native.snap_embedding_certificate`) равен эталону на Python: тот же вызов, тот же `SourceSnapEmbeddingCertificateV1`, то же исключение.

Эталон — `_embedding._compute_source_snap_embedding_certificate(before, after, faces, intended_corners, unclassifiable_corners, snapping_law)`: чистая функция точных `Fraction`-позиций до и после привязки,
без бюджета, памяти и счётчиков, поэтому равенство — это двенадцать полей и одно исключение (`ValueError("physical edge '...' has inconsistent endpoints")`, с текстом эталона). Вставка стоит там, где
стоит эталон: обёртка с памятью `build_source_snap_embedding_certificate` остаётся на Python и зовёт то, что стоит в модуле на месте листа.

Источники: синтетические вызовы, которые делает `tools/native_embedding_synthetic.py` (генерируются здесь же, без корпуса на диске; `CFTUV_EMBEDDING_SYNTHETIC` — сколько, по умолчанию 600), записи вызовов набора тестов
ядра (небольшое зерно в репозитории: `tests/data/native_embedding_kernel_suite.recs.xz`) и ПОЛЕВОЙ корпус владельца (вызовы кнопки на мешах сцены, `E:/cftuv_native_corpus/embedding/field`; уровень `native_field`:
его нет в CI). Отказ ПОРТА (`NativePortUnsupported`) — второй вид исхода, а не расхождение: порт называет вход, которого не несёт (другой тип, вершина грани, которой нет в позициях — `KeyError` эталона), и
вызывающий запускает эталон; в синтетике допустим ровно такой отказ и ровно на таких входах, в корпусах записей его нет.
"""

from __future__ import annotations

import dataclasses
import inspect
import json
import os
import sys
import types
from collections import Counter
from fractions import Fraction
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    pytest.skip(
        "расширение cftuv_native не собрано: `python tools/native_build.py` ставит его в dev-venv (сверка вставки snap_embedding_certificate с эталоном пропущена)",
        allow_module_level=True,
    )

from native_gate import field_tier, skip_unless_available  # noqa: E402

skip_unless_available(cftuv_native, "snap_embedding")

import native_embedding_corpus as nec  # noqa: E402
import native_embedding_synthetic as synthetic  # noqa: E402

import cftuv_envelope._embedding as embedding  # noqa: E402
from cftuv_envelope.contracts.metric import GridSnappingLawV1, SourceSnapEmbeddingCertificateV1  # noqa: E402
from cftuv_envelope.ids import PhysicalEdgeId, SourceFaceId, SourceVertexId  # noqa: E402

SYNTHETIC_CASES = int(os.environ.get("CFTUV_EMBEDDING_SYNTHETIC", "600"))
KERNEL_SUITE_SEED = ROOT / "tests" / "data" / "native_embedding_kernel_suite.recs.xz"
FIELD_DIRECTORY = nec.corpus_directory() / "field"
FIELD_FILES = sorted(FIELD_DIRECTORY.glob("*.recs.xz")) if FIELD_DIRECTORY.is_dir() else []
needs_field = field_tier(bool(FIELD_FILES), f"нет полевого корпуса встроек ({FIELD_DIRECTORY}): `blender -b E:/testScene.blend --python tools/native_embedding_export.py`")
LAW = GridSnappingLawV1.SOURCE_ONLY_GRID_SNAP_V1

CHECKED: Counter = Counter()


@pytest.fixture(scope="module", autouse=True)
def _report_the_comparison_count(request):
    yield
    reporter = request.config.pluginmanager.get_plugin("terminalreporter")
    if reporter is not None:
        reporter.write_line(
            f"native snap_embedding DROP-IN compared with the Python oracle (python {sys.version.split()[0]}): " + ", ".join(f"{key} {value}" for key, value in sorted(CHECKED.items()))
        )


def native(arguments):
    return nec.outcome_of(cftuv_native.snap_embedding_certificate, arguments)


def oracle(arguments):
    return nec.outcome_of(nec.ORACLE, arguments)


def declined(outcome) -> bool:
    return outcome[0] == "raised" and outcome[1] == "NativePortUnsupported"


def point(x, y=0, z=0):
    return (Fraction(x), Fraction(y), Fraction(z))


def vertex(index: int) -> SourceVertexId:
    return SourceVertexId(f"v{index}")


def square_call(*, after=None, shift=0):
    """A unit square split by a diagonal: two triangles, five physical edges."""

    before = {vertex(0): point(0, 0), vertex(1): point(1, 0), vertex(2): point(1, 1), vertex(3): point(0, 1)}
    faces = synthetic.Mesh("v{}")
    faces.add_face("f0", [0, 1, 2], ["a", "b", "d"])
    faces.add_face("f1", [0, 2, 3], ["d", "c", "e"])
    return (before, before if after is None else after, faces.face_objects(), (), (), LAW)


# --------------------------------------------------------------------------
# Граница
# --------------------------------------------------------------------------


def test_the_dropin_has_the_signature_of_the_oracle():
    wanted = inspect.signature(nec.ORACLE)
    found = inspect.signature(cftuv_native.snap_embedding_certificate)
    assert [(item.name, item.kind) for item in found.parameters.values()] == [(item.name, item.kind) for item in wanted.parameters.values()]


def test_the_operation_is_pinned_to_the_one_file_it_mirrors():
    pin = cftuv_native.pin
    assert pin.OPERATION_FILES["snap_embedding"] == ("_embedding.py",)
    assert cftuv_native.native_status()["snap_embedding"] == "available"
    assert cftuv_native.NativePortUnsupported in cftuv_native.NATIVE_REFUSALS


def test_the_result_is_the_oracles_class_with_the_oracles_objects_and_exact_int_counts():
    arguments = next(args for label, args in synthetic.cases(60, 5) if label.startswith("grid/") and args[1] is not args[0])
    want, got = nec.ORACLE(*arguments), cftuv_native.snap_embedding_certificate(*arguments)
    assert type(got) is SourceSnapEmbeddingCertificateV1 and got == want
    assert got.snapping_law is arguments[5]
    assert [id(item) for item in got.source_vertex_ids] == [id(item) for item in want.source_vertex_ids] == [id(item) for item in sorted(arguments[0], key=lambda item: item.value)]
    assert all(type(getattr(got, name)) is int for name in nec.CERTIFICATE_FIELDS if name not in ("snapping_law", "source_vertex_ids"))


def test_a_stale_pin_refuses_by_name_before_it_computes_anything():
    pin = cftuv_native.pin
    pin._VERDICTS["snap_embedding"] = ("stale", ("_embedding.py",))
    try:
        with pytest.raises(cftuv_native.NativePortStale, match="_embedding.py"):
            cftuv_native.snap_embedding_certificate(*square_call())
        assert cftuv_native.native_status()["snap_embedding"] == "stale(_embedding.py)"
    finally:
        pin.refresh()
    assert cftuv_native.native_status()["snap_embedding"] == "available"


def test_a_class_of_the_oracle_that_changed_shape_is_stale_by_name():
    from cftuv_native import embedding_op

    certificate = dataclasses.make_dataclass("SourceSnapEmbeddingCertificateV1", [(name, object) for name in nec.CERTIFICATE_FIELDS[:-1]], frozen=True)
    metric = types.SimpleNamespace(SourceSnapEmbeddingCertificateV1=certificate)
    surface = types.SimpleNamespace(SourceFaceV1=dataclasses.make_dataclass("SourceFaceV1", ["face_id", "vertex_cycle", "edge_cycle"]))
    ids = types.SimpleNamespace(SourceVertexId=SourceVertexId, PhysicalEdgeId=PhysicalEdgeId)
    with pytest.raises(cftuv_native.NativePortStale, match="contracts/metric.py"):
        embedding_op._bind(ids, metric, surface)


# --------------------------------------------------------------------------
# Исходы
# --------------------------------------------------------------------------


def test_the_exception_of_the_oracle_is_raised_with_its_text():
    mesh = synthetic.Mesh("é{}")
    mesh.positions = {mesh.vertex(i): point(i, i % 2) for i in range(4)}
    mesh.add_face("я0", [0, 1, 2], ["е\U0001d518", "x'y", "z"])
    mesh.add_face("я1", [0, 2, 3], ["z", "w", "е\U0001d518"])
    arguments = (mesh.positions, mesh.positions, mesh.face_objects(), (), (), LAW)
    want = oracle(arguments)
    assert want[0] == "raised" and want[1] == "ValueError" and "inconsistent endpoints" in want[2]
    assert native(arguments) == want


def test_the_edge_named_is_the_first_inconsistent_in_order_of_first_sight():
    arguments = square_call()
    faces = list(arguments[2])
    faces.append(nec.FaceLike(SourceFaceId("f2"), (vertex(0), vertex(1), vertex(3)), (PhysicalEdgeId("d"), PhysicalEdgeId("a"), PhysicalEdgeId("c"))))
    arguments = (arguments[0], arguments[1], tuple(faces), (), (), LAW)
    want = oracle(arguments)
    assert want[0] == "raised" and want[2] == "physical edge 'a' has inconsistent endpoints"
    assert native(arguments) == want


def test_a_snap_that_collapses_an_edge_and_crosses_two_others_is_counted_as_the_oracle_counts_it():
    before, _same, faces, _a, _b, law = square_call()
    after = {vertex(0): point(0, 0), vertex(1): point(0, 0), vertex(2): point(1, 1), vertex(3): point(1, 1)}
    arguments = (before, after, faces, ((vertex(1), vertex(0), vertex(3)),), ((vertex(0), vertex(1), vertex(2)),), law)
    want = oracle(arguments)
    assert want[0] == "ok" and want[1][nec.CERTIFICATE_FIELDS.index("newly_coincident_vertex_pair_count")] == 2
    assert native(arguments) == want


@pytest.mark.parametrize(
    "case",
    ["float coordinate", "bool coordinate", "subclass vertex id", "after lacks a vertex", "after has an extra vertex", "positions of two coordinates", "faces as a generator", "ordered dict", "short edge cycle", "corner of two"],
)
def test_an_input_the_port_does_not_carry_is_declined_by_name_and_the_oracle_still_answers(case):
    before, after, faces, intended, unclassifiable, law = square_call(after={vertex(0): point(0, 0), vertex(1): point(1, 0), vertex(2): point(1, 1), vertex(3): point(0, 1)})
    after = dict(after)
    if case == "float coordinate":
        before = {**before, vertex(0): (0.5, Fraction(0), Fraction(0))}
        after = dict(before)
    elif case == "bool coordinate":
        before = {**before, vertex(0): (True, Fraction(0), Fraction(0))}
        after = dict(before)
    elif case == "subclass vertex id":
        class Sub(SourceVertexId):
            pass

        before = {**before, Sub("v9"): point(5)}
        after = dict(before)
    elif case == "after lacks a vertex":
        del after[vertex(3)]
    elif case == "after has an extra vertex":
        after[vertex(9)] = point(7)
    elif case == "positions of two coordinates":
        before = {key: value[:2] for key, value in before.items()}
        after = dict(before)
    elif case == "faces as a generator":
        faces = (face for face in faces)
    elif case == "ordered dict":
        import collections

        before = collections.OrderedDict(before)
        after = before
    elif case == "short edge cycle":
        faces = faces + (nec.FaceLike(SourceFaceId("short"), (vertex(0), vertex(1), vertex(2)), (PhysicalEdgeId("a"), PhysicalEdgeId("b"))),)
    elif case == "corner of two":
        intended = ((vertex(0), vertex(1)),)
    arguments = (before, after, faces, intended, unclassifiable, law)
    with pytest.raises(cftuv_native.NativePortUnsupported):
        cftuv_native.snap_embedding_certificate(*arguments)
    if case != "faces as a generator":  # a generator is spent by the first reader, the oracle's own call is made on a fresh one
        assert oracle(arguments)[0] in ("ok", "raised")
    CHECKED["declined by name"] += 1


# --------------------------------------------------------------------------
# Источники
# --------------------------------------------------------------------------


def test_synthetic_cases_equal_the_oracle():
    mismatches, refused, shown = [], Counter(), Counter()
    for label, arguments in synthetic.cases(SYNTHETIC_CASES):
        want, got = oracle(arguments), native(arguments)
        CHECKED["synthetic"] += 1
        if declined(got):
            # the one refusal the generator provokes: a vertex of a face that the positions lack (the oracle raises KeyError)
            refused[(label.split("/")[0], want[0], want[1] if want[0] == "raised" else "")] += 1
            CHECKED["synthetic:declined"] += 1
        elif got != want:
            mismatches.append((label, want, got))
        elif want[0] == "ok":
            shown.update(name for name in nec.VIOLATION_FIELDS if want[1][nec.CERTIFICATE_FIELDS.index(name)])
        else:
            shown["raised"] += 1
    assert not mismatches, mismatches[:3]
    assert set(refused) <= {("missing", "raised", "KeyError")}, refused
    if SYNTHETIC_CASES >= 400:
        assert all(shown[name] >= 10 for name in (*nec.VIOLATION_FIELDS, "raised")), f"the generator must make every counter non-zero: {dict(shown)}"


def records_equal_the_oracle(records, source: str) -> None:
    mismatches = []
    for record in records:
        arguments = nec.decode_call(record)
        recorded, want, got = nec.answer_of(record), oracle(arguments), native(arguments)
        CHECKED[source] += 1
        if not (recorded == want == got):
            mismatches.append((record["id"], record["source"], recorded, want, got))
    assert not mismatches, mismatches[:3]


def test_the_kernel_suite_records_equal_the_oracle():
    records = nec.read_records(KERNEL_SUITE_SEED)
    assert len(records) > 50
    shape = nec.shape_of(records)
    assert shape["raised"] >= 1 and shape["after_is_before"] > 0 and shape["violating"] > 0
    records_equal_the_oracle(records, "kernel_suite")


def test_corpus_provenance_stays_with_its_source(tmp_path, monkeypatch):
    monkeypatch.setattr(nec.sys, "version", "3.11.11 field")
    nec.write_index(tmp_path, {"field": {"mesh": {"records": 1}}, "blender": "4.5.12", "scene": "scene.blend"})
    monkeypatch.setattr(nec.sys, "version", "3.13.1 synthetic")
    nec.write_index(tmp_path, {"synthetic": {"records": 2}})
    index = json.loads((tmp_path / "index.json").read_text(encoding="utf-8"))
    assert "python" not in index
    assert index["provenance"]["field"] == {"python": "3.11.11", "oracle_digest": nec.oracle_digest(), "blender": "4.5.12", "scene": "scene.blend"}
    assert index["provenance"]["synthetic"] == {"python": "3.13.1", "oracle_digest": nec.oracle_digest()}
    assert index["field"] == {"mesh": {"records": 1}}


@needs_field
def test_the_field_corpus_equals_the_oracle():
    records = nec.read_corpus(nec.corpus_directory(), "field")
    assert len(records) >= 150
    records_equal_the_oracle(records, "field")
    for source, count in sorted(Counter(record["source"] for record in records).items()):
        calls = sum(record["count"] for record in records if record["source"] == source)
        print(f"EMBEDDING_FIELD_REPLAY source={source} distinct_records={count} calls={calls} mismatches=0")


@needs_field
def test_the_synthetic_corpus_on_disk_equals_the_oracle():
    path = nec.corpus_directory() / "synthetic.recs.xz"
    if not path.exists():
        pytest.skip("нет записанного синтетического корпуса (`python tools/native_embedding_corpus.py synthetic`)")
    records = nec.read_records(path)
    records_equal_the_oracle([record for record in records if record["answer"][0] != "raised" or record["answer"][1] != "KeyError"], "synthetic corpus")


# --------------------------------------------------------------------------
# Обёртка с памятью остаётся на Python
# --------------------------------------------------------------------------


def _script() -> list:
    cases = [args for label, args in synthetic.cases(240, 3) if not label.startswith(("missing", "inconsistent"))]
    return [cases[0], cases[1], cases[0], cases[2], cases[3], cases[4], cases[0], cases[1], cases[5], cases[2]]


def run_memo_script(compute):
    """The memo wrapper over `compute`: certificates, statistics after every step, and the order of the values it holds."""

    steps = []
    saved = embedding._compute_source_snap_embedding_certificate
    embedding._compute_source_snap_embedding_certificate = compute
    try:
        with embedding.embedding_memo_limit(3):
            for before, after, faces, intended, unclassifiable, law in _script():
                found = embedding.build_source_snap_embedding_certificate(
                    before=before, after=after, faces=faces, intended_corners=intended, unclassifiable_corners=unclassifiable, snapping_law=law
                )
                steps.append((found, embedding.embedding_memo_stats(), [key[1:] for key in embedding._memo], list(embedding._memo.values())))
    finally:
        embedding._compute_source_snap_embedding_certificate = saved
    return steps


def test_the_memo_wrapper_keeps_its_contents_order_and_statistics_with_the_native_leaf_in_place():
    wanted = run_memo_script(nec.ORACLE)
    found = run_memo_script(cftuv_native.snap_embedding_certificate)
    assert [step[0] for step in found] == [step[0] for step in wanted]
    assert [step[1:] for step in found] == [step[1:] for step in wanted], "hits, misses, entries and the order of the keys are the memo's, not the leaf's"
    assert wanted[-1][1]["hits"] > 0 and wanted[-1][1]["misses"] > 3


def test_the_dispatcher_uses_the_real_native_leaf_and_a_memo_hit_keeps_identity(monkeypatch):
    from cftuv_envelope import backend

    arguments = square_call()
    original, calls = cftuv_native.snap_embedding_certificate, []

    def recorded(*actual):
        calls.append(actual)
        return original(*actual)

    monkeypatch.setattr(cftuv_native, "snap_embedding_certificate", recorded)
    embedding.clear_embedding_memo()
    try:
        with embedding.embedding_memo_limit(3), backend.use_backend("PYTHON", "PYTHON", "NATIVE") as ledger:
            kwargs = dict(zip(("before", "after", "faces", "intended_corners", "unclassifiable_corners", "snapping_law"), arguments))
            first = embedding.build_source_snap_embedding_certificate(**kwargs)
            second = embedding.build_source_snap_embedding_certificate(**kwargs)
            assert first is second and first == nec.ORACLE(*arguments)
        assert len(calls) == 1 and all(a is b for a, b in zip(calls[0], arguments))
        record = ledger.record()
        assert (record.embedding_native_calls, record.embedding_python_calls, record.embedding_cache_hits) == (1, 0, 1)
        assert not record.embedding_fallbacks
    finally:
        embedding.clear_embedding_memo()


@pytest.mark.parametrize("missing_position", [True, False])
def test_real_native_dispatch_preserves_missing_position_fallback_and_inconsistent_edge_error(monkeypatch, missing_position):
    from cftuv_envelope import backend

    before, after, faces, intended, unclassifiable, law = square_call()
    if missing_position:
        before = {key: value for key, value in before.items() if key != vertex(3)}
        after = before
    else:
        faces = faces + (nec.FaceLike(SourceFaceId("f2"), (vertex(0), vertex(1), vertex(3)), (PhysicalEdgeId("d"), PhysicalEdgeId("a"), PhysicalEdgeId("c"))),)
    arguments = before, after, faces, intended, unclassifiable, law
    wanted = oracle(arguments)
    assert wanted[0] == "raised" and wanted[1] == ("KeyError" if missing_position else "ValueError")
    native_leaf, python_leaf = cftuv_native.snap_embedding_certificate, embedding._compute_source_snap_embedding_certificate
    native_calls, python_calls = [], []

    def native_recorded(*actual):
        native_calls.append(actual)
        return native_leaf(*actual)

    def python_recorded(*actual):
        python_calls.append(actual)
        return python_leaf(*actual)

    monkeypatch.setattr(cftuv_native, "snap_embedding_certificate", native_recorded)
    monkeypatch.setattr(embedding, "_compute_source_snap_embedding_certificate", python_recorded)
    with backend.use_backend("PYTHON", "PYTHON", "NATIVE") as ledger:
        assert nec.outcome_of(backend.embedding_compute, arguments) == wanted
    assert len(native_calls) == 1 and all(a is b for a, b in zip(native_calls[0], arguments))
    assert len(python_calls) == int(missing_position)
    assert all(a is b for call in python_calls for a, b in zip(call, arguments))
    assert len(ledger.record().embedding_fallbacks) == int(missing_position)
