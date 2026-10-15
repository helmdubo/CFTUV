"""R3 corpus: реальная граница, полные исходы и строгая exact wire без pickle."""

from __future__ import annotations

import copy
import json
import lzma
import os
import subprocess
import sys
from decimal import Decimal
from pathlib import Path

import pytest
import sympy as sp
from mpmath import iv

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "tools", ROOT / "kernel" / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv_envelope.contracts.analysis import TurnOrientation  # noqa: E402
from cftuv_envelope.reference import adaptive_density_atlas as atlas  # noqa: E402
from cftuv_envelope.reference import adaptive_density_fan as fan  # noqa: E402
from cftuv_envelope.reference import density_fan_binding as binding  # noqa: E402
from cftuv_envelope.reference import symbolic_backend as symbolic  # noqa: E402
from cftuv_envelope.reference.adaptive_density_band import WINDOW_LAW_VORONOI  # noqa: E402
import native_density_corpus as corpus  # noqa: E402
import native_density_probes as probes  # noqa: E402
import native_density_synthetic as synthetic  # noqa: E402
from native_density_wire import CorpusWireError, Wire  # noqa: E402


@pytest.fixture(scope="module")
def document():
    return synthetic.synthetic_document()


def _call(recorder, compute=None):
    metric = synthetic.metric()
    ideal = synthetic.principal_ideal(metric)
    return recorder.wrapper(compute)(metric, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
                                     4, (None,), lifted=True, law=WINDOW_LAW_VORONOI, spec_id="test", refusals=[])


def test_synthetic_replays_success_refusal_exception_and_warm_hits(document):
    shape = corpus.shape_of(document)
    assert shape["records"] == 13
    assert shape["raised"] == 5 and shape["refusals"] == 1
    assert shape["without_search_work"] == 4
    assert corpus.replay(document)["mismatch"] == 0
    assert corpus.replay(document)["equal"] == 13
    cases = {row["test"]: row for row in document["records"]}
    for name in ("q5_principal_h1", "principal_h2", "nested_root_h3"):
        assert cases[name]["outcome"]["answer"][0] == "ok"
        assert cases[name + ":cache_hit"]["outcome"]["budgets"] == []
        assert cases[name]["outcome"]["answer"] == cases[name + ":cache_hit"]["outcome"]["answer"]
    caps = [b["cap"] for b in cases["principal_h2"]["outcome"]["budgets"]]
    assert caps == [1 << 16, 1 << 17]
    failed = cases["search_cap0"]["outcome"]
    assert failed["answer"][1] == "DensityRationalAuthorityExhausted"
    assert sum(b["shell_probes"] + b["order_steps"] for b in failed["budgets"]) > 0


def test_writer_reader_are_json_and_replay_is_independent_of_process_state(tmp_path, document):
    path = tmp_path / "density.recs.xz"
    size = corpus.write_document(path, document)
    assert size == path.stat().st_size
    assert json.loads(lzma.decompress(path.read_bytes())) == document
    loaded = corpus.read_document(path)
    symbolic.BACKEND_COUNTS["corpus_test.sentinel"] = 413
    old = dict(symbolic.BACKEND_COUNTS)
    try:
        assert corpus.replay(loaded)["equal"] == 13
        assert symbolic.BACKEND_COUNTS == old
    finally:
        symbolic.BACKEND_COUNTS.pop("corpus_test.sentinel", None)


@pytest.mark.parametrize("field", corpus.OUTCOME_KEYS)
def test_replay_compares_every_outcome_field(document, field):
    altered = copy.deepcopy(document)
    altered["records"] = altered["records"][:1]
    value = altered["records"][0]["outcome"][field]
    if type(value) is list:
        value.append("different")
    else:
        value["different"] = 1
    result = corpus.replay(altered)
    assert result["mismatch"] == 1
    assert result["first_mismatches"][0]["fields"] == [field]


def test_wrapper_reraises_original_exception_and_records_work_before_it():
    recorder = corpus.Recorder("failure")
    failure = LookupError("original failure")

    def failing(*args, **kwargs):
        budget = atlas.DensityExactWorkBudget(fan.DensityRationalAuthorityExhausted, 100)
        budget.spend_order_steps(3)
        budget.spend_shell_probes(2)
        kwargs["refusals"].append(("test", "prior named refusal"))
        symbolic.count("corpus_exception", "visited")
        raise failure

    initialize, before = atlas.DensityExactWorkBudget.__init__, dict(symbolic.BACKEND_COUNTS)
    try:
        with pytest.raises(LookupError) as caught:
            _call(recorder, failing)
        assert caught.value is failure
        assert atlas.DensityExactWorkBudget.__init__ is initialize
        outcome = recorder.rows[0]["outcome"]
        assert outcome["answer"] == ["raised", "LookupError", "original failure", {}]
        assert outcome["refusals"] == [["test", "prior named refusal"]]
        assert outcome["budgets"] == [{"cap": 100, "order_steps": 3, "shell_probes": 2,
                                      "exhausted_type": "DensityRationalAuthorityExhausted"}]
        assert outcome["counts"] == {"corpus_exception.visited": 1}
        assert corpus.replay(recorder.document(), compute=failing)["equal"] == 1
    finally:
        symbolic.BACKEND_COUNTS.clear()
        symbolic.BACKEND_COUNTS.update(before)


def test_installed_boundary_and_compile_alias_restore_even_on_exception():
    from cftuv_envelope.reference import compile as compile_module

    old, alias = binding.bind_density_fan, compile_module.bind_density_fan
    recorder = corpus.Recorder("install")
    with pytest.raises(RuntimeError):
        with recorder.installed():
            assert binding.bind_density_fan is compile_module.bind_density_fan
            assert binding.bind_density_fan is not old
            raise RuntimeError("leave")
    assert binding.bind_density_fan is old and compile_module.bind_density_fan is alias


def test_disabled_injected_test_is_named_and_original_exception_still_propagates():
    recorder = corpus.Recorder("kernel_suite")
    recorder.enabled, recorder.test = False, "monkeypatch-test"

    def failure(*args, **kwargs):
        raise RuntimeError("injected")

    with pytest.raises(RuntimeError, match="injected"):
        _call(recorder, failure)
    assert recorder.rows == []
    assert recorder.document()["excluded_patched_calls"] == {
        "monkeypatch-test": {"calls": 1, "reason": "explicitly excluded patched oracle"},
    }


def test_same_exception_type_and_message_with_other_outcome_is_a_mismatch():
    from cftuv_envelope.reference.common import ReferenceGeometryError
    from cftuv_envelope.reference.contracts import ReferenceOutcome

    recorder = corpus.Recorder("named-exception")

    def undecidable(*args, **kwargs):
        raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, "identical message")

    def invalid(*args, **kwargs):
        raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID, "identical message")

    with pytest.raises(ReferenceGeometryError):
        _call(recorder, undecidable)
    result = corpus.replay(recorder.document(), compute=invalid)
    assert result["mismatch"] == 1
    failure = result["first_mismatches"][0]
    assert failure["expected"]["answer"][1:3] == failure["actual"]["answer"][1:3]
    assert failure["expected"]["answer"][3] != failure["actual"]["answer"][3]


def test_attributed_ideal_tuple_is_refused_instead_of_losing_its_attributes():
    owner_metric = synthetic.metric()

    class AttributedTuple(tuple):
        window_law = "a fact not carried by plain tuple"

    ideal = AttributedTuple(synthetic.principal_ideal(owner_metric))
    with pytest.raises(CorpusWireError, match="attributed tuple"):
        corpus.Recorder("not-plain").wrapper()(owner_metric, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION,
                                              4, (None,), lifted=True, law=WINDOW_LAW_VORONOI, spec_id="test", refusals=[])


def test_exact_expression_dag_preserves_args_and_unevaluated_power():
    angle = sp.atan2(sp.Integer(2), sp.Integer(1), evaluate=False)
    expressions = (
        sp.Pow(sp.Add(sp.Integer(2), sp.sqrt(2), evaluate=False), sp.Rational(1, 2), evaluate=False),
        sp.Add(sp.sin(angle, evaluate=False), sp.cos(sp.pi / 4, evaluate=False), evaluate=False),
        sp.Pow(sp.Rational(4), sp.Rational(1, 2), evaluate=False),
        sp.Add(sp.Integer(3), sp.Integer(1), evaluate=False),
    )
    wire = Wire()
    indices = [wire.expression(expression) for expression in expressions]
    decoded = Wire(json.loads(json.dumps(wire.nodes)))
    for expression, index in zip(expressions, indices):
        result = decoded.unexpression(index)
        assert sp.srepr(result) == sp.srepr(expression)
        assert result.args == expression.args
    with pytest.raises(CorpusWireError):
        wire.expression(sp.Float("0.1"))
    with pytest.raises(CorpusWireError):
        wire.expression(sp.Symbol("untrusted"))


def test_interval_memo_preserves_exact_binary_endpoints():
    saved = iv.prec
    try:
        iv.prec = 160
        interval = iv.sqrt(iv.mpf(2))
        wire = Wire()
        decoded = wire.unvalue(wire.value(interval))
        assert decoded._mpi_ == interval._mpi_
    finally:
        iv.prec = saved


def test_cached_pi_stays_a_dynamic_constant_when_decimal_envelope_raises_precision():
    from cftuv_envelope.reference.direction_binding import _density_decimal_envelope

    saved = iv.prec
    try:
        iv.prec = 53
        original = synthetic.metric()
        original._density_exact_memo.intervals[sp.pi] = iv.pi
        expression = sp.sin(sp.Mul(sp.Rational(1, 4), sp.pi, evaluate=False), evaluate=False)
        wire = Wire()
        body = wire.metric(original)
        at = wire.expression(expression)
        low_precision_pi = [list(endpoint) for endpoint in iv.pi._mpi_]
        expected = _density_decimal_envelope(expression, original)
        decoded_wire = Wire(copy.deepcopy(wire.nodes))
        decoded = decoded_wire.unmetric(body)
        assert decoded._density_exact_memo.intervals[sp.pi] is iv.pi
        assert wire.value(iv.pi) == ["interval_constant", "pi"]
        assert _density_decimal_envelope(decoded_wire.unexpression(at), decoded) == expected
        # Старые статические интервалы остаются статическими: это иной исход, не повод "исправить" старую запись.
        static = decoded_wire.unvalue(["interval", low_precision_pi])
        assert static._mpi_ == tuple(tuple(endpoint) for endpoint in low_precision_pi)
        decoded._density_exact_memo.intervals.clear()
        decoded._density_exact_memo.intervals[sp.pi] = static
        old_answer = _density_decimal_envelope(decoded_wire.unexpression(at), decoded)
        assert old_answer.absolute_error_bound > expected.absolute_error_bound
        with pytest.raises(CorpusWireError, match="unsupported dynamic interval constant"):
            wire.value(iv.e)
        with pytest.raises(CorpusWireError, match="unsupported typed value"):
            decoded_wire.unvalue(["interval_constant", "e"])
    finally:
        iv.prec = saved


@pytest.mark.parametrize("node", (["Q", True, 1], ["Q", 1, 0], ["Q", 2, 4], ["Pow", 0, 0], ["eval", "__import__('os')"]))
def test_wire_rejects_nonexact_malformed_and_cyclic_nodes(node):
    with pytest.raises(CorpusWireError):
        Wire([node]).unexpression(0)


def test_strict_document_rejects_unknown_fields_and_changed_digest(document):
    malformed = copy.deepcopy(document)
    malformed["work_v2"] = [0, 0, 0]
    with pytest.raises(CorpusWireError, match="document"):
        corpus.validate_document(malformed)
    malformed = copy.deepcopy(document)
    malformed["records"][0]["outcome"]["answer"].append("corrupted")
    with pytest.raises(CorpusWireError, match="digest"):
        corpus.validate_document(malformed)


def test_probe_corpus_records_undecidable_nested_sign_and_rational_decimal_straws():
    document = probes.synthetic_document()
    assert probes.replay(document)["equal"] == 7
    cases = {row["name"]: row["outcome"] for row in document["records"]}
    for name in ("nested_tight_positive_refuses", "nested_tight_negative_refuses"):
        assert cases[name]["answer"] == ["raised", "ReferenceGeometryError",
                                        "Density A exact sign is not certified without generic factorization",
                                        {"outcome": "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE"}]
    for numerator, center in ((1657, "0.3236328125"), (3647, "0.7123046875")):
        outcome = cases[f"rational_straw_{numerator}_5120"]["answer"]
        assert outcome[0] == "ok"
        envelope = outcome[1]
        assert Decimal(envelope["lower"]) == Decimal(center) - Decimal("1e-27")
        assert Decimal(envelope["upper"]) == Decimal(center) + Decimal("1e-27")
        assert Decimal(envelope["absolute_error_bound"]) == Decimal("2e-27")


def test_replay_has_identical_result_in_fresh_hash_seed_processes(tmp_path, document):
    target = tmp_path / "seed.recs.xz"
    corpus.write_document(target, document)
    for seed in ("0", "17"):
        result = subprocess.run([sys.executable, str(ROOT / "tools" / "native_density_corpus.py"),
                                 "replay", "--file", str(target)], cwd=ROOT,
                                env={**os.environ, "PYTHONHASHSEED": seed}, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        assert '"equal":13' in result.stdout and '"mismatch":0' in result.stdout


def test_nonempty_shell_events_survive_warm_wire_and_call_local_recorder(monkeypatch, tmp_path):
    from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector, record_density_shell_candidate
    from cftuv_envelope.reference.density_shell_v2 import density_shell_v2

    original = binding.certify_adaptive_huber_density_direction_fan
    def certify(metric, *args, **kwargs):
        record_density_shell_candidate(metric, density_shell_v2(sp.Rational(1657, 5120)))
        return original(metric, *args, **kwargs)
    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", certify)
    owner, caller, recorder = synthetic.metric(), DensityShellEventCollector(), corpus.Recorder("events")
    ideal, call = synthetic.principal_ideal(owner), recorder.wrapper()
    args = (owner, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 4, (None,))
    kwargs = dict(lifted=True, law=WINDOW_LAW_VORONOI, spec_id="spec", refusals=[], shell_event_collector=caller)
    first = call(*args, **kwargs)
    assert call(*args, **kwargs) is first
    assert len(caller.diagnostics()) == 3
    cold, warm = recorder.rows
    assert cold["outcome"]["shell_events"] == warm["outcome"]["shell_events"]
    assert len(warm["outcome"]["shell_events"]) == 3 and warm["outcome"]["budgets"] == []
    wire = Wire(recorder.wire.nodes)
    restored = wire.unmetric(warm["input"]["metric"])
    assert restored._density_exact_memo.fan_bindings == owner._density_exact_memo.fan_bindings
    assert restored._density_exact_memo.shell_event_capture is None
    target = tmp_path / "shell-events.recs.xz"
    corpus.write_document(target, recorder.document())
    assert corpus.replay(corpus.read_document(target))["equal"] == 2
    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", original)
    # Actual events, отсутствующие в expected, и потерянные expected events оба видимы.
    assert corpus.replay(recorder.document())["first_mismatches"][0]["fields"] == ["shell_events"]
    legacy = copy.deepcopy(recorder.document())
    legacy["records"] = legacy["records"][:1]
    del legacy["records"][0]["outcome"]["shell_events"]
    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", certify)
    assert corpus.replay(legacy)["first_mismatches"][0]["fields"] == ["shell_events"]


def test_tool_reader_adapts_legacy_memo_two_tuple_only(monkeypatch):
    from cftuv_envelope.reference.density_shell_events import DensityFanBindingMemoEntry

    owner = synthetic.metric()
    ideal = synthetic.principal_ideal(owner)
    result = binding.bind_density_fan(owner, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 4, (None,),
                                     lifted=True, law=WINDOW_LAW_VORONOI, spec_id="spec", refusals=[])
    key = next(iter(owner._density_exact_memo.fan_bindings))
    wire = Wire()
    body = wire.metric(owner)
    body["memo"]["fan_bindings"] = wire.value({key: result})
    restored = Wire(wire.nodes).unmetric(body)
    assert restored._density_exact_memo.fan_bindings[key] == DensityFanBindingMemoEntry(result, ())
    def forbidden(*args, **kwargs):
        raise AssertionError("старый warm tuple не превращается в cold recompute")
    monkeypatch.setattr(binding, "certify_adaptive_huber_density_direction_fan", forbidden)
    assert binding.bind_density_fan(restored, ideal, TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, 4, (None,),
                                    lifted=True, law=WINDOW_LAW_VORONOI, spec_id="other", refusals=[]) == result


@pytest.mark.parametrize("payload", (
    ["density_shell_rational_v2", True, 1, 10**27],
    ["density_shell_rational_v2", 2, 4, 5 * 10**26],
    ["density_shell_rational_v2", 1, 0, None],
    ["density_shell_rational_v2", 1, 3, 0],
    ["density_shell_outcomes_v2", ["unknown"], None, None],
    ["density_shell_outcomes_v2", ["DENSITY_DECIMAL_SHELL_V2_APPLIED"], None, True],
    ["density_shell_outcomes_v2", ["DENSITY_DECIMAL_GRID_POINT_V2"], None, None],
    ["fan_binding_memo_v2", ["tuple", []], ["tuple", []]],
    ["fan_binding_memo_v2", ["tuple", [["tuple", []], None]], ["list", []]],
))
def test_shell_event_wire_rejects_wrong_types_and_inconsistent_metadata(payload):
    with pytest.raises(CorpusWireError):
        Wire().unvalue(payload)


def test_optional_outcome_extension_remains_closed(document):
    row = copy.deepcopy(document["records"][0]["outcome"])
    row["unknown_event"] = []
    with pytest.raises(CorpusWireError, match="outcome"):
        corpus.validate_outcome(row)
    del row["unknown_event"]
    row["shell_events"] = []
    with pytest.raises(CorpusWireError, match="nonempty"):
        corpus.validate_outcome(row)


@pytest.mark.parametrize("value", (sp.Integer(0), -sp.Rational(1657, 5120), sp.sqrt(2), sp.Rational(1, 3)))
def test_shell_event_wire_keeps_zero_signed_rational_and_nonrational_metadata(value):
    from cftuv_envelope.reference.density_shell_events import DensityFanBindingMemoEntry
    from cftuv_envelope.reference.density_shell_v2 import density_shell_v2

    candidate, wire = density_shell_v2(value), Wire()
    pair = (candidate.value.wire(), candidate.outcomes)
    entry = DensityFanBindingMemoEntry(((), None), (pair,))
    assert wire.unvalue(wire.value(entry)) == entry
    duplicate = DensityFanBindingMemoEntry(entry.result, (pair, pair))
    with pytest.raises(CorpusWireError, match="distinct"):
        wire.unvalue(wire.value(duplicate))
    wrong = DensityFanBindingMemoEntry(entry.result, (((True, 1, 1), candidate.outcomes),))
    with pytest.raises(CorpusWireError, match="integer"):
        wire.unvalue(wire.value(wrong))
