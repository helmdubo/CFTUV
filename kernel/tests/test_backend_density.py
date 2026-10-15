"""Изолированная raw density стадия: immutable batch, отдельный журнал и честные ошибки."""

import inspect
from types import SimpleNamespace

import pytest

from cftuv_envelope import backend
from cftuv_envelope.reference import density_raw_frontend_v1 as frontend
from cftuv_envelope.reference.density_oracle_v1 import RawDensityBatchInputV1
from test_density_raw_frontend_v1 import _frontend, _forty_five_degree_raw


@pytest.fixture
def batch():
    producer = _frontend()
    return RawDensityBatchInputV1((_forty_five_degree_raw(),), producer.canon_ids, producer.limits, "NARROW_ROTATION_BAND")


@pytest.fixture
def native(monkeypatch):
    names = {name: type(name, (RuntimeError,), {}) for name, _ in backend._NAMED_REFUSALS}
    module = SimpleNamespace(**names, NATIVE_REFUSALS=tuple(names.values()),
        native_build_id=lambda: "density-build", native_status=lambda: {"coverage": "available", "clip": "available", "density": "available"})
    monkeypatch.setattr(backend, "_native", lambda: (module, ""))
    return module


def test_default_python_is_real_raw_operation_and_preserves_arguments(batch, native):
    native.density_compute = lambda _batch: pytest.fail("native was not requested")
    expected = frontend.evaluate_raw_density_batch(batch)
    assert backend.density_compute(batch) == expected
    with backend.use_backend("PYTHON") as ledger:
        assert ledger is None
        assert backend.density_compute(batch) == expected
    assert inspect.signature(backend.use_backend).parameters["density_backend"].kind is inspect.Parameter.KEYWORD_ONLY
    assert inspect.signature(backend.backend_identity).parameters["density_backend"].kind is inspect.Parameter.KEYWORD_ONLY


def test_separate_native_counts_merge_status_identity_and_context(batch, native):
    answer = object()
    native.density_compute = lambda held: answer if held is batch else pytest.fail("batch changed")
    assert backend.native_status().density_available
    assert backend.native_status().as_record()["density"] == "available"
    assert backend.backend_identity("PYTHON") == "PYTHON"
    assert backend.backend_identity("PYTHON", density_backend="NATIVE") == "PYTHON|density=NATIVE:density-build"
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        assert backend.active_density_backend() is backend.KernelBackendV1.NATIVE
        assert backend.active_backend() is backend.KernelBackendV1.PYTHON
        assert backend.density_compute(batch) is answer
        with backend.use_backend("PYTHON", density_backend="NATIVE", borrowed_ledger=ledger) as borrowed:
            assert borrowed is ledger
        with pytest.raises(ValueError, match="BACKEND_BORROW_SCOPE_MISMATCH"):
            with backend.use_backend("PYTHON", borrowed_ledger=ledger):
                pass
        record = ledger.record()
    assert backend.active_density_backend() is backend.KernelBackendV1.PYTHON
    assert record.native_calls == record.python_calls == 0
    assert record.density_native_calls == 1 and record.density_ran == "native"
    merged = record.merged(record)
    assert merged.density_native_calls == 2
    assert merged.as_record()["density_native_calls"] == 2


@pytest.mark.parametrize("cause", ("absent_module", "absent_operation", "NativePortStale", "NativeUnsupportedPython", "NativePortUnsupported"))
def test_named_port_fallback_is_recorded_once(batch, native, monkeypatch, cause):
    answer = object()
    monkeypatch.setattr(frontend, "evaluate_raw_density_batch", lambda held: answer if held is batch else pytest.fail("batch changed"))
    if cause == "absent_module":
        monkeypatch.setattr(backend, "_native", lambda: (None, "missing"))
    elif cause != "absent_operation":
        def refused(_batch):
            raise getattr(native, cause)("named")
        native.density_compute = refused
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        assert backend.density_compute(batch) is answer
    record = ledger.record()
    assert record.density_python_calls == 1 and record.density_native_calls == 0
    expected = "NATIVE_UNAVAILABLE" if cause.startswith("absent") else dict(backend._NAMED_REFUSALS)[cause].value
    assert record.density_outcomes == (expected,)
    assert record.density_fallbacks[0][1:3] == ("density", 1)
    assert record.fallbacks == ()
    assert record.merged(record).density_fallbacks[0][2] == 2


@pytest.mark.parametrize("error", (TypeError("cftuv_native: internal"), RuntimeError("native panic: internal"), ZeroDivisionError("inverse"), ValueError("UnregisteredOpaque")))
def test_internal_and_profile_errors_propagate_without_python(batch, native, monkeypatch, error):
    def raised(_batch):
        raise error
    native.density_compute = raised
    monkeypatch.setattr(frontend, "evaluate_raw_density_batch", lambda _: pytest.fail("internal error cannot fallback"))
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        with pytest.raises(type(error)) as caught:
            backend.density_compute(batch)
    assert caught.value is error
    assert ledger.record().density_python_calls == 0
    assert ledger.record().density_fallbacks == ()


def test_native_division_refuses_domain_without_python(batch, native, monkeypatch):
    def refused(_batch):
        raise native.NativeDivisionDiverged("division")
    native.density_compute = refused
    monkeypatch.setattr(frontend, "evaluate_raw_density_batch", lambda _: pytest.fail("division cannot fallback"))
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        with pytest.raises(backend.NativeDomainRefused) as caught:
            backend.density_compute(batch)
    assert caught.value.outcome is backend.BackendOutcomeV1.NATIVE_DIVISION_DIVERGED
    assert ledger.record().density_outcomes == ("NATIVE_DIVISION_DIVERGED",)


def test_missing_density_status_does_not_disable_other_stages(native):
    native.native_status = lambda: {"coverage": "available", "clip": "available"}
    status = backend.native_status()
    assert status.available and status.density == backend.UNAVAILABLE
    assert not status.density_available


def test_memo_operation_is_optional_keyword_and_preserves_old_tuple_api(batch, native):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityAttemptMemoSnapshotV1, DensityMemoBatchResultV1
    snapshot = DensityAttemptMemoSnapshotV1()
    assert inspect.signature(backend.density_compute).parameters["memo_snapshot"].kind is inspect.Parameter.KEYWORD_ONLY
    assert type(backend.density_compute(batch)) is tuple
    expected = frontend.evaluate_raw_density_batch_with_memo(batch, snapshot)
    assert type(expected) is DensityMemoBatchResultV1
    assert backend.density_compute(batch, memo_snapshot=snapshot) == expected
    calls = []
    def compute(held_batch, held_snapshot):
        calls.append((held_batch, held_snapshot))
        return expected
    native.density_compute_with_memo = compute
    native.density_compute = lambda _: pytest.fail("memo route called stateless native")
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        assert backend.density_compute(batch, memo_snapshot=snapshot) is expected
    assert calls == [(batch, snapshot)]
    assert ledger.record().density_native_calls == 1 and ledger.record().density_python_calls == 0


@pytest.mark.parametrize("cause", ("absent_operation", "NativePortStale", "NativePortUnsupported"))
def test_named_memo_port_fallback_calls_same_python_snapshot_once(batch, native, monkeypatch, cause):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityAttemptMemoSnapshotV1
    snapshot, calls, answer = DensityAttemptMemoSnapshotV1(), [], object()
    def oracle(held, held_snapshot):
        calls.append((held, held_snapshot))
        return answer
    monkeypatch.setattr(frontend, "evaluate_raw_density_batch_with_memo", oracle)
    if cause != "absent_operation":
        def refused(*_):
            raise getattr(native, cause)("named memo refusal")
        native.density_compute_with_memo = refused
    with backend.use_backend("PYTHON", density_backend="NATIVE") as ledger:
        assert backend.density_compute(batch, memo_snapshot=snapshot) is answer
    assert calls == [(batch, snapshot)]
    assert ledger.record().density_python_calls == 1 and ledger.record().density_native_calls == 0


def test_memo_internal_exception_retains_delta_and_never_falls_back(batch, native, monkeypatch):
    from cftuv_envelope.reference.density_attempt_memo_v1 import DensityAttemptMemoSnapshotV1
    snapshot = DensityAttemptMemoSnapshotV1()
    error = ZeroDivisionError("memo native internal")
    error.density_attempt_memo_delta = snapshot
    def fail(*_):
        raise error
    native.density_compute_with_memo = fail
    monkeypatch.setattr(frontend, "evaluate_raw_density_batch_with_memo", lambda *_: pytest.fail("internal fallback"))
    with backend.use_backend("PYTHON", density_backend="NATIVE"):
        with pytest.raises(ZeroDivisionError) as caught:
            backend.density_compute(batch, memo_snapshot=snapshot)
    assert caught.value is error and caught.value.density_attempt_memo_delta is snapshot
    with pytest.raises(TypeError, match="DensityAttemptMemoSnapshotV1"):
        backend.density_compute(batch, memo_snapshot=())
