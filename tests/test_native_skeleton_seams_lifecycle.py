"""Рекордер не оставляет свои функции в модулях, импортированных внутри блока."""
import sys
from pathlib import Path
from types import ModuleType

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import native_corpus as nc
import native_skeleton_seams as seams
from cftuv_envelope.wavefront import event_time, events


@pytest.fixture(autouse=True)
def restore_process_state():
    before = nc.capture_state(None, None)
    yield
    nc.restore_state(before)


def recorder():
    return seams.SeamRecorder(nc.capture_state(None, None))


def late_module(monkeypatch, suffix):
    module = ModuleType("cftuv_envelope._late_seam_" + suffix)
    monkeypatch.setitem(sys.modules, module.__name__, module)
    exec("from cftuv_envelope.wavefront.event_time import compare_times", module.__dict__)
    return module


def test_late_import_restores_original_and_cannot_write_to_the_old_recorder(monkeypatch):
    original = event_time.compare_times
    record = recorder()
    with record.installed():
        late = late_module(monkeypatch, "normal")
        assert late.compare_times is event_time.compare_times is not original
        assert late.compare_times(event_time.ZERO_TIME, event_time.ZERO_TIME, None) == 0
    assert late.compare_times is original
    counted = record.seen["COMPARE_TIMES"]
    assert late.compare_times(event_time.ZERO_TIME, event_time.ZERO_TIME, None) == 0
    assert record.seen["COMPARE_TIMES"] == counted == 1


def test_exception_exit_restores_the_late_import(monkeypatch):
    original = event_time.compare_times
    with pytest.raises(RuntimeError, match="inside recording"):
        with recorder().installed():
            late = late_module(monkeypatch, "exception")
            raise RuntimeError("inside recording")
    assert late.compare_times is event_time.compare_times is original


def test_without_late_import_every_preexisting_callable_and_descriptor_is_unchanged():
    original = event_time.compare_times
    normalized = event_time.EventTimeV1.__dict__["normalized"]
    methods = {name: events.EventQueueV1.__dict__[name] for name in ("push", "pop_level", "_count_at_time", "peek_time")}
    with recorder().installed():
        assert event_time.compare_times is not original
        assert event_time.compare_times(event_time.ZERO_TIME, event_time.ZERO_TIME, None) == 0
    assert event_time.compare_times is original
    assert event_time.EventTimeV1.__dict__["normalized"] is normalized
    assert all(events.EventQueueV1.__dict__[name] is method for name, method in methods.items())


def test_nested_exit_preserves_outer_wrappers_then_restores_late_aliases(monkeypatch):
    original = event_time.compare_times
    normalized = event_time.EventTimeV1.normalized
    with recorder().installed():
        outer_compare = event_time.compare_times
        outer_normalized = event_time.EventTimeV1.normalized
        with recorder().installed():
            late = late_module(monkeypatch, "nested")
            late.normalized = event_time.EventTimeV1.normalized
            assert late.normalized is not outer_normalized
        assert late.compare_times is event_time.compare_times is outer_compare
        assert late.normalized is event_time.EventTimeV1.normalized is outer_normalized
    assert late.compare_times is original
    assert late.normalized is normalized
