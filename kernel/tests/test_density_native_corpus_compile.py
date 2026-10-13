"""Рекордер R3 стоит на настоящей compile-границе ядра; не подменяет фикстуру веером."""

from __future__ import annotations

import sys
from pathlib import Path

import cftuv_envelope as kernel
from cftuv_envelope import backend
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))

import native_density_corpus as corpus  # noqa: E402


@pytest.mark.parametrize("density", (2, 3, 4))
def test_real_field_fixture_compilation_is_recorded_and_replays(tmp_path, density):
    fixture = ROOT / "kernel" / "fixtures" / "building_patch17_crowded_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((fixture / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((fixture / f"decal_request_density{density}.json").read_bytes())
    recorder = corpus.Recorder(f"kernel:building_patch17:d{density}")
    from cftuv_envelope.reference import compile as compiler
    with backend.use_backend("PYTHON", "PYTHON", "PYTHON", "PYTHON"), recorder.installed():
        result = kernel.compile_reference_envelopes(snapshot, request,
            _direction_binding_attachment=compiler._attach_direction_bindings_legacy_v1)
    assert result.outcome.value == "EXACT"
    assert recorder.rows
    path = tmp_path / "compiled.recs.xz"
    corpus.write_document(path, recorder.document())
    assert corpus.replay(corpus.read_document(path))["equal"] == len(recorder.rows)
    assert recorder.rows[0]["outcome"]["answer"][0] == "ok"
    assert any(row["outcome"]["budgets"] for row in recorder.rows)
