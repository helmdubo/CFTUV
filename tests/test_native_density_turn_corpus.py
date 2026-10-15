"""Raw-turn corpus: типы/порядок тёплой памяти, H0, политики, частичные отказы и aliases."""

from __future__ import annotations

import copy
import json
import lzma
import os
import subprocess
import sys
from fractions import Fraction
from pathlib import Path

import pytest
import sympy as sp

ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "kernel" / "src", ROOT / "tools"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from cftuv_envelope import backend  # noqa: E402
import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope.reference import angular, density_turn as turn, symbolic_backend as symbolic  # noqa: E402
from cftuv_envelope.reference.common import ReferenceGeometryError  # noqa: E402
from cftuv_envelope.reference.contracts import ReferenceOutcome  # noqa: E402
import native_density_turn_corpus as corpus  # noqa: E402
from native_density_wire import CorpusWireError  # noqa: E402


@pytest.fixture(scope="module")
def document():
    return corpus.synthetic_document()


def _rows(document):
    return {row["test"]: row for row in document["records"]}


def _after(document, row):
    return corpus.TurnWire(document["expressions"]).unmetric(row["outcome"]["memo_after"])._density_exact_memo


def _before(document, row):
    return corpus.TurnWire(document["expressions"]).unmetric(row["input"]["metric"])._density_exact_memo


def test_synthetic_all_outcomes_and_diagnostics_replay(document):
    assert corpus.shape_of(document) == {
        "records": 37, "raised": 12, "zero_hidden_count": 4,
        "atom_key_cached": 15, "fan_key_cached": 11, "expression_nodes": 115,
    }
    assert corpus.replay(document) == {
        "records": 37, "equal": 37, "mismatch": 0, "first_mismatches": [],
        "diagnostic_mismatch": 0, "first_diagnostic_mismatches": [],
    }


def test_h0_reuses_only_the_atom_and_does_not_construct_cosine_or_basis(document):
    rows = _rows(document)
    first, repeated = rows["principal_positive_h0:r0"], rows["principal_positive_h0:r1"]
    assert first["outcome"]["answer"] == repeated["outcome"]["answer"]
    assert repeated["diagnostics"]["atom_key_cached"] is True
    assert repeated["diagnostics"]["fan_key_cached"] is False
    memo = _after(document, repeated)
    assert memo.fans == {}
    atom = next(iter(memo.turn_atoms.values()))
    assert atom.cosine_squared is atom.turn_sign is atom.principal_turn is atom.basis is None
    assert len(corpus.ATOM_FIELDS) == 9


def test_h1_through_h5_replay_the_warm_fan_without_cold_reset(document):
    rows = _rows(document)
    for count in range(1, 6):
        first, repeated = rows[f"principal_positive_h{count}:r0"], rows[f"principal_positive_h{count}:r1"]
        assert first["outcome"]["answer"] == repeated["outcome"]["answer"]
        assert repeated["diagnostics"]["atom_key_cached"] and repeated["diagnostics"]["fan_key_cached"]
        assert len(_before(document, repeated).fans) == 1
        assert len(_after(document, repeated).fans) == 1


def test_irrational_signed_cosine_failure_keeps_the_partial_atom_and_repeats_it(document):
    rows = _rows(document)
    zero, failed, repeated = (rows[name] for name in ("irrational_cos_h0", "irrational_cos_h1", "irrational_cos_h1:repeat"))
    assert zero["outcome"]["answer"][0] == "ok"
    expected = ["raised", "ReferenceGeometryError", "Density A signed-cos-squared is not rational in the declared Gram metric",
                {"outcome": "REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE"}]
    assert failed["outcome"]["answer"] == repeated["outcome"]["answer"] == expected
    assert failed["diagnostics"]["atom_key_cached"] is True
    memo = _after(document, failed)
    atom = next(iter(memo.turn_atoms.values()))
    assert atom.incoming_squared == 1 and atom.outgoing_squared == sp.Rational(5, 2)
    assert atom.cosine_squared is atom.turn_sign is atom.principal_turn is atom.basis is None
    assert memo.fans == {}
    assert failed["outcome"]["memo_after"] == repeated["outcome"]["memo_after"]


def test_short_rotation_failure_preserves_later_partial_stages(document):
    rows = _rows(document)
    row = rows["short_rotation"]
    assert row["outcome"]["answer"] == ["raised", "IndexError", "tuple index out of range", {}]
    memo = _after(document, row)
    atom = next(iter(memo.turn_atoms.values()))
    assert atom.cosine_squared == 0
    assert atom.turn_sign is atom.principal_turn is None
    assert atom.basis == (1, 0, 0, 1)
    assert not memo.fans
    assert rows["short_rotation:repeat"]["diagnostics"]["atom_key_cached"] is True


def test_canonical_and_table_policy_inputs_remain_distinct_and_table_skips_principal_turn(document):
    rows = _rows(document)
    row = rows["canonical_half_turn"]
    wire = corpus.TurnWire(document["expressions"])
    assert wire.unvalue(row["input"]["canonical"]) == Fraction(1, 2)
    assert wire.unvalue(row["input"]["rotation"]) is None
    table = rows["rational_table_h3"]
    assert wire.unvalue(table["input"]["rotation"]) == ((12, 5), (1, 1), (5, 12))
    atom = next(iter(_after(document, table).turn_atoms.values()))
    assert atom.cosine_squared == 0 and atom.principal_turn is None and atom.turn_sign is None
    assert atom.basis is not None
    for name in ("principal_negative", "principal_zero", "clockwise", "reflected_owner"):
        assert rows[name]["outcome"]["answer"][0] == "ok"


def test_count_rejection_precedes_vectors_and_does_not_create_an_atom(document):
    rows = _rows(document)
    for value in ("-1", "6", "bad", "None"):
        row = rows["count_bad:" + value]
        assert row["outcome"]["answer"][3] == {"outcome": "ANGULAR_PROFILE_SELECTION_UNCERTAIN"}
        assert _after(document, row).turn_atoms == {}
        assert row["diagnostics"]["lookup"] == "count_rejected_before_lookup"


@pytest.mark.parametrize("field", corpus.OUTCOME_KEYS)
def test_every_observable_field_including_memo_after_is_compared(document, field):
    altered = copy.deepcopy(document)
    altered["records"] = altered["records"][:1]
    target = altered["records"][0]["outcome"][field]
    if type(target) is list:
        target.append("different")
    else:
        target["different"] = 1
    result = corpus.replay(altered)
    assert result["mismatch"] == 1
    assert result["first_mismatches"][0]["fields"] == [field]


def test_diagnostic_mismatch_is_named_separately_from_the_observable_answer(document):
    altered = copy.deepcopy(document)
    altered["records"] = altered["records"][:1]
    altered["records"][0]["diagnostics"]["atom_key_cached"] = True
    result = corpus.replay(altered)
    assert result["equal"] == 1 and result["mismatch"] == 0
    assert result["diagnostic_mismatch"] == 1


def test_wire_preserves_turn_atom_slots_and_insertion_order(document):
    row = _rows(document)["principal_positive_h4:r1"]
    wire = corpus.TurnWire(document["expressions"])
    decoded = wire.unmetric(row["input"]["metric"])
    body = wire.metric(decoded)
    assert body == row["input"]["metric"]
    atom_wire = body["turn"]["turn_atoms"][1][0][1]
    assert set(atom_wire[1]) == set(corpus.ATOM_FIELDS)
    del atom_wire[1]["basis"]
    with pytest.raises(CorpusWireError, match="turn_atom"):
        wire.unmetric(body)


def test_exception_payload_and_original_identity_are_not_masked():
    recorder = corpus.Recorder("named-error")
    failure = ReferenceGeometryError(ReferenceOutcome.REFERENCE_CERTIFIED_PREDICATE_UNDECIDABLE, "same message")

    def failing(*args, **kwargs):
        symbolic.count("turn_failure", "visited")
        raise failure

    before = dict(symbolic.BACKEND_COUNTS)
    try:
        with pytest.raises(ReferenceGeometryError) as caught:
            recorder.wrapper(failing)(corpus._metric(), corpus._vector(1, 0), corpus._vector(0, 1), 1, 1)
        assert caught.value is failure
        assert recorder.rows[0]["outcome"]["counts"] == {"turn_failure.visited": 1}

        def other(*args, **kwargs):
            symbolic.count("turn_failure", "visited")
            raise ReferenceGeometryError(ReferenceOutcome.REFERENCE_INPUT_CONTRACT_INVALID, "same message")

        assert corpus.replay(recorder.document(), other)["mismatch"] == 1
    finally:
        symbolic.BACKEND_COUNTS.clear()
        symbolic.BACKEND_COUNTS.update(before)


def test_installed_owner_and_angular_alias_are_the_same_wrapper_and_restore():
    owner, alias = turn._huber_density_interpolated_normals, angular._huber_density_interpolated_normals
    recorder = corpus.Recorder("alias")
    with recorder.installed():
        assert turn._huber_density_interpolated_normals is angular._huber_density_interpolated_normals
        angular._interpolated_normals(corpus._metric(), corpus._vector(1, 0), corpus._vector(1, 2), 1,
                                      kernel.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION, huber_density=True)
    assert len(recorder.rows) == 1
    assert turn._huber_density_interpolated_normals is owner and angular._huber_density_interpolated_normals is alias
    assert corpus.replay(recorder.document())["equal"] == 1


def test_real_kernel_compile_records_pre_bind_turn_calls_and_replays():
    from cftuv_envelope.reference import compile as compiler

    fixture = ROOT / "kernel" / "fixtures" / "building_patch17_crowded_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads((fixture / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((fixture / "decal_request_density3.json").read_bytes())
    recorder = corpus.Recorder("kernel:building_patch17:d3")
    with backend.use_backend("PYTHON", "PYTHON", "PYTHON", "PYTHON"), recorder.installed():
        # Этот Recorder проверяет замороженные вызовы старой привязки, до raw Density.
        result = kernel.compile_reference_envelopes(
            snapshot, request,
            _direction_binding_attachment=compiler._attach_direction_bindings_legacy_v1,
        )
    assert result.outcome.value == "EXACT"
    assert len(recorder.rows) > 4
    assert corpus.replay(recorder.document())["equal"] == len(recorder.rows)


def test_json_roundtrip_and_fresh_hash_seed_processes(tmp_path, document):
    path = tmp_path / "turns.xz"
    corpus.write_document(path, document)
    assert json.loads(lzma.decompress(path.read_bytes())) == document
    assert corpus.replay(corpus.read_document(path))["equal"] == 37
    for seed in ("0", "17"):
        result = subprocess.run([sys.executable, str(ROOT / "tools/native_density_turn_corpus.py"), "replay", "--file", str(path)],
                                cwd=ROOT, env={**os.environ, "PYTHONHASHSEED": seed}, capture_output=True, text=True)
        assert result.returncode == 0, result.stdout + result.stderr
        assert '"equal":37' in result.stdout and '"diagnostic_mismatch":0' in result.stdout
