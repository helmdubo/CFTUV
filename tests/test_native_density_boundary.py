"""Живой typed boundary opt-in; никаких default/pin переключений."""

from dataclasses import replace
from fractions import Fraction
import importlib
import os
from pathlib import Path
import sys

import pytest
from native_gate import skip_unless_available

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'kernel/src'))
from cftuv_envelope.reference import density_oracle_v1 as facts, density_result_v1 as formatter
from cftuv_envelope.reference.density_construct_v1 import ExpandLimitsV1
from cftuv_envelope.reference.density_raw_frontend_v1 import evaluate_raw_density_batch
from cftuv_envelope.reference.density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE
from cftuv_envelope.ids import SelectionCertificateId


@pytest.fixture
def native():
    path = os.environ.get('CFTUV_DENSITY_CANDIDATE_PACKAGE')
    if path:
        sys.path.insert(0, path)
    module = importlib.import_module('cftuv_native')
    if path:
        assert Path(module.__file__).is_relative_to(Path(path))
    else:
        skip_unless_available(module, 'density')
    return module


def _raw(q=4, count=1):
    point = facts.RawDensityPointV1
    metric = facts.RawDensityMetricV1((1, 0, 1), (1, 0, 1), 1)
    geometry = facts.RawDensityGeometryV1(metric,
        facts.RawDensityIncidentV1(point(0, 1), point(0, 0), True, 'corner', 'previous', 'corner'),
        facts.RawDensityIncidentV1(point(0, 0), point(1, -1), True, 'corner', 'corner', 'next'))
    selection = facts.RawDensitySelectionV1(q, count, 'HUBER_EMANATED_COUNT_DENSITY_A_V1',
        f'LINEAR_REFLEX_DENSITY_{q-2}_V1', (q+3)//4, 'selection')
    canonical = facts.RawDensityCanonicalFactsV1((Fraction(1, 4), Fraction(1, 4)), None, (), (('binding_present', False),))
    return facts.RawDensitySpecInput(geometry, geometry, 1, selection, canonical, spec_id='a')


def _batch(raws, search=131072):
    ids = facts.DensityOracleCanonV1(facts.DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE,
                                    facts.DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)
    limits = facts.DensityOracleLimitsV1(65536, search, 96, 1000000, ExpandLimitsV1(4096, 16, 65536))
    return facts.RawDensityBatchInputV1(raws, ids, limits, 'NARROW_ROTATION_BAND')


def _observed(value):
    assert set(name for name, _ in value.rust_work_diagnostic) <= {
        'class_merge_checks', 'conjugation_products', 'enclosure_rounds'}
    authority = formatter.format_authority(value.authority, SelectionCertificateId('selection'))
    return (replace(value, authority=None, rust_work_diagnostic=()), authority,
            tuple(formatter.format_certificate(item) for item in value.binding_certificates))


def test_live_q5_and_same_batch_failed_narrow_retry_then_success_memo(native):
    for batch in (_batch((_raw(q=5),)), _batch((_raw(), replace(_raw(), spec_id='b')), search=1)):
        old, new = evaluate_raw_density_batch(batch), native.density_compute_candidate(batch)
        assert tuple(map(_observed, old)) == tuple(map(_observed, new))
    assert len(new[0].ordered_work_articles) == 3 and len(new[1].ordered_work_articles) == 2


def test_live_h0_raw_profile_and_empty_batch_preserve_valid_no_binding(native):
    batch = _batch((_raw(q=2, count=0),))
    old, new = evaluate_raw_density_batch(batch), native.density_compute_candidate(batch)
    assert tuple(map(_observed, old)) == tuple(map(_observed, new))
    assert new[0].effective_hidden_count == 0 and new[0].binding_certificates == ()
    assert native.density_compute_candidate(replace(batch, specs=())) == ()


def test_native_typeerror_stays_loud_and_public_pin_remains_disabled(native):
    batch = _batch((_raw(),))
    def corrupt(*args):
        raise TypeError('encoder shape gap')
    with pytest.raises(TypeError, match='encoder shape gap'):
        native.density_op.density_compute(corrupt, batch, candidate=True)
    if os.environ.get('CFTUV_DENSITY_CANDIDATE_PACKAGE') and 'density' not in native.pin.OPERATION_FILES:
        with pytest.raises(native.NativePortStale, match='pin is not enabled'):
            native.density_compute(batch)
    else:
        assert tuple(map(_observed, native.density_compute(batch))) == tuple(map(_observed, evaluate_raw_density_batch(batch)))


def test_live_unproven_selection_stays_internal_and_retains_partial_effects(native):
    valid = _raw(q=5)
    # Типы допустимы, но исходный интервал не доказывает заявленный bucket=3.
    raw = replace(valid, selection=replace(valid.selection, bucket_c=3, source_hidden_count=2))
    batch = _batch((raw,))
    with pytest.raises(ValueError, match='DENSITY_RAW_SELECTION_BUCKET_UNPROVEN'):
        evaluate_raw_density_batch(batch)
    with pytest.raises(native.DensityNativeInternalErrorV1) as caught:
        native.density_compute_candidate(batch)
    assert caught.value.spec_index == 0 and caught.value.completed_results == ()
    assert caught.value.partial_effects.failure is None
