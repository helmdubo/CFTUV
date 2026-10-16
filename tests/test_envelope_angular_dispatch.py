"""Host ANGULAR dispatch preserves the frozen oracle and fallback boundary."""
import ast
import hashlib
import inspect
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
APPROVED_REFERENCE_AST = 'a809d58b53987591ca2e51c1804f6fbc49f45ab606fd653f71f2716e19b3d7e0'


def test_reference_body_and_signature_remain_frozen():
    tree = ast.parse((ROOT / 'cftuv/envelope_angular_relations.py').read_text(encoding='utf-8'))
    functions = {n.name:n for n in tree.body if isinstance(n, ast.FunctionDef)}
    reference = functions['_build_angular_relations_reference']
    assert hashlib.sha256(ast.dump(reference, include_attributes=False).encode()).hexdigest() == APPROVED_REFERENCE_AST
    assert ast.dump(reference.args) == ast.dump(functions['build_angular_relations'].args)


@pytest.fixture
def dispatch(monkeypatch):
    from cftuv import envelope_angular_relations as angular, envelope_angular_binding as binding
    from cftuv_envelope import backend
    log = []
    kernel, sympy = object(), object()
    raw = dict(analysis_bundle=object(), revision='r', patch_ids={}, patch_domains={}, frames={},
               use_id_by_ref={}, sector_id_by_ref={}, launch_id_by_ref={}, record_by_ref={},
               vertex_ids={}, triangle_ids={}, profile=object())
    expected = (frozenset({'oracle'}), frozenset(), frozenset())
    def reference(k, sp, **kwargs):
        assert k is kernel and sp is sympy
        assert kwargs == raw
        log.append('oracle')
        return expected
    monkeypatch.setattr(angular, '_build_angular_relations_reference', reference)
    monkeypatch.setattr(backend, 'active_angular_backend', lambda: backend.KernelBackendV1.NATIVE)
    monkeypatch.setattr(binding, 'build_binding_inputs', lambda k: (log.append('binding'), {'kernel':k})[1])
    monkeypatch.setattr(binding, 'build_dependencies', lambda k, sp: (log.append('dependencies'), (k,sp))[1])
    return angular, backend, log, lambda: angular.build_angular_relations(kernel, sympy, **raw), expected


def test_python_order_calls_reference_before_any_binding(dispatch, monkeypatch):
    angular, backend, log, call, expected = dispatch
    monkeypatch.setattr(backend, 'active_angular_backend', lambda: backend.KernelBackendV1.PYTHON)
    monkeypatch.setattr(backend, 'angular_compute', lambda **kwargs: pytest.fail('native entered'))
    assert call() == expected and log == ['oracle']


@pytest.mark.parametrize('done', [False, True])
def test_named_decline_once_or_valid_empty_native_success(dispatch, monkeypatch, done):
    angular, backend, log, call, expected = dispatch
    empty = (frozenset(),)*3
    def compute(**kwargs):
        assert set(kwargs) == {'binding_inputs','dependencies','raw','symbolic_mode'}
        assert kwargs['symbolic_mode'] in ('SYMPY','NATIVE_EXACT','SHADOW')
        log.append('native')
        return done, empty
    monkeypatch.setattr(backend, 'angular_compute', compute)
    assert call() == (empty if done else expected)
    assert log == ['binding','dependencies','native'] + ([] if done else ['oracle'])


def test_native_error_does_not_retry_oracle(dispatch, monkeypatch):
    angular, backend, log, call, expected = dispatch
    error = RuntimeError('original native/measure error')
    def fail(**kwargs):
        log.append('native')
        raise error
    monkeypatch.setattr(backend, 'angular_compute', fail)
    with pytest.raises(RuntimeError) as found:
        call()
    assert found.value is error and log == ['binding','dependencies','native']


def test_export_alias_still_targets_only_public_wrapper():
    from cftuv import envelope_angular_relations as angular, envelope_request_export as export
    assert export._build_angular_relations is angular.build_angular_relations
    assert inspect.signature(angular.build_angular_relations) == inspect.signature(angular._build_angular_relations_reference)


def test_replay_only_known_geometry_prices_and_preserves_backend_counts():
    from cftuv import envelope_angular_binding as binding
    from cftuv_envelope.reference import planar_types, symbolic_backend
    before = dict(planar_types.SYMBOLIC_FALLBACK_COUNTS)
    backend_before = dict(symbolic_backend.BACKEND_COUNTS)
    try:
        binding.replay_event('canonical')
        binding.replay_event('normalize')
        assert planar_types.SYMBOLIC_FALLBACK_COUNTS == {key: value + (key in ('canonical','normalize')) for key,value in before.items()}
        for event in ('sign', 'other', True):
            with pytest.raises(ValueError):
                binding.replay_event(event)
        assert symbolic_backend.BACKEND_COUNTS == backend_before
    finally:
        planar_types.SYMBOLIC_FALLBACK_COUNTS.clear()
        planar_types.SYMBOLIC_FALLBACK_COUNTS.update(before)
