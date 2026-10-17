"""ANGULAR binds once per process and checks cheaply per call, without letting a changed oracle through.

ANGULAR_BIND_AT_WORKER_START_V1 / ANGULAR_BIND_ONCE_CHECK_PER_CALL_V1 / ANGULAR_BIND_SOURCE_SCOPE_V1: the source digests and code fingerprints
(wire.bind + freeze) are taken once per bind, a pool worker binds in its start, and a call only compares identities with the snapshots of the
bind. These tests use TEST-ONLY pins observed from the live host in the test itself (monkeypatched over the owner-approved product table,
restored), never product constants; with an empty table `warm_angular` binds nothing.
"""
import inspect
import sys
from pathlib import Path
from types import MappingProxyType

import pytest

ROOT = Path(__file__).resolve().parents[1]
for _path in (ROOT / 'kernel' / 'src', ROOT):
    if str(_path) not in sys.path:
        sys.path.insert(0, str(_path))

try:
    import cftuv_native  # noqa: F401
except ModuleNotFoundError as error:
    if error.name != 'cftuv_native':
        raise
    pytest.skip('the cftuv_native extension is not built: the ANGULAR binding checks are skipped', allow_module_level=True)

import sympy  # noqa: E402

import cftuv_envelope as kernel  # noqa: E402
from cftuv import envelope_angular_binding as host  # noqa: E402
from cftuv_envelope import backend  # noqa: E402
from cftuv_native import angular_bindings as bindings, angular_op as op, angular_runtime_pins as runtime, angular_wire as wire, pin  # noqa: E402


def _observed_pins():
    inputs = host.build_binding_inputs(kernel)
    profile = inputs.pop('profile')
    wb = wire.bind(**inputs)
    deps = host.build_dependencies(kernel, sympy)
    objects = {name: bindings._get(owner, key) for name, owner, key in bindings._slots(deps, wb)}
    return profile, dict(hashes=bindings.observe_hashes(deps, wb), runtime=runtime.observe(objects), wire_metadata=wb.metadata)


@pytest.fixture
def test_only_pins(monkeypatch):
    profile, pins = _observed_pins()
    monkeypatch.setattr(op, 'APPROVED_PINS', MappingProxyType({profile: pins}))
    monkeypatch.setattr(op, '_BINDINGS', {})
    pin.refresh()
    yield profile
    pin.refresh()


def _warm():
    return backend.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))


def test_empty_product_table_binds_nothing(monkeypatch):
    monkeypatch.setattr(op, 'APPROVED_PINS', MappingProxyType({}))
    before = dict(op._BINDINGS)
    assert _warm() is False
    assert op._BINDINGS == before


def test_warm_binds_once_per_process(test_only_pins):
    assert _warm() is True
    bound = op._BINDINGS[test_only_pins][3]
    assert _warm() is True and op._BINDINGS[test_only_pins][3] is bound, 'bound once: a later call reuses the bind'


def test_a_replaced_oracle_function_is_stale_on_the_next_call(test_only_pins, monkeypatch):
    from cftuv import envelope_angular_relations as angular
    assert _warm() is True
    monkeypatch.setattr(angular, 'chart_points', lambda *args: pytest.fail('a stale oracle ran'))
    with pytest.raises(cftuv_native.NativePortStale):
        cftuv_native.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))
    assert _warm() is False


def test_changed_code_of_a_bound_function_is_stale(test_only_pins, monkeypatch):
    from cftuv_envelope.reference import angle_measure
    assert _warm() is True
    target = angle_measure.reflex_angle_intervals_over_pi
    monkeypatch.setattr(target, '__code__', (lambda *args, **kwargs: None).__code__.replace(co_name=target.__code__.co_name))
    with pytest.raises(cftuv_native.NativePortStale, match='FUNCTION_STATE_CHANGED|CALLABLE_OR_CLASS_CHANGED'):
        cftuv_native.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))


def test_a_new_member_of_a_bound_class_is_stale(test_only_pins, monkeypatch):
    assert _warm() is True
    monkeypatch.setattr(kernel.ExactRationalV1, 'angular_test_member', lambda self: None, raising=False)
    with pytest.raises(cftuv_native.NativePortStale, match='CLASS_MEMBERS_CHANGED'):
        cftuv_native.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))


def test_a_stale_file_pin_declines_by_name_before_any_bind(test_only_pins, monkeypatch):
    monkeypatch.setitem(pin._VERDICTS, 'angular', ('stale', ('reference/metric.py',)))
    with pytest.raises(cftuv_native.NativePortStale, match='reference/metric.py'):
        cftuv_native.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))
    assert _warm() is False and test_only_pins not in op._BINDINGS


def test_a_changed_source_digest_fails_the_bind(test_only_pins, monkeypatch):
    """A new process (or a rebind) whose oracle source differs from the approved digest refuses by name: the once-per-bind check is the gate."""
    profile = test_only_pins
    pins = dict(op.APPROVED_PINS[profile])
    pins['hashes'] = dict(pins['hashes'], **{'wire.oracle': '0' * 64})
    monkeypatch.setattr(op, 'APPROVED_PINS', MappingProxyType({profile: pins}))
    with pytest.raises(cftuv_native.NativePortStale, match='SOURCE_PIN_MISMATCH:wire.oracle'):
        cftuv_native.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))
    assert profile not in op._BINDINGS


def test_the_source_scope_reads_exactly_what_inspect_reads():
    inputs = host.build_binding_inputs(kernel)
    inputs.pop('profile')
    deps = host.build_dependencies(kernel, sympy)
    wb = wire.bind(**inputs)
    objects = [bindings._get(owner, key) for _, owner, key in bindings._slots(deps, wb)]
    objects += [kind for kinds in inputs['classes'].values() for kind in kinds]
    objects = [type(value) if isinstance(value, bindings.Enum) else value for value in objects]
    with wire.source_scope():
        scoped = [wire.getsource(value) for value in objects]
        again = [wire.getsource(value) for value in objects]
    assert scoped == again == [inspect.getsource(value) for value in objects]
    assert wire._SCOPE[0] is None, 'the scope ends with the bind'


def _rewarm():
    return cftuv_native.warm_angular(binding_inputs=host.build_binding_inputs(kernel), dependencies=host.build_dependencies(kernel, sympy))


def test_a_same_code_copy_of_a_bound_function_is_stale(test_only_pins, monkeypatch):
    """Identity, not equal code: a functional copy put in place of the bound object is a different callable."""
    import types
    from cftuv_envelope.reference import angle_measure
    assert _warm() is True
    original = angle_measure.reflex_angle_intervals_over_pi
    copy = types.FunctionType(original.__code__, original.__globals__, original.__name__, original.__defaults__, original.__closure__)
    copy.__kwdefaults__ = original.__kwdefaults__
    copy.__qualname__ = original.__qualname__
    monkeypatch.setattr(angle_measure, 'reflex_angle_intervals_over_pi', copy)
    with pytest.raises(cftuv_native.NativePortStale, match='CALLABLE_OR_CLASS_CHANGED'):
        _rewarm()


def test_a_replaced_member_of_a_bound_class_is_stale(test_only_pins, monkeypatch):
    """Same names, same count, another value: the per-call member check compares values, not only the namespace size."""
    assert _warm() is True
    name = next(key for key, value in vars(kernel.ExactRationalV1).items() if inspect.isfunction(value))
    monkeypatch.setattr(kernel.ExactRationalV1, name, lambda *args, **kwargs: None)
    with pytest.raises(cftuv_native.NativePortStale, match='CLASS_MEMBERS_CHANGED'):
        _rewarm()


def test_the_operation_gates_anchors_before_encode(test_only_pins, monkeypatch):
    """prepare_checked trusts the early gate of the same call (anchors_checked=True): that gate must run before encode can reach stale host code."""
    from cftuv_envelope.reference import angle_measure
    from cftuv_native import angular_operation as operation
    assert _warm() is True
    bound = op._BINDINGS[test_only_pins][3]
    monkeypatch.setattr(angle_measure, 'reflex_angle_intervals_over_pi', lambda *args, **kwargs: None)
    monkeypatch.setattr(wire, 'encode', lambda *args, **kwargs: pytest.fail('encode ran past a stale anchor'))
    raw = dict({name: {} for name in bindings.IDS if name not in ('domain', 'metric')}, revision='r')
    with pytest.raises(bindings.NativePortStale, match='CALLABLE_OR_CLASS_CHANGED'):
        operation.compute(wire_binding=bound.wire_binding, assembly_binding=bound, native_patch=lambda patch: pytest.fail('native ran'),
                          raw=raw, symbolic_mode='SYMPY', before_assemble=lambda: pytest.fail('assembly began'))


def test_a_bind_whose_member_snapshot_is_not_the_live_class_refuses(test_only_pins):
    """The bind-time snapshots are compared with the live objects when the bound assembly is built, not trusted."""
    import dataclasses
    assert _warm() is True
    bound = op._BINDINGS[test_only_pins][3]
    index = next(i for i, anchor in enumerate(bound.anchors) if anchor[4] is not None)
    name, owner, key, value, members, state = bound.anchors[index]
    forged = bound.anchors[:index] + ((name, owner, key, value, members[1:], state),) + bound.anchors[index + 1:]
    with pytest.raises(bindings.NativePortStale, match='CLASS_MEMBERS_CHANGED:' + name):
        dataclasses.replace(bound, anchors=forged)


def test_the_scope_cache_never_serves_another_object():
    """The per-scope cache is keyed by id; an entry left by a dead object with a reused id must not be served."""
    from cftuv_envelope.reference import angle_measure
    target = angle_measure.reflex_angle_intervals_over_pi
    with wire.source_scope():
        wire._SCOPE[0]['text'][id(target)] = (object(), 'not the source')
        assert wire.getsource(target) == inspect.getsource(target)


def test_the_scope_class_lookup_is_first_match_like_inspect(tmp_path, monkeypatch):
    """Two classes with one qualname in a file: inspect's finder returns the first in visiting order; so must the one-walk table."""
    import importlib.util
    path = tmp_path / 'angular_twin_classes.py'
    path.write_text('class Twin:\n    first = 1\n\n\nclass Twin:\n    second = 2\n', encoding='utf-8')
    spec = importlib.util.spec_from_file_location('angular_twin_classes', path)
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(sys.modules, 'angular_twin_classes', module)
    spec.loader.exec_module(module)
    expected = inspect.getsource(module.Twin)
    if sys.version_info[:2] == (3, 11):
        assert 'first' in expected, 'the 3.11 finder returns the first match (3.13+ reads __firstlineno__ and is not this path)'
    with wire.source_scope():
        assert wire.getsource(module.Twin) == expected
