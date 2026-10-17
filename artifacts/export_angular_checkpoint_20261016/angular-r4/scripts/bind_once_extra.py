

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
    name = next(key for key, value in vars(kernel.ExactRationalV1).items() if inspect.isfunction(value) and not key.startswith('__'))
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
                          raw=raw, symbolic_mode='exact', before_assemble=lambda: pytest.fail('assembly began'))


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
    assert 'first' in expected
    with wire.source_scope():
        assert wire.getsource(module.Twin) == expected
