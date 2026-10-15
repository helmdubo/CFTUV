"""Portable pure-shim orchestration gates; no extension or approved-pin fixture."""
import ast
import builtins
import importlib
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
SHIM = ROOT / 'native/cftuv-python/python/cftuv_native'


@pytest.fixture
def seam(monkeypatch):
    # Загружаем только чистые модули; __init__ колеса требует расширение.
    package = ModuleType('cftuv_native')
    package.__path__ = [str(SHIM)]
    monkeypatch.setitem(__import__('sys').modules, 'cftuv_native', package)
    for name in ('operation', 'wire', 'bindings', 'assemble', 'runtime_pins'):
        monkeypatch.delitem(__import__('sys').modules, 'cftuv_native.angular_' + name, raising=False)
    op = importlib.import_module('cftuv_native.angular_operation')
    wire, bindings, assembly = op.wire, op.bindings, op.assembly
    log = []
    bound = object.__new__(bindings.BoundAssembly)
    raw = dict(revision='r', **{name: {} for name in bindings.IDS if name not in ('domain', 'metric')})
    encoded = wire.AngularWire(({'patch_id': 7},), (), {})
    monkeypatch.setattr(bindings.BoundAssembly, 'validate', lambda *args: log.append('validate'))
    monkeypatch.setattr(wire, 'encode', lambda *args, **kwargs: (log.append('encode'), encoded)[1])
    monkeypatch.setattr(bindings, 'prepare_checked', lambda *args, **kwargs: (log.append('prepare'), 'prepared')[1])
    def assemble(*args, **kwargs):
        log.append('assemble')
        return (frozenset(),) * 3
    monkeypatch.setattr(assembly, 'assemble', assemble)
    def native(patch):
        log.append('native')
        return (0, ())
    options = dict(wire_binding=SimpleNamespace(metadata={}), assembly_binding=bound,
                   native_patch=native, raw=raw, symbolic_mode='NATIVE_EXACT',
                   before_assemble=lambda: log.append('boundary'))
    return SimpleNamespace(op=op, wire=wire, bindings=bindings, assembly=assembly, log=log, options=options)


def test_boundary_once_immediately_before_successful_assembly(seam):
    assert seam.op.compute(**seam.options) == (frozenset(),) * 3
    assert seam.log == ['validate', 'encode', 'native', 'prepare', 'boundary', 'assemble']


@pytest.mark.parametrize('phase', ['binding', 'wire', 'native', 'prepare'])
def test_decline_never_crosses_boundary(seam, monkeypatch, phase):
    if phase == 'binding':
        monkeypatch.setattr(seam.bindings.BoundAssembly, 'validate', lambda *args: seam.bindings.BindingDecline('fixture'))
    elif phase == 'wire':
        monkeypatch.setattr(seam.wire, 'encode', lambda *args, **kwargs: seam.wire.WireDecline('fixture'))
    elif phase == 'native':
        seam.options['native_patch'] = lambda patch: (2, 'FIXTURE', 'unsupported')
    else:
        monkeypatch.setattr(seam.bindings, 'prepare_checked', lambda *args, **kwargs: seam.assembly.AssemblyDecline('fixture'))
    assert type(seam.op.compute(**seam.options)) is seam.op.OperationDecline
    assert 'boundary' not in seam.log and 'assemble' not in seam.log


@pytest.mark.parametrize('error_type', [ValueError, 'stale'])
def test_post_boundary_original_exception_propagates_without_retry(seam, monkeypatch, error_type):
    error = (seam.bindings.NativePortStale if error_type == 'stale' else error_type)('original')
    def fail(*args, **kwargs):
        seam.log.append('effect')
        raise error
    monkeypatch.setattr(seam.assembly, 'assemble', fail)
    with pytest.raises(type(error)) as found:
        seam.op.compute(**seam.options)
    assert found.value is error
    assert seam.log == ['validate', 'encode', 'native', 'prepare', 'boundary', 'effect']


def test_stale_before_encode_does_not_cross_boundary(seam, monkeypatch):
    error = seam.bindings.NativePortStale('changed helper')
    def stale(*args):
        raise error
    monkeypatch.setattr(seam.bindings.BoundAssembly, 'validate', stale)
    with pytest.raises(type(error)) as found:
        seam.op.compute(**seam.options)
    assert found.value is error and seam.log == []


def test_wire_and_host_builder_preserve_import_wall():
    for path in SHIM.glob('angular_*.py'):
        tree = ast.parse(path.read_text())
        assert not any(isinstance(n, ast.ImportFrom) and n.level == 0 and (n.module or '').split('.')[0] == 'cftuv' for n in ast.walk(tree))
        assert not any(isinstance(n, ast.Import) and any(a.name.split('.')[0] == 'cftuv' for a in n.names) for n in ast.walk(tree))
    tree = ast.parse((ROOT / 'cftuv/envelope_angular_binding.py').read_text())
    assert not any(isinstance(n, ast.ImportFrom) and (n.module or '').startswith('cftuv_native') for n in ast.walk(tree))
    assert not any(isinstance(n, ast.Import) and any(a.name.startswith('cftuv_native') for a in n.names) for n in ast.walk(tree))


def test_runtime_import_wire_with_host_import_blocked(monkeypatch):
    original = builtins.__import__
    def deny_host(name, *args, **kwargs):
        if name == 'cftuv' or name.startswith('cftuv.'):
            raise AssertionError('host discovery forbidden')
        return original(name, *args, **kwargs)
    monkeypatch.setattr(builtins, '__import__', deny_host)
    spec = importlib.util.spec_from_file_location('_angular_wire_import_test', SHIM / 'angular_wire.py')
    module = importlib.util.module_from_spec(spec)
    monkeypatch.setitem(__import__('sys').modules, spec.name, module)
    spec.loader.exec_module(module)
    assert callable(module.bind) and callable(module.encode)
