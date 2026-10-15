"""Portable host profile/offline authoring boundary checks; no pin approval."""
import ast
import importlib.util
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('version,full,expected',[(11,False,'CPython-3.11:light'),(13,True,'CPython-3.13:full'),(10,False,'CPython-3.10:light')])
def test_profile_names_actual_runtime_and_optional_model(monkeypatch,version,full,expected):
    from cftuv import envelope_angular_binding as host
    monkeypatch.setattr(host.sys,'version_info',SimpleNamespace(major=3,minor=version))
    monkeypatch.setattr(host.sys,'implementation',SimpleNamespace(name='cpython'))
    if full:monkeypatch.setitem(host.sys.modules,'cftuv.model',ModuleType('cftuv.model'))
    else:monkeypatch.delitem(host.sys.modules,'cftuv.model',raising=False)
    assert host.binding_profile()==expected


def test_generator_import_has_no_host_or_native_or_blender_execution():
    path=ROOT/'tools/native_angular_pins.py'
    spec=importlib.util.spec_from_file_location('_angular_pin_generator_test',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert callable(module.candidate)
    tree=ast.parse(path.read_text(encoding='utf-8-sig'))
    assignments=[node for node in ast.walk(tree)if isinstance(node,(ast.Assign,ast.AugAssign))]
    assert not any(isinstance(n,ast.Name)and n.id=='APPROVED_PINS' for node in assignments for n in ast.walk(node))


def test_production_profile_table_stays_empty():
    tree=ast.parse((ROOT/'native/cftuv-python/python/cftuv_native/angular_op.py').read_text())
    statement=next(n for n in tree.body if isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='APPROVED_PINS'for t in n.targets))
    assert isinstance(statement.value,ast.Call)and isinstance(statement.value.args[0],ast.Dict)and not statement.value.args[0].keys
