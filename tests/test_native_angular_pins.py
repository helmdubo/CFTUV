"""Portable host profile/offline authoring boundary checks; the approved table is owner data, never generated."""
import ast
import hashlib
import importlib.util
import json
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


#: ANGULAR_PINS_APPROVED_V1 (владелец, 2026-10-17): ровно кандидаты r4 на 0cb37dad. Любое добавление или изменение - новое утверждение владельца.
APPROVED_DIGESTS={'CPython-3.11:full':'6fade87372c0a91a8174392ff02e305e598fab7c607c35327c23fea50bb562af',
                  'CPython-3.11:light':'cd3bdd69b65512114503142a5cc9da4e455839b56acf0a13b6977f1927f14d8c'}


def test_production_profile_table_is_exactly_the_owner_approved_pins():
    path=ROOT/'native/cftuv-python/python/cftuv_native/angular_approved_pins.py'
    tree=ast.parse(path.read_text(encoding='utf-8'))
    body=[n for n in tree.body if not(isinstance(n,ast.Expr)and isinstance(n.value,ast.Constant))]
    assert len(body)==1 and isinstance(body[0],ast.Assign)and[t.id for t in body[0].targets]==['APPROVED'],'data only: one literal, no code'
    table=ast.literal_eval(body[0].value)
    assert set(table)==set(APPROVED_DIGESTS)
    for profile,digest in APPROVED_DIGESTS.items():
        assert hashlib.sha256(json.dumps(table[profile],sort_keys=True,separators=(',',':')).encode()).hexdigest()==digest,profile
    op_tree=ast.parse((ROOT/'native/cftuv-python/python/cftuv_native/angular_op.py').read_text(encoding='utf-8'))
    statement=next(n for n in op_tree.body if isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='APPROVED_PINS'for t in n.targets))
    assert ast.dump(statement.value)==ast.dump(ast.parse('MappingProxyType(_APPROVED)',mode='eval').body)
