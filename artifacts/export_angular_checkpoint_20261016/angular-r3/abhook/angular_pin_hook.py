"""TEST-ONLY runtime injection of isolated-parity ANGULAR pins into cftuv_native.angular_op of THIS process.

Not product code. Active only when ANGULAR_HARNESS_PROFILES names test-only profile JSON files (os.pathsep-separated). Each profile must carry
approved=True, production_approved=False and the isolated approval scope; its pins (hashes/runtime/wire_metadata) are put into a fresh
APPROVED_PINS table when cftuv_native.angular_op is executed (or at once when it is already imported). Every injection is appended to
ANGULAR_HARNESS_LOG (one JSON line: pid, profiles, module file, build id).
"""
import copy, importlib.abc, json, os, sys
from types import MappingProxyType

SCOPE = 'isolated parity evaluation only; not production acceptance'
NAME = 'cftuv_native.angular_op'


def _table():
    table = {}
    for path in filter(None, os.environ.get('ANGULAR_HARNESS_PROFILES', '').split(os.pathsep)):
        with open(path, encoding='utf-8') as stream:
            profile = json.load(stream)
        assert profile['approved'] is True and profile['production_approved'] is False and profile['approval_scope'] == SCOPE, 'TEST_ONLY_PROFILE_REQUIRED ' + path
        pins = {key: copy.deepcopy(profile[key]) for key in ('hashes', 'runtime', 'wire_metadata')}
        shapes = pins['wire_metadata']['class_shapes']
        for name, value in shapes.items():
            assert type(value) is list and all(type(v) is str for v in value), 'CLASS_SHAPES'
            shapes[name] = tuple(value)
        assert profile['profile'] not in table, 'DUPLICATE_PROFILE'
        table[profile['profile']] = pins
    return table


def _log(entry):
    path = os.environ.get('ANGULAR_HARNESS_LOG')
    if path:
        with open(path, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(entry) + '\n')


def _inject(module):
    assert not module.APPROVED_PINS, 'PRODUCT_TABLE_NOT_EMPTY'
    table = _table()
    module.APPROVED_PINS = MappingProxyType(table)
    package = sys.modules.get('cftuv_native')
    _log(dict(pid=os.getpid(), argv0=sys.argv[0] if sys.argv else '', profiles=sorted(table), module=getattr(module, '__file__', None),
              package=getattr(package, '__file__', None)))


class _Finder(importlib.abc.MetaPathFinder):
    def find_spec(self, name, path, target=None):
        if name != NAME:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, 'find_spec'):
                continue
            spec = finder.find_spec(name, path, target)
            if spec is not None:
                break
        else:
            return None
        loader = spec.loader
        original = loader.exec_module

        def exec_module(module):
            original(module)
            _inject(module)
        loader.exec_module = exec_module
        return spec


def install():
    if not os.environ.get('ANGULAR_HARNESS_PROFILES'):
        return
    if NAME in sys.modules:
        _inject(sys.modules[NAME])
        return
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
