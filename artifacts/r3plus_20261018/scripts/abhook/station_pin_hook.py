"""TEST-ONLY runtime installation of the chain_station pin CANDIDATE into cftuv_native.chain_station_op of THIS process (field A/B of R3+ S2).

Not product code. Active only when STATION_HARNESS_APPROVE=1: when `cftuv_native.chain_station_op` is executed (or at once when it is already imported), its `APPROVED_PINS` becomes
a copy of `CANDIDATE["files"]` (the files are still compared with the tree: a moved oracle stays `stale`). The candidate's file is not changed and the build id stays. Every
injection is appended to STATION_HARNESS_LOG (one JSON line: pid, module file, build id).
"""
import importlib.abc
import json
import os
import sys

NAME = 'cftuv_native.chain_station_op'


def _log(entry):
    path = os.environ.get('STATION_HARNESS_LOG')
    if path:
        with open(path, 'a', encoding='utf-8') as stream:
            stream.write(json.dumps(entry) + '\n')


def _inject(module):
    assert not module.APPROVED_PINS and module.CANDIDATE['approved'] is False, 'PRODUCT_TABLE_NOT_EMPTY'
    module.APPROVED_PINS = dict(module.CANDIDATE['files'])
    module.refresh()
    _log(dict(pid=os.getpid(), argv0=sys.argv[0] if sys.argv else '', module=getattr(module, '__file__', None)))


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
        original = spec.loader.exec_module

        def exec_module(module):
            original(module)
            _inject(module)
        spec.loader.exec_module = exec_module
        return spec


def install():
    if os.environ.get('STATION_HARNESS_APPROVE') != '1':
        return
    if NAME in sys.modules:
        _inject(sys.modules[NAME])
        return
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
