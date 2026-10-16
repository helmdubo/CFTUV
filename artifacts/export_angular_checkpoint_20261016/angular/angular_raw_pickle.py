"""Приватный trusted pickle: mappingproxy сохраняет тип и общий backing mapping."""
import copyreg
import dataclasses
import gc
import io
import pickle
from types import MappingProxyType


def restore_mappingproxy(mapping):
    return MappingProxyType(mapping)


def reduce_mappingproxy(value):
    # Копия dict потеряла бы тождество backing mapping и связь двух views.
    references = gc.get_referents(value)
    if len(references) != 1:
        raise TypeError('UNSUPPORTED_MAPPINGPROXY_REFERENTS')
    return restore_mappingproxy, (references[0],)


def dumps(value):
    buffer = io.BytesIO()
    writer = pickle.Pickler(buffer, protocol=5)
    writer.dispatch_table = dict(copyreg.dispatch_table)
    writer.dispatch_table[MappingProxyType] = reduce_mappingproxy
    writer.dump(value)
    return buffer.getvalue()


@dataclasses.dataclass
class Representative:
    values: object


def selftest():
    original_dispatch = dict(copyreg.dispatch_table)
    shared = ['same-object']
    inner = {'shared': shared}
    inner_proxy = MappingProxyType(inner)
    backing = {'nested': inner_proxy, 'shared': shared}
    first, second = MappingProxyType(backing), MappingProxyType(backing)
    record = Representative({'first': first, 'shared': shared})
    source = [first, first, second, backing, record, inner, inner_proxy]
    restored = pickle.loads(dumps(source))
    a, repeated, b, mapping, item, inner_mapping, nested = restored
    assert type(a) is type(b) is type(nested) is MappingProxyType
    assert type(item) is Representative
    assert a is repeated is item.values['first'] and a is not b
    assert gc.get_referents(a)[0] is gc.get_referents(b)[0] is mapping
    assert gc.get_referents(nested)[0] is inner_mapping and a['nested'] is nested
    assert a['shared'] is b['shared'] is nested['shared'] is item.values['shared']
    mapping['new'] = 'visible-in-both-views'
    assert a['new'] == b['new'] == 'visible-in-both-views'
    assert dict(copyreg.dispatch_table) == original_dispatch
    try: a['forbidden'] = 1
    except TypeError: pass
    else: raise AssertionError('MAPPINGPROXY_MUTABLE')
    return {'status':'RAW_PICKLE_SELFTEST_PASS', 'proxy_type':True, 'backing_identity':True,
            'shared_values':True, 'nested_dataclass':True, 'global_copyreg_unchanged':True}


if __name__ == '__main__':
    raise SystemExit('Import angular_raw_pickle and explicitly call selftest; no implicit work')
