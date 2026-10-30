"""Заголовки ответа домена пиклятся кортежем значений полей (SLOTS_STATE_TUPLE_V1) и возвращаются теми же.

Пикл `dataclass(frozen=True, slots=True)` по умолчанию зовёт `dataclasses.fields` на КАЖДОМ объекте (`_dataclass_getstate` даёт список);
заголовки, которые едут в каждом ответе домена, считают имена полей раз на класс и дают кортеж в порядке `fields`.
"""

from __future__ import annotations

import dataclasses
import pickle
from array import array
from fractions import Fraction

import pytest

from cftuv_envelope.backend import BackendRecordV1
from cftuv_envelope.materialize.interval import CERTIFIED, AlphaIntervalV1
from cftuv_envelope.materialize.parametric import PARAMETRIC_LAW, ParametricCertificateV1


def _headers() -> tuple:
    record = BackendRecordV1(
        "native", 3, 1, (("NATIVE_PORT_DECLINED", "coverage", 1, "why"),), skeleton_native_calls=2,
        density_fallbacks=(("NATIVE_PORT_DECLINED", "density", 1, ""),), chain_station_python_calls=4,
    )
    interval = AlphaIntervalV1(CERTIFIED, "", 0.25, 0.2, None, 7, 2)
    certificate = ParametricCertificateV1(
        PARAMETRIC_LAW, Fraction(1, 4), 3, (), "", 2, 1, array("d", [0.0, -0.0, 1.0, 2.0, 3.0, 4.0]), array("d", [0.5, 0.25]),
        counters=(("PARAM_MOVING_VERTICES", 1),), approximate=("PARAM_NON_AFFINE_DEVELOPABLE_OFFSET",), approximate_rate=0.5, memo="HIT",
    )
    return record, interval, certificate


@pytest.mark.parametrize("header", _headers(), ids=lambda header: type(header).__name__)
def test_a_reply_header_pickles_as_the_tuple_of_its_field_values_and_comes_back_the_same(header):
    names = tuple(item.name for item in dataclasses.fields(header))
    state = header.__getstate__()

    assert type(state) is tuple and state == tuple(getattr(header, name) for name in names)
    back = pickle.loads(pickle.dumps(header, protocol=5))
    assert type(back) is type(header)
    assert [repr(getattr(back, name)) for name in names] == [repr(getattr(header, name)) for name in names]  # знак нуля в наклонах тоже
