"""Небольшие настоящие вызовы Density: q5, principal H>=1, nested roots, отказы, cache hit."""

from __future__ import annotations

import contextlib

import sympy as sp

from cftuv_envelope.contracts.analysis import TurnOrientation
from cftuv_envelope.reference import adaptive_density_fan as fan
from cftuv_envelope.reference.adaptive_density_band import WINDOW_LAW_NARROW_BAND, WINDOW_LAW_VORONOI
from cftuv_envelope.reference.angular import _huber_density_interpolated_normals
from cftuv_envelope.reference.metric import ExactPlanarMetric
from native_density_corpus import Recorder
from native_density_wire import ExpressionVector


def metric(sign=1):
    identity = ((sp.Integer(1), sp.Integer(0)), (sp.Integer(0), sp.Integer(1)))
    return ExactPlanarMetric(identity, identity, sign)


def principal_ideal(owner_metric, outgoing=(3, 1), hidden=1, orientation=None):
    return _huber_density_interpolated_normals(
        owner_metric, ExpressionVector(sp.Integer(1), sp.Integer(0)),
        ExpressionVector(*map(sp.sympify, outgoing)), hidden,
        1 if orientation is None else orientation,
    )


def synthetic_document():
    recorder = Recorder("synthetic")
    call = recorder.wrapper()
    ccw = TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION
    scenarios = (
        ("q5_principal_h1", (3, 1), 1, 5, True, WINDOW_LAW_VORONOI, None),
        ("principal_h2", (1, 2), 2, 4, True, WINDOW_LAW_VORONOI, None),
        ("nested_root_h3", (0, 1), 3, 6, True, WINDOW_LAW_VORONOI, None),
        ("narrow_band", (3, 1), 1, 5, True, WINDOW_LAW_NARROW_BAND, None),
        ("search_cap0", (3, 1), 1, 4, True, WINDOW_LAW_VORONOI, 0),
        ("search_cap1", (3, 1), 1, 4, True, WINDOW_LAW_VORONOI, 1),
        ("unsupported_q", (3, 1), 1, 7, True, WINDOW_LAW_VORONOI, None),
        ("exact_limit_irrational", (0, 1), 2, 6, True, WINDOW_LAW_VORONOI, None),
        ("band_refusal_fallback", (0, 1), 2, 6, True, WINDOW_LAW_NARROW_BAND, 0),
    )
    for name, outgoing, hidden, q, lifted, law, cap in scenarios:
        owner_metric = metric()
        ideal = principal_ideal(owner_metric, outgoing, hidden)
        recorder.test = name
        previous = fan._DENSITY_EXACT_WORK_CAP
        if cap is not None:
            fan._DENSITY_EXACT_WORK_CAP = cap
        try:
            # Отказ уже записан рекордером; синтетика обязана продолжить к следующим исходам.
            with contextlib.suppress(ValueError):
                result = call(owner_metric, ideal, ccw, q, (None,) * hidden,
                              lifted=lifted, law=law, spec_id=name, refusals=[])
                recorder.test = name + ":cache_hit"
                repeated = call(owner_metric, ideal, ccw, q, (None,) * hidden,
                                lifted=lifted, law=law, spec_id=name, refusals=[])
                assert repeated is result
        finally:
            fan._DENSITY_EXACT_WORK_CAP = previous
    return recorder.document()
