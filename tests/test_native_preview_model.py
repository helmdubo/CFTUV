"""PREVIEW_MODEL_NATIVE_V1: модель превью ширины (`build_model`) портом против эталона питона (`python_model`).

Держится одно: при заказе `NATIVE` модель РАВНА модели эталона до бита - наклоны, окна вершин и доменов, карантин, тень, приближённость (байты и `dtype`), строки исходов
(исход, степень, досягаемость - те же числа), раскладочные записи образца (`_EntriesV1`). Эталон сверяется по отпечатку исходника: сдвинулся эталон - порт надо догнать.
"""

from __future__ import annotations

import hashlib
import inspect
import math
import random
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "kernel" / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "kernel" / "src"))

from cftuv import envelope_mesh_assembly as assembly  # noqa: E402
from cftuv import envelope_width_preview_model as pm  # noqa: E402

try:
    import cftuv_native
except ModuleNotFoundError as error:
    if error.name != "cftuv_native":
        raise
    cftuv_native = None

needs_native = pytest.mark.skipif(
    cftuv_native is None or not hasattr(cftuv_native, "preview_model_build"), reason="расширение cftuv_native не собрано (или собрано без preview_model_build)"
)

#: Отпечаток эталона, с которого снят порт (`preview_model.rs`): исходники функций, которые он повторяет. Другой - эталон сдвинулся без порта.
ORACLE = (
    "python_model", "_domain_reason", "_partner_state", "_normal_sources", "_display_reach", "_trusted_reach", "_weld_conflicts", "_entries_of", "_scatter_last", "_vertex_windows",
)
ORACLE_DIGEST = "0c43c96b9d0ac9076d32fdec0a995b78030937fe698327dda2a6e3d5596ab005"


def _oracle_digest() -> str:
    digest = hashlib.sha256()
    for name in ORACLE:
        digest.update(inspect.getsource(getattr(pm, name)).replace("\r\n", "\n").encode("utf-8"))
    return digest.hexdigest()


def test_the_oracle_of_the_port_did_not_move():
    assert _oracle_digest() == ORACLE_DIGEST, "python_model (or a function it calls) changed: port the change to native/cftuv-python/src/preview_model.rs, then update ORACLE_DIGEST"


def _bits(value):
    return None if value is None else (value.dtype.str, value.shape, value.tobytes())


def _float_bits(value):
    return value if value is None else struct.pack("<d", value)


def _same_model(mine, theirs) -> None:
    assert type(mine) is type(theirs)
    if isinstance(theirs, pm.PreviewRefusalV1):
        assert mine == theirs
        return
    for name in ("a1", "e1", "vlow", "vhigh", "loop_counts", "dlow", "dhigh", "quarantined", "vertex_quarantined", "shadow_low", "shadow_high", "approximate"):
        assert _bits(getattr(mine, name)) == _bits(getattr(theirs, name)), name
    assert len(mine.rows) == len(theirs.rows)
    for one, other in zip(mine.rows, theirs.rows):
        assert one[:4] == other[:4] and _float_bits(one[4]) == _float_bits(other[4]), (one, other)


def _random_sample(rng: random.Random, *, alpha: float):
    """Образец из случайных доменов: общие вершины (сварка), соседи по стене, причины, интервалы, развёртки, наклоны с редким расхождением на сварке."""

    vertex_count = rng.randint(4, 40)
    count = rng.randint(1, 12)
    slope_of_vertex = np.array([[rng.uniform(-1, 1) for _ in range(3)] for _ in range(vertex_count)])
    domains = []
    loop_start = 0
    for number in range(count):
        size = rng.randint(1, min(8, vertex_count))
        vertices = np.array(sorted(rng.sample(range(vertex_count), size)), dtype=np.int32)
        loops = rng.randint(0, 6)
        kind = rng.random()
        markers = ()
        slopes = slope_of_vertex[vertices].copy()
        if rng.random() < 0.15:
            slopes[rng.randrange(size)] += rng.choice((1e-12, 1e-6, math.nan))
        uv_slopes = np.array([[rng.uniform(-1, 1), rng.uniform(-1, 1)] for _ in range(loops)]).reshape(-1, 2)
        if kind < 0.1:
            markers = (pm.DOMAIN_NO_CERTIFICATE,)
        elif kind < 0.15:
            markers = ("PARAM_SOMETHING_ELSE",)
        has = not markers
        approximate = (pm.DEVELOPABLE_MARKER,) if has and rng.random() < 0.3 else ()
        low = rng.choice((alpha * 0.5, alpha * 0.9, float("nan")))
        high = rng.choice((None, alpha * 1.1, alpha * 2.0, alpha))
        domains.append(
            pm.DomainSampleV1(
                patch_id=10 + number,
                domain_id=f"d{number}",
                token="" if rng.random() < 0.05 else "tok",
                structure="s",
                status=rng.choice((pm.INTERVAL_CERTIFIED,) * 6 + ("AT_EVENT",)),
                low=low,
                high=high,
                vertices=vertices,
                loop_start=loop_start,
                loop_count=loops,
                interval=None if rng.random() < 0.05 else object(),
                slopes=slopes if has else None,
                uv_slopes=uv_slopes if has else None,
                markers=markers,
                approximate=approximate,
                approximate_rate=rng.choice((0.0, 0.5, 3.0, float("nan"))) if approximate else 0.0,
                partners=(),
                normal_alpha=rng.choice((float("nan"), alpha, alpha * 0.97)) if approximate else float("nan"),
            )
        )
        loop_start += loops
    rows = []
    for domain in domains:
        partners = tuple(sorted(rng.sample(range(count), rng.randint(0, min(2, count)))))
        rows.append(pm.relocated_domain(domain, domain.vertices, domain.loop_start, partners))
    sample = pm.PreviewSampleV1(
        key=("k",), alpha_text=repr(alpha), alpha=alpha, positions=np.zeros((vertex_count, 3)), uvs=np.array([[rng.random(), rng.random()] for _ in range(loop_start)]).reshape(-1, 2),
        domains=tuple(rows), arrays_digest="", source_revision="r", offset=rng.choice((0.0, 0.002, 0.01)),
    )
    ledger = None
    if rng.random() < 0.5:
        entries = {}
        for domain in rows:
            if rng.random() < 0.3:
                state = rng.choice((pm.TRUST_QUARANTINED, pm.TRUST_ABANDONED, pm.TRUST_REDUCED))
                entries[(domain.patch_id, domain.domain_id)] = pm.DomainTrustV1(state, 1, 0.5, rng.choice((0.02, math.inf)))
        ledger = pm.TrustLedgerV1(sample.key if rng.random() < 0.8 else ("other",), entries)
    return sample, ledger


def _both(sample, ledger, monkeypatch):
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, assembly.MESH_ASSEMBLY_NATIVE)
    native = pm._native_model(sample, ledger)
    native_entries = sample.cache.pop("entries", None)
    reference = pm.python_model(sample, ledger)
    return native, native_entries, reference


@needs_native
def test_the_native_model_is_the_python_model_bit_for_bit_on_random_samples(monkeypatch):
    rng = random.Random(20261101)
    compared = refused = 0
    for _ in range(600):
        sample, ledger = _random_sample(rng, alpha=rng.choice((0.25, 0.031, 1.5)))
        native, native_entries, reference = _both(sample, ledger, monkeypatch)
        assert native is not None, assembly.FALLBACKS
        _same_model(native, reference)
        python_entries = sample.cache.get("entries")
        if python_entries is not None:
            for name in ("domain", "vertex", "bounds", "total", "order", "starts", "groups"):
                assert _bits(getattr(native_entries, name)) == _bits(getattr(python_entries, name)), name
        compared += 1
        refused += isinstance(reference, pm.PreviewRefusalV1)
    assert compared == 600 and 0 < refused < compared


@needs_native
@pytest.mark.parametrize("spread", (pm.WELD_SLOPE_TOLERANCE, math.nextafter(pm.WELD_SLOPE_TOLERANCE, 1.0)))
def test_the_weld_tolerance_is_strict_in_both(spread, monkeypatch):
    """Расхождение наклонов сваренной вершины, равное допуску, сварку держит (`>` а не `>=`), на ulp больше - нет; порт и эталон решают одинаково."""

    def domain(number, slope):
        return pm.DomainSampleV1(
            patch_id=number, domain_id=f"d{number}", token="t", structure="s", status=pm.INTERVAL_CERTIFIED, low=0.1, high=0.4, vertices=np.array([0], dtype=np.int32),
            loop_start=0, loop_count=0, interval=object(), slopes=np.array([[0.0, slope, 0.0]]), uv_slopes=np.zeros((0, 2)),
        )

    sample = pm.PreviewSampleV1(key=("w",), alpha_text="0.25", alpha=0.25, positions=np.zeros((1, 3)), uvs=np.zeros((0, 2)), domains=(domain(1, 0.0), domain(2, spread)), arrays_digest="", source_revision="r")
    native, _entries, reference = _both(sample, None, monkeypatch)
    _same_model(native, reference)
    held = spread > pm.WELD_SLOPE_TOLERANCE
    assert isinstance(reference, pm.PreviewRefusalV1) == held


@needs_native
def test_a_row_the_port_would_not_repeat_is_declined_and_the_python_model_answers(monkeypatch):
    rng = random.Random(7)
    sample, ledger = _random_sample(rng, alpha=0.25)
    broken = pm.relocated_domain(sample.domains[0], np.array([sample.vertex_count + 5], dtype=np.int32), sample.domains[0].loop_start, ())
    sample = pm.PreviewSampleV1(**{name: getattr(sample, name) for name in pm.PreviewSampleV1.__slots__ if name != "cache"} | {"domains": (broken, *sample.domains[1:])})
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, assembly.MESH_ASSEMBLY_NATIVE)
    assert pm._native_model(sample, ledger) is None
    assert any(key[1] == pm.PREVIEW_MODEL_OPERATION for key in assembly.FALLBACKS)


def test_the_python_order_answers_without_the_port(monkeypatch):
    rng = random.Random(3)
    sample, ledger = _random_sample(rng, alpha=0.25)
    monkeypatch.setenv(assembly.MESH_ASSEMBLY_ENVIRONMENT, assembly.MESH_ASSEMBLY_PYTHON)
    assert pm._native_model(sample, ledger) is None
    _same_model(pm.build_model(sample, ledger), pm.python_model(sample, ledger))
