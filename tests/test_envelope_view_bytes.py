"""Вид домена байтами (VIEW_BYTES_V1) и ответ воркера кортежем полей (SLOTS_STATE_TUPLE_V1).

Держится одно: вид, разобранный из байтов, РАВЕН виду, собранному из батча (`build_domain_view`), до бита - `repr` тот же (знак нуля, `int` против `float`,
кортеж против списка, строки дословно), а токен структуры в заголовке - тот, что посчитал бы родитель. Вид, который байты не повторили бы точно, отказан по
имени (`ViewBytesUnsupported`) и едет объектом, как прежде. Результат несёт байты и разбирает вид один раз, при первом чтении; пикл несёт байты.
"""

from __future__ import annotations

import dataclasses
import functools
import io
import math
import pickle
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_view as view_module  # noqa: E402
from cftuv import envelope_view_bytes as codec  # noqa: E402
from cftuv.envelope_domain_pool import DomainTaskResultV1, encode_frame, read_frame  # noqa: E402
from cftuv.envelope_production_export import deferred_result, lean_result, produce_domain, rebased_result  # noqa: E402
from cftuv.envelope_production_view import DomainViewV1, build_domain_view, structure_token  # noqa: E402
from cftuv.envelope_production_weld import DomainVerticesV1  # noqa: E402
from content_fixtures import cold_domain, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))
ALPHA = 0.25
#: Плоские домены и домен-развёртка (у вершины своя нормаль смещения).
CASES = [(0, 0.0), (2, 0.0), (ROW - 1, 1.0)]
TOKEN = "0123456789abcdef0123"


@functools.lru_cache(maxsize=None)
def _eager(patch: int, lifted: float):
    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    deferred, prepared, _revision, _request = cold_domain(source, patch, EDGES, alpha=ALPHA)
    return produce_domain(patch, deferred.domain_id, prepared, str(float(ALPHA)))


def _synthetic() -> DomainViewV1:
    """Вид со всем, что байты обязаны повторить: `-0.0`, субнормальное и предельное числа, `None` и повтор ссылки, не-ASCII, пустые стена и цепь."""

    vertices = DomainVerticesV1(
        7,
        ((0.0, -0.0, 1.5), (5e-324, -1.7976931348623157e308, 0.1), (1.0 / 3.0, 2.0, -3.25), (1e-300, 0.0, -0.0)),
        ((0.0, 0.0, 1.0), (-0.0, 0.0, -1.0), (0.6, 0.8, 0.0), (0.0, 1.0, 0.0)),
        (None, "location:src:12", "шов·α", "location:src:12"),
        ((), (0, 2), (3, 1)),
    )
    return DomainViewV1(
        None,
        vertices,
        ((0, 1, 2), (2, 1, 3, 0)),
        tuple((index / 7.0, -0.0 if index == 3 else float(index)) for index in range(7)),
        (-1, 2**40),
        ((0, 1), (1, 2), (2, 3)),
        (("boundary:WALL:a", ("location:src:12", "k1")), ("пусто", ())),
        "rev:Ω",
    )


def _with_vertices(view: DomainViewV1, **changes) -> DomainViewV1:
    return dataclasses.replace(view, vertices=dataclasses.replace(view.vertices, **changes))


# --------------------------------------------------------------------------
# Кодек
# --------------------------------------------------------------------------


def test_the_bytes_repeat_the_view_to_the_bit_and_carry_the_structure_token():
    view = _synthetic()
    decoded = codec.decode_view(codec.encode_view(view, TOKEN))

    assert decoded == view and repr(decoded) == repr(view)
    assert decoded.token == TOKEN and structure_token(decoded) == TOKEN
    assert math.copysign(1.0, decoded.vertices.positions[0][1]) == -1.0 and math.copysign(1.0, decoded.uvs[3][1]) == -1.0
    assert decoded.vertices.positions[1][0] == 5e-324 and decoded.owners == (-1, 2**40)
    assert decoded.vertices.refs == (None, "location:src:12", "шов·α", "location:src:12")
    assert decoded.vertices.walls[0] == () and decoded.boundary_chains[1] == ("пусто", ())


def test_a_refused_view_travels_as_its_outcome_detail_and_revision():
    view = DomainViewV1(("ADAPTER_NON_FINITE_BATCH", "vertex k:7 - ∞"), None, (), (), (), (), (), "rev:1")
    decoded = codec.decode_view(codec.encode_view(view, ""))

    assert decoded == view and repr(decoded) == repr(view) and decoded.token == ""


@pytest.mark.parametrize(
    "spoil",
    [
        pytest.param(lambda view: _with_vertices(view, positions=((0, -0.0, 1.5),) + view.vertices.positions[1:]), id="int-coordinate"),
        pytest.param(lambda view: _with_vertices(view, refs=(None, 12, "a", "b")), id="ref-not-str"),
        pytest.param(lambda view: _with_vertices(view, refs=(None, "\ud800", "a", "b")), id="ref-not-utf8"),
        pytest.param(lambda view: _with_vertices(view, normals=view.vertices.normals[:-1]), id="rows-of-other-length"),
        pytest.param(lambda view: dataclasses.replace(view, faces=[list(face) for face in view.faces]), id="faces-a-list"),
        pytest.param(lambda view: dataclasses.replace(view, faces=((-1, 1, 2), (2, 1, 3, 0))), id="negative-index"),
        pytest.param(lambda view: dataclasses.replace(view, owners=(-1, 2**63)), id="owner-beyond-int64"),
        pytest.param(lambda view: dataclasses.replace(view, owners=(-1.0, 2.0)), id="owner-not-int"),
        pytest.param(lambda view: dataclasses.replace(view, seams=((0, 1, 2),)), id="seam-not-a-pair"),
        pytest.param(lambda view: dataclasses.replace(view, boundary_chains=(("a", ["k"]),)), id="chain-keys-a-list"),
        pytest.param(lambda view: dataclasses.replace(view, failure=("ADAPTER_X",)), id="failure-not-a-pair"),
    ],
)
def test_a_view_the_bytes_would_not_repeat_exactly_is_refused_by_name(spoil):
    with pytest.raises(codec.ViewBytesUnsupported):
        codec.encode_view(spoil(_synthetic()), TOKEN)


def test_a_token_of_another_length_and_bytes_of_another_version_are_refused_by_name():
    with pytest.raises(codec.ViewBytesUnsupported):
        codec.encode_view(_synthetic(), "short")
    data = bytearray(codec.encode_view(_synthetic(), TOKEN))
    data[4] ^= 0xFF  # версия
    with pytest.raises(codec.ViewBytesUnsupported):
        codec.decode_view(bytes(data))


# --------------------------------------------------------------------------
# Результат домена
# --------------------------------------------------------------------------


@pytest.mark.parametrize("patch, lifted", CASES)
def test_the_view_from_the_bytes_of_a_real_domain_is_the_view_built_from_its_batch(patch, lifted):
    eager = _eager(patch, lifted)
    built = build_domain_view(eager)
    deferred = deferred_result(eager)

    assert "view_bytes" in deferred.__dict__ and "view" not in deferred.__dict__  # вид едет байтами, а не объектом
    assert deferred.view == built and repr(deferred.view) == repr(built)
    assert deferred.view.token == structure_token(built)  # токен заголовка - тот, что посчитал бы родитель


def test_the_view_is_decoded_once_at_the_first_read_and_the_pickle_carries_only_the_bytes(monkeypatch):
    calls = []
    decode = codec.decode_view
    monkeypatch.setattr(codec, "decode_view", lambda data: calls.append(len(data)) or decode(data))
    copy = pickle.loads(pickle.dumps(deferred_result(_eager(2, 0.0))))

    assert "view" not in copy.__dict__ and not calls
    first = copy.view
    assert copy.view is first and len(calls) == 1  # второе чтение не разбирает
    again = pickle.loads(pickle.dumps(copy))
    assert "view" not in again.__dict__ and again.view == first and len(calls) == 2  # пикл несёт байты, а не разобранный вид


def test_the_bytes_follow_the_view_through_copies_and_never_outlive_it():
    eager = _eager(2, 0.0)
    deferred = deferred_result(eager)

    kept = deferred.with_changes(placement="cache")
    assert kept.__dict__["view_bytes"] is deferred.__dict__["view_bytes"] and kept.view == deferred.view
    moved = deferred.with_changes(patch_id=99)
    assert "view_bytes" not in moved.__dict__ and moved.view is None  # чужой вид снят вместе с байтами
    plain = deferred.materialized()
    assert "view_bytes" not in plain.__dict__ and plain.view is None
    lean = lean_result(eager)
    assert "view_bytes" in lean.__dict__ and lean.view == build_domain_view(eager)
    shifted = rebased_result(lean.with_changes(normals_alpha=ALPHA), view=lean.view, certificate=lean.parametric)
    assert "view_bytes" not in shifted.__dict__ and shifted.view is lean.view  # сдвинутая копия несёт разобранный вид своей ширины


def test_a_view_the_bytes_cannot_carry_travels_as_an_object(monkeypatch):
    eager = _eager(2, 0.0)
    built = build_domain_view(eager)
    odd = dataclasses.replace(built, owners=tuple(float(item) for item in built.owners))
    monkeypatch.setattr(view_module, "build_domain_view", lambda result: odd)
    monkeypatch.setenv("CFTUV_MESH_ASSEMBLY_BACKEND", "PYTHON")  # путь питона (VIEW_BYTES_NATIVE_V1: порт читает батч сам)

    deferred = deferred_result(eager)
    assert "view_bytes" not in deferred.__dict__ and deferred.view is odd


def test_a_worker_reply_frame_carries_the_view_bytes_and_the_parent_reads_the_same_view():
    eager = _eager(ROW - 1, 1.0)
    reply = read_frame(io.BytesIO(encode_frame(DomainTaskResultV1(3, production=deferred_result(eager)))))

    assert reply.task_id == 3 and "view" not in reply.production.__dict__
    assert reply.production.view == build_domain_view(eager) and reply.production == eager


def test_the_reply_pickles_as_the_tuple_of_its_field_values():
    reply = DomainTaskResultV1(5, export_timings=(("EXPORT", 1.5),), needs_blob=True, prepared_key="k", stored=(("k", 3),))
    state = reply.__getstate__()

    assert type(state) is tuple and state == tuple(getattr(reply, item.name) for item in dataclasses.fields(reply))
    assert pickle.loads(pickle.dumps(reply, protocol=5)) == reply
