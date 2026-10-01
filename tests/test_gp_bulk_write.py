"""Массовая запись штрихов GP: контракт обеих версий API и индексов sidecar.

Blender здесь нет, поэтому frame/drawing — записывающие двойники: они и есть
измерение. Равенство ДАННЫХ с прежней записью «штрих за штрихом» доказано не
здесь, а дампами GP на `building` (Blender 4.5) и смоках (4.3, 4.5):
`artifacts/gp_render_bulk/`.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from cftuv.debug import (
    _GP_V3_RADIUS_PER_PIXEL,
    _GpStrokeBatch,
    _write_gp_strokes,
)


def _point(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


class _Attribute:
    def __init__(self, data_type, domain):
        self.data_type = data_type
        self.domain = domain
        self.written = {}
        self.data = self

    def foreach_set(self, field, values):
        self.written[field] = list(values)


class _Attributes:
    def __init__(self):
        self.items = {"position": _Attribute("FLOAT_VECTOR", "POINT")}

    def get(self, name):
        return self.items.get(name)

    def new(self, name, data_type, domain):
        self.items[name] = _Attribute(data_type, domain)
        return self.items[name]


class _V3Drawing:
    def __init__(self, stroke_count=0):
        self.strokes = [None] * stroke_count
        self.attributes = _Attributes()
        self.add_strokes_calls = []
        self.tagged = 0

    def add_strokes(self, sizes):
        self.add_strokes_calls.append(list(sizes))
        self.strokes = self.strokes + [None] * len(sizes)

    def tag_positions_changed(self):
        self.tagged += 1


class _V2Points:
    def __init__(self):
        self.added = 0
        self.fields = {}

    def add(self, count):
        self.added += count

    def foreach_set(self, field, values):
        self.fields[field] = list(values)


class _V2Stroke:
    def __init__(self):
        self.points = _V2Points()


class _V2Strokes(list):
    def new(self):
        stroke = _V2Stroke()
        self.append(stroke)
        return stroke


SPECS = [
    ([_point(0, 0, 0), _point(1, 0, 0)], 3, 4, False),
    ([_point(0, 1, 0), _point(1, 1, 0), _point(2, 1, 0)], 0, 9, True),
]


def test_v3_frame_takes_all_strokes_in_one_add_strokes_and_array_writes():
    drawing = _V3Drawing()

    _write_gp_strokes(SimpleNamespace(drawing=drawing), SPECS)

    assert drawing.add_strokes_calls == [[2, 3]]
    attributes = drawing.attributes.items
    assert attributes["position"].written["vector"] == [
        0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 0, 2, 1, 0,
    ]
    assert attributes["radius"].written["value"] == [
        max(0.0005, 4 * _GP_V3_RADIUS_PER_PIXEL)
    ] * 2 + [max(0.0005, 9 * _GP_V3_RADIUS_PER_PIXEL)] * 3
    assert attributes["opacity"].written["value"] == [1.0] * 5
    assert attributes["material_index"].written["value"] == [3, 0]
    assert attributes["cyclic"].written["value"] == [False, True]
    assert (
        attributes["material_index"].data_type,
        attributes["material_index"].domain,
    ) == ("INT", "CURVE")
    assert (attributes["cyclic"].data_type, attributes["cyclic"].domain) == (
        "BOOLEAN",
        "CURVE",
    )
    assert drawing.tagged == 1


def test_v3_writes_the_attributes_the_per_stroke_path_created_even_when_trivial():
    """Все пять атрибутов нужны и тогда, когда значения нулевые/ложные.

    Запись «по одному» создавала `material_index`, `cyclic`, `opacity` и
    `radius` сеттером каждого штриха; дамп drawing на `building` сверяет набор
    атрибутов побитово, и пропуск «лишнего» нуля его бы изменил.
    """

    drawing = _V3Drawing()

    _write_gp_strokes(
        SimpleNamespace(drawing=drawing),
        [([_point(0, 0, 0), _point(1, 0, 0)], 0, 1, False)],
    )

    assert set(drawing.attributes.items) == {
        "position",
        "radius",
        "opacity",
        "material_index",
        "cyclic",
    }


def test_v3_refuses_a_drawing_that_already_has_strokes():
    with pytest.raises(RuntimeError, match="empty drawing"):
        _write_gp_strokes(SimpleNamespace(drawing=_V3Drawing(1)), SPECS)


def test_v2_frame_is_written_stroke_by_stroke_with_array_fields():
    strokes = _V2Strokes()

    _write_gp_strokes(SimpleNamespace(strokes=strokes), SPECS)

    assert [stroke.points.added for stroke in strokes] == [2, 3]
    assert strokes[1].points.fields["co"] == [0, 1, 0, 1, 1, 0, 2, 1, 0]
    assert strokes[1].points.fields["strength"] == [1.0] * 3
    assert strokes[1].points.fields["pressure"] == [1.0] * 3
    assert [
        (s.material_index, s.line_width, s.use_cyclic) for s in strokes
    ] == [(3, 4, False), (0, 9, True)]


def test_frame_without_any_stroke_api_is_a_named_failure():
    with pytest.raises(RuntimeError, match="no drawing/strokes API"):
        _write_gp_strokes(SimpleNamespace(), SPECS)


def test_empty_batch_writes_nothing():
    drawing = _V3Drawing()

    _write_gp_strokes(SimpleNamespace(drawing=drawing), [])

    assert drawing.add_strokes_calls == []


def test_batch_hands_out_the_index_the_stroke_will_have_in_its_frame():
    """Индекс в sidecar — число штрихов кадра к моменту добавления.

    Штрих короче двух точек не рисуется и индекса не занимает: `None`, как и
    при записи по одному.
    """

    frame_a = SimpleNamespace(drawing=_V3Drawing())
    frame_b = SimpleNamespace(drawing=_V3Drawing())
    batch = _GpStrokeBatch()
    line = [_point(0, 0, 0), _point(1, 0, 0)]

    assert batch.add(frame_a, line, 0) == 0
    assert batch.add(frame_b, line, 0) == 0
    assert batch.add(frame_a, [_point(0, 0, 0)], 0) is None
    assert batch.add(frame_a, line, 0, cyclic=True) == 1
    assert batch.pending() == 3
    assert frame_a.drawing.add_strokes_calls == []


def test_batch_flush_writes_each_frame_once_and_empties_itself():
    frame_a = SimpleNamespace(drawing=_V3Drawing())
    frame_b = SimpleNamespace(drawing=_V3Drawing())
    batch = _GpStrokeBatch()
    line = [_point(0, 0, 0), _point(1, 0, 0)]
    batch.add(frame_a, line, 0)
    batch.add(frame_b, line, 0)
    batch.add(frame_a, line, 0)

    batch.flush()

    assert frame_a.drawing.add_strokes_calls == [[2, 2]]
    assert frame_b.drawing.add_strokes_calls == [[2]]
    assert batch.pending() == 0
    batch.flush()
    assert frame_a.drawing.add_strokes_calls == [[2, 2]]


def test_batch_index_continues_after_a_frame_that_already_has_strokes():
    frame = SimpleNamespace(strokes=_V2Strokes([_V2Stroke(), _V2Stroke()]))
    batch = _GpStrokeBatch()

    assert batch.add(
        frame, [_point(0, 0, 0), _point(1, 0, 0)], 0
    ) == 2
