"""HOTSPOT_MARKUP_V1: формат разметки, проверка по именам и тонкий слой текстового датаблока."""

from __future__ import annotations

import json
import types

import pytest

from cftuv import hotspot_blender
from cftuv.hotspot_markup import (
    FORMAT_NAME,
    FORMAT_VERSION,
    HotspotMarkV1,
    HotspotMarkupV1,
    MARK_BAND_INVALID,
    MARK_BAND_ON_TRIM,
    MARK_CORNER_BAND_WIDER_THAN_RECT,
    MARK_FIELD_MISSING,
    MARK_FIELD_UNKNOWN,
    MARK_ID_DUPLICATE,
    MARK_ID_INVALID,
    MARK_KIND_UNKNOWN,
    MARK_NAME_INVALID,
    MARK_RECT_EMPTY,
    MARK_RECT_INVALID,
    MARK_RECT_OUT_OF_TEXTURE,
    MARK_RECTS_OVERLAP,
    MARK_REPEAT_WITHOUT_FULL_SPAN,
    MARK_UP_INVALID,
    MARKUP_FIELD_UNKNOWN,
    MARKUP_FORMAT_UNKNOWN,
    MARKUP_MARKS_INVALID,
    MARKUP_NOT_AN_OBJECT,
    MARKUP_NOT_JSON,
    MARKUP_TEXTURE_INVALID,
    MARKUP_VERSION_UNSUPPORTED,
    PROBLEM_NAMES,
    empty_markup,
    load_markup,
    markup_to_dict,
    next_mark_id,
    save_markup,
    validate_markup,
)


def _sheet(*marks, width=1024, height=512):
    return HotspotMarkupV1(width, height, "trim.png", tuple(marks))


TRIM = HotspotMarkV1(0, "drips", "trim", 0, 0, 1024, 128, "+y")
CORNER = HotspotMarkV1(1, "edge", "corner", 0, 128, 1024, 96, "-y", 12)
VERTICAL = HotspotMarkV1(2, "pipe", "trim", 900, 0, 124, 512, "-x")


def _names(problems):
    return [problem.name for problem in problems]


def test_a_valid_sheet_round_trips_through_json_with_its_format_and_version():
    markup = _sheet(CORNER, TRIM)
    text, problems = save_markup(markup)
    assert problems == ()
    data = json.loads(text)
    assert data["format"] == FORMAT_NAME and data["version"] == FORMAT_VERSION
    assert data["texture"] == {"width": 1024, "height": 512, "image": "trim.png"}
    assert [item["id"] for item in data["marks"]] == [0, 1], "marks are written in id order"
    assert data["marks"][0] == {"id": 0, "name": "drips", "kind": "trim", "rect": {"x": 0, "y": 0, "w": 1024, "h": 128}, "up": "+y"}
    assert data["marks"][1]["band"] == 12 and "band" not in data["marks"][0], "only a corner carries its band"
    loaded, problems = load_markup(text)
    assert problems == ()
    assert loaded == markup.with_marks(markup.marks)
    assert save_markup(loaded)[0] == text, "save is canonical: load and save again gives the same text"


def test_a_vertical_trim_spans_the_texture_height_and_its_thickness_is_its_width():
    assert validate_markup(_sheet(VERTICAL)) == ()
    assert VERTICAL.thickness == 124 and VERTICAL.length == 512
    assert TRIM.thickness == 128 and TRIM.length == 1024


@pytest.mark.parametrize(
    "mark, name",
    (
        (HotspotMarkV1(-1, "a", "trim", 0, 0, 1024, 64), MARK_ID_INVALID),
        (HotspotMarkV1(True, "a", "trim", 0, 0, 1024, 64), MARK_ID_INVALID),
        (HotspotMarkV1(0, 7, "trim", 0, 0, 1024, 64), MARK_NAME_INVALID),
        (HotspotMarkV1(0, "a", "cap", 0, 0, 1024, 64), MARK_KIND_UNKNOWN),
        (HotspotMarkV1(0, "a", "trim", 0, 0, 1024, 64, "up"), MARK_UP_INVALID),
        (HotspotMarkV1(0, "a", "trim", 0, 0.5, 1024, 64), MARK_RECT_INVALID),
        (HotspotMarkV1(0, "a", "trim", 0, 0, 1024, 0), MARK_RECT_EMPTY),
        (HotspotMarkV1(0, "a", "trim", 0, 500, 1024, 64), MARK_RECT_OUT_OF_TEXTURE),
        (HotspotMarkV1(0, "a", "trim", -1, 0, 1024, 64), MARK_RECT_OUT_OF_TEXTURE),
        (HotspotMarkV1(0, "a", "trim", 0, 0, 1000, 64), MARK_REPEAT_WITHOUT_FULL_SPAN),
        (HotspotMarkV1(0, "a", "trim", 24, 0, 1000, 64), MARK_REPEAT_WITHOUT_FULL_SPAN),
        (HotspotMarkV1(0, "a", "trim", 0, 0, 64, 500, "+x"), MARK_REPEAT_WITHOUT_FULL_SPAN),
        (HotspotMarkV1(0, "a", "trim", 0, 0, 1024, 64, "+y", 4), MARK_BAND_ON_TRIM),
        (HotspotMarkV1(0, "a", "corner", 0, 0, 1024, 64, "+y", -2), MARK_BAND_INVALID),
        (HotspotMarkV1(0, "a", "corner", 0, 0, 1024, 64, "+y", 65), MARK_CORNER_BAND_WIDER_THAN_RECT),
        (HotspotMarkV1(0, "a", "corner", 0, 0, 40, 512, "-x", 41), MARK_CORNER_BAND_WIDER_THAN_RECT),
    ),
)
def test_every_rejected_mark_is_named(mark, name):
    assert name in _names(validate_markup(_sheet(mark)))
    text, problems = save_markup(_sheet(mark))
    assert text is None and name in _names(problems), "a rejected markup is never written"


def test_duplicate_ids_and_overlapping_rects_are_named_with_the_later_mark():
    other = HotspotMarkV1(0, "copy", "trim", 0, 64, 1024, 128)
    problems = validate_markup(_sheet(TRIM, other))
    assert _names(problems) == [MARK_ID_DUPLICATE, MARK_RECTS_OVERLAP]
    touching = HotspotMarkV1(5, "next", "trim", 0, 128, 1024, 16)
    assert validate_markup(_sheet(TRIM, touching)) == (), "rects that share an edge do not overlap"


def test_a_texture_without_a_positive_size_is_named():
    assert _names(validate_markup(HotspotMarkupV1(0, 512))) == [MARKUP_TEXTURE_INVALID]


@pytest.mark.parametrize(
    "text, name",
    (
        ("{", MARKUP_NOT_JSON),
        ("[]", MARKUP_NOT_AN_OBJECT),
        (json.dumps({"format": "OTHER", "version": 1}), MARKUP_FORMAT_UNKNOWN),
        (json.dumps({"format": FORMAT_NAME, "version": 2}), MARKUP_VERSION_UNSUPPORTED),
        (json.dumps({"format": FORMAT_NAME, "version": 1, "texture": {"width": 4, "height": 4}, "extra": 1}), MARKUP_FIELD_UNKNOWN),
        (json.dumps({"format": FORMAT_NAME, "version": 1, "texture": {"width": 4}}), MARKUP_TEXTURE_INVALID),
        (json.dumps({"format": FORMAT_NAME, "version": 1, "texture": {"width": 4, "height": 4}, "marks": {}}), MARKUP_MARKS_INVALID),
    ),
)
def test_a_text_that_is_not_this_markup_is_refused_by_name(text, name):
    markup, problems = load_markup(text)
    assert markup is None and _names(problems) == [name]


def test_a_mark_with_missing_or_unknown_fields_is_named_and_the_rest_still_loads():
    data = markup_to_dict(_sheet(TRIM, CORNER))
    data["marks"].append({"id": 9, "name": "x"})
    data["marks"][0]["caps"] = True
    markup, problems = load_markup(json.dumps(data))
    assert _names(problems) == [MARK_FIELD_UNKNOWN, MARK_FIELD_MISSING]
    assert [mark.id for mark in markup.marks] == [0, 1], "a broken mark never takes the others with it"


def test_every_problem_name_is_unique_and_upper_case():
    assert len(set(PROBLEM_NAMES)) == len(PROBLEM_NAMES)
    assert all(name == name.upper() for name in PROBLEM_NAMES)


def test_next_id_and_an_empty_sheet():
    assert next_mark_id(empty_markup(256, 256)) == 0
    assert next_mark_id(_sheet(TRIM, VERTICAL)) == 3
    assert save_markup(empty_markup(256, 256))[1] == ()


class _Text:
    def __init__(self, name, body=""):
        self.name, self.body = name, body
        self.writes = 0

    def as_string(self):
        return self.body

    def from_string(self, body):
        self.body = body
        self.writes += 1


class _Texts(dict):
    def new(self, name):
        self[name] = _Text(name)
        return self[name]


class _Material(dict):
    name = "Trim"


@pytest.fixture
def texts(monkeypatch):
    store = _Texts()
    monkeypatch.setattr(hotspot_blender, "bpy", types.SimpleNamespace(data=types.SimpleNamespace(texts=store)))
    return store


def test_the_text_datablock_is_written_only_after_validation_and_never_lost(texts):
    assert hotspot_blender.write_markup_text("Sheet", _sheet(TRIM)) == ()
    first = texts["Sheet"].body
    bad = _sheet(TRIM, HotspotMarkV1(0, "dup", "trim", 0, 200, 1024, 8))
    assert _names(hotspot_blender.write_markup_text("Sheet", bad)) == [MARK_ID_DUPLICATE]
    assert texts["Sheet"].body == first, "a refused save keeps the previous text"
    assert hotspot_blender.write_markup_text("Sheet", _sheet(TRIM)) == () and texts["Sheet"].writes == 1, "an unchanged markup is not rewritten"
    markup, problems = hotspot_blender.read_markup_text("Sheet")
    assert problems == () and markup.marks == (TRIM,)
    assert _names(hotspot_blender.read_markup_text("Gone")[1]) == [hotspot_blender.MARKUP_TEXT_MISSING]


def test_the_material_names_its_markup_through_a_custom_property(texts):
    material = _Material()
    assert hotspot_blender.read_material_markup(material)[2][0].name == hotspot_blender.MARKUP_NOT_LINKED
    hotspot_blender.write_markup_text("Sheet", _sheet(TRIM))
    hotspot_blender.link_markup(material, "Sheet")
    assert material[hotspot_blender.MATERIAL_MARKUP_PROPERTY] == "Sheet"
    name, markup, problems = hotspot_blender.read_material_markup(material)
    assert (name, problems) == ("Sheet", ()) and markup.mark(0) == TRIM
    hotspot_blender.link_markup(material, "")
    assert hotspot_blender.linked_markup_name(material) == ""
