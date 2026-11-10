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


def test_the_first_markup_is_one_valid_whole_square_trim_and_only_it_rebases():
    from cftuv.hotspot_markup import covers_whole_square, first_markup

    markup = first_markup(2048, 1024, "sheet.png")
    assert save_markup(markup)[1] == () and covers_whole_square(markup) and next_mark_id(markup) == 1
    assert not covers_whole_square(empty_markup(256, 256)) and not covers_whole_square(_sheet(TRIM, VERTICAL))


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


# --------------------------------------------------------------------------
# HOTSPOT_EDITOR_UX_V1: поворот, разворот стрелки, край полосы, сетка
# --------------------------------------------------------------------------


def _trim(mark_id, y, h, width=1024, up="+y", kind="trim", band=0):
    return HotspotMarkV1(mark_id, f"m{mark_id}", kind, 0, y, width, h, up, band)


def test_a_quarter_turn_is_clockwise_in_texture_pixels_and_the_arrow_turns_with_the_strip():
    """Пиксели текстуры, начало - левый нижний угол, y вверх: по часовой (x, y) -> (y, -x), то есть +y -> +x -> -y -> -x -> +y."""

    from cftuv.hotspot_markup import UP_CLOCKWISE, UP_VECTORS, rotated_mark, validate_markup

    for up, turned in UP_CLOCKWISE.items():
        (x, y), (tx, ty) = UP_VECTORS[up], UP_VECTORS[turned]
        assert (tx, ty) == (y, -x), (up, turned)
    markup = HotspotMarkupV1(1024, 1024, "sheet.png", (_trim(0, 64, 128),))
    seen = []
    for _turn in range(4):
        mark, problem = rotated_mark(markup, 0, 8)
        assert problem is None and validate_markup(markup.with_marks([mark])) == (), (mark, problem)
        seen.append(mark.up)
        markup = markup.with_marks([mark])
    assert seen == ["+x", "-y", "-x", "+y"]


def test_a_horizontal_full_span_strip_turns_into_a_vertical_full_height_strip_of_the_same_thickness_on_a_non_square_texture():
    from cftuv.hotspot_markup import rotated_mark

    markup = HotspotMarkupV1(1024, 512, "sheet.png", (_trim(3, 64, 128, kind="corner", band=12),))
    mark, problem = rotated_mark(markup, 0, 8)
    assert problem is None
    assert (mark.x, mark.y, mark.w, mark.h, mark.up, mark.thickness, mark.band) == (448, 0, 128, 512, "+x", 128, 12), "centre x 512 kept"
    back, _problem = rotated_mark(markup.with_marks([mark]), 0, 8)
    assert (back.x, back.y, back.w, back.h, back.up) == (0, 192, 1024, 128, "-y"), "the centre of the vertical strip along y is the texture middle"
    coarse, _problem = rotated_mark(HotspotMarkupV1(1024, 512, "", (_trim(0, 0, 100),)), 0, 64)
    assert coarse.x == 448, "the start snaps to the grid: (512 - 50) / 64 -> 7 * 64"


def test_a_turn_keeps_the_centre_but_is_clamped_into_the_texture():
    from cftuv.hotspot_markup import rotated_mark

    draft = HotspotMarkupV1(1024, 512, "", (HotspotMarkV1(0, "draft", "trim", 1000, 0, 24, 32, "+y"),))
    mark, problem = rotated_mark(draft, 0, 1)
    assert problem is None and (mark.x, mark.y, mark.w, mark.h, mark.up) == (992, 0, 32, 512, "+x"), "centre 1012 - 16 = 996 is clamped to 1024 - 32"
    low = HotspotMarkupV1(1024, 512, "", (HotspotMarkV1(0, "low", "trim", 0, 0, 8, 40, "+y"),))
    assert rotated_mark(low, 0, 16)[0].x == 0, "a start below 0 is clamped to 0"


def test_a_turn_onto_another_mark_or_out_of_the_texture_is_refused_by_name_and_changes_nothing():
    from cftuv.hotspot_markup import MARK_ROTATE_BLOCKED, rotated_mark

    markup = HotspotMarkupV1(1024, 512, "", (_trim(0, 0, 128), _trim(5, 256, 64)))
    mark, problem = rotated_mark(markup, 0, 8)
    assert mark is None and problem.name == MARK_ROTATE_BLOCKED and problem.mark_id == 0 and "overlaps mark(s) 5" in problem.detail
    thick = HotspotMarkupV1(1024, 512, "", (HotspotMarkV1(0, "wide", "trim", 0, 0, 600, 512, "+x"),))
    mark, problem = rotated_mark(thick, 0, 8)
    assert mark is None and problem.name == MARK_ROTATE_BLOCKED and "600 px thick across a 512 px" in problem.detail
    assert markup.marks[0] == _trim(0, 0, 128), "the pure turn never edits its input"


def test_flipping_points_the_arrow_the_other_way_and_leaves_the_rectangle():
    from cftuv.hotspot_markup import flipped_mark

    for up, other in (("+y", "-y"), ("-y", "+y"), ("+x", "-x"), ("-x", "+x")):
        mark = HotspotMarkV1(1, "m", "corner", 0, 128, 1024, 96, up, 12)
        flipped = flipped_mark(mark)
        assert flipped.up == other and (flipped.x, flipped.y, flipped.w, flipped.h, flipped.band) == (0, 128, 1024, 96, 12)


def test_dragging_a_band_edge_keeps_the_band_symmetric_on_the_grid_and_thinner_than_the_mark():
    from cftuv.hotspot_markup import band_from_edge

    corner = HotspotMarkV1(1, "edge", "corner", 0, 128, 1024, 96, "+y", 12)  # средняя линия y = 176
    assert band_from_edge(corner, 196, 8) == 40 and band_from_edge(corner, 156, 8) == 40, "either edge, the same symmetric band"
    assert band_from_edge(corner, 199, 8) == 48, "2 * 23 = 46 rounds to the grid"
    assert band_from_edge(corner, 176, 8) == 0 and band_from_edge(corner, 900, 8) == 88 and band_from_edge(corner, -50, 1) == 95, "0 .. < 96"
    vertical = HotspotMarkV1(2, "post", "corner", 400, 0, 64, 512, "-x", 0)  # средняя линия x = 432
    assert band_from_edge(vertical, 442, 4) == 20 and band_from_edge(vertical, 10, 4) == 60
    assert band_from_edge(HotspotMarkV1(3, "thin", "corner", 0, 0, 1024, 8, "+y"), 100, 16) == 0, "a grid wider than the mark leaves no band"


def test_ctrl_plus_and_ctrl_minus_halve_and_double_the_grid_on_both_keyboards():
    from cftuv.hotspot_markup import grid_key_step, grid_step

    assert [grid_key_step(key, True) for key in ("EQUAL", "NUMPAD_PLUS", "MINUS", "NUMPAD_MINUS")] == [-1, -1, 1, 1]
    assert [grid_key_step(key, False) for key in ("EQUAL", "NUMPAD_PLUS", "MINUS", "NUMPAD_MINUS")] == [0, 0, 0, 0], "without Ctrl: the Image Editor's"
    assert grid_key_step("LEFT_BRACKET", False) == -1 and grid_key_step("RIGHT_BRACKET", True) == 1 and grid_key_step("A", True) == 0
    assert (grid_step(8, -1), grid_step(8, 1), grid_step(1, -1), grid_step(1024, 1), grid_step(8, 0)) == (4, 16, 1, 1024, 8)


def test_the_header_names_the_findings_once_each():
    from cftuv.hotspot_markup import problem_names

    text = "MARK_RECTS_OVERLAP (mark 1): mark 1 overlaps mark 0; MARK_RECTS_OVERLAP (mark 1): again; MARK_RECT_EMPTY (mark 2): rect 0 x 0 has no area"
    assert problem_names(text) == ("MARK_RECTS_OVERLAP (mark 1)", "MARK_RECT_EMPTY (mark 2)") and problem_names("") == ()

def test_mark_ids_are_never_reused_and_a_markup_without_the_counter_takes_the_floor_of_the_decals():
    """HOTSPOT_MARK_ID_MONOTONIC_V1: удалили метку с наибольшим номером - новая метка получает следующий номер, а не её."""

    from dataclasses import replace

    from cftuv.hotspot_markup import MARKUP_NEXT_ID_INVALID, WHOLE_TEXTURE_MARK_ID

    legacy = _sheet(TRIM, VERTICAL).with_marks([TRIM])  # метка 2 удалена, счётчика нет (разметка до него)
    assert next_mark_id(legacy) == 1, "without the counter only the marks are known"
    assert next_mark_id(legacy, floor=3) == 3, "the decals of the file still name mark 2: the floor keeps it from coming back"
    counted = replace(legacy, next_id=3)
    assert next_mark_id(counted) == 3 and next_mark_id(counted, floor=1) == 3
    text, problems = save_markup(counted)
    assert problems == () and json.loads(text)["next_id"] == 3
    loaded, problems = load_markup(text)
    assert problems == () and loaded.next_id == 3 and next_mark_id(loaded) == 3
    old_text, _problems = save_markup(legacy)
    assert "next_id" not in json.loads(old_text), "a markup that never had the counter is written as before"
    assert load_markup(old_text) == (legacy, ()), "the old JSON reads unchanged"
    data = json.loads(text)
    for bad in (-1, "3", 2.5, True):
        data["next_id"] = bad
        assert MARKUP_NEXT_ID_INVALID in _names(load_markup(json.dumps(data))[1]), bad
    reserved = HotspotMarkV1(WHOLE_TEXTURE_MARK_ID, "x", "trim", 0, 400, 1024, 64, "+y")
    assert MARK_ID_INVALID in _names(validate_markup(_sheet(TRIM, reserved))), "the whole-texture id is not a mark id of a markup"
    assert next_mark_id(_sheet(TRIM)) < WHOLE_TEXTURE_MARK_ID
