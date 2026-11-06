"""HOTSPOT_PANEL_EDITOR_V1: чистые части панели и мини-редактора без Blender - картинка из дерева узлов материала, именованные отказы,
автосоздание и связывание датаблока разметки, пункты выбора метки кнопки."""

from __future__ import annotations

import json

import pytest

from cftuv import hotspot_blender
from cftuv.hotspot_blender import (
    HOTSPOT_IMAGE_HAS_NO_PIXELS,
    HOTSPOT_MATERIAL_HAS_NO_IMAGE,
    HOTSPOT_MATERIAL_MISSING,
    MATERIAL_MARKUP_PROPERTY,
    ensure_material_markup,
    mark_items,
    material_images,
    resolve_material_image,
)
from cftuv.hotspot_markup import HotspotMarkV1, HotspotMarkupV1, load_markup


class _Image:
    def __init__(self, name, width=1024, height=512):
        self.name, self.size = name, (width, height)


class _Link:
    def __init__(self, node, muted=False):
        self.from_node, self.is_muted = node, muted


class _Socket:
    def __init__(self, name, *upstream):
        self.name, self.links = name, [item if isinstance(item, _Link) else _Link(item) for item in upstream]


class _Inputs(list):
    def get(self, name):
        return next((socket for socket in self if socket.name == name), None)


class _Node:
    def __init__(self, kind, image=None, inputs=()):
        self.type, self.image, self.inputs = kind, image, _Inputs(inputs)


class _Tree:
    def __init__(self, *nodes):
        self.nodes = list(nodes)


class _Material(dict):
    def __init__(self, name, *nodes, use_nodes=True):
        super().__init__()
        self.name, self.node_tree, self.use_nodes = name, _Tree(*nodes), use_nodes


class _Text:
    def __init__(self, name):
        self.name, self.body = name, ""

    def as_string(self):
        return self.body

    def from_string(self, value):
        self.body = value


class _Texts(dict):
    def new(self, name):
        self[name] = _Text(name)
        return self[name]


class _Data:
    def __init__(self):
        self.texts = _Texts()


@pytest.fixture
def data(monkeypatch):
    fake = _Data()
    monkeypatch.setattr(hotspot_blender.bpy, "data", fake, raising=False)
    return fake


def _names(images):
    return [image.name for image in images]


def test_the_image_feeding_principled_base_color_comes_first_even_through_a_mix_node():
    roughness, colour, mask = _Image("rough"), _Image("albedo"), _Image("mask")
    loose = _Node("TEX_IMAGE", roughness)
    feeding = _Node("TEX_IMAGE", colour)
    masking = _Node("TEX_IMAGE", mask)
    mix = _Node("MIX", inputs=(_Socket("A", feeding), _Socket("B", masking)))
    principled = _Node("BSDF_PRINCIPLED", inputs=(_Socket("Base Color", mix), _Socket("Roughness", loose)))
    material = _Material("Trim", loose, masking, principled, mix, feeding)
    assert _names(material_images(material)) == ["albedo", "rough", "mask"]
    image, refusal, _detail, candidates = resolve_material_image(material)
    assert image is colour and refusal == "" and len(candidates) == 3


def test_without_principled_the_image_nodes_are_listed_in_node_order_without_repeats_and_muted_links_do_not_feed():
    first, second = _Image("first"), _Image("second")
    muted = _Node("BSDF_PRINCIPLED", inputs=(_Socket("Base Color", _Link(_Node("TEX_IMAGE", second), muted=True)),))
    material = _Material("Plain", _Node("TEX_IMAGE", first), _Node("TEX_IMAGE", None), _Node("TEX_IMAGE", second), _Node("TEX_IMAGE", first), muted)
    assert _names(material_images(material)) == ["first", "second"]


def test_the_choice_wins_then_the_image_of_the_markup_then_the_first():
    a, b, c = _Image("a"), _Image("b"), _Image("c")
    material = _Material("Many", _Node("TEX_IMAGE", a), _Node("TEX_IMAGE", b), _Node("TEX_IMAGE", c))
    assert resolve_material_image(material, "c", "b")[0] is c
    assert resolve_material_image(material, "gone", "b")[0] is b
    assert resolve_material_image(material, "", "gone")[0] is a


@pytest.mark.parametrize(
    "material, refusal",
    [
        (None, HOTSPOT_MATERIAL_MISSING),
        (_Material("Flat", _Node("BSDF_PRINCIPLED", inputs=(_Socket("Base Color"),))), HOTSPOT_MATERIAL_HAS_NO_IMAGE),
        (_Material("NoNodes", _Node("TEX_IMAGE", _Image("x")), use_nodes=False), HOTSPOT_MATERIAL_HAS_NO_IMAGE),
        (_Material("Missing", _Node("TEX_IMAGE", _Image("lost.png", 0, 0))), HOTSPOT_IMAGE_HAS_NO_PIXELS),
    ],
    ids=["no-material", "no-image-node", "nodes-off", "image-without-pixels"],
)
def test_no_usable_image_is_a_named_refusal_never_a_silent_nothing(material, refusal):
    image, name, detail, _candidates = resolve_material_image(material)
    assert image is None and name == refusal and detail


def test_mark_up_creates_the_empty_markup_of_the_image_and_links_it_to_the_material_once(data):
    material = _Material("Trim")
    image = _Image("sheet.png", 2048, 1024)
    name, created, problems = ensure_material_markup(material, image)
    assert (name, created, problems) == ("sheet.png.hotspot", True, ())
    assert material[MATERIAL_MARKUP_PROPERTY] == "sheet.png.hotspot"
    markup, findings = load_markup(data.texts[name].as_string())
    assert findings == () and (markup.width, markup.height, markup.image, markup.marks) == (2048, 1024, "sheet.png", ())
    data.texts[name].from_string(data.texts[name].as_string() + "\n")  # правка руками: повторное нажатие её не трогает
    before = data.texts[name].as_string()
    assert ensure_material_markup(material, image) == (name, False, ())
    assert data.texts[name].as_string() == before, "an existing markup is never overwritten"


def test_mark_up_keeps_the_markup_the_material_already_links_and_reuses_a_text_of_the_same_image(data):
    linked = _Material("Linked")
    linked[MATERIAL_MARKUP_PROPERTY] = "custom.hotspot"
    assert ensure_material_markup(linked, _Image("sheet.png"))[:2] == ("custom.hotspot", True)
    assert json.loads(data.texts["custom.hotspot"].as_string())["texture"]["image"] == "sheet.png"
    other = _Material("Other")
    assert ensure_material_markup(other, _Image("custom"))[:2] == ("custom.hotspot", False)
    assert other[MATERIAL_MARKUP_PROPERTY] == "custom.hotspot"


def test_the_mark_selector_lists_id_name_and_kind_of_the_material_markup():
    markup = HotspotMarkupV1(1024, 512, "sheet.png", (HotspotMarkV1(0, "drips", "trim", 0, 0, 1024, 128, "+y"), HotspotMarkV1(3, "edge", "corner", 0, 128, 1024, 96, "+y", 12)))
    items = mark_items(markup)
    assert [(item[0], item[1], item[3]) for item in items] == [("0", "0: drips (trim)", 0), ("3", "3: edge (corner)", 3)]
    assert mark_items(None)[0][:2] == ("-1", "no marks") and mark_items(HotspotMarkupV1(1024, 512, "", ()))[0][0] == "-1"


def test_each_mark_has_its_own_stable_semi_transparent_fill_and_a_contrasting_band_tint():
    from cftuv.hotspot_markup import mark_colours

    fills = [mark_colours(mark_id, False)[0] for mark_id in range(8)]
    assert len({tuple(round(value, 3) for value in fill[:3]) for fill in fills}) == 8, "neighbouring ids get distinct hues"
    assert mark_colours(3, False) == mark_colours(3, False)
    fill, band, _outline = mark_colours(3, False)
    active_fill, active_band, active_outline = mark_colours(3, True)
    assert 0.2 <= fill[3] <= 0.25 and active_fill[3] > fill[3] and active_band[3] > band[3] and active_outline == (1.0, 1.0, 1.0, 1.0)
    assert sum(band[:3]) < sum(fill[:3]), "the corner band is a darker tint of the same hue"


def test_the_backdrop_list_names_size_and_a_size_other_than_the_markup_s():
    from cftuv.hotspot_blender import backdrop_label

    assert backdrop_label(_Image("albedo", 1024, 512), (1024, 512)) == "albedo (1024 x 512 px)"
    assert backdrop_label(_Image("normal", 2048, 1024), (1024, 512)) == "normal (2048 x 1024 px) - not the markup's 1024 x 512"
    assert backdrop_label(_Image("any", 64, 64)) == "any (64 x 64 px)"


def test_a_backdrop_of_another_size_is_refused_by_name_while_the_markup_has_marks_and_rebases_an_empty_markup():
    """HOTSPOT_BACKDROP_V1: метка - целые пиксели своей текстуры; растянуть её на чужую картинку молча нельзя."""

    from cftuv.hotspot_blender import BACKDROP_REBASE, BACKDROP_SHOW, HOTSPOT_BACKDROP_SIZE_MISMATCH, backdrop_switch

    assert backdrop_switch((1024, 512), (1024, 512), 3) == (BACKDROP_SHOW, "", "")
    action, refusal, detail = backdrop_switch((2048, 1024), (1024, 512), 3)
    assert (action, refusal) == ("", HOTSPOT_BACKDROP_SIZE_MISMATCH) and "1024 x 512" in detail and "2048 x 1024" in detail and "3 mark" in detail
    assert backdrop_switch((2048, 1024), (1024, 512), 0) == (BACKDROP_REBASE, "", "")
    assert backdrop_switch((0, 0), (1024, 512), 0)[1] == HOTSPOT_IMAGE_HAS_NO_PIXELS


def test_mark_up_picks_a_backdrop_of_the_markup_size_before_refusing():
    from cftuv.hotspot_blender import HOTSPOT_BACKDROP_SIZE_MISMATCH, pick_backdrop

    big, small, named = _Image("big", 2048, 1024), _Image("small", 1024, 512), _Image("sheet.png", 1024, 512)
    mark = HotspotMarkV1(0, "drips", "trim", 0, 0, 1024, 128, "+y")
    marked = HotspotMarkupV1(1024, 512, "sheet.png", (mark,))
    assert pick_backdrop((big, small, named), big, None) == (big, "", ""), "no markup yet: the first choice"
    assert pick_backdrop((big, small, named), small, marked)[0] is small, "the user's choice when its size fits"
    assert pick_backdrop((big, small, named), big, marked)[0] is named, "then the markup's own image"
    assert pick_backdrop((big, small), big, marked)[0] is small, "then any image of the markup size"
    image, refusal, detail = pick_backdrop((big,), big, marked)
    assert image is None and refusal == HOTSPOT_BACKDROP_SIZE_MISMATCH and detail
    assert pick_backdrop((big,), big, HotspotMarkupV1(1024, 512, "sheet.png", ())) == (big, "", ""), "an empty markup takes the image's size"


class _Object(dict):
    def __init__(self, name, kind="MESH", decal=True, data=None):
        super().__init__()
        self.name, self.type, self.data = name, kind, data if data is not None else object()
        if decal:
            self["cftuv_source_revision"] = "r"


def test_object_mode_apply_targets_every_selected_decal_by_its_marker_and_counts_the_rest_by_name():
    """HOTSPOT_OBJECT_APPLY_V1: декаль - по маркеру, а не по имени; не декали - по именам; связанные копии одного меша - одна декаль."""

    from cftuv.hotspot_blender import split_decal_targets

    shared = object()
    a, b = _Object("wall.CFTUV_Decal"), _Object("renamed by hand")
    fake, lamp = _Object("floor.CFTUV_Decal", decal=False), _Object("Light", kind="LIGHT", decal=False)
    twin_a, twin_b = _Object("twin", data=shared), _Object("twin.001", data=shared)
    decals, skipped = split_decal_targets((a, fake, b, lamp, twin_a, twin_b), "cftuv_source_revision")
    assert decals == (a, b, twin_a) and skipped == ("floor.CFTUV_Decal", "Light")
    assert split_decal_targets((), "cftuv_source_revision") == ((), ())


def test_the_selection_line_aggregates_the_labels_of_every_selected_decal():
    from cftuv.hotspot_islands import labels_text

    assert labels_text([]) == "" and labels_text(["0: drips (trim)", "0: drips (trim)"]) == "0: drips (trim)"
    assert labels_text(["0: drips (trim)", "2: rust (trim)", "0: drips (trim)"]) == "mixed: 0: drips (trim), 2: rust (trim)"


def test_the_marks_are_drawn_only_in_the_mini_editor_area_unless_the_user_asks_for_them_elsewhere():
    """HOTSPOT_EDITOR_WINDOW_V1: ключ (окно, экран, область) мини-окна; та же картинка в другом Image Editor (UV Editor декали) - не повод рисовать."""

    from cftuv.hotspot_blender import OVERLAY_EDITOR, OVERLAY_ELSEWHERE, overlay_where

    mini = (1, 2, 3)
    assert overlay_where((1, 2, 3), mini, "sheet.png", ("sheet.png",), False) == OVERLAY_EDITOR
    assert overlay_where((1, 2, 3), mini, "other.png", ("sheet.png",), False) == OVERLAY_EDITOR, "the window is keyed, not the image"
    assert overlay_where((1, 2, 4), mini, "sheet.png", ("sheet.png",), False) == "", "a second Image Editor with the same image"
    assert overlay_where((9, 8, 3), mini, "sheet.png", ("sheet.png",), False) == "", "the same area address in another window"
    assert overlay_where((1, 2, 3), None, "sheet.png", ("sheet.png",), False) == "", "the mini editor is closed"
    assert overlay_where((None, None, None), (None, None, None), "sheet.png", ("sheet.png",), False) == ""
    assert overlay_where((1, 2, 4), mini, "sheet.png", ("sheet.png",), True) == OVERLAY_ELSEWHERE, "Show marks in UV Editor"
    assert overlay_where((1, 2, 4), None, "other.png", ("sheet.png", ""), True) == "" and overlay_where((1, 2, 4), None, "", ("",), True) == ""
