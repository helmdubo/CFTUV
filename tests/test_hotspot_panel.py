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
