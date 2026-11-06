"""HOTSPOT_ATLAS_MAPPING_V1 в писателе меша: без записи отображения UV меша не трогаются (режим выключен - меш тот же побитово), с записью -
UV ядра в меше заменяются UV атласа одной функцией, которую зовут все писатели UV (кнопка, подтверждение, кадры живой ширины)."""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np

from cftuv import envelope_production_mesh, hotspot_blender
from cftuv.hotspot_islands import MARK_ATTRIBUTE, encode, record_from_json, record_of, record_to_json
from cftuv.hotspot_markup import HotspotMarkV1
from cftuv.hotspot_mapping import MESH_PROPERTY, HotspotMappingV1, atlas_uvs, mapping_to_json

TRIM = HotspotMarkV1(0, "drips", "trim", 0, 0, 1024, 64, "+y")

ROOT = Path(__file__).resolve().parents[1]


class _Data:
    def __init__(self, values, width):
        self.values, self.width = np.asarray(values), width

    def __len__(self):
        return self.values.shape[0] // self.width

    def foreach_get(self, _name, buffer):
        buffer[:] = self.values.reshape(-1)

    def foreach_set(self, _name, buffer):
        self.values = np.asarray(buffer, dtype=self.values.dtype).copy()


class _Attribute:
    def __init__(self, values, data_type, domain, width=1):
        self.data, self.data_type, self.domain = _Data(values, width), data_type, domain


class _Attributes(dict):
    """`mesh.attributes`: `new` заводит нулевой атрибут нужного размера (углы или грани), `remove` снимает."""

    def __init__(self, mesh):
        super().__init__()
        self.mesh = mesh

    def new(self, name, type, domain):  # noqa: A002 - имя аргумента Blender
        width = {"FLOAT_VECTOR": 3, "FLOAT2": 2}.get(type, 1)
        count = len(self.mesh.loops) if domain == "CORNER" else len(self.mesh.polygons)
        self[name] = _Attribute(np.zeros(count * width, dtype=np.int32 if type == "INT" else np.float32), type, domain, width)
        return self[name]

    def remove(self, attribute):
        del self[next(key for key, value in self.items() if value is attribute)]


class _Layer:
    def __init__(self, attribute):
        self.data = attribute.data


class _Mesh(dict):
    """Меш Blender в объёме писателя: свойства ID, углы, грани, позиции, атрибуты и слой UV `UVMap` (атрибут FLOAT2 углов)."""

    name = "Decal.CFTUV_Decal"
    session_uid = 7

    def __init__(self, uvs, corner_vertices, totals, domain, owner):
        super().__init__()
        self.attributes = _Attributes(self)
        self.loops = list(corner_vertices)
        self.polygons = _Data(np.asarray(totals, dtype=np.int32), 1)
        self.attributes["UVMap"] = _Attribute(np.asarray(uvs, dtype=np.float32).reshape(-1), "FLOAT2", "CORNER", 2)
        self.attributes[".corner_vert"] = _Attribute(np.asarray(corner_vertices, dtype=np.int32), "INT", "CORNER")
        self.attributes["cftuv_domain"] = _Attribute(np.asarray(domain, dtype=np.int32), "INT", "FACE")
        self.attributes["cftuv_owner"] = _Attribute(np.asarray(owner, dtype=np.int32), "INT", "FACE")
        positions = np.zeros((32, 3), dtype=np.float32)
        positions[:, 0] = np.arange(32) % 10 * 0.5
        positions[:, 1] = np.arange(32) // 10
        self.attributes["position"] = _Attribute(positions.reshape(-1), "FLOAT_VECTOR", "POINT", 3)
        self.uv_layers = {"UVMap": _Layer(self.attributes["UVMap"])}

    def uvs(self):
        return self.attributes["UVMap"].data.values.reshape(-1, 2).copy()

    def values(self, name):
        attribute = self.attributes[name]
        return attribute.data.values.reshape(-1, attribute.data.width).copy()


def _fold():
    """Две полосы по разные стороны шва из трёх вершин (0, 1, 2): домен 3 и домен 5, станции навстречу."""

    uvs = [(0.0, 0.0), (1.5, 0.0), (1.5, 1.0), (0.0, 1.0), (1.5, 0.0), (2.5, 0.0), (2.5, 1.0), (1.5, 1.0),
           (0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, 0.0), (2.5, 0.0), (2.5, 1.0), (1.0, 1.0)]
    corners = [0, 1, 11, 10, 1, 2, 12, 11, 2, 1, 21, 22, 1, 0, 20, 21]
    return _Mesh(uvs, corners, [4, 4, 4, 4], [3, 3, 5, 5], [0, 0, 1, 1])


def _corner():
    return HotspotMappingV1("Sheet", HotspotMarkV1(1, "edge", "corner", 0, 128, 1024, 96, "+y", 12), 1024, 512)


def test_without_the_mapping_record_the_writer_leaves_the_uvs_alone_and_never_reaches_the_hotspot_layer(monkeypatch):
    mesh = _fold()
    before = mesh.uvs()
    names = set(mesh.attributes)
    monkeypatch.setattr(hotspot_blender, "map_mesh_uvs", lambda *_args: (_ for _ in ()).throw(AssertionError("called")))
    monkeypatch.setattr(hotspot_blender, "remember_mesh_islands", lambda *_args: (_ for _ in ()).throw(AssertionError("called")))
    envelope_production_mesh._map_hotspot(mesh)
    envelope_production_mesh._remember_hotspot(mesh)
    assert envelope_production_mesh._carry_hotspot(None, None) is None
    assert np.array_equal(mesh.uvs(), before) and MESH_PROPERTY not in mesh and set(mesh.attributes) == names, "Hotspot off: nothing new on the mesh"


def test_with_the_record_the_mesh_uvs_become_the_atlas_uvs_of_the_pure_mapping():
    mesh = _fold()
    kernel = mesh.uvs()
    mesh[MESH_PROPERTY] = mapping_to_json(_corner())  # запись v1 (меш прежней версии) читается
    envelope_production_mesh._map_hotspot(mesh)
    group = np.repeat(np.array([3, 3, 5, 5], dtype=np.int64) * (1 << 20) + np.array([0, 0, 1, 1]), 4)
    expected, counters = atlas_uvs(kernel.astype(np.float64), np.array(mesh.loops), group, _corner())
    assert np.array_equal(mesh.uvs(), expected.astype(np.float32))
    assert counters["HOTSPOT_CORNER_SIDE_PAIRS"] == 1
    last = hotspot_blender.LAST_COUNTERS[mesh.session_uid]
    assert {key: last[key] for key in counters} == counters and last["HOTSPOT_ISLANDS"] == 2 and last["HOTSPOT_ISLANDS_OFF_DEFAULT"] == 0
    assert set(mesh.attributes) == {"UVMap", ".corner_vert", "cftuv_domain", "cftuv_owner", "position", MARK_ATTRIBUTE}, "no kernel UV copy is stored"
    assert set(mesh.values(MARK_ATTRIBUTE)[:, 0].tolist()) == {encode(1, False)}
    seam = kernel[:, 1] == 0.0
    corners = np.array(mesh.loops)
    for vertex in (0, 1, 2):
        here = mesh.uvs()[seam & (corners == vertex)]
        assert np.max(np.abs(here - here[0])) == 0.0, "the two wings meet at every shared seam vertex"


def test_apply_inverts_the_current_marks_and_island_marks_survive_a_rebuild_of_the_faces():
    mesh = _fold()
    kernel = mesh.uvs()
    mesh[MESH_PROPERTY] = record_to_json(record_of(_corner()).with_marks([TRIM]))
    envelope_production_mesh._map_hotspot(mesh)
    record = record_from_json(mesh[MESH_PROPERTY])
    old = mesh.values(MARK_ATTRIBUTE)[:, 0].astype(np.int64)
    strip = hotspot_blender.recovered_strip(mesh, record, old)
    assert np.max(np.abs(strip[:, 1] - kernel[:, 1])) <= 1e-6, "the corner mark inverts back to the kernel v"
    codes = np.array([encode(0, False), encode(0, False), encode(1, False), encode(1, False)])  # домен 3 - trim, его пара 5 - одна corner
    counters = hotspot_blender.remap_mesh(mesh, record, record, old, codes)
    assert counters["HOTSPOT_CORNER_WING_UNPAIRED"] == 1 and counters["HOTSPOT_ISLANDS_OFF_DEFAULT"] == 1
    trim_rows = np.repeat([True, True, False, False], 4)
    alone, _numbers = atlas_uvs(kernel[trim_rows].astype(np.float64), np.array(mesh.loops)[trim_rows], np.zeros(8, dtype=np.int64), record.mapping(encode(0, False)))
    assert np.max(np.abs(mesh.uvs()[trim_rows, 1] - alone[:, 1])) <= 1e-6, "the trim side lies across its mark exactly as a fresh mapping"
    hotspot_blender.remember_mesh_islands(mesh)
    remembered = record_from_json(mesh[MESH_PROPERTY])
    assert [(island.domain, island.code) for island in remembered.islands] == [(3, encode(0, False))]
    del mesh.attributes[MARK_ATTRIBUTE]  # грани пересобраны: кодов граней нет, новые номера владельцев, UV ядра в слое
    mesh.attributes["cftuv_owner"].data.values = np.array([7, 7, 2, 2], dtype=np.int32)
    mesh.attributes["UVMap"].data.values = kernel.reshape(-1).copy()
    envelope_production_mesh._map_hotspot(mesh)
    assert mesh.values(MARK_ATTRIBUTE)[:, 0].tolist() == codes.tolist()
    assert hotspot_blender.LAST_COUNTERS[mesh.session_uid]["HOTSPOT_ISLAND_MARK_UNMATCHED"] == 0
    fresh, _ = hotspot_blender.map_islands(kernel, np.array(mesh.loops), np.repeat(np.array([3, 3, 5, 5]) * (1 << 20) + [7, 7, 2, 2], 4), np.repeat(codes, 4), remembered)
    assert np.array_equal(mesh.uvs(), fresh.astype(np.float32)), "a rebuild maps from the kernel UVs again"


def test_a_broken_record_keeps_the_kernel_uvs_and_is_named():
    mesh = _fold()
    before = mesh.uvs()
    mesh[MESH_PROPERTY] = "{not json"
    envelope_production_mesh._map_hotspot(mesh)
    assert np.array_equal(mesh.uvs(), before)
    assert hotspot_blender.LAST_COUNTERS[mesh.session_uid] == {hotspot_blender.HOTSPOT_MAPPING_RECORD_INVALID: 1}


def _calls_in(function_name: str) -> set:
    tree = ast.parse((ROOT / "cftuv" / "envelope_production_mesh.py").read_text(encoding="utf-8"))
    function = next(node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == function_name)
    return {node.func.id for node in ast.walk(function) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)}


def test_every_writer_of_the_decal_uvs_maps_them_and_the_preview_and_the_exact_frame_share_the_function():
    """Каждый, кто кладёт UV в меш декали (`_write_uvs` либо `foreach_set("uv")`), зовёт `_map_hotspot` в той же функции."""

    tree = ast.parse((ROOT / "cftuv" / "envelope_production_mesh.py").read_text(encoding="utf-8"))
    writers = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name not in ("_write_uvs", "_fill_mesh_buffers"):
            for call in ast.walk(node):
                if isinstance(call, ast.Call) and (
                    (isinstance(call.func, ast.Name) and call.func.id == "_write_uvs")
                    or (isinstance(call.func, ast.Attribute) and call.func.attr == "foreach_set" and call.args and getattr(call.args[0], "value", None) == "uv")
                ):
                    writers.add(node.name)
    assert writers == {"_fill_mesh", "rebuild_mesh_from_image", "write_preview_geometry"}, writers
    for name in writers:
        assert "_map_hotspot" in _calls_in(name), f"{name} writes the decal UVs without the hotspot mapping"
    assert "_fill_mesh_buffers" in _calls_in("_fill_mesh")
