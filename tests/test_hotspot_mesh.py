"""HOTSPOT_ATLAS_MAPPING_V1 в писателе меша: без записи отображения UV меша не трогаются (режим выключен - меш тот же побитово), с записью -
UV ядра в меше заменяются UV атласа одной функцией, которую зовут все писатели UV (кнопка, подтверждение, кадры живой ширины)."""

from __future__ import annotations

import ast
from pathlib import Path

import numpy as np

from cftuv import envelope_production_mesh, hotspot_blender
from cftuv.hotspot_markup import HotspotMarkV1
from cftuv.hotspot_mapping import MESH_PROPERTY, HotspotMappingV1, atlas_uvs, mapping_to_json

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
    pass


class _Layer:
    def __init__(self, attribute):
        self.data = attribute.data


class _Mesh(dict):
    """Меш Blender в объёме писателя: свойства ID, углы, грани, атрибуты и слой UV `UVMap` (атрибут FLOAT2 углов)."""

    name = "Decal.CFTUV_Decal"
    session_uid = 7

    def __init__(self, uvs, corner_vertices, totals, domain, owner):
        super().__init__()
        self.attributes = _Attributes()
        self.attributes["UVMap"] = _Attribute(np.asarray(uvs, dtype=np.float32).reshape(-1), "FLOAT2", "CORNER", 2)
        self.attributes[".corner_vert"] = _Attribute(np.asarray(corner_vertices, dtype=np.int32), "INT", "CORNER")
        self.attributes["cftuv_domain"] = _Attribute(np.asarray(domain, dtype=np.int32), "INT", "FACE")
        self.attributes["cftuv_owner"] = _Attribute(np.asarray(owner, dtype=np.int32), "INT", "FACE")
        self.loops = list(corner_vertices)
        self.polygons = _Data(np.asarray(totals, dtype=np.int32), 1)
        self.uv_layers = {"UVMap": _Layer(self.attributes["UVMap"])}

    def uvs(self):
        return self.attributes["UVMap"].data.values.reshape(-1, 2).copy()


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
    monkeypatch.setattr(hotspot_blender, "map_mesh_uvs", lambda *_args: (_ for _ in ()).throw(AssertionError("called")))
    envelope_production_mesh._map_hotspot(mesh)
    assert np.array_equal(mesh.uvs(), before) and MESH_PROPERTY not in mesh


def test_with_the_record_the_mesh_uvs_become_the_atlas_uvs_of_the_pure_mapping():
    mesh = _fold()
    kernel = mesh.uvs()
    mesh[MESH_PROPERTY] = mapping_to_json(_corner())
    envelope_production_mesh._map_hotspot(mesh)
    group = np.repeat(np.array([3, 3, 5, 5], dtype=np.int64) * (1 << 20) + np.array([0, 0, 1, 1]), 4)
    expected, counters = atlas_uvs(kernel.astype(np.float64), np.array(mesh.loops), group, _corner())
    assert np.array_equal(mesh.uvs(), expected.astype(np.float32))
    assert counters["HOTSPOT_CORNER_SIDE_PAIRS"] == 1
    assert hotspot_blender.LAST_COUNTERS[mesh.session_uid] == counters
    seam = kernel[:, 1] == 0.0
    corners = np.array(mesh.loops)
    for vertex in (0, 1, 2):
        here = mesh.uvs()[seam & (corners == vertex)]
        assert np.max(np.abs(here - here[0])) == 0.0, "the two wings meet at every shared seam vertex"


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
