"""Blender background smoke: сборка меша из numpy (`rebuild_mesh_from_image`) равна сборке `_fill_mesh` по ВСЕМ массивам меша.

WIDTH_REFRESH_LEAN_V1 пересобирает меш декали на месте по образу точного прогона (`MeshImageV1`), когда раскладка сменилась, и возвращает им меш при отмене. Путь продукта
(`_clear_mesh_in_place` + `_fill_mesh`: `from_pydata`, слой UV, атрибуты граней, цикл по рёбрам для швов) остаётся эталоном, и здесь он сверяется с numpy-путём не по
дайджесту квитанции (его видят позиции, грани, UV, швы и два атрибута), а по каждому массиву меша: вершины, рёбра, петли (вершина и ребро), начала и длины граней, гладкость
граней, слои UV, ВСЕ атрибуты (в том числе `sharp_face`), швы. Затем:

1. меш, собранный numpy, равен эталону; пересборка уже заполненного меша равна пересборке пустого (очистка на месте ничего не оставляет);
2. запись по месту (`write_preview_geometry` с номерами владельцев) даёт меш, равный сборке тех же раскладки и новых чисел;
3. швы помечены теми же рёбрами, а число помеченных совпадает с эталоном;
4. то же для граней из 3, 4, 5 и 6 вершин и для нескольких доменов.

Прогон (в Blender 4.1 и выше): blender --background --factory-startup --python-exit-code 1 --python <этот файл>
Последняя строка при успехе: ENVELOPE_MESH_NUMPY_PATHS_BLENDER_SMOKE_OK
"""

from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import bpy
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
for path in (REPO_ROOT, REPO_ROOT / "kernel" / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))



def _retire_the_profile_addon_copy() -> None:
    """Профиль Blender держит включённой УСТАНОВЛЕННУЮ копию `cftuv`: импорт `cftuv.*` взял бы её, а не рабочее дерево. Копию выключает её же `unregister` в этом процессе (файлы не трогаются)."""

    installed = sys.modules.get("cftuv")
    origin = getattr(installed, "__file__", None)
    if installed is None or origin is None or Path(origin).resolve().is_relative_to(REPO_ROOT):
        return
    try:
        installed.unregister()
    except Exception as exc:  # noqa: BLE001 - назван и не скрыт
        print(f"NUMPY_PATHS installed addon copy did not unregister cleanly: {type(exc).__name__}: {exc}")


_retire_the_profile_addon_copy()
for module_name in tuple(sys.modules):
    if module_name == "cftuv" or module_name.startswith("cftuv."):
        del sys.modules[module_name]

from cftuv import envelope_production_mesh as mesh_module  # noqa: E402
from cftuv.envelope_width_preview_model import mesh_image_of  # noqa: E402

assert Path(mesh_module.__file__).resolve().is_relative_to(REPO_ROOT), mesh_module.__file__


def _arrays(scale: float = 1.0, owners_shift: int = 0):
    """Сетка 4x3 (шесть четырёхугольников), три треугольника, пятиугольник и шестиугольник; три домена; швы по двум стыкам."""

    positions = [(0.1 * x * scale, 0.2 * y * scale, 0.01 * x * y * scale) for x in range(4) for y in range(3)]
    faces = [(x * 3 + y, (x + 1) * 3 + y, (x + 1) * 3 + y + 1, x * 3 + y + 1) for x in range(3) for y in range(2)]
    base = len(positions)
    positions += [(2.0 + 0.1 * index * scale, 0.3 * (index % 2) * scale, 0.0) for index in range(11)]
    faces += [(base, base + 1, base + 2), (base + 2, base + 3, base + 4), (base + 4, base + 5, base + 6)]  # три треугольника
    faces += [(base + 5, base + 6, base + 7, base + 8, base + 9)]  # пятиугольник
    faces += [(base, base + 2, base + 4, base + 6, base + 8, base + 10)]  # шестиугольник
    uvs = [(positions[vertex][0] / 0.25, positions[vertex][1] / 0.25) for face in faces for vertex in face]
    face_domain = [index // 4 for index in range(len(faces))]
    face_owner = [(index % 3) + owners_shift for index in range(len(faces))]
    seams = [(0, 3), (3, 6), (base, base + 2), (base + 4, base + 6)]
    return SimpleNamespace(
        positions=tuple(positions), faces=tuple(faces), uvs=tuple(uvs), face_domain=tuple(face_domain), face_owner=tuple(face_owner), seam_edges=tuple(sorted(seams))
    )


def _fingerprint(mesh) -> dict:
    """Каждый массив меша: байты числовых массивов и значения атрибутов; внутренние атрибуты (с точки) не читаются, их массивы читаются прямыми свойствами."""

    def numbers(collection, name, dtype, size=1):
        buffer = np.empty(len(collection) * size, dtype=dtype)
        collection.foreach_get(name, buffer)
        return buffer.tobytes()

    result = {
        "counts": (len(mesh.vertices), len(mesh.edges), len(mesh.loops), len(mesh.polygons)),
        "co": numbers(mesh.vertices, "co", np.float32, 3),
        "edge_vertices": numbers(mesh.edges, "vertices", np.int32, 2),
        "use_seam": numbers(mesh.edges, "use_seam", np.bool_),
        "loop_vertex": numbers(mesh.loops, "vertex_index", np.int32),
        "loop_edge": numbers(mesh.loops, "edge_index", np.int32),
        "loop_start": numbers(mesh.polygons, "loop_start", np.int32),
        "loop_total": numbers(mesh.polygons, "loop_total", np.int32),
        "use_smooth": numbers(mesh.polygons, "use_smooth", np.bool_),
        "materials": [item.name for item in mesh.materials],
        "uv_layers": [item.name for item in mesh.uv_layers],
    }
    for layer in mesh.uv_layers:
        result["uv:" + layer.name] = numbers(layer.data, "uv", np.float32, 2)
    for attribute in mesh.attributes:
        if attribute.name.startswith("."):
            continue
        prop = {"FLOAT_VECTOR": "vector", "FLOAT2": "vector", "FLOAT_COLOR": "color", "BYTE_COLOR": "color"}.get(attribute.data_type, "value")
        result["attr:" + attribute.name] = (attribute.domain, attribute.data_type, [repr(getattr(item, prop)) if prop != "value" else item.value for item in attribute.data])
    return result


def _same(first, second, label) -> None:
    assert set(first) == set(second), (label, sorted(set(first) ^ set(second)))
    for name in first:
        assert first[name] == second[name], (label, name)


def _product_mesh(name, arrays):
    """Сборка продукта: пустой меш, `_clear_mesh_in_place` и `_fill_mesh`."""

    mesh = bpy.data.meshes.new(name)
    mesh_module._clear_mesh_in_place(mesh)
    marked = mesh_module._fill_mesh(mesh, arrays)
    return mesh, marked


def _numpy_mesh(mesh, arrays):
    image = mesh_image_of(arrays)
    positions = np.asarray(arrays.positions, dtype=np.float64).reshape(-1, 3)
    uvs = np.asarray(arrays.uvs, dtype=np.float64).reshape(-1, 2)
    return mesh_module.rebuild_mesh_from_image(mesh, positions, uvs, image)


def _main() -> None:
    arrays = _arrays()
    reference, marked = _product_mesh("reference", arrays)
    assert marked == len(arrays.seam_edges) == 4, marked

    # 1. Пустой меш и уже заполненный.
    fresh = bpy.data.meshes.new("numpy_fresh")
    assert _numpy_mesh(fresh, arrays) == marked
    _same(_fingerprint(reference), _fingerprint(fresh), "fresh")
    assert mesh_module.mesh_content_digest(fresh) == mesh_module.mesh_content_digest(reference)
    again = _numpy_mesh(fresh, arrays)  # пересборка поверх заполненного меша: очистка на месте ничего не оставляет
    assert again == marked
    _same(_fingerprint(reference), _fingerprint(fresh), "rebuilt twice")
    other = _arrays(scale=1.7, owners_shift=2)
    assert _numpy_mesh(fresh, other) == len(other.seam_edges)  # другие числа, те же раскладка и швы
    expected_other, _marked = _product_mesh("reference_other", other)
    _same(_fingerprint(expected_other), _fingerprint(fresh), "other numbers")
    assert mesh_module.mesh_content_digest(fresh) == mesh_module.mesh_content_digest(expected_other) != mesh_module.mesh_content_digest(reference)

    # 2. Запись по месту: те же раскладка и швы, новые позиции, UV и номера владельцев.
    inplace, _marked = _product_mesh("inplace", arrays)
    image = mesh_image_of(other)
    frame_positions = np.ascontiguousarray(np.asarray(other.positions, dtype=np.float64), dtype=np.float32).reshape(-1)
    frame_uvs = np.ascontiguousarray(np.asarray(other.uvs, dtype=np.float64), dtype=np.float32).reshape(-1)
    mesh_module.write_preview_geometry(inplace, frame_positions, frame_uvs, image.face_owner)
    _same(_fingerprint(expected_other), _fingerprint(inplace), "in place")
    assert mesh_module.mesh_content_digest(inplace) == mesh_module.mesh_content_digest(expected_other)

    # 3. Меш другой раскладки: пересборка по образу меняет число граней, вершин и швы так же, как сборка продукта.
    smaller = SimpleNamespace(
        positions=arrays.positions[:12], faces=arrays.faces[:6], uvs=arrays.uvs[:24], face_domain=arrays.face_domain[:6], face_owner=arrays.face_owner[:6], seam_edges=((0, 3),)
    )
    expected_small, small_marked = _product_mesh("reference_small", smaller)
    assert _numpy_mesh(inplace, smaller) == small_marked == 1
    _same(_fingerprint(expected_small), _fingerprint(inplace), "another layout")

    # 4. Пустой образ (ни одной грани) не падает и даёт пустой меш.
    empty = SimpleNamespace(positions=(), faces=(), uvs=(), face_domain=(), face_owner=(), seam_edges=())
    assert _numpy_mesh(inplace, empty) == 0 and len(inplace.vertices) == len(inplace.polygons) == 0
    print(f"NUMPY MESH PATHS: identical to the product fill on {bpy.app.version_string}: {len(arrays.faces)} faces of 3-6 vertices, {marked} seams")
    print("ENVELOPE_MESH_NUMPY_PATHS_BLENDER_SMOKE_OK")


if __name__ == "__main__":
    _main()
