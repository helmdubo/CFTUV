"""Blender 4.5 background: декаль Both под Hotspot (BEVEL_FACES_BOTH_HOTSPOT_V1) - остров Hotspot = СЕГМЕНТ Both, одним куском в метке.

Синтетика в памяти (ничего не сохраняется): куб с бевелом в 2 сегмента (Face Strength) и стена-колонка `_noisy_wall(0.04, levels=3)` из
`test_bevel_faces_build`; разметка - текст в памяти (трим `+y` и corner `-y`, текстура 2048 x 2048), материал декали ссылается на неё.

1. Hotspot выключен - меш Both побитово тот же, что без разметки вовсе (V3), и у него есть атрибут сегмента;
2. Hotspot включён, метка trim / corner / corner со Swap: те же вершины и грани, что у нажатия без Hotspot; острова Hotspot (группы
   `mesh_arrays`) = сегменты; UV меша = отображение UV крыльев нажатия без Hotspot рамкой сегмента (`map_islands`), побитово; швы - те же,
   что у V3, и ровно рёбра разрыва UV; острова UV (по непрерывности) = те же, что у V3; на рельсе внутри сегмента разрыв 0; вывернутых
   граней UV 0; у куба каждый остров в прямоугольнике своей метки (у стены - число углов за толщиной метки: юбка шире полосы трима,
   BEVEL_WINGS_SKIRT_PAST_TRIM, и за её длиной: метка повторяется вдоль);
3. живая ширина под Hotspot: точный пересчёт побитово равен нажатию на этой ширине;
4. метка острова в Edit Mode: одна выделенная грань копии - метка ложится на весь сегмент (копия и юбки), остальные сегменты побитово те же;
   метка переживает живую ширину (UNMATCHED 0); декаль Both, собранная без Hotspot, - метка тоже на весь сегмент.

Прогон: blender -b --factory-startup --python-exit-code 1 --python tests/blender/test_bevel_both_hotspot.py
Последняя строка при успехе: BEVEL_BOTH_HOTSPOT_BLENDER_OK
"""

from __future__ import annotations

import sys
from pathlib import Path

import bpy

try:
    import sympy  # noqa: F401
except ImportError:  # --factory-startup: sympy живёт в каталоге модулей пользователя
    sys.path.append(bpy.utils.user_resource("SCRIPTS", path="modules"))

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_bevel_faces_build as T  # noqa: E402

import bmesh  # noqa: E402
import numpy as np  # noqa: E402

TEXT = "BothSheet.hotspot"
SIZE = 2048
SUMMARY = []


def _marks():
    from cftuv.hotspot_markup import HotspotMarkV1

    return HotspotMarkV1(0, "trim", "trim", 0, 64, SIZE, 256, "+y"), HotspotMarkV1(1, "edge", "corner", 0, 1024, SIZE, 256, "-y", 16)


def _markup():
    from cftuv.envelope_production_mesh import DEFAULT_DECAL_MATERIAL
    from cftuv.hotspot_blender import link_markup, write_markup_text
    from cftuv.hotspot_markup import HotspotMarkupV1

    problems = write_markup_text(TEXT, HotspotMarkupV1(SIZE, SIZE, "sheet.png", _marks()))
    assert problems == (), problems
    material = bpy.data.materials.get(DEFAULT_DECAL_MATERIAL) or bpy.data.materials.new(DEFAULT_DECAL_MATERIAL)
    link_markup(material, TEXT)


def _hotspot(enabled, mark_id=0, swap=False):
    settings = bpy.context.scene.hotspotuv_hotspot
    settings.enabled, settings.mark_id, settings.swap_wings = enabled, mark_id, swap


def _source(kind):
    if kind == "cube":
        obj = T._modifier_bevel("bh_cube", "cube", 2, True)
        T._settings().envelope_debug_alpha = T.BOTH_ALPHA
    else:
        obj = T._object("bh_columns", T._noisy_wall(0.04, levels=3))
        T._settings().envelope_debug_alpha = T.ALPHA
    return obj, frozenset(polygon.index for polygon in obj.data.polygons if polygon.material_index == 1)


def _read(collection, name, dtype, size=1):
    out = np.empty(len(collection) * size, dtype=dtype)
    collection.foreach_get(name, out)
    return out


def _state(mesh):
    """`(позиции, вершины углов, число углов граней, UV, швы)` меша декали."""

    seams = _read(mesh.edges, "vertices", np.int32, 2).reshape(-1, 2)[_read(mesh.edges, "use_seam", np.bool_)]
    return (
        _read(mesh.vertices, "co", np.float32, 3).reshape(-1, 3), _read(mesh.loops, "vertex_index", np.int32), _read(mesh.polygons, "loop_total", np.int32),
        _read(mesh.uv_layers["UVMap"].data, "uv", np.float32, 2).reshape(-1, 2), {tuple(sorted(pair)) for pair in seams.tolist()},
    )


def _press(obj, bevel):
    if bpy.context.object is not None and bpy.context.object.mode != "OBJECT":
        bpy.ops.object.mode_set(mode="OBJECT")
    for item in bpy.context.view_layer.objects:  # декаль, выделенная для метки, не входит в Edit Mode источника
        item.select_set(False)
    T._enter_face_selection(obj, bevel)
    built, refused, status, seconds = T._press(obj, "BOTH")
    bpy.ops.object.mode_set(mode="OBJECT")
    return bpy.data.objects[obj.name + ".CFTUV_Decal"].data, status, seconds


def _uv_islands(mesh):
    uv = mesh.uv_layers["UVMap"].data
    loop_at, edge_faces = {}, {}
    for polygon in mesh.polygons:
        for loop in polygon.loop_indices:
            loop_at[(polygon.index, mesh.loops[loop].vertex_index)] = loop
        for key in polygon.edge_keys:
            edge_faces.setdefault(key, []).append(polygon.index)
    return T._islands(mesh, uv, loop_at, edge_faces), loop_at, edge_faces


def _segment_checks(name, mesh, off, strict_rect):
    """Острова Hotspot = сегменты, UV = рамка сегмента UV крыльев V3 `off`, швы V3, разрыв на рельсе 0, вывернутых 0, острова в своих метках."""

    from cftuv.hotspot_blender import face_codes, mesh_arrays, mesh_record
    from cftuv.hotspot_islands import map_islands

    positions, vertices, totals, atlas, seams = _state(mesh)
    assert np.array_equal(positions, off[0]) and np.array_equal(vertices, off[1]) and np.array_equal(totals, off[2]), (name, "the same mesh as without Hotspot")
    record, error = mesh_record(mesh)
    assert record is not None and record.segments, (name, error, "a Both mesh reads its islands as segments")
    _vertices, _totals, group = mesh_arrays(mesh)
    copy = T._copy_domain(mesh)
    domain = _read(mesh.attributes["cftuv_domain"].data, "value", np.int32)
    owner = _read(mesh.attributes["cftuv_owner"].data, "value", np.int32)
    segments = set((owner[domain == copy] // 2).tolist())
    with_copy = set(group[domain == copy].tolist())
    lonely = set(group.tolist()) - with_copy
    assert len(with_copy) == len(segments), (name, "a Hotspot island is a segment", len(with_copy), len(segments))
    codes = face_codes(mesh, len(totals))
    expected, _counters = map_islands(off[3], vertices.astype(np.int64), np.repeat(group, totals), np.repeat(codes, totals), record)
    assert np.array_equal(atlas, expected.astype(np.float32)), (name, "the UVs are the segment frame of the V3 wing UVs, bit for bit")
    assert seams == off[4], (name, "the seams are the V3 seams", len(seams), len(off[4]))
    islands, loop_at, edge_faces = _uv_islands(mesh)
    uv = mesh.uv_layers["UVMap"].data
    rail_gap, mismatch = 0.0, 0
    for (a, b), faces in edge_faces.items():
        if len(faces) != 2:
            continue
        gap = max((uv[loop_at[(faces[0], x)]].uv - uv[loop_at[(faces[1], x)]].uv).length for x in (a, b))
        mismatch += (gap > 1e-5) != ((a, b) in seams)
        if (domain[faces[0]] == copy) != (domain[faces[1]] == copy) and (a, b) not in seams:
            rail_gap = max(rail_gap, gap)
    assert mismatch == 0 and rail_gap <= 1e-6, (name, "seams are exactly the UV breaks, the rail is continuous", mismatch, rail_gap)
    flipped = sum(1 for polygon in mesh.polygons if T._uv_face_area(uv, polygon) < 0.0)
    assert flipped == 0, (name, "no UV face is flipped", flipped)
    across = along = 0  # метки обеих строк - горизонтальные полосы: поперёк - y, вдоль - x (за длиной метка повторяется, wrap)
    for code in np.unique(codes).tolist():
        mark = record.mapping(code).mark
        pixels = atlas[np.repeat(codes, totals) == code].astype(np.float64) * SIZE
        across += int(np.count_nonzero((pixels[:, 1] < mark.y - 1e-2) | (pixels[:, 1] > mark.y + mark.h + 1e-2)))
        along += int(np.count_nonzero((pixels[:, 0] < mark.x - 1e-2) | (pixels[:, 0] > mark.x + mark.w + 1e-2)))
    if strict_rect:
        assert across == 0 and along == 0, (name, "every island lies inside its mark", across, along)
    return {"segments": len(segments), "hotspot_islands": len(with_copy) + len(lonely), "lonely": len(lonely), "uv_islands": len(islands),
            "seams": len(seams), "rail_gap": rail_gap, "flipped": flipped, "across": across, "along": along, "loops": int(atlas.shape[0])}


def _run_both_under_hotspot(kind):
    from cftuv.envelope_production_mesh import mesh_content_digest

    T._clean()
    _markup()
    _hotspot(False)
    obj, bevel = _source(kind)
    mesh, status, _seconds = _press(obj, bevel)
    off = _state(mesh)
    off_islands = len(_uv_islands(mesh)[0])
    plain = mesh_content_digest(mesh)
    assert mesh.get("cftuv_hotspot") is None and mesh.attributes.get("cftuv_bevel_segment") is not None, "Hotspot off: no mapping record"
    T._uv_checks(f"{kind} off", mesh, T._copy_domain(mesh), strict_islands=kind == "cube", trim_tolerance=0.0 if kind == "cube" else 0.5)
    for mark_id, swap, label in ((0, False, "trim"), (1, False, "corner"), (1, True, "corner swapped")):
        _hotspot(True, mark_id, swap)
        mesh, status, seconds = _press(obj, bevel)
        assert "BEVEL_WINGS_OFF" not in status, (kind, status, "Both never turns its segments off under Hotspot")
        numbers = _segment_checks(f"{kind} {label}", mesh, off, strict_rect=kind == "cube")
        assert numbers["uv_islands"] == off_islands, (kind, label, "the UV islands of V3", numbers, off_islands)
        SUMMARY.append(
            f"{kind} {label}: {seconds:.2f} s, {numbers['hotspot_islands']} hotspot islands for {numbers['segments']} segments "
            f"({numbers['lonely']} skirt-only), UV islands {numbers['uv_islands']} (V3 {off_islands}), seams {numbers['seams']} = V3, "
            f"rail gap {numbers['rail_gap']:.2g}, flipped {numbers['flipped']}, of {numbers['loops']} loops past the mark thickness {numbers['across']} "
            f"(skirt past trim), past its length {numbers['along']} (the mark repeats along)"
        )
    _hotspot(False)
    mesh, _status, _seconds = _press(obj, bevel)
    assert mesh_content_digest(mesh) == plain, "Hotspot off again: the V3 mesh bit for bit"
    T._clean()


def _decal_edit_select(decal, faces):
    for item in bpy.context.view_layer.objects:
        item.select_set(False)
    bpy.context.view_layer.objects.active = decal
    decal.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.context.tool_settings.mesh_select_mode = (False, False, True)
    bm = bmesh.from_edit_mesh(decal.data)
    bm.faces.ensure_lookup_table()
    for item in (*bm.verts, *bm.edges, *bm.faces):
        item.select = False
    for index in faces:
        bm.faces[index].select_set(True)
    bm.select_flush_mode()
    bmesh.update_edit_mesh(decal.data)


def _pick(decal, face, mark_id):
    """Edit Mode на декали, выделена одна грань `face`, выбор метки `mark_id` (HOTSPOT_APPLY_ON_PICK_V1); `(коды граней, группы, числа)`."""

    from cftuv.hotspot_blender import LAST_COUNTERS, face_codes, mesh_arrays

    _decal_edit_select(decal, [face])
    result = bpy.ops.hotspotuv.hotspot_pick_mark(mark_id=mark_id)
    note = bpy.context.scene.hotspotuv_hotspot.apply_note
    assert result == {"FINISHED"} and not note.startswith("Mark set for the next"), (result, note)
    bpy.ops.object.mode_set(mode="OBJECT")
    _vertices, totals, group = mesh_arrays(decal.data)
    return face_codes(decal.data, len(totals)), group, dict(LAST_COUNTERS.get(decal.data.session_uid, {}))


def _run_island_marks_and_live_width():
    from cftuv.envelope_production_mesh import mesh_content_digest
    from cftuv.hotspot_blender import LAST_COUNTERS, face_codes, mesh_arrays
    from cftuv.hotspot_islands import NO_MARK, code_for

    T._clean()
    _markup()
    trim, corner = _marks()
    _hotspot(False)
    obj, bevel = _source("cube")
    mesh, _status, _seconds = _press(obj, bevel)
    decal = bpy.data.objects[obj.name + ".CFTUV_Decal"]
    copy = T._copy_domain(mesh)
    domain = _read(mesh.attributes["cftuv_domain"].data, "value", np.int32)
    face = int(np.flatnonzero(domain == copy)[0])
    before = _state(mesh)[3].copy()
    codes, group, counters = _pick(decal, face, corner.id)  # декаль без Hotspot: остров - весь сегмент
    island = group == group[face]
    assert np.all(codes[island] == code_for(corner, False)) and np.all(codes[~island] == NO_MARK), "an unmapped Both decal takes the mark per segment"
    loops = np.repeat(island, _state(mesh)[2])
    assert np.array_equal(_state(decal.data)[3][~loops], before[~loops]), "the other segments keep their wing UVs"
    unmapped_faces = int(island.sum())

    _hotspot(True, corner.id)
    mesh, _status, _seconds = _press(obj, bevel)
    decal = bpy.data.objects[obj.name + ".CFTUV_Decal"]
    _vertices, totals, group = mesh_arrays(mesh)
    before = _state(mesh)[3].copy()
    codes, group, counters = _pick(decal, face, trim.id)
    island = group == group[face]
    assert np.all(codes[island] == code_for(trim, False)) and np.all(codes[~island] == code_for(corner, False)), "the mark lies on the whole segment"
    assert island.sum() > np.count_nonzero(island & (domain == copy)), "the segment island takes its skirts too"
    assert counters.get("HOTSPOT_CORNER_PAIR_EXPANDED", 0) == 0 and counters["HOTSPOT_ISLANDS_SELECTED"] == 1, counters
    loops = np.repeat(island, totals)
    after = _state(decal.data)[3]
    assert np.array_equal(after[~loops], before[~loops]) and not np.array_equal(after[loops], before[loops]), "only the picked segment moved"
    path = next(name for name in counters if name.startswith("HOTSPOT_REMAP_"))
    # Живая ширина: метка острова держится (якоря - центры граней копии), точный пересчёт = нажатию на этой ширине.
    applied, failed = T._counts(T._controller().width_live)
    T._settings().envelope_debug_alpha = T.BOTH_ALPHA * 1.25
    scheduler = T._pump_width_live()
    assert T._counts(scheduler) == (applied + 1, failed), scheduler.counters
    live_mesh = bpy.data.objects[obj.name + ".CFTUV_Decal"].data
    live_counters = dict(LAST_COUNTERS.get(live_mesh.session_uid, {}))
    _v, live_totals, live_group = mesh_arrays(live_mesh)
    live_codes = face_codes(live_mesh, len(live_totals))
    picked = live_group == live_group[face]
    assert live_counters.get("HOTSPOT_ISLAND_MARK_UNMATCHED", 0) == 0, live_counters
    assert np.all(live_codes[picked] == code_for(trim, False)) and np.all(live_codes[~picked] == code_for(corner, False)), "the segment mark survives the live width"
    live, live_state = mesh_content_digest(live_mesh), _state(live_mesh)
    T._controller().clear()
    _hotspot(True, corner.id)  # выбор метки острова стал и меткой следующей кнопки (HOTSPOT_APPLY_ON_PICK_V1): кнопка - с меткой живой ширины
    mesh, _status, _seconds = _press(obj, bevel)
    pressed, pressed_state = mesh_content_digest(mesh), _state(mesh)
    differ = [name for name, a, b in zip(("positions", "corners", "totals", "uv"), live_state[:4], pressed_state[:4]) if a.shape != b.shape or not np.array_equal(a, b)]
    differ += ["seams"] if live_state[4] != pressed_state[4] else []
    assert pressed == live, ("live width under Hotspot equals a press at that width (the press carries the segment mark)", differ)
    SUMMARY.append(
        f"island marks: unmapped Both decal - mark on {unmapped_faces} faces of one segment; Hotspot decal - {int(island.sum())} faces "
        f"({int(np.count_nonzero(island & (domain == copy)))} copy) of one segment by {path}, others bit for bit; live width "
        f"{T.BOTH_ALPHA * 1.25:g}: UNMATCHED {live_counters.get('HOTSPOT_ISLAND_MARK_UNMATCHED', 0)}, exact recompute = press"
    )
    T._clean()
    _hotspot(False)
    T._settings().envelope_debug_alpha = T.ALPHA


def _main():
    import cftuv

    try:
        cftuv.register()
    except Exception:  # уже зарегистрирован
        pass
    settings = T._settings()
    settings.envelope_debug_alpha = T.ALPHA
    settings.envelope_debug_workers = 0
    _run_both_under_hotspot("cube")
    _run_both_under_hotspot("columns")
    _run_island_marks_and_live_width()
    for line in SUMMARY:
        print("[BevelBothHotspotTest]", line, flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("BEVEL_BOTH_HOTSPOT_BLENDER_OK", flush=True)


if __name__ == "__main__":
    _main()
