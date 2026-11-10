"""Blender 4.5 background: живой показ разметки на декалях, пока открыто окно MarkUp (HOTSPOT_MARKUP_LIVE_ON_RELEASE_V1).

Синтетика в памяти (ничего не сохраняется). Окна в фоне нет: работающий инструмент - его ключ в `_RUNNING`, закрытие - хвост `cancel` и таймер
обновления вручную (как п. 10 `test_hotspot_markup`); шаги отмены - `ed.undo_push` / `ed.undo` / `ed.redo` фонового Blender.

1. Складка `test_hotspot_markup` (trim и corner, метка острова): законченная правка (перенос, вид через меню метки) меняет UV до закрытия,
   повтор того же текста ничего не делает, непринятый черновик - HOTSPOT_MARKUP_LIVE_DRAFT_REFUSED, меш - последний принятый показ;
2. Ctrl+Z при показе - снимок возвращён обработчиком ДО шага (HOTSPOT_MARKUP_LIVE_WITHDRAWN), данные меша - как до сессии; Ctrl+Shift+Z
   возвращает снимок так же; показ после отмены и повтора - тот же побитово;
3. закрытие: хвост `cancel` возвращает снимок, обновление - ровно последний показ побитово, один Ctrl+Z - состояние до сессии, Ctrl+Shift+Z -
   снова итог;
4. сброс черновика возвращает снимок; меш в Edit Mode не пишется (HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE) и показывается после выхода; меш,
   который после показа правил другой оператор, не возвращается (HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE); остановка инструмента возвращает снимок;
5. декали Both (куб - trim, стена - corner, и копии стены): острова - сегменты, правка trim и corner - показ равен обновлению закрытия
   побитово, один Ctrl+Z - все декали до сессии; время показа на отпускание напечатано.

Прогон: blender -b --factory-startup --python-exit-code 1 --python tests/blender/test_hotspot_live.py
Последняя строка при успехе: HOTSPOT_LIVE_BLENDER_OK
"""

from __future__ import annotations

import os
import re
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import test_bevel_both_hotspot as B  # noqa: E402 - он же ставит путь sympy и модуль сборки Both
import test_hotspot_markup as H  # noqa: E402

import bpy  # noqa: E402
import numpy as np  # noqa: E402

COPIES = 4
SUMMARY = []


def _state(name):
    """`(UV слоя, коды граней | None, запись)` меша объекта `name` - всё, что пишет обновление декали."""

    from cftuv.hotspot_blender import current_atlas, face_codes

    mesh = bpy.data.objects[name].data
    codes = face_codes(mesh, len(mesh.polygons))
    return current_atlas(mesh, "UVMap").copy(), None if codes is None else codes.copy(), str(mesh.get("cftuv_hotspot", ""))


def _same(a, b) -> bool:
    codes = (a[1] is None and b[1] is None) or (a[1] is not None and b[1] is not None and np.array_equal(a[1], b[1]))
    return np.array_equal(a[0], b[0]) and codes and a[2] == b[2]


def _editor():
    return bpy.context.scene.hotspotuv_hotspot_editor


def _index(mark_id):
    return next(index for index, item in enumerate(_editor().marks) if item.mark_id == mark_id)


def _edit(**values):
    """Правка инструмента: поля метки `mark` черновика без проверки (как перетаскивание), затем законченная правка (`commit_edit`)."""

    from cftuv import hotspot_editor as E
    from cftuv import hotspot_live

    item = _editor().marks[_index(values.pop("mark"))]
    E._SUSPEND["commit"] = True
    try:
        for key, value in values.items():
            setattr(item, key, value)
    finally:
        E._SUSPEND["commit"] = False
    problems = E.commit_edit(bpy.context)
    return problems, hotspot_live.status()


def _ms(line):
    found = re.search(r"previewed in ([0-9.]+) ms", line)
    assert found, line
    return float(found.group(1))


def _start(text):
    from cftuv import hotspot_editor as E

    assert E.load_draft(bpy.context.scene, text, discard=True) == ""
    E._RUNNING.add(-1)  # инструмент работает (ключ области мини-окна)


def _close(text, names):
    """Закрытие мини-окна: хвост `cancel` (снимок назад, обновление заказано), затем таймер. `(состояния после cancel, строка обновления)`."""

    from cftuv import hotspot_refresh
    from cftuv.hotspot_editor import HOTSPOTUV_OT_HotspotMarkupEdit

    HOTSPOTUV_OT_HotspotMarkupEdit.cancel(types.SimpleNamespace(_drag=None, _area_pointer=-1), bpy.context)
    restored = [_state(name) for name in names]
    assert [name for name, _generation in hotspot_refresh._STATE["pending"]] == [text]
    bpy.app.timers.unregister(hotspot_refresh.run_pending)
    hotspot_refresh.run_pending()
    return restored, bpy.context.scene.hotspotuv_hotspot.apply_note


def _fold():
    """Декаль складки: острова домена `low` - метка 2 (rust), `high` - метка 0 (drips). `(low, high)`."""

    from cftuv.hotspot_markup import HotspotMarkV1

    bpy.context.scene.hotspotuv_settings.envelope_debug_alpha = 0.25
    H._build_fold()
    marks = (HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, "+y"), HotspotMarkV1(1, "edge", "corner", 0, 256, 1024, 96, "+y", 12), HotspotMarkV1(2, "rust", "trim", 0, 400, 1024, 64, "+y"))
    H._markup(*marks)
    H._select_seam(bpy.data.objects[H.SOURCE])
    H._hotspot(True, 0)
    H._press()
    _uv, _vertex, domain = H._arrays()
    low, high = sorted(np.unique(domain))
    H._decal_edit(low)
    H._hotspot(True, 2)
    H._apply()
    H._object_mode()
    return low, high


def _check_mark_switch(touch_first):
    """Поток владельца (Object Mode): окно открыто, метка декали правится живо (`touch_first`) или нет; в окне добавлена метка 1; панель
    переключает декаль на неё (оператор выбора метки со своим шагом отмены); правки метки 1 - декаль идёт за ними каждый раз; закрытие -
    итог = последний показ, Ctrl+Z - состояние после выбора метки (его шаг), ещё Ctrl+Z - до сессии."""

    from cftuv import hotspot_editor as E
    from cftuv import hotspot_live
    from cftuv.hotspot_markup import HotspotMarkV1

    bpy.context.scene.hotspotuv_settings.envelope_debug_alpha = 0.25
    H._build_fold()
    H._markup(HotspotMarkV1(0, "drips", "trim", 0, 64, 1024, 128, "+y"))
    H._select_seam(bpy.data.objects[H.SOURCE])
    H._hotspot(True, 0)
    H._press()
    _uv, _vertex, domain = H._arrays()
    low, high = sorted(np.unique(domain))
    H._object_mode()
    _start(H.TEXT)
    bpy.ops.ed.undo_push(message="Mark Up")
    pre = _state(H.DECAL)
    if touch_first:
        _edit(mark=0, y=80)
        H._rows(low, (80.0, 208.0), "live: mark 0 moved")
    item = _editor().marks.add()  # новая метка в окне (перетаскивание по пустому месту)
    E._SUSPEND["commit"] = True
    item.mark_id, item.label, item.kind, item.up, item.x, item.y, item.w, item.h = E.new_mark_id(_editor()), "rust", "trim", "+y", 0, 400, 1024, 64
    E._SUSPEND["commit"] = False
    assert E.commit_edit(bpy.context) == () and item.mark_id == 1
    for obj in bpy.context.view_layer.objects:
        obj.select_set(obj.name == H.DECAL)
    bpy.context.view_layer.objects.active = bpy.data.objects[H.DECAL]
    assert bpy.ops.hotspotuv.hotspot_pick_mark("EXEC_DEFAULT", True, mark_id=1) == {"FINISHED"}  # панель: декаль - на метку 1, шаг отмены
    for where in (low, high):
        H._rows(where, (400.0, 464.0), "the panel switched the decal to mark 1")
    picked = _state(H.DECAL)
    for step, (y, h) in enumerate(((400, 48), (320, 48), (256, 96))):
        _problems, line = _edit(mark=1, y=y, h=h)
        for where in (low, high):
            H._rows(where, (float(y), float(y + h)), f"edit {step} of the switched-to mark: the decal follows ({line})")
    live = _state(H.DECAL)
    restored, note = _close(H.TEXT, [H.DECAL])
    assert _same(restored[0], picked), "close: back to the state the pick left"
    assert _same(_state(H.DECAL), live), ("the refresh on close is the last live preview", note)
    bpy.ops.ed.undo()
    assert _same(_state(H.DECAL), picked), "Ctrl+Z after close: the whole session after the pick"
    bpy.ops.ed.undo()
    assert not _same(_state(H.DECAL), picked)
    if not touch_first:
        assert _same(_state(H.DECAL), pre), "Ctrl+Z again: before the pick, before the session"
    SUMMARY.append(f"mark switch ({'mark edited live first' if touch_first else 'mark untouched first'}): the decal follows every edit of the switched-to mark; {hotspot_live.status()[:60]}")


def _check_fold_session():
    from cftuv import hotspot_live
    from cftuv.hotspot_blender import mesh_record

    low, high = _fold()
    _start(H.TEXT)  # как «Mark Up»: черновик загружен, шаг отмены записан; меш декали этим шагом не тронут
    bpy.ops.ed.undo_push(message="base")
    bpy.ops.ed.undo_push(message="Mark Up")
    pre = _state(H.DECAL)
    _uv, _vertex, domain = H._arrays()
    problems, line = _edit(mark=2, y=420)
    assert problems == () and "1 decal(s) previewed" in line and "1 island(s) remapped, 1 kept bitwise" in line, line
    H._rows(low, (420.0, 484.0), "the moved mark, live before the window closes")
    moved = _state(H.DECAL)
    assert np.array_equal(moved[0][domain == high], pre[0][domain == high]), "islands of other marks stay bit for bit"
    assert mesh_record(bpy.data.objects[H.DECAL].data)[0].mark(2).y == 420
    first_ms = _ms(line)
    _edit(mark=2, y=420)  # тот же текст (щелчок без перетаскивания): ничего не пишется
    assert _same(_state(H.DECAL), moved) and hotspot_live.status() == line
    _editor().active = _index(0)
    assert bpy.ops.hotspotuv.hotspot_mark_kind(kind="corner") == {"FINISHED"}  # меню метки - та же законченная правка
    kind_line = hotspot_live.status()
    live = _state(H.DECAL)
    assert "1 decal(s) previewed" in kind_line and not np.array_equal(live[0][domain == high], pre[0][domain == high]), kind_line
    assert mesh_record(bpy.data.objects[H.DECAL].data)[0].mark(0).kind == "corner"
    problems, line = _edit(mark=1, y=400)  # на метку 2: черновик не принят, текст и декали - прежний принятый показ
    assert problems and line.startswith("HOTSPOT_MARKUP_LIVE_DRAFT_REFUSED") and _same(_state(H.DECAL), live), line
    _edit(mark=1, y=256)
    assert _same(_state(H.DECAL), live)

    # Ctrl+Z при показе: снимок возвращается обработчиком ДО шага отмены, данные - как до сессии (меш из файла Blender иначе взял бы из
    # памяти с превью; меш этой сессии он перечитывает сам - обработчик назван строкой в обоих случаях).
    bpy.ops.ed.undo()
    assert _same(_state(H.DECAL), pre) and hotspot_live.status().startswith("HOTSPOT_MARKUP_LIVE_WITHDRAWN: undo: 1 decal(s)"), hotspot_live.status()
    bpy.ops.ed.redo()
    assert _same(_state(H.DECAL), pre)
    _redo_edits()
    assert _same(_state(H.DECAL), live), "the preview after undo is the same, bit for bit"
    # Ctrl+Shift+Z при показе: шаг отмены назад, показ, затем повтор шага - снимок возвращается так же, до шага.
    bpy.ops.ed.undo()
    _redo_edits()
    bpy.ops.ed.redo()
    assert _same(_state(H.DECAL), pre) and hotspot_live.status().startswith("HOTSPOT_MARKUP_LIVE_WITHDRAWN: redo: 1 decal(s)"), hotspot_live.status()
    _redo_edits()
    assert _same(_state(H.DECAL), live), "the preview after redo is the same, bit for bit"

    restored, note = _close(H.TEXT, [H.DECAL])
    assert _same(restored[0], pre), "close: the snapshot is back before the refresh"
    assert "1 decal(s) (1 object(s)) updated" in note, note
    final = _state(H.DECAL)
    assert _same(final, live), "the refresh on close is exactly the last live preview"
    bpy.ops.ed.undo()
    assert _same(_state(H.DECAL), pre), "one Ctrl+Z after close: the whole session"
    bpy.ops.ed.redo()
    assert _same(_state(H.DECAL), final)
    SUMMARY.append(f"fold: move {first_ms:.1f} ms, kind {_ms(kind_line):.1f} ms; undo and redo withdraw the preview before the step; close = last preview bit for bit, one undo = session")
    return low


def _redo_edits():
    """После отмены черновик мог вернуться: те же правки снова (перенос метки 2 и corner у метки 0)."""

    from cftuv import hotspot_editor as E

    E._RUNNING.add(-1)
    _edit(mark=2, y=420)
    _edit(mark=0, kind="corner")


def _check_discard_edit_mode_stale(low):
    from cftuv import hotspot_editor as E
    from cftuv import hotspot_live

    _start(H.TEXT)
    before = _state(H.DECAL)
    _edit(mark=2, y=448)
    assert not _same(_state(H.DECAL), before)
    _edit(mark=1, y=400)  # непринятый черновик (на метку 2)
    assert _editor().dirty and bpy.ops.hotspotuv.hotspot_markup_discard() == {"FINISHED"}
    assert _same(_state(H.DECAL), before) and hotspot_live.status().startswith("HOTSPOT_MARKUP_LIVE_WITHDRAWN: discard"), hotspot_live.status()

    H._decal_edit(None)  # декаль в Edit Mode: живой показ её не пишет и называет
    problems, line = _edit(mark=2, y=192)
    assert problems == () and f"HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE=1 ({bpy.data.objects[H.DECAL].data.name})" in line, line
    H._object_mode()
    assert np.array_equal(_state(H.DECAL)[0], before[0]), "the Edit Mode mesh is not written"
    _problems, line = _edit(mark=2, y=192)  # тот же текст после выхода - показывается
    assert "1 decal(s) previewed" in line and not np.array_equal(_state(H.DECAL)[0], before[0]), line

    H._decal_edit(low)  # другой оператор пишет меш поверх показа
    H._hotspot(True, 0)
    H._apply()
    H._object_mode()
    picked = _state(H.DECAL)
    E.HOTSPOTUV_OT_HotspotMarkupEdit._finish(types.SimpleNamespace(_area_pointer=-1, _area=types.SimpleNamespace(tag_redraw=lambda: None)), bpy.context)
    line = hotspot_live.status()
    assert "HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE=1" in line and _same(_state(H.DECAL), picked), line

    _start(H.TEXT)
    _edit(mark=0, y=0)  # метка 0 теперь у обоих островов
    E.HOTSPOTUV_OT_HotspotMarkupEdit._finish(types.SimpleNamespace(_area_pointer=-1, _area=types.SimpleNamespace(tag_redraw=lambda: None)), bpy.context)
    assert _same(_state(H.DECAL), picked) and hotspot_live.status().startswith("HOTSPOT_MARKUP_LIVE_WITHDRAWN: tool stopped"), hotspot_live.status()
    SUMMARY.append("discard and tool stop restore the snapshot; Edit Mode skipped by name and shown after Tab; a mesh another operator wrote is stale, kept")


def _check_both():
    B.T._clean()
    B._markup()
    B._hotspot(True, 0)
    cube, cube_bevel = B._source("cube")
    B._press(cube, cube_bevel)
    B._hotspot(True, 1)
    wall, wall_bevel = B._source("columns")
    B._press(wall, wall_bevel)
    decals = [cube.name + ".CFTUV_Decal", wall.name + ".CFTUV_Decal"]
    original = bpy.data.objects[decals[1]]
    for index in range(COPIES):  # ещё декали той же разметки (копии стены): время показа на сцене из нескольких декалей
        copy = original.copy()
        copy.data = original.data.copy()
        copy.name = f"{original.name}.copy{index}"
        bpy.context.scene.collection.objects.link(copy)
        decals.append(copy.name)
    loops = sum(len(bpy.data.objects[name].data.loops) for name in decals)
    from cftuv.hotspot_blender import mesh_record

    assert all(mesh_record(bpy.data.objects[name].data)[0].segments for name in decals), "Both decals: islands are segments"
    _start(B.TEXT)
    bpy.ops.ed.undo_push(message="Mark Up Both")
    pre = [_state(name) for name in decals]
    timings = []
    problems, line = _edit(mark=0, y=320)  # trim куба
    assert problems == () and "1 decal(s) previewed" in line and f"unchanged {len(decals) - 1}" in line, line
    timings.append(("trim moved", _ms(line)))
    assert not _same(_state(decals[0]), pre[0]) and all(_same(_state(name), state) for name, state in zip(decals[1:], pre[1:]))
    problems, line = _edit(mark=1, y=1280)  # corner стены и её копий
    assert problems == () and f"{len(decals) - 1} decal(s) previewed" in line and "same marks 1" in line, line
    timings.append(("corner moved", _ms(line)))
    problems, line = _edit(mark=1, up="+y")  # Swap-подобный разворот стрелки corner
    assert problems == () and f"{len(decals) - 1} decal(s) previewed" in line, line
    timings.append(("corner flipped", _ms(line)))
    live = [_state(name) for name in decals]
    assert all(not _same(a, b) for a, b in zip(live, pre))
    restored, note = _close(B.TEXT, decals)
    assert all(_same(a, b) for a, b in zip(restored, pre)), "close: every snapshot back"
    assert f"{len(decals)} decal(s) ({len(decals)} object(s)) updated" in note, note
    assert all(_same(_state(name), state) for name, state in zip(decals, live)), "the refresh on close is exactly the last live preview"
    bpy.ops.ed.undo()
    assert all(_same(_state(name), state) for name, state in zip(decals, pre)), "one Ctrl+Z: every decal before the session"
    SUMMARY.append(
        f"Both: {len(decals)} decals ({loops} loops; cube trim, wall corner + {COPIES} copies), per release "
        + ", ".join(f"{label} {ms:.1f} ms" for label, ms in timings) + "; preview = close bit for bit, one undo = session"
    )
    B.T._clean()


def main():
    H._register()
    _check_mark_switch(touch_first=False)
    _check_mark_switch(touch_first=True)
    low = _check_fold_session()
    _check_discard_edit_mode_stale(low)
    settings = B.T._settings()
    settings.envelope_debug_alpha, settings.envelope_debug_workers = B.T.ALPHA, 0
    _check_both()
    for line in SUMMARY:
        print("[HotspotLiveTest]", line, flush=True)
    from cftuv.envelope_domain_pool import shutdown_domain_pool

    shutdown_domain_pool()
    print("HOTSPOT_LIVE_BLENDER_OK", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:  # noqa: BLE001 - фоновый Blender: ненулевой код выхода
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        os._exit(1)
    sys.stdout.flush()
    os._exit(0)
