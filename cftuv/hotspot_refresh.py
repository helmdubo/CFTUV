"""Декали после правки разметки (HOTSPOT_REFRESH_ON_CLOSE_V1, HOTSPOT_MARK_FALLBACK_V1, HOTSPOT_MARK_ID_MONOTONIC_V1). Чистая часть -
`hotspot_islands.refresh_record`; здесь - меши `bpy.data`, Edit Mode, шаг отмены и строка состояния.

* Закрытие мини-окна (`cancel` инструмента меток) проверяет и пишет черновик; принятый - и только он - заказывает обновление декалей этой
  разметки таймером (`schedule_refresh`): `cancel` идёт внутри закрытия окна, где ни оператор, ни шаг отмены звать нельзя. Таймер зовёт
  оператор `hotspotuv.hotspot_refresh_markup` с `undo=True` в контексте главного окна: всё обновление - один шаг отмены (UNDO_REQUIRED_REASON:
  данные без шага отмены и затем Ctrl+Z - падение); ничего не изменилось - оператор CANCELLED, шага нет. Загрузка файла между закрытием и
  таймером обновление снимает (поколение файла).
* Декаль - меш объекта любой сцены (скрытый тоже) с записью `cftuv_hotspot` этой разметки; меш нескольких объектов - один раз. Связанный
  из библиотеки или override - пропуск HOTSPOT_REFRESH_SKIPPED_LINKED, запись не читается - HOTSPOT_REFRESH_SKIPPED_RECORD_INVALID. Меш в Edit Mode
  обновляется после выхода в Object Mode и в нём остаётся (HOTSPOT_REFRESH_EDIT_MODE_LEFT): шаг отмены Edit Mode вернул бы UV и коды
  граней, но не запись меша; остался в Edit Mode (не того окна) - HOTSPOT_REFRESH_SKIPPED_EDIT_MODE.
* Метка острова в разметке есть и её геометрия та же - UV острова побитово; другая геометрия того же вида - перенос рамки (точно) или, если
  в сессии есть UV ядра, отображение от них (точно); метки нет - замена `fallback_mark` и путь выбора метки (обращение, развёртка ряда шва).
  Таблица записи обновляется вместе с UV: ширина и пересборка дальше берут новые прямоугольники.
"""

from __future__ import annotations

import json

import bpy
import numpy as np
from bpy.app.handlers import persistent
from bpy.props import StringProperty

from .envelope_production_mesh import DECAL_UV_LAYER
from .hotspot_blender import (
    COUNTER_UNWARPED,
    LAST_COUNTERS,
    SETTINGS_ATTRIBUTE,
    current_atlas,
    exact_strip,
    face_codes,
    mesh_arrays,
    mesh_record,
    read_markup_text,
    remap_mesh,
    remembered_record,
)
from .hotspot_island_ops import build_strip, remap_path
from .hotspot_islands import (
    COUNTER_KEPT,
    COUNTER_MARKS_CHANGED,
    COUNTER_REMAPPED,
    FALLBACK_COUNTERS,
    changed_islands,
    known_codes,
    record_is_current,
    record_to_json,
    referenced_mark_ids,
    refresh_record,
)
from .hotspot_mapping import COUNTER_UNPAIRED, MESH_PROPERTY

HOTSPOT_REFRESH_SKIPPED_LINKED = "HOTSPOT_REFRESH_SKIPPED_LINKED"
HOTSPOT_REFRESH_SKIPPED_RECORD_INVALID = "HOTSPOT_REFRESH_SKIPPED_RECORD_INVALID"
HOTSPOT_REFRESH_SKIPPED_EDIT_MODE = "HOTSPOT_REFRESH_SKIPPED_EDIT_MODE"
SKIP_COUNTERS = (HOTSPOT_REFRESH_SKIPPED_LINKED, HOTSPOT_REFRESH_SKIPPED_RECORD_INVALID, HOTSPOT_REFRESH_SKIPPED_EDIT_MODE)
#: Декали этой разметки, которым нечего менять (метки те же): меш не тронут.
HOTSPOT_REFRESH_DECALS_UNCHANGED = "HOTSPOT_REFRESH_DECALS_UNCHANGED"
#: Объекты, вышедшие из Edit Mode для записи: остаются в Object Mode, один Ctrl+Z возвращает и Edit Mode, и прежние UV, коды и запись.
HOTSPOT_REFRESH_EDIT_MODE_LEFT = "HOTSPOT_REFRESH_EDIT_MODE_LEFT"
#: Обновление не заказано: черновик не принят (текст - прежний принятый) / текст разметки не читается.
HOTSPOT_REFRESH_SKIPPED_DRAFT_REFUSED = "HOTSPOT_REFRESH_SKIPPED_DRAFT_REFUSED"
HOTSPOT_REFRESH_SKIPPED_MARKUP_INVALID = "HOTSPOT_REFRESH_SKIPPED_MARKUP_INVALID"
HOTSPOT_REFRESH_FAILED = "HOTSPOT_REFRESH_FAILED"

_STATE: dict = {"generation": 0, "loading": False, "pending": []}


def _log(line: str) -> None:
    print(f"[CFTUV][Hotspot] {line}", flush=True)


def _linked(mesh) -> bool:
    return mesh.library is not None or getattr(mesh, "override_library", None) is not None or not getattr(mesh, "is_editable", True)


def _record_markup(mesh) -> str:
    """Имя разметки нечитаемой записи, если его видно (иначе пусто): пропуск считается только у своей разметки."""

    try:
        data = json.loads(str(mesh[MESH_PROPERTY]))
    except (TypeError, ValueError):
        return ""
    return str(data.get("markup", "")) if isinstance(data, dict) else ""


def decal_targets(text_name: str) -> tuple:
    """`([(меш, объекты, запись)], {пропуск: [имена]})`: меши объектов `bpy.data` с записью разметки `text_name`, каждый один раз."""

    users: dict = {}
    for obj in bpy.data.objects:
        if obj.type == "MESH" and obj.data is not None and MESH_PROPERTY in obj.data.keys():
            users.setdefault(obj.data.as_pointer(), (obj.data, []))[1].append(obj)
    targets, skipped = [], {name: [] for name in SKIP_COUNTERS}
    for mesh, objects in users.values():
        record, error = mesh_record(mesh)
        if error:
            if _record_markup(mesh) in ("", text_name):
                skipped[HOTSPOT_REFRESH_SKIPPED_RECORD_INVALID].append(mesh.name)
            continue
        if record is None or record.markup != text_name:
            continue
        if _linked(mesh):
            skipped[HOTSPOT_REFRESH_SKIPPED_LINKED].append(mesh.name)
            continue
        targets.append((mesh, objects, record))
    return targets, skipped


def plan_mesh(mesh, record, markup) -> tuple:
    """`(новая запись, коды до, коды после, числа)` декали или `None` - менять нечего (запись и коды те же)."""

    _vertices, totals, group = mesh_arrays(mesh)
    codes = face_codes(mesh, len(totals))
    codes = known_codes(record, np.full(len(totals), record.default, dtype=np.int64) if codes is None else codes)
    fresh, new_codes, counters = refresh_record(record, markup, group, codes)
    if fresh == record and np.array_equal(new_codes, codes):
        return None
    return fresh, codes, new_codes, counters


def refresh_mesh(mesh, record, markup, source=None) -> dict | None:
    """Обновить декаль по разметке (Object Mode): UV островов изменившихся и ушедших меток, коды граней и запись; `None` - менять нечего."""

    plan = plan_mesh(mesh, record, markup)
    if plan is None:
        return None
    fresh, old_codes, new_codes, counters = plan
    _vertices, totals, group = mesh_arrays(mesh)
    before = current_atlas(mesh, DECAL_UV_LAYER).copy()
    kernel = build_strip(mesh, record, old_codes)
    numbers = {}
    if kernel is None:
        strip, numbers = exact_strip(mesh, record, old_codes, source, DECAL_UV_LAYER)
    else:
        strip = kernel
    counters.update(remap_mesh(mesh, record, fresh, old_codes, new_codes, DECAL_UV_LAYER, strip, transfer=kernel is None))
    counters.update(numbers)
    counters[remap_path(kernel is not None, numbers.get(COUNTER_UNWARPED, 0))] = 1
    mesh[MESH_PROPERTY] = record_to_json(remembered_record(mesh, fresh, DECAL_UV_LAYER))
    counters[COUNTER_REMAPPED], counters[COUNTER_KEPT] = changed_islands(before, current_atlas(mesh, DECAL_UV_LAYER), np.repeat(group, totals))
    LAST_COUNTERS[mesh.session_uid] = counters
    mesh.update()
    return counters


def refresh_decals(context, text_name: str) -> tuple:
    """`(уровень отчёта, строка, изменено ли)`: все декали разметки `text_name` по её тексту; одна строка с именованными числами."""

    markup, problems = read_markup_text(text_name)
    if markup is None or problems:
        detail = "; ".join(problem.text() for problem in problems)
        return "WARNING", f"{HOTSPOT_REFRESH_SKIPPED_MARKUP_INVALID}: markup {text_name!r} does not load: {detail}"[:400], False
    targets, skipped = decal_targets(text_name)
    # Только запись: у меша в Edit Mode атрибуты граней не читаются (данные в BMesh), поэтому выход из Edit Mode - до чтения кодов.
    wanted = [(mesh, objects, record) for mesh, objects, record in targets if not record_is_current(record, markup)]
    unchanged = len(targets) - len(wanted)
    left = 0
    if any(mesh.is_editmode for mesh, _objects, _record in wanted) and getattr(context, "mode", "") == "EDIT_MESH":
        left = sum(1 for obj in context.view_layer.objects if obj.mode == "EDIT")
        bpy.ops.object.mode_set(mode="OBJECT")
    done = []
    # В Edit Mode НЕ возвращаемся: шаг отмены Edit Mode - шаг BMesh, он вернул бы UV и коды граней, но не запись меша (ID-свойство), и
    # таблица записи разошлась бы с кодами (удалённая метка, старый прямоугольник). Шаг в Object Mode - memfile: один Ctrl+Z возвращает всё
    # вместе и сам входит в Edit Mode (полевой прогон HOTSPOT_REFRESH_ON_CLOSE_V1).
    for mesh, objects, record in wanted:
        if mesh.is_editmode:
            skipped[HOTSPOT_REFRESH_SKIPPED_EDIT_MODE].append(mesh.name)
            continue
        counters = refresh_mesh(mesh, record, markup, objects[0].parent)
        if counters is None:
            unchanged += 1
            continue
        done.append((mesh, objects, counters))
        _log(f"{mesh.name}: " + ", ".join(f"{key}={value}" for key, value in counters.items()))
    line = status_line(text_name, done, unchanged, skipped, left)
    settings = getattr(context.scene, SETTINGS_ATTRIBUTE, None)
    if settings is not None and done:
        settings.apply_note = line[:400]
    _log(line)
    return ("WARNING" if any(skipped.values()) else "INFO"), line, bool(done)


def status_line(text_name: str, done, unchanged: int, skipped: dict, left: int = 0) -> str:
    """Одна строка: декали (объекты) и острова обновлены, пропуски и замены - итогом и каждый ненулевой счёт по имени (пропуски - с именами
    мешей); полные числа каждого меша - строкой консоли `refresh_decals`."""

    total = lambda key: sum(int(counters.get(key, 0)) for _mesh, _objects, counters in done)  # noqa: E731
    skips = [f"{name}={len(names)} ({', '.join(names)[:80]})" for name, names in skipped.items() if names]
    fallback = [f"{name}={total(name)}" for name in FALLBACK_COUNTERS if total(name)]
    extra = [f"{name}={value}" for name, value in ((COUNTER_MARKS_CHANGED, total(COUNTER_MARKS_CHANGED)), (COUNTER_UNPAIRED, total(COUNTER_UNPAIRED)), (HOTSPOT_REFRESH_EDIT_MODE_LEFT, left)) if value]
    return "; ".join([
        f"Hotspot {text_name!r}: {len(done)} decal(s) ({sum(len(objects) for _mesh, objects, _counters in done)} object(s)) updated, "
        f"{total(COUNTER_REMAPPED)} island(s) remapped, {total(COUNTER_KEPT)} kept bitwise, {HOTSPOT_REFRESH_DECALS_UNCHANGED}={unchanged}",
        f"skipped {sum(len(names) for names in skipped.values())}" + (": " + ", ".join(skips) if skips else ""),
        f"fallback {sum(total(name) for name in FALLBACK_COUNTERS)} island(s)" + (": " + ", ".join(fallback) if fallback else ""),
        *extra,
    ])


def record_mark_floor(text_name: str) -> int:
    """Пол номера новой метки разметки `text_name` (HOTSPOT_MARK_ID_MONOTONIC_V1): наибольший номер, на который ссылаются записи и коды граней
    мешей файла этой разметки, плюс один (у разметки без счётчика это единственная память об удалённых метках)."""

    ids: set = set()
    for mesh in bpy.data.meshes:
        if MESH_PROPERTY not in mesh.keys():
            continue
        record, _error = mesh_record(mesh)
        if record is None or record.markup != text_name:
            continue
        codes = face_codes(mesh, len(mesh.polygons))
        ids |= referenced_mark_ids(record, () if codes is None else codes)
    return max(ids) + 1 if ids else 0


# --------------------------------------------------------------------------
# Закрытие окна -> таймер -> оператор с шагом отмены
# --------------------------------------------------------------------------


def schedule_refresh(text_name: str, problems=()) -> str:
    """Закрыто окно разметки `text_name`: принятый черновик заказывает обновление декалей таймером; непринятый - строка отказа (пусто - заказано)."""

    if _STATE["loading"] or not text_name:
        return ""
    if problems:
        line = f"{HOTSPOT_REFRESH_SKIPPED_DRAFT_REFUSED}: the draft of {text_name!r} is not accepted, decals keep the last accepted markup: " + "; ".join(problem.text() for problem in problems)
        _log(line)
        return line
    _STATE["pending"].append((text_name, _STATE["generation"]))
    if not bpy.app.timers.is_registered(run_pending):
        bpy.app.timers.register(run_pending, first_interval=0.0)
    return ""


def _main_window():
    manager = bpy.context.window_manager
    windows = [] if manager is None else [window for window in manager.windows if window.screen is not None and not window.screen.is_temporary]
    return windows[0] if windows else None


def run_pending():
    """Таймер: заказанные обновления того же файла, каждое - оператор `hotspotuv.hotspot_refresh_markup` с шагом отмены."""

    pending, _STATE["pending"] = _STATE["pending"], []
    for text_name, generation in dict.fromkeys(pending):
        if generation != _STATE["generation"]:
            continue
        window = _main_window()
        try:
            if window is None:
                bpy.ops.hotspotuv.hotspot_refresh_markup("EXEC_DEFAULT", True, text_name=text_name)
            else:
                with bpy.context.temp_override(window=window):
                    bpy.ops.hotspotuv.hotspot_refresh_markup("EXEC_DEFAULT", True, text_name=text_name)
        except RuntimeError as exc:
            _log(f"{HOTSPOT_REFRESH_FAILED}: {text_name!r}: {exc}")
    return None


class HOTSPOTUV_OT_HotspotRefreshMarkup(bpy.types.Operator):
    bl_idname = "hotspotuv.hotspot_refresh_markup"
    bl_label = "Refresh Hotspot Decals"
    bl_description = "Remap every decal of this markup to its current marks (changed marks moved, deleted marks replaced); one undo step"
    bl_options = {"UNDO", "INTERNAL"}

    text_name: StringProperty(name="Markup")

    def execute(self, context):
        level, line, changed = refresh_decals(context, self.text_name)
        self.report({level}, line)
        return {"FINISHED"} if changed else {"CANCELLED"}


@persistent
def _load_pre(*_args) -> None:
    _STATE["loading"] = True
    _STATE["pending"].clear()


@persistent
def _load_post(*_args) -> None:
    _STATE["loading"] = False
    _STATE["generation"] += 1


def register_refresh() -> None:
    if getattr(HOTSPOTUV_OT_HotspotRefreshMarkup, "is_registered", False):
        bpy.utils.unregister_class(HOTSPOTUV_OT_HotspotRefreshMarkup)
    bpy.utils.register_class(HOTSPOTUV_OT_HotspotRefreshMarkup)
    for handlers, handler in ((bpy.app.handlers.load_pre, _load_pre), (bpy.app.handlers.load_post, _load_post)):
        if handler not in handlers:
            handlers.append(handler)


def unregister_refresh() -> None:
    for handlers, handler in ((bpy.app.handlers.load_pre, _load_pre), (bpy.app.handlers.load_post, _load_post)):
        if handler in handlers:
            handlers.remove(handler)
    if bpy.app.timers.is_registered(run_pending):
        bpy.app.timers.unregister(run_pending)
    _STATE["pending"].clear()
    if getattr(HOTSPOTUV_OT_HotspotRefreshMarkup, "is_registered", False):
        bpy.utils.unregister_class(HOTSPOTUV_OT_HotspotRefreshMarkup)


__all__ = (
    "HOTSPOTUV_OT_HotspotRefreshMarkup",
    "HOTSPOT_REFRESH_DECALS_UNCHANGED",
    "HOTSPOT_REFRESH_EDIT_MODE_LEFT",
    "HOTSPOT_REFRESH_FAILED",
    "HOTSPOT_REFRESH_SKIPPED_DRAFT_REFUSED",
    "HOTSPOT_REFRESH_SKIPPED_EDIT_MODE",
    "HOTSPOT_REFRESH_SKIPPED_LINKED",
    "HOTSPOT_REFRESH_SKIPPED_MARKUP_INVALID",
    "HOTSPOT_REFRESH_SKIPPED_RECORD_INVALID",
    "SKIP_COUNTERS",
    "decal_targets",
    "plan_mesh",
    "record_mark_floor",
    "refresh_decals",
    "refresh_mesh",
    "register_refresh",
    "run_pending",
    "schedule_refresh",
    "status_line",
    "unregister_refresh",
)
