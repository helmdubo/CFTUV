"""Живой показ разметки на декалях, пока открыто окно MarkUp (HOTSPOT_MARKUP_LIVE_ON_RELEASE_V1). Обновление с шагом отмены остаётся одно -
на закрытии окна (`hotspot_refresh`, HOTSPOT_REFRESH_ON_CLOSE_V1); живой показ - превью без шага отмены, и всё здесь - о том, чтобы оно не
попало в историю отмены.

* Каждая законченная правка инструмента меток (отпускание кнопки, клавиши, меню метки) после принятого черновика кладёт текст разметки на
  её декали той же функцией, что и закрытие (`hotspot_refresh.refresh_mesh`), но всегда ОТ СНИМКА: перед первой живой записью меша в
  сессии снимается то, что пишет обновление (UV слоя декали, коды граней `cftuv_hotspot_mark` или их отсутствие, запись `cftuv_hotspot`), и
  каждая следующая правка сначала возвращает снимок. Поэтому превью - ровно то, что даст закрытие, при любом пути правок. Текст тот же, что
  при прошлом показе, - ничего не делается.
* Закрытие окна, сброс черновика, загрузка другого черновика, остановка инструмента - снимок возвращается (`withdraw`), затем закрытие, как и
  раньше, заказывает обновление одним шагом отмены от состояния до сессии: один Ctrl+Z - вся сессия.
* Ctrl+Z / Ctrl+Shift+Z в любом окне: обработчики `undo_pre` / `redo_pre` возвращают снимок ДО шага отмены (HOTSPOT_MARKUP_LIVE_WITHDRAWN,
  причина undo/redo). Без этого шаг отмены оставлял бы превью: фоновый Blender 4.5 показал, что записи UV и ID-свойства меша без шага отмены
  переживают Ctrl+Z и Ctrl+Shift+Z (меш берётся из памяти как «не изменившийся»), и следующий шаг вписал бы превью в историю. Следующая
  правка показывает разметку снова.
* Меш снимка правили не мы (другой оператор, пересборка, отмена): запись не та, что мы писали, - снимок не возвращается и забывается
  (HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE), его состояние - новое «до». Меш в Edit Mode живой показ не пишет (HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE:
  из Edit Mode без шага отмены не выйти; закрытие обновит его, как раньше); вошёл в Edit Mode после показа - снимок не вернуть
  (HOTSPOT_MARKUP_LIVE_KEPT_EDIT_MODE, данные меша согласованы: превью). Связанные, override и нечитаемые записи - правила `decal_targets`.
* Ссылок на данные Blender между событиями нет: меш ищется по `session_uid` (после отмены прежняя ссылка Python - висячая).
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import bpy
import numpy as np
from bpy.app.handlers import persistent

from .envelope_production_mesh import DECAL_UV_LAYER
from .hotspot_blender import _uv_store, current_atlas, mesh_record, read_markup_text
from .hotspot_islands import COUNTER_KEPT, COUNTER_REMAPPED, MARK_ATTRIBUTE, record_is_current
from .hotspot_mapping import MESH_PROPERTY
from .hotspot_markup import WHOLE_TEXTURE_MARK_ID
from .hotspot_refresh import HOTSPOT_REFRESH_SKIPPED_MARKUP_INVALID, decal_targets, refresh_mesh

HOTSPOT_MARKUP_LIVE_APPLIED = "HOTSPOT_MARKUP_LIVE_APPLIED"
#: Черновик не принят: декали показывают последнюю принятую разметку (текст), как и закрытие.
HOTSPOT_MARKUP_LIVE_DRAFT_REFUSED = "HOTSPOT_MARKUP_LIVE_DRAFT_REFUSED"
HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE = "HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE"
#: Атрибут кодов граней есть, но не целый атрибут граней своего размера: снимок его не вернул бы - меш не пишется.
HOTSPOT_MARKUP_LIVE_SKIPPED_FOREIGN_CODES = "HOTSPOT_MARKUP_LIVE_SKIPPED_FOREIGN_CODES"
HOTSPOT_MARKUP_LIVE_FAILED = "HOTSPOT_MARKUP_LIVE_FAILED"
HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE = "HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE"
HOTSPOT_MARKUP_LIVE_WITHDRAWN = "HOTSPOT_MARKUP_LIVE_WITHDRAWN"
HOTSPOT_MARKUP_LIVE_KEPT_EDIT_MODE = "HOTSPOT_MARKUP_LIVE_KEPT_EDIT_MODE"
LIVE_SKIPS = (HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE, HOTSPOT_MARKUP_LIVE_SKIPPED_FOREIGN_CODES, HOTSPOT_MARKUP_LIVE_FAILED)
#: Причины возврата снимка.
WITHDRAW_CLOSE, WITHDRAW_DISCARD, WITHDRAW_RELOAD, WITHDRAW_STOPPED, WITHDRAW_UNDO, WITHDRAW_REDO = "close", "discard", "reload", "tool stopped", "undo", "redo"


@dataclass
class LiveSnapshot:
    """То, что пишет обновление декали, до первой живой записи сессии; `written` - запись, которую положил последний показ (пусто - меш в снимке)."""

    name: str
    faces: int
    uv: np.ndarray
    codes: np.ndarray | None
    record: str
    marks: tuple = ()
    written: str = ""
    written_uv: np.ndarray | None = None
    inputs: tuple | None = None


#: Сессия живого показа: разметка, снимки по `session_uid` меша, текст последнего показа и его строка.
_LIVE: dict = {"text": "", "meshes": {}, "applied": "", "line": ""}


def _log(line: str) -> None:
    print(f"[CFTUV][Hotspot] {line}", flush=True)


def status() -> str:
    """Строка последнего показа или возврата (внизу мини-окна); пусто - показов не было."""

    return str(_LIVE["line"])


def _mesh_of(uid: int):
    return next((mesh for mesh in bpy.data.meshes if mesh.session_uid == uid), None)


def _take(mesh) -> LiveSnapshot | None:
    attribute = mesh.attributes.get(MARK_ATTRIBUTE)
    codes = None
    if attribute is not None:
        if attribute.domain != "FACE" or attribute.data_type != "INT" or len(attribute.data) != len(mesh.polygons):
            return None
        codes = np.empty(len(attribute.data), dtype=np.int32)
        attribute.data.foreach_get("value", codes)
    record, _error = mesh_record(mesh)
    marks = () if record is None else tuple(int(mark.id) for mark in record.marks)
    return LiveSnapshot(mesh.name, len(mesh.polygons), current_atlas(mesh, DECAL_UV_LAYER).copy(), codes, str(mesh[MESH_PROPERTY]), marks)


def _inputs(snap: LiveSnapshot, markup) -> tuple | None:
    """Всё, от чего зависит обновление меша снимка: размер текстуры и метки таблицы его записи; `None` - метки нет (замена `fallback_mark`
    смотрит на всю разметку) или псевдометка всей текстуры. Те же входы, что у прошлого показа, - тот же показ побитово."""

    marks = tuple(markup.mark(mark_id) for mark_id in snap.marks)
    if WHOLE_TEXTURE_MARK_ID in snap.marks or any(mark is None for mark in marks):
        return None
    return int(markup.width), int(markup.height), marks


def _holds_preview(mesh, snap: LiveSnapshot) -> bool:
    """Меш всё ещё показывает ровно наш последний показ: запись и UV те, что мы писали, число углов и граней то же (живая ширина, пересборка,
    выбор метки меняют хоть что-то из этого)."""

    if mesh is None or str(mesh.get(MESH_PROPERTY, "")) != snap.written or (len(mesh.loops), len(mesh.polygons)) != (snap.uv.shape[0], snap.faces):
        return False
    return snap.written_uv is not None and np.array_equal(current_atlas(mesh, DECAL_UV_LAYER), snap.written_uv)


def _restore(mesh, snap: LiveSnapshot) -> None:
    store, field = _uv_store(mesh, DECAL_UV_LAYER)
    store.foreach_set(field, snap.uv.reshape(-1))
    attribute = mesh.attributes.get(MARK_ATTRIBUTE)
    if snap.codes is None:
        if attribute is not None:
            mesh.attributes.remove(attribute)
    else:
        if attribute is None or attribute.domain != "FACE" or attribute.data_type != "INT":
            if attribute is not None:
                mesh.attributes.remove(attribute)
            attribute = mesh.attributes.new(name=MARK_ATTRIBUTE, type="INT", domain="FACE")
        attribute.data.foreach_set("value", snap.codes)
    mesh[MESH_PROPERTY] = snap.record
    snap.written, snap.written_uv, snap.inputs = "", None, None
    mesh.update()


def _baseline(mesh, stale: list):
    """Меш в состоянии до сессии: снимок снят (первая запись) или возвращён; `None` - снимок не снять (чужой атрибут кодов)."""

    snap = _LIVE["meshes"].get(mesh.session_uid)
    if snap is not None and snap.written and not _holds_preview(mesh, snap):
        stale.append(mesh.name)  # меш правили не мы: его состояние - новое «до»
        snap = None
    if snap is None:
        snap = _take(mesh)
        if snap is None:
            return None
        _LIVE["meshes"][mesh.session_uid] = snap
    elif snap.written:
        _restore(mesh, snap)
    return snap


def preview(text_name: str, problems=()) -> str:
    """Законченная правка черновика разметки `text_name`: принятый текст - на её декали от снимков; строка исхода (она же `status`)."""

    if _LIVE["text"] and _LIVE["text"] != text_name:
        withdraw(WITHDRAW_RELOAD)
    if problems:
        _LIVE["line"] = f"{HOTSPOT_MARKUP_LIVE_DRAFT_REFUSED}: decals show the last accepted markup of {text_name!r}"
        return _LIVE["line"]
    text = bpy.data.texts.get(text_name)
    key = "" if text is None else text.as_string()
    if key and key == _LIVE["applied"]:
        return _LIVE["line"]
    markup, invalid = read_markup_text(text_name)
    if markup is None or invalid:
        _LIVE["line"] = f"{HOTSPOT_REFRESH_SKIPPED_MARKUP_INVALID}: markup {text_name!r} does not load"
        return _LIVE["line"]
    started = time.perf_counter()
    _LIVE["text"] = text_name
    targets, skipped = decal_targets(text_name)
    skipped.update({name: [] for name in LIVE_SKIPS})
    stale, done, counts = [], [], {"unchanged": 0, "kept": 0}
    for mesh, objects, _record in targets:
        outcome = _preview_mesh(mesh, objects[0].parent, markup, skipped, stale)
        if isinstance(outcome, dict):
            done.append(outcome)
        elif outcome:
            counts[outcome] += 1
    # Пропущенный (Edit Mode, сбой) меш ждёт следующей правки: тот же текст тогда показывается снова, а не считается показанным.
    _LIVE["applied"] = "" if skipped[HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE] or skipped[HOTSPOT_MARKUP_LIVE_FAILED] else key
    _LIVE["line"] = _line(text_name, done, counts, skipped, stale, time.perf_counter() - started)
    _log(_LIVE["line"])
    return _LIVE["line"]


def _preview_mesh(mesh, source, markup, skipped: dict, stale: list):
    """Один меш: числа обновления (показ записан), `"kept"` - входы те же, что у его показа (меш не тронут), `"unchanged"` - менять от снимка
    нечего, пусто - пропуск по имени в `skipped`."""

    if mesh.is_editmode:
        skipped[HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE].append(mesh.name)
        return ""
    known = _LIVE["meshes"].get(mesh.session_uid)
    inputs = None if known is None else _inputs(known, markup)
    if known is not None and known.written and inputs is not None and inputs == known.inputs and _holds_preview(mesh, known):
        return "kept"  # правка не его меток: тот же показ, ничего не пишется
    snap = _baseline(mesh, stale)
    if snap is None:
        skipped[HOTSPOT_MARKUP_LIVE_SKIPPED_FOREIGN_CODES].append(mesh.name)
        return ""
    record, _error = mesh_record(mesh)
    try:
        counters = None if record is None or record_is_current(record, markup) else refresh_mesh(mesh, record, markup, source)
    except Exception as exc:  # noqa: BLE001 - показ не роняет инструмент: меш назад в снимок, причина по имени
        _restore(mesh, snap)
        skipped[HOTSPOT_MARKUP_LIVE_FAILED].append(f"{mesh.name}: {type(exc).__name__}: {exc}")
        return ""
    if counters is None:
        return "unchanged"
    snap.written, snap.written_uv, snap.inputs = str(mesh[MESH_PROPERTY]), current_atlas(mesh, DECAL_UV_LAYER).copy(), _inputs(snap, markup)
    return counters


def _line(text_name, done, counts, skipped, stale, seconds) -> str:
    skips = [f"{name}={len(names)} ({', '.join(names)[:80]})" for name, names in skipped.items() if names]
    total = lambda key: sum(int(counters.get(key, 0)) for counters in done)  # noqa: E731
    return "; ".join([
        f"{HOTSPOT_MARKUP_LIVE_APPLIED}: {text_name!r}: {len(done)} decal(s) previewed in {seconds * 1000.0:.1f} ms, {total(COUNTER_REMAPPED)} island(s) "
        f"remapped, {total(COUNTER_KEPT)} kept bitwise, unchanged {counts['unchanged']}, same marks {counts['kept']} (one undo step on close)",
        *skips,
        *([f"{HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE}={len(stale)} ({', '.join(stale)[:80]})"] if stale else []),
    ])


def withdraw(reason: str) -> str:
    """Превью сессии -> снимки (данные мешей - как до сессии); сессия забыта. Строка исхода по имени; пусто - нечего было возвращать."""

    snaps, text_name = _LIVE["meshes"], _LIVE["text"]
    _LIVE.update(text="", meshes={}, applied="")
    restored, stale, kept = 0, [], []
    for uid, snap in snaps.items():
        if not snap.written:
            continue
        mesh = _mesh_of(uid)
        if mesh is not None and mesh.is_editmode:
            kept.append(mesh.name)
        elif not _holds_preview(mesh, snap):
            stale.append(snap.name)
        else:
            _restore(mesh, snap)
            restored += 1
    if not (restored or stale or kept):
        return ""
    line = "; ".join([
        f"{HOTSPOT_MARKUP_LIVE_WITHDRAWN}: {reason}: {restored} decal(s) of {text_name!r} back to their state before the live preview",
        *([f"{HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE}={len(stale)} ({', '.join(stale)[:80]})"] if stale else []),
        *([f"{HOTSPOT_MARKUP_LIVE_KEPT_EDIT_MODE}={len(kept)} ({', '.join(kept)[:80]})"] if kept else []),
    ])
    _LIVE["line"] = line
    _log(line)
    return line


@persistent
def _undo_pre(*_args) -> None:
    withdraw(WITHDRAW_UNDO)


@persistent
def _redo_pre(*_args) -> None:
    withdraw(WITHDRAW_REDO)


@persistent
def _load_pre(*_args) -> None:
    _LIVE.update(text="", meshes={}, applied="", line="")


_HANDLERS = (("undo_pre", _undo_pre), ("redo_pre", _redo_pre), ("load_pre", _load_pre))


def register_live() -> None:
    for name, handler in _HANDLERS:
        handlers = getattr(bpy.app.handlers, name)
        if handler not in handlers:
            handlers.append(handler)


def unregister_live() -> None:
    for name, handler in _HANDLERS:
        handlers = getattr(bpy.app.handlers, name)
        if handler in handlers:
            handlers.remove(handler)
    _LIVE.update(text="", meshes={}, applied="", line="")


__all__ = (
    "HOTSPOT_MARKUP_LIVE_APPLIED",
    "HOTSPOT_MARKUP_LIVE_DRAFT_REFUSED",
    "HOTSPOT_MARKUP_LIVE_FAILED",
    "HOTSPOT_MARKUP_LIVE_KEPT_EDIT_MODE",
    "HOTSPOT_MARKUP_LIVE_SKIPPED_EDIT_MODE",
    "HOTSPOT_MARKUP_LIVE_SKIPPED_FOREIGN_CODES",
    "HOTSPOT_MARKUP_LIVE_SNAPSHOT_STALE",
    "HOTSPOT_MARKUP_LIVE_WITHDRAWN",
    "LIVE_SKIPS",
    "LiveSnapshot",
    "WITHDRAW_CLOSE",
    "WITHDRAW_DISCARD",
    "WITHDRAW_RELOAD",
    "WITHDRAW_REDO",
    "WITHDRAW_STOPPED",
    "WITHDRAW_UNDO",
    "preview",
    "register_live",
    "status",
    "unregister_live",
    "withdraw",
)
