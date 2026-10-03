"""Оператор «Build Decal Mesh»: продуктовый путь Envelope до объекта Blender.

Тонкая обёртка. Что считается, решает `envelope_production_export`
(`run_production`: тёплая сессия, пул воркеров, названные исходы), что
пишется — `envelope_production_mesh` (`write_decal_object`: один объект,
UV, атрибуты граней). Здесь только Blender: выделение, настройки, регистрация и
строки статуса.

ВЫДЕЛЕНИЕ, ПЛОТНОСТЬ, ДОПУСК РАСТЯЖЕНИЯ, ALPHA, ВОРКЕРЫ — те же настройки панели Envelope Debug
(`envelope_debug_alpha`, `envelope_debug_fan_density`, `envelope_debug_max_stretch`,
`envelope_debug_workers`),
и та же сессия контроллера на окне: нажатие сразу после отладочной кнопки
берёт готовые подготовки из её кэша, не собирая ни одной. Движок отладки
(LEGACY/QUEUE) не читается: продуктовый путь — очередь.

Регистрация живёт ЗДЕСЬ, а не в `operators.py` (тот стоит на потолке размера):
`register_production_operator` зовётся из хука регистрации сессии, рядом с
настройкой «Worker Python».

ПОЧЕМУ ФЛАГ UNDO ОБЯЗАТЕЛЕН (`UNDO_REQUIRED_REASON`). Оператор работает, пока
источник в EDIT-режиме, и создаёт датаблоки (объект, меш, материал). Без шага отмены
после него следующий Ctrl+Z декодирует последний MEMFILE-шаг (Blender записал его при
входе в EDIT) с повторным использованием «неизменившихся» ID: сцена и коллекция
берутся как есть и всё ещё ссылаются на декаль, а объекта декали в том memfile нет —
он освобождается, и Blender падает на висячем указателе в депсграфе
(`DepsgraphNodeBuilder::build_materials`) либо в аутлайнере. Шаг отмены оператора —
даже BMesh-шаг в EDIT-режиме — лечит это: `BKE_undosys_step_push` видит, что Main
менялся после последней записи memfile (`is_memfile_undo_written`), и кладёт перед
BMesh-шагом внутренний memfile-шаг с декалью; один Ctrl+Z убирает декаль, Ctrl+Shift+Z
возвращает. Прежняя причина снятия флага («шаг BMesh не отслеживает создание
объекта») была проверена только фоновым Blender из OBJECT-режима и была неверна:
воспроизведение в UI Blender 4.5 — падение без флага, чистая отмена с флагом
(DECISIONS, 2026-10-03). То же правило — у кнопок отладки Envelope (`operators.py`).
"""

from __future__ import annotations

import bmesh
import bpy
from bpy.props import FloatProperty, PointerProperty, StringProperty

from .analysis import build_analysis_bundle
from .analysis_surface import source_revision_from_bmesh
from .envelope_production_mesh import (
    DEFAULT_DECAL_MATERIAL,
    DEFAULT_DECAL_OFFSET,
    ProductionWriteError,
    write_decal_object,
)
from .envelope_source_preflight import reject_source, zero_length_edge_refusal

SETTINGS_ATTRIBUTE = "hotspotuv_decal_mesh"
UNDO_REQUIRED_REASON = (
    "the operator creates datablocks while the source is in EDIT mode; without an "
    "undo step the next undo reuses the scene unchanged and frees the decal object "
    "(dangling pointer, crash in the depsgraph); the pushed step makes Blender write "
    "the memfile that holds the decal"
)


class HOTSPOTUV_DecalMeshSettings(bpy.types.PropertyGroup):
    offset: FloatProperty(
        name="Decal Offset",
        default=DEFAULT_DECAL_OFFSET,
        min=0.0,
        soft_max=0.5,
        unit="LENGTH",
        description=(
            "Lift of the decal mesh above the surface along the patch normal "
            "(z-fighting policy of the host; the mesh itself lies on the "
            "certified plane of every patch)"
        ),
    )
    material_name: StringProperty(
        name="Decal Material",
        default=DEFAULT_DECAL_MATERIAL,
        description=(
            "Material of the single slot; created when missing and never "
            "overwritten when it exists"
        ),
    )
    status: StringProperty(name="Decal Mesh Status", default="")
    timing: StringProperty(name="Decal Mesh Timing", default="")


def _session(context):
    from .envelope_debug_session import (
        WINDOW_MANAGER_SESSION_ATTRIBUTE,
        EnvelopeDebugSessionController,
    )

    window_manager = context.window_manager
    controller = getattr(window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, None)
    if not isinstance(controller, EnvelopeDebugSessionController):
        controller = EnvelopeDebugSessionController()
        setattr(window_manager, WINDOW_MANAGER_SESSION_ATTRIBUTE, controller)
    return controller


def _runtime_key(value):
    as_pointer = getattr(value, "as_pointer", None)
    return int(as_pointer()) if callable(as_pointer) else id(value)


def _restore_edge_selection(obj, indices) -> None:
    if obj is None or obj.name not in bpy.data.objects or obj.mode != "EDIT":
        return
    wanted = {int(index) for index in indices}
    bm = bmesh.from_edit_mesh(obj.data)
    bm.edges.ensure_lookup_table()
    for edge in bm.edges:
        edge.select = edge.index in wanted
    bmesh.update_edit_mesh(obj.data)


class HOTSPOTUV_OT_BuildEnvelopeDecalMesh(bpy.types.Operator):
    bl_idname = "hotspotuv.build_envelope_decal_mesh"
    bl_label = "Build Decal Mesh"
    bl_description = (
        "Materialize the exact Envelope coverage of the selected seams into "
        "one decal mesh object with UVs; every refused domain is named"
    )
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        obj = context.active_object
        return (
            obj is not None
            and obj.type == "MESH"
            and obj.mode == "EDIT"
            and bool(context.tool_settings.mesh_select_mode[1])
        )

    def execute(self, context):
        from .envelope_production_export import (
            production_timing_text,
            receipt_console_lines,
            receipt_report_level,
            receipt_status_text,
            run_production,
        )
        from .envelope_request_policy import envelope_stretch_budget

        settings = context.scene.hotspotuv_settings
        mesh_settings = getattr(context.scene, SETTINGS_ATTRIBUTE)
        source_obj = context.active_object
        mesh_settings.status = "Building decal mesh..."
        mesh_settings.timing = ""
        source_bm = bmesh.from_edit_mesh(source_obj.data)
        source_bm.edges.ensure_lookup_table()
        selected = [edge.index for edge in source_bm.edges if edge.select]
        if not selected:
            mesh_settings.status = "Failed: select a whole PhysicalChain"
            self.report({"WARNING"}, "ENVELOPE_DEBUG_EMPTY_SELECTION")
            return {"CANCELLED"}
        # Предполёт источника: ребро нулевой длины называется здесь, до анализа и
        # расчёта, и остаётся выделенным; хост ничего не сваривает молча.
        refusal = zero_length_edge_refusal(source_bm, selected)
        if refusal is not None:
            mesh_settings.status = f"Failed: {refusal.message}"
            return reject_source(self, context, source_obj, source_bm, refusal)
        controller = _session(context)
        source_object_key = _runtime_key(source_obj)
        source_data_key = _runtime_key(source_obj.data)
        source_bm.faces.ensure_lookup_table()
        face_indices = tuple(face.index for face in source_bm.faces)
        source_revision = source_revision_from_bmesh(
            source_bm, source_obj, face_indices
        )
        try:
            bundle = controller.get_analysis_bundle(
                source_object_key,
                source_data_key,
                source_revision,
                lambda: build_analysis_bundle(source_bm, face_indices, source_obj),
            )
            run = run_production(
                controller,
                bundle,
                frozenset(selected),
                float(settings.envelope_debug_alpha),
                source_object_key=source_object_key,
                source_data_key=source_data_key,
                density=settings.envelope_debug_fan_density,
                developable_stretch_budget=envelope_stretch_budget(settings.envelope_debug_max_stretch),
                workers=settings.envelope_debug_workers,
            )
        except Exception as exc:  # noqa: BLE001 - причина идёт владельцу
            mesh_settings.status = f"Decal mesh failed: {type(exc).__name__}"
            self.report({"ERROR"}, f"Envelope decal mesh failed: {exc}")
            return {"CANCELLED"}
        finally:
            _restore_edge_selection(source_obj, selected)
        try:
            receipt = write_decal_object(
                source_obj,
                run.results,
                offset=float(mesh_settings.offset),
                material_name=str(mesh_settings.material_name).strip()
                or DEFAULT_DECAL_MATERIAL,
            )
        except ProductionWriteError as exc:
            mesh_settings.status = f"Decal mesh not written: {exc.outcome}"
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        # Строка и уровень отчёта — по КВИТАНЦИИ: пропуск писателя (`ADAPTER_*`)
        # и мягкая находка стоят в них наравне с отказом продуктового пути.
        mesh_settings.status = receipt_status_text(receipt)
        mesh_settings.timing = (
            f"{production_timing_text(run)} | "
            f"{receipt.faces} faces, {receipt.vertices} vertices"
        )
        for line in receipt_console_lines(receipt, run.results):
            print(line, flush=True)
        self.report(
            {receipt_report_level(receipt)},
            f"Decal mesh: {mesh_settings.status}",
        )
        return {"FINISHED"}


_CLASSES = (HOTSPOTUV_DecalMeshSettings, HOTSPOTUV_OT_BuildEnvelopeDecalMesh)


def register_production_operator() -> None:
    """Регистрация настроек, оператора и свойства сцены. Повтор безопасен."""

    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(
        bpy.types.Scene,
        SETTINGS_ATTRIBUTE,
        PointerProperty(type=HOTSPOTUV_DecalMeshSettings),
    )


def unregister_production_operator() -> None:
    """Снятие в обратном порядке; без регистрации не делает ничего."""

    if hasattr(bpy.types.Scene, SETTINGS_ATTRIBUTE):
        delattr(bpy.types.Scene, SETTINGS_ATTRIBUTE)
    for cls in reversed(_CLASSES):
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)


__all__ = (
    "HOTSPOTUV_DecalMeshSettings",
    "HOTSPOTUV_OT_BuildEnvelopeDecalMesh",
    "SETTINGS_ATTRIBUTE",
    "UNDO_REQUIRED_REASON",
    "register_production_operator",
    "unregister_production_operator",
)
