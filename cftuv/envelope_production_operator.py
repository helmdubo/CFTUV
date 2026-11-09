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
from bpy.app.handlers import persistent
from bpy.props import BoolProperty, EnumProperty, FloatProperty, PointerProperty, StringProperty

from .analysis import build_analysis_bundle
from .analysis_surface import source_revision_from_bmesh
from .bevel_faces import BOTH, plan_console_lines, plan_summary
from .bevel_faces_blender import FACE_SIDE_ITEMS, BevelFacesRefused, face_side_of, plan_face_build, restore_face_selection
from .bevel_select_operator import HOTSPOTUV_OT_SeamBevel, HOTSPOTUV_OT_SelectBevelFaces
from .bevel_copy import (
    BEVEL_COPY_RAIL_SPLIT,
    BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK,
    BEVEL_COPY_RAIL_SPLIT_REFUSED,
    BEVEL_COPY_RAIL_UNPAIRED,
    copy_result,
    note_copy,
    rail_unpaired,
)
from .bevel_wings import parts_summary
from .bevel_wings_blender import counter_lines as wing_counter_lines
from .bevel_wings_blender import press_record as wings_press_record
from .envelope_gc_pause import gc_paused_call
from .envelope_kernel_backend import (
    DEFAULT_KERNEL_BACKEND,
    KERNEL_BACKEND_ITEMS,
    backend_console_lines,
    drop_legacy_skeleton_setting,
    fold_legacy_skeleton_setting,
    kernel_backend_of,
)
from .envelope_production_mesh import (
    DEFAULT_DECAL_MATERIAL,
    DEFAULT_DECAL_OFFSET,
    ProductionWriteError,
    build_mesh_arrays,
    resolve_decal_material_name,
    write_decal_object,
)
from .envelope_pool_prewarm import cancel_pool_prewarm, schedule_pool_prewarm
from .envelope_request_policy import ENVELOPE_PRODUCT_UV_POLICY_ITEMS, ENVELOPE_UV_POLICY_DIRECT_STRIP
from .envelope_source_preflight import reject_source, zero_length_edge_refusal, zero_length_refusal_in_faces
from .envelope_width_adjust import WIDTH_NOT_POSITIVE, width_refusal
from .envelope_width_live import remember_build, schedule_idle_preparation, tag_view3d_redraw
from .envelope_width_mesh_preview import build_sample, capture_ownership, note_button_display, sample_key
from .hotspot_blender import counter_lines, press_record

SETTINGS_ATTRIBUTE = "hotspotuv_decal_mesh"
UNDO_REQUIRED_REASON = (
    "the operator creates datablocks while the source is in EDIT mode; without an "
    "undo step the next undo reuses the scene unchanged and frees the decal object "
    "(dangling pointer, crash in the depsgraph); the pushed step makes Blender write "
    "the memfile that holds the decal"
)


def _kernel_backend_changed(self, _context) -> None:
    """Владелец выбрал переключатель: прежний ключ настройки скелета больше не говорит за него (иначе старый `PYTHON` перебил бы выбор)."""

    drop_legacy_skeleton_setting(self)


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
    kernel_backend: EnumProperty(
        name="Kernel backend",
        items=KERNEL_BACKEND_ITEMS,
        default=DEFAULT_KERNEL_BACKEND,
        update=_kernel_backend_changed,
        description=(
            "One switch for every stage the kernel has in Rust (coverage/clip, skeleton, embedding): the native (Rust) "
            "implementation (default) or the frozen Python reference. The answer is bitwise the same; a stage the native "
            "kernel cannot compute is computed in Python and named in the console. A scene that chose Python keeps it. "
            "Applies to the next Build Decal Mesh"
        ),
    )
    uv_policy_id: EnumProperty(
        name="UV layout",
        items=ENVELOPE_PRODUCT_UV_POLICY_ITEMS,
        default=ENVELOPE_UV_POLICY_DIRECT_STRIP,
        description=(
            "UV mapping of the decal strips and corner fans. Applies to the next Build Decal Mesh; "
            "Adjust Decal Width keeps the UV layout used by that Build"
        ),
    )
    drag_exact_ticks: BoolProperty(
        name="Exact ticks while dragging",
        default=False,
        description=(
            "While Adjust Decal Width is dragged, order an exact refresh at the pointer width whenever the hand pauses (WIDTH_REFRESH_INCREMENTAL_V1): "
            "only the domains the affine certificate does not serve are recomputed, the rest move by the certificate and are verified by a full refresh (WIDTH_REFRESH_LEAN_V1) every few ticks; "
            "no checks or reports of a commit. Confirming still recomputes and writes the committed result, cancelling restores the mesh bit for bit. Off by default: the worker pool is busy while it runs"
        ),
    )
    drag_live_mesh: BoolProperty(
        name="Live exact mesh while dragging",
        default=False,
        description=(
            "While Adjust Decal Width is dragged, stream exact refreshes (LIVE_DRAG_STREAM_V1): every mouse step orders one, a running refresh is never cancelled, "
            "the latest width wins and superseded widths are dropped; the mesh shows the exact result of the nearest width and the model moves it to the pointer. "
            "The structure is verified in the background, a batch of domains inside each tick and more ticks while the hand rests (DRAG_VERIFY_BACKGROUND_V1); the hand never waits for a full verification. "
            "Confirming still recomputes and writes the committed result, cancelling restores the mesh bit for bit. "
            "Off by default: field test"
        ),
    )
    drag_settle_on_pause: BoolProperty(
        name="Settle topology on pause",
        default=True,
        description=(
            "With the live exact mesh (LIVE_DRAG_SETTLE_ON_PAUSE_V1): while the hand moves, only model frames move the mesh (its structure stays the one of the last exact refresh); "
            "exact refreshes still compute in the background, and the newest one is swapped in only when the hand rests (LIVE_DRAG_SETTLE_PENDING until then) or on release"
        ),
    )
    drag_gpu_affine: BoolProperty(
        name="GPU affine drag",
        default=False,
        description=(
            "While Adjust Decal Width is dragged, Blender's Geometry Nodes move the decal by the model (GPU_AFFINE_DRAG_V1): every mouse step sets one number on a temporary "
            "hidden modifier instead of rewriting the mesh from Python. The modifier and its attributes exist only during the drag: confirming and cancelling remove them first, "
            "and saving, loading or an undo step remove any leftover. Confirming still recomputes and writes the committed result. Off by default: field test"
        ),
    )
    drag_coherent: BoolProperty(
        name="Coherent drag",
        default=True,
        description=(
            "While Adjust Decal Width is dragged, every domain follows the hand (LIVE_DRAG_COHERENT_V1): a domain past the window of its affine certificate is extrapolated, "
            "a domain without one moves by the difference of its last two exact samples; the frame is approximate near collisions, like the preview lines. "
            "With the live exact mesh, no exact result lands while the hand moves, and when it rests ONE whole exact frame of the rest width replaces the mesh at once. "
            "Confirming still recomputes and writes the committed result. On by default; acts only with the live exact mesh"
        ),
    )
    face_side: EnumProperty(
        name="Faces",
        items=FACE_SIDE_ITEMS,
        default=BOTH,
        description=(
            "Build Decal Mesh in Face Select mode (BEVEL_FACES_INWARD_OUTWARD_V1, BEVEL_FACES_BOTH_V3): Both (default) copies the selected bevel faces "
            "offset like the decal and welds the copy to the decal that grows outward by the decal width, one mesh, one UV island per bevel segment; Inward fills the selected bevel faces, "
            "Outward the faces around them; edge selection is unchanged"
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


#: Как часто таймер главного потока спрашивает фоновую проверку первого нажатия (SNAPSHOT_BYTE_CHECK_FIRST_PRESS_IN_BACKGROUND_V1), пока она идёт.
BYTE_CHECK_POLL_SECONDS = 0.5


def _watch_byte_check(controller, scene_name: str) -> None:
    """Итог фоновой проверки первого нажатия - в строки панели (потоку воркеров свойства Blender трогать нельзя, поэтому таймер главного потока).

    Чистый итог дописывается к строке времени, найденное и оборванное заменяют статус и называют исход (`job.text()`); строки, которые нажатие или другое действие уже переписали, не трогаются.
    Сам отказ найденного домена от строки не зависит: следующее действие отказывает его (`_scan`).
    """

    job = controller.byte_check_job
    settings = getattr(bpy.data.scenes.get(scene_name), SETTINGS_ATTRIBUTE, None)
    if job is None or settings is None:
        return
    shown = (str(settings.status), str(settings.timing))

    def poll():
        if controller.byte_check_job is not job:
            return None
        if not job.finished:
            return BYTE_CHECK_POLL_SECONDS
        scene = bpy.data.scenes.get(scene_name)
        current = getattr(scene, SETTINGS_ATTRIBUTE, None)
        if current is not None and (str(current.status), str(current.timing)) == shown:
            if job.state == "CLEAN":
                current.timing = f"{current.timing} | snapshot byte check: {job.checked} verified"
            else:
                current.status = job.text()
            tag_view3d_redraw()
        return None

    bpy.app.timers.register(poll, first_interval=BYTE_CHECK_POLL_SECONDS)


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
            and (bool(context.tool_settings.mesh_select_mode[1]) or bool(context.tool_settings.mesh_select_mode[2]))
        )

    # GC_PAUSED_DURING_BUTTON_V1: циклический сборщик стоит на паузе, пока идёт кнопка (прежнее состояние возвращается в `finally`).
    @gc_paused_call
    def execute(self, context):
        settings = context.scene.hotspotuv_settings
        mesh_settings = getattr(context.scene, SETTINGS_ATTRIBUTE)
        source_obj = context.active_object
        mesh_settings.status = "Building decal mesh..."
        mesh_settings.timing = ""
        fold_legacy_skeleton_setting(mesh_settings)
        kernel_backend = kernel_backend_of(mesh_settings)
        # WIDTH_NOT_POSITIVE_REFUSED_AT_PRESS_V1: ширина <= 0 - отказ по имени до анализа и пула; прежний меш декали не тронут (огибающая пуста, каждый батч пуст).
        refusal = width_refusal(settings.envelope_debug_alpha)
        if refusal:
            mesh_settings.status = f"Failed: {WIDTH_NOT_POSITIVE}"
            print(f"[CFTUV][DecalMesh] {refusal}", flush=True)
            self.report({"ERROR"}, refusal)
            return {"CANCELLED"}
        # HOTSPOT_MARKUP_V1: метка разрешается до анализа и пула; отказ (Uniform, нет разметки, метки, непринятая разметка) - по имени, меш не тронут.
        hotspot, refusal, detail = press_record(context.scene)
        if refusal:
            mesh_settings.status = f"Failed: {refusal}"
            print(f"[CFTUV][Hotspot] {refusal}: {detail}", flush=True)
            self.report({"ERROR"}, f"{refusal}: {detail}")
            return {"CANCELLED"}
        source_bm = bmesh.from_edit_mesh(source_obj.data)
        source_bm.edges.ensure_lookup_table()
        face_side = face_side_of(context, mesh_settings)  # BEVEL_FACES_INWARD_OUTWARD_V1: режим граней без режима рёбер - нажатие по граням бевела
        face = None
        if face_side is None:
            selected = [edge.index for edge in source_bm.edges if edge.select]
            if not selected:
                mesh_settings.status = "Failed: select a whole PhysicalChain"
                self.report({"WARNING"}, "ENVELOPE_DEBUG_EMPTY_SELECTION")
                return {"CANCELLED"}
            # Предполёт источника: ребро нулевой длины называется здесь, до анализа и
            # расчёта, и остаётся выделенным; хост ничего не сваривает молча.
            refusal = zero_length_edge_refusal(source_bm, selected)
        else:
            face = self._plan_faces(mesh_settings, source_obj, source_bm, face_side)
            if face is None:
                return {"CANCELLED"}
            selected, refusal = (), zero_length_refusal_in_faces(source_bm, face.plan.scope)
        if refusal is not None:
            mesh_settings.status = f"Failed: {refusal.message}"
            return reject_source(self, context, source_obj, source_bm, refusal)
        controller = _session(context)
        try:
            result = self._build_and_write(context, controller, source_obj, source_bm, selected, kernel_backend, hotspot, face)
        finally:
            # SNAPSHOT_BYTE_CHECK_LAUNCH_AFTER_BUTTON_V1: фоновая проверка первого нажатия стартует ПОСЛЕ хвоста кнопки (она занимает первичные ядра и замедляла хвост); в `finally`, чтобы отказ хвоста её не терял.
            controller.launch_pending_byte_check()
        if "FINISHED" in result:
            _watch_byte_check(controller, context.scene.name)
        return result

    def _plan_faces(self, mesh_settings, source_obj, source_bm, side):
        """План нажатия по граням бевела (BEVEL_FACES_INWARD_OUTWARD_V1) либо `None`: отказ плана назван в статусе и консоли."""

        try:
            face_build = plan_face_build(source_bm, source_obj, side)  # отпечаток меша нажатия берёт он, один раз
        except BevelFacesRefused as exc:
            mesh_settings.status = f"Failed: {exc.outcome}"
            print(f"[CFTUV][BevelFaces] REFUSED {exc}", flush=True)
            self.report({"ERROR"}, str(exc))
            return None
        for line in plan_console_lines(face_build.plan):
            print(line, flush=True)
        if face_build.copy is not None:  # BEVEL_FACES_BOTH_V2: решения UV копии - строкой с числами (правило 4)
            print(f"[CFTUV][BevelCopy] {len(face_build.copy.faces)} faces, {len(face_build.copy.vertex_ids)} vertices" + "".join(f", {name} {count}" for name, count in face_build.copy.events), flush=True)
        return face_build

    def _build_and_write(self, context, controller, source_obj, source_bm, selected, kernel_backend, hotspot=None, face_build=None):
        """Прогон, массивы, запись объекта, запись для живой ширины, строки и отчёт; итог оператора. Фоновую проверку запускает вызывающий (`execute`).

        `face_build` (`FaceBuildV1`) - нажатие по граням бевела: анализ его граней, источники из его пакета, выделение граней возвращается.
        """

        from .envelope_production_export import (
            production_timing_text,
            receipt_console_lines,
            receipt_report_level,
            receipt_status_text,
            run_production,
        )
        from .envelope_request_policy import envelope_dissolve_uv_slide, envelope_stretch_budget

        settings = context.scene.hotspotuv_settings
        mesh_settings = getattr(context.scene, SETTINGS_ATTRIBUTE)
        source_object_key = _runtime_key(source_obj)
        source_data_key = _runtime_key(source_obj.data)
        source_bm.faces.ensure_lookup_table()
        if face_build is None:
            face_indices = tuple(face.index for face in source_bm.faces)
            source_revision = mesh_revision = source_revision_from_bmesh(source_bm, source_obj, face_indices)
            build = lambda: build_analysis_bundle(source_bm, face_indices, source_obj, source_revision)  # noqa: E731 - отпечаток уже взят выше: второй раз его не берут
        else:
            source_revision, mesh_revision = face_build.revision, face_build.mesh_revision
            build = lambda: face_build.build_bundle(source_bm, source_obj)  # noqa: E731
        try:
            bundle = controller.get_analysis_bundle(source_object_key, source_data_key, source_revision, build)
            selected = selected if face_build is None else face_build.sources(bundle)
            run = run_production(
                controller,
                bundle,
                frozenset(selected),
                float(settings.envelope_debug_alpha),
                source_object_key=source_object_key,
                source_data_key=source_data_key,
                density=settings.envelope_debug_fan_density,
                uv_policy_id=getattr(mesh_settings, "uv_policy_id", ENVELOPE_UV_POLICY_DIRECT_STRIP),
                developable_stretch_budget=envelope_stretch_budget(settings.envelope_debug_max_stretch),
                silhouette_uv_slide=envelope_dissolve_uv_slide(settings.envelope_debug_dissolve_uv_tolerance),
                workers=settings.envelope_debug_workers,
                kernel_backend=kernel_backend,
                defer_byte_check=True,
            )
        except Exception as exc:  # noqa: BLE001 - причина идёт владельцу
            outcome = getattr(exc, "outcome", None)  # отказ хоста или граней - по имени исхода, иное исключение - по типу
            mesh_settings.status = f"Failed: {getattr(outcome, 'value', outcome)}" if outcome else f"Decal mesh failed: {type(exc).__name__}"
            self.report({"ERROR"}, f"Envelope decal mesh failed: {exc}")
            return {"CANCELLED"}
        finally:
            if face_build is None:
                _restore_edge_selection(source_obj, selected)
            else:
                restore_face_selection(source_obj, face_build.plan.selected)
        both = face_build is not None and face_build.plan.side == BOTH and face_build.copy is not None
        wings, wings_off = _note_both(face_build, bundle, run, mesh_settings) if both else (None, "")
        try:
            offset = float(mesh_settings.offset)
            # Массивы строятся один раз: их же читает образец превью ширины (`envelope_width_mesh_preview`), а писатель берёт готовые.
            arrays = build_mesh_arrays(run.results, offset)
            receipt = write_decal_object(
                source_obj,
                run.results,
                offset=offset,
                material_name=resolve_decal_material_name(mesh_settings.material_name, bpy.data.materials),
                width=float(settings.envelope_debug_alpha),
                arrays=arrays,
                hotspot=hotspot,
                wings=wings,
            )
        except ProductionWriteError as exc:
            mesh_settings.status = f"Decal mesh not written: {exc.outcome}"
            self.report({"ERROR"}, str(exc))
            return {"CANCELLED"}
        # Запись для живой ширины: пакет анализа, выделение, плотность и допуск этого нажатия. Меш уже записан:
        # сбой записи не отменяет кнопку, а называется строкой консоли, и ширина тогда не живая до следующего нажатия.
        try:
            record = remember_build(
                controller,
                source_obj.name,
                bundle,
                run,
                source_object_key=source_object_key,
                source_data_key=source_data_key,
                selected=selected,
                density=settings.envelope_debug_fan_density,
                stretch_percent=int(settings.envelope_debug_max_stretch),
                dissolve_percent=float(settings.envelope_debug_dissolve_uv_tolerance),
                width=float(settings.envelope_debug_alpha),
                kernel_backend=run.kernel_backend,
                skeleton_backend=run.skeleton_backend,
                embedding_backend=run.embedding_backend,
                mesh_digest=mesh_revision.digest,
            )
            sample = build_sample(run.results, arrays, sample_key(record, offset), float(settings.envelope_debug_alpha))
            note_button_display(controller, sample)
            # Владение мешем (тождество, размеры, поколение раскладки, её отпечаток) — тут же после записи: превью меша принадлежит только ему.
            capture_ownership(controller, bpy.data.objects.get(receipt.object_name or ""), sample)
            schedule_idle_preparation(controller, record)  # WIDTH_PREVIEW_INPUTS_DEFERRED_V1: пути линий превью строит простой после нажатия, а не кнопка
        except Exception as exc:  # noqa: BLE001 - живая ширина не ломает кнопку
            controller.width_build = None
            print(f"[CFTUV][WidthLive] the build was not remembered: {type(exc).__name__}: {exc}", flush=True)
        # Строка и уровень отчёта — по КВИТАНЦИИ: пропуск писателя (`ADAPTER_*`) и мягкая находка стоят в них наравне с отказом продуктового пути.
        mesh_settings.status = receipt_status_text(receipt) + ("" if face_build is None else f" | {plan_summary(face_build.plan)}")
        if both:  # BEVEL_FACES_BOTH_V3: копия и внешняя часть, отказ внешней части по имени, деления рёбер рельса, вершины рельса без пары, крылья не строятся - всё названо
            mesh_settings.status += _both_status(face_build, arrays, receipt, wings_off)
        mesh_settings.timing = (
            f"{production_timing_text(run)} | "
            f"{receipt.faces} faces, {receipt.vertices} vertices"
        )
        for line in (*backend_console_lines(run.results, run.kernel_backend, run.skeleton_backend, run.embedding_backend), *receipt_console_lines(receipt, run.results), *counter_lines(receipt.object_name), *wing_counter_lines(receipt.object_name)):
            print(line, flush=True)
        unresolved = face_build is not None and face_build.plan.split is not None and face_build.plan.split.unresolved
        self.report(
            {"WARNING" if unresolved else receipt_report_level(receipt)},
            f"Decal mesh: {mesh_settings.status}",
        )
        return {"FINISHED"}


def _note_both(face_build, bundle, run, mesh_settings) -> tuple:
    """BEVEL_FACES_BOTH_V2: копия граней бевела - домен сборки меша этого нажатия (номер патча за пакетом), до первых массивов; `(запись крыльев, исход отказа крыльев)`."""

    copy_patch = 1 + max((int(patch) for patch in bundle.patch_graph.nodes), default=-1)
    note_copy(copy_result(face_build.copy, copy_patch, bundle.source_revision), (result.domain_id for result in run.results))
    return wings_press_record(face_build, copy_patch, getattr(mesh_settings, "uv_policy_id", ENVELOPE_UV_POLICY_DIRECT_STRIP))


def _both_status(face_build, arrays, receipt, wings_off: str) -> str:
    """Хвост строки статуса нажатия Both: части (копия, внешняя часть и её отказы), вершины рельса копии, вставшие в рёбра внешней части
    (BEVEL_COPY_RAIL_SPLIT_V3), и отказы деления, вершины рельса без пары, отказ крыльев."""

    unpaired = rail_unpaired(arrays)
    counters = dict(getattr(arrays, "weld_counters", ()) or ())
    text = f" | {parts_summary(len(face_build.copy.faces), receipt.domains, receipt.skipped)}"
    for name in (BEVEL_COPY_RAIL_SPLIT, BEVEL_COPY_RAIL_SPLIT_REFUSED, BEVEL_COPY_RAIL_SPLIT_MITER_FALLBACK):
        text += f", {name} {counters[name]}" if counters.get(name) else ""
    return text + (f", {BEVEL_COPY_RAIL_UNPAIRED} {unpaired}" if unpaired else "") + (f" | {wings_off}" if wings_off else "")


_CLASSES = (HOTSPOTUV_DecalMeshSettings, HOTSPOTUV_OT_BuildEnvelopeDecalMesh, HOTSPOTUV_OT_SelectBevelFaces, HOTSPOTUV_OT_SeamBevel)


def fold_scene_settings() -> int:
    """Переносит выбор прежней настройки скелета в главный переключатель во всех сценах файла (`fold_legacy_skeleton_setting`); число сцен, где она лежала."""

    folded = 0
    for scene in bpy.data.scenes:
        settings = getattr(scene, SETTINGS_ATTRIBUTE, None)
        if settings is not None and fold_legacy_skeleton_setting(settings) is not None:
            folded += 1
    return folded


@persistent
def _fold_after_load(*_args) -> None:
    """Загрузка файла: сцены, сохранённые с прежней настройкой скелета, получают главный переключатель (панель показывает то, что считает кнопка)."""

    try:
        fold_scene_settings()
    except Exception as exc:  # noqa: BLE001 - миграция не ломает загрузку: чтение `kernel_backend_of` и так учитывает прежний ключ
        print(f"[CFTUV][Backend] legacy skeleton setting was not folded: {type(exc).__name__}: {exc}", flush=True)


def _fold_once():
    """Таймер регистрации: аддон включён в открытой сцене (у `load_post` повода нет), а в ограниченном контексте регистрации `bpy.data` не читается."""

    _fold_after_load()
    return None


def register_production_operator() -> None:
    """Регистрация настроек, оператора и свойства сцены. Повтор безопасен."""

    from .envelope_width_modal import register_width_tools
    from .hotspot_panel import register_hotspot

    for cls in _CLASSES:
        if getattr(cls, "is_registered", False):
            bpy.utils.unregister_class(cls)
        bpy.utils.register_class(cls)
    setattr(
        bpy.types.Scene,
        SETTINGS_ATTRIBUTE,
        PointerProperty(type=HOTSPOTUV_DecalMeshSettings),
    )
    register_width_tools()
    register_hotspot()  # HOTSPOT_MARKUP_V1: режим Hotspot панели декали
    if _fold_after_load not in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.append(_fold_after_load)
    if not bpy.app.timers.is_registered(_fold_once):
        bpy.app.timers.register(_fold_once, first_interval=0.0)
    schedule_pool_prewarm()  # воркеры пула стартуют заранее: первое нажатие не платит за их запуск (фоновый Blender не прогревает)


def unregister_production_operator() -> None:
    """Снятие в обратном порядке; без регистрации не делает ничего."""

    from .envelope_width_modal import unregister_width_tools
    from .hotspot_panel import unregister_hotspot

    cancel_pool_prewarm()
    unregister_width_tools()
    unregister_hotspot()
    while _fold_after_load in bpy.app.handlers.load_post:
        bpy.app.handlers.load_post.remove(_fold_after_load)
    if bpy.app.timers.is_registered(_fold_once):
        bpy.app.timers.unregister(_fold_once)
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
    "fold_scene_settings",
    "register_production_operator",
    "unregister_production_operator",
)
