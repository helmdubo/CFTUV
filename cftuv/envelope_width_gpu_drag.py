"""GPU_AFFINE_DRAG_V1: кадр модели превью ширины считает Blender (Geometry Nodes, C++), а не запись меша из Python.

ЗАЧЕМ. Полевой тест владельца (GUI, `cover.008`, живой меш и оседание включены): тяга идёт рывками. Каждый кадр модели — запись из Python (`foreach_set` позиций и UV и `mesh.update()`),
а в GUI каждая такая запись — копия COW меша, нормали и пересборка батча GPU за O(вершин) на главном потоке. Здесь кадр — один параметр модификатора: событие руки пишет число в
сокет «Width», depsgraph пересчитывает узлы в C++, Python меш не пишет.

ЧТО ЛЕЖИТ В МЕШЕ (только пока идёт перетаскивание с настройкой «GPU affine drag»). Меш держит геометрию базы модели (последний точный образец) — ровно то, что пишет точный путь. Рядом —
скрытые атрибуты (имена с точкой: в интерфейсе их не видно), ровно числа модели (`envelope_width_preview_model`), float32:
    `.cftuv_gn_dp`    точка, вектор   — наклон позиции вершины по ширине (`a1`; у вершины, которую модель не двигает, — ноль);
    `.cftuv_gn_vlow`, `.cftuv_gn_vhigh` точка — окно жизни вершины (`vlow`, `vhigh`; карантин и замороженные — пустое окно);
    `.cftuv_gn_duv`   угол, 2D        — `e1`: `uv = uv0 + e1 * dt / alpha`;
    `.cftuv_gn_llow`, `.cftuv_gn_lhigh` угол — окно жизни домена петли (`dlow`, `dhigh`, карантин — пустое окно);
    `.cftuv_gn_vcap`  точка, `.cftuv_gn_lcap` угол — предельная ширина вершины и вершины петли (LIVE_DRAG_COHERENT_VERTEX_CLAMPED, связный кадр; `inf` - предела нет).

УЗЛЫ (группа `.CFTUV GPU Affine Drag`, модификатор `CFTUV GPU Affine Drag`, вход `Width`, вход `Width0` = ширина базы). Ровно формулы кадра модели:
    dt      = Width - Width0;  dv = min(Width, vcap) - Width0;  dl = min(Width, lcap) - Width0   (без предела dv = dl = dt ровно)
    позиция = P0 + dp * (dv, если vlow < Width < vhigh, иначе 0)
    uv      = uv0 + duv * (dl / Width, если llow < Width < lhigh, иначе 0) + uv0 * ((dl - dt) / Width, там же)   (запись в слой `UVMap`, угол, 2D; без предела второй член - ноль)
Отличие от кадра Python — только округление: Python считает в binary64 и округляет итог до float32, узлы считают во float32. Окна сравниваются во float32: ширина в пределах
одного ULP от края окна может решиться иначе (вершина придержана либо нет); это меряется и называется в замере (`tools/blender_gpu_drag.py`).

НИЧЕГО НЕ ОСТАЁТСЯ. Модификатор, атрибуты и группа узлов существуют только во время перетаскивания. Каждый путь снятия назван (`REMOVED_*`): подтверждение и отмена
(`finish_drag`), снятие модели (`drop_model`), начало перетаскивания (остаток прошлого — `GPU_DRAG_REMOVED_STALE`), запись файла (`save_pre`), загрузка (`load_pre`, `load_post`),
шаг истории (`undo_pre`, `redo_pre`) и снятие аддона. Автосохранение Blender во время модального оператора не пишет файл (откладывает себя), а шаг отмены оператор кладёт после
возврата, когда снятие уже прошло.

Модуль `bpy` берёт лениво: числа (`rig_arrays`, `emulate`) от Blender не зависят и проверяются хост-тестами.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

GPU_AFFINE_DRAG_V1 = "GPU_AFFINE_DRAG_V1"
MODIFIER_NAME = "CFTUV GPU Affine Drag"
GROUP_NAME = ".CFTUV GPU Affine Drag"
WIDTH_SOCKET = "Width"
BASE_SOCKET = "Width0"
UV_LAYER = "UVMap"
ATTRIBUTE_PREFIX = ".cftuv_gn_"
SLOPE = ATTRIBUTE_PREFIX + "dp"
VERTEX_LOW = ATTRIBUTE_PREFIX + "vlow"
VERTEX_HIGH = ATTRIBUTE_PREFIX + "vhigh"
UV_SLOPE = ATTRIBUTE_PREFIX + "duv"
LOOP_LOW = ATTRIBUTE_PREFIX + "llow"
LOOP_HIGH = ATTRIBUTE_PREFIX + "lhigh"
VERTEX_CAP = ATTRIBUTE_PREFIX + "vcap"
LOOP_CAP = ATTRIBUTE_PREFIX + "lcap"
#: `(имя, тип, домен, поле foreach_set)` атрибутов узлов.
ATTRIBUTES = (
    (SLOPE, "FLOAT_VECTOR", "POINT", "vector"),
    (VERTEX_LOW, "FLOAT", "POINT", "value"),
    (VERTEX_HIGH, "FLOAT", "POINT", "value"),
    (UV_SLOPE, "FLOAT2", "CORNER", "vector"),
    (LOOP_LOW, "FLOAT", "CORNER", "value"),
    (LOOP_HIGH, "FLOAT", "CORNER", "value"),
    (VERTEX_CAP, "FLOAT", "POINT", "value"),
    (LOOP_CAP, "FLOAT", "CORNER", "value"),
)

#: Пути снятия (каждый назван в журнале превью меша).
REMOVED_AT_CONFIRM = "GPU_DRAG_REMOVED_AT_CONFIRM"
REMOVED_AT_CANCEL = "GPU_DRAG_REMOVED_AT_CANCEL"
REMOVED_ON_MODEL_DROP = "GPU_DRAG_REMOVED_ON_MODEL_DROP"
REMOVED_STALE = "GPU_DRAG_REMOVED_STALE"
REMOVED_ON_SAVE = "GPU_DRAG_REMOVED_ON_SAVE"
REMOVED_ON_LOAD = "GPU_DRAG_REMOVED_ON_LOAD"
REMOVED_ON_HISTORY = "GPU_DRAG_REMOVED_ON_HISTORY"
REMOVED_ON_UNREGISTER = "GPU_DRAG_REMOVED_ON_UNREGISTER"
REMOVED_ON_FAILURE = "GPU_DRAG_REMOVED_ON_FAILURE"
REMOVAL_PATHS = (
    REMOVED_AT_CONFIRM,
    REMOVED_AT_CANCEL,
    REMOVED_ON_MODEL_DROP,
    REMOVED_STALE,
    REMOVED_ON_SAVE,
    REMOVED_ON_LOAD,
    REMOVED_ON_HISTORY,
    REMOVED_ON_UNREGISTER,
    REMOVED_ON_FAILURE,
)
#: Узлы не построились либо модификатор не принят: кадр этого перетаскивания пишет Python, как без настройки (отказ назван).
REFUSED = "GPU_DRAG_REFUSED"
#: Версия формулы группы узлов: группа с другой (или без) пометкой перестраивается.
FORMULA = "GPU_AFFINE_DRAG_V1+VERTEX_CAP"


class GpuDragError(RuntimeError):
    """Узлы не могут показать кадр; `outcome` — имя (`GPU_DRAG_REFUSED:<причина>`)."""

    def __init__(self, outcome: str, message: str = "") -> None:
        super().__init__(message or outcome)
        self.outcome = outcome


@dataclass(slots=True)
class GpuRigV1:
    """Что поставлено на объект декали: под какую модель (тождество), на какой объект и меш, и идентификаторы сокетов."""

    model: object
    object_name: str
    object_pointer: int
    mesh_pointer: int
    width_identifier: str
    base_identifier: str
    base_width: float
    install_seconds: float = 0.0


# --------------------------------------------------------------------------
# Числа (без Blender)
# --------------------------------------------------------------------------


def rig_arrays(a1, e1, vlow, vhigh, dlow, dhigh, loop_counts, quarantined=None, vertex_quarantined=None, vcap=None, lcap=None) -> dict:
    """Плоские float32 атрибутов узлов из массивов модели: карантин сложен в окна (пустое окно - никогда не жив), как в `_live_masks`; пределы вершин и петель (`None` - `inf`)."""

    low = np.asarray(vlow, dtype=np.float64).copy()
    high = np.asarray(vhigh, dtype=np.float64).copy()
    if vertex_quarantined is not None:
        low[np.asarray(vertex_quarantined, dtype=bool)] = np.inf
        high[np.asarray(vertex_quarantined, dtype=bool)] = -np.inf
    dlow = np.asarray(dlow, dtype=np.float64).copy()
    dhigh = np.asarray(dhigh, dtype=np.float64).copy()
    if quarantined is not None:
        dlow[np.asarray(quarantined, dtype=bool)] = np.inf
        dhigh[np.asarray(quarantined, dtype=bool)] = -np.inf
    counts = np.asarray(loop_counts)
    return {
        SLOPE: np.ascontiguousarray(a1, dtype=np.float32).reshape(-1),
        VERTEX_LOW: low.astype(np.float32),
        VERTEX_HIGH: high.astype(np.float32),
        UV_SLOPE: np.ascontiguousarray(e1, dtype=np.float32).reshape(-1),
        LOOP_LOW: np.repeat(dlow, counts).astype(np.float32),
        LOOP_HIGH: np.repeat(dhigh, counts).astype(np.float32),
        VERTEX_CAP: (np.full(low.shape[0], np.inf) if vcap is None else np.asarray(vcap, dtype=np.float64)).astype(np.float32),
        LOOP_CAP: (np.full(int(counts.sum()), np.inf) if lcap is None else np.asarray(lcap, dtype=np.float64)).astype(np.float32),
    }


def emulate(positions, uvs, arrays: dict, width: float, base_width: float):
    """Что посчитают узлы (float32, те же операции в том же порядке): `(позиции (N*3), UV (L*2))`. Им судится расхождение с кадром Python без Blender."""

    f32 = np.float32
    w, w0 = f32(width), f32(base_width)
    dt = f32(w - w0)
    live_v = (arrays[VERTEX_LOW] < w) & (w < arrays[VERTEX_HIGH])
    moved = live_v.astype(f32) * (np.minimum(w, arrays[VERTEX_CAP]) - w0)
    p0 = np.asarray(positions, dtype=f32).reshape(-1, 3)
    out_p = p0 + arrays[SLOPE].reshape(-1, 3) * moved[:, None]
    live_l = (arrays[LOOP_LOW] < w) & (w < arrays[LOOP_HIGH])
    dl = np.minimum(w, arrays[LOOP_CAP]) - w0
    scale = live_l.astype(f32) * (dl / w)
    back = live_l.astype(f32) * ((dl - dt) / w)
    uv0 = np.asarray(uvs, dtype=f32).reshape(-1, 2)
    out_uv = (uv0 + arrays[UV_SLOPE].reshape(-1, 2) * scale[:, None]) + uv0 * back[:, None]
    return out_p.reshape(-1), out_uv.reshape(-1)


# --------------------------------------------------------------------------
# Узлы (Blender)
# --------------------------------------------------------------------------


def _named(nodes, name: str, data_type: str):
    node = nodes.new("GeometryNodeInputNamedAttribute")
    node.data_type = data_type
    node.inputs["Name"].default_value = name
    return node.outputs["Attribute"]


def _math(nodes, links, operation: str, a, b):
    node = nodes.new("ShaderNodeMath")
    node.operation = operation
    links.new(a, node.inputs[0])
    links.new(b, node.inputs[1])
    return node.outputs[0]


def _inside(nodes, links, low, width, high):
    """`low < width < high` булевым полем."""

    outputs = []
    for a, b in ((low, width), (width, high)):
        node = nodes.new("FunctionNodeCompare")
        node.data_type = "FLOAT"
        node.operation = "LESS_THAN"
        links.new(a, node.inputs["A"])
        links.new(b, node.inputs["B"])
        outputs.append(node.outputs["Result"])
    both = nodes.new("FunctionNodeBooleanMath")
    both.operation = "AND"
    links.new(outputs[0], both.inputs[0])
    links.new(outputs[1], both.inputs[1])
    return both.outputs[0]


def _vector(nodes, links, operation: str, a, b):
    node = nodes.new("ShaderNodeVectorMath")
    node.operation = operation
    links.new(a, node.inputs[0])
    links.new(b, node.inputs["Scale"] if operation == "SCALE" else node.inputs[1])
    return node.outputs["Vector"]


def _build_group(bpy):
    """Группа узлов ровно с формулами кадра модели (см. докстроку модуля)."""

    group = bpy.data.node_groups.new(GROUP_NAME, "GeometryNodeTree")
    group["cftuv_formula"] = FORMULA  # группа прежней формулы (без пределов), оставшаяся в файле, перестраивается
    if hasattr(group, "is_modifier"):
        group.is_modifier = True
    interface = group.interface
    interface.new_socket("Geometry", in_out="INPUT", socket_type="NodeSocketGeometry")
    interface.new_socket(WIDTH_SOCKET, in_out="INPUT", socket_type="NodeSocketFloat")
    interface.new_socket(BASE_SOCKET, in_out="INPUT", socket_type="NodeSocketFloat")
    interface.new_socket("Geometry", in_out="OUTPUT", socket_type="NodeSocketGeometry")
    nodes, links = group.nodes, group.links
    source = nodes.new("NodeGroupInput")
    sink = nodes.new("NodeGroupOutput")
    width, base = source.outputs[WIDTH_SOCKET], source.outputs[BASE_SOCKET]
    dt = _math(nodes, links, "SUBTRACT", width, base)
    live_v = _inside(nodes, links, _named(nodes, VERTEX_LOW, "FLOAT"), width, _named(nodes, VERTEX_HIGH, "FLOAT"))
    dv = _math(nodes, links, "SUBTRACT", _math(nodes, links, "MINIMUM", width, _named(nodes, VERTEX_CAP, "FLOAT")), base)
    moved = _math(nodes, links, "MULTIPLY", live_v, dv)
    offset = _vector(nodes, links, "SCALE", _named(nodes, SLOPE, "FLOAT_VECTOR"), moved)
    place = nodes.new("GeometryNodeSetPosition")
    links.new(source.outputs[0], place.inputs["Geometry"])
    links.new(offset, place.inputs["Offset"])
    live_l = _inside(nodes, links, _named(nodes, LOOP_LOW, "FLOAT"), width, _named(nodes, LOOP_HIGH, "FLOAT"))
    dl = _math(nodes, links, "SUBTRACT", _math(nodes, links, "MINIMUM", width, _named(nodes, LOOP_CAP, "FLOAT")), base)
    scale = _math(nodes, links, "MULTIPLY", live_l, _math(nodes, links, "DIVIDE", dl, width))
    back = _math(nodes, links, "MULTIPLY", live_l, _math(nodes, links, "DIVIDE", _math(nodes, links, "SUBTRACT", dl, dt), width))
    uv0 = _named(nodes, UV_LAYER, "FLOAT_VECTOR")
    uv = _vector(nodes, links, "ADD", _vector(nodes, links, "ADD", uv0, _vector(nodes, links, "SCALE", _named(nodes, UV_SLOPE, "FLOAT_VECTOR"), scale)),
                 _vector(nodes, links, "SCALE", uv0, back))
    store = nodes.new("GeometryNodeStoreNamedAttribute")
    store.data_type = "FLOAT2"
    store.domain = "CORNER"
    store.inputs["Name"].default_value = UV_LAYER
    links.new(place.outputs["Geometry"], store.inputs["Geometry"])
    links.new(uv, store.inputs["Value"])
    tail = store.outputs["Geometry"]
    for name, _kind, _domain, _field in ATTRIBUTES:  # вычисленный меш без наших атрибутов: они нужны только узлам
        remove = nodes.new("GeometryNodeRemoveAttribute")
        remove.inputs["Name"].default_value = name
        links.new(tail, remove.inputs["Geometry"])
        tail = remove.outputs["Geometry"]
    links.new(tail, sink.inputs[0])
    return group


def _group(bpy):
    group = bpy.data.node_groups.get(GROUP_NAME)
    if group is not None and group.bl_idname == "GeometryNodeTree" and WIDTH_SOCKET in group.interface.items_tree and group.get("cftuv_formula") == FORMULA:
        return group
    if group is not None:
        bpy.data.node_groups.remove(group)
    return _build_group(bpy)


def _write_attributes(mesh, arrays: dict) -> None:
    for name, kind, domain, field in ATTRIBUTES:
        attribute = mesh.attributes.get(name)
        if attribute is not None and (attribute.data_type != kind or attribute.domain != domain):
            mesh.attributes.remove(attribute)
            attribute = None
        if attribute is None:
            attribute = mesh.attributes.new(name, kind, domain)
        values = arrays[name]
        size = len(attribute.data) * (3 if kind == "FLOAT_VECTOR" else 2 if kind == "FLOAT2" else 1)
        if size != len(values):
            raise GpuDragError(f"{REFUSED}:SHAPE", f"{name}: the mesh holds {size} values, the model {len(values)}")
        attribute.data.foreach_set(field, values)
    mesh.update()


def install(decal, arrays: dict, base_width: float, model) -> GpuRigV1:
    """Атрибуты модели в меш, группа и модификатор на объект, `Width0` = ширина базы. Меш уже держит геометрию базы (это проверил и обеспечил зовущий)."""

    import bpy

    started = time.perf_counter()
    mesh = decal.data
    try:
        _write_attributes(mesh, arrays)
        group = _group(bpy)
        modifier = decal.modifiers.get(MODIFIER_NAME)
        if modifier is None or modifier.type != "NODES":
            if modifier is not None:
                decal.modifiers.remove(modifier)
            modifier = decal.modifiers.new(MODIFIER_NAME, "NODES")
        if modifier.node_group is not group:
            modifier.node_group = group
        modifier.show_in_editmode = False
        modifier.show_on_cage = False
        items = group.interface.items_tree
        width_id, base_id = items[WIDTH_SOCKET].identifier, items[BASE_SOCKET].identifier
        modifier[base_id] = float(base_width)
    except GpuDragError:
        raise
    except Exception as exc:  # noqa: BLE001 - отказ узлов назван, кадр пишет Python
        raise GpuDragError(f"{REFUSED}:{type(exc).__name__}", str(exc)) from exc
    return GpuRigV1(model, decal.name, int(decal.as_pointer()), int(mesh.as_pointer()), width_id, base_id, float(base_width), time.perf_counter() - started)


def current(rig: GpuRigV1 | None, decal, model) -> bool:
    """Поставленное на объекте - под эту модель и этот меш, модификатор и атрибут наклона на месте (сохранение файла и снятие извне их убирают)."""

    if rig is None or decal is None or rig.model is not model:
        return False
    if int(decal.as_pointer()) != rig.object_pointer or int(decal.data.as_pointer()) != rig.mesh_pointer:
        return False
    modifier = decal.modifiers.get(MODIFIER_NAME)
    return modifier is not None and modifier.type == "NODES" and modifier.node_group is not None and decal.data.attributes.get(SLOPE) is not None


def set_width(rig: GpuRigV1, decal, width: float) -> None:
    """Кадр: одно число в сокет и метка пересчёта объекта; позиции и UV считает depsgraph."""

    modifier = decal.modifiers.get(MODIFIER_NAME)
    if modifier is None:
        raise GpuDragError(f"{REFUSED}:MODIFIER_GONE")
    try:
        modifier[rig.width_identifier] = float(width)
        decal.update_tag(refresh={"DATA"})
    except Exception as exc:  # noqa: BLE001 - отказ узлов назван, кадр пишет Python
        raise GpuDragError(f"{REFUSED}:{type(exc).__name__}", str(exc)) from exc
    _redraw_views()


def _redraw_views() -> None:
    """Метка `update_tag` пересчитывает объект, но перерисовку вида не заказывает (запись меша Python заказывает её уведомлением `mesh.update()`): кадр из таймера без события руки иначе не был бы
    виден до следующего события."""

    import bpy

    manager = getattr(bpy.context, "window_manager", None)
    for window in getattr(manager, "windows", ()):
        screen = getattr(window, "screen", None)
        for area in getattr(screen, "areas", ()):
            if area.type == "VIEW_3D":
                area.tag_redraw()


def remove(decal) -> bool:
    """Модификатор и атрибуты сняты с объекта декали и его меша (группа без пользователей - тоже). `True` - было что снимать."""

    import bpy

    removed = False
    if decal is not None:
        modifier = decal.modifiers.get(MODIFIER_NAME)
        if modifier is not None:
            decal.modifiers.remove(modifier)
            removed = True
        mesh = getattr(decal, "data", None)
        if mesh is not None and hasattr(mesh, "attributes"):
            removed = _remove_attributes(mesh) or removed
    _drop_unused_group(bpy)
    return removed


def _remove_attributes(mesh) -> bool:
    found = False
    for name, _kind, _domain, _field in ATTRIBUTES:
        attribute = mesh.attributes.get(name)
        if attribute is not None:
            mesh.attributes.remove(attribute)
            found = True
    if found:
        mesh.update()
    return found


def _drop_unused_group(bpy) -> None:
    group = bpy.data.node_groups.get(GROUP_NAME)
    if group is not None and group.users == 0:
        bpy.data.node_groups.remove(group)


def strip_everywhere() -> int:
    """Снять всё наше со всех объектов и мешей файла и удалить группу (запись, загрузка, история, остаток прошлого). Число объектов и мешей, с которых снято."""

    import bpy

    count = 0
    for item in bpy.data.objects:
        modifier = item.modifiers.get(MODIFIER_NAME) if hasattr(item, "modifiers") else None
        if modifier is not None:
            item.modifiers.remove(modifier)
            count += 1
    for mesh in bpy.data.meshes:
        if mesh.attributes.get(SLOPE) is not None or any(mesh.attributes.get(name) is not None for name, *_rest in ATTRIBUTES):
            _remove_attributes(mesh)
            count += 1
    group = bpy.data.node_groups.get(GROUP_NAME)
    if group is not None:
        bpy.data.node_groups.remove(group)
    return count


__all__ = (
    "ATTRIBUTES",
    "GPU_AFFINE_DRAG_V1",
    "GROUP_NAME",
    "GpuDragError",
    "GpuRigV1",
    "MODIFIER_NAME",
    "REFUSED",
    "REMOVAL_PATHS",
    "REMOVED_AT_CANCEL",
    "REMOVED_AT_CONFIRM",
    "REMOVED_ON_FAILURE",
    "REMOVED_ON_HISTORY",
    "REMOVED_ON_LOAD",
    "REMOVED_ON_MODEL_DROP",
    "REMOVED_ON_SAVE",
    "REMOVED_ON_UNREGISTER",
    "REMOVED_STALE",
    "current",
    "emulate",
    "install",
    "remove",
    "rig_arrays",
    "set_width",
    "strip_everywhere",
)
