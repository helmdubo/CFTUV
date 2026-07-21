"""GPU preview overlay декалей (F3): display adapter поверх evaluator'а.

Контракт (workplan, FP1-FP4 + F3):
- evaluator остаётся единственным источником геометрии; overlay рисует
  ТОТ ЖЕ выход (parity by construction), confirm — прежний точный BMesh;
- во время drag ноль записей в mesh datablock и ноль depsgraph updates;
- модуль не импортирует ``gpu``/``bpy`` на верхнем уровне: batch-логика
  (триангуляция, буферы, решение о пересборке, state machine lifecycle)
  — чистая и тестируется headless через injected adapter;
- PreviewFailurePolicy=CLEAR: не-UPDATED кадр очищает overlay, как mesh
  adapter, и не маскирует текущий отказ последним валидным batch;
- ``bpy.app.background`` => фабрика возвращает ``None`` (mesh fallback).
"""

from dataclasses import dataclass, field

from .decal_geometry import GeometryBatch
from .surface_ir import PreviewFailurePolicy


# Палитра по component_kind — согласована с debug-раскраской (RGBA).
_KIND_COLORS = {
    "SEGMENT": (0.24, 0.52, 0.89, 0.50),
    "MITER": (0.36, 0.72, 0.36, 0.55),
    "BEVEL": (0.72, 0.45, 0.90, 0.55),
    "KITE": (0.93, 0.74, 0.25, 0.55),
    "FAN": (0.80, 0.55, 0.90, 0.55),
    "ACUTE_SPLIT": (0.92, 0.42, 0.34, 0.55),
    "HAIRPIN": (0.85, 0.30, 0.55, 0.55),
    "CAP": (0.35, 0.80, 0.78, 0.55),
    "JUNCTION": (0.95, 0.90, 0.40, 0.55),
}
_DEFAULT_COLOR = (0.60, 0.60, 0.60, 0.50)
_LINE_COLOR = (1.0, 0.62, 0.10, 1.0)


def _face_basis(normal):
    """Ортонормальный 2D-базис в плоскости грани по её нормали."""

    nx, ny, nz = float(normal[0]), float(normal[1]), float(normal[2])
    length = (nx * nx + ny * ny + nz * nz) ** 0.5
    if length <= 1e-12:
        return (1.0, 0.0, 0.0), (0.0, 1.0, 0.0)
    nx, ny, nz = nx / length, ny / length, nz / length
    if abs(nz) <= 0.9:
        ux, uy, uz = -ny, nx, 0.0
    else:
        ux, uy, uz = 0.0, -nz, ny
    ul = (ux * ux + uy * uy + uz * uz) ** 0.5
    ux, uy, uz = ux / ul, uy / ul, uz / ul
    vx = ny * uz - nz * uy
    vy = nz * ux - nx * uz
    vz = nx * uy - ny * ux
    return (ux, uy, uz), (vx, vy, vz)


def _project_2d(positions, normal):
    axis_u, axis_v = _face_basis(normal)
    points = []
    for position in positions:
        px = float(position[0])
        py = float(position[1])
        pz = float(position[2])
        points.append(
            (
                px * axis_u[0] + py * axis_u[1] + pz * axis_u[2],
                px * axis_v[0] + py * axis_v[1] + pz * axis_v[2],
            )
        )
    return points


def triangulate_face(positions, normal):
    """Детерминированный ear clipping; возвращает индексные тройки.

    Вогнутые n-gons поддерживаются; вырожденные (<3 вершин) дают ().
    Порядок обхода произвольный — ориентация нормализуется по знаковой
    площади проекции.
    """

    count = len(positions)
    if count < 3:
        return ()
    if count == 3:
        return ((0, 1, 2),)
    points = _project_2d(positions, normal)

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    indices = list(range(count))
    area2 = sum(
        cross((0.0, 0.0), points[i], points[(i + 1) % count])
        for i in range(count)
    )
    if area2 < 0.0:
        indices.reverse()
    triangles = []
    guard = 0
    while len(indices) > 3 and guard < 4 * count * count:
        guard += 1
        clipped = False
        for k in range(len(indices)):
            i0 = indices[(k - 1) % len(indices)]
            i1 = indices[k]
            i2 = indices[(k + 1) % len(indices)]
            a, b, c = points[i0], points[i1], points[i2]
            if cross(a, b, c) <= 1e-12:
                continue
            contains = False
            for j in indices:
                if j in (i0, i1, i2):
                    continue
                p = points[j]
                if (
                    cross(a, b, p) >= -1e-12
                    and cross(b, c, p) >= -1e-12
                    and cross(c, a, p) >= -1e-12
                ):
                    contains = True
                    break
            if contains:
                continue
            triangles.append((i0, i1, i2))
            indices.pop(k)
            clipped = True
            break
        if not clipped:
            # Остаточный контур почти вырожден: fan как страховка.
            break
    if len(indices) >= 3:
        anchor = indices[0]
        for k in range(1, len(indices) - 1):
            triangles.append((anchor, indices[k], indices[k + 1]))
    return tuple(triangles)


def topology_signature(faces):
    """Ширино-стабильная сигнатура: kind/side/счётчик вершин на грань.

    Чистое движение вершин при width drag сигнатуру не меняет (UPDATE,
    перезаливка буферов); topology-событие (band flip, collision) — меняет
    (REBUILD batch'ей).
    """

    if isinstance(faces, GeometryBatch):
        faces = faces.faces
    return tuple(
        (
            face.component_kind,
            face.component_side,
            tuple(face.vert_keys),
        )
        for face in faces
    )


def _transform_position(position, matrix_world):
    """Переводит evaluator-local позицию в world без mathutils dependency."""

    x = float(position[0])
    y = float(position[1])
    z = float(position[2])
    if matrix_world is None:
        return (x, y, z)
    return (
        float(matrix_world[0][0]) * x
        + float(matrix_world[0][1]) * y
        + float(matrix_world[0][2]) * z
        + float(matrix_world[0][3]),
        float(matrix_world[1][0]) * x
        + float(matrix_world[1][1]) * y
        + float(matrix_world[1][2]) * z
        + float(matrix_world[1][3]),
        float(matrix_world[2][0]) * x
        + float(matrix_world[2][1]) * y
        + float(matrix_world[2][2]) * z
        + float(matrix_world[2][3]),
    )


def build_preview_buffers(faces, matrix_world=None):
    """Собирает плоские POD-буферы для GPU batch'ей.

    Возвращает dict: tri_positions/tri_colors/tri_uvs (развёрнутые по
    треугольникам), line_positions (граничный контур: рёбра с одной
    инцидентной гранью), signature, counts.
    """

    if isinstance(faces, GeometryBatch):
        faces = faces.faces
    tri_positions = []
    tri_colors = []
    tri_uvs = []
    edge_counts = {}
    edge_positions = {}
    for face in faces:
        positions = face.positions
        world_positions = tuple(
            _transform_position(position, matrix_world)
            for position in positions
        )
        count = len(positions)
        for index in range(count):
            edge = frozenset(
                (
                    face.vert_keys[index],
                    face.vert_keys[(index + 1) % count],
                )
            )
            edge_counts[edge] = edge_counts.get(edge, 0) + 1
            edge_positions.setdefault(
                edge,
                (
                    world_positions[index],
                    world_positions[(index + 1) % count],
                ),
            )
        color = _KIND_COLORS.get(face.component_kind, _DEFAULT_COLOR)
        u_fracs = face.u_fracs
        v_lengths = face.v_lengths
        for i0, i1, i2 in triangulate_face(positions, face.surface_normal):
            for corner in (i0, i1, i2):
                tri_positions.append(world_positions[corner])
                tri_colors.append(color)
                tri_uvs.append(
                    (
                        (float(u_fracs[corner]) + 1.0) * 0.5,
                        float(v_lengths[corner]),
                    )
                )
    line_positions = []
    for edge, count in edge_counts.items():
        if count == 1:
            first, second = edge_positions[edge]
            line_positions.append(first)
            line_positions.append(second)
    return {
        "tri_positions": tri_positions,
        "tri_colors": tri_colors,
        "tri_uvs": tri_uvs,
        "line_positions": line_positions,
        "signature": topology_signature(faces),
        "face_count": len(faces),
    }


@dataclass
class GpuPreviewController:
    """State machine overlay: A6-семантика + идемпотентный lifecycle.

    Вся GPU-специфика делегируется adapter'у; контроллер тестируется
    headless с fake adapter.
    """

    adapter: object
    matrix_world: object | None = None
    failure_policy: PreviewFailurePolicy = PreviewFailurePolicy.CLEAR
    active: bool = False
    failed: bool = False
    last_buffers: dict | None = None
    last_signature: tuple | None = None
    updates: int = field(default=0)
    rebuilds: int = field(default=0)

    def start(self):
        if self.active or self.failed:
            return self.active
        try:
            self.adapter.add_handler(self._draw)
        except Exception:
            self.failed = True
            return False
        self.active = True
        return True

    def stop(self):
        if not self.active:
            return
        try:
            self.adapter.remove_handler()
        finally:
            self.active = False
        self._request_redraw()

    def update(self, faces, status="UPDATED"):
        """Publish one batch; any invalid/empty frame clears visible state."""

        if not self.active or self.failed:
            return "INACTIVE"
        face_payload = faces.faces if isinstance(faces, GeometryBatch) else faces
        if status != "UPDATED" or not face_payload:
            self.clear()
            self._request_redraw()
            return "CLEARED"
        buffers = build_preview_buffers(faces, self.matrix_world)
        rebuild = buffers["signature"] != self.last_signature
        try:
            self.adapter.upload(buffers, rebuild=rebuild)
        except Exception:
            self.failed = True
            self.stop()
            return "FAILED"
        self.last_buffers = buffers
        self.last_signature = buffers["signature"]
        self.updates += 1
        if rebuild:
            self.rebuilds += 1
        self._request_redraw()
        return "UPDATED"

    def clear(self):
        if self.failure_policy != PreviewFailurePolicy.CLEAR:
            raise AssertionError("Unsupported preview failure policy")
        self.last_buffers = None
        self.last_signature = None
        try:
            self.adapter.clear()
        except AttributeError:
            pass

    def _request_redraw(self):
        try:
            self.adapter.tag_redraw()
        except Exception:
            pass

    def _draw(self):
        # Draw handler не имеет права бросать: ошибка → тихий fail-flag,
        # modal при следующем кадре уйдёт в mesh fallback.
        if not self.active or self.last_buffers is None:
            return
        try:
            self.adapter.draw()
        except Exception:
            self.failed = True


class BlenderGpuAdapter:
    """Реальный adapter: базовый ``gpu`` API (Blender 4.1+), lazy import."""

    def __init__(self, textured=False, image=None):
        import bpy  # noqa: F401 — проверка среды
        import gpu
        from gpu_extras.batch import batch_for_shader

        self._bpy = bpy
        self._gpu = gpu
        self._batch_for_shader = batch_for_shader
        self._handler = None
        self._tri_batch = None
        self._line_batch = None
        self._tri_shader = None
        self._line_shader = None
        self._texture = None
        self._textured = bool(textured and image is not None)
        if self._textured:
            self._texture = gpu.texture.from_image(image)

    def supported(self):
        return not self._bpy.app.background

    def add_handler(self, draw_fn):
        space = self._bpy.types.SpaceView3D
        self._handler = space.draw_handler_add(
            draw_fn, (), "WINDOW", "POST_VIEW"
        )

    def remove_handler(self):
        if self._handler is None:
            return
        try:
            self._bpy.types.SpaceView3D.draw_handler_remove(
                self._handler, "WINDOW"
            )
        finally:
            self._handler = None

    def upload(self, buffers, rebuild=True):
        # gpu-batch'и в базовом API пересоздаются; различие
        # UPDATE/REBUILD остаётся семантикой контроллера (перф-метрика).
        gpu = self._gpu
        if self._textured:
            self._tri_shader = gpu.shader.from_builtin("IMAGE")
            content = {
                "pos": buffers["tri_positions"],
                "texCoord": buffers["tri_uvs"],
            }
        else:
            self._tri_shader = gpu.shader.from_builtin("SMOOTH_COLOR")
            content = {
                "pos": buffers["tri_positions"],
                "color": buffers["tri_colors"],
            }
        self._tri_batch = (
            self._batch_for_shader(self._tri_shader, "TRIS", content)
            if buffers["tri_positions"]
            else None
        )
        self._line_shader = gpu.shader.from_builtin("UNIFORM_COLOR")
        self._line_batch = (
            self._batch_for_shader(
                self._line_shader,
                "LINES",
                {"pos": buffers["line_positions"]},
            )
            if buffers["line_positions"]
            else None
        )

    def clear(self):
        self._tri_batch = None
        self._line_batch = None

    def draw(self):
        gpu = self._gpu
        gpu.state.blend_set("ALPHA")
        gpu.state.depth_test_set("LESS_EQUAL")
        gpu.state.depth_mask_set(False)
        try:
            if self._tri_batch is not None:
                self._tri_shader.bind()
                if self._textured and self._texture is not None:
                    self._tri_shader.uniform_sampler("image", self._texture)
                self._tri_batch.draw(self._tri_shader)
            if self._line_batch is not None:
                self._line_shader.bind()
                self._line_shader.uniform_float("color", _LINE_COLOR)
                self._line_batch.draw(self._line_shader)
        finally:
            gpu.state.depth_mask_set(True)
            gpu.state.blend_set("NONE")
            gpu.state.depth_test_set("NONE")

    def tag_redraw(self):
        manager = getattr(self._bpy.context, "window_manager", None)
        for window in getattr(manager, "windows", ()) or ():
            for area in getattr(window.screen, "areas", ()) or ():
                if area.type == "VIEW_3D":
                    area.tag_redraw()


def resolve_preview_image(source_obj):
    """Первая image-текстура активного материала источника (или None)."""

    material = getattr(source_obj, "active_material", None)
    node_tree = getattr(material, "node_tree", None)
    for node in getattr(node_tree, "nodes", ()) or ():
        if getattr(node, "type", "") == "TEX_IMAGE":
            image = getattr(node, "image", None)
            if image is not None:
                return image
    return None


def create_controller(source_obj=None, display_mode="GPU"):
    """Фабрика production-контроллера.

    Возвращает None (mesh fallback), если display_mode == 'MESH', среда
    headless (``bpy.app.background``) или gpu недоступен. Ошибки среды
    никогда не пробрасываются — GPU preview строго опционален.
    """

    if display_mode not in ("GPU", "GPU_TEXTURED"):
        return None
    try:
        import bpy

        if bpy.app.background:
            return None
        image = (
            resolve_preview_image(source_obj)
            if display_mode == "GPU_TEXTURED"
            else None
        )
        adapter = BlenderGpuAdapter(
            textured=display_mode == "GPU_TEXTURED", image=image
        )
        if not adapter.supported():
            return None
    except Exception:
        return None
    matrix_world = getattr(source_obj, "matrix_world", None)
    if hasattr(matrix_world, "copy"):
        matrix_world = matrix_world.copy()
    return GpuPreviewController(
        adapter=adapter,
        matrix_world=matrix_world,
    )
