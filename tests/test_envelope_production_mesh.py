"""Писатель продуктового меша: массивы из батчей и запись в объект Blender.

Blender здесь заменён небольшой моделью `bpy` (меш, объекты, материалы, слои UV,
атрибуты, шов ребра), достаточной, чтобы проверить КОНТРАКТ писателя, а не сам
Blender: позиции = локальная позиция батча + нормаль · смещение, UV петель =
факты `uv_facts`, атрибуты граней, один объект на источник, идемпотентная
пересборка, названные пропуски в квитанции, слот материала не перезаписывается.
Настоящий Blender проверяет `tests/blender/test_envelope_production_mesh.py`.

Хранилище Blender одинарной точности (float32): модель его повторяет, поэтому
сравнения «из меша» идут с допуском float32, а сравнения «из массивов» — точные.
"""

from __future__ import annotations

import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_mesh as writer  # noqa: E402
from cftuv.envelope_production_export import (  # noqa: E402
    MATERIALIZED,
    ProductionDomainResultV1,
    produce_domain,
)
from cftuv.envelope_production_mesh import (  # noqa: E402
    DECAL_DOMAIN_ATTRIBUTE,
    DECAL_OWNER_ATTRIBUTE,
    DECAL_REVISION_PROPERTY,
    DECAL_UV_LAYER,
    OUTCOME_EMPTY_BATCH,
    OUTCOME_NAME_TAKEN,
    OUTCOME_NON_FINITE,
    OUTCOME_NORMAL_MISSING,
    OUTCOME_VERTEX_MISSING,
    ProductionWriteError,
    build_mesh_arrays,
    decal_object_name,
    mesh_content_digest,
    write_decal_object,
)
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402


# --------------------------------------------------------------------------
# Модель `bpy`
# --------------------------------------------------------------------------


def _f32(value: float) -> float:
    return struct.unpack("f", struct.pack("f", float(value)))[0]


class _Collection(list):
    def foreach_set(self, name, flat):
        width = len(flat) // len(self) if len(self) else 0
        assert len(flat) == width * len(self), (name, len(flat), len(self))
        for index, item in enumerate(self):
            chunk = flat[index * width : (index + 1) * width]
            setattr(
                item,
                name,
                tuple(_f32(x) for x in chunk) if name == "uv" else chunk[0],
            )

    def get(self, name):
        return next((item for item in self if item.name == name), None)


class _Mesh:
    def __init__(self, name):
        self._name = name
        self._registry = None
        self.users = 0
        self.vertices = _Collection()
        self.polygons = _Collection()
        self.edges = _Collection()
        self.uv_layers = _UvLayers(self)
        self.attributes = _Attributes(self)
        self.materials = []
        self.loop_count = 0

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        if self._registry is not None:
            self._registry.rename(self._name, value)
        self._name = value

    def from_pydata(self, vertices, edges, faces):
        assert not edges
        self.vertices = _Collection(
            SimpleNamespace(co=tuple(_f32(x) for x in item)) for item in vertices
        )
        self.polygons = _Collection(
            SimpleNamespace(vertices=tuple(face)) for face in faces
        )
        pairs = []
        for face in faces:
            for a, b in zip(face, face[1:] + face[:1]):
                pair = tuple(sorted((a, b)))
                if pair not in pairs:
                    pairs.append(pair)
        self.edges = _Collection(
            SimpleNamespace(vertices=pair, use_seam=False) for pair in pairs
        )
        self.loop_count = sum(len(face) for face in faces)

    def update(self):
        pass


class _UvLayers(_Collection):
    def __init__(self, mesh):
        super().__init__()
        self._mesh = mesh

    def new(self, name):
        layer = SimpleNamespace(
            name=name,
            data=_Collection(
                SimpleNamespace(uv=(0.0, 0.0)) for _ in range(self._mesh.loop_count)
            ),
        )
        self.append(layer)
        return layer


class _Attributes(_Collection):
    def __init__(self, mesh):
        super().__init__()
        self._mesh = mesh

    def new(self, name, type, domain):  # noqa: A002 - сигнатура Blender
        assert (type, domain) == ("INT", "FACE")
        attribute = SimpleNamespace(
            name=name,
            data=_Collection(
                SimpleNamespace(value=0) for _ in range(len(self._mesh.polygons))
            ),
        )
        self.append(attribute)
        return attribute


class _Object:
    def __init__(self, name, mesh):
        self.name = name
        self.type = "MESH"
        self._data = None
        self.data = mesh
        self.parent = None
        self.location = (0.0, 0.0, 0.0)
        self.users_collection = ()
        self._props = {}

    @property
    def data(self):
        return self._data

    @data.setter
    def data(self, mesh):
        if self._data is not None:
            self._data.users -= 1
        self._data = mesh
        mesh.users += 1

    def keys(self):
        return self._props.keys()

    def __setitem__(self, key, value):
        self._props[key] = value

    def __getitem__(self, key):
        return self._props[key]


class _Registry:
    def __init__(self, factory):
        self._items = {}
        self._factory = factory

    def new(self, name, *args):
        base, number = name, 0
        while name in self._items:
            number += 1
            name = f"{base}.{number:03d}"
        item = self._factory(name, *args)
        item.name = name
        self.add(item)
        return item

    def add(self, item):
        self._items[item.name] = item
        if hasattr(item, "_registry"):
            item._registry = self

    def rename(self, old, new):
        # Blender дописывает `.001` занятому имени; тест держит имя свободным.
        assert new not in self._items, f"name {new!r} is taken"
        self._items[new] = self._items.pop(old)

    def get(self, name):
        return self._items.get(name)

    def remove(self, item):
        self._items.pop(item.name, None)

    def __len__(self):
        return len(self._items)

    def __iter__(self):
        return iter(self._items.values())


class _Scene:
    def __init__(self):
        linked = []
        self.linked = linked
        self.collection = SimpleNamespace(
            objects=SimpleNamespace(link=linked.append)
        )


def _fake_bpy():
    return SimpleNamespace(
        data=SimpleNamespace(
            meshes=_Registry(lambda name: _Mesh(name)),
            objects=_Registry(lambda name, mesh: _Object(name, mesh)),
            materials=_Registry(lambda name: SimpleNamespace(name=name)),
        ),
        context=SimpleNamespace(scene=_Scene()),
    )


@pytest.fixture
def fake_bpy(monkeypatch):
    bpy = _fake_bpy()
    monkeypatch.setattr(writer, "bpy", bpy)
    return bpy


def _source(bpy, name="Source"):
    mesh = bpy.data.meshes.new(f"{name}Mesh")
    source = _Object(name, mesh)
    bpy.data.objects.add(source)
    return source


# --------------------------------------------------------------------------
# Результаты домена: настоящие батчи ядра
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def row_results():
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController
    from cftuv.envelope_production_export import run_production

    controller = EnvelopeDebugSessionController()
    run = run_production(
        controller,
        quad_row_bundle(3),
        frozenset(range(3)),
        0.25,
        source_object_key="o",
        source_data_key="m",
        density=None,
        workers=0,
    )
    assert [item.outcome for item in run.results] == [MATERIALIZED] * 3
    return run.results


@pytest.fixture(scope="module")
def field_result():
    """Полевой домен с веером: несколько регионов и интерфейсные цепи (швы UV)."""

    import cftuv_envelope as kernel
    from cftuv_envelope.wavefront import prepare_conveyor

    root = KERNEL_SRC.parent / "fixtures" / "building_002_point_contact_v1"
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    prepared = prepare_conveyor(snapshot, request)
    result = produce_domain(
        7, "field", prepared, str(request.requested_alpha.value)
    )
    assert result.is_materialized, result.detail
    return result


def _fake_domain(patch_id, vertices, faces, *, normal=(0.0, 0.0, 1.0), seams=()):
    """Батч минимального вида: только то, что читает писатель."""

    def vertex(key, xyz):
        return SimpleNamespace(
            vert_key=SimpleNamespace(value=key),
            position=SimpleNamespace(x=xyz[0], y=xyz[1], z=xyz[2]),
        )

    def face(index, keys, claim="claim:0"):
        return SimpleNamespace(
            face_id=SimpleNamespace(value=f"face:{index}"),
            ordered_vert_keys=tuple(SimpleNamespace(value=k) for k in keys),
            uv_facts=tuple(
                SimpleNamespace(uv=SimpleNamespace(u=float(i), v=0.5))
                for i, _k in enumerate(keys)
            ),
            ownership_claim_id=SimpleNamespace(value=claim),
        )

    batch = SimpleNamespace(
        vertices=tuple(vertex(k, v) for k, v in vertices.items()),
        faces=tuple(face(i, keys) for i, keys in enumerate(faces)),
        interface_chains=tuple(
            SimpleNamespace(
                ordered_vert_keys=tuple(SimpleNamespace(value=k) for k in chain)
            )
            for chain in seams
        ),
        source_revision=SimpleNamespace(value="rev"),
    )
    return ProductionDomainResultV1(
        patch_id, f"domain{patch_id}", MATERIALIZED, batch, normal=normal
    )


SQUARE = {"a": (0, 0, 0), "b": (1, 0, 0), "c": (1, 1, 0), "d": (0, 1, 0)}
TWO_TRIANGLES = (("a", "b", "c"), ("a", "c", "d"))


# --------------------------------------------------------------------------
# Массивы
# --------------------------------------------------------------------------


def test_positions_are_the_batch_position_plus_the_patch_normal_times_the_offset(
    row_results,
):
    arrays = build_mesh_arrays(row_results, 0.02)

    batch_points = sorted(
        (item.vert_key.value, (item.position.x, item.position.y, item.position.z))
        for item in row_results[0].batch.vertices
    )
    first = len(batch_points)
    for (_key, point), lifted in zip(batch_points, arrays.positions[:first]):
        assert lifted == (point[0], point[1], point[2] + 0.02 * 1.0)
    assert row_results[0].normal == (0.0, 0.0, 1.0)


def test_the_offset_follows_each_domains_own_normal():
    tilted = _fake_domain(0, SQUARE, TWO_TRIANGLES, normal=(0.0, 0.6, 0.8))
    flat = _fake_domain(1, SQUARE, TWO_TRIANGLES)

    arrays = build_mesh_arrays([tilted, flat], 0.5)

    assert arrays.positions[0] == (0.0, 0.5 * 0.6, 0.5 * 0.8)
    assert arrays.positions[4] == (0.0, 0.0, 0.5)


def test_loop_uvs_are_the_uv_facts_of_every_face_in_order(row_results):
    arrays = build_mesh_arrays(row_results, 0.0)

    expected = [
        (fact.uv.u, fact.uv.v)
        for result in row_results
        for face in result.batch.faces
        for fact in face.uv_facts
    ]
    assert list(arrays.uvs) == expected
    assert len(arrays.uvs) == sum(len(loop) for loop in arrays.faces)


def test_face_attributes_carry_the_patch_and_the_stable_owner_ordinal(field_result):
    arrays = build_mesh_arrays([field_result], 0.0)

    assert set(arrays.face_domain) == {7}
    assert len(arrays.face_domain) == len(arrays.face_owner) == len(arrays.faces)
    expected = [
        int(face.ownership_claim_id.value.split(":")[1])
        for face in field_result.batch.faces
    ]
    assert list(arrays.face_owner) == expected
    assert len(set(expected)) > 1


def test_vertices_weld_by_the_semantic_key_inside_a_domain_and_never_across_domains():
    first = _fake_domain(0, SQUARE, TWO_TRIANGLES)
    second = _fake_domain(1, SQUARE, TWO_TRIANGLES)

    arrays = build_mesh_arrays([first, second], 0.0)

    # Четыре ключа на домен: диагональ делит `a` и `c`; одинаковые ключи и
    # позиции соседнего домена НЕ сварены (ни по ключу, ни по расстоянию).
    assert len(arrays.positions) == 8
    assert arrays.faces[0] == (0, 1, 2) and arrays.faces[2] == (4, 5, 6)
    assert arrays.positions[:4] == arrays.positions[4:]


def test_two_distinct_keys_at_one_point_are_not_merged_by_distance():
    domain = _fake_domain(
        0,
        {"a": (0, 0, 0), "b": (1, 0, 0), "c": (0, 1, 0), "a2": (0, 0, 0)},
        (("a", "b", "c"), ("a2", "c", "b")),
    )

    arrays = build_mesh_arrays([domain], 0.0)

    assert len(arrays.positions) == 4
    assert arrays.positions[0] == arrays.positions[1] == (0.0, 0.0, 0.0)


def test_the_order_is_by_patch_and_key_and_the_digest_depends_on_the_offset():
    shuffled = [_fake_domain(2, SQUARE, TWO_TRIANGLES), _fake_domain(1, SQUARE, TWO_TRIANGLES)]

    arrays = build_mesh_arrays(shuffled, 0.1)

    assert arrays.domains == (1, 2)
    assert arrays.face_domain == (1, 1, 2, 2)
    assert arrays.digest == build_mesh_arrays(reversed(shuffled), 0.1).digest
    assert arrays.digest != build_mesh_arrays(shuffled, 0.2).digest


def test_interface_chains_become_seam_edges_in_the_domain_index():
    domain = _fake_domain(0, SQUARE, TWO_TRIANGLES, seams=(("a", "c"),))

    arrays = build_mesh_arrays([domain], 0.0)

    assert arrays.seam_edges == ((0, 2),)


def test_the_field_domain_marks_the_uv_seams_of_its_interface_chains(field_result):
    arrays = build_mesh_arrays([field_result], 0.0)

    chains = field_result.batch.interface_chains
    assert chains and arrays.seam_edges
    assert len(arrays.seam_edges) == sum(
        len(chain.ordered_vert_keys) - 1 for chain in chains
    )


@pytest.mark.parametrize(
    "result,outcome",
    (
        (_fake_domain(0, {}, ()), OUTCOME_EMPTY_BATCH),
        (
            _fake_domain(0, {"a": (float("nan"), 0, 0), "b": (1, 0, 0), "c": (0, 1, 0)}, (("a", "b", "c"),)),
            OUTCOME_NON_FINITE,
        ),
        (
            _fake_domain(0, {"a": (0, 0, 0), "b": (1, 0, 0), "c": (0, 1, 0)}, (("a", "b", "z"),)),
            OUTCOME_VERTEX_MISSING,
        ),
        (
            _fake_domain(0, SQUARE, TWO_TRIANGLES, normal=(float("nan"), 0.0, 1.0)),
            OUTCOME_NORMAL_MISSING,
        ),
    ),
)
def test_a_batch_the_adapter_cannot_map_is_skipped_and_named(result, outcome):
    arrays = build_mesh_arrays([result, _fake_domain(1, SQUARE, TWO_TRIANGLES)], 0.0)

    assert arrays.domains == (1,)
    assert [(item[0], item[2]) for item in arrays.skipped] == [(0, outcome)]
    assert len(arrays.faces) == 2


def test_a_nan_uv_is_a_named_skip():
    domain = _fake_domain(0, SQUARE, TWO_TRIANGLES)
    domain.batch.faces[0].uv_facts[0].uv.u = float("nan")

    arrays = build_mesh_arrays([domain], 0.0)

    assert arrays.skipped and arrays.skipped[0][2] == OUTCOME_NON_FINITE
    assert not arrays.faces


def test_a_refused_domain_is_named_in_the_skipped_list_with_its_outcome(row_results):
    refused = ProductionDomainResultV1(9, "domain9", "STATION_CHAIN_UNNAMED", None, "why")

    arrays = build_mesh_arrays([*row_results, refused], 0.0)

    assert arrays.skipped == ((9, "domain9", "STATION_CHAIN_UNNAMED", "why"),)
    assert arrays.domains == (0, 1, 2)


# --------------------------------------------------------------------------
# Запись в объект
# --------------------------------------------------------------------------


def test_one_child_object_holds_every_domain_with_uv_attributes_and_one_material(
    fake_bpy, row_results
):
    source = _source(fake_bpy)

    receipt = write_decal_object(
        source, row_results, offset=0.02, material_name="CFTUV_Decal"
    )

    assert receipt.object_name == decal_object_name("Source") == "Source.CFTUV_Decal"
    decal = fake_bpy.data.objects.get(receipt.object_name)
    assert decal.parent is source and decal.location == (0.0, 0.0, 0.0)
    assert decal[DECAL_REVISION_PROPERTY] == row_results[0].batch.source_revision.value
    mesh = decal.data
    assert mesh.name == "Source.CFTUV_Decal"
    assert len(mesh.polygons) == receipt.faces == sum(len(r.batch.faces) for r in row_results)
    assert len(mesh.vertices) == receipt.vertices
    layer = mesh.uv_layers.get(DECAL_UV_LAYER)
    assert layer is not None and len(layer.data) == receipt.loops
    expected_uv = [
        (fact.uv.u, fact.uv.v)
        for result in row_results
        for face in result.batch.faces
        for fact in face.uv_facts
    ]
    for stored, wanted in zip(layer.data, expected_uv):
        assert stored.uv == (_f32(wanted[0]), _f32(wanted[1]))
    domains = [item.value for item in mesh.attributes.get(DECAL_DOMAIN_ATTRIBUTE).data]
    assert domains == [patch for r in row_results for patch in [r.patch_id] * len(r.batch.faces)]
    assert mesh.attributes.get(DECAL_OWNER_ATTRIBUTE) is not None
    assert [item.name for item in mesh.materials] == ["CFTUV_Decal"]
    assert receipt.material_created and receipt.domains == (0, 1, 2)
    assert receipt.skipped == () and receipt.offset == 0.02
    assert all(abs(v.co[2] - _f32(0.02)) < 1e-6 for v in mesh.vertices)
    assert receipt.mesh_digest == mesh_content_digest(mesh)
    assert fake_bpy.context.scene.linked == [decal]


def test_the_object_is_a_child_in_the_collection_of_its_source(fake_bpy, row_results):
    source = _source(fake_bpy)
    collection = SimpleNamespace(objects=SimpleNamespace(link=lambda item: seen.append(item)))
    seen: list = []
    source.users_collection = (collection,)

    write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert len(seen) == 1 and fake_bpy.context.scene.linked == []


def test_a_rebuild_is_idempotent_one_object_one_mesh_one_material(fake_bpy, row_results):
    source = _source(fake_bpy)
    first = write_decal_object(source, row_results, offset=0.02, material_name="M")
    decal = fake_bpy.data.objects.get(first.object_name)
    meshes_before = len(fake_bpy.data.meshes)

    second = write_decal_object(source, row_results, offset=0.02, material_name="M")

    assert not first.replaced and second.replaced
    assert fake_bpy.data.objects.get(first.object_name) is decal
    assert len(fake_bpy.data.objects) == 2  # источник и декаль
    assert len(fake_bpy.data.meshes) == meshes_before
    assert decal.data.name == "Source.CFTUV_Decal" and decal.data.users == 1
    assert len(fake_bpy.data.materials) == 1 and not second.material_created
    assert second.mesh_digest == first.mesh_digest
    assert second.arrays_digest == first.arrays_digest
    assert fake_bpy.context.scene.linked == [decal]
    # Другое смещение — другой меш в ТОМ ЖЕ объекте.
    third = write_decal_object(source, row_results, offset=0.05, material_name="M")
    assert third.mesh_digest != first.mesh_digest
    assert fake_bpy.data.objects.get(first.object_name) is decal


def test_an_existing_material_is_reused_and_never_overwritten(fake_bpy, row_results):
    source = _source(fake_bpy)
    owner_made = fake_bpy.data.materials.new("CFTUV_Decal")
    owner_made.owner_setting = "kept"

    receipt = write_decal_object(
        source, row_results, offset=0.0, material_name="CFTUV_Decal"
    )

    decal = fake_bpy.data.objects.get(receipt.object_name)
    assert decal.data.materials == [owner_made]
    assert owner_made.owner_setting == "kept"
    assert not receipt.material_created and len(fake_bpy.data.materials) == 1


def test_the_seams_of_the_interface_chains_reach_the_mesh_edges(fake_bpy):
    source = _source(fake_bpy)
    domain = _fake_domain(0, SQUARE, TWO_TRIANGLES, seams=(("a", "c"),))

    receipt = write_decal_object(source, [domain], offset=0.0, material_name="M")

    mesh = fake_bpy.data.objects.get(receipt.object_name).data
    marked = sorted(edge.vertices for edge in mesh.edges if edge.use_seam)
    assert marked == [(0, 2)] and receipt.seam_edges == 1


def test_a_skipped_domain_is_named_in_the_receipt_and_absent_from_the_mesh(
    fake_bpy, row_results
):
    source = _source(fake_bpy)
    refused = ProductionDomainResultV1(5, "d5", "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED", None, "x")
    broken = _fake_domain(6, {}, ())

    receipt = write_decal_object(
        source, [*row_results, refused, broken], offset=0.0, material_name="M"
    )

    assert receipt.domains == (0, 1, 2)
    assert {(item[0], item[2]) for item in receipt.skipped} == {
        (5, "NEAR_PLANAR_RESIDUAL_BUDGET_EXCEEDED"),
        (6, OUTCOME_EMPTY_BATCH),
    }
    mesh = fake_bpy.data.objects.get(receipt.object_name).data
    domains = {item.value for item in mesh.attributes.get(DECAL_DOMAIN_ATTRIBUTE).data}
    assert domains == {0, 1, 2}


def test_an_empty_result_creates_nothing_and_blanks_a_stale_object(fake_bpy, row_results):
    source = _source(fake_bpy)
    refused = ProductionDomainResultV1(1, "d1", "COVERAGE_IS_NOT_EXACT", None, "x")

    nothing = write_decal_object(source, [refused], offset=0.0, material_name="M")

    assert nothing.object_name is None and nothing.faces == 0
    assert fake_bpy.data.objects.get("Source.CFTUV_Decal") is None
    assert nothing.skipped == ((1, "d1", "COVERAGE_IS_NOT_EXACT", "x"),)

    write_decal_object(source, row_results, offset=0.0, material_name="M")
    blank = write_decal_object(source, [refused], offset=0.0, material_name="M")

    decal = fake_bpy.data.objects.get("Source.CFTUV_Decal")
    assert blank.replaced and blank.faces == 0
    assert len(decal.data.polygons) == 0 and decal.data.name == "Source.CFTUV_Decal"


def test_an_object_with_the_name_but_without_the_marker_is_not_touched(fake_bpy, row_results):
    source = _source(fake_bpy)
    stranger_mesh = fake_bpy.data.meshes.new("Stranger")
    stranger = _Object("Source.CFTUV_Decal", stranger_mesh)
    fake_bpy.data.objects.add(stranger)

    with pytest.raises(ProductionWriteError) as caught:
        write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert caught.value.outcome == OUTCOME_NAME_TAKEN
    assert fake_bpy.data.objects.get("Source.CFTUV_Decal").data is stranger_mesh
    assert len(stranger_mesh.polygons) == 0


def test_the_module_never_merges_by_distance():
    """Исполняемая форма запрета: ни расстояний, ни допусков в писателе."""

    source = Path(writer.__file__).read_text(encoding="utf-8")
    for forbidden in ("remove_doubles", "merge_distance", "isclose", "hypot", "threshold"):
        assert forbidden not in source, forbidden


# --------------------------------------------------------------------------
# Панель
# --------------------------------------------------------------------------


class _Layout:
    def __init__(self):
        self.calls = []

    def separator(self):
        self.calls.append(("separator",))

    def operator(self, idname, **kwargs):
        self.calls.append(("operator", idname, kwargs["text"]))

    def row(self, **_kwargs):
        return self

    def prop(self, _data, name, **_kwargs):
        self.calls.append(("prop", name))

    def label(self, *, text):
        self.calls.append(("label", text))


def test_the_panel_draws_the_button_the_settings_and_the_status_lines(monkeypatch):
    from cftuv.envelope_debug_panel import draw_decal_mesh_rows

    bpy_module = sys.modules["bpy"]
    mesh_settings = SimpleNamespace(
        status="MATERIALIZED 2 / refused 1 (X)", timing="Decal warm 0.10 s"
    )
    monkeypatch.setattr(
        bpy_module,
        "context",
        SimpleNamespace(scene=SimpleNamespace(hotspotuv_decal_mesh=mesh_settings)),
        raising=False,
    )
    layout = _Layout()

    draw_decal_mesh_rows(layout)

    assert ("operator", "hotspotuv.build_envelope_decal_mesh", "Build Decal Mesh") in layout.calls
    assert ("prop", "offset") in layout.calls and ("prop", "material_name") in layout.calls
    assert ("label", "MATERIALIZED 2 / refused 1 (X)") in layout.calls
    assert ("label", "Decal warm 0.10 s") in layout.calls
    mesh_settings.status = mesh_settings.timing = ""
    quiet = _Layout()
    draw_decal_mesh_rows(quiet)
    assert not [item for item in quiet.calls if item[0] == "label"]
    monkeypatch.setattr(
        bpy_module, "context", SimpleNamespace(scene=SimpleNamespace()), raising=False
    )
    absent = _Layout()
    draw_decal_mesh_rows(absent)
    assert absent.calls == []
