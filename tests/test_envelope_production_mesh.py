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
            value = self._registry.rename(self._name, value)
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
        # Blender режет имя ID до 63 байт и дописывает `.001` при коллизии.
        name = name.encode("utf-8")[:63].decode("utf-8", "ignore")
        base, number = name, 0
        while name in self._items:
            number += 1
            name = f"{base.encode('utf-8')[:59].decode('utf-8', 'ignore')}.{number:03d}"
        item = self._factory(name, *args)
        item.name = name
        self.add(item)
        return item

    def add(self, item):
        self._items[item.name] = item
        if hasattr(item, "_registry"):
            item._registry = self

    def rename(self, old, new):
        """Как Blender: занятое имя получает `.001`; возвращает итоговое имя."""

        new = new.encode("utf-8")[:63].decode("utf-8", "ignore")
        if new == old:
            return new
        base, number = new, 0
        while new in self._items:
            number += 1
            new = f"{base.encode('utf-8')[:59].decode('utf-8', 'ignore')}.{number:03d}"
        self._items[new] = self._items.pop(old)
        return new

    def get(self, name):
        return self._items.get(name)

    def remove(self, item):
        self._items.pop(item.name, None)

    def __len__(self):
        return len(self._items)

    def __iter__(self):
        return iter(self._items.values())


class _Coll:
    """Коллекция: `objects.link` помнит объект, а объект помнит коллекцию."""

    def __init__(self):
        self.linked = []
        self.objects = SimpleNamespace(link=self._link)

    def _link(self, item):
        self.linked.append(item)
        item.users_collection = tuple(item.users_collection) + (self,)


class _Scene:
    def __init__(self):
        self.collection = _Coll()
        self.linked = self.collection.linked


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


def _fake_domain(
    patch_id,
    vertices,
    faces,
    *,
    normal=(0.0, 0.0, 1.0),
    seams=(),
    source_normal=(0.0, 0.0, 1.0),
    counters=(),
):
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
        patch_id,
        f"domain{patch_id}",
        MATERIALIZED,
        batch,
        normal=normal,
        source_normal=source_normal,
        counters=counters,
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


def test_an_unfolded_domain_offsets_each_vertex_along_its_own_normal():
    """Домен-развёртка: нормаль смещения своя на вершину (закон ядра), не нормаль плоскости."""

    from dataclasses import replace

    law = "SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1"
    folded = replace(
        _fake_domain(0, SQUARE, TWO_TRIANGLES),
        vertex_normals=(
            ("a", (0.0, 0.0, 1.0)),
            ("b", (0.0, 0.0, 1.0)),
            ("c", (1.0, 0.0, 0.0)),
            ("d", (0.0, 1.0, 0.0)),
        ),
        offset_normal_law=law,
    )

    arrays = build_mesh_arrays([folded], 0.5)

    assert arrays.positions == (
        (0.0, 0.0, 0.5),
        (1.0, 0.0, 0.5),
        (1.5, 1.0, 0.0),
        (0.0, 1.5, 0.0),
    )
    assert not arrays.skipped

    missing = replace(folded, vertex_normals=folded.vertex_normals[:3])
    skipped = build_mesh_arrays([missing], 0.5).skipped
    assert [item[2] for item in skipped] == ["ADAPTER_NORMAL_MISSING"]
    assert "vertex d" in skipped[0][3]


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
    collection = _Coll()
    source.users_collection = (collection,)

    write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert len(collection.linked) == 1 and fake_bpy.context.scene.linked == []


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


# --------------------------------------------------------------------------
# Аудит среза 4: имена ID, швы, нормаль источника, предупреждения, статус
# --------------------------------------------------------------------------


def _decal_objects(bpy):
    return [item for item in bpy.data.objects if DECAL_REVISION_PROPERTY in item.keys()]


def test_a_decal_name_is_clipped_to_the_blender_limit_on_a_character_boundary():
    from cftuv.envelope_production_mesh import DECAL_OBJECT_SUFFIX, ID_NAME_LIMIT_BYTES

    for source in ("S" * 60, "Ж" * 40, "short"):
        name = decal_object_name(source)
        assert len(name.encode("utf-8")) <= ID_NAME_LIMIT_BYTES, source
        assert name.endswith(DECAL_OBJECT_SUFFIX)
    assert decal_object_name("short") == "short.CFTUV_Decal"
    clipped = decal_object_name("Ж" * 40)
    assert clipped[: -len(DECAL_OBJECT_SUFFIX)] == "Ж" * 25  # 50 байт + 12 суффикса


def test_a_long_source_name_never_multiplies_the_decal_object(fake_bpy, row_results):
    source = _source(fake_bpy, "S" * 60)
    wanted = decal_object_name(source.name)
    assert len(wanted.encode("utf-8")) <= 63 and wanted != source.name + ".CFTUV_Decal"

    first = write_decal_object(source, row_results, offset=0.0, material_name="M")
    again = write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert first.object_name == again.object_name == wanted
    assert not first.replaced and again.replaced
    assert len(_decal_objects(fake_bpy)) == 1
    assert again.mesh_digest == first.mesh_digest
    assert again.warnings == ()


def test_two_sources_whose_clipped_names_collide_keep_one_decal_each(fake_bpy, row_results):
    first = _source(fake_bpy, "S" * 60 + "A")
    second = _source(fake_bpy, "S" * 60 + "B")

    for _ in range(2):
        write_decal_object(first, row_results, offset=0.0, material_name="M")
        write_decal_object(second, row_results, offset=0.0, material_name="M")

    decals = _decal_objects(fake_bpy)
    assert len(decals) == 2
    assert {item.parent.name for item in decals} == {first.name, second.name}
    # Имя второго Blender разрешил суффиксом: оба нажатия нашли СВОЙ объект по маркеру.
    assert sum(item.name.endswith(".001") for item in decals) == 1


def test_the_decal_is_found_by_its_marker_after_the_owner_renames_either_object(
    fake_bpy, row_results
):
    source = _source(fake_bpy)
    first = write_decal_object(source, row_results, offset=0.0, material_name="M")
    decal = fake_bpy.data.objects.get(first.object_name)
    fake_bpy.data.objects.rename(decal.name, "MyOwnName")
    decal.name = "MyOwnName"

    second = write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert second.replaced and second.object_name == "MyOwnName"
    assert len(_decal_objects(fake_bpy)) == 1
    # Переименованный источник: родитель переживает переименование, метка обновляется.
    fake_bpy.data.objects.rename(source.name, "Renamed")
    source.name = "Renamed"
    third = write_decal_object(source, row_results, offset=0.0, material_name="M")
    assert third.replaced and len(_decal_objects(fake_bpy)) == 1
    assert decal["cftuv_source_object"] == "Renamed"


def test_two_decals_of_one_source_are_named_and_only_one_is_rebuilt(fake_bpy, row_results):
    source = _source(fake_bpy)
    first = write_decal_object(source, row_results, offset=0.0, material_name="M")
    stray_mesh = fake_bpy.data.meshes.new("Stray")
    stray = _Object("Stray.CFTUV_Decal", stray_mesh)
    stray["cftuv_source_revision"] = "old"
    stray.parent = source
    fake_bpy.data.objects.add(stray)

    receipt = write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert receipt.object_name == first.object_name
    assert [item[1] for item in receipt.warnings] == [writer.OUTCOME_DUPLICATE_DECALS]
    assert stray["cftuv_source_revision"] == "old" and stray.data is stray_mesh


def test_a_parent_or_collection_that_drifted_is_restored_and_named(fake_bpy, row_results):
    source = _source(fake_bpy)
    home = _Coll()
    source.users_collection = (home,)
    first = write_decal_object(source, row_results, offset=0.0, material_name="M")
    decal = fake_bpy.data.objects.get(first.object_name)
    assert first.warnings == () and decal.users_collection == (home,)
    decal.parent = None
    decal.users_collection = ()

    second = write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert decal.parent is source and decal.users_collection == (home,)
    assert {item[1] for item in second.warnings} == {
        writer.OUTCOME_PARENT_REASSERTED,
        writer.OUTCOME_COLLECTION_REASSERTED,
    }


def test_a_shared_mesh_datablock_is_kept_and_named(fake_bpy, row_results):
    source = _source(fake_bpy)
    first = write_decal_object(source, row_results, offset=0.0, material_name="M")
    decal = fake_bpy.data.objects.get(first.object_name)
    twin = _Object("Twin", decal.data)
    fake_bpy.data.objects.add(twin)
    assert decal.data.users == 2

    second = write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert [item[1] for item in second.warnings] == [writer.OUTCOME_MESH_SHARED]
    assert twin.data is not decal.data and twin.data.users == 1
    assert second.mesh_name == decal.data.name


def test_an_orphan_mesh_with_the_decal_name_does_not_leak_a_suffix(fake_bpy, row_results):
    source = _source(fake_bpy)
    orphan = fake_bpy.data.meshes.new("Source.CFTUV_Decal")
    assert orphan.users == 0

    receipt = write_decal_object(source, row_results, offset=0.0, material_name="M")

    assert receipt.mesh_name == "Source.CFTUV_Decal"
    assert fake_bpy.data.meshes.get("Source.CFTUV_Decal.001") is None


def test_a_seam_that_is_not_a_mesh_edge_is_counted_and_named_not_reported_as_marked(
    fake_bpy,
):
    source = _source(fake_bpy)
    # `b`-`d` — диагональ, которой нет среди рёбер двух треугольников.
    domain = _fake_domain(0, SQUARE, TWO_TRIANGLES, seams=(("a", "c"), ("b", "d")))

    receipt = write_decal_object(source, [domain], offset=0.0, material_name="M")

    assert receipt.seam_edges_requested == 2 and receipt.seam_edges == 1
    assert [(item[0], item[1]) for item in receipt.warnings] == [
        (None, writer.OUTCOME_SEAM_EDGE_MISSING)
    ]
    assert "2 seam edges requested" in receipt.warnings[0][2]
    assert "1 marked" in receipt.warnings[0][2]


def test_a_domain_whose_normal_opposes_the_source_is_skipped_by_name():
    inward = _fake_domain(0, SQUARE, TWO_TRIANGLES, source_normal=(0.0, 0.0, -1.0))
    fine = _fake_domain(1, SQUARE, TWO_TRIANGLES)

    arrays = build_mesh_arrays([inward, fine], 0.02)

    assert arrays.domains == (1,)
    assert [(item[0], item[2]) for item in arrays.skipped] == [
        (0, writer.OUTCOME_NORMAL_OPPOSES_SOURCE)
    ]
    assert "into the surface" in arrays.skipped[0][3]


def test_an_orthogonal_normal_is_not_accepted_either():
    edge_on = _fake_domain(0, SQUARE, TWO_TRIANGLES, source_normal=(1.0, 0.0, 0.0))

    arrays = build_mesh_arrays([edge_on], 0.02)

    assert not arrays.domains and arrays.skipped[0][2] == writer.OUTCOME_NORMAL_OPPOSES_SOURCE


def test_soft_findings_of_a_written_domain_are_warnings_and_the_domain_stays():
    unknown = _fake_domain(0, SQUARE, TWO_TRIANGLES, source_normal=None)
    flipped = _fake_domain(
        1,
        SQUARE,
        TWO_TRIANGLES,
        counters=((writer.OUTCOME_FLIPPED_VS_SOURCE, 3),),
    )

    arrays = build_mesh_arrays([unknown, flipped], 0.0)

    assert arrays.domains == (0, 1) and not arrays.skipped
    assert [(item[0], item[1]) for item in arrays.warnings] == [
        (0, writer.OUTCOME_SOURCE_NORMAL_UNKNOWN),
        (1, writer.OUTCOME_FLIPPED_VS_SOURCE),
    ]
    assert "3 faces" in arrays.warnings[1][2]


def test_the_status_and_console_come_from_the_receipt_so_adapter_skips_are_visible(
    fake_bpy, row_results
):
    from cftuv.envelope_production_export import (
        receipt_console_lines,
        receipt_status_text,
    )

    source = _source(fake_bpy)
    refused = ProductionDomainResultV1(5, "d5", "STATION_CHAIN_UNNAMED", None, "why")
    broken = _fake_domain(6, {}, ())
    inward = _fake_domain(7, SQUARE, TWO_TRIANGLES, source_normal=(0.0, 0.0, -1.0))
    everything = [*row_results, refused, broken, inward]

    receipt = write_decal_object(source, everything, offset=0.0, material_name="M")

    status = receipt_status_text(receipt)
    assert status == (
        "MATERIALIZED 3 / refused 3 (ADAPTER_EMPTY_BATCH, "
        "ADAPTER_NORMAL_OPPOSES_SOURCE, STATION_CHAIN_UNNAMED)"
    )
    lines = receipt_console_lines(receipt, everything)
    assert len([item for item in lines if " REFUSED patch " in item]) == 3
    assert any("patch 6" in item and "ADAPTER_EMPTY_BATCH" in item for item in lines)
    assert lines[-1].endswith(status)


def test_a_warning_reaches_the_status_and_the_console_with_its_name(fake_bpy):
    from cftuv.envelope_production_export import (
        receipt_console_lines,
        receipt_status_text,
    )

    source = _source(fake_bpy)
    domain = _fake_domain(0, SQUARE, TWO_TRIANGLES, seams=(("b", "d"),))

    receipt = write_decal_object(source, [domain], offset=0.0, material_name="M")

    assert receipt_status_text(receipt) == (
        "MATERIALIZED 1 / refused 0 | warnings: ADAPTER_SEAM_EDGE_MISSING"
    )
    lines = receipt_console_lines(receipt, [domain])
    assert any("WARNING mesh: ADAPTER_SEAM_EDGE_MISSING" in item for item in lines)


def test_the_real_batches_carry_the_source_normal_and_the_chart_orientation(row_results):
    for result in row_results:
        assert result.source_normal is not None and any(result.source_normal)
        assert result.chart_orientation.startswith("COORDINATE_")
        dot = sum(a * b for a, b in zip(result.normal, result.source_normal))
        assert dot > 0.99
    arrays = build_mesh_arrays(row_results, 0.02)
    assert arrays.warnings == () and arrays.domains == (0, 1, 2)


def test_the_report_level_is_a_warning_for_any_missing_domain_or_finding(fake_bpy, row_results):
    from cftuv.envelope_production_export import receipt_report_level

    source = _source(fake_bpy)
    clean = write_decal_object(source, row_results, offset=0.0, material_name="M")
    assert receipt_report_level(clean) == "INFO"

    # Пропуск ПИСАТЕЛЯ (а не продуктового пути): раньше уровень это не видел.
    adapter_only = write_decal_object(
        source,
        [*row_results, _fake_domain(6, {}, ())],
        offset=0.0,
        material_name="M",
    )
    assert adapter_only.skipped and receipt_report_level(adapter_only) == "WARNING"

    finding_only = write_decal_object(
        source,
        [_fake_domain(0, SQUARE, TWO_TRIANGLES, seams=(("b", "d"),))],
        offset=0.0,
        material_name="M",
    )
    assert not finding_only.skipped and receipt_report_level(finding_only) == "WARNING"


# --------------------------------------------------------------------------
# Закон топологии: грани разной длины в одном меше
# --------------------------------------------------------------------------

QUAD_THEN_TRIANGLE = (("a", "b", "c", "d"), ("a", "c", "b"))


def test_a_quad_and_a_triangle_are_written_as_loops_of_their_own_length():
    domain = _fake_domain(0, SQUARE, QUAD_THEN_TRIANGLE, seams=(("a", "c"),))

    arrays = build_mesh_arrays([domain], 0.0)

    assert arrays.faces == ((0, 1, 2, 3), (0, 2, 1))
    assert len(arrays.uvs) == 4 + 3 == sum(len(loop) for loop in arrays.faces)
    assert arrays.face_domain == (0, 0) and len(arrays.face_owner) == 2
    assert arrays.seam_edges == ((0, 2),)


def test_the_quads_of_a_field_domain_are_four_loops_and_the_fans_stay_triangles(
    fake_bpy, field_result
):
    source = _source(fake_bpy)

    receipt = write_decal_object(source, [field_result], offset=0.0, material_name="M")

    mesh = fake_bpy.data.objects.get(receipt.object_name).data
    sizes = [len(item.vertices) for item in mesh.polygons]
    assert set(sizes) == {3, 4}
    assert receipt.quads == sizes.count(4) > 0 and receipt.triangles == sizes.count(3) > 0
    assert receipt.faces == receipt.quads + receipt.triangles == len(mesh.polygons)
    assert receipt.loops == 4 * receipt.quads + 3 * receipt.triangles == mesh.loop_count
    assert receipt.decal_topology_law == "QUAD_STRIPS_V1"
    # Грани закона совпадают с гранями батча: ни разреза, ни склейки писателем.
    assert sizes == [len(face.ordered_vert_keys) for face in field_result.batch.faces]
    assert receipt.seam_edges == receipt.seam_edges_requested > 0
    assert not any(item[1] == writer.OUTCOME_SEAM_EDGE_MISSING for item in receipt.warnings)


def test_a_triangle_law_receipt_names_its_law_and_has_no_quads(fake_bpy):
    from dataclasses import replace

    domain = replace(
        _fake_domain(0, SQUARE, TWO_TRIANGLES), decal_topology_law="TRIANGLES_V1"
    )
    source = _source(fake_bpy)

    receipt = write_decal_object(source, [domain], offset=0.0, material_name="M")

    assert receipt.decal_topology_law == "TRIANGLES_V1"
    assert (receipt.quads, receipt.triangles, receipt.faces) == (0, 2, 2)


def test_domains_under_two_laws_are_named_together_in_the_receipt():
    from dataclasses import replace

    quads = replace(_fake_domain(0, SQUARE, QUAD_THEN_TRIANGLE), decal_topology_law="QUAD_STRIPS_V1")
    triangles = replace(_fake_domain(1, SQUARE, TWO_TRIANGLES), decal_topology_law="TRIANGLES_V1")

    arrays = build_mesh_arrays([quads, triangles], 0.0)

    assert arrays.decal_topology_law == "QUAD_STRIPS_V1,TRIANGLES_V1"
