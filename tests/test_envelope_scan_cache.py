"""Повторы родителя на каждом шаге ширины (S5): что кэшируется, чем доказан ключ и что инвалидирует.

Три повтора, которые шаг платил заново, хотя их входы между шагами не менялись:

1. ОТПЕЧАТОК МЕША ИСТОЧНИКА (`envelope_source_check`): обход bmesh и sha256 по шестнадцатеричным записям - 4 мс на `building`, 36 мс на `cover.008`. Ключ - байты шести массивов
   меша (позиции, пары вершин рёбер, швы, вершины петель, начала и длины граней); отпечаток ревизии - чистая функция тех же данных, поэтому равные байты дают равный отпечаток.
   Каждое из шести изменений, режим Edit и объект без меша отправляют шаг полным путём.
2. КОПИЯ ЭКСПОРТА БЕЗ ПОЛОСЫ (`EnvelopeTopologyExportV1.without_chart_band`): сканирование просит её на каждый домен, `replace` копировал словарь доменов на каждый вызов
   (O(доменов) на домен). Экспорт неизменяем, поэтому копия - чистая функция объекта: одна на объект, у копии своя пустая запись.
3. ЗАПРОС ПРИ ШИРИНЕ ШАГА (`envelope_scan_memo.request_at`): `dataclasses.replace` одного поля заменён прямой сборкой; равенство с `replace` и отсутствие `__post_init__` держит тест.

И два повтора, которые НЕ кэшируются, с причиной, записанной тестом: сварка и швы (`weld_vertices`, `cross_domain_seams`) - функции чисел (равенство позиций побитово, митра по нормалям, UV на
складке), которые меняются от ширины к ширине при той же структуре, а токен структуры их не определяет; кадры задач пула (`order_by_cost`) различаются на каждой ширине её текстом (`alpha_text` в задаче).
"""

from __future__ import annotations

import dataclasses
import sys
from decimal import Decimal
from fractions import Fraction
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_scan_memo as scan_memo  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_production_export import run_production  # noqa: E402
from cftuv.envelope_source_check import mesh_fingerprint, source_digest_now  # noqa: E402
from cftuv.envelope_topology_export import build_envelope_topology_export  # noqa: E402
from content_fixtures import with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 4


# --------------------------------------------------------------------------
# 1. Отпечаток меша источника
# --------------------------------------------------------------------------


class _Collection:
    """Коллекция меша Blender: длина и `foreach_get` по имени (то, что читает отпечаток)."""

    def __init__(self, length, **arrays):
        self.length = length
        self.arrays = {name: np.asarray(values) for name, values in arrays.items()}

    def __len__(self):
        return self.length

    def foreach_get(self, name, buffer):
        buffer[:] = self.arrays[name].reshape(-1).astype(buffer.dtype)


class _Source:
    """Объект Blender с мешом: квад и треугольник (5 вершин), все шесть массивов, режим."""

    def __init__(self, mode="OBJECT"):
        self.mode = mode
        self.data = SimpleNamespace(
            vertices=_Collection(5, co=np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [2, 0, 0]], dtype=np.float32)),
            edges=_Collection(6, vertices=[[0, 1], [1, 2], [2, 3], [3, 0], [1, 4], [4, 2]], use_seam=[False, True, False, False, False, False]),
            loops=_Collection(7, vertex_index=[0, 1, 2, 3, 1, 4, 2]),
            polygons=_Collection(2, loop_start=[0, 4], loop_total=[4, 3]),
        )


class _Computed:
    """Полный путь с счётчиком: возвращает новое значение на каждый вызов, чтобы промах был виден."""

    def __init__(self):
        self.calls = 0

    def __call__(self, source):
        self.calls += 1
        return f"digest-{self.calls}"


def test_an_unchanged_mesh_asks_the_full_path_once_and_the_record_answers_after():
    controller, source, full = EnvelopeDebugSessionController(), _Source(), _Computed()

    answers = [source_digest_now(controller, source, full) for _ in range(4)]

    assert answers == ["digest-1"] * 4 and full.calls == 1
    assert controller.width_source_check.digest == "digest-1" and controller.width_source_check.fingerprint == mesh_fingerprint(source)


def _mutations():
    def vertex(source):
        source.data.vertices.arrays["co"][2, 1] = 1.0001

    def edge_pair(source):
        source.data.edges.arrays["vertices"][4] = [4, 1]

    def seam(source):
        source.data.edges.arrays["use_seam"][0] = True

    def loop(source):
        source.data.loops.arrays["vertex_index"][5] = 0

    def start(source):
        source.data.polygons.arrays["loop_start"][1] = 3

    def total(source):
        source.data.polygons.arrays["loop_total"][0] = 3

    def a_face_more(source):
        source.data.polygons = _Collection(3, loop_start=[0, 4, 7], loop_total=[4, 3, 0])

    return {"vertex": vertex, "edge pair": edge_pair, "seam": seam, "loop vertex": loop, "face start": start, "face length": total, "face count": a_face_more}


@pytest.mark.parametrize("name", sorted(_mutations()))
def test_each_of_the_six_arrays_and_their_counts_sends_the_step_the_full_way(name):
    controller, source, full = EnvelopeDebugSessionController(), _Source(), _Computed()
    assert source_digest_now(controller, source, full) == "digest-1"

    _mutations()[name](source)

    assert source_digest_now(controller, source, full) == "digest-2" and full.calls == 2, name
    assert source_digest_now(controller, source, full) == "digest-2" and full.calls == 2  # и новая запись снова отвечает


def test_the_edit_mode_and_an_object_without_a_mesh_always_take_the_full_path_and_are_never_recorded():
    controller, full = EnvelopeDebugSessionController(), _Computed()
    editing = _Source(mode="EDIT")

    assert [source_digest_now(controller, editing, full) for _ in range(2)] == ["digest-1", "digest-2"]
    assert controller.width_source_check is None  # данные Edit ещё не в массивах меша: отпечатка байтов нет
    assert mesh_fingerprint(SimpleNamespace(mode="OBJECT", data=None)) is None and mesh_fingerprint(SimpleNamespace(mode="OBJECT")) is None
    assert [source_digest_now(controller, SimpleNamespace(mode="OBJECT", data=None), full) for _ in range(2)] == ["digest-3", "digest-4"]


def test_a_record_of_another_mesh_never_answers_for_this_one():
    controller, full = EnvelopeDebugSessionController(), _Computed()
    first, second = _Source(), _Source()
    second.data.vertices.arrays["co"][0, 0] = 0.5

    assert source_digest_now(controller, first, full) == "digest-1"
    assert source_digest_now(controller, second, full) == "digest-2"  # одна запись на сессию: другой меш - полный путь
    assert source_digest_now(controller, first, full) == "digest-3"


def test_the_fingerprint_is_the_bytes_of_the_arrays_not_their_identity():
    one, other = _Source(), _Source()

    assert mesh_fingerprint(one) == mesh_fingerprint(other) and one.data is not other.data
    other.data.vertices.arrays["co"][4, 0] = 2.5
    assert mesh_fingerprint(one) != mesh_fingerprint(other)


def test_the_width_step_asks_the_source_check_and_names_a_changed_source(monkeypatch):
    """`_begin` сверяет отпечаток через запись сессии: правка меша между шагами - `SOURCE_CHANGED`, без правки полный путь не вызывается."""

    from cftuv import envelope_width_live as live

    controller, source, full = EnvelopeDebugSessionController(), _Source(), _Computed()
    monkeypatch.setattr(live, "_current_digest", full)

    assert [live._source_digest(controller, source) for _ in range(3)] == ["digest-1"] * 3 and full.calls == 1
    source.data.vertices.arrays["co"][1, 2] = 0.25
    assert live._source_digest(controller, source) == "digest-2"


# --------------------------------------------------------------------------
# 2. Копия экспорта без полосы
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def topology():
    bundle = with_revision(quad_row_bundle(ROW), "row")
    return build_envelope_topology_export(bundle).with_chart_band(None, frozenset({0, 1}), Fraction(1, 4))


def test_the_export_without_a_band_is_one_copy_per_object_equal_to_the_replaced_one(topology):
    first = topology.without_chart_band()

    assert first is topology.without_chart_band() and first.chart_band is None and topology.chart_band is not None
    assert first == dataclasses.replace(topology, chart_band=None)  # то же значение, что давал `replace` на каждый вызов
    assert first.without_chart_band() is first  # у копии без полосы копировать нечего


def test_another_band_is_another_object_with_its_own_copy_and_a_replaced_export_does_not_share_the_record(topology):
    other = topology.with_chart_band(None, frozenset({2}), Fraction(1, 4))
    narrower = dataclasses.replace(topology, developable_stretch_budget=Fraction(1, 7))

    assert other is not topology and other.without_chart_band() is not topology.without_chart_band()
    assert other.without_chart_band() == topology.without_chart_band()  # сама метрика целого патча от выделения не зависит
    assert narrower._derived is not topology._derived and narrower.without_chart_band() is not topology.without_chart_band()
    assert narrower.without_chart_band().developable_stretch_budget == Fraction(1, 7)  # запись допуска не утекла в копию объекта с другим допуском


# --------------------------------------------------------------------------
# 3. Запрос при ширине шага
# --------------------------------------------------------------------------


def _record_with_a_request():
    controller = EnvelopeDebugSessionController()
    bundle = with_revision(quad_row_bundle(ROW), "row")
    run_production(
        controller, bundle, frozenset(range(ROW)), 0.25, source_object_key="o", source_data_key="m", density=None, workers=0, domain_pool=None
    )
    (records,) = [item for item in controller.scan_memo._runs.values()]
    return next(iter(records.values()))


@pytest.mark.parametrize("alpha", [Decimal("0.2262"), Decimal("0.25"), Decimal("0.0421422086656093"), Decimal("1E-9"), Decimal("12")])
def test_the_request_at_a_width_is_what_replace_of_one_field_gave(alpha):
    from cftuv.envelope_request_export import _load_kernel

    record = _record_with_a_request()
    kernel, _ = _load_kernel()

    fast = scan_memo.request_at(record, alpha)
    reference = dataclasses.replace(record.request, requested_alpha=kernel.LocalLengthV1(alpha))

    assert type(fast) is type(reference) and fast == reference and hash(fast) == hash(reference)
    assert {item.name: getattr(fast, item.name) for item in dataclasses.fields(fast)} == {item.name: getattr(reference, item.name) for item in dataclasses.fields(reference)}
    assert fast is not record.request and record.request.requested_alpha != fast.requested_alpha or alpha == Decimal("0.25")


def test_the_request_has_no_post_init_so_the_direct_copy_skips_nothing_replace_would_run():
    """Прямая сборка не вызывает `__post_init__`: появится он у запроса - этот тест падает, и копию надо переделать (либо вернуть `replace`)."""

    from cftuv_envelope.contracts.request import DecalRequestV1

    assert "__post_init__" not in vars(DecalRequestV1) and not hasattr(DecalRequestV1, "__post_init__")


# --------------------------------------------------------------------------
# Идентичность домена патча
# --------------------------------------------------------------------------


def test_the_selection_by_patch_names_the_domains_the_scan_derived():
    from cftuv.envelope_request_export import _typed_value

    controller = EnvelopeDebugSessionController()
    bundle = with_revision(quad_row_bundle(ROW), "row")

    run = run_production(controller, bundle, frozenset(range(ROW)), 0.25, source_object_key="o", source_data_key="m", density=None, workers=0, domain_pool=None)

    assert run.selected_by_patch == tuple((patch, (patch,)) for patch in range(ROW))
    assert [item.domain_id for item in run.results] == [_typed_value("patch-domain", run.revision, patch) for patch in range(ROW)]


# --------------------------------------------------------------------------
# Что НЕ кэшируется, и почему
# --------------------------------------------------------------------------


def test_the_weld_is_a_function_of_the_numbers_not_of_the_structure_so_it_is_not_cached_by_the_token():
    """Два прогона с той же структурой (ссылки вершин те же, токен тот же) и позициями, что различаются одним ulp, дают разную сварку: общая вершина сварена лишь при побитовом равенстве."""

    from cftuv.envelope_production_weld import COUNTER_WELD_VERTICES_MERGED, DomainVerticesV1, weld_vertices

    shared = "location:src:7"

    def domains(second_x):
        first = DomainVerticesV1(0, ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)), ((0.0, 0.0, 1.0),) * 2, (shared, None))
        other = DomainVerticesV1(1, ((second_x, 0.0, 0.0), (0.0, 1.0, 0.0)), ((0.0, 0.0, 1.0),) * 2, (shared, None))
        return [first, other]

    equal = weld_vertices(domains(0.0), 0.0)
    apart = weld_vertices(domains(np.nextafter(0.0, 1.0)), 0.0)

    assert len(equal.positions) == 3 and len(apart.positions) == 4  # та же структура ссылок, но сварка разная
    assert dict(equal.counters)[COUNTER_WELD_VERTICES_MERGED] == 1 and dict(apart.counters)[COUNTER_WELD_VERTICES_MERGED] == 0


def test_the_task_frames_of_the_pool_differ_at_every_width_by_their_text_so_they_are_not_cached():
    from cftuv.envelope_domain_pool import DomainTaskV1, order_by_cost

    def frame(alpha_text):
        task = DomainTaskV1(0, 3, "domain", None, None, alpha_text, frozenset({1}))
        return order_by_cost([task])[0][1]

    assert frame("0.2262") != frame("0.2284") and frame("0.2262") == frame("0.2262")
