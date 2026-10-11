"""WIDTH_REFRESH_LEAN_V1: обновление точного пути ширины без работы, нужной только фиксации.

Что держат тесты (доказательство того, что lean отличается от COMMIT ТОЛЬКО пропущенной работой):

1. ГЕОМЕТРИЯ. На тёплой сессии lean-обновление и полное на той же ширине дают побитово одно: исходы доменов, детали, счётчики материализатора, нормали, подпись
   структуры, заверенный интервал, вид домена и шесть массивов меша (позиции, грани, UV, владельцы граней, швы, состав) — в родителе и в пуле (пикл по трубе).
   Массивы lean-результата с отчётами совпадают с полными ЦЕЛИКОМ, включая дайджест: вид полон.
2. ЧТО ПРОПУЩЕНО, а что нет. Воркер не пакует батч (`heavy` пуст, ответ меньше), массивы без отчётов (`reports=False`: дайджест и отчёты пусты, шесть массивов те же),
   а проверки ядра, влияющие на исход (`VALIDATE`), остаются: домен, который валидатор отказывает, отказан и в lean-обновлении тем же исходом.
3. КЭШИ. Lean-результат не лежит ни в кэше результатов сессии, ни в хранилище по содержимому, и оба отказывают ему названно (`LeanResultError`): он не может ответить на
   поиск зафиксированного результата. Следующий прогон COMMIT на той же ширине считает заново и кладёт полный результат.
4. ЗАПРЕТЫ. У lean-результата нет батча, нормалей вершин и дайджеста (чтение — `LeanResultError`, а не молчаливое `None`); lean-массивы не принимает писатель с квитанцией.
5. ИМЯ. Класс обновления назван в прогоне, в профиле и в строке панели; у COMMIT строка и счётчики прежние.
"""

from __future__ import annotations

import pickle
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_export as production  # noqa: E402
from cftuv.envelope_debug_session import EnvelopeDebugSessionController  # noqa: E402
from cftuv.envelope_production_export import (  # noqa: E402
    PRODUCTION_REFRESH_LEAN_DOMAINS,
    PRODUCTION_REFRESH_LEAN_FULL_DOMAINS,
    PRODUCTION_RESULT_CACHE_HIT,
    PRODUCTION_RESULT_CACHE_MISS,
    REFRESH_COMMIT,
    REFRESH_LEAN,
    LeanResultError,
    production_timing_text,
    run_production,
)
from cftuv.envelope_production_mesh import (  # noqa: E402
    OUTCOME_LEAN_ARRAYS,
    ProductionWriteError,
    build_mesh_arrays,
    write_decal_object,
)
from cftuv.envelope_production_view import build_domain_view  # noqa: E402
from cftuv.envelope_production_weld import OUTCOME_WELD_HALF_EDGE_CONFLICT  # noqa: E402
from content_fixtures import InProcessPool, with_revision  # noqa: E402
from envelope_fixture_bundles import pin_near_planar_only, quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))
ALPHA = 0.25
STEP = 0.2262
OFFSET = 0.001

#: Поля ответа домена, которые обязаны совпасть (метки запуска — секунды, размещение, путь шага — в ответ не входят).
ANSWER = (
    "patch_id",
    "domain_id",
    "outcome",
    "detail",
    "counters",
    "normal",
    "diagnostics",
    "source_normal",
    "chart_orientation",
    "offset_normal_law",
    "offset_normals_digest",
    "decal_topology_law",
    "structure_digest",
    "alpha_interval",
)
#: Шесть массивов меша и состав: то, что уходит в Blender.
GEOMETRY = (
    "positions",
    "faces",
    "uvs",
    "face_domain",
    "face_owner",
    "seam_edges",
    "domains",
    "skipped",
    "domain_keys",
    "domain_vertex_index",
    "domain_loop_counts",
    "domain_tokens",
    "source_revision",
    "decal_topology_law",
)


@pytest.fixture(autouse=True)
def _pool_always(monkeypatch):
    """Малая партия остаётся в родителе (порог); пулу нужны байты пересылки."""

    from cftuv import envelope_queue_pool

    monkeypatch.setattr(envelope_queue_pool, "COVERAGE_POOL_MIN_BYTES", 0)


@pytest.fixture(scope="module")
def row():
    return with_revision(quad_row_bundle(ROW), "row")


@pytest.fixture(scope="module")
def clipped_row():
    """Последний патч слегка вышел из плоскости: его домен near-planar с РЕЗКОЙ по граням источника (вершины на рёбрах источника), остальные плоские."""

    return with_revision(quad_row_bundle(ROW, lifted_corner=0.3), "clipped")


@pytest.fixture(scope="module")
def unfolded_row():
    """Последний патч вышел из плоскости сильно: его домен — развёртка (у каждой вершины своя нормаль смещения), остальные плоские."""

    return with_revision(quad_row_bundle(ROW, lifted_corner=1.0), "unfolded")


def _press(controller, bundle, alpha, *, lean=False, pooled=False):
    return run_production(
        controller,
        bundle,
        EDGES,
        alpha,
        source_object_key="object",
        source_data_key="mesh",
        density=None,
        workers=2 if pooled else 0,
        domain_pool=InProcessPool() if pooled else None,
        lean=lean,
    )


def _warm(bundle, *, pooled=False):
    """Сессия после кнопки: подготовки домена в кэше (без них домен холодный, а холодный домен всегда идёт полным путём)."""

    controller = EnvelopeDebugSessionController()
    _press(controller, bundle, ALPHA, pooled=pooled)
    return controller


def _answer(result):
    return tuple(repr(getattr(result, name)) for name in ANSWER)


def _view_of(result):
    view = result.view
    return build_domain_view(result) if view is None else view


def _both(bundle, *, pooled):
    """`(полный, lean)` на ширине STEP: две тёплые сессии с одной историей, различается только класс обновления."""

    full = _press(_warm(bundle, pooled=pooled), bundle, STEP, pooled=pooled)
    lean_controller = _warm(bundle, pooled=pooled)
    lean = _press(lean_controller, bundle, STEP, lean=True, pooled=pooled)
    return full, lean, lean_controller


# --------------------------------------------------------------------------
# 1. Геометрия и ответ доменов — побитово
# --------------------------------------------------------------------------


@pytest.mark.parametrize("pooled", [False, True], ids=["parent", "pool"])
@pytest.mark.parametrize("fixture", ["row", "clipped_row", "unfolded_row"])
def test_the_lean_refresh_answers_every_domain_exactly_as_the_full_one(request, fixture, pooled):
    bundle = request.getfixturevalue(fixture)

    full, lean, _controller = _both(bundle, pooled=pooled)

    assert len(full.results) == len(lean.results) == ROW
    assert [_answer(item) for item in full.results] == [_answer(item) for item in lean.results]
    assert [repr(_view_of(item)) for item in full.results] == [repr(_view_of(item)) for item in lean.results]  # плоские массивы, из которых пишется меш
    assert {item.outcome for item in full.results} == {"MATERIALIZED"}
    if fixture == "unfolded_row":
        assert any(item.vertex_normals for item in full.results)  # домен-развёртка: у каждой вершины своя нормаль смещения, и она тоже сверена (в виде)
    if fixture == "clipped_row":
        assert any(item.clip_memo for item in full.results)  # домен с резкой по граням источника
    assert all(item.lean for item in lean.results)
    assert not any(item.lean for item in full.results)


def test_a_refused_domain_is_refused_the_same_way_by_the_lean_refresh(unfolded_row, monkeypatch):
    pin_near_planar_only(monkeypatch)  # лестница кривизны на near-planar: сильно приподнятый патч отказан на метрике

    full, lean, _controller = _both(unfolded_row, pooled=False)

    assert any(not item.is_materialized for item in full.results)
    assert [_answer(item) for item in full.results] == [_answer(item) for item in lean.results]
    assert all(item.lean for item in lean.results if item.is_materialized)
    assert [item.outcome for item in lean.results] == [item.outcome for item in full.results]


@pytest.mark.parametrize("pooled", [False, True], ids=["parent", "pool"])
def test_the_lean_arrays_are_the_full_arrays_bit_for_bit_in_the_six_mesh_arrays_and_in_all_of_them_with_reports(row, pooled):
    full, lean, _controller = _both(row, pooled=pooled)

    reference = build_mesh_arrays(full.results, OFFSET)
    lean_arrays = build_mesh_arrays(lean.results, OFFSET, reports=False)
    reported = build_mesh_arrays(lean.results, OFFSET)

    assert reference.reports and reported.reports and not lean_arrays.reports
    assert [repr(getattr(lean_arrays, name)) for name in GEOMETRY] == [repr(getattr(reference, name)) for name in GEOMETRY]
    assert reported == reference  # вид lean-результата полон: с отчётами массивы совпадают целиком, включая дайджест
    assert reference.digest and lean_arrays.digest == "" and lean_arrays.offset_counters == () and lean_arrays.seam_counters == ()
    assert lean_arrays.positions and lean_arrays.faces  # проверка не пуста: геометрия есть


def test_the_lean_arrays_keep_the_geometry_counters_the_reports_do_not_touch(row):
    full, lean, _controller = _both(row, pooled=False)

    reference = dict(build_mesh_arrays(full.results, OFFSET).weld_counters)
    lean_arrays = dict(build_mesh_arrays(lean.results, OFFSET, reports=False).weld_counters)

    assert set(reference) - set(lean_arrays) == {OUTCOME_WELD_HALF_EDGE_CONFLICT}  # счётчик обхода рёбер — отчёт: lean его не пишет, а не пишет нулём
    assert {name: value for name, value in reference.items() if name in lean_arrays} == lean_arrays


# --------------------------------------------------------------------------
# 2. Что пропущено и что осталось
# --------------------------------------------------------------------------


def test_the_worker_does_not_pack_the_batch_of_a_lean_domain_and_the_reply_is_smaller(row):
    full, lean, _controller = _both(row, pooled=True)

    sent_full = [pickle.dumps(item) for item in full.results]
    sent_lean = [pickle.dumps(item) for item in lean.results]

    assert all(item.__dict__.get("heavy") is not None for item in full.results)
    assert all(item.__dict__.get("heavy") is None and item.view is not None for item in lean.results)
    assert sum(map(len, sent_lean)) < sum(map(len, sent_full)) * 0.7  # батч — основная доля байтов ответа (на малом домене вид сравним с батчем: на `building` 1.6 МБ против 0.3)
    assert all(pickle.loads(blob).lean for blob in sent_lean)  # метка переживает пересылку


def test_a_domain_the_validator_refuses_is_refused_by_the_lean_refresh_with_the_same_outcome(row, monkeypatch):
    """VALIDATE влияет на исход домена, поэтому lean его НЕ пропускает: иначе lean показал бы геометрию, которую полный путь отказал."""

    from cftuv_envelope.materialize import domain as domain_module

    issue = SimpleNamespace(code=SimpleNamespace(value="INJECTED_ISSUE"), path=("batch", "faces"))
    monkeypatch.setattr(domain_module, "validate_geometry_batch", lambda *args, **kwargs: [issue])

    full, lean, _controller = _both(row, pooled=False)

    assert {item.outcome for item in full.results} == {"BATCH_DID_NOT_VALIDATE"}
    assert [_answer(item) for item in lean.results] == [_answer(item) for item in full.results]
    assert not any(item.is_materialized for item in lean.results)


def test_a_cold_session_runs_every_domain_the_full_way_inside_a_lean_run(row):
    """Холодный домен считает воркер вместе с подготовкой, и его результат ложится в кэши: lean его не касается (профиль называет число)."""

    controller = EnvelopeDebugSessionController()

    run = _press(controller, row, ALPHA, lean=True)

    assert run.refresh == REFRESH_LEAN and run.cold
    assert run.counter(PRODUCTION_REFRESH_LEAN_DOMAINS) == 0 and run.counter(PRODUCTION_REFRESH_LEAN_FULL_DOMAINS) == ROW
    assert not any(item.lean for item in run.results)
    assert controller.production_result_count == ROW  # полные результаты — в кэше, как у кнопки


# --------------------------------------------------------------------------
# 3. Кэши: lean-результат не зафиксирован
# --------------------------------------------------------------------------


@pytest.mark.parametrize("pooled", [False, True], ids=["parent", "pool"])
def test_a_lean_result_is_stored_nowhere_and_a_commit_at_the_same_width_computes_anew(row, pooled):
    controller = _warm(row, pooled=pooled)
    cached_before = controller.production_result_count
    stored_before = controller.content_store.result_count

    first = _press(controller, row, STEP, lean=True, pooled=pooled)
    again = _press(controller, row, STEP, lean=True, pooled=pooled)

    assert first.counter(PRODUCTION_REFRESH_LEAN_DOMAINS) == again.counter(PRODUCTION_REFRESH_LEAN_DOMAINS) == ROW
    assert controller.production_result_count == cached_before and controller.content_store.result_count == stored_before
    assert again.counter(PRODUCTION_RESULT_CACHE_HIT) == 0 and again.counter(PRODUCTION_RESULT_CACHE_MISS) == ROW  # второе lean-обновление не нашло первого

    commit = _press(controller, row, STEP, pooled=pooled)

    assert commit.refresh == REFRESH_COMMIT
    assert commit.counter(PRODUCTION_RESULT_CACHE_HIT) == 0 and commit.counter(PRODUCTION_RESULT_CACHE_MISS) == ROW  # lean не ответил на поиск зафиксированного
    assert not any(item.lean for item in commit.results)
    assert all(item.batch is not None and item.content_digest for item in commit.results)  # зафиксированный ответ полон
    assert controller.production_result_count == cached_before + ROW
    repeat = _press(controller, row, STEP, pooled=pooled)
    assert repeat.counter(PRODUCTION_RESULT_CACHE_HIT) == ROW  # а вот COMMIT кэш уже отдаёт


def test_the_commit_after_lean_refreshes_is_the_commit_without_them(row):
    """Следы lean-обновлений (память шага воркера, записи сборки) не меняют зафиксированный ответ: тот же COMMIT на другой сессии без них."""

    clean = _press(_warm(row), row, STEP)
    controller = _warm(row)
    for index in range(1, 4):
        _press(controller, row, ALPHA + 0.01 * index, lean=True)
    touched = _press(controller, row, STEP)

    assert [_answer(item) for item in touched.results] == [_answer(item) for item in clean.results]
    assert [item.content_digest for item in touched.results] == [item.content_digest for item in clean.results]
    assert build_mesh_arrays(touched.results, OFFSET) == build_mesh_arrays(clean.results, OFFSET)


def test_both_storage_boundaries_refuse_a_lean_result_by_name(row):
    controller = _warm(row)
    lean = next(item for item in _press(controller, row, STEP, lean=True).results if item.is_materialized)

    with pytest.raises(LeanResultError, match="LEAN_RESULT_MISUSED"):
        controller.remember_production_result(("key",), lean)
    with pytest.raises(LeanResultError, match="LEAN_RESULT_MISUSED"):
        controller.content_store.register_result("key", ("slot",), lean)


# --------------------------------------------------------------------------
# 4. Запреты
# --------------------------------------------------------------------------


def test_a_lean_result_has_no_batch_no_normals_no_digest_and_says_so_loudly(row):
    lean = next(item for item in _press(_warm(row), row, STEP, lean=True).results if item.is_materialized)

    for name in ("batch", "vertex_normals", "content_digest"):
        with pytest.raises(LeanResultError, match="LEAN_RESULT_MISUSED"):
            getattr(lean, name)
    with pytest.raises(LeanResultError):
        lean.materialized()
    with pytest.raises(LeanResultError):
        lean.with_changes(batch=None)
    assert lean.with_changes(seconds=1.5).lean and lean.with_changes(placement="parent").view is lean.view  # метки запуска lean не разворачивают
    assert production.deferred_result(lean) is lean and production.lean_result(lean) is lean
    assert lean.is_materialized and lean.view is not None and lean.heavy is None


def test_a_refusal_stays_a_refusal_under_lean():
    refusal = production._refusal(1, "domain", "SOME_REFUSAL", "why")

    assert production.lean_result(refusal) is refusal and not refusal.lean and not refusal.is_materialized


def test_lean_arrays_are_not_accepted_by_the_writer_with_a_receipt(row):
    lean = _press(_warm(row), row, STEP, lean=True)
    arrays = build_mesh_arrays(lean.results, OFFSET, reports=False)

    with pytest.raises(ProductionWriteError) as caught:
        write_decal_object(object(), lean.results, arrays=arrays)

    assert caught.value.outcome == OUTCOME_LEAN_ARRAYS


# --------------------------------------------------------------------------
# 5. Имя
# --------------------------------------------------------------------------


def test_the_refresh_class_is_named_in_the_run_the_profile_and_the_panel_line(row):
    controller = _warm(row)
    commit = _press(controller, row, STEP)
    lean = _press(controller, row, STEP + 0.01, lean=True)

    assert (commit.refresh, lean.refresh) == (REFRESH_COMMIT, REFRESH_LEAN) == ("COMMIT", "WIDTH_REFRESH_LEAN_V1")
    assert commit.counter(PRODUCTION_REFRESH_LEAN_DOMAINS) is None  # у COMMIT счётчиков lean нет: профиль прежний
    assert lean.counter(PRODUCTION_REFRESH_LEAN_DOMAINS) == ROW and lean.counter(PRODUCTION_REFRESH_LEAN_FULL_DOMAINS) == 0
    assert "WIDTH_REFRESH_LEAN_V1" not in production_timing_text(commit)
    assert f"| WIDTH_REFRESH_LEAN_V1 {ROW} lean, 0 full" in production_timing_text(lean)


def test_a_lean_result_survives_the_pipe_with_its_view(row):
    lean = next(item for item in _press(_warm(row), row, STEP, lean=True).results if item.is_materialized)

    back = pickle.loads(pickle.dumps(lean))

    assert back.lean and repr(back.view) == repr(lean.view) and _answer(back) == _answer(lean)



# ==========================================================================
# 6. Точный lean-кадр во время перетаскивания: меш и образец на экране заменены (`lean_exact_now`), отмена возвращает начало перетаскивания
# ==========================================================================

import dataclasses  # noqa: E402

import numpy as np  # noqa: E402

from cftuv import envelope_production_mesh as production_mesh  # noqa: E402
from cftuv import envelope_width_live as live  # noqa: E402
from cftuv import envelope_width_mesh_preview as preview  # noqa: E402
from cftuv import envelope_width_preview_model as pm  # noqa: E402
from test_envelope_width_preview_model import _fold_sample, _owned  # noqa: E402


def _exact(alpha, **kwargs):
    """Точный прогон на другой ширине с той же раскладкой, что у образца на экране (`_owned` кладёт ширину 0.25): то, что отдаёт поток счёта."""

    sample = _fold_sample(alpha)[0]
    return SimpleNamespace(sample=dataclasses.replace(sample, **kwargs), model=None, check=None, trust=None, trust_events=())


def _relaid(sample):
    """Тот же образец с другой раскладкой (домен сменил структуру): отпечаток другой, образ тот же."""

    return dataclasses.replace(sample, layout_digest="another layout")


def _frames_written(world):
    return len(world.mesh.vertices.sets) + len(world.mesh.uv_layers[0].data.sets) + len(world.mesh.attributes[0].data.sets)


def _width_of(world):
    return world.mesh[production_mesh.DECAL_WIDTH_PROPERTY]


def test_the_mesh_image_of_a_sample_is_the_mesh_the_writer_lays_down(row):
    sample = _row_sample_of(row, 0.25)
    arrays = build_mesh_arrays(_press(_warm(row), row, 0.25).results, OFFSET)
    image = sample.image

    assert [tuple(image.face_vertices[start : start + len(face)].tolist()) for start, face in zip(image.face_starts.tolist(), arrays.faces)] == list(arrays.faces)
    assert image.face_domain.tolist() == list(arrays.face_domain) and image.face_owner.tolist() == list(arrays.face_owner)
    assert [tuple(pair) for pair in image.seams.tolist()] == list(arrays.seam_edges) and image.face_count == len(arrays.faces)
    assert all(item.dtype == np.int32 for item in (image.face_starts, image.face_vertices, image.face_domain, image.face_owner, image.seams))


def test_every_sample_carries_the_layout_digest_of_the_mesh_it_lays_down(row):
    first, second = _row_sample_of(row, 0.25), _row_sample_of(row, 0.2525)

    assert first.layout_digest and first.layout_digest == second.layout_digest  # тот же состав, другие числа
    assert len({first.layout_digest, _fold_sample(0.25)[0].layout_digest}) == 2  # другой состав - другой отпечаток
    other = dataclasses.replace(second.image, face_owner=second.image.face_owner + 3)
    assert pm.layout_digest_of(other, first.vertex_count, first.loop_count) == first.layout_digest  # номера владельцев раскладкой не считаются


def _row_sample_of(bundle, alpha):
    run = _press(_warm(bundle), bundle, alpha)
    arrays = build_mesh_arrays(run.results, OFFSET)
    return preview.build_sample(run.results, arrays, ("row",), alpha)


def test_a_lean_frame_with_the_same_layout_writes_positions_uv_and_owners_and_replaces_the_display(monkeypatch):
    world = _owned(monkeypatch)
    controller, base, mesh = world.controller, world.base, world.mesh
    frame = _exact(0.27, image=dataclasses.replace(_fold_sample(0.27)[0].image, face_owner=base.image.face_owner + 5))
    sample = frame.sample
    assert sample.layout_digest == base.layout_digest and sample.alpha != base.alpha

    assert preview.lean_exact_now(controller, frame) == ""

    assert [name for name, _ in mesh.vertices.sets] == ["co"] and mesh.uv_layers[0].data.sets[0][0] == "uv" and mesh.updated == 1
    assert np.array_equal(mesh.vertices.sets[0][1], np.asarray(sample.positions, dtype=np.float32).reshape(-1))
    assert np.array_equal(mesh.uv_layers[0].data.sets[0][1], np.asarray(sample.uvs, dtype=np.float32).reshape(-1))
    assert np.array_equal(mesh.attributes[0].data.sets[0][1], sample.image.face_owner)  # номера владельцев - тот же атрибут граней, что у точной записи
    assert _width_of(world) == sample.alpha  # ширина записи меша - ширина кадра (её читают проверка владения и сверка после истории)
    assert controller.width_displayed is sample and controller.width_aux == (base,)  # образец на экране заменён, прежний стал вспомогательным
    assert controller.width_mesh_owner.sample is sample and controller.width_mesh_owner.generation == 2  # владение снято заново, под образцом кадра
    state = controller.width_mesh_preview
    assert state.outcome == REFRESH_LEAN and state.alpha == 0.27 and state.live == len(base.domains) and state.held == 0
    log = preview.log_of(controller)
    assert (log.lean_frames, log.lean_rebuilds) == (1, 0)
    assert "WIDTH_REFRESH_LEAN_V1: exact geometry at width 0.27" in preview.status_lines(controller)[0]


def test_a_lean_frame_with_another_layout_rebuilds_the_mesh_in_place_from_the_image(monkeypatch):
    world = _owned(monkeypatch)
    controller, base = world.controller, world.base
    rebuilt: list = []

    def record(mesh, positions, uvs, image):
        rebuilt.append((mesh, positions, uvs, image))
        return 3

    monkeypatch.setattr(production_mesh, "rebuild_mesh_from_image", record)
    frame = _exact(0.27)
    frame = SimpleNamespace(**{**vars(frame), "sample": _relaid(frame.sample)})

    assert preview.lean_exact_now(controller, frame) == ""

    assert len(rebuilt) == 1 and rebuilt[0][0] is world.mesh and rebuilt[0][1] is frame.sample.positions and rebuilt[0][3] is frame.sample.image
    assert _frames_written(world) == 0  # позиции и UV не пишутся отдельно: меш собран целиком
    assert controller.width_displayed is frame.sample and _width_of(world) == 0.27 and controller.width_mesh_owner.sample is frame.sample
    assert (preview.log_of(controller).lean_frames, preview.log_of(controller).lean_rebuilds) == (1, 1)
    assert base is not controller.width_displayed


def test_cancel_returns_what_the_drag_started_from_after_lean_frames_replaced_the_display(monkeypatch):
    world = _owned(monkeypatch)
    controller, base, model, mesh = world.controller, world.base, world.model, world.mesh
    preview.begin_drag(controller)
    assert controller.width_drag_active and controller.width_drag_origin is None  # что показывали, запоминает первый кадр, а не вход в инструмент

    assert preview.lean_exact_now(controller, _exact(0.27)) == ""
    assert controller.width_drag_origin.displayed is base
    assert preview.lean_exact_now(controller, _exact(0.28)) == ""
    assert controller.width_drag_origin.displayed is base  # второй кадр начало не меняет
    assert controller.width_displayed.alpha == 0.28 and controller.width_aux[0].alpha == 0.27

    assert preview.finish_drag(controller, restore=True) is True

    assert not controller.width_drag_active and controller.width_drag_origin is None
    assert controller.width_displayed is base and controller.width_aux == () and controller.width_model is model  # образец, вспомогательные, модель - начала перетаскивания
    assert controller.width_mesh_owner.sample is base and controller.width_mesh_preview is None and _width_of(world) == base.alpha
    assert np.array_equal(mesh.vertices.sets[-1][1], np.asarray(base.positions, dtype=np.float32).reshape(-1))  # меш: позиции базы побитово
    assert np.array_equal(mesh.uv_layers[0].data.sets[-1][1], np.asarray(base.uvs, dtype=np.float32).reshape(-1))
    assert np.array_equal(mesh.attributes[0].data.sets[-1][1], base.image.face_owner)  # и номера владельцев базы
    assert preview.log_of(controller).lean_origin_restores == 1


def test_cancel_after_a_rebuild_rebuilds_the_origin_from_its_image(monkeypatch):
    world = _owned(monkeypatch)
    controller, base = world.controller, world.base
    rebuilt: list = []
    monkeypatch.setattr(production_mesh, "rebuild_mesh_from_image", lambda mesh, positions, uvs, image: rebuilt.append(image) or 0)
    preview.begin_drag(controller)
    relaid = _exact(0.27)
    relaid = SimpleNamespace(**{**vars(relaid), "sample": _relaid(relaid.sample)})
    assert preview.lean_exact_now(controller, relaid) == ""

    assert preview.finish_drag(controller, restore=True) is True

    assert rebuilt == [relaid.sample.image, base.image]  # кадр пересобрал меш, отмена пересобрала его обратно по образу начала
    assert controller.width_displayed is base and _width_of(world) == base.alpha


def test_the_confirm_leaves_the_mesh_as_the_frames_left_it_and_forgets_the_origin(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    preview.begin_drag(controller)
    assert preview.lean_exact_now(controller, _exact(0.27)) == ""
    written = _frames_written(world)

    assert preview.finish_drag(controller, restore=False) is False

    assert not controller.width_drag_active and controller.width_drag_origin is None and _frames_written(world) == written
    assert controller.width_displayed.alpha == 0.27  # зафиксированный ответ заменит и это


def test_the_origin_holds_what_stood_just_before_the_first_frame_so_a_prime_that_landed_earlier_is_kept(monkeypatch):
    world = _owned(monkeypatch)
    controller, base = world.controller, world.base
    preview.begin_drag(controller)
    late_aux = (_fold_sample(0.2525)[0],)
    controller.width_aux = late_aux  # затравка легла уже за время перетаскивания, до первого кадра

    assert preview.lean_exact_now(controller, _exact(0.27)) == ""
    assert preview.finish_drag(controller, restore=True) is True

    assert controller.width_displayed is base and controller.width_aux is late_aux


def test_cancel_without_lean_frames_is_the_old_cancel(monkeypatch):
    world = _owned(monkeypatch)
    controller, base = world.controller, world.base
    preview.begin_drag(controller)
    assert preview.preview_mesh_now(controller, 0.26) is not None

    assert preview.finish_drag(controller, restore=True) is True

    assert controller.width_displayed is base and controller.width_mesh_preview is None
    assert np.array_equal(world.mesh.vertices.sets[-1][1], np.asarray(base.positions, dtype=np.float32).reshape(-1))
    assert preview.log_of(controller).lean_origin_restores == 0  # образец не менялся: возвращать нечего, кроме позиций кадра модели


def test_a_lean_frame_that_does_not_fit_is_refused_by_name_and_writes_nothing(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    frame = _exact(0.27)

    other = SimpleNamespace(**{**vars(frame), "sample": dataclasses.replace(frame.sample, key=("another source",))})
    assert preview.lean_exact_now(controller, other) == preview.LEAN_KEY_MISMATCH
    assert preview.log_of(controller).refusals == {preview.LEAN_KEY_MISMATCH: 1}
    assert _frames_written(world) == 0 and controller.width_mesh_preview is None and controller.width_model is world.model

    controller.width_displayed = None
    assert preview.lean_exact_now(controller, frame) == preview.LEAN_NO_BASE and _frames_written(world) == 0


def test_a_mesh_that_is_not_the_owned_base_takes_no_lean_frame_and_drops_the_model_by_name(monkeypatch):
    world = _owned(monkeypatch)
    controller = world.controller
    controller.width_mesh_owner.sample = object()  # владение записано под другим образцом

    reason = preview.lean_exact_now(controller, _exact(0.27))

    assert reason == preview.LAYOUT_GENERATION_STALE and _frames_written(world) == 0
    assert controller.width_model is None and controller.width_displayed is None  # как у кадра модели: чужой меш снимает модель названно


def test_a_mesh_without_the_owner_attribute_takes_no_lean_frame_and_the_model_is_dropped_by_name(monkeypatch):
    world = _owned(monkeypatch)
    world.mesh.attributes.clear()

    reason = preview.lean_exact_now(world.controller, _exact(0.27))

    assert reason == production_mesh.OUTCOME_PREVIEW_NO_OWNER_ATTRIBUTE and world.controller.width_model is None


# ==========================================================================
# 7. Заказ точного lean-обновления инструментом (`request_drag_tick`)
# ==========================================================================

from test_envelope_width_live import _World  # noqa: E402


def _drag_world(monkeypatch, *, calls):
    """Мир живой ширины после кнопки с образцом на экране и перетаскиванием, чей lean-кадр пишет подмена (запись в меш проверена выше)."""

    world = _World(monkeypatch)
    controller = world.controller
    arrays = build_mesh_arrays(world.run.results, OFFSET)
    controller.width_displayed = preview.build_sample(world.run.results, arrays, preview.sample_key(world.record, OFFSET), 0.25)
    preview.begin_drag(controller)
    world.mesh_settings.drag_exact_ticks = True
    monkeypatch.setattr(live, "lean_exact_now", lambda controller, run: calls.append(run) or "")
    return world


def test_the_tool_orders_an_exact_tick_only_while_the_drag_runs_the_switch_is_on_and_a_sample_is_on_screen(monkeypatch):
    world = _drag_world(monkeypatch, calls=[])
    controller, context = world.controller, world.context

    assert live.request_drag_tick(context, controller, 0.27) is True
    assert controller.width_live.counters.requested == 1
    displayed = controller.width_displayed
    for name, off, on in (
        ("drag_exact_ticks", False, True),  # настройка выключена
        ("width_drag_active", False, True),  # перетаскивание кончилось
        ("width_displayed", None, displayed),  # на экране нет образца точной записи
    ):
        holder = world.mesh_settings if name == "drag_exact_ticks" else controller
        setattr(holder, name, off)
        assert live.request_drag_tick(context, controller, 0.28) is False, name
        setattr(holder, name, on)
    assert controller.width_live.counters.requested == 1
    world.settings.envelope_debug_fan_density = "4"  # политика сменилась после кнопки: тик не заказывается
    assert live.request_drag_tick(context, controller, 0.29) is False
    assert controller.width_live.counters.requested == 1


def test_an_exact_tick_is_a_lean_run_that_writes_no_committed_result_and_fills_no_cache(monkeypatch):
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = world.controller
    cached, stored = controller.production_result_count, controller.content_store.result_count

    assert live.request_drag_tick(world.context, controller, 0.27)
    live.settle_width_live(controller)

    scheduler = controller.width_live
    assert (scheduler.counters.requested, scheduler.counters.applied) == (1, 1) and len(calls) == 1
    run = calls[0].run
    assert run.refresh == REFRESH_LEAN and run.counter(PRODUCTION_REFRESH_LEAN_DOMAINS) == ROW and calls[0].arrays.reports is False
    assert isinstance(calls[0].model, (pm.PreviewModelV1, pm.PreviewRefusalV1))  # модель и сверка - как у зафиксированного ответа: кадр ставит образец и кормит модель
    assert world.written == []  # зафиксированной записи нет: ни квитанции, ни дайджеста меша
    assert controller.production_result_count == cached and controller.content_store.result_count == stored
    assert world.mesh_settings.status.startswith("WIDTH_REFRESH_LEAN_V1: exact geometry at width 0.27")
    assert "WIDTH_REFRESH_LEAN_V1" in world.mesh_settings.timing and "live width 0.27" in world.mesh_settings.timing


def test_a_tick_that_lands_after_the_drag_ended_is_dropped_by_name(monkeypatch):
    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = world.controller

    assert live.request_drag_tick(world.context, controller, 0.27)
    controller.width_drag_active = False  # подтверждение или отмена пришли раньше, чем досчиталось
    live.settle_width_live(controller)

    scheduler = controller.width_live
    assert calls == [] and scheduler.counters.applied == 0
    assert live.LEAN_DRAG_ENDED in scheduler.status_text


def test_the_confirm_supersedes_the_pending_tick_and_its_commit_equals_the_direct_run_after_any_ticks(monkeypatch):
    """Подтверждение - зафиксированный ответ (COMMIT) на последней ширине; следов тиков в нём нет, как бы их ни было."""

    calls: list = []
    world = _drag_world(monkeypatch, calls=calls)
    controller = world.controller
    for tick in (0.26, 0.27, 0.28):
        assert live.request_drag_tick(world.context, controller, tick)
        live.settle_width_live(controller)
    assert len(calls) == 3
    assert live.request_drag_tick(world.context, controller, 0.29)  # ещё один тик в пути в момент подтверждения

    preview.finish_drag(controller, restore=False)
    world.drag(0.3)  # подтверждение: запись ширины в ползунок -> заказ COMMIT
    live.settle_width_live(controller)

    assert len(calls) == 3  # ни одного lean-кадра после подтверждения
    assert len(world.written) == 1 and world.written[0][3] == 0.3
    reference = run_production(
        EnvelopeDebugSessionController(), world.bundle, EDGES, 0.3, source_object_key="object", source_data_key="mesh", density="1", domain_pool=None
    )
    assert build_mesh_arrays(world.written[0][0], OFFSET).digest == build_mesh_arrays(reference.results, OFFSET).digest
    assert not any(item.lean for item in world.written[0][0])  # зафиксированный результат полон


def test_an_unavailable_lean_frame_is_a_named_unavailable_not_a_silent_miss(monkeypatch):
    world = _drag_world(monkeypatch, calls=[])
    monkeypatch.setattr(live, "lean_exact_now", lambda controller, run: preview.LEAN_KEY_MISMATCH)

    assert live.request_drag_tick(world.context, world.controller, 0.27)
    live.settle_width_live(world.controller)

    scheduler = world.controller.width_live
    assert scheduler.counters.applied == 0 and preview.LEAN_KEY_MISMATCH in scheduler.status_text
    assert world.written == []
