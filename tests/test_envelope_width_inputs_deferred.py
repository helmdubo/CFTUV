"""Входы линий превью ширины строятся ПОСЛЕ кнопки (WIDTH_PREVIEW_INPUTS_DEFERRED_V1, хост без Blender).

Утверждения:

1. РАЗДЕЛЕНИЕ БЕЗ ИЗМЕНЕНИЯ ОТВЕТА: `prepare_preview_inputs(...).complete()` побитово равен ЗАМОРОЖЕННОЙ копии прежнего `build_preview_inputs` (грани, грани рёбер, пути, исходы, число рёбер) на настоящей стене владельца,
   плоской сетке, двух патчах через шов, патче без граней и вырожденной стороне; `has_runs` равен `bool(runs)` на всех них (кнопка отвечает «есть что рисовать» без построения путей);
2. ЧТЕНИЕ ЛЕНИВО И ОДИН РАЗ: вопрос доступности ширины входы не строит; событие ползунка (или ось инструмента) строит их ОДИН раз тем же значением, что построила бы кнопка, и линии превью равны линиям готовых входов;
3. КТО ПОСТРОИЛ НАЗВАН: простой после нажатия - `idle`, событие раньше него - `event`; простой, пришедший вторым, называет `IDLE_ALREADY_BUILT`, вытесненную запись не трогает (`IDLE_SUPERSEDED`);
4. СБОЙ ПОСТРОЕНИЯ НЕ МОЛЧИТ: строка консоли `WIDTH_PREVIEW_INPUTS_BUILD_FAILED`, запись снята, как прежде снимал её сбой `remember_build`.
"""

from __future__ import annotations

import dataclasses
import sys
import time
from pathlib import Path

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_width_live as live  # noqa: E402
from cftuv import envelope_width_preview as preview_module  # noqa: E402
from cftuv.envelope_width_mesh_preview import log_of  # noqa: E402
from cftuv.envelope_width_preview import (  # noqa: E402
    OUTCOME_DEGENERATE,
    OUTCOME_NO_FACE,
    OUTCOME_NO_FAN,
    PendingPreviewInputsV1,
    PreviewInputsV1,
    _chain,
    _make_face,
    _side_of,
    build_preview_inputs,
    compute_width_preview,
    prepare_preview_inputs,
)
from test_envelope_width_live import _World  # noqa: E402
from test_envelope_width_preview import _Mesh, _bottom, _edges_between, _grid, _load_wall  # noqa: E402


def _reference_build_preview_inputs(surface, selected_by_patch) -> PreviewInputsV1:
    """ЗАМОРОЖЕННАЯ копия `build_preview_inputs` до разделения на две половины: оракул побитового равенства."""

    started = time.perf_counter()
    patches = {int(patch_id) for patch_id, _edges in selected_by_patch}
    positions = {item.vertex_id: item.position for item in surface.vertices}
    edge_faces = {item.edge_id: tuple(item.source_face_ids) for item in surface.edges}
    faces = {}
    for face in surface.faces:
        if int(face.patch_id) in patches:
            built = _make_face(face, positions)
            if built is not None:
                faces[int(face.face_id)] = built
    runs, missing, degenerate, edges, notes = [], 0, 0, 0, []
    for patch_id, edge_ids in selected_by_patch:
        sides = []
        for edge_id in sorted(int(item) for item in edge_ids):
            edges += 1
            owners = [
                face_id
                for face_id in edge_faces.get(edge_id, ())
                if face_id in faces and faces[face_id].patch_id == int(patch_id)
            ]
            if not owners:
                missing += 1
            for face_id in owners:
                side = _side_of(faces, face_id, edge_id)
                if side is None:
                    degenerate += 1
                else:
                    sides.append(side)
        runs.extend(_chain(faces, edge_faces, int(patch_id), sides, notes))
    degenerate += sum(1 for run in runs if not run.closed for end in run.ends if end is None)
    return PreviewInputsV1(
        faces,
        edge_faces,
        tuple(runs),
        ((OUTCOME_NO_FACE, missing), (OUTCOME_DEGENERATE, degenerate), (OUTCOME_NO_FAN, sum(notes))),
        edges,
        time.perf_counter() - started,
    )


def _same(left: PreviewInputsV1, right: PreviewInputsV1) -> None:
    assert left.faces == right.faces
    assert left.edge_faces == right.edge_faces
    assert left.runs == right.runs
    assert left.outcomes == right.outcomes
    assert left.edges == right.edges


def _cases():
    """`(имя, поверхность, выбранное)` - то, на чём входы строятся по-разному: стена владельца, сетка с цепью и углом, два патча через шов, патч без граней, цепь, у которой нет грани, вырожденная сторона."""

    cases = []
    surface, selected = _load_wall()
    cases.append(("wall", surface, selected))
    mesh, ids = _grid(12, 8, 0.5)
    cases.append(("bottom", mesh.surface(), [(0, _bottom(mesh, ids, 12))]))
    cells = [((i, 0), (i + 1, 0)) for i in range(4)] + [((4, j), (4, j + 1)) for j in range(3)]
    cases.append(("corner", mesh.surface(), [(0, _edges_between(mesh, ids, cells))]))
    split, split_ids = _grid(6, 4, 1.0, patch=lambda i, j: 0 if i < 3 else 1)
    seam = _edges_between(split, split_ids, [((3, j), (3, j + 1)) for j in range(4)])
    cases.append(("seam", split.surface(), [(0, seam), (1, seam)]))
    cases.append(("no faces", mesh.surface(), [(7, _bottom(mesh, ids, 4))]))
    cases.append(("mixed", mesh.surface(), [(0, _bottom(mesh, ids, 3)), (9, _bottom(mesh, ids, 2))]))
    cases.append(("empty", mesh.surface(), [(0, [])]))
    flat = _Mesh()
    a, b, c, d = (flat.vertex(p) for p in ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0)))
    face = flat.face(0, (a, b, c, d))
    assert face == 0
    twin = _Mesh()
    p0, p1, p2, p3 = (twin.vertex(p) for p in ((0, 0, 0), (0, 0, 0), (1, 1, 0), (0, 1, 0)))  # две совпавшие вершины: сторона нулевой длины
    twin.face(0, (p0, p1, p2, p3))
    cases.append(("zero side", twin.surface(), [(0, [twin.edge(p0, p1), twin.edge(p1, p2)])]))
    return cases


@pytest.mark.parametrize("name, surface, selected", _cases(), ids=lambda value: value if isinstance(value, str) else "")
def test_the_two_halves_equal_the_frozen_single_function_bit_for_bit(name, surface, selected):
    expected = _reference_build_preview_inputs(surface, selected)

    pending = prepare_preview_inputs(surface, selected)
    assert isinstance(pending, PendingPreviewInputsV1) and not pending.built
    assert pending.has_runs == bool(expected.runs), name  # «есть что рисовать» без построения путей
    assert not pending.built
    _same(pending.complete("event"), expected)
    _same(build_preview_inputs(surface, selected), expected)  # прежнее имя - то же значение
    for width in (0.05, 0.31, 1.7):
        assert compute_width_preview(pending.complete("event"), width, lift=0.01).polylines == compute_width_preview(expected, width, lift=0.01).polylines


def test_the_cases_cover_both_answers_of_has_runs():
    answers = {name: prepare_preview_inputs(surface, selected).has_runs for name, surface, selected in _cases()}
    assert answers["wall"] and answers["bottom"] and answers["seam"]
    assert not answers["no faces"] and not answers["empty"]


def test_the_pending_inputs_build_once_and_name_the_first_builder():
    mesh, ids = _grid(12, 8, 0.5)
    pending = prepare_preview_inputs(mesh.surface(), [(0, _bottom(mesh, ids, 12))])

    first = pending.complete("idle")
    assert pending.built and pending.built_by == "idle"
    assert pending.complete("event") is first and pending.built_by == "idle"  # второй не строит и не переименовывает
    assert pending.take_builder() == "idle" and pending.take_builder() == ""  # в журнал - один раз


# --------------------------------------------------------------------------
# Запись кнопки, простой и событие ползунка
# --------------------------------------------------------------------------


def test_the_button_remembers_prepared_inputs_and_the_question_of_availability_does_not_build_the_paths(monkeypatch):
    world = _World(monkeypatch)
    record = world.record
    assert isinstance(record.preview_inputs, PendingPreviewInputsV1) and not record.preview_inputs.built

    problem = live.availability_problem(world.controller, "row", live.DecalProbeV1("row.CFTUV_Decal", "row", f"host-source:{world.digest_now}:row"))

    assert problem == "" and not record.preview_inputs.built  # панель спрашивает на каждой перерисовке: пути не строятся
    assert live.has_preview_runs(record.preview_inputs)


def test_the_first_slider_event_builds_the_paths_once_and_its_lines_equal_the_eager_inputs(monkeypatch):
    world = _World(monkeypatch)
    eager = build_preview_inputs(world.bundle.patch_surface, world.run.selected_by_patch)

    state = live.preview_now(world.controller, 0.3, 0.02)

    assert state is not None
    assert world.record.preview_inputs.built and world.record.preview_inputs.built_by == live.INPUTS_BUILT_EVENT
    assert log_of(world.controller).inputs_built == {live.INPUTS_BUILT_EVENT: 1}
    assert state.polylines == compute_width_preview(eager, 0.3, lift=0.02).polylines
    live.preview_now(world.controller, 0.31, 0.02)
    assert log_of(world.controller).inputs_built == {live.INPUTS_BUILT_EVENT: 1}  # построено один раз


def test_the_idle_preparation_builds_the_paths_before_the_event_and_is_named(monkeypatch):
    world = _World(monkeypatch)

    assert live.run_idle_preparation(world.controller, world.record) == live.IDLE_BUILT
    assert world.record.preview_inputs.built_by == live.INPUTS_BUILT_IDLE
    assert log_of(world.controller).inputs_built == {live.INPUTS_BUILT_IDLE: 1}
    assert live.run_idle_preparation(world.controller, world.record) == live.IDLE_ALREADY_BUILT
    eager = build_preview_inputs(world.bundle.patch_surface, world.run.selected_by_patch)
    assert live.preview_now(world.controller, 0.3, 0.02).polylines == compute_width_preview(eager, 0.3, lift=0.02).polylines
    assert log_of(world.controller).inputs_built == {live.INPUTS_BUILT_IDLE: 1}  # событие построенное не пересчитывает и не пишет себе


def test_an_idle_that_comes_second_or_for_a_replaced_build_changes_nothing(monkeypatch):
    world = _World(monkeypatch)
    live.preview_now(world.controller, 0.3, 0.02)  # событие пришло раньше простоя

    assert live.run_idle_preparation(world.controller, world.record) == live.IDLE_ALREADY_BUILT
    assert log_of(world.controller).inputs_built == {live.INPUTS_BUILT_EVENT: 1}

    newer = _World(monkeypatch)
    old = newer.record
    newer.controller.width_build = dataclasses.replace(old)  # кнопка нажата ещё раз: запись другая
    assert live.run_idle_preparation(newer.controller, old) == live.IDLE_SUPERSEDED
    assert not old.preview_inputs.built
    newer.controller.width_build = None
    assert live.run_idle_preparation(newer.controller, old) == live.IDLE_SUPERSEDED


def test_a_record_with_ready_inputs_is_not_deferred(monkeypatch):
    world = _World(monkeypatch)
    ready = dataclasses.replace(world.record, preview_inputs=build_preview_inputs(world.bundle.patch_surface, world.run.selected_by_patch))
    world.controller.width_build = ready
    assert live.run_idle_preparation(world.controller, ready) == live.IDLE_NOT_DEFERRED
    assert live.resolved_preview_inputs(world.controller, ready) is ready.preview_inputs
    assert log_of(world.controller).inputs_built == {}


def test_the_idle_timer_is_registered_on_the_main_loop_and_builds_when_it_fires(monkeypatch):
    world = _World(monkeypatch)

    live.schedule_idle_preparation(world.controller, world.record)

    timers = world.fake.timers
    assert timers.first_intervals == [live.IDLE_PREPARE_SECONDS] and not world.record.preview_inputs.built
    timers.fire()
    assert world.record.preview_inputs.built_by == live.INPUTS_BUILT_IDLE and not timers.registered  # одноразовый: вернул None


def test_a_build_that_fails_is_named_and_unremembers_the_build_like_a_failed_remember(monkeypatch, capsys):
    world = _World(monkeypatch)

    def broken(*_args, **_kwargs):
        raise ValueError("no fan")

    monkeypatch.setattr(preview_module, "_chain", broken)

    assert live.resolved_preview_inputs(world.controller, world.record) is None
    assert world.controller.width_build is None  # ширина не живая до следующей кнопки
    assert f"{live.INPUTS_BUILD_FAILED}: the build was not remembered: ValueError: no fan" in capsys.readouterr().out
    assert log_of(world.controller).inputs_built == {}


def test_an_unregistered_timer_failure_in_the_idle_is_named_and_does_not_reach_the_main_loop(monkeypatch, capsys):
    world = _World(monkeypatch)

    def broken(*_args, **_kwargs):
        raise RuntimeError("boom")

    monkeypatch.setattr(live, "run_idle_preparation", broken)
    live.schedule_idle_preparation(world.controller, world.record)
    world.fake.timers.fire()  # исключение не уходит в цикл Blender

    assert f"{live.INPUTS_BUILD_FAILED}: the idle preparation failed: RuntimeError: boom" in capsys.readouterr().out
