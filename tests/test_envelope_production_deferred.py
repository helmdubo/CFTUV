"""Отложенный результат воркера: вид домена, батч и дайджесты читаются как у eager-результата, а считаются при первом чтении.

Живая ширина читала из ответа воркера почти один вид домена (плоские массивы для писателя меша), а разворот графа записей батча
(0.4 с на `building` на каждом шаге, под GIL) и дайджесты (~18 % CPU домена) не использовала. Воркер присылает вид и пикл батча
(`deferred_result`), родитель разворачивает и считает только тому, кто спросил. Здесь держится одно: ЧТО ПРОЧИТАНО, ТО РАВНО
eager-результату (поля, батч, оба дайджеста, массивы меша), и ничто не пропало молча.
"""

from __future__ import annotations

import ast
import dataclasses
import functools
import hashlib
import pickle
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
KERNEL_SRC = ROOT / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_production_mesh as writer  # noqa: E402
from cftuv.envelope_content_store import RelabelV1, carried_to_run  # noqa: E402
from cftuv.envelope_production_export import (  # noqa: E402
    ProductionDomainResultV1,
    deferred_result,
    export_production_json,
    produce_domain,
)
from cftuv.envelope_production_view import DomainViewV1, build_domain_view  # noqa: E402
from content_equivalence import result_projection  # noqa: E402
from content_fixtures import cold_domain, renumbered, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))
ALPHA = 0.25
#: Плоские домены и домен-развёртка (последний патч у поднятого угла: нормаль смещения своя на вершину).
CASES = [(0, 0.0), (2, 0.0), (ROW - 1, 1.0), (2, 1.0)]


@functools.lru_cache(maxsize=None)
def _domain(patch: int, lifted: float):
    """`(отложенный результат воркера, eager-результат на той же подготовке, подготовка)`."""

    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    deferred, prepared, _revision, _request = cold_domain(source, patch, EDGES, alpha=ALPHA)
    eager = produce_domain(patch, deferred.domain_id, prepared, str(float(ALPHA)))
    return deferred, eager, prepared


# --------------------------------------------------------------------------
# Форма и равенство
# --------------------------------------------------------------------------


@pytest.mark.parametrize("patch, lifted", CASES)
def test_a_worker_result_is_deferred_until_something_reads_it(patch, lifted):
    deferred = pickle.loads(pickle.dumps(_domain(patch, lifted)[0]))  # свежая копия: чтений в ней ещё не было

    assert isinstance(deferred.view, DomainViewV1) and isinstance(deferred.heavy, bytes)
    assert {"batch", "vertex_normals", "content_digest"}.isdisjoint(deferred.__dict__)
    assert deferred.is_materialized  # вопрос про исход ничего не разворачивает
    assert deferred.outcome == "MATERIALIZED" and deferred.counters and deferred.normal is not None
    assert {"batch", "vertex_normals", "content_digest"}.isdisjoint(deferred.__dict__)

    assert deferred.batch is not None  # чтение достаёт
    assert "batch" in deferred.__dict__ and "vertex_normals" in deferred.__dict__
    assert "content_digest" not in deferred.__dict__  # дайджест считается отдельно и лишь по требованию
    assert deferred.content_digest
    assert "content_digest" in deferred.__dict__


@pytest.mark.parametrize("patch, lifted", CASES)
def test_everything_read_from_a_deferred_result_equals_the_eager_answer(patch, lifted):
    from cftuv_envelope.canonical import geometry_batch_semantic_digest
    from cftuv_envelope.codec import canonical_json_bytes

    deferred, eager, _prepared = _domain(patch, lifted)
    deferred = pickle.loads(pickle.dumps(deferred))

    assert deferred == eager  # поля сравнения: батч, дайджесты, нормали, счётчики, диагностики
    assert deferred.batch == eager.batch
    assert deferred.batch.semantic_digest == eager.batch.semantic_digest
    assert deferred.batch.semantic_digest.value == geometry_batch_semantic_digest(eager.batch).sha256_hex
    assert deferred.content_digest == eager.content_digest == hashlib.sha256(canonical_json_bytes(eager.batch)).hexdigest()
    assert canonical_json_bytes(deferred.batch) == canonical_json_bytes(eager.batch)
    assert deferred.vertex_normals == eager.vertex_normals
    assert deferred.offset_normals_digest == eager.offset_normals_digest
    assert result_projection(deferred) == result_projection(eager)
    assert bool(eager.vertex_normals) == (patch == ROW - 1 and lifted > 0)  # развёртку проверяем, и плоский домен тоже


@pytest.mark.parametrize("patch, lifted", CASES)
def test_the_view_a_worker_sends_is_the_view_built_from_the_batch(patch, lifted):
    deferred, eager, _prepared = _domain(patch, lifted)

    assert deferred.view == build_domain_view(eager)
    assert deferred.view.failure is None and deferred.view.faces and deferred.view.vertices.positions


@pytest.mark.parametrize("lifted", (0.0, 1.0))
def test_the_mesh_writer_reads_the_view_and_never_unpacks_the_batch(lifted):
    pairs = [_domain(patch, lifted) for patch in range(ROW)]
    deferred = [pickle.loads(pickle.dumps(item[0])) for item in pairs]
    eager = [item[1] for item in pairs]

    from_view = writer.build_mesh_arrays(deferred, 0.02)
    from_batch = writer.build_mesh_arrays(eager, 0.02)

    assert from_view == from_batch  # позиции, грани, UV, владельцы, швы, дайджест массивов, счётчики сварки и шва
    assert from_view.digest == from_batch.digest and from_view.seam_counters == from_batch.seam_counters
    assert from_view.domains == tuple(range(ROW)) and from_view.faces
    assert all({"batch", "vertex_normals", "content_digest"}.isdisjoint(item.__dict__) for item in deferred)


def test_a_result_whose_view_was_dropped_is_written_from_its_batch():
    deferred, eager, _prepared = _domain(2, 0.0)
    plain = deferred.materialized()

    assert plain.view is None and plain.heavy is None
    assert writer.build_mesh_arrays([plain], 0.02) == writer.build_mesh_arrays([eager], 0.02)


# --------------------------------------------------------------------------
# Копии, пересылка, замена полей
# --------------------------------------------------------------------------


def test_a_deferred_result_stays_deferred_through_pickle_and_equals_itself_after():
    deferred, eager, _prepared = _domain(2, 1.0)
    sent = pickle.loads(pickle.dumps(deferred))
    resent = pickle.loads(pickle.dumps(sent))

    for copy in (sent, resent):
        assert {"batch", "vertex_normals"}.isdisjoint(copy.__dict__) and copy.heavy == deferred.heavy
        assert copy.view == deferred.view
    assert resent == eager
    again = pickle.loads(pickle.dumps(resent))  # прочитанный результат пикл не раздувает: развёрнутое в пикл не идёт
    assert {"batch", "vertex_normals"}.isdisjoint(again.__dict__) and again == eager


def test_with_changes_keeps_the_deferral_and_replace_unpacks_it_honestly():
    deferred = pickle.loads(pickle.dumps(_domain(2, 0.0)[0]))
    eager = _domain(2, 0.0)[1]

    kept = deferred.with_changes(placement="cache", seconds=0.0, clip_memo="")
    assert {"batch", "vertex_normals", "content_digest"}.isdisjoint(kept.__dict__)
    assert kept.heavy is deferred.heavy and kept.view is deferred.view and kept.placement == "cache"

    moved = deferred.with_changes(patch_id=99)
    assert moved.patch_id == 99 and moved.view is None and moved.heavy is deferred.heavy  # чужой вид снят, пикл остался
    assert writer.build_mesh_arrays([moved], 0.02).domains == (99,)

    emptied = deferred.with_changes(batch=None)
    assert emptied.batch is None and emptied.heavy is None and emptied.view is None and not emptied.is_materialized

    replaced = dataclasses.replace(deferred, seconds=2.0)
    assert replaced.view is None and replaced.heavy is None and replaced == eager and replaced.is_materialized

    with pytest.raises(TypeError):
        deferred.with_changes(no_such_field=1)


def test_a_moved_result_carries_neither_the_old_view_nor_the_old_batch_bytes():
    source = with_revision(quad_row_bundle(ROW), "row")
    target = with_revision(
        renumbered(source, {patch: patch + 3 for patch in range(ROW)}), "another-object", "d" * 64
    )
    before, _prepared, _revision, _request = cold_domain(source, 2, EDGES)
    cold, _prepared, revision_to, request_to = cold_domain(target, 5, EDGES)
    assert before.heavy is not None and cold.heavy is not None

    moved = carried_to_run(before, RelabelV1(revision_to, request_to, 5))

    assert moved.heavy is None and moved.view is None  # воркер выводит их снова из ПЕРЕНЕСЁННОГО батча
    assert moved.patch_id == 5 and result_projection(moved) == result_projection(cold)
    assert moved.batch.semantic_digest.value != before.batch.semantic_digest.value
    again = deferred_result(moved)
    assert again.view == build_domain_view(moved) and again.heavy is not None and again == moved


def test_the_worker_entry_points_answer_deferred_and_the_parent_computes_eager():
    from cftuv.envelope_domain_pool import DomainTaskV1, solve_task
    from cftuv.envelope_production_export import ProductionInputV1

    deferred, eager, prepared = _domain(2, 0.0)
    task = DomainTaskV1(
        0, 2, deferred.domain_id, None, None, str(float(ALPHA)), EDGES,
        production=ProductionInputV1(pickle.dumps(prepared, protocol=5)),
    )

    answered = solve_task(task).production

    assert deferred.heavy is not None and answered.heavy is not None and answered.view is not None  # воркер: отложенный результат
    assert eager.heavy is None and eager.view is None and "batch" in eager.__dict__  # родитель: прежний eager-результат
    assert answered == eager and answered.content_digest == eager.content_digest


def test_host_code_never_calls_dataclasses_replace_on_a_production_result():
    """`replace` разворачивает отложенный результат целиком (и сбрасывает вид): копии результата идут через `with_changes`.

    Хранилище по содержимому (`envelope_content_store`) вне правила намеренно: его `replace` стоят на результате, который `relabel_result`
    уже развернул (`materialized`), либо на запасном пути для объектов без `with_changes`.
    """

    names = {"result", "produced", "based", "moved", "cached", "payload"}
    offenders = []
    for name in ("envelope_production_export.py", "envelope_production_mesh.py", "envelope_width_live.py"):
        path = ROOT / "cftuv" / name
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=name)):
            if isinstance(node, ast.Call) and getattr(node.func, "id", "") == "replace" and node.args:
                first = node.args[0]
                label = first.id if isinstance(first, ast.Name) else getattr(first, "attr", "")
                if label in names:
                    offenders.append(f"{name}:{node.lineno} replace({label}, ...)")
    assert offenders == [], offenders


def test_a_refusal_is_returned_by_deferral_as_it_is():
    refusal = ProductionDomainResultV1(1, "d", "SOME_REFUSAL", None, "why")

    assert deferred_result(refusal) is refusal and not refusal.is_materialized
    assert refusal.with_changes(placement="parent").batch is None


# --------------------------------------------------------------------------
# Дайджесты: каждый читатель получает прежнее значение
# --------------------------------------------------------------------------


def test_the_evidence_json_of_a_deferred_run_is_the_json_of_an_eager_run(tmp_path):
    pairs = [_domain(patch, 1.0) for patch in range(ROW)]
    deferred = [pickle.loads(pickle.dumps(item[0])) for item in pairs]
    eager = [item[1] for item in pairs]

    first = export_production_json(deferred, tmp_path / "deferred", label="run")
    second = export_production_json(eager, tmp_path / "eager", label="run")

    assert first.read_bytes() == second.read_bytes()  # content_digest, semantic_digest, дайджест нормалей, исходы
    for left in sorted((tmp_path / "deferred").glob("*.geometry_batch.json")):
        assert left.read_bytes() == (tmp_path / "eager" / left.name).read_bytes()  # батч кодеком ядра


#: Места хоста и инструментов, которые читают дайджесты результата домена. Читатель, которого здесь нет, не проверен на отложенном
#: результате: новое чтение обязано быть названо (и закрыто тестом выше), а не появиться молча.
DIGEST_READERS = {
    # (файл, имя поля): что читает
    ("cftuv/envelope_content_store.py", "content_digest"): "relabel_result: дайджест переписанного результата (результат развёрнут `materialized`)",
    ("cftuv/envelope_production_export.py", "content_digest"): "produce_domain (дайджест ядра) и export_production_json (строка свидетельства)",
    ("cftuv/envelope_production_export.py", "semantic_digest"): "export_production_json (строка свидетельства), запечатывание батча при первом чтении",
}
#: Инструменты, читающие `semantic_digest` ЭТАЛОННОГО покрытия ядра (`RawCoverage`), а не результата продуктового пути.
TOOL_DIGEST_READERS = {
    ("tools/run_envelope_mr1_building_gate.py", "semantic_digest"): "дайджест эталонного покрытия",
    ("tools/export_building_002_point_contact_fixture.py", "semantic_digest"): "дайджест эталонного покрытия",
}
_DIGEST_FIELDS = ("content_digest", "semantic_digest")


def _digest_reads():
    found = set()
    for folder in ("cftuv", "tools"):
        for path in sorted((ROOT / folder).rglob("*.py")):
            relative = path.relative_to(ROOT).as_posix()
            for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=relative)):
                if isinstance(node, ast.Attribute) and node.attr in _DIGEST_FIELDS and isinstance(node.ctx, ast.Load):
                    found.add((relative, node.attr))
    return found


def test_every_host_reader_of_a_result_digest_is_named():
    reads = {item for item in _digest_reads() if item[0].startswith("cftuv/")}

    assert reads == set(DIGEST_READERS), sorted(reads ^ set(DIGEST_READERS))


def test_no_tool_reads_a_result_digest_as_an_attribute_without_being_named_here():
    """Инструменты читают дайджесты результата по имени поля (`getattr`, список `ANSWER_FIELDS`): ленивое свойство их отдаёт."""

    assert {item for item in _digest_reads() if item[0].startswith("tools/")} == set(TOOL_DIGEST_READERS)
    source = (ROOT / "tools" / "blender_clip_memo_ab.py").read_text(encoding="utf-8")
    assert '"content_digest"' in source and "getattr(result, name)" in source


def test_a_deferred_worker_result_is_the_eager_answer_in_the_kernel_counters_too():
    deferred, eager, _prepared = _domain(2, 1.0)

    assert deferred.counters == eager.counters  # EXACT_WORK_* и числа материализатора не зависят от отложенных дайджестов
    assert deferred.diagnostics == eager.diagnostics and deferred.decal_topology_law == eager.decal_topology_law
