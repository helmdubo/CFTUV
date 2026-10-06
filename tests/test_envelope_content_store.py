"""Хранилище по содержимому и перенос результата домена на другую ревизию.

Перенос обещает ровно одно: ответ при первой ревизии с переписанными идентичностями хоста равен холодному
прогону на новой — с точностью до меток, которые ядро выводит само от ревизии (порядок граней, номера
`claim:N`/`region:N`/`node:N`, хэши `claim:envelope-instance:*`, число операций точной арифметики). Что именно
сравнивается и что нет, написано в `content_equivalence`. Чего перенос не делает, он называет: результат без
записи идентичностей и результат с неучтённой идентичностью хоста — `ContentRelabelFailed`, а не догадка.
"""

from __future__ import annotations

import dataclasses
import hashlib
import pickle
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv import envelope_content_store as store_module  # noqa: E402
from cftuv.envelope_content_store import (  # noqa: E402
    ContentRelabelFailed,
    ContentStoreV1,
    RelabelV1,
    carried_to_run,
    relabel_result,
)
from content_equivalence import result_projection  # noqa: E402
from content_fixtures import cold_domain, renumbered, with_revision  # noqa: E402
from envelope_fixture_bundles import quad_row_bundle  # noqa: E402

ROW = 5
EDGES = frozenset(range(ROW))


def _moved_scene(lifted=0.0, shift=3):
    """Тот же меш в двух видах: исходный и другой (другой объект, ревизия, номера патчей)."""

    source = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
    target = with_revision(
        renumbered(source, {patch: patch + shift for patch in range(ROW)}), "another-object", "d" * 64
    )
    return source, target, shift


def _pair(patch, *, lifted=0.0, shift=3):
    """`(результат на источнике, результат холодного прогона на цели, куда переносить)`."""

    source, target, shift = _moved_scene(lifted, shift)
    before, _prepared, _revision, _request = cold_domain(source, patch, EDGES)
    cold, _prepared, revision_to, request_to = cold_domain(target, patch + shift, EDGES)
    return before, cold, RelabelV1(revision_to, request_to, patch + shift)


# --------------------------------------------------------------------------
# Перенос равен холодному прогону на цели
# --------------------------------------------------------------------------


@pytest.mark.parametrize("patch", range(ROW))
def test_a_moved_result_is_the_answer_of_a_cold_run_at_the_target(patch):
    before, cold, relabel = _pair(patch)

    moved = carried_to_run(before, relabel)

    assert moved is not before
    assert moved.patch_id == cold.patch_id
    assert moved.domain_id == cold.domain_id
    assert result_projection(moved) == result_projection(cold)
    assert moved.labels.revision == relabel.revision_to
    assert moved.labels.patch_id == relabel.patch_to


@pytest.mark.parametrize("patch", range(ROW))
def test_a_moved_result_recomputes_the_structure_signature_of_its_new_keys_and_keeps_the_interval(patch):
    from cftuv_envelope.materialize.structure import batch_structure

    before, cold, relabel = _pair(patch)

    moved = carried_to_run(before, relabel)

    assert moved.structure_digest and cold.structure_digest
    assert moved.structure_digest == batch_structure(moved.batch).digest == cold.structure_digest
    assert before.alpha_interval is not None and moved.alpha_interval == before.alpha_interval == cold.alpha_interval


def test_two_cold_runs_of_one_content_agree_up_to_kernel_labels():
    """Один и тот же домен при двух ревизиях: числа ответа те же, дайджесты — нет (они покрывают метки ревизии)."""

    source = with_revision(quad_row_bundle(ROW), "row")
    renamed = with_revision(source, "another-object", "c" * 64)
    first, _prepared, _revision, _request = cold_domain(source, 2, EDGES)
    second, _prepared, _revision, _request = cold_domain(renamed, 2, EDGES)

    one, other = result_projection(first), result_projection(second)
    for name in ("outcome", "counters", "normal", "source_normal", "chart_orientation", "decal_topology_law"):
        assert one[name] == other[name], name
    assert first.content_digest != second.content_digest
    assert first.batch.semantic_digest != second.batch.semantic_digest


def test_every_field_of_a_result_is_classified_for_the_move():
    """Поле результата либо переписывается обходом строк, либо пересчитывается явно, либо не несёт идентичностей.

    Новое поле без решения — красный тест: дайджест или номер, не пересчитанные при переносе, остались бы
    устаревшими молча.
    """

    from cftuv.envelope_production_export import ProductionDomainResultV1

    rewritten_strings = {"domain_id", "outcome", "detail", "diagnostics", "vertex_normals", "batch", "counters"}
    recomputed = {"patch_id", "content_digest", "offset_normals_digest", "labels", "structure_digest"}
    # Присланный воркером вид и пикл батча несут чужие идентичности: перенос их НЕ переписывает, а сбрасывает (результат переноса
    # собран заново без них, воркер выводит их снова) - `test_a_moved_result_carries_neither_the_old_view_nor_the_old_batch_bytes`.
    dropped = {"view", "heavy"}
    identity_free = {
        "normal",
        "source_normal",
        "chart_orientation",
        "offset_normal_law",
        "decal_topology_law",
        "seconds",
        "placement",
        "clip_memo",
        # Заверенный интервал ширины - числа и названия, ключей и идентичностей ревизии в нём нет.
        "alpha_interval",
        # Путь шага ширины (`FAST_HIT` / `FALLBACK:<причина>`) - метка запуска, идентичностей в ней нет.
        "step_path",
        # Запись бэкенда ядра (`BackendRecordV1`): числа и имена исходов, идентичностей хоста в них нет.
        "backend_record",
    }

    names = {item.name for item in dataclasses.fields(ProductionDomainResultV1)}

    assert names == rewritten_strings | recomputed | dropped | identity_free


def test_a_moved_unfolded_domain_keeps_its_offset_normals_under_the_new_names():
    before, cold, relabel = _pair(ROW - 1, lifted=1.0)
    assert before.offset_normal_law == "SOURCE_VERTEX_ANGLE_WEIGHTED_NORMAL_V1"

    moved = carried_to_run(before, relabel)

    assert result_projection(moved) == result_projection(cold)
    from cftuv_envelope.materialize.offset_normal import offset_normals_digest

    assert moved.offset_normals_digest == offset_normals_digest(moved.vertex_normals) != before.offset_normals_digest
    assert {name for name, _ in moved.vertex_normals} == {item.vert_key.value for item in moved.batch.vertices}


def test_a_moved_batch_is_self_consistent_for_the_kernel():
    """Дайджесты батча пересчитаны ядром: кодек читает батч и сверяет собственный дайджест."""

    from cftuv_envelope import GeometryBatchCodecV1
    from cftuv_envelope.canonical import canonical_json_bytes, geometry_batch_semantic_digest

    before, _cold, relabel = _pair(2)

    moved = carried_to_run(before, relabel)

    assert GeometryBatchCodecV1.loads(GeometryBatchCodecV1.dumps(moved.batch)) == moved.batch
    assert moved.batch.semantic_digest.value == geometry_batch_semantic_digest(moved.batch).sha256_hex
    assert moved.content_digest == hashlib.sha256(canonical_json_bytes(moved.batch)).hexdigest()
    assert moved.content_digest != before.content_digest
    assert moved.batch.semantic_digest != before.batch.semantic_digest


def test_nothing_of_the_old_identities_survives_in_a_moved_result():
    before, _cold, relabel = _pair(2)

    moved = carried_to_run(before, relabel)
    raw = pickle.dumps(moved)

    assert before.labels.revision.encode() not in raw
    assert before.labels.request_id.encode() not in raw
    old_tokens = {
        item.token for item in before.labels.tokens if item.token not in {n.token for n in moved.labels.tokens}
    }
    assert old_tokens and not any(token.encode() in raw for token in old_tokens)
    assert f"host-patch:{before.labels.revision}:{before.labels.patch_id}".encode() not in raw
    assert before.labels.tokens != moved.labels.tokens


def test_moving_to_the_same_identities_returns_the_same_object():
    before, _cold, _relabel = _pair(1)
    labels = before.labels

    same = RelabelV1(labels.revision, labels.request_id, labels.patch_id)

    assert carried_to_run(before, same) is before
    assert relabel_result(before, labels.revision, labels.request_id, labels.patch_id) is before
    # Другой id запроса при той же ревизии и том же патче переноса не требует: он метка вычисления.
    assert relabel_result(before, labels.revision, "another-request", labels.patch_id) is before


def test_a_refusal_is_moved_like_an_answer():
    """Отказ материализатора — тоже ответ: исход и деталь называют идентичности хоста, и они переписываются."""

    before, cold, relabel = _pair(2)
    revision = before.labels.revision
    refusal = dataclasses.replace(
        before,
        outcome="STATION_CHAIN_UNNAMED",
        batch=None,
        content_digest="",
        counters=(),
        vertex_normals=(),
        offset_normals_digest="",
        diagnostics=(),
        detail=f"vertex host-vertex:{revision}:5 of {before.domain_id}",
    )

    moved = carried_to_run(refusal, relabel)

    assert moved.outcome == "STATION_CHAIN_UNNAMED" and moved.batch is None and moved.content_digest == ""
    assert moved.detail == f"vertex host-vertex:{relabel.revision_to}:5 of {cold.domain_id}"
    assert moved.domain_id == cold.domain_id and moved.patch_id == cold.patch_id


# --------------------------------------------------------------------------
# Чего перенос не делает: называет
# --------------------------------------------------------------------------


def test_a_result_without_a_record_is_not_moved():
    before, _cold, relabel = _pair(1)
    bare = dataclasses.replace(before, labels=None)

    with pytest.raises(ContentRelabelFailed):
        carried_to_run(bare, relabel)
    # Свежий результат на подготовке из хранилища получает запись подготовки (`base`) и переносится.
    moved = carried_to_run(bare, dataclasses.replace(relabel, base=before.labels))
    assert moved.labels.revision == relabel.revision_to


def test_an_identity_of_the_host_outside_the_record_is_named_not_moved():
    before, _cold, relabel = _pair(1)
    foreign = dataclasses.replace(
        before, detail=f"stray host-v0:chain-use:{'0' * 24} in {before.labels.revision}"
    )

    with pytest.raises(ContentRelabelFailed, match="not in the record"):
        carried_to_run(foreign, relabel)


def test_a_chain_source_lineage_outside_the_record_is_named_not_moved():
    before, _cold, relabel = _pair(1)
    foreign = dataclasses.replace(before, detail=f"chain-source:host-patch:x:1:{'1' * 24}")

    with pytest.raises(ContentRelabelFailed):
        carried_to_run(foreign, relabel)


def test_a_str_subclass_in_a_result_is_named_not_skipped():
    """Обход переписывает `type(x) is str`; потомок `str` обход пропустил бы молча, поэтому он называется."""

    class Tag(str):
        pass

    before, _cold, relabel = _pair(1)
    foreign = dataclasses.replace(before, detail=Tag("host-vertex:" + before.labels.revision + ":5"))

    with pytest.raises(ContentRelabelFailed, match="str subclass"):
        carried_to_run(foreign, relabel)


def _walk(value, seen):
    """Каждый узел графа результата: датаклассы по полям, контейнеры по элементам."""

    if id(value) in seen:
        return
    seen.add(id(value))
    yield value
    if dataclasses.is_dataclass(value) and not isinstance(value, type):
        for item in dataclasses.fields(value):
            yield from _walk(getattr(value, item.name), seen)
    elif isinstance(value, dict):
        for key, item in value.items():
            yield from _walk(key, seen)
            yield from _walk(item, seen)
    elif isinstance(value, (tuple, list, set, frozenset)):
        for item in value:
            yield from _walk(item, seen)


def test_no_value_of_a_result_is_a_str_subclass_outside_the_enums():
    """Обход переноса видит `type(x) is str`: потомков `str` в настоящих батчах быть не должно (кроме перечислений)."""

    import enum

    results = []
    for lifted in (0.0, 1.0):  # плоские домены и развёрнутый (нормали вершин)
        bundle = with_revision(quad_row_bundle(ROW, lifted_corner=lifted), "row")
        results += [cold_domain(bundle, patch, EDGES)[0] for patch in range(ROW)]
    assert any(item.vertex_normals for item in results)

    strings = 0
    for result in results:
        for value in _walk(result, set()):
            if isinstance(value, str) and not isinstance(value, enum.Enum):
                assert type(value) is str, (type(value), value)
                strings += 1
    assert strings > 1000


# --------------------------------------------------------------------------
# Хранилище
# --------------------------------------------------------------------------


def test_the_store_is_process_local_and_refuses_every_serialisation():
    import copy

    store = ContentStoreV1()
    store.register_preparation("k", object(), object())

    for dump in (pickle.dumps, copy.copy, copy.deepcopy):
        with pytest.raises(TypeError, match="process-local"):
            dump(store)
    from cftuv.envelope_debug_session import EnvelopeDebugSessionController

    with pytest.raises(TypeError, match="process-local"):
        pickle.dumps(EnvelopeDebugSessionController())


def _labeled():
    return SimpleNamespace(labels=object())


def test_the_store_serves_a_preparation_and_its_results_by_key():
    forgotten = []
    store = ContentStoreV1(on_forget=forgotten.append)
    prepared, labeling = object(), object()
    first, other = _labeled(), _labeled()

    entry = store.register_preparation("k", prepared, labeling)
    store.register_result("k", ("a",), first)
    store.register_result("k", ("a",), other)  # слот занимает первый
    store.register_result("k", ("b",), other)

    assert store.find("k") is entry and entry.prepared is prepared
    assert store.result("k", ("a",)) is first and store.result("k", ("b",)) is other
    assert store.result("k", ("c",)) is None and store.result("missing", ("a",)) is None
    assert store.key_of(prepared) == ("k", entry) and store.key_of(object()) is None
    assert store.holds(prepared) and store.holds(first) and store.holds(other)
    assert not store.holds(object())
    assert len(store) == 1 and store.result_count == 2
    # Результат без записи идентичностей в хранилище не принимается.
    store.register_result("k", ("z",), SimpleNamespace(labels=None))
    store.register_result("absent", ("a",), _labeled())
    assert store.result_count == 2
    assert store.register_preparation("k", object(), object()) is entry

    store.forget("k")

    assert len(store) == 0 and store.result_count == 0 and not store.holds(prepared)
    assert {id(item) for item in forgotten} == {id(prepared), id(first), id(other)}


def test_the_store_forgets_the_oldest_entry_and_the_oldest_result(monkeypatch):
    monkeypatch.setattr(store_module, "CONTENT_STORE_ENTRY_LIMIT", 2)
    monkeypatch.setattr(store_module, "CONTENT_STORE_RESULT_LIMIT", 3)
    forgotten = []
    store = ContentStoreV1(on_forget=forgotten.append)
    objects = {name: object() for name in "abc"}

    for name in "ab":
        store.register_preparation(name, objects[name], object())
        store.register_result(name, ("s",), _labeled())
        store.register_result(name, ("t",), _labeled())
    assert store.result_count == 3  # самый давний результат вытеснен
    store.find("a")  # обращение освежает давность записи
    store.register_preparation("c", objects["c"], object())

    assert len(store) == 2 and store.find("b") is None
    assert objects["b"] in forgotten and store.holds(objects["a"]) and store.holds(objects["c"])
    assert store.result_count <= 3


def test_clearing_the_store_forgets_every_object():
    forgotten = []
    store = ContentStoreV1(on_forget=forgotten.append)
    for name in "xyz":
        store.register_preparation(name, object(), object())
        store.register_result(name, ("s",), _labeled())

    store.clear()

    assert len(store) == 0 and store.result_count == 0 and len(forgotten) == 6
