"""Точная память стадии резки (`materialize.clip_memo`): попадание побитово равно промаху, ключ полон, граница держится.

Утверждений шесть, и каждое стоит на проверке, а не на словах докстроки модуля:

1. ПОПАДАНИЕ == ПРОМАХ. Домен под законами резки по треугольникам и по граням, собранный без памяти, с промахом и с
   попаданием: батч (канонические байты), дайджесты, нормали смещения, диагностики и ВСЕ счётчики (в том числе цена
   `EXACT_WORK_*`: память канонизации к резке та же) равны; при другой alpha, где покрытие насыщено, попадание отдаёт ту же
   геометрию, а UV пересчитан (батч другой, ответ равен собранному без памяти).
2. ПОБОЧНЫЕ ЭФФЕКТЫ СТАДИИ ВОСПРОИЗВЕДЕНЫ. После попадания подъём (`counters`, нормали смещения по позициям), бюджет и память
   канонизации равны тем, что оставил промах.
3. КЛЮЧ ПОЛОН. Каждый именованный аргумент стадии, каждое поле каждого треугольника подъёма, каждый допуск и отпечаток кода
   меняют ключ; стадия не меняет собственный вход; перечень аргументов `clip_geometry` — ровно то, что передаёт `cut_domain`;
   постоянные допуска модулей резки перечислены в `clip_policy`.
4. ГРАНИЦА. Записи и байты ограничены (LRU), смена отпечатка кода сбрасывает память, память выключается, незнакомый тип входа
   считает стадию без неё, запись не портится потребителем.
5. ПОТОЛОК — АВТОРИТЕТ ОТКАЗА. Записанная цена, которая не влезает в остаток потолка, не повторяется: стадия считается заново.
6. ОТПЕЧАТОК КОДА — по закону установщика (проверяется независимой свёрткой каталога).
"""

from __future__ import annotations

import ast
import dataclasses
import hashlib
import inspect
import math
import os
import pickle
from fractions import Fraction
from pathlib import Path

import pytest

import developable_factories as df
from developable_route import materialize_developable

from cftuv_envelope import exact_sqrt_sum as esq
from cftuv_envelope.codec import canonical_json_bytes
from cftuv_envelope.contracts.geometry_batch import DecalTopologyLawV1
from cftuv_envelope.contracts.metric import NearPlanarLiftLawV1
from cftuv_envelope.exact_sqrt_sum import SqrtSumV1, exact_work_budget
from cftuv_envelope.materialize import clip, clip_cells, clip_memo, clip_snap, lift, lift_surface
from cftuv_envelope.materialize.clip import ClippedV1, clip_geometry, clip_policy
from cftuv_envelope.materialize.clip_memo import (
    BYPASS,
    HIT,
    MEMO,
    MISS,
    OFF,
    ClipKeyUnsupported,
    ClipMemoV1,
    clip_key,
    kernel_code_identity,
    memo_disabled,
    run_clip,
)

POLYGONS = DecalTopologyLawV1.PLANAR_POLYGONS_V1
BY_TRIANGLES = NearPlanarLiftLawV1.SOURCE_TRIANGLES_CLIPPED_V1
BY_FACES = NearPlanarLiftLawV1.SOURCE_FACES_CLIPPED_V1
ROUTE = ("r0a", "r0b")
#: `(имя, постройка, alpha промаха, alpha, при которой покрытие уже насыщено и резка та же)`.
FIXTURES = {
    "fold": (df.fold_strip, "3.5", "6"),
    "slant": (df.slant_fold, "3.0", "6"),
    "quarter": (df.quarter_cylinder, "1.6", "3.2"),
}
LAWS = {"triangles": BY_TRIANGLES, "faces": BY_FACES}


def build(name, alpha, lift_law):
    make, _first, _saturated = FIXTURES[name]
    result, _prepared = materialize_developable(
        make(), ROUTE, alpha=alpha, decal_topology_law=POLYGONS, near_planar_lift_law=lift_law
    )
    assert result.is_materialized, result.detail
    return result


def answer(result, *, price=True):
    """Всё, что составляет ответ домена: байты батча, дайджесты, нормали, диагностики, счётчики."""

    counters = tuple(
        item for item in result.counters if price or not item[0].startswith("EXACT_WORK_")
    )
    return (
        result.outcome.value,
        result.detail,
        counters,
        tuple(result.diagnostics),
        result.content_digest,
        result.offset_normals_digest,
        tuple(result.vertex_normals),
        result.offset_normal_law,
        canonical_json_bytes(result.batch),
    )


# --------------------------------------------------------------------------
# 1. Попадание побитово равно промаху
# --------------------------------------------------------------------------


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("name", FIXTURES)
def test_a_hit_is_byte_identical_to_a_miss_and_to_a_build_without_the_memo(name, law):
    lift_law = LAWS[law]
    _make, first, _saturated = FIXTURES[name]
    with memo_disabled():
        off = build(name, first, lift_law)
    miss = build(name, first, lift_law)
    hit = build(name, first, lift_law)
    assert (off.clip_memo, miss.clip_memo, hit.clip_memo) == (OFF, MISS, HIT)
    assert answer(off) == answer(miss) == answer(hit)
    assert (MEMO.hits, MEMO.misses) == (1, 1)


@pytest.mark.parametrize("law", LAWS)
@pytest.mark.parametrize("name", FIXTURES)
def test_a_saturated_width_hits_the_geometry_and_recomputes_the_uv(name, law):
    """Другая alpha при насыщенном покрытии: резка та же (попадание), UV другой, ответ равен собранному без памяти."""

    lift_law = LAWS[law]
    _make, first, saturated = FIXTURES[name]
    with memo_disabled():
        reference = build(name, first, lift_law)
        without = build(name, saturated, lift_law)
    build(name, first, lift_law)
    wide = build(name, saturated, lift_law)
    assert wide.clip_memo == HIT
    assert answer(wide, price=False) == answer(without, price=False)
    # UV и числа ширины другие, поэтому весь батч НЕ взят из памяти: память держит только геометрическую резку.
    assert wide.content_digest != reference.content_digest


def test_a_changed_input_is_a_miss_and_never_a_stale_hit():
    name, lift_law = "fold", BY_FACES
    _make, first, _saturated = FIXTURES[name]
    build(name, first, lift_law)
    narrower = build(name, "1.5", lift_law)
    assert narrower.clip_memo == MISS
    with memo_disabled():
        assert answer(narrower, price=False) == answer(build(name, "1.5", lift_law), price=False)


# --------------------------------------------------------------------------
# 2. Побочные эффекты стадии воспроизведены
# --------------------------------------------------------------------------


def _watched(monkeypatch):
    """Подсматривает `(подъём, бюджет, входы)` каждого вызова `run_clip`; состояние читается ПОСЛЕ домена."""

    seen: list = []
    original = clip.run_clip

    def spy(compute, plane, budget, policy, **inputs):
        seen.append((plane, budget, policy, inputs))
        return original(compute, plane, budget, policy, **inputs)

    monkeypatch.setattr(clip, "run_clip", spy)
    return seen


def _memory_state():
    return (
        dict(esq._FACTORIZATION_MEMO),
        dict(esq._SQUAREFREE_MEMO),
        dict(esq._PRIME_SUPPORT_MEMO),
        set(esq._KNOWN_PRIME_SET),
    )


@pytest.mark.parametrize("name", ("quarter", "fold"))
def test_a_hit_leaves_the_lift_the_budget_and_the_factorization_memory_as_a_miss_does(name, monkeypatch):
    seen = _watched(monkeypatch)
    _make, first, _saturated = FIXTURES[name]
    miss = build(name, first, BY_FACES)
    miss_memory = _memory_state()
    hit = build(name, first, BY_FACES)
    hit_memory = _memory_state()
    (miss_plane, miss_budget, _p, _i), (hit_plane, hit_budget, _q, _j) = seen
    assert (miss.clip_memo, hit.clip_memo) == (MISS, HIT)
    assert miss_plane is not hit_plane
    assert miss_plane._normal_by_position == hit_plane._normal_by_position
    assert miss_plane.counters() == hit_plane.counters()
    assert miss_plane._max_outside == hit_plane._max_outside
    assert miss_budget.counters() == hit_budget.counters()
    assert miss_memory == hit_memory


def test_the_offset_normals_of_the_clip_vertices_survive_a_hit(monkeypatch):
    """Развёртка: нормаль смещения новых вершин пишет подъём по позиции; попавшая вершина обязана получить её же."""

    seen = _watched(monkeypatch)
    _make, first, _saturated = FIXTURES["quarter"]
    miss = build("quarter", first, BY_FACES)
    hit = build("quarter", first, BY_FACES)
    assert hit.clip_memo == HIT
    assert miss.vertex_normals and miss.vertex_normals == hit.vertex_normals
    assert seen[0][0]._normal_by_position and seen[0][0]._normal_by_position == seen[1][0]._normal_by_position


# --------------------------------------------------------------------------
# 3. Ключ полон
# --------------------------------------------------------------------------


def _captured(monkeypatch, name="quarter", lift_law=BY_FACES):
    seen = _watched(monkeypatch)
    _make, first, _saturated = FIXTURES[name]
    build(name, first, lift_law)
    (plane, budget, policy, inputs), *_rest = seen
    return plane, budget, policy, inputs


def _perturbed(value):
    """Значение, которое кодируется иначе, чем `value`: у контейнера меняется первый элемент, у скаляра — он сам."""

    if isinstance(value, bool):
        return not value
    if isinstance(value, (DecalTopologyLawV1, NearPlanarLiftLawV1)):
        members = list(type(value))
        return members[(members.index(value) + 1) % len(members)]
    if isinstance(value, (int, Fraction)):
        return value + 1
    if isinstance(value, float):
        return math.nextafter(value, math.inf)  # на один ульп: ключ точен, а не округлён
    if isinstance(value, str):
        return value + "x"
    if isinstance(value, SqrtSumV1):
        return value + SqrtSumV1.rational(1)
    if isinstance(value, (frozenset, set)):
        return frozenset(value) | {frozenset(("__a__", "__b__"))}
    if isinstance(value, dict):
        if not value:
            return {"__a__": 0}
        first = next(iter(value))
        return {**value, first: _perturbed(value[first])}
    if isinstance(value, (tuple, list)):
        if not value:
            return type(value)((0,))
        return type(value)([_perturbed(value[0]), *value[1:]])
    if dataclasses.is_dataclass(value):
        field = dataclasses.fields(value)[0].name
        return dataclasses.replace(value, **{field: _perturbed(getattr(value, field))})
    raise AssertionError(f"no perturbation for {type(value).__name__}")


def test_every_named_input_of_the_stage_changes_the_key(monkeypatch):
    plane, _budget, policy, inputs = _captured(monkeypatch)
    base = clip_key(inputs, plane.triangles, policy)
    assert set(inputs) == {"points", "cycles", "polygons", "law", "seam", "fans", "flows", "by_faces"}
    for name, value in inputs.items():
        changed = {**inputs, name: _perturbed(value)}
        assert clip_key(changed, plane.triangles, policy) != base, name
    # Имя аргумента — тоже часть записи: тот же набор значений под другими именами — другой ключ.
    renamed = {("alias" if name == "law" else name): value for name, value in inputs.items()}
    assert clip_key(renamed, plane.triangles, policy) != base


def test_a_single_coordinate_a_single_vertex_name_and_a_single_seam_pair_change_the_key(monkeypatch):
    plane, _budget, policy, inputs = _captured(monkeypatch)
    base = clip_key(inputs, plane.triangles, policy)
    points = dict(inputs["points"])
    first = next(iter(points))
    x, y = points[first]
    nudged = {**inputs, "points": {**points, first: (x + SqrtSumV1.rational(Fraction(1, 10**9)), y)}}
    assert clip_key(nudged, plane.triangles, policy) != base
    renamed = {**inputs, "points": {(first + "'" if key == first else key): value for key, value in points.items()}}
    assert clip_key(renamed, plane.triangles, policy) != base
    reordered = {**inputs, "points": dict(reversed(list(points.items())))}
    assert len(points) > 1 and clip_key(reordered, plane.triangles, policy) != base
    polygon = inputs["polygons"][0]
    flipped = [(tuple(reversed(polygon[0])),) + tuple(polygon[1:]), *inputs["polygons"][1:]]
    assert len(polygon[0]) > 2 and clip_key({**inputs, "polygons": flipped}, plane.triangles, policy) != base


def test_every_field_of_every_source_triangle_changes_the_key(monkeypatch):
    plane, _budget, policy, inputs = _captured(monkeypatch)
    triangles = tuple(plane.triangles)
    base = clip_key(inputs, triangles, policy)
    for field in dataclasses.fields(triangles[0]):
        value = getattr(triangles[0], field.name)
        changed = (dataclasses.replace(triangles[0], **{field.name: _perturbed(value)}), *triangles[1:])
        assert clip_key(inputs, changed, policy) != base, field.name
    assert clip_key(inputs, triangles[:-1], policy) != base
    assert clip_key(inputs, tuple(reversed(triangles)), policy) != base


def test_every_tolerance_the_stage_reads_and_the_code_identity_change_the_key(monkeypatch):
    plane, _budget, _policy, inputs = _captured(monkeypatch)
    base = clip_key(inputs, plane.triangles, clip_policy())
    for module, name in (
        (clip_snap, "SOURCE_VERTEX_CORNER_SNAP_CELLS"),
        (clip_snap, "NODE_EDGE_SNAP_CELLS"),
        (clip, "CLIP_DIAGONAL_CHORD_BUDGET"),
        (clip_cells, "CLIP_DIAGONAL_CHORD_BUDGET"),
        (clip_snap, "ENCLOSURE_BITS"),
        (clip_cells, "ENCLOSURE_BITS"),
        (lift_surface, "ENCLOSURE_BITS"),
        (lift, "ENCLOSURE_BITS"),
    ):
        with monkeypatch.context() as patch:
            patch.setattr(module, name, getattr(module, name) + 1)
            assert clip_key(inputs, plane.triangles, clip_policy()) != base, f"{module.__name__}.{name}"
    with monkeypatch.context() as patch:
        patch.setattr(clip_memo, "kernel_code_identity", lambda: "another code")
        assert clip_key(inputs, plane.triangles, clip_policy()) != base


def test_the_stage_does_not_change_its_own_input(monkeypatch):
    """Ключ, снятый до стадии и после неё на тех же объектах, один: стадия вход не портит."""

    seen: list = []
    original = clip.clip_geometry

    def audited(plane, budget, **inputs):
        before = clip_key(inputs, plane.triangles, clip_policy())
        result = original(plane, budget, **inputs)
        seen.append((before, clip_key(inputs, plane.triangles, clip_policy())))
        return result

    monkeypatch.setattr(clip, "clip_geometry", audited)
    for name, lift_law in (("quarter", BY_FACES), ("fold", BY_TRIANGLES)):
        _make, first, _saturated = FIXTURES[name]
        build(name, first, lift_law)
    assert len(seen) == 2 and all(before == after for before, after in seen)


def test_the_key_inputs_are_exactly_what_the_stage_takes_and_cut_domain_passes(monkeypatch):
    plane, _budget, _policy, inputs = _captured(monkeypatch)
    assert set(inspect.signature(clip_geometry).parameters) - {"plane", "budget"} == set(inputs)
    # `cut_domain` зовёт стадию только через память: ни прямого `clip_geometry`, ни `_cut_by_faces`, ни `ClipStageV1`.
    tree = ast.parse(Path(clip.__file__).read_text(encoding="utf-8"))
    (cut_domain,) = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef) and node.name == "cut_domain"]
    called = {
        node.func.id for node in ast.walk(cut_domain) if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
    }
    assert "run_clip" in called and not called & {"clip_geometry", "_cut_by_faces", "ClipStageV1"}


def test_every_tolerance_constant_of_the_clip_modules_is_in_the_policy_or_named_as_no_policy():
    """Новая числовая постоянная в модулях резки без записи в `clip_policy` — красный тест, а не устаревший результат."""

    # Перевод единиц для чисел записи и параметры фильтра знака (`_FILTER_*`: фильтр доказывает знак либо уступает
    # точному пути): ответа не решают.
    not_a_policy = {"NANOMETRES_PER_METRE", "_FILTER_MARGIN", "_FILTER_COORDINATE_LIMIT"}
    flat = []

    def walk(value):
        if isinstance(value, tuple):
            for item in value:
                walk(item)
        else:
            flat.append(value)

    walk(clip_policy())
    missing = []
    for module in (clip, clip_cells, clip_snap):
        tree = ast.parse(Path(module.__file__).read_text(encoding="utf-8"))
        for node in tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1 or not isinstance(node.targets[0], ast.Name):
                continue
            name = node.targets[0].id
            if not name.isupper() or name in not_a_policy:
                continue
            value = getattr(module, name)
            if isinstance(value, (str, tuple)):
                continue  # имена счётчиков и списки имён
            if not any(value == item and type(value) is type(item) for item in flat):
                missing.append(f"{module.__name__}.{name}")
    assert not missing, f"tolerances read by the clip but absent from clip_policy(): {missing}"


# --------------------------------------------------------------------------
# 4. Граница: записи, байты, отпечаток кода, выключение, незнакомый тип
# --------------------------------------------------------------------------


class _Plane:
    """Подъём-заглушка: треугольники и запись нормалей, которых у неё нет (`lifted` пуст)."""

    def __init__(self, triangles=()):
        self.triangles = triangles
        self.replayed: list = []

    def replay_lifted(self, lifted):
        self.replayed.append(dict(lifted))


def _clipped(tag="a"):
    return ClippedV1(
        polygons=[(("a", "b", tag),)],
        cycles=[],
        vertex_lists=[],
        extra_lists=[],
        points={},
        snapped={},
        lifted={},
        counters=(("N", 1),),
        note=tag,
    )


def _budget():
    return exact_work_budget(stage="MATERIALIZE_TEST", domain_id="clip-memo")


def _entry(size=10, spent=(0, 0, 0, 0, 0, 0)):
    return clip_memo._Entry(b"x" * size, spent, 0.0)


def test_the_memo_is_bounded_by_entries_and_evicts_the_least_recently_used():
    memo = ClipMemoV1(entry_limit=2, byte_limit=1 << 20)
    memo.store("a", _entry())
    memo.store("b", _entry())
    assert memo.lookup("a") is not None  # `a` свежее `b`
    memo.store("c", _entry())
    assert (memo.lookup("a") is not None, memo.lookup("b"), memo.lookup("c") is not None) == (True, None, True)
    assert memo.evictions == 1 and len(memo) == 2


def test_the_memo_is_bounded_by_bytes_and_refuses_one_entry_larger_than_the_limit():
    memo = ClipMemoV1(entry_limit=100, byte_limit=25)
    memo.store("a", _entry(10))
    memo.store("b", _entry(10))
    memo.store("c", _entry(10))
    assert memo.lookup("a") is None and memo.bytes == 20 and len(memo) == 2
    memo.store("big", _entry(26))
    assert memo.lookup("big") is None and memo.bytes == 20


def test_a_change_of_the_code_identity_clears_the_memo(monkeypatch):
    memo = ClipMemoV1()
    memo.store("a", _entry())
    assert memo.lookup("a") is not None
    monkeypatch.setattr(clip_memo, "kernel_code_identity", lambda: "the code was edited")
    assert memo.lookup("a") is None
    assert (len(memo), memo.bytes, memo.clears) == (0, 0, 1)


def test_the_code_identity_is_the_installer_law_over_the_kernel_package():
    """Независимая свёртка: путь внутри пакета и содержимое с LF по всем .py (тот же закон, что у установщика)."""

    root = Path(clip_memo.__file__).resolve().parents[1]
    digest = hashlib.sha256()
    for current, directories, files in os.walk(root):
        directories[:] = sorted(item for item in directories if item != "__pycache__")
        for name in sorted(files):
            if name.endswith(".py"):
                path = os.path.join(current, name)
                digest.update(os.path.relpath(path, root).replace("\\", "/").encode())
                digest.update(Path(path).read_bytes().replace(b"\r\n", b"\n"))
    assert kernel_code_identity() == digest.hexdigest()[:16]


def test_a_stage_with_an_input_the_encoder_does_not_know_runs_without_the_memo():
    calls = []

    def compute(plane, budget, *, value):
        calls.append(value)
        return _clipped()

    before = MEMO.bypassed
    result, status = run_clip(compute, _Plane(), _budget(), (), value=object())
    assert status == BYPASS and len(calls) == 1 and MEMO.bypassed == before + 1 and len(MEMO) == 0
    with pytest.raises(ClipKeyUnsupported):
        clip_key({"value": object()}, (), ())


def test_a_disabled_memo_computes_every_time_and_stores_nothing():
    calls = []

    def compute(plane, budget, *, value):
        calls.append(value)
        return _clipped()

    with memo_disabled():
        for _ in range(2):
            assert run_clip(compute, _Plane(), _budget(), (), value=1)[1] == OFF
    assert len(calls) == 2 and len(MEMO) == 0 and MEMO.enabled


def test_a_hit_hands_out_a_fresh_copy_that_the_consumer_cannot_spoil():
    def compute(plane, budget, *, value):
        return _clipped()

    first, status = run_clip(compute, _Plane(), _budget(), (), value=1)
    assert status == MISS
    second, status = run_clip(compute, _Plane(), _budget(), (), value=1)
    assert status == HIT
    second.polygons.append("spoiled")
    third, status = run_clip(compute, _Plane(), _budget(), (), value=1)
    assert status == HIT and third.polygons == first.polygons and third is not second


def test_a_miss_that_fails_stores_nothing_and_the_failure_goes_on():
    def compute(plane, budget, *, value):
        raise RuntimeError("the stage refused")

    with pytest.raises(RuntimeError, match="refused"):
        run_clip(compute, _Plane(), _budget(), (), value=1)
    assert len(MEMO) == 0 and MEMO.misses == 0


# --------------------------------------------------------------------------
# 5. Потолок — авторитет отказа
# --------------------------------------------------------------------------


def test_the_recorded_price_is_added_to_the_budget_only_when_it_fits_under_the_cap():
    budget = exact_work_budget(stage="MATERIALIZE_TEST", domain_id="cap", cap=100)
    assert budget.replay((1, 2, 3, 4, 5, 6)) is True
    assert budget.spent_by_article() == (1, 2, 3, 4, 5, 6) and budget.spent == 21
    assert budget.replay((0, 0, 0, 0, 80, 0)) is False  # 21 + 80 > 100: счёт не тронут
    assert budget.spent_by_article() == (1, 2, 3, 4, 5, 6)
    assert budget.replay((0, 0, 0, 0, 79, 0)) is True and budget.spent == 100
    unlimited = esq.unlimited_reference_budget()
    assert unlimited.replay((7, 0, 0, 0, 0, 0)) is True and unlimited.spent == 7


def test_a_price_that_does_not_fit_is_not_replayed_and_the_stage_is_recomputed():
    calls = []

    def compute(plane, budget, *, value):
        calls.append(1)
        return _clipped()

    run_clip(compute, _Plane(), _budget(), (), value=1)
    key = clip_key({"value": 1}, (), ())
    # Запись, цена которой больше потолка этого бюджета: попадание не вправе её «проглотить».
    MEMO.clear()
    MEMO.store(key, clip_memo._Entry(pickle.dumps((_clipped(), esq.factorization_memory_delta(esq.factorization_memory_marker())), protocol=5), (0, 0, 0, 0, 10**6, 0), 0.0))
    tight = exact_work_budget(stage="MATERIALIZE_TEST", domain_id="cap", cap=1000)
    _result, status = run_clip(compute, _Plane(), tight, (), value=1)
    assert status == MISS and len(calls) == 2 and tight.spent == 0
    roomy = exact_work_budget(stage="MATERIALIZE_TEST", domain_id="cap")
    _result, status = run_clip(compute, _Plane(), roomy, (), value=1)
    assert status == HIT and len(calls) == 2 and roomy.radical_materializations == 10**6


def test_a_hit_puts_back_the_memory_entries_the_stage_added_and_charges_the_same_price():
    """Стадия после попадания платит за память столько же, сколько после промаха: записи на месте, цена та же."""

    def compute(plane, budget, *, value):
        esq.squarefree_split(11 * 13 * 13, budget)
        esq.prime_support(17 * 19, budget)
        return _clipped()

    esq.reset_factorization_memory()
    first = _budget()
    assert run_clip(compute, _Plane(), first, (), value=1)[1] == MISS
    assert first.radical_materializations == 2
    miss_memory = _memory_state()
    esq.reset_factorization_memory()
    second = _budget()
    assert run_clip(compute, _Plane(), second, (), value=1)[1] == HIT
    assert second.counters() == first.counters()
    assert _memory_state() == miss_memory and 11 * 13 * 13 in esq._SQUAREFREE_MEMO
    esq.reset_factorization_memory()


def test_the_factorization_memory_delta_returns_what_the_stage_added_and_nothing_else():
    esq.reset_factorization_memory()
    esq.squarefree_split(2 * 3 * 5 * 7 * 7)
    marker = esq.factorization_memory_marker()
    before = _memory_state()
    budget = _budget()
    esq.squarefree_split(11 * 13 * 13, budget)
    esq.prime_support(11 * 17, budget)
    delta = esq.factorization_memory_delta(marker)
    assert {key for key, _value in delta.squarefree} == {11 * 13 * 13}
    assert {key for key, _value in delta.supports} == {11 * 17}
    assert set(delta.primes) == {11, 13, 17}
    assert all(key not in before[0] for key, _value in delta.factorizations)
    after = _memory_state()
    esq.reset_factorization_memory()
    esq.replay_factorization_memory(delta)
    replayed = _memory_state()
    assert replayed[1] == {11 * 13 * 13: after[1][11 * 13 * 13]}
    assert replayed[2] == {11 * 17: after[2][11 * 17]}
    assert replayed[3] == {11, 13, 17}
    # Повтор идемпотентен и не вытесняет лежащее.
    esq.replay_factorization_memory(delta)
    assert _memory_state() == replayed
    esq.reset_factorization_memory()
