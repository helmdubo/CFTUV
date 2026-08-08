"""Ворота представления тождества: новое представление ПОБИТОВО равно старому.

ЧТО СМЕНИЛОСЬ И ПОЧЕМУ ЭТО ВООБЩЕ НАДО ПРОВЕРЯТЬ. Ключи вхождений и портов
несут точные величины, и КАЖДЫЙ словарный вопрос про такой ключ пересчитывал
его хэш, а `Fraction.__hash__` считает модульный обратный. Новое представление
(`ExactIdentityKeyV1`) запоминает хэш у самого ключа, а память состояния
пролёта (`SpanStateMemoV1`) перестаёт пересобирать один и тот же пролёт по
одиннадцать раз подряд. Ни то, ни другое НЕ ВПРАВЕ сдвинуть ни один ответ, и
это утверждение проверяется прогоном, а не рассуждением.

ДОВОД ПО ПОСТРОЕНИЮ — и он здесь сильнее обычного. `ExactIdentityKeyV1` есть
подтип `tuple`: его `repr`, `==`, `<`, индексация и ЗНАЧЕНИЕ хэша совпадают с
обычным кортежем до бита, поэтому старое и новое представления РАВНЫ ДРУГ
ДРУГУ, а не «эквивалентны по смыслу». Память пролёта живёт ровно столько,
сколько живёт вид, а вид строится из наложения, которое за его жизнь не
меняется: все шесть мест, где вид создаётся, читают наложение, а четыре
мутирующих места работают на КЛОНЕ (`clone_overlay`/`_clone` пересоздают и
словари, и сами вершины).

ДОВОД ПРОГОНОМ — главный, потому что утверждения в этом репозитории имеют
привычку расходиться с реальностью (`AGENTS.md`). Весь корпус 63, весь корпус
весов и стен (с отказами), длинные цепочки смертей и все четыре полевые
фикстуры sem_clb гоняются ДВАЖДЫ — старое представление и новое — и сверяются
по тем же осям, что и теневая сверка гидратации: исход, semantic_digest, узлы,
уровни, счётчики, статус доказательства и ПОЛНЫЙ мультисет его долгов, грани с
точными площадями, покрытие на четырёх alpha.

ОТКАЗНЫЙ ПУТЬ ЦЕННЕЕ УСПЕШНОГО. Там, где фронт не закрылся, равенство обязано
держаться тем более: иначе отказ был бы артефактом представления, а не
свойством входа. Корпус здесь НЕ фильтруется по исходу.
"""

from __future__ import annotations

import fractions
from pathlib import Path

import pytest

import cftuv_envelope as kernel
from cftuv_envelope.wavefront import prepare_conveyor
from cftuv_envelope.wavefront.exact_identity import (
    ExactIdentityKeyV1,
    ExactIdentityModeV1,
    identity_key,
    identity_mode,
    legacy_identity,
)
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.skeleton import SplitSearch, build_skeleton

from shadow_axes import coverage_axes, partition_axes, skeleton_axes
from wavefront_cases import named_corpus, partial_source_corpus, star
from weighted_wall_differential_cases import weighted_wall_differential_corpus


_FIELD_CASES = (
    "building_all_seams_patch_001_lost_resolved_v1",
    "building_all_seams_patch_006_lost_resolved_v1",
    "building_all_seams_patch_011_lost_resolved_v1",
    "building_all_seams_patch_105_lost_resolved_v1",
)


def _observable(polygon, *, legacy: bool, search: SplitSearch,
                downstream: bool) -> dict:
    """Всё, что о фигуре можно наблюдать снаружи, одним словарём."""

    if legacy:
        with legacy_identity():
            skeleton = build_skeleton(polygon, split_search=search)
    else:
        skeleton = build_skeleton(polygon, split_search=search)
    axes = {"skeleton": skeleton_axes(skeleton)}
    if downstream:
        partition = build_faces(polygon, skeleton)
        axes["faces"] = partition_axes(partition)
        axes["coverage"] = coverage_axes(partition)
    return axes


def _assert_shadow(label, polygon, *, downstream: bool = True, searches=None):
    """Старое и новое представления совпадают на ОБОИХ режимах поиска разрезов.

    `downstream` (грани, площади, покрытие на четырёх alpha) считается на
    синтетических корпусах и выключен на полевых по той же причине, что и в
    теневой сверке гидратации: `coverage_at` зовётся здесь БЕЗ БЮДЖЕТА, знак
    умеет уходить в факторизацию, и одна полевая фикстура не возвращалась за
    25 минут. Полевые грани и покрытие сверяются там, где у них есть alpha
    домена и бюджет: `artifacts/exact_identity_intern/ab_identity.py` и
    полевая релиз-матрица.
    """

    outcomes = []
    for search in (searches or tuple(SplitSearch)):
        new = _observable(
            polygon, legacy=False, search=search, downstream=downstream
        )
        old = _observable(
            polygon, legacy=True, search=search, downstream=downstream
        )
        assert new == old, (label, search.value)
        outcomes.append(new["skeleton"]["outcome"])
    return outcomes[0]


# --------------------------------------------------------------------------
# Корпус 63: именованные фигуры плюс частичный источник.
# --------------------------------------------------------------------------

_NAMED = tuple(named_corpus()) + tuple(partial_source_corpus())


def test_named_corpus_is_sixty_three_cases():
    """Размер корпуса — часть ворот: молчаливо усохший корпус не ворота."""

    assert len(_NAMED) == 63


@pytest.mark.parametrize("name,polygon", _NAMED, ids=[n for n, _ in _NAMED])
def test_new_identity_matches_legacy_on_the_named_corpus(name, polygon):
    _assert_shadow(name, polygon)


# --------------------------------------------------------------------------
# Корпус весов и стен: отказные домены и длинные цепочки смертей.
# --------------------------------------------------------------------------

_WEIGHTED = tuple(
    (case.name, case.polygon) for case in weighted_wall_differential_corpus()
)


def test_weighted_wall_corpus_is_twenty_three_cases():
    assert len(_WEIGHTED) == 23


@pytest.mark.parametrize(
    "name,polygon", _WEIGHTED, ids=[n for n, _ in _WEIGHTED]
)
def test_new_identity_matches_legacy_on_the_weighted_wall_corpus(
    name, polygon
):
    _assert_shadow(name, polygon)


def test_the_weighted_wall_corpus_actually_contains_refusals():
    """Ворота отказного пути обязаны ИМЕТЬ отказ, иначе они его не проверяют."""

    outcomes = {
        name: build_skeleton(polygon).outcome.value
        for name, polygon in _WEIGHTED
    }
    assert any(value != "EXACT" for value in outcomes.values()), outcomes


# --------------------------------------------------------------------------
# Длинная цепочка смертей.
# --------------------------------------------------------------------------

_LONG_CHAIN = tuple((f"star_15_seed_{seed}", star(15, seed)) for seed in range(4))


@pytest.mark.parametrize(
    "name,polygon",
    [row for row in _LONG_CHAIN if row[1] is not None],
    ids=[name for name, polygon in _LONG_CHAIN if polygon is not None],
)
def test_new_identity_matches_legacy_on_long_death_chains(name, polygon):
    _assert_shadow(name, polygon)


# --------------------------------------------------------------------------
# Поле: все четыре фикстуры sem_clb.
# --------------------------------------------------------------------------


def _field_inputs(case_name):
    root = (
        Path(__file__).parents[1]
        / "fixtures"
        / "sem_clb_02_lost_domains_v1"
        / "cases"
        / case_name
    )
    snapshot = kernel.AnalysisSnapshotCodecV1.loads(
        (root / "analysis_snapshot.json").read_bytes()
    )
    request = kernel.DecalRequestCodecV1.loads(
        (root / "decal_request.json").read_bytes()
    )
    (domain,) = snapshot.patch_domains
    return snapshot, request, domain.patch_domain_id


@pytest.mark.parametrize("case_name", _FIELD_CASES)
def test_new_identity_matches_legacy_on_the_field_fixtures(case_name):
    snapshot, request, domain_id = _field_inputs(case_name)
    prepared = prepare_conveyor(snapshot, request, patch_domain_id=domain_id)
    (region,) = prepared.regions
    polygon = region.bridge.polygon
    assert polygon is not None, (case_name, prepared.detail)
    _assert_shadow(
        case_name,
        polygon,
        downstream=False,
        searches=(SplitSearch.MOTORCYCLE,),
    )


# --------------------------------------------------------------------------
# Само представление: утверждения, у которых МОЖНО построить нарушение.
# --------------------------------------------------------------------------


def test_the_cached_key_is_indistinguishable_from_the_plain_tuple():
    """Подстановочность — не стиль, а условие корректности сверки выше.

    Если бы `repr` расходился, поехал бы дайджест (он сортирует точки по
    `repr`); если бы расходилось значение хэша, ключ и обычный кортеж с тем же
    содержимым разъехались бы по словарям, ОСТАВАЯСЬ равными по `==`. Это
    худший вид молчаливой ошибки, и здесь он ловится, а не описывается.
    """

    payload = (
        ((1, fractions.Fraction(3, 4)), (5, fractions.Fraction(-7, 9))),
        ((1, fractions.Fraction(1, 2)),),
    )
    key = ExactIdentityKeyV1(payload)
    assert repr(key) == repr(payload)
    assert hash(key) == hash(payload)
    assert key == payload and payload == key
    assert not (key < payload) and not (payload < key)
    assert {payload: "v"}[key] == "v"
    assert {key: "v"}[payload] == "v"
    assert key[0] is payload[0]
    assert tuple(key) == payload


def test_legacy_mode_really_switches_the_representation_off():
    """Старое представление — не декорация: ключ там ГОЛЫЙ кортеж.

    Без этой проверки теневая сверка могла бы сравнивать новый путь сам с
    собой и зеленеть по построению.
    """

    payload = ((1, fractions.Fraction(1, 3)),)
    assert identity_mode() is ExactIdentityModeV1.CACHED
    assert type(identity_key(payload)) is ExactIdentityKeyV1
    with legacy_identity():
        assert identity_mode() is ExactIdentityModeV1.LEGACY_TUPLE
        assert type(identity_key(payload)) is tuple
    assert identity_mode() is ExactIdentityModeV1.CACHED


def test_legacy_mode_really_switches_the_span_state_memo_off():
    """Второй механизм карточки выключается тем же переключателем.

    Проверяется НАБЛЮДАЕМЫМ следствием, а не наличием поля: в рабочем режиме
    два вопроса про один и тот же пролёт возвращают ОДИН И ТОТ ЖЕ объект, в
    старом — два разных. Сверка, в которой одна из половин молча осталась
    включённой, зеленела бы сама по себе.
    """

    from cftuv_envelope.wavefront.symbolic_overlay import exact_overlay_view

    builder, overlay, leaf = _tiny_overlay()
    view = exact_overlay_view(builder, overlay)
    assert view.span_state(leaf) is view.span_state(leaf)
    with legacy_identity():
        legacy_view = exact_overlay_view(builder, overlay)
        first = legacy_view.span_state(leaf)
        second = legacy_view.span_state(leaf)
    assert first is not second
    assert first == second


def test_the_span_state_memo_refuses_a_foreign_generation():
    """Чужое поколение память не обслуживает и молча ответ не подменяет.

    Иначе утверждение «поколение вынесено из ключа во владельца» было бы
    прозой: подсунуть чужое наложение и получить старую запись никто бы не
    заметил.
    """

    from cftuv_envelope.wavefront.exact_candidate_view import SpanStateMemoV1

    owner, instant = object(), object()
    memo = SpanStateMemoV1(owner, instant)
    assert memo.admits(owner, instant)
    assert not memo.admits(object(), instant)
    assert not memo.admits(owner, object())
    memo.entries["k"] = "v"
    memo.clear()
    assert not memo.entries


def _tiny_overlay():
    """Наименьшее наложение с четырьмя пролётами и живыми портами.

    Строится вручную по образцу `_symbolic_edge_fixture` из сюиты транзакции:
    строитель здесь duck-typed, потому что `span_state` читает у него ровно
    два поля — несущую прямую и вхождение рантаймового ребра.
    """

    from types import SimpleNamespace

    from cftuv_envelope.wavefront.event_time import (
        EventPointV1,
        SupportLineV1,
        ZERO_TIME,
    )
    from cftuv_envelope.wavefront.sqrt_sum import SqrtSumV1
    from cftuv_envelope.wavefront.superlevel_closure import (
        SegmentRefV1,
        SpanFamilyRefV1,
    )
    from cftuv_envelope.wavefront.symbolic_overlay import (
        JunctionRefV1,
        SymbolicOverlayV1,
        SymbolicSpanBindingV1,
        SymbolicVertexV1,
    )

    point_key = identity_key(
        (((1, fractions.Fraction(3)),), ((1, fractions.Fraction(5)),))
    )
    point = EventPointV1(SqrtSumV1(point_key[0]), SqrtSumV1(point_key[1]))
    names = ("A", "B", "C", "D")
    leaves = []
    for index in range(len(names)):
        occurrence = identity_key(((index, index + 1), point_key, point_key))
        family = SpanFamilyRefV1(occurrence, ((index, index + 1),))
        leaves.append(SegmentRefV1(family, None, None, occurrence))
    refs = {name: JunctionRefV1("TEST", (name,)) for name in names}
    vertices = {}
    starts, ends = {}, {}
    for index, name in enumerate(names):
        ref = refs[name]
        prev_leaf = leaves[index - 1]
        next_leaf = leaves[index]
        vertices[ref] = SymbolicVertexV1(
            ref,
            refs[names[index - 1]],
            refs[names[(index + 1) % len(names)]],
            prev_leaf,
            next_leaf,
            ZERO_TIME,
            point,
            None,
            frozenset({("TEST", name)}),
        )
        starts.setdefault(next_leaf, ref)
        ends.setdefault(prev_leaf, ref)
    spans = {
        leaf: SymbolicSpanBindingV1(
            leaf, index, starts.get(leaf), ends.get(leaf)
        )
        for index, leaf in enumerate(leaves)
    }
    overlay = SymbolicOverlayV1(vertices, spans, set(leaves), ZERO_TIME)
    builder = SimpleNamespace(
        _prime_universe=(),
        edges=[
            SimpleNamespace(
                line=SupportLineV1(index + 1, 1, 0, 1),
                span=(0, 0, 1, 1),
            )
            for index in range(len(leaves))
        ],
    )
    return builder, overlay, leaves[0]


# --------------------------------------------------------------------------
# Цена: ВОРОТА НА СЧЁТЧИКАХ, А НЕ НА СЕКУНДАХ.
#
# Секунда — свойство машины, а не входа, и гейт по ней зелёный или красный от
# соседнего процесса (урок ошибки №19 в DECISIONS). Предмет карточки —
# МОДУЛЬНЫЕ ОБРАТНЫЕ на путях тождества, и они считаются числом.
# --------------------------------------------------------------------------


def _count_fraction_hashes(monkeypatch, run):
    """Сколько раз спрошен `Fraction.__hash__`. Значение хэша не меняется.

    Подмена живёт внутри одного теста и снимается `monkeypatch`: в ядро она не
    идёт, там `Fraction` не патчится вовсе.
    """

    original = fractions.Fraction.__hash__
    calls = [0]

    def counting(self):
        calls[0] += 1
        return original(self)

    monkeypatch.setattr(fractions.Fraction, "__hash__", counting)
    try:
        run()
    finally:
        monkeypatch.undo()
    return calls[0]


def test_the_new_representation_asks_for_far_fewer_modular_inverses(
    monkeypatch,
):
    """Число модульных обратных на путях тождества падает КРАТНО.

    Порог намеренно грубый (десятикратно), а не подогнанный под измеренное:
    ворота обязаны ловить возврат прежней цены, а не фиксировать конкретное
    число, которое законно шевелится от корпуса к корпусу. Точные числа
    полевых доменов живут в расписке карточки, а не в пороге.
    """

    polygon = dict(named_corpus())["cross"]

    def new_run():
        build_skeleton(polygon)

    def legacy_run():
        with legacy_identity():
            build_skeleton(polygon)

    legacy_calls = _count_fraction_hashes(monkeypatch, legacy_run)
    new_calls = _count_fraction_hashes(monkeypatch, new_run)
    assert legacy_calls > 0
    assert new_calls * 10 < legacy_calls, (legacy_calls, new_calls)


def test_the_span_state_memo_bounds_rebuilds_by_distinct_leaves(monkeypatch):
    """Пересборок пролёта ровно столько, сколько РАЗЛИЧНЫХ листов у вида.

    Это и есть объявленная граница «не больше числа различных (t, вхождение,
    поколение)»: время и поколение у вида одни, значит остаётся лист.
    """

    from cftuv_envelope.wavefront import symbolic_overlay as module

    builder, overlay, leaf = _tiny_overlay()
    built = [0]
    original = module.CandidateSpanStateV1

    def counting(*args, **kwargs):
        built[0] += 1
        return original(*args, **kwargs)

    monkeypatch.setattr(module, "CandidateSpanStateV1", counting)
    view = module.exact_overlay_view(builder, overlay)
    leaves = tuple(overlay.spans)
    for _ in range(7):
        for item in leaves:
            view.span_state(item)
    assert built[0] == len(set(leaves))
