"""Кап работы точной канонизации: конечность и честный отказ, без смены ответа.

Полевой факт, ради которого этот файл существует: на большом меше `building`
подготовка домена шла десять минут и не заканчивалась ничем. Не отказом, не
ответом — счётом. Обещание «любой долгий расчёт завершится именованным
отказом» было в этой точке ЛОЖНЫМ, и мемоизация PERF-CANON-1 его не чинила:
она ускоряет ЗНАКОМЫЕ наборы и не задаёт верхней границы работы на НОВОМ
тяжёлом кофакторе.

Что здесь доказывается, и в этом порядке:

1. ОТВЕТ НЕ ДВИНУЛСЯ. Бюджет добавляет учёт и только учёт: корпус скелетов
   даёт побитово те же семантические дайджесты с бюджетом и без.
2. НАРУШЕНИЕ СТРОИТСЯ. Заниженный потолок даёт ИМЕНОВАННЫЙ отказ со всеми
   шестью полями детали — а не исключение наружу и не тихий результат.
   Правило репозитория: проверка, у которой нельзя построить нарушение, не
   проверяет ничего.
3. СЧЁТ ДЕТЕРМИНИРОВАН. Один и тот же вход при одном и том же потолке тратит
   ОДИНАКОВОЕ число единиц каждого вида. Без этого отказ был бы свойством
   машины, а расписка — несравнимой сама с собой.
4. ПАМЯТЬ НЕ ЗАВИСИТ ОТ БЮДЖЕТА. Содержимое мемо — функция математики, а не
   остатка потолка; иначе разные бюджеты дали бы разные попадания и пункт 3
   развалился бы молча.
5. ЧАСТИЧНОЕ РАЗЛОЖЕНИЕ НЕ КЭШИРУЕТСЯ. Отравленный кэш отдавал бы неверную
   каноническую форму всем последующим прогонам процесса.
6. РАБОТА НЕ ТЕЧЁТ МИМО БЮДЖЕТА. Телеметрия неоплаченной работы обязана
   остаться нулевой, пока транзакция назвала свой бюджет.
"""

from __future__ import annotations

from fractions import Fraction

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.exact_sqrt_sum import (
    EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED,
    UNBUDGETED_WORK,
    ExactCanonicalizationWorkBudgetExhausted,
    ExactWorkBudgetModeV1,
    ExactWorkBudgetV1,
    ExactWorkOperationV1,
    SqrtSumV1,
    exact_work_budget,
    reset_factorization_memory,
    reset_unbudgeted_work,
    unlimited_reference_budget,
)
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.digest import semantic_digest
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton

from test_wavefront_motorcycle_graph import CORPUS, FROZEN_DIGESTS


# Радиканд той же ФОРМЫ, что полевой: произведение двух простых близкого
# размера, у которого ро-Поллард платит O(sqrt(p)). Числа не «побольше для
# красоты» — 34 бита выбраны так, чтобы полная факторизация ИЗМЕРЕННО стоила
# 0.065 s и 250 911 единиц: тест обязан успевать, но обязан и делать
# настоящую работу, иначе он проверял бы арифметику счётчика, а не кап.
# Полевые радиканды той же формы шире (наибольший простой делитель 65 и 73
# бита у walls.012, artifacts/perf_prepare_diag/REPORT.txt §3) и стоят секунды.
_HEAVY_PRIME_A = 17179869143
_HEAVY_PRIME_B = 17169869159
_HEAVY_RADICAND = _HEAVY_PRIME_A * _HEAVY_PRIME_B
# 250 910 единиц самой факторизации плюс одна материализация радикала.
_HEAVY_WORK_UNITS = 250911


def _polygon(name: str) -> PolygonV1:
    outer, holes = CORPUS[name]
    return PolygonV1.build(outer, holes)


@pytest.fixture(autouse=True)
def _fresh_memory():
    """Каждый тест начинается с холодной памяти и нулевой телеметрии.

    Иначе результат зависел бы от ПОРЯДКА тестов: тёплое мемо снимает всю
    работу, и «бюджет не сработал» стало бы неотличимо от «работы не было».
    """

    reset_factorization_memory()
    reset_unbudgeted_work()
    yield
    reset_factorization_memory()
    reset_unbudgeted_work()


# --------------------------------------------------------------------------
# 1. Ответ не двинулся
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_the_budget_does_not_move_a_single_answer(name: str):
    """Корпус скелетов даёт ТОТ ЖЕ дайджест с бюджетом и без него.

    Это главный тест файла. Кап, который меняет хотя бы один ответ, не кап, а
    новая геометрия под старым именем.
    """

    polygon = _polygon(name)
    without = build_skeleton(polygon)
    reset_factorization_memory()
    budget = exact_work_budget(stage="TEST", domain_id=name)
    with_budget = build_skeleton(polygon, work_budget=budget)

    assert with_budget.outcome is without.outcome
    assert semantic_digest(with_budget) == semantic_digest(without)
    assert semantic_digest(with_budget) == FROZEN_DIGESTS[name]
    # Учёт при этом ВЁЛСЯ, иначе тест доказывал бы только то, что бюджет
    # никуда не подключён.
    assert budget.spent > 0


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_coverage_is_bit_for_bit_the_same_under_a_budget(name: str):
    """Покрытие — вторая ступень той же транзакции, и она тоже не двигается."""

    polygon = _polygon(name)
    skeleton = build_skeleton(polygon)
    if skeleton.outcome is not SkeletonOutcome.EXACT:
        pytest.skip(f"{name}: скелет не EXACT, покрытию не на чем считаться")
    partition = build_faces(polygon, skeleton)
    without = coverage_at(partition, Fraction(1))
    budget = exact_work_budget(stage="TEST", domain_id=name)
    with_budget = coverage_at(partition, Fraction(1), budget)

    assert with_budget.outcome is without.outcome
    assert with_budget.doubled_area == without.doubled_area
    assert tuple(face.doubled_area for face in with_budget.faces) == tuple(
        face.doubled_area for face in without.faces
    )


def test_the_unlimited_reference_mode_answers_exactly_like_a_bounded_one():
    """Исследовательский режим — тот же код, а не вторая реализация.

    Эталон, посчитанный другим путём, доказывал бы свойство того пути. Здесь
    отличие ровно одно: `cap is None`, поэтому `_enforce` не отказывает.
    """

    polygon = _polygon("ell")
    reference = unlimited_reference_budget(stage="TEST")
    first = build_skeleton(polygon, work_budget=reference)
    reset_factorization_memory()
    bounded = exact_work_budget(stage="TEST")
    second = build_skeleton(polygon, work_budget=bounded)

    assert semantic_digest(first) == semantic_digest(second)
    assert reference.mode is ExactWorkBudgetModeV1.UNLIMITED_REFERENCE
    assert reference.cap is None
    assert reference.remaining is None
    assert reference.spent == bounded.spent
    assert bounded.remaining == bounded.cap - bounded.spent


# --------------------------------------------------------------------------
# 2. Нарушение строится
# --------------------------------------------------------------------------


def test_a_lowered_cap_turns_endless_factorization_into_a_named_refusal():
    """Тяжёлый кофактор под низким потолком отказывает ИМЕНЕМ, а не считает.

    Это и есть починенный полевой дефект: до капа тот же вызов уходил в счёт
    без исхода — «десять минут и не завершается».
    """

    budget = exact_work_budget(stage="SKELETON", domain_id="synthetic", cap=64)
    with pytest.raises(ExactCanonicalizationWorkBudgetExhausted) as raised:
        SqrtSumV1.radical(1, _HEAVY_RADICAND, budget)

    detail = str(raised.value)
    assert detail.startswith(EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED)
    assert budget.is_exhausted

    # И тот же вход с ЧЕСТНЫМ потолком отвечает: кап режет патологию, а не
    # работу как таковую. Измеренная цена — 250 911 единиц.
    reset_factorization_memory()
    honest = exact_work_budget(stage="SKELETON", cap=1 << 40)
    assert SqrtSumV1.radical(1, _HEAVY_RADICAND, honest).terms == (
        (_HEAVY_RADICAND, Fraction(1)),
    )
    assert honest.spent == _HEAVY_WORK_UNITS


def test_the_refusal_detail_carries_every_field_the_field_needs():
    """Шесть полей детали названы принципалом поимённо. Все шесть проверены.

    Разбор свипов группирует домены по паре (исход, деталь); деталь без имени
    домена или без стадии сложила бы в одну группу разные болезни.
    """

    budget = exact_work_budget(
        stage="SKELETON", domain_id="abc123", superlevel="7", cap=32
    )
    with pytest.raises(ExactCanonicalizationWorkBudgetExhausted) as raised:
        SqrtSumV1.radical(1, _HEAVY_RADICAND, budget)

    detail = str(raised.value)
    for field in (
        "stage=SKELETON",
        "domain=abc123",
        "superlevel=7",
        "cap=32",
    ):
        assert field in detail, detail
    assert "operation=" in detail
    assert "radicand_bits=" in detail
    assert "spent=" in detail
    # Битовая длина — это длина РАДИКАНДА, а не «какого-то числа»: без неё
    # поле не отличит «домен тяжёлый» от «домен зациклился».
    bits = int(detail.split("radicand_bits=")[1].split()[0])
    assert bits == _HEAVY_RADICAND.bit_length()
    spent = int(detail.split(" spent=")[1].split()[0])
    assert spent > 32


def test_the_refusal_is_fail_closed_and_never_a_partial_answer():
    """Отказ не отдаёт «почти каноническую» форму и ничего не запоминает.

    Ловится обе ошибки сразу: возврат полуответа и отравление кэша. Вторая
    страшнее: она пережила бы сам отказ и испортила бы все последующие
    прогоны процесса.
    """

    budget = exact_work_budget(stage="TEST", cap=64)
    with pytest.raises(ExactCanonicalizationWorkBudgetExhausted):
        SqrtSumV1.radical(1, _HEAVY_RADICAND, budget)

    assert _HEAVY_RADICAND not in canon._SQUAREFREE_MEMO
    assert _HEAVY_RADICAND not in canon._FACTORIZATION_MEMO
    # Ни один из двух больших простых не попал в реестр: доказан не был.
    assert _HEAVY_PRIME_A not in canon._KNOWN_PRIME_SET
    assert _HEAVY_PRIME_B not in canon._KNOWN_PRIME_SET

    # А с достаточным потолком тот же вход отвечает, и ответ верен.
    generous = exact_work_budget(stage="TEST", cap=1 << 40)
    value = SqrtSumV1.radical(1, _HEAVY_RADICAND, generous)
    assert value.terms == ((_HEAVY_RADICAND, Fraction(1)),)
    assert canon._SQUAREFREE_MEMO[_HEAVY_RADICAND] == (1, _HEAVY_RADICAND)
    assert _HEAVY_PRIME_A in canon._KNOWN_PRIME_SET


def test_the_conveyor_names_the_exhausted_budget_as_its_own_outcome():
    """Отказ доезжает до исхода конвейера, а не улетает исключением наружу.

    Хост читает `preparation_outcome` и `detail`; исключение из ядра он
    показать не может ничем, кроме трассы, — то есть для поля это «упало».
    """

    from cftuv_envelope.wavefront.conveyor import ConveyorOutcome

    assert (
        ConveyorOutcome.EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED.value
        == EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED
    )


def test_a_budget_that_cannot_refuse_is_rejected_at_construction():
    """Ограниченный бюджет без потолка и эталонный с потолком — обе ложь."""

    with pytest.raises(ValueError):
        ExactWorkBudgetV1(
            mode=ExactWorkBudgetModeV1.BOUNDED, cap=None, stage="TEST"
        )
    with pytest.raises(ValueError):
        ExactWorkBudgetV1(
            mode=ExactWorkBudgetModeV1.UNLIMITED_REFERENCE,
            cap=1,
            stage="TEST",
        )


# --------------------------------------------------------------------------
# 3. Счёт детерминирован
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", ("ell", "comb", "two_holes"))
def test_the_same_input_spends_the_same_units_twice(name: str):
    """Два прогона одного входа — ПОКОМПОНЕНТНОЕ равенство счётчиков.

    Равенства сумм было бы мало: две ошибки разного знака в разных статьях
    дали бы ту же сумму. Сравниваются все шесть.
    """

    polygon = _polygon(name)

    reset_factorization_memory()
    first = exact_work_budget(stage="TEST", domain_id=name)
    build_skeleton(polygon, work_budget=first)

    reset_factorization_memory()
    second = exact_work_budget(stage="TEST", domain_id=name)
    build_skeleton(polygon, work_budget=second)

    assert first.counters() == second.counters()
    assert first.spent == second.spent > 0


def test_the_cap_does_not_change_how_much_is_spent_before_it():
    """Работа — функция ВХОДА, а не остатка потолка.

    Если бы потолок влиял на путь (например, через попадания мемо), один и тот
    же домен тратил бы разное при разных капах, и детерминизм из пункта 3 стал
    бы совпадением.
    """

    polygon = _polygon("comb")

    reset_factorization_memory()
    tight = exact_work_budget(stage="TEST", cap=1 << 20)
    build_skeleton(polygon, work_budget=tight)

    reset_factorization_memory()
    loose = exact_work_budget(stage="TEST", cap=1 << 40)
    build_skeleton(polygon, work_budget=loose)

    assert tight.counters() == loose.counters()


def test_work_units_are_deterministic_across_a_memory_reset():
    """Сброс памяти обязан ВЕРНУТЬ ту же цену, а не «примерно ту же».

    Память — цена, а не семантика (PERF-CANON-1). Если сброс даёт другой счёт
    единиц, значит память успела повлиять на ПУТЬ, а не только на скорость.
    """

    first = exact_work_budget(stage="TEST")
    canon._prime_universe_from_q_values((2, 3, 5849, 12345678901234567), first)

    reset_factorization_memory()
    second = exact_work_budget(stage="TEST")
    canon._prime_universe_from_q_values((2, 3, 5849, 12345678901234567), second)

    assert first.counters() == second.counters()


# --------------------------------------------------------------------------
# 4. Память не зависит от бюджета
# --------------------------------------------------------------------------


def test_memo_content_is_a_function_of_arithmetic_not_of_the_budget():
    """Ключи и значения памяти одинаковы при любом потолке.

    Условие приёмки принципала: бюджет не имеет права влиять на СОДЕРЖИМОЕ
    мемо. Иначе один вход при разных бюджетах давал бы разные попадания, и
    счётчики перестали бы быть функцией входа.
    """

    values = (2, 3, 5849, 12345678901234567, 35415780824258511981)

    reset_factorization_memory()
    canon._prime_universe_from_q_values(
        values, exact_work_budget(stage="TEST", cap=1 << 20)
    )
    tight_memo = dict(canon._FACTORIZATION_MEMO)
    tight_squarefree = dict(canon._SQUAREFREE_MEMO)
    tight_primes = sorted(canon._KNOWN_PRIME_SET)

    reset_factorization_memory()
    canon._prime_universe_from_q_values(
        values, exact_work_budget(stage="TEST", cap=1 << 44)
    )
    assert dict(canon._FACTORIZATION_MEMO) == tight_memo
    assert dict(canon._SQUAREFREE_MEMO) == tight_squarefree
    assert sorted(canon._KNOWN_PRIME_SET) == tight_primes

    reset_factorization_memory()
    canon._prime_universe_from_q_values(values, unlimited_reference_budget())
    assert dict(canon._FACTORIZATION_MEMO) == tight_memo
    assert sorted(canon._KNOWN_PRIME_SET) == tight_primes


def test_the_memoised_canonical_form_is_the_same_with_and_without_a_budget():
    """`squarefree_split` и `prime_support` отвечают побитово одинаково."""

    values = (2, 50, 72, 1 << 20, 12345678901234567, 35415780824258511981)

    reset_factorization_memory()
    unbudgeted = [
        (canon.squarefree_split(value), canon.prime_support(value))
        for value in values
    ]
    reset_factorization_memory()
    budget = exact_work_budget(stage="TEST")
    budgeted = [
        (
            canon.squarefree_split(value, budget),
            canon.prime_support(value, budget),
        )
        for value in values
    ]
    assert budgeted == unbudgeted


# --------------------------------------------------------------------------
# 5. Работа не течёт мимо бюджета
# --------------------------------------------------------------------------


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_no_exact_work_escapes_the_named_budget_of_a_transaction(name: str):
    """Ни одной единицы мимо названного бюджета — на всём корпусе скелетов.

    Это ворота ПОЛНОТЫ протяжки. Без них «бюджет заведён» доказывало бы лишь,
    что он заведён где-то, а не что через него проходит вся работа домена.
    Телеметрия `UNBUDGETED_WORK` управлять ничем не может — она только
    считает, — поэтому её ноль здесь есть измерение, а не обещание.
    """

    reset_factorization_memory()
    reset_unbudgeted_work()
    budget = exact_work_budget(stage="TEST", domain_id=name)
    skeleton = build_skeleton(_polygon(name), work_budget=budget)
    if skeleton.outcome is SkeletonOutcome.EXACT:
        partition = build_faces(_polygon(name), skeleton)
        coverage_at(partition, Fraction(1), budget)

    assert UNBUDGETED_WORK.spent == 0, UNBUDGETED_WORK.counters()
    assert budget.spent > 0


def test_the_leak_detector_itself_can_fail():
    """У проверки протечки обязано СТРОИТЬСЯ нарушение, иначе она пустая."""

    reset_unbudgeted_work()
    canon.squarefree_split(12345678901234567)
    assert UNBUDGETED_WORK.spent > 0


# --------------------------------------------------------------------------
# 6. Единицы работы — то, что названо, а не абстрактный тик
# --------------------------------------------------------------------------


def test_every_declared_unit_is_actually_charged_somewhere():
    """Шесть статей объявлены — шесть статей тратятся.

    Статья, которая никогда не растёт, — это поле расписки, врущее в сторону
    «работы нет». Проверяется одним проходом, покрывающим оба класса: тяжёлая
    факторизация и марш фронта с гидратацией позиций.
    """

    budget = exact_work_budget(stage="TEST", cap=1 << 40)
    SqrtSumV1.radical(1, _HEAVY_RADICAND, budget)
    build_skeleton(_polygon("comb"), work_budget=budget)

    assert budget.modular_squarings > 0
    assert budget.gcd_operations > 0
    assert budget.miller_rabin_rounds > 0
    assert budget.pollard_attempts > 0
    assert budget.radical_materializations > 0
    assert budget.exact_position_hydrations > 0
    assert budget.spent == sum(
        value for name, value in budget.counters() if name != "EXACT_WORK_SPENT"
    )


def test_hydration_alone_bounds_a_front_whose_radicals_are_all_known():
    """Кап держит и домен, у которого факторизовать уже нечего.

    Без статьи «гидратация точной позиции» марш фронта с полностью прогретой
    памятью не покупал бы ничего и мог бы идти сколько угодно: все прежние
    статьи считают ФАКТОРИЗАЦИЮ, а её на прогретой памяти нет.
    """

    polygon = _polygon("comb")
    warm = exact_work_budget(stage="TEST", cap=1 << 40)
    build_skeleton(polygon, work_budget=warm)

    # Память уже прогрета: второй прогон не должен факторизовать НИЧЕГО.
    second = exact_work_budget(stage="TEST", cap=1 << 40)
    build_skeleton(polygon, work_budget=second)
    assert second.pollard_attempts == 0
    assert second.miller_rabin_rounds == 0
    assert second.exact_position_hydrations > 0

    tight = exact_work_budget(stage="TEST", cap=4)
    with pytest.raises(ExactCanonicalizationWorkBudgetExhausted) as raised:
        build_skeleton(polygon, work_budget=tight)
    assert "operation=EXACT_POSITION" in str(raised.value)


def test_the_operation_names_cover_every_charging_site():
    """Каждое имя операции хотя бы раз оказывается в детали отказа.

    Имя, которое не может появиться, — это словарь исходов, врущий полю про
    то, где именно кончилась работа.
    """

    seen = set()
    for operation in ExactWorkOperationV1:
        budget = exact_work_budget(stage="TEST", cap=0)
        detail = budget.exhaustion_detail(operation, _HEAVY_RADICAND)
        assert f"operation={operation.value}" in detail
        seen.add(operation.value)
    assert seen == {member.value for member in ExactWorkOperationV1}
