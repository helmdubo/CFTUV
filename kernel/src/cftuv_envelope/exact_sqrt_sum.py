"""Нейтральная сумма квадратных корней с рациональными коэффициентами.

Зачем это здесь. Скорость вершины волнового фронта равна `1/sin(alpha/2)` вдоль
биссектрисы, единичная нормаль ребра несёт `sqrt`, и поэтому время события в
общем случае иррационально. На целочисленной решётке иррациональность имеет
ровно одну форму: `sum c_m * sqrt(m)` по РАЗЛИЧНЫМ бесквадратным `m >= 1` с
рациональными `c_m`. Ничего другого в этой задаче не появляется.

Форма каноническая, и это главное её свойство. Квадратные корни различных
бесквадратных целых линейно независимы над Q (классическая теорема), поэтому
представление ЕДИНСТВЕННО, а значит:

- равенство двух величин — это совпадение конечного набора дробей, то есть
  ЧИСТО РАЦИОНАЛЬНАЯ проверка без единого извлечения корня;
- ноль отличается от «очень маленького» доказательством, а не допуском.

Знак — единственное место, где рациональной проверки не хватает, и он тоже
решается точно: сначала целочисленная интервальная оболочка (фильтр, не
решатель), при неудаче — рекурсивное сопряжение по простому из носителя.
Обе ветви на целых числах, порога нет ни в одной.

Чего здесь нет: float, mpmath, SymPy, epsilon. Оболочка построена на `isqrt`,
то есть на целых, и её границы точны по построению.
"""

from __future__ import annotations

from bisect import insort
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum
from fractions import Fraction
from math import gcd, isqrt, lcm
import random


# Наблюдение за тем, ЧЕМ решается знак. Заведено вместе с самой величиной,
# потому что вопрос среза — «можно ли сравнивать точно и дёшево» — без этих
# четырёх чисел не имеет измеримого ответа.
#
# `closed_rational_*` — ответ дала одна дробь, корней не было вообще;
# `closed_by_enclosure` — доказала целочисленная оболочка (фильтр);
# `closed_by_conjugation` — понадобилось точное сопряжение по простому.
SIGN_COUNTS = {
    "total": 0,
    "closed_rational_zero": 0,
    "closed_rational_nonzero": 0,
    "closed_by_enclosure": 0,
    "closed_by_conjugation": 0,
}


def reset_sign_counts() -> None:
    for key in SIGN_COUNTS:
        SIGN_COUNTS[key] = 0


class NegativeRadicandError(ValueError):
    """Под корнем отрицательное. В этой задаче не бывает и молча не проходит."""


class ZeroSqrtSumDivisorError(ZeroDivisionError):
    """Деление на доказанный ноль. Именованный отказ, а не NaN."""


# --------------------------------------------------------------------------
# Детерминированный кап работы точной канонизации
#
# Почему он здесь, а не «где-нибудь наверху». Единственная неограниченная
# работа этого модуля — целочисленная факторизация радиканда, и она вся
# проходит через одну воронку (`_factorization_pairs` -> `_rho_factors` ->
# `_pollard_rho` -> `_pollard_rho_brent_attempt`). Число «сколько её сделано»
# обязано жить там, где она делается, — иначе это второй экземпляр величины.
#
# Почему НЕ wall-clock. Секунда не воспроизводима: она свойство машины, а не
# входа. Один и тот же домен обязан на любой машине потратить ОДИНАКОВОЕ число
# единиц и получить ОДИНАКОВЫЙ исход — иначе отказ становится случайным, а
# полевая расписка перестаёт быть сравнимой сама с собой. Поэтому единицы
# только детерминированные: модульные возведения, операции gcd, раунды
# Миллера—Рабина, попытки Полларда, материализации канонических радикалов,
# гидратации точных позиций.
# --------------------------------------------------------------------------

EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED = (
    "EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED"
)


class ExactCanonicalizationWorkBudgetExhausted(ValueError):
    """Именованный отказ точной канонизации. Fail-closed, не «почти ответ»."""


class ExactWorkBudgetModeV1(str, Enum):
    """Режим бюджета. Продуктовый путь всегда `BOUNDED`.

    `UNLIMITED_REFERENCE` существует ради исследовательских прогонов: эталон,
    с которым сравнивают ограниченный путь, обязан быть посчитан ТЕМ ЖЕ кодом,
    иначе сравнение доказывает свойство второй реализации, а не капа. Режим
    объявлен членом перечисления, а не `cap=None` втихую, потому что «без
    границы» — это решение, и в расписке оно обязано читаться словом.
    """

    BOUNDED = "BOUNDED"
    UNLIMITED_REFERENCE = "UNLIMITED_REFERENCE"


class ExactWorkOperationV1(str, Enum):
    """Операция, на которой бюджет кончился. Идёт в деталь отказа."""

    PRIME_UNIVERSE = "PRIME_UNIVERSE"
    COPRIME_BASIS = "COPRIME_BASIS"
    PRIMALITY = "PRIMALITY"
    POLLARD_RHO_BRENT = "POLLARD_RHO_BRENT"
    SQUAREFREE_SPLIT = "SQUAREFREE_SPLIT"
    PRIME_SUPPORT = "PRIME_SUPPORT"
    EXACT_POSITION = "EXACT_POSITION"


class ExactWorkBudgetV1:
    """Детерминированный счётчик и потолок точной работы одной транзакции.

    Шесть счётчиков, и каждый — ФИЗИЧЕСКАЯ операция, а не абстрактный «тик»:

    * `modular_squarings` — шаги орбиты Брента (`y = y*y + c mod n`) и
      модульное возведение Миллера—Рабина (оплачивается длиной показателя в
      битах: `pow(base, d, n)` стоит ровно столько удвоений);
    * `gcd_operations` — каждый `gcd` факторизации и построения базиса;
    * `miller_rabin_rounds` — один раунд на базу доказательства простоты;
    * `pollard_attempts` — одна попытка Брента (пара параметров `(y, c)`);
    * `radical_materializations` — промах памяти `squarefree_split` или
      `prime_support`, то есть НОВЫЙ канонический радикал;
    * `exact_position_hydrations` — материализация точной позиции события.

    Сумма — `spent`, и она ограничена `cap`. Оплата идёт ВПЕРЁД: цикл сначала
    покупает свои шаги и только потом их делает, иначе исчерпание замечалось бы
    после того, как работа уже сделана, — то есть никогда для бесконечной.

    Объект намеренно НЕ frozen и НЕ глобален: он живёт ровно столько, сколько
    транзакция домена, и передаётся вниз явным параметром. Глобальная
    переменная или thread-local сделали бы «сколько работы потратил домен»
    свойством процесса, а не входа.
    """

    __slots__ = (
        "mode",
        "cap",
        "stage",
        "domain_id",
        "superlevel",
        "modular_squarings",
        "gcd_operations",
        "miller_rabin_rounds",
        "pollard_attempts",
        "radical_materializations",
        "exact_position_hydrations",
    )

    def __init__(
        self,
        *,
        mode: ExactWorkBudgetModeV1,
        cap: int | None,
        stage: str,
        domain_id: str = "",
        superlevel: str = "",
    ) -> None:
        if mode is ExactWorkBudgetModeV1.BOUNDED and cap is None:
            raise ValueError("ограниченный бюджет без потолка не ограничен")
        if mode is ExactWorkBudgetModeV1.UNLIMITED_REFERENCE and cap is not None:
            raise ValueError("эталонный бюджет с потолком не эталон")
        self.mode = mode
        self.cap = cap
        self.stage = stage
        self.domain_id = domain_id
        self.superlevel = superlevel
        self.modular_squarings = 0
        self.gcd_operations = 0
        self.miller_rabin_rounds = 0
        self.pollard_attempts = 0
        self.radical_materializations = 0
        self.exact_position_hydrations = 0

    # ---- состояние ------------------------------------------------------

    @property
    def spent(self) -> int:
        return (
            self.modular_squarings
            + self.gcd_operations
            + self.miller_rabin_rounds
            + self.pollard_attempts
            + self.radical_materializations
            + self.exact_position_hydrations
        )

    @property
    def remaining(self) -> int | None:
        """Остаток. `None` у эталонного режима: у него остатка нет по смыслу."""

        return None if self.cap is None else self.cap - self.spent

    @property
    def is_exhausted(self) -> bool:
        return self.cap is not None and self.spent > self.cap

    def counters(self) -> tuple[tuple[str, int], ...]:
        return (
            ("EXACT_WORK_MODULAR_SQUARINGS", self.modular_squarings),
            ("EXACT_WORK_GCD_OPERATIONS", self.gcd_operations),
            ("EXACT_WORK_MILLER_RABIN_ROUNDS", self.miller_rabin_rounds),
            ("EXACT_WORK_POLLARD_ATTEMPTS", self.pollard_attempts),
            (
                "EXACT_WORK_RADICAL_MATERIALIZATIONS",
                self.radical_materializations,
            ),
            (
                "EXACT_WORK_EXACT_POSITION_HYDRATIONS",
                self.exact_position_hydrations,
            ),
            ("EXACT_WORK_SPENT", self.spent),
        )

    def at_stage(self, stage: str) -> "ExactWorkBudgetV1":
        """Тот же счёт, другая стадия. Потраченное НЕ обнуляется.

        Подготовка и покрытие оплачивают одну транзакцию домена; обнуление на
        границе стадии сделало бы кап границей самой дорогой стадии, а не
        работы домена.
        """

        self.stage = stage
        return self

    # ---- траты ----------------------------------------------------------

    def spend_modular_squarings(
        self, count: int, *, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        self.modular_squarings += count
        self._enforce(operation, radicand)

    def spend_gcd_operations(
        self, count: int, *, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        self.gcd_operations += count
        self._enforce(operation, radicand)

    def spend_miller_rabin_rounds(
        self, count: int, *, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        self.miller_rabin_rounds += count
        self._enforce(operation, radicand)

    def spend_pollard_attempts(
        self, count: int, *, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        self.pollard_attempts += count
        self._enforce(operation, radicand)

    def spend_radical_materializations(
        self, count: int, *, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        self.radical_materializations += count
        self._enforce(operation, radicand)

    def spend_exact_position_hydrations(
        self, count: int, *, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        self.exact_position_hydrations += count
        self._enforce(operation, radicand)

    # ---- отказ ----------------------------------------------------------

    def exhaustion_detail(
        self, operation: ExactWorkOperationV1, radicand: int
    ) -> str:
        """Деталь отказа. Её читает разбор свипов, группируя по (исход, деталь).

        Поля названы принципалом поимённо: стадия, битовая длина радиканда,
        потраченная работа, идентичность домена, superlevel, операция. Всё
        шесть здесь, и ни одно не пересказано словами — только числа и имена.
        """

        return (
            f"{EXACT_CANONICALIZATION_WORK_BUDGET_EXHAUSTED} "
            f"stage={self.stage} "
            f"operation={operation.value} "
            f"radicand_bits={int(radicand).bit_length()} "
            f"cap={self.cap} "
            f"spent={self.spent} "
            f"modular_squarings={self.modular_squarings} "
            f"gcd_operations={self.gcd_operations} "
            f"miller_rabin_rounds={self.miller_rabin_rounds} "
            f"pollard_attempts={self.pollard_attempts} "
            f"radical_materializations={self.radical_materializations} "
            f"exact_position_hydrations={self.exact_position_hydrations} "
            f"domain={self.domain_id} "
            f"superlevel={self.superlevel}"
        )

    def _enforce(
        self, operation: ExactWorkOperationV1, radicand: int
    ) -> None:
        if self.cap is None or self.spent <= self.cap:
            return
        raise ExactCanonicalizationWorkBudgetExhausted(
            self.exhaustion_detail(operation, radicand)
        )


# Объявленный потолок точной канонизации ОДНОЙ транзакции домена (подготовка и
# покрытие вместе). Категория WORK_BUDGET: превышение не меняет ни одного
# ответа, оно превращает бесконечный счёт в именованный отказ.
#
# ГДЕ СНЯТЫ ЧИСЛА. На 6ce0227 — вершине field-stable кандидата, а НЕ на вершине
# аудит-ветки. Причина названа принципалом и проверена: на вершине walls.012
# обрывается на 2 узлах вместо 12, а wall 2.001 жуёт подготовку 15–20 s вместо
# 2.3 s, поэтому потолок, снятый там, унаследовал бы ЦЕНУ РЕГРЕССИИ и объявил
# бы её нормой здорового домена. Модули, которые считают работу
# (`exact_sqrt_sum`, `event_time`, `motorcycle`, `coverage`), на обеих вершинах
# ПОБИТОВО одинаковы, поэтому счётчик переносится без правки семантики.
#
# ИЗМЕРЕНО НА 6ce0227 (density 0, alpha 0.45, харнесс
# `artifacts/perf_prepare_diag` плюс `artifacts/exact_work_budget/`; полные
# строки — `artifacts/exact_work_budget/RECEIPT.json`):
#   walls.012 d0        554 580 единиц   0.9 s   EXACT
#   walls.012 d1         33 665 единиц   0.6 s   EXACT   <- бывшее зависание
#   wall 2.001 d0        24 508 единиц   2.6 s   EXACT   (alpha 0.254 — то же)
#   building патч 109    13 651 единица  0.7 s   EXACT
#   building патч 121     8 772 единицы  0.1 s   EXACT
# Худший ИЗМЕРЕННЫЙ ЗДОРОВЫЙ домен — walls.012 d0, 554 580 единиц. Разброс
# внутри здоровой пятёрки 63x (8 772…554 580), и это часть обоснования запаса:
# домены законно отличаются друг от друга на два порядка.
#
# ПЕРЕМЕРЕНО ПОСЛЕ ОТКАТА ОБЩЕГО ПОДВЫРАЖЕНИЯ. `_apply_trace_crash` вернул два
# отдельных вызова `_as_sqrt_sum` вместо одного вынесенного — карточка обязана
# добавлять учёт и не трогать путь. Все шесть полевых прогонов дали ТЕ ЖЕ
# счётчики до единицы: восстановленный вызов бьёт в память канонизации, поэтому
# стоит секунды и НОЛЬ единиц. Это не совпадение, а свойство статьи
# `radical_materializations`: она платится за ПРОМАХ памяти, а не за вызов.
# Обоснование потолка от отката не сдвинулось, и это проверено, а не заявлено.
#
# ПОТОЛОК ОГРАНИЧЕН С ДВУХ СТОРОН, и обе границы измерены.
# Снизу: он обязан пропускать walls.012 density 1 — тот самый вход, который до
# PERF-CANON-1 не возвращался 900+ s. Запас над ним 249x.
# Сверху: его ЦЕНА обязана оставаться ценой домена, а не зависанием. Цена
# ЕДИНИЦЫ снята боевым путём (`artifacts/exact_work_budget/unit_price.py`:
# `_rho_factors` на полусоставном числе заданной ширины, секунды делятся на
# потраченные единицы), а не голым циклом — голый цикл на этой машине вдвое
# дешевле и объявил бы потолок дешевле, чем он есть. Измерено 453–501 нс на
# единицу при радиканде 246 бит (максимум walls.012, REPORT.txt §3), значит
# полный потолок стоит 3.8–4.2 s. Полевой случай владельца — десять минут без
# исхода; четыре секунды с именем — это ответ, а не зависание.
#
# 8 388 608 = 2^23: запас 15.1x над худшим измеренным ЗДОРОВЫМ доменом.
# Почему не больше: 2^24 стоил бы 8.4 s на потолке, то есть на меше в 122
# домена патология перестала бы отличаться от зависания по ощущению владельца.
# Почему не меньше: 2^22 даёт всего 7.6x, а здоровые домены расходятся в 63
# раза уже внутри измеренной пятёрки — такой кап резал бы законную работу.
#
# ЧТО КАП ОТСЕКАЕТ ПО СУЩЕСТВУ. Ро-Поллард находит делитель p примерно за
# 1.18*sqrt(p) шагов, поэтому 2^23 единиц — это делители примерно до 2^45.
# Простые крупнее рождаются не из геометрии, а из near-planar проекции,
# поднимающей знаменатели метрики (REPORT.txt §3): это ПАТОЛОГИЯ ВХОДА, и
# правильный продуктовый ответ на неё — именованный отказ. (Основную долю этого
# роста снимает приведённый целочисленный базис плоскости, NEAR_PLANAR V2, 2026-10-03:
# радиканды `building` 227-264 -> 138-145 бит; кап остаётся защитой от остального.)
# Отказ по существу домена выглядит иначе: патч 89 `building` отвергается не
# капом работы, а бюджетом ширины NEAR_PLANAR_WIDTH_DISTORTION_BUDGET_EXCEEDED.
_EXACT_CANONICALIZATION_WORK_CAP = 1 << 23


def exact_work_budget(
    *,
    stage: str,
    domain_id: str = "",
    superlevel: str = "",
    cap: int | None = None,
) -> ExactWorkBudgetV1:
    """Свежий ограниченный бюджет одной транзакции. Потолок по умолчанию — свой."""

    return ExactWorkBudgetV1(
        mode=ExactWorkBudgetModeV1.BOUNDED,
        cap=_EXACT_CANONICALIZATION_WORK_CAP if cap is None else cap,
        stage=stage,
        domain_id=domain_id,
        superlevel=superlevel,
    )


def unlimited_reference_budget(
    *,
    stage: str = "REFERENCE",
    domain_id: str = "",
    superlevel: str = "",
) -> ExactWorkBudgetV1:
    """Бюджет исследовательского прогона: считает, но никогда не отказывает."""

    return ExactWorkBudgetV1(
        mode=ExactWorkBudgetModeV1.UNLIMITED_REFERENCE,
        cap=None,
        stage=stage,
        domain_id=domain_id,
        superlevel=superlevel,
    )


# Учёт работы, за которую НИКТО не назвал бюджет. Это не управление, а
# ТЕЛЕМЕТРИЯ — ровно как соседний `SIGN_COUNTS`: отказать эта запись не может
# ничем и никогда. Она отвечает на единственный вопрос приёмки: «прошла ли хоть
# одна единица точной работы мимо транзакционного бюджета». Полевые ворота
# карточки читают именно её и требуют нуля.
UNBUDGETED_WORK = unlimited_reference_budget(stage="UNBUDGETED")


def reset_unbudgeted_work() -> None:
    """Обнулить телеметрию неоплаченной работы. Ответов не меняет."""

    UNBUDGETED_WORK.modular_squarings = 0
    UNBUDGETED_WORK.gcd_operations = 0
    UNBUDGETED_WORK.miller_rabin_rounds = 0
    UNBUDGETED_WORK.pollard_attempts = 0
    UNBUDGETED_WORK.radical_materializations = 0
    UNBUDGETED_WORK.exact_position_hydrations = 0


def _named(budget: ExactWorkBudgetV1 | None) -> ExactWorkBudgetV1:
    """Бюджет вызова: названный — свой, безымянный — телеметрия неоплаченного."""

    return UNBUDGETED_WORK if budget is None else budget


# --------------------------------------------------------------------------
# Целочисленная факторизация: нужна ровно для бесквадратной части радикала
# --------------------------------------------------------------------------

_MILLER_RABIN_BASES = (2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37)


def _is_prime(n: int, budget: ExactWorkBudgetV1) -> bool:
    """Детерминированный Миллер—Рабин для n < 3.3e24, то есть для всех наших.

    Оплата: один раунд на базу плюс модульные возведения. Возведение
    `pow(base, d, n)` стоит `d.bit_length()` удвоений — это и есть его цена,
    и она растёт вместе с числом, ради чего кап и заводился.
    """

    if n < 2:
        return False
    for small in _MILLER_RABIN_BASES:
        if n % small == 0:
            return n == small
    d, r = n - 1, 0
    while d % 2 == 0:
        d //= 2
        r += 1
    exponent_cost = d.bit_length()
    for base in _MILLER_RABIN_BASES:
        budget.spend_miller_rabin_rounds(
            1, operation=ExactWorkOperationV1.PRIMALITY, radicand=n
        )
        budget.spend_modular_squarings(
            exponent_cost,
            operation=ExactWorkOperationV1.PRIMALITY,
            radicand=n,
        )
        x = pow(base, d, n)
        if x == 1 or x == n - 1:
            continue
        witnessed = False
        steps = 0
        for _ in range(r - 1):
            x = x * x % n
            steps += 1
            if x == n - 1:
                witnessed = True
                break
        budget.spend_modular_squarings(
            steps, operation=ExactWorkOperationV1.PRIMALITY, radicand=n
        )
        if not witnessed:
            return False
    return True


def _pollard_rho(n: int, budget: ExactWorkBudgetV1) -> int:
    """Нетривиальный делитель составного n. Брент, детерминированный посев."""

    if n % 2 == 0:
        return 2
    rng = random.Random(n)
    while True:
        budget.spend_pollard_attempts(
            1, operation=ExactWorkOperationV1.POLLARD_RHO_BRENT, radicand=n
        )
        c = rng.randrange(1, n)
        y = rng.randrange(0, n)
        divisor = _pollard_rho_brent_attempt(
            n, y, c, batch_size=64, budget=budget
        )
        if divisor is not None:
            assert 1 < divisor < n and n % divisor == 0
            return divisor


def _pollard_rho_brent_attempt(
    n: int,
    y: int,
    c: int,
    *,
    batch_size: int,
    budget: ExactWorkBudgetV1,
) -> int | None:
    """Одна попытка Брента; `None` требует следующую пару параметров.

    GCD берётся с произведения не более `batch_size` разностей. Если пакет
    дал `g == n` (включая `q == 0`), сохранённая граница пакета проигрывается
    по одному шагу. Так общий делитель не теряется внутри `gcd(0, n) == n`,
    а неудачная орбита не меняет состояние локального `Random(n)`.

    ЗДЕСЬ И БЫЛА БЕСКОНЕЧНОСТЬ. Внешний цикл удваивает `power` без границы, и
    на кофакторе с большим простым делителем он считает столько, сколько
    попросит вход. Теперь каждый шаг орбиты покупается ВПЕРЁД, поэтому
    исчерпание наступает до работы, а не после неё, и приходит именем.
    """

    if batch_size < 1:
        raise ValueError("размер пакета Брента должен быть положительным")
    operation = ExactWorkOperationV1.POLLARD_RHO_BRENT
    power = 1
    while True:
        x = y
        budget.spend_modular_squarings(power, operation=operation, radicand=n)
        for _ in range(power):
            y = (y * y + c) % n
        offset = 0
        while offset < power:
            checkpoint = y
            steps = min(batch_size, power - offset)
            budget.spend_modular_squarings(
                steps, operation=operation, radicand=n
            )
            budget.spend_gcd_operations(1, operation=operation, radicand=n)
            product = 1
            for _ in range(steps):
                y = (y * y + c) % n
                product = product * abs(x - y) % n
            divisor = gcd(product, n)
            if divisor == 1:
                offset += steps
                continue
            if 1 < divisor < n:
                assert n % divisor == 0
                return divisor

            budget.spend_modular_squarings(
                steps, operation=operation, radicand=n
            )
            budget.spend_gcd_operations(
                steps, operation=operation, radicand=n
            )
            replay = checkpoint
            for _ in range(steps):
                replay = (replay * replay + c) % n
                divisor = gcd(abs(x - replay), n)
                if divisor == 1:
                    continue
                if 1 < divisor < n:
                    assert n % divisor == 0
                    return divisor
                # Орбита замкнулась раньше доказанного делителя: новая попытка.
                return None
            return None
        power *= 2


def _rho_factors(n: int, budget: ExactWorkBudgetV1) -> dict[int, int]:
    """Разложение с нуля: Миллер—Рабин, точный квадрат, ро-Поллард."""

    factors: dict[int, int] = {}
    stack = [n]
    while stack:
        value = stack.pop()
        if value == 1:
            continue
        if _is_prime(value, budget):
            factors[value] = factors.get(value, 0) + 1
            continue
        root = isqrt(value)
        if root * root == value:
            stack.append(root)
            stack.append(root)
            continue
        divisor = _pollard_rho(value, budget)
        assert 1 < divisor < value and value % divisor == 0
        stack.append(divisor)
        stack.append(value // divisor)
    return factors


# --------------------------------------------------------------------------
# Память канонизации: реестр доказанных простых и взаимно простой базис
#
# Разложение на простые ЕДИНСТВЕННО, поэтому ни память, ни базис не могут
# изменить ответ: они меняют только ПУТЬ, которым он получен. Всё, что ниже,
# — цена, а не семантика; сброс памяти обязан давать те же кортежи.
#
# Ёмкости названы, потому что владелец потребовал реестра допусков. Категория
# у всех трёх одна — NUMERIC/WORK budget: превышение не отказ и не другой
# ответ, а возврат к прежнему (дорогому) пути.
# --------------------------------------------------------------------------

# Память разложений (Р1). На walls.012 density 0 измерено 35 вызовов
# `_factorize` при 15 различных радикандах; на худшем измеренном домене
# `building` (патч 109) — 60 вызовов. 8192 записи — два порядка запаса над
# худшим измеренным доменом; запись держит целое-ключ и кортеж пар.
_FACTORIZATION_MEMO_ENTRIES = 1 << 13

# Реестр доказанных простых (Р2). На walls.012 их 28 на домен, на пятёрке
# самых тяжёлых доменов `building` — меньше трёхсот. 8192 — тот же порядок
# запаса. Переполнение очищает реестр целиком: половинчатый реестр остался бы
# корректным, но непредсказуемым по цене, а полный сброс воспроизводим.
_KNOWN_PRIME_REGISTRY_ENTRIES = 1 << 13

# Бюджет расщеплений при построении взаимно простого базиса. Каждое
# расщепление строго уменьшает сумму элементов, поэтому цикл конечен и без
# бюджета; бюджет — верхняя граница, названная явно. Измерено: базис пяти
# радикандов walls.012 строится за 0.3 мс и 10 расщеплений.
_COPRIME_BASIS_SPLIT_BUDGET = 1 << 12

_KNOWN_PRIMES: list[int] = []
_KNOWN_PRIME_SET: set[int] = set()

# Память канонизации явными словарями, а не `lru_cache`. Причина одна и она
# не про вкус: бюджет обязан приходить сюда ПАРАМЕТРОМ, а `lru_cache` включил
# бы его в ключ и превратил каждую транзакцию в промах. Вытеснение осталось
# LRU-порядком (попадание переносит запись в конец), поэтому цена не поехала.
#
# ЧАСТИЧНОЕ РАЗЛОЖЕНИЕ СЮДА НЕ ПОПАДАЕТ НИКОГДА. Запись делается ПОСЛЕ
# полного разложения; исчерпание бюджета летит исключением снизу и до записи
# не доходит. Кэш, отравленный незавершённым результатом, отдавал бы неверную
# каноническую форму всем последующим прогонам процесса — это хуже, чем не
# иметь кэша вовсе.
_FACTORIZATION_MEMO: dict[int, tuple[tuple[int, int], ...]] = {}
_SQUAREFREE_MEMO: dict[int, tuple[int, int]] = {}
_PRIME_SUPPORT_MEMO: dict[int, tuple[int, ...]] = {}


def reset_factorization_memory() -> None:
    """Сбросить память канонизации. Ответы не меняются — меняется только цена."""

    _KNOWN_PRIMES.clear()
    _KNOWN_PRIME_SET.clear()
    _FACTORIZATION_MEMO.clear()
    _SQUAREFREE_MEMO.clear()
    _PRIME_SUPPORT_MEMO.clear()


@contextmanager
def isolated_factorization_memory():
    """Память канонизации, ХОЛОДНАЯ внутри блока; память вызывающего возвращается.

    Нужна там, где ответ зависит от бюджета точной работы (ярлык вершины
    развёртки: хватило ли бюджета на точный знак): попадание в память
    возвращается до оплаты, и тот же вход при тёплой памяти стоил бы меньше —
    ярлык оказался бы свойством ИСТОРИИ процесса, а не входа. Внутри блока
    цена — функция входа; после блока память вызывающего восстановлена целиком,
    поэтому окружающий счёт (цену подготовки и покрытия) блок не двигает.
    """

    saved = (
        list(_KNOWN_PRIMES),
        set(_KNOWN_PRIME_SET),
        dict(_FACTORIZATION_MEMO),
        dict(_SQUAREFREE_MEMO),
        dict(_PRIME_SUPPORT_MEMO),
    )
    reset_factorization_memory()
    try:
        yield
    finally:
        reset_factorization_memory()
        known, known_set, factorization, squarefree, support = saved
        _KNOWN_PRIMES.extend(known)
        _KNOWN_PRIME_SET.update(known_set)
        _FACTORIZATION_MEMO.update(factorization)
        _SQUAREFREE_MEMO.update(squarefree)
        _PRIME_SUPPORT_MEMO.update(support)


def _register_prime(prime: int) -> None:
    """Запомнить доказанное простое. Реестр возрастающий и без повторов."""

    if prime in _KNOWN_PRIME_SET:
        return
    if len(_KNOWN_PRIMES) >= _KNOWN_PRIME_REGISTRY_ENTRIES:
        _KNOWN_PRIMES.clear()
        _KNOWN_PRIME_SET.clear()
    insort(_KNOWN_PRIMES, prime)
    _KNOWN_PRIME_SET.add(prime)


def _strip_known_primes(n: int) -> tuple[dict[int, int], int]:
    """Снять уже доказанные простые ДЕЛЕНИЕМ, вернуть кофактор.

    Реестр возрастающий, поэтому обход прекращается на первом простом,
    большем остатка. Ни одного нового доказательства здесь не возникает.
    """

    factors: dict[int, int] = {}
    remainder = n
    for prime in _KNOWN_PRIMES:
        if prime > remainder:
            break
        if remainder % prime:
            continue
        power = 0
        while remainder % prime == 0:
            remainder //= prime
            power += 1
        factors[prime] = power
    return factors, remainder


def _factorization_pairs(
    n: int, budget: ExactWorkBudgetV1 | None = None
) -> tuple[tuple[int, int], ...]:
    """Разложение как неизменяемый кортеж пар, по возрастанию простого.

    Кешируемый слой отделён от `_factorize` намеренно: наружу обязан уходить
    свежий `dict`, иначе вызывающий получил бы общий изменяемый объект.
    """

    if n < 2:
        return ()
    budget = _named(budget)
    cached = _FACTORIZATION_MEMO.get(n)
    if cached is not None:
        # Попадание переносит запись в конец: порядок словаря есть порядок
        # вытеснения, и без переноса он стал бы FIFO вместо LRU.
        del _FACTORIZATION_MEMO[n]
        _FACTORIZATION_MEMO[n] = cached
        return cached
    factors, remainder = _strip_known_primes(n)
    if remainder > 1:
        for prime, power in _rho_factors(remainder, budget).items():
            factors[prime] = factors.get(prime, 0) + power
    for prime in factors:
        _register_prime(prime)
    pairs = tuple(sorted(factors.items()))
    if len(_FACTORIZATION_MEMO) >= _FACTORIZATION_MEMO_ENTRIES:
        del _FACTORIZATION_MEMO[next(iter(_FACTORIZATION_MEMO))]
    _FACTORIZATION_MEMO[n] = pairs
    return pairs


def _factorize(
    n: int, budget: ExactWorkBudgetV1 | None = None
) -> dict[int, int]:
    """Разложение `n` на простые: `{простое: степень}`.

    Р1 — память. Функция чистая (разложение единственно), поэтому мемоизация
    не может изменить ответ. Измерено на walls.012 density 0: 35 вызовов при
    15 различных радикандах, 6.01 s против 1.93 s.

    Р2 — реестр простых. Уже доказанные простые снимаются делением, и
    ро-Поллард запускается только на кофакторе. Радикалы одного домена родом
    из одних и тех же законов прихода и делят почти все свои простые: на
    walls.012 ВСЕ 28 простых происходят из первого же набора, поэтому десять
    последующих радикандов (до 201 бита) разлагаются делением досуха.
    """

    return dict(_factorization_pairs(n, budget))


def _coprime_basis(
    values: tuple[int, ...], budget: ExactWorkBudgetV1 | None = None
) -> list[int]:
    """Взаимно простой базис набора. Только gcd, ни одной факторизации.

    Каждое расщепление заменяет пару `(a, b)` на `(g, a//g, b//g)`, что
    строго уменьшает сумму элементов, поэтому цикл конечен. По исчерпании
    `_COPRIME_BASIS_SPLIT_BUDGET` расщепления прекращаются, и элемент кладётся
    в базис как есть: базис перестаёт быть взаимно простым, но его назначение
    — только УЗНАТЬ простые, и ответ от этого не зависит.
    """

    budget = _named(budget)
    basis: list[int] = []
    stack = [value for value in values if value > 1]
    splits = 0
    while stack:
        value = stack.pop()
        placed = False
        for index, atom in enumerate(basis):
            budget.spend_gcd_operations(
                1,
                operation=ExactWorkOperationV1.COPRIME_BASIS,
                radicand=value,
            )
            common = gcd(value, atom)
            if common == 1:
                continue
            if common == value and common == atom:
                placed = True
                break
            if splits >= _COPRIME_BASIS_SPLIT_BUDGET:
                break
            splits += 1
            basis.pop(index)
            stack.append(common)
            if atom // common > 1:
                stack.append(atom // common)
            if value // common > 1:
                stack.append(value // common)
            placed = True
            break
        if not placed:
            basis.append(value)
    return basis


def _seed_factorization_basis(
    radicands: tuple[int, ...], budget: ExactWorkBudgetV1 | None = None
) -> None:
    """Р2: разложить НАБОР радикандов через его взаимно простой базис.

    Поэлементная факторизация платит за наибольший простой делитель КАЖДОГО
    радиканда; базис платит за него один раз. Измерено на walls.012 density 0:
    набор из пяти радикандов (203…246 бит) даёт базис из десяти элементов
    максимум в 117 бит за 0.3 мс gcd-ов, и его факторизация стоит 0.121 s
    против 1.83 s поэлементной.

    Тождество, на котором это стоит: базис порождает те же простые, что и
    сам набор, а разложение на простые единственно. Поэтому `squarefree_split`
    и `prime_support` возвращают побитово те же значения.
    """

    budget = _named(budget)
    residues = []
    for radicand in radicands:
        _, remainder = _strip_known_primes(radicand)
        if remainder > 1:
            residues.append(remainder)
    if len(residues) < 2:
        return
    for atom in _coprime_basis(tuple(residues), budget):
        _factorization_pairs(atom, budget)


def squarefree_split(
    n: int, budget: ExactWorkBudgetV1 | None = None
) -> tuple[int, int]:
    """`n = g*g*m` с бесквадратным `m`. Кешируется: радикалы повторяются.

    Промах памяти — это МАТЕРИАЛИЗАЦИЯ КАНОНИЧЕСКОГО РАДИКАЛА, и она одна
    оплачивается: попадание работы не делает и платить ему не за что. Ключ и
    значение памяти зависят ТОЛЬКО от `n`; бюджет в них не входит ни одним
    битом, иначе один и тот же вход при разных потолках давал бы разные
    попадания, и счётчики перестали бы быть функцией входа.
    """

    if n < 0:
        raise NegativeRadicandError(f"под корнем {n}")
    if n in (0, 1):
        return (0, 0) if n == 0 else (1, 1)
    cached = _SQUAREFREE_MEMO.get(n)
    if cached is not None:
        return cached
    budget = _named(budget)
    budget.spend_radical_materializations(
        1, operation=ExactWorkOperationV1.SQUAREFREE_SPLIT, radicand=n
    )
    outside, inside = 1, 1
    for prime, power in _factorize(n, budget).items():
        outside *= prime ** (power // 2)
        if power % 2:
            inside *= prime
    result = (outside, inside)
    _SQUAREFREE_MEMO[n] = result
    return result


def prime_support(
    radicand: int, budget: ExactWorkBudgetV1 | None = None
) -> tuple[int, ...]:
    """Простые бесквадратного радикала, по возрастанию."""

    if radicand <= 1:
        return ()
    cached = _PRIME_SUPPORT_MEMO.get(radicand)
    if cached is not None:
        return cached
    budget = _named(budget)
    budget.spend_radical_materializations(
        1,
        operation=ExactWorkOperationV1.PRIME_SUPPORT,
        radicand=radicand,
    )
    support = tuple(sorted(_factorize(radicand, budget)))
    _PRIME_SUPPORT_MEMO[radicand] = support
    return support


def _prime_universe_from_q_values(
    q_values: tuple[int | Fraction, ...],
    budget: ExactWorkBudgetV1 | None = None,
) -> tuple[int, ...]:
    """Простые нечётных степеней примитивных `q`, один раз на операцию.

    Для `q = p/r` подкоренное целое канонической формы равно `p*r`, потому
    что `sqrt(p/r) = sqrt(p*r)/r`. Факторизация здесь конечна и проверяется
    обратным произведением; дальше деление использует только делимость.

    Это ЕДИНСТВЕННОЕ место, где набор радикандов виден целиком, поэтому
    взаимно простой базис (Р2) строится именно здесь. Сам обход остаётся
    прежним: `_factorize` вызывается по одному разу на радиканд и в том же
    возрастающем порядке — базис только удешевляет эти вызовы.
    """

    budget = _named(budget)
    transformed: set[int] = set()
    for raw_q in q_values:
        q = Fraction(raw_q)
        if q < 0:
            raise NegativeRadicandError(f"под корнем {q}")
        if q == 0:
            continue
        transformed.add(q.numerator * q.denominator)
    _seed_factorization_basis(tuple(sorted(transformed)), budget)
    primes: set[int] = set()
    for radicand in sorted(transformed):
        factors = _factorize(radicand, budget)
        reconstructed = 1
        for prime, power in factors.items():
            reconstructed *= prime**power
            if power % 2:
                primes.add(prime)
        if reconstructed != radicand:
            raise ArithmeticError(
                f"факторизация {radicand} не восстановила исходное число"
            )
    return tuple(sorted(primes))


def _support_from_prime_universe(
    radicand: int,
    prime_universe: tuple[int, ...],
) -> tuple[int, ...] | None:
    """Точный носитель бесквадратного радикала либо `None`.

    Остаток обязан стать единицей, произведение — исходным радикандом, а
    повторная делимость тем же простым запрещена. Любой промах означает,
    что локальное доказательство неполно и вся операция должна уйти в legacy.
    """

    if radicand <= 1:
        return () if radicand == 1 else None
    remainder = radicand
    product = 1
    support: list[int] = []
    for prime in prime_universe:
        if remainder % prime != 0:
            continue
        remainder //= prime
        product *= prime
        support.append(prime)
        if remainder % prime == 0:
            return None
        if remainder == 1:
            break
    if remainder != 1 or product != radicand:
        return None
    return tuple(support)


def _pick_prime_from_universe(
    terms: dict[int, Fraction],
    prime_universe: tuple[int, ...],
) -> int | None:
    """Минимальный простой носителя, если ВСЕ радикалы восстановлены точно."""

    smallest: int | None = None
    for radicand, coefficient in terms.items():
        if not coefficient or radicand == 1:
            continue
        support = _support_from_prime_universe(radicand, prime_universe)
        if not support:
            return None
        if smallest is None or support[0] < smallest:
            smallest = support[0]
    return smallest


# --------------------------------------------------------------------------
# Целочисленное ядро знака, оболочки и произведения
#
# Набор `c_m = a_m / L` с общим знаменателем `L` — это целые `a_m` и ОДНО
# положительное число. Оболочка и произведение на них принимают те же
# решения, что на `Fraction`, без сотен тысяч нормировок по gcd: нормировка
# остаётся одна на результирующий член. Ничего здесь не приближает и не
# хранит: хранимые `terms` по-прежнему `Fraction`, `repr` и порядок членов
# прежние, а счётчики знака и статьи бюджета ведутся ровно там же, где
# раньше (фильтр, не решивший знак, возвращает `None` и НЕ трогает ни одного
# счётчика).
# --------------------------------------------------------------------------


def _integer_form(
    terms: tuple[tuple[int, Fraction], ...],
) -> tuple[int, list[tuple[int, int]]]:
    """`(L, [(m, a_m)])`, `c_m = a_m / L`, `L` — наименьший общий знаменатель."""

    common = 1
    for _, coefficient in terms:
        denominator = coefficient.denominator
        if denominator != 1:
            common = lcm(common, denominator)
    if common == 1:
        return 1, [
            (radicand, coefficient.numerator) for radicand, coefficient in terms
        ]
    return common, [
        (radicand, coefficient.numerator * (common // coefficient.denominator))
        for radicand, coefficient in terms
    ]


def _integer_enclosure(
    items: list[tuple[int, int]], bits: int
) -> tuple[int, int]:
    """Границы `2^bits * sum a_m*sqrt(m)` на целых: те же `isqrt`, что у `enclosure`."""

    scale = 1 << bits
    shift = 2 * bits
    low = high = 0
    for radicand, numerator in items:
        if radicand == 1:
            exact = numerator * scale
            low += exact
            high += exact
            continue
        floor_root = isqrt(radicand << shift)
        if numerator > 0:
            low += numerator * floor_root
            high += numerator * (floor_root + 1)
        else:
            low += numerator * (floor_root + 1)
            high += numerator * floor_root
    return low, high


def _integer_certified_sign(
    items: list[tuple[int, int]], bits: int
) -> int | None:
    low, high = _integer_enclosure(items, bits)
    if low > 0:
        return 1
    if high < 0:
        return -1
    return None


def _scaled_difference_items(
    plus: "SqrtSumV1",
    plus_factor: Fraction,
    minus: "SqrtSumV1",
    minus_factor: Fraction,
) -> list[tuple[int, int]]:
    """Ненулевые `(m, a_m)` разности `plus*plus_factor - minus*minus_factor`."""

    return _scaled_difference_parts(plus, plus_factor, minus, minus_factor)[1]


def _scaled_difference_parts(
    plus: "SqrtSumV1",
    plus_factor: Fraction,
    minus: "SqrtSumV1",
    minus_factor: Fraction,
) -> tuple[int, list[tuple[int, int]]]:
    """`(D, [(m, a_m)])`: `plus*plus_factor - minus*minus_factor = sum a_m*sqrt(m) / D`.

    `D` положительно, поэтому знак и ноль читаются из целых `a_m` без единого
    `Fraction`; нулевые `a_m` отброшены. Слияние радикандов повторяет
    `scaled(...) - scaled(...)`: слагаемые `plus` перекрывают друг друга, как в
    `as_map`, слагаемые `minus` складываются. Множители бывают `int`.
    """

    plus_common, plus_items = _integer_form(plus.terms)
    minus_common, minus_items = _integer_form(minus.terms)
    plus_scale = plus_common * plus_factor.denominator
    minus_scale = minus_common * minus_factor.denominator
    big = lcm(plus_scale, minus_scale)
    plus_multiplier = plus_factor.numerator * (big // plus_scale)
    minus_multiplier = minus_factor.numerator * (big // minus_scale)
    merged: dict[int, int] = {}
    for radicand, numerator in plus_items:
        merged[radicand] = numerator * plus_multiplier
    for radicand, numerator in minus_items:
        merged[radicand] = merged.get(radicand, 0) - numerator * minus_multiplier
    return big, [(radicand, value) for radicand, value in merged.items() if value]


def _multiply_integer_items(
    left: list[tuple[int, int]], right: list[tuple[int, int]]
) -> list[tuple[int, int]]:
    """Произведение двух `sum a_m*sqrt(m)` на целых; радиканды по возрастанию.

    `sqrt(a)*sqrt(b) = g*sqrt(a*b/g^2)`, `g = gcd(a, b)`. Слияние по радиканду
    и отброс нулей — те же, что у `SqrtSumV1.__mul__`; общий знаменатель
    произведения вызывающий собирает сам.
    """

    merged: dict[int, int] = {}
    for left_radicand, left_numerator in left:
        for right_radicand, right_numerator in right:
            common = gcd(left_radicand, right_radicand)
            radicand = (left_radicand // common) * (right_radicand // common)
            merged[radicand] = (
                merged.get(radicand, 0)
                + left_numerator * right_numerator * common
            )
    return sorted(
        (radicand, value) for radicand, value in merged.items() if value
    )


def _reduced_form(
    common: int, items: list[tuple[int, int]]
) -> tuple[int, list[tuple[int, int]]]:
    """Тот же набор `(L, a_m)`, сокращённый на общий делитель; значение то же."""

    divisor = gcd(common, *[value for _, value in items])
    if divisor == 1:
        return common, items
    return common // divisor, [(m, value // divisor) for m, value in items]


def _scaled_by_reciprocal(
    numerator_common: int,
    numerator_items: list[tuple[int, int]],
    denominator_common: int,
    denominator_items: list[tuple[int, int]],
) -> "SqrtSumV1":
    """`numerator / rational` для рационального знаменателя, по одной дроби."""

    rational = Fraction(
        denominator_items[0][1] if denominator_items else 0,
        denominator_common,
    )
    factor = Fraction(1) / rational
    return SqrtSumV1(
        tuple(
            (
                radicand,
                Fraction(
                    value * factor.numerator,
                    numerator_common * factor.denominator,
                ),
            )
            for radicand, value in numerator_items
        )
    )


# Одна ширина оболочки на `SqrtSumV1.sign` и `_filtered_sign`: разойдись они,
# фильтр решал бы не то, что решает `sign` до сопряжения, и счётчики
# `SIGN_COUNTS` разъехались бы (ответ — нет: фолбэк точный).
SIGN_FILTER_BITS = 64


def _filtered_sign(
    items: list[tuple[int, int]], filter_bits: int = SIGN_FILTER_BITS
) -> int | None:
    """Знак `sum a_m*sqrt(m)`, если его решает то, что `sign` решает ДО сопряжения.

    Счётчики `SIGN_COUNTS` ведутся как у `SqrtSumV1.sign`. `None` — фильтр не
    доказал знак: счётчики не тронуты, и вызывающий идёт исходным путём, у
    которого и счёт, и бюджет прежние.
    """

    if not items:
        SIGN_COUNTS["total"] += 1
        SIGN_COUNTS["closed_rational_zero"] += 1
        return 0
    if len(items) == 1 and items[0][0] == 1:
        SIGN_COUNTS["total"] += 1
        SIGN_COUNTS["closed_rational_nonzero"] += 1
        numerator = items[0][1]
        return (numerator > 0) - (numerator < 0)
    certified = _integer_certified_sign(items, filter_bits)
    if certified is not None:
        SIGN_COUNTS["total"] += 1
        SIGN_COUNTS["closed_by_enclosure"] += 1
    return certified


# --------------------------------------------------------------------------
# Сама величина
# --------------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class SqrtSumV1:
    """`sum c_m * sqrt(m)`, m бесквадратные и различные, c_m != 0.

    Хранится отсортированным кортежем, поэтому величина хешируема и попадает в
    дайджест побитово воспроизводимо. `Fraction`, а не float — по той же
    причине, по которой невязка привязки хранится дробью.
    """

    terms: tuple[tuple[int, Fraction], ...]

    # ---- построение -----------------------------------------------------

    @staticmethod
    def zero() -> "SqrtSumV1":
        return SqrtSumV1(())

    @staticmethod
    def rational(value: Fraction | int) -> "SqrtSumV1":
        value = Fraction(value)
        return SqrtSumV1(((1, value),)) if value else SqrtSumV1(())

    @staticmethod
    def radical(
        coefficient: Fraction | int,
        radicand: Fraction | int,
        budget: "ExactWorkBudgetV1 | None" = None,
    ) -> "SqrtSumV1":
        """`coefficient * sqrt(radicand)`, приведённое к канонической форме.

        Радиканд бывает РАЦИОНАЛЬНЫМ, и это не послабление типа, а требование
        входа. Квадрат скорости фронта `q` равен `(s/|n|)^2 * |d|^2`, и у
        полевого патча `(s/|n|)^2` — дробь со знаменателем 844687660141 либо
        1439659412197 (оба БЕСКВАДРАТНЫ), поэтому целым `q` может стать лишь
        при ребре длиной в сам знаменатель узлов решётки. Доступно 942 195 при
        самом мелком допустимом шаге окна — меньше в 896 510 раз. Целочисленный
        радиканд означал бы «полевой вход не отображается», а не «типы строже».

        Приведение точное и однострочное: `sqrt(p/r) = (1/r)*sqrt(p*r)`, потому
        что `p/r = p*r/r^2` и `r > 0`. Знаменатель уезжает в коэффициент, где
        `Fraction` его держит и так, а под корнем остаётся целое, и всё, что
        ниже — бесквадратное разложение, сопряжение, оболочка — работает без
        единой правки.

        На ЦЕЛОМ радиканде ветка не выполняется вовсе, поэтому прежний путь
        остаётся побитово тем же: `Fraction(4).denominator == 1`, и `int(4)`
        возвращает то же самое `4`, которое сюда приходило раньше. Ровно этим
        сохраняется неподвижность `FROZEN_DIGESTS`.

        `budget` — бюджет транзакции. Он нужен здесь потому, что НОВЫЙ радиканд
        приходит в ядро именно отсюда: `squarefree_split` на промахе памяти
        уходит в факторизацию, и её цена определяется наибольшим простым
        делителем, то есть входом, а не кодом. `None` — прогон без названного
        бюджета; работа тогда идёт в телеметрию неоплаченного, и полевые
        ворота требуют, чтобы там был ноль.
        """

        coefficient = Fraction(coefficient)
        if coefficient == 0 or radicand == 0:
            return SqrtSumV1(())
        if isinstance(radicand, Fraction) and radicand.denominator != 1:
            coefficient /= radicand.denominator
            radicand = radicand.numerator * radicand.denominator
        else:
            radicand = int(radicand)
        outside, inside = squarefree_split(radicand, budget)
        return SqrtSumV1(((inside, coefficient * outside),))

    @staticmethod
    def _from_map(mapping: dict[int, Fraction]) -> "SqrtSumV1":
        return SqrtSumV1(
            tuple(sorted((m, c) for m, c in mapping.items() if c))
        )

    # ---- арифметика -----------------------------------------------------

    def as_map(self) -> dict[int, Fraction]:
        return dict(self.terms)

    def __add__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        merged = self.as_map()
        for radicand, coefficient in other.terms:
            old = merged.get(radicand)
            if old is not None:
                merged[radicand] = old + coefficient
            elif type(coefficient) is Fraction:
                merged[radicand] = coefficient
            else:
                merged[radicand] = Fraction(0) + coefficient
        return SqrtSumV1._from_map(merged)

    def __neg__(self) -> "SqrtSumV1":
        return SqrtSumV1(tuple((m, -c) for m, c in self.terms))

    def __sub__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        """`self + (-other)` без промежуточной величины и без `Fraction(0)`."""

        merged = self.as_map()
        for radicand, coefficient in other.terms:
            old = merged.get(radicand)
            if old is not None:
                merged[radicand] = old - coefficient
                continue
            negated = -coefficient
            if type(negated) is not Fraction:
                negated = Fraction(0) + negated
            merged[radicand] = negated
        return SqrtSumV1._from_map(merged)

    def scaled(self, factor: Fraction | int) -> "SqrtSumV1":
        factor = Fraction(factor)
        if factor == 0:
            return SqrtSumV1(())
        return SqrtSumV1(tuple((m, c * factor) for m, c in self.terms))

    def scaled_difference(
        self,
        factor: Fraction | int,
        other: "SqrtSumV1",
        other_factor: Fraction | int,
    ) -> "SqrtSumV1":
        """`self*factor - other*other_factor` одним проходом.

        Те же члены, тот же тип коэффициента и тот же порядок, что у
        `self.scaled(factor) - other.scaled(other_factor)`, без двух
        промежуточных величин и без нормировки дроби на каждом шаге.
        """

        big, items = _scaled_difference_parts(
            self, factor, other, other_factor
        )
        return SqrtSumV1(
            tuple(
                (radicand, Fraction(value, big))
                for radicand, value in sorted(items)
            )
        )

    def difference_sign(
        self,
        other: "SqrtSumV1",
        budget: "ExactWorkBudgetV1 | None" = None,
    ) -> int:
        """Знак `self - other`, как у `(self - other).sign(budget=budget)`.

        Фильтр читает знак из целых без промежуточной величины и ведёт счётчики
        `SIGN_COUNTS` ровно как `sign`; не решив, он их не трогает, и вопрос
        идёт прежним путём с тем же сопряжением и бюджетом.
        """

        decided = _filtered_sign(_scaled_difference_parts(self, 1, other, 1)[1])
        if decided is not None:
            return decided
        return (self - other).sign(budget=budget)

    def difference_is_zero(self, other: "SqrtSumV1") -> bool:
        """`(self - other).is_zero` без самой разности."""

        return not _scaled_difference_parts(self, 1, other, 1)[1]

    def __mul__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        """Произведение. `sqrt(a)*sqrt(b) = g*sqrt(a*b/g^2)`, g = gcd(a, b).

        Обе величины бесквадратны, поэтому `a*b/g^2` бесквадратно тоже —
        замыкание не требует ни одной новой факторизации.
        """

        if not self.terms or not other.terms:
            return SqrtSumV1(())
        left_common, left_items = _integer_form(self.terms)
        right_common, right_items = _integer_form(other.terms)
        denominator = left_common * right_common
        return SqrtSumV1(
            tuple(
                (radicand, Fraction(numerator, denominator))
                for radicand, numerator in _multiply_integer_items(
                    left_items, right_items
                )
            )
        )

    def __truediv__(self, other: "SqrtSumV1") -> "SqrtSumV1":
        """Деление без названного бюджета. Оператор бюджета принять не может.

        Питоновский оператор не берёт третьего аргумента, поэтому именованный
        путь называется `divided_by`. Оставлять оператор было обязательно: он
        стоит в модулях, которые эта карточка трогать не вправе, и его удаление
        стало бы правкой чужой зоны ради формы.
        """

        return self.divided_by(other, None)

    def divided_by(
        self, other: "SqrtSumV1", budget: "ExactWorkBudgetV1 | None"
    ) -> "SqrtSumV1":
        """Деление домножением на сопряжённые. Результат — канонический.

        Каждое сопряжение по простому p убирает p из носителя знаменателя,
        поэтому шагов ровно столько, сколько различных простых, и в конце
        знаменатель рационален. Приближения здесь нет ни на одном шаге.

        Число сопряжений конечно и без бюджета; бюджет здесь нужен не циклу, а
        `_pick_prime`, который спрашивает носитель ПРОИЗВОДНОГО радикала и тем
        самым способен запустить ро-Полларда на числе, которого никто не видел.
        Ровно этим путём полевой прогон стоял 900 с (DECISIONS, 2026-08-04).
        """

        if other.is_zero:
            raise ZeroSqrtSumDivisorError("деление на точный ноль")
        numerator, denominator = self, other
        while True:
            rational = denominator.as_rational()
            if rational is not None:
                return numerator.scaled(Fraction(1) / rational)
            prime = _pick_prime(denominator.as_map(), budget)
            outside, inside = _split_by_prime(denominator.as_map(), prime)
            root = SqrtSumV1.radical(1, prime, budget)
            conjugate = SqrtSumV1._from_map(outside) - (
                SqrtSumV1._from_map(inside) * root
            )
            numerator = numerator * conjugate
            denominator = denominator * conjugate

    # ---- решения --------------------------------------------------------

    @property
    def is_zero(self) -> bool:
        """Точный ноль. Рациональная проверка: коэффициентов просто нет."""

        return not self.terms

    def is_rational(self) -> bool:
        return all(radicand == 1 for radicand, _ in self.terms)

    def as_rational(self) -> Fraction | None:
        if not self.is_rational():
            return None
        return dict(self.terms).get(1, Fraction(0))

    def enclosure(self, bits: int) -> tuple[Fraction, Fraction]:
        """Строгая оболочка на целых: `[lo, hi]`, lo <= value <= hi.

        `sqrt(m)` заключается между `isqrt(m<<2b)/2^b` и следующим узлом. Это
        фильтр, а не решатель: он либо доказывает знак, либо уступает точному
        пути ниже. Порога в нём нет — есть граница, вычисленная точно.
        """

        common, items = _integer_form(self.terms)
        low, high = _integer_enclosure(items, bits)
        denominator = common << bits
        return Fraction(low, denominator), Fraction(high, denominator)

    def certified_sign(self, bits: int) -> int | None:
        """Знак, доказанный оболочкой, либо None. Тождества не доказывает."""

        return _integer_certified_sign(_integer_form(self.terms)[1], bits)

    def sign(
        self,
        *,
        filter_bits: int = SIGN_FILTER_BITS,
        budget: "ExactWorkBudgetV1 | None" = None,
    ) -> int:
        """Точный знак. Оболочка — фильтр, сопряжение — решатель.

        Отката нет и быть не может: ветвь сопряжения конечна и целочисленна,
        поэтому «не смог» здесь не существует как исход.
        """

        SIGN_COUNTS["total"] += 1
        if not self.terms:
            SIGN_COUNTS["closed_rational_zero"] += 1
            return 0
        if len(self.terms) == 1 and self.terms[0][0] == 1:
            SIGN_COUNTS["closed_rational_nonzero"] += 1
            coefficient = self.terms[0][1]
            return (coefficient > 0) - (coefficient < 0)
        certified = self.certified_sign(filter_bits)
        if certified is not None:
            SIGN_COUNTS["closed_by_enclosure"] += 1
            return certified
        SIGN_COUNTS["closed_by_conjugation"] += 1
        return _exact_sign(self.as_map(), filter_bits, budget)


# --------------------------------------------------------------------------
# Исполняемая каноника
#
# Равенство времён и величин читается из ПУСТОТЫ набора коэффициентов разности
# (`times_are_equal`, `difference_is_zero`), и это верно только на канонической
# форме: `sqrt(8)` против `2*sqrt(2)` — равные величины с непустой разностью, а
# знак (`compare_times == 0`) назвал бы их равными. Каноника держалась
# дисциплиной конструкторов; здесь она исполняется на входе в машину времён
# (`EventTimeV1`, `from_algebraic_sum`), а не лежит слухом.
#
# Два слоя, граница между ними названа. ВСЕГДА ВКЛЮЧЁН дешёвый слой, O(члены):
# радиканды — целые, строго возрастают от единицы; коэффициенты — ненулевые
# рациональные; радиканд не делится на 4, 9, 25, 49 и не полный квадрат.
# Он ловит неотсортированное, повторённое, нулевое и частые неканонические
# радиканды, но НЕ полноту бесквадратности. Полноту даёт аудит
# (`set_canonical_audit`): каждый радиканд раскладывается независимо от памяти
# канонизации. Набор тестов ядра включает аудит (`kernel/tests/conftest.py`);
# в продукте он выключен — разложение на горячем пути и есть цена, которой
# каноника нас избавляет.
# --------------------------------------------------------------------------


class NonCanonicalSqrtSumError(ValueError):
    """Сумма корней вне канонической формы. Именованный отказ, не фолбэк."""

    def __init__(self, where: str, reason: str) -> None:
        self.where = where
        self.reason = reason
        super().__init__(f"NON_CANONICAL_SQRT_SUM:{where}: {reason}")


_CANONICAL_AUDIT = False
_AUDITED_SQUAREFREE: set[int] = set()


def set_canonical_audit(enabled: bool) -> bool:
    """Включить полную проверку бесквадратности; вернуть прежнее состояние."""

    global _CANONICAL_AUDIT
    previous = _CANONICAL_AUDIT
    _CANONICAL_AUDIT = bool(enabled)
    return previous


def canonical_audit_enabled() -> bool:
    return _CANONICAL_AUDIT


def _audit_squarefree(radicand: int) -> bool:
    """Бесквадратность целого без записи в память канонизации и без бюджета.

    Простые из реестра снимаются делением (только чтение), остаток раскладывается
    `_rho_factors` на одноразовом бюджете: ни память разложений, ни телеметрия
    неоплаченной работы, ни статьи чужого бюджета не двигаются, поэтому аудит
    не меняет цену и счётчики тестов, у которых они закреплены.
    """

    if radicand in _AUDITED_SQUAREFREE:
        return True
    factors, remainder = _strip_known_primes(radicand)
    if any(power > 1 for power in factors.values()):
        return False
    if remainder > 1:
        scratch = unlimited_reference_budget(stage="CANONICAL_AUDIT")
        if any(
            power > 1 for power in _rho_factors(remainder, scratch).values()
        ):
            return False
    _AUDITED_SQUAREFREE.add(radicand)
    return True


def require_canonical(value: SqrtSumV1, where: str) -> SqrtSumV1:
    """Величина — в канонической форме, иначе `NonCanonicalSqrtSumError`.

    Вызывается там, где величина ВХОДИТ в машину времён, а не в арифметике:
    арифметика замкнута по канонике (произведение бесквадратных по теореме, а
    `radical` идёт через `squarefree_split`), поэтому проверять каждый
    промежуточный результат — платить за то, что уже доказано.
    """

    previous = 0
    for radicand, coefficient in value.terms:
        if type(radicand) is not int or radicand <= previous:
            raise NonCanonicalSqrtSumError(
                where,
                f"радиканд {radicand!r} после {previous}: нужны целые, строго "
                "возрастающие от единицы",
            )
        if not coefficient or (
            type(coefficient) is not Fraction and type(coefficient) is not int
        ):
            raise NonCanonicalSqrtSumError(
                where,
                f"коэффициент {coefficient!r} при радиканде {radicand}: нужно "
                "ненулевое рациональное",
            )
        if not (radicand & 3 and radicand % 9 and radicand % 25 and radicand % 49):
            raise NonCanonicalSqrtSumError(
                where, f"радиканд {radicand} делится на квадрат малого простого"
            )
        if radicand > 1:
            root = isqrt(radicand)
            if root * root == radicand:
                raise NonCanonicalSqrtSumError(
                    where, f"радиканд {radicand} — полный квадрат"
                )
        if _CANONICAL_AUDIT and not _audit_squarefree(radicand):
            raise NonCanonicalSqrtSumError(
                where, f"радиканд {radicand} не бесквадратный"
            )
        previous = radicand
    return value


def _divide_with_prime_universe(
    numerator: SqrtSumV1,
    denominator: SqrtSumV1,
    prime_universe: tuple[int, ...],
    budget: ExactWorkBudgetV1 | None = None,
) -> SqrtSumV1:
    """Точное деление с локально доказанным носителем примитивных `q`.

    Сопряжения и их порядок совпадают с `__truediv__`. Отличается только
    источник минимального простого: делимость по конечному universe вместо
    факторизации каждого производного радикала. Если хотя бы один радикал
    целиком не восстановлен, прежний путь запускается заново на ИСХОДНЫХ
    операндах — частично сопряжённые величины никогда не смешиваются с legacy.
    """

    if denominator.is_zero:
        return numerator.divided_by(denominator, budget)
    numerator_common, numerator_items = _integer_form(numerator.terms)
    denominator_common, denominator_items = _integer_form(denominator.terms)
    while True:
        if len(denominator_items) <= 1 and all(
            radicand == 1 for radicand, _ in denominator_items
        ):
            return _scaled_by_reciprocal(
                numerator_common,
                numerator_items,
                denominator_common,
                denominator_items,
            )
        prime = _pick_prime_from_universe(
            dict(denominator_items),
            prime_universe,
        )
        # Корень `sqrt(prime)` материализуется здесь и только здесь: промах
        # памяти `squarefree_split` платит бюджет, как платил `radical(1, p)`.
        if prime is None or squarefree_split(prime, budget) != (1, prime):
            return numerator.divided_by(denominator, budget)
        # `E = A + B*sqrt(p)`, сопряжённое `A - B*sqrt(p)`: члены, делящиеся на
        # p, меняют знак, а радиканды остаются прежними (`sqrt(k)*sqrt(p)` при
        # `p` не делящем `k` даёт тот же `k*p`).
        conjugate = [
            (radicand, -value if radicand % prime == 0 else value)
            for radicand, value in denominator_items
        ]
        numerator_items = _multiply_integer_items(numerator_items, conjugate)
        denominator_items = _multiply_integer_items(
            denominator_items, conjugate
        )
        numerator_common *= denominator_common
        denominator_common *= denominator_common
        numerator_common, numerator_items = _reduced_form(
            numerator_common, numerator_items
        )
        denominator_common, denominator_items = _reduced_form(
            denominator_common, denominator_items
        )


def radical_sum(
    parts: "tuple[tuple[Fraction | int, Fraction | int], ...]",
    budget: "ExactWorkBudgetV1 | None" = None,
) -> "SqrtSumV1":
    """`sum coefficient_i * sqrt(radicand_i)` — то же, что цепочка `radical(...) + ...`.

    Слагаемые идут в порядке `parts`, `squarefree_split` зовётся на тех же
    (ненулевых) радикандах в том же порядке, поэтому статьи бюджета и промахи
    памяти те же, что у цепочки. Сумма копится в целых там, где коэффициент и
    радиканд целые; `Fraction` создаётся один раз на член результата, а нулевые
    суммы отбрасываются, как отбрасывает их `_from_map`.
    """

    merged: dict[int, tuple[int, int]] = {}
    for coefficient, radicand in parts:
        if type(coefficient) is int:
            numerator, denominator = coefficient, 1
        else:
            coefficient = Fraction(coefficient)
            numerator, denominator = coefficient.numerator, coefficient.denominator
        if numerator == 0 or radicand == 0:
            continue
        if isinstance(radicand, Fraction) and radicand.denominator != 1:
            denominator *= radicand.denominator
            radicand = radicand.numerator * radicand.denominator
        else:
            radicand = int(radicand)
        outside, inside = squarefree_split(radicand, budget)
        numerator *= outside
        old = merged.get(inside)
        if old is not None:
            numerator = numerator * old[1] + old[0] * denominator
            denominator *= old[1]
        merged[inside] = (numerator, denominator)
    return SqrtSumV1(
        tuple(
            (radicand, Fraction(numerator, denominator))
            for radicand, (numerator, denominator) in sorted(merged.items())
            if numerator
        )
    )


def _split_by_prime(
    terms: dict[int, Fraction], prime: int
) -> tuple[dict[int, Fraction], dict[int, Fraction]]:
    """`E = A + B*sqrt(p)`. Носитель A и B уже без p."""

    outside: dict[int, Fraction] = {}
    inside: dict[int, Fraction] = {}
    for radicand, coefficient in terms.items():
        if radicand % prime == 0:
            key = radicand // prime
            inside[key] = inside.get(key, Fraction(0)) + coefficient
        else:
            outside[radicand] = outside.get(radicand, Fraction(0)) + coefficient
    return outside, inside


def _pick_prime(
    terms: dict[int, Fraction], budget: ExactWorkBudgetV1 | None = None
) -> int | None:
    """Наименьшее простое носителя. Детерминированно, значит воспроизводимо."""

    primes: set[int] = set()
    for radicand, coefficient in terms.items():
        if coefficient and radicand > 1:
            primes.update(prime_support(radicand, budget))
    return min(primes) if primes else None


def _exact_sign(
    terms: dict[int, Fraction],
    filter_bits: int,
    budget: ExactWorkBudgetV1 | None = None,
) -> int:
    """Знак рекурсией по простым носителя. Целые числа, конечное число шагов.

    Разбиение `E = A + B*sqrt(p)` убирает p из носителя обеих частей, поэтому
    глубина рекурсии равна числу различных простых и рекурсия конечна.
    При разных знаках A и B решает `A^2 - p*B^2`: он равен нулю только если
    `sqrt(p)` лежит в меньшем поле, чего не бывает, — поэтому ноль здесь
    означает `B = 0`, и эта ветка отработана выше.
    """

    terms = {m: c for m, c in terms.items() if c}
    if not terms:
        return 0
    prime = _pick_prime(terms, budget)
    if prime is None:
        value = terms[1]
        return (value > 0) - (value < 0)
    certified = SqrtSumV1._from_map(terms).certified_sign(filter_bits)
    if certified is not None:
        return certified

    outside, inside = _split_by_prime(terms, prime)
    outside_sign = _exact_sign(outside, filter_bits, budget)
    inside_sign = _exact_sign(inside, filter_bits, budget)
    if inside_sign == 0:
        return outside_sign
    if outside_sign == 0:
        return inside_sign
    if outside_sign == inside_sign:
        return outside_sign

    left = SqrtSumV1._from_map(outside)
    right = SqrtSumV1._from_map(inside)
    discriminant = left * left - (right * right).scaled(prime)
    discriminant_sign = _exact_sign(discriminant.as_map(), filter_bits, budget)
    if discriminant_sign == 0:
        # Доказуемо недостижимо, но исход назван, а не пропущен молча.
        return 0
    return outside_sign if discriminant_sign > 0 else inside_sign
