"""Запрет на точную работу мимо бюджета — устройством кода, а не измерением.

ЗАЧЕМ ЭТОТ ФАЙЛ СУЩЕСТВУЕТ. Кап работы (`test_exact_work_budget.py`) обещает,
что любой долгий расчёт кончится ИМЕНОВАННЫМ отказом, а не счётом. Обещание
верно ровно настолько, насколько работа проходит ЧЕРЕЗ бюджет. До этой карточки
полнота протяжки держалась на измерении: полевой инструмент
`artifacts/exact_work_budget/unbudgeted_sites.py` ходил по трём доменам и
считал вызовы `SqrtSumV1.sign()` без бюджета.

Измерение защищает только пройденные пути, и это доказано числом, а не
опасением: между двумя вершинами одной ветки дыра выросла с 9 мест / 16 801
вызова до 12 мест / 88 042 вызовов — от правки, БЮДЖЕТА ВООБЩЕ НЕ КАСАВШЕЙСЯ.
Пятикратный рост произошёл молча, потому что ни одна проверка не спрашивала
«появилось ли НОВОЕ место», — спрашивали только «сколько их сегодня на трёх
доменах».

Здесь запрет ТОТАЛЬНЫЙ и статический: разбирается ИСХОДНИК всего ядра, и любой
вызов дорогой точной операции обязан назвать бюджет. Место, добавленное будущей
правкой, падает в тот же миг, когда оно написано, — до всякого прогона по полю
и независимо от того, ходит ли туда хоть один тест. Рядом стоит динамический
сторож: он ловит то, чего статический разбор увидеть не может в принципе —
вызов через оператор, через алиас, через `cmp_to_key`.

СОСТАВ ДОРОГОЙ ПОВЕРХНОСТИ обоснован в `CHARGED_EXACT_SURFACE` ниже: туда
входит ровно то, что способно дойти до факторизации (ро-Поллард, Миллер—Рабин)
или до гидратации точной позиции, то есть до статей, которые бюджет считает.
"""

from __future__ import annotations

import ast
import sys
from fractions import Fraction
from pathlib import Path

import pytest

from cftuv_envelope import exact_sqrt_sum as canon
from cftuv_envelope.exact_sqrt_sum import (
    SqrtSumV1,
    UNBUDGETED_WORK,
    exact_work_budget,
    reset_factorization_memory,
    reset_unbudgeted_work,
)
from cftuv_envelope.wavefront.coverage import coverage_at
from cftuv_envelope.wavefront.faces import build_faces
from cftuv_envelope.wavefront.polygon import PolygonV1
from cftuv_envelope.wavefront.skeleton import SkeletonOutcome, build_skeleton

# Правило запрета живёт отдельным модулем: тем же пользуется хостовая сюита
# (`tests/test_architecture.py`), которой ядро импортировать нельзя.
from exact_work_budget_ban import (
    CHARGED_EXACT_SURFACE,
    EXEMPTION_REASONS,
    _grouped,
    scan_tree,
    unbudgeted_sites_in_source,
)
from test_wavefront_motorcycle_graph import CORPUS


KERNEL_SOURCE_ROOT = Path(__file__).resolve().parents[1] / "src"


# (модуль относительно kernel/src, имя вызова) -> (число мест, причина).
#
# Ключ НЕ содержит номера строки намеренно: обычная правка сдвигает строки и
# ломала бы список каждым коммитом, а его задача — ловить ПОЯВЛЕНИЕ места, а не
# его переезд. Число заморожено: вырасти оно не может (тест ниже), уменьшиться
# может — это и есть закрытие дыры.
FROZEN_UNBUDGETED_SITES: dict[tuple[str, str], tuple[int, str]] = {
    ("cftuv_envelope/exact_sqrt_sum.py", "divided_by"): (
        1,
        "OPERATOR_HAS_NO_THIRD_ARGUMENT",
    ),
    ("cftuv_envelope/reference/adaptive_density_atlas.py", "sign"): (
        9,
        "RECEIVER_IS_NOT_A_SQRT_SUM",
    ),
    ("cftuv_envelope/reference/direction_binding.py", "radical"): (
        2,
        "RECEIVER_IS_NOT_A_SQRT_SUM",
    ),
    ("cftuv_envelope/reference/direction_binding.py", "sign"): (
        5,
        "RECEIVER_IS_NOT_A_SQRT_SUM",
    ),
    ("cftuv_envelope/reference/direction_window_exact.py", "sign"): (
        4,
        "RECEIVER_IS_NOT_A_SQRT_SUM",
    ),
    ("cftuv_envelope/reference/angular.py", "sign"): (
        1,
        "REFERENCE_ARRANGEMENT_NOT_THREADED_YET",
    ),
    ("cftuv_envelope/reference/arrangement.py", "sign"): (
        13,
        "REFERENCE_ARRANGEMENT_NOT_THREADED_YET",
    ),
    ("cftuv_envelope/reference/planar_types.py", "radical"): (
        1,
        "REFERENCE_ARRANGEMENT_NOT_THREADED_YET",
    ),
}


# --------------------------------------------------------------------------
# 4. Ворота: ни одного места вне бюджета, кроме замороженных
# --------------------------------------------------------------------------


def test_no_charged_exact_call_escapes_a_named_budget():
    """Главные ворота карточки: новое место падает здесь, а не в поле.

    Именно эта проверка отличает структурный запрет от измерения. Ей не нужен
    ни полевой слепок, ни путь исполнения: она читает ИСХОДНИК целиком, и
    место, до которого сегодня не доходит ни один тест, для неё такое же
    место, как самое горячее.
    """

    grouped = _grouped(scan_tree(KERNEL_SOURCE_ROOT))
    unregistered = sorted(
        f"{module}:{sorted(lines)} -> {name}"
        for (module, name), lines in grouped.items()
        if (module, name) not in FROZEN_UNBUDGETED_SITES
    )
    assert not unregistered, (
        "точная работа мимо бюджета в незарегистрированном месте:\n"
        + "\n".join(unregistered)
        + "\n\nЛибо назовите бюджет в вызове, либо заведите причину в "
        "EXEMPTION_REASONS и место в FROZEN_UNBUDGETED_SITES."
    )


def test_the_frozen_exemption_list_does_not_grow():
    """Замороженное место не размножается: число мест не растёт ни на одно."""

    grouped = _grouped(scan_tree(KERNEL_SOURCE_ROOT))
    grown = sorted(
        f"{module} {name}: было {frozen}, стало {len(grouped[(module, name)])}"
        for (module, name), (frozen, _) in FROZEN_UNBUDGETED_SITES.items()
        if len(grouped.get((module, name), ())) > frozen
    )
    assert not grown, "\n".join(grown)


def test_every_name_of_the_charged_surface_exists_in_the_kernel():
    """Имя в поверхности обязано быть настоящим — иначе запрет пуст в этой строке.

    Список составлен вручную, и опечатка в нём не падает сама: она просто
    перестаёт что-либо запрещать, и молча. Проверка ищет каждое имя как
    `def`/атрибут в исходнике ядра; ни одно из них не должно исчезнуть при
    переименовании.
    """

    declared = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted(KERNEL_SOURCE_ROOT.rglob("*.py"))
    )
    missing = sorted(
        name
        for name in CHARGED_EXACT_SURFACE
        if f"def {name}(" not in declared
    )
    assert not missing, (
        "поверхность называет то, чего в ядре нет — запрет в этих строках "
        f"пуст: {missing}"
    )


def test_every_frozen_exemption_names_a_registered_reason():
    """Исключение без обоснования — не исключение, а забытое место."""

    for key, (_, reason) in FROZEN_UNBUDGETED_SITES.items():
        assert reason in EXEMPTION_REASONS, (key, reason)
        assert len(EXEMPTION_REASONS[reason]) > 200, reason


def test_no_frozen_exemption_is_dead():
    """Мёртвая запись врёт в сторону «дыра ещё есть». Убирайте вместе с местом."""

    grouped = _grouped(scan_tree(KERNEL_SOURCE_ROOT))
    dead = sorted(
        f"{module} {name}"
        for (module, name) in FROZEN_UNBUDGETED_SITES
        if not grouped.get((module, name))
    )
    assert not dead, "\n".join(dead)


# --------------------------------------------------------------------------
# 5. Нарушение СТРОИТСЯ — иначе запрет не проверяет ничего
# --------------------------------------------------------------------------

# Заведомо нарушающий фрагмент. Он существует ровно затем, чтобы показать, что
# запрет валит НОВОЕ место, а не переписывает сегодняшний список.
_A_NEW_SITE_A_FUTURE_EDIT_COULD_ADD = '''
from .event_time import compare_times


def a_reasonable_looking_helper(left, right, values):
    """Ровно то, что напишет следующая правка: точное сравнение без счёта."""

    if compare_times(left, right) < 0:
        return None
    return [value.sign() for value in values]
'''

_THE_SAME_SITE_WRITTEN_AS_IF_PAID = '''
from .event_time import compare_times


def a_reasonable_looking_helper(left, right, values, budget):
    if compare_times(left, right, budget) < 0:
        return None
    return [value.sign(budget=budget) for value in values]
'''

_A_SITE_THAT_PRETENDS_TO_BE_PAID = '''
def a_helper(values, budget):
    return [value.sign(budget=None) for value in values]
'''


def test_the_ban_catches_a_site_that_does_not_exist_yet():
    """Построенное нарушение: новое место валится обоими именами вызова."""

    found = unbudgeted_sites_in_source(
        _A_NEW_SITE_A_FUTURE_EDIT_COULD_ADD, "cftuv_envelope/wavefront/new.py"
    )
    assert {name for _, name, _ in found} == {"compare_times", "sign"}


def test_the_ban_lets_the_very_same_site_through_once_it_names_a_budget():
    """Тот же фрагмент, оплаченный, проходит — запрет не запрещает работу."""

    assert not unbudgeted_sites_in_source(
        _THE_SAME_SITE_WRITTEN_AS_IF_PAID, "cftuv_envelope/wavefront/new.py"
    )


def test_the_ban_is_not_fooled_by_an_explicit_none():
    """`budget=None` — это безбюджетный вызов, записанный как оплаченный."""

    found = unbudgeted_sites_in_source(
        _A_SITE_THAT_PRETENDS_TO_BE_PAID, "cftuv_envelope/wavefront/new.py"
    )
    assert [name for _, name, _ in found] == ["sign"]


def test_the_ban_catches_a_new_site_inside_a_frozen_module():
    """Замороженное место не становится пропуском для всего модуля.

    Проверяется на настоящем исходнике: к файлу, у которого исключение уже
    есть, дописывается ещё один такой же вызов, и число мест обязано вырасти
    за замороженное.
    """

    module = "cftuv_envelope/reference/arrangement.py"
    path = KERNEL_SOURCE_ROOT / module
    source = path.read_text(encoding="utf-8")
    frozen, _ = FROZEN_UNBUDGETED_SITES[(module, "sign")]

    before = unbudgeted_sites_in_source(source, module)
    after = unbudgeted_sites_in_source(
        source + "\n\ndef _added_by_a_future_edit(value):\n"
        "    return value.sign()\n",
        module,
    )
    assert len([1 for _, name, _ in before if name == "sign"]) == frozen
    assert len([1 for _, name, _ in after if name == "sign"]) == frozen + 1


# --------------------------------------------------------------------------
# 6. Динамический сторож: то, чего статический разбор не видит в принципе
# --------------------------------------------------------------------------


UNBUDGETED_EXACT_WORK = "UNBUDGETED_EXACT_WORK_OUTSIDE_A_NAMED_BUDGET"


class UnbudgetedExactWorkError(AssertionError):
    """Дорогая точная операция вызвана вне бюджетного контекста.

    Имя, а не голое падение: по правилу проекта (AGENTS.md п.4) исход
    называется, иначе его нельзя ни сгруппировать в разборе, ни отличить от
    любого другого `AssertionError` в логе прогона.
    """


class _Guard:
    """Перехват дорогой поверхности НА САМОМ ОБЪЕКТЕ, а не по имени в тексте.

    Зачем он нужен рядом со статическим запретом. Обход AST ловит вызов по
    имени и потому слеп ровно к трём вещам: к оператору (`a / b` — это
    `divided_by(other, None)`), к вызову через алиас или переменную, и к
    вызову из `cmp_to_key`-компаратора, собранного во время счёта. Сторож
    видит вызов, а не текст, поэтому все три для него одинаковы.

    Обратная сторона у него тоже одна и известна: он видит только пройденное.
    Поэтому он ДОПОЛНЯЕТ запрет, а не заменяет его.
    """

    def __init__(self, *, strict: bool = False) -> None:
        # `strict` — та самая форма из постановки: не собрать список, а
        # ПОДНЯТЬ ИМЕНОВАННУЮ ОШИБКУ на первом же вызове вне бюджета. Собирать
        # удобнее для отчёта (видны все места сразу), поднимать — честнее для
        # доказательства, что сторож действительно останавливает счёт.
        self.strict = strict
        self.violations: list[str] = []
        self._restore: list[tuple[object, str, object]] = []

    def _blame(self, operation: str, depth: int) -> None:
        frame = sys._getframe(depth + 1)
        where = f"{operation} @ {frame.f_code.co_filename}:{frame.f_lineno}"
        self.violations.append(where)
        if self.strict:
            raise UnbudgetedExactWorkError(
                f"{UNBUDGETED_EXACT_WORK} operation={operation} at={where}"
            )

    def __enter__(self) -> "_Guard":
        guard = self

        original_sign = canon.SqrtSumV1.sign

        def sign(self, *, filter_bits: int = 64, budget=None):
            if budget is None:
                guard._blame("sign", 1)
            return original_sign(self, filter_bits=filter_bits, budget=budget)

        original_radical = canon.SqrtSumV1.radical

        def radical(coefficient, radicand, budget=None):
            if budget is None:
                guard._blame("radical", 1)
            return original_radical(coefficient, radicand, budget)

        original_divided_by = canon.SqrtSumV1.divided_by

        def divided_by(self, other, budget):
            if budget is None:
                guard._blame("divided_by", 1)
            return original_divided_by(self, other, budget)

        original_truediv = canon.SqrtSumV1.__truediv__

        def truediv(self, other):
            guard._blame("truediv", 1)
            return original_truediv(self, other)

        self._restore = [
            (canon.SqrtSumV1, "sign", original_sign),
            (canon.SqrtSumV1, "radical", staticmethod(original_radical)),
            (canon.SqrtSumV1, "divided_by", original_divided_by),
            (canon.SqrtSumV1, "__truediv__", original_truediv),
        ]
        canon.SqrtSumV1.sign = sign
        canon.SqrtSumV1.radical = staticmethod(radical)
        canon.SqrtSumV1.divided_by = divided_by
        canon.SqrtSumV1.__truediv__ = truediv
        return self

    def __exit__(self, *exc) -> None:
        for owner, name, value in self._restore:
            setattr(owner, name, value)
        return None


def _run_one_domain(name: str):
    outer, holes = CORPUS[name]
    polygon = PolygonV1.build(outer, holes)
    budget = exact_work_budget(stage="GUARD", domain_id=name)
    skeleton = build_skeleton(polygon, work_budget=budget)
    if skeleton.outcome is SkeletonOutcome.EXACT:
        partition = build_faces(polygon, skeleton, budget)
        coverage = coverage_at(partition, Fraction(1), budget)
        # Обе объявленные границы спрашиваются здесь намеренно: они СВОЙСТВА,
        # и без явного вопроса их точные предикаты не выполнились бы вовсе.
        assert partition.every_contour_is_simple is not None
        assert partition.every_face_is_positive is not None
        assert coverage.does_not_exceed_the_polygon is not None
    return budget


@pytest.mark.parametrize("name", sorted(CORPUS))
def test_no_charged_call_of_a_transaction_runs_outside_its_budget(name: str):
    """Весь корпус скелетов под сторожем: ни одного вызова вне бюджета.

    Это тот же вопрос, что задаёт полевой инструмент карточки на трёх доменах,
    только заданный внутри сюиты и на всех фигурах корпуса — и заданный про
    ВЫЗОВЫ, а не про потраченные единицы. Место с нулём единиц сегодня платит
    завтра: единицы зависят от домена, а протяжка — нет.
    """

    reset_factorization_memory()
    reset_unbudgeted_work()
    with _Guard() as guard:
        budget = _run_one_domain(name)

    assert not guard.violations, "\n".join(sorted(set(guard.violations)))
    assert UNBUDGETED_WORK.spent == 0, UNBUDGETED_WORK.counters()
    assert budget.spent > 0


def test_the_dynamic_guard_itself_can_fail():
    """У сторожа обязано СТРОИТЬСЯ нарушение, иначе он не проверяет ничего.

    Три построенных нарушения покрывают три класса, ради которых сторож и
    заведён: прямой безбюджетный вызов, вызов через ОПЕРАТОР (статический
    разбор его не видит) и вызов из компаратора, собранного во время счёта.
    """

    left = SqrtSumV1.radical(1, 2, exact_work_budget(stage="T"))
    right = SqrtSumV1.radical(1, 3, exact_work_budget(stage="T"))
    with _Guard() as guard:
        (left - right).sign()
        left / right
        sorted([left, right], key=lambda value: value.sign())

    kinds = {violation.split(" @ ")[0] for violation in guard.violations}
    # `radical` попадает сюда следом за `divided_by`: безбюджетное деление
    # тянет за собой безбюджетную материализацию корня сопряжения. Это часть
    # ответа, а не шум, поэтому проверяется вхождение трёх классов, а не
    # равенство множеств — иначе тест запрещал бы сторожу видеть больше.
    assert {"sign", "truediv", "divided_by"} <= kinds


def test_the_truediv_exemption_is_static_only():
    """Оператор деления записан в исключениях — и на корпусе НЕ выполняется.

    Ровно этим исключение `OPERATOR_HAS_NO_THIRD_ARGUMENT` остаётся честным:
    оно про форму записи, а не про работу, которая идёт мимо счёта. Если
    будущая правка начнёт делить оператором внутри транзакции, этот тест
    назовёт место, и исключение придётся пересмотреть.
    """

    reset_factorization_memory()
    with _Guard() as guard:
        _run_one_domain("comb")

    assert not [
        violation
        for violation in guard.violations
        if violation.startswith("truediv")
    ]


def test_the_guard_raises_a_named_error_and_stops_the_count():
    """Строгий сторож не копит список, а ОСТАНАВЛИВАЕТ счёт именем.

    Форма из постановки карточки: вызов вне бюджета поднимает именованную
    ошибку. Доказано построенным нарушением — и тем, что имя стоит в тексте,
    а не выводится из типа исключения.
    """

    value = SqrtSumV1.radical(1, 2, exact_work_budget(stage="T")) - SqrtSumV1.radical(
        1, 3, exact_work_budget(stage="T")
    )
    with _Guard(strict=True):
        with pytest.raises(UnbudgetedExactWorkError) as raised:
            value.sign()

    assert UNBUDGETED_EXACT_WORK in str(raised.value)
    assert "operation=sign" in str(raised.value)


def test_the_strict_guard_lets_a_paid_transaction_finish():
    """И он же молчит на оплаченной транзакции — иначе запрещал бы работу."""

    reset_factorization_memory()
    with _Guard(strict=True) as guard:
        budget = _run_one_domain("comb")

    assert not guard.violations
    assert budget.spent > 0
