"""Реестр допусков под тестом плюс архитектурный запрет голых порогов.

Реестр, который никто не сверяет с кодом, — это md-файл с числами: он
расходится с ядром ровно так же, как разошлись прозаические инварианты
`AGENTS.md`. Поэтому здесь три группы проверок, и только вторая с третьей
делают реестр исполняемым:

1. ПОЛНОТА ЗАПИСИ. Каждое поле заполнено, `value` и `bound_law` взаимно
   исключают друг друга, фикстуры указывают на СУЩЕСТВУЮЩИЕ тесты (файл
   разбирается, имя ищется среди определений — строка «похожая на путь» не
   считается ссылкой).

2. ОТ КОДА К РЕЕСТРУ. Каждая константа «формы допуска» в `kernel/src`
   (float-литерал либо `Fraction`/`Decimal`/`Rational` от литералов) либо
   объявлена `declaration_sites` какой-то записи, либо стоит в замороженном
   списке исключений с причиной. Список append-only и его рост запрещён
   тестом: добавить исключение можно, но только вместе с числом в этом файле,
   то есть осознанно.

3. НИ ОДНОГО ГОЛОГО ПОРОГА. В сравнениях геометрических модулей `kernel/src`
   не бывает числового литерала «формы допуска» — ни `abs(x) < 1e-7`, ни
   `< 0.0001`. Исключения (сегодня их два, и оба — точная математическая
   половина, а не допуск) перечислены поимённо с обоснованием.

ПОЧЕМУ ПРОВЕРКА ИДЁТ ПО ДВУМ ДВЕРЯМ, а не по одной. Голый литерал прямо в
сравнении — самая заметная форма, но самая редкая: настоящий допуск обычно
получает имя и приезжает в сравнение локальной переменной, которую AST не
проследит. Поэтому вторая дверь стоит у ОБЪЯВЛЕНИЯ величины — там, где
владелец потребовал «одно значение и одно место». Вместе они закрывают обе
формы: безымянную и именованную.

ЧЕГО ЭТОТ ФАЙЛ НЕ ДЕЛАЕТ. Он не запрещает целочисленные литералы в
сравнениях: `len(x) > 0`, `q == 2`, `denominator <= 1` — структурные факты, а
не допуски, и запрет на них дал бы список исключений длиннее самого ядра. Он
не трогает производные границы фильтров (`filter_bits`, `iv.prec`, ширина
оболочки): они вычисляются на каждом вызове и в реестр не ходят по вердикту.
"""

from __future__ import annotations

import ast
from dataclasses import fields
from fractions import Fraction
from functools import cache
from pathlib import Path

import pytest

from cftuv_envelope.contracts.envelopes import AngleTolerancePolicyIdV1
from cftuv_envelope.contracts.metric import ExactRationalV1
from cftuv_envelope.contracts.tolerance_policy import (
    TOLERANCE_POLICIES_V1,
    TOLERANCE_POLICY_REGISTRY_LAW_V1,
    TOLERANCE_POLICY_REGISTRY_V1,
    TolerancePolicyAllowedEffectV1,
    TolerancePolicyCategoryV1,
    TolerancePolicyIdV1,
    TolerancePolicyUnitsV1,
    TolerancePolicyV1,
    tolerance_policies_in_category,
    tolerance_policy,
)


KERNEL_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = KERNEL_ROOT.parent
KERNEL_SOURCE = KERNEL_ROOT / "src"

CONSTANT_CONSTRUCTORS = frozenset({"Fraction", "Decimal", "Rational", "mpf"})

# Нейтральные элементы: `Fraction(0)` и `Fraction(1)` — инициализаторы
# накопителей и единицы умножения. Допуском не бывают по определению: сдвинуть
# их нельзя, не сломав арифметику.
NEUTRAL_VALUES = frozenset({Fraction(0), Fraction(1)})


# --------------------------------------------------------------------------
# Разбор исходников
# --------------------------------------------------------------------------


@cache
def _python_files() -> tuple[Path, ...]:
    return tuple(sorted(KERNEL_SOURCE.rglob("*.py")))


@cache
def _parse(path: Path) -> ast.Module:
    return ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def _relative(path: Path) -> str:
    return path.relative_to(REPO_ROOT).as_posix()


def _dotted(path: Path, name: str) -> str:
    module = path.relative_to(KERNEL_SOURCE).with_suffix("").as_posix()
    return f"{module.replace('/', '.')}.{name}"


@cache
def _integer_constants(path: Path) -> frozenset[str]:
    """Имена модуля, связанные с ЦЕЛЫМ литеральным выражением.

    Без них `Fraction(1, _SCALE)` притворилась бы производной величиной, хотя
    `_SCALE` — `10 ** CONE_ANGLE_DECIMALS`, то есть литерал, спрятанный за
    одним переименованием. Разбор идёт до неподвижной точки: имена этого
    уровня раскрываются в именах следующего.
    """

    names: set[str] = set()
    for _ in range(4):
        grown = False
        for node in _parse(path).body:
            targets = ()
            if isinstance(node, ast.Assign):
                targets = node.targets
            elif isinstance(node, ast.AnnAssign) and node.value is not None:
                targets = (node.target,)
            else:
                continue
            if not _is_literal(node.value, frozenset(names)):
                continue
            for target in targets:
                if isinstance(target, ast.Name) and target.id not in names:
                    names.add(target.id)
                    grown = True
        if not grown:
            break
    return frozenset(names)


def _is_literal(node: ast.AST, names: frozenset[str] = frozenset()) -> bool:
    """Выражение, целиком собранное из литералов: `10 ** 6`, `-1`, `1 << 13`."""

    if isinstance(node, ast.Constant):
        return isinstance(node.value, (int, float, str))
    if isinstance(node, ast.Name):
        return node.id in names
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
        return _is_literal(node.operand, names)
    if isinstance(node, ast.BinOp):
        return _is_literal(node.left, names) and _is_literal(node.right, names)
    return False


def _callee_name(node: ast.AST) -> str | None:
    if not isinstance(node, ast.Call):
        return None
    func = node.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _tolerance_shaped(
    node: ast.AST, names: frozenset[str] = frozenset()
) -> str | None:
    """Форма допуска: float-литерал либо точный конструктор от литералов.

    Целое само по себе формой допуска НЕ считается: `1 << 16` становится
    допуском (капом работы) только будучи именованным, и ловится второй
    дверью — по имени объявления, а не по литералу в выражении.
    """

    if isinstance(node, ast.Constant) and isinstance(node.value, float):
        return "float"
    if isinstance(node, ast.UnaryOp) and isinstance(node.op, ast.USub):
        return _tolerance_shaped(node.operand, names)
    name = _callee_name(node)
    if (
        name in CONSTANT_CONSTRUCTORS
        and isinstance(node, ast.Call)
        and node.args
        and all(_is_literal(argument, names) for argument in node.args)
    ):
        return name
    return None


def _literal_value(node: ast.AST) -> Fraction | None:
    """Числовое значение формы допуска, когда его удаётся вычислить точно."""

    try:
        materialised = ast.literal_eval(node)
    except (ValueError, SyntaxError, TypeError):
        if isinstance(node, ast.Call):
            try:
                arguments = [ast.literal_eval(item) for item in node.args]
            except (ValueError, SyntaxError, TypeError):
                return None
            try:
                return Fraction(*arguments)
            except (TypeError, ValueError, ZeroDivisionError):
                return None
        return None
    if isinstance(materialised, (int, float)):
        return Fraction(materialised)
    return None


# --------------------------------------------------------------------------
# 1. Полнота записи реестра
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "policy", TOLERANCE_POLICIES_V1, ids=lambda policy: policy.id.value
)
def test_every_policy_fills_every_field(policy: TolerancePolicyV1):
    """Пустое поле — это «не знаю», а «не знаю» не бывает у допуска в поле."""

    for field in fields(policy):
        value = getattr(policy, field.name)
        if field.name in ("value", "bound_law"):
            continue
        if isinstance(value, bool):
            continue
        assert value != "" and value is not None, (policy.id, field.name)
    assert len(policy.scope) > 40, policy.id
    assert len(policy.authority) > 20, policy.id


@pytest.mark.parametrize(
    "policy", TOLERANCE_POLICIES_V1, ids=lambda policy: policy.id.value
)
def test_a_policy_is_either_a_number_or_a_law_and_never_both(
    policy: TolerancePolicyV1,
):
    """Число и закон взаимно исключают друг друга.

    Запись с обоими объявила бы величину дважды — ровно ту болезнь, ради
    которой реестр заводился. Запись без обоих не объявляет ничего.
    """

    assert (policy.value is None) != (policy.bound_law is None), policy.id
    if policy.value is None:
        assert policy.units in (
            TolerancePolicyUnitsV1.NONE,
            TolerancePolicyUnitsV1.METRES,
        ), policy.id
    else:
        assert isinstance(policy.value, ExactRationalV1)
        assert policy.units is not TolerancePolicyUnitsV1.NONE, policy.id


@pytest.mark.parametrize(
    "policy", TOLERANCE_POLICIES_V1, ids=lambda policy: policy.id.value
)
def test_fixtures_point_at_tests_that_actually_exist(policy: TolerancePolicyV1):
    """Фикстура — ссылка на живой тест, а не строка, похожая на путь.

    Файл разбирается, имя ищется среди определений верхнего уровня. Опечатка
    и переименование теста ловятся здесь, а не на приёмке следующей карточки.
    """

    for reference in (policy.positive_fixture, policy.negative_fixture):
        assert "::" in reference, (policy.id, reference)
        relative, name = reference.split("::")
        path = REPO_ROOT / relative
        assert path.is_file(), (policy.id, reference)
        defined = {
            node.name
            for node in ast.walk(_parse(path))
            if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        }
        assert name in defined, (policy.id, reference)
    assert policy.positive_fixture != policy.negative_fixture, policy.id


def test_registry_is_an_enumerable_collection_without_duplicates():
    assert TOLERANCE_POLICY_REGISTRY_V1.policies == TOLERANCE_POLICIES_V1
    assert TOLERANCE_POLICY_REGISTRY_V1.registry_law == (
        TOLERANCE_POLICY_REGISTRY_LAW_V1
    )
    identifiers = [policy.id for policy in TOLERANCE_POLICIES_V1]
    assert len(identifiers) == len(set(identifiers))
    assert set(identifiers) == set(TolerancePolicyIdV1)
    for policy in TOLERANCE_POLICIES_V1:
        assert tolerance_policy(policy.id) is policy
    with pytest.raises(KeyError):
        tolerance_policy("PRODUCT_SKIRT_ABSOLUTE_V1")


def test_the_angle_policy_name_became_the_registry_key_as_promised():
    """`AngleTolerancePolicyIdV1` обещал стать ключом реестра. Обещание сверено.

    Совпадение имён здесь — не совпадение по памяти, а проверка: если реестр
    когда-нибудь переименует запись, сертификаты канонизации станут ссылаться
    на несуществующую политику, и это видно немедленно.
    """

    for member in AngleTolerancePolicyIdV1:
        assert member.value in {item.value for item in TolerancePolicyIdV1}
        policy = tolerance_policy(TolerancePolicyIdV1(member.value))
        assert policy.category is TolerancePolicyCategoryV1.AUTHORING_INTENT


# Категории вердикта, у которых сегодня нет ни одной записи, — с причиной.
# Список — заявление о ядре, а не забывчивость: широкая фаза `_embedding`
# считает пары треугольников точным целочисленным тестом БЕЗ запаса, а
# гистерезиса в v1 нет вовсе (`docs/decal_corner_bands.md`).
DECLARED_EMPTY_CATEGORIES = {
    TolerancePolicyCategoryV1.BROAD_PHASE_MARGIN: (
        "широкая фаза ядра работает без запаса: AABB пар треугольников и "
        "кандидатные пары аранжировки решаются точным целочисленным тестом"
    ),
    TolerancePolicyCategoryV1.UI_HYSTERESIS: (
        "гистерезиса в v1 нет: переключение band'а дискретно, на границе "
        "побеждает мягкий (docs/decal_corner_bands.md)"
    ),
}


def test_declared_empty_categories_are_empty_and_the_rest_are_not():
    """Пустая категория объявлена пустой, а не забыта.

    Ловит обе ошибки сразу: заведут запас широкой фазы — тест упадёт на
    непустой «объявленно пустой» категории; выкинут последнюю запись живой
    категории — упадёт на пустой необъявленной.
    """

    for category in TolerancePolicyCategoryV1:
        policies = tolerance_policies_in_category(category)
        if category in DECLARED_EMPTY_CATEGORIES:
            assert policies == (), (category, DECLARED_EMPTY_CATEGORIES[category])
        else:
            assert policies, category


def test_effects_that_change_the_answer_carry_a_recorded_outcome():
    """Допуск, меняющий ответ, обязан оставлять именованную запись.

    Прямое следствие п.4 `AGENTS.md`: тихое исчезновение запрещено. Эффекты
    «выбрать быстрый путь» и «сменить цену» ответ не меняют — им сертификат не
    нужен и они топологию не трогают. Всё остальное меняет ответ, и там
    топологический бит обязан быть объявлен явно, а не подразумеваться.
    """

    answer_preserving = {
        TolerancePolicyAllowedEffectV1.CHOOSE_FAST_OR_EXACT_PATH,
        TolerancePolicyAllowedEffectV1.CHANGE_COST_NEVER_THE_ANSWER,
    }
    for policy in TOLERANCE_POLICIES_V1:
        if policy.allowed_effect in answer_preserving:
            assert not policy.changes_topology, policy.id
        if policy.changes_topology:
            assert policy.allowed_effect not in answer_preserving, policy.id
        if policy.allowed_effect is (
            TolerancePolicyAllowedEffectV1.NAMED_REFUSAL_ONLY
        ):
            # Кап, который отказывает молча, не отличим от зависания: поле
            # обязано увидеть, сколько именно работы было оплачено.
            assert policy.telemetry_counters, policy.id
            assert policy.category is TolerancePolicyCategoryV1.WORK_BUDGET


# --------------------------------------------------------------------------
# 2. От кода к реестру: каждая константа формы допуска названа
# --------------------------------------------------------------------------

# ЗАМОРОЖЕННЫЙ СПИСОК ИСКЛЮЧЕНИЙ, append-only. Каждая строка — объявление
# «это не допуск», и оно обязано быть проверяемым чтением одной строки кода.
# Рост списка запрещён числом ниже: добавление исключения — осознанное
# действие с правкой этого файла, а не побочный эффект.
NON_TOLERANCE_CONSTANTS = {
    "cftuv_envelope.surface_cone_angle._STEP": (
        "шаг последнего ЗАПИСЫВАЕМОГО десятичного знака оболочки конуса "
        "(1/10**CONE_ANGLE_DECIMALS). Ширина оболочки заранее НЕ обещается: "
        "поиск расширяет окно, пока проверка не пройдёт, а фактическая ширина "
        "ИЗМЕРЯЕТСЯ и пишется в named_epsilon. Это производная граница ошибки, "
        "а такие по вердикту §11 через реестр не ходят"
    ),
}

# Число замороженных исключений. Растёт только вместе со словарём выше и
# только осознанно: тест сравнивает на равенство, а не на «не больше».
FROZEN_NON_TOLERANCE_CONSTANT_COUNT = 1


def _declared_constants() -> dict[str, str]:
    """Все константы формы допуска в `kernel/src`: точка объявления -> запись."""

    found: dict[str, str] = {}
    for path in _python_files():
        names = _integer_constants(path)
        for node in ast.walk(_parse(path)):
            if isinstance(node, ast.Assign):
                targets, value = node.targets, node.value
            elif isinstance(node, ast.AnnAssign) and node.value is not None:
                targets, value = (node.target,), node.value
            else:
                continue
            if _tolerance_shaped(value, names) is None:
                continue
            number = _literal_value(value)
            if number is not None and number in NEUTRAL_VALUES:
                continue
            for target in targets:
                if isinstance(target, ast.Name):
                    found[_dotted(path, target.id)] = (
                        f"{_relative(path)}:{node.lineno}"
                    )
                elif isinstance(target, ast.Subscript) and isinstance(
                    target.value, ast.Name
                ):
                    found[_dotted(path, target.value.id)] = (
                        f"{_relative(path)}:{node.lineno}"
                    )
    return found


def _registered_declaration_sites() -> set[str]:
    sites: set[str] = set()
    for policy in TOLERANCE_POLICIES_V1:
        for site in policy.declaration_sites:
            assert site.startswith("cftuv_envelope."), (policy.id, site)
            sites.add(site)
    return sites


def test_every_tolerance_shaped_constant_is_registered_or_excused():
    """Величина формы допуска либо в реестре, либо объявлена не допуском.

    Это вторая дверь запрета голых порогов, и главная: настоящий допуск почти
    всегда получает имя, а имя AST проследит до сравнения не всегда. Здесь
    ловится САМО ОБЪЯВЛЕНИЕ — «одно значение и одно место» владельца.
    """

    declared = _declared_constants()
    registered = _registered_declaration_sites()
    unexplained = sorted(
        f"{name} ({where})"
        for name, where in declared.items()
        if name not in registered and name not in NON_TOLERANCE_CONSTANTS
    )
    assert not unexplained, (
        "Константы формы допуска без записи в реестре: "
        f"{unexplained}. Настоящий допуск заводит запись TolerancePolicyV1 "
        "(имя, значение, категория, allowed_effect, обе фикстуры); всё "
        "остальное объявляется в NON_TOLERANCE_CONSTANTS с причиной."
    )


def test_registered_declaration_sites_still_exist_in_the_source():
    """Реестр ссылается на живые символы, а не на память о них.

    Симметрия обязательна: без этой проверки запись пережила бы удаление
    своей константы и реестр начал бы врать в сторону «всё названо».
    """

    declared = _declared_constants()
    module_level_names = set()
    for path in _python_files():
        for node in _parse(path).body:
            targets = ()
            if isinstance(node, ast.Assign):
                targets = node.targets
            elif isinstance(node, ast.AnnAssign):
                targets = (node.target,)
            for target in targets:
                if isinstance(target, ast.Name):
                    module_level_names.add(_dotted(path, target.id))
    for site in sorted(_registered_declaration_sites()):
        assert site in declared or site in module_level_names, site


def test_the_frozen_excuse_list_does_not_grow_by_itself():
    """Список исключений append-only, и его длина — замороженное число."""

    assert len(NON_TOLERANCE_CONSTANTS) == FROZEN_NON_TOLERANCE_CONSTANT_COUNT
    for name, reason in NON_TOLERANCE_CONSTANTS.items():
        assert len(reason) > 30, name
    declared = _declared_constants()
    stale = sorted(name for name in NON_TOLERANCE_CONSTANTS if name not in declared)
    assert not stale, (
        f"Исключения без своей константы: {stale}. Список append-only, но "
        "мёртвая строка в нём — это разрешение, выданное неизвестно чему."
    )


# --------------------------------------------------------------------------
# 3. Ни одного голого порога в сравнениях
# --------------------------------------------------------------------------

# ЗАМОРОЖЕННЫЙ СПИСОК, append-only. Ключ — `файл:выражение` (не строка: сдвиг
# кода не должен требовать правки списка), значение — обоснование. Обе записи
# покрывают одно и то же выражение `Fraction(1, 2)` в трёх местах двух модулей,
# и это точная математическая ПОЛОВИНА: половина оборота есть развёрнутый угол
# π, половина параметра отрезка есть его середина. ТОЧНЫЕ величины закона, а не
# допуски: сдвинуть их нельзя, не меняя смысл сравнения.
NAKED_COMPARISON_LITERALS = {
    "cftuv_envelope/reference/angular.py:Fraction(1, 2)": (
        "половина оборота — это в точности развёрнутый угол π; сравнение "
        "делит вогнутые и выпуклые, а не терпит отклонение"
    ),
    "cftuv_envelope/reference/evaluation_geometry.py:Fraction(1, 2)": (
        "половина параметра отрезка — точная середина, граница выбора "
        "ближайшего конца; допуска в ней нет"
    ),
}

FROZEN_NAKED_COMPARISON_LITERAL_COUNT = 2


def _naked_comparison_literals() -> dict[str, list[str]]:
    """Литералы формы допуска, стоящие операндом сравнения."""

    found: dict[str, list[str]] = {}
    for path in _python_files():
        names = _integer_constants(path)
        relative = path.relative_to(KERNEL_SOURCE).as_posix()
        for node in ast.walk(_parse(path)):
            if not isinstance(node, ast.Compare):
                continue
            for operand in (node.left, *node.comparators):
                if _tolerance_shaped(operand, names) is None:
                    continue
                number = _literal_value(operand)
                if number is not None and number in NEUTRAL_VALUES:
                    continue
                key = f"{relative}:{ast.unparse(operand)}"
                found.setdefault(key, []).append(
                    f"{_relative(path)}:{node.lineno}"
                )
    return found


def test_geometry_modules_carry_no_naked_numeric_thresholds():
    """Голый порог в сравнении запрещён: `abs(x) < 1e-7` здесь не живёт.

    Это правило владельца в исполняемой форме: «дисциплина, отличающая нас от
    рассыпанных magic numbers — каждый допуск имеет ИМЯ, одно значение и одно
    место». Литерал прямо в сравнении не имеет ни имени, ни места: его нельзя
    ни найти, ни изменить одним движением, ни объяснить полю.

    Как чинить провал: если число — настоящий допуск, заведите ему запись
    `TolerancePolicyV1` и константу с именем; если это точная математическая
    величина закона, добавьте её сюда с обоснованием и поднимите замороженное
    число. Третьего пути нет намеренно.
    """

    found = _naked_comparison_literals()
    unexplained = {
        key: where
        for key, where in found.items()
        if key not in NAKED_COMPARISON_LITERALS
    }
    assert not unexplained, (
        f"Голые числовые пороги в сравнениях: {sorted(unexplained)}. "
        f"Места: {unexplained}."
    )


def test_the_frozen_naked_literal_list_does_not_grow_and_stays_alive():
    """Список исключений заморожен по длине и не содержит мёртвых строк."""

    assert (
        len(NAKED_COMPARISON_LITERALS) == FROZEN_NAKED_COMPARISON_LITERAL_COUNT
    )
    found = _naked_comparison_literals()
    stale = sorted(key for key in NAKED_COMPARISON_LITERALS if key not in found)
    assert not stale, (
        f"Исключения без своего сравнения: {stale}. Мёртвая строка в списке "
        "исключений — это разрешение, выданное неизвестно чему."
    )


def test_the_rule_can_actually_fail():
    """Проверка, у которой нельзя построить нарушение, не проверяет ничего.

    Закон репозитория, оплаченный P0-4: два самодоказательных поля были сняты
    именно потому, что вход, на котором они упали бы, не существует. Поэтому
    здесь строится нарушение и проверяется, что детектор его видит.
    """

    module = ast.parse("if abs(residual) < 1e-7:\n    pass\n")
    compare = next(
        node for node in ast.walk(module) if isinstance(node, ast.Compare)
    )
    assert _tolerance_shaped(compare.comparators[0]) == "float"

    module = ast.parse("if residual > Fraction(1, 10**7):\n    pass\n")
    compare = next(
        node for node in ast.walk(module) if isinstance(node, ast.Compare)
    )
    assert _tolerance_shaped(compare.comparators[0]) == "Fraction"

    module = ast.parse("if len(items) > 0:\n    pass\n")
    compare = next(
        node for node in ast.walk(module) if isinstance(node, ast.Compare)
    )
    assert _tolerance_shaped(compare.comparators[0]) is None
