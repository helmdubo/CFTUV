"""Правило запрета «точная работа мимо бюджета» — одно на обе сюиты.

Вынесено из теста отдельным модулем не ради красоты: правило обязано быть
ОДНИМ. Ядро проверяет им `kernel/src`, хост — `cftuv/` (`tests/test_architecture
.py`), и если бы каждая сюита несла свою копию, две копии разошлись бы ровно в
тот день, когда одну из них поправят.

Здесь нет ни одного импорта тяжелее `ast`: модуль читают обе стороны, и
хостовая сторона не имеет права тянуть за собой ядро.

Полное обоснование состава поверхности и списка исключений —
`kernel/tests/test_exact_work_budget_coverage.py`.
"""

from __future__ import annotations

import ast
from pathlib import Path


# --------------------------------------------------------------------------
# 1. Что именно считается дорогой точной операцией
# --------------------------------------------------------------------------

# Имя вызова -> (позиция бюджета среди позиционных аргументов, имена ключевых).
# `None` в позиции означает «только по ключевому слову».
#
# ПОЧЕМУ ИМЕННО ЭТИ. Бюджет считает шесть статей, и все шесть растут ровно в
# двух местах: в разложении радиканда (`_factorize` -> Миллер—Рабин,
# ро-Поллард) и в гидратации точной позиции. Ниже перечислено всё, откуда в эти
# два места есть дорога:
#
#   * `sign`         — при неудаче целочисленной оболочки уходит в сопряжение
#                      по простому, а простое берётся из `prime_support`, то
#                      есть из факторизации НОВОГО радиканда. Это и есть тот
#                      путь, которым полевой прогон стоял 900 с.
#   * `radical`      — единственная дверь, через которую новый радиканд вообще
#                      попадает в ядро (`squarefree_split`).
#   * `divided_by`   — `_pick_prime` спрашивает носитель ПРОИЗВОДНОГО радикала,
#                      которого никто не видел, плюс `radical` на каждом шаге.
#   * `squarefree_split`, `prime_support`, `_prime_universe_from_q_values`,
#     `_divide_with_prime_universe` — те же двери, только названные прямо.
#
# Дальше — ОБЁРТКИ, которые сами ничего не считают, но передают бюджет вниз.
# Они в списке не для симметрии: вызывающий, забывший бюджет у обёртки,
# открывает ровно ту же дыру, что и вызывающий `sign` напрямую, — и именно так
# она и была открыта (71 824 из 88 042 вызовов пришли через `compare_times`).
#
#   * `compare_times`, `concurrency_time`, `sliding_time`, `sliding_point`,
#     `event_point`, `_event_point`, `_event_point_with_prime_universe`,
#     `EventTimeV1.normalized` — точная геометрия события;
#   * `bounds_time`, `orientation`, `segments_cross`, `contour_crossings`,
#     `clip_to_halfplane`, `build_faces`, `coverage_at` — предикаты и этапы,
#     каждый из которых внутри зовёт `sign`.
#
# ЧЕГО ЗДЕСЬ НЕТ И ПОЧЕМУ. `enclosure` и `certified_sign` — целочисленный
# фильтр на `isqrt`: их цена ограничена числом слагаемых и битами, они не
# факторизуют ничего и бюджету не подотчётны. `is_zero`, `as_rational`,
# `scaled`, `__add__`, `__mul__` — чистая арифметика над готовыми
# коэффициентами, новых радикандов не рождает (`__mul__` замкнут на gcd).
CHARGED_EXACT_SURFACE: dict[str, tuple[int | None, tuple[str, ...]]] = {
    "sign": (None, ("budget",)),
    "radical": (2, ("budget",)),
    "divided_by": (1, ("budget",)),
    "squarefree_split": (1, ("budget",)),
    "prime_support": (1, ("budget",)),
    "_prime_universe_from_q_values": (1, ("budget",)),
    "_divide_with_prime_universe": (3, ("budget",)),
    "compare_times": (2, ("budget",)),
    "concurrency_time": (3, ("budget",)),
    "sliding_time": (3, ("budget",)),
    "sliding_point": (3, ("budget",)),
    "event_point": (3, ("budget",)),
    "_event_point": (None, ("budget",)),
    "_event_point_with_prime_universe": (4, ("budget",)),
    "normalized": (2, ("budget",)),
    "bounds_time": (1, ("budget",)),
    "orientation": (3, ("budget",)),
    "segments_cross": (4, ("budget",)),
    "contour_crossings": (1, ("budget",)),
    "clip_to_halfplane": (None, ("budget",)),
    "build_faces": (2, ("work_budget",)),
    "coverage_at": (2, ("work_budget",)),
}


# --------------------------------------------------------------------------
# 2. Именованный замороженный список исключений
# --------------------------------------------------------------------------

# Причины, по которым место ЗАКОННО остаётся без названного бюджета. Список
# append-only: имя причины заводится один раз и не переписывается.
EXEMPTION_REASONS: dict[str, str] = {
    "OPERATOR_HAS_NO_THIRD_ARGUMENT": (
        "Питоновский оператор третьего аргумента не принимает, поэтому "
        "`SqrtSumV1.__truediv__` физически не может назвать бюджет и зовёт "
        "`divided_by(other, None)`. Именованный путь рядом есть и называется "
        "`divided_by`; оператор оставлен, потому что он стоит в чужих зонах. "
        "Дыра здесь СТАТИЧЕСКАЯ, а не действующая: динамический сторож ниже "
        "меряет число делений оператором на всём корпусе скелетов и требует "
        "нуля, поэтому реальная работа через оператор сегодня не идёт."
    ),
    "RECEIVER_IS_NOT_A_SQRT_SUM": (
        "Совпадение ИМЕНИ, а не вызов дорогой поверхности. Статический запрет "
        "сознательно сделан избыточно широким — он ловит вызов по имени, не "
        "разбирая тип получателя, потому что ложное срабатывание стоит одной "
        "строки в этом списке, а пропуск стоит зависания в поле. Здесь "
        "получатель — башенное расширение `_TowerValue`/`_TowerField` "
        "(`reference/direction_binding.py`, `reference/direction_window_exact"
        ".py`) со своими `sign`/`radical`, либо свободная функция "
        "`reference.angular.exact_sign`, переданная параметром `sign=` "
        "(`reference/adaptive_density_atlas.py`). Ни один из них не трогает "
        "`exact_sqrt_sum`, и бюджету подотчётен быть не может."
    ),
    "REFERENCE_ARRANGEMENT_NOT_THREADED_YET": (
        "ИЗМЕРЕННАЯ открытая дыра, а не законное исключение. Точный "
        "арранжемент (`reference/arrangement.py`, `reference/planar_types.py`, "
        "`reference/angular.py`) считает знаки на настоящих `SqrtSumV1`, "
        "полученных из `exact_quadratic_value`, и делает это без бюджета: "
        "измерено 13 340 безбюджетных вызовов `sign` и 1 310 делений на шести "
        "файлах тестов эталонного слоя (см. отчёт карточки "
        "BUDGET-COVERAGE-STRUCTURAL). Протяжка туда — не передача уже "
        "имеющегося счёта, а заведение транзакции в слое, где её сегодня нет "
        "вовсе: `arrangement.py` — 1 568 строк мелких чистых предикатов, "
        "бюджету взяться неоткуда. Дыра НАЗВАНА и ЗАМОРОЖЕНА числом: вырасти "
        "она уже не может, а закрывает её отдельная карточка."
    ),
}


# --------------------------------------------------------------------------
# 3. Сам запрет: обход AST
# --------------------------------------------------------------------------


def _callee_name(node: ast.Call) -> str | None:
    func = node.func
    if isinstance(func, ast.Attribute):
        return func.attr
    if isinstance(func, ast.Name):
        return func.id
    return None


def _budget_is_named(node: ast.Call, position: int | None, keywords) -> bool:
    """Назван ли бюджет у этого вызова.

    Литеральный `None` за бюджет НЕ считается: `sign(budget=None)` — это в
    точности безбюджетный вызов, только записанный так, будто он оплачен.
    Именно поэтому проверка смотрит на значение, а не на присутствие имени.
    """

    for keyword in node.keywords:
        if keyword.arg in keywords:
            value = keyword.value
            return not (
                isinstance(value, ast.Constant) and value.value is None
            )
        if keyword.arg is None:
            # `**kwargs` — сказать про бюджет нечего, считаем названным, иначе
            # запрет ловил бы форму записи, а не отсутствие счёта.
            return True
    if position is not None and len(node.args) > position:
        value = node.args[position]
        if isinstance(value, ast.Starred):
            return True
        return not (isinstance(value, ast.Constant) and value.value is None)
    return False


def unbudgeted_sites_in_source(
    source: str, module: str
) -> tuple[tuple[str, str, int], ...]:
    """Все вызовы дорогой поверхности без названного бюджета в одном файле.

    Возвращает кортеж `(модуль, имя вызова, строка)`. Функция — единственное
    место правила; и боевой обход ядра, и построенные нарушения ниже ходят
    через неё, поэтому «проверка проверяет то же, что запрещает» здесь не
    заявление, а следствие устройства.
    """

    found: list[tuple[str, str, int]] = []
    for node in ast.walk(ast.parse(source)):
        if not isinstance(node, ast.Call):
            continue
        name = _callee_name(node)
        if name not in CHARGED_EXACT_SURFACE:
            continue
        position, keywords = CHARGED_EXACT_SURFACE[name]
        if not _budget_is_named(node, position, keywords):
            found.append((module, name, node.lineno))
    return tuple(found)


def scan_tree(root: Path) -> tuple[tuple[str, str, int], ...]:
    found: list[tuple[str, str, int]] = []
    for path in sorted(root.rglob("*.py")):
        module = str(path.relative_to(root))
        found.extend(
            unbudgeted_sites_in_source(
                path.read_text(encoding="utf-8"), module
            )
        )
    return tuple(found)


def _grouped(sites) -> dict[tuple[str, str], list[int]]:
    grouped: dict[tuple[str, str], list[int]] = {}
    for module, name, lineno in sites:
        grouped.setdefault((module, name), []).append(lineno)
    return grouped


