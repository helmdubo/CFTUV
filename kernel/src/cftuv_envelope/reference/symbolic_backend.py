"""Переключатель символьного бэкенда эталона: `SYMPY` | `NATIVE_EXACT` | `SHADOW`.

Шаг 2 плана SYMPY-OFF-HOT-PATH (шаг 1 — аудит `kernel/tests/test_symbolic_backend_audit.py`,
шаг 3 — переключение умолчания и однократная перезапись базовых линий после чистого теневого
прогона). По умолчанию `SYMPY`: ответы и байты те же, что до введения переключателя.

`SYMPY` — прежний путь, ничего не меняется (проверка режима — одно чтение списка).
`NATIVE_EXACT` — считается ТОЛЬКО новый путь (`native_exact`) в заменённых горячих местах; ради
замера скорости, умолчанием не является и ни один закоммиченный дайджест не трогает.
`SHADOW` — считаются оба пути, возвращается ответ `sympy`, а новый сверяется с ним по ЗНАЧЕНИЮ
(точное равенство чисел, не строк). Любое расхождение — именованный исход
`EXACT_SYMBOLIC_BACKEND_DISAGREEMENT`: по умолчанию исключение (`DisagreementPolicyV1.RAISE`),
для сквозного прогона корпуса — запись (`RECORD`), чтобы один прогон собрал ВСЕ расхождения.

Тихого исчезновения нет (AGENTS.md, пункт 4): каждое решение, которое родная арифметика не
взяла на себя, учитывается в `BACKEND_COUNTS` под именем (`outside_field`, `sign_undecided`),
и сверка, которую нельзя провести (выражение вне поля), считается отдельно от расхождения.
Сверка ТЕКСТА (`srepr` в `ExactScalar`) — отдельный счёт `text_equal` / `text_differs`: текст входит
в дайджесты, и именно его расхождение скажет шагу 3, какие дайджесты менять.

Режим держится модульной переменной по той же причине, что и `ExactIdentityModeV1`: заменяемые
функции — свободные функции без строителя в подписи. Опасности «глобальное состояние меняет
ответ» нет только в `SYMPY` и `SHADOW`; `NATIVE_EXACT` может менять ТЕКСТ многочленных величин,
поэтому его включает ворота-харнесс (`artifacts/sympy_off_hot_path/`) и тест, а не продукт.
"""

from __future__ import annotations

from contextlib import contextmanager
from enum import Enum
from typing import Iterator

EXACT_SYMBOLIC_BACKEND_DISAGREEMENT = "EXACT_SYMBOLIC_BACKEND_DISAGREEMENT"


class SymbolicBackendV1(str, Enum):
    SYMPY = "SYMPY"
    NATIVE_EXACT = "NATIVE_EXACT"
    SHADOW = "SHADOW"


class DisagreementPolicyV1(str, Enum):
    """Что делает теневая сверка при расхождении значений."""

    RAISE = "RAISE"
    RECORD = "RECORD"


class ExactSymbolicBackendDisagreement(AssertionError):
    """Родная арифметика и sympy дали РАЗНЫЕ значения. Код — `EXACT_SYMBOLIC_BACKEND_DISAGREEMENT`."""

    code = EXACT_SYMBOLIC_BACKEND_DISAGREEMENT

    def __init__(self, site: str, detail: str) -> None:
        super().__init__(f"{EXACT_SYMBOLIC_BACKEND_DISAGREEMENT}: {site}: {detail}")
        self.site = site
        self.detail = detail


_MODE: list[SymbolicBackendV1] = [SymbolicBackendV1.SYMPY]
_POLICY: list[DisagreementPolicyV1] = [DisagreementPolicyV1.RAISE]

#: Счётчики `"<место>.<событие>" -> число`. Наблюдение: на ответ не влияет.
BACKEND_COUNTS: dict[str, int] = {}
#: Записанные расхождения (политика `RECORD`); потолок, чтобы корпус не съел память.
DISAGREEMENTS: list[tuple[str, str]] = []
_DISAGREEMENT_RECORD_LIMIT = 200


def backend_mode() -> SymbolicBackendV1:
    return _MODE[0]


def set_backend_mode(mode: SymbolicBackendV1) -> SymbolicBackendV1:
    """Сменить режим; вернуть прежний. Только для ворот и тестов."""

    previous = _MODE[0]
    _MODE[0] = SymbolicBackendV1(mode)
    return previous


def disagreement_policy() -> DisagreementPolicyV1:
    return _POLICY[0]


def set_disagreement_policy(policy: DisagreementPolicyV1) -> DisagreementPolicyV1:
    previous = _POLICY[0]
    _POLICY[0] = DisagreementPolicyV1(policy)
    return previous


@contextmanager
def symbolic_backend(
    mode: SymbolicBackendV1,
    policy: DisagreementPolicyV1 | None = None,
) -> Iterator[None]:
    """Контекст режима (и, если названа, политики расхождения); режим возвращается всегда."""

    previous_mode = set_backend_mode(mode)
    previous_policy = _POLICY[0]
    if policy is not None:
        set_disagreement_policy(policy)
    try:
        yield
    finally:
        _MODE[0] = previous_mode
        _POLICY[0] = previous_policy


def count(site: str, event: str, amount: int = 1) -> None:
    key = f"{site}.{event}"
    BACKEND_COUNTS[key] = BACKEND_COUNTS.get(key, 0) + amount


def reset_backend_counts() -> None:
    BACKEND_COUNTS.clear()
    DISAGREEMENTS.clear()


def disagreement(site: str, detail: str) -> None:
    """Расхождение значений: счёт, запись и (по политике) исключение."""

    count(site, "disagreement")
    if len(DISAGREEMENTS) < _DISAGREEMENT_RECORD_LIMIT:
        DISAGREEMENTS.append((site, detail))
    if _POLICY[0] is DisagreementPolicyV1.RAISE:
        raise ExactSymbolicBackendDisagreement(site, detail)
