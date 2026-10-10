"""Переключатель символьного бэкенда эталона: `SYMPY` | `NATIVE_EXACT` | `SHADOW`.

План SYMPY-OFF-HOT-PATH: шаг 1 — аудит `kernel/tests/test_symbolic_backend_audit.py`, шаг 2 —
родная арифметика за переключателем, шаг 3 — УМОЛЧАНИЕ `NATIVE_EXACT` (`DEFAULT_BACKEND`;
переключено после теневого прогона без расхождений, DECISIONS 2026-10-04). Откат — одна константа
ниже; `SYMPY` и `SHADOW` остаются выбираемыми (ворота: `CFTUV_SYMBOLIC_BACKEND`).

`SYMPY` — прежний путь вычислений целиком; эталон сверки и откат ПО ЗНАЧЕНИЯМ, а не по именам. С EXACT_SCALAR_TEXT_CANON_V2 строка
alpha-величины (ключ события границы, имя экземпляра огибающей, `effective_alpha`) — функция её значения, и пишет её один читатель
(`ExactScalar.canonical`) при ЛЮБОМ режиме. Прежней формы `srepr(factor(cancel(...)))` под `SYMPY` больше не получить, уступки sympy
для текста нет (величина вне родного поля либо многочленная — именованный отказ `EXACT_SCALAR_TEXT_CANON_UNSUPPORTED`), поэтому
возврат на `SYMPY` не возвращает старые имена и дайджесты: их даёт только дерево до V2.
`NATIVE_EXACT` — считается ТОЛЬКО родной путь (`native_exact`) в заменённых горячих местах;
УМОЛЧАНИЕ. Ответы и дайджесты доменов `building` побитово те же, что у `SYMPY`.
`SHADOW` — считаются оба пути, возвращается ответ `sympy`, а новый сверяется с ним по ЗНАЧЕНИЮ
(точное равенство чисел, не строк). Любое расхождение — именованный исход
`EXACT_SYMBOLIC_BACKEND_DISAGREEMENT`: по умолчанию исключение (`DisagreementPolicyV1.RAISE`),
для сквозного прогона корпуса — запись (`RECORD`), чтобы один прогон собрал ВСЕ расхождения.

Тихого исчезновения нет (AGENTS.md, пункт 4): каждое решение, которое родная арифметика не
взяла на себя, учитывается в `BACKEND_COUNTS` под именем (`outside_field`, `sign_undecided`),
и сверка, которую нельзя провести (выражение вне поля), считается отдельно от расхождения.
Сверка ТЕКСТА (`ExactScalar`) — отдельные счета `text_equal` / `text_legacy_form` / `text_multi_term`: текст входит в дайджесты.
`text_equal` — строка обычного `from_value` совпала с каноном V2 (рациональные); `text_legacy_form` — у одночленных форма sympy
отличается от канона по замыслу, значения равны; `text_multi_term` — канона нет. Расхождение ЗНАЧЕНИЯ строки — `disagreement`.

Режим держится модульной переменной по той же причине, что и `ExactIdentityModeV1`: заменяемые
функции — свободные функции без строителя в подписи. Умолчание — константа МОДУЛЯ, а не окружение:
процессы пула доменов импортируют ядро заново и обязаны получить тот же режим, что и главный.
Выбор другого режима делают только ворота-харнесс (`artifacts/sympy_off_hot_path/`) и тесты.
Режим не меняет строку alpha-величин. Строку остальных величин (координаты) пишет путь их вычисления: родная величина — по канону V2
(многочленная — именованный отказ), выражение `sympy` — прежней формой sympy; значение у обеих одно. Ключ содержимого домена
включает отпечаток кода (`code_identity`), поэтому старые записи кэша не читаются.
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


#: Умолчание продукта. Меняется ЗДЕСЬ и только здесь (тест `test_the_default_backend_is_native_exact`).
DEFAULT_BACKEND = SymbolicBackendV1.NATIVE_EXACT

_MODE: list[SymbolicBackendV1] = [DEFAULT_BACKEND]
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
