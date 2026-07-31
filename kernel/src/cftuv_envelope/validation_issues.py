"""Общий словарь проверок: код отказа, запись о нём и сборка списка.

Живёт отдельно от `validation.py` по одной причине: словарь принадлежит не
одному проверяющему. Ветка near-planar не помещалась в `validation.py`
(модуль стоит ровно на своём потолке в `tests/test_architecture.py`, и потолок
не поднимается), поэтому проверки метрики уехали в `validation_metric.py`. Оба
модуля называют отказы одними и теми же именами, и если бы код отказа жил у
одного из них, второй импортировал бы его через первого — то есть через цикл
или через отложенный импорт внутри функции. Ни то ни другое не выражает того,
что здесь на самом деле есть: общий предок.

Здесь нет ни одной проверки — только форма, в которой они говорят.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class ValidationCode(str, Enum):
    SCHEMA_VERSION = "SCHEMA_VERSION"
    CAPABILITY = "CAPABILITY"
    DUPLICATE_ID = "DUPLICATE_ID"
    MISSING_REFERENCE = "MISSING_REFERENCE"
    CROSS_CONTRACT_MISMATCH = "CROSS_CONTRACT_MISMATCH"
    POLICY_MISMATCH = "POLICY_MISMATCH"
    ANGULAR_CERTIFICATE = "ANGULAR_CERTIFICATE"
    SEED_VARIANT_MISMATCH = "SEED_VARIANT_MISMATCH"
    PLAN_KEY_MISMATCH = "PLAN_KEY_MISMATCH"
    OWNERSHIP_DECLARATION = "OWNERSHIP_DECLARATION"
    TESSELLATION_AUTHORITY = "TESSELLATION_AUTHORITY"
    GEOMETRY_BATCH = "GEOMETRY_BATCH"
    FORBIDDEN_TOPOLOGY_IDENTITY = "FORBIDDEN_TOPOLOGY_IDENTITY"
    SURFACE_TOPOLOGY = "SURFACE_TOPOLOGY"
    SURFACE_METRIC = "SURFACE_METRIC"
    ROUTE_TOPOLOGY = "ROUTE_TOPOLOGY"
    TERMINAL_RELATION = "TERMINAL_RELATION"
    ANGULAR_SELECTION_UNCERTAIN = "ANGULAR_SELECTION_UNCERTAIN"
    STATION_FACT = "STATION_FACT"
    EVALUATION_GEOMETRY = "EVALUATION_GEOMETRY"


@dataclass(frozen=True, slots=True)
class ValidationIssue:
    code: ValidationCode
    path: tuple[str, ...]
    message: str


class ContractValidationError(ValueError):
    def __init__(self, issues: tuple[ValidationIssue, ...]) -> None:
        self.issues = issues
        detail = "; ".join(
            f"{issue.code.value}@{'.'.join(issue.path)}: {issue.message}"
            for issue in issues
        )
        super().__init__(detail)


def raise_for_issues(issues: tuple[ValidationIssue, ...]) -> None:
    if issues:
        raise ContractValidationError(issues)


def add_issue(
    issues: list[ValidationIssue],
    code: ValidationCode,
    path: tuple[str, ...],
    message: str,
) -> None:
    issues.append(ValidationIssue(code, path, message))
