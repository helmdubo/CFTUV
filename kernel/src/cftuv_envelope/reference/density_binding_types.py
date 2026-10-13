"""Общие имена предикатов и отказ рациональной привязки направлений."""

from __future__ import annotations


BINDING_MONOTONE = "BINDING_MONOTONE"
BINDING_SUBTURN_LE_DELTA_MAX = "BINDING_SUBTURN_LE_DELTA_MAX"
BINDING_SUBTURN_RATIONAL_DIRECTION_REQUIRED = (
    "BINDING_SUBTURN_RATIONAL_DIRECTION_REQUIRED"
)
BINDING_INSIDE_OWN_ORDINAL_WINDOW = "BINDING_INSIDE_OWN_ORDINAL_WINDOW"
BINDING_REFINEMENT_LIMIT = "BINDING_REFINEMENT_LIMIT"
BINDING_REFUSAL_REASON = "привязка: сертификат не доказан ({predicate})"

_PROVEN_PREDICATES = frozenset(
    {
        BINDING_MONOTONE,
        BINDING_SUBTURN_LE_DELTA_MAX,
        BINDING_INSIDE_OWN_ORDINAL_WINDOW,
    }
)
_DECIMAL_DIGITS = 27
_DECIMAL_SCALE = 10**_DECIMAL_DIGITS


class DirectionBindingCertificateUnproven(ValueError):
    """Именованный отказ, если хотя бы один точный предикат не доказан."""

    def __init__(self, predicate: str):
        self.predicate = predicate
        super().__init__(BINDING_REFUSAL_REASON.format(predicate=predicate))
