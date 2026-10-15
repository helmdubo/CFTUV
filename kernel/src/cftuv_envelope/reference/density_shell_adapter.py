"""Изолированный A1-кандидат; замороженная legacy-оболочка остаётся своим владельцем."""

from ..numeric import CertifiedDecimalIntervalV1, IntervalEndpointKind
from .density_binding_values import _density_decimal_envelope as _legacy_density_decimal_envelope
from .density_shell_events import record_density_shell_candidate
from .density_shell_v2 import DensityShellV2Candidate, density_shell_v2


def _density_decimal_envelope_v2(expression, metric=None):
    """Прежний допуск, затем целочисленная оболочка по значению.

    Продуктовая точка входа пока не переключена. Неподдержанное V2
    выражение сохраняет старую оболочку; V2 не усиливает допуск или знак.
    Именованные исходы принятого значения передаются текущему захвату веера.
    """

    legacy = _legacy_density_decimal_envelope(expression, metric)
    candidate = density_shell_v2(expression)
    if not isinstance(candidate, DensityShellV2Candidate):
        return legacy
    record_density_shell_candidate(metric, candidate)
    shell = candidate.shell
    return CertifiedDecimalIntervalV1(
        shell.lower,
        shell.upper,
        IntervalEndpointKind.CLOSED,
        IntervalEndpointKind.CLOSED,
        shell.absolute_error_bound,
    )
