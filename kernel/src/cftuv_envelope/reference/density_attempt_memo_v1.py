"""Замкнутые записи успешных попыток Density; ключ не является входом геометрии."""

from dataclasses import dataclass, fields, is_dataclass, replace
from fractions import Fraction

from . import density_oracle_v1 as facts


def _closed_fact(value):
    if type(value) in (int, bool, str, Fraction, type(None)):
        return
    if type(value) is tuple:
        for item in value:
            _closed_fact(item)
        return
    if (not is_dataclass(value) or not value.__dataclass_params__.frozen
            or type(value).__module__ not in (__name__, facts.__name__)):
        raise TypeError("Density attempt memo contains non-closed facts")
    for field in fields(value):
        _closed_fact(getattr(value, field.name))


@dataclass(frozen=True, slots=True)
class DensityAttemptKeyV1:
    canon_ids: facts.DensityOracleCanonV1
    gram: tuple[Fraction, Fraction, Fraction]
    inverse_gram: tuple[Fraction, Fraction, Fraction]
    owner_orientation_sign: int
    ideal_primal_literals: tuple[tuple[tuple, tuple], ...]
    orientation: str
    q: int
    binding_reasons: tuple[str | None, ...]
    adaptive_only: bool
    window_law: str


@dataclass(frozen=True, slots=True)
class DensityAttemptReceiptV1:
    key: DensityAttemptKeyV1
    binding_certificates: tuple[facts.BindingCertificateFactsV1 | None, ...]
    authority: facts.AdaptiveFanAuthorityFactsV1 | None
    shell_events: tuple[facts.DensityShellEventFactsV1, ...]


@dataclass(frozen=True, slots=True)
class DensityAttemptMemoSnapshotV1:
    entries: tuple[DensityAttemptReceiptV1, ...] = ()

    def __post_init__(self):
        if type(self.entries) is not tuple or any(type(item) is not DensityAttemptReceiptV1 for item in self.entries):
            raise TypeError("Density memo snapshot requires immutable successful-attempt receipts")
        if len({item.key for item in self.entries}) != len(self.entries):
            raise ValueError("Density memo snapshot contains duplicate attempt keys")
        for item in self.entries:
            if type(item.key) is not DensityAttemptKeyV1:
                raise TypeError("Density attempt memo requires a closed attempt key")
            if type(item.key.canon_ids) is not facts.DensityOracleCanonV1:
                raise TypeError("Density attempt key requires a canon namespace")
            if (type(item.binding_certificates) is not tuple
                    or any(value is not None and type(value) is not facts.BindingCertificateFactsV1
                           for value in item.binding_certificates)
                    or item.authority is not None and type(item.authority) is not facts.AdaptiveFanAuthorityFactsV1
                    or type(item.shell_events) is not tuple
                    or any(type(value) is not facts.DensityShellEventFactsV1 for value in item.shell_events)):
                raise TypeError("Density attempt receipt has malformed result facts")
            item.key.canon_ids.validate_candidate()
            _closed_fact(item)


@dataclass(frozen=True, slots=True)
class DensityMemoBatchResultV1:
    results: tuple[facts.RawDensitySpecResultV1, ...]
    delta: DensityAttemptMemoSnapshotV1


def density_attempt_key(laws, metric, ideal, orientation, q, binding_reasons, adaptive_only, window_law, canon_ids):
    def matrix(values):
        return tuple(laws.fraction(values[row][column]) for row, column in ((0, 0), (0, 1), (1, 1)))
    literals = tuple(tuple(laws.arena.literal_tree(value.value) for value in vector.expressions()) for vector in ideal)
    return DensityAttemptKeyV1(canon_ids, matrix(metric.gram), matrix(metric.inverse_gram),
        metric.owner_orientation_sign, literals, orientation.value, q,
        tuple(None if reason is None else reason.value for reason in binding_reasons), adaptive_only, window_law)


def _restore_shells(events):
    from .density_shell_v2 import DensityShellOutcomeBundleV2, DensityShellOutcomeCode, DensityShellRationalMetadataV2
    return tuple((event.value_wire, DensityShellOutcomeBundleV2(
        tuple(DensityShellOutcomeCode(code) for code in event.codes),
        None if event.rational_numerator is None else DensityShellRationalMetadataV2(
            event.rational_numerator, event.rational_denominator, event.rational_grid_units),
        event.legacy_projected_min_width_units)) for event in events)


def _restore_result(receipt):
    from ..contracts import envelopes as ir
    from .adaptive_density_fan import _authority_id
    from .density_result_v1 import format_certificate, format_window
    value = receipt.authority
    authority = None
    if value is not None:
        windows = tuple(format_window(window) for window in value.ordinal_windows)
        witness = value.previous_height_witness
        authority = ir.AdaptiveMinimalRationalFanAuthorityV2(
            _authority_id(value.max_subturn_q, value.minimal_common_height, value.bound_primitive_integer_vectors, windows),
            value.max_subturn_q, value.minimal_common_height, value.exhaustive_previous_height,
            value.termination_height_upper_bound, value.bound_primitive_integer_vectors,
            tuple(None if reason is None else ir.DirectionBindingReasonV1(reason) for reason in value.binding_reasons),
            windows, ir.AdaptiveFareyHeightRangeWitnessV2(witness.first_height, witness.last_height,
                witness.primitive_candidate_counts), frozenset(value.proven_predicates))
    return tuple(format_certificate(item) for item in receipt.binding_certificates), authority


class DensityAttemptMemoV1:
    """Один вызов с неизменяемым снимком; успешный prefix доступен даже при исключении."""

    def __init__(self, snapshot=DensityAttemptMemoSnapshotV1()):
        if type(snapshot) is not DensityAttemptMemoSnapshotV1:
            raise TypeError("Density attempts require DensityAttemptMemoSnapshotV1")
        self.receipts = {entry.key: entry for entry in snapshot.entries}
        self._initial_keys = frozenset(self.receipts)
        self._local_results = {}

    def lookup(self, key):
        receipt = self.receipts.get(key)
        if receipt is None:
            return None
        if key not in self._local_results:
            self._local_results[key] = _restore_result(receipt), _restore_shells(receipt.shell_events)
        return self._local_results[key]

    def remember(self, key, result, shell_outcomes):
        from .density_raw_frontend_v1 import _authority, _certificate, _shell_facts
        authority = _authority(result[1])
        if authority is not None:
            authority = replace(authority, authority_id=None)
        receipt = DensityAttemptReceiptV1(key, tuple(_certificate(item) for item in result[0]),
            authority, _shell_facts(shell_outcomes))
        self.receipts[key] = receipt
        self._local_results[key] = result, tuple(shell_outcomes)

    def delta(self):
        return DensityAttemptMemoSnapshotV1(tuple(value for key, value in self.receipts.items() if key not in self._initial_keys))


def merge_density_attempt_delta(memo, delta):
    """Host transaction owns the dictionary; conflict is an internal failure."""
    if type(memo) is not dict or type(delta) is not DensityAttemptMemoSnapshotV1:
        raise TypeError("Density memo merge requires a host dictionary and closed delta")
    for receipt in delta.entries:
        previous = memo.get(receipt.key)
        if previous is not None and previous != receipt:
            raise ValueError("DENSITY_ATTEMPT_MEMO_V1_CONFLICT")
    memo.update((receipt.key, receipt) for receipt in delta.entries)
