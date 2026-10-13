"""Actual A1 adapter: полная compilation, Decimal quantum, memo replay и цены.

Меняются только две consumer aliases на время отдельного policy run. Исходный
legacy evaluator остаётся прежним; это gate, а не включение нового default.
"""

from __future__ import annotations

import argparse
from contextlib import contextmanager
from dataclasses import fields, is_dataclass, replace
from decimal import Decimal
from enum import Enum
import hashlib
import json
import lzma
from pathlib import Path
import pickle
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel" / "src"))

import cftuv_envelope as kernel  # noqa: E402
from cftuv_envelope.codec import to_canonical_data  # noqa: E402
from cftuv_envelope.reference import adaptive_density_atlas as atlas  # noqa: E402
from cftuv_envelope.reference import adaptive_density_fan as fan, direction_binding as binding  # noqa: E402
from cftuv_envelope.reference import density_binding_values as values  # noqa: E402
from cftuv_envelope.reference import density_shell_adapter as adapter  # noqa: E402
from cftuv_envelope.reference.density_shell_v2 import DensityShellV2Candidate  # noqa: E402
from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector, SHELL_EVENT_OUTCOMES  # noqa: E402
from cftuv_envelope.reference.metric import _DensityExactMemo  # noqa: E402

FIXTURES = ROOT / "kernel" / "fixtures"
CAP_NAMES = {"search_cap": (fan, "_DENSITY_EXACT_WORK_CAP"),
             "preparation_cap": (atlas, "DENSITY_FAN_PREPARATION_WORK_CAP"),
             "box_cap": (fan, "_BOX_REFINEMENT_CAP")}


def fixture_cases():
    groups = ("binding_noise_canonical_v1", "density4_exact_limit_v1", "building_patch17_crowded_v1",
              "building_patch10_density4_v1", "wall_noise_top_join_density4_v1")
    cases = []
    for group in groups:
        for request in sorted((FIXTURES / group).rglob("decal_request*.json")):
            cases.append((request.parent.relative_to(FIXTURES).as_posix(), request.name, {}))
    # Одинаковые actual recorded-settings профили по обе стороны сравнения.
    for name in CAP_NAMES:
        cases.append(("building_patch10_density4_v1", "decal_request.json", {name: 0}))
    return tuple(cases)


def fixture_inputs(folder, request_name):
    root = FIXTURES / folder
    snapshot = kernel.AnalysisSnapshotCodecV1.loads_with_compatibility_receipt(
        (root / "analysis_snapshot.json").read_bytes())
    request = kernel.DecalRequestCodecV1.loads((root / request_name).read_bytes())
    return snapshot.record, request, snapshot.compatibility_receipt.positive_scaled_normal_compat_hits


@contextmanager
def adapter_policy(enabled, settings):
    owners = (binding, values)
    aliases = tuple(owner._density_decimal_envelope for owner in owners)
    legacy = values._density_decimal_envelope
    saved_caps = {name: getattr(owner, symbol) for name, (owner, symbol) in CAP_NAMES.items()}
    try:
        if enabled:
            for owner in owners:
                owner._density_decimal_envelope = adapter._density_decimal_envelope_v2
        for name, cap in settings.items():
            owner, symbol = CAP_NAMES[name]
            setattr(owner, symbol, cap)
        yield
    finally:
        for owner, alias in zip(owners, aliases, strict=True):
            owner._density_decimal_envelope = alias
        for name, cap in saved_caps.items():
            owner, symbol = CAP_NAMES[name]
            setattr(owner, symbol, cap)
        assert values._density_decimal_envelope is legacy


@contextmanager
def observed_budgets():
    budgets, initialize = [], atlas.DensityExactWorkBudget.__init__
    def remember(budget, exhausted, cap):
        initialize(budget, exhausted, cap)
        budgets.append(budget)
    atlas.DensityExactWorkBudget.__init__ = remember
    try:
        yield budgets
    finally:
        atlas.DensityExactWorkBudget.__init__ = initialize


def ordered_budgets(budgets):
    return [dict(cap=b.cap, shell_probes=b.shell_probes, order_steps=b.order_steps,
                 exhausted_type=b.exhausted.__name__) for b in budgets]


@contextmanager
def observed_adapter(enabled):
    """Наблюдатели делегируют исходные callbacks один раз; значений не подставляют."""
    stats = dict(calls=0, applied=0, not_applicable={}, recorded_inside_capture=0, raised=0)
    original, reader, record = adapter._density_decimal_envelope_v2, adapter.density_shell_v2, adapter.record_density_shell_candidate
    def called(expression, metric=None):
        stats["calls"] += 1
        try:
            return original(expression, metric)
        except Exception:
            stats["raised"] += 1
            raise
    def read(expression):
        result = reader(expression)
        if isinstance(result, DensityShellV2Candidate):
            stats["applied"] += 1
        else:
            counts = stats["not_applicable"]
            counts[result.code.value] = counts.get(result.code.value, 0) + 1
        return result
    def recorded(metric, candidate):
        if metric is not None and metric._density_exact_memo.shell_event_capture is not None:
            stats["recorded_inside_capture"] += 1
        return record(metric, candidate)
    if enabled:
        adapter._density_decimal_envelope_v2, adapter.density_shell_v2, adapter.record_density_shell_candidate = called, read, recorded
    try:
        yield stats
    finally:
        adapter._density_decimal_envelope_v2, adapter.density_shell_v2, adapter.record_density_shell_candidate = original, reader, record


def event_classification(events, stats, phase):
    if events:
        return "MEMO_BUNDLE_REPLAY" if not stats["recorded_inside_capture"] and phase == "memo_hit" else "RECORDED_ACCEPTED_VALUES"
    if not stats["calls"]:
        return "NO_ADAPTER_CALL"
    if stats["raised"] and not stats["applied"]:
        return "REFUSAL_BEFORE_ACCEPTED_VALUE"
    if not stats["applied"]:
        return "ONLY_NOT_APPLICABLE_VALUES"
    if not stats["recorded_inside_capture"]:
        return "ACCEPTED_OUTSIDE_FAN_CAPTURE_ONLY"
    raise AssertionError("DENSITY_SHELL_ACCEPTED_CAPTURE_LOST_EVENTS")


def run_policy(snapshot, request, enabled, settings, *, direction_binding_attachment=None):
    memo, collector, phases = _DensityExactMemo(), DensityShellEventCollector(), []
    for phase in ("cold", "memo_hit", "pickle_new_memo"):
        with observed_adapter(enabled) as stats, adapter_policy(enabled, settings):
            if phase == "pickle_new_memo":
                memo = pickle.loads(pickle.dumps(memo))
                assert not memo.fan_bindings and memo.shell_event_capture is None
            with observed_budgets() as budgets:
                result = kernel.compile_reference_envelopes(
                    snapshot, request, _density_exact_memo=memo, _density_shell_event_collector=collector,
                    _direction_binding_attachment=direction_binding_attachment)
            diagnostics = collector.diagnostics()
            assert tuple(item for item in result.diagnostics if item.outcome in SHELL_EVENT_OUTCOMES) == diagnostics
            assert pickle.loads(pickle.dumps(result)) == result
            phases.append(dict(phase=phase, result=result, events=diagnostics, budgets=ordered_budgets(budgets),
                               adapter_calls=stats, event_classification=event_classification(diagnostics, stats, phase)))
    return phases


def strip_events(result):
    diagnostics = tuple(item for item in result.diagnostics if item.outcome not in SHELL_EVENT_OUTCOMES)
    compilation = result.compilation
    if compilation is not None:
        compilation = replace(compilation, diagnostics=tuple(
            item for item in compilation.diagnostics if item.outcome not in SHELL_EVENT_OUTCOMES))
    return replace(result, diagnostics=diagnostics, compilation=compilation)


def packed(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False).encode("ascii")


def _identity(value):
    if is_dataclass(value) and not isinstance(value, type):
        for name in ("envelope_spec_id", "seed_id", "hidden_support_id", "corner_relation_id", "source_seed_id"):
            if hasattr(value, name):
                return type(value).__name__ + ":" + str(getattr(value, name))
    return packed(exact_tree(value)).decode("ascii")


def exact_tree(value):
    """Все поля и raw Decimal tuple; codec отдельно проверяет canonical output."""
    if isinstance(value, Decimal):
        part = value.as_tuple()
        return {"decimal_tuple": [part.sign, list(part.digits), part.exponent]}
    if isinstance(value, Enum):
        return {"enum": type(value).__name__, "value": value.value}
    if is_dataclass(value) and not isinstance(value, type):
        return {"record": type(value).__name__, "fields": {f.name: exact_tree(getattr(value, f.name)) for f in fields(value)}}
    if isinstance(value, (set, frozenset)):
        return {"set": [exact_tree(item) for item in sorted(value, key=_identity)]}
    if isinstance(value, (tuple, list)):
        return [exact_tree(item) for item in value]
    if isinstance(value, dict):
        if any(type(key) is not str for key in value):
            raise TypeError("DENSITY_SHELL_COMPILE_NON_STRING_MAPPING_KEY")
        return {key: exact_tree(item) for key, item in value.items()}
    if value is None or type(value) in (str, int, bool, float):
        return value
    raise TypeError("DENSITY_SHELL_COMPILE_UNSUPPORTED_TYPE:" + type(value).__name__)


def decimal_rows(value, path="result"):
    if isinstance(value, Decimal):
        return {path: value}
    rows = {}
    if is_dataclass(value) and not isinstance(value, type):
        for field in fields(value):
            rows.update(decimal_rows(getattr(value, field.name), path + "." + field.name))
    elif isinstance(value, (tuple, list, set, frozenset)):
        sequence = sorted(value, key=_identity) if isinstance(value, (set, frozenset)) else value
        for index, item in enumerate(sequence):
            label = _identity(item) if isinstance(value, (set, frozenset)) else str(index)
            rows.update(decimal_rows(item, path + "[" + label + "]"))
    return rows


def decimal_deltas(left, right):
    before, after = decimal_rows(left), decimal_rows(right)
    result = []
    for path in sorted(set(before) | set(after)):
        old, new = before.get(path), after.get(path)
        if old is not None and new is not None and old.as_tuple() == new.as_tuple():
            continue
        numeric_equal = old is not None and new is not None and old == new
        result.append(dict(path=path, old=None if old is None else str(old), new=None if new is None else str(new),
                           old_raw=exact_tree(old), new_raw=exact_tree(new), numeric_equal=numeric_equal,
                           outcome="DENSITY_COMPILE_DECIMAL_QUANTUM_CHANGED" if numeric_equal
                           else "DENSITY_COMPILE_DECIMAL_VALUE_CHANGED"))
    return result


def tree_deltas(left, right, path="result"):
    if left == right:
        return []
    if type(left) is dict and type(right) is dict and set(left) == set(right):
        return [row for key in sorted(left) for row in tree_deltas(left[key], right[key], path + "." + key)]
    if type(left) is list and type(right) is list and len(left) == len(right):
        return [row for at, (old, new) in enumerate(zip(left, right, strict=True))
                for row in tree_deltas(old, new, path + "[" + str(at) + "]")]
    outcome = "DENSITY_COMPILE_AUTHORITY_ID_CHANGED" if path.endswith("authority_id") else "DENSITY_COMPILE_OUTPUT_CHANGED"
    return [dict(path=path, old=left, new=right, outcome=outcome)]


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _save_blob(directory, data):
    digest = _digest(data)
    if directory is not None:
        target = directory / (digest + ".json.xz")
        if not target.exists():
            target.write_bytes(lzma.compress(data, preset=1))
    return digest


def phase_summary(phase, directory):
    result, functional = phase["result"], strip_events(phase["result"])
    full_bytes, functional_bytes = kernel.canonical_json_bytes(result), kernel.canonical_json_bytes(functional)
    events = to_canonical_data(phase["events"])
    return dict(phase=phase["phase"], outcome=result.outcome.value, compilation_present=result.compilation is not None,
                full_canonical_sha256=_save_blob(directory, full_bytes),
                full_raw_sha256=_save_blob(directory, packed(exact_tree(result))),
                functional_canonical_sha256=_save_blob(directory, functional_bytes),
                functional_raw_sha256=_save_blob(directory, packed(exact_tree(functional))),
                events=events, event_rows=len(events),
                event_code_rows={code.value: sum(row["outcome"] == code.value for row in events) for code in SHELL_EVENT_OUTCOMES},
                event_serialized_bytes=len(kernel.canonical_json_bytes(phase["events"])),
                full_canonical_bytes=len(full_bytes), functional_canonical_bytes=len(functional_bytes),
                ledger_canonical_bytes_delta=len(full_bytes) - len(functional_bytes), budgets=phase["budgets"],
                adapter_calls=phase["adapter_calls"], event_classification=phase["event_classification"])


def compare_policies(old, new, directory=None):
    legacy = [phase_summary(p, directory) for p in old]
    candidate = [phase_summary(p, directory) for p in new]
    comparisons = []
    for before, after in zip(legacy, candidate, strict=True):
        comparisons.append(dict(phase=before["phase"], budgets_equal=before["budgets"] == after["budgets"],
                                canonical_equal=before["functional_canonical_sha256"] == after["functional_canonical_sha256"],
                                raw_equal=before["functional_raw_sha256"] == after["functional_raw_sha256"]))
    stable = {}
    for name, phases in (("legacy", legacy), ("candidate", candidate)):
        stable[name] = all((p["full_canonical_sha256"], p["full_raw_sha256"], p["events"]) ==
                           (phases[0]["full_canonical_sha256"], phases[0]["full_raw_sha256"], phases[0]["events"])
                           for p in phases[1:])
    return dict(legacy=legacy, candidate=candidate, comparisons=comparisons, stable=stable,
                decimal_deltas=decimal_deltas(strip_events(old[0]["result"]), strip_events(new[0]["result"])),
                full_output_deltas=tree_deltas(exact_tree(strip_events(old[0]["result"])),
                                              exact_tree(strip_events(new[0]["result"]))))


def check_case(folder, request_name, settings=None, directory=None, *, direction_binding_attachment=None):
    settings = {} if settings is None else settings
    snapshot, request, compatibility_hits = fixture_inputs(folder, request_name)
    old = run_policy(snapshot, request, False, settings, direction_binding_attachment=direction_binding_attachment)
    new = run_policy(snapshot, request, True, settings, direction_binding_attachment=direction_binding_attachment)
    result = compare_policies(old, new, directory)
    return dict(case=folder + "/" + request_name, settings=settings, compatibility_hits=compatibility_hits,
                snapshot_sha256=_digest(kernel.canonical_json_bytes(snapshot)),
                request_sha256=_digest(kernel.canonical_json_bytes(request)), **result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    reports = []
    for folder, request_name, settings in fixture_cases():
        report = check_case(folder, request_name, settings, args.out)
        reports.append(report)
        (args.out / ("case-" + _digest(packed((folder, request_name, settings))) + ".json")).write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf8")
        print(json.dumps(dict(case=report["case"], settings=settings, stable=report["stable"],
                              comparisons=report["comparisons"], decimal_deltas=len(report["decimal_deltas"])),
                         sort_keys=True), flush=True)
    failed = any(not all(r["stable"].values()) or not all(
        p["budgets_equal"] and p["canonical_equal"] and p["raw_equal"] for p in r["comparisons"]) for r in reports)
    summary = dict(schema="cftuv.actual-shell-compile-differential.v1", cases=reports,
                   policy="candidate adapter only; source legacy evaluator unchanged",
                   stripped_outcomes=[code.value for code in SHELL_EVENT_OUTCOMES],
                   materialized_field_claim=None, failed=failed)
    (args.out / "report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf8")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
