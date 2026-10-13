"""Fresh whole compilation A/B: candidate raw operation, defaults не меняются."""

from __future__ import annotations

import argparse
from collections import Counter
from contextlib import contextmanager
from dataclasses import replace
from fractions import Fraction
import hashlib
import json
import lzma
import os
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "kernel/src"))
sys.path.insert(0, str(ROOT / "tools"))
sys.set_int_max_str_digits(0)

import cftuv_envelope as kernel
from cftuv_envelope.reference import compile as compiler, adaptive_density_fan as fan, adaptive_density_atlas as atlas
from cftuv_envelope.reference.density_compile_v1 import attach_density_compilation_v1
from cftuv_envelope.reference import density_compile_v1 as candidate
from cftuv_envelope.reference.density_construct_v1 import ExpandLimitsV1
from cftuv_envelope.reference.density_oracle_v1 import DensityOracleCanonV1, DensityOracleLimitsV1, DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1
from cftuv_envelope.reference.density_raw_frontend_v1 import RawDensityFrontendV1
from cftuv_envelope.reference.density_scalar_v2 import DENSITY_SOURCE_CONSTRUCTION_V1
from cftuv_envelope.reference.density_shell_v2 import DENSITY_SHELL_CANON_V2_CANDIDATE
from cftuv_envelope.reference.density_shell_events import DensityShellEventCollector, DENSITY_CANON_EVENT_OUTCOMES
from cftuv_envelope.reference.metric import _DensityExactMemo
from cftuv_envelope.reference.contracts import ReferenceDiagnosticSeverity, ReferenceEvaluationDiagnosticV1, ReferenceOutcome
from density_raw_compile_audit import sources, decode_input, raw_wire, source_hashes
from density_raw_wire import read_raw_spec, _record
from cftuv_envelope.reference import density_oracle_v1 as result_facts
from cftuv_envelope.reference.density_attach_v1 import _validate_success, _validate_failure
from density_shell_compile_diff import adapter_policy, observed_budgets, ordered_budgets, packed, exact_tree, tree_deltas, decimal_deltas

CANONS = DensityOracleCanonV1(DENSITY_SOURCE_CONSTRUCTION_V1, DENSITY_SHELL_CANON_V2_CANDIDATE,
                            DENSITY_PRINCIPAL_ANGLE_A2_FIXED160_V1)


def limits_for(settings):
    return DensityOracleLimitsV1(settings.get("preparation_cap", atlas.DENSITY_FAN_PREPARATION_WORK_CAP),
        settings.get("search_cap", fan._DENSITY_EXACT_WORK_CAP), settings.get("box_cap", fan._BOX_REFINEMENT_CAP),
        fan._RESOURCE_HEIGHT_CAP, ExpandLimitsV1(4096, 16, 65536))


class ObservedFrontend(RawDensityFrontendV1):
    def __init__(self, limits, rows, timings):
        super().__init__(CANONS, limits, window_law=compiler.FAN_WINDOW_LAW)
        self.rows, self.timings = rows, timings

    def evaluate(self, raw):
        started = time.perf_counter()
        try:
            result = super().evaluate(raw)
        finally:
            self.timings["evaluate_seconds"] += time.perf_counter()-started
        self.rows.append({"input": raw_wire(raw), "result": raw_wire(result)})
        return result


@contextmanager
def candidate_attachment(settings, facts, articles, timings):
    original, extract = compiler._attach_direction_bindings, candidate.extract_raw_density_spec_input
    def extracted(*args):
        started = time.perf_counter()
        try:
            return extract(*args)
        finally:
            timings["extract_seconds"] += time.perf_counter()-started
            timings["extract_calls"] += 1
    def attached(compilation, **kwargs):
        if not compiler._is_explicit_density(compilation):
            return original(compilation, **kwargs)
        # Один frontend, его metric/sign/attempt memo общие для всех sorted specs.
        started = time.perf_counter()
        try:
            frontend = ObservedFrontend(limits_for(settings), facts, timings)
            return attach_density_compilation_v1(compilation, frontend, ordered_work=articles, **kwargs)
        finally:
            timings["attach_seconds"] += time.perf_counter()-started
    compiler._attach_direction_bindings = attached
    candidate.extract_raw_density_spec_input = extracted
    try:
        yield
    finally:
        compiler._attach_direction_bindings = original
        candidate.extract_raw_density_spec_input = extract


def without_named_events(result):
    def selected(diagnostics):
        return tuple(item for item in diagnostics if item.outcome not in DENSITY_CANON_EVENT_OUTCOMES)
    compilation = result.compilation
    if compilation is not None:
        compilation = replace(compilation, diagnostics=selected(compilation.diagnostics))
    return replace(result, diagnostics=selected(result.diagnostics), compilation=compilation)


def _blob(directory, data):
    digest = hashlib.sha256(data).hexdigest()
    path = directory / (digest + ".json.xz")
    if not path.exists():
        path.write_bytes(lzma.compress(data, preset=1))
    return digest


def summarize(result, budgets, facts, articles, directory, timings):
    events = tuple(item for item in result.diagnostics if item.outcome in DENSITY_CANON_EVENT_OUTCOMES)
    functional = without_named_events(result)
    return {"outcome": result.outcome.value, "compilation_present": result.compilation is not None,
        "full_canonical_sha256": _blob(directory, kernel.canonical_json_bytes(result)),
        "full_raw_sha256": _blob(directory, packed(exact_tree(result))),
        "functional_canonical_sha256": _blob(directory, kernel.canonical_json_bytes(functional)),
        "functional_raw_sha256": _blob(directory, packed(exact_tree(functional))),
        "events": exact_tree(events), "event_counts": dict(Counter(item.outcome.value for item in events)),
        "event_serialized_bytes": len(kernel.canonical_json_bytes(events)),
        "budgets": ordered_budgets(budgets), "raw_specs": len(facts), "timings": dict(timings),
        "typed_facts_sha256": _blob(directory, packed(facts)), "ordered_work_articles": raw_wire(tuple(articles))}


def run_one(inputs, settings, enabled, directory):
    snapshot, request, domain, issues = inputs
    facts, articles = [], []
    timings, started = Counter(), time.perf_counter()
    with adapter_policy(True, settings), observed_budgets() as budgets:
        try:
            if enabled:
                with candidate_attachment(settings, facts, articles, timings):
                    result = kernel.compile_reference_envelopes(snapshot, request, domain, snapshot_issues=issues,
                        _density_exact_memo=_DensityExactMemo(), _density_shell_event_collector=DensityShellEventCollector())
            else:
                result = kernel.compile_reference_envelopes(snapshot, request, domain, snapshot_issues=issues,
                    _density_exact_memo=_DensityExactMemo(), _density_shell_event_collector=DensityShellEventCollector())
        except Exception as error:
            timings["compile_seconds"] = time.perf_counter()-started
            return None, {"raised": type(error).__name__, "detail": str(error), "budgets": ordered_budgets(budgets),
                          "raw_specs": len(facts), "timings": dict(timings), "typed_facts_sha256": _blob(directory, packed(facts))}
        timings["compile_seconds"] = time.perf_counter()-started
        return result, summarize(result, budgets, facts, articles, directory, timings)


def compare(inputs, settings, directory):
    old, before = run_one(inputs, settings, False, directory)
    new, after = run_one(inputs, settings, True, directory)
    comparison = {"before": before, "after": after, "settings": settings,
                  "budgets_equal": before["budgets"] == after["budgets"]}
    if old is None or new is None:
        comparison["functional_equal"] = False
        return comparison
    comparison.update(full_canonical_equal=before["full_canonical_sha256"] == after["full_canonical_sha256"],
        full_raw_equal=before["full_raw_sha256"] == after["full_raw_sha256"],
        functional_equal=before["functional_canonical_sha256"] == after["functional_canonical_sha256"]
                         and before["functional_raw_sha256"] == after["functional_raw_sha256"],
        decimal_deltas=decimal_deltas(old, new),
        functional_deltas=tree_deltas(exact_tree(without_named_events(old)), exact_tree(without_named_events(new))),
        full_deltas=tree_deltas(exact_tree(old), exact_tree(new)))
    return comparison


def _read_blob(directory, digest):
    payload = lzma.decompress((directory / (digest + ".json.xz")).read_bytes())
    if hashlib.sha256(payload).hexdigest() != digest:
        raise ValueError("RAW_COMPILE_OUTPUT_BLOB_SHA_MISMATCH")
    return json.loads(payload)


def _events(tree):
    fields = tree["fields"]
    compilation = fields["compilation"]
    return {"result": fields["diagnostics"],
            "compilation": None if compilation is None else compilation["fields"]["diagnostics"]}


def _event_code(value):
    return value["fields"]["outcome"]["value"]


def _expected_source_events(facts):
    message = f"construction={CANONS.construction} shell={CANONS.shell} angle={CANONS.angle}"
    ids = [row["input"]["spec_id"] for row in facts]
    if len(ids) != len(set(ids)):
        raise ValueError("RAW_COMPILE_DUPLICATE_EVALUATED_SPEC")
    return exact_tree(tuple(ReferenceEvaluationDiagnosticV1(
        ReferenceOutcome.DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED, ReferenceDiagnosticSeverity.INFO,
        message, envelope_spec_id=identity) for identity in sorted(ids)))


def _stored_fact(cls, data, ratios=(), children=None):
    if data is None:
        return None
    converters = {name: lambda value: Fraction(*value) for name in ratios}
    converters.update({} if children is None else children)
    return _record(cls, data, converters)


def _stored_interval(data):
    value = result_facts.DecimalValueFactsV1
    return _stored_fact(result_facts.DecimalIntervalFactsV1, data, children={
        name: lambda row: _stored_fact(value, row) for name in ("lower", "upper", "absolute_error_bound")})


def _stored_certificate(data):
    return _stored_fact(result_facts.BindingCertificateFactsV1, data, children={
        "ideal_window_lower_slope_envelope": _stored_interval, "ideal_window_upper_slope_envelope": _stored_interval,
        "certified_window_width_lower_bound": lambda row: _stored_fact(result_facts.DecimalValueFactsV1, row)})


def _stored_window(data):
    ratios = ("termination_lower_slope", "termination_upper_slope", "certified_termination_width",
              "admissible_lower_outward", "admissible_lower_inward", "admissible_upper_inward", "admissible_upper_outward")
    if "pieces" not in data:
        return _stored_fact(result_facts.LocalWindowFactsV1, data, ratios,
                            {"full_lower_slope_envelope": _stored_interval, "full_upper_slope_envelope": _stored_interval})
    def piece(row):
        return _stored_fact(result_facts.ChartPieceFactsV1, row, children={
            "lower_slope_envelope": _stored_interval, "upper_slope_envelope": _stored_interval})
    return _stored_fact(result_facts.AtlasWindowFactsV1, data, ratios,
                        {"pieces": lambda rows: tuple(piece(row) for row in rows)})


def _stored_authority(data):
    if data is None:
        return None
    if "ray_law" in data:
        return _stored_fact(result_facts.CanonicalFanAuthorityFactsV1, data, ("canonical_reflex_excess_over_pi",))
    return _stored_fact(result_facts.AdaptiveFanAuthorityFactsV1, data, children={
        "ordinal_windows": lambda rows: tuple(_stored_window(row) for row in rows),
        "previous_height_witness": lambda row: _stored_fact(result_facts.FareyHeightWitnessFactsV1, row)})


def _stored_result(data):
    converters = {"binding_certificates": lambda rows: tuple(_stored_certificate(row) for row in rows),
        "authority": _stored_authority,
        "count_lift": lambda row: _stored_fact(result_facts.CountLiftFactsV1, row, ("evaluation_turn_cosine_squared",)),
        "noise_record": lambda row: _stored_fact(result_facts.NoiseRecordFactsV1, row, (
            "canonical_reflex_excess_over_pi", "evaluation_turn_cosine_squared", "binding_lateral_offset_gram_squared_bound",
            "edge_direction_sine_squared_bound", "direction_sine_bound")),
        "canonical_subturn_authority": lambda row: _stored_fact(result_facts.CanonicalSubturnAuthorityFactsV1, row, (
            "canonical_reflex_excess_over_pi", "canonical_subturn_over_pi", "raw_residual_upper_bound_radians")),
        "law_refusals": lambda row: _stored_fact(result_facts.CanonicalLawRefusalFactsV1, row),
        "failure": lambda row: _stored_fact(result_facts.DensityFailureFactsV1, row)}
    for name, cls in (("canon_usage", result_facts.CanonUsageFactsV1),
                      ("ordered_work_articles", result_facts.DensityPriceArticleV1),
                      ("shell_events", result_facts.DensityShellEventFactsV1)):
        converters[name] = lambda rows, cls=cls: tuple(_stored_fact(cls, row) for row in rows)
    return _record(result_facts.RawDensitySpecResultV1, data, converters)


def validate_events(row, directory):
    if "raised" in row["before"] or "raised" in row["after"]:
        return ["INTERNAL_EXCEPTION"]
    old = _events(_read_blob(directory, row["before"]["full_raw_sha256"]))
    new = _events(_read_blob(directory, row["after"]["full_raw_sha256"]))
    facts = _read_blob(directory, row["after"]["typed_facts_sha256"])
    expected, failures = _expected_source_events(facts), []
    if len(facts) != row["after"]["raw_specs"]:
        failures.append("TYPED_FACT_COUNT_CHANGED")
    for fact in facts:
        try:
            raw, result = read_raw_spec(fact["input"]), _stored_result(fact["result"])
            _validate_failure(result) if result.outcome != "OK" else _validate_success(raw, result)
        except (TypeError, ValueError) as error:
            failures.append("TYPED_RESULT_SHAPE:" + str(error))
    shell_codes = {value.value for value in DENSITY_CANON_EVENT_OUTCOMES[:3]}
    source_code = DENSITY_SOURCE_CONSTRUCTION_V1
    for location in old:
        before, after = old[location], new[location]
        if (before is None) != (after is None):
            failures.append(location + ":COMPILATION_PRESENCE_CHANGED")
        if before is None or after is None:
            continue
        if [_ for _ in before if _event_code(_) in shell_codes] != [_ for _ in after if _event_code(_) in shell_codes]:
            failures.append(location + ":SHELL_EVENTS_CHANGED")
        if any(_event_code(value) == source_code + "_APPLIED" for value in before):
            failures.append(location + ":UNEXPECTED_PREVIOUS_SOURCE_EVENT")
        if [value for value in after if _event_code(value) == source_code + "_APPLIED"] != expected:
            failures.append(location + ":SOURCE_EVENTS_NOT_EXACT_EVALUATED_SPEC_PREFIX")
    return failures


def validate_records(path, name="event-validation"):
    if Path(name).name != name or not name:
        raise ValueError("Raw compile validation receipt name must be a filename")
    counts, timing, samples, failure_kinds = Counter(), Counter(), {}, Counter()
    with (path / (name+"-failures.jsonl")).open("wb") as output:
        for line in (path / "records.jsonl").open("rb"):
            row = json.loads(line)
            failures = validate_events(row, path / "blobs")
            counts["calls"] += 1
            counts["exact_event_transport"] += not failures
            counts["functional_equal"] += row["functional_equal"]
            counts["budgets_equal"] += row["budgets_equal"]
            counts["decimal_deltas"] += len(row.get("decimal_deltas", ()))
            counts["typed_results_checked"] += row["after"]["raw_specs"]
            if failures:
                failure_kinds.update(failures)
                output.write(packed({"run": row["run"], "input": row["input"], "failures": failures}) + b"\n")
            for policy in ("before", "after"):
                for name, value in row[policy].get("timings", {}).items():
                    key = policy + ":" + name
                    timing[key] += value
                    samples.setdefault(key, []).append(value)
    percentiles = {key: {"p50": sorted(values)[len(values)//2],
                         "p95": sorted(values)[min(len(values)-1, int(len(values)*.95))], "max": max(values)}
                   for key, values in samples.items()}
    accepted = counts["calls"] == counts["exact_event_transport"] == counts["functional_equal"] == counts["budgets_equal"]
    receipt = {"scope": "Exact old A1 shell events and one source canon event per actually evaluated spec; both result and compilation diagnostics. No IDs or value wires whitelisted.",
        "pid": os.getpid(), "accepted": accepted, "counts": dict(counts), "failure_kinds": dict(failure_kinds),
        "timing_totals": dict(timing), "timing_percentiles": percentiles,
        "records_sha256": hashlib.sha256((path / "records.jsonl").read_bytes()).hexdigest()}
    (path / (name+".json")).write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({"accepted": accepted, "counts": dict(counts), "failure_kinds": dict(failure_kinds)}), flush=True)
    return int(not accepted)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--corpus-root", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--validate-records", type=Path)
    parser.add_argument("--validation-name", default="event-validation")
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    if args.validate_records is not None:
        return validate_records(args.validate_records, args.validation_name)
    if args.corpus_root is None or args.out is None:
        parser.error("--corpus-root and --out are required for the live compile gate")
    args.out.mkdir(parents=True, exist_ok=False)
    directory = args.out / "blobs"
    directory.mkdir()
    manifest = json.loads((args.corpus_root / "a1-compile-combined.json").read_text())
    before, counts, started = source_hashes(), Counter(), time.perf_counter()
    with (args.out / "records.jsonl").open("wb") as stream:
        for index, source in enumerate(sources(args.corpus_root, manifest)):
            if args.limit is not None and index >= args.limit:
                break
            inputs = decode_input(source["path"], source["input"])
            checked = compare(inputs, {}, directory)
            row = {"run": source["run"], "input": source["input"], "version": source["version"], **checked}
            stream.write(packed(row) + b"\n")
            stream.flush()
            counts["calls"] += 1
            counts["functional_equal"] += checked["functional_equal"]
            counts["budgets_equal"] += checked["budgets_equal"]
            counts["full_raw_equal"] += checked.get("full_raw_equal", False)
            counts["raw_specs"] += checked["after"]["raw_specs"]
            counts["candidate_internal_exception"] += "raised" in checked["after"]
            if index % 100 == 0:
                print(json.dumps(dict(counts)), flush=True)
    after = source_hashes()
    changed = sorted(key for key in set(before) | set(after) if before.get(key) != after.get(key))
    receipt = {"scope": "Whole fresh explicit Density compile vs previous A1 candidate; only four enumerated named events separated for functional comparison. Full outputs, raw Decimal quantum, events and ordered budgets retained. No Blender/default switch.",
        "pid": os.getpid(), "counts": dict(counts), "seconds": time.perf_counter()-started,
        "source_hashes": before, "source_changed": changed, "partial": args.limit is not None,
        "corpus_manifest_sha256": hashlib.sha256((args.corpus_root / "a1-compile-combined.json").read_bytes()).hexdigest()}
    (args.out / "receipt.json").write_text(json.dumps(receipt, indent=2), encoding="utf-8")
    print(json.dumps({"counts": dict(counts), "source_changed": changed}), flush=True)
    return int(bool(changed) or counts["functional_equal"] != counts["calls"] or counts["budgets_equal"] != counts["calls"])


if __name__ == "__main__":
    raise SystemExit(main())
