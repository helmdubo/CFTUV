"""CLI для воспроизводимого S-WF0 research run."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

try:
    from .fixtures import all_fixtures
    from .harness import METHODS, run_all, write_json
except ImportError:  # Direct script execution documented in README.
    from fixtures import all_fixtures
    from harness import METHODS, run_all, write_json


def _summary_rows(payload):
    for fixture in payload["fixtures"]:
        key = fixture["fixture"]
        if fixture["variant"]:
            key += f"/{fixture['variant']}"
        for method in METHODS:
            record = fixture["methods"][method]
            yield {
                "fixture": key,
                "method": method,
                "status": record["status"],
                "coverage_rate": record["metrics"]["coverage_rate"],
                "width_error_p95": record["metrics"]["width_error_p95"],
                "width_error_max": record["metrics"]["width_error_max"],
                "owner_error_rate": record["metrics"]["owner_error_rate"],
                "source_s_error_p95": record["metrics"]["source_s_error_p95"],
                "event_edit": record["metrics"]["event_sequence_edit_distance"],
                "event_alpha_error_max": record["metrics"]["event_alpha_error_max"],
                "locus_f1": record["metrics"]["cut_freeze_locus_f1"],
                "double_cover_count": record["metrics"]["double_cover_count"],
                "boundary_leak_count": record["metrics"]["boundary_leak_count"],
                "compile_ms": record["compile_ms"],
                "extract_ms": record["extract_ms"],
            }


def _event_rows(payload):
    for fixture in payload["fixtures"]:
        key = fixture["fixture"]
        if fixture["variant"]:
            key += f"/{fixture['variant']}"
        for method in METHODS:
            record = fixture["methods"][method]
            for event_index, event in enumerate(record["field_events"]):
                yield {
                    "fixture": key,
                    "method": method,
                    "event_index": event_index,
                    "kind": event["kind"],
                    "alpha": event["alpha"],
                    "source": "PL_LABELED_FIELD",
                }
            for event_index, event in enumerate(record["native_events"]):
                yield {
                    "fixture": key,
                    "method": method,
                    "event_index": event_index,
                    "kind": event["kind"],
                    "alpha": event["alpha"],
                    "source": "CURRENT_RAIL_NATIVE",
                }
        skeleton = fixture["straight_skeleton_2d"]
        for event_index, event in enumerate(skeleton.get("events", ())):
            yield {
                "fixture": key,
                "method": "STRAIGHT_SKELETON_2D",
                "event_index": event_index,
                "kind": "SKELETON_NODE",
                "alpha": event["alpha"],
                "source": "STRAIGHT_SKELETON_NATIVE",
            }


def _retriangulation_rows(payload):
    comparison = payload["retriangulation"]
    if comparison["status"] != "ok":
        return
    for method, metrics in comparison["methods"].items():
        yield {"method": method, **metrics}


def _write_csv(path, rows):
    rows = list(rows)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "artifacts" / "s_wf0",
    )
    parser.add_argument("--fixture", action="append", default=[])
    args = parser.parse_args()
    fixtures = all_fixtures()
    if args.fixture:
        selected = set(args.fixture)
        fixtures = tuple(
            fixture
            for fixture in fixtures
            if fixture.name in selected or fixture.key in selected
        )
        missing = selected.difference({fixture.name for fixture in fixtures}).difference(
            {fixture.key for fixture in fixtures}
        )
        if missing:
            parser.error(f"unknown fixture(s): {', '.join(sorted(missing))}")
    payload = run_all(fixtures)
    write_json(args.output_dir / "results.json", payload)
    _write_csv(args.output_dir / "summary.csv", _summary_rows(payload))
    _write_csv(args.output_dir / "events.csv", _event_rows(payload))
    _write_csv(
        args.output_dir / "retriangulation.csv",
        _retriangulation_rows(payload),
    )
    print(f"[S-WF0] wrote {args.output_dir}", flush=True)


if __name__ == "__main__":
    main()
