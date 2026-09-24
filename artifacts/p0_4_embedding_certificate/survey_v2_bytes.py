"""Hash every legacy V2 metric byte record in the frozen fixture universe."""

from __future__ import annotations

from collections import Counter
import functools
from hashlib import sha256
import inspect
import json
import os
from pathlib import Path
import sys

import pytest

from cftuv_envelope.codec import RationalAffinePlanarMetricCodecV2
from cftuv_envelope.planar_metric import build_rational_affine_planar_metric


class _CurrentItem:
    nodeid = "outside-pytest-item"
    collected = 0

    def pytest_runtest_setup(self, item):
        self.nodeid = item.nodeid

    def pytest_collection_finish(self, session):
        self.collected = len(session.items)


def main() -> int:
    repository = Path(os.environ["CFTUV_SURVEY_REPOSITORY"]).resolve()
    kernel = repository / "kernel"
    current = _CurrentItem()
    records = []
    ordinal_by_node = Counter()
    original_builder = build_rational_affine_planar_metric
    signature = inspect.signature(original_builder)

    @functools.wraps(original_builder)
    def audited_builder(*args, **kwargs):
        bound = signature.bind(*args, **kwargs)
        bound.apply_defaults()
        bound.arguments["source_vertices"] = tuple(
            bound.arguments["source_vertices"]
        )
        bound.arguments["source_faces"] = tuple(bound.arguments["source_faces"])
        metric = original_builder(**bound.arguments)
        ordinal_by_node[current.nodeid] += 1
        encoded = RationalAffinePlanarMetricCodecV2.dumps(metric)
        records.append(
            {
                "case": (
                    f"{current.nodeid}::metric-call-"
                    f"{ordinal_by_node[current.nodeid]}"
                ),
                "byte_sha256": sha256(encoded).hexdigest(),
                "byte_count": len(encoded),
            }
        )
        return metric

    class _ReplaceImportedBuilders:
        def pytest_collection_modifyitems(self):
            for module in tuple(sys.modules.values()):
                namespace = getattr(module, "__dict__", None)
                if namespace is None:
                    continue
                for name, value in tuple(namespace.items()):
                    if value is original_builder:
                        namespace[name] = audited_builder

    tests = (
        kernel / "tests/test_building_002_point_contact_fixture.py",
        kernel / "tests/test_grid_wiring.py",
        kernel / "tests/test_near_planar_policy.py",
        kernel / "tests/test_near_planar_snapshot_validation.py",
        kernel / "tests/test_planar_metric_v2.py",
        repository / "tests/test_envelope_host_adapter.py",
    )
    status = pytest.main(
        [*(str(item) for item in tests), "-q"],
        plugins=[current, _ReplaceImportedBuilders()],
    )
    canonical = json.dumps(
        records, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    report = {
        "schema": "P0_4_V2_BYTES_SURVEY_V1",
        "repository_head": os.environ.get("CFTUV_SURVEY_LABEL", "unknown"),
        "pytest_exit_code": int(status),
        "pytest_cases_collected": current.collected,
        "metric_call_count": len(records),
        "total_encoded_bytes": sum(item["byte_count"] for item in records),
        "case_bytes_receipt_sha256": sha256(canonical).hexdigest(),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return int(bool(status))


if __name__ == "__main__":
    raise SystemExit(main())
