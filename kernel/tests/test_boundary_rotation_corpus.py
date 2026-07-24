from __future__ import annotations

import json
from pathlib import Path


FIXTURE_ROOT = (
    Path(__file__).resolve().parents[1]
    / "fixtures"
    / "session_c_r2c_boundary_rotation_v1"
)


def test_boundary_rotation_manifest_is_complete_and_executable():
    manifest = json.loads(
        (FIXTURE_ROOT / "manifest.json").read_text(encoding="utf-8")
    )
    cases = manifest["cases"]
    assert manifest["session"] == (
        "SESSION_C_R2C_REGULARIZED_BOUNDARY_ROTATION_SYSTEM"
    )
    assert len(cases) == 20
    assert {item["case_id"] for item in cases} == {
        f"R2C-{index:03d}" for index in range(1, 21)
    }
    assert all(item["fixture"] and item["expected"] for item in cases)
    assert all("::test_" in item["pytest_nodeid"] for item in cases)
