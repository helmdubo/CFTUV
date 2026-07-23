from __future__ import annotations

import hashlib
import json
from pathlib import Path


FIXTURE_ROOT = Path(__file__).parents[1] / "fixtures" / "session_c_planar_v1"
REQUIRED_CASE_IDS = {
    "SC-P01-STRAIGHT-STRIP",
    "SC-P02-PHYSICAL-CAPS",
    "SC-P03-ANGULAR-K0",
    "SC-P04-ANGULAR-K1",
    "SC-P05-ANGULAR-K2",
    "SC-P06-ANGULAR-INTERNALIZED",
    "SC-P07-DATA-CHAIN-SPLIT-MERGE",
    "SC-P08-ENDPOINT-BOUNDARY-SLIDE",
    "SC-P09-HOLE-INTERIOR-SPLIT",
    "SC-P10-CONCAVE-OUTER-SPLIT",
    "SC-P11-MULTIPLE-PCHAINS-ONE-DOMAIN",
    "SC-P12-SEAM-SELF-TWO-USES",
    "SC-P13-PHYSICAL-SEAM-A-B",
    "SC-P14-T-JUNCTION",
    "SC-P15-X-JUNCTION",
    "SC-P16-Y-JUNCTION",
    "SC-P17-CROSS-PATCH-JUNCTION",
    "SC-P18-CLOSED-CHAIN",
    "SC-P19-VERY-SHORT-SOURCE",
    "SC-P20-SIMULTANEOUS-EXACT-CONTACTS",
    "SC-P21-RETRIANGULATED-SURFACE",
    "SC-P22-UNIFORM-SCALE-TRANSLATION",
    "SC-P23-REVERSED-STORAGE-SEMANTIC-ORIENTATION",
}


def _fixture_sha256(path: Path) -> str:
    content = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(content).hexdigest()


def test_session_c_coordinate_corpus_is_complete_and_declarative():
    manifest = json.loads((FIXTURE_ROOT / "manifest.json").read_text(encoding="utf-8"))
    payload = json.loads((FIXTURE_ROOT / manifest["case_file"]).read_text(encoding="utf-8"))
    cases = payload["cases"]

    assert manifest["case_count"] == len(cases) == 23
    assert {item["id"] for item in cases} == REQUIRED_CASE_IDS
    assert {item["expected_profile_k"] for item in cases if item["expected_profile_k"] is not None} == {0, 1, 2}
    for item in cases:
        assert item["analysis_snapshot"]
        assert item["decal_request"]
        assert "expected_raw_topology" in item
        assert "holes" in item["expected_raw_topology"]
        assert item["expected_silhouette_lineage"]
        assert item["expected_contributor_count"] >= 1
        assert isinstance(item["expected_named_outcomes"], list)


def test_session_c_corpus_hash_manifest_is_complete_and_newline_stable():
    manifest_path = FIXTURE_ROOT / "fixture_hash_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["hash_law"] == "SHA256_UTF8_LF_NORMALIZED_V1"
    assert manifest["source_commit"] == "a18bd486ba198d6a14b81b703184c2402681576e"
    actual = {
        path.relative_to(FIXTURE_ROOT).as_posix(): _fixture_sha256(path)
        for path in sorted(FIXTURE_ROOT.rglob("*"))
        if path.is_file() and path != manifest_path
    }
    assert actual == manifest["files"]
