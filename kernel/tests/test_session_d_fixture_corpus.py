from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import sympy as sp

from session_d_case_registry import execute_session_d_case


FIXTURE_ROOT = (
    Path(__file__).parents[1] / "fixtures" / "session_d_interactions_v1"
)
REQUIRED_CASE_IDS = {
    f"SD-I{index:02d}-{suffix}"
    for index, suffix in enumerate(
        (
            "OPPOSING-BEFORE-MUTUAL-ARRIVAL",
            "OPPOSING-AT-EXACT-MUTUAL-ARRIVAL",
            "OPPOSING-AFTER-MUTUAL-ARRIVAL",
            "UNEQUAL-SOURCE-DISTANCE-RATIONAL-EVENT",
            "EXPLICIT-STRIP-SELF-CONTACT",
            "SAME-GEOMETRY-DIFFERENT-REQUEST",
            "SAME-GEOMETRY-DIFFERENT-PATCH-DOMAIN",
            "C13-THIRD-STRIP-EXPOSED-K0-PROFILE",
            "C13-THIRD-STRIP-EXPOSED-K1-PROFILE",
            "C13-THIRD-STRIP-EXPOSED-K2-PROFILE",
            "C13-THIRD-STRIP-BEFORE-PROFILE-ARRIVAL",
            "ONE-SIDED-ARRIVAL-SATURATED-COMPONENT",
            "EQUALITY-LOCUS-TERMINATES-AT-OUTER-BOUNDARY",
            "EQUALITY-LOCUS-TERMINATES-AT-HOLE",
            "TWO-INDEPENDENT-SAME-ALPHA-CONTACTS",
            "TRIPLE-MEETING-ONE-EVENT",
            "CAP-CONTACT-ENDPOINT-BOUNDED",
            "INTERNALIZED-ANGULAR-SUPPORT-NOT-A-FRONT",
            "COINCIDENT-ARRIVAL-LAWS-NAMED-UNPROVEN",
            "REVERSED-INPUT-ORDER-STABLE",
            "RIGID-TRANSFORM-AND-UNIFORM-SCALE",
            "SELF-CONTACT-AFTER-BOUNDARY-CLIP",
            "CROSS-PATCH-JUNCTION-PROJECTIONS-NO-COLLISION",
        ),
        start=1,
    )
}


def _load_payload() -> tuple[dict, dict]:
    manifest = json.loads(
        (FIXTURE_ROOT / "manifest.json").read_text(encoding="utf-8")
    )
    payload = json.loads(
        (FIXTURE_ROOT / manifest["case_file"]).read_text(encoding="utf-8")
    )
    return manifest, payload


MANIFEST, PAYLOAD = _load_payload()
CASES = PAYLOAD["cases"]


def _normalized_sha256(path: Path) -> str:
    content = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(content).hexdigest()


def _matches(actual, expected) -> bool:
    if isinstance(expected, str) and isinstance(actual, str):
        try:
            return sp.sympify(actual) == sp.sympify(expected)
        except (sp.SympifyError, TypeError):
            return actual == expected
    return actual == expected


def test_session_d_corpus_is_complete_and_declarative():
    assert MANIFEST["case_count"] == len(CASES) == 23
    assert {item["id"] for item in CASES} == REQUIRED_CASE_IDS
    assert PAYLOAD["schema"] == (
        "cftuv.envelope.session_d.interaction_fixture_set.v1"
    )
    assert MANIFEST["policy"] == "INTRAPATCH_POLICY_B_V1"
    for item in CASES:
        assert item["scenario"]
        assert item["source_stage"]
        assert item["expected"]


@pytest.mark.parametrize("case", CASES, ids=lambda item: item["id"])
def test_every_session_d_case_executes_from_the_canonical_declaration(case):
    receipt = execute_session_d_case(case)
    for key, expected in case["expected"].items():
        assert key in receipt.observed
        actual = receipt.observed[key]
        if isinstance(expected, list):
            assert len(actual) == len(expected)
            assert all(
                _matches(actual_item, expected_item)
                for actual_item, expected_item in zip(actual, expected)
            ), (key, actual, expected)
        else:
            assert _matches(actual, expected), (key, actual, expected)


def test_session_d_corpus_hash_manifest_is_complete_and_newline_stable():
    manifest_path = FIXTURE_ROOT / "fixture_hash_manifest.json"
    hash_manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert hash_manifest["hash_law"] == "SHA256_UTF8_LF_NORMALIZED_V1"
    assert (
        hash_manifest["source_commit"]
        == "6c61ab311613cb7cf96e5c5098e857e78a98f0b8"
    )
    actual = {
        path.relative_to(FIXTURE_ROOT).as_posix(): _normalized_sha256(path)
        for path in sorted(FIXTURE_ROOT.rglob("*"))
        if path.is_file() and path != manifest_path
    }
    assert actual == hash_manifest["files"]
