from __future__ import annotations

import hashlib
import json

from cftuv_envelope import (
    AnalysisSnapshotCodecV1,
    CompiledPlanCodecV1,
    DecalRequestCodecV1,
    semantic_plan_digest,
    snapshot_digest,
    validate_cross_contract_references,
)


def _seed_id(seed):
    return seed.seed_id.value


def _fixture_sha256(path):
    content = path.read_bytes().replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    return hashlib.sha256(content).hexdigest()


def test_fixture_manifest_is_hash_complete(fixture_root):
    manifest_path = fixture_root / "fixture_hash_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["source_commit"] == "1fb0394a634c089ed590f544e4d524be28f52123"
    assert manifest["source_schema"] == "cftuv.envelope.ec0.corpus.v5"
    assert manifest["hash_law"] == "SHA256_UTF8_LF_NORMALIZED_V1"
    actual = {
        path.relative_to(fixture_root).as_posix(): _fixture_sha256(path)
        for path in sorted(fixture_root.rglob("*"))
        if path.is_file() and path != manifest_path
    }
    assert actual == manifest["files"]


def test_fixture_hash_law_is_newline_platform_independent(tmp_path):
    lf_path = tmp_path / "lf.json"
    crlf_path = tmp_path / "crlf.json"
    lf_path.write_bytes(b'{\n  "value": 1\n}\n')
    crlf_path.write_bytes(b'{\r\n  "value": 1\r\n}\r\n')
    assert _fixture_sha256(lf_path) == _fixture_sha256(crlf_path)


def test_all_accepted_cases_project_without_semantic_id_loss(projections, fixture_root):
    assert len(projections) == 23
    assert sum(len(item.plans) for item in projections) == 26
    for projection in projections:
        issues = validate_cross_contract_references(
            projection.snapshot, projection.request, projection.plans
        )
        assert issues == (), (projection.case_id, issues)
        raw_path = next(
            path
            for path in (fixture_root / "cases").glob("*.json")
            if json.loads(path.read_text(encoding="utf-8"))["case_id"] == projection.case_id
        )
        raw = json.loads(raw_path.read_text(encoding="utf-8"))
        snapshot = raw["analysis_snapshot"]
        assert {item["id"] for item in snapshot["patches"]} == {
            item.patch_id.value for item in projection.snapshot.patches
        }
        assert {item["id"] for item in snapshot["patch_domains"]} == {
            item.patch_domain_id.value for item in projection.snapshot.patch_domains
        }
        assert {item["id"] for item in snapshot["physical_chains"]} == {
            item.physical_chain_id.value for item in projection.snapshot.physical_chains
        }
        assert {item["id"] for item in snapshot["chain_uses"]} == {
            item.chain_use_id.value for item in projection.snapshot.chain_uses
        }
        raw_evaluations = raw["expected_compiled_plan"]["patch_evaluations"]
        plans = {plan.evaluation_plan_id.value: plan for plan in projection.plans}
        for raw_eval in raw_evaluations:
            plan = plans[raw_eval["id"]]
            assert {item["id"] for item in raw_eval["seeds"]} == {
                _seed_id(item) for item in plan.seeds
            }
            assert {item["id"] for item in raw_eval["front_components"]} == {
                item.front_component_id.value for item in plan.front_components
            }
            assert {item["id"] for item in raw_eval["envelope_specs"]} == {
                item.envelope_spec_id.value for item in plan.envelope_specs
            }
            assert {item["id"] for item in raw_eval["envelope_instances"]} == {
                item.instance_id.value for item in plan.envelope_instances
            }
            assert {item["id"] for item in raw_eval["ownership"]["claims"]} == {
                item.ownership_claim_id.value for item in plan.ownership.claims
            }


def test_public_contract_round_trip_and_repeated_serialization(projections):
    for projection in projections:
        pairs = [
            (AnalysisSnapshotCodecV1, projection.snapshot),
            (DecalRequestCodecV1, projection.request),
        ]
        pairs.extend((CompiledPlanCodecV1, plan) for plan in projection.plans)
        for codec, record in pairs:
            first = codec.dumps(record)
            second = codec.dumps(record)
            assert first == second
            decoded = codec.loads(first)
            assert decoded == record
            assert codec.dumps(decoded) == first


def test_snapshot_and_plan_hashes_are_stable(projections):
    for projection in projections:
        assert snapshot_digest(projection.snapshot) == snapshot_digest(projection.snapshot)
        for plan in projection.plans:
            assert semantic_plan_digest(plan) == semantic_plan_digest(plan)
