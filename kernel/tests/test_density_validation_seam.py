from __future__ import annotations

from types import SimpleNamespace

import pytest

from cftuv_envelope.contracts.request import AngularProfileSelectionPolicyId
from cftuv_envelope.reference.contracts import ReferenceOutcome
from cftuv_envelope.interactions import arrival as arrival_module
from cftuv_envelope.interactions import resolved_coverage as resolved_module
from cftuv_envelope.reference import raw_coverage as raw_module
from cftuv_envelope.reference import validation as validation_module


_POLICIES = (
    (AngularProfileSelectionPolicyId.MIN_K_FOR_MAX_SUBTURN_V1, False),
    (
        AngularProfileSelectionPolicyId.HUBER_EMANATED_COUNT_DENSITY_A_V1,
        True,
    ),
)


def _compilation(policy):
    return SimpleNamespace(
        analysis_snapshot=object(),
        plan_key=SimpleNamespace(patch_domain_id=object()),
        decal_request=SimpleNamespace(
            angular_profile_selection_policy_id=policy,
        ),
        evaluation_geometry_binding=None,
    )


def _capture_payload_validation(monkeypatch):
    observed = []

    def validate(*args, **kwargs):
        del args
        observed.append(kwargs["density_bounded"])
        return None, (
            SimpleNamespace(
                outcome=(
                    ReferenceOutcome.REFERENCE_ANGLE_SUPPORT_CERTIFICATE_MISMATCH
                ),
                message="expected seam stop",
            ),
        )

    monkeypatch.setattr(
        validation_module,
        "validate_reference_geometry_payload",
        validate,
    )
    return observed


@pytest.mark.parametrize(("policy", "expected"), _POLICIES)
def test_raw_coverage_passes_request_density_authority(
    monkeypatch,
    policy,
    expected,
):
    observed = _capture_payload_validation(monkeypatch)

    raw_module._evaluate_reference_raw_coverage(_compilation(policy), "1")

    assert observed == [expected]


@pytest.mark.parametrize(("policy", "expected"), _POLICIES)
def test_arrival_passes_request_density_authority(
    monkeypatch,
    policy,
    expected,
):
    observed = _capture_payload_validation(monkeypatch)

    arrival_module._compile_arrival_models(_compilation(policy), (), ())

    assert observed == [expected]


@pytest.mark.parametrize(("policy", "expected"), _POLICIES)
def test_resolved_coverage_passes_request_density_authority(
    monkeypatch,
    policy,
    expected,
):
    observed = _capture_payload_validation(monkeypatch)
    monkeypatch.setattr(
        resolved_module,
        "validate_interaction_inputs",
        lambda *args: (),
    )
    monkeypatch.setattr(
        resolved_module,
        "compile_interaction_components",
        lambda *args: (),
    )
    monkeypatch.setattr(
        resolved_module,
        "compile_arrival_models",
        lambda *args: ((), ()),
    )
    monkeypatch.setattr(
        resolved_module,
        "generate_interaction_candidates",
        lambda *args: (),
    )
    monkeypatch.setattr(
        resolved_module,
        "prove_mutual_arrivals",
        lambda *args: ((), ()),
    )

    resolved_module._resolve_coverage_interactions(
        _compilation(policy),
        (),
        object(),
    )

    assert observed == [expected]
