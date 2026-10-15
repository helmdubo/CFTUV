"""Explicit adapter resolution for the shared frozen Density controller.

Controller bodies remain in adaptive_density_fan; the optional laws parameter
is threaded through helpers and atlas callbacks. No function globals/imports
are replaced, and the default adapter is the old leaf implementation.
"""

from .density_legacy_laws_v1 import DensityLegacyLawsV1
from .density_attempt_memo_v1 import density_attempt_key


def resolve_density_laws(laws):
    return DensityLegacyLawsV1() if laws is None else laws


def bind_owned_density_fan(laws, metric, ideal, orientation, q, binding_reasons,
                          *, lifted, law, spec_id, refusals, canon_ids,
                          shell_event_collector=None):
    """Explicit shared-controller invocation; one arena and success-only memo.

    Internal owned profile failures propagate unchanged. A legacy narrow-band
    refusal alone records the old named fallback before the Voronoi call.
    Canon usage is replayed in success/failure/memo paths; warm success creates
    no new budget object. Mathematical A2/A1 events belong to the leaf gateway.
    """
    from . import adaptive_density_fan as fan
    from .density_fan_binding import _BAND_REFUSALS
    from .density_oracle_v1 import CanonUsageFactsV1, DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED

    canon_ids.validate_candidate()
    usage = (CanonUsageFactsV1(canon_ids.construction, DENSITY_SOURCE_CONSTRUCTION_V1_APPLIED),)
    memo = laws.attempt_memo

    def build(window_law):
        adaptive_only = lifted or window_law != "VORONOI_OF_NEIGHBOUR_MIDPOINTS"
        key = density_attempt_key(laws, metric, ideal, orientation, q, binding_reasons,
                                  adaptive_only, window_law, canon_ids)
        cached = memo.lookup(key)
        if cached is not None:
            result, shell_outcomes = cached
            laws.shell_outcomes.extend(shell_outcomes)
            return result
        first_outcome = len(laws.shell_outcomes)
        if adaptive_only:
            result = ((None,)*(len(ideal)-2), fan.certify_adaptive_density_fan(
                metric, ideal, orientation, q, binding_reasons=binding_reasons,
                resource_height_cap=laws.limits.height_cap, window_law=window_law, laws=laws))
        else:
            result = fan.certify_density_bindings_and_adaptive_fallback(
                metric, ideal, orientation, q, binding_reasons=binding_reasons,
                resource_height_cap=laws.limits.height_cap, laws=laws)
        memo.remember(key, result, laws.shell_outcomes[first_outcome:])
        return result

    try:
        if law == "VORONOI_OF_NEIGHBOUR_MIDPOINTS":
            return build(law)
        try:
            return build(law)
        except _BAND_REFUSALS as error:
            refusals.append((spec_id, f"{type(error).__name__}: {error}"))
        return build("VORONOI_OF_NEIGHBOUR_MIDPOINTS")
    finally:
        if shell_event_collector is not None:
            shell_event_collector.replay_canon(spec_id, usage, canon_ids)
