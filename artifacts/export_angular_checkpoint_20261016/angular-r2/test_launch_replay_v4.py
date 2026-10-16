"""Launcher continuation depends on acceptance, not on exit code (regression of launch-bounded-replay-v2)."""
import pytest
from launch_replay_v4 import continuation

DONE = dict(status='COMPLETE_REPLAY_SCOPE', accepted=True)


def test_exit_zero_with_unaccepted_result_stops():
    # The exact v2 defect: worker/coordinator exit 0, replay.json accepted=false.
    assert continuation(0, dict(DONE, accepted=False)) == (False, 'STOP_AFTER_NONACCEPTED_SCOPE')


def test_accepted_complete_scope_continues():
    assert continuation(0, dict(DONE)) == (True, 'CONTINUE')


@pytest.mark.parametrize('code', [1, -1, None, '0', True])
def test_nonzero_or_untyped_exit_stops_even_if_accepted(code):
    assert continuation(code, dict(DONE))[0] is False


@pytest.mark.parametrize('accepted', [1, 'true', None, [True]])
def test_truthy_but_not_true_acceptance_stops(accepted):
    assert continuation(0, dict(DONE, accepted=accepted)) == (False, 'STOP_AFTER_NONACCEPTED_SCOPE')


@pytest.mark.parametrize('state', [None, {}, dict(status='INCOMPLETE', accepted=True)])
def test_missing_or_incomplete_state_stops(state):
    assert continuation(0, state) == (False, 'STOP_AFTER_INCOMPLETE_SCOPE')
