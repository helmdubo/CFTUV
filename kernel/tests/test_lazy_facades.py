from __future__ import annotations

import cftuv_envelope


# Полный публичный список уже отдельно заморожен дайджестом в
# test_public_contracts.py. Здесь замораживается оставшаяся часть dir(), чтобы
# ленивый фасад не менял даже исторически видимые служебные имена.
_TOP_LEVEL_DIR_EXTRAS = (
    "_PUBLIC_VALUE_MODULES",
    "__all__",
    "__builtins__",
    "__cached__",
    "__doc__",
    "__file__",
    "__loader__",
    "__name__",
    "__package__",
    "__path__",
    "__spec__",
    "_analysis",
    "_coverage",
    "_debug",
    "_density_policy",
    "_envelopes",
    "_events",
    "_geometry_batch",
    "_ids",
    "_metric",
    "_numeric",
    "_outcomes",
    "_ownership",
    "_plan",
    "_request",
    "_seeds",
    "_surface",
    "_tessellation",
)


def test_top_level_dir_matches_eager_facade_snapshot():
    expected = sorted((*cftuv_envelope.__all__, *_TOP_LEVEL_DIR_EXTRAS))
    assert sorted(dir(cftuv_envelope)) == expected
