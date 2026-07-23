from __future__ import annotations

import ast
from pathlib import Path
from types import SimpleNamespace

from cftuv.envelope_debug_renderer import (
    ENVELOPE_DEBUG_LABEL_LAYER,
    ENVELOPE_DEBUG_LAYER_STYLES,
    envelope_debug_object_name,
    envelope_debug_profile_text_name,
    envelope_debug_text_name,
    visibility_from_settings,
)


REQUIRED_LAYERS = {
    "ENV_00_PATCH_DOMAIN",
    "ENV_01_HOLES",
    "ENV_02_BARRIERS",
    "ENV_10_PHYSICAL_CHAINS",
    "ENV_11_CHAIN_USES",
    "ENV_12_SELECTED_SOURCES",
    "ENV_13_PATCH_PAIRS",
    "ENV_14_SEAM_SELF_PAIRS",
    "ENV_20_SOURCE_SUPPORTS",
    "ENV_21_MOVING_FRONTS",
    "ENV_22_HIDDEN_SUPPORTS",
    "ENV_30_ENVELOPE_INSTANCES",
    "ENV_40_RAW_COVERAGE",
    "ENV_50_INTERACTION_COMPONENTS",
    "ENV_51_FRONT_READINGS",
    "ENV_52_EQUALITY_LOCI",
    "ENV_60_RESOLVED_COVERAGE",
    "ENV_61_RETAINED_REGIONS",
    "ENV_62_REMOVED_REGIONS",
    "ENV_70_EVENT_ANCHORS",
    "ENV_80_DIAGNOSTICS",
    "ENV_LABELS",
}


def test_v0b_layer_contract_is_complete_and_named():
    assert set(ENVELOPE_DEBUG_LAYER_STYLES) == REQUIRED_LAYERS
    assert ENVELOPE_DEBUG_LABEL_LAYER == "ENV_LABELS"
    assert envelope_debug_object_name("Cube") == "CFTUV_DEBUG_Envelope_Cube"
    assert (
        envelope_debug_text_name("Cube")
        == "CFTUV_EnvelopeDebug_Cube.json"
    )
    assert (
        envelope_debug_profile_text_name("Cube")
        == "CFTUV_EnvelopeProfile_Cube.json"
    )


def test_visibility_groups_map_to_runtime_layers_without_geometry_changes():
    settings = SimpleNamespace(
        envelope_debug_show_domains=False,
        envelope_debug_show_chains=True,
        envelope_debug_show_supports=False,
        envelope_debug_show_envelopes=True,
        envelope_debug_show_raw=False,
        envelope_debug_show_readings=True,
        envelope_debug_show_equality=False,
        envelope_debug_show_resolved=True,
        envelope_debug_show_diagnostics=False,
        envelope_debug_show_labels=True,
    )
    visibility = visibility_from_settings(settings)
    assert visibility["ENV_00_PATCH_DOMAIN"] is False
    assert visibility["ENV_10_PHYSICAL_CHAINS"] is True
    assert visibility["ENV_12_SELECTED_SOURCES"] is True
    assert visibility["ENV_13_PATCH_PAIRS"] is True
    assert visibility["ENV_14_SEAM_SELF_PAIRS"] is True
    assert visibility["ENV_20_SOURCE_SUPPORTS"] is False
    assert visibility["ENV_40_RAW_COVERAGE"] is False
    assert visibility["ENV_51_FRONT_READINGS"] is True
    assert visibility["ENV_52_EQUALITY_LOCI"] is False
    assert visibility["ENV_60_RESOLVED_COVERAGE"] is True
    assert visibility["ENV_80_DIAGNOSTICS"] is False
    assert visibility["ENV_LABELS"] is True


def test_renderer_does_not_import_legacy_or_production_geometry():
    path = (
        Path(__file__).resolve().parents[1]
        / "cftuv"
        / "envelope_debug_renderer.py"
    )
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    imports = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add(node.module or "")
    assert not imports & {
        "cftuv.decal_voronoi",
        "cftuv.decals",
        "decal_voronoi",
        "decals",
        "cftuv_envelope.contracts.geometry_batch",
        "cftuv_envelope.contracts.ownership",
    }
