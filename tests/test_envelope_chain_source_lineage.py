"""Запись `chain-source` хоста: ЦЕПЬ патча до разреза по изломам, и формат у неё один с ядром.

Ядро (`reference/corner_treatment.py`) считает два куска «одной цепью» ровно тогда, когда у них есть общая
запись `chain-source` ПАТЧА-ВЛАДЕЛЬЦА угла. Шовная цепь двух патчей несёт записи обоих, и запись соседа цепью
владельца делать нельзя; поэтому хост пишет в запись идентификатор патча, а формат берёт у ядра
(`cftuv_envelope.contracts.lineage`), а не дублирует литералом.
"""

from __future__ import annotations

import sys
from collections import defaultdict
from pathlib import Path

KERNEL_SRC = Path(__file__).resolve().parents[1] / "kernel" / "src"
if str(KERNEL_SRC) not in sys.path:
    sys.path.insert(0, str(KERNEL_SRC))

from cftuv.envelope_host_adapter import build_envelope_analysis_snapshot  # noqa: E402
from cftuv_envelope._corner_treatment import shared_source_lineage  # noqa: E402
from cftuv_envelope.contracts.lineage import CHAIN_SOURCE_LINEAGE_PREFIX, owner_chain_source_prefix  # noqa: E402

from envelope_fixture_bundles import planar_quad_bundle, u_route_bundle  # noqa: E402


def _by_chain(snapshot):
    uses = defaultdict(list)
    for use in snapshot.chain_uses:
        uses[use.physical_chain_id].append(use)
    return uses


def _chain_sources(chain):
    return {item.value for item in chain.data_record_lineage if item.value.startswith(CHAIN_SOURCE_LINEAGE_PREFIX)}


def test_every_use_has_a_chain_source_record_of_its_own_patch():
    for bundle in (u_route_bundle(), planar_quad_bundle()):
        snapshot = build_envelope_analysis_snapshot(bundle)
        uses = _by_chain(snapshot)
        for chain in snapshot.physical_chains:
            sources = _chain_sources(chain)
            assert sources
            for use in uses[chain.physical_chain_id]:
                prefix = owner_chain_source_prefix(use.owner_patch_id)
                assert any(item.startswith(prefix) for item in sources), (use.chain_use_id, sources)
            # Шовная цепь двух патчей несёт записи обоих: записей не меньше, чем владельцев цепи.
            owners = {use.owner_patch_id for use in uses[chain.physical_chain_id]}
            assert len(sources) >= len(owners)


def test_pieces_cut_from_one_source_chain_share_a_record_only_within_their_patch():
    snapshot = build_envelope_analysis_snapshot(u_route_bundle())
    uses = _by_chain(snapshot)
    chains = {item.physical_chain_id: item for item in snapshot.physical_chains}
    shared_any = False
    for first in snapshot.physical_chains:
        for second in snapshot.physical_chains:
            if first.physical_chain_id.value >= second.physical_chain_id.value:
                continue
            for use in uses[first.physical_chain_id]:
                for other in uses[second.physical_chain_id]:
                    if use.owner_patch_id != other.owner_patch_id:
                        continue
                    shared = shared_source_lineage(chains[use.physical_chain_id], chains[other.physical_chain_id], use.owner_patch_id)
                    shared_any = shared_any or bool(shared)
                    prefix = owner_chain_source_prefix(use.owner_patch_id)
                    assert all(item.value.startswith(prefix) for item in shared)
                    # Общий набор не зависит от порядка кусков.
                    assert shared == shared_source_lineage(chains[other.physical_chain_id], chains[use.physical_chain_id], use.owner_patch_id)
    assert shared_any, "the split route must give at least one pair of pieces of one source chain"
