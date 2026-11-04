"""Правило Straight strips (`UV_DIRECT_STRIP_V1`): одна pChain равна одному непрерывному UV-острову.

Остров - связная компонента граней батча: две грани связаны ребром, если обе петли ребра дают одни UV. pChain - запись происхождения владельца
(`contracts.lineage.owner_chain_source_prefix`), то есть исходная цепь до резки по изломам: фаска без шва и излом одной цепи лежат внутри неё.

Перепись только считает; вердикт (`violations`) и храповик читают инструмент `tools/uv_island_census.py` и тесты. Для других UV-законов правило не заявлено.
"""

from __future__ import annotations

from dataclasses import dataclass

from .contracts.lineage import owner_chain_source_prefix


@dataclass(frozen=True, slots=True)
class UvIslandV1:
    faces: tuple[int, ...]
    pchains: tuple[str, ...]


@dataclass(frozen=True, slots=True)
class UvIslandCensusV1:
    islands: tuple[UvIslandV1, ...]
    #: Рёбра, на которых UV двух граней расходятся.
    seam_edges: int
    #: Те из них, у которых обе грани в одном острове (остров замкнут вокруг шва).
    internal_seam_edges: int

    @property
    def pchains(self) -> tuple[str, ...]:
        return tuple(sorted({pchain for island in self.islands for pchain in island.pchains}))

    def owned_islands(self, pchain: str) -> tuple[UvIslandV1, ...]:
        """Острова, все грани которых принадлежат одной этой pChain."""

        return tuple(island for island in self.islands if island.pchains == (pchain,))

    @property
    def wedges(self) -> tuple[UvIslandV1, ...]:
        """Острова на нескольких pChain: веер угла между разными цепями либо склейка двух цепей."""

        return tuple(island for island in self.islands if len(island.pchains) > 1)

    @property
    def satisfied(self) -> tuple[str, ...]:
        return tuple(pchain for pchain in self.pchains if len(self.owned_islands(pchain)) == 1)

    @property
    def violations(self) -> tuple[tuple[str, int], ...]:
        """`(pChain, сколько островов ей принадлежит)`, где их не ровно один."""

        return tuple((pchain, len(self.owned_islands(pchain))) for pchain in self.pchains if len(self.owned_islands(pchain)) != 1)


def uv_island_census(faces, uvs, pchains) -> UvIslandCensusV1:
    """`faces[i]` - ключи вершин грани по порядку, `uvs[i]` - `{ключ: uv}` этой грани, `pchains[i]` - имена pChain грани."""

    parent = list(range(len(faces)))

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    edges: dict = {}
    for index, keys in enumerate(faces):
        for at, key in enumerate(keys):
            edges.setdefault(frozenset((key, keys[(at + 1) % len(keys)])), []).append(index)
    seams = []
    for edge, users in edges.items():
        for other in users[1:]:
            if all(uvs[users[0]][key] == uvs[other][key] for key in edge):
                parent[find(users[0])] = find(other)
            else:
                seams.append((users[0], other))
    groups: dict = {}
    for index in range(len(faces)):
        groups.setdefault(find(index), []).append(index)
    islands = tuple(
        UvIslandV1(tuple(group), tuple(sorted({pchain for index in group for pchain in pchains[index]})))
        for group in sorted(groups.values())
    )
    return UvIslandCensusV1(islands, len(seams), sum(1 for first, second in seams if find(first) == find(second)))


def pchains_of_uses(context) -> dict:
    """`{имя вхождения: pChain}` домена; вхождение без записи владельца названо своей физической цепью."""

    found = {}
    for use_id, use in context.uses_by_id.items():
        chain = context.chains_by_id.get(use.physical_chain_id)
        prefix = owner_chain_source_prefix(use.owner_patch_id)
        records = sorted(item.value for item in chain.data_record_lineage if item.value.startswith(prefix)) if chain is not None else []
        found[use_id.value] = records[0] if records else f"physical:{use.physical_chain_id.value}"
    return found


def batch_island_census(batch, named: dict) -> UvIslandCensusV1:
    """Перепись батча материализатора; `named` - `{имя вхождения: pChain}` (`pchains_of_uses`)."""

    faces = [tuple(key.value for key in face.ordered_vert_keys) for face in batch.faces]
    uvs = [{fact.vert_key.value: fact.uv for fact in face.uv_facts} for face in batch.faces]
    pchains = [tuple(sorted({named.get(use.value, f"use:{use.value}") for use in face.provenance.chain_use_ids})) for face in batch.faces]
    return uv_island_census(faces, uvs, pchains)
