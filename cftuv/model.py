from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from mathutils import Vector
from typing import Optional


class PatchType(str, Enum):
    """Dispatch key for the patch UV strategy."""

    WALL = "WALL"
    FLOOR = "FLOOR"
    SLOPE = "SLOPE"


class WorldFacing(str, Enum):
    """Coarse patch orientation relative to world Z."""

    UP = "UP"
    DOWN = "DOWN"
    SIDE = "SIDE"


class LoopKind(str, Enum):
    """Kind of closed boundary loop."""

    OUTER = "OUTER"
    HOLE = "HOLE"


class FrameRole(str, Enum):
    """Alignment role of a boundary chain in the local patch basis."""

    H_FRAME = "H_FRAME"
    V_FRAME = "V_FRAME"
    FREE = "FREE"


class ChainNeighborKind(str, Enum):
    """Topology class of a boundary chain neighbor."""

    PATCH = "PATCH"
    MESH_BORDER = "MESH_BORDER"
    SEAM_SELF = "SEAM_SELF"


@dataclass
class BoundaryChain:
    """Continuous part of a boundary loop with one neighbor.
    Chain — первичная единица placement в solve."""

    vert_indices: list[int] = field(default_factory=list)
    vert_cos: list[Vector] = field(default_factory=list)
    edge_indices: list[int] = field(default_factory=list)
    side_face_indices: list[int] = field(default_factory=list)
    neighbor_patch_id: int = -1
    is_closed: bool = False
    frame_role: FrameRole = FrameRole.FREE
    start_loop_index: int = 0
    end_loop_index: int = 0
    start_corner_index: int = -1
    end_corner_index: int = -1

    @property
    def neighbor_kind(self) -> ChainNeighborKind:
        """Derive the neighbor kind from the encoded neighbor id."""

        if self.neighbor_patch_id == -2:
            return ChainNeighborKind.SEAM_SELF
        if self.neighbor_patch_id == -1:
            return ChainNeighborKind.MESH_BORDER
        return ChainNeighborKind.PATCH

    @property
    def has_patch_neighbor(self) -> bool:
        return self.neighbor_kind == ChainNeighborKind.PATCH

    @property
    def start_vert_index(self) -> int:
        return self.vert_indices[0] if self.vert_indices else -1

    @property
    def end_vert_index(self) -> int:
        return self.vert_indices[-1] if self.vert_indices else -1


@dataclass
class BoundaryCorner:
    """Junction corner between two neighboring chains inside one loop.
    Corner не имеет собственных координат — его позиция возникает
    как результат размещения chains."""

    loop_vert_index: int = 0
    vert_index: int = -1
    vert_co: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 0.0)))
    prev_chain_index: int = 0
    next_chain_index: int = 0
    turn_angle_deg: float = 0.0
    prev_role: FrameRole = FrameRole.FREE
    next_role: FrameRole = FrameRole.FREE

    @property
    def corner_type(self) -> str:
        return f"{self.prev_role.value}_TO_{self.next_role.value}"


@dataclass
class BoundaryLoop:
    """Closed boundary loop of a patch."""

    vert_indices: list[int] = field(default_factory=list)
    vert_cos: list[Vector] = field(default_factory=list)
    edge_indices: list[int] = field(default_factory=list)
    side_face_indices: list[int] = field(default_factory=list)
    kind: LoopKind = LoopKind.OUTER
    depth: int = 0
    chains: list[BoundaryChain] = field(default_factory=list)
    corners: list[BoundaryCorner] = field(default_factory=list)


@dataclass
class PatchNode:
    """Patch node stored inside the central PatchGraph IR."""

    patch_id: int
    face_indices: list[int]
    centroid: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 0.0)))
    normal: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 1.0)))
    area: float = 0.0
    perimeter: float = 0.0
    patch_type: PatchType = PatchType.WALL
    world_facing: WorldFacing = WorldFacing.SIDE
    basis_u: Vector = field(default_factory=lambda: Vector((1.0, 0.0, 0.0)))
    basis_v: Vector = field(default_factory=lambda: Vector((0.0, 0.0, 1.0)))
    boundary_loops: list[BoundaryLoop] = field(default_factory=list)
    mesh_verts: list[Vector] = field(default_factory=list)
    mesh_tris: list[tuple[int, int, int]] = field(default_factory=list)

    @property
    def semantic_key(self) -> str:
        patch_type = self.patch_type.value if hasattr(self.patch_type, "value") else str(self.patch_type)
        world_facing = self.world_facing.value if hasattr(self.world_facing, "value") else str(self.world_facing)
        return f"{patch_type}.{world_facing}"


@dataclass
class SeamEdge:
    """Shared seam relation between two neighboring patches."""

    patch_a_id: int
    patch_b_id: int
    shared_length: float = 0.0
    shared_vert_indices: list[int] = field(default_factory=list)
    longest_edge_verts: tuple[int, int] = (0, 0)
    longest_edge_length: float = 0.0


@dataclass(frozen=True)
class UVSettings:
    """Immutable snapshot of UV settings passed through the pipeline."""

    texel_density: int = 512
    texture_size: int = 2048
    uv_scale: float = 1.0
    uv_range_limit: float = 16.0

    @property
    def final_scale(self) -> float:
        return (self.texel_density / self.texture_size) * self.uv_scale

    @staticmethod
    def from_blender_settings(settings) -> "UVSettings":
        """Build a UVSettings object from the Blender PropertyGroup."""

        return UVSettings(
            texel_density=int(settings.target_texel_density),
            texture_size=int(settings.texture_size),
            uv_scale=float(settings.uv_scale),
            uv_range_limit=float(settings.uv_range_limit),
        )


@dataclass(frozen=True)
class MeshPreflightIssue:
    """Preflight issue blocking the solve pipeline."""

    code: str
    message: str
    face_indices: tuple[int, ...] = ()
    edge_indices: tuple[int, ...] = ()
    vert_indices: tuple[int, ...] = ()


@dataclass
class MeshPreflightReport:
    """Preflight result for solve input mesh."""

    checked_face_indices: tuple[int, ...] = ()
    issues: list[MeshPreflightIssue] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        return not self.issues

# ============================================================
# PatchGraph — центральный IR
# ============================================================

@dataclass
class PatchGraph:
    """Central IR with patch nodes, seam edges, and lookup tables."""

    nodes: dict[int, PatchNode] = field(default_factory=dict)
    edges: dict[tuple[int, int], SeamEdge] = field(default_factory=dict)
    face_to_patch: dict[int, int] = field(default_factory=dict)
    _adjacency: dict[int, set[int]] = field(default_factory=dict)

    def add_node(self, node: PatchNode) -> None:
        self.nodes[node.patch_id] = node
        self._adjacency.setdefault(node.patch_id, set())
        for face_index in node.face_indices:
            self.face_to_patch[face_index] = node.patch_id

    def add_edge(self, seam: SeamEdge) -> None:
        key = (min(seam.patch_a_id, seam.patch_b_id), max(seam.patch_a_id, seam.patch_b_id))
        self.edges[key] = seam
        self._adjacency.setdefault(seam.patch_a_id, set()).add(seam.patch_b_id)
        self._adjacency.setdefault(seam.patch_b_id, set()).add(seam.patch_a_id)

    def get_neighbors(self, patch_id: int) -> set[int]:
        return self._adjacency.get(patch_id, set())

    def get_seam(self, patch_a: int, patch_b: int) -> Optional[SeamEdge]:
        key = (min(patch_a, patch_b), max(patch_a, patch_b))
        return self.edges.get(key)

    def get_patch_semantic_key(self, patch_id: int) -> str:
        node = self.nodes.get(patch_id)
        if node is None:
            return "UNKNOWN"
        return node.semantic_key

    def get_chain(self, patch_id: int, loop_index: int, chain_index: int) -> Optional[BoundaryChain]:
        node = self.nodes.get(patch_id)
        if node is None or loop_index < 0 or loop_index >= len(node.boundary_loops):
            return None
        boundary_loop = node.boundary_loops[loop_index]
        if chain_index < 0 or chain_index >= len(boundary_loop.chains):
            return None
        return boundary_loop.chains[chain_index]

    def find_chains_touching_vertex(
        self,
        patch_id: int,
        vert_index: int,
        exclude: Optional[tuple[int, int]] = None,
    ) -> list[tuple[int, int]]:
        node = self.nodes.get(patch_id)
        if node is None or vert_index < 0:
            return []

        matches = []
        for loop_index, boundary_loop in enumerate(node.boundary_loops):
            for chain_index, chain in enumerate(boundary_loop.chains):
                if exclude == (loop_index, chain_index):
                    continue
                if chain.start_vert_index == vert_index or chain.end_vert_index == vert_index:
                    matches.append((loop_index, chain_index))
        return matches

    def get_chain_endpoint_neighbors(self, patch_id: int, loop_index: int, chain_index: int) -> dict[str, list[tuple[int, int]]]:
        chain = self.get_chain(patch_id, loop_index, chain_index)
        if chain is None:
            return {"start": [], "end": []}

        return {
            "start": self.find_chains_touching_vertex(
                patch_id,
                chain.start_vert_index,
                exclude=(loop_index, chain_index),
            ),
            "end": self.find_chains_touching_vertex(
                patch_id,
                chain.end_vert_index,
                exclude=(loop_index, chain_index),
            ),
        }

    def describe_chain_transition(self, owner_patch_id: int, chain: BoundaryChain) -> str:
        owner_key = self.get_patch_semantic_key(owner_patch_id)
        neighbor_kind = chain.neighbor_kind.value if hasattr(chain.neighbor_kind, "value") else str(chain.neighbor_kind)
        if neighbor_kind == ChainNeighborKind.PATCH.value:
            neighbor_key = self.get_patch_semantic_key(chain.neighbor_patch_id)
            return f"{owner_key} -> {neighbor_key}"
        if neighbor_kind == ChainNeighborKind.SEAM_SELF.value:
            return f"{owner_key} -> {owner_key}"
        return f"{owner_key} -> {neighbor_kind}"

    def traverse_bfs(self, root_id: int) -> list[list[int]]:
        visited = {root_id}
        levels = [[root_id]]
        current = [root_id]
        while current:
            next_level = []
            for patch_id in current:
                for neighbor_id in self.get_neighbors(patch_id):
                    if neighbor_id in visited:
                        continue
                    visited.add(neighbor_id)
                    next_level.append(neighbor_id)
            if next_level:
                levels.append(next_level)
            current = next_level
        return levels

    def find_root(self, patch_ids: Optional[set[int]] = None, strategy: str = "MAX_AREA") -> int:
        candidates = patch_ids or set(self.nodes.keys())
        if not candidates:
            raise ValueError("PatchGraph is empty")
        if strategy == "MIN_AREA":
            return min(candidates, key=lambda patch_id: self.nodes[patch_id].area)
        return max(candidates, key=lambda patch_id: self.nodes[patch_id].area)

    def connected_components(self) -> list[set[int]]:
        components = []
        visited = set()
        for patch_id in self.nodes:
            if patch_id in visited:
                continue
            component = set()
            stack = [patch_id]
            while stack:
                current_id = stack.pop()
                if current_id in visited:
                    continue
                visited.add(current_id)
                component.add(current_id)
                for neighbor_id in self.get_neighbors(current_id):
                    if neighbor_id not in visited:
                        stack.append(neighbor_id)
            components.append(component)
        return components


# ============================================================
# ScaffoldMap — solve IR (persistent result)
# Виртуальная 2D карта размещения chains/corners/patches.
# Может быть кэширован, отредактирован вручную, частично пересчитан.
# ============================================================

@dataclass(frozen=True)
class ScaffoldPointKey:
    """Уникальная ссылка на source point в PatchGraph."""

    patch_id: int
    loop_index: int
    chain_index: int
    source_point_index: int


@dataclass(frozen=True)
class ScaffoldChainPlacement:
    """Размещённый chain с UV координатами."""

    patch_id: int
    loop_index: int
    chain_index: int
    frame_role: FrameRole
    source_kind: str = "chain"
    points: tuple[tuple[ScaffoldPointKey, Vector], ...] = ()


@dataclass
class ScaffoldPatchPlacement:
    """Per-patch envelope — результат scaffold placement."""

    patch_id: int
    loop_index: int
    root_chain_index: int = 0
    corner_positions: dict[int, Vector] = field(default_factory=dict)
    chain_placements: list[ScaffoldChainPlacement] = field(default_factory=list)
    bbox_min: Vector = field(default_factory=lambda: Vector((0.0, 0.0)))
    bbox_max: Vector = field(default_factory=lambda: Vector((0.0, 0.0)))
    closure_error: float = 0.0
    max_chain_gap: float = 0.0
    gap_reports: tuple[tuple[int, int, float], ...] = ()
    closure_valid: bool = True
    notes: tuple[str, ...] = ()
    # Phase 3: envelope fields
    status: str = "COMPLETE"
    dependency_patches: tuple = ()
    unplaced_chain_indices: tuple = ()
    pinned: bool = False
    origin_offset: tuple = (0.0, 0.0)  # sleeping field, Phase 5


@dataclass
class ScaffoldQuiltPlacement:
    """Per-quilt scaffold — коллекция patch placements с порядком сборки."""

    quilt_index: int
    root_patch_id: int
    patches: dict[int, ScaffoldPatchPlacement] = field(default_factory=dict)
    build_order: list = field(default_factory=list)  # Phase 3: ChainRef tuples


@dataclass
class ScaffoldMap:
    """Корневой контейнер solve результата — коллекция quilts."""

    quilts: list[ScaffoldQuiltPlacement] = field(default_factory=list)
