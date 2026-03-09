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
    """Continuous part of a boundary loop with one neighbor."""

    vert_cos: list[Vector] = field(default_factory=list)
    edge_indices: list[int] = field(default_factory=list)
    neighbor_patch_id: int = -1
    is_closed: bool = False
    frame_role: FrameRole = FrameRole.FREE

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


@dataclass
class BoundaryCorner:
    """Junction corner between two neighboring chains inside one loop."""

    loop_vert_index: int = 0
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

    vert_cos: list[Vector] = field(default_factory=list)
    edge_indices: list[int] = field(default_factory=list)
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

