# SKILL: PatchGraph — Working with the Central Data Model

## When to use
Any task that reads, writes, or transforms PatchGraph data.
This is the IR (intermediate representation) of the entire system.

## Data flow rule

```
analysis.py  ──WRITES──>  PatchGraph  ──READ BY──>  solve.py
                                      ──READ BY──>  debug.py
```

Никто кроме analysis.py не создаёт PatchGraph.
Никто не модифицирует PatchGraph после создания (treat as immutable after build).

## Creating a PatchGraph (only in analysis.py)

```python
from .model import PatchGraph, PatchNode, SeamEdge, PatchType, LoopKind, FrameRole

graph = PatchGraph()

# Добавление узла
node = PatchNode(
    patch_id=0,
    face_indices=[1, 2, 3, 4],
    centroid=Vector((0, 0, 0)),
    normal=Vector((0, 0, 1)),
    area=2.5,
    perimeter=6.0,
    patch_type=PatchType.FLOOR,
    basis_u=Vector((1, 0, 0)),
    basis_v=Vector((0, 1, 0)),
)
graph.add_node(node)    # автоматически заполняет face_to_patch и _adjacency

# Добавление связи
seam = SeamEdge(
    patch_a_id=0, patch_b_id=1,
    shared_length=1.5,
    shared_vert_indices=[10, 11, 12],
    longest_edge_verts=(10, 12),
    longest_edge_length=0.8,
)
graph.add_edge(seam)    # автоматически обновляет _adjacency
```

## Querying PatchGraph (in solve.py, debug.py)

```python
# Соседи патча — O(1)
neighbors: set[int] = graph.get_neighbors(patch_id)

# Шов между двумя патчами
seam: SeamEdge = graph.get_seam(patch_a, patch_b)  # порядок не важен

# BFS от корня
levels: list[list[int]] = graph.traverse_bfs(root_id)
# levels[0] = [root], levels[1] = [direct neighbors], ...

# Корень (самый большой patch)
root = graph.find_root(strategy='MAX_AREA')

# Связные компоненты
components: list[set[int]] = graph.connected_components()

# Face → patch lookup
patch_id = graph.face_to_patch[face_index]
```

## Accessing BMesh through PatchGraph (in solve.py)

PatchGraph хранит индексы, BMesh хранит объекты. Конвертация:

```python
node = graph.nodes[patch_id]

# Получить BMFace объекты
faces = [bm.faces[fi] for fi in node.face_indices]

# Получить UV для patch
uv_layer = bm.loops.layers.uv.verify()
for fi in node.face_indices:
    for loop in bm.faces[fi].loops:
        uv = loop[uv_layer].uv
```

## Boundary topology

```python
node = graph.nodes[patch_id]

for loop in node.boundary_loops:
    # loop.kind: LoopKind.OUTER or LoopKind.HOLE
    # loop.vert_cos: list[Vector] — 3D coords по порядку обхода
    # loop.depth: int — nesting depth (0 = outermost)

    for chain in loop.chains:
        # chain.neighbor_patch_id: int (или NB_MESH_BORDER / NB_SEAM_SELF)
        # chain.frame_role: FrameRole.H_FRAME / V_FRAME / FREE
        # chain.vert_cos: list[Vector]
        # chain.is_closed: bool
```

## Common patterns

```python
# Обработать все WALL патчи
for pid, node in graph.nodes.items():
    if node.patch_type == PatchType.WALL:
        ...

# Найти все H_FRAME chains патча
h_chains = [
    chain
    for loop in node.boundary_loops
    for chain in loop.chains
    if chain.frame_role == FrameRole.H_FRAME
]

# Обойти граф от самого большого patch, уровень за уровнем
root = graph.find_root()
for level in graph.traverse_bfs(root):
    for pid in level:
        node = graph.nodes[pid]
        # process node...
```
