# CFTUV Band Spine Parametrization — Implementation Plan

## Problem Statement

Из логов (6 patches: P0-P5):
- P0, P1, P3, P5 — обычные H/V патчи, Quilt 0, работают: 16/16 chains, pinned=88
- P2, P4 — BAND патчи, каждый в отдельном Quilt (Q1, Q2)
- Классификатор structural_tokens корректно определяет BAND: `P2 -> BAND (cap_sim=1.00)`, `P4 -> BAND (cap_sim=1.00)`
- Runtime reject: `band_rt_reject=ambiguous_caps` -> блокирует всё
- Результат: CAP chains -> FREE [BRIDGE], pinned=30/60, половина UV conformal

Frontier trace для Q1 (P4):
```
Seed: P4/C2 STRAIGHTEN -> V_FRAME            # SIDE ok
Step1: P4/C3 FREE [BRIDGE]                    # CAP fail
Step2: P4/C0 V_FRAME                          # SIDE ok (через continuation? нет — напрямую из straighten_chain_refs)
Step3: P4/C1 FREE [BRIDGE]                    # CAP fail
```

Обе SIDE получают STRAIGHTEN -> V_FRAME. Обе CAP падают в FREE.

---

## Root Cause Analysis

### Почему `ambiguous_caps`

Файл: `cftuv/analysis_derived.py` lines 800-812

```python
runtime_cap_indices = [
    chain_index
    for chain_index, runtime_role in enumerate(runtime_role_sequence)
    if runtime_role == orthogonal_axis   # V_FRAME для H-spine
]
```

`runtime_role_sequence` строится на line 786-793:
```python
for chain_index, chain in enumerate(outer_chains):
    runtime_role = effective_role_by_chain_index.get(chain_index, chain.frame_role)
    if runtime_role == FrameRole.FREE:         # <- только FREE перезаписывается
        if chain_index in side_candidate_indices:
            runtime_role = runtime_band_axis
        elif chain_index in cap_candidate_indices:
            runtime_role = orthogonal_axis
    runtime_role_by_chain_index[chain_index] = runtime_role
```

**Проблема**: если chain уже имеет `effective_role = V_FRAME` от наследования (`inherited_spines=1`), он НЕ перезаписывается (не FREE). Но его роль V_FRAME совпадает с `orthogonal_axis` -> он считается "третьим CAP" -> `len(runtime_cap_indices) = 3` -> `ambiguous_caps`.

P2 лог: `inherited_spines=1 junction_axis=H_FRAME single_sided=Y`. Один chain унаследовал H/V от соседа. Этот inherited chain попадает в runtime_role_sequence с V_FRAME, создавая 3-й cap.

### Почему отдельные Quilts

P2 и P4 не имеют tree edges к Q0. Tree edges строятся по H/V frame seams. У BAND патчей все chains FREE (native), STRAIGHTEN — только effective role. Tree builder видит FREE-FREE seams -> нет edge -> отдельный Quilt.

---

## Implementation Details

### File: `cftuv/analysis_records.py`

**Добавить после line ~400 (после `_PatchGraphDerivedTopology`):**

```python
from mathutils import Vector  # уже импортирован

@dataclass(frozen=True)
class BandSpineData:
    """Pre-computed spine parametrization for a BAND patch."""
    patch_id: int
    side_a_ref: ChainRef
    side_b_ref: ChainRef
    cap_start_ref: ChainRef        # CAP at V=0 end
    cap_end_ref: ChainRef          # CAP at V=1 end
    spine_points: tuple[Vector, ...]        # 3D midpoints along spine
    spine_arc_lengths: tuple[float, ...]    # cumulative arc length at each spine point, normalized [0..1]
    total_arc_length: float
    cap_start_width: float          # 3D cross-band distance at V=0
    cap_end_width: float            # 3D cross-band distance at V=1
    chain_uv_targets: dict[ChainRef, tuple[tuple[float, float], ...]]
        # (U, V) for each vert of each of the 4 chains
    spine_axis: FrameRole           # resolved axis (H or V) from _resolve_straighten_axis
```

**Добавить поле в `_PatchGraphDerivedTopology`:**
```python
band_spine_data: dict[int, BandSpineData] = field(default_factory=dict)
    # patch_id -> BandSpineData, only for patches classified as BAND
```

---

### File: NEW `cftuv/band_spine.py`

```python
"""Band spine pre-parametrization.

Builds a midpoint spine curve between two SIDE chains of a BAND patch
and computes (U, V) coordinates for all 4 boundary chains.

V = normalized arc length along spine [0..1]
U = signed cross-band distance from spine (positive = side_a, negative = side_b)
"""
```

**Функции:**

#### `build_band_spine_data(graph, patch_id, loop_sig, shape_class) -> Optional[BandSpineData]`
- Вход: PatchGraph, patch_id, LoopSignature (с токенами), PatchShapeClass
- Если shape != BAND -> return None
- Извлечь SIDE/CAP chain refs из tokens (effective_frame_role == STRAIGHTEN -> SIDE, остальные FREE -> CAP)
- Определить ориентацию: какой CAP на start (V=0), какой на end (V=1)
  - Используя corner connectivity: SIDE_A.start_corner -> prev_chain = CAP_start
  - Или: SIDE_A.end_corner -> next_chain = CAP_end
- Вызвать `_build_spine()` и `_parametrize_chains()`

#### `_orient_sides(graph, patch_id, side_a_ref, side_b_ref, cap_refs) -> tuple[vert_cos_a, vert_cos_b, cap_start_ref, cap_end_ref]`
- Убедиться что side_a и side_b идут в одном направлении
- Проверка: `side_a.vert_cos[0]` ближе к `side_b.vert_cos[0]` (same end) или к `side_b.vert_cos[-1]` (opposite end)
- Если opposite -> развернуть side_b: `vert_cos_b = reversed(vert_cos_b)`
- Определить cap_start = CAP, чей endpoint совпадает с side_a.vert_cos[0] и side_b.vert_cos[0] (через corners)
- Определить cap_end = другой CAP

#### `_build_spine(vert_cos_a, vert_cos_b) -> tuple[spine_points, arc_lengths, total_length]`
- Ресамплинг: привести обе SIDE к одинаковому числу точек N = max(len_a, len_b)
  - Использовать линейную интерполяцию по arc length
- Midpoints: `spine[i] = (resampled_a[i] + resampled_b[i]) / 2`
- Arc lengths: cumulative sum of `(spine[i+1] - spine[i]).length`
- Normalize: `arc_lengths[i] /= total_length`

#### `_parametrize_side(spine_points, spine_arc_lengths, side_vert_cos, sign) -> tuple[tuple[float, float], ...]`
- Для каждой вершины SIDE chain:
  - Проецировать на ближайший сегмент spine
  - V = интерполированный arc_length в точке проекции
  - U = sign * расстояние от spine до вершины
- sign = +1 для side_a, -1 для side_b

#### `_parametrize_cap(spine_points, spine_arc_lengths, cap_vert_cos, v_value, cap_width) -> tuple[tuple[float, float], ...]`
- Для каждой вершины CAP chain:
  - V = v_value (0.0 или 1.0)
  - U = interpolated position across band width, from +cap_width/2 to -cap_width/2
  - Нормализация: U пропорционален положению вершины CAP между двумя SIDE endpoints

#### `_resolve_spine_axis(chain_a, node) -> FrameRole`
- Reuse `_resolve_straighten_axis()` из `frontier_place.py`
- Или дублировать логику: project chord onto basis, dominant component -> H or V

---

### File: `cftuv/analysis_derived.py`

**Изменения в `_build_patch_graph_derived_topology()` (после line 1063):**

```python
# After building straighten_chain_refs...
from .band_spine import build_band_spine_data

band_spine_data: dict[int, BandSpineData] = {}
for patch_id, shape_class in patch_shape_classes.items():
    if shape_class != PatchShapeClass.BAND:
        continue
    sigs = loop_signatures.get(patch_id, [])
    if not sigs:
        continue
    spine = build_band_spine_data(graph, patch_id, sigs[0], shape_class)
    if spine is not None:
        band_spine_data[patch_id] = spine
```

**Передать в конструктор `_PatchGraphDerivedTopology`:**
```python
return _PatchGraphDerivedTopology(
    ...
    straighten_chain_refs=frozenset(straighten_chain_refs),
    band_spine_data=band_spine_data,     # <-- новое поле
)
```

**Fix `ambiguous_caps` (line 800-812):**

Вариант: если structural_tokens уже определил BAND, доверять им:

```python
# Before building runtime_cap/side_indices, check structural classification
structural_shape = patch_shape_classes.get(patch_id)
if structural_shape == PatchShapeClass.BAND:
    # Use structural token SIDE/CAP assignment instead of runtime heuristic
    structural_side_indices = []
    structural_cap_indices = []
    for sig in loop_signatures.get(patch_id, []):
        for token in sig.chain_tokens:
            if token.chain_role_class == ChainRoleClass.SIDE:
                structural_side_indices.append(token.chain_ref[2])  # chain_index
            elif token.chain_role_class == ChainRoleClass.CAP:
                structural_cap_indices.append(token.chain_ref[2])
    if len(structural_side_indices) == 2 and len(structural_cap_indices) == 2:
        runtime_side_indices = structural_side_indices
        runtime_cap_indices = structural_cap_indices
        # Rebuild runtime_role_sequence from structural assignment
        for si in structural_side_indices:
            runtime_role_by_chain_index[si] = runtime_band_axis
        for ci in structural_cap_indices:
            runtime_role_by_chain_index[ci] = orthogonal_axis
        runtime_role_sequence = tuple(
            runtime_role_by_chain_index.get(i, FrameRole.FREE)
            for i in range(chain_count)
        )
```

Вставить после line 798, перед line 799 (`runtime_side_indices = []`).

**ВАЖНО**: `patch_shape_classes` и `loop_signatures` нужно передать в эту функцию. Сейчас `_build_patch_topology_summaries()` не получает их напрямую. Проверить call chain и добавить параметры если нужно.

---

### File: `cftuv/analysis.py`

**`build_straighten_structural_support()` — расширить return:**

```python
# Было: return (inherited_role_map, patch_structural_summaries, patch_shape_classes, straighten_chain_refs)
# Стало:
return (inherited_role_map, patch_structural_summaries, patch_shape_classes, straighten_chain_refs, band_spine_data)
```

`band_spine_data` приходит из `_PatchGraphDerivedTopology.band_spine_data`.

---

### File: `cftuv/operators.py`

**Unpack 5-tuple:**
```python
# Было:
inherited_role_map, patch_structural_summaries, patch_shape_classes, straighten_chain_refs = (
    build_straighten_structural_support(...)
)
# Стало:
inherited_role_map, patch_structural_summaries, patch_shape_classes, straighten_chain_refs, band_spine_data = (
    build_straighten_structural_support(...)
)
```

Передать `band_spine_data` в `build_root_scaffold_map()`.

---

### File: `cftuv/solve_transfer.py`

Аналогично `operators.py` — unpack 5-tuple, передать дальше.

---

### File: `cftuv/solve_frontier.py`

**`build_root_scaffold_map()` (line 590):**
- Добавить параметр: `band_spine_data: Optional[dict[int, BandSpineData]] = None`
- Передать в `build_quilt_scaffold_chain_frontier()`

**`build_quilt_scaffold_chain_frontier()` (line 394):**
- Добавить параметр: `band_spine_data`
- Передать в `_cf_bootstrap_frontier_runtime()` -> `FrontierRuntimePolicy.__init__()`

---

### File: `cftuv/frontier_state.py`

**`FrontierRuntimePolicy.__init__()`:**
- Добавить поле: `self.band_spine_data: dict[int, BandSpineData] = band_spine_data or {}`

**`effective_placement_role()` (line 109) — добавить CAP promotion:**

```python
def effective_placement_role(self, chain_ref, chain):
    if chain.frame_role != FrameRole.FREE:
        return chain.frame_role
    if chain_ref in self.straighten_chain_refs:
        return FrameRole.STRAIGHTEN

    # NEW: CAP of BAND gets orthogonal role
    patch_id = chain_ref[0]
    spine = self.band_spine_data.get(patch_id)
    if spine is not None:
        if chain_ref == spine.cap_start_ref or chain_ref == spine.cap_end_ref:
            # CAP gets orthogonal to spine axis
            if spine.spine_axis == FrameRole.H_FRAME:
                return FrameRole.V_FRAME
            elif spine.spine_axis == FrameRole.V_FRAME:
                return FrameRole.H_FRAME

    # ... rest of existing logic (inherited, runtime, etc.)
```

---

### File: `cftuv/frontier_place.py`

**Добавить функцию `_build_spine_chain_placement()`:**

```python
def _build_spine_chain_placement(
    chain_ref: ChainRef,
    chain: BoundaryChain,
    node: PatchNode,
    spine_data: BandSpineData,
    anchor_offset: Vector,         # UV offset from frontier anchor
    anchor_scale: tuple[float, float],  # (scale_u, scale_v) drift correction
    final_scale: float,
) -> list[tuple[ScaffoldPointKey, Vector]]:
    """Build placement points from pre-computed spine UV targets."""
    uv_targets = spine_data.chain_uv_targets.get(chain_ref)
    if uv_targets is None:
        return []  # fallback to standard placement

    points = []
    for vert_index, (u, v) in enumerate(uv_targets):
        # Apply drift correction
        corrected_u = u * anchor_scale[0]
        corrected_v = v * anchor_scale[1]
        # Apply anchor offset
        uv = Vector((corrected_u + anchor_offset.x, corrected_v + anchor_offset.y))
        key = ScaffoldPointKey(chain_ref[0], chain_ref[1], chain_ref[2], vert_index)
        points.append((key, uv))
    return points
```

**В `_cf_place_chain()` (line 1213):**

```python
# At entry, check if chain belongs to BAND with spine data
spine_data = runtime_policy.band_spine_data.get(chain_ref[0]) if runtime_policy else None
if spine_data is not None and chain_ref in spine_data.chain_uv_targets:
    # Use spine placement instead of standard
    anchor_offset, anchor_scale = _compute_spine_anchor_transform(
        spine_data, start_anchor, end_anchor
    )
    points = _build_spine_chain_placement(
        chain_ref, chain, node, spine_data,
        anchor_offset, anchor_scale, final_scale
    )
    if points:
        return ScaffoldChainPlacement(
            patch_id=chain_ref[0],
            loop_index=chain_ref[1],
            chain_index=chain_ref[2],
            frame_role=effective_role,  # resolved H/V
            points=tuple(points),
            ...
        )
```

**Добавить `_compute_spine_anchor_transform()`:**
```python
def _compute_spine_anchor_transform(
    spine_data: BandSpineData,
    start_anchor: Optional[ChainAnchor],
    end_anchor: Optional[ChainAnchor],
) -> tuple[Vector, tuple[float, float]]:
    """Compute offset and scale from anchor positions to spine UV space."""
    # Default: no transform
    offset = Vector((0.0, 0.0))
    scale = (1.0, 1.0)

    # If anchor provides a position, align spine origin to it
    if start_anchor is not None and start_anchor.uv is not None:
        # Spine UV (0,0) maps to anchor UV
        spine_origin_uv = spine_data.chain_uv_targets[...][0]  # first point of chain
        offset = start_anchor.uv - Vector(spine_origin_uv)

    # If both anchors exist on a CAP, compute U-scale drift
    if start_anchor is not None and end_anchor is not None:
        real_span = (end_anchor.uv - start_anchor.uv).length
        spine_span = spine_data.cap_start_width  # or cap_end_width
        if spine_span > 1e-9:
            scale = (real_span / spine_span, scale[1])

    return offset, scale
```

---

### File: `cftuv/frontier_score.py`

**`_cf_role_tier()` (line 233) — CAP of BAND:**

```python
# After STRAIGHTEN check (line 250):
# Check if chain is CAP of BAND with spine
spine = runtime_policy.band_spine_data.get(chain_ref[0]) if runtime_policy else None
if spine is not None:
    if chain_ref == spine.cap_start_ref or chain_ref == spine.cap_end_ref:
        return 2, 'band_cap_spine'    # Same tier as STRAIGHTEN
```

---

### File: `cftuv/solve_pin_policy.py`

**`_decide_chain_pin()` (line 95) — BAND spine rule:**

```python
# At entry, before H/V check:
# If chain belongs to BAND with spine data -> pin all
if band_spine_data is not None and chain_placement.patch_id in band_spine_data:
    spine = band_spine_data[chain_placement.patch_id]
    chain_ref = (chain_placement.patch_id, chain_placement.loop_index, chain_placement.chain_index)
    if chain_ref in spine.chain_uv_targets:
        return ChainPinDecision(pin_all=True, reason='band_spine_connected')
```

**`build_patch_pin_map()` — передать `band_spine_data`:**
- Добавить параметр
- Передать в `_decide_chain_pin()`

---

## Data Flow Summary (After Changes)

```
structural_tokens.py          band_spine.py
      |                            |
  classify BAND              build_band_spine_data()
  SIDE/CAP tokens            midpoint spine, U/V coords
      |                            |
      +------- analysis_derived.py --------+
              |                            |
        straighten_chain_refs      band_spine_data
              |                            |
              +---- analysis.py (5-tuple) -+
                           |
              +---- operators.py / solve_transfer.py ----+
                           |
              +---- solve_frontier.py ---+
                           |
              +---- frontier_state.py ---+
              |      effective_placement_role():
              |        SIDE -> STRAIGHTEN
              |        CAP  -> H/V (from spine axis)
              |
    +---------+---------+
    |                   |
frontier_score.py   frontier_place.py
  CAP tier=2          _build_spine_chain_placement()
  'band_cap_spine'    spine UV + anchor drift
    |                   |
    +------- frontier_eval.py --------+
                    |
           solve_pin_policy.py
             pin_all for all BAND chains
                    |
             solve_transfer.py
               apply UV, pin, conformal
```

---

## Critical Invariants

1. **SIDE = FREE pair** (structural_tokens). H/V chains NEVER SIDE. Inviolable.
2. **CAP similarity >= 0.5** (structural_tokens). Diverging CAPs = not BAND.
3. **Spine axis resolved once** at pre-computation, not at each placement.
4. **Both SIDEs same direction** along spine before midpoint computation. Always verify orientation.
5. **Anchor drift scales U only**, not V. V comes from arc length, stable.
6. **CAP width can differ** at start vs end (trapezoidal band). Interpolate U-scale along V.
7. **Spine UV is patch-local**. Global UV = spine UV + anchor transform.
8. **Pin all 4 BAND chains**. No conformal fallback for BAND internals.

---

## Test Scenarios

| Scenario | Expected |
|----------|----------|
| Straight band (P2, P4 from logs) | Both SIDEs + both CAPs pinned. pinned=60/60. No [BRIDGE] |
| C-shape band | SIDEs follow arc. Spine follows curve. U consistent across band |
| BAND adjacent to H/V patch | CAP anchor from neighbor. Drift correction adjusts U-scale |
| Isolated BAND (no neighbors) | Spine UV used as-is. Anchor offset = (0,0) |
| Band with different CAP widths | Trapezoidal. cap_start_width != cap_end_width. U interpolated |
| Very short CAP (2 verts) | Still gets spine UV. U = +-width/2 |

---

## Open Questions for Review

1. **Bezier vs polyline spine**: для первой версии полилиния из midpoints достаточна. Bezier fitting — оптимизация на потом.
2. **Ресамплинг SIDEs**: если side_a имеет 10 вершин, side_b — 13, нужна интерполяция. Ресамплить по arc length к max(len_a, len_b).
3. **STRAIGHTEN resolve timing**: spine_axis вычисляется при build_band_spine_data(). Совпадает ли с `_resolve_straighten_axis()` в frontier_place.py? Должен совпадать — одна и та же геометрия, тот же node basis.
4. **Tree edges для BAND**: сейчас BAND патчи изолированы в отдельных quilts. Если CAP получает H/V role от spine, нужно ли добавлять tree edge? Вероятно нет — tree edges строятся по native frame_role, не effective.
