# Phase 3 — Пошаговые правки solve.py

## 4 правки. Выполнять строго по порядку.

---

## ПРАВКА 1: Новые поля ScaffoldPatchPlacement

Открой solve.py, найди (Ctrl+F):

```python
    notes: tuple[str, ...] = ()
```

Это последняя строка dataclass `ScaffoldPatchPlacement`.
СРАЗУ ПОСЛЕ неё добавь:

```python
    # Phase 3: envelope fields
    status: str = "COMPLETE"
    dependency_patches: tuple = ()
    unplaced_chain_indices: tuple = ()
    pinned: bool = False
    origin_offset: tuple = (0.0, 0.0)  # sleeping field, Phase 5
```

---

## ПРАВКА 2: build_order в ScaffoldQuiltPlacement

Найди:

```python
    patches: dict[int, ScaffoldPatchPlacement] = field(default_factory=dict)
```

Это внутри dataclass `ScaffoldQuiltPlacement`.
СРАЗУ ПОСЛЕ добавь:

```python
    build_order: list = field(default_factory=list)
```

---

## ПРАВКА 3: Вставить новые функции

Найди:

```python
def build_root_scaffold_map(
```

ПЕРЕД этой строкой вставь ВЕСЬ код из файла `phase3_chain_frontier.py`,
начиная с `CHAIN_FRONTIER_THRESHOLD = 0.3` и до строки
`return quilt_scaffold` (конец функции `build_quilt_scaffold_chain_frontier`).

НЕ вставляй закомментированный блок "ПРАВКА 4" из того файла.

---

## ПРАВКА 4: Заменить build_root_scaffold_map

Найди СУЩЕСТВУЮЩУЮ функцию `build_root_scaffold_map`.
Она большая (30-50 строк). Замени ЕЁ ЦЕЛИКОМ на:

```python
def build_root_scaffold_map(
    graph: PatchGraph,
    solve_plan: Optional[SolvePlan] = None,
    final_scale: float = 1.0,
) -> ScaffoldMap:
    """Build ScaffoldMap using chain-first strongest-frontier algorithm."""
    scaffold_map = ScaffoldMap()
    if solve_plan is None:
        return scaffold_map

    for quilt in solve_plan.quilts:
        quilt_scaffold = build_quilt_scaffold_chain_frontier(graph, quilt, final_scale)
        scaffold_map.quilts.append(quilt_scaffold)

    return scaffold_map
```

Старый код build_root_scaffold_map (который строил patch-by-patch) удаляется.
Старые функции (_build_root_patch_scaffold, _build_child_patch_scaffold и т.д.)
можно оставить как мёртвый код — они больше не вызываются.

---

## Проверка

1. Сохрани solve.py
2. Перезапусти Blender
3. Выдели faces на Cylinder.007
4. Нажми "Scaffold Debug" — должен показать новый формат с [Frontier] логами
5. Нажми "Solve Phase 1 Preview" — должен работать с [Validate] и [Frontier] логами

### Что должно быть в System Console:

```
[CFTUV][Frontier] Seed: P0 L0C2 H_FRAME (0.0000,0.0000)->(0.8331,0.0000)
[CFTUV][Frontier] Step 1: P0 L0C3 V_FRAME score:1.80 ep:1
[CFTUV][Frontier] Step 2: P0 L0C1 V_FRAME score:1.50 ep:1
...
[CFTUV][Frontier] Quilt 0: placed X/Y chains
```

Chains размещаются по score, не по порядку loop. H/V с anchors идут первыми.
FREE без anchors остаются не размещёнными → Conformal доработает.

---

## Если что-то сломалось

Самые вероятные проблемы:

1. **ImportError** — убедись что новые функции вставлены ПЕРЕД build_root_scaffold_map,
   а не после. Они используют _build_frame_chain_from_one_end и другие функции,
   определённые выше в файле.

2. **AttributeError: 'ScaffoldQuiltPlacement' has no attribute 'build_order'** —
   Правка 2 не применена.

3. **AttributeError: 'ScaffoldPatchPlacement' has no attribute 'status'** —
   Правка 1 не применена.

4. **Validation mismatches** — новый builder строит scaffold иначе.
   Если [Validate] показывает mismatches — скинь лог, разберёмся.
