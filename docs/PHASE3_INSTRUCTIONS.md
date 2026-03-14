# Phase 3 Instructions
## Handoff Notes For The Next AI Session

---

## Important

Старые инструкции вида "вставь блок перед `build_root_scaffold_map()`" больше не актуальны.

Phase 3 уже внедрена в кодовую базу. Этот файл теперь нужен как короткий handoff:

- что считать текущим baseline;
- какие вещи уже стабилизированы;
- куда смотреть в коде;
- чего не нужно ломать при следующем фикс-проходе.

---

## Current Entry Points

### analysis.py

Смотри в первую очередь:

- `build_patch_graph()`
- `_split_loop_into_chains_by_neighbor()`
- `_classify_chain_frame_role()`
- `_build_geometric_loop_corners()`
- `_try_geometric_outer_loop_split()`

### solve.py

Смотри в первую очередь:

- `build_solver_graph()`
- `plan_solve_phase1()`
- `build_quilt_scaffold_chain_frontier()`
- `_cf_choose_seed_chain()`
- `_cf_find_anchors()`
- `_cf_resolve_candidate_anchors()`
- `_cf_score_candidate()`
- `_cf_place_chain()`
- `_resolve_scaffold_uv_targets()`
- `_apply_patch_scaffold_to_uv()`
- `_execute_phase1_preview_impl()`

### operators.py

Смотри:

- `_prepare_patch_graph()`
- `validate_solver_input_mesh()`

---

## Current Stable Rules

Эти правила уже доказали свою пользу и не должны откатываться без сильной причины.

1. Solve path остается chain-first strongest-frontier.
2. Semantic/scoring работает только внутри текущего quilt.
3. Между разными `PatchType` propagation не строится.
4. `cross_patch + cross_patch` dual-anchor closure запрещен для раннего замыкания patch.
5. Для H/V dual-anchor placement работают `axis_mismatch` / `span_mismatch` guards.
6. Mesh preflight должен стопать solve на duplicate / non-manifold / degenerate topology.
7. `OUTER` loop isolated patch может использовать geometric fallback split.
8. `HOLE` loops в этот fallback не входят.
9. UV transfer validation нельзя отключать ради "тихой" работы.
10. Unsupported patches должны иметь fallback conformal pass, а не исчезать молча.

---

## Current Known Debt

Новый агент должен знать, что это еще не доведено:

1. Solve dataclasses пока живут в `solve.py`.
2. Persistent `anchor_registry` в `ScaffoldMap` пока нет.
3. Explicit `ContinuationEdge` пока нет.
4. `ScaffoldSegment` legacy artifact еще лежит в `model.py`.
5. `format_root_scaffold_report()` еще не равен полноценному frontier replay.

---

## Current Known Problem Area

Главная незакрытая тема на момент handoff:

### Isolated single patch with holes

Сценарий:

- одна плоская wall patch;
- outer loop теперь правильно режется в `H/V/FREE`;
- scaffold строится;
- но поздний `Conformal` может визуально почти не расслаблять interior.

Что важно:

- по логам `Conformal` реально вызывается;
- значит проблема, скорее всего, уже в pin policy или слишком жестком scaffold boundary,
  а не в отсутствии вызова `bpy.ops.uv.unwrap()`.

То есть следующий агент должен смотреть не "почему unwrap skipped", а:

- какие UV loops pinятся;
- нужно ли для isolated quilt пинить весь `H_FRAME/V_FRAME` или только anchors/corners;
- как не поломать multi-patch continuity.

---

## Recommended Next Debug Order

Если следующий агент продолжает работу, идти лучше так:

1. Проверить isolated single-patch case.
2. Сравнить `resolved_scaffold_points`, `uv_targets`, `pinned`.
3. Проверить визуальный эффект после `Patch X Conformal` логов.
4. Чинить pin policy, а не откатывать frontier.
5. Только потом улучшать report/debug layer.

---

## Validation Mesh Set

После любых правок желательно снова прогонять эти классы кейсов:

1. Tube / cylinder без caps, 2 vertical seams.
2. Pseudo-cube with walls + caps.
3. Semi-column:
   - front wall
   - back wall
   - top cap
   - bottom cap
4. Isolated back wall patch.
5. Isolated flat wall patch with inner window holes.

Если правка ломает первые три кейса ради isolated wall, это почти наверняка плохой tradeoff.

---

## Practical Warning

Если видишь странный UV-результат, сначала исключи mesh corruption:

- duplicate faces;
- bridge поверх существующего ngon;
- non-manifold edges.

Сейчас preflight это уже ловит, но при новых операторах или обходных путях этот gate
можно случайно обойти.

---

## Short Summary For The Next Agent

Phase 3 уже активна и в целом рабочая.

Текущий baseline:

- frontier стабилен;
- wrap guards стабилизированы;
- preflight работает;
- target resolution и validation работают;
- conformal вызывается per-patch.

Текущая незакрытая задача:

- isolated patch pinning / late conformal behavior после outer geometric split.

