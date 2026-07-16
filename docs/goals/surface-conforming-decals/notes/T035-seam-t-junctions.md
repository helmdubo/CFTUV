# T035 — Decal Seams network T-junctions

Valence `3+` остаётся границей oriented chain-runs: producer не выбирает
произвольную ветку и не превращает Junction в solve entity. Для `Decal Seams`
поверх этих runs теперь строится decal-only junction layer.

- incident owner normals задают offset-плоскости;
- их least-squares intersection создаёт общий junction core;
- branches обрезаются до общей boundary с учётом `Seam Width`, decal offset и
  доступной длины первого сегмента;
- centered coplanar seams используют MITER/BEVEL joins и явный spine edge;
- cross-sections сортируются вокруг surface normal и образуют отдельный patch
  на каждой owner surface;
- junction patch разделяет реальные BMesh edges с branch faces после weld,
  поэтому нет coplanar overlap или edge с тремя faces;
- схема работает для valence `3+`, включая плоский T и trihedral corner.

Blender 4.3.2 full manual selected-edge path для плоского T дал 16 vertices,
7 faces и 7-sided junction patch. Шесть junction edges имеют по две faces,
единственное нижнее ребро является внешней границей; overfull edges `0`.
Trihedral fixture дал общий core `(0.02, 0.02, 0.02)`, 16 vertices, 9 quads,
три surface junction faces, minimum area `0.01`, loose vertices `0`.
Blender 4.1.1 повторил плоский T результат `16/7` с той же edge topology.
