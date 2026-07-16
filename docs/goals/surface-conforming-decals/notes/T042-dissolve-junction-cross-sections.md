# T042 — Remove obsolete junction cross-sections

После T039 биссекторная линия стала правильной, но branch и junction half-face
оставались двумя coplanar faces. Их общий endpoint section был виден как старая
прямоугольная линия рядом с биссектрисой.

Исправление является частью junction assembly, а не финальным cosmetic cleanup:

- `_DecalSurfaceSection` переносит точные UV anchors с branch endpoint;
- junction half-face использует эти anchors и продолжает longitudinal UV от
  той же station;
- сразу после создания half-face её общий cross-section edge dissolves вместе
  с соответствующей branch face;
- разные coincident BMesh vertices сохраняются до штатного final weld, чтобы
  обе соседние branches успели удалить собственные cross-sections;
- смысловые `core → spine` и `core → averaged miter` edges не dissolves.

Blender 4.3.2 и 4.1.1 дали `remaining_cross_edges: 0` для planar T и trihedral.
Оба fixtures имеют по шесть итоговых continuous branch faces, все шесть core
edges разделены двумя faces; loose и overfull geometry отсутствует. Полный
Python suite: `56 passed`.

После reload через MCP тот же production scope пользователя из пяти seam edges
был перегенерирован прямо в открытой Blender 4.3 сцене. Mesh изменился с
`26 verts / 41 edges / 16 faces` на `26 / 35 / 10`: исчезли ровно шесть
branch-end cross-sections, при этом vertex count и биссекторная конструкция
сохранились.
