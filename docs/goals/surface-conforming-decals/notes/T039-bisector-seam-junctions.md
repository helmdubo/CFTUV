# T039 — Bisector divider for Decal Seams junctions

Проверка T037 показала, что radial branch-spine edges не соответствовали
пользовательской красной схеме: общий surface sector оставался визуально одной
декалью. Требуемая линия идёт между соседними chains по усреднённому направлению.

Исправленная схема:

- реальная сторона one-sided wing определяет занятый angular region; shortest
  gap больше не подменяет owner-surface topology;
- outer-контуры соседних branches пересекаются как бесконечные линии;
- bounded miter point либо midpoint fallback задаёт внешний конец divider;
- ребро `common core → miter point` разделяет region на две реальные half-faces;
- каждая half-face получает UV в frame собственной branch.

Blender 4.3.2 и 4.1.1 дали одинаковую топологию. Planar T: `18` vertices,
`12` faces, шесть junction half-faces и шесть core edges с двумя linked faces.
Trihedral: `22` vertices, `12` faces и три биссекторных divider-вектора
`(-0.1,-0.1,0)`, `(-0.1,0,-0.1)`, `(0,-0.1,-0.1)`; loose и overfull geometry
отсутствует. Полный Python suite: `56 passed`.
