# T037 — Radial Decal Seams junction miter

Предыдущий общий surface cap обеспечивал топологическую непрерывность T-стыка,
но не продолжал направления incident seam до центра. Теперь junction остаётся
derived decal-only слоем, а каждая owner surface разбивается на радиальные
сектора между соседними branches.

- у каждого endpoint section хранится явная spine vertex;
- sections сортируются вокруг owner normal;
- сектор строится от общего offset core по одной половине первой branch boundary
  и возвращается по половине следующей;
- каждое ребро branch spine → core разделяется ровно двумя соседними faces;
- незанятый угловой промежуток больше 180 градусов не закрывается лишней face;
- UV каждого сектора ориентирован по своей исходящей branch, а радиальные рёбра
  являются естественными UV seams для смены направления.

Blender 4.3.2 full manual selected-edge planar T дал `17` vertices, `9` faces,
три junction sectors и три core edges, у каждого ровно две linked faces. Три
core faces имеют `4/4/5` сторон; loose и overfull geometry отсутствует.
Trihedral fixture дал три quad sectors и те же три двусторонних core edges.
Blender 4.1.1 повторил planar T topology `17/9` и `2/2/2` linked faces.
