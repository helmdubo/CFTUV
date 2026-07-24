# Research references mapped to implementation responsibilities

## A Straight Skeleton Approximating the Medial Axis

Use for the local reflex profile law:

- zero-length hidden supports at alpha zero;
- equal subdivision of reflex excess;
- speed-bound interpretation;
- quality controlled by maximum subturn or an equivalent certified bound.

Do not interpret this as a replacement for later kinetic split events.

## Skeletons and offsetting — Stefan Huber

Use for:

- topological distinction between coincident geometry and distinct boundary occurrences;
- owner/generator cells;
- precomputed evolution structure for repeated offset queries;
- point-contact interpretation.

## CGAL Straight_skeleton_2

Use as a design/oracle reference for:

- halfedge topology;
- edge, split and vertex events;
- event queue and invalidation;
- construction/query separation.

Do not directly substitute a Patch-boundary straight skeleton for selected-source decal growth, and do not allow representation-only bisectors to become ownership or UV semantics.

## Medial Skeletal Diagram, arXiv:2310.09395v3

Use for the architectural principle:

```text
sparse skeleton + expressive primitives
```

It is not the direct runtime algorithm for a decal on a 2D manifold.

## Curved-surface research direction

Evaluate, under a separate research gate:

- developable unfolding;
- multi-source Fast Marching;
- Heat Method as a reusable approximate candidate field;
- Vector Heat for tangent/co-normal transport;
- Surface Voronoi with curve generators and multiple arrival candidates.

No curved implementation starts until the planar cell-complex contracts are stable.
