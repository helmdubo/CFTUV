# CFTUV Regression Snapshot

## PatchGraph Snapshot

Mesh: green_faculty_club_building_a_a_walls.002
P0 WALL.SIDE | faces:29 loops:2 holes:1 chains:13 corners:14 runs:13 | roles:H6 V7 F0
Topology: patches:1 loops:2 chains:13 corners:14 holes:1
Roles: H:6 V:7 Free:0 | Runs: H:6 V:7 Free:0
Neighbors: Patch:0 Self:1 Border:12
Junctions: total:14 interesting:2 open:12 closed:2 valence:[v1:14]
JV20 valence:1 open:N border:N self:Y patches:[P0] signature:[V_FRAME->V_FRAME] corners:[P0L1K0]
JV23 valence:1 open:N border:N self:Y patches:[P0] signature:[V_FRAME->V_FRAME] corners:[P0L1K1]

PatchGraph snapshot | patches:1 loops:2 chains:13 corners:14 runs:13 junctions:14

## Scaffold Snapshot

Mesh: green_faculty_club_building_a_a_walls.002
Anomalies: 2
  - ADDR Q0/P0/L0/C7 frame_scatter scatter:0.901131 target:-0.562718 chains:3
  - ADDR Q0/P0/L0/C1 frame_scatter scatter:0.901131 target:-2.976299 chains:3

Patches: 1
Quilts: 1
Skipped patches: []
Unsupported patches: []
Invalid closure patches: []
Conformal fallback patches: 0
Closure seams: 0 | max_span_mismatch:0.000000 | max_axis_phase:0.000000
Frame groups: 4 | max_row_scatter:0.901131 | max_column_scatter:0.000000

## Quilt 0
root_patch: ADDR Q0/P0
patch_ids: [0]
status: stop:frontier_exhausted patches:1 anomalies:2
build_order: steps:12 first:[ADDR Q0/P0/L0/C6, ADDR Q0/P0/L0/C5, ADDR Q0/P0/L0/C4] last:[ADDR Q0/P0/L0/C2, ADDR Q0/P0/L0/C3]
closure_seams: 0
frame_groups: 4 outliers:2 max_row_scatter:0.901131 max_column_scatter:0.000000
frontier_telemetry:
  placements: 12 main:12 tree_ingress:0 closure_follow:0
  stalls: terminal:1
  scores: min:1.500 p25:2.110 p50:2.123 p75:2.136 max:2.660
  rank_order: viable>role>ingress>patch_fit>anchor>closure>score>length
  rescue_ratio: 0.000
  rescue_gap: measured:0 below_threshold:0 main_viable:0 no_anchor:0 mean:0.000 max:0.000
  rescue_classes: -
  best_rejected_max: -1.000
  duration: 0.002s
patches:
  - ADDR Q0/P0 WALL.SIDE status:COMPLETE transfer:ok loop:0 root:ADDR Q0/P0/L0/C6 chains:12/12 sc:12/12 pts:24/24 uv:24 pin:24/24 suspicious:2 flags:[frame_scatter]
    pin_reasons:connected_hv=12
    Chain ADDR Q0/P0/L0/C1 H_FRAME pts:2 start:(0.9205,-2.0752) end:(0.7103,-2.0752) d:(-0.2101,0.0000) axis:0.000000 inh:1 anchor:SP/- code:frame_scatter st:main
    Chain ADDR Q0/P0/L0/C7 H_FRAME pts:2 start:(0.0000,-1.4638) end:(0.3647,-1.4638) d:(0.3647,0.0000) axis:0.000000 inh:1 anchor:SP/- code:frame_scatter st:main

---
Regression snapshot | quilts:1 | patches:1 | unsupported:0 | invalid_closure:0 | conformal_fallback_patches:0
