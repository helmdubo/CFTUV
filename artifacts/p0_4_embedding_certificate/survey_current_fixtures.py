"""Run current metric fixtures and audit them with the P0-4 exact predicates."""

from __future__ import annotations

from collections import Counter
from fractions import Fraction
import functools
from hashlib import sha256
import inspect
import json
import os
from pathlib import Path
import sys

import pytest

from cftuv_envelope._embedding import (
    build_projection_embedding_certificate,
    build_source_snap_embedding_certificate,
    projection_violations,
    source_snap_violations,
)
from cftuv_envelope.contracts.metric import (
    AffineChartOrientationV1,
    NearPlanarProjectionCertificateV1,
)
from cftuv_envelope.planar_metric import build_rational_affine_planar_metric
from cftuv_envelope.source_grid import (
    intended_right_corner_facts,
    resolve_source_grid,
)


class _CurrentItem:
    nodeid = "outside-pytest-item"
    collected = 0

    def pytest_collection_finish(self, session):
        self.collected = len(session.items)

    def pytest_runtest_setup(self, item):
        self.nodeid = item.nodeid


def _raw_position(vertex):
    return tuple(
        Fraction(*float(value).as_integer_ratio())
        for value in (vertex.position.x, vertex.position.y, vertex.position.z)
    )


def _fraction(value):
    return Fraction(value.numerator, value.denominator)


def _sub(left, right):
    return tuple(a - b for a, b in zip(left, right, strict=True))


def _cross3(left, right):
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _orient2(left, vertex, right):
    a, b = _sub(left, vertex), _sub(right, vertex)
    return (a[0] * b[1] - a[1] * b[0] > 0) - (
        a[0] * b[1] - a[1] * b[0] < 0
    )


def _corner_details(corners, faces, before, after):
    result = []
    for previous, vertex, following in corners:
        incident_edges = []
        for face in faces:
            cycle = face.vertex_cycle
            for index, item in enumerate(cycle):
                if item != vertex:
                    continue
                if {cycle[index - 1], cycle[(index + 1) % len(cycle)]} == {
                    previous,
                    following,
                }:
                    incident_edges.append(
                        (face.edge_cycle[index - 1].value, face.edge_cycle[index].value)
                    )
        before_left = _sub(before[previous], before[vertex])
        before_right = _sub(before[following], before[vertex])
        after_left = _sub(after[previous], after[vertex])
        after_right = _sub(after[following], after[vertex])
        result.append(
            {
                "vertex_ids": [item.value for item in (previous, vertex, following)],
                "incident_physical_edge_ids": incident_edges,
                "before_left": [str(item) for item in before_left],
                "before_right": [str(item) for item in before_right],
                "after_left": [str(item) for item in after_left],
                "after_right": [str(item) for item in after_right],
                "before_after_positions_equal": all(
                    before[item] == after[item]
                    for item in (previous, vertex, following)
                ),
                "before_cross_zero": not any(_cross3(before_left, before_right)),
                "before_dot_sign": (
                    sum(a * b for a, b in zip(before_left, before_right, strict=True))
                    > 0
                ) - (
                    sum(a * b for a, b in zip(before_left, before_right, strict=True))
                    < 0
                ),
            }
        )
    return result


def _audit_call(arguments, metric, nodeid, ordinal):
    owner_patch_id = arguments["owner_patch_id"]
    faces = tuple(
        sorted(
            (
                face
                for face in arguments["source_faces"]
                if face.patch_id == owner_patch_id
            ),
            key=lambda item: item.face_id.value,
        )
    )
    required_ids = tuple(
        sorted(
            {vertex_id for face in faces for vertex_id in face.vertex_cycle},
            key=lambda item: item.value,
        )
    )
    vertex_by_id = {item.vertex_id: item for item in arguments["source_vertices"]}
    raw = {
        vertex_id: _raw_position(vertex)
        for vertex_id, vertex in vertex_by_id.items()
        if vertex_id in required_ids
    }
    grid = resolve_source_grid(
        positions=dict(raw),
        faces=faces,
        snapping_law=arguments["grid_policy"],
        enforce_embedding=False,
    )
    intended = intended_right_corner_facts(raw, faces)
    snap = build_source_snap_embedding_certificate(
        before=raw,
        after=grid.positions,
        faces=faces,
        intended_corners=intended.intended,
        unclassifiable_corners=intended.unclassifiable,
        snapping_law=arguments["grid_policy"],
    )
    projection = None
    projection_failures = ()
    if type(metric.planarity_certificate) is NearPlanarProjectionCertificateV1:
        projected = {
            record.source_vertex_id: (
                _fraction(record.domain_coordinate.x),
                _fraction(record.domain_coordinate.y),
            )
            for record in metric.exact_source_vertex_coordinates
        }
        sign = (
            1
            if metric.chart_orientation
            is AffineChartOrientationV1.COORDINATE_CCW_MATCHES_OWNER_PATCH
            else -1
        )
        projection = build_projection_embedding_certificate(
            before=grid.positions,
            projected=projected,
            faces=faces,
            normal=tuple(
                _fraction(item)
                for item in (
                    metric.planarity_certificate.exact_plane_normal.x,
                    metric.planarity_certificate.exact_plane_normal.y,
                    metric.planarity_certificate.exact_plane_normal.z,
                )
            ),
            expected_orientation_sign=sign,
        )
        projection_failures = projection_violations(projection)
    anchor_details = None
    if projection is not None and len(projection.source_anchor_vertex_ids) == 3:
        source_anchor = projection.source_anchor_vertex_ids
        target_anchor = projection.projected_anchor_vertex_ids
        source_left = _sub(grid.positions[source_anchor[1]], grid.positions[source_anchor[0]])
        source_right = _sub(grid.positions[source_anchor[2]], grid.positions[source_anchor[0]])
        target_left = _sub(projected[target_anchor[1]], projected[target_anchor[0]])
        target_right = _sub(projected[target_anchor[2]], projected[target_anchor[0]])
        anchor_details = {
            "source_anchor_source_cross_nonzero": any(
                _cross3(source_left, source_right)
            ),
            "source_anchor_projected_orientation": _orient2(
                projected[source_anchor[1]],
                projected[source_anchor[0]],
                projected[source_anchor[2]],
            ),
            "projected_anchor_projected_orientation": _orient2(
                projected[target_anchor[1]],
                projected[target_anchor[0]],
                projected[target_anchor[2]],
            ),
        }
    planarity = metric.planarity_certificate
    return {
        "case": f"{nodeid}::metric-call-{ordinal}",
        "reference_metric_id": metric.reference_metric_id.value,
        "planarity": type(metric.planarity_certificate).__name__,
        "snapping_law": arguments["grid_policy"].value,
        "source_snap_outcomes": [item.value for item in source_snap_violations(snap)],
        "unclassifiable_source_corners": [
            [vertex.value for vertex in corner]
            for corner in intended.unclassifiable
        ],
        "unclassifiable_source_corner_positions": [
            [
                [str(coordinate) for coordinate in raw[vertex]]
                for vertex in corner
            ]
            for corner in intended.unclassifiable
        ],
        "unclassifiable_source_corner_details": _corner_details(
            intended.unclassifiable, faces, raw, grid.positions
        ),
        "degenerated_intended_right_corner_count": (
            snap.degenerated_intended_right_corner_count
        ),
        "unchanged_unclassifiable_source_corner_count": (
            snap.unchanged_unclassifiable_source_corner_count
        ),
        "projection_outcomes": [item.value for item in projection_failures],
        "source_snap_exact_pair_tests": snap.exact_pair_test_count,
        "source_vertex_count": snap.source_vertex_count,
        "source_edge_count": snap.source_edge_count,
        "projection_exact_pair_tests": (
            0 if projection is None else projection.exact_pair_test_count
        ),
        "source_anchor_vertex_ids": (
            []
            if projection is None
            else [item.value for item in projection.source_anchor_vertex_ids]
        ),
        "projected_anchor_vertex_ids": (
            []
            if projection is None
            else [item.value for item in projection.projected_anchor_vertex_ids]
        ),
        "anchor_details": anchor_details,
        "max_residual_squared": (
            None
            if not hasattr(planarity, "max_residual_squared")
            else str(_fraction(planarity.max_residual_squared))
        ),
        "residual_budget": (
            None
            if not hasattr(planarity, "residual_budget")
            else str(_fraction(planarity.residual_budget))
        ),
        "boundary_occurrence_count": (
            0 if projection is None else projection.source_boundary_occurrence_count
        ),
    }


def main() -> int:
    current = _CurrentItem()
    records = []
    ordinal_by_node = Counter()
    original_builder = build_rational_affine_planar_metric
    signature = inspect.signature(original_builder)

    @functools.wraps(original_builder)
    def audited_builder(*args, **kwargs):
        bound = signature.bind(*args, **kwargs)
        bound.apply_defaults()
        bound.arguments["source_vertices"] = tuple(
            bound.arguments["source_vertices"]
        )
        bound.arguments["source_faces"] = tuple(bound.arguments["source_faces"])
        metric = original_builder(**bound.arguments)
        ordinal_by_node[current.nodeid] += 1
        records.append(
            _audit_call(
                bound.arguments,
                metric,
                current.nodeid,
                ordinal_by_node[current.nodeid],
            )
        )
        return metric

    class _ReplaceImportedBuilders:
        def pytest_collection_modifyitems(self):
            for module in tuple(sys.modules.values()):
                namespace = getattr(module, "__dict__", None)
                if namespace is None:
                    continue
                for name, value in tuple(namespace.items()):
                    if value is original_builder:
                        namespace[name] = audited_builder

    repository = Path(__file__).resolve().parents[2]
    root = repository / "kernel"
    tests = (
        root / "tests/test_building_002_point_contact_fixture.py",
        root / "tests/test_grid_wiring.py",
        root / "tests/test_near_planar_policy.py",
        root / "tests/test_near_planar_snapshot_validation.py",
        root / "tests/test_planar_metric_v2.py",
        repository / "tests/test_envelope_host_adapter.py",
    )
    pytest_arguments = [*(str(item) for item in tests), "-q"]
    if os.environ.get("P0_4_SURVEY_K"):
        pytest_arguments.extend(("-k", os.environ["P0_4_SURVEY_K"]))
    status = pytest.main(
        pytest_arguments,
        plugins=[current, _ReplaceImportedBuilders()],
    )
    findings = [
        item
        for item in records
        if item["source_snap_outcomes"] or item["projection_outcomes"]
    ]
    if os.environ.get("P0_4_SURVEY_COMPACT_FINDINGS"):
        detail_keys = (
            "case",
            "snapping_law",
            "source_snap_outcomes",
            "projection_outcomes",
            "source_anchor_vertex_ids",
            "projected_anchor_vertex_ids",
            "anchor_details",
            "max_residual_squared",
            "residual_budget",
            "unclassifiable_source_corner_details",
        )
        findings = [
            {key: item[key] for key in detail_keys if item.get(key) not in (None, [], ())}
            for item in findings
        ]
    encoded_records = json.dumps(
        records, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    report = {
        "schema": "P0_4_CURRENT_METRIC_FIXTURE_SURVEY_V1",
        "test_paths": [str(item.relative_to(repository)) for item in tests],
        "pytest_cases_collected": current.collected,
        "pytest_exit_code": int(status),
        "successful_metric_builds": len(records),
        "admitted": len(records) - len(findings),
        "named_findings": findings,
        "case_receipt_sha256": sha256(encoded_records).hexdigest(),
        "planarity_counts": dict(
            sorted(Counter(item["planarity"] for item in records).items())
        ),
        "snapping_law_counts": dict(
            sorted(Counter(item["snapping_law"] for item in records).items())
        ),
        "max_source_vertex_count": max(
            (item["source_vertex_count"] for item in records), default=0
        ),
        "max_source_edge_count": max(
            (item["source_edge_count"] for item in records), default=0
        ),
        "max_boundary_occurrence_count": max(
            (item["boundary_occurrence_count"] for item in records), default=0
        ),
        "max_source_snap_exact_pair_tests": max(
            (item["source_snap_exact_pair_tests"] for item in records), default=0
        ),
        "max_projection_exact_pair_tests": max(
            (item["projection_exact_pair_tests"] for item in records), default=0
        ),
        "unchanged_unclassifiable_source_corner_count": sum(
            item["unchanged_unclassifiable_source_corner_count"]
            for item in records
        ),
        "unchanged_unclassifiable_source_corner_cases": [
            item["case"]
            for item in records
            if item["unchanged_unclassifiable_source_corner_count"]
        ],
    }
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return int(bool(status))


if __name__ == "__main__":
    raise SystemExit(main())
