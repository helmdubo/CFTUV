"""Separated geometry-generator and coverage-contributor provenance."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class BoundaryGeneratorProvenanceV1:
    support_ids: frozenset[str] = frozenset()
    physical_edge_ids: frozenset[str] = frozenset()
    physical_chain_ids: frozenset[str] = frozenset()
    boundary_loop_ids: frozenset[str] = frozenset()
    source_face_ids: frozenset[str] = frozenset()
    boundary_constraint_ids: frozenset[str] = frozenset()


@dataclass(frozen=True, slots=True)
class CoverageContributorProvenanceV1:
    envelope_spec_ids: frozenset[str] = frozenset()
    envelope_instance_ids: frozenset[str] = frozenset()
    chain_use_ids: frozenset[str] = frozenset()
    patch_domain_ids: frozenset[str] = frozenset()
    lineage_ids: frozenset[str] = frozenset()


@dataclass(frozen=True, slots=True)
class ReferenceProvenanceV1:
    boundary_generator: BoundaryGeneratorProvenanceV1 = (
        BoundaryGeneratorProvenanceV1()
    )
    coverage_contributors: CoverageContributorProvenanceV1 = (
        CoverageContributorProvenanceV1()
    )

    @property
    def support_ids(self) -> frozenset[str]:
        return self.boundary_generator.support_ids

    @property
    def physical_edge_ids(self) -> frozenset[str]:
        return self.boundary_generator.physical_edge_ids

    @property
    def source_face_ids(self) -> frozenset[str]:
        return self.boundary_generator.source_face_ids

    @property
    def physical_chain_ids(self) -> frozenset[str]:
        return self.boundary_generator.physical_chain_ids

    @property
    def boundary_loop_ids(self) -> frozenset[str]:
        return self.boundary_generator.boundary_loop_ids

    @property
    def boundary_constraint_ids(self) -> frozenset[str]:
        return self.boundary_generator.boundary_constraint_ids

    @property
    def envelope_spec_ids(self) -> frozenset[str]:
        return self.coverage_contributors.envelope_spec_ids

    @property
    def envelope_instance_ids(self) -> frozenset[str]:
        return self.coverage_contributors.envelope_instance_ids

    @property
    def chain_use_ids(self) -> frozenset[str]:
        return self.coverage_contributors.chain_use_ids

    @property
    def patch_domain_ids(self) -> frozenset[str]:
        return self.coverage_contributors.patch_domain_ids

    @property
    def lineage_ids(self) -> frozenset[str]:
        return self.coverage_contributors.lineage_ids


def make_reference_provenance(
    *,
    envelope_spec_ids: frozenset[str] = frozenset(),
    envelope_instance_ids: frozenset[str] = frozenset(),
    support_ids: frozenset[str] = frozenset(),
    physical_edge_ids: frozenset[str] = frozenset(),
    physical_chain_ids: frozenset[str] = frozenset(),
    boundary_loop_ids: frozenset[str] = frozenset(),
    source_face_ids: frozenset[str] = frozenset(),
    chain_use_ids: frozenset[str] = frozenset(),
    patch_domain_ids: frozenset[str] = frozenset(),
    boundary_constraint_ids: frozenset[str] = frozenset(),
    lineage_ids: frozenset[str] = frozenset(),
) -> ReferenceProvenanceV1:
    return ReferenceProvenanceV1(
        boundary_generator=BoundaryGeneratorProvenanceV1(
            support_ids=support_ids,
            physical_edge_ids=physical_edge_ids,
            physical_chain_ids=physical_chain_ids,
            boundary_loop_ids=boundary_loop_ids,
            source_face_ids=source_face_ids,
            boundary_constraint_ids=boundary_constraint_ids,
        ),
        coverage_contributors=CoverageContributorProvenanceV1(
            envelope_spec_ids=envelope_spec_ids,
            envelope_instance_ids=envelope_instance_ids,
            chain_use_ids=chain_use_ids,
            patch_domain_ids=patch_domain_ids,
            lineage_ids=lineage_ids,
        ),
    )


def merge_boundary_generator_provenance(
    *items: BoundaryGeneratorProvenanceV1,
) -> BoundaryGeneratorProvenanceV1:
    return BoundaryGeneratorProvenanceV1(
        support_ids=frozenset().union(*(item.support_ids for item in items)),
        physical_edge_ids=frozenset().union(
            *(item.physical_edge_ids for item in items)
        ),
        physical_chain_ids=frozenset().union(
            *(item.physical_chain_ids for item in items)
        ),
        boundary_loop_ids=frozenset().union(
            *(item.boundary_loop_ids for item in items)
        ),
        source_face_ids=frozenset().union(
            *(item.source_face_ids for item in items)
        ),
        boundary_constraint_ids=frozenset().union(
            *(item.boundary_constraint_ids for item in items)
        ),
    )


def merge_coverage_contributor_provenance(
    *items: CoverageContributorProvenanceV1,
) -> CoverageContributorProvenanceV1:
    return CoverageContributorProvenanceV1(
        envelope_spec_ids=frozenset().union(
            *(item.envelope_spec_ids for item in items)
        ),
        envelope_instance_ids=frozenset().union(
            *(item.envelope_instance_ids for item in items)
        ),
        chain_use_ids=frozenset().union(
            *(item.chain_use_ids for item in items)
        ),
        patch_domain_ids=frozenset().union(
            *(item.patch_domain_ids for item in items)
        ),
        lineage_ids=frozenset().union(*(item.lineage_ids for item in items)),
    )


def merge_provenance(*items: ReferenceProvenanceV1) -> ReferenceProvenanceV1:
    return ReferenceProvenanceV1(
        boundary_generator=merge_boundary_generator_provenance(
            *(item.boundary_generator for item in items)
        ),
        coverage_contributors=merge_coverage_contributor_provenance(
            *(item.coverage_contributors for item in items)
        ),
    )
