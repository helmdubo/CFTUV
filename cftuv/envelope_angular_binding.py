"""Host-owned ANGULAR binding inputs; native dispatch is not integrated. Host imports belong HERE, never in native wire.
Observed metadata is diagnostic, not approved pins. Future host wrapper injects
its approved reference attribute after rename; current default binds live oracle.
"""
import sys
from cftuv_envelope.reference import planar_types as _metric_types

def binding_profile():
    """Ключ фактического runtime и набора host-классов; wheel не нужен."""
    implementation='CPython' if sys.implementation.name=='cpython' else sys.implementation.name
    return implementation+'-'+str(sys.version_info.major)+'.'+str(sys.version_info.minor)+(':'+('full' if sys.modules.get('cftuv.model') is not None else 'light'))


def build_binding_inputs(kernel):
    from cftuv import envelope_angular_relations as angular, envelope_angular_sites as sites
    from cftuv import envelope_chart_band as chart, envelope_request_export as export
    from cftuv import envelope_export_input as light, envelope_topology_export as topology
    from cftuv import surface_ir, surface_index
    import sys
    model=sys.modules.get("cftuv.model")
    from cftuv_envelope.contracts import metric
    from cftuv_envelope.reference import metric as exact_metric, planar_types
    classes={
      'bundle':(surface_ir.AnalysisBundle,light._LightBundleV1,topology.AnalysisBundleIdView),
      'graph':(light._LightGraphV1,topology._PatchGraphIdView),
      'surface':(surface_ir.PatchSurfaceIR,topology._PatchSurfaceIdView),
      'patch':(light._LightPatchV1,),
      'loop':(light._LightLoopV1,),
      'corner':(light._LightCornerV1,),
      'record':(export._HostChainRecord,), 'chain':(export._HostChainSlice,),
      'face':(surface_ir.SourceFace,), 'frame':(metric.RationalAffinePlanarMetricV2,),
      'certificate':(metric.ExactSourcePlaneCertificateV1,),
      'rational':(metric.ExactRationalV1,), 'point':(metric.ExactPoint2V1,),
      'matrix':(metric.ExactMatrix2V1,), 'coordinate':(metric.ExactSourceVertexCoordinateV2,),
      'source_id':(kernel.SourceVertexId,), 'domain_id':(kernel.PatchDomainId,),
    }
    if model is not None:
        for name,kind in [('graph','PatchGraph'),('patch','PatchNode'),('loop','BoundaryLoop'),('corner','BoundaryCorner'),('chain','BoundaryChain')]:
            classes[name]=(*classes[name],getattr(model,kind))
    callables={'oracle':angular._build_angular_relations_reference,'sites':angular.angular_sites,
      'site_endpoint':sites._is_endpoint,'chart_points':angular.chart_points,
      'chart_edge_ends':angular.chart_edge_ends,'chart_face_points':angular.chart_face_points,
      'cut_path_vertices':angular.cut_path_vertices,'cut_of':chart._cut_of,
      'measure':angular.certified_reflex_measure,'typed_value':export._typed_value,
      'normalize':planar_types.exact_normalize,'canonical':planar_types._canonical_expr,
      'scalar_from':planar_types.ExactScalar.from_value,'scalar_expr':planar_types.ExactScalar.as_expr,
      'vector_scale':planar_types.vector_scale,'exact_sign':planar_types.exact_sign,
      'expr':planar_types._expr,'parse_expr':planar_types._parse_expr_uncached,
      'metric.from_descriptor':exact_metric.ExactPlanarMetric.from_descriptor,
      'kernel.ExactPlanarMetric':kernel.ExactPlanarMetric}
    for name in ('dot_g','length_g','unit_g','oriented_cross','owner_normal_g','angle_g'):
        callables['metric.'+name]=getattr(exact_metric.ExactPlanarMetric,name)
    for kind in classes['surface']:
        callables['surface.patch_faces.'+kind.__name__]=kind.patch_faces
    for name in ('patch_faces_of','surface_index_of','_indexed','_forgetter'):
        callables['surface_index.'+name]=getattr(surface_index,name)
    callables['surface_index.init']=surface_index.SurfaceIndexV1.__init__
    callables['surface_index.matches']=surface_index.SurfaceIndexV1.matches
    for name in ('patch_graph','patch_surface'):
        callables['bundle_view.'+name]=getattr(topology.AnalysisBundleIdView,name).fget
    anchors=((angular,'_build_angular_relations_reference',callables['oracle']),
             (angular,'angular_sites',callables['sites']),
             (sites,'_is_endpoint',callables['site_endpoint']),
             *((angular,name,callables[name]) for name in ('chart_points','chart_edge_ends','chart_face_points','cut_path_vertices')),
             (kernel,'ExactPlanarMetric',kernel.ExactPlanarMetric))
    return dict(profile=binding_profile(),kernel=kernel,classes=classes,callables=callables,anchors=anchors)


def replay_event(event):
    """Воспроизвести только символическую цену пропущенной rational-геометрии."""
    if type(event) is not str or event not in ('canonical', 'normalize'):
        raise ValueError('UNKNOWN_ANGULAR_GEOMETRY_EVENT')
    _metric_types.SYMBOLIC_FALLBACK_COUNTS[event] += 1


def build_dependencies(kernel, sympy):
    """Фактические объекты хоста; утверждение source/runtime pins выполняет shim."""
    from types import SimpleNamespace
    from cftuv_envelope.reference import angle_measure
    from .envelope_angular_relations import certified_reflex_measure
    from .envelope_request_export import EnvelopeHostAdapterError, _typed_value
    from .envelope_host_outcomes import EnvelopeDebugHostOutcome
    from .envelope_debug_profile import ANGULAR_STAGE_COUNTERS

    return SimpleNamespace(kernel=kernel, sympy=sympy, angle_measure=angle_measure,
                           measure=certified_reflex_measure, replay_event=replay_event,
                           host_error=EnvelopeHostAdapterError, outcomes=EnvelopeDebugHostOutcome,
                           typed_value=_typed_value, stage_names=ANGULAR_STAGE_COUNTERS)
