"""Unapplied ANGULAR raw wire draft. No geometry, SymPy, metric, or effects.
Observed source digests are binding metadata, NOT approved native pins.
"""
from dataclasses import dataclass, fields, is_dataclass
from hashlib import sha256
from inspect import getsource
from types import MappingProxyType

MAX_ITEMS=100000
MAX_RATIONAL_BITS=16384
@dataclass(frozen=True)
class WireDecline:
    reason: str
    path: str = ''
@dataclass(frozen=True)
class AngularWire:
    patches: tuple
    site_metadata: tuple
    binding_metadata: object
@dataclass(frozen=True)
class Binding:
    kernel: object
    classes: object
    callables: object
    metadata: object
    anchors: tuple
class _Decline(Exception):
    pass

def _need(ok, reason):
    if not ok: raise _Decline(reason)
def _i(value):
    _need(type(value) is int and -(1<<63)<=value<(1<<63),'INTEGER_I64_REQUIRED')
    return value
def _seq(value, kinds=(tuple,list)):
    _need(type(value) in kinds and len(value)<=MAX_ITEMS,'SEQUENCE_SHAPE_OR_LIMIT')
    return value
def _map(value):
    _need(type(value) in (dict,MappingProxyType) and len(value)<=MAX_ITEMS,'MAPPING_SHAPE_OR_LIMIT')
    return value
def _shape(kind):
    return tuple(field.name for field in fields(kind)) if is_dataclass(kind) else ()
def _source(value):
    return sha256(getsource(value).replace('\r\n','\n').encode()).hexdigest()

def bind(*, kernel, classes, callables, anchors):
    """Receive actual host objects; never discover/import the host package.

    Host-owned builder supplies owner/attribute/captured-value identity anchors.
    Observed source metadata is NOT an approval; binding gate checks static pins.
    """
    if type(classes) is not dict or type(callables) is not dict:
        raise TypeError('EXPLICIT_CLASS_AND_CALLABLE_MAPS_REQUIRED')
    if type(anchors) is not tuple or not anchors:
        raise TypeError('EXPLICIT_HOST_ANCHORS_REQUIRED')
    for key in ('oracle','sites','site_endpoint'):
        if key not in callables or not any(value is callables[key] for _,_,value in anchors):
            raise ValueError('REQUIRED_HOST_ANCHOR:'+key)
    for owner,name,value in anchors:
        if type(name) is not str or getattr(owner,name) is not value:
            raise ValueError('HOST_ANCHOR_MISMATCH')
    metadata={'schema':'cftuv.angular.raw-binding-draft.v1',
      'actual_oracle':'cftuv.envelope_angular_relations.build_angular_relations',
      'future_reference_intent':'same body renamed only AFTER separate source approval',
      'source_sha256':{name:_source(value) for name,value in callables.items()},
      'class_shapes':{kind.__module__+'.'+kind.__qualname__:_shape(kind) for kinds in classes.values() for kind in kinds},
      'class_source_sha256':{kind.__module__+'.'+kind.__qualname__:_source(kind) for kinds in classes.values() for kind in kinds},
      'native_pins_approved':False, 'native_spd_admission_pending':True,
      'mode_proof':'Ordinary rational SymPy-input metric closure identical in SYMPY/NATIVE_EXACT; SHADOW excluded; no general AST acceptance'}
    return Binding(kernel,MappingProxyType(dict(classes)),MappingProxyType(dict(callables)),metadata,anchors)

def encode(binding, *, analysis_bundle, revision, patch_ids, patch_domains, frames,
           use_id_by_ref, sector_id_by_ref, launch_id_by_ref, record_by_ref,
           vertex_ids, triangle_ids, profile=None, symbolic_mode='SYMPY'):
    """All-patch preflight. Any decline discards ALL staging; original oracle owns errors/effects.
    Sidecar preserves original corner int/cut:str, refs and domain identities;
    native (patch,loop,ordinal) is an opaque routing token, never a minted contract ID.
    """
    path='admission'
    try:
        _need(all(getattr(owner,name) is value for owner,name,value in binding.anchors),'CALLABLE_IDENTITY_CHANGED')
        _need(type(symbolic_mode) is str and symbolic_mode in ('SYMPY','NATIVE_EXACT'),'SYMBOLIC_MODE_OR_SHADOW_ORACLE_REQUIRED')
        classes=binding.classes
        def exact(value,name):
            _need(type(value) in classes[name],'EXACT_CLASS_REQUIRED '+name)
            key=type(value).__module__+'.'+type(value).__qualname__
            _need(_shape(type(value))==binding.metadata['class_shapes'][key],'CLASS_SHAPE_CHANGED '+name)
            return value
        def rat(value):
            exact(value,'rational');n,d=value.numerator,value.denominator
            _need(type(n) is int and type(d) is int and d>0,'RATIONAL_NUMDEN_REQUIRED')
            _need(max(n.bit_length(),d.bit_length())<=MAX_RATIONAL_BITS,'RATIONAL_SIZE_LIMIT')
            return n,d
        exact(analysis_bundle,'bundle');graph=exact(analysis_bundle.patch_graph,'graph');surface=exact(analysis_bundle.patch_surface,'surface')
        _need(type(surface).patch_faces is binding.callables['surface.patch_faces.'+type(surface).__name__],'PATCH_FACES_IDENTITY_CHANGED')
        nodes=_map(graph.nodes)
        for mapping in (patch_ids,patch_domains,frames,use_id_by_ref,sector_id_by_ref,launch_id_by_ref,record_by_ref,vertex_ids,triangle_ids):_map(mapping)
        for node_key in nodes:_i(node_key)
        for ref in record_by_ref:
            _need(type(ref) is tuple and len(ref)==3,'CHAIN_REF_SHAPE')
            for part in ref:_i(part)
        reverse={}
        for host,source in vertex_ids.items():
            _i(host);exact(source,'source_id');_need(type(source.value) is str,'SOURCE_ID_VALUE_REQUIRED');_need(source not in reverse,'SOURCE_ID_NOT_INJECTIVE');reverse[source]=host
        for host in triangle_ids:_i(host)
        normalized={}
        records_by_patch={}
        for ref,record in sorted(record_by_ref.items()):
            _need(type(ref) is tuple and len(ref)==3,'CHAIN_REF_SHAPE');ref=tuple(_i(v) for v in ref)
            exact(record,'record');chain=exact(record.chain,'chain')
            vertices=tuple(_i(x) for x in _seq(chain.vert_indices));edges=tuple(_i(x) for x in _seq(chain.edge_indices))
            _need(len(vertices)>=2 and len(edges)+1==len(vertices),'CHAIN_TOPOLOGY_SHAPE')
            _need(type(record.source_is_closed) is bool,'CHAIN_CLOSED_FLAG')
            _need(ref==(_i(record.patch_id),_i(record.loop_index),_i(record.chain_index)),'CHAIN_REF_FIELDS_DIFFER')
            source=(ref[0],ref[1],_i(record.source_chain_index))
            normalized.setdefault(source,[]).append(ref);records_by_patch.setdefault(ref[0],[]).append((ref,vertices,edges))
        normalized={key:tuple(value) for key,value in normalized.items()}
        raw_faces=_seq(surface.faces,(tuple,))
        faces_by_patch={}
        for face in raw_faces:
            exact(face,'face');_i(face.patch_id);faces_by_patch.setdefault(face.patch_id,[]).append(face)
        present_vertices=tuple(sorted(vertex_ids))
        patches=[];metadata=[]
        for patch_id,patch in sorted(nodes.items()):
            path='patch:'+str(patch_id);_i(patch_id);exact(patch,'patch')
            domain=exact(patch_domains[patch_id],'domain_id');frame=exact(frames[patch_id],'frame')
            _need(type(domain.value) is str,'DOMAIN_ID_VALUE_REQUIRED')
            frame_domain=exact(frame.patch_domain_id,'domain_id');_need(type(frame_domain.value) is str,'FRAME_DOMAIN_ID_VALUE_REQUIRED')
            exact(frame.planarity_certificate,'certificate')
            # ExactSourcePlane has no cut, band, or chart remapping. Near-planar/legacy decline V1.
            _need(frame.patch_domain_id==domain,'FRAME_DOMAIN_MISMATCH')
            orientation=frame.chart_orientation
            enum=binding.kernel.AffineChartOrientationV1
            _need(type(orientation) is enum,'ORIENTATION_CLASS')
            if orientation is enum.COORDINATE_CCW_MATCHES_OWNER_PATCH:owner=1
            elif orientation is enum.COORDINATE_CW_MATCHES_OWNER_PATCH:owner=-1
            else:raise _Decline('ORIENTATION_VALUE')
            matrix=exact(frame.exact_gram_matrix,'matrix')
            gram=((rat(matrix.m00),rat(matrix.m01)),(rat(matrix.m10),rat(matrix.m11)))
            coordinates=[];seen=set()
            for item in _seq(frame.exact_source_vertex_coordinates,(frozenset,)):
                exact(item,'coordinate');source=exact(item.source_vertex_id,'source_id');xy=exact(item.domain_coordinate,'point')
                _need(type(source.value) is str,'COORDINATE_ID_VALUE_REQUIRED')
                host=reverse[source];_need(host not in seen,'DUPLICATE_COORDINATE');seen.add(host)
                coordinates.append((host,(rat(xy.x),rat(xy.y))))
            coordinates.sort(key=lambda item:item[0])
            faces=[]
            for face in faces_by_patch.get(patch_id,()):
                vertices=tuple(_i(x) for x in _seq(face.vertex_cycle,(tuple,)))
                edges=tuple(_i(x) for x in _seq(face.edge_cycle,(tuple,)))
                triangles=tuple(_i(x) for x in _seq(face.triangle_ids,(tuple,)))
                _need(len(vertices)>=3 and len(vertices)==len(edges),'FACE_TOPOLOGY_SHAPE')
                faces.append((_i(face.face_id),vertices,edges,triangles))
            for loop in _seq(patch.boundary_loops):
                exact(loop,'loop');_seq(loop.chains);_seq(loop.corners)
                for corner in loop.corners:
                    exact(corner,'corner');_i(corner.vert_index);_i(corner.prev_chain_index);_i(corner.next_chain_index)
            # This pinned enumeration is permitted; it owns no geometric/counter effects.
            found=binding.callables['sites'](patch,patch_id,normalized,record_by_ref,domain)
            wire_sites=[];sidecar=[]
            for ordinal,(loop,corner,cut,anchor,previous,following) in enumerate(_seq(found,(tuple,))):
                _i(loop);_i(anchor);_need(type(cut) is bool,'SITE_CUT_FLAG')
                _need(type(corner) is int or (type(corner) is str and corner.startswith('cut:')),'SITE_ID_SHAPE')
                wire_sites.append(((patch_id,loop,ordinal),cut,anchor,previous,following))
                sidecar.append((ordinal,loop,corner,cut,anchor,previous,following))
            patches.append({'version':1,'patch_id':patch_id,'owner_sign':owner,'gram':gram,'chart':'RationalAffine',
              'band_or_ring_cut':False,'points':tuple(coordinates),'chains':tuple(records_by_patch.get(patch_id,())),
              'faces':tuple(faces),'sites':tuple(wire_sites),'present_vertices':present_vertices, 'support_triangles':None})
            metadata.append({'patch_id':patch_id,'domain_id':domain,'sites':tuple(sidecar),'frame':frame})
        return AngularWire(tuple(patches),tuple(metadata),binding.metadata)
    except _Decline as error:return WireDecline(str(error),path)
    except Exception as error:return WireDecline('PREFLIGHT_CALLBACK_OR_INPUT_ERROR:'+type(error).__name__,path)
