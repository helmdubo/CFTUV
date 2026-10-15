"""External unintegrated draft: accepted ordinary-rational plans -> host records.
No native parity claim. SKIP geometry has no symbolic events only on admitted
ordinary rational input. Dependencies are actual pinned callables supplied by shim.
Two-root/two-factor bridge is a candidate; it returns sp.Expr, like angle_g.
All bridge expressions are staged before ANY replay/measure/profile effect.
Ordinary staging failure declines to the independent whole oracle, which owns its
error prefix. BaseException is never caught. No equivalence claim for resource/
interrupt timing or arbitrary callbacks/monkeypatches; actual SymPy Rationals,
sqrt/factor and host dependencies must be pinned. Staging does not isolate or
rewrite Decimal/iv state; ordinary rational SymPy operations do not use them.
"""
from dataclasses import dataclass
from math import gcd

@dataclass(frozen=True)
class AssemblyDecline:
    reason: str

@dataclass(frozen=True)
class PreparedAssembly:
    patches: tuple
    dependencies: object
    revision: str

class _Invalid(ValueError):
    pass

def _need(condition, reason):
    if not condition:
        raise _Invalid(reason)

def _tuple(value, size=None):
    _need(type(value) is tuple and (size is None or len(value) == size), 'TUPLE_SHAPE')
    return value

def _int(value):
    _need(type(value) is int, 'EXACT_INT')
    return value

def _rat(value):
    a, b = _tuple(value, 2)
    _int(a); _int(b)
    _need(b > 0, 'POSITIVE_DENOMINATOR')
    _need(gcd(a, b) == 1, 'CANONICAL_RATIONAL')
    return value

def _vec(value):
    for q in _tuple(value, 2):
        _rat(q)
    return value

def prepare(wire, native_plans, *, maps, revision, dependencies):
    """Whole-op preflight only. Decline is possible ONLY before assembly effects.
    maps: patch_ids/use_id_by_ref/sector_id_by_ref/launch_id_by_ref/vertex_ids.
    dependencies: kernel, sympy, angle_measure, measure, replay_event, stage_names,
    host_error, outcomes, typed_value. Pinning remains the future shim's duty.
    """
    try:
        _need(dependencies.sympy.core.parameters.global_parameters.evaluate is True, 'SYMPY_EVALUATE_DISABLED')
        _need(type(revision) is str and type(maps) is dict, 'ASSEMBLY_INPUT')
        _need(len(wire.patches) == len(wire.site_metadata), 'SIDECAR_COUNT')
        plans = _tuple(native_plans)
        _need(len(plans) == len(wire.patches), 'PATCH_COUNT')
        names = _tuple(dependencies.stage_names)
        _need(len(set(names)) == len(names) and all(type(n) is str for n in names), 'STAGE_NAMES')
        for name in ('patch_ids', 'use_id_by_ref', 'sector_id_by_ref', 'launch_id_by_ref', 'vertex_ids'):
            _need(type(maps[name]) is dict, 'EXACT_MAP')
        prepared = []
        for raw, side, plan in zip(wire.patches, wire.site_metadata, plans):
            status, rows = _tuple(plan, 2)
            _need(type(status) is int and status == 0, 'NATIVE_DECLINE')
            _need(raw['patch_id'] == side['patch_id'], 'PATCH_BINDING')
            sites = _tuple(side['sites']); raw_sites = _tuple(raw['sites'])
            _need(len(_tuple(rows)) == len(sites) == len(raw_sites), 'SITE_COUNT')
            patch_id = _int(side['patch_id']); domain = side['domain_id']
            _need(type(domain.value) is str, 'DOMAIN_VALUE')
            patch_record = maps['patch_ids'][patch_id]
            staged = []
            for ordinal, (row, site, raw_site) in enumerate(zip(rows, sites, raw_sites)):
                n, key, decision, before, after = _tuple(row, 5)
                sn, loop, corner, cut, anchor, previous, following = _tuple(site, 7)
                _need(type(n) is int and n == sn == ordinal, 'SITE_ORDINAL')
                _need(_tuple(key, 3) == raw_site[0] == (patch_id, loop, ordinal), 'OPAQUE_SITE_KEY')
                _int(loop); _int(anchor)
                _need(type(corner) is int or (type(corner) is str and corner.startswith('cut:')), 'CORNER_ID')
                _need(type(cut) is bool, 'CUT_FLAG')
                _need(_tuple(raw_site, 5)[1:] == (cut, anchor, previous, following), 'SITE_SIDECAR_BINDING')
                for events in (before, after):
                    _need(all(type(x) is str and x in names for x in _tuple(events)), 'STAGE_EVENT')
                decision = _tuple(decision)
                if decision and decision[0] == 'SKIP':
                    _need(len(decision) == 2 and decision[1] in ('OffJunction', 'BeyondChartReach', 'ForwardCollinear', 'NonReflex') and not after, 'SKIP_SHAPE')
                    payload = None
                else:
                    _need(len(decision) == 10 and decision[0] == 'REFLEX', 'REFLEX_SHAPE')
                    _, incoming, outgoing, turn, a, b, dot, cross, det, events = decision
                    _vec(incoming); _vec(outgoing)
                    _need(type(turn) is int and turn in (-1, 1), 'TURN')
                    for q in (a, b, dot, cross, det): _rat(q)
                    _need(a[0] > 0 and b[0] > 0 and det[0] > 0, 'POSITIVE_SCALARS')
                    _need(all(type(x) is str and x in ('canonical', 'normalize') for x in _tuple(events)), 'GEOMETRY_EVENT')
                    _need(after == ('ANGULAR_RELATIONS_BUILT',), 'AFTER_SUCCESS')
                    # Preserve original contract objects; native ordinal is not an ID.
                    ids = (maps['sector_id_by_ref'][previous], maps['use_id_by_ref'][previous],
                           maps['use_id_by_ref'][following], maps['launch_id_by_ref'][previous],
                           maps['launch_id_by_ref'][following], maps['vertex_ids'][anchor])
                    try:
                        scalars = _bridge(dependencies.sympy, decision)
                    except Exception as error:
                        return AssemblyDecline(
                            'SCALAR_STAGING_FAILED:' + type(error).__name__)
                    payload = (decision, ids, loop, corner, scalars)
                staged.append((before, after, payload))
            prepared.append((patch_id, patch_record, domain, side['frame'].reference_metric_id, tuple(staged)))
        return PreparedAssembly(tuple(prepared), dependencies, revision)
    except _Invalid as error:
        return AssemblyDecline(str(error))
    except Exception as error:
        return AssemblyDecline('ASSEMBLY_PREFLIGHT_INPUT_OR_CALLBACK:' + type(error).__name__)

def _bridge(sp, decision):
    _, _, _, _, a, b, dot, cross, det, _ = decision
    q = lambda pair: sp.Rational(*pair)
    denominator = sp.sqrt(q(a)) * sp.sqrt(q(b))
    # No ExactScalar.from_value: actual angle_g returns SymPy expressions.
    return sp.factor(q(dot) / denominator), sp.factor(q(cross) * sp.sqrt(q(det)) / denominator)

def assemble(prepared, *, profile=None):
    """No decline/fallback once effects begin; preserve oracle exception prefixes."""
    if type(prepared) is not PreparedAssembly:
        raise TypeError('assemble requires PreparedAssembly')
    d = prepared.dependencies; k = d.kernel
    sectors, certificates, relations = [], [], []
    used = set()
    for patch_id, patch_record, domain, metric_id, sites in prepared.patches:
        counters = dict.fromkeys(d.stage_names, 0)
        for before, after, payload in sites:
            for event in before: counters[event] += 1
            if payload is None: continue
            decision, ids, loop, corner, scalars = payload
            sector, incoming_use, outgoing_use, incoming_launch, outgoing_launch, anchor = ids
            cosine, sine = scalars
            for event in decision[9]: d.replay_event(event)
            orientation = (k.TurnOrientation.CCW_IN_OWNER_PATCH_ORIENTATION if decision[3] > 0
                           else k.TurnOrientation.CW_IN_OWNER_PATCH_ORIENTATION)
            try:
                measure = d.measure(k, d.angle_measure, sine, cosine, orientation)
            except d.angle_measure.CertifiedAngleUnavailable as error:
                raise d.host_error(d.outcomes.ENVELOPE_DEBUG_EXACT_ANGULAR_CERTIFICATE_UNAVAILABLE,
                                   str(error), patch_domain_id=domain.value) from error
            if sector in used:
                raise d.host_error(d.outcomes.ENVELOPE_DEBUG_MULTIPLE_ANGULAR_RELATIONS_PER_CHAIN_UNSUPPORTED,
                                  'V0 cannot encode two Angular relations through one owner-sector record',
                                  patch_domain_id=domain.value)
            used.add(sector)
            def direction(raw):
                return k.CertifiedAffineSupportDirectionV2(
                    k.ExactVector2V1(*(k.ExactRationalV1(*q) for q in raw)), metric_id)
            sectors.append(k.OrientedOwnerSectorV1(sector, patch_record, domain, True,
                (incoming_use, outgoing_use),
                k.SourceSupportRefV1(incoming_use, incoming_launch, direction(decision[1])),
                k.SourceSupportRefV1(outgoing_use, outgoing_launch, direction(decision[2])), orientation,
                k.InteriorSelectionLaw.OWNER_PATCH_INTERIOR_BETWEEN_ORDERED_SUPPORTS))
            angle_id = k.AngleCertificateId(d.typed_value('angle-certificate', prepared.revision, patch_id, loop, corner))
            certificates.append(k.ReflexAngleCertificateV1(angle_id, sector,
                k.AngleMeasureLaw.ORIENTED_OWNER_SECTOR_ANGLE,
                k.AngleMeasureSource.HOST_ANALYSIS_EXACT_OR_CERTIFIED,
                k.StrictAngleRangeCertificate.STRICT_PI_LT_PHI_LT_2PI,
                k.ReflexExcessLaw.DELTA_EQUALS_PHI_MINUS_PI, measure, False))
            relations.append(k.CornerRelationV1(
                k.CornerRelationId(d.typed_value('corner-relation', prepared.revision, patch_id, loop, corner)),
                anchor, sector, angle_id, False))
            for event in after: counters[event] += 1
        if profile is not None:
            for name, value in sorted(counters.items()):
                profile.set_counter(name, value, domain.value)
    return frozenset(sectors), frozenset(certificates), frozenset(relations)