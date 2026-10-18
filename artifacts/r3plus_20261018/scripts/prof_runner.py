"""R3+ Phase A: one cold press of the REAL operator, headless, with stage laps and a cProfile restricted to PLAN_COMPILE.

blender -b <scene> --python-exit-code 1 --python prof_runner.py -- --tree T --native-site S --build-id B --case building|cover008
        --workers 0|12 --output out.json [--pstats out.pstats] [--laps 1]

workers 0: the product's serial path (the parent serves every domain itself), laps + cProfile of compile_reference_envelopes (outermost calls only).
workers 12: the product pool; laps only in the parent (worker CPU is not visible here) - used for the button wall.
Nothing is saved, no .blend is written.
"""
import argparse
import cProfile
import hashlib
import importlib.util
import json
import os
import pstats
import sys
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def digest(value):
    return hashlib.sha256(json.dumps(value, separators=(',', ':'), sort_keys=True).encode()).hexdigest()


PC_EXTRA = (
    ("kernel:reference.compile", "validate_snapshot_request_references", "PC.validate_snapshot_request_references"),
    ("kernel:reference.compile", "validate_reference_geometry_certificates", "PC.validate_reference_geometry_certificates"),
    ("kernel:reference.compile", "resolve_corner_selection", "PC.resolve_corner_selection"),
    ("kernel:reference.common", "station_plans_of", "PC.station_plans_of"),
    ("kernel:reference.compile", "initial_front_support_features", "PC.initial_front_support_features"),
    ("kernel:reference.compile", "_canonical_fan_authorities", "PC.canonical_fan_authorities"),
    ("kernel:reference.compile", "_attach_direction_bindings_legacy_v1", "PC.attach_direction_bindings_legacy_v1"),
    ("kernel:reference.compile", "_adaptive_density_spec", "PC.adaptive_density_spec"),
    ("kernel:reference.compile", "_density_spec_with_hidden_count", "PC.density_spec_with_hidden_count"),
    ("kernel:backend", "density_compute", "PCD.backend_density_compute"),
    ("native:density_op", "density_compute_with_memo", "PCD.native_op_density_compute_with_memo"),
    ("native:pin", "verdict", "PCD.pin_verdict"),
    ("native:density_pin", "runtime_stale", "PCD.density_pin_runtime_stale"),
    ("native:density_op", "_validate_memo_snapshot", "PCD.op_validate_memo_snapshot"),
    ("native:density_op", "_validate_report", "PCD.op_validate_report"),
    ("native:density_op", "_replay_density_work", "PCD.op_replay_density_work"),
    ("kernel:reference.density_attach_v1", "_validate_success", "PCD.validate_success"),
    ("kernel:reference.density_compile_v1", "_prepared_attachment", "PCD.prepared_attachment"),
    ("kernel:reference.density_compile_v1", "_raw_prefix", "PCD.raw_prefix"),
    ("kernel:reference.density_attach_v1", "attach_density_results_v1", "PCD.attach_density_results_v1"),
    ("kernel:reference.common", "GeometryContext._face_side_normal", "PCS.face_side_normal"),
    ("kernel:reference.common", "GeometryContext._unit_g", "PCS.unit_g"),
    ("kernel:reference.common", "GeometryContext._validate_segment_corners", "PCS.validate_segment_corners"),
    ("kernel:reference.common", "GeometryContext._analysis_direction", "PCS.analysis_direction"),
    ("kernel:reference.evaluation_geometry", "build_evaluation_geometry_binding", "PCE.build_evaluation_geometry_binding"),
    ("kernel:reference.evaluation_geometry", "verify_evaluation_geometry_binding", "PCE.verify_evaluation_geometry_binding"),
    ("kernel:reference.corner_treatment", "corner_treatment_errors", "PCC.corner_treatment_errors"),
    ("kernel:_chain_station", "chain_station_plan_errors", "PCC.chain_station_plan_errors"),
    ("kernel:validation_metric", "validate_rational_affine_planar_metric", "PCV.validate_rational_affine_planar_metric"),
    ("kernel:reference.validation", "_validate_angle_certificates", "PCV.validate_angle_certificates"),
    ("kernel:reference.angular", "_verify_evaluation_binding_reasons", "DBV.verify_evaluation_binding_reasons_one"),
    ("kernel:reference.angular", "_interpolated_normals", "DBV.interpolated_normals"),
    ("kernel:reference.angular", "_canonical_subturn_fan_or_source", "DBV.canonical_subturn_fan_or_source"),
    ("kernel:reference.angular", "_incident_normal", "DBV.incident_normal"),
    ("kernel:reference.evaluation_binding_noise", "noise_records_error", "DBV.noise_records_error"),
)


def main():
    p = argparse.ArgumentParser()
    for name in ('tree', 'native-site', 'build-id', 'output'):
        p.add_argument('--' + name, required=True)
    p.add_argument('--case', choices=('building', 'cover008'), required=True)
    p.add_argument('--workers', type=int, default=0)
    p.add_argument('--pstats', default='')
    p.add_argument('--laps', type=int, default=1)
    a = p.parse_args(sys.argv[sys.argv.index('--') + 1:])
    output = Path(a.output)
    assert not output.exists(), output
    report = {'pid': os.getpid(), 'argv': sys.argv, 'tree': a.tree, 'case': a.case, 'workers': a.workers}
    code = 1
    controller = None
    try:
        tree = Path(a.tree).resolve()
        spec = importlib.util.spec_from_file_location('prof_tree_helpers', tree / 'tools/blender_tree_ab.py')
        helpers = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = helpers
        spec.loader.exec_module(helpers)
        helpers.load_tree(tree, a.native_site)
        report['native'] = helpers.verify_native(a.native_site, a.build_id)
        import bpy
        if a.case == 'cover008':
            bpy.context.window.scene = bpy.data.scenes['GEOMETRY']
        scene = bpy.context.scene
        settings = scene.hotspotuv_settings
        mesh_settings = scene.hotspotuv_decal_mesh
        obj = bpy.data.objects['building' if a.case == 'building' else 'cover.008']
        settings.envelope_debug_alpha = 0.2239 if a.case == 'building' else 0.04214220866560936
        settings.envelope_debug_fan_density = '2'
        settings.envelope_debug_max_stretch = 42 if a.case == 'building' else 20
        settings.envelope_debug_workers = a.workers
        mesh_settings.kernel_backend = 'NATIVE'
        mesh_settings.offset = 0.02 if a.case == 'building' else 0.004999999888241291
        chosen = helpers.select_edges(obj, 'seam' if a.case == 'building' else 'stored')
        report.update(blend=bpy.data.filepath, blender=bpy.app.version_string, selected=len(chosen), selection_sha=digest(sorted(chosen)))
        from cftuv.envelope_production_operator import _session
        controller = _session(bpy.context)
        if a.workers and a.laps:
            import cftuv.envelope_domain_pool as pool_module
            dump_dir = str(output.with_suffix('')) + '_workers'
            Path(dump_dir).mkdir(parents=True, exist_ok=False)
            call = helpers.WORKER_MAIN_CALL
            assert call in pool_module._BOOTSTRAP
            inject = (f"sys.path.insert(0, {str(HERE)!r})\nimport coldlaps as _coldlaps\n_coldlaps.install_worker(spec['package'], {dump_dir!r}, laps=True)\n"
                      f"for _m, _d, _l in {PC_EXTRA!r}:\n    _coldlaps.wrap(spec['package'], _m, _d, _l)\n")
            pool_module._BOOTSTRAP = pool_module._BOOTSTRAP.replace(call, inject + call)
            report['worker_dump_dir'] = dump_dir
        if a.workers:
            from cftuv.envelope_pool_prewarm import prewarm_pool
            from cftuv.envelope_worker_python import read_worker_python
            report['pool_prewarm_seconds'] = prewarm_pool(a.workers, read_worker_python())
        else:
            from cftuv.envelope_pool_prewarm import warm_code_identity
            warm_code_identity()
            from cftuv.envelope_domain_pool import warm_worker_identities
            warm_worker_identities()
        import coldlaps
        profiler = cProfile.Profile() if a.pstats else None
        if a.laps:
            report['lap_missing'] = coldlaps.install('cftuv')
            for mod, dotted, lap in PC_EXTRA:
                coldlaps.wrap('cftuv', mod, dotted, lap)
            report['lap_missing'] = list(coldlaps.MISSING)
        if profiler is not None:
            import cftuv_envelope.reference.compile as compile_module
            import cftuv_envelope.wavefront.conveyor as conveyor_module
            original = conveyor_module.compile_reference_envelopes
            depth = [0]

            def profiled(*args, **kwargs):
                if depth[0]:
                    return original(*args, **kwargs)
                depth[0] = 1
                profiler.enable()
                try:
                    return original(*args, **kwargs)
                finally:
                    profiler.disable()
                    depth[0] = 0
            conveyor_module.compile_reference_envelopes = profiled
        coldlaps.reset()
        cpu0 = time.process_time()
        started = time.perf_counter()
        result = bpy.ops.hotspotuv.build_envelope_decal_mesh('EXEC_DEFAULT')
        report['button_wall'] = time.perf_counter() - started
        report['button_cpu_parent'] = time.process_time() - cpu0
        report.update(result=sorted(result), status=mesh_settings.status, timing=mesh_settings.timing)
        assert result == {'FINISHED'}, report['status']
        report['laps'] = {k: {'n': v[0], 's': round(v[1], 6)} for k, v in sorted(coldlaps.LAP.items())}
        job = controller.byte_check_job
        if job is not None:
            job.wait(300)
            report['byte_check'] = {'state': job.state, 'checked': job.checked, 'expected': job.expected, 'seconds': job.seconds}
        from cftuv.envelope_production_mesh import decal_object_name
        decal = bpy.data.objects[decal_object_name(obj.name)]
        mesh = decal.data
        geometry = {'positions': [[float(x).hex() for x in v.co] for v in mesh.vertices], 'faces': [list(f.vertices) for f in mesh.polygons],
                    'seams': [e.index for e in mesh.edges if e.use_seam], 'uv': [[[float(x).hex() for x in item.uv] for item in layer.data] for layer in mesh.uv_layers]}
        report['decal'] = {'vertices': len(mesh.vertices), 'faces': len(mesh.polygons), 'digest': digest(geometry)}
        if profiler is not None:
            profiler.create_stats()
            profiler.dump_stats(a.pstats)
        code = 0
    except Exception:
        report['failure'] = traceback.format_exc()
        print(report['failure'], flush=True)
    finally:
        if controller is not None and getattr(controller, 'byte_check_job', None) is not None:
            job = controller.byte_check_job
            if not job.finished:
                job.stop.set()
                job.wait(10)
        module = sys.modules.get('cftuv.envelope_domain_pool')
        if module is not None:
            try:
                module.shutdown_domain_pool()
            except Exception:
                report['cleanup_failure'] = traceback.format_exc()
                code = 1
        report['exit'] = code
        output.write_text(json.dumps(report, indent=1, default=str), encoding='utf8')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
