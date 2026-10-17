"""Один холодный вызов настоящего оператора; запускается только внутри Blender -b."""
import argparse
import ctypes
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import traceback

SCRIPT_STARTED = time.perf_counter()


def creation(pid):
    from ctypes import wintypes
    k = ctypes.WinDLL('kernel32', use_last_error=True)
    k.OpenProcess.restype = wintypes.HANDLE
    k.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
    k.CloseHandle.argtypes = [wintypes.HANDLE]
    h = k.OpenProcess(0x1000, False, int(pid))
    if not h:
        return None
    values = [wintypes.FILETIME() for _ in range(4)]
    try:
        if not k.GetProcessTimes(h, *(ctypes.byref(v) for v in values)):
            return None
        return (values[0].dwHighDateTime << 32) | values[0].dwLowDateTime
    finally:
        k.CloseHandle(h)


def digest(value):
    return hashlib.sha256(json.dumps(value, separators=(',', ':'), sort_keys=True).encode()).hexdigest()


def main():
    p = argparse.ArgumentParser()
    for name in ('tree', 'native-site', 'build-id', 'output'):
        p.add_argument('--' + name, required=True)
    p.add_argument('--case', choices=('building', 'cover008'), required=True)
    a = p.parse_args(sys.argv[sys.argv.index('--') + 1:])
    output = Path(a.output)
    assert not output.exists(), output
    report = {'schema': 'cftuv.real.operator.v1', 'pid': os.getpid(), 'creation': creation(os.getpid()),
              'argv': sys.argv, 'tree': a.tree, 'case': a.case, 'processes': []}
    owned = []
    original = subprocess.Popen

    class RecordedPopen(original):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            row = {'pid': self.pid, 'creation': creation(self.pid), 'argv': self.args, 'exit': None}
            report['processes'].append(row)
            owned.append((self, row))
            output.with_suffix('.processes.json').write_text(json.dumps(report['processes'], indent=2, default=str))

    subprocess.Popen = RecordedPopen
    code = 1
    controller = None
    try:
        for drive in ('C:/', 'E:/'):
            assert shutil.disk_usage(drive).free >= 1.5 * 1024**3, 'DISK_STOP ' + drive
        assert not any(k in os.environ for k in ('CFTUV_FRAME_BACKEND', 'CFTUV_DENSITY_BACKEND',
            'CFTUV_SNAPSHOT_RECORD_DIR', 'CFTUV_W1_RECORD_DIR', 'CFTUV_SNAPSHOT_BYTE_CHECK')), 'nondefault environment'
        tree = Path(a.tree).resolve()
        report['tree_head'] = subprocess.run(['git', '-C', str(tree), 'rev-parse', 'HEAD'],
            check=True, capture_output=True, text=True).stdout.strip()
        spec = importlib.util.spec_from_file_location('operator_tree_helpers', tree / 'tools/blender_tree_ab.py')
        helpers = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = helpers
        spec.loader.exec_module(helpers)
        helpers.load_tree(tree, a.native_site)
        report['native'] = helpers.verify_native(a.native_site, a.build_id)
        assert report['native']['build_id'] == a.build_id
        status = report['native']['host_native_status']
        for stage in ('coverage', 'clip', 'skeleton', 'snap_embedding', 'effective_alpha',
                      'materialize_w1', 'snapshot_check', 'density'):
            assert status.get(stage) == 'available', (stage, status.get(stage))
        if 'frame' in status:
            assert status['frame'] == 'available', ('frame', status['frame'])
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
        settings.envelope_debug_workers = 12
        mesh_settings.kernel_backend = 'NATIVE'
        mesh_settings.offset = 0.02 if a.case == 'building' else 0.004999999888241291
        chosen = helpers.select_edges(obj, 'seam' if a.case == 'building' else 'stored')
        report.update(blend=bpy.data.filepath, blender=bpy.app.version_string,
                      selected=len(chosen), selection_sha=digest(sorted(chosen)))
        assert chosen, 'empty selection'
        from cftuv.envelope_production_operator import _session
        controller = _session(bpy.context)
        # UI запускает этот прогрев при регистрации; -b требует явного штатного вызова.
        # Подготовки доменов не строятся, реальная identity не подменяется.
        from cftuv.envelope_pool_prewarm import prewarm_pool
        from cftuv.envelope_worker_python import read_worker_python
        report['pool_prewarm_seconds'] = prewarm_pool(12, read_worker_python())
        from cftuv.envelope_domain_pool import get_domain_pool
        report['worker_prewarms'] = get_domain_pool(12, read_worker_python()).worker_prewarms
        assert len(report['worker_prewarms']) == 12, 'pool prewarm did not ready 12 workers'
        report['script_setup_to_button_seconds'] = time.perf_counter() - SCRIPT_STARTED
        started = time.perf_counter()
        result = bpy.ops.hotspotuv.build_envelope_decal_mesh('EXEC_DEFAULT')
        report['button_wall'] = time.perf_counter() - started
        report['angular_env'] = os.environ.get('CFTUV_ANGULAR_BACKEND')
        report['angular_status'] = status.get('angular')
        report.update(result=sorted(result), status=mesh_settings.status, timing=mesh_settings.timing)
        assert result == {'FINISHED'}, report['status']
        job = controller.byte_check_job
        assert job is not None, 'cold pool press did not expose its background byte check'
        assert job.wait(300), 'byte check did not finish within 300s'
        report['byte_check'] = {'state': job.state, 'checked': job.checked, 'expected': job.expected,
                                'violations': len(job.violations), 'seconds': job.seconds, 'text': job.text()}
        assert job.state == 'CLEAN' and not job.violations
        from cftuv.envelope_production_mesh import decal_object_name
        decal = bpy.data.objects[decal_object_name(obj.name)]
        mesh = decal.data
        geometry = {'positions': [[float(x).hex() for x in v.co] for v in mesh.vertices],
                    'faces': [list(f.vertices) for f in mesh.polygons],
                    'seams': [e.index for e in mesh.edges if e.use_seam],
                    'uv': [[[float(x).hex() for x in item.uv] for item in layer.data] for layer in mesh.uv_layers]}
        report['decal'] = {'vertices': len(mesh.vertices), 'faces': len(mesh.polygons),
                           'digest': digest(geometry)}
        code = 0
    except Exception:
        report['failure'] = traceback.format_exc()
        print(report['failure'], flush=True)
    finally:
        # Штатное закрытие собственного пула; продукт сам владеет своими worker PID.
        if controller is not None and controller.byte_check_job is not None:
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
        for process, row in owned:
            row['exit'] = process.poll()
            if row['exit'] is None:
                report['cleanup_failure'] = 'owned child still running'
                code = 1
        report['exit'] = code
        output.write_text(json.dumps(report, indent=2, default=str), encoding='utf8')
    return code


if __name__ == '__main__':
    raise SystemExit(main())
