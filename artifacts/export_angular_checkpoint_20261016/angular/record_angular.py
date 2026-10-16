"""Запись замороженного ANGULAR oracle из доверенных tasks; только явный CLI."""
from __future__ import annotations

import argparse
import contextlib
import ctypes
from ctypes import wintypes
import dataclasses
from decimal import getcontext, localcontext
import hashlib
import io
import json
import lzma
import os
from pathlib import Path
import pickle
import shutil
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import angular_raw_pickle

ROOT = Path('D:/CFTUV-export-frame-20261015')
EVIDENCE = Path('D:/CFTUV-export-20261015-evidence/angular')
TASKS = Path('D:/cftuv_native_corpus/frame-9164149537a1e704/snapshot_check')
HEAD = '7618154b627e63d475de1e5ce3e4e574b74c68d0'
KERNEL = '9164149537a1e704'
SCHEMA = 'cftuv.angular.oracle-capture.v1'


def decimal_context():
    context = getcontext()
    return {'prec':context.prec, 'rounding':context.rounding, 'Emin':context.Emin,
            'Emax':context.Emax, 'capitals':context.capitals, 'clamp':context.clamp,
            'flags':[[kind.__name__, enabled] for kind,enabled in context.flags.items()],
            'traps':[[kind.__name__, enabled] for kind,enabled in context.traps.items()]}


def process_creation():
    dll = ctypes.WinDLL('kernel32', use_last_error=True)
    dll.GetCurrentProcess.restype = wintypes.HANDLE
    dll.GetProcessTimes.argtypes = [wintypes.HANDLE] + [ctypes.POINTER(wintypes.FILETIME)] * 4
    fields = [wintypes.FILETIME() for _ in range(4)]
    if not dll.GetProcessTimes(dll.GetCurrentProcess(), *(ctypes.byref(f) for f in fields)):
        raise ctypes.WinError(ctypes.get_last_error())
    return (fields[0].dwHighDateTime << 32) | fields[0].dwLowDateTime


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1048576), b''): h.update(block)
    return h.hexdigest()


def save(path, value):
    temporary = path.with_suffix(path.suffix + '.pending')
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    temporary.replace(path)


def write_pickle(path, value):
    temporary = path.with_suffix(path.suffix + '.pending')
    if path.exists() or temporary.exists(): raise RuntimeError('OUTPUT_ALREADY_EXISTS ' + str(path))
    with lzma.open(temporary, 'wb', preset=3) as handle: pickle.dump(value, handle, protocol=5)
    temporary.replace(path)


def selected_tasks(path, corpus, start, count):
    # Не раскрываем внутренние host object graphs пропущенных задач.
    with lzma.open(path, 'rb') as handle:
        ordinal = 0
        selected = 0
        while True:
            try: case, blob = pickle.load(handle)
            except EOFError: return
            if not isinstance(blob, bytes): raise TypeError('TASK_BLOB_SCHEMA_CHANGED')
            if ordinal >= start:
                yield ordinal, hashlib.sha256(blob).hexdigest(), case, corpus.restore_historical_density_backend(pickle.loads(blob))
                selected += 1
                if count is not None and selected >= count: return
            ordinal += 1


def exception(error):
    result = {'type': type(error).__module__ + '.' + type(error).__qualname__, 'message': str(error)}
    for name in ('outcome', 'patch_domain_id'):
        value = getattr(error, name, None)
        result[name] = getattr(value, 'value', value)
    return result


def ast_expression(value, sp):
    # Порядок args сохраняется, никаких factor/cancel/упрощения в наблюдателе.
    if value.is_Integer: return ['Integer', str(value.p)]
    if value.is_Rational: return ['Rational', str(value.p), str(value.q)]
    tags = {sp.Add: 'Add', sp.Mul: 'Mul', sp.Pow: 'Pow'}
    if value.func not in tags:
        raise TypeError('UNSUPPORTED_EXPRESSION_NODE ' + str(value.func))
    return [tags[value.func], [ast_expression(item, sp) for item in value.args]]


def counter_delta(before, after):
    names = list(before) + [key for key in after if key not in before]
    return [[key, after.get(key, 0) - before.get(key, 0)] for key in names]


class Recorder:
    def __init__(self, sp, codec, planar, symbolic):
        self.sp, self.codec, self.planar, self.symbolic = sp, codec, planar, symbolic
        self.calls, self.errors, self.current = [], [], None

    def counts(self):
        return (dict(self.planar.SYMBOLIC_FALLBACK_COUNTS), dict(self.symbolic.BACKEND_COUNTS))

    def telemetry(self, before, after):
        return {'fallback_before': list(before[0].items()), 'fallback_after': list(after[0].items()),
                'fallback_delta': counter_delta(before[0], after[0]), 'backend_before': list(before[1].items()),
                'backend_after': list(after[1].items()), 'backend_delta': counter_delta(before[1], after[1])}

    def observe(self, target, function):
        # Ошибка записи не меняет ход oracle: задача доходит до исходного исхода,
        # затем весь capture громко отклоняется, а не становится новым эталоном.
        try:
            # Кодек вызывает Decimal.normalize(): наблюдение не вправе менять
            # flags/traps живого oracle или следующей задачи. Prec/traps копируются.
            with localcontext(getcontext().copy()): return function()
        except Exception as error:
            self.errors.append({'observer': target, 'error': exception(error)})
            return None

    @contextlib.contextmanager
    def installed(self):
        from cftuv import envelope_request_export as export, envelope_angular_relations as angular
        from cftuv.envelope_debug_profile import EnvelopeDebugProfileBuilderV1 as Profile
        from cftuv_envelope.reference.metric import ExactPlanarMetric
        from mpmath import iv
        original = export._build_angular_relations
        measure = angular.certified_reflex_measure
        set_counter = Profile.set_counter
        angle_g = ExactPlanarMetric.angle_g
        recorder = self

        def angular_wrapper(kernel, sympy, **kwargs):
            row = {'ordinal': len(recorder.calls), 'measures': [], 'angle_g_calls': [], 'profile_writes': []}
            raw = {key: value for key, value in kwargs.items() if key != 'profile'}
            row['raw_input_pickle'] = recorder.observe('raw angular input', lambda: angular_raw_pickle.dumps(raw))
            parent, recorder.current = recorder.current, row
            recorder.calls.append(row)
            before = recorder.counts()
            row['geometry_cursor'] = before
            try:
                result = original(kernel, sympy, **kwargs)
            except Exception as error:
                row['exception'] = exception(error)
                raise
            else:
                row['output_pickle'] = recorder.observe('angular output pickle', lambda: pickle.dumps(result, protocol=5))
                row['output_codec_bytes'] = recorder.observe('angular output codec', lambda: recorder.codec.canonical_json_bytes(result))
                return result
            finally:
                after = recorder.counts()
                row['telemetry'] = recorder.telemetry(before, after)
                row['geometry_tail'] = recorder.telemetry(row.pop('geometry_cursor'), after)
                recorder.current = parent

        def measure_wrapper(kernel, angle_measure, sine, cosine, orientation):
            row = {'orientation': getattr(orientation, 'value', orientation),
                   'decimal_before': decimal_context(), 'iv_prec_before': iv.prec}
            if recorder.current is not None:
                recorder.current['measures'].append(row)
                row['sine'] = recorder.observe('sine AST', lambda: {'srepr': recorder.sp.srepr(sine), 'ast': ast_expression(sine, recorder.sp)})
                row['cosine'] = recorder.observe('cosine AST', lambda: {'srepr': recorder.sp.srepr(cosine), 'ast': ast_expression(cosine, recorder.sp)})
            before = recorder.counts()
            if recorder.current is not None:
                row['geometry_since_previous_measure'] = recorder.telemetry(recorder.current['geometry_cursor'], before)
            try: value = measure(kernel, angle_measure, sine, cosine, orientation)
            except Exception as error:
                row['exception'] = exception(error)
                raise
            else:
                row['output_codec_bytes'] = recorder.observe('measure codec', lambda: recorder.codec.canonical_json_bytes(value))
                return value
            finally:
                after = recorder.counts()
                row['telemetry'] = recorder.telemetry(before, after)
                row.update(decimal_after=decimal_context(), iv_prec_after=iv.prec)
                if recorder.current is not None: recorder.current['geometry_cursor'] = after

        def angle_wrapper(metric, incoming, outgoing):
            if recorder.current is None: return angle_g(metric, incoming, outgoing)
            row = {}
            row['inputs'] = recorder.observe('angle immediate inputs', lambda: {
                'gram_srepr': [[recorder.sp.srepr(value) for value in values] for values in metric.gram],
                'owner_orientation_sign': metric.owner_orientation_sign,
                'incoming_scalar_expressions': [incoming.x.expression, incoming.y.expression],
                'outgoing_scalar_expressions': [outgoing.x.expression, outgoing.y.expression]})
            if recorder.current is not None: recorder.current['angle_g_calls'].append(row)
            before = recorder.counts()
            try: result = angle_g(metric, incoming, outgoing)
            except Exception as error:
                row['exception'] = exception(error)
                raise
            else:
                row['cosine'] = recorder.observe('angle cosine AST', lambda: {'srepr':recorder.sp.srepr(result[0]), 'ast':ast_expression(result[0], recorder.sp)})
                row['sine'] = recorder.observe('angle sine AST', lambda: {'srepr':recorder.sp.srepr(result[1]), 'ast':ast_expression(result[1], recorder.sp)})
                return result
            finally: row['telemetry'] = recorder.telemetry(before, recorder.counts())

        def counter_wrapper(profile, name, value, patch_domain_id=None):
            if recorder.current is not None:
                recorder.current['profile_writes'].append([name, value, patch_domain_id])
            return set_counter(profile, name, value, patch_domain_id)

        assert original is angular.build_angular_relations, 'ANGULAR_SEAM_CHANGED'
        export._build_angular_relations = angular_wrapper
        angular.certified_reflex_measure = measure_wrapper
        Profile.set_counter = counter_wrapper
        ExactPlanarMetric.angle_g = angle_wrapper
        try: yield
        finally:
            # Только наши exact wrapper objects: поздний import не должен держать
            # ссылку на завершённую запись. Чужие decorators не разворачиваются.
            aliases = {id(angular_wrapper):(angular_wrapper, original), id(measure_wrapper):(measure_wrapper, measure),
                       id(counter_wrapper):(counter_wrapper, set_counter), id(angle_wrapper):(angle_wrapper, angle_g)}
            for module in list(sys.modules.values()):
                name = getattr(module, '__name__', '')
                if name in ('cftuv', 'cftuv_envelope') or name.startswith(('cftuv.', 'cftuv_envelope.')):
                    for attribute, value in list(vars(module).items()):
                        replacement = aliases.get(id(value))
                        if replacement is not None and value is replacement[0]: setattr(module, attribute, replacement[1])
            export._build_angular_relations = original
            angular.certified_reflex_measure = measure
            Profile.set_counter = set_counter
            ExactPlanarMetric.angle_g = angle_g
            assert export._build_angular_relations is angular.build_angular_relations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tasks', type=Path, default=TASKS)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--name', default='')
    parser.add_argument('--limit', type=int)
    parser.add_argument('--archive', help='Точное имя одного доверенного *.tasks.xz')
    parser.add_argument('--task-index', type=int, help='Нулевой индекс внутри единственного архива')
    parser.add_argument('--baseline', action='store_true', help='Тот же task_inputs без recorder wrappers; отдельный новый output')
    parser.add_argument('--start', type=int, default=0, help='Начало неизменяемого ordered chunk внутри --archive')
    parser.add_argument('--count', type=int, help='Число задач chunk; предшествующие blobs не раскрываются')
    args = parser.parse_args()
    if args.limit is not None and args.limit < 1: parser.error('--limit must be positive')
    if args.task_index is not None and (args.task_index < 0 or args.archive is None or args.limit != 1): parser.error('--task-index requires --archive and --limit 1')
    if args.start < 0 or (args.count is not None and args.count < 1): parser.error('invalid --start/--count')
    if (args.start or args.count is not None) and args.archive is None: parser.error('--start/--count require --archive')
    if args.task_index is not None and (args.start or args.count is not None): parser.error('--task-index conflicts with --start/--count')
    start = args.task_index if args.task_index is not None else args.start
    count = 1 if args.task_index is not None else args.count
    out = args.out.resolve()
    assert out.is_relative_to(EVIDENCE.resolve()) and out != EVIDENCE.resolve() and not out.exists(), 'NEW_OWN_OUTPUT_REQUIRED'
    assert not (out.is_relative_to(args.tasks.resolve()) or args.tasks.resolve().is_relative_to(out)), 'SOURCE_OUTPUT_OVERLAP'
    for drive in ('C:/', 'D:/', 'E:/'):
        assert shutil.disk_usage(drive).free >= 1.5 * 1024**3, 'DISK_FLOOR ' + drive
    assert subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip() == HEAD
    assert not subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True).strip()
    sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'kernel/src')]
    import native_snapshot_check_corpus as corpus
    import native_build_id
    import native_ci_corpus
    corpus.install_host_stubs()
    import cftuv_envelope as kernel
    from cftuv_envelope import codec
    from cftuv_envelope.materialize.clip_memo import kernel_code_identity
    from cftuv_envelope.reference import planar_types, symbolic_backend
    from cftuv_envelope.reference.metric import ExactPlanarMetric  # одинаковые импорты до baseline/capture
    from cftuv.envelope_export_input import TaskInputsV1, task_inputs
    from cftuv.envelope_kernel_backend import entered_backend
    import sympy
    import mpmath
    assert kernel_code_identity() == KERNEL, 'KERNEL_MOVED'
    files = corpus.corpus_files(args.tasks, args.name)
    if args.archive:
        files = [path for path in files if path.name == args.archive]
        assert len(files) == 1, 'EXACT_ARCHIVE_NOT_FOUND'
    assert files, 'NO_TASK_ARCHIVES'
    out.mkdir(parents=True)
    manifest = {'schema': SCHEMA, 'status': 'INCOMPLETE', 'head': HEAD, 'kernel_identity': KERNEL,
                'python': sys.version, 'sympy': sympy.__version__, 'mpmath': mpmath.__version__,
                'symbolic_mode': symbolic_backend.backend_mode().value, 'decimal_context': decimal_context(),
                'runner_sha256': sha(__file__), 'raw_pickle_helper_sha256':sha(angular_raw_pickle.__file__),
                'raw_pickle_policy':'private Pickler dispatch; mappingproxy exact type/backing/shared identity; reader must import angular_raw_pickle',
                'pid': os.getpid(), 'creation_filetime': process_creation(),
                'argv': sys.argv, 'started': time.time(), 'native_source_build': native_build_id.tree_parts(ROOT / 'native'),
                'sources': {str(path): sha(path) for path in files}, 'tasks': [],
                'oracle_source_sha256': {}, 'limit': args.limit, 'archive':args.archive, 'task_index':args.task_index,
                'start':start, 'count':count,
                'capture_mode':'baseline_without_wrappers' if args.baseline else 'angular_wrappers', 'phase':'READY',
                'scope': 'EXPORT only; no prepare/produce/pool; retained process cache history; master PYTHON'}
    for relative in ('cftuv/envelope_angular_relations.py', 'cftuv/envelope_angular_sites.py', 'cftuv/envelope_chart_band.py',
                     'cftuv/envelope_angle_certificate.py', 'kernel/src/cftuv_envelope/reference/metric.py',
                     'kernel/src/cftuv_envelope/reference/planar_types.py', 'kernel/src/cftuv_envelope/reference/angle_measure.py'):
        manifest['oracle_source_sha256'][relative] = sha(ROOT / relative)
    save(out / 'manifest.json', manifest)
    try:
        total = 0
        for source in files:
            for source_ordinal, source_task_sha, case, task in selected_tasks(source, corpus, start, count):
                if args.limit is not None and total >= args.limit: break
                if getattr(task, 'export', None) is None: raise RuntimeError('TASK_WITHOUT_EXPORT ' + str(task.task_id))
                for drive in ('C:/', 'D:/', 'E:/'):
                    assert shutil.disk_usage(drive).free >= 1.5 * 1024**3, 'DISK_FLOOR ' + drive
                task = dataclasses.replace(task, cold=None)
                recorder = Recorder(sympy, codec, planar_types, symbolic_backend)
                row = {'case': case, 'task_id': task.task_id, 'domain_id': task.domain_id, 'ordinal': total,
                       'source': str(source), 'source_task_index':source_ordinal, 'source_task_blob_sha256':source_task_sha,
                       'task_pickle': pickle.dumps(task, protocol=5), 'decimal_before':decimal_context(), 'iv_prec_before':mpmath.iv.prec}
                manifest.update(phase='TASK_ORACLE_RUNNING', current_task={'source':str(source),'index':source_ordinal,'input_sha256':source_task_sha})
                save(out / 'manifest.json', manifest)
                before = recorder.counts()
                instrumentation = contextlib.nullcontext() if args.baseline else recorder.installed()
                with entered_backend('PYTHON'), instrumentation, contextlib.redirect_stdout(io.StringIO()):
                    try: result = task_inputs(task)
                    except Exception as error: row['exception'] = exception(error)
                    else:
                        row['outcome'] = 'inputs' if isinstance(result, TaskInputsV1) else 'refusal'
                        counters = result.profile.snapshot().counters if isinstance(result, TaskInputsV1) else result.export_counters
                        row['profile_counters'] = [[item.name,item.value,item.patch_domain_id] for item in counters]
                        if result.snapshot is not None:
                            row['snapshot_bytes'] = recorder.observe('task snapshot codec', lambda: kernel.AnalysisSnapshotCodecV1.dumps(result.snapshot))
                        if isinstance(result, TaskInputsV1):
                            row['request_bytes'] = recorder.observe('task request codec', lambda: codec.DecalRequestCodecV1.dumps(result.request))
                            row['labeling_pickle'] = recorder.observe('task labeling pickle', lambda: pickle.dumps(result.labeling, protocol=5))
                        else:
                            row['refusal'] = {'outcome': getattr(result.refusal.outcome, 'value', result.refusal.outcome),
                                              'message': result.refusal.message, 'domain': result.refusal.patch_domain_id}
                row['telemetry'] = recorder.telemetry(before, recorder.counts())
                row['decimal_after'] = decimal_context()
                row['iv_prec_after'] = mpmath.iv.prec
                row['angular_calls'] = recorder.calls
                row['observer_errors'] = recorder.errors
                target = out / ('task-' + str(total).zfill(6) + '.pickle.xz')
                manifest['phase'] = 'TASK_PERSISTING'
                save(out / 'manifest.json', manifest)
                write_pickle(target, row)
                manifest['tasks'].append({'case': case, 'domain': task.domain_id, 'path': target.name,
                                          'sha256': sha(target), 'angular_calls': len(recorder.calls),
                                          'observer_errors': len(recorder.errors), 'exception': row.get('exception'),
                                          'source_task_index':source_ordinal, 'source_task_blob_sha256':source_task_sha})
                total += 1
                manifest['phase'] = 'TASK_DURABLE'
                save(out / 'manifest.json', manifest)
                if recorder.errors: raise RuntimeError('OBSERVER_FAILED: oracle ran, capture not accepted')
            if args.limit is not None and total >= args.limit: break
        assert all(sha(path) == digest for path, digest in manifest['sources'].items()), 'SOURCE_ARCHIVE_CHANGED'
        assert all(sha(ROOT / path) == digest for path, digest in manifest['oracle_source_sha256'].items()), 'ORACLE_SOURCE_CHANGED'
        assert subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip() == HEAD
        assert not subprocess.check_output(['git', '-C', str(ROOT), 'status', '--porcelain'], text=True).strip(), 'SOURCE_TREE_CHANGED'
        assert native_ci_corpus.kernel_identity(ROOT) == KERNEL, 'KERNEL_CHANGED_DURING_CAPTURE'
        manifest.update(status='INCOMPLETE_LIMIT_CAPTURE' if args.limit is not None else 'RECORDED_NOT_PARITY_ACCEPTED',
                        ended=time.time(), task_count=total, phase='COMPLETE')
        save(out / 'manifest.json', manifest)
        print(json.dumps({'status':manifest['status'], 'tasks':total, 'out':str(out)}), flush=True)
    except Exception as error:
        manifest.update(status='FAILED_NOT_ACCEPTED', failure=exception(error), ended=time.time())
        save(out / 'manifest.json', manifest)
        raise


if __name__ == '__main__': main()
