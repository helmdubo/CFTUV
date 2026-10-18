"""TEST-ONLY wrapper of the tree's own tools/blender_tree_ab.py: every Blender it starts gets the chain_station pin hook (R3+ S2 field A/B).

The comparison, PID records and headless check stay the tool's own (tool.run_one is called unchanged after the command/env are extended):
  * the parent Blender runs  --python abhook/station_pin_hook.py  before the tool's own --python;
  * PYTHONPATH=abhook makes the pool workers (Blender's python.exe) run abhook/sitecustomize.py (Blender's sitecustomize first, then the hook);
  * STATION_HARNESS_APPROVE=1 / STATION_HARNESS_LOG; TMP/TEMP on D:.
Usage: station_tree_ab.py --tool-tree TREE --hook-log LOG --tmp DIR -- <tree_ab arguments>
"""
import importlib.util
import sys
from pathlib import Path

HOOK = Path(__file__).resolve().parent / 'abhook'


def main():
    argv = sys.argv[1:]
    split = argv.index('--')
    own, rest = argv[:split], argv[split + 1:]
    options = dict(zip(own[::2], own[1::2]))
    tree = Path(options['--tool-tree']).resolve()
    log_path = options['--hook-log']
    temporary = Path(options['--tmp']).resolve()
    temporary.mkdir(parents=True, exist_ok=True)
    assert temporary.drive.lower() == 'd:' and Path(log_path).resolve().drive.lower() == 'd:'
    spec = importlib.util.spec_from_file_location('station_harness_tree_ab', tree / 'tools/blender_tree_ab.py')
    tool = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = tool
    spec.loader.exec_module(tool)
    original = tool.run_one

    def run_one(command, out, tree_label, work_dir, environment, timeout, ours, guard, log=print):
        command = list(command)
        at = command.index('--python')
        command[at:at] = ['--python', str(HOOK / 'station_pin_hook.py')]
        env = dict(environment, PYTHONPATH=str(HOOK), STATION_HARNESS_APPROVE='1', STATION_HARNESS_LOG=log_path,
                   TMP=str(temporary), TEMP=str(temporary), PYTHONDONTWRITEBYTECODE='1')
        return original(command, out, tree_label, work_dir, env, timeout, ours, guard, log)

    tool.run_one = run_one
    return tool.driver_main(rest)


if __name__ == '__main__':
    raise SystemExit(main())
