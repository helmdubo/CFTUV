from pathlib import Path
D = Path('D:/cftuv-angular-gates-20261017/operator')
c = D / 'operator_controller.py'
s = c.read_text(encoding='utf-8')


def rep(old, new):
    global s
    assert s.count(old) == 1, old
    s = s.replace(old, new)


rep('"""Парные холодные кнопки.', '"""ANGULAR copy (TEST-ONLY pin hook on the branch side, --branch-env, --reps). Парные холодные кнопки.')
rep("    p.add_argument('--blender', default=",
    "    p.add_argument('--branch-env', action='append', default=[])\n"
    "    p.add_argument('--branch-hook-profiles', default='')\n"
    "    p.add_argument('--reps', type=int, default=3)\n"
    "    p.add_argument('--blender', default=")
rep("            for repeat in range(1, 4):", "            for repeat in range(1, a.reps + 1):")
rep("                    argv = [a.blender, '-b', str(scene), '--python-exit-code', '1', '--python',\n",
    "                    hook = ['--python', str(Path(__file__).resolve().parents[1] / 'abhook/angular_pin_hook.py')] if side == 'branch' and a.branch_hook_profiles else []\n"
    "                    argv = [a.blender, '-b', str(scene), '--python-exit-code', '1', *hook, '--python',\n")
rep("                        if key.startswith('CFTUV_'):\n                            del env[key]\n",
    "                        if key.startswith('CFTUV_'):\n                            del env[key]\n"
    "                    if side == 'branch':\n"
    "                        env.update(item.split('=', 1) for item in a.branch_env)\n"
    "                        if a.branch_hook_profiles:\n"
    "                            env.update(PYTHONPATH=str(Path(__file__).resolve().parents[1] / 'abhook'), ANGULAR_HARNESS_PROFILES=a.branch_hook_profiles,\n"
    "                                       ANGULAR_HARNESS_LOG=str(out / (label + '.hook.jsonl')))\n")
rep("for n in range(1, 4)]", "for n in range(1, a.reps + 1)]")
c.write_text(s, encoding='utf-8')

r = D / 'operator_runner.py'
s = r.read_text(encoding='utf-8')
rep("        report['button_wall'] = time.perf_counter() - started\n",
    "        report['button_wall'] = time.perf_counter() - started\n"
    "        report['angular_env'] = os.environ.get('CFTUV_ANGULAR_BACKEND')\n"
    "        report['angular_status'] = status.get('angular')\n")
r.write_text(s, encoding='utf-8')
print('ok')
