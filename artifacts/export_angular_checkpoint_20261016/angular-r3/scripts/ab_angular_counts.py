"""Per side/case/workers/press: ANGULAR [native, python, fallbacks] summed over domains, plus domain angular_ran counts, from a tree_ab report."""
import json, sys
from collections import Counter


def walk(node, path, out):
    if isinstance(node, dict):
        if 'angular_calls' in node and isinstance(node['angular_calls'], list) and len(node['angular_calls']) == 3 and all(isinstance(v, int) for v in node['angular_calls']) and 'domains' not in node:
            out.append((path, node['angular_calls']))
        for k, v in node.items():
            walk(v, path + (str(k),), out)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            walk(v, path + (str(i),), out)


report = json.load(open(sys.argv[1]))
found = []
walk(report, (), found)
for path, calls in found:
    print('/'.join(path)[-120:], calls)
ran = Counter()
def ranwalk(node, path):
    if isinstance(node, dict):
        if 'angular_ran' in node:
            ran[(path[:6], node['angular_ran'])] += 1
        for k, v in node.items():
            ranwalk(v, path + (str(k),))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            ranwalk(v, path + (str(i),))
ranwalk(report, ())
for k, v in sorted(ran.items(), key=str):
    print(k, v)
