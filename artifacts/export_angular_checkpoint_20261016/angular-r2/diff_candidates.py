import json, sys
old = json.load(open(sys.argv[1])); new = json.load(open(sys.argv[2]))
def walk(a, b, p=''):
    if isinstance(a, dict) and isinstance(b, dict):
        for k in sorted(set(a) | set(b)):
            if k not in a: print('ADDED', p + '/' + k)
            elif k not in b: print('REMOVED', p + '/' + k)
            else: walk(a[k], b[k], p + '/' + k)
    elif a != b:
        print('CHANGED', p, str(a)[:80], '->', str(b)[:80])
walk(old, new)
