import ast, hashlib, sys
from pathlib import Path
ROOT = Path('D:/CFTUV-export-angular-20261016')


def portable(node):
    if isinstance(node, ast.AST):
        parts = []
        for name in node._fields:
            value = getattr(node, name, None)
            if value is None or (isinstance(value, list) and not value):
                continue
            parts.append(f"{name}={portable(value)}")
        return f"{type(node).__name__}({', '.join(parts)})"
    if isinstance(node, list):
        return "[" + ", ".join(portable(item) for item in node) + "]"
    return repr(node)


tree = ast.parse((ROOT / 'cftuv/envelope_angular_relations.py').read_text(encoding='utf-8'))
functions = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
ref = functions['_build_angular_relations_reference']
print(sys.version.split()[0], 'legacy', hashlib.sha256(ast.dump(ref, include_attributes=False).encode()).hexdigest(),
      'portable', hashlib.sha256(portable(ref).encode()).hexdigest())
