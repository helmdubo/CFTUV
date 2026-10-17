from pathlib import Path
p = Path('D:/CFTUV-export-angular-20261016/tests/test_envelope_angular_dispatch.py')
raw = p.read_bytes().decode()
crlf = '\r\n' in raw
s = raw.replace('\r\n', '\n')
old_const = "APPROVED_REFERENCE_AST = 'a809d58b53987591ca2e51c1804f6fbc49f45ab606fd653f71f2716e19b3d7e0'\n"
new_const = (
    "APPROVED_REFERENCE_AST = 'a809d58b53987591ca2e51c1804f6fbc49f45ab606fd653f71f2716e19b3d7e0'\n"
    "# The same body through _portable_ast_dump (empty fields dropped): one value on 3.10, 3.11 and 3.13 (3.13 stopped printing empty fields in ast.dump).\n"
    "APPROVED_REFERENCE_PORTABLE_AST = '95d4453f0b94222d8bd950e77226235cb415541a06265968fe6dcc9e494b3484'\n\n\n"
    "def _portable_ast_dump(node):\n"
    "    if isinstance(node, ast.AST):\n"
    "        parts = [f'{name}={_portable_ast_dump(value)}' for name in node._fields\n"
    "                 for value in (getattr(node, name, None),) if not (value is None or (isinstance(value, list) and not value))]\n"
    "        return f\"{type(node).__name__}({', '.join(parts)})\"\n"
    "    if isinstance(node, list):\n"
    "        return '[' + ', '.join(_portable_ast_dump(item) for item in node) + ']'\n"
    "    return repr(node)\n")
assert s.count(old_const) == 1
s = s.replace(old_const, new_const)
old = "    assert hashlib.sha256(ast.dump(reference, include_attributes=False).encode()).hexdigest() == APPROVED_REFERENCE_AST\n"
new = ("    assert hashlib.sha256(_portable_ast_dump(reference).encode()).hexdigest() == APPROVED_REFERENCE_PORTABLE_AST\n"
       "    if sys.version_info < (3, 13):  # the historical ast.dump value that CI 3.10/3.11 has always checked\n"
       "        assert hashlib.sha256(ast.dump(reference, include_attributes=False).encode()).hexdigest() == APPROVED_REFERENCE_AST\n")
assert s.count(old) == 1
s = s.replace(old, new)
assert s.count("import inspect\n") == 1
s = s.replace("import inspect\n", "import inspect\nimport sys\n", 1)
if crlf:
    s = s.replace('\n', '\r\n')
p.write_bytes(s.encode())
