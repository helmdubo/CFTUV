# TEST-ONLY: pool workers (Blender's python.exe, honours PYTHONPATH) run Blender's own sitecustomize first, then the chain_station pin hook.
import os, runpy, sys
_original = os.path.join(os.path.dirname(os.path.dirname(sys.executable)), 'lib', 'site-packages', 'sitecustomize.py')
if os.path.isfile(_original):
    runpy.run_path(_original, run_name='sitecustomize')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import station_pin_hook  # noqa: E402,F401
