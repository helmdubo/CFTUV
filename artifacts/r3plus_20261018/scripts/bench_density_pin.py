import cProfile, pstats, sys, time
sys.path[:0] = ['D:/r3plus-scratch/base-site', 'D:/CFTUV-r3plus/kernel/src']
import cftuv_envelope.reference.compile  # noqa
import cftuv_envelope.reference.density_compile_v1  # noqa
from cftuv_native import density_pin, pin
print('verdict', pin.verdict('density'))
t = time.perf_counter(); density_pin.runtime_stale(); print('first', time.perf_counter() - t)
n = 200
t = time.perf_counter()
for _ in range(n):
    density_pin.runtime_stale()
print('warm per call ms', (time.perf_counter() - t) / n * 1e3)
pr = cProfile.Profile(); pr.enable()
for _ in range(n):
    density_pin.runtime_stale()
pr.disable()
pstats.Stats(pr).sort_stats('tottime').print_stats(12)
print('rows', len(density_pin.FUNCTIONS), len(density_pin.CONSTANTS), len(density_pin.CLASSES), len(density_pin.ENUM_SUBSETS))
for file, q, _ in density_pin.CONSTANTS:
    v = density_pin._resolve(file, q)
    print(' const', file, q, type(v).__name__)
