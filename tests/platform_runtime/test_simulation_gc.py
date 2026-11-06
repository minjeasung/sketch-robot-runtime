"""Initialization cycles must not be rescanned during the measured-state loop."""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]


def test_initialization_heap_is_excluded_but_runtime_cycles_are_collected():
    # GC is process-global. Isolate this test from pytest and extension objects.
    code = r'''
import gc
import importlib.util
import weakref
from pathlib import Path
from contextlib import nullcontext
p = Path('scripts/platform_simulation_app.py')
spec = importlib.util.spec_from_file_location('simulation_gc_test', p)
app = importlib.util.module_from_spec(spec)
spec.loader.exec_module(app)
class Cycle:
    def __init__(self): self.self = self
old = Cycle()
old_ref = weakref.ref(old)
# The old heap's objects may still receive references during a simulation.
# A full scan must exclude that heap, while new unreachable cycles stay collectable.
scope = getattr(app, 'simulation_gc_scope', nullcontext)
try:
    with scope():
        del old
        new = Cycle()
        new_ref = weakref.ref(new)
        del new
        gc.collect(2)
        assert old_ref() is not None, 'initialization heap was rescanned in the control loop'
        assert new_ref() is None, 'new runtime cycles leaked'
        assert gc.isenabled(), 'automatic collection was disabled'
        raise RuntimeError('simulated simulator failure')
except RuntimeError:
    pass
gc.collect(2)
assert old_ref() is None, 'initialization heap was not released after failure'
'''
    result = subprocess.run([sys.executable, '-c', code], cwd=ROOT,
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr
