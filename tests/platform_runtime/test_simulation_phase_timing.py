"""Diagnostics separate wall stalls without changing physics or freshness."""
import importlib.util
from pathlib import Path


def cls():
    path=Path(__file__).resolve().parents[2]/'scripts/platform_simulation_app.py'
    spec=importlib.util.spec_from_file_location('simulation_phase_timing_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module.SimulationPhaseTiming


def test_phase_timing_attributes_render_stall_and_physical_publication_gap():
    timing=cls()(session='s',revision='r',capacity=3)
    timing.begin(10.,1);timing.mark('physics',10.002);timing.published(10.003)
    assert timing.finish(10.004) is None
    timing.begin(10.004,2);timing.mark('physics',10.006)
    timing.mark('render',10.126);timing.mark('camera_readback_publish',10.136)
    timing.published(10.138)
    slow=timing.finish(10.14)
    assert abs(slow['phases_s']['render']-.12)<1e-9
    assert abs(slow['publication_gap_s']-.135)<1e-9
    assert slow['step_index']==2 and slow['session_id']=='s' and slow['scene_revision']=='r'
    assert slow['total_wall_s']>.1


def test_timing_retains_bounded_recent_steps_and_scheduler_gaps():
    timing=cls()(session='s',revision='r',capacity=3)
    for index in range(8):
        start=10.+index*.002;timing.begin(start,index);timing.finish(start+.001)
    assert len(timing.recent)==3
    timing.begin(11.,8);timing.mark('physics',11.001)
    slow=timing.finish(11.002)
    assert slow['between_steps_wall_s']>.9
    assert len(timing.snapshot()['recent_steps'])==3
    assert len(timing.snapshot()['slow_steps'])<=3
