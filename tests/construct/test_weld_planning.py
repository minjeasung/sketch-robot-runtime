import copy
import importlib.util
from pathlib import Path

def test_weld_planning_tightens_collision_resolution_without_changing_spray():
    path=Path(__file__).parents[2]/'scripts/construct_moveit.launch.py'
    spec=importlib.util.spec_from_file_location('construct_planning',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original={'response_adapters':['ValidateSolution'],'left_manipulator':{},'right_manipulator':{},'planner_configs':{'RRTConnectkConfigDefault':{}}}
    spray=copy.deepcopy(original);module.apply_process_planning_settings(spray,'spray')
    assert spray==original
    weld=copy.deepcopy(original);module.apply_process_planning_settings(weld,'weld')
    assert weld['left_manipulator']['longest_valid_segment_fraction']==.0001
    assert weld['right_manipulator']['longest_valid_segment_fraction']==.0001
    assert weld['totg']['path_tolerance']==.001
    assert weld['response_adapters']==original['response_adapters']
