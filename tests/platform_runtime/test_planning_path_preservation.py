"""Assembled MoveIt parameters retain collision validation after retiming."""
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[2]
ADAPTERS=['default_planning_response_adapters/AddTimeOptimalParameterization',
          'default_planning_response_adapters/ValidateSolution',
          'default_planning_response_adapters/DisplayMotionPath']


def parameters(tmp_path,monkeypatch,process,platform='commercial'):
    defaults=tmp_path/'default_configs';defaults.mkdir(exist_ok=True)
    (defaults/'ompl_planning.yaml').write_text(yaml.safe_dump({
        'planning_plugins':['ompl_interface/OMPLPlanner'],
        'request_adapters':['default_planning_request_adapters/CheckStartStateCollision'],
        'response_adapters':ADAPTERS}))
    monkeypatch.setitem(sys.modules,'ament_index_python.packages',
                        SimpleNamespace(get_package_share_directory=lambda _:str(tmp_path)))
    path=ROOT/'src/sketch_control/launch/platform_simulation.launch.py'
    spec=importlib.util.spec_from_file_location('platform_planning_test',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    sides=['left','right'] if platform=='commercial' else ['single']
    arms={side:{'group':side+'_manipulator','controller':side+'_controller',
                'joints':[side+'_j'+str(i) for i in range(6)]} for side in sides}
    urdf='<robot name="test">'+''.join(
        '<joint name="'+joint+'" type="revolute"><limit lower="-3.14" upper="3.14" velocity="2."/></joint>'
        for arm in arms.values() for joint in arm['joints'])+'</robot>'
    (tmp_path/'robot.srdf').write_text('<robot name="test"/>')
    return module.moveit_parameters({'assets':tmp_path,'bundle':{'arms':arms,
        'selection':{'process':process}},'active':arms[sides[-1]],
        'robot_description':urdf,'control_namespace':''})


@pytest.mark.parametrize('platform',['commercial','rb10'])
@pytest.mark.parametrize('process',['spray','weld','paint'])
def test_pipeline_bounds_retiming_deviation_and_collision_edge_spacing(tmp_path,monkeypatch,platform,process):
    result=parameters(tmp_path,monkeypatch,process,platform);pipeline=result['ompl']
    # The adapter's ParamListener receives the pipeline namespace, so the
    # installed parameter must flatten to ompl.totg.path_tolerance.
    assert pipeline['totg']=={'path_tolerance':.001}
    assert pipeline['planner_configs']['RRTConnectkConfigDefault']['longest_valid_segment_fraction']==.0001
    for group in result['robot_description_kinematics']:
        assert pipeline[group]['longest_valid_segment_fraction']==.0001
    assert pipeline['response_adapters']==ADAPTERS
    assert pipeline['request_adapters']==['default_planning_request_adapters/CheckStartStateCollision']
    assert all(limit['max_velocity']==2. and limit['max_acceleration']==1.
               for limit in result['robot_description_planning']['joint_limits'].values())


def test_paint_keeps_original_collision_model_and_joint_limits(tmp_path,monkeypatch):
    result=parameters(tmp_path,monkeypatch,'paint');pipeline=result['ompl']
    assert pipeline['response_adapters']==ADAPTERS
    assert result['robot_description_semantic']=='<robot name="test"/>'
    assert 'allowed_collision_matrix' not in result
    for limit in result['robot_description_planning']['joint_limits'].values():
        assert limit=={'has_position_limits':True,'min_position':-3.14,'max_position':3.14,
                       'has_velocity_limits':True,'max_velocity':2.,
                       'has_acceleration_limits':True,'max_acceleration':1.}


def test_installed_moveit_adapter_exposes_exact_totg_parameter_names():
    header=Path('/opt/ros/jazzy/include/moveit_ros_planning/default_response_adapter_parameters.hpp')
    if not header.exists():pytest.skip('installed Jazzy adapter header unavailable')
    text=header.read_text()
    assert 'double path_tolerance = 0.1;' in text
    assert 'prefix_ + "totg.path_tolerance"' in text
    assert 'double resample_dt = 0.1;' in text
